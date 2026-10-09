/*
 * 三模式手势控制器 - 最终定稿
 * 硬件：ESP32 + OLED(0x3C) + APDS9960(0x39) + 无源蜂鸣器(GPIO5)
 * 蓝牙复合HID：键盘、鼠标、媒体
 * 模式切换：BOOT键(GPIO0)短按
 * 蜂鸣器：PWM非阻塞
 * 手势：圆弧状态机（顺/逆时针）+ 接近度
 */

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "ApdsSensor.h"
#include <NimBLEDevice.h>
#include <NimBLEHIDDevice.h>
#include <NimBLECharacteristic.h>
#include <atomic>
#include <string.h>

// ==================== 引脚定义 ====================
#define OLED_SDA    21
#define OLED_SCL    22
#define OLED_ADDR   0x3C
#define SCREEN_W    128
#define SCREEN_H    64
#define BUZZER_PIN  5
#define BOOT_PIN    0

// ==================== 蜂鸣器PWM ====================
#define BUZZER_CHANNEL 0
#define BUZZER_FREQ    4000
#define BUZZER_RES     8
static unsigned long beepEndMs = 0;

void beepNonBlock(int ms) {
    ledcWrite(BUZZER_CHANNEL, 128); // 50%占空比
    beepEndMs = millis() + ms;
}

// ==================== OLED ====================
Adafruit_SSD1306 display(SCREEN_W, SCREEN_H, &Wire, -1);
bool oledOK = false;

// ==================== APDS9960 ====================
ApdsSensor apds;

// ==================== 模式 ====================
enum Mode : uint8_t { MODE_KB = 0, MODE_MOUSE, MODE_MEDIA, MODE_N };
const char* modeNames[] = {"1.Keyboard", "2.Mouse", "3.Media"};
Mode currentMode = MODE_KB;

// ==================== 手势方向 ====================
enum GDir { G_NONE, G_UP, G_DOWN, G_LEFT, G_RIGHT, G_CW, G_CCW };

// ==================== 圆弧状态机 ====================
#define ARC_MAX 6
#define ARC_TIMEOUT_MS 500
GDir arcQueue[ARC_MAX];
int arcLen = 0;
unsigned long arcLastGestureMs = 0;

void arcReset() {
    arcLen = 0;
    arcLastGestureMs = 0;
}

void arcPush(GDir d) {
    if (d != G_NONE && arcLen < ARC_MAX) {
        arcQueue[arcLen++] = d;
    }
}

bool isCW(GDir a, GDir b) {
    const GDir seq[] = {G_UP, G_RIGHT, G_DOWN, G_LEFT};
    int ai = -1, bi = -1;
    for (int i = 0; i < 4; i++) {
        if (seq[i] == a) ai = i;
        if (seq[i] == b) bi = i;
    }
    return (ai >= 0 && bi >= 0 && bi == (ai + 1) % 4);
}

bool isCCW(GDir a, GDir b) {
    const GDir seq[] = {G_UP, G_LEFT, G_DOWN, G_RIGHT};
    int ai = -1, bi = -1;
    for (int i = 0; i < 4; i++) {
        if (seq[i] == a) ai = i;
        if (seq[i] == b) bi = i;
    }
    return (ai >= 0 && bi >= 0 && bi == (ai + 1) % 4);
}

GDir checkArc() {
    if (arcLen < 3) return G_NONE;
    bool cw = true, ccw = true;
    for (int i = 1; i < arcLen; i++) {
        if (!isCW(arcQueue[i - 1], arcQueue[i])) cw = false;
        if (!isCCW(arcQueue[i - 1], arcQueue[i])) ccw = false;
    }
    if (cw) return G_CW;
    if (ccw) return G_CCW;
    return G_NONE;
}

bool isArcCandidate() {
    if (arcLen < 2) return true;
    bool cw = true, ccw = true;
    for (int i = 1; i < arcLen; i++) {
        if (!isCW(arcQueue[i - 1], arcQueue[i])) cw = false;
        if (!isCCW(arcQueue[i - 1], arcQueue[i])) ccw = false;
    }
    return cw || ccw;
}

const char* gestureName(GDir gesture) {
    switch (gesture) {
        case G_UP:    return "UP";
        case G_DOWN:  return "DOWN";
        case G_LEFT:  return "LEFT";
        case G_RIGHT: return "RIGHT";
        case G_CW:    return "CW";
        case G_CCW:   return "CCW";
        default:      return "?";
    }
}

// ==================== BLE 复合 HID ====================
NimBLEHIDDevice* hid;
NimBLECharacteristic* inpKb;
NimBLECharacteristic* inpMouse;
NimBLECharacteristic* inpMedia;
NimBLEServer* bleServer;

enum HidTapType : uint8_t {
    HID_TAP_KEYBOARD,
    HID_TAP_MOUSE_BUTTON,
    HID_TAP_MEDIA,
    HID_MOUSE_REPORT
};

struct HidTap {
    HidTapType type;
    uint8_t value;
    int8_t dx;
    int8_t dy;
    int8_t wheel;
};

constexpr uint8_t HID_TAP_QUEUE_CAPACITY = 12;
constexpr uint32_t HID_TAP_PRESS_MS = 25;
HidTap hidTapQueue[HID_TAP_QUEUE_CAPACITY];
uint8_t hidTapHead = 0;
uint8_t hidTapTail = 0;
uint8_t hidTapCount = 0;
HidTap activeHidTap = {HID_TAP_KEYBOARD, 0, 0, 0, 0};
bool hidTapActive = false;
unsigned long hidTapReleaseAt = 0;
std::atomic<bool> hidTapCancelRequested{false};

#define RID_KEYBOARD 0x01
#define RID_MOUSE    0x02
#define RID_MEDIA    0x03

const uint8_t kbDesc[] = {
    0x05, 0x01, 0x09, 0x06, 0xA1, 0x01,
    0x85, RID_KEYBOARD,
    0x05, 0x07, 0x19, 0xE0, 0x29, 0xE7, 0x15, 0x00, 0x25, 0x01, 0x75, 0x01, 0x95, 0x08, 0x81, 0x02,
    0x95, 0x01, 0x75, 0x08, 0x81, 0x03,
    0x95, 0x06, 0x75, 0x08, 0x15, 0x00, 0x25, 0x65, 0x05, 0x07, 0x19, 0x00, 0x29, 0x65, 0x81, 0x00,
    0xC0
};

const uint8_t mouseDesc[] = {
    0x05, 0x01, 0x09, 0x02, 0xA1, 0x01,
    0x85, RID_MOUSE,
    0x09, 0x01, 0xA1, 0x00,
    0x95, 0x05, 0x75, 0x01, 0x05, 0x09, 0x19, 0x01, 0x29, 0x05, 0x81, 0x02,
    0x95, 0x01, 0x75, 0x03, 0x81, 0x03,
    0x95, 0x01, 0x75, 0x08, 0x05, 0x01, 0x09, 0x30, 0x09, 0x31, 0x09, 0x38,
    0x15, 0x81, 0x25, 0x7F, 0x81, 0x06,
    0xC0, 0xC0
};

const uint8_t mediaDesc[] = {
    0x05, 0x0C, 0x09, 0x01, 0xA1, 0x01,
    0x85, RID_MEDIA,
    0x15, 0x00, 0x25, 0x01, 0x75, 0x01, 0x95, 0x08,
    0x09, 0xE9, 0x09, 0xEA, 0x09, 0xB5, 0x09, 0xB6, 0x09, 0xCD, 0x09, 0xB0, 0x09, 0xB1, 0x09, 0xB2,
    0x81, 0x02,
    0xC0
};

class GestureServerCallbacks : public NimBLEServerCallbacks {
    void onConnect(NimBLEServer* server, NimBLEConnInfo& connInfo) override {
        server->updateConnParams(connInfo.getConnHandle(), 12, 24, 0, 600);
        Serial.printf("[BLE] Connected; active connections: %u; interval=%.1f ms; bonded=%s\n",
                      server->getConnectedCount(),
                      connInfo.getConnInterval() * 1.25f,
                      connInfo.isBonded() ? "yes" : "no");
    }

    void onDisconnect(NimBLEServer* server, NimBLEConnInfo& connInfo, int reason) override {
        (void)server;
        (void)connInfo;
        hidTapCancelRequested.store(true, std::memory_order_relaxed);
        Serial.printf("[BLE] Disconnected; reason=%d; advertising will restart\n", reason);
    }

    void onAuthenticationComplete(NimBLEConnInfo& connInfo) override {
        Serial.printf("[BLE] Authentication complete; encrypted=%s\n",
                      connInfo.isEncrypted() ? "yes" : "no");
    }
};

static GestureServerCallbacks bleCallbacks;

bool sendHidReport(NimBLECharacteristic* characteristic, const char* reportName) {
    if (!bleServer || bleServer->getConnectedCount() == 0) {
        Serial.printf("[BLE] No connected host; skipped %s report\n", reportName);
        return false;
    }
    if (!characteristic || !characteristic->notify()) {
        Serial.printf("[BLE] %s report notification failed; check pairing and report subscription\n", reportName);
        return false;
    }
    return true;
}

bool sendKeyboardReport(uint8_t mod, uint8_t key) {
    uint8_t report[7] = {mod, 0, key, 0, 0, 0, 0};
    inpKb->setValue(report, sizeof(report));
    return sendHidReport(inpKb, "keyboard");
}

bool sendMouseReport(uint8_t btn, int8_t dx, int8_t dy, int8_t dw) {
    uint8_t report[4] = {btn, (uint8_t)dx, (uint8_t)dy, (uint8_t)dw};
    inpMouse->setValue(report, sizeof(report));
    return sendHidReport(inpMouse, "mouse");
}

bool sendMediaReport(uint8_t bit, bool pressed) {
    uint8_t report = pressed ? (uint8_t)(1U << bit) : 0;
    inpMedia->setValue(&report, sizeof(report));
    return sendHidReport(inpMedia, pressed ? "media" : "media release");
}

bool sendTapReport(const HidTap& tap, bool pressed) {
    switch (tap.type) {
        case HID_TAP_KEYBOARD:
            return sendKeyboardReport(0, pressed ? tap.value : 0);
        case HID_TAP_MOUSE_BUTTON:
            return sendMouseReport(pressed ? tap.value : 0, 0, 0, 0);
        case HID_TAP_MEDIA:
            return sendMediaReport(tap.value, pressed);
        case HID_MOUSE_REPORT:
            return sendMouseReport(0, tap.dx, tap.dy, tap.wheel);
    }
    return false;
}

void serviceHidTaps() {
    if (hidTapCancelRequested.exchange(false, std::memory_order_relaxed)) {
        hidTapHead = hidTapTail = hidTapCount = 0;
        hidTapActive = false;
    }

    if (hidTapActive) {
        if ((int32_t)(millis() - hidTapReleaseAt) < 0) return;
        sendTapReport(activeHidTap, false);
        hidTapActive = false;
    }

    while (hidTapCount > 0) {
        activeHidTap = hidTapQueue[hidTapHead];
        hidTapHead = (hidTapHead + 1) % HID_TAP_QUEUE_CAPACITY;
        --hidTapCount;

        if (activeHidTap.type == HID_MOUSE_REPORT) {
            sendTapReport(activeHidTap, true);
            continue;
        }
        if (sendTapReport(activeHidTap, true)) {
            hidTapReleaseAt = millis() + HID_TAP_PRESS_MS;
            hidTapActive = true;
            return;
        }
    }
}

bool queueHidTap(HidTapType type, uint8_t value) {
    if (!bleServer || bleServer->getConnectedCount() == 0) {
        Serial.println("[BLE] No connected host; skipped HID tap");
        return false;
    }
    if (hidTapCount == HID_TAP_QUEUE_CAPACITY) {
        Serial.println("[BLE] HID tap queue full; skipped action");
        return false;
    }

    hidTapQueue[hidTapTail] = {type, value, 0, 0, 0};
    hidTapTail = (hidTapTail + 1) % HID_TAP_QUEUE_CAPACITY;
    ++hidTapCount;
    serviceHidTaps();
    return true;
}

void sendMouse(uint8_t btn, int8_t dx, int8_t dy, int8_t dw) {
    if (btn == 0 && (hidTapActive || hidTapCount > 0)) {
        if (!bleServer || bleServer->getConnectedCount() == 0) {
            sendMouseReport(btn, dx, dy, dw);
            return;
        }
        if (hidTapCount == HID_TAP_QUEUE_CAPACITY) {
            Serial.println("[BLE] HID report queue full; skipped mouse movement");
            return;
        }
        hidTapQueue[hidTapTail] = {HID_MOUSE_REPORT, 0, dx, dy, dw};
        hidTapTail = (hidTapTail + 1) % HID_TAP_QUEUE_CAPACITY;
        ++hidTapCount;
        return;
    }
    sendMouseReport(btn, dx, dy, dw);
}

bool initBLE() {
    if (!NimBLEDevice::init("Gesture3Mode")) {
        Serial.println("[BLE] NimBLE initialization failed");
        return false;
    }
    NimBLEDevice::setSecurityAuth(true, false, true);

    bleServer = NimBLEDevice::createServer();
    if (!bleServer) {
        Serial.println("[BLE] GATT server creation failed");
        return false;
    }
    bleServer->setCallbacks(&bleCallbacks, false);

    hid = new NimBLEHIDDevice(bleServer);
    if (!hid->setManufacturer("Espressif")) {
        Serial.println("[BLE] Manufacturer setup failed");
        return false;
    }
    hid->setHidInfo(0x00, 0x01);

    uint8_t reportMap[sizeof(kbDesc) + sizeof(mouseDesc) + sizeof(mediaDesc)];
    size_t offset = 0;
    memcpy(reportMap + offset, kbDesc, sizeof(kbDesc));
    offset += sizeof(kbDesc);
    memcpy(reportMap + offset, mouseDesc, sizeof(mouseDesc));
    offset += sizeof(mouseDesc);
    memcpy(reportMap + offset, mediaDesc, sizeof(mediaDesc));
    hid->setReportMap(reportMap, sizeof(reportMap));

    inpKb = hid->getInputReport(RID_KEYBOARD);
    inpMouse = hid->getInputReport(RID_MOUSE);
    inpMedia = hid->getInputReport(RID_MEDIA);
    if (!inpKb || !inpMouse || !inpMedia) {
        Serial.println("[BLE] HID report characteristic creation failed");
        return false;
    }

    if (!bleServer->start()) {
        Serial.println("[BLE] GATT service start failed");
        return false;
    }

    bleServer->advertiseOnDisconnect(true);
    NimBLEAdvertising* adv = bleServer->getAdvertising();
    adv->enableScanResponse(true);
    if (!adv->setName("Gesture3Mode") ||
        !adv->addServiceUUID(hid->getHidService()->getUUID()) ||
        !adv->setAppearance(GENERIC_HID) ||
        !adv->start()) {
        Serial.println("[BLE] Advertising configuration or start failed");
        return false;
    }

    Serial.println("[BLE] HID service started and advertising as Gesture3Mode");
    return true;
}

// ==================== 手势执行 ====================
void executeGesture(GDir g) {
    if (g == G_NONE) return;
    const char* tag = "";
    switch (currentMode) {
        case MODE_KB:
            switch (g) {
                case G_UP:    queueHidTap(HID_TAP_KEYBOARD, 0x4B); tag = "PageUp"; break;
                case G_DOWN:  queueHidTap(HID_TAP_KEYBOARD, 0x4E); tag = "PageDown"; break;
                case G_LEFT:  queueHidTap(HID_TAP_KEYBOARD, 0x50); tag = "Left"; break;
                case G_RIGHT: queueHidTap(HID_TAP_KEYBOARD, 0x28); tag = "Enter"; break;
                default: return;
            }
            break;

        case MODE_MOUSE:
            if (g == G_CW) {
                sendMouse(0, 0, 0, 1); tag = "WheelUp";
            } else if (g == G_CCW) {
                sendMouse(0, 0, 0, -1); tag = "WheelDown";
            } else {
                switch (g) {
                    case G_UP:    sendMouse(0, 0, -20, 0); tag = "MoveUp"; break;
                    case G_DOWN:  sendMouse(0, 0, 20, 0);  tag = "MoveDown"; break;
                    case G_LEFT:  sendMouse(0, -20, 0, 0); tag = "MoveLeft"; break;
                    case G_RIGHT: sendMouse(0, 20, 0, 0);  tag = "MoveRight"; break;
                    default: return;
                }
            }
            break;

        case MODE_MEDIA:
            switch (g) {
                case G_UP:    queueHidTap(HID_TAP_MEDIA, 0); tag = "Vol+"; break;
                case G_DOWN:  queueHidTap(HID_TAP_MEDIA, 1); tag = "Vol-"; break;
                case G_LEFT:  queueHidTap(HID_TAP_MEDIA, 3); tag = "Prev"; break;
                case G_RIGHT: queueHidTap(HID_TAP_MEDIA, 2); tag = "Next"; break;
                default: return;
            }
            break;
    }
    Serial.printf("[%s] %s\n", modeNames[currentMode], tag);
}

// 接近度处理
void handleProximity(uint16_t prox) {
    static uint16_t lastProx = 0;
    static unsigned long lastProxTime = 0;
    const uint16_t THRESHOLD = 150; // 可根据实际调节
    const unsigned long PROX_COOLDOWN = 450;

    if (prox > THRESHOLD && lastProx <= THRESHOLD && (millis() - lastProxTime > PROX_COOLDOWN)) {
        lastProxTime = millis();
        switch (currentMode) {
            case MODE_MOUSE:
                queueHidTap(HID_TAP_MOUSE_BUTTON, 1);
                Serial.println("[Mouse] Left click (proximity)");
                break;
            case MODE_MEDIA:
                queueHidTap(HID_TAP_MEDIA, 4);
                Serial.println("[Media] Play/Pause (proximity)");
                break;
            default:
                break;
        }
    }
    lastProx = prox;
}

// ==================== OLED ====================
void oledShow(const char* gestureStr) {
    if (!oledOK) return;
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(1);
    display.setCursor(0, 0);
    display.println("Gesture 3Mode");
    display.setCursor(0, 18);
    display.print("Mode: ");
    display.println(modeNames[currentMode]);
    display.setCursor(0, 36);
    display.print("Gesture: ");
    display.println(gestureStr ? gestureStr : "-");
    display.setCursor(0, 54);
    display.println("BOOT=switch");
    display.display();
}

// ==================== 设置 ====================
void setup() {
    Serial.begin(115200);
    delay(200);

    // 蜂鸣器PWM
    ledcSetup(BUZZER_CHANNEL, BUZZER_FREQ, BUZZER_RES);
    ledcAttachPin(BUZZER_PIN, BUZZER_CHANNEL);
    ledcWrite(BUZZER_CHANNEL, 0);

    pinMode(BOOT_PIN, INPUT_PULLUP);

    Wire.begin(OLED_SDA, OLED_SCL);
    Wire.setClock(100000);

    if (display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
        oledOK = true;
        display.clearDisplay();
        display.setCursor(0, 24);
        display.println("Initializing...");
        display.display();
    } else {
        Serial.println("OLED init failed!");
    }

    uint8_t sensorId = 0;
    if (!apds.begin(sensorId)) {
        Serial.println("[Sensor] APDS9960 initialization failed; see sensor diagnostics above");
        while (1) delay(1000);
    }
    Serial.printf("[Sensor] I2C device at 0x39 initialized; chip ID=0x%02X\n",
                  sensorId);

    if (!initBLE()) {
        while (1) delay(1000);
    }

    oledShow(NULL);
    beepNonBlock(90);
    Serial.println("System ready");
}

// ==================== 主循环 ====================
void loop() {
    serviceHidTaps();

    // 蜂鸣器自动停止
    if (beepEndMs && millis() >= beepEndMs) {
        ledcWrite(BUZZER_CHANNEL, 0);
        beepEndMs = 0;
    }

    // ---- BOOT键切换 ----
    static unsigned long lastBootMs = 0;
    static bool bootWasLow = false;
    bool bootNow = (digitalRead(BOOT_PIN) == LOW);
    if (bootNow && !bootWasLow && (millis() - lastBootMs > 280)) {
        lastBootMs = millis();
        currentMode = (Mode)((currentMode + 1) % MODE_N);
        arcReset();
        oledShow("SWITCH");
        beepNonBlock(60);
        Serial.printf("Mode -> %s\n", modeNames[currentMode]);
    }
    bootWasLow = bootNow;

    // ---- 接近度 ----
    uint8_t proximity = 0;
    if (apds.readProximity(proximity)) {
        handleProximity(proximity);
    }

    // ---- 手势读取 ----
    const ApdsGesture raw = apds.readGesture();
    GDir dir = G_NONE;
    if (raw != ApdsGesture::None) {
        const char* rawName = "unknown";
        switch (raw) {
            case ApdsGesture::Up:    dir = G_UP;    rawName = "UP"; break;
            case ApdsGesture::Down:  dir = G_DOWN;  rawName = "DOWN"; break;
            case ApdsGesture::Left:  dir = G_LEFT;  rawName = "LEFT"; break;
            case ApdsGesture::Right: dir = G_RIGHT; rawName = "RIGHT"; break;
            default: break;
        }
        // ④ 只在传感器报告到手势时打印原始值，避免空闲时刷屏。
        Serial.printf("[Gesture] SparkFun direction=%s\n", rawName);
    }

    GDir gesturesToExecute[ARC_MAX + 1];
    uint8_t gestureCount = 0;

    if (currentMode == MODE_MOUSE && arcLen > 0 &&
        millis() - arcLastGestureMs >= ARC_TIMEOUT_MS) {
        for (int i = 0; i < arcLen; i++) {
            gesturesToExecute[gestureCount++] = arcQueue[i];
        }
        arcReset();
    }

    if (dir != G_NONE) {
        if (currentMode != MODE_MOUSE) {
            arcReset();
            gesturesToExecute[gestureCount++] = dir;
        } else if (arcLen == 0) {
            arcPush(dir);
            arcLastGestureMs = millis();
        } else {
            arcPush(dir);
            if (!isArcCandidate()) {
                for (int i = 0; i < arcLen - 1; i++) {
                    gesturesToExecute[gestureCount++] = arcQueue[i];
                }
                GDir latestDirection = arcQueue[arcLen - 1];
                arcReset();
                arcPush(latestDirection);
                arcLastGestureMs = millis();
            } else {
                arcLastGestureMs = millis();
                GDir arcGesture = checkArc();
                if (arcGesture != G_NONE) {
                    gesturesToExecute[gestureCount++] = arcGesture;
                    arcReset();
                }
            }
        }
    }

    for (uint8_t i = 0; i < gestureCount; i++) {
        executeGesture(gesturesToExecute[i]);
        oledShow(gestureName(gesturesToExecute[i]));
    }

    // 定期刷新OLED
    static unsigned long lastOledRefresh = 0;
    if (millis() - lastOledRefresh > 850) {
        lastOledRefresh = millis();
        oledShow(NULL);
    }

    // ⑤ 每 5 秒输出一次蓝牙连接和广播状态，便于区分识别问题与连接问题。
    static unsigned long lastBleStatusMs = 0;
    if (millis() - lastBleStatusMs > 5000) {
        lastBleStatusMs = millis();
        if (bleServer) {
            NimBLEAdvertising* advertising = bleServer->getAdvertising();
            const uint8_t connections = bleServer->getConnectedCount();
            if (connections == 0 && advertising && !advertising->isAdvertising()) {
                if (advertising->start()) {
                    Serial.println("[BLE] Advertising watchdog restarted advertising");
                } else {
                    Serial.println("[BLE] Advertising watchdog failed to restart advertising");
                }
            }
            Serial.printf("[BLE status] connections=%u, advertising=%s\n",
                          connections,
                          advertising && advertising->isAdvertising() ? "continue..." : "advertising stopped");
        }
    }

    delay(15);
}
