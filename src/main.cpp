/*
 * 三模式手势控制器 - 最终定稿
 * 硬件：ESP32 + OLED(0x3C) + APDS9960(0x39) + 无源蜂鸣器(GPIO5)
 * 蓝牙复合HID：键盘、鼠标、媒体
 * 模式切换：BOOT键(GPIO0)短按
 * 蜂鸣器：PWM非阻塞
 * 手势：圆弧状态机（顺/逆时针）+ 接近度
 * 2026-06-15
 */

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_APDS9960.h>
#include <NimBLEDevice.h>
#include <NimBLEHIDDevice.h>
#include <NimBLECharacteristic.h>
#include <string.h>

// ==================== 引脚定义 ====================
#define OLED_SDA    21
#define OLED_SCL    22
#define OLED_ADDR   0x3C
#define SCREEN_W    128
#define SCREEN_H    64
#define BUZZER_PIN  5
#define BOOT_PIN    0
#define APDS_ADDR   0x39

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
Adafruit_APDS9960 apds;

bool writeReg(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(APDS_ADDR);
    Wire.write(reg);
    Wire.write(val);
    return (Wire.endTransmission() == 0);
}

bool readReg(uint8_t reg, uint8_t& val) {
    Wire.beginTransmission(APDS_ADDR);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0 ||
        Wire.requestFrom((uint8_t)APDS_ADDR, (uint8_t)1) != 1) {
        return false;
    }
    val = Wire.read();
    return true;
}

bool writeRegChecked(uint8_t reg, uint8_t val) {
    if (!writeReg(reg, val)) {
        Serial.printf("[Sensor] Register 0x%02X write failed\n", reg);
        return false;
    }

    uint8_t actual = 0;
    if (!readReg(reg, actual)) {
        Serial.printf("[Sensor] Register 0x%02X read-back failed\n", reg);
        return false;
    }
    if (actual != val) {
        Serial.printf("[Sensor] Register 0x%02X mismatch: wrote 0x%02X, read 0x%02X\n",
                      reg, val, actual);
        return false;
    }
    Serial.printf("[Sensor] Register 0x%02X verified: 0x%02X\n", reg, actual);
    return true;
}

bool configureGestureRegisters() {
    bool ok = true;
    ok &= writeRegChecked(0x8F, 0x08); // CONTROL：4x proximity gain，100 mA LED drive
    ok &= writeRegChecked(0x8E, 0x87); // PPULSE：16 us，8 个 proximity LED 脉冲
    ok &= writeRegChecked(0xA0, 0x06); // GPENTH：低反射兼容芯片进入阈值 6
    ok &= writeRegChecked(0xA1, 0x03); // GEXTH：退出阈值 3
    ok &= writeRegChecked(0xA2, 0x40); // GCONF1：FIFO 达到 4 组数据后报告有效
    ok &= writeRegChecked(0xA3, 0x20); // GCONF2：2x 手势增益
    ok &= writeRegChecked(0xA6, 0x81); // GPULSE：2 个 16 us 脉冲（计数值 + 1），与厂家示例一致
    ok &= writeRegChecked(0xAA, 0x00); // GCONF3：启用上下、左右两组光电二极管
    ok &= writeRegChecked(0xAB, 0x00); // GCONF4：先退出手势模式，再由库启动
    return ok;
}

bool initializeCompatibleApds() {
    // 兼容芯片 ID 不同，直接按 APDS9960 寄存器初始化，不依赖 begin() 的 ID 检查副作用。
    if (!writeRegChecked(0x80, 0x00)) return false;
    delay(10);

    bool ok = true;
    ok &= writeRegChecked(0x81, 0xFC); // ATIME：约 11.12 ms 积分时间
    ok &= writeRegChecked(0x83, 0xFF); // WTIME：最短等待时间
    ok &= configureGestureRegisters();
    if (!ok) return false;

    if (!writeRegChecked(0x80, 0x01)) return false; // 仅上电
    delay(10);
    if (!writeRegChecked(0x80, 0x05)) return false; // 上电 + 接近度
    delay(10);
    if (!writeRegChecked(0x80, 0x45)) return false; // 上电 + 接近度 + 手势引擎
    if (!writeRegChecked(0xAB, 0x01)) return false; // 启动手势状态机

    Serial.println("[Sensor] Compatible APDS9960-register initialization complete");
    return true;
}

uint8_t readSensorProximity() {
    uint8_t proximity = 0;
    if (!readReg(0x9C, proximity)) {
        static unsigned long lastErrorMs = 0;
        if (millis() - lastErrorMs >= 1000) {
            lastErrorMs = millis();
            Serial.println("[Sensor] Failed to read proximity register 0x9C");
        }
        return 0;
    }
    return proximity;
}

uint8_t readSensorGesture() {
    static uint8_t upCount = 0;
    static uint8_t downCount = 0;
    static uint8_t leftCount = 0;
    static uint8_t rightCount = 0;
    static unsigned long lastMotionMs = 0;
    uint8_t status = 0;
    if (!readReg(0xAF, status)) {
        static unsigned long lastErrorMs = 0;
        if (millis() - lastErrorMs >= 1000) {
            lastErrorMs = millis();
            Serial.println("[Sensor] Failed to read gesture status register 0xAF");
        }
        return 0;
    }
    const bool fifoOverflow = (status & 0x02) != 0;
    if (fifoOverflow) {
        static unsigned long lastOverflowLogMs = 0;
        if (millis() - lastOverflowLogMs >= 1000) {
            lastOverflowLogMs = millis();
            Serial.println("[Sensor] Gesture FIFO overflow; some samples were lost");
        }
    }
    uint8_t fifoLevel = 0;
    if (!readReg(0xAE, fifoLevel)) {
        Serial.println("[Sensor] Failed to read gesture FIFO level register 0xAE");
        return 0;
    }
    if (fifoLevel > 32) {
        Serial.printf("[Sensor] Invalid gesture FIFO level: %u\n", fifoLevel);
        return 0;
    }
    if (fifoLevel == 0) {
        static unsigned long lastIdleLogMs = 0;
        if (millis() - lastIdleLogMs >= 1000) {
            lastIdleLogMs = millis();
            uint8_t enable = 0;
            uint8_t gestureConfig = 0;
            uint8_t proximity = 0;
            readReg(0x80, enable);
            readReg(0xAB, gestureConfig);
            readReg(0x9C, proximity);
            Serial.printf("[Gesture] idle: GSTATUS=0x%02X GFLVL=%u ENABLE=0x%02X GCONF4=0x%02X PROX=%u\n",
                          status, fifoLevel, enable, gestureConfig, proximity);
        }
        if (lastMotionMs != 0 && millis() - lastMotionMs > 300) {
            upCount = downCount = leftCount = rightCount = 0;
            lastMotionMs = 0;
        }
        return 0;
        
    }
    uint8_t detectedGesture = 0;
    for (uint8_t i = 0; i < fifoLevel; ++i) {
        Wire.beginTransmission(APDS_ADDR);
        Wire.write(0xFC);
        if (Wire.endTransmission(false) != 0 ||
            Wire.requestFrom((uint8_t)APDS_ADDR, (uint8_t)4) != 4) {
            Serial.println("[Sensor] Failed to read a complete UDLR FIFO sample");
            while (Wire.available()) Wire.read();
            break;
        }

        const int up = Wire.read();
        const int down = Wire.read();
        const int left = Wire.read();
        const int right = Wire.read();
        if (detectedGesture != 0 || fifoOverflow) continue;

        const int upDownDiff = (abs(up - down) > 13) ? up - down : 0;
        const int leftRightDiff = (abs(left - right) > 13) ? left - right : 0;
        if (upDownDiff == 0 && leftRightDiff == 0) continue;
        lastMotionMs = millis();

        if (upDownDiff < 0) {
            if (downCount > 0) detectedGesture = APDS9960_UP;
            else if (upCount < UINT8_MAX) ++upCount;
        } else if (upDownDiff > 0) {
            if (upCount > 0) detectedGesture = APDS9960_DOWN;
            else if (downCount < UINT8_MAX) ++downCount;
        }

        if (leftRightDiff < 0) {
            if (rightCount > 0) detectedGesture = APDS9960_LEFT;
            else if (leftCount < UINT8_MAX) ++leftCount;
        } else if (leftRightDiff > 0) {
            if (leftCount > 0) detectedGesture = APDS9960_RIGHT;
            else if (rightCount < UINT8_MAX) ++rightCount;
        }

        if (detectedGesture != 0) {
            upCount = downCount = leftCount = rightCount = 0;
            lastMotionMs = 0;
        }
    }
    if (fifoOverflow) {
        upCount = downCount = leftCount = rightCount = 0;
        lastMotionMs = 0;
        return 0;
    }
    if (detectedGesture == 0 && lastMotionMs != 0 && millis() - lastMotionMs > 300) {
        upCount = downCount = leftCount = rightCount = 0;
        lastMotionMs = 0;
    }
    return detectedGesture;
}

// ==================== 模式 ====================
enum Mode : uint8_t { MODE_KB = 0, MODE_MOUSE, MODE_MEDIA, MODE_N };
const char* modeNames[] = {"1.Keyboard", "2.Mouse", "3.Media"};
Mode currentMode = MODE_KB;

// ==================== 手势方向 ====================
enum GDir { G_NONE, G_UP, G_DOWN, G_LEFT, G_RIGHT, G_CW, G_CCW };

// ==================== 圆弧状态机 ====================
#define ARC_MAX 6
GDir arcQueue[ARC_MAX];
int arcLen = 0;

void arcReset() { arcLen = 0; }

void arcPush(GDir d) {
    if (d == G_NONE) return;
    if (arcLen > 0 && arcQueue[arcLen - 1] == d) return;
    if (arcLen >= ARC_MAX) {
        memmove(arcQueue, arcQueue + 1, (ARC_MAX - 1) * sizeof(GDir));
        arcLen--;
    }
    arcQueue[arcLen++] = d;
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

// ==================== BLE 复合 HID ====================
NimBLEHIDDevice* hid;
NimBLECharacteristic* inpKb;
NimBLECharacteristic* inpMouse;
NimBLECharacteristic* inpMedia;
NimBLEServer* bleServer;

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
        (void)connInfo;
        Serial.printf("[BLE] Connected; active connections: %u\n", server->getConnectedCount());
    }

    void onDisconnect(NimBLEServer* server, NimBLEConnInfo& connInfo, int reason) override {
        (void)server;
        (void)connInfo;
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
    Serial.printf("[BLE] %s report notification accepted\n", reportName);
    return true;
}

void sendKb(uint8_t mod, uint8_t key) {
    uint8_t report[7] = {mod, 0, key, 0, 0, 0, 0};
    inpKb->setValue(report, sizeof(report));
    sendHidReport(inpKb, "keyboard");
}
void releaseKb() { sendKb(0, 0); }

void sendMouse(uint8_t btn, int8_t dx, int8_t dy, int8_t dw) {
    uint8_t report[4] = {btn, (uint8_t)dx, (uint8_t)dy, (uint8_t)dw};
    inpMouse->setValue(report, sizeof(report));
    sendHidReport(inpMouse, "mouse");
}

void sendMediaBit(uint8_t bit) {
    uint8_t report = (uint8_t)(1 << bit);
    inpMedia->setValue(&report, sizeof(report));
    sendHidReport(inpMedia, "media");
    delay(20);
    report = 0;
    inpMedia->setValue(&report, sizeof(report));
    sendHidReport(inpMedia, "media release");
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
                case G_UP:    sendKb(0, 0x4B); releaseKb(); tag = "PageUp"; break;
                case G_DOWN:  sendKb(0, 0x4E); releaseKb(); tag = "PageDown"; break;
                case G_LEFT:  sendKb(0, 0x50); releaseKb(); tag = "Left"; break;
                case G_RIGHT: sendKb(0, 0x28); releaseKb(); tag = "Enter"; break;
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
                case G_UP:    sendMediaBit(0); tag = "Vol+"; break;
                case G_DOWN:  sendMediaBit(1); tag = "Vol-"; break;
                case G_LEFT:  sendMediaBit(3); tag = "Prev"; break;
                case G_RIGHT: sendMediaBit(2); tag = "Next"; break;
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
                sendMouse(1, 0, 0, 0);
                delay(30);
                sendMouse(0, 0, 0, 0);
                Serial.println("[Mouse] Left click (proximity)");
                break;
            case MODE_MEDIA:
                sendMediaBit(4); // Play/Pause
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

    // ① 读取并打印芯片 ID；同时确认 I2C 地址 0x39 确实有应答。
    uint8_t sensorId = 0;
    if (!readReg(0x92, sensorId)) {
        Serial.println("[Sensor] No I2C response at 0x39 or failed to read chip ID");
        while (1) delay(1000);
    }
    Serial.printf("[Sensor] I2C device at 0x39 responded; chip ID=0x%02X (Adafruit standard: 0xAB)\n",
                  sensorId);
    if (sensorId == 0x00 || sensorId == 0xFF) {
        Serial.println("[Sensor] Invalid chip ID; stopping to avoid writing to an unrelated I2C device");
        while (1) delay(1000);
    }

    // 标准芯片使用 Adafruit 初始化；不同 ID 的兼容芯片只通过数据手册寄存器接口初始化。
    if (sensorId == 0xAB) {
        if (!apds.begin(10, APDS9960_AGAIN_4X, APDS_ADDR, &Wire)) {
            Serial.println("[Sensor] Standard APDS9960 initialization failed");
            while (1) delay(1000);
        }
        Serial.println("[Sensor] Standard APDS9960 initialization succeeded");
        if (!configureGestureRegisters()) {
            Serial.println("[Sensor] Register configuration verification failed; stopping");
            while (1) delay(1000);
        }
        apds.enableProximity(true);
        apds.enableGesture(true);
    } else {
        Serial.println("[Sensor] Nonstandard ID; trying compatible init. Confirm APDS9960 register compatibility");
        if (!initializeCompatibleApds()) {
            Serial.println("[Sensor] Compatible initialization failed; stopping");
            while (1) delay(1000);
        }
    }

    uint8_t enableReg = 0;
    if (!readReg(0x80, enableReg) || enableReg != 0x45) {
        Serial.printf("[Sensor] Enable verification failed: ENABLE expected 0x45, got 0x%02X\n", enableReg);
        while (1) delay(1000);
    }
    Serial.println("[Sensor] Proximity and gesture enabled; ENABLE=0x45");

    if (!initBLE()) {
        while (1) delay(1000);
    }

    oledShow(NULL);
    beepNonBlock(90);
    Serial.println("System ready");
}

// ==================== 主循环 ====================
void loop() {
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
    uint16_t proximity = readSensorProximity();
    handleProximity(proximity);

    // ---- 手势读取 ----
    static unsigned long lastGestureMs = 0;
    uint8_t raw = readSensorGesture();
    GDir dir = G_NONE;
    if (raw) {
        const char* rawName = "未知";
        switch (raw) {
            case APDS9960_UP:    dir = G_UP;    rawName = "UP"; break;
            case APDS9960_DOWN:  dir = G_DOWN;  rawName = "DOWN"; break;
            case APDS9960_LEFT:  dir = G_LEFT;  rawName = "LEFT"; break;
            case APDS9960_RIGHT: dir = G_RIGHT; rawName = "RIGHT"; break;
        }
        // ④ 只在传感器报告到手势时打印原始值，避免空闲时刷屏。
        Serial.printf("[Gesture] Sensor raw value=0x%02X, direction=%s\n", raw, rawName);
    }

    arcPush(dir);
    GDir finalGesture = checkArc();
    if (finalGesture == G_NONE && dir != G_NONE) {
        finalGesture = dir;
    }

    if (finalGesture != G_NONE && (millis() - lastGestureMs > 350)) {
        lastGestureMs = millis();
        executeGesture(finalGesture);
        const char* txt = "?";
        switch (finalGesture) {
            case G_UP:    txt = "UP";    break;
            case G_DOWN:  txt = "DOWN";  break;
            case G_LEFT:  txt = "LEFT";  break;
            case G_RIGHT: txt = "RIGHT"; break;
            case G_CW:    txt = "CW";    break;
            case G_CCW:   txt = "CCW";   break;
            default: break;
        }
        oledShow(txt);
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
            Serial.printf("[BLE status] connections=%u, advertising=%s\n",
                          bleServer->getConnectedCount(),
                          advertising && advertising->isAdvertising() ? "contine..." : "advertising stopped");
        }
    }

    delay(15);
}
