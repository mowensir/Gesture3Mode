/*
 * ============================================================
 *  手势三模式控制器 - 最终定稿（PAJ7620U2 版）
 *  硬件：ESP32-D0WD + OLED SSD1306(0x3C) + PAJ7620U2(0x73)
 *        无源蜂鸣器(GPIO5, PWM驱动) + BOOT键(GPIO0)
 *  蓝牙：NimBLE 复合 HID，单设备名 "Gesture 3Mode"
 *        同时提供 键盘 / 鼠标 / Android媒体 三类报告
 *  模式：BOOT 短按循环切换
 *        1.Keyboard(电脑PPT/键盘)  2.Mouse(电脑鼠标)  3.Media(安卓媒体)
 *  手势：PAJ7620U2 硬件直出 9 种手势，无需软件拼圆弧
 * ============================================================
 *
 *  接线（经典 ESP32-D0WD 30Pin）：
 *    OLED  VCC->3.3V  GND->GND  SDA->GPIO21  SCL->GPIO22 (地址0x3C)
 *    PAJ   VCC->3.3V  GND->GND  SDA->GPIO21  SCL->GPIO22 (地址0x73)
 *          若模块无板载上拉：SDA/SCL 各接 4.7kΩ 到 3.3V
 *          若模块有 PS 跳线且断开：VL 脚单独接 3.3V
 *    蜂鸣器 I/O->GPIO5  VCC->3.3V  GND->GND（无源，PWM驱动）
 *    BOOT键：GPIO0 内部上拉，按下=LOW（板载BOOT键即可）
 *
 *  上电后串口(115200)会打印 I2C 扫描结果：
 *    应看到 0x3C(OLED) 和 0x73(PAJ7620U2)
 * ============================================================
 */

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <RevEng_PAJ7620.h>
#include <NimBLEDevice.h>
#include <NimBLEServer.h>
#include <NimBLEHIDDevice.h>
#include <NimBLECharacteristic.h>

// ==================== 引脚 ====================
#define OLED_SDA    21
#define OLED_SCL    22
#define OLED_ADDR   0x3C
#define SCREEN_W    128
#define SCREEN_H    64
#define BUZZER_PIN  5
#define BOOT_PIN    0

// ==================== 蜂鸣器 PWM（无源，4kHz 方波）====================
#define BUZZER_CH   0
#define BUZZER_FREQ 4000
#define BUZZER_RES  8
static unsigned long beepEndMs = 0;

void beepNonBlock(int ms) {
    ledcWrite(BUZZER_CH, 128);          // 50% 占空比
    beepEndMs = millis() + ms;
}

// ==================== OLED ====================
Adafruit_SSD1306 display(SCREEN_W, SCREEN_H, &Wire, -1);
static bool oledOK = false;

// ==================== PAJ7620U2 ====================
RevEng_PAJ7620 paj = RevEng_PAJ7620();
static bool pajOK = false;

// ==================== 模式 ====================
enum Mode : uint8_t { MODE_KB = 0, MODE_MOUSE, MODE_MEDIA, MODE_N };
static const char* modeNames[] = {"1.Keyboard", "2.Mouse", "3.Media"};
static Mode currentMode = MODE_KB;

// ==================== BLE 复合 HID ====================
#define RID_KEYBOARD 1
#define RID_MOUSE    2
#define RID_MEDIA    3

static NimBLECharacteristic* inpKb = nullptr;   // Report ID 1
static NimBLECharacteristic* inpMs = nullptr;   // Report ID 2
static NimBLECharacteristic* inpCc = nullptr;   // Report ID 3
static bool deviceConnected = false;

// HID 报告描述符：键盘(1) + 鼠标(2) + 消费类媒体(3)
// 注意：BLE HID 每个 Report ID 对应独立 characteristic，
//      因此 setValue 的数据里【不再】带 Report ID 前缀。
static const uint8_t REPORT_MAP[] = {
    // ---------- Keyboard (Report ID 1) ----------
    0x05, 0x01,        // Usage Page (Generic Desktop)
    0x09, 0x06,        // Usage (Keyboard)
    0xA1, 0x01,        // Collection (Application)
    0x85, RID_KEYBOARD,//   Report ID (1)
    0x05, 0x07,        //   Usage Page (Key Codes)
    0x19, 0xE0,        //   Usage Minimum (224)
    0x29, 0xE7,        //   Usage Maximum (231)
    0x15, 0x00, 0x25, 0x01,
    0x75, 0x01, 0x95, 0x08, 0x81, 0x02,   // Modifier byte
    0x95, 0x01, 0x75, 0x08, 0x81, 0x01,   // Reserved (Const)
    0x95, 0x06, 0x75, 0x08, 0x15, 0x00, 0x25, 0x65,
    0x05, 0x07, 0x19, 0x00, 0x29, 0x65, 0x81, 0x00, // 6 key array
    0xC0,

    // ---------- Mouse (Report ID 2) ----------
    0x05, 0x01,        // Usage Page (Generic Desktop)
    0x09, 0x02,        // Usage (Mouse)
    0xA1, 0x01,        // Collection (Application)
    0x85, RID_MOUSE,   //   Report ID (2)
    0x09, 0x01,        //   Usage (Pointer)
    0xA1, 0x00,        //   Collection (Physical)
    0x05, 0x09,        //     Usage Page (Button)
    0x19, 0x01, 0x29, 0x05,
    0x15, 0x00, 0x25, 0x01,
    0x75, 0x01, 0x95, 0x05, 0x81, 0x02,   // 5 buttons
    0x95, 0x01, 0x75, 0x03, 0x81, 0x01,   // 3-bit padding
    0x05, 0x01,        //     Usage Page (Generic Desktop)
    0x09, 0x30, 0x09, 0x31, 0x09, 0x38,   //     X, Y, Wheel
    0x15, 0x81, 0x25, 0x7F,
    0x75, 0x08, 0x95, 0x03, 0x81, 0x06,   // 3 bytes signed (X,Y,Wheel)
    0xC0, 0xC0,

    // ---------- Consumer / Media (Report ID 3) ----------
    0x05, 0x0C,        // Usage Page (Consumer)
    0x09, 0x01,        // Usage (Consumer Control)
    0xA1, 0x01,        // Collection (Application)
    0x85, RID_MEDIA,   //   Report ID (3)
    0x15, 0x00, 0x26, 0xFF, 0x03,
    0x19, 0x00, 0x2A, 0xFF, 0x03,
    0x75, 0x10, 0x95, 0x01, 0x81, 0x00,   // 16-bit usage
    0xC0,
};

// ---------- HID 发送函数 ----------
// 键盘：8 字节 {modifier, reserved, key0..key5}
static void sendKey(uint8_t key) {
    if (!deviceConnected || !inpKb) return;
    uint8_t r[8] = {0x00, 0x00, key, 0, 0, 0, 0, 0};
    inpKb->setValue(r, 8); inpKb->notify();
    delay(20);
    uint8_t n[8] = {0};
    inpKb->setValue(n, 8); inpKb->notify();
}

// 鼠标：4 字节 {buttons, dx, dy, wheel}
static void sendMouse(int8_t dx, int8_t dy, int8_t wheel, uint8_t btn) {
    if (!deviceConnected || !inpMs) return;
    uint8_t r[4] = {btn, (uint8_t)dx, (uint8_t)dy, (uint8_t)wheel};
    inpMs->setValue(r, 4); inpMs->notify();
}
// 鼠标单击（按下再释放）
static void mouseClick(uint8_t buttonBit) {
    sendMouse(0, 0, 0, buttonBit);
    delay(30);
    sendMouse(0, 0, 0, 0);
}

// 媒体：2 字节 {usage低8, usage高8}，然后全 0 释放
static void sendConsumer(uint16_t usage) {
    if (!deviceConnected || !inpCc) return;
    uint8_t r[2] = {(uint8_t)(usage & 0xFF), (uint8_t)((usage >> 8) & 0xFF)};
    inpCc->setValue(r, 2); inpCc->notify();
    delay(30);
    uint8_t n[2] = {0, 0};
    inpCc->setValue(n, 2); inpCc->notify();
}

// ---------- BLE 服务器回调 ----------
class ServerCallbacks : public NimBLEServerCallbacks {
    void onConnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo) override {
        deviceConnected = true;
        Serial.println("[BLE] 已连接");
    }
    void onDisconnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo, int reason) override {
        deviceConnected = false;
        Serial.println("[BLE] 已断开，重新广播");
    }
};

static void initBLE() {
    NimBLEDevice::init("Gesture 3Mode");
    NimBLEDevice::setSecurityAuth(true, false, false);
    NimBLEServer* server = NimBLEDevice::createServer();
    server->setCallbacks(new ServerCallbacks());
    server->advertiseOnDisconnect(true);   // 断开自动重广播

    NimBLEHIDDevice* hid = new NimBLEHIDDevice(server);

    hid->setManufacturer("Espressif");
    hid->setHidInfo(0x00, 0x01);
    hid->setReportMap((uint8_t*)REPORT_MAP, sizeof(REPORT_MAP));
    inpKb = hid->getInputReport(RID_KEYBOARD);
    inpMs = hid->getInputReport(RID_MOUSE);
    inpCc = hid->getInputReport(RID_MEDIA);

    if (!server->start()) {
        Serial.println("[BLE] GATT 服务启动失败");
        return;
    }

    NimBLEAdvertising* adv = server->getAdvertising();
    adv->enableScanResponse(false);
    if (!adv->setName("Gesture 3Mode")) {
        Serial.println("[BLE] 无法将设备名加入主广播包");
        return;
    }
    if (!adv->setAppearance(0x03C1) ||
        !adv->addServiceUUID(hid->getHidService()->getUUID())) {
        Serial.println("[BLE] BLE 广播数据配置失败");
        return;
    }
    if (!adv->start()) {
        Serial.println("[BLE] BLE 广播启动失败");
        return;
    }
    Serial.println("[BLE] 广播中，设备名: Gesture 3Mode");
}

// ==================== OLED 显示 ====================
static void oledShow(const char* gestureStr) {
    if (!oledOK) return;
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);  display.println("Gesture 3Mode");
    display.setCursor(0, 16); display.print("Mode: "); display.println(modeNames[currentMode]);
    display.setCursor(0, 32); display.print("Gesture: "); display.println(gestureStr ? gestureStr : "-");
    display.setCursor(0, 48); display.print("BLE: "); display.println(deviceConnected ? "linked" : "searching");
    display.setCursor(0, 58); display.println("BOOT=switch");
    display.display();
}

// ==================== 手势 -> 动作 ====================
static void doGesture(Gesture g) {
    if (g == GES_NONE) return;
    const char* tag = "-";

    if (currentMode == MODE_KB) {
        // 模式1 电脑控制：上/下=音量，左/右=翻页，前=播放/暂停，挥手=静音
        switch (g) {
            case GES_UP:      sendConsumer(0x00E9); tag = "Vol+";        break;
            case GES_DOWN:    sendConsumer(0x00EA); tag = "Vol-";        break;
            case GES_LEFT:    sendKey(0x4B);         tag = "PageUp";      break;
            case GES_RIGHT:   sendKey(0x4E);         tag = "PageDown";    break;
            case GES_FORWARD: sendConsumer(0x00CD); tag = "Play/Pause";  break;
            case GES_WAVE:    sendConsumer(0x00E2); tag = "Mute";        break;
            default: return;   // 其余手势在键盘模式不响应
        }
    } else if (currentMode == MODE_MOUSE) {
        // 模式2 电脑鼠标：四向移动±20，顺/逆=滚轮，靠近=左键，远离=右键
        switch (g) {
            case GES_UP:            sendMouse(0, -20, 0, 0); tag = "MoveUp";    break;
            case GES_DOWN:          sendMouse(0,  20, 0, 0); tag = "MoveDown";  break;
            case GES_LEFT:          sendMouse(-20, 0, 0, 0); tag = "MoveLeft";  break;
            case GES_RIGHT:         sendMouse(20,  0, 0, 0); tag = "MoveRight"; break;
            case GES_CLOCKWISE:     sendMouse(0, 0, 1, 0);   tag = "WheelUp";   break;
            case GES_ANTICLOCKWISE: sendMouse(0, 0, -1, 0);  tag = "WheelDown"; break;
            case GES_FORWARD:       mouseClick(0x01);        tag = "LeftClick"; break;
            case GES_BACKWARD:      mouseClick(0x02);        tag = "RightClick";break;
            default: return;
        }
    } else if (currentMode == MODE_MEDIA) {
        // 模式3 安卓媒体：上=音量+ 下=音量- 左=上一曲 右=下一曲 靠近=播放/暂停 挥手=静音
        switch (g) {
            case GES_UP:      sendConsumer(0x00E9); tag = "Vol+";   break;
            case GES_DOWN:    sendConsumer(0x00EA); tag = "Vol-";   break;
            case GES_LEFT:    sendConsumer(0x00B6); tag = "Prev";   break;
            case GES_RIGHT:   sendConsumer(0x00B5); tag = "Next";   break;
            case GES_FORWARD: sendConsumer(0x00CD); tag = "Play/Pause"; break;
            case GES_WAVE:    sendConsumer(0x00E2); tag = "Mute";   break;
            default: return;
        }
    }
    Serial.printf("[%s] %s\n", modeNames[currentMode], tag);
    oledShow(tag);
}

static const char* gestureName(Gesture g) {
    switch (g) {
        case GES_UP:            return "UP";
        case GES_DOWN:          return "DOWN";
        case GES_LEFT:          return "LEFT";
        case GES_RIGHT:         return "RIGHT";
        case GES_FORWARD:       return "FORWARD";
        case GES_BACKWARD:      return "BACKWARD";
        case GES_CLOCKWISE:     return "CW";
        case GES_ANTICLOCKWISE: return "CCW";
        case GES_WAVE:          return "WAVE";
        default:                return "-";
    }
}

// ==================== I2C 扫描（上电自检）====================
static void i2cScan() {
    Serial.println("--- I2C 扫描 (SDA=21, SCL=22) ---");
    for (uint8_t a = 8; a < 120; a++) {
        Wire.beginTransmission(a);
        if (Wire.endTransmission() == 0) Serial.printf("  找到设备 0x%02X\n", a);
    }
    Serial.println("--- 扫描结束（应有 0x3C=OLED, 0x73=PAJ7620U2）---");
}

// ==================== setup ====================
void setup() {
    Serial.begin(115200);
    delay(500);

    pinMode(BOOT_PIN, INPUT_PULLUP);

    // 蜂鸣器 PWM
    ledcSetup(BUZZER_CH, BUZZER_FREQ, BUZZER_RES);
    ledcAttachPin(BUZZER_PIN, BUZZER_CH);
    ledcWrite(BUZZER_CH, 0);

    // I2C（经典 ESP32 常用 SDA=21, SCL=22）
    Wire.begin(OLED_SDA, OLED_SCL);
    Wire.setClock(100000);

    // OLED
    if (display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
        oledOK = true;
        display.clearDisplay();
        display.setTextSize(1);
        display.setTextColor(SSD1306_WHITE);
        display.setCursor(0, 24);
        display.println("Init...");
        display.display();
        Serial.println("[OLED] 0x3C 就绪");
    } else {
        Serial.println("[OLED] 0x3C 初始化失败（检查接线/上拉）");
    }

    // PAJ7620U2（地址内部固定 0x73，库自动处理）
    if (paj.begin(&Wire)) {
        pajOK = true;
        paj.setGestureEntryTime(200);   // 手就位时间，减少误判
        paj.setGestureExitTime(500);    // 手势退出时间
        Serial.println("[PAJ] 0x73 PAJ7620U2 就绪");
    } else {
        Serial.println("[PAJ] 0x73 初始化失败（检查 3.3V 供电/上拉/PS跳线）");
    }

    i2cScan();
    initBLE();
    oledShow(NULL);
    beepNonBlock(90);
    Serial.println("=== 系统就绪，挥手测试 ===");
}

// ==================== loop ====================
void loop() {
    // 蜂鸣器定时关闭
    if (beepEndMs && millis() >= beepEndMs) {
        ledcWrite(BUZZER_CH, 0);
        beepEndMs = 0;
    }

    // BOOT 键切换模式（300ms 防抖）
    static unsigned long lastBootMs = 0;
    static bool bootWasLow = false;
    bool bootNow = (digitalRead(BOOT_PIN) == LOW);
    if (bootNow && !bootWasLow && (millis() - lastBootMs > 300)) {
        lastBootMs = millis();
        currentMode = (Mode)((currentMode + 1) % MODE_N);
        oledShow("SWITCH");
        beepNonBlock(70);
        Serial.printf("切换模式 -> %s\n", modeNames[currentMode]);
    }
    bootWasLow = bootNow;

    // 手势读取（350ms 冷却，避免连续重复触发）
    static unsigned long lastGestureMs = 0;
    if (pajOK && (millis() - lastGestureMs > 350)) {
        Gesture g = paj.readGesture();
        if (g != GES_NONE) {
            lastGestureMs = millis();
            Serial.printf("[Raw] %s\n", gestureName(g));  // 原始手势名，方便调试
            doGesture(g);          // 内部已更新 OLED 与串口日志
        }
    }

    // OLED 定期刷新（回显模式，约 900ms）
    static unsigned long lastOledRefresh = 0;
    if (millis() - lastOledRefresh > 900) {
        lastOledRefresh = millis();
        oledShow(NULL);
    }

    delay(15);
}
