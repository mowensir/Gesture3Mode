/*
 * ============================================================
 *  手势三模式控制器 - 最终定稿（PAJ7620U2 版）
 *  硬件：ESP32-D0WD + OLED SSD1306(0x3C) + PAJ7620U2(0x73)
 *        无源蜂鸣器(GPIO5, PWM驱动) + BOOT键(GPIO0)
 *  蓝牙：NimBLE 复合 HID，单设备名 "Gesture 3Mode"
 *        同时提供 键盘 / 鼠标 / Android媒体 三类报告
 *  模式：BOOT 短按循环切换
 *        1.Presenter(演示)  2.Browser(浏览)  3.Media(媒体)
 *  手势：PAJ7620U2 硬件直出 9 种手势，无需软件拼圆弧
 * ============================================================
 *
 *  接线（经典 ESP32-D0WD 30Pin）：
 *    OLED  VCC->3.3V  GND->GND  SDA->GPIO21  SCL->GPIO22  (地址0x3C)
 *    PAJ   VCC->3.3V  GND->GND  SDA->GPIO21  SCL->GPIO22  (地址0x73)
 *          若模块无板载上拉：SDA/SCL 各接 4.7kΩ 到 3.3V
 *          若模块有 PS 跳线且断开：VL 脚单独接 3.3V
 *    蜂鸣器 I/O->GPIO5  VCC->3.3V  GND->GND（无源，PWM驱动）
*    BOOT键：GPIO0 内部上拉，按下=LOW（板载BOOT键即可       ）
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
static constexpr uint8_t OLED_SDA = 21;
static constexpr uint8_t OLED_SCL = 22;
static constexpr uint8_t OLED_ADDR = 0x3C;
static constexpr int SCREEN_W = 128;
static constexpr int SCREEN_H = 64;
static constexpr uint8_t BUZZER_PIN = 5;
static constexpr uint8_t BOOT_PIN = 0;

// ==================== 蜂鸣器 PWM（无源，4kHz 方波）====================
static constexpr uint8_t BUZZER_CH = 0;
static constexpr uint16_t BUZZER_FREQ = 4000;
static constexpr uint8_t BUZZER_RES = 8;
static unsigned long beepEndMs = 0;

// PAJ7620U2 手势判定参数；可调整进入/退出判定时间以适配手势速度与误触情况。
static constexpr unsigned long PAJ_GESTURE_ENTRY_TIME_MS = 100;
static constexpr unsigned long PAJ_GESTURE_EXIT_TIME_MS = 300;
// 两次手势读取之间的最短间隔，用于抑制传感器对同一动作的重复报告。
static constexpr unsigned long GESTURE_COOLDOWN_MS = 250;
// 可调节各动作的重复次数；演示模式音量/媒体方向键默认 3 次，浏览滚轮/媒体音量默认 5 次。
static constexpr uint8_t PRESENTER_VOLUME_REPEAT_COUNT = 3;
static constexpr uint8_t BROWSER_SCROLL_REPEAT_COUNT = 5;
static constexpr uint8_t MEDIA_VOLUME_REPEAT_COUNT = 5;
static constexpr uint8_t MEDIA_SEEK_REPEAT_COUNT = 3;
    // HID 按下保持时间、重复动作间隔及 BOOT 按键防抖时间（毫秒）。
static constexpr unsigned long KEY_PRESS_DURATION_MS = 20;
static constexpr unsigned long CONSUMER_PRESS_DURATION_MS = 30;
static constexpr unsigned long HID_REPEAT_INTERVAL_MS = 25;
static constexpr unsigned long BOOT_DEBOUNCE_MS = 30;
// RSSI 查询及 OLED 状态页周期刷新间隔（毫秒）。
static constexpr unsigned long STATUS_REFRESH_INTERVAL_MS = 2000;

static void beepNonBlock(unsigned long durationMs) {
    ledcWrite(BUZZER_CH, 128);          // 50% 占空比
    beepEndMs = millis() + durationMs;
}

// ==================== OLED ====================
static Adafruit_SSD1306 display(SCREEN_W, SCREEN_H, &Wire, -1);
static bool oledOK = false;

// ==================== PAJ7620U2 ====================
static RevEng_PAJ7620 paj;
static bool pajOK = false;

// ==================== 模式 ====================
enum Mode : uint8_t { MODE_PRESENTER = 0, MODE_BROWSER, MODE_MEDIA, MODE_N };
static const char* const modeNames[] = {"Presenter", "Browser", "Media"};
static Mode currentMode = MODE_PRESENTER;

// ==================== BLE 复合 HID ====================
static constexpr uint8_t HID_REPORT_ID_KEYBOARD = 1;
static constexpr uint8_t HID_REPORT_ID_MOUSE = 2;
static constexpr uint8_t HID_REPORT_ID_CONSUMER_CONTROL = 3;

static NimBLEHIDDevice* hidDevice = nullptr;
static NimBLECharacteristic* keyboardInputReport = nullptr;
static NimBLECharacteristic* mouseInputReport = nullptr;
static NimBLECharacteristic* consumerControlInputReport = nullptr;
static NimBLEServer* bleServer = nullptr;
static volatile bool deviceConnected = false;
static int8_t bleRssi = 0; // Accessed only from the main loop.
static volatile bool bleRssiValid = false;
static const char* lastGestureTag = "-";
static uint32_t totalGestures = 0;
static unsigned long lastGestureTime = 0;

// HID 报告描述符：键盘(1) + 鼠标(2) + 消费类媒体(3)
// 注意：BLE HID 每个 Report ID 对应独立 characteristic，
//      因此 setValue 的数据里【不再】带 Report ID 前缀。
static const uint8_t REPORT_MAP[] = {
    // ---------- Keyboard (Report ID 1) ----------
    0x05, 0x01,        // Usage Page (Generic Desktop)
    0x09, 0x06,        // Usage (Keyboard)
    0xA1, 0x01,        // Collection (Application)
    0x85, HID_REPORT_ID_KEYBOARD,// Report ID (1)
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
    0x85, HID_REPORT_ID_MOUSE, // Report ID (2)
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
    0x85, HID_REPORT_ID_CONSUMER_CONTROL, // Report ID (3)
    0x15, 0x00, 0x26, 0xFF, 0x03,
    0x19, 0x00, 0x2A, 0xFF, 0x03,
    0x75, 0x10, 0x95, 0x01, 0x81, 0x00,   // 16-bit usage
    0xC0,
};

// ---------- HID 发送函数 ----------
// 发送单个键盘按下报告并及时发送释放报告，避免主机持续认为按键被按住。
static void sendKey(uint8_t modifier, uint8_t key) {
    if (!deviceConnected || !keyboardInputReport) return;
    uint8_t r[8] = {modifier, 0x00, key, 0, 0, 0, 0, 0};
    keyboardInputReport->setValue(r, 8); keyboardInputReport->notify();
    delay(KEY_PRESS_DURATION_MS);
    uint8_t n[8] = {0};
    keyboardInputReport->setValue(n, 8); keyboardInputReport->notify();
}

static void sendKey(uint8_t key) {
    sendKey(0, key);
}

// 发送鼠标相对位移/滚轮报告；调用方负责需要时另发零值释放报告。
static void sendMouse(int8_t dx, int8_t dy, int8_t wheel, uint8_t btn) {
    if (!deviceConnected || !mouseInputReport) return;
    uint8_t r[4] = {btn, (uint8_t)dx, (uint8_t)dy, (uint8_t)wheel};
    mouseInputReport->setValue(r, 4); mouseInputReport->notify();
}
// 发送 Consumer Control 按下报告并发送零值释放报告。
static void sendConsumer(uint16_t usage) {
    if (!deviceConnected || !consumerControlInputReport) return;
    uint8_t r[2] = {(uint8_t)(usage & 0xFF), (uint8_t)((usage >> 8) & 0xFF)};
    consumerControlInputReport->setValue(r, 2); consumerControlInputReport->notify();
    delay(CONSUMER_PRESS_DURATION_MS);
    uint8_t n[2] = {0, 0};
    consumerControlInputReport->setValue(n, 2); consumerControlInputReport->notify();
}

// 按指定次数重复 Consumer Control 操作，便于调节快速音量等动作的步幅。
static void repeatConsumer(uint16_t usage, uint8_t count) {
    for (uint8_t i = 0; i < count; i++) {
        sendConsumer(usage);
        if (i + 1 < count) delay(HID_REPEAT_INTERVAL_MS);
    }
}

// 按指定次数重复键盘按键，媒体快进/快退通过重复方向键实现。
static void repeatKey(uint8_t modifier, uint8_t key, uint8_t count) {
    for (uint8_t i = 0; i < count; i++) {
        sendKey(modifier, key);
        if (i + 1 < count) delay(HID_REPEAT_INTERVAL_MS);
    }
}

// 重复发送滚轮脉冲，每次脉冲后立即释放滚轮值。
static void repeatWheel(int8_t direction, uint8_t count) {
    for (uint8_t i = 0; i < count; i++) {
        sendMouse(0, 0, direction, 0);
        sendMouse(0, 0, 0, 0);
        if (i + 1 < count) delay(HID_REPEAT_INTERVAL_MS);
    }
}

// ---------- BLE 服务器回调 ----------
class ServerCallbacks : public NimBLEServerCallbacks {
    void onConnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo) override {
        deviceConnected = true;
        bleRssiValid = false;
        Serial.println("[BLE] 已连接");
    }
    void onDisconnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo, int reason) override {
        deviceConnected = false;
        bleRssiValid = false;
        Serial.println("[BLE] 已断开，重新广播");
    }
};

// 初始化复合 BLE HID 服务及广播；失败时返回 false，由 setup() 报告错误。
static bool initBLE() {
    NimBLEDevice::init("Gesture 3Mode");
    NimBLEDevice::setSecurityAuth(true, false, false);
    bleServer = NimBLEDevice::createServer();
    if (!bleServer) {
        Serial.println("[BLE] 创建 GATT 服务器失败");
        return false;
    }
    bleServer->setCallbacks(new ServerCallbacks());
    bleServer->advertiseOnDisconnect(true);   // 断开自动重广播

    hidDevice = new NimBLEHIDDevice(bleServer);
    if (!hidDevice) {
        Serial.println("[BLE] 创建 HID 设备失败");
        return false;
    }

    hidDevice->setManufacturer("Espressif");
    hidDevice->setHidInfo(0x00, 0x01);
    hidDevice->setReportMap((uint8_t*)REPORT_MAP, sizeof(REPORT_MAP));
    keyboardInputReport = hidDevice->getInputReport(HID_REPORT_ID_KEYBOARD);
    mouseInputReport = hidDevice->getInputReport(HID_REPORT_ID_MOUSE);
    consumerControlInputReport = hidDevice->getInputReport(HID_REPORT_ID_CONSUMER_CONTROL);

    if (!bleServer->start()) {
        Serial.println("[BLE] GATT 服务启动失败");
        return false;
    }

    NimBLEAdvertising* adv = bleServer->getAdvertising();
    if (!adv) {
        Serial.println("[BLE] 获取广播实例失败");
        return false;
    }
    adv->enableScanResponse(false);
    if (!adv->setName("Gesture 3Mode")) {
        Serial.println("[BLE] 无法将设备名加入主广播包");
        return false;
    }
    if (!adv->setAppearance(0x03C1) ||
        !adv->addServiceUUID(hidDevice->getHidService()->getUUID())) {
        Serial.println("[BLE] BLE 广播数据配置失败");
        return false;
    }
    if (!adv->start()) {
        Serial.println("[BLE] BLE 广播启动失败");
        return false;
    }
    Serial.println("[BLE] 广播中，设备名: Gesture 3Mode");
    return true;
}

// ==================== OLED 显示 ====================
// 绘制当前模式、最近动作、连接/RSSI 和手势统计；缓存手势标签不会被刷新清空。
static void oledShow() {
    if (!oledOK) return;
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);

    display.setCursor(0, 0);
    display.print("Gesture 3Mode");

    const int barX = 82;
    const int barY = 11;
    const uint8_t barHeights[] = {4, 6, 8, 10};
    uint8_t filled = 0;
    if (deviceConnected && bleRssiValid) {
        if      (bleRssi >= -50) filled = 4;
        else if (bleRssi >= -65) filled = 3;
        else if (bleRssi >= -80) filled = 2;
        else                     filled = 1;
    }
    for (uint8_t i = 0; i < 4; i++) {
        const int x = barX + i * 6;
        const int y = barY - barHeights[i];
        if (i < filled) display.fillRect(x, y, 3, barHeights[i], SSD1306_WHITE);
        else            display.drawRect(x, y, 3, barHeights[i], SSD1306_WHITE);
    }
    for (uint8_t i = 0; i < MODE_N; i++) {
        const int x = 108 + i * 8;
        if (i == currentMode) display.fillCircle(x, 4, 2, SSD1306_WHITE);
        else                  display.drawCircle(x, 4, 2, SSD1306_WHITE);
    }

    display.setCursor(0, 13);
    display.print("Mode: ");
    display.println(modeNames[currentMode]);
    display.setCursor(0, 26);
    display.print("Gesture: ");
    display.println(lastGestureTag);
    display.setCursor(0, 39);
    display.print(deviceConnected ? "BLE: linked" : "BLE: searching");
    if (deviceConnected) {
        display.setCursor(76, 39);
        if (bleRssiValid) display.printf("%ddBm", bleRssi);
        else              display.print("--dBm");
    }
    display.setCursor(0, 53);
    display.printf("Total:%lu", static_cast<unsigned long>(totalGestures));
    if (lastGestureTime != 0) {
        const unsigned long secondsAgo = (millis() - lastGestureTime) / 1000;
        display.printf(" Last:%lus", secondsAgo);
    }
    display.display();
}

static void showInitStatus(const char* message, uint8_t y) {
    Serial.println(message);
    if (!oledOK) return;
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, y);
    display.println(message);
    display.display();
}

// 读取当前 BLE peer 的 RSSI；查询失败时标记为无效，避免显示过期信号值。
static void updateBleRssi() {
    if (!deviceConnected || !bleServer || bleServer->getConnectedCount() == 0) {
        bleRssiValid = false;
        return;
    }

    const NimBLEConnInfo peer = bleServer->getPeerInfo(0);
    int8_t rssi = 0;
    if (ble_gap_conn_rssi(peer.getConnHandle(), &rssi) == 0) {
        bleRssi = rssi;
        bleRssiValid = true;
    } else {
        bleRssiValid = false;
        Serial.println("[BLE] RSSI 读取失败");
    }
}

// ==================== 手势 -> 动作 ====================
// 根据当前模式执行手势映射，并统一更新统计、蜂鸣提示和 OLED 显示。
static void doGesture(Gesture g) {
    if (g == GES_NONE) return;
    const char* tag = "-";

    if (currentMode == MODE_PRESENTER) {
        switch (g) {
            case GES_LEFT:            sendKey(0x4B);           tag = "PageUp";    break;
            case GES_RIGHT:           sendKey(0x4E);           tag = "PageDown";  break;
            case GES_UP:              sendKey(0x2B);           tag = "Tab";       break;
            case GES_DOWN:            sendKey(0x02, 0x2B);     tag = "Shift+Tab"; break;
            case GES_FORWARD:         sendKey(0x28);           tag = "Enter";     break;
            case GES_BACKWARD:        tag = "No action";                         break;
            case GES_CLOCKWISE:       repeatConsumer(0x00E9, PRESENTER_VOLUME_REPEAT_COUNT); tag = "Vol+ x3"; break;
            case GES_ANTICLOCKWISE:   repeatConsumer(0x00EA, PRESENTER_VOLUME_REPEAT_COUNT); tag = "Vol- x3"; break;
            case GES_WAVE:            sendConsumer(0x00E2);    tag = "Mute";      break;
            default: return;
        }
    } else if (currentMode == MODE_BROWSER) {
        switch (g) {
            case GES_LEFT:            repeatWheel(1, BROWSER_SCROLL_REPEAT_COUNT);  tag = "ScrollUp";     break;
            case GES_RIGHT:           repeatWheel(-1, BROWSER_SCROLL_REPEAT_COUNT); tag = "ScrollDown";   break;
            case GES_UP:              sendKey(0x2B);           tag = "Tab";          break;
            case GES_DOWN:            sendKey(0x02, 0x2B);     tag = "Shift+Tab";    break;
            case GES_FORWARD:         sendKey(0x28);           tag = "Enter";        break;
            case GES_BACKWARD:        sendKey(0x01, 0x15);     tag = "Ctrl+R";       break;
            case GES_CLOCKWISE:       sendKey(0x03, 0x2E);     tag = "ZoomIn";       break;
            case GES_ANTICLOCKWISE:   sendKey(0x01, 0x2D);     tag = "ZoomOut";      break;
            case GES_WAVE:            sendKey(0x4A);           tag = "Home";         break;
            default: return;
        }
    } else if (currentMode == MODE_MEDIA) {
        switch (g) {
            case GES_UP:              repeatConsumer(0x00E9, MEDIA_VOLUME_REPEAT_COUNT); tag = "Vol+ x5";        break;
            case GES_DOWN:            repeatConsumer(0x00EA, MEDIA_VOLUME_REPEAT_COUNT); tag = "Vol- x5";        break;
            case GES_LEFT:            sendConsumer(0x00B5);      tag = "NextTrack";    break;
            case GES_RIGHT:           sendConsumer(0x00B6);      tag = "PrevTrack";    break;
            case GES_FORWARD:         sendConsumer(0x00CD);      tag = "Play/Pause";   break;
            case GES_BACKWARD:        tag = "No action";                            break;
            case GES_CLOCKWISE:       repeatKey(0, 0x4F, MEDIA_SEEK_REPEAT_COUNT);  tag = "SeekForward";  break;
            case GES_ANTICLOCKWISE:   repeatKey(0, 0x50, MEDIA_SEEK_REPEAT_COUNT);  tag = "SeekBackward"; break;
            case GES_WAVE:            sendConsumer(0x00E2);      tag = "Mute";         break;
            default: return;
        }
    }
    totalGestures++;
    lastGestureTime = millis();
    lastGestureTag = tag;
    Serial.printf("[%s] %s\n", modeNames[currentMode], tag);
    beepNonBlock(40);
    oledShow();
}

// 将传感器枚举转换为便于串口调试的名称。
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
// 启动自检用 I2C 地址扫描，仅输出发现的设备，不参与运行时判定。
static void i2cScan() {
    Serial.println("--- I2C 扫描 (SDA=21, SCL=22) ---");
    for (uint8_t a = 8; a < 120; a++) {
        Wire.beginTransmission(a);
        if (Wire.endTransmission() == 0) Serial.printf("  找到设备 0x%02X\n", a);
    }
    Serial.println("--- 扫描结束（应有 0x3C=OLED, 0x73=PAJ7620U2）---");
}

// ==================== setup ====================
// 初始化 GPIO、I2C 外设、手势传感器及 BLE HID，并输出各项自检状态。
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
        display.setCursor(0, 0);
        display.println("System Init...");
        display.display();
        Serial.println("[OLED] 0x3C 就绪");
    } else {
        Serial.println("[OLED] 0x3C 初始化失败（检查接线/上拉）");
    }
    Wire.beginTransmission(0x73);
    const bool pajAddressFound = Wire.endTransmission() == 0;
    showInitStatus(pajAddressFound ? "PAJ7620: OK" : "PAJ7620: FAIL!", 14);
    showInitStatus(oledOK ? "OLED: OK" : "OLED: FAIL!", 26);

    // PAJ7620U2（地址内部固定 0x73，库自动处理）
    bool sensorReady = paj.begin(&Wire) != 0;
    if (!sensorReady) {
        delay(100);
        sensorReady = paj.begin(&Wire) != 0;
    }
    if (sensorReady) {
        pajOK = true;
        paj.setGestureEntryTime(PAJ_GESTURE_ENTRY_TIME_MS);
        paj.setGestureExitTime(PAJ_GESTURE_EXIT_TIME_MS);
        Serial.println("[PAJ] 0x73 PAJ7620U2 就绪");
    } else {
        Serial.println("[PAJ] 0x73 初始化失败（检查 3.3V 供电/上拉/PS跳线）");
    }
    showInitStatus(sensorReady ? "Gesture: OK" : "Gesture: FAIL!", 38);

    i2cScan();
    showInitStatus("BLE: start...", 50);
    const bool bleReady = initBLE();
    showInitStatus(bleReady ? "BLE: OK" : "BLE: FAIL!", 50);
    if (!bleReady) {
        Serial.println("[BLE] 系统继续运行，但 BLE HID 不可用");
    }
    delay(500);
    beepNonBlock(90);
    oledShow();
    Serial.println("=== 系统就绪，挥手测试 ===");
}

// ==================== loop ====================
// 主循环处理蜂鸣器定时关闭、BOOT 防抖、手势读取和 BLE/OLED 状态刷新。
void loop() {
    // 蜂鸣器定时关闭
    if (beepEndMs && static_cast<int32_t>(millis() - beepEndMs) >= 0) {
        ledcWrite(BUZZER_CH, 0);
        beepEndMs = 0;
    }

    // BOOT 键切换模式（稳定状态防抖）
    static bool bootRawState = HIGH;
    static bool bootStableState = HIGH;
    static unsigned long bootStateChangedMs = 0;
    const bool bootNow = digitalRead(BOOT_PIN);
    if (bootNow != bootRawState) {
        bootRawState = bootNow;
        bootStateChangedMs = millis();
    }
    if (bootRawState != bootStableState &&
        millis() - bootStateChangedMs >= BOOT_DEBOUNCE_MS) {
        bootStableState = bootRawState;
        if (bootStableState == LOW) {
            currentMode = (Mode)((currentMode + 1) % MODE_N);
            oledShow();
            beepNonBlock(70);
            Serial.printf("切换模式 -> %s\n", modeNames[currentMode]);
        }
    }

    // 手势冷却时间避免同一动作被连续重复触发
    static unsigned long lastGestureMs = 0;
    if (pajOK && (millis() - lastGestureMs > GESTURE_COOLDOWN_MS)) {
        Gesture g = paj.readGesture();
        if (g != GES_NONE) {
            lastGestureMs = millis();
            Serial.printf("[Raw] %s\n", gestureName(g));  // 原始手势名，方便调试
            doGesture(g);          // 内部已更新 OLED 与串口日志
        }
    }

    // 连接状态变化时立即刷新；RSSI 和状态屏按配置周期更新。
    static bool previousConnectionState = false;
    if (deviceConnected != previousConnectionState) {
        previousConnectionState = deviceConnected;
        oledShow();
    }
    static unsigned long lastStatusRefresh = 0;
    if (millis() - lastStatusRefresh >= STATUS_REFRESH_INTERVAL_MS) {
        lastStatusRefresh = millis();
        updateBleRssi();
        oledShow();
    }

    delay(15);
}
