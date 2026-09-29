/*
 * 手势蓝牙控制器 v2.0（定稿版）
 * 硬件：ESP32 + APDS9960/PAJ9960U2 + 0.96" SSD1306 OLED + 无源蜂鸣器
 * 功能：三模式 BLE HID，短按 BOOT(GPIO0) 切换
 *       模式1 电脑键盘（PPT翻页：上/下=PageUp/Down，左=←，右=Enter）
 *       模式2 电脑鼠标（方向=光标±20，靠近=左键，顺/逆时针=滚轮）
 *       模式3 安卓媒体（方向=音量±/上一曲/下一曲，顺/逆时针=音量±）
 * 圆弧判定：三种模式均启用，连续3个相邻方向步进确认一次圆弧
 */

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_APDS9960.h>
#include <NimBleKeyboard.h>
#include <BleMouse.h>

/* ===================== 引脚与参数 ===================== */
#define OLED_SDA             21
#define OLED_SCL             22
#define OLED_RST             -1
#ifndef OLED_ADDR
#define OLED_ADDR            0x3C
#endif
#define OLED_W               128
#define OLED_H               64

#ifndef APDS_ADDR
#define APDS_ADDR            0x39
#endif

#define BUZZER_PIN           5
#define BOOT_PIN             0

#define GESTURE_PROX_THRESH  30
#define GESTURE_GAIN         2       // 0=1x, 1=2x, 2=4x, 3=8x
#define GESTURE_COOLDOWN_MS  300
#define ARC_STEPS_TO_CONFIRM 3

/* ===================== 全局对象 ===================== */
Adafruit_SSD1306 display(OLED_W, OLED_H, &Wire, OLED_RST);
Adafruit_APDS9960 apds;

BleKeyboard kb("GestureCtrl-Key", "SchoolProject", 100);
BleMouse    mo("GestureCtrl-Mouse", "SchoolProject", 100);

enum Mode : uint8_t {
  MODE_PC_KEY   = 1,
  MODE_PC_MOUSE = 2,
  MODE_ANDROID  = 3
};
volatile Mode g_mode = MODE_PC_KEY;

/* ===================== 圆弧状态机 ===================== */
enum Dir : uint8_t {
  DIR_NONE  = 0,
  DIR_UP    = 1,
  DIR_DOWN  = 2,
  DIR_LEFT  = 3,
  DIR_RIGHT = 4
};

enum ArcState : uint8_t {
  ARC_IDLE    = 0,
  ARC_PENDING = 1,
  ARC_ARC     = 2
};

static ArcState arcState = ARC_IDLE;
static Dir arcPendingDir = DIR_NONE;
static Dir arcLastDir = DIR_NONE;
static uint8_t arcCwCnt = 0;
static uint8_t arcCcwCnt = 0;
static uint32_t arcTimeMs = 0;

#define SOLO_TIMEOUT_MS 160
#define ARC_TIMEOUT_MS 900

// 判断从前一方向转到当前方向是否构成顺时针的一步。
static bool isCwStep(Dir prev, Dir cur) {
  return (prev == DIR_RIGHT && cur == DIR_DOWN) ||
         (prev == DIR_DOWN  && cur == DIR_LEFT) ||
         (prev == DIR_LEFT  && cur == DIR_UP)   ||
         (prev == DIR_UP    && cur == DIR_RIGHT);
}
// 判断从前一方向转到当前方向是否构成逆时针的一步。
static bool isCcwStep(Dir prev, Dir cur) {
  return (prev == DIR_RIGHT && cur == DIR_UP)   ||
         (prev == DIR_UP    && cur == DIR_LEFT) ||
         (prev == DIR_LEFT  && cur == DIR_DOWN) ||
         (prev == DIR_DOWN  && cur == DIR_RIGHT);
}
// 清空圆弧识别状态和计数器，供识别完成、超时或模式切换时调用。
static void arcReset() {
  arcState = ARC_IDLE;
  arcPendingDir = DIR_NONE;
  arcLastDir = DIR_NONE;
  arcCwCnt = 0;
  arcCcwCnt = 0;
}

#define EVT_NONE 0x00
#define EVT_SOLO(d) (0x40 | ((d) & 0x0F))
#define EVT_CW 0x81
#define EVT_CCW 0x82

// 将一个方向输入送入圆弧状态机；识别完成时返回单方向、顺时针或逆时针事件。
static uint8_t arcFeed(Dir d) {
  uint32_t now = millis();

  if (arcState == ARC_PENDING && now - arcTimeMs > SOLO_TIMEOUT_MS) {
    Dir solo = arcPendingDir;
    arcReset();
    return EVT_SOLO(solo);
  }
  if (arcState == ARC_ARC && now - arcTimeMs > ARC_TIMEOUT_MS) {
    arcReset();
    return 0;
  }

  if (arcState == ARC_IDLE) {
    arcState = ARC_PENDING;
    arcPendingDir = d;
    arcTimeMs = now;
    return 0;
  }

  if (arcState == ARC_PENDING) {
    if (d == arcPendingDir) {
      Dir solo = arcPendingDir;
      arcReset();
      return EVT_SOLO(solo);
    }
    if (isCwStep(arcPendingDir, d) || isCcwStep(arcPendingDir, d)) {
      arcState = ARC_ARC;
      arcLastDir = d;
      arcTimeMs = now;
      if (isCwStep(arcPendingDir, d)) arcCwCnt = 1;
      else arcCcwCnt = 1;
      return 0;
    }
    arcPendingDir = d;
    arcTimeMs = now;
    return 0;
  }

  if (d == arcLastDir) return 0;

  if (isCwStep(arcLastDir, d)) {
    arcCcwCnt = 0;
    arcCwCnt++;
  } else if (isCcwStep(arcLastDir, d)) {
    arcCwCnt = 0;
    arcCcwCnt++;
  } else {
    arcState = ARC_PENDING;
    arcPendingDir = d;
    arcLastDir = DIR_NONE;
    arcCwCnt = 0;
    arcCcwCnt = 0;
    arcTimeMs = now;
    return 0;
  }

  arcLastDir = d;
  arcTimeMs = now;

  if (arcCwCnt >= ARC_STEPS_TO_CONFIRM) { arcReset(); return EVT_CW; }
  if (arcCcwCnt >= ARC_STEPS_TO_CONFIRM) { arcReset(); return EVT_CCW; }
  return EVT_NONE;
}

// 在没有新手势输入时处理等待超时：提交孤立方向，或清理超时圆弧。
static uint8_t arcPoll() {
  if (arcState != ARC_PENDING) {
    if (arcState == ARC_ARC && millis() - arcTimeMs > ARC_TIMEOUT_MS) arcReset();
    return EVT_NONE;
  }
  if (millis() - arcTimeMs <= SOLO_TIMEOUT_MS) return EVT_NONE;
  Dir solo = arcPendingDir;
  arcReset();
  return EVT_SOLO(solo);
}

// 将 APDS9960 的手势常量转换为程序内部的方向枚举，未知值返回 DIR_NONE。
static Dir apdsToDir(uint8_t gesture) {
  switch (gesture) {
    case APDS9960_UP: return DIR_UP;
    case APDS9960_DOWN: return DIR_DOWN;
    case APDS9960_LEFT: return DIR_LEFT;
    case APDS9960_RIGHT: return DIR_RIGHT;
    default: return DIR_NONE;
  }
}

/* ===================== OLED ===================== */
// 在 OLED 上显示当前控制模式及对应 BLE 设备的连接状态。
static void oledShowMode() {
  const char* name = "";
  switch (g_mode) {
    case MODE_PC_KEY:   name = "PC KEY (PPT)";   break;
    case MODE_PC_MOUSE: name = "PC MOUSE";       break;
    case MODE_ANDROID:  name = "ANDROID MEDIA";  break;
  }
  bool connected = (g_mode == MODE_PC_MOUSE) ? mo.isConnected() : kb.isConnected();
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);  display.println(F("Gesture-BLE Ctrl"));
  display.setCursor(0, 18); display.print(F("Mode ")); display.print((int)g_mode);
  display.print(F(": ")); display.println(name);
  display.setCursor(0, 44);
  display.print(connected ? F("BLE connected") : F("BLE waiting..."));
  display.display();
}

// 在 OLED 上显示最近执行的动作或错误提示。
static void oledShowAction(const char* act) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0); display.println(F("Gesture-BLE Ctrl"));
  display.setTextSize(2);
  display.setCursor(0, 24); display.println(act);
  display.display();
}

/* ===================== 蜂鸣器（非阻塞） ===================== */
static uint32_t beepEndMs = 0;
// 启动指定频率和时长的提示音，并记录预计结束时间。
static void beepStart(uint16_t freq = 2300, uint16_t dur = 55) {
  tone(BUZZER_PIN, freq, dur);
  beepEndMs = millis() + dur;
}
// 检查提示音是否到期；到期后关闭蜂鸣器，不阻塞主循环。
static void beepUpdate() {
  if (beepEndMs && millis() >= beepEndMs) {
    noTone(BUZZER_PIN);
    beepEndMs = 0;
  }
}

/* ===================== BLE 管理 ===================== */
// 按当前模式启动所需的键盘或鼠标 HID；每种设备只启动一次。
static void ensureHID() {
  static bool keyboardStarted = false;
  static bool mouseStarted = false;
  if (g_mode == MODE_PC_MOUSE && !mouseStarted) {
    mo.begin();
    mouseStarted = true;
  } else if (g_mode != MODE_PC_MOUSE && !keyboardStarted) {
    kb.begin();
    keyboardStarted = true;
  }
}

// 循环切换控制模式，并重置手势识别、启动 HID、更新 OLED 和提示音。
static void switchMode() {
  g_mode = (Mode)((uint8_t)g_mode + 1);
  if (g_mode > MODE_ANDROID) g_mode = MODE_PC_KEY;
  arcReset();
  ensureHID();
  beepStart(2200, 75);
  oledShowMode();
  Serial.printf("[MODE] switched -> %d\n", (int)g_mode);
}

/* ===================== BOOT 中断 ===================== */
static volatile bool bootPressed = false;
static uint32_t lastBootMs = 0;
// BOOT 键下降沿中断：通过简单消抖后通知主循环执行模式切换。
static void IRAM_ATTR bootISR() {
  uint32_t now = millis();
  if (now - lastBootMs > 280) {
    bootPressed = true;
    lastBootMs = now;
  }
}

/* ===================== 动作执行 ===================== */
// 查询当前模式对应的 BLE HID 设备是否已连接。
static bool isBleReady() {
  return (g_mode == MODE_PC_MOUSE) ? mo.isConnected() : kb.isConnected();
}

// 按当前模式将单方向手势映射为键盘按键、鼠标移动或媒体控制。
static void execDirection(Dir d) {
  static uint32_t lastActMs = 0;
  uint32_t now = millis();
  if (now - lastActMs < GESTURE_COOLDOWN_MS) return;
  lastActMs = now;

  if (!isBleReady()) { oledShowAction("NO BLE"); beepStart(1000, 90); return; }

  const char* name = "";
  switch (d) {
    case DIR_UP:    name = "UP";    break;
    case DIR_DOWN:  name = "DOWN";  break;
    case DIR_LEFT:  name = "LEFT";  break;
    case DIR_RIGHT: name = "RIGHT"; break;
    default: return;
  }

  switch (g_mode) {
    case MODE_PC_KEY:
      if (d == DIR_UP)        kb.write(KEY_PAGE_UP);
      else if (d == DIR_DOWN) kb.write(KEY_PAGE_DOWN);
      else if (d == DIR_LEFT) kb.write(KEY_LEFT_ARROW);
      else if (d == DIR_RIGHT) { kb.write(KEY_RETURN); name = "ENTER"; }
      break;
    case MODE_PC_MOUSE:
      if (d == DIR_UP)        mo.move(0, -20);
      else if (d == DIR_DOWN) mo.move(0, 20);
      else if (d == DIR_LEFT) mo.move(-20, 0);
      else if (d == DIR_RIGHT) { mo.click(MOUSE_LEFT); name = "CLICK"; }
      break;
    case MODE_ANDROID:
      if (d == DIR_UP)        kb.write(KEY_MEDIA_VOLUME_UP);
      else if (d == DIR_DOWN) kb.write(KEY_MEDIA_VOLUME_DOWN);
      else if (d == DIR_LEFT) kb.write(KEY_MEDIA_PREVIOUS_TRACK);
      else if (d == DIR_RIGHT) kb.write(KEY_MEDIA_NEXT_TRACK);
      break;
  }
  oledShowAction(name);
  beepStart(2400, 42);
}

// 按当前模式执行顺时针或逆时针圆弧对应的翻页、滚轮或音量动作。
static void execCircle(bool cw) {
  static uint32_t lastActMs = 0;
  uint32_t now = millis();
  if (now - lastActMs < GESTURE_COOLDOWN_MS) return;
  lastActMs = now;

  if (!isBleReady()) { oledShowAction("NO BLE"); beepStart(1000, 90); return; }

  Serial.printf("[GESTURE] %s\n", cw ? "CW" : "CCW");

  switch (g_mode) {
    case MODE_PC_KEY:
      if (cw) { kb.write(KEY_PAGE_DOWN); oledShowAction("Next >>"); }
      else    { kb.write(KEY_PAGE_UP);   oledShowAction("<< Prev"); }
      break;
    case MODE_PC_MOUSE:
      if (cw) { mo.move(0, 0, 1);  oledShowAction("Wheel Up"); }
      else    { mo.move(0, 0, -1); oledShowAction("Wheel Down"); }
      break;
    case MODE_ANDROID:
      if (cw) { kb.write(KEY_MEDIA_VOLUME_UP);   oledShowAction("Vol +"); }
      else    { kb.write(KEY_MEDIA_VOLUME_DOWN); oledShowAction("Vol -"); }
      break;
  }
  beepStart(cw ? 2700 : 1900, 47);
}

/* ===================== APDS 初始化 ===================== */
// 初始化 APDS9960 并启用接近与手势检测；初始化失败时返回 false。
static bool initAPDS() {
  if (!apds.begin(10, APDS9960_AGAIN_4X, APDS_ADDR, &Wire)) {
    Serial.println(F("[APDS] begin() failed @ 0x39"));
    return false;
  }
  apds.enableProximity(true);
  apds.enableGesture(true);
  apds.setGestureProximityThreshold(GESTURE_PROX_THRESH);
  apds.setGestureGain(GESTURE_GAIN);  // 2 = 4x
  Serial.println(F("[APDS] init OK"));
  return true;
}

/* ===================== SETUP / LOOP ===================== */
// Arduino 启动入口：初始化串口、I2C、OLED、传感器、按键中断及 BLE HID。
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println(F("\nGesture-BLE Controller booting..."));

  Wire.begin(OLED_SDA, OLED_SCL);

  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println(F("[OLED] begin() failed! Halting."));
    while (1) delay(1000);  // 无显示则死机，便于排查
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0); display.println(F("Init..."));
  display.display();

  pinMode(BUZZER_PIN, OUTPUT);
  noTone(BUZZER_PIN);

  if (!initAPDS()) {
    oledShowAction("APDS FAIL");
    while (1) delay(1000);
  }

  pinMode(BOOT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(BOOT_PIN), bootISR, FALLING);

  ensureHID();
  oledShowMode();
  beepStart(1800, 65);
  Serial.println(F("[BLE] advertising..."));
}

// Arduino 主循环：处理提示音、按键、手势状态机及 OLED 状态刷新。
void loop() {
  beepUpdate();  // 非阻塞蜂鸣器

  if (bootPressed) {
    bootPressed = false;
    switchMode();
  }

  uint8_t timeoutEvent = arcPoll();
  if ((timeoutEvent & 0xC0) == 0x40) {
    execDirection((Dir)(timeoutEvent & 0x0F));
  }

  uint8_t g = apds.readGesture();
  if (g != 0) {
    Dir d = apdsToDir(g);

    if (d != DIR_NONE) {
      uint8_t evt = arcFeed(d);
      if (evt == EVT_CW) {
        execCircle(true);
      } else if (evt == EVT_CCW) {
        execCircle(false);
      } else if ((evt & 0xC0) == 0x40) {
        execDirection((Dir)(evt & 0x0F));
      }
    }
  }

  // 每1.5秒刷新OLED状态
  static uint32_t refreshMs = 0;
  if (millis() - refreshMs > 1500) {
    refreshMs = millis();
    oledShowMode();
  }

  delay(5);
}
