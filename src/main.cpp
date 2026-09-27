/*
 * 手势蓝牙控制器 v2.0（定稿版）
 * 硬件：ESP32 + APDS9960/PAJ9960U2 + 0.96" SSD1306 OLED + 无源蜂鸣器
 * 功能：三模式 BLE HID，短按 BOOT(GPIO0) 切换
 *       模式1 电脑键盘（PPT翻页：上/下=PageUp/Down，左=←，右=Enter）
 *       模式2 电脑鼠标（方向=光标±20，靠近=左键，顺/逆时针=滚轮）
 *       模式3 安卓媒体（方向=音量±/上一曲/下一曲，顺/逆时针=音量±）
 * 圆弧判定：仅模式2启用，连续3个相邻方向步进确认一次圆弧
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
#define OLED_ADDR            0x3C
#define OLED_W               128
#define OLED_H               64

#define APDS_ADDR            0x39

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

/* ===================== 圆弧状态机（仅模式2启用） ===================== */
enum Dir : uint8_t {
  DIR_NONE  = 0,
  DIR_UP    = 1,
  DIR_DOWN  = 2,
  DIR_LEFT  = 3,
  DIR_RIGHT = 4
};

enum ArcState : uint8_t {
  ARC_IDLE    = 0,
  ARC_ARC     = 1
};

static ArcState arcState = ARC_IDLE;
static Dir  arcLastDir = DIR_NONE;
static uint8_t arcCwCnt = 0, arcCcwCnt = 0;
static uint32_t arcStartMs = 0;

#define ARC_TIMEOUT_MS 800

static bool isCwStep(Dir prev, Dir cur) {
  return (prev == DIR_RIGHT && cur == DIR_DOWN) ||
         (prev == DIR_DOWN  && cur == DIR_LEFT) ||
         (prev == DIR_LEFT  && cur == DIR_UP)   ||
         (prev == DIR_UP    && cur == DIR_RIGHT);
}
static bool isCcwStep(Dir prev, Dir cur) {
  return (prev == DIR_RIGHT && cur == DIR_UP)   ||
         (prev == DIR_UP    && cur == DIR_LEFT) ||
         (prev == DIR_LEFT  && cur == DIR_DOWN) ||
         (prev == DIR_DOWN  && cur == DIR_RIGHT);
}

static void arcReset() {
  arcState = ARC_IDLE;
  arcLastDir = DIR_NONE;
  arcCwCnt = 0;
  arcCcwCnt = 0;
}

// 返回：0=无事件，1=顺时针完成，2=逆时针完成，其他=忽略
static int arcFeed(Dir d) {
  if (arcState == ARC_IDLE) {
    arcState = ARC_ARC;
    arcLastDir = d;
    arcStartMs = millis();
    // 第一个方向不算步进，只作为起始
    return 0;
  }

  // 超时重置
  if (millis() - arcStartMs > ARC_TIMEOUT_MS) {
    arcReset();
    return 0;
  }

  if (d == arcLastDir) return 0; // 同方向持续不计数

  if (isCwStep(arcLastDir, d)) {
    arcCcwCnt = 0;
    arcCwCnt++;
  } else if (isCcwStep(arcLastDir, d)) {
    arcCwCnt = 0;
    arcCcwCnt++;
  } else {
    // 折回方向：重置状态，以当前方向为新起点
    arcReset();
    arcState = ARC_ARC;
    arcLastDir = d;
    arcStartMs = millis();
    return 0;
  }

  arcLastDir = d;
  arcStartMs = millis();

  if (arcCwCnt >= ARC_STEPS_TO_CONFIRM) { arcReset(); return 1; }
  if (arcCcwCnt >= ARC_STEPS_TO_CONFIRM) { arcReset(); return 2; }
  return 0;
}

/* ===================== OLED ===================== */
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
static void beepStart(uint16_t freq = 2300, uint16_t dur = 55) {
  tone(BUZZER_PIN, freq, dur);
  beepEndMs = millis() + dur;
}
static void beepUpdate() {
  if (beepEndMs && millis() >= beepEndMs) {
    noTone(BUZZER_PIN);
    beepEndMs = 0;
  }
}

/* ===================== BLE 管理 ===================== */
static void ensureHID() {
  // 每次切换模式前先断开旧服务，再开启新服务，确保广播可见
  if (g_mode == MODE_PC_MOUSE) {
    if (kb.isConnected()) kb.end();
    mo.begin();
  } else {
    if (mo.isConnected()) mo.end();
    kb.begin();
  }
}

static void switchMode() {
  g_mode = (Mode)((uint8_t)g_mode + 1);
  if (g_mode > MODE_ANDROID) g_mode = MODE_PC_KEY;
  ensureHID();
  beepStart(2200, 75);
  oledShowMode();
  Serial.printf("[MODE] switched -> %d\n", (int)g_mode);
}

/* ===================== BOOT 中断 ===================== */
static volatile bool bootPressed = false;
static uint32_t lastBootMs = 0;
static void IRAM_ATTR bootISR() {
  uint32_t now = millis();
  if (now - lastBootMs > 280) {
    bootPressed = true;
    lastBootMs = now;
  }
}

/* ===================== 动作执行 ===================== */
static bool isBleReady() {
  return (g_mode == MODE_PC_MOUSE) ? mo.isConnected() : kb.isConnected();
}

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

static void execCircle(bool cw) {
  static uint32_t lastActMs = 0;
  uint32_t now = millis();
  if (now - lastActMs < GESTURE_COOLDOWN_MS) return;
  lastActMs = now;

  if (!isBleReady()) { oledShowAction("NO BLE"); beepStart(1000, 90); return; }

  Serial.printf("[GESTURE] %s\n", cw ? "CW" : "CCW");

  switch (g_mode) {
    case MODE_PC_KEY:
      // 键盘模式也用圆弧？这里预留，实际上不会走到（见 loop）
      break;
    case MODE_PC_MOUSE:
      // 顺时针→滚轮向上（负值），逆时针→滚轮向下（正值）
      if (cw) { mo.move(0, 0, -1); oledShowAction("Wheel Up"); }
      else    { mo.move(0, 0, 1);  oledShowAction("Wheel Down"); }
      break;
    case MODE_ANDROID:
      if (cw) { kb.write(KEY_MEDIA_VOLUME_UP);   oledShowAction("Vol +"); }
      else    { kb.write(KEY_MEDIA_VOLUME_DOWN); oledShowAction("Vol -"); }
      break;
  }
  beepStart(cw ? 2700 : 1900, 47);
}

/* ===================== APDS 初始化 ===================== */
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

void loop() {
  beepUpdate();  // 非阻塞蜂鸣器

  if (bootPressed) {
    bootPressed = false;
    switchMode();
  }

  uint8_t g = apds.readGesture();
  if (g != 0) {
    Dir d = DIR_NONE;
    switch (g) {
      case APDS9960_UP:    d = DIR_UP;    break;
      case APDS9960_DOWN:  d = DIR_DOWN;  break;
      case APDS9960_LEFT:  d = DIR_LEFT;  break;
      case APDS9960_RIGHT: d = DIR_RIGHT; break;
      default: break;
    }

    if (d != DIR_NONE) {
      // 模式2（鼠标）才启用圆弧判定，其余模式直接执行单方向
      if (g_mode == MODE_PC_MOUSE) {
        int evt = arcFeed(d);
        if (evt == 1) execCircle(true);
        else if (evt == 2) execCircle(false);
        // 圆弧判定过程中不产生单方向动作
      } else {
        execDirection(d);
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
