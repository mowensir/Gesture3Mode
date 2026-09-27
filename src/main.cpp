/*
 * 手势蓝牙控制器（学校作品 / 毕设演示）
 * ---------------------------------------------------------------
 * 硬件：ESP32 + APDS9960/PAJ9960U2 手势传感器 + 0.96" SSD1306 OLED
 *       (128x64, I2C) + 无源蜂鸣器
 * 功能：BLE HID 三模式，短按 BOOT(GPIO0) 切换
 *         MODE 1 电脑键盘  -> PPT 翻页 / 方向键
 *         MODE 2 电脑鼠标  -> 光标移动 / 左键 / 滚轮
 *         MODE 3 安卓媒体  -> 音量 / 曲目 / 播放暂停
 * 顺/逆时针：APDS9960 原生只给上下左右四方向，用“连续相邻方向步进”计数
 *       识别。连续 3 个顺时针步进=顺时针完成；3 个逆时针步进=逆时针完成。
 *       顺时针 -> 下页/滚轮上/音量+；逆时针 -> 上页/滚轮下/音量-。
 * 依赖：
 *   adafruit/Adafruit SSD1306
 *   adafruit/Adafruit GFX Library
 *   adafruit/Adafruit APDS9960
 *   h2zero/NimBLE-Arduino
 *   webmonkey/ESP32-NIMBLE-Keyboard (GitHub)
 *   wakwak-koba/ESP32-NIMBLE-Mouse    (GitHub)
 */

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_APDS9960.h>
#include <NimBleKeyboard.h>
#include <BleMouse.h>

/* ========================= 引脚与参数 ========================= */
#define OLED_SDA             21
#define OLED_SCL             22
#define OLED_RST             -1
#define OLED_ADDR            0x3C    // 背面 0x78 焊盘 -> 7 位 0x3C
#define OLED_W               128
#define OLED_H               64

#define APDS_ADDR            0x39

#define BUZZER_PIN           5
#define BOOT_PIN             0       // ESP32 自带 BOOT = GPIO0

#define GESTURE_PROX_THRESH  30
#define GESTURE_GAIN         4       // APDS9960_GGAIN_4
#define GESTURE_LED_DRIVE    3       // APDS9960_LEDDRIVE_100MA
#define GESTURE_COOLDOWN_MS  350     // 两次有效手势最小间隔
#define ARC_STEPS_TO_CONFIRM 3       // 连续 3 个同方向步进确认圆弧

/* ========================= 全局对象 ========================= */
Adafruit_SSD1306 display(OLED_W, OLED_H, &Wire, OLED_RST);
Adafruit_APDS9960  apds;

// 直接实例化对象（webmonkey 分支不支持指针）
BleKeyboard kb("GestureCtrl-Key", "SchoolProject", 100);
BleMouse    mo("GestureCtrl-Mouse", "SchoolProject", 100);

enum Mode : uint8_t {
  MODE_PC_KEY   = 1,
  MODE_PC_MOUSE = 2,
  MODE_ANDROID  = 3
};
volatile Mode g_mode = MODE_PC_KEY;

/* ========================= 顺/逆圆弧状态机 =========================
 * 思路：APDS9960 每次只上报一个方向（上/下/左/右）。单方向手势希望“即时响应”，
 * 顺/逆时针希望“走完一个圆弧再响应”。用三段式状态解决两者的矛盾：
 *   IDLE     ：空闲
 *   PENDING  ：刚收到第一帧，等待确认是孤立方向还是圆弧起点
 *   ARC      ：已进入圆弧判定，累计顺时针/逆时针步进
 * 判定：
 *   - PENDING 状态在 SOLO_TIMEOUT_MS 内无新帧 -> 作为单方向提交
 *   - PENDING 收到第二帧且与第一帧相邻 -> 转 ARC
 *   - ARC 连续累计满 ARC_STEPS_TO_CONFIRM 个同方向步进 -> 圆弧完成
 *   - 超时 / 出现折回方向 -> 重置（折回那帧作为新起点）
 */
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
static Dir  arcPendingDir = DIR_NONE;   // PENDING 阶段的第一帧
static Dir  arcLastDir    = DIR_NONE;   // ARC 阶段的上一帧
static uint8_t arcCwCnt   = 0;         // 连续顺时针步进数
static uint8_t arcCcwCnt  = 0;         // 连续逆时针步进数
static uint32_t arcTimeMs = 0;         // 状态进入时间

#define SOLO_TIMEOUT_MS  160            // 第一帧判为单方向的最大等待
#define ARC_TIMEOUT_MS   900            // 圆弧判定的总超时

// arcFeed 返回值：高 2 位=事件类型，低 4 位=方向（仅 ARC_SOLO 有效）
//   00 0000 = 无事件
//   01 dddd = 单方向 d
//   10 0001 = 顺时针完成
//   10 0010 = 逆时针完成
#define EVT_NONE    0x00
#define EVT_SOLO(d) (0x40 | ((d) & 0x0F))
#define EVT_CW      0x81
#define EVT_CCW     0x82

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
  arcState   = ARC_IDLE;
  arcPendingDir = DIR_NONE;
  arcLastDir = DIR_NONE;
  arcCwCnt   = 0;
  arcCcwCnt  = 0;
}

static uint8_t arcFeed(Dir d) {
  uint32_t now = millis();

  // 超时处理：PENDING 超时->提交单方向；ARC 超时->作废
  if (arcState == ARC_PENDING && now - arcTimeMs > SOLO_TIMEOUT_MS) {
    Dir solo = arcPendingDir;
    arcReset();
    return EVT_SOLO(solo);
  }
  if (arcState == ARC_ARC && now - arcTimeMs > ARC_TIMEOUT_MS) {
    arcReset();
    return EVT_NONE;
  }

  if (arcState == ARC_IDLE) {
    // 第一帧：进入 PENDING 等待确认
    arcState = ARC_PENDING;
    arcPendingDir = d;
    arcTimeMs = now;
    return EVT_NONE;
  }

  if (arcState == ARC_PENDING) {
    // 第二帧：相同=持续，作为单方向处理；相邻=进入圆弧；折回=替换起点
    if (d == arcPendingDir) {
      Dir solo = arcPendingDir;
      arcReset();
      return EVT_SOLO(solo);
    }
    if (isCwStep(arcPendingDir, d) || isCcwStep(arcPendingDir, d)) {
      // 进入圆弧：第一帧也算一个步进
      arcState = ARC_ARC;
      arcLastDir = d;
      arcTimeMs = now;
      if (isCwStep(arcPendingDir, d))  arcCwCnt  = 1;
      else                            arcCcwCnt = 1;
      if (arcCwCnt  >= ARC_STEPS_TO_CONFIRM) { arcReset(); return EVT_CW;  }
      if (arcCcwCnt >= ARC_STEPS_TO_CONFIRM) { arcReset(); return EVT_CCW; }
      return EVT_NONE;
    }
    // 折回方向：替换起点，继续 PENDING
    arcPendingDir = d;
    arcTimeMs = now;
    return EVT_NONE;
  }

  // ARC 阶段
  if (d == arcLastDir) return EVT_NONE;   // 持续手势不计数

  if (isCwStep(arcLastDir, d)) {
    arcCcwCnt = 0;
    if (arcCwCnt < 255) arcCwCnt++;
  } else if (isCcwStep(arcLastDir, d)) {
    arcCwCnt = 0;
    if (arcCcwCnt < 255) arcCcwCnt++;
  } else {
    // 非相邻折回：作废，本次作为新起点
    arcState = ARC_PENDING;
    arcPendingDir = d;
    arcLastDir = DIR_NONE;
    arcCwCnt = 0;
    arcCcwCnt = 0;
    arcTimeMs = now;
    return EVT_NONE;
  }

  arcLastDir = d;
  arcTimeMs = now;
  if (arcCwCnt  >= ARC_STEPS_TO_CONFIRM) { arcReset(); return EVT_CW;  }
  if (arcCcwCnt >= ARC_STEPS_TO_CONFIRM) { arcReset(); return EVT_CCW; }
  return EVT_NONE;
}

static uint8_t arcPoll() {
  uint32_t now = millis();
  if (arcState == ARC_PENDING && now - arcTimeMs > SOLO_TIMEOUT_MS) {
    Dir solo = arcPendingDir;
    arcReset();
    return EVT_SOLO(solo);
  }
  if (arcState == ARC_ARC && now - arcTimeMs > ARC_TIMEOUT_MS) {
    arcReset();
  }
  return EVT_NONE;
}

/* 从 APDS 手势码转 Dir */
static Dir apdsToDir(uint8_t g) {
  switch (g) {
    case APDS9960_UP:    return DIR_UP;
    case APDS9960_DOWN:  return DIR_DOWN;
    case APDS9960_LEFT:  return DIR_LEFT;
    case APDS9960_RIGHT: return DIR_RIGHT;
    default:             return DIR_NONE;
  }
}

/* ========================= OLED ========================= */
static void oledShowMode() {
  const char* name = "";
  switch (g_mode) {
    case MODE_PC_KEY:   name = "PC KEY (PPT)";   break;
    case MODE_PC_MOUSE: name = "PC MOUSE";       break;
    case MODE_ANDROID:  name = "ANDROID MEDIA";  break;
  }
  bool connected = g_mode == MODE_PC_MOUSE
                     ? mo.isConnected()
                     : kb.isConnected();
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);  display.println(F("Gesture-BLE Ctrl"));
  display.setCursor(0, 18); display.print(F("Mode")); display.print((int)g_mode);
  display.print(F(": ")); display.println(name);
  display.setCursor(0, 46);
  display.print(connected ? F("BLE connected") : F("BLE waiting..."));
  display.display();
}

static void oledShowAction(const char* act) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0); display.println(F("Gesture-BLE Ctrl"));
  display.setTextSize(2);
  display.setCursor(0, 22); display.println(act);
  display.display();
}

/* ========================= 蜂鸣器 ========================= */
static void beep(uint16_t freq = 2000, uint16_t dur = 60) {
  tone(BUZZER_PIN, freq, dur);
  delay(dur);
  noTone(BUZZER_PIN);
}

/* ========================= BLE 模式 ========================= */
static void ensureHID() {
  static bool keyboardStarted = false;
  static bool mouseStarted = false;
  if ((g_mode == MODE_PC_KEY || g_mode == MODE_ANDROID) && !keyboardStarted) {
    kb.begin();
    keyboardStarted = true;
  } else if (g_mode == MODE_PC_MOUSE && !mouseStarted) {
    mo.begin();
    mouseStarted = true;
  }
}

static void switchMode() {
  g_mode = (Mode)((uint8_t)g_mode + 1);
  if (g_mode > MODE_ANDROID) g_mode = MODE_PC_KEY;
  ensureHID();
  beep(2200, 70);
  oledShowMode();
  Serial.printf("[MODE] switched -> %d\n", (int)g_mode);
}

/* ========================= BOOT 中断 ========================= */
static volatile bool bootPressed = false;
static uint32_t lastBootMs = 0;
static void IRAM_ATTR bootISR() {
  uint32_t now = millis();
  if (now - lastBootMs > 250) {
    bootPressed = true;
    lastBootMs = now;
  }
}

/* ========================= 动作 ========================= */
static bool isBleReady() {
  if (g_mode == MODE_PC_MOUSE) return mo.isConnected();
  return kb.isConnected();
}

static void execGesture(Dir d) {
  static uint32_t lastActMs = 0;
  if (millis() - lastActMs < GESTURE_COOLDOWN_MS) return;
  lastActMs = millis();

  const char* name = "";
  switch (d) {
    case DIR_UP:    name = "UP";    break;
    case DIR_DOWN:  name = "DOWN";  break;
    case DIR_LEFT:  name = "LEFT";  break;
    case DIR_RIGHT: name = "RIGHT"; break;
    default: return;
  }

  if (!isBleReady()) { oledShowAction("BLE?"); beep(1000, 80); return; }

  if (g_mode == MODE_PC_KEY) {
    if (d == DIR_UP)        kb.write(KEY_PAGE_UP);
    else if (d == DIR_DOWN) kb.write(KEY_PAGE_DOWN);
    else if (d == DIR_LEFT) kb.write(KEY_LEFT_ARROW);
    else if (d == DIR_RIGHT){ kb.write(KEY_RETURN); name = "ENTER"; }
    oledShowAction(name);
  } else if (g_mode == MODE_PC_MOUSE) {
    if (d == DIR_UP)        mo.move(0, -20);
    else if (d == DIR_DOWN) mo.move(0, 20);
    else if (d == DIR_LEFT) mo.move(-20, 0);
    else if (d == DIR_RIGHT){ mo.click(MOUSE_LEFT); name = "CLICK"; }
    oledShowAction(name);
  } else if (g_mode == MODE_ANDROID) {
    if (d == DIR_UP)        kb.write(KEY_MEDIA_VOLUME_UP);
    else if (d == DIR_DOWN) kb.write(KEY_MEDIA_VOLUME_DOWN);
    else if (d == DIR_LEFT) kb.write(KEY_MEDIA_PREVIOUS_TRACK);
    else if (d == DIR_RIGHT)kb.write(KEY_MEDIA_NEXT_TRACK);
    oledShowAction(name);
  }
  beep(2400, 45);
}

static void execCircle(bool cw) {
  static uint32_t lastActMs = 0;
  if (millis() - lastActMs < GESTURE_COOLDOWN_MS) return;
  lastActMs = millis();

  if (!isBleReady()) { oledShowAction("BLE?"); beep(1000, 80); return; }

  Serial.printf("[GESTURE] %s\n", cw ? "CW" : "CCW");

  if (g_mode == MODE_PC_KEY) {
    if (cw) { kb.write(KEY_PAGE_DOWN); oledShowAction("Next >>"); }
    else    { kb.write(KEY_PAGE_UP);   oledShowAction("<< Prev"); }
  } else if (g_mode == MODE_PC_MOUSE) {
    if (cw) { mo.move(0, 0, 1);  oledShowAction("Wheel+"); }
    else    { mo.move(0, 0, -1); oledShowAction("Wheel-"); }
  } else if (g_mode == MODE_ANDROID) {
    if (cw) { kb.write(KEY_MEDIA_VOLUME_UP);   oledShowAction("Vol +"); }
    else    { kb.write(KEY_MEDIA_VOLUME_DOWN); oledShowAction("Vol -"); }
  }
  beep(cw ? 2600 : 1800, 50);
}

/* ========================= APDS 初始化 ========================= */
static bool initAPDS() {
  // 修正参数顺序：增益常量、地址、Wire
  if (!apds.begin(10, APDS9960_AGAIN_4X, APDS_ADDR, &Wire)) {
    Serial.println(F("[APDS] begin() failed @ 0x39"));
    return false;
  }
  apds.enableProximity(true);
  apds.enableGesture(true);
  apds.setGestureProximityThreshold(GESTURE_PROX_THRESH);
  apds.setGestureGain(GESTURE_GAIN);
  Serial.println(F("[APDS] init OK"));
  return true;
}

/* ========================= SETUP / LOOP ========================= */
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println(F("\nGesture-BLE Controller booting..."));

  Wire.begin(OLED_SDA, OLED_SCL);

  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println(F("[OLED] begin() failed"));
  } else {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0); display.println(F("Init..."));
    display.display();
  }

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
  beep(1800, 60);
  Serial.println(F("[BLE] advertising..."));
}

void loop() {
  if (bootPressed) {
    bootPressed = false;
    switchMode();
  }

  uint8_t timeoutEvent = arcPoll();
  if ((timeoutEvent & 0xC0) == 0x40) {
    execGesture((Dir)(timeoutEvent & 0x0F));
  }

  uint8_t g = apds.readGesture();
  if (g != 0) {
    Dir d = apdsToDir(g);
    if (d != DIR_NONE) {
      uint8_t evt = arcFeed(d);
      uint8_t type = evt & 0xC0;
      if (evt == EVT_CW) {
        execCircle(true);
      } else if (evt == EVT_CCW) {
        execCircle(false);
      } else if (type == 0x40) {          // 单方向
        execGesture((Dir)(evt & 0x0F));
      }
    }
  }

  static uint32_t refreshMs = 0;
  if (millis() - refreshMs > 1500) {
    refreshMs = millis();
    oledShowMode();
  }

  delay(10);
}
