# Gesture 3Mode - ESP32 三模式手势蓝牙控制器

这是一个基于 ESP32 + SSD1306 OLED + APDS9960 手势传感器 + BLE HID 的三模式无线控制器。当前代码实现了一个单一的 HID 设备名为 `Gesture3Mode`，通过 BOOT 键切换模式，并可用于电脑键盘控制、电脑鼠标控制和安卓/桌面媒体控制。

## 功能概览

| 模式 | 手势 | 动作 |
| --- | --- | --- |
| 1.Keyboard | 上 | PageUp |
| 1.Keyboard | 下 | PageDown |
| 1.Keyboard | 左 | Left Arrow |
| 1.Keyboard | 右 | Enter |
| 2.Mouse | 顺时针圆弧 (CW) | 滚轮向上 |
| 2.Mouse | 逆时针圆弧 (CCW) | 滚轮向下 |
| 2.Mouse | 上 | 鼠标向上移动 |
| 2.Mouse | 下 | 鼠标向下移动 |
| 2.Mouse | 左 | 鼠标向左移动 |
| 2.Mouse | 右 | 鼠标向右移动 |
| 2.Mouse | 接近度触发 | 左键单击 |
| 3.Media | 上 | 音量+ |
| 3.Media | 下 | 音量- |
| 3.Media | 左 | 上一曲 |
| 3.Media | 右 | 下一曲 |
| 3.Media | 接近度触发 | 播放/暂停 |

说明：
- 短按开发板自带 `BOOT` 键（GPIO0）循环切换 `1.Keyboard -> 2.Mouse -> 3.Media -> 1.Keyboard`。
- OLED 会显示当前模式、最近手势和 `BOOT=switch`。
- 每次有效动作都会发出一次短促蜂鸣提示。
- BLE 设备名称固定为 `Gesture3Mode`，通过 HID 报告提供键盘、鼠标和媒体功能。

## 硬件连接

| 模块 | 引脚 | ESP32 | 说明 |
| --- | --- | --- | --- |
| OLED SSD1306 | GND | GND | 共地 |
|  | VCC | 3V3 | 推荐 3.3V |
|  | SDA | GPIO21 | I²C 数据线 |
|  | SCL | GPIO22 | I²C 时钟线 |
| APDS9960 / 兼容模块 | GND | GND | 共地 |
|  | VCC | 3V3 | 必须 3.3V，不可接 5V |
|  | SDA | GPIO21 | 与 OLED 共用 I²C |
|  | SCL | GPIO22 | 与 OLED 共用 I²C |
|  | INT | GPIO4 | 可选，当前代码采用轮询 |
| 蜂鸣器 | I/O | GPIO5 | 连接无源蜂鸣器 |
| BOOT 键 | — | GPIO0 | 开发板自带，作为模式切换按钮 |

> I²C 地址：OLED 默认 `0x3C`，APDS9960 默认 `0x39`。代码中均以这两个地址进行初始化和读写。

## 代码中的实际行为

### 1. APDS9960 初始化

工程会先读取传感器 `0x92` 处的 chip ID：
- 若为标准 APDS9960，ID 为 `0xAB`，调用 `apds.begin()` 初始化。
- 若为非标准/兼容模块，代码会绕过 `Adafruit_APDS9960.begin()`，直接按 APDS9960 寄存器映射执行初始化。

初始化过程包括：
- 读取 `GSTATUS (0xAF)` / `GFLVL (0xAE)` / 手势 FIFO (`0xFC`–`0xFF`)
- 通过寄存器写入校验，确认接近度和手势引擎已启用
- 检查 `ENABLE` 寄存器是否为 `0x45`，即 `proximity + gesture enabled`

### 2. 圆弧识别

代码使用一个 `arcQueue` 和 `checkArc()` 来识别顺时针/逆时针手势：
- 方向按 `UP / RIGHT / DOWN / LEFT` 的相邻关系构成顺时针序列
- 方向按 `UP / LEFT / DOWN / RIGHT` 的相邻关系构成逆时针序列
- 只要连续方向符合一条圆周序列，便判定为 `G_CW` 或 `G_CCW`
- 如果没有形成完整圆弧，则按单向手势处理

这意味着：
- 单方向手势用于即时控制（如按键、鼠标移动、媒体按键）
- 圆弧手势用于滚轮动作（鼠标模式）

### 3. BLE HID

当前代码使用 `NimBLE-Arduino` + `NimBLEHIDDevice`，在设备启动时：
- 创建 GATT server
- 注册键盘、鼠标、媒体 3 个 HID report
- 以 `Gesture3Mode` 名称广播并提供 HID service

代码中连接实际使用的是一个设备名，而不是按模式拆分成多个蓝牙设备。

## 工程结构

```text
GestureBleController/
├── platformio.ini
├── README.md
├── lib/
│   └── NimBLEMouse/
└── src/
    └── main.cpp
```

## 编译与烧录

1. 安装 [PlatformIO](https://platformio.org/)。
2. 用 USB 数据线连接 ESP32。
3. 在工程目录运行：

```bash
pio run -t upload
```

4. 观察串口输出，确认以下日志正常：
- `[Sensor] I2C device at 0x39 responded; chip ID=0xXX`
- `[Sensor] Standard APDS9960 initialization succeeded`
- `[BLE] HID service started and advertising as Gesture3Mode`

5. 打开串口监视器，波特率设为 `115200`。

### 烧录异常处理

如果 Upload 阶段卡在 `Connecting...`：
- 按住 BOOT 键
- 再短按 EN（复位）
- 等到 `Writing` 开始后松开 BOOT

## 使用方法

1. 上电后，OLED 会显示当前模式，例如 `Mode: 1.Keyboard`。
2. 通过短按 `BOOT` 键切换模式。`BOOT` 触发间隔需要大于约 `280ms`，否则会忽略抖动。
3. 使用蓝牙设备搜索并连接 `Gesture3Mode`。
4. 站在设备前约 5cm 左右挥手，手不要太近或太远，避免接近度饱和。
5. 若在鼠标模式时接近度超过阈值，会触发左键单击；媒体模式会触发播放/暂停。

## 调试建议

- OLED 不亮：先单独用 SSD1306 示例程序确认 I²C 地址和接线；若模块焊盘在 `0x7A` 侧，可能需要检查实际地址。
- APDS 初始化失败：查看串口输出 `[Sensor] No I2C response at 0x39`，检查 3.3V、电源地、SDA/SCL 接线，以及模块是否兼容 APDS9960 寄存器布局。
- 手势不灵敏：观察 `[Gesture] Sensor raw value=...` 日志，调整手势距离和挥动轨迹。
- BLE 连接异常：确认目标设备蓝牙已打开，并处于可配对状态；设备名称为 `Gesture3Mode`。
- 误触发：`handleProximity()` 中的阈值为 `150`，如果环境较灵敏，可适当微调。

## 依赖项

- `adafruit/Adafruit SSD1306`
- `adafruit/Adafruit GFX Library`
- `adafruit/Adafruit APDS9960`
- `h2zero/NimBLE-Arduino`
- `lib/NimBLEMouse`（本项目内自定义/修补版本）

## 许可证

本项目代码采用 MIT 开源协议，供学习和非商业用途参考。第三方库请遵守其各自许可证条款。
