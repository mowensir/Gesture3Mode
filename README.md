# Gesture 3Mode - ESP32-D0WD 三模式手势蓝牙控制器

本项目沿用 `D:\work\GestureBLEController` 的 PAJ7620U2 固件逻辑和三模式 BLE HID 功能，目标硬件改为图片中的经典 ESP32-D0WD 30 针开发板（4MB Flash、CH340、Type-C）。手势传感器、OLED、无源蜂鸣器及接线与参考项目相同；经典 ESP32 的 I²C 使用 GPIO21/22。

## 硬件与接线

| 模块 | 引脚 | ESP32-D0WD |
|---|---|---|
| OLED SSD1306 | VCC / GND | 3V3 / GND |
| OLED SSD1306 | SDA / SCL | GPIO21 / GPIO22 |
| PAJ7620U2 | VCC / GND | 3V3 / GND |
| PAJ7620U2 | SDA / SCL | GPIO21 / GPIO22（与 OLED 并联） |
| PAJ7620U2 | INT | 不接，固件轮询 |
| 无源蜂鸣器 | I/O | GPIO5 |
| 无源蜂鸣器 | VCC / GND | 3V3 / GND |
| 板载 BOOT 键 | — | GPIO0，短按切换模式 |

OLED 地址为 `0x3C`，PAJ7620U2 地址为 `0x73`。传感器必须使用 3.3V 供电；所有外设共地。如果 I²C 模块没有板载上拉，SDA 和 SCL 各接一个 4.7kΩ 电阻至 3.3V。若 PAJ7620U2 模块的 PS 跳线断开，按模块要求将 VL 接至 3.3V。

## 功能

BLE HID 以单一设备名 **Gesture 3Mode** 广播，同时提供键盘、鼠标和媒体控制。首次使用时在电脑或手机蓝牙设置中配对连接。

| 模式 | 手势 | 动作 |
|---|---|---|
| Keyboard | 上 / 下 | 音量 + / - |
| Keyboard | 左 / 右 | PageUp / PageDown |
| Keyboard | 前进 / 挥手 | 播放/暂停 / 静音 |
| Mouse | 上 / 下 / 左 / 右 | 鼠标移动，每次 20 |
| Mouse | 顺时针 / 逆时针 | 滚轮上 / 下 |
| Mouse | 前进 / 后退 | 鼠标左键 / 右键单击 |
| Media | 上 / 下 | 音量 + / - |
| Media | 左 / 右 | 上一曲 / 下一曲 |
| Media | 前进 / 挥手 | 播放/暂停 / 静音 |

短按板载 BOOT 键，按 `Keyboard → Mouse → Media` 循环切换。上电及切换模式时蜂鸣器短响；OLED 显示模式和最近识别的手势。OLED 或蜂鸣器未连接时，其它功能仍可使用。手势由 PAJ7620U2 直接识别，不使用 APDS9960 或软件圆弧拼接。

## 编译与烧录

使用 VS Code PlatformIO 打开当前项目，构建环境为 `esp32dev`。配置目标为经典 ESP32、4MB Flash 和 Arduino 框架，PAJ7620U2、OLED 与 NimBLE 依赖由 `platformio.ini` 管理。

```text
pio run
pio run -t upload
```

串口监视器波特率为 115200。启动日志中的 I²C 扫描应找到 `0x3C` 和 `0x73`。若上传停在 `Connecting...`，按住 BOOT、短按 EN，开始写入后松开 BOOT。

## 自检与排查

- I²C 扫描只找到 `0x3C`：检查 PAJ7620U2 的 3.3V、电源共地、SDA/SCL 接线及上拉电阻。
- 找不到 OLED：确认 OLED 的 `0x3C` 地址和 GPIO21/22 接线。
- 串口显示 PAJ7620U2 初始化失败：确认传感器型号为 PAJ7620U2（地址 `0x73`），并检查供电和模块 PS/VL 设置。
- 蓝牙搜索设备名：`Gesture 3Mode`。连接后可以用对应模式测试 HID 动作。

> ESP32-D0WD 是经典 ESP32，不是 ESP32-S3：请将 OLED 与 PAJ7620U2 接到 GPIO21/22，而不是参考 S3 项目的 GPIO8/9。GPIO0 是启动配置引脚，正常使用中短按切换模式即可。
