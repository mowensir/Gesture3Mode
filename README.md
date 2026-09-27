# 手势蓝牙控制器（Gesture-BLE Controller）

基于 ESP32 + APDS9960/PAJ9960U2 手势传感器 + SSD1306 OLED + BLE HID 的三模式无线控制器，可用作 PPT 翻页器、电脑鼠标、安卓媒体遥控器。

## 功能一览

| 模式 | 顺/逆时针 | 上 | 下 | 左 | 右 |
| --- | --- | --- | --- | --- | --- |
| ① 电脑键盘 (PPT) | 顺=下页 / 逆=上页 | PageUp | PageDown | ← | Enter |
| ② 电脑鼠标 | 顺=滚轮上 / 逆=滚轮下 | 光标上 | 光标下 | 光标左 | 左键单击 |
| ③ 安卓媒体 | 顺=音量+ / 逆=音量- | 音量+ | 音量- | 上一曲 | 下一曲 |

短按开发板自带 **BOOT 键 (GPIO0)** 循环切换三种模式；OLED 实时显示当前模式与蓝牙连接状态；每次有效动作触发蜂鸣器短促提示音。

## 硬件清单与接线

| 模块 | 引脚 | ESP32 | 说明 |
| --- | --- | --- | --- |
| OLED SSD1306 | GND | GND | 共地 |
|  | VCC | 3V3 | 0.96" 模块通常 3.3V/5V 均可，建议 3V3 |
|  | SDA | GPIO21 | I²C 数据 |
|  | SCL | GPIO22 | I²C 时钟 |
| APDS9960 | GND | GND | 共地 |
|  | VCC | 3V3 | 务必 3.3V，不可接 5V |
|  | SDA | GPIO21 | 与 OLED 并联在同一 I²C 总线 |
|  | SCL | GPIO22 | 与 OLED 并联在同一 I²C 总线 |
|  | INT | GPIO4（可选） | 本工程采用轮询，可不接 |
| 蜂鸣器 | I/O | GPIO5 | 无源蜂鸣器，`tone()` 给频率 |
| BOOT 键 | — | GPIO0 | 开发板自带，用作模式切换键 |

> I²C 地址：OLED 默认 `0x3C`（背面焊盘若在 `0x7A` 侧则为 `0x3D`）；APDS9960 默认 `0x39`。并联在同一总线不冲突。

## 工程结构

```
GestureBleController/
├── platformio.ini        # PlatformIO 配置 + 依赖库
├── README.md
└── src/
    └── main.cpp          # 主程序（含顺/逆圆弧状态机）
```

## 编译与烧录

1. 安装 [PlatformIO](https://platformio.org/)（VS Code 插件或 CLI）。
2. 用数据线（非纯充电线）把 ESP32 插到电脑。
3. 打开本工程目录，点击底部 **→ (Upload)** 或执行 `pio run -t upload`。
4. 终端出现 `Writing at 0x... (100%)` + `Hard resetting via RTS pin...` + `[SUCCESS]` 即成功。
5. 点击底部 **插头图标** 打开串口监视器（波特率 115200），观察初始化日志。

烧录后 ESP32 会自动复位并运行程序，无需手动重启。若终端卡在 `Connecting...`，按住 BOOT 再点一次 EN，或在 Upload 刚出 `Connecting...` 时按住 BOOT 直到出现 `Writing` 再松。

## 使用流程

1. 上电后 OLED 显示当前模式，例如 `Mode1: PC KEY (PPT)`，下方显示 `BLE waiting...`。
2. 在电脑或手机的蓝牙设置里搜索 `GestureCtrl-Key`（键盘/媒体模式）或 `GestureCtrl-Mouse`（鼠标模式）并配对。
3. 配对成功后 OLED 显示 `BLE connected`。
4. 在目标设备前 5cm 左右挥动手势；手不要太远、也不要贴太近导致饱和。
5. 短按 BOOT 键切换模式，蜂鸣一声表示切换成功。

## 核心实现：顺/逆圆弧状态机

APDS9960 原生只会上报「上/下/左/右」四方向，没有直接的顺时针/逆时针事件。本工程用**方向序列步进计数**识别圆弧：

- **三段式状态**：`IDLE` → 收到第一帧进入 `PENDING`（等待确认是孤立方向还是圆弧起点）→ 第二帧与起点相邻则进入 `ARC`，否则 160ms 内无后续即提交为单方向。
- **步进判定**：右→下→左→上→右 视为顺时针步进，右→上→左→下→右 视为逆时针步进。
- **完成确认**：圆弧阶段连续累计 **3 个同方向步进**即判定为顺时针/逆时针完成，立即触发对应动作。
- **容错**：出现折回方向（如右→左）则作废当前圆弧、把折回那帧作为新起点；900ms 无进展也自动作废，避免长时间误判。

这样既能让「上/下/左/右」单方向即时响应（PPT 方向键、鼠标移动），又能让「画圈」动作映射到翻页/滚轮/音量，兼顾实时性与手势丰富度。

可调参数集中在 `main.cpp` 顶部：

| 参数 | 含义 | 建议值 |
| --- | --- | --- |
| `GESTURE_PROX_THRESH` | 手势入口接近阈值，越大越要手很近 | 30 |
| `GESTURE_GAIN` | 手势增益 | 4 (4×) |
| `GESTURE_COOLDOWN_MS` | 两次有效动作最小间隔，防连发 | 350 |
| `ARC_STEPS_TO_CONFIRM` | 连续步进数达到此值确认圆弧 | 3 |
| `SOLO_TIMEOUT_MS` | 第一帧判为单方向的最大等待 | 160 |

若识别太灵敏就增大 `GESTURE_PROX_THRESH` / 降低增益，识别不到就反向调整；若一次挥手翻多页就增大 `GESTURE_COOLDOWN_MS`。

## 调试要点

- **OLED 不亮**：先单独跑 Adafruit 的 `ssd1306_128x64_i2c` 示例确认接线和地址；背面焊盘在 `0x7A` 侧时把 `OLED_ADDR` 改为 `0x3D`。
- **APDS 初始化失败**：串口会打印 `[APDS] begin() failed @ 0x39`。用 I²C 扫描确认 `0x39` 是否存在；核对 3.3V 供电、SDA/SCL 是否接反。
- **手势不触发或误触发**：看串口日志的 `[GESTURE]` 行；挥手时保持 5cm 左右、手势轨迹尽量走圆弧而非折线。
- **BLE 连不上**：确认电脑/手机蓝牙已开启；首次配对可能需几秒。键盘和鼠标库使用 ESP32 Arduino 自带的传统 BLE 栈，工程不需要额外加入 `NimBLE-Arduino`。
- **烧录/运行时注意**：BOOT(GPIO0) 仅在**复位/上电瞬间**决定进入下载模式；正常运行时把它作为模式切换键不会影响烧录。若 GPIO0 被外部电路强拉，可能导致无法进入下载模式。

## 依赖库

- `adafruit/Adafruit SSD1306`
- `adafruit/Adafruit GFX Library`
- `adafruit/Adafruit APDS9960`
- `T-vK/ESP32-BLE-Keyboard`
- `T-vK/ESP32-BLE-Mouse`

## License

代码按 MIT 协议开放，可自由用于学习与非商业项目。各第三方库请遵守其各自的许可证。
