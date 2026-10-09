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
- OLED 显示当前模式、最近识别到的手势和 `BOOT=switch`；手势提示约 850 ms 后会被定时刷新清除。
- 无源蜂鸣器由 GPIO5 输出 4 kHz、约 50% 占空比的 PWM 音调；上电时提示约 90 ms，切换模式时提示约 60 ms。普通手势和接近度动作本身不会蜂鸣。
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

工程在 `lib/SparkFun_APDS9960/` vendoring SparkFun APDS-9960 Arduino Library 1.4.1，并保留上游 `LICENSE.md`、`library.properties` 和源码归属。传感器模块先读 ID，再调用库的 `init()` 和 `enableGestureSensor(false)`；仅接受上游支持的 `0xAB`、`0x9C`，未知 ID 不会触发寄存器写入。I²C 总线由主程序在 OLED 初始化前配置为 GPIO21/22，库适配版不会重复调用无参数 `Wire.begin()`，避免重置共享总线。

为保持主循环非阻塞，项目在 SparkFun 类中增加 `readGestureNonBlocking()`：每次调用只轮询/读取当前 FIFO，不等待手势结束；采集最多 32 组数据，在 `GVALID` 清除且 FIFO 静默达到库的 30 ms 间隔后，调用 SparkFun 原有 `processGestureData()` / `decodeGesture()` 输出结果。FIFO 溢出、超过缓存上限或 I²C 读取失败时丢弃当前手势。初始化后检查 `ENABLE=0x4D`（PON/WEN/PEN/GEN）和 `GCONF4=0x01`。

本地适配还修正了上游 I²C 读操作未检查实际返回字节数、块写操作错误写入数据，以及共享 `Wire` 总线初始化的问题。完整上游许可证和源文件保存在 `lib/SparkFun_APDS9960/`；本地接口扩展见该目录的 README。

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
手势动作通过 BLE HID 通知发送；未连接主机时不会缓存或补发动作。断开连接后自动重新广播，并由每 5 秒一次的状态检查兜底恢复意外停止的广播。连接时设备会请求 15–30 ms 的连接间隔、0 延迟和 6 秒 supervision timeout；最终参数由主机协商决定。连接后会记录实际 interval 和 bonding 状态。

键盘按键、媒体键和鼠标左键点击由同一个非阻塞状态机发送：按下报告发出后至少保持约 25 ms，再发送释放报告；期间到来的点击及鼠标移动/滚轮报告按顺序排队（最多 12 项）。队列满时会明确记录并丢弃新动作。BLE 断开时会清除正在等待的点击，避免重连后补发旧操作。主循环不再为 HID 点击/媒体键释放调用阻塞延时；成功通知不逐条写串口，避免频繁串口输出拖慢主循环，通知失败仍会记录错误。

### 4. 其它外围设备和按键

- **OLED SSD1306**：使用 I²C 地址 `0x3C`、GPIO21/22。初始化失败时只打印错误并继续运行，其它功能不依赖 OLED。屏幕显示模式、最近识别方向/圆弧方向，以及 `BOOT=switch`；屏幕约每 850 ms 刷新一次并把手势栏清为 `-`，因此方向显示时间最长不到 850 ms。
- **无源蜂鸣器**：GPIO5 使用 LEDC 输出 4 kHz、8 位分辨率、占空比 128（约 50%）的方波；通过主循环计时关闭，不使用持续阻塞延时。它只在启动和切换模式时鸣叫，不表示 BLE 动作成功。
- **BOOT 按键**：GPIO0 配置为上拉输入，按下时为低电平；只在按下沿切换模式，并用约 280 ms 的间隔过滤抖动。GPIO0 也是 ESP32 启动配置相关引脚，正常运行时短按即可，烧录时按板卡要求进入下载模式。

## 工程结构

```text
gesture/
├── platformio.ini
├── README.md
├── lib/
│   └── SparkFun_APDS9960/
│       ├── LICENSE.md
│       └── src/
│           ├── SparkFun_APDS9960.cpp
│           └── SparkFun_APDS9960.h
├── include/
│   └── README.md
└── src/
    ├── ApdsSensor.cpp
    ├── ApdsSensor.h
    └── main.cpp
```

仓库还包含 `test/README.md`；PlatformIO 的 `.pio/` 为本机构建生成目录，不属于源代码结构。

## 编译与烧录

1. 安装 [PlatformIO](https://platformio.org/)。
2. 用 USB 数据线连接 ESP32。
3. 在工程目录运行：

```bash
pio run -t upload
```

4. 观察串口输出，确认以下日志正常：
- `[Sensor] SparkFun APDS9960 initialized (ID=0xAB, ENABLE=0x4D)`（ID 也可能是 `0x9C`）
- `[BLE] HID service started and advertising as Gesture3Mode`

5. 打开串口监视器，波特率设为 `115200`。

### 烧录异常处理

如果 Upload 阶段卡在 `Connecting...`：
- 按住 BOOT 键
- 再短按 EN（复位）
- 等到 `Writing` 开始后松开 BOOT

## 使用方法

1. 上电后，蜂鸣器鸣叫约 90 ms；若 OLED 初始化成功，会显示当前模式，例如 `Mode: 1.Keyboard`。
2. 通过短按 `BOOT` 键切换模式。`BOOT` 触发间隔需要大于约 `280ms`，否则会忽略抖动。
3. 使用蓝牙设备搜索并连接 `Gesture3Mode`。
4. 站在设备前约 5cm 左右挥手，手不要太近或太远，避免接近度饱和。
5. 若在鼠标模式时接近度超过阈值，会触发左键单击；媒体模式会触发播放/暂停。
6. 模式切换时蜂鸣器鸣叫约 60 ms；识别手势时 OLED 短暂显示方向，但不会因此蜂鸣。

## 调试建议

- OLED 不亮：先单独用 SSD1306 示例程序确认 I²C 地址和接线；若模块焊盘在 `0x7A` 侧，可能需要检查实际地址。
- APDS 初始化失败：查看 `[Sensor]` 后续诊断，检查 3.3V、电源地、SDA/SCL 接线及芯片 ID。
- 手势不灵敏：观察 `[Gesture] SparkFun direction=...` 日志，调整手势距离和挥动轨迹。
- BLE 连接异常：确认目标设备蓝牙已打开，并处于可配对状态；设备名称为 `Gesture3Mode`。
- 误触发：`handleProximity()` 中的阈值为 `150`，如果环境较灵敏，可适当微调。

## 依赖项

- `adafruit/Adafruit SSD1306`
- `adafruit/Adafruit GFX Library`
- `NimBLE-Arduino`
- `ul-gh/ESP32Touch`（在 PlatformIO 配置中声明）

APDS9960 使用项目内 vendoring 的 SparkFun 1.4.1 驱动（含原始许可证及本地非阻塞扩展）；BLE HID 由 NimBLE 库直接实现。

## 许可证

本项目自有代码采用 MIT 开源协议。vendored 的 SparkFun APDS9960 驱动遵循其 `lib/SparkFun_APDS9960/LICENSE.md` 中保留的上游 Beerware 许可与归属说明；其它第三方依赖遵守各自许可证。
