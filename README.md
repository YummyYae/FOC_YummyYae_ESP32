# FOC_YummyYae_ESP32

基于 ESP32-S3 + ESP-IDF v5.5.1 的 FOC 电机控制与 POV LED 显示项目。  
当前版本集成了：

- FOC 快环（Core1，5kHz）
- 速度环控制任务（Core0，500Hz）
- TLC5947 级联驱动（4 片，共 32 颗 RGB）
- 1920Hz POV 列刷新任务（Core1）
- SoftAP + TCP 图像/控制/遥测链路（主机端）

## 1. 工程结构

```text
FOC_YummyYae_ESP32/
├─ main/
│  └─ main.c
├─ components/
│  ├─ foc/                # FOC 核心、驱动、启动校准与 NVS 标定管理
│  ├─ sensors/            # MT6701、TLC5947 以及图像映射
│  ├─ tasks/              # ap_task / control_task / foc_task / led_pov_task
│  └─ algorithm/          # PID 算法
├─ docs/
│  ├─ architecture.md
│  ├─ hardware_pins.md
│  ├─ tcp_protocol.md
│  └─ tlc5947.pdf
└─ reference/STM32_Example
```

## 2. 运行架构（当前）

- `ap_task`（Core0）  
  启动热点 `FOC_LED_AP`，开启 TCP 服务器（端口 `5005`），接收图片帧/转速指令，并以 5Hz 回传遥测。
- `control_task`（Core0，500Hz）  
  读取角度与时间差计算实时 RPM，运行速度 PID，输出 `Uq/Ud` 目标。
- `foc_driver + foc_task`（Core1，5kHz）  
  执行电机快环中断与三相 PWM 更新。
- `led_pov_task`（Core1，1920Hz）  
  按机械角映射列号，刷新 `192x32x3` POV 图像的一列。

## 3. 关键硬件映射

### 3.1 MT6701（角度传感器）

- MOSI: GPIO42
- MISO: GPIO41
- SCK: GPIO40
- CS: GPIO39

### 3.2 三相驱动（FOC PWM）

- U 相: GPIO17
- V 相: GPIO18
- W 相: GPIO8

### 3.3 TLC5947（4 片级联）

- SCLK: GPIO3
- XLAT: GPIO11
- BLANK: GPIO10
- SIN(MOSI): GPIO6

内部映射按硬件连接处理：

- 每片控制 8 颗 RGB（共 32 颗）
- CH0-CH7：蓝色（LED0->LED7）
- CH8-CH23：按红绿交替，且 LED7->LED0 反序

## 4. TCP 通信（主机 AP 端）

- Wi-Fi SSID: `FOC_LED_AP`
- Password: `12345678`
- TCP 端口: `5005`
- 帧头固定 8 字节：  
  `magic[4] + frame_id(u16, little-endian) + payload_len(u16, little-endian)`

### 4.1 下行（PC -> ESP32）

- `TCP1`：图像帧  
  `payload_len = 192 * 32 * 3 = 18432`  
  数据格式：RGB888，按行优先 `row(0..31) -> col(0..191) -> [R,G,B]`
- `RPM1`：设置目标转速  
  `payload_len = 4`，`float32`（little-endian）
- `GAM1`：设置亮度伽马  
  `payload_len = 4`，`float32`（little-endian）

### 4.2 上行（ESP32 -> PC）

- `TEL1`：遥测（5Hz）  
  `payload_len = 20`，5 个 `float32`：
  1. 实时机械转速 `mechanical_rpm`
  2. 目标机械转速 `target_mechanical_rpm`
  3. 当前 `uq`
  4. 机械角（rad）
  5. 电角度（rad）

## 5. 校准逻辑（已恢复为“魔法数有效则跳过”）

启动流程：

1. 从 NVS 读取标定结构体（`magic + version + zero + pole_pairs + direction`）
2. 若 `magic/version` 校验通过：直接加载并跳过校准
3. 若无效或不存在：执行上电校准，并写回 NVS

当前配置（`components/foc/src/foc_driver_esp32.c`）：

- `FOC_CALIBRATION_KEY = "calib_v7"`
- `FOC_CALIB_VERSION = 7`
- `FOC_FORCE_STARTUP_CALIBRATION = 0`

## 6. 编译与烧录

在工程根目录执行：

```bash
idf.py set-target esp32s3
idf.py build
idf.py -p COMx flash monitor
```

## 7. 常用调参入口

- 目标转速默认值：`components/tasks/src/control_task.c`  
  若目标为 0，会在启动时兜底到 `100 rpm`
- 速度环参数：`components/algorithm/src/pid.c` 与 `foc_speed_pid_init(...)`
- LED SPI 频率：`components/sensors/src/tlc5947.c` 的 `TLC5947_SPI_CLOCK_HZ`
- 亮度映射上限与伽马：`TLC5947_PWM_MAX` 与 `s_gamma`
- 图像上下翻转：`TLC5947_IMAGE_FLIP_VERTICAL`

## 8. 文档索引

- 架构说明：`docs/architecture.md`
- 引脚说明：`docs/hardware_pins.md`
- 协议说明：`docs/tcp_protocol.md`
- TLC5947 数据手册：`docs/tlc5947.pdf`

