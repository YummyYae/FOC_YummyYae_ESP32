# 图像与控制传输协议说明（TCP）

## 1. 连接哪个 AP

- 主机（FOC 工程）开启 AP：
  - SSID：`FOC_LED_AP`
  - 密码：`12345678`
  - 主机 IP：`192.168.4.1`
  - TCP 端口：`5005`
- 从机（softAP 工程）以 STA 模式连接 `FOC_LED_AP`，并向 `192.168.4.1:5005` 建立 TCP 连接。

## 2. 数据包格式（基本结构）

每个数据包包含 8 字节的 Header 后接可选长度的 Payload：

1. `magic[4]`：4 字节 ASCII 标识符，决定了包的类型。
2. `frame_id/seq[2]`：2 字节 `uint16` 小端，表示帧序号/包序号。
3. `payload_len[2]`：2 字节 `uint16` 小端，表示后续 Payload 的字节长度。

## 3. 各类型数据包详细说明

### 3.1 图像数据包（从机 -> 主机）
- **Magic**: `"TCP1"`
- **Payload 长度**: `18432` 字节
- **Payload 格式限制**: RGB888 图像数据（从串口包原样透传）
  - 图像尺寸固定：`192 列 x 32 行`
  - 总像素：`192 * 32 = 6144`
  - 每像素 3 字节（`R,G,B`），每通道 8bit（`0..255`）
  - 存储顺序：行优先（row-major）
  - 像素索引：`pixel = row * 192 + col`
  - 字节偏移：`offset = pixel * 3`

### 3.2 目标转速设置包（从机 -> 主机）
- **Magic**: `"RPM1"`
- **Payload 长度**: `4` 字节
- **Payload 格式**: `float`（通过 4 字节直接强转）小端浮点数，表示新的目标机械 RPM 速度。

### 3.3 Gamma 值设置包（从机 -> 主机）
- **Magic**: `"GAM1"`
- **Payload 长度**: `4` 字节
- **Payload 格式**: `float` 小端浮点数，表示新的 Gamma 校正值（对比度表）。主机收到后会自动更新对比度计算数据。

### 3.4 遥测状态数据包（主机 -> 从机）
- **Magic**: `"TEL1"`
- **发送频率**: 自动触发，周期约为 `200ms`（由主机定时发送）
- **Payload 长度**: `20` 字节（由 5 个 `float` 组成）
  - `payload[0..3]`: `mechanical_rpm`
  - `payload[4..7]`: `target_mechanical_rpm`
  - `payload[8..11]`: `uq`
  - `payload[12..15]`: `shaft_angle_rad` (轴角度，0 ~ 2π)
  - `payload[16..19]`: `electrical_angle_rad` (电角度，0 ~ 2π)

## 4. 主机显示方向约定

- 驱动当前启用了垂直翻转：
  - 输入 `row=0` 映射到物理最底行
  - 输入 `row=31` 映射到物理最顶行
- 如需取消，可在 `tlc5947.c` 中修改 `TLC5947_IMAGE_FLIP_VERTICAL`。
