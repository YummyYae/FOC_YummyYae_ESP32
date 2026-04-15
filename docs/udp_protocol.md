# 图像传输协议说明（TCP）

## 1. 连接哪个 AP

- 主机（FOC 工程）开启 AP：
  - SSID：`FOC_LED_AP`
  - 密码：`12345678`
  - 主机 IP：`192.168.4.1`
  - TCP 端口：`5005`
- 从机（softAP 工程）以 STA 模式连接 `FOC_LED_AP`，并向 `192.168.4.1:5005` 建立 TCP 连接。

## 2. TCP 数据包（从机 -> 主机）

总长度固定 `18440` 字节，顺序如下：

1. `magic[4]`：ASCII `"TCP1"`
2. `frame_id[2]`：`uint16` 小端
3. `payload_len[2]`：`uint16` 小端，固定 `18432`
4. `payload[18432]`：RGB888 图像数据（从串口包原样透传）

## 3. RGB888 数据在 payload 内的排列

- 图像尺寸固定：`192 列 x 32 行`
- 总像素：`192 * 32 = 6144`
- 每像素 3 字节（`R,G,B`），每通道 8bit（`0..255`）
- 存储顺序：行优先（row-major）

计算公式：

- 像素索引：`pixel = row * 192 + col`
- 字节偏移：`offset = pixel * 3`
- 三通道：
  - `payload[offset + 0] = R`
  - `payload[offset + 1] = G`
  - `payload[offset + 2] = B`

范围：

- `row: 0..31`
- `col: 0..191`

## 4. 主机显示方向约定

- 驱动当前启用了垂直翻转：
  - 输入 `row=0` 映射到物理最底行
  - 输入 `row=31` 映射到物理最顶行
- 如需取消，可在 `tlc5947.c` 中修改 `TLC5947_IMAGE_FLIP_VERTICAL`。
