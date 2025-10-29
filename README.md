当然可以 ✅
以下是一个清晰的 **README.md**，说明如何修改 **Teensy ↔ GUI** 之间的数据协议，包含上下行（双向）的修改步骤。

---

# 🧠 Hip-Exo 通信协议修改指南

> 本文档说明：当需要新增、删除或修改从 **Teensy ↔ GUI** 传输的参数（例如 `max_torque_cfg` 、`gait_freq` 等），需要在 **Teensy 端** 与 **GUI 端** 同步修改的部分。

---

## ⚙️ 数据方向一览

| 方向                    | 说明             | 长度（字节）      | 帧头         | 示例结构                                             |
| --------------------- | -------------- | ----------- | ---------- | ------------------------------------------------ |
| **下行 (GUI → Teensy)** | GUI 发送控制参数、配置值 | `0x20 (32)` | `A5 5A 20` | 含 Rescaling、Flex/Ext Gain、Delay、max_torque_cfg 等 |
| **上行 (Teensy → GUI)** | Teensy 发送实时状态  | `0x20 (32)` | `A5 5A 20` | 含 角度、力矩、IMU 状态、max_torque_cfg 、步频等               |

---

## 🔽 一、GUI → Teensy：参数下发帧

### 1️⃣ 修改 Teensy 端（接收部分）

文件：`loop()` 或自定义的 `Receive_RS232_Data()`

```cpp
float new_param = rd_i16((uint8_t*)data_rs232_rx, OFFSET) / SCALE;
```

> ✅ 步骤：

1. 在 `data_rs232_rx[]` 中选择空余位置（或扩展总帧长）。
2. 在 Teensy 的解析函数中，用 `rd_i16()` 或 `rd_f32()` 解析该字段。
3. 若帧总长度改变（例如 20 → 32），修改

   ```cpp
   const uint8_t RS232_DATALENGTH = 32;
   ```
4. 在 nRF52 中转代码（central）也同步更新：

   ```cpp
   int rs232_datalength = 32;
   ```

   或使用“按帧头长度转发”的自动机制。

---

### 2️⃣ 修改 GUI 端（发送部分）

文件：`_tx_params()`

```python
payload = bytearray(29)
put_s16(offset, value)
payload[new_offset] = ...
header = struct.pack('<BBB', 0xA5, 0x5A, 0x20)
self.ser.write(header + payload)
```

> ✅ 步骤：

1. 新增 SpinBox 或 QDoubleSpinBox 控件绑定到参数。
2. 在 `_tx_params()` 中读取控件值，并进行缩放 (`×100` 或 `×1000`)。
3. 在 payload 中正确写入偏移位置。
4. 更新帧头的长度字节（`0x20` 或实际长度）。
5. Teensy 与 central 的 `rs232_datalength` 一致即可。

---

## 🔼 二、Teensy → GUI：状态上报帧

### 1️⃣ 修改 Teensy 端（发送部分）

文件：`Transmit_ble_Data()`

```cpp
int16_t new_field100 = (int16_t)roundf(new_field * 100.0f);
data_ble[OFFSET_LOW]  = (uint8_t)(new_field100 & 0xFF);
data_ble[OFFSET_HIGH] = (uint8_t)((new_field100 >> 8) & 0xFF);
```

> ✅ 步骤：

1. 在 payload 的 `data_ble[]` 数组中选定空余位置，写入新值。
2. 若需扩展总帧长（默认 32），修改：

   ```cpp
   const uint8_t FRAME_LEN = 34; // 例如加2字节
   data_ble[2] = FRAME_LEN;
   ```
3. 确保发送总字节数匹配 `FRAME_LEN`。

---

### 2️⃣ 修改 GUI 端（接收部分）

文件：`_read_serial()`

```python
payload = self.ser.read(29)
data = struct.unpack('<hhhhhhh', payload[:14])
new_val_i = int(payload[OFFSET] | (payload[OFFSET+1] << 8))
new_val = new_val_i / 100.0
self.lbl_new.setText(f"NewVal: {new_val:.2f}")
```

> ✅ 步骤：

1. 根据 Teensy 的 data_ble 布局，计算字段偏移。
2. 按小端序解析为 int16 或 float。
3. 更新显示（`QLabel`、`plot` 等）。
4. 如果帧长变动，也调整：

   ```python
   if self.ser.in_waiting < NEW_LEN: return
   if self.ser.read(1) != b'\x20': ...
   payload = self.ser.read(NEW_LEN-3)
   ```

---

## 🧩 三、调试要点

| 项            | 常见问题                                    | 解决方案 |
| ------------ | --------------------------------------- | ---- |
| GUI 值一直 0    | `rs232_datalength` 不匹配 → 改为 32 或按帧头长度转发 |      |
| Teensy 收不到参数 | 帧头或长度字节错误（`0x20`）                       |      |
| GUI 解析错乱     | 偏移没对齐、小端序反了                             |      |
| 速率异常         | `Serial.readBytes()` 阻塞，可改为非阻塞轮询        |      |

---

## ✅ 小结

> 修改通信内容时，务必 **四处同步：**

| 环节            | 文件                     | 内容                  |
| ------------- | ---------------------- | ------------------- |
| **GUI 发**     | `_tx_params()`         | 新参数 → payload 偏移、帧长 |
| **central 转** | `rs232_datalength`     | 与 GUI 帧长一致          |
| **Teensy 收**  | `Receive_RS232_Data()` | 按 offset 解析         |
| **Teensy 发**  | `Transmit_ble_Data()`  | 加入新字段，改 FRAME_LEN   |
| **GUI 收**     | `_read_serial()`       | 新字段 offset、显示更新     |

---

> 💡 **建议**：建立一个共享的 `packet_map.md` （字段名、偏移、缩放倍数）表格文件，方便后续添加或调试新信号。


---

# 🧩 Example: Add “LOGTAG” Command Frame (Teensy ↔ GUI)

> **Purpose:**
> Send short text labels (≤10 ASCII chars) from GUI to Teensy,
> and record them in SD log (`extra3` column).

---

## 🔹 1. GUI Side (Python)

### ▶ Add Widgets

```python
# in _build_layout()
self.edt_label = QLineEdit()
self.edt_label.setPlaceholderText("Label (max 10 chars)")
self.edt_label.setMaxLength(10)
top.addWidget(self.edt_label)

self.chk_label_persist = QCheckBox("Persist")
top.addWidget(self.chk_label_persist)

btn_send_label = QPushButton("Send Label")
btn_send_label.clicked.connect(self._send_logtag)
top.addWidget(btn_send_label)
```

### ▶ Add Function

```python
def _send_logtag(self):
    if not (self.connected and self.ser):
        return
    tag = self.edt_label.text().encode('ascii', 'ignore')[:10]
    tag_len = len(tag)
    flags = 0x01 if self.chk_label_persist.isChecked() else 0x00

    payload = bytearray(29)
    payload[0:2] = b'LG'       # magic header
    payload[2] = tag_len
    payload[13] = flags
    payload[3:3+tag_len] = tag

    header = struct.pack('<BBB', 0xA5, 0x5A, 0x20)
    self.ser.write(header + payload)
```

---

## 🔹 2. Teensy Side (C++)

### ▶ Global Variables

```cpp
static char logtag[11] = {0};
static bool logtag_valid = false;
static bool logtag_persist = false;
```

### ▶ In Receive Function (after reading payload)

```cpp
if (data_rs232_rx[0] == 'L' && data_rs232_rx[1] == 'G') {
  uint8_t n = min((int)data_rs232_rx[2], 10);
  memset(logtag, 0, sizeof(logtag));
  memcpy(logtag, &data_rs232_rx[3], n);
  logtag_valid = true;
  logtag_persist = (data_rs232_rx[13] & 0x01);

  Serial.print("LOGTAG received: ");
  Serial.println(logtag);
  return;  // skip normal param parsing
}
```

### ▶ In SD Logging Section

```cpp
if (logtag_valid) {
  logger.print(logtag);
  logger.print(',');
  if (!logtag_persist) logtag_valid = false;
} else {
  logger.print(0.0f, 4);
  logger.print(',');
}
```

---

## ✅ Result

| GUI Action                 | Teensy Behavior                     | SD Log Output        |
| -------------------------- | ----------------------------------- | -------------------- |
| Type “test1” → Send Label  | prints `LOGTAG received: test1`     | `extra3 = test1`     |
| Type “walkA” (persist off) | printed once                        | one row labeled      |
| Type “run” + persist on    | repeated on every row until changed | continuous `run` tag |

---
