import serial
from serial.tools import list_ports
import struct
import time
from math import *
import csv
from PyQt5 import QtWidgets, QtCore, QtGui
import pyqtgraph as pg
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5.QtCore import *
import sys
import os
from datetime import datetime
"""
Hip‑Exo GUI – **refactored for clarity**
--------------------------------------------------
Key runtime‑tunable parameters (sent over BLE → Teensy)
------------------------------------------------------
| Symbol | Spin‑box | What it means | How it enters the equation |
|--------|----------|--------------|----------------------------|
| **k_gain** | k‑scale | Overall gain. <br>Final motor torque τ is proportional to it. | τ = k_gain · (k_flex/k_ext) · sin Δ |
| **k_flex** | Flex‑assist | Extra scaling while **swing‑leg flexing** (hip flexion). |
| **k_ext** | Ext‑assist | Extra scaling while **stance‑leg extending** (hip extension). |
| **delay** | Phase delay | Index 0‑99 → ~0‑990 ms phase shift applied to Δ. |

Users can hover each spin‑box to see the same description.
Realtime values (angles & torques) are now also shown numerically next to the plots.
"""
from serial.tools import list_ports

def find_available_ports():
    connected_ports = []
    available_ports = list(list_ports.comports())

    for port, desc, hwid in available_ports:
        try:
            ser = serial.Serial(port)
            if ser.readable():
                connected_ports.append(port)
            ser.close()
        except serial.SerialException:
            pass

    return connected_ports

# -------------  helpers unchanged (find_available_ports, saturation …) -------------

#  (due to space, helpers are identical to the previous version)
#  …  << KEEP THE ORIGINAL HELPER FUNCTIONS HERE >>


class MainWindow(QWidget):
    """Main application window – cleaned & documented."""

    def __init__(self):
        super().__init__()
        self.setWindowTitle("NYU Biro Lab Hip‑Exo Energy gate Controller v2.0")

        # === Serial setup vars ===
        self.ser = None
        self.connected = False

        # === Realtime data buffers ===
        self.win_size = 150
        self.t_buffer = [0]*self.win_size
        self.L_IMU_buf = self.t_buffer.copy()
        self.R_IMU_buf = self.t_buffer.copy()
        self.L_tau_buf = self.t_buffer.copy()
        self.R_tau_buf = self.t_buffer.copy()
        self.L_tau_d_buf = self.t_buffer.copy()
        self.R_tau_d_buf = self.t_buffer.copy()

        # ------------  build UI  ------------
        self._build_layout()
        self._build_plots()
        self._build_value_displays()
        log_dir = "data"
        os.makedirs(log_dir, exist_ok=True)
        filename = datetime.now().strftime("%Y%m%d_%H%M%S") + ".csv"
        self.csv_path = os.path.join(log_dir, filename)

        # === 创建 CSV 文件并写入表头 ===
        with open(self.csv_path, "w", newline="") as f:
            writer = csv.writer(f)
            # 表头：时间戳 + R数据 + L数据
            writer.writerow([
                "timestamp",
                "R_angle(θ)", "R_Torque Cmd+"
                "", "R_Torque Est",
                "L_angle(θ)", "L_Torque Cmd", "L_Torque Est",
                "Lθ Display", "Rθ Display", "Lτ Display", "Rτ Display"
            ])
        # ------------  timer  ------------
        self.timer = QtCore.QTimer(self)
        self.timer.setInterval(20)    # 50 Hz refresh
        self.timer.timeout.connect(self._update_everything)
        self.timer.start()

    # --------------------------------------------------------------------- UI helpers
    def _build_layout(self):
        main = QHBoxLayout(self)
        left = QVBoxLayout();  main.addLayout(left)
        plots = QVBoxLayout(); main.addLayout(plots, 1)

        # --- COM & connect row ---
        top = QHBoxLayout(); left.addLayout(top)
        top.addWidget(QLabel("Port:"))
        self.cmb_port = QComboBox(); self.cmb_port.addItems(find_available_ports()); top.addWidget(self.cmb_port)
        self.btn_connect = QPushButton("Connect"); self.btn_connect.clicked.connect(self._connect_clicked)
        top.addWidget(self.btn_connect)
        self.btn_log = QPushButton("Start Logging"); top.addWidget(self.btn_log)

        # ★★★ 新增：Power 按钮（大、显眼） ★★★
        self.btn_power = QPushButton("POWER OFF")
        self.btn_power.setCheckable(True)
        self.btn_power.setChecked(False)         # 默认 OFF
        self.btn_power.setMinimumHeight(48)
        self.btn_power.setCursor(Qt.PointingHandCursor)
        self.btn_power.toggled.connect(self._on_power_toggled)
        top.addWidget(self.btn_power)

        # --- parameter box ---
        grp = QGroupBox("Torque Parameters, Peak Torque 9NM")
        grid = QGridLayout(grp); left.addWidget(grp)

        def make_dspin(val=0.0, mn=-20, mx=20, step=0.2, tip=""):
            sb = QDoubleSpinBox(); sb.setDecimals(2); sb.setSingleStep(step)
            sb.setRange(mn, mx); sb.setValue(val); sb.setToolTip(tip)
            sb.valueChanged.connect(self._tx_params); return sb

        def make_isin(val=0, mn=0, mx=200, step=1, tip=""):
            sb = QSpinBox(); sb.setRange(mn, mx); sb.setSingleStep(step)
            sb.setValue(val); sb.setToolTip(tip)
            sb.valueChanged.connect(self._tx_params); return sb

        # === 显示的 4 个旋钮 ===
        self.sb_k   = make_dspin(val=0.00, mn=0, mx=10, step=0.10, tip="Rescaling gain (0..10)")
        self.sb_k.valueChanged.connect(self._on_k_changed)

        self.sb_kf  = make_dspin(val=2.5, mn=0, mx=10, step=0.10, tip="Flex_Assist_gain (0..10)")
        self.sb_ke  = make_dspin(val=2.5, mn=0, mx=10, step=0.10, tip="Ext_Assist_gain (0..10)")
        self.sb_p_on   = make_dspin(val=2.5, mn=-10, mx=10, step=0.05, tip="Gate power-on threshold p_on (on τ·ω)")

        # === 隐蔽的 3 个默认量（不放 UI）===
        self.assist_def = 40   # Total_Delay_Idx
        self.offL_def   = 0    # L offset
        self.offR_def   = 0    # R offset
        self.sb_lead   = 35


        # === 加在 _build_layout() 的参数区里（self.sb_lead 定义之后）===
        self.sb_maxT = QDoubleSpinBox()
        self.sb_maxT.setDecimals(1)
        self.sb_maxT.setSingleStep(0.5)
        self.sb_maxT.setRange(0.0,15.0)   
        self.sb_maxT.setValue(12.0)
        self.sb_maxT.setToolTip("Max torque saturation (Nm)")
        self.sb_maxT.valueChanged.connect(self._tx_params)

        # === 原来的 labels/spins 改成 6 个 ===
        labels = ["gate_K", "Right_gain", "Left_gain", "gate_p_on",  "MaxT (Nm)"]
        spins  = [ self.sb_k,   self.sb_kf,  self.sb_ke,  self.sb_p_on, self.sb_maxT ]
        for i,(lb,sb) in enumerate(zip(labels,spins)):
            grid.addWidget(QLabel(lb), 0, i)
            grid.addWidget(sb,         1, i)

        # 右侧图区域
        self.plot_layout = plots



    def _build_plots(self):
    # ---------- 颜色 ----------
        pen_angle = pg.mkPen((0,150,0), width=2)   # 绿 – 角度
        pen_cmd   = pg.mkPen((0,0,255),  width=2)  # 蓝 – Torque Cmd
        pen_est   = pg.mkPen((255,0,0),  width=2)  # 红 – Torque Est

        # ===== LEFT LEG =====
        self.plot_left = pg.PlotWidget(background='w')
        self.plot_left.setTitle("Right Leg – Angle & Torque")
        self.plot_left.setLabel('left', "Angle (deg)")
        self.plot_left.setLabel('bottom', "Time (s)")
        self.plot_left.showGrid(x=True,y=True)

        # 主 view：角度
        v_left = self.plot_left.getViewBox()
        self.L_angle_line = self.plot_left.plot(pen=pen_angle, name="L Angle (θ)")

        # 右侧 view：力矩
        self.v_left_right = pg.ViewBox()
        self.plot_left.scene().addItem(self.v_left_right)
        self.v_left_right.setXLink(v_left)
        self.plot_left.showAxis('right')
        ax = self.plot_left.getAxis('right')
        ax.linkToView(self.v_left_right)
        ax.setLabel("Torque (Nm)")
        self.L_tau_d_line = pg.PlotDataItem(pen=pen_cmd)
        self.L_tau_line   = pg.PlotDataItem(pen=pen_est)
        self.v_left_right.addItem(self.L_tau_d_line)
        self.v_left_right.addItem(self.L_tau_line)

        # ★ legend：手动把右侧 view 的曲线也加进去
        legL = self.plot_left.addLegend(offset=(10,10))
        legL.addItem(self.L_angle_line, "R Angle (θ)")
        legL.addItem(self.L_tau_d_line,  "R Torque Cmd")
        legL.addItem(self.L_tau_line,    "R Torque Est")

        # 同步大小
        self.plot_left.getViewBox().sigResized.connect(
            lambda: self.v_left_right.setGeometry(self.plot_left.getViewBox().sceneBoundingRect())
        )
        self.plot_layout.addWidget(self.plot_left, 1)

        # ===== RIGHT LEG =====
        self.plot_right = pg.PlotWidget(background='w')
        self.plot_right.setTitle("Left Leg – Angle & Torque")
        self.plot_right.setLabel('left', "Angle (deg)")
        self.plot_right.setLabel('bottom', "Time (s)")
        self.plot_right.showGrid(x=True,y=True)

        v_right = self.plot_right.getViewBox()
        self.R_angle_line = self.plot_right.plot(pen=pen_angle, name="R Angle (θ)")

        self.v_right_right = pg.ViewBox()
        self.plot_right.scene().addItem(self.v_right_right)
        self.v_right_right.setXLink(v_right)
        self.plot_right.showAxis('right')
        axR = self.plot_right.getAxis('right')
        axR.linkToView(self.v_right_right)
        axR.setLabel("Torque (Nm)")
        self.R_tau_d_line = pg.PlotDataItem(pen=pen_cmd)
        self.R_tau_line   = pg.PlotDataItem(pen=pen_est)
        self.v_right_right.addItem(self.R_tau_d_line)
        self.v_right_right.addItem(self.R_tau_line)

        legR = self.plot_right.addLegend(offset=(10,10))
        legR.addItem(self.R_angle_line, "L Angle (θ)")
        legR.addItem(self.R_tau_d_line, "L Torque Cmd")
        legR.addItem(self.R_tau_line,   "L Torque Est")
        self.plot_right.getViewBox().sigResized.connect(
            lambda: self.v_right_right.setGeometry(self.plot_right.getViewBox().sceneBoundingRect())
        )

        self.plot_layout.addWidget(self.plot_right, 1)



    def _build_value_displays(self):
        """Small numerical readouts under the plots."""
        grid = QGridLayout(); self.plot_layout.addLayout(grid)
        font = QFont(); font.setPointSize(14)
        def lbl(txt):
            l = QLabel(txt); l.setFont(font); return l
        self.lbl_Lang = lbl("Lθ: 0.0°");   self.lbl_Rang = lbl("Rθ: 0.0°")
        self.lbl_Ltau = lbl("Lτ: 0.0 Nm"); self.lbl_Rtau = lbl("Rτ: 0.0 Nm")
        grid.addWidget(self.lbl_Lang,0,0); grid.addWidget(self.lbl_Rang,0,1)
        grid.addWidget(self.lbl_Ltau,1,0); grid.addWidget(self.lbl_Rtau,1,1)
        # 现有四个标签下，再补两条状态
        self.lbl_imu  = QLabel("IMU init: —")
        self.lbl_maxt = QLabel("MaxT: — Nm")
        font = QFont(); font.setPointSize(14)
        self.lbl_imu.setFont(font); self.lbl_maxt.setFont(font)
        grid.addWidget(self.lbl_imu,  2, 0)
        grid.addWidget(self.lbl_maxt, 2, 1)

    def _set_power_ui(self, on: bool):
        # 统一设置按钮样式/文本
        if on:
            self.btn_power.setChecked(True)
            self.btn_power.setText("POWER ON")
            self.btn_power.setStyleSheet(
                "font-size:24px; font-weight:700; padding:12px 20px;"
                "background:#2e7d32; color:white; border-radius:10px;"
            )
        else:
            self.btn_power.setChecked(False)
            self.btn_power.setText("POWER OFF")
            self.btn_power.setStyleSheet(
                "font-size:24px; font-weight:700; padding:12px 20px;"
                "background:#b71c1c; color:white; border-radius:10px;"
            )

    def _on_power_toggled(self, checked: bool):
        # 按钮切换 → 改 sb_k（会自动触发 _tx_params）
        if checked:
            if self.sb_k.value() <= 0.0:
                self.sb_k.setValue(5.0)   # ★ 默认上电到 5.0，可自行改
            self._set_power_ui(True)
        else:
            self.sb_k.setValue(0.0)       # 关电即 0
            self._set_power_ui(False)

    def _on_k_changed(self, v: float):
        # 旋钮改变 → 同步按钮外观；>0 视为 ON
        self._set_power_ui(v > 0.0)

    # ------------------------------------------------------------------ serial logic
    def _connect_clicked(self):
        port = self.cmb_port.currentText()
        try:
            self.ser = serial.Serial(port,115200,timeout=0)
            self.connected = True
            self.btn_connect.setText("Connected")
            self.btn_connect.setStyleSheet("background:#4caf50;color:white")
            # ★ 设置一次电源 UI（跟当前 sb_k 同步），并立刻下发
            self._set_power_ui(self.sb_k.value() > 0.0)
            self._tx_params()
        except serial.SerialException:
            QMessageBox.critical(self,"Error",f"Cannot open {port}")

    def _tx_params(self):
        if not (self.connected and self.ser): 
            return

        # 量化 ×100 → int16
        k100   = int(round(self.sb_k.value()  * 100))
        kf100  = int(round(self.sb_kf.value() * 100))
        ke100  = int(round(self.sb_ke.value() * 100))
        pon100 = int(round(self.sb_p_on.value() * 100))

        # 限幅以匹配 MCU 约束
        def clip16(x): return max(-32768, min(32767, x))
        k100  = clip16(k100);  kf100 = clip16(kf100); ke100 = clip16(ke100); pon100 = clip16(pon100)

        # 隐蔽量：40 / 0 / 0
        assist  = int(self.assist_def)   # uint8
        offL    = int(self.offL_def)     # int8
        offR    = int(self.offR_def)     # int8

        # 新增：gate 参数
        k_gate100   = int(round(6.0 * 100))   # 若希望 GUI 控 k，也可加一个 spin；此处默认 6.00
        lead_ms     = 35

        # ---- 构造 payload（17 字节）----
        # <hhhBbbhhB  = 3*int16 + uint8 + 2*int8 + 2*int16 + uint8  = 14 字节
        # 再补 2 字节 reserved，使总负载=16？注意：MCU 读 17 字节，这里我们补 2 → 16 + 1? 具体如下：
        # 我们会在末尾再 pad 2 字节，合计 16? 让我们明确：3*2 +1 +2*1 +2*2 +1 = 14 + 2 + 1? → 是 14
        # 因 MCU 端 readBytes(..., 17)，我们在尾部补 3 字节，使 payload=17。
        # 先打 14 字节“老结构”
        payload14 = struct.pack(
            '<hhhBbbhhB',
            k100, kf100, ke100,
            assist,
            offL, offR,
            k_gate100,
            pon100,
            lead_ms
        )   # ← 这里正好 14 字节

        # 写入 max_torque 到 index 14..15
        mt100 = int(round(self.sb_maxT.value() * 100))
        mt100 = max(-32768, min(32767, mt100))
        mt_bytes = struct.pack('<h', mt100)  # 2 字节，小端

        # 凑够 17 字节，再加 1 字节 pad（index 16）
        payload = payload14 + mt_bytes + b'\x00'   # 14+2+1 = 17

        # 帧头第三字节 0x14（=20），与 Teensy 的 findFrameHeader() 匹配
        header = struct.pack('<BBB', 0xA5, 0x5A, 0x14)

        self.ser.write(header + payload)


    # ------------------------------------------------------------- main update loop
    def _update_everything(self):
        if not self.connected: return
        self._read_serial()

    def _read_serial(self):
        # Need 32‑byte packet: A5 5A len(32) + 29 data
        if self.ser.in_waiting < 32: return
        if self.ser.read(1)!=b'\xA5': return
        if self.ser.read(1)!=b'\x5A': return
        if self.ser.read(1)!=b'\x20': return
        payload = self.ser.read(29)

        # === 解析：IMU init 状态与 max_torque 回读（data_ble[17], [18..19]）===
        if len(payload) >= 17:
            imu_ok_flag = payload[14]                         # 0 or 1
            mt100_rx    = int(payload[15] | (payload[16] << 8))  # 有符号 int16
            if mt100_rx >= 32768: mt100_rx -= 65536
            maxT_rx = mt100_rx / 100.0

            if imu_ok_flag == 1:
                self.lbl_imu.setText("IMU init: OK")
                self.lbl_imu.setStyleSheet("color:#2e7d32")
            else:
                self.lbl_imu.setText("IMU init: FAIL")
                self.lbl_imu.setStyleSheet("color:#c62828")

            self.lbl_maxt.setText(f"MaxT: {maxT_rx:.1f} Nm")


        data = struct.unpack('<hhhhhhh', payload[:14])
        t_ms, L_angle, R_angle, L_tau, R_tau, L_tau_d, R_tau_d = [x/100 for x in data]
        t = (t_ms * 10) / 1000.0           # ← 简单 *100，再换到秒

        self.t_buffer = self.t_buffer[1:] + [t]

        # --- push into circular buffers ---
        # self.t_buffer = self.t_buffer[1:]+[t_ms/1000]
        self.L_IMU_buf = self.L_IMU_buf[1:]+[L_angle]
        self.R_IMU_buf = self.R_IMU_buf[1:]+[R_angle]
        self.L_tau_buf = self.L_tau_buf[1:]+[L_tau]
        self.R_tau_buf = self.R_tau_buf[1:]+[R_tau]
        self.L_tau_d_buf = self.L_tau_d_buf[1:]+[L_tau_d]
        self.R_tau_d_buf = self.R_tau_d_buf[1:]+[R_tau_d]

        # --- update plots ---

        self.L_angle_line.setData(self.t_buffer, self.R_IMU_buf)
        self.L_tau_line.setData(self.t_buffer, self.L_tau_buf)
        self.L_tau_d_line.setData(self.t_buffer, self.L_tau_d_buf)

        self.R_angle_line.setData(self.t_buffer, self.L_IMU_buf)
        self.R_tau_line.setData(self.t_buffer, self.R_tau_buf)
        self.R_tau_d_line.setData(self.t_buffer, self.R_tau_d_buf)


        # --- numeric displays ---
        self.lbl_Lang.setText(f"Lθ: {L_angle:.1f}°")
        self.lbl_Rang.setText(f"Rθ: {R_angle:.1f}°")
        self.lbl_Ltau.setText(f"Lτ: {L_tau:.1f} Nm (cmd {L_tau_d:.1f})")
        self.lbl_Rtau.setText(f"Rτ: {R_tau:.1f} Nm (cmd {R_tau_d:.1f})")

        # --- numeric displays ---
        self.lbl_Lang.setText(f"Lθ: {L_angle:.1f}°")
        self.lbl_Rang.setText(f"Rθ: {R_angle:.1f}°")
        self.lbl_Ltau.setText(f"Lτ: {L_tau:.1f} Nm (cmd {L_tau_d:.1f})")
        self.lbl_Rtau.setText(f"Rτ: {R_tau:.1f} Nm (cmd {R_tau_d:.1f})")

        # --- 数据保存到 CSV ---
        # --- save to CSV (注意 R/L 对调) ---
        with open(self.csv_path, "a", newline="") as f:
            writer = csv.writer(f)
            writer.writerow([
                t,                       # timestamp
                R_angle,                 # R Angle (θ)
                L_tau_d,                 # R Torque Cmd
                L_tau,                   # R Torque Est
                L_angle,                 # L Angle (θ)
                R_tau_d,                 # L Torque Cmd
                R_tau,                   # L Torque Est
                f"{L_angle:.1f}",        # Lθ Display
                f"{R_angle:.1f}",        # Rθ Display
                f"{L_tau:.1f} (cmd {L_tau_d:.1f})",
                f"{R_tau:.1f} (cmd {R_tau_d:.1f})"
            ])


        

# -------------------------------------------------------------------------- main
if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    pg.setConfigOptions(antialias=True, foreground='k')  # smoother plots + black text
    w = MainWindow(); w.show()
    sys.exit(app.exec_())
