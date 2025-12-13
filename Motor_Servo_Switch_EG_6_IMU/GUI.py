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
        self.setWindowTitle("Hip‑Exo Energy gate Controller v2.0")

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
        self._imu_init_request = False
        self._motor_init_request = False

        # ------------  build UI  ------------
        self._build_layout()
        self._build_plots()
        self._build_value_displays()

        # === ★ 默认 Power OFF 设置 ===
        self._maxT_before_off = 12.0     # 记忆上次非零力矩上限
        self.sb_max_torque_cfg.setValue(0.0)  # 启动默认0.0Nm
        self._set_power_ui(False)             # 按钮红色 OFF 状态


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
        grp = QGroupBox("Hip-Exo Parameters (Downlink = 32 bytes)")
        grid = QGridLayout(grp); left.addWidget(grp)

        def make_dspin(val=0.0, mn=-20, mx=20, step=0.01, dec=2, tip=""):
            sb = QDoubleSpinBox()
            sb.setDecimals(dec)
            sb.setSingleStep(step)
            sb.setRange(mn, mx)
            sb.setValue(val)
            sb.setToolTip(tip)
            sb.valueChanged.connect(self._tx_params)
            return sb

        def make_isin(val=0, mn=0, mx=200, step=1, tip=""):
            sb = QSpinBox()
            sb.setRange(mn, mx)
            sb.setSingleStep(step)
            sb.setValue(val)
            sb.setToolTip(tip)
            sb.valueChanged.connect(self._tx_params)
            return sb

        # === 可视化旋钮（建议这些放 UI） ===
        # self.sb_Rescaling_gain     = make_dspin(0.00, 0, 10, 0.10, 2, "Rescaling_gain (0..10)")
        # self.sb_Rescaling_gain.valueChanged.connect(self._on_k_changed)  # 电源键联动

        # self.sb_Flex_Assist_gain   = make_dspin(2.50, 0, 10, 0.10, 2, "Flex_Assist_gain (0..10)")
        # self.sb_Ext_Assist_gain    = make_dspin(2.50, 0, 10, 0.10, 2, "Ext_Assist_gain (0..10)")
        self.sb_gate_k             = make_dspin(1.00, 0, 10, 0.10, 2, "gate_k (0..10)")
        self.sb_gate_p_on          = make_dspin(8, 0, 50, 1, 2, "gate_p_on (0..10)")
        self.sb_scale_all          = make_dspin(0.20, -1.0, 1.0, 0.01, 2, "scale_all (-1..1)")
# 新增：max_torque_cfg 旋钮（2.0~15.0 Nm）
        self.sb_max_torque_cfg = make_dspin(
            val=12.0, mn=0.0, mx=15.0, step=0.1, dec=1,
            tip="max_torque_cfg (Nm) — 力矩上限，0.0~15.0"
        )

        
        # self.sb_lead_frac          = make_dspin(0.060, 0.0, 0.100, 0.001, 3, "lead_frac (0..0.1)")
        self.sb_ext_phase_frac_L   = make_dspin(0.300, 0.0, 0.500, 0.001, 3, "ext_phase_frac_L (0..0.5)")
        self.sb_ext_phase_frac_R   = make_dspin(0.300, 0.0, 0.500, 0.001, 3, "ext_phase_frac_R (0..0.5)")
        self.sb_ext_gain           = make_dspin(0.50, -3.0, 3.0, 0.01, 2, "ext_gain (-3..3)")
        self.sb_Assist_delay_gain  = make_isin(40, 0, 99, 1, "Assist_delay_gain index (0..99)")

        

        # === 隐藏但固定下发（phase_offset_*） ===
        self.offL_def = 0  # int8
        self.offR_def = 0  # int8
        # === 隐藏但固定下发参数 ===
        self.Rescaling_gain_def   = 0.00
        self.Flex_Assist_gain_def = 2.50
        self.Ext_Assist_gain_def  = 2.50
        self.lead_frac_def        = 0.060
        
        # === 摆放到网格（按你喜好排）===
        labels = [
            "gate_k", "gate_p_on", "Assist_delay_gain",
            "ext_phase_frac_L", "ext_phase_frac_R",
            "ext_gain", "scale_all"
        ]
        spins = [

            self.sb_gate_k, self.sb_gate_p_on, self.sb_Assist_delay_gain,
            self.sb_ext_phase_frac_L, self.sb_ext_phase_frac_R,
            self.sb_ext_gain, self.sb_scale_all
        ]
        labels += ["max_torque_cfg"]
        spins  += [ self.sb_max_torque_cfg ]
        # 两行排版
        for i, (lb, sb) in enumerate(zip(labels, spins)):
            r, c = divmod(i, 4)
            grid.addWidget(QLabel(lb), r*2, c)
            grid.addWidget(sb,        r*2+1, c)
        cols = 4

        # ================== 在两行控件“下面”插入 LogTag 行 ==================
        # 计算下一个可用的“网格行”（注意我们每行占两行：标签行+控件行）
        rows_used = (len(labels) + cols - 1) // cols      # 逻辑行数（这里=2）
        base_row  = 2 * rows_used                         # 下一块内容从这里开始

        # 分隔线（可选）
        line = QFrame()
        line.setFrameShape(QFrame.HLine)
        line.setFrameShadow(QFrame.Sunken)
        grid.addWidget(line, base_row, 0, 1, cols)       # 跨满所有列

        # LogTag 行（更大更显眼）
        logtag_row = QHBoxLayout()
        logtag_row.setSpacing(8)

        lbl_logtag = QLabel("LogTag:")
        lbl_logtag.setStyleSheet("font-size:15px; font-weight:600;")
        logtag_row.addWidget(lbl_logtag)

        self.edt_label = QLineEdit()
        self.edt_label.setPlaceholderText("Enter log label (max 10 chars)")
        self.edt_label.setMaxLength(10)
        self.edt_label.setFixedWidth(240)
        self.edt_label.setMinimumHeight(34)
        self.edt_label.setStyleSheet("""
            QLineEdit { font-size:16px; padding:6px 10px; border:2px solid #1976d2; border-radius:8px; }
            QLineEdit:focus { border-color:#0d47a1; }
        """)
        self.edt_label.returnPressed.connect(self._send_logtag)
        logtag_row.addWidget(self.edt_label)

        self.chk_label_persist = QCheckBox("Persist")
        self.chk_label_persist.setStyleSheet("font-size:14px;")
        logtag_row.addWidget(self.chk_label_persist)

        btn_send_label = QPushButton("SEND LABEL")
        btn_send_label.clicked.connect(self._send_logtag)
        btn_send_label.setMinimumHeight(38)
        btn_send_label.setMinimumWidth(140)
        btn_send_label.setCursor(Qt.PointingHandCursor)
        btn_send_label.setStyleSheet("""
            QPushButton {
                background:#e53935; color:white; font-size:16px; font-weight:700;
                padding:6px 14px; border:none; border-radius:10px;
            }
            QPushButton:hover { background:#c62828; }
            QPushButton:pressed { background:#b71c1c; }
        """)
        logtag_row.addWidget(btn_send_label)
        logtag_row.addStretch(1)

        # 把整行 LogTag 放到 grid 的下一行，并让它“横跨所有列”
        grid.addLayout(logtag_row, base_row + 1, 0, 1, cols)

        # ================== 这里是你要的新按钮 ==================
        # 这个按钮的逻辑：每点一次，自动把 edt 里写成 "2mile" / "3mile" / "4mile"...
        # 然后自动勾上 Persist，然后调你现成的 self._send_logtag()
        self._next_mile_value = 2   # 起始就是你说的 2mile

        auto_row = QHBoxLayout()
        auto_row.setSpacing(8)

        self.btn_auto_mile = QPushButton(f"AUTO Label (next: {self._next_mile_value}ms)")
        self.btn_auto_mile.setMinimumHeight(58)
        self.btn_auto_mile.setCursor(Qt.PointingHandCursor)
        self.btn_auto_mile.setStyleSheet("""
            QPushButton {
                background:#4caf50;
                color:white;
                font-size:20px;
                font-weight:700;
                padding:200px 300px;
                border:none;
                border-radius:14px;
            }
            QPushButton:hover { background:#43a047; }
            QPushButton:pressed { background:#388e3c; }
        """)
        self.btn_auto_mile.clicked.connect(self._auto_cycle_mile)
        auto_row.addWidget(self.btn_auto_mile)
        auto_row.addStretch(1)

        # 放在 LogTag 行的下面一行
        grid.addLayout(auto_row, base_row + 2, 0, 1, cols)

        # ==== ✅ 新增：SD 文件状态提示 ====
        self.lbl_init_status = QLabel("SD log: FAIL")  # 默认就当没建成功
        self.lbl_init_status.setStyleSheet("font-size:16px; color:#c62828; padding:6px;")
        grid.addWidget(self.lbl_init_status, base_row + 3, 0, 1, cols)


        # 右侧图区域
        self.plot_layout = plots
        self.lbl_status = QLabel("Idle")
        self.lbl_status.setStyleSheet("color:#555; padding:4px 6px;")
        left.addWidget(self.lbl_status)
        self.lbl_imul = QLabel("IMUL —")
        self.lbl_imur = QLabel("IMUR —")
        self.lbl_imu1 = QLabel("IMU1 —")
        self.lbl_imu2 = QLabel("IMU2 —")
        self.lbl_imu3 = QLabel("IMU3 —")
        self.lbl_imu4 = QLabel("IMU4 —")

        self.lbl_imus = [
            self.lbl_imul, self.lbl_imur, self.lbl_imu1,
            self.lbl_imu2, self.lbl_imu3, self.lbl_imu4
                ]
        for lbl in self.lbl_imus:
            lbl.setStyleSheet("color:#777; font-size:14px; padding:2px;")

        # === ✅ 新增 IMU Init 按钮 ===
        self.btn_imu_init = QPushButton("INIT")
        self.btn_imu_init.setFixedWidth(60)
        self.btn_imu_init.setCursor(Qt.PointingHandCursor)
        self.btn_imu_init.setStyleSheet("""
            QPushButton {
                background-color: #2196f3;
                color: white;
                font-size: 14px;
                font-weight: bold;
                border: none;
                border-radius: 6px;
                padding: 4px 6px;
            }
            QPushButton:hover { background-color: #1976d2; }
            QPushButton:pressed { background-color: #1565c0; }
        """)
        self.btn_imu_init.clicked.connect(self._on_click_imu_init)

        # 新建一行水平布局，把按钮 + 6 个 label 都放进去
        imu_row = QHBoxLayout()
        imu_row.addWidget(self.btn_imu_init)
        for lbl in self.lbl_imus:
            imu_row.addWidget(lbl)
        imu_row.addStretch(1)

        # 把这行加进 left 主布局
        left.addLayout(imu_row)
        # === ✅ 新增 MOTOR Init 按钮 ===
        self.btn_motor_init = QPushButton("MOTOR INIT")
        self.btn_motor_init.setFixedWidth(90)
        self.btn_motor_init.setCursor(Qt.PointingHandCursor)
        self.btn_motor_init.setStyleSheet("""
            QPushButton {
                background-color: #ff9800;
                color: white;
                font-size: 14px;
                font-weight: bold;
                border: none;
                border-radius: 6px;
                padding: 4px 6px;
            }
            QPushButton:hover { background-color: #fb8c00; }
            QPushButton:pressed { background-color: #ef6c00; }
        """)
        self.btn_motor_init.clicked.connect(self._on_click_motor_init)

        # 把这个按钮加在 IMU 那一行的后面
        imu_row.addWidget(self.btn_motor_init)

    def _on_click_imu_init(self):
        """点击 IMU INIT 按钮时触发：设标志并立即发送"""
        if not (self.connected and self.ser):
            print("[GUI] Not connected, cannot send IMU reinit.")
            return

        # --- 设定标志 ---
        self._imu_init_request = True
        print("[GUI] IMU init requested.")

        try:
            self._tx_params()  # 主动发出参数帧（payload[25]=1）
            self.ser.flush()   # 强制写出缓冲区
            print("[GUI] IMU reinit frame sent.")
        except Exception as e:
            print(f"[GUI] Send error: {e}")

        # --- 延迟清零（避免太快被清掉）---
        QTimer.singleShot(100, lambda: setattr(self, "_imu_init_request", False))

    def _on_click_motor_init(self):
        """点击 MOTOR INIT 按钮时触发：设标志并立即发送"""
        if not (self.connected and self.ser):
            print("[GUI] Not connected, cannot send MOTOR reinit.")
            return

        self._motor_init_request = True
        print("[GUI] MOTOR init requested.")

        try:
            self._tx_params()  # 主动发出参数帧（payload[26]=1）
            self.ser.flush()
            print("[GUI] MOTOR reinit frame sent.")
        except Exception as e:
            print(f"[GUI] Send error: {e}")

        # 延迟清零
        QTimer.singleShot(100, lambda: setattr(self, "_motor_init_request", False))



    def _auto_cycle_mile(self):
        """
        自动 mile 按钮的处理：
        1. 把输入框内容设成当前要发的 mile，比如 "2mile"
        2. 自动勾选 Persist（因为你说“要一直发送”）
        3. 调用现有的 self._send_logtag()，走你原来的写入/发送流程
        4. 把下一次要发的 mile +1，并更新按钮文案
        """
        text = f"{self._next_mile_value}mile"
        self.edt_label.setText(text)

        # 保证“一直发送”的体验和你上面那个一致
        self.chk_label_persist.setChecked(True)

        # 用你现有的发送逻辑；你原来是输完点按钮/回车，这里直接复用
        self._send_logtag()

        # 下次 +1
        self._next_mile_value += 1
        self.btn_auto_mile.setText(f"AUTO Label (next: {self._next_mile_value}log)")

        # # ================== 在两行控件“下面”插入 LogTag 行 ==================
        # # 计算下一个可用的“网格行”（注意我们每行占两行：标签行+控件行）
        # rows_used = (len(labels) + cols - 1) // cols      # 逻辑行数（这里=2）
        # base_row  = 2 * rows_used                         # 下一块内容从这里开始

        # # 分隔线（可选）
        # line = QFrame()
        # line.setFrameShape(QFrame.HLine)
        # line.setFrameShadow(QFrame.Sunken)
        # grid.addWidget(line, base_row, 0, 1, cols)       # 跨满所有列

        # # LogTag 行（更大更显眼）
        # logtag_row = QHBoxLayout()
        # logtag_row.setSpacing(8)

        # lbl_logtag = QLabel("LogTag:")
        # lbl_logtag.setStyleSheet("font-size:15px; font-weight:600;")
        # logtag_row.addWidget(lbl_logtag)

        # self.edt_label = QLineEdit()
        # self.edt_label.setPlaceholderText("Enter log label (max 10 chars)")
        # self.edt_label.setMaxLength(10)
        # self.edt_label.setFixedWidth(240)
        # self.edt_label.setMinimumHeight(34)
        # self.edt_label.setStyleSheet("""
        #     QLineEdit { font-size:16px; padding:6px 10px; border:2px solid #1976d2; border-radius:8px; }
        #     QLineEdit:focus { border-color:#0d47a1; }
        # """)
        # self.edt_label.returnPressed.connect(self._send_logtag)
        # logtag_row.addWidget(self.edt_label)

        # self.chk_label_persist = QCheckBox("Persist")
        # self.chk_label_persist.setStyleSheet("font-size:14px;")
        # logtag_row.addWidget(self.chk_label_persist)

        # btn_send_label = QPushButton("SEND LABEL")
        # btn_send_label.clicked.connect(self._send_logtag)
        # btn_send_label.setMinimumHeight(38)
        # btn_send_label.setMinimumWidth(140)
        # btn_send_label.setCursor(Qt.PointingHandCursor)
        # btn_send_label.setStyleSheet("""
        #     QPushButton {
        #         background:#e53935; color:white; font-size:16px; font-weight:700;
        #         padding:6px 14px; border:none; border-radius:10px;
        #     }
        #     QPushButton:hover { background:#c62828; }
        #     QPushButton:pressed { background:#b71c1c; }
        # """)
        # logtag_row.addWidget(btn_send_label)
        # logtag_row.addStretch(1)

        # # 把整行 LogTag 放到 grid 的下一行，并让它“横跨所有列”
        # grid.addLayout(logtag_row, base_row + 1, 0, 1, cols)
        # # 右侧图区域
        # self.plot_layout = plots
        # self.lbl_status = QLabel("Idle")
        # self.lbl_status.setStyleSheet("color:#555; padding:4px 6px;")
        # left.addWidget(self.lbl_status)



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
    # 按钮切换 → 改 max_torque_cfg（会自动触发 _tx_params）
        if checked:
            # 恢复到关电前的值（若为0则给默认12.0）
            val = self._maxT_before_off if self._maxT_before_off > 0.0 else 12.0
            self.sb_max_torque_cfg.setValue(val)
            self._set_power_ui(True)
        else:
            # 记忆当前上限，然后设为0作为“关闭”
            self._maxT_before_off = float(self.sb_max_torque_cfg.value())
            self.sb_max_torque_cfg.setValue(0.0)
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
            # ★ 设置一次电源 UI（跟当前 sb_max_torque_cfg 同步），并立刻下发
            self._set_power_ui(self.sb_max_torque_cfg.value() > 0.0)
            self._tx_params()
        except serial.SerialException:
            QMessageBox.critical(self,"Error",f"Cannot open {port}")

    def _tx_params(self):
        if not (self.connected and self.ser):
            return

        # ---------- helpers ----------
        def clip16(x):  # 有符号16位
            return max(-32768, min(32767, int(x)))

        def s16_from_float(x, scale):  # 缩放后转int16
            return clip16(round(x * scale))

        # ---------- 读取UI（即使某些控件隐藏，也要下发固定值） ----------
        Rescaling_gain     = float(self.Rescaling_gain_def)
        Flex_Assist_gain   = float(self.Flex_Assist_gain_def)
        Ext_Assist_gain    = float(self.Ext_Assist_gain_def)
        Assist_delay_gain  = int(self.sb_Assist_delay_gain.value())
        phase_offset_L     = int(self.offL_def)   # 隐藏但固定下发
        phase_offset_R     = int(self.offR_def)   # 隐藏但固定下发
        gate_k             = float(self.sb_gate_k.value())
        gate_p_on          = float(self.sb_gate_p_on.value())
        lead_frac          = float(self.lead_frac_def)
        ext_phase_frac_L   = float(self.sb_ext_phase_frac_L.value())
        ext_phase_frac_R   = float(self.sb_ext_phase_frac_R.value())
        ext_gain           = float(self.sb_ext_gain.value())
        scale_all          = float(self.sb_scale_all.value())
        max_torque_cfg     = float(self.sb_max_torque_cfg.value())  # ★ 新增

        # ---------- 限幅（与固件一致） ----------
        Rescaling_gain    = max(0.0,  min(10.0, Rescaling_gain))
        Flex_Assist_gain  = max(0.0,  min(10.0, Flex_Assist_gain))
        Ext_Assist_gain   = max(0.0,  min(10.0, Ext_Assist_gain))
        Assist_delay_gain = max(0,    min(99,   Assist_delay_gain))
        gate_k            = max(0.0,  min(10.0, gate_k))
        gate_p_on         = max(0.0,  min(1000.0, gate_p_on))
        lead_frac         = max(0.0,  min(0.1,  lead_frac))
        ext_phase_frac_L  = max(0.0,  min(0.5,  ext_phase_frac_L))
        ext_phase_frac_R  = max(0.0,  min(0.5,  ext_phase_frac_R))
        ext_gain          = max(-3.0, min(3.0,  ext_gain))   # 如果你只想负值，把上限改成 0.0
        scale_all         = max(-1.0, min(1.0,  scale_all))
        max_torque_cfg    = max(0.0,  min(15.0, max_torque_cfg))   # ★ 新增

        # ---------- 打包payload（29字节） ----------
        payload = bytearray(29)  # 全部清零（含占位23..28）

        def put_s16(ix, val):    # 小端
            v = s16_from_float(val, 100)
            payload[ix]   = (v & 0xFF)
            payload[ix+1] = ((v >> 8) & 0xFF)

        def put_s16x1000(ix, val):
            v = s16_from_float(val, 1000)
            payload[ix]   = (v & 0xFF)
            payload[ix+1] = ((v >> 8) & 0xFF)

        # 对齐固件偏移
        put_s16(0,  Rescaling_gain)     # [0..1]
        put_s16(2,  Flex_Assist_gain)   # [2..3]
        put_s16(4,  Ext_Assist_gain)    # [4..5]

        payload[6] = Assist_delay_gain  # [6]

        payload[7] = (phase_offset_L & 0xFF)  # [7]
        payload[8] = (phase_offset_R & 0xFF)  # [8]

        put_s16(9,  gate_k)              # [9..10]
        put_s16(11, gate_p_on)           # [11..12]

        put_s16x1000(13, lead_frac)      # [13..14]
        put_s16x1000(15, ext_phase_frac_L)  # [15..16]
        put_s16x1000(17, ext_phase_frac_R)  # [17..18]

        put_s16(19, ext_gain)            # [19..20]
        put_s16(21, scale_all)           # [21..22]
        put_s16(23, max_torque_cfg)
        payload[25] = 1 if self._imu_init_request else 0
        payload[26] = 1 if self._motor_init_request else 0


        # [23..28] 保留位：保持为 0

        # ---------- 帧头 + 发送（总长32字节） ----------
        header = struct.pack('<BBB', 0xA5, 0x5A, 0x20)
        self.ser.write(header + payload)


    def _send_logtag(self):
        """Send a LOGTAG frame: 32 bytes total."""
        if not (self.connected and self.ser):
            return

        tag = self.edt_label.text().encode('ascii', 'ignore')[:10]
        tag_len = len(tag)
        flags = 0x01 if self.chk_label_persist.isChecked() else 0x00

        payload = bytearray(29)
        payload[0] = ord('L')  # 'L'
        payload[1] = ord('G')  # 'G'
        payload[2] = tag_len
        payload[13] = flags
        if tag_len:
            payload[3:3+tag_len] = tag

        header = struct.pack('<BBB', 0xA5, 0x5A, 0x20)
        self.ser.write(header + payload)

    # ------------------------------------------------------------- main update loop
    def _update_everything(self):
        if not self.connected: return
        self._read_serial()

    def _read_serial(self):
        # 尝试把缓存里多余字节扔掉，直到找到 A5 5A 20 作为帧头
        while self.ser.in_waiting >= 3:
            head = self.ser.read(1)
            if head != b'\xA5':
                continue
            if self.ser.read(1) != b'\x5A':
                # 继续滚动
                continue
            if self.ser.read(1) != b'\x20':
                continue
            break
        else:
            return  # 缓冲里帧头还没凑齐

        if self.ser.in_waiting < 29:
            return  # 载荷没到齐，下次再读
        payload = self.ser.read(29)
        if len(payload) != 29:
            return

        # === IMU init 状态与 max_torque 回读（data_ble[17], [18..19]）===
        # payload[14] = imu_ok, [15..16] = max_torque_cfg ×100 (int16, little-endian)
        imu_ok_flag = payload[14]                         # 0 or 1
        mt100_rx = int(payload[15] | (payload[16] << 8))
        if mt100_rx >= 32768:  # 二补码还原
            mt100_rx -= 65536
        maxT_rx = mt100_rx / 100.0
        if imu_ok_flag == 1:
            self.lbl_imu.setText("IMU init: OK")
            self.lbl_imu.setStyleSheet("color:#2e7d32")
        else:
            self.lbl_imu.setText("IMU init: FAIL")
            self.lbl_imu.setStyleSheet("color:#c62828")
        self.lbl_maxt.setText(f"MaxT: {maxT_rx:.1f} Nm")
         # === ✅ 新增：这是板子发来的“SD 文件创建是否成功” ===
        # data_ble[20] → payload[17]
        sd_ok_flag = payload[17]   # 0 = fail, 1 = success
        if sd_ok_flag == 1:
            self.lbl_init_status.setText("SD log: OK")
            self.lbl_init_status.setStyleSheet("font-size:16px; color:#2e7d32; padding:6px;")
        else:
            # 保持红色，不用改也行，但写上更明确
            self.lbl_init_status.setText("SD log: FAIL")
            self.lbl_init_status.setStyleSheet("font-size:16px; color:#c62828; padding:6px;")

    # === 前14字节：7个 int16（t_cs, L_angle, R_angle, L_tau, R_tau, L_tau_d, R_tau_d）===
        data = struct.unpack('<hhhhhhh', payload[:14])
        t_cs, L_angle_i, R_angle_i, L_tau_i, R_tau_i, L_tau_d_i, R_tau_d_i = data
        # 单位换算：角/力矩 ÷100；时间厘秒→秒
        L_angle = L_angle_i / 100.0
        R_angle = R_angle_i / 100.0
        L_tau   = L_tau_i   / 100.0
        R_tau   = R_tau_i   / 100.0
        L_tau_d = L_tau_d_i / 100.0
        R_tau_d = R_tau_d_i / 100.0
        t       = (t_cs & 0xFFFF) / 100.0  # 0.01s 精度（~655.35s 回绕）

        # === 新增：步态频率 gait_freq（payload[20..21] = data_ble[23..24]）===
        gait_freq = None
        # 注意：payload 索引 0..28；确保有 index 21
        gf100 = int(payload[20] | (payload[21] << 8))
        if gf100 >= 32768:
            gf100 -= 65536
        gait_freq = gf100 / 100.0  # Hz
        # === 新增：LOGTAG 回显状态 ===
        tag_valid = payload[22]  # 假设固件发在 data_ble[25]
        tag_char  = chr(payload[23]) if payload[23] != 0 else ''
        if tag_valid:
            self.lbl_status.setText(
                f"Reading… t={t:.2f}s, gait={gait_freq:.2f} Hz, tag='{tag_char}'"
            )
        else:
            self.lbl_status.setText(
                f"Reading… t={t:.2f}s, gait={gait_freq:.2f} Hz, tag=—"
            )
        # === 🧩 新增：6 路 IMU 状态解析 ===
        imu_bits = payload[24]  # 对应 data_ble[27]
        labels = ["L", "R", "1", "2", "3", "4"]
        states = [(imu_bits >> i) & 0x01 for i in range(6)]

        # 这里可以用 QLabel 列表提前定义，比如：
        # self.lbl_imus = [self.lbl_imul, self.lbl_imur, self.lbl_imu1, self.lbl_imu2, self.lbl_imu3, self.lbl_imu4]
        for i, ok in enumerate(states):
            if ok:
                self.lbl_imus[i].setStyleSheet("color:#2e7d32; font-weight:bold;")  # 绿色
                self.lbl_imus[i].setText(f"IMU{labels[i]} OK")
            else:
                self.lbl_imus[i].setStyleSheet("color:#c62828; font-weight:bold;")  # 红色
                self.lbl_imus[i].setText(f"IMU{labels[i]} —")


        # --- push into circular buffers ---
        self.logging = False
        self.t_buffer    = self.t_buffer[1:]    + [t]
        self.L_IMU_buf   = self.L_IMU_buf[1:]   + [L_angle]
        self.R_IMU_buf   = self.R_IMU_buf[1:]   + [R_angle]
        self.L_tau_buf   = self.L_tau_buf[1:]   + [L_tau]
        self.R_tau_buf   = self.R_tau_buf[1:]   + [R_tau]
        self.L_tau_d_buf = self.L_tau_d_buf[1:] + [L_tau_d]
        self.R_tau_d_buf = self.R_tau_d_buf[1:] + [R_tau_d]

        # --- update plots ---（⚠️ 按你所说：左右映射保持现状，不改）
        self.L_angle_line.setData(self.t_buffer, self.R_IMU_buf)
        self.L_tau_line.setData(self.t_buffer, self.L_tau_buf)
        self.L_tau_d_line.setData(self.t_buffer, self.L_tau_d_buf)

        self.R_angle_line.setData(self.t_buffer, self.L_IMU_buf)
        self.R_tau_line.setData(self.t_buffer, self.R_tau_buf)
        self.R_tau_d_line.setData(self.t_buffer, self.R_tau_d_buf)

        # --- numeric displays ---
        self.lbl_Lang.setText(f"Lθ: {L_angle:.1f}°")
        self.lbl_Rang.setText(f"Rθ: {R_angle:.1f}°")
        self.lbl_Ltau.setText(f"Lτ: {L_tau:.1f} Nm (cmd {L_tau_d:.1f})")
        self.lbl_Rtau.setText(f"Rτ: {R_tau:.1f} Nm (cmd {R_tau_d:.1f})")

        # --- 数据保存到 CSV（脏点开关：self.logging 若不存在则默认 True） ---
        SAVE_CSV = getattr(self, 'logging', True)
        if False and getattr(self, 'csv_path', None):
            try:
                with open(self.csv_path, "a", newline="") as f:
                    writer = csv.writer(f)
                    writer.writerow([
                        t,                       # timestamp (s)
                        R_angle,                 # R Angle (θ)
                        L_tau_d,                 # R Torque Cmd
                        L_tau,                   # R Torque Est
                        L_angle,                 # L Angle (θ)
                        R_tau_d,                 # L Torque Cmd
                        R_tau,                   # L Torque Est
                        f"{L_angle:.1f}",        # Lθ Display
                        f"{R_angle:.1f}",        # Rθ Display
                        f"{L_tau:.1f} (cmd {L_tau_d:.1f})",
                        f"{R_tau:.1f} (cmd {R_tau_d:.1f})",
                        # 新增：可选记录
                        int(imu_ok_flag),
                        f"{maxT_rx:.2f}",
                        f"{gait_freq:.2f}" if gait_freq is not None else ""
                    ])
            except Exception:
                pass


        

# -------------------------------------------------------------------------- main
if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    pg.setConfigOptions(antialias=True, foreground='k')  # smoother plots + black text
    w = MainWindow(); w.show()
    sys.exit(app.exec_())
