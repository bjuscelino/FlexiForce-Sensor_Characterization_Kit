#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
FlexiForce_v3.1 - LFO Single Screen (PySide6) - FULL FEATURES

Layout:
- Left: scrollable column with ALL connection/config/actuator/acquisition/export/log controls
- Right: module tabs with dual plots (R vs t + R vs P), progress bar, module-specific controls (Repeat cycles)

Core features (ported from v2.9):
- Serial connect/disconnect (reader thread + queue)
- Presets + parameters (area, N/unit, Rtop/Rbot, mode, Vref, Rfactor)
- Calibration using R_medido
- Actuator controls (Home, Zero, Stop, Move, Up/Down with step; clamp between ZERO and MAX)
- Hold force (P-control with Kp, step limited) while acquisition is ON
- 4 modules: Linearity, Lin+Hyst, Drift, Repeat (N cycles)
- Start/Stop acquisition
- Linear/Log10 scale toggle
- CSV export of active module
- Config save/load to JSON (config_flexiforce.json)
- Log hidden by default + show/hide toggle + copy/clear

Author: Juscelino Valter Barbosas Junior
"""

import sys
import os
import json
import csv
import time
import numpy as np
import serial
import serial.tools.list_ports as list_ports

from datetime import datetime
from queue import Queue, Empty

from PySide6.QtCore import Qt, QTimer, QThread, Signal
from PySide6.QtGui import QFont, QGuiApplication
from PySide6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QGridLayout,
    QPushButton, QLabel, QComboBox, QTextEdit, QLineEdit, QTabWidget, QGroupBox,
    QFileDialog, QMessageBox, QProgressBar, QSpinBox, QSizePolicy, QSplitter,
    QScrollArea, QFrame
)

from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure


APP_VERSION = "FlexiForce_v3.1"
APP_TITLE = f"FlexiForce Control GUI {APP_VERSION}"
AUTHOR = "Juscelino Valter Barbosas Junior"
BUILD_DATE = datetime.now().strftime("%d/%m/%Y")

BAUDRATE = 9600
READ_TIMEOUT = 0.1
UPDATE_MS = 100
CFG_FILE = "config_flexiforce.json"

SENSOR_PRESETS = {
    "Custom": {"area_mm2": 100.0},
    "A301-25": {"area_mm2": 71.3},
    "A201-25": {"area_mm2": 71.3},
    "A101-100": {"area_mm2": 71.3},
}

MODULES = [
    ("linearity", "Linearity",
     "Linearity Test – 5 steps up @ 20 / 40 / 60 / 80 / 100% da carga.", "TEST:LINEARITY", 8),
    ("lin_hyst", "Lin + Hyst",
     "Linearity & Hysteresis – subida 20→100% e descida 80→20%.", "TEST:LIN_HYST", 12),
    ("drift", "Drift",
     "Drift – carga estática em ~50% por 60 segundos.", "TEST:DRIFT", 60),
    ("repeat", "Repeat",
     "Repeatability – repete LIN_HYST N vezes.", "TEST:LIN_HYST", 12),
]


def list_serial_ports():
    try:
        return [p.device for p in list_ports.comports()]
    except Exception:
        return []


def safe_float(x, default=np.nan):
    try:
        return float(str(x).strip())
    except Exception:
        return default


def safe_int(x, default=None):
    try:
        return int(round(float(str(x).strip())))
    except Exception:
        return default


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def fmt_R_value(ohms: float) -> str:
    if not np.isfinite(ohms):
        return "--"
    if ohms >= 9.5e5:
        return f"{ohms/1e6:.3f} MΩ"
    if ohms >= 9.5e3:
        return f"{ohms/1e3:.3f} kΩ"
    return f"{ohms:.1f} Ω"


def fmt_R_units(arr):
    arr = np.array(arr, dtype=float)
    m = np.isfinite(arr)
    if not np.any(m):
        return arr, "R_sensor (Ω)"
    med = float(np.nanmedian(arr[m]))
    if med >= 9.5e5:
        return arr/1e6, "R_sensor (MΩ)"
    if med >= 9.5e3:
        return arr/1e3, "R_sensor (kΩ)"
    return arr, "R_sensor (Ω)"


# -------------------- Serial (Thread + Queue) --------------------

class SerialReader(QThread):
    line_received = Signal(str)

    def __init__(self, ser: serial.Serial):
        super().__init__()
        self.ser = ser
        self._run = True

    def run(self):
        while self._run and self.ser and self.ser.is_open:
            try:
                line = self.ser.readline().decode("utf-8", errors="ignore").strip()
                if line:
                    self.line_received.emit(line)
            except Exception:
                time.sleep(0.05)

    def stop(self):
        self._run = False


class SerialWorker:
    def __init__(self):
        self.ser: serial.Serial | None = None
        self.reader: SerialReader | None = None
        self.q: Queue[str] = Queue()

    def connect(self, port: str, baud=BAUDRATE, timeout=READ_TIMEOUT):
        self.disconnect()
        try:
            self.ser = serial.Serial(port, baudrate=baud, timeout=timeout)
            time.sleep(0.2)
            try:
                self.ser.reset_input_buffer()
                self.ser.reset_output_buffer()
            except Exception:
                pass

            self.reader = SerialReader(self.ser)
            self.reader.line_received.connect(lambda s: self.q.put(s))
            self.reader.start()
            return True, f"Conectado: {port} @ {baud}"
        except Exception as e:
            self.ser = None
            self.reader = None
            return False, f"Falha ao conectar: {e}"

    def disconnect(self):
        if self.reader and self.reader.isRunning():
            self.reader.stop()
            self.reader.wait(400)
        self.reader = None
        if self.ser:
            try:
                self.ser.close()
            except Exception:
                pass
        self.ser = None

    def is_connected(self):
        return self.ser is not None and getattr(self.ser, "is_open", False)

    def send_line(self, text: str):
        if not self.is_connected():
            return False, "Não conectado."
        if not text.endswith("\n"):
            text += "\n"
        try:
            self.ser.write(text.encode("utf-8"))
            return True, f">>> {text.strip()}"
        except Exception as e:
            return False, f"Erro ao enviar: {e}"


# -------------------- DataModel --------------------

class DataModel:
    def __init__(self):
        self.reset()
        self.n_per_unit = 1.0
        self.area_mm2 = 100.0
        self.Vref = 1023.0
        self.R_top = 22000.0
        self.R_bot = 22000.0
        self.mode = "direto"
        self.Rfactor = 1.0

    def reset(self):
        self.t0 = None
        self.t = []
        self.pos = []
        self.op = []
        self.vd = []
        self.lc = []
        self.Rs = []
        self.P = []
        self.F = []
        self.total_lines = 0
        self.valid_lines = 0

    def force_from_lc(self, lc):
        return lc * self.n_per_unit

    def pressure_from_force(self, F):
        area_m2 = self.area_mm2 * 1e-6
        if area_m2 <= 0:
            return np.nan
        return (F / area_m2) / 1e6

    def resistance_from_adc(self, vd):
        vd = float(vd)
        if not np.isfinite(vd) or vd <= 0:
            return np.nan
        return vd * self.Rfactor

    def add(self, pos, op, vd, lc):
        now = time.time()
        if self.t0 is None:
            self.t0 = now
        t = now - self.t0
        R = self.resistance_from_adc(vd)
        F = self.force_from_lc(lc)
        P = self.pressure_from_force(F)

        self.t.append(t)
        self.pos.append(pos)
        self.op.append(op)
        self.vd.append(vd)
        self.lc.append(lc)
        self.Rs.append(R)
        self.P.append(P)
        self.F.append(F)
        self.valid_lines += 1

    def rows(self):
        rows = [["t_s", "position_us", "opamp", "voltdiv", "loadcell",
                 "R_sensor_ohm", "force_N", "press_MPa"]]
        for i in range(len(self.t)):
            rows.append([
                f"{self.t[i]:.6f}", self.pos[i], self.op[i], self.vd[i], self.lc[i],
                "" if not np.isfinite(self.Rs[i]) else f"{self.Rs[i]:.6f}",
                "" if not np.isfinite(self.F[i]) else f"{self.F[i]:.6f}",
                "" if not np.isfinite(self.P[i]) else f"{self.P[i]:.6f}",
            ])
        return rows


# -------------------- Plot widgets --------------------

class DualPlot(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        lay = QHBoxLayout()
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(8)
        self.setLayout(lay)

        self.fig_rt = Figure(figsize=(6, 5), dpi=100)
        self.ax_rt = self.fig_rt.add_subplot(111)
        self.canvas_rt = FigureCanvas(self.fig_rt)
        self.canvas_rt.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

        self.fig_pr = Figure(figsize=(6, 5), dpi=100)
        self.ax_pr = self.fig_pr.add_subplot(111)
        self.canvas_pr = FigureCanvas(self.fig_pr)
        self.canvas_pr.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

        lay.addWidget(self.canvas_rt, 1)
        lay.addWidget(self.canvas_pr, 1)

        self.reset_axes()

    def reset_axes(self):
        self.ax_rt.cla()
        self.ax_rt.grid(True, alpha=0.3)
        self.ax_rt.set_xlabel("Tempo (s)")
        self.ax_rt.set_ylabel("R_sensor (Ω)")

        self.ax_pr.cla()
        self.ax_pr.grid(True, alpha=0.3)
        self.ax_pr.set_xlabel("Pressão (MPa)")
        self.ax_pr.set_ylabel("R_sensor (Ω)")

        self.canvas_rt.draw_idle()
        self.canvas_pr.draw_idle()

    def update(self, t, Rscaled, P, Rylabel, logscale: bool):
        # R vs t
        self.ax_rt.cla()
        self.ax_rt.grid(True, alpha=0.3)
        self.ax_rt.set_xlabel("Tempo (s)")
        self.ax_rt.set_ylabel(Rylabel)
        self.ax_rt.plot(t, Rscaled, lw=1)
        if logscale:
            try:
                self.ax_rt.set_yscale("log", base=10)
            except Exception:
                self.ax_rt.set_yscale("log")
        else:
            self.ax_rt.set_yscale("linear")
        self.canvas_rt.draw_idle()

        # R vs P
        self.ax_pr.cla()
        self.ax_pr.grid(True, alpha=0.3)
        self.ax_pr.set_xlabel("Pressão (MPa)")
        self.ax_pr.set_ylabel(Rylabel)
        m = np.isfinite(Rscaled) & np.isfinite(P)
        if np.any(m):
            self.ax_pr.scatter(P[m], Rscaled[m], s=10, alpha=0.85)
        if logscale:
            try:
                self.ax_pr.set_yscale("log", base=10)
            except Exception:
                self.ax_pr.set_yscale("log")
        else:
            self.ax_pr.set_yscale("linear")
        self.canvas_pr.draw_idle()


class ModuleTab(QWidget):
    start_clicked = Signal(str)
    clear_clicked = Signal(str)

    def __init__(self, mod_key: str, title: str, desc: str, is_repeat: bool, parent=None):
        super().__init__(parent)
        self.mod_key = mod_key
        self.is_repeat = is_repeat

        root = QVBoxLayout()
        root.setContentsMargins(8, 8, 8, 8)
        root.setSpacing(8)
        self.setLayout(root)

        header = QHBoxLayout()
        lbl = QLabel(desc)
        lbl.setWordWrap(True)
        header.addWidget(lbl, 1)

        if is_repeat:
            header.addWidget(QLabel("Ciclos:"))
            self.spin_cycles = QSpinBox()
            self.spin_cycles.setRange(1, 99)
            self.spin_cycles.setValue(3)
            header.addWidget(self.spin_cycles)
        else:
            self.spin_cycles = None

        self.btn_start = QPushButton("▶ Iniciar teste")
        self.btn_clear = QPushButton("🧹 Limpar")

        header.addWidget(self.btn_start)
        header.addWidget(self.btn_clear)
        root.addLayout(header)

        pb_row = QHBoxLayout()
        pb_row.addWidget(QLabel("Progresso:"))
        self.progress = QProgressBar()
        self.progress.setRange(0, 100)
        self.progress.setValue(0)
        pb_row.addWidget(self.progress, 1)
        root.addLayout(pb_row)

        self.plots = DualPlot()
        root.addWidget(self.plots, 1)

        self.btn_start.clicked.connect(lambda: self.start_clicked.emit(self.mod_key))
        self.btn_clear.clicked.connect(lambda: self.clear_clicked.emit(self.mod_key))


# -------------------- Main Window --------------------

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.serial = SerialWorker()

        # state
        self.acq = False
        self.first_ok = False
        self.logscale = False

        self.hold_force = False
        self._last_force = np.nan

        self.current_module = "linearity"
        self.data_by_module = {m[0]: DataModel() for m in MODULES}
        self.data = self.data_by_module[self.current_module]

        # repeat
        self.repeat_mode_active = False
        self.repeat_cycle_total = 0
        self.repeat_cycle_index = 0
        self.repeat_cycle_running = False

        # progress timing
        self.module_started_at = None
        self.module_expected_s = 10

        self._build_ui()
        self._load_cfg()
        self._refresh_ports()
        self._update_title()

        self.timer = QTimer(self)
        self.timer.timeout.connect(self._update_loop)
        self.timer.start(UPDATE_MS)

    # ---------- UI ----------
    def _apply_theme(self):
        self.setStyleSheet("""
            QWidget { background-color: #0f0f12; color: #e8e8ea; }
            QGroupBox {
                border: 1px solid #2a2a33;
                border-radius: 10px;
                margin-top: 10px;
                font-weight: 700;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 6px 0 6px;
                color: #d8c8ff;
            }
            QLineEdit, QComboBox, QSpinBox, QTextEdit {
                background-color: #14141a;
                border: 1px solid #2a2a33;
                border-radius: 8px;
                padding: 6px;
                selection-background-color: #7b2cff;
                selection-color: #ffffff;
            }
            QTabWidget::pane {
                border: 1px solid #2a2a33;
                border-radius: 10px;
                padding: 4px;
            }
            QTabBar::tab {
                background: #14141a;
                border: 1px solid #2a2a33;
                border-bottom: none;
                padding: 8px 12px;
                margin-right: 6px;
                border-top-left-radius: 10px;
                border-top-right-radius: 10px;
                color: #d0d0d6;
            }
            QTabBar::tab:selected {
                background: #1a1a22;
                color: #ffffff;
                border-color: #6f2cff;
            }
            QPushButton {
                background-color: #5a1db8;
                color: white;
                padding: 8px 12px;
                border-radius: 10px;
                font-weight: 700;
                border: 1px solid #7b2cff;
            }
            QPushButton:hover { background-color: #6f2cff; }
            QPushButton:pressed { background-color: #4a1698; }
            QPushButton:disabled {
                background-color: #242430;
                border-color: #2a2a33;
                color: #7f7f88;
            }
            QProgressBar {
                border: 1px solid #2a2a33;
                border-radius: 8px;
                background: #14141a;
                text-align: center;
                color: #ffffff;
            }
            QProgressBar::chunk {
                background-color: #7b2cff;
                border-radius: 8px;
            }
            QStatusBar { background: #0b0b0e; color: #bdbdc6; }
        """)

    def _build_ui(self):
        self.setWindowTitle(APP_TITLE)
        self.resize(1550, 980)
        self._apply_theme()

        splitter = QSplitter(Qt.Horizontal)
        self.setCentralWidget(splitter)

        # -------- Left scrollable column --------
        left_scroll = QScrollArea()
        left_scroll.setWidgetResizable(True)
        left_scroll.setFrameShape(QFrame.NoFrame)

        left_container = QWidget()
        left_scroll.setWidget(left_container)
        left_layout = QVBoxLayout(left_container)
        left_layout.setContentsMargins(8, 8, 8, 8)
        left_layout.setSpacing(10)

        splitter.addWidget(left_scroll)

        # -------- Right: module tabs --------
        right = QWidget()
        right_layout = QVBoxLayout(right)
        right_layout.setContentsMargins(8, 8, 8, 8)
        right_layout.setSpacing(8)

        self.mod_tabs = QTabWidget()
        self.mod_tab_widgets: dict[str, ModuleTab] = {}
        for mod_key, title, desc, cmd, exp_s in MODULES:
            tab = ModuleTab(mod_key, title, desc, is_repeat=(mod_key == "repeat"))
            self.mod_tabs.addTab(tab, title)
            self.mod_tab_widgets[mod_key] = tab
            tab.start_clicked.connect(self._start_test)
            tab.clear_clicked.connect(self._clear_graph)

        self.mod_tabs.currentChanged.connect(self._on_module_changed)
        right_layout.addWidget(self.mod_tabs, 1)

        splitter.addWidget(right)
        splitter.setSizes([520, 1030])

        # -------- Left controls groups --------
        # Connection
        g_conn = QGroupBox("Conexão Serial")
        gl = QGridLayout(g_conn)
        self.cb_port = QComboBox()
        self.btn_refresh_ports = QPushButton("🔄 Atualizar")
        self.btn_connect = QPushButton("🟢 Conectar")
        self.btn_disconnect = QPushButton("🔴 Desconectar")
        self.btn_disconnect.setEnabled(False)

        gl.addWidget(QLabel("Porta:"), 0, 0)
        gl.addWidget(self.cb_port, 0, 1, 1, 2)
        gl.addWidget(self.btn_refresh_ports, 1, 0)
        gl.addWidget(self.btn_connect, 1, 1)
        gl.addWidget(self.btn_disconnect, 1, 2)

        left_layout.addWidget(g_conn)

        self.btn_refresh_ports.clicked.connect(self._refresh_ports)
        self.btn_connect.clicked.connect(self._on_connect)
        self.btn_disconnect.clicked.connect(self._on_disconnect)

        # Sensor parameters
        g_params = QGroupBox("Parâmetros do Sensor")
        gp = QGridLayout(g_params)

        self.cb_sensor = QComboBox()
        self.cb_sensor.addItems(list(SENSOR_PRESETS.keys()))
        self.cb_sensor.setCurrentText("A301-25")

        self.ed_area = QLineEdit("71.3")
        self.ed_nper = QLineEdit("1.0")
        self.ed_rtop = QLineEdit("22000")
        self.ed_rbot = QLineEdit("22000")
        self.cb_mode = QComboBox()
        self.cb_mode.addItems(["Direto (sensor→GND)", "Invertido (sensor→Vref)"])
        self.cb_mode.setCurrentText("Direto (sensor→GND)")
        self.ed_vref = QLineEdit("1023")
        self.ed_rfact = QLineEdit("1.0")

        r = 0
        gp.addWidget(QLabel("Preset:"), r, 0); gp.addWidget(self.cb_sensor, r, 1)
        gp.addWidget(QLabel("Área (mm²):"), r, 2); gp.addWidget(self.ed_area, r, 3)
        r += 1
        gp.addWidget(QLabel("N/unid. LC:"), r, 0); gp.addWidget(self.ed_nper, r, 1)
        gp.addWidget(QLabel("R_top (Ω):"), r, 2); gp.addWidget(self.ed_rtop, r, 3)
        r += 1
        gp.addWidget(QLabel("R_bot (Ω):"), r, 0); gp.addWidget(self.ed_rbot, r, 1)
        gp.addWidget(QLabel("Modo:"), r, 2); gp.addWidget(self.cb_mode, r, 3)
        r += 1
        gp.addWidget(QLabel("Vref_ADC:"), r, 0); gp.addWidget(self.ed_vref, r, 1)
        gp.addWidget(QLabel("Fator R:"), r, 2); gp.addWidget(self.ed_rfact, r, 3)

        left_layout.addWidget(g_params)

        self.cb_sensor.currentTextChanged.connect(self._on_preset)

        # Calibration
        g_cal = QGroupBox("Calibração")
        gc = QGridLayout(g_cal)
        self.ed_rmeas = QLineEdit("")
        self.btn_calibrate = QPushButton("🧪 Calibrar (R_medido)")

        gc.addWidget(QLabel("R_medido (Ω):"), 0, 0)
        gc.addWidget(self.ed_rmeas, 0, 1)
        gc.addWidget(self.btn_calibrate, 1, 0, 1, 2)

        left_layout.addWidget(g_cal)
        self.btn_calibrate.clicked.connect(self._on_calibrate)

        # Actuator / Servo
        g_act = QGroupBox("Atuador / Servo")
        ga = QGridLayout(g_act)

        self.ed_zero = QLineEdit("1150")
        self.ed_home = QLineEdit("1300")
        self.ed_max = QLineEdit("1500")
        self.ed_step = QLineEdit("10")
        self.ed_move = QLineEdit("1400")

        self.btn_home = QPushButton("🏠 Home")
        self.btn_zero = QPushButton("🎯 Zero")
        self.btn_stop = QPushButton("⛔ Stop")
        self.btn_move = QPushButton("⬆️ Enviar MOVE")
        self.btn_up = QPushButton("🔼 Subir")
        self.btn_down = QPushButton("🔽 Descer")

        ga.addWidget(QLabel("ZERO (µs):"), 0, 0); ga.addWidget(self.ed_zero, 0, 1)
        ga.addWidget(QLabel("HOME (µs):"), 0, 2); ga.addWidget(self.ed_home, 0, 3)
        ga.addWidget(QLabel("MAX (µs):"), 1, 0); ga.addWidget(self.ed_max, 1, 1)
        ga.addWidget(QLabel("STEP (µs):"), 1, 2); ga.addWidget(self.ed_step, 1, 3)

        ga.addWidget(QLabel("MOVE (µs):"), 2, 0); ga.addWidget(self.ed_move, 2, 1)
        ga.addWidget(self.btn_move, 2, 2, 1, 2)
        ga.addWidget(self.btn_up, 3, 0, 1, 2)
        ga.addWidget(self.btn_down, 3, 2, 1, 2)

        ga.addWidget(self.btn_home, 4, 0)
        ga.addWidget(self.btn_zero, 4, 1)
        ga.addWidget(self.btn_stop, 4, 2, 1, 2)

        left_layout.addWidget(g_act)

        self.btn_home.clicked.connect(lambda: self._send_cmd("GOTO:HOME"))
        self.btn_zero.clicked.connect(lambda: self._send_cmd("GOTO:ZERO"))
        self.btn_stop.clicked.connect(lambda: self._send_cmd("STOP"))
        self.btn_move.clicked.connect(self._on_move)
        self.btn_up.clicked.connect(self._on_up)
        self.btn_down.clicked.connect(self._on_down)

        # Hold force
        g_hold = QGroupBox("Manter Força")
        gh = QGridLayout(g_hold)
        self.ed_f_target = QLineEdit("10.0")
        self.ed_kp = QLineEdit("1.0")
        self.btn_hold = QPushButton("🧲 OFF")

        gh.addWidget(QLabel("Alvo (N):"), 0, 0); gh.addWidget(self.ed_f_target, 0, 1)
        gh.addWidget(QLabel("Kp (µs/N):"), 0, 2); gh.addWidget(self.ed_kp, 0, 3)
        gh.addWidget(self.btn_hold, 1, 0, 1, 4)

        left_layout.addWidget(g_hold)
        self.btn_hold.clicked.connect(self._toggle_hold)

        # Live readouts
        g_live = QGroupBox("Leituras")
        glv = QGridLayout(g_live)
        self.lbl_force = QLabel("--")
        self.lbl_press = QLabel("--")
        self.lbl_r = QLabel("--")
        glv.addWidget(QLabel("Força (N):"), 0, 0); glv.addWidget(self.lbl_force, 0, 1)
        glv.addWidget(QLabel("Pressão (MPa):"), 1, 0); glv.addWidget(self.lbl_press, 1, 1)
        glv.addWidget(QLabel("R_sensor:"), 2, 0); glv.addWidget(self.lbl_r, 2, 1)
        left_layout.addWidget(g_live)

        # Acquisition + export
        g_acq = QGroupBox("Aquisição / Exportação")
        gaq = QGridLayout(g_acq)
        self.btn_scale = QPushButton("📈 Linear")
        self.btn_export_csv = QPushButton("💾 Exportar CSV")
        self.btn_acq = QPushButton("⏺ Iniciar")
        self.btn_acq.setEnabled(False)

        gaq.addWidget(QLabel("Escala:"), 0, 0); gaq.addWidget(self.btn_scale, 0, 1)
        gaq.addWidget(self.btn_export_csv, 1, 0, 1, 2)
        gaq.addWidget(self.btn_acq, 2, 0, 1, 2)
        left_layout.addWidget(g_acq)

        self.btn_scale.clicked.connect(self._toggle_scale)
        self.btn_export_csv.clicked.connect(self._export_csv_active)
        self.btn_acq.clicked.connect(self._toggle_acq)

        # Log
        g_log = QGroupBox("Log")
        glog = QVBoxLayout(g_log)
        hb = QHBoxLayout()
        self.btn_logtoggle = QPushButton("👁 Mostrar")
        self.btn_copy_log = QPushButton("📋 Copiar")
        self.btn_clear_log = QPushButton("🧽 Limpar")
        hb.addWidget(self.btn_logtoggle)
        hb.addStretch(1)
        hb.addWidget(self.btn_copy_log)
        hb.addWidget(self.btn_clear_log)
        glog.addLayout(hb)

        self.txt_log = QTextEdit()
        self.txt_log.setReadOnly(True)
        self.txt_log.setFont(QFont("Consolas", 9))
        self.txt_log.setMinimumHeight(220)
        self.txt_log.setVisible(False)  # hidden by default
        glog.addWidget(self.txt_log, 1)

        left_layout.addWidget(g_log)

        self.btn_logtoggle.clicked.connect(self._toggle_log_visibility)
        self.btn_copy_log.clicked.connect(self._copy_log)
        self.btn_clear_log.clicked.connect(self._clear_log)

        left_layout.addStretch(1)

        # status bar footer signature
        self.statusBar().showMessage(f"{AUTHOR} | {APP_VERSION} | Build: {BUILD_DATE}")

    # ---------- Logging ----------
    def log(self, txt: str):
        ts = datetime.now().strftime("%H:%M:%S")
        self.txt_log.append(f"[{ts}] {txt}")

    # ---------- Config save/load ----------
    def _cfg_dict(self):
        return {
            "preset": self.cb_sensor.currentText(),
            "area_mm2": self.ed_area.text(),
            "n_per_unit": self.ed_nper.text(),
            "R_top": self.ed_rtop.text(),
            "R_bot": self.ed_rbot.text(),
            "mode": self.cb_mode.currentText(),
            "Vref": self.ed_vref.text(),
            "Rfactor": self.ed_rfact.text(),
            "zero": self.ed_zero.text(),
            "home": self.ed_home.text(),
            "max": self.ed_max.text(),
            "step": self.ed_step.text(),
            "move": self.ed_move.text(),
            "Kp": self.ed_kp.text(),
            "F_target": self.ed_f_target.text(),
            "logscale": self.logscale,
        }

    def _apply_cfg(self, d: dict):
        def put(ed: QLineEdit, key: str):
            if key in d and str(d[key]).strip() != "":
                ed.setText(str(d[key]))

        if d.get("preset"):
            self.cb_sensor.setCurrentText(d["preset"])
            self._on_preset(d["preset"])

        put(self.ed_area, "area_mm2")
        put(self.ed_nper, "n_per_unit")
        put(self.ed_rtop, "R_top")
        put(self.ed_rbot, "R_bot")
        if d.get("mode"):
            self.cb_mode.setCurrentText(d["mode"])
        put(self.ed_vref, "Vref")
        put(self.ed_rfact, "Rfactor")

        put(self.ed_zero, "zero")
        put(self.ed_home, "home")
        put(self.ed_max, "max")
        put(self.ed_step, "step")
        put(self.ed_move, "move")
        put(self.ed_kp, "Kp")
        put(self.ed_f_target, "F_target")

        if "logscale" in d:
            self.logscale = bool(d["logscale"])
            self.btn_scale.setText("📈 Log10" if self.logscale else "📈 Linear")

    def _load_cfg(self):
        try:
            if os.path.exists(CFG_FILE):
                with open(CFG_FILE, "r", encoding="utf-8") as f:
                    self._apply_cfg(json.load(f))
        except Exception as e:
            # don't spam UI before log widget exists
            pass

    def _save_cfg(self):
        try:
            with open(CFG_FILE, "w", encoding="utf-8") as f:
                json.dump(self._cfg_dict(), f, ensure_ascii=False, indent=2)
        except Exception:
            pass

    # ---------- Serial UI ----------
    def _refresh_ports(self):
        cur = self.cb_port.currentText()
        ports = list_serial_ports()
        self.cb_port.clear()
        self.cb_port.addItems(ports)
        if cur in ports:
            self.cb_port.setCurrentText(cur)

    def _on_connect(self):
        port = self.cb_port.currentText().strip()
        if not port:
            QMessageBox.warning(self, "Atenção", "Selecione uma porta.")
            return
        ok, msg = self.serial.connect(port)
        self.log(msg)
        if ok:
            self.btn_connect.setEnabled(False)
            self.btn_disconnect.setEnabled(True)
            self.btn_acq.setEnabled(True)
        self._update_title()

    def _on_disconnect(self):
        self._stop_acq()
        self.serial.disconnect()
        self.btn_connect.setEnabled(True)
        self.btn_disconnect.setEnabled(False)
        self.btn_acq.setEnabled(False)
        self._update_title()
        self.log("Desconectado.")

    def _send_cmd(self, c: str):
        ok, msg = self.serial.send_line(c)
        self.log(msg)

    # ---------- Preset / Calib ----------
    def _on_preset(self, _=None):
        preset = self.cb_sensor.currentText()
        if preset in SENSOR_PRESETS:
            self.ed_area.setText(str(SENSOR_PRESETS[preset]["area_mm2"]))

    def _on_calibrate(self):
        Rm = safe_float(self.ed_rmeas.text(), np.nan)
        if not np.isfinite(Rm):
            QMessageBox.warning(self, "Calibrar", "Preencha R_medido (Ω) do multímetro.")
            return
        vals = np.array([v for v in self.data.vd if np.isfinite(v)], dtype=float)
        if vals.size == 0:
            QMessageBox.warning(self, "Calibrar", "Colete alguns pontos antes de calibrar.")
            return
        vd_med = float(np.median(vals[-20:]))
        if vd_med <= 0:
            QMessageBox.warning(self, "Calibrar", "Valor interno inválido.")
            return
        F = Rm / vd_med
        self.ed_rfact.setText(f"{F:.6g}")
        self.log(f"Fator R ajustado: {F:.6g}")

    # ---------- Actuator ----------
    def _clamp_move(self, val: int) -> int:
        z = safe_int(self.ed_zero.text(), 1150) or 1150
        m = safe_int(self.ed_max.text(), 1500) or 1500
        if z > m:
            z, m = m, z
        return int(clamp(val, z, m))

    def _on_move(self):
        move = safe_int(self.ed_move.text(), None)
        if move is None:
            QMessageBox.warning(self, "MOVE inválido", "Informe um valor numérico.")
            return
        move = self._clamp_move(move)
        self.ed_move.setText(str(move))
        self._send_cmd(f"MOVE:{move}")

    def _on_up(self):
        step = safe_int(self.ed_step.text(), None)
        if step is None or step <= 0:
            QMessageBox.warning(self, "STEP inválido", "Informe STEP>0.")
            return
        curr = safe_int(self.ed_move.text(), None)
        if curr is None:
            curr = safe_int(self.ed_home.text(), 1300) or 1300
        newv = self._clamp_move(curr + step)
        self.ed_move.setText(str(newv))
        self._send_cmd(f"MOVE:{newv}")

    def _on_down(self):
        step = safe_int(self.ed_step.text(), None)
        if step is None or step <= 0:
            QMessageBox.warning(self, "STEP inválido", "Informe STEP>0.")
            return
        curr = safe_int(self.ed_move.text(), None)
        if curr is None:
            curr = safe_int(self.ed_home.text(), 1300) or 1300
        newv = self._clamp_move(curr - step)
        self.ed_move.setText(str(newv))
        self._send_cmd(f"MOVE:{newv}")

    # ---------- Hold Force ----------
    def _toggle_hold(self):
        self.hold_force = not self.hold_force
        self.btn_hold.setText("🧲 ON" if self.hold_force else "🧲 OFF")
        if self.hold_force:
            self.log("Manter força: ON")
        else:
            self.log("Manter força: OFF")

    # ---------- Acquisition ----------
    def _toggle_scale(self):
        self.logscale = not self.logscale
        self.btn_scale.setText("📈 Log10" if self.logscale else "📈 Linear")
        self._refresh_plots_and_status()

    def _toggle_acq(self):
        if self.acq:
            self._stop_acq()
        else:
            self._start_acq()

    def _start_acq(self, reason: str | None = None):
        self.acq = True
        self.data.reset()
        self.first_ok = False
        self.module_started_at = time.time()
        self.btn_acq.setText("⏹ Parar")
        self._update_title()
        self.log("Aquisição iniciada." + (f" {reason}" if reason else ""))

    def _stop_acq(self):
        if not self.acq:
            return
        self.acq = False
        self.btn_acq.setText("⏺ Iniciar")
        self._update_title()
        self.log("Aquisição pausada.")

    # ---------- Module switching ----------
    def _on_module_changed(self, idx: int):
        mod_key = MODULES[idx][0]
        self.current_module = mod_key
        self.data = self.data_by_module[mod_key]
        self._refresh_plots_and_status()

    # ---------- Tests ----------
    def _start_test(self, module_name: str):
        # switch to module tab
        for i, m in enumerate(MODULES):
            if m[0] == module_name:
                self.mod_tabs.setCurrentIndex(i)
                break

        self.current_module = module_name
        self.data = self.data_by_module[module_name]
        self._clear_graph(module_name)

        exp_s = next(m[4] for m in MODULES if m[0] == module_name)
        self.module_expected_s = max(1, int(exp_s))
        self.module_started_at = time.time()

        if module_name == "repeat":
            self._start_repeat_test()
            return

        self._start_acq(reason=f"(módulo {module_name})")
        self._run_test_sequence(module_name)

    def _run_test_sequence(self, module_name: str):
        if not self.serial.is_connected():
            self.log("Não é possível iniciar o teste: não conectado.")
            return
        cmd = next(m[3] for m in MODULES if m[0] == module_name)
        self.log(f"Comando: {cmd}")
        self._send_cmd(cmd)

    # Repeat logic
    def _start_repeat_test(self):
        if not self.serial.is_connected():
            self.log("Não é possível iniciar Repeat: não conectado.")
            return
        tab = self.mod_tab_widgets["repeat"]
        n = int(tab.spin_cycles.value()) if tab.spin_cycles else 3
        n = int(clamp(n, 1, 99))

        self.repeat_mode_active = True
        self.repeat_cycle_total = n
        self.repeat_cycle_index = 0
        self.repeat_cycle_running = False

        self._start_acq(reason=f"(repeat {n} ciclos)")
        self.log(f"Repeat: {n} ciclo(s) de LIN_HYST.")
        self._start_next_repeat_cycle()

    def _start_next_repeat_cycle(self):
        if not self.repeat_mode_active:
            return
        if not self.serial.is_connected():
            self.log("Conexão perdida durante Repeat.")
            self.repeat_mode_active = False
            self.repeat_cycle_running = False
            return
        if self.repeat_cycle_index >= self.repeat_cycle_total:
            self.log("✅ Repeat concluído.")
            self.repeat_mode_active = False
            self.repeat_cycle_running = False
            return

        self.repeat_cycle_index += 1
        self.repeat_cycle_running = True
        self.log(f"▶️ Ciclo {self.repeat_cycle_index}/{self.repeat_cycle_total} (LIN_HYST)")
        self._send_cmd("TEST:LIN_HYST")
        self.module_started_at = time.time()

    # ---------- Clear / Export ----------
    def _clear_graph(self, module_name: str | None = None):
        if module_name is None:
            module_name = self.current_module
        if module_name in self.data_by_module:
            self.data_by_module[module_name].reset()
        self.data = self.data_by_module.get(module_name, self.data)

        self.lbl_r.setText("--")
        self.lbl_force.setText("--")
        self.lbl_press.setText("--")

        tab = self.mod_tab_widgets.get(module_name)
        if tab:
            tab.plots.reset_axes()
            tab.progress.setValue(0)

        self.log(f"Limpo: {module_name}")

    def _export_csv_active(self):
        if not self.data.t:
            QMessageBox.information(self, "Exportar CSV", "Sem dados para exportar.")
            return
        default = f"FlexiForce_{self.current_module}_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        fp, _ = QFileDialog.getSaveFileName(self, "Salvar CSV", default, "CSV (*.csv)")
        if not fp:
            return
        try:
            with open(fp, "w", newline="", encoding="utf-8") as f:
                w = csv.writer(f)
                for r in self.data.rows():
                    w.writerow(r)
            self.log(f"CSV salvo: {fp}")
        except Exception as e:
            QMessageBox.critical(self, "Erro ao salvar", str(e))

    # ---------- Log actions ----------
    def _toggle_log_visibility(self):
        vis = not self.txt_log.isVisible()
        self.txt_log.setVisible(vis)
        self.btn_logtoggle.setText("🙈 Ocultar" if vis else "👁 Mostrar")

    def _copy_log(self):
        QGuiApplication.clipboard().setText(self.txt_log.toPlainText())
        self.log("Log copiado ✅")

    def _clear_log(self):
        self.txt_log.clear()

    # ---------- Parsing & Update loop ----------
    def _parse_and_add(self, line: str) -> bool:
        parts = [p for p in line.replace(";", ",").replace("\t", ",").split(",") if p.strip() != ""]
        if len(parts) < 4:
            return False
        nums = []
        for p in parts:
            v = safe_float(p, None)
            if v is not None and np.isfinite(v):
                nums.append(v)
            if len(nums) == 4:
                break
        if len(nums) < 4:
            return False
        pos, op, vd, lc = nums[:4]
        self.data.add(pos, op, vd, lc)
        return True

    def _handle_control_line(self, line: str):
        txt = (line or "").strip()
        if not txt:
            return
        up = txt.upper()

        if "DONE LIN_HYST" in up:
            self.log("DONE LIN_HYST")
            if self.repeat_mode_active and self.repeat_cycle_running:
                self.repeat_cycle_running = False
                self._start_next_repeat_cycle()
            return

        if "DONE REPEAT" in up:
            self.log("DONE REPEAT")
            self.repeat_mode_active = False
            self.repeat_cycle_running = False
            return

    def _update_progress(self):
        tab = self.mod_tab_widgets.get(self.current_module)
        if not tab:
            return
        if self.module_started_at is None or not self.acq:
            return

        if self.current_module == "repeat" and self.repeat_mode_active:
            base = (self.repeat_cycle_index - 1) / max(1, self.repeat_cycle_total)
            within = min(1.0, (time.time() - self.module_started_at) / max(1, self.module_expected_s))
            prog = (base + within / max(1, self.repeat_cycle_total)) * 100.0
            tab.progress.setValue(int(clamp(prog, 0, 100)))
        else:
            elapsed = time.time() - self.module_started_at
            prog = (elapsed / max(1, self.module_expected_s)) * 100.0
            tab.progress.setValue(int(clamp(prog, 0, 100)))

    def _refresh_plots_and_status(self):
        if not self.data.valid_lines:
            return

        t = np.array(self.data.t, dtype=float)
        R = np.array(self.data.Rs, dtype=float)
        P = np.array(self.data.P, dtype=float)
        F = np.array(self.data.F, dtype=float)

        r_last = R[-1] if len(R) else np.nan
        f_last = F[-1] if len(F) else np.nan
        p_last = P[-1] if len(P) else np.nan

        self.lbl_r.setText(fmt_R_value(r_last))
        self.lbl_force.setText("--" if not np.isfinite(f_last) else f"{f_last:.3f}")
        self.lbl_press.setText("--" if not np.isfinite(p_last) else f"{p_last:.5f}")

        Rscaled, Rylabel = fmt_R_units(R)
        tab = self.mod_tab_widgets.get(self.current_module)
        if tab:
            tab.plots.update(t, Rscaled, P, Rylabel, self.logscale)

    def _sync_model_params(self):
        # apply UI params to ALL modules (so switching tab keeps consistent conversions)
        for dm in self.data_by_module.values():
            dm.n_per_unit = safe_float(self.ed_nper.text(), 1.0)
            dm.area_mm2 = safe_float(self.ed_area.text(), 100.0)
            dm.R_top = safe_float(self.ed_rtop.text(), 22000.0)
            dm.R_bot = safe_float(self.ed_rbot.text(), 22000.0)
            dm.Vref = safe_float(self.ed_vref.text(), 1023.0)
            dm.Rfactor = safe_float(self.ed_rfact.text(), 1.0)
            dm.mode = "direto" if self.cb_mode.currentText().startswith("Direto") else "invertido"

    def _update_loop(self):
        self._sync_model_params()

        # drain serial queue
        try:
            while True:
                line = self.serial.q.get_nowait()
                if self.acq:
                    self.data.total_lines += 1
                    if not self._parse_and_add(line):
                        self._handle_control_line(line)
                else:
                    self._handle_control_line(line)
        except Empty:
            pass

        # plots & progress
        if self.data.valid_lines > 0:
            self._refresh_plots_and_status()
        self._update_progress()

        # Hold force P-control (simple and safe)
        if self.hold_force and self.acq and self.data.valid_lines > 0 and self.serial.is_connected():
            curr_force = self.data.F[-1]
            if np.isfinite(curr_force):
                self._last_force = curr_force
                target = safe_float(self.ed_f_target.text(), 0.0)
                Kp = safe_float(self.ed_kp.text(), 1.0)
                err = target - curr_force
                step_us = int(round(Kp * err))
                if step_us != 0:
                    step_us = int(clamp(step_us, -20, 20))  # limit
                    curr = safe_int(self.ed_move.text(), safe_int(self.ed_home.text(), 1300) or 1300) or 1300
                    newv = self._clamp_move(curr + step_us)
                    if newv != curr:
                        self.ed_move.setText(str(newv))
                        self.serial.send_line(f"MOVE:{newv}")

        self._update_title()

    # ---------- Window title ----------
    def _update_title(self):
        if not self.serial.is_connected():
            self.setWindowTitle(f"🔴 {APP_TITLE}")
        else:
            self.setWindowTitle(f"{'🔵' if self.acq else '🟢'} {APP_TITLE}")

    # ---------- Qt close event ----------
    def closeEvent(self, event):
        self._save_cfg()
        try:
            self._stop_acq()
            self.serial.disconnect()
        except Exception:
            pass
        event.accept()


def main():
    app = QApplication(sys.argv)
    w = MainWindow()
    w.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
