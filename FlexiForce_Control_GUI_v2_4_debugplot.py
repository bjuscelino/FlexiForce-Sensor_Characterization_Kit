#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import json, os, time, csv, threading
from datetime import datetime
from queue import Queue, Empty
import tkinter as tk
from tkinter import ttk, messagebox, filedialog
import numpy as np
import serial, serial.tools.list_ports as list_ports
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure

APP_TITLE   = "FlexiForce Control GUI v2.2 (Módulos + ciclos)"
BAUDRATE    = 9600
READ_TIMEOUT= 0.1
UPDATE_MS   = 100
CFG_FILE    = "config_flexiforce.json"

SENSOR_PRESETS = {
    "Custom": {"area_mm2": 100.0},
    "A301-25": {"area_mm2": 71.3},
    "A201-25": {"area_mm2": 71.3},
    "A101-100": {"area_mm2": 71.3},
}

def list_serial_ports():
    return [p.device for p in serial.tools.list_ports.comports()]

def safe_float(x, default=np.nan):
    try:
        return float(x)
    except Exception:
        return default

def safe_int(x, default=None):
    try:
        return int(round(float(str(x).strip())))
    except Exception:
        return default

class SerialWorker:
    def __init__(self):
        self.ser = None
        self.keep = threading.Event()
        self.th = None
        self.q = Queue()

    def connect(self, port, baud=BAUDRATE, timeout=READ_TIMEOUT):
        self.disconnect()
        try:
            self.ser = serial.Serial(port, baudrate=baud, timeout=timeout)
            time.sleep(0.2)
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
            self.keep.set()
            self.th = threading.Thread(target=self._loop, daemon=True)
            self.th.start()
            return True, f"Conectado: {port} @ {baud}"
        except Exception as e:
            self.ser = None
            return False, f"Falha ao conectar: {e}"

    def disconnect(self):
        self.keep.clear()
        if self.th and self.th.is_alive():
            self.th.join(timeout=0.4)
        if self.ser:
            try:
                self.ser.close()
            except Exception:
                pass
        self.ser = None

    def is_connected(self):
        return self.ser is not None and self.ser.is_open

    def send_line(self, text):
        if not self.is_connected():
            return False, "Não conectado."
        if not text.endswith("\n"):
            text += "\n"
        try:
            self.ser.write(text.encode("utf-8"))
            return True, f">>> {text.strip()}"
        except Exception as e:
            return False, f"Erro ao enviar: {e}"

    def _loop(self):
        while self.keep.is_set() and self.ser and self.ser.is_open:
            try:
                line = self.ser.readline().decode("utf-8", errors="ignore").strip()
                if line:
                    self.q.put(line)
            except Exception:
                time.sleep(0.05)

class DataModel:
    def __init__(self):
        self.reset()
        self.n_per_unit = 1.0
        self.area_mm2   = 100.0
        self.Vref   = 1023.0
        self.R_top  = 22000.0
        self.R_bot  = 22000.0
        self.mode   = "direto"
        self.Rfactor= 1.0

    def reset(self):
        self.t0=None
        self.t=[]; self.pos=[]; self.op=[]; self.vd=[]; self.lc=[]
        self.Rs=[]; self.P=[]; self.F=[]
        self.total_lines=0; self.valid_lines=0

    def force_from_lc(self, lc):
        return lc * self.n_per_unit

    def pressure_from_force(self, F):
        area_m2 = self.area_mm2 * 1e-6
        if area_m2 <= 0:
            return np.nan
        return (F/area_m2)/1e6

    def resistance_from_adc(self, vd):
        """Converte leitura ADC em resistência aproximada.

        Em vez de assumir um divisor de tensão ideal, usamos um modelo
        linear simples R = Rfactor * vd. O botão "Calibrar!" ajusta
        Rfactor automaticamente para que R_sensor no ponto atual
        coincida com o valor medido no multímetro.
        """
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
        self.t.append(t); self.pos.append(pos); self.op.append(op); self.vd.append(vd); self.lc.append(lc)
        self.Rs.append(R); self.P.append(P); self.F.append(F)
        self.valid_lines += 1

    def rows(self):
        rows=[["t_s","position_us","opamp","voltdiv","loadcell","R_sensor_ohm","force_N","press_MPa"]]
        for i in range(len(self.t)):
            rows.append([
                f"{self.t[i]:.6f}", self.pos[i], self.op[i], self.vd[i], self.lc[i],
                "" if not np.isfinite(self.Rs[i]) else f"{self.Rs[i]:.6f}",
                "" if not np.isfinite(self.F[i])  else f"{self.F[i]:.6f}",
                "" if not np.isfinite(self.P[i])  else f"{self.P[i]:.6f}",
            ])
        return rows

class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.serial = SerialWorker()
        self.data   = DataModel()
        self.acq    = False
        self.first_ok=False
        self.logscale=False
        self.hold_force=False
        self.target_force_N=0.0
        self.Kp=1.0
        self._last_move = None
        self._last_force = np.nan

        self.current_module = "linearity"
        self.plots = {}
        self._tab_module = {}
        self.data_by_module = {}

        self.repeat_cycles_var = tk.IntVar(value=3)

        self._build_ui()
        # Cria um DataModel por módulo para que cada aba tenha seus próprios dados
        for mod in ["linearity","lin_hyst","drift","repeat"]:
            self.data_by_module[mod] = DataModel()
        self.data = self.data_by_module[self.current_module]
        self._load_cfg()
        self._update_title()
        self.protocol("WM_DELETE_WINDOW", self._on_close)
        self.after(UPDATE_MS, self._update_loop)

    def _build_ui(self):
        self.title(APP_TITLE)
        self.geometry("1320x920")
        self.minsize(1200,860)

        top = ttk.Frame(self, padding=6)
        top.grid(row=0, column=0, sticky="ew")
        for i in range(30):
            top.columnconfigure(i, weight=0)
        top.columnconfigure(29, weight=1)

        ttk.Label(top, text="Porta:").grid(row=0, column=0, sticky="e")
        self.cb_port = ttk.Combobox(top, values=list_serial_ports(), width=8)
        self.cb_port.grid(row=0, column=1, padx=4)
        ttk.Button(top, text="🔄 Atualizar", command=self.refresh_ports).grid(row=0, column=2, padx=4)
        self.btn_connect = ttk.Button(top, text="Conectar", command=self.on_connect)
        self.btn_connect.grid(row=0, column=3, padx=4)
        self.btn_disconnect = ttk.Button(top, text="Desconectar", command=self.on_disconnect, state="disabled")
        self.btn_disconnect.grid(row=0, column=4, padx=4)

        ttk.Label(top,text="Preset:").grid(row=0,column=5,sticky="e")
        self.cb_sensor=ttk.Combobox(top,values=list(SENSOR_PRESETS.keys()),state="readonly",width=10)
        self.cb_sensor.set("A301-25"); self.cb_sensor.grid(row=0,column=6,padx=2)
        self.cb_sensor.bind("<<ComboboxSelected>>", self.on_preset)

        ttk.Label(top,text="Área (mm²):").grid(row=0,column=7,sticky="e")
        self.ed_area=ttk.Entry(top,width=8); self.ed_area.insert(0,"71.3"); self.ed_area.grid(row=0,column=8,padx=2)

        ttk.Label(top,text="N/unid. LC:").grid(row=0,column=9,sticky="e")
        self.ed_nper=ttk.Entry(top,width=6); self.ed_nper.insert(0,"1.0"); self.ed_nper.grid(row=0,column=10,padx=2)

        ttk.Label(top,text="R_top (Ω):").grid(row=0,column=11,sticky="e")
        self.ed_rtop=ttk.Entry(top,width=8); self.ed_rtop.insert(0,"22000"); self.ed_rtop.grid(row=0,column=12,padx=2)

        ttk.Label(top,text="R_bot (Ω):").grid(row=0,column=13,sticky="e")
        self.ed_rbot=ttk.Entry(top,width=8); self.ed_rbot.insert(0,"22000"); self.ed_rbot.grid(row=0,column=14,padx=2)

        ttk.Label(top,text="Modo:").grid(row=0,column=15,sticky="e")
        self.cb_mode=ttk.Combobox(top,values=["Direto (sensor→GND)","Invertido (sensor→Vref)"],state="readonly",width=20)
        self.cb_mode.set("Direto (sensor→GND)"); self.cb_mode.grid(row=0,column=16,padx=2)

        ttk.Label(top,text="Vref_ADC:").grid(row=0,column=17,sticky="e")
        self.ed_vref=ttk.Entry(top,width=6); self.ed_vref.insert(0,"1023"); self.ed_vref.grid(row=0,column=18,padx=2)

        ttk.Label(top,text="Fator R:").grid(row=0,column=19,sticky="e")
        self.ed_rfact=ttk.Entry(top,width=6); self.ed_rfact.insert(0,"1.0"); self.ed_rfact.grid(row=0,column=20,padx=2)

        ttk.Label(top,text="R_medido (Ω):").grid(row=0,column=21,sticky="e")
        self.ed_rmeas=ttk.Entry(top,width=10); self.ed_rmeas.grid(row=0,column=22,padx=2)
        ttk.Button(top,text="Calibrar!",command=self.on_calibrate).grid(row=0,column=23,padx=4)

        self.btn_scale=ttk.Button(top,text="Escala: Linear",command=self.on_toggle_scale); self.btn_scale.grid(row=0,column=24,padx=6)
        ttk.Button(top,text="Exportar CSV",command=self.on_export_csv).grid(row=0,column=25,padx=6)
        self.btn_acq=ttk.Button(top,text="Iniciar Aquisição",command=self.on_toggle_acq,state="disabled"); self.btn_acq.grid(row=0,column=26,padx=6)

        ctrl=ttk.LabelFrame(self,text="Controle e Status",padding=6)
        ctrl.grid(row=1,column=0,sticky="ew",padx=6,pady=6)
        for i in range(20):
            ctrl.columnconfigure(i, weight=0)
        ctrl.columnconfigure(19, weight=1)

        self.ed_zero=ttk.Entry(ctrl,width=7); self.ed_zero.insert(0,"1150")
        self.ed_home=ttk.Entry(ctrl,width=7); self.ed_home.insert(0,"1300")
        self.ed_max =ttk.Entry(ctrl,width=7); self.ed_max.insert(0,"1500")
        self.ed_step=ttk.Entry(ctrl,width=7); self.ed_step.insert(0,"10")
        ttk.Label(ctrl,text="ZERO (µs):").grid(row=0,column=0,sticky="e"); self.ed_zero.grid(row=0,column=1,padx=3)
        ttk.Label(ctrl,text="HOME (µs):").grid(row=0,column=2,sticky="e"); self.ed_home.grid(row=0,column=3,padx=3)
        ttk.Label(ctrl,text="MAX (µs):").grid(row=0,column=4,sticky="e");  self.ed_max.grid(row=0,column=5,padx=3)
        ttk.Label(ctrl,text="STEP (µs):").grid(row=0,column=6,sticky="e"); self.ed_step.grid(row=0,column=7,padx=3)
        ttk.Button(ctrl,text="GOTO:HOME",command=lambda:self.send_cmd("GOTO:HOME")).grid(row=0,column=8,padx=3)
        ttk.Button(ctrl,text="GOTO:ZERO",command=lambda:self.send_cmd("GOTO:ZERO")).grid(row=0,column=9,padx=3)
        ttk.Button(ctrl,text="STOP ⛔",command=lambda:self.send_cmd("STOP")).grid(row=0,column=10,padx=3)

        self.ed_move=ttk.Entry(ctrl,width=7); self.ed_move.insert(0,"1400")
        ttk.Label(ctrl,text="MOVE (µs):").grid(row=1,column=0,sticky="e"); self.ed_move.grid(row=1,column=1,padx=3)
        ttk.Button(ctrl,text="Enviar MOVE",command=self.on_move).grid(row=1,column=2,padx=3)
        ttk.Button(ctrl,text="🔼 Subir",command=self.on_up).grid(row=1,column=3,padx=3)
        ttk.Button(ctrl,text="🔽 Descer",command=self.on_down).grid(row=1,column=4,padx=3)

        ttk.Separator(ctrl,orient="vertical").grid(row=0,column=11,rowspan=2,sticky="ns",padx=6)
        ttk.Label(ctrl,text="Alvo Força (N):").grid(row=0,column=12,sticky="e")
        self.ed_f_target=ttk.Entry(ctrl,width=8); self.ed_f_target.insert(0,"10.0"); self.ed_f_target.grid(row=0,column=13,padx=3)
        ttk.Label(ctrl,text="Kp (µs/N):").grid(row=0,column=14,sticky="e")
        self.ed_kp=ttk.Entry(ctrl,width=8); self.ed_kp.insert(0,"1.0"); self.ed_kp.grid(row=0,column=15,padx=3)
        self.btn_hold=ttk.Button(ctrl,text="Manter Força: OFF",command=self.on_toggle_hold); self.btn_hold.grid(row=0,column=16,padx=6)

        ttk.Label(ctrl,text="Força (N):").grid(row=1,column=12,sticky="e")
        self.lbl_force=ttk.Label(ctrl,text="--"); self.lbl_force.grid(row=1,column=13,sticky="w")
        ttk.Label(ctrl,text="Pressão (MPa):").grid(row=1,column=14,sticky="e")
        self.lbl_press=ttk.Label(ctrl,text="--"); self.lbl_press.grid(row=1,column=15,sticky="w")
        ttk.Label(ctrl,text="R_sensor:").grid(row=1,column=16,sticky="e")
        self.lbl_r=ttk.Label(ctrl,text="--"); self.lbl_r.grid(row=1,column=17,sticky="w")

        self.notebook = ttk.Notebook(self)
        self.notebook.grid(row=2, column=0, sticky="nsew", padx=6)
        self.notebook.bind("<<NotebookTabChanged>>", self._on_tab_changed)

        module_infos = [
            ("linearity", "Linearity",
             "Linearity Test – 5 steps up @ 20 / 40 / 60 / 80 / 100% da carga."),
            ("lin_hyst", "Linearity + Hysteresis",
             "Linearity & Hysteresis Test – subida 20→100% e descida 80→20%."),
            ("drift", "Drift",
             "Drift Test – carga estática em ~50% por 60 segundos."),
            ("repeat", "Repeatability",
             "Repeatability Test – repete o teste de Linearity & Hysteresis N vezes."),
        ]

        for module_name, tab_title, desc in module_infos:
            tab = ttk.Frame(self.notebook)
            self.notebook.add(tab, text=tab_title)
            tab_id = self.notebook.tabs()[-1]
            self._tab_module[tab_id] = module_name
            self._build_module_tab(tab, module_name, desc)

        logbar = ttk.Frame(self)
        logbar.grid(row=3, column=0, sticky="ew", padx=6, pady=(4,0))
        self.btn_logtoggle = ttk.Button(logbar, text="Esconder LOG", command=self.on_toggle_log)
        self.btn_logtoggle.pack(side="right")

        log = ttk.LabelFrame(self, text="Log", padding=6)
        log.grid(row=4, column=0, sticky="nsew", padx=6, pady=(0,6))
        log.rowconfigure(0, weight=1)
        log.columnconfigure(0, weight=1)
        self.log_frame = log

        btns = ttk.Frame(log)
        btns.grid(row=1, column=0, sticky="e", pady=(6,0))
        ttk.Button(btns,text="Copiar Log",command=self.on_copy_log).pack(side="left",padx=(0,6))
        ttk.Button(btns,text="Limpar Log",command=self.on_clear_log).pack(side="left")

        self.txt_log = tk.Text(log, height=6)
        self.txt_log.grid(row=0, column=0, sticky="nsew")

        self.columnconfigure(0, weight=1)
        self.rowconfigure(2, weight=1)

        self.status = ttk.Label(self, text="")
        self.status.grid(row=5, column=0, sticky="w", padx=6)

    def _build_module_tab(self, parent, module_name, desc):
        parent.columnconfigure(0, weight=1)
        parent.rowconfigure(1, weight=1)

        header = ttk.Frame(parent, padding=(4,4))
        header.grid(row=0, column=0, sticky="ew")
        header.columnconfigure(0, weight=1)

        ttk.Label(header, text=desc, wraplength=800, justify="left").grid(
            row=0, column=0, sticky="w"
        )

        if module_name == "repeat":
            ttk.Label(header, text="Ciclos:").grid(row=0, column=1, sticky="e", padx=(10,2))
            spn = ttk.Spinbox(header, from_=1, to=99, width=4, textvariable=self.repeat_cycles_var)
            spn.grid(row=0, column=2, sticky="w")

            col_btn_start = 3
        else:
            col_btn_start = 1

        ttk.Button(
            header,
            text="Iniciar teste",
            command=lambda m=module_name: self.start_test(m)
        ).grid(row=0, column=col_btn_start, padx=8)
        ttk.Button(
            header,
            text="Limpar gráfico",
            command=lambda m=module_name: self.clear_graph(m)
        ).grid(row=0, column=col_btn_start + 1, padx=4)

        plots = ttk.Frame(parent)
        plots.grid(row=1, column=0, sticky="nsew")
        plots.columnconfigure(0, weight=1)
        plots.columnconfigure(1, weight=1)
        plots.rowconfigure(0, weight=1)

        def mkfig(parent_frame):
            fig = Figure(figsize=(6,5), dpi=100)
            ax = fig.add_subplot(111)
            canvas = FigureCanvasTkAgg(fig, master=parent_frame)
            canvas.get_tk_widget().grid(row=0, column=0, sticky="nsew")
            return fig, ax, canvas

        left = ttk.Frame(plots)
        left.grid(row=0, column=0, sticky="nsew", padx=(0,3))
        left.columnconfigure(0, weight=1)
        left.rowconfigure(0, weight=1)

        right = ttk.Frame(plots)
        right.grid(row=0, column=1, sticky="nsew", padx=(3,0))
        right.columnconfigure(0, weight=1)
        right.rowconfigure(0, weight=1)

        fig_rt, ax_rt, canvas_rt = mkfig(left)
        ax_rt.set_xlabel("Tempo (s)")
        ax_rt.set_ylabel("R_sensor (Ω)")
        ax_rt.grid(True, alpha=0.3)

        fig_pr, ax_pr, canvas_pr = mkfig(right)
        ax_pr.set_xlabel("Pressão (MPa)")
        ax_pr.set_ylabel("R_sensor (Ω)")
        ax_pr.grid(True, alpha=0.3)

        self.plots[module_name] = {
            "fig_rt": fig_rt, "ax_rt": ax_rt, "canvas_rt": canvas_rt,
            "fig_pr": fig_pr, "ax_pr": ax_pr, "canvas_pr": canvas_pr,
        }

    # ======== Aquisição helpers =========
    def _start_acq(self, reason=None):
        if self.acq:
            self.data.reset()
            self.first_ok = False
            if reason:
                self.log(f"Novo teste iniciado {reason}; dados anteriores limpos.")
            else:
                self.log("Novo teste iniciado; dados anteriores limpos.")
            return
        self.acq = True
        self.data.reset()
        self.first_ok = False
        self.btn_acq["text"] = "Parar Aquisição"
        msg = "Aquisição iniciada."
        if reason:
            msg += f" {reason}"
        self.log(msg)

    def _stop_acq(self):
        if not self.acq:
            return
        self.acq = False
        self.btn_acq["text"] = "Iniciar Aquisição"
        self.log("Aquisição pausada.")

    # ---------- Top actions ----------
    def refresh_ports(self):
        self.cb_port["values"] = list_serial_ports()

    def on_connect(self):
        port = self.cb_port.get().strip()
        if not port:
            return messagebox.showwarning("Atenção","Selecione uma porta.")
        ok, msg = self.serial.connect(port)
        self.log(msg)
        if ok:
            self.btn_connect["state"]="disabled"
            self.btn_disconnect["state"]="normal"
            self.btn_acq["state"]="normal"
        self._update_title()

    def on_disconnect(self):
        self._stop_acq()
        self.serial.disconnect()
        self.btn_connect["state"]="normal"
        self.btn_disconnect["state"]="disabled"
        self.btn_acq["state"]="disabled"
        self._update_title()
        self.log("Desconectado.")

    def on_toggle_scale(self):
        self.logscale = not self.logscale
        self.btn_scale.config(text="Escala: Log10" if self.logscale else "Escala: Linear")
        self._refresh_plots_and_status()

    def on_toggle_acq(self):
        if self.acq:
            self._stop_acq()
        else:
            self._start_acq()

    def on_preset(self, *_):
        preset = self.cb_sensor.get()
        if preset in SENSOR_PRESETS:
            self.ed_area.delete(0,"end")
            self.ed_area.insert(0,str(SENSOR_PRESETS[preset]["area_mm2"]))
            self.log(f"Preset '{preset}' aplicado: área {SENSOR_PRESETS[preset]['area_mm2']} mm²")

    def on_calibrate(self):
        """Ajusta Rfactor para que R_sensor bata com o multímetro.

        Usa a mediana das últimas leituras de vd (ADC). Se o usuário
        informar R_medido (Ω) no campo superior, calculamos:

            Rfactor = R_medido / median(vd)

        Assim, R_sensor ≈ Rfactor * vd.
        """
        Rm = safe_float(self.ed_rmeas.get(), np.nan)
        if not np.isfinite(Rm):
            return messagebox.showwarning("Calibrar","Preencha R_medido (Ω) medido no multímetro.")
        vals = np.array([v for v in self.data.vd if np.isfinite(v)])
        if vals.size == 0:
            return messagebox.showwarning("Calibrar","Colete alguns pontos antes de calibrar.")
        vd_med = float(np.median(vals[-20:]))
        if vd_med <= 0:
            return messagebox.showwarning("Calibrar","Valor interno inválido.")
        F = Rm / vd_med
        self.ed_rfact.delete(0,"end")
        self.ed_rfact.insert(0,f"{F:.6g}")
        self.log(f"Fator R ajustado automaticamente: {F:.6g}")

    def on_move(self):
        move = safe_int(self.ed_move.get(), None)
        if move is None:
            return messagebox.showwarning("MOVE inválido","Informe um valor numérico.")
        move = self._clamp_move(move)
        self._last_move = move
        self.ed_move.delete(0,"end")
        self.ed_move.insert(0,str(move))
        self.send_cmd(f"MOVE:{move}")

    def on_up(self):
        step = safe_int(self.ed_step.get(), None)
        if step is None or step<=0:
            return messagebox.showwarning("STEP inválido","Informe STEP>0.")
        curr = safe_int(self.ed_move.get(), None)
        if curr is None:
            curr = safe_int(self.ed_home.get(), 1300)
        newv = self._clamp_move(curr + step)
        self._last_move = newv
        self.ed_move.delete(0,"end")
        self.ed_move.insert(0,str(newv))
        self.send_cmd(f"MOVE:{newv}")

    def on_down(self):
        step = safe_int(self.ed_step.get(), None)
        if step is None or step<=0:
            return messagebox.showwarning("STEP inválido","Informe STEP>0.")
        curr = safe_int(self.ed_move.get(), None)
        if curr is None:
            curr = safe_int(self.ed_home.get(), 1300)
        newv = self._clamp_move(curr - step)
        self._last_move = newv
        self.ed_move.delete(0,"end")
        self.ed_move.insert(0,str(newv))
        self.send_cmd(f"MOVE:{newv}")

    def on_toggle_hold(self):
        self.hold_force = not self.hold_force
        self.btn_hold.config(text=f"Manter Força: {'ON' if self.hold_force else 'OFF'}")
        if self.hold_force:
            self.target_force_N = safe_float(self.ed_f_target.get(), 0.0)
            self.Kp = safe_float(self.ed_kp.get(), 1.0)
            self.log(f"Manter força ATIVADO | Alvo={self.target_force_N} N | Kp={self.Kp} µs/N")
        else:
            self.log("Manter força DESATIVADO")

    def on_toggle_log(self):
        if self.log_frame.winfo_viewable():
            self.log_frame.grid_remove()
            self.btn_logtoggle.config(text="Mostrar LOG")
        else:
            self.log_frame.grid()
            self.btn_logtoggle.config(text="Esconder LOG")

    def _on_tab_changed(self, event):
        tab_id = self.notebook.select()
        module = self._tab_module.get(tab_id)
        if module:
            self.current_module = module
            # Aponta self.data para o buffer de dados deste módulo
            if module in self.data_by_module:
                self.data = self.data_by_module[module]
            if hasattr(self, "txt_log"):
                self.log(f"Módulo ativo: {module}")
            # Atualiza o gráfico com os dados já existentes da aba selecionada
            self._refresh_plots_and_status()

    def start_test(self, module_name: str):
        self.current_module = module_name
        for tab_id, mod in self._tab_module.items():
            if mod == module_name:
                self.notebook.select(tab_id)
                break
        # Aponta self.data para o módulo escolhido e limpa apenas seus dados
        if module_name in self.data_by_module:
            self.data = self.data_by_module[module_name]
        self.clear_graph(module_name)
        self._start_acq(reason=f"(módulo {module_name})")
        self._run_test_sequence(module_name)

    def _run_test_sequence(self, module_name: str):
        if not self.serial.is_connected():
            self.log("Não é possível iniciar o teste: não conectado.")
            return

        if module_name == "repeat":
            n = self.repeat_cycles_var.get()
            if n < 1:
                n = 1
            if n > 99:
                n = 99
            self.repeat_cycles_var.set(n)
            self.log(f"Ajustando número de ciclos de repetição para {n}.")
            self.send_cmd(f"SET:REPEAT:{n}")

        cmd_map = {
            "linearity": "TEST:LINEARITY",
            "lin_hyst": "TEST:LIN_HYST",
            "drift": "TEST:DRIFT",
            "repeat": "TEST:REPEAT",
        }
        cmd = cmd_map.get(module_name)
        if not cmd:
            self.log(f"Nenhum comando associado ao módulo {module_name}.")
            return

        self.log(f"Enviando comando de teste: {cmd}")
        self.send_cmd(cmd)

    def clear_graph(self, module_name: str | None = None):
        if module_name is None:
            module_name = self.current_module
        # Limpa os dados apenas do módulo especificado
        if module_name in self.data_by_module:
            self.data_by_module[module_name].reset()
        self.data = self.data_by_module.get(module_name, self.data)
        self.lbl_r.config(text="--")
        self.lbl_force.config(text="--")
        self.lbl_press.config(text="--")
        if module_name not in self.plots:
            return
        ax_rt = self.plots[module_name]["ax_rt"]
        canvas_rt = self.plots[module_name]["canvas_rt"]
        ax_pr = self.plots[module_name]["ax_pr"]
        canvas_pr = self.plots[module_name]["canvas_pr"]

        ax_rt.cla()
        ax_rt.grid(True, alpha=0.3)
        ax_rt.set_xlabel("Tempo (s)")
        ax_rt.set_ylabel("R_sensor (Ω)")
        canvas_rt.draw_idle()

        ax_pr.cla()
        ax_pr.grid(True, alpha=0.3)
        ax_pr.set_xlabel("Pressão (MPa)")
        ax_pr.set_ylabel("R_sensor (Ω)")
        canvas_pr.draw_idle()

        self.log(f"Gráficos e dados limpos para módulo {module_name}.")

    def send_cmd(self, c):
        ok, msg = self.serial.send_line(c)
        self.log(msg)

    def _clamp_move(self, val):
        z = safe_int(self.ed_zero.get(), 1150)
        m = safe_int(self.ed_max.get(), 1500)
        if z is None:
            z = 1150
        if m is None:
            m = 1500
        if z > m:
            z, m = m, z
        return max(z, min(m, val))

    def _update_loop(self):
        self.data.n_per_unit = safe_float(self.ed_nper.get(), 1.0)
        self.data.area_mm2   = safe_float(self.ed_area.get(), 100.0)
        self.data.R_top      = safe_float(self.ed_rtop.get(), 22000.0)
        self.data.R_bot      = safe_float(self.ed_rbot.get(), 22000.0)
        self.data.Vref       = safe_float(self.ed_vref.get(), 1023.0)
        self.data.Rfactor    = safe_float(self.ed_rfact.get(), 1.0)
        self.data.mode       = "direto" if self.cb_mode.get().startswith("Direto") else "invertido"

        got=False
        try:
            while True:
                line=self.serial.q.get_nowait()
                if self.acq:
                    self.data.total_lines += 1
                    if self._parse_and_add(line):
                        got=True
                        if not self.first_ok and self.data.valid_lines>=3:
                            self.log("📡 Dados recebidos OK"); self.first_ok=True
                    else:
                        if self.data.valid_lines<5:
                            self.log(f"<<< {line}")
        except Empty:
            pass

        # Atualiza gráficos sempre que houver pelo menos 1 ponto válido
        if self.data.valid_lines>0:
            self._refresh_plots_and_status()

        if self.hold_force and self.acq and self.data.valid_lines>0:
            curr_force = self.data.F[-1]
            if np.isfinite(curr_force):
                self._last_force = curr_force
                err = self.target_force_N - curr_force
                Kp  = safe_float(self.ed_kp.get(), 1.0)
                step_us = int(round(Kp * err))
                if step_us != 0:
                    step_us = max(-20, min(20, step_us))
                    curr = safe_int(self.ed_move.get(), None)
                    if curr is None:
                        curr = safe_int(self.ed_home.get(), 1300)
                    newv = self._clamp_move(curr + step_us)
                    if newv != curr:
                        self._last_move = newv
                        self.ed_move.delete(0,"end")
                        self.ed_move.insert(0,str(newv))
                        self.send_cmd(f"MOVE:{newv}")

        self._update_title()
        self.after(UPDATE_MS, self._update_loop)

    def _parse_and_add(self, line):
        parts=[p for p in line.replace(";",",").replace("\t",",").split(",") if p.strip()!=""]
        if len(parts)<4:
            return False
        nums=[]
        for p in parts:
            v=safe_float(p, None)
            if v is not None and np.isfinite(v):
                nums.append(v)
            if len(nums)==4:
                break
        if len(nums)<4:
            return False
        pos,op,vd,lc=nums[:4]
        self.data.add(pos,op,vd,lc)
        # Loga algumas amostras válidas para depuração
        if self.data.valid_lines <= 5:
            try:
                self.log(f"DATA: pos={pos}, vd={vd}, lc={lc}")
            except Exception:
                pass
        return True

    def _fmt_R(self, arr):
        arr=np.array(arr); mask=np.isfinite(arr)
        if not np.any(mask):
            return arr, "R_sensor (Ω)"
        med=np.nanmedian(arr[mask])
        if med>=9.5e5:
            return arr/1e6, "R_sensor (MΩ)"
        elif med>=9.5e3:
            return arr/1e3, "R_sensor (kΩ)"
        else:
            return arr, "R_sensor (Ω)"

    def _refresh_plots_and_status(self):
        if not self.data.valid_lines:
            return
        t=np.array(self.data.t); R=np.array(self.data.Rs); P=np.array(self.data.P); F=np.array(self.data.F)

        r_last = R[-1] if np.isfinite(R[-1]) else np.nan
        f_last = F[-1] if np.isfinite(F[-1]) else np.nan
        p_last = P[-1] if np.isfinite(P[-1]) else np.nan

        if np.isfinite(r_last):
            if r_last>=9.5e5:
                rtxt=f"{r_last/1e6:.3f} MΩ"
            elif r_last>=9.5e3:
                rtxt=f"{r_last/1e3:.3f} kΩ"
            else:
                rtxt=f"{r_last:.1f} Ω"
        else:
            rtxt="--"
        self.lbl_r.config(text=rtxt)
        self.lbl_force.config(text="--" if not np.isfinite(f_last) else f"{f_last:.3f}")
        self.lbl_press.config(text="--" if not np.isfinite(p_last) else f"{p_last:.5f}")

        Rscaled, Rylabel = self._fmt_R(R)

        module = getattr(self, "current_module", None)
        if not module or module not in self.plots:
            return

        ax_rt = self.plots[module]["ax_rt"]
        canvas_rt = self.plots[module]["canvas_rt"]
        ax_pr = self.plots[module]["ax_pr"]
        canvas_pr = self.plots[module]["canvas_pr"]

        ax_rt.cla()
        ax_rt.grid(True,alpha=0.3)
        ax_rt.set_xlabel("Tempo (s)")
        ax_rt.set_ylabel(Rylabel)
        ax_rt.plot(t, Rscaled, lw=1)
        if self.logscale:
            try:
                ax_rt.set_yscale("log", base=10)
            except Exception:
                ax_rt.set_yscale("log")
        else:
            ax_rt.set_yscale("linear")
        canvas_rt.draw_idle()

        ax_pr.cla()
        ax_pr.grid(True,alpha=0.3)
        ax_pr.set_xlabel("Pressão (MPa)")
        ax_pr.set_ylabel(Rylabel)
        mask=np.isfinite(Rscaled) & np.isfinite(P)
        if np.any(mask):
            ax_pr.scatter(P[mask], Rscaled[mask], s=10, alpha=0.8)
        if self.logscale:
            try:
                ax_pr.set_yscale("log", base=10)
            except Exception:
                ax_pr.set_yscale("log")
        else:
            ax_pr.set_yscale("linear")
        canvas_pr.draw_idle()

    def on_export_csv(self):
        if not self.data.t:
            return messagebox.showinfo("Exportar CSV","Sem dados para exportar.")
        default=f"FlexiForce_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        fp=filedialog.asksaveasfilename(
            title="Salvar CSV",
            defaultextension=".csv",
            initialfile=default,
            filetypes=[("CSV","*.csv")]
        )
        if not fp:
            return
        try:
            with open(fp,"w",newline="",encoding="utf-8") as f:
                w=csv.writer(f)
                for r in self.data.rows():
                    w.writerow(r)
            self.log(f"CSV salvo: {fp}")
        except Exception as e:
            messagebox.showerror("Erro ao salvar",str(e))

    def on_copy_log(self):
        try:
            texto = self.txt_log.get("1.0", "end")
            self.clipboard_clear()
            self.clipboard_append(texto)
            self.log("Log copiado para a área de transferência ✅")
        except Exception as e:
            messagebox.showerror("Erro", f"Falha ao copiar log: {e}")

    def on_clear_log(self):
        try:
            self.txt_log.delete("1.0","end")
        except Exception as e:
            messagebox.showerror("Erro", f"Falha ao limpar log: {e}")

    def log(self, txt):
        ts=datetime.now().strftime("%H:%M:%S")
        self.txt_log.insert("end",f"[{ts}] {txt}\n")
        self.txt_log.see("end")

    def _update_title(self):
        if not self.serial.is_connected():
            self.title(f"🔴 {APP_TITLE}")
        else:
            self.title(f"{'🔵' if self.acq else '🟢'} {APP_TITLE}")

    def _cfg_dict(self):
        return {
            "preset": self.cb_sensor.get(),
            "area_mm2": self.ed_area.get(),
            "n_per_unit": self.ed_nper.get(),
            "R_top": self.ed_rtop.get(),
            "R_bot": self.ed_rbot.get(),
            "mode": self.cb_mode.get(),
            "Vref": self.ed_vref.get(),
            "Rfactor": self.ed_rfact.get(),
            "zero": self.ed_zero.get(),
            "home": self.ed_home.get(),
            "max": self.ed_max.get(),
            "step": self.ed_step.get(),
            "move": self.ed_move.get(),
            "Kp": self.ed_kp.get(),
            "F_target": self.ed_f_target.get(),
        }

    def _apply_cfg(self, d):
        def put(entry, key):
            if key in d and d[key]!="":
                entry.delete(0,"end")
                entry.insert(0,str(d[key]))
        if d.get("preset"):
            self.cb_sensor.set(d["preset"])
            self.on_preset()
        put(self.ed_area,"area_mm2")
        put(self.ed_nper,"n_per_unit")
        put(self.ed_rtop,"R_top")
        put(self.ed_rbot,"R_bot")
        if d.get("mode"):
            self.cb_mode.set(d["mode"])
        put(self.ed_vref,"Vref")
        put(self.ed_rfact,"Rfactor")
        put(self.ed_zero,"zero")
        put(self.ed_home,"home")
        put(self.ed_max,"max")
        put(self.ed_step,"step")
        put(self.ed_move,"move")
        put(self.ed_kp,"Kp")
        put(self.ed_f_target,"F_target")

    def _load_cfg(self):
        try:
            if os.path.exists(CFG_FILE):
                with open(CFG_FILE,"r",encoding="utf-8") as f:
                    self._apply_cfg(json.load(f))
                    self.log("Config carregada.")
        except Exception as e:
            self.log(f"Falha ao carregar config: {e}")

    def _save_cfg(self):
        try:
            with open(CFG_FILE,"w",encoding="utf-8") as f:
                json.dump(self._cfg_dict(), f, ensure_ascii=False, indent=2)
        except Exception as e:
            self.log(f"Falha ao salvar config: {e}")

    def _on_close(self):
        self._save_cfg()
        try:
            self.serial.disconnect()
        except Exception:
            pass
        self.destroy()

def main():
    App().mainloop()

if __name__=="__main__":
    main()