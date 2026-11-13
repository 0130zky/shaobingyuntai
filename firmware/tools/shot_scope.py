#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""三摩擦轮弹速上位机 —— 轮速 / 电流 / 拨盘 / 弹速 实时示波 + 逐发统计

数据源
    板上 USART1（丝印 UART2）经 DAPLink 虚拟串口输出的 JustFloat 帧：
    25 路 float(LE) + 帧尾 00 00 80 7F，默认 460800 8N1。
    通道定义见 Application/module_task.c 的 FrictionScope_Send()。

    0..2  三轮实际转速 rpm（已按发射方向取正）
    3     统一目标转速 rpm（与 0..2 同号）
    4     裁判弹速 m/s（原始值，不替换低弹速）
    5     射击计数（低 16 位，变化 = 新的一发）
    6     收到射击消息的板载时刻 ms（低 16 位）
    7     采样时刻 ms（低 16 位，用来重建精确时基）
    8     距上一帧裁判数据的间隔 ms
    9     发射机构编号
    10..12 三轮电流指令（C620 计数，±16384 = ±20A）
    13..15 三轮转矩电流反馈（同上）
    16..20 拨盘角度 deg / 目标 deg / 速度 dps / 输出 / 电流（后两者归一化）
    21..24 拨盘反馈有效 / 卡弹 / 待进弹计数 / 三轮到速就绪

用法
    tools/shot_scope.sh                          # 打开界面，选端口后点“连接”
    tools/shot_scope.sh --replay xxx.csv         # 直接载入记录做离线分析
    .venv/bin/python tools/shot_scope.py --self-test xxx.csv   # 命令行出统计

判读
    每发以“射击计数变化”为锚点，在 ±0.8s 窗口里找三轮平均转速的最低点，给出
    每轮最低转速、掉速百分比、恢复到 ±100rpm 的时间、峰值电流、进弹时拨盘速度。
    采样时必须用 JustFloat 的 25 通道固件，DAPLink 串口一次只能被一个程序占用，
    用本工具采集时请关闭 VOFA+ 和 capture_friction.py。
"""

import argparse
import csv
import datetime
import math
import json
import os
import queue
import signal
import struct
import sys
import threading
import time
from pathlib import Path

import numpy as np
from analysis_core import measure_shot, analyze_all, mark_duplicates, shot_stats
from board_clock import BoardClock, reconstruct_times
from telemetry_protocol import Decoder
from serial_transport import SerialPort

import tkinter as tk
from tkinter import ttk, filedialog, messagebox

import matplotlib
matplotlib.use('TkAgg')
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

try:
    import serial
    from serial.tools import list_ports
except ImportError:                                     # pragma: no cover
    sys.stderr.write('缺少 pyserial：.venv/bin/pip install pyserial\n')
    raise

# ---------------------------------------------------------------------------
# 协议与标定
# ---------------------------------------------------------------------------
TAIL = b'\x00\x00\x80\x7f'
NCH = 25
DEFAULT_BAUD = 460800
POLL_MS = 100        # 界面节拍：一次重画约 35ms，100ms 一拍既不卡也不烧 CPU

IDX = {
    'rpm': (0, 1, 2),
    'target': 3,
    'speed': 4,
    'count': 5,
    'shot_rx': 6,
    'sample_ms': 7,
    'ref_age': 8,
    'shooter': 9,
    'cmd': (10, 11, 12),
    'cur': (13, 14, 15),
    'dial_angle': 16,
    'dial_target': 17,
    'dial_dps': 18,
    'dial_out': 19,
    'dial_cur': 20,
    'dial_ok': 21,
    'jam': 22,
    'pending': 23,
    'ready': 24,
}

WHEELS = ('12点', '4点', '8点')
WCOLOR = ('#ff7a5c', '#35c4a0', '#4f9cff')

# C620 反馈：±16384 ≈ ±20A（见 Application/booster_task.c 的注释）
AMP_PER_RAW = 20.0 / 16384.0
# DM4310 MIT 帧量程，与 dial_config.h 的 DIAL_DM_T_MAX 一致
DIAL_T_MAX = 10.0

PRE_S = 0.80            # 分析窗口：射击消息之前
POST_S = 0.80           # 之后
BASE_S = 0.30           # 基线取窗口最前面这段时间
DROP_LO = 0.03          # 平均掉速低于 3% 视为“没打到”
RECOV_TOL = 100.0       # 恢复到目标 ±100rpm 算恢复
RECOV_HOLD = 0.050      # 且维持 50ms
DUP_GAP = 0.70          # 疑似重复上报的间隔上限

BG = '#12151b'
PANEL = '#1a1f27'
PANEL2 = '#202733'
BORDER = '#2c3542'
FG = '#dbe3ee'
MUTED = '#8794a5'
GRID = '#242c38'
OK = '#42d392'
WARN = '#ffb454'
BAD = '#ff5f6d'
ACCENT = '#5aa9ff'


# ---------------------------------------------------------------------------
# 数据缓冲
# ---------------------------------------------------------------------------
class Session:
    """按时间顺序追加的采样缓冲区。读线程写、界面线程读，用锁隔开。"""

    MIN_CAP = 16384

    def __init__(self, nch=NCH):
        self.nch = nch
        self._t = np.zeros(self.MIN_CAP, dtype=np.float64)
        self._d = np.zeros((self.MIN_CAP, nch), dtype=np.float32)
        self.n = 0
        self.lock = threading.Lock()

    def append(self, tarr, darr):
        k = len(tarr)
        if k == 0:
            return
        with self.lock:
            keep = np.r_[np.diff(tarr)>0, True]
            if self.n:keep &= tarr > self._t[self.n-1]
            tarr,darr = tarr[keep],darr[keep]
            k = len(tarr)
            if not k:return
            need = self.n + k
            if need > len(self._t):
                cap = len(self._t)
                while cap < need:
                    cap *= 2
                nt = np.zeros(cap, dtype=np.float64)
                nd = np.zeros((cap, self.nch), dtype=np.float32)
                nt[:self.n] = self._t[:self.n]
                nd[:self.n] = self._d[:self.n]
                self._t, self._d = nt, nd
            self._t[self.n:need] = tarr
            self._d[self.n:need] = darr
            self.n = need

    def rows_since(self, n0):
        with self.lock:
            if n0 >= self.n:
                return None, None
            return self._t[n0:self.n].copy(), self._d[n0:self.n].copy()

    def window(self, t0, t1):
        with self.lock:
            i0 = int(np.searchsorted(self._t[:self.n], t0))
            i1 = int(np.searchsorted(self._t[:self.n], t1, side='right'))
            return self._t[i0:i1].copy(), self._d[i0:i1].copy()

    def clear(self):
        with self.lock:
            self.n = 0


# ---------------------------------------------------------------------------
# 串口读取
# ---------------------------------------------------------------------------
class Reader(threading.Thread):
    """独立线程：读串口 → 解 JustFloat → 重建时基 → 追加进 Session。"""

    def __init__(self, port, baud, session, on_status):
        super().__init__(daemon=True)
        self.port = port
        self.baud = baud
        self.session = session
        self.on_status = on_status          # (level, text)
        self.stop_evt = threading.Event()
        self.frames = 0
        self.errors = 0
        self.fps = 0.0
        self.nch = None
        self._t = 0.0                       # 板载时基，秒
        self._prev_ms = None
        self._clock = BoardClock()

    def stop(self):
        self.stop_evt.set()

    # -- 时间基：用第 7 通道的 HAL_GetTick 低 16 位重建，避免主机投递抖动 --
    def _board_times(self, ms):
        return np.asarray([self._clock.advance(value) for value in ms], dtype=np.float64)

    def run(self):
        try:
            port = SerialPort(self.port, self.baud)
        except Exception as exc:
            self.on_status('error', str(exc));return
        self.nch = 25
        decoder = Decoder()
        self.on_status('ok', '已连接 %s @ %d' % (self.port, self.baud))
        start = time.monotonic()
        try:
            while not self.stop_evt.is_set():
                for values in decoder.feed(port.read(timeout=.05)):
                    stamp = self._clock.advance(values[7])
                    if not self._clock.changed:continue
                    self.session.append(np.asarray([stamp]), np.asarray([values], dtype=np.float32))
                    self.frames += 1
                self.errors = decoder.invalid
                self.fps = self.frames/max(.001,time.monotonic()-start)
        except Exception as exc:
            if not self.stop_evt.is_set():self.on_status('error', str(exc))
        finally:
            port.close()


class MonitorReader(Reader):
    """Follow the collector's CSV, keeping the serial port owned by the collector."""
    def run(self):
        fp = None
        current = None
        self.nch = NCH
        try:
            while not self.stop_evt.is_set():
                try:
                    status = json.loads(Path(self.port).read_text())
                    path = status.get('csv_file')
                    if path and path != current:
                        if fp:
                            fp.close()
                        fp = open(path, newline='')
                        fp.readline()  # header; preserve tell/seek for partial rows
                        current = path
                        self.on_status('ok', '后台数据共享：' + Path(path).name)
                    rows = []
                    if fp:
                        for _ in range(4096):
                            offset = fp.tell()
                            line = fp.readline()
                            if not line or not line.endswith('\n'):
                                fp.seek(offset)
                                break
                            values = next(csv.reader([line]))
                            if len(values) == NCH + 2:
                                rows.append([float(v) for v in values[2:]])
                    if rows:
                        data = np.asarray(rows, dtype=np.float32)
                        self.session.append(self._board_times(data[:, IDX['sample_ms']]), data)
                        self.frames += len(rows)
                    elif not status.get('connected'):
                        self.on_status('error', '后台采集等待串口连接')
                except (OSError, ValueError, KeyError) as exc:
                    self.on_status('error', '等待后台采集：' + str(exc))
                self.stop_evt.wait(0.1)
        finally:
            if fp:
                fp.close()


def detect_channels(buf):
    """从连续两帧的帧尾间距推断通道数（10 / 24 / 25）。"""
    if len(buf) < 256:
        return None
    last = -1
    while True:
        ti = buf.find(TAIL, last + 1)
        if ti < 0:
            return None
        if last >= 0:
            d = ti - last
            if d % 4 == 0:
                n = d // 4 - 1
                if n in (10, 24, 25):
                    # 再验证下一帧也对得上
                    nxt = buf.find(TAIL, ti + 4)
                    if nxt < 0 or nxt - ti == d:
                        return n
        last = ti


def plausible(vals, nch):
    """过滤假帧尾/半帧：数字必须有限且落在各通道的物理量程里。"""
    if not all(math.isfinite(v) for v in vals):
        return False
    if any(abs(v) > 1.0e6 for v in vals):
        return False
    rpm = vals[0:4]
    if any(abs(v) > 30000.0 for v in rpm):
        return False
    if not -1.0 <= vals[IDX['speed']] <= 1000.0:
        return False
    if any(not 0.0 <= vals[i] <= 65535.0 for i in (5, 6, 7)):
        return False
    if not 0.0 <= vals[IDX['ref_age']] <= 65535.0:
        return False
    if nch >= 24:
        if any(abs(v) > 40000.0 for v in vals[10:16]):
            return False
        if vals[IDX['dial_ok']] not in (0.0, 1.0):
            return False
    if nch >= 25 and vals[IDX['ready']] not in (0.0, 1.0):
        return False
    return True


# ---------------------------------------------------------------------------
# 逐发分析
# ---------------------------------------------------------------------------
# ---------------------------------------------------------------------------
# 界面
# ---------------------------------------------------------------------------
class App:
    WINDOW_S = 10.0

    def __init__(self, root, port=None, replay=None):
        self.root = root
        self.session = Session()
        self.reader = None
        self.port = port
        self.target_speed = 16.45
        self.tol = 0.05

        self.seen = 0
        self.prev_count = None
        self.pending = []       # 已收到消息、还没分析完的事件
        self.shots = []
        self.markers = []
        self.paused = False
        self._last_t = 0.0
        self.t_min = None
        self.follow = True      # 曲线是否跟随最新数据（拖时间轴后暂停跟随）
        self.view_end = 0.0
        self.replaying = False

        self.record_fp = None
        self.record_writer = None
        self.record_path = None
        self._record_rows = 0
        self._last_flush = 0.0
        self.status_q = queue.Queue()
        self._draw_key = None    # 上次画图时的视图指纹，没变就不重画
        self._picked = None      # 用户当前点中的那一发（表格行号）
        self._quit = False       # Ctrl+C / SIGTERM 时置位，下一拍干净退出

        self._build_style()
        self._build_ui()
        self._poll()
        if replay:
            self.load_csv(replay)
        elif port:
            self.connect(port)

    # ---------------- 外观 ----------------
    def _build_style(self):
        st = ttk.Style()
        try:
            st.theme_use('clam')
        except tk.TclError:
            pass
        st.configure('.', background=BG, foreground=FG, fieldbackground=PANEL,
                     bordercolor=BORDER, lightcolor=PANEL, darkcolor=PANEL,
                     focuscolor=ACCENT, selectbackground='#2b4a6f',
                     selectforeground=FG)
        st.configure('TFrame', background=BG)
        st.configure('Card.TFrame', background=PANEL, relief='flat')
        st.configure('TLabel', background=BG, foreground=FG)
        st.configure('Card.TLabel', background=PANEL, foreground=FG)
        st.configure('Muted.TLabel', background=PANEL, foreground=MUTED,
                     font=('PingFang SC', 10))
        st.configure('Title.TLabel', background=BG, foreground=FG,
                     font=('PingFang SC', 13, 'bold'))
        st.configure('Big.TLabel', background=PANEL, foreground=FG,
                     font=('Menlo', 19, 'bold'))
        st.configure('TButton', background=PANEL2, foreground=FG,
                     padding=(10, 4), borderwidth=0)
        st.map('TButton', background=[('active', '#2b3444')])
        st.configure('Accent.TButton', background='#24507d', foreground=FG)
        st.map('Accent.TButton', background=[('active', '#2f6ba8')])
        st.configure('TCombobox', fieldbackground=PANEL2, background=PANEL2,
                     foreground=FG, arrowcolor=FG)
        st.configure('TEntry', fieldbackground=PANEL2, foreground=FG)
        st.configure('TSpinbox', fieldbackground=PANEL2, foreground=FG)
        st.configure('TNotebook', background=BG, borderwidth=0)
        st.configure('TNotebook.Tab', background=PANEL, foreground=MUTED,
                     padding=(16, 7))
        st.map('TNotebook.Tab',
               background=[('selected', PANEL2)], foreground=[('selected', FG)])
        st.configure('Treeview', background=PANEL, fieldbackground=PANEL,
                     foreground=FG, rowheight=23, borderwidth=0)
        st.configure('Treeview.Heading', background=PANEL2, foreground=MUTED,
                     relief='flat')
        st.map('Treeview', background=[('selected', '#2b4a6f')])
        st.configure('TCheckbutton', background=BG, foreground=FG)

        matplotlib.rcParams.update({
            'figure.facecolor': BG, 'savefig.facecolor': BG,
            'axes.facecolor': PANEL, 'axes.edgecolor': BORDER,
            'axes.labelcolor': MUTED, 'axes.titlecolor': FG,
            'text.color': FG, 'xtick.color': MUTED, 'ytick.color': MUTED,
            'grid.color': GRID, 'grid.linewidth': 0.7,
            'legend.facecolor': PANEL2, 'legend.edgecolor': BORDER,
            'legend.labelcolor': FG,
            'font.sans-serif': ['PingFang SC', 'Heiti SC', 'Arial Unicode MS',
                                'DejaVu Sans'],
            'axes.unicode_minus': False,
            'font.size': 9,
        })

    # ---------------- 布局 ----------------
    def _build_ui(self):
        self.root.title('三摩擦轮弹速上位机')
        self.root.configure(bg=BG)
        sw, sh = self.root.winfo_screenwidth(), self.root.winfo_screenheight()
        self.root.geometry('%dx%d' % (min(1460, sw - 40), min(920, sh - 110)))
        self.root.minsize(1120, 680)

        bar = ttk.Frame(self.root, padding=(10, 8, 10, 6))
        bar.pack(side='top', fill='x')
        ttk.Label(bar, text='弹速上位机', style='Title.TLabel').pack(side='left')
        ttk.Label(bar, text='  端口', style='Muted.TLabel',
                  background=BG).pack(side='left', padx=(10, 4))
        self.port_var = tk.StringVar(value=self.port or '')
        self.cmb_port = ttk.Combobox(bar, textvariable=self.port_var, width=30)
        self.cmb_port.pack(side='left')
        ttk.Button(bar, text='刷新', command=self.refresh_ports).pack(side='left', padx=4)
        self.baud_var = tk.StringVar(value=str(DEFAULT_BAUD))
        ttk.Combobox(bar, textvariable=self.baud_var, width=7,
                     values=('115200', '230400', '460800', '921600')).pack(
            side='left', padx=(2, 6))
        self.btn_conn = ttk.Button(bar, text='连接', style='Accent.TButton',
                                   command=self.toggle_connect)
        self.btn_conn.pack(side='left')

        ttk.Label(bar, text='窗口', foreground=MUTED).pack(side='left', padx=(16, 2))
        self.spin_window = ttk.Spinbox(bar, from_=2, to=120, width=5,
                                       command=self.apply_window)
        self.spin_window.set(str(int(self.WINDOW_S)))
        self.spin_window.pack(side='left')
        self.spin_window.bind('<Return>', self.apply_window)
        self.spin_window.bind('<FocusOut>', self.apply_window)
        ttk.Label(bar, text='s', foreground=MUTED).pack(side='left')

        self.btn_rec = ttk.Button(bar, text='开始记录', command=self.toggle_record)
        self.btn_rec.pack(side='right', padx=4)
        ttk.Button(bar, text='打开记录', command=self.pick_csv).pack(side='right', padx=4)
        self.btn_pause = ttk.Button(bar, text='暂停曲线', command=self.toggle_pause)
        self.btn_pause.pack(side='right', padx=4)

        phase_bar = ttk.Frame(self.root, padding=(10, 0, 10, 6))
        phase_bar.pack(side='top', fill='x')
        ttk.Label(phase_bar, text='拨盘初始偏移（顺时针为正）').pack(side='left')
        self.phase_var = tk.StringVar(value='2.00')
        ttk.Spinbox(phase_bar, from_=-10, to=10, increment=0.1,
                    textvariable=self.phase_var, width=7).pack(side='left', padx=6)
        ttk.Label(phase_bar, text='°').pack(side='left')
        ttk.Button(phase_bar, text='应用初始偏移', command=self.apply_dial_phase).pack(side='left', padx=6)
        ttk.Label(phase_bar, text='当前槽位角度（顺时针+）', foreground=MUTED).pack(side='left', padx=(8, 2))
        self.phase_angle_var = tk.StringVar(value='—')
        ttk.Label(phase_bar, textvariable=self.phase_angle_var,
                  font=('Menlo', 11), width=9).pack(side='left')
        self.phase_status = ttk.Label(phase_bar, text='先关闭CH6；应用后重新使能。输入为顺时针偏移，范围±10°。', foreground=MUTED)
        self.phase_status.pack(side='left', padx=(8, 0))

        body = ttk.Frame(self.root, padding=(10, 0, 10, 0))
        body.pack(side='top', fill='both', expand=True)
        body.columnconfigure(0, weight=0, minsize=500)
        body.columnconfigure(1, weight=1)
        body.rowconfigure(0, weight=1)

        left = ttk.Frame(body)
        left.grid(row=0, column=0, sticky='nsew', padx=(0, 10))
        left.rowconfigure(2, weight=1)
        left.columnconfigure(0, weight=1)

        self._build_stats(left)
        self._build_readout(left)
        self._build_table(left)

        right = ttk.Frame(body)
        right.grid(row=0, column=1, sticky='nsew')
        right.rowconfigure(0, weight=1)
        right.columnconfigure(0, weight=1)
        self.nb = ttk.Notebook(right)
        self.nb.grid(row=0, column=0, sticky='nsew')
        self.nb.bind('<<NotebookTabChanged>>', lambda _e: self.draw_all(force=True))
        self._build_tab_wave()
        self._build_tab_dial()
        self._build_tab_compare()
        self._build_tab_shot()
        self._build_tab_raw()

        status = ttk.Frame(self.root, padding=(12, 4, 12, 6))
        status.pack(side='bottom', fill='x')
        self.lbl_conn = ttk.Label(status, text='就绪', foreground=MUTED, width=26,
                                  anchor='w')
        self.lbl_conn.pack(side='left')
        self.lbl_right = ttk.Label(status, text='', foreground=MUTED)
        self.lbl_right.pack(side='right')
        self.btn_follow = ttk.Button(status, text='跟随', width=6,
                                     command=self.enable_follow)
        self.btn_follow.pack(side='right', padx=6)
        self.timeline = ttk.Scale(status, from_=0.0, to=1.0, orient='horizontal',
                                  command=self._on_timeline)
        self.timeline.pack(side='right', fill='x', expand=True, padx=10)

    def _card(self, parent, title):
        fr = ttk.Frame(parent, style='Card.TFrame', padding=10)
        fr.pack(fill='x', pady=(0, 8))
        head = ttk.Frame(fr, style='Card.TFrame')
        head.pack(fill='x')
        ttk.Label(head, text=title, style='Muted.TLabel').pack(side='left')
        fr.head = head
        return fr

    def _build_stats(self, parent):
        fr = self._card(parent, '本次统计')
        # 目标 / 容差 放在统计卡标题行：既贴近判定结果，也给顶部工具条省地方
        head = fr.head
        self.entry_tol = ttk.Entry(head, width=5)
        self.entry_tol.insert(0, '%.2f' % self.tol)
        self.entry_tol.pack(side='right')
        ttk.Label(head, text='±', style='Muted.TLabel').pack(side='right', padx=4)
        self.entry_target = ttk.Entry(head, width=6)
        self.entry_target.insert(0, '%.2f' % self.target_speed)
        self.entry_target.pack(side='right')
        ttk.Label(head, text='目标', style='Muted.TLabel').pack(side='right',
                                                               padx=(0, 2))
        for e in (self.entry_target, self.entry_tol):
            e.bind('<Return>', self.apply_target)
            e.bind('<FocusOut>', self.apply_target)
        grid = ttk.Frame(fr, style='Card.TFrame')
        grid.pack(fill='x', pady=(4, 0))
        self.stat_vars = {}
        items = [('n', '发数'), ('mean', '均值'), ('std', '标准差'),
                 ('range', '极差'), ('hit', '命中'), ('bias', '偏差')]
        for col, (key, name) in enumerate(items):
            cell = ttk.Frame(grid, style='Card.TFrame')
            cell.grid(row=0, column=col, sticky='w', padx=(0, 16))
            ttk.Label(cell, text=name, style='Muted.TLabel').pack(anchor='w')
            var = tk.StringVar(value='—')
            ttk.Label(cell, textvariable=var, style='Big.TLabel',
                      foreground=FG).pack(anchor='w')
            self.stat_vars[key] = var
        self.lbl_verdict = ttk.Label(fr, text='', style='Card.TLabel',
                                     font=('PingFang SC', 11))
        self.lbl_verdict.pack(anchor='w', pady=(6, 0))

    def _build_readout(self, parent):
        fr = self._card(parent, '实时读数')
        self.ro_vars = {}
        head = ttk.Frame(fr, style='Card.TFrame')
        head.pack(fill='x')
        for c, txt in enumerate(('', '转速', '指令', '电流')):
            ttk.Label(head, text=txt, style='Muted.TLabel', width=9 if c else 6
                      ).grid(row=0, column=c, sticky='w')
        for w, name in enumerate(WHEELS):
            row = ttk.Frame(fr, style='Card.TFrame')
            row.pack(fill='x')
            ttk.Label(row, text=name, style='Card.TLabel', width=6,
                      foreground=WCOLOR[w]).grid(row=0, column=0, sticky='w')
            for c, key in enumerate(('rpm', 'cmd', 'amp'), start=1):
                var = tk.StringVar(value='—')
                ttk.Label(row, textvariable=var, style='Card.TLabel', width=9,
                          font=('Menlo', 12)).grid(row=0, column=c, sticky='w')
                self.ro_vars[(w, key)] = var
        line = ttk.Frame(fr, style='Card.TFrame')
        line.pack(fill='x', pady=(6, 0))
        for key, name in (('target', '目标'), ('speed', '弹速'),
                          ('dial', '拨盘'), ('ready', '就绪')):
            ttk.Label(line, text=name, style='Muted.TLabel',
                      foreground=MUTED).pack(side='left', padx=(0, 2))
            var = tk.StringVar(value='—')
            ttk.Label(line, textvariable=var, style='Card.TLabel',
                      font=('Menlo', 11)).pack(side='left', padx=(0, 12))
            self.ro_vars[('extra', key)] = var

    def _build_table(self, parent):
        fr = ttk.Frame(parent)
        fr.pack(fill='both', expand=True)
        title = ttk.Frame(fr)
        title.pack(fill='x', pady=(2, 4))
        ttk.Label(title, text='发弹记录', style='Muted.TLabel',
                  background=BG).pack(side='left')
        ttk.Button(title, text='清空', command=self.clear_all).pack(side='right')
        ttk.Button(title, text='导出统计', command=self.export_shots).pack(
            side='right', padx=4)

        cols = ('n', 't', 'spd', 'd', 'w0', 'w1', 'w2', 'rec', 'amp', 'flag')
        heads = ('#', '时刻', '弹速', 'Δ', '12点最低', '4点最低', '8点最低', '恢复', '峰值', '判定')
        widths = (28, 46, 58, 44, 48, 48, 48, 46, 44, 58)
        wrap = ttk.Frame(fr)
        wrap.pack(fill='both', expand=True)
        self.tree = ttk.Treeview(wrap, columns=cols, show='headings',
                                 height=12, selectmode='browse')
        for c, h, wdt in zip(cols, heads, widths):
            self.tree.heading(c, text=h)
            self.tree.column(c, width=wdt, anchor='center',
                             stretch=(c == 'flag'))
        sb = ttk.Scrollbar(wrap, orient='vertical', command=self.tree.yview)
        self.tree.configure(yscrollcommand=sb.set)
        self.tree.pack(side='left', fill='both', expand=True)
        sb.pack(side='right', fill='y')
        self.tree.tag_configure('ok', foreground=OK)
        self.tree.tag_configure('hi', foreground=WARN)
        self.tree.tag_configure('lo', foreground=BAD)
        self.tree.tag_configure('dup', foreground=MUTED)
        self.tree.bind('<<TreeviewSelect>>', lambda _e: self._on_pick_shot())

    # ---------------- 图 ----------------
    def _new_fig(self, nrows=2, ratios=None):
        fig = Figure(figsize=(9, 6), dpi=100)
        gs = fig.add_gridspec(nrows, 1, hspace=0.12, left=0.055, right=0.99,
                              top=0.97, bottom=0.08,
                              height_ratios=ratios or [2] + [1] * (nrows - 1))
        axes = []
        for r in range(nrows):
            ax = fig.add_subplot(gs[r], sharex=axes[0] if axes else None)
            ax.grid(True, alpha=0.6)
            axes.append(ax)
        return fig, axes

    def _canvas(self, fig, parent):
        cv = FigureCanvasTkAgg(fig, master=parent)
        cv.get_tk_widget().configure(bg=BG, highlightthickness=0)
        cv.get_tk_widget().pack(fill='both', expand=True)
        return cv

    def _build_tab_wave(self):
        fr = ttk.Frame(self.nb)
        self.nb.add(fr, text='轮速 · 电流')
        fig, (ax1, ax2) = self._new_fig(2, (5, 3))
        ax1.set_ylabel('rpm')
        ax2.set_ylabel('A')
        ax2.set_xlabel('t / s')
        ax1.set_ylim(3500, 6200)
        self.fig_wave = fig
        self.ax_rpm, self.ax_amp = ax1, ax2
        self.ln_rpm = [ax1.plot([], [], color=WCOLOR[w], lw=1.3,
                                label=WHEELS[w])[0] for w in range(3)]
        self.ln_target = ax1.plot([], [], color='#cfd8e3', lw=0.9, ls='--',
                                  label='目标')[0]
        self.ln_cmd = [ax2.plot([], [], color=WCOLOR[w], lw=0.8, ls='--',
                                alpha=0.55)[0] for w in range(3)]
        self.ln_amp = [ax2.plot([], [], color=WCOLOR[w], lw=1.2,
                                label=WHEELS[w])[0] for w in range(3)]
        ax1.legend(loc='lower right', ncol=4, fontsize=8, framealpha=0.9)
        ax2.legend(loc='upper right', ncol=3, fontsize=8, framealpha=0.9)
        self.cv_wave = self._canvas(fig, fr)

    def _build_tab_dial(self):
        fr = ttk.Frame(self.nb)
        self.nb.add(fr, text='拨盘')
        fig, (a1, a2, a3) = self._new_fig(3, (3, 2, 2))
        a1.set_ylabel('deg（顺时针为正）')
        a2.set_ylabel('dps')
        a3.set_ylabel('N·m')
        a3.set_xlabel('t / s')
        self.fig_dial = fig
        self.ax_dial = (a1, a2, a3)
        self.ln_angle = a1.plot([], [], color='#b07cff', lw=1.2, label='角度')[0]
        self.ln_dtgt = a1.plot([], [], color='#cfd8e3', lw=0.9, ls='--',
                               label='目标')[0]
        self.ln_ddps = a2.plot([], [], color='#b07cff', lw=1.2, label='速度')[0]
        self.ln_dcur = a3.plot([], [], color='#b07cff', lw=1.2, label='电流')[0]
        a1.legend(loc='upper left', ncol=2, fontsize=8, framealpha=0.9)
        a2.legend(loc='upper left', fontsize=8, framealpha=0.9)
        a3.legend(loc='upper left', fontsize=8, framealpha=0.9)
        self.cv_dial = self._canvas(fig, fr)

    def _build_tab_compare(self):
        fr = ttk.Frame(self.nb)
        self.nb.add(fr, text='发弹对比')
        top = ttk.Frame(fr)
        top.pack(fill='x', padx=6, pady=(6, 0))
        ttk.Label(top, text='轮：', foreground=MUTED,
                  background=BG).pack(side='left')
        self.cmp_wheel = tk.StringVar(value='平均')
        cb = ttk.Combobox(top, textvariable=self.cmp_wheel, width=8, state='readonly',
                          values=('平均',) + WHEELS)
        cb.pack(side='left')
        cb.bind('<<ComboboxSelected>>', lambda _e: self.draw_compare(force=True))
        ttk.Label(top, text='   已对齐到掉速起点；点选左侧行可高亮对应那一发',
                  foreground=MUTED, background=BG).pack(side='left', padx=6)
        self.lbl_cmp = ttk.Label(top, text='', foreground=MUTED, background=BG)
        self.lbl_cmp.pack(side='right')

        fig, (a1, a2) = self._new_fig(2, (3, 2))
        a1.set_ylabel('Δrpm（相对发射前基线）')
        a2.set_ylabel('A')
        a2.set_xlabel('相对掉速起点 / s')
        a1.axvline(0, color='#4a5566', lw=0.8)
        a2.axvline(0, color='#4a5566', lw=0.8)
        self.fig_cmp = fig
        self.ax_cmp = (a1, a2)
        self.cv_cmp = self._canvas(fig, fr)

    def _build_tab_shot(self):
        fr = ttk.Frame(self.nb)
        self.nb.add(fr, text='单发三轮变化')
        self.shot_detail_label = ttk.Label(fr, text='等待发弹；左侧选一发查看，未选择时显示最新一发')
        self.shot_detail_label.pack(fill='x', padx=6, pady=4)
        cols=('wheel','before','minimum','drop','pct','after','delta','recover','over')
        self.shot_detail=ttk.Treeview(fr, columns=cols, show='headings', height=3)
        for c,h in zip(cols,('轮位','发前rpm','最低rpm','掉速rpm','掉速%','发后rpm','发后Δrpm','恢复ms','超调rpm')):
            self.shot_detail.heading(c,text=h)
            self.shot_detail.column(c,width=76,anchor='center')
        self.shot_detail.pack(fill='x',padx=6)
        ttk.Label(fr,text='发前：掉速前300～50ms中位数基线；发后：起点后400～500ms均值；恢复：从最低点起，回到各轮基线±100rpm并保持50ms。',foreground=MUTED).pack(fill='x',padx=6)
        controls=ttk.Frame(fr)
        controls.pack(fill='x',padx=6,pady=3)
        ttk.Label(controls,text='显示范围：').pack(side='left')
        self.shot_zoom=tk.StringVar(value='完整过程')
        zoom=ttk.Combobox(controls,textvariable=self.shot_zoom,state='readonly',width=12,values=('完整过程','过弹细节','恢复过程'))
        zoom.pack(side='left',padx=5)
        zoom.bind('<<ComboboxSelected>>',lambda _e:self.draw_shot())
        self.show_shot_command=tk.BooleanVar(value=True)
        ttk.Checkbutton(controls,text='叠加电流指令（同色虚线；实线为反馈）',variable=self.show_shot_command,command=self.draw_shot).pack(side='left',padx=8)
        fig,(a1,a2,a3)=self._new_fig(3,(3,2,2))
        a1.get_gridspec().update(left=.10, right=.98, hspace=.14)
        self.ax_shot=(a1,a2,a3)
        self.cv_shot=self._canvas(fig,fr)

    def draw_shot(self):
        sel=self.tree.selection()
        index=int(sel[0]) if sel else len(self.shots)-1
        key=(index,id(self.shots[index]) if 0<=index<len(self.shots) else None,self.shot_zoom.get(),self.show_shot_command.get())
        if getattr(self,'_shot_draw_key',None)==key:return
        self._shot_draw_key=key
        self.shot_detail.delete(*self.shot_detail.get_children())
        for ax in self.ax_shot:ax.clear()
        if not 0<=index<len(self.shots):
            self.shot_detail_label.configure(text='等待发弹；左侧选一发查看')
            self.cv_shot.draw_idle();return
        shot=self.shots[index]
        self.shot_detail_label.configure(text='第%d条｜弹速 %.3f m/s｜%s｜0ms为遥测掉速阈值，非接触传感器' % (index+1,shot['speed'],shot['note'] or '测量完成'))
        def value(x,fmt='%.0f'):
            return fmt%x if math.isfinite(x) else '—'
        for w in range(3):
            self.shot_detail.insert('', 'end', values=(WHEELS[w],value(shot['bases'][w]),value(shot['mins'][w]),value(shot['drop_rpm'][w]),value(100*shot['drop'][w],'%.2f'),value(shot['after_rpm'][w]),value(shot['after_delta'][w],'%+.1f'),value(shot['recovery_ms'][w]),value(shot['overshoot_rpm'][w])))
        seg=shot['seg']
        a1,a2,a3=self.ax_shot
        if seg:
            for w in range(3):
                a1.plot(seg['t']*1000,seg['rpm'][:,w],color=WCOLOR[w],label=WHEELS[w])
                a1.axhline(shot['bases'][w],color=WCOLOR[w],lw=.6,ls='--',alpha=.5)
                a1.scatter([shot['min_ms'][w]],[shot['mins'][w]],color=WCOLOR[w],s=22)
                a2.plot(seg['t']*1000,seg['rpm'][:,w]-shot['bases'][w],color=WCOLOR[w],label=WHEELS[w])
                a3.plot(seg['t']*1000,seg['current_raw'][:,w],color=WCOLOR[w],label=WHEELS[w]+'反馈')
                if self.show_shot_command.get():
                    a3.plot(seg['t']*1000,seg['command_raw'][:,w],color=WCOLOR[w],ls='--',alpha=.65,label=WHEELS[w]+'指令')
            a1.legend(ncol=3,fontsize=8)
            a3.legend(ncol=3,fontsize=7)
        a1.set_ylabel('rpm');a2.set_ylabel('Δ rpm');a3.set_ylabel('电流原始值');a3.set_xlabel('相对掉速起点 / ms')
        limits={'完整过程':(-300,600),'过弹细节':(-20,80),'恢复过程':(0,200)}[self.shot_zoom.get()]
        for ax in self.ax_shot:
            ax.axvline(0,color=MUTED,lw=.8);ax.grid(alpha=.25);ax.set_xlim(*limits)
        for ax in self.ax_shot[:-1]:ax.tick_params(labelbottom=False)
        self.cv_shot.draw_idle()

    def _build_tab_raw(self):
        fr = ttk.Frame(self.nb)
        self.nb.add(fr, text='原始通道')
        cols = ('ch', 'name', 'val', 'unit')
        self.rawtree = ttk.Treeview(fr, columns=cols, show='headings')
        for c, h, wdt in zip(cols, ('通道', '含义', '数值', '单位'),
                             (60, 260, 160, 80)):
            self.rawtree.heading(c, text=h)
            self.rawtree.column(c, width=wdt, anchor='w')
        self.rawtree.pack(fill='both', expand=True, padx=8, pady=8)
        names = [
            ('0', '12点转速', 'rpm'), ('1', '4点转速', 'rpm'),
            ('2', '8点转速', 'rpm'), ('3', '目标转速', 'rpm'),
            ('4', '裁判弹速', 'm/s'), ('5', '射击计数', ''),
            ('6', '射击消息时刻', 'ms'), ('7', '采样时刻', 'ms'),
            ('8', '裁判数据间隔', 'ms'), ('9', '发射机构编号', ''),
            ('10', '12点电流指令', 'raw'), ('11', '4点电流指令', 'raw'),
            ('12', '8点电流指令', 'raw'), ('13', '12点反馈电流', 'raw'),
            ('14', '4点反馈电流', 'raw'), ('15', '8点反馈电流', 'raw'),
            ('16', '拨盘角度', 'deg'), ('17', '拨盘目标', 'deg'),
            ('18', '拨盘速度', 'dps'), ('19', '拨盘输出', '归一化'),
            ('20', '拨盘电流', '归一化'), ('21', '拨盘反馈有效', ''),
            ('22', '卡弹', ''), ('23', '待进弹计数', ''),
            ('24', '三轮到速就绪', ''),
        ]
        for i, (ch, name, unit) in enumerate(names):
            self.rawtree.insert('', 'end', iid=str(i),
                                values=(ch, name, '—', unit))

    # ---------------- 串口 ----------------
    def refresh_ports(self):
        items = list(list_ports.comports())

        def rank(p):
            d = p.device.lower()
            if 'usbmodem' in d or 'usbserial' in d or 'slab' in d:
                return (0, d)          # DAPLink / USB 转串口优先
            return (2 if 'bluetooth' in d else 1, d)

        items.sort(key=rank)
        values = ['%s  —  %s' % (p.device, p.description) for p in items]
        self._ports = [p.device for p in items]
        self.cmb_port['values'] = values
        if values and not self.port_var.get():
            self.port_var.set(values[0])

    def _selected_port(self):
        raw = self.port_var.get().strip()
        if raw in getattr(self, '_ports', []):
            return raw
        for dev in getattr(self, '_ports', []):
            if raw.startswith(dev):
                return dev
        return raw.split()[0] if raw else ''

    def toggle_connect(self):
        if self.reader is not None:
            self.disconnect()
            return
        self.connect(self._selected_port())

    def connect(self, port):
        if not port:
            messagebox.showwarning('未选择端口', '先选一个串口再连接。')
            return
        self.port = port
        try:
            baud = int(self.baud_var.get())
        except ValueError:
            baud = DEFAULT_BAUD
        # 新连接从干净的时基开始：Reader 板载时间从 0 起，旧数据留着会把时间轴拧乱
        self.clear_all()
        reader_type = MonitorReader if port.endswith('status.json') else Reader
        self.reader = reader_type(port, baud, self.session, self._status_from_thread)
        self.reader.start()
        self.btn_conn.configure(text='断开')
        self.replaying = False

    def apply_dial_phase(self):
        try:
            clockwise = float(self.phase_var.get())
            if not math.isfinite(clockwise) or abs(clockwise)>10:
                raise ValueError('初始偏移须在-10°至+10°之间')
            if not isinstance(self.reader, MonitorReader):
                raise ValueError('请以 --monitor 方式连接后台采集后调节')
            directory = Path(self.reader.port).parent
            status = json.loads((directory / 'status.json').read_text())
            if not status.get('connected') or time.time()-datetime.datetime.fromisoformat(status['updated_at']).timestamp() > 3:
                raise ValueError('后台串口未连接或状态已过期')
            command = {'phase_centideg': round(-clockwise*100), 'created_unix_s': time.time()}
            temporary = directory / 'dial_phase_command.tmp'
            temporary.write_text(json.dumps(command))
            temporary.replace(directory / 'dial_phase_command.json')
            self.phase_status.configure(text='已提交，等待板端确认…')
            self.root.after(300, lambda: self.check_phase_reply(command['created_unix_s'], directory))
        except (ValueError, OSError, KeyError) as exc:
            self.phase_status.configure(text=str(exc))

    def check_phase_reply(self, submitted, directory):
        try:
            status = json.loads((directory / 'status.json').read_text())
            reply = status.get('dial_phase_reply', {})
            if reply.get('received_unix_s', 0) >= submitted:
                if reply['accepted']:
                    self.phase_status.configure(text='板端已应用：顺时针%.2f°；重新使能后生效。' % -reply['phase_deg'])
                else:
                    self.phase_status.configure(text='板端拒绝：请先关闭CH6，再点击应用。')
                return
        except (OSError, ValueError, KeyError):
            pass
        if time.time()-submitted > 4:
            self.phase_status.configure(text='未收到板端确认；请检查DAPLink TX→UART1 RX接线。')
        else:
            self.root.after(300, lambda: self.check_phase_reply(submitted, directory))

    def disconnect(self):
        if self.reader is not None:
            self.reader.stop()
            self.reader.join(timeout=1.0)
            self.reader = None
        self.btn_conn.configure(text='连接')
        self._set_conn('已断开', MUTED)

    def _status_from_thread(self, level, text):
        # 读线程不能碰 Tk 组件（Tk 只在主线程里安全），丢进队列让 _poll 取
        self.status_q.put((level, text))

    def _set_conn(self, text, color):
        self.lbl_conn.configure(text=text, foreground=color)

    # ---------------- 定时 ----------------
    def _poll(self):
        if self._quit:
            self.on_close()
            return
        while True:
            try:
                level, text = self.status_q.get_nowait()
            except queue.Empty:
                break
            self._set_conn(text, OK if level == 'ok' else BAD)
        self._ingest()
        if not self.paused and self._view_key() != self._draw_key:
            self.draw_all()
        self._flush_record()
        self._update_readout()
        self._update_status()
        if self.follow:
            self._tl_sync = True
            self.timeline.set(1.0)
            self._tl_sync = False
        self.root.after(POLL_MS, self._poll)

    def _ingest(self):
        t, d = self.session.rows_since(self.seen)
        if t is None or not len(t):
            return
        self.seen += len(t)
        self._last_t = float(t[-1])
        if self.t_min is None:
            self.t_min = float(t[0])

        # 射击计数变化 = 新的一发。逐样本比，避免一批里跨两次计数时重复记账。
        cnt = d[:, IDX['count']].astype(np.int64)
        prev = self.prev_count
        events = []
        for j in range(len(cnt)):
            c = int(cnt[j])
            if prev is None:
                prev = c
                continue
            if c != prev and 0 < (c-prev) % 65536 < 32768:
                events.append({'t': float(t[j]), 'speed': float(d[j, IDX['speed']])})
            prev = c
        self.prev_count = prev
        self.pending.extend(events)

        if self.record_writer is not None:
            for j in range(len(t)):
                self.record_writer.writerow([t[j]] + list(d[j]))
                self._record_rows += 1

        # 窗口数据够了就结算这一发
        while self.pending and self.pending[0]['t'] + POST_S < self._last_t - 0.05:
            ev = self.pending.pop(0)
            win_t, win_d = self.session.window(ev['t'] - PRE_S, ev['t'] + POST_S)
            # 窗口里再按计数找事件，只留下属于本次待结算的这一条
            shots = [s for s in analyze_all(win_t, win_d, self.target_speed)
                     if abs(s['t'] - ev['t']) < 0.15]
            self.add_shots(shots)

    def add_shots(self, shots):
        if not shots:
            return
        self.shots.extend(shots)
        self.mark_duplicates_fix()
        self.refresh_table()
        self.refresh_stats()
        self.add_markers(shots)
        self.draw_compare(force=True)
        self.draw_shot()

    def mark_duplicates_fix(self):
        for s in self.shots:
            s['dup'] = False
            if s['note'] == '疑似重复上报':
                s['note'] = ''
        mark_duplicates(self.shots)

    # ---------------- 记录 ----------------
    def toggle_record(self):
        if self.record_writer is not None:
            self.stop_record()
            return
        outdir = Path(__file__).resolve().parent / 'records'
        outdir.mkdir(exist_ok=True)
        name = 'shot_%s.csv' % datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
        self.record_path = outdir / name
        self.record_fp = open(self.record_path, 'w', newline='')
        self.record_writer = csv.writer(self.record_fp)
        self.record_writer.writerow(CSV_FIELDS)
        self._record_rows = 0
        self._record_t0 = 0.0
        self._last_flush = time.monotonic()
        self.btn_rec.configure(text='停止记录')
        self._set_conn('记录中：%s' % name, ACCENT)

    def _flush_record(self):
        if self.record_fp is None:
            return
        now = time.monotonic()
        if now - self._last_flush > 1.0:
            self.record_fp.flush()
            self._last_flush = now

    def stop_record(self):
        if self.record_fp is None:
            return
        self.record_fp.flush()
        self.record_fp.close()
        self.record_fp = None
        self.record_writer = None
        self.btn_rec.configure(text='开始记录')
        self._set_conn('已保存 %s（%d 行）' % (self.record_path.name,
                                             self._record_rows), OK)
        if self.shots:
            self.export_shots(self.record_path.with_name(
                self.record_path.stem + '_shots.csv'), quiet=True)

    def export_shots(self, path=None, quiet=False):
        if not self.shots:
            if not quiet:
                messagebox.showinfo('没有数据', '还没有记录到发弹。')
            return
        if path is None:
            path = filedialog.asksaveasfilename(
                defaultextension='.csv', initialfile='shots.csv',
                filetypes=[('CSV', '*.csv')])
            if not path:
                return
        with open(path, 'w', newline='') as fp:
            w = csv.writer(fp)
            w.writerow(['t_s', 'speed_mps', 'delta', 'min_12', 'min_4', 'min_8',
                        'drop_pct', 'recovery_ms', 'peak_a', 'dial_dps',
                        'ready', 'latency_ms', 'flag'] + [f'{metric}_{wheel}' for wheel in ('12','4','8') for metric in ('before_rpm','min_rpm','drop_rpm','drop_pct','after_rpm','after_delta_rpm','recovery_ms','overshoot_rpm','onset_ms','minimum_ms')])
            for s in self.shots:
                w.writerow([
                    '%.3f' % s['t'], '%.4f' % s['speed'],
                    '%.4f' % (s['speed'] - self.target_speed),
                    *['%.0f' % v for v in s['mins']],
                    '%.2f' % (100 * s['drop_mean']),
                    '%.1f' % s['recov_ms'],
                    '%.2f' % max(s['peak_amp']),
                    '%.1f' % s['dial_dps'], s['ready'],
                    '%.1f' % s['latency_ms'],
                    s['note'] or 'OK'] + [v for wheel in range(3) for v in (s['bases'][wheel],s['mins'][wheel],s['drop_rpm'][wheel],100*s['drop'][wheel],s['after_rpm'][wheel],s['after_delta'][wheel],s['recovery_ms'][wheel],s['overshoot_rpm'][wheel],s['onset_ms'][wheel],s['min_ms'][wheel])])
        if not quiet:
            self._set_conn('已导出 %s' % Path(path).name, OK)

    def pick_csv(self):
        path = filedialog.askopenfilename(filetypes=[('CSV', '*.csv')])
        if path:
            self.load_csv(path)

    def load_csv(self, path):
        try:
            t, d = read_csv(path)
        except Exception as exc:            # noqa: BLE001
            messagebox.showerror('读取失败', str(exc))
            return
        self.disconnect()
        self.session.clear()
        self.session.append(t, d)
        self.seen = len(t)
        self._last_t = float(t[-1]) if len(t) else 0.0
        self.t_min = float(t[0]) if len(t) else None
        self.follow = True
        self.view_end = self._last_t
        self.shots = analyze_all(t, d, self.target_speed)
        self.pending.clear()
        self.prev_count = int(d[-1, IDX['count']]) if len(d) else None
        self.replaying = True
        # 回放时把视图停在最后一发附近，否则 100 多秒的记录要自己拖很久
        span = self._last_t - self.t_min if self.t_min is not None else 0.0
        self.WINDOW_S = min(20.0, max(4.0, span))
        if self.shots:
            self.follow = False
            self.view_end = min(self._last_t, self.shots[-1]['t'] + 2.0)
        else:
            self.follow = True
            self.view_end = self._last_t
        self.spin_window.set('%d' % int(self.WINDOW_S))
        self._sync_timeline()
        self.refresh_table()
        self.refresh_stats()
        self.clear_markers()
        self.add_markers(self.shots)
        self.draw_all(force=True)
        self._set_conn('已载入 %s · %d 发' % (
            Path(path).name, len(self.shots)), ACCENT)

    # ---------------- 表格 / 统计 ----------------
    def clear_all(self):
        self.session.clear()
        self.seen = 0
        self.shots = []
        self.pending.clear()
        self.prev_count = None
        self._last_t = 0.0
        self.t_min = None
        self.follow = True
        self.view_end = 0.0
        self.clear_markers()
        self.refresh_table()
        self.refresh_stats()
        for ax in (self.ax_cmp[0], self.ax_cmp[1]):
            for ln in list(ax.lines):
                if ln.get_gid() == 'shot':
                    ln.remove()
        self.draw_all(force=True)

    def refresh_table(self):
        sel = self.tree.selection()
        keep = sel[0] if sel else None
        self.tree.delete(*self.tree.get_children())
        lo = self.target_speed - self.tol
        hi = self.target_speed + self.tol
        for n, s in enumerate(self.shots, 1):
            if s['dup']:
                tag, flag = 'dup', '重复'
            elif not s['valid']:
                tag, flag = 'dup', s['note'] or '无效'
            elif s['speed'] > hi:
                tag, flag = 'hi', '偏高'
            elif s['speed'] < lo:
                tag, flag = 'lo', '偏低'
            else:
                tag, flag = 'ok', 'OK'
            rec = s['recov_ms']
            self.tree.insert('', 'end', iid=str(n - 1), tags=(tag,), values=(
                n, '%.2f' % s['t'], '%.3f' % s['speed'],
                '%+.3f' % (s['speed'] - self.target_speed),
                '%.0f' % s['mins'][0], '%.0f' % s['mins'][1],
                '%.0f' % s['mins'][2],
                '—' if not math.isfinite(rec) else '%.0f' % rec,
                '%.1f' % max(s['peak_amp']) if s['valid'] else '—',
                flag))
        if keep is not None and self.tree.exists(keep):
            self.tree.selection_set(keep)

    def refresh_stats(self):
        lo = self.target_speed - self.tol
        hi = self.target_speed + self.tol
        st = shot_stats(self.shots, lo, hi)
        if not st:
            for v in self.stat_vars.values():
                v.set('—')
            self.lbl_verdict.configure(text='等待发弹…', foreground=MUTED)
            return
        self.stat_vars['n'].set(str(st['n']))
        self.stat_vars['mean'].set('%.3f' % st['mean'])
        self.stat_vars['std'].set('%.3f' % st['std'])
        self.stat_vars['range'].set('%.3f' % (st['max'] - st['min']))
        self.stat_vars['hit'].set('%d/%d' % (st['hit'], st['n']))
        self.stat_vars['bias'].set('%+.3f' % (st['mean'] - self.target_speed))
        pct = 100.0 * st['hit'] / st['n']
        if st['hit'] == st['n']:
            self.lbl_verdict.configure(
                text='全部落在 %.2f ± %.2f 内' % (self.target_speed, self.tol),
                foreground=OK)
        else:
            self.lbl_verdict.configure(
                text='命中率 %.0f%%，标准差 %.3f m/s（目标 ±%.2f）' %
                     (pct, st['std'], self.tol), foreground=WARN)

    # ---------------- 绘制 ----------------
    def add_markers(self, shots):
        for s in shots:
            if not s['valid'] or s['dup']:
                continue
            color = OK if abs(s['speed'] - self.target_speed) <= self.tol else WARN
            ln = self.ax_rpm.axvline(s['t'], color=color, lw=0.8, alpha=0.7)
            self.markers.append(ln)

    def clear_markers(self):
        for ln in self.markers:
            ln.remove()
        self.markers.clear()

    def _view_key(self):
        """当前画面由什么决定：数据条数、是否跟随、视图终点、页签。

        数据没变就不重画——回放一趟 100 多秒的记录时，否则会一直满核跑。
        """
        tab = self.nb.index(self.nb.select()) if self.nb.tabs() else -1
        return (self.seen, self.follow, round(self.view_end, 3),
                round(self.WINDOW_S, 3), tab)

    def draw_all(self, force=False):
        self._draw_key = self._view_key()
        tab = self.nb.index(self.nb.select()) if self.nb.tabs() else 0
        if tab == 0:
            self.draw_wave()
        elif tab == 1:
            self.draw_dial()
        elif tab == 2:
            self.draw_compare(force=force)
        elif tab == 3:
            self.draw_shot()
        else:
            self.draw_raw()

    def _on_pick_shot(self):
        """选中一发：叠图里高亮，同时把主视图挪到那一发附近。

        只在选中项真的换了的时候挪视图，表格刷新后恢复选中不会把视图拽回去。
        """
        sel = self.tree.selection()
        i = int(sel[0]) if sel else None
        if i is None or not (0 <= i < len(self.shots)):
            self._picked = None
            self.draw_compare(force=True)
            return
        if i != self._picked:
            self._picked = i
            self.follow = False
            self.view_end = min(self._last_t, self.shots[i]['t'] + 1.0)
            if self.WINDOW_S > 12.0:
                self.WINDOW_S = 12.0
                self.spin_window.set('12')
            self._sync_timeline()
            self.draw_all(force=True)
            return
        self.draw_compare(force=True)
        self.draw_shot()

    def _window_data(self):
        t1 = self._last_t if self.follow else self.view_end
        t0 = max(0.0, t1 - self.WINDOW_S)
        t, d = self.session.window(t0, t1 + 1e-6)
        # 长窗口下 300Hz 采样有几万个点，抽稀到 ~4000 点，曲线形状不变但重画快得多
        if t is not None and len(t) > 4000:
            step = len(t) // 4000 + 1
            t, d = t[::step], d[::step]
        return t, d, t0, t1

    def apply_window(self, _evt=None):
        try:
            val = float(self.spin_window.get())
        except ValueError:
            return
        self.WINDOW_S = min(120.0, max(2.0, val))
        self.draw_all(force=True)

    def _sync_timeline(self):
        if self.t_min is None:
            return
        span = max(1e-6, self._last_t - self.t_min)
        frac = (self.view_end - self.t_min) / span
        self._tl_sync = True
        self.timeline.set(min(1.0, max(0.0, frac)))
        self._tl_sync = False

    def _on_timeline(self, value):
        if getattr(self, '_tl_sync', False) or self.t_min is None:
            return
        span = max(1e-6, self._last_t - self.t_min)
        self.view_end = self.t_min + float(value) * span
        self.follow = False
        self.draw_all(force=True)

    def enable_follow(self):
        self.follow = True
        self.draw_all(force=True)

    def draw_wave(self):
        t, d, t0, t1 = self._window_data()
        if t is None or len(t) < 2:
            return
        for w in range(3):
            self.ln_rpm[w].set_data(t, d[:, IDX['rpm'][w]])
            self.ln_amp[w].set_data(t, d[:, IDX['cur'][w]] * AMP_PER_RAW)
            self.ln_cmd[w].set_data(t, d[:, IDX['cmd'][w]] * AMP_PER_RAW)
        self.ln_target.set_data(t, d[:, IDX['target']])
        self.ax_rpm.set_xlim(t0, t1)
        lo = float(min(d[:, list(IDX['rpm'])].min(), d[:, IDX['target']].min()))
        hi = float(max(d[:, list(IDX['rpm'])].max(), d[:, IDX['target']].max()))
        self.ax_rpm.set_ylim(lo - 200, hi + 200)
        a = float(np.max(np.abs(d[:, list(IDX['cur'])]))) * AMP_PER_RAW
        a = max(a, float(np.max(np.abs(d[:, list(IDX['cmd'])]))) * AMP_PER_RAW)
        self.ax_amp.set_ylim(-max(2.0, a * 1.15), max(2.0, a * 1.15))
        self.cv_wave.draw_idle()

    def draw_dial(self):
        t, d, t0, t1 = self._window_data()
        if t is None or len(t) < 2:
            return
        # 固件角度以逆时针为正，界面统一按顺时针为正显示，和"初始偏移"同一参考系
        self.ln_angle.set_data(t, -d[:, IDX['dial_angle']])
        self.ln_dtgt.set_data(t, -d[:, IDX['dial_target']])
        self.ln_ddps.set_data(t, -d[:, IDX['dial_dps']])
        self.ln_dcur.set_data(t, d[:, IDX['dial_cur']] * DIAL_T_MAX)
        a1, a2, a3 = self.ax_dial
        a1.set_xlim(t0, t1)
        ang = -d[:, IDX['dial_angle']]
        a1.set_ylim(float(ang.min()) - 10, float(ang.max()) + 10)
        v = float(np.max(np.abs(d[:, IDX['dial_dps']])))
        a2.set_ylim(-max(50.0, v * 1.1), max(50.0, v * 1.1))
        c = float(np.max(np.abs(d[:, IDX['dial_cur']]))) * DIAL_T_MAX
        a3.set_ylim(-max(1.0, c * 1.2), max(1.0, c * 1.2))
        self.cv_dial.draw_idle()

    def draw_compare(self, force=False):
        sel = self.tree.selection()
        sel_i = int(sel[0]) if sel else None
        sel_shot = (self.shots[sel_i]
                    if sel_i is not None and 0 <= sel_i < len(self.shots) else None)
        wheel = self.cmp_wheel.get()
        wi = None if wheel == '平均' else WHEELS.index(wheel)
        shots = [s for s in self.shots if s['valid'] and s['seg']]
        a1, a2 = self.ax_cmp
        for ax in (a1, a2):
            for ln in list(ax.lines):
                if ln.get_gid() == 'shot':
                    ln.remove()
            for tx in list(ax.texts):
                tx.remove()
        if not shots:
            self.lbl_cmp.configure(text='还没有可比对的发弹')
            self.cv_cmp.draw_idle()
            return
        n = len(shots)
        cmap = matplotlib.colormaps['viridis']
        for k, s in enumerate(shots):
            seg = s['seg']
            base = (seg['base'] if wi is None
                    else float(seg['bases'][wi]))
            if wi is None:
                y = seg['rpm'].mean(axis=1) - base
                amp = seg['amp'].mean(axis=1)
            else:
                y = seg['rpm'][:, wi] - base
                amp = seg['amp'][:, wi]
            sel_k = s is sel_shot
            color = '#ffd166' if sel_k else cmap(k / max(1, n - 1))
            lw = 2.2 if sel_k else 0.8
            alpha = 1.0 if sel_k else 0.55
            ln = a1.plot(seg['t'], y, color=color, lw=lw, alpha=alpha)[0]
            ln.set_gid('shot')
            ln2 = a2.plot(seg['t'], amp, color=color, lw=lw, alpha=alpha)[0]
            ln2.set_gid('shot')
            if sel_k or n <= 12:
                a1.annotate('%.2f' % s['speed'],
                            (float(seg['t'][-1]), float(y[-1])),
                            fontsize=7, color=color, alpha=alpha,
                            xytext=(3, 0), textcoords='offset points')
        a1.set_xlim(-0.5, 1.0)
        a2.set_xlim(-0.5, 1.0)
        self.lbl_cmp.configure(text='%d 发叠图' % n)
        self.cv_cmp.draw_idle()

    def draw_raw(self):
        t, d = self.session.rows_since(max(0, self.seen - 400))
        if d is None or not len(d):
            return
        row = d[-1]
        for i in range(min(NCH, len(self.rawtree.get_children()))):
            v = float(row[i])
            txt = ('%.4f' % v) if i in (4,) else ('%.1f' % v if i >= 16 else '%d' % round(v))
            self.rawtree.set(str(i), 'val', txt)

    # ---------------- 读数 / 状态 ----------------
    def _update_readout(self):
        t, d = self.session.rows_since(max(0, self.seen - 1))
        if d is None or not len(d):
            return
        row = d[-1]
        for w in range(3):
            self.ro_vars[(w, 'rpm')].set('%d' % round(row[IDX['rpm'][w]]))
            self.ro_vars[(w, 'cmd')].set('%d' % round(row[IDX['cmd'][w]]))
            self.ro_vars[(w, 'amp')].set('%.1f' % (row[IDX['cur'][w]] * AMP_PER_RAW))
        self.ro_vars[('extra', 'target')].set('%d' % round(row[IDX['target']]))
        self.ro_vars[('extra', 'speed')].set('%.3f' % row[IDX['speed']])
        clockwise = -row[IDX['dial_angle']]
        # 拨盘 8 格、每格 45°；多圈累计对操作无意义，只显示距最近格点的偏移（±22.5°），
        # 它就是要填进“初始偏移”的值。
        slot = clockwise - 45.0 * round(clockwise / 45.0)
        self.ro_vars[('extra', 'dial')].set(
            '%+.1f° / %+.0fdps' % (slot, -row[IDX['dial_dps']]))
        self.phase_angle_var.set('%+.1f°' % slot)
        ready = row[IDX['ready']]
        self.ro_vars[('extra', 'ready')].set('是' if ready else '否')

    def _update_status(self):
        if self.reader is not None:
            r = self.reader
            nch = r.nch if isinstance(r.nch, int) else 0
            fps = '%d 通道 · %.0f 帧/秒 · 累计 %d 帧 · 重同步 %d' % (
                nch, r.fps, r.frames, r.errors)
        else:
            fps = '离线回放 · 累计 %d 帧' % self.seen
        t, d = self.session.rows_since(max(0, self.seen - 1))
        if d is not None and len(d):
            row = d[-1]
            self.lbl_right.configure(
                text='%s · 裁判间隔 %.0fms · 等待进弹 %d · 卡弹 %d' % (
                    fps, row[IDX['ref_age']], row[IDX['pending']], row[IDX['jam']]))
        else:
            self.lbl_right.configure(text=fps)

    # ---------------- 其他 ----------------
    def toggle_pause(self):
        self.paused = not self.paused
        self.btn_pause.configure(text='继续曲线' if self.paused else '暂停曲线')

    def apply_target(self, _evt=None):
        try:
            sp = float(self.entry_target.get())
            tol = abs(float(self.entry_tol.get()))
        except ValueError:
            return
        if not (0 < sp < 400):
            return
        self.target_speed, self.tol = sp, max(tol, 0.001)
        self.refresh_table()
        self.refresh_stats()
        self.draw_compare(force=True)

    def on_close(self):
        if self.record_writer is not None:
            self.stop_record()
        self.disconnect()
        self.root.destroy()


CSV_FIELDS = ['host_elapsed_s', 'rpm_12h', 'rpm_4h', 'rpm_8h', 'target_rpm',
              'bullet_speed_mps', 'shot_count_u16', 'shot_rx_ms_u16',
              'sample_ms_u16', 'referee_age_ms', 'shooter_id',
              'cmd_12h_raw', 'cmd_4h_raw', 'cmd_8h_raw',
              'current_12h_raw', 'current_4h_raw', 'current_8h_raw',
              'dial_angle_deg', 'dial_target_deg', 'dial_speed_dps',
              'dial_output_norm', 'dial_current_norm', 'dial_fb_ok',
              'dial_jammed', 'dial_slot', 'wheels_ready']


def read_csv(path):
    """读记录（本工具或 tools/capture_friction.py 写的同名列）。"""
    with open(path, newline='') as fp:
        rd = csv.reader(fp)
        head = next(rd)
        rows = [r for r in rd if len(r) >= len(head)]
    arr = np.asarray(rows, dtype=np.float64)
    col = {name: i for i, name in enumerate(head)}
    t = arr[:, col['host_elapsed_s']]
    d = np.zeros((len(arr), NCH), dtype=np.float32)
    for i, name in enumerate(CSV_FIELDS[1:]):
        if name in col:
            d[:, i] = arr[:, col[name]]
    # Offline replay uses the same board clock as the live Reader, avoiding USB batching jitter.
    if 'sample_ms_u16' in col and len(t)>1:
        t=np.asarray(reconstruct_times(d[:,IDX['sample_ms']],float(t[0])))
        keep=np.r_[np.diff(t)>0,True]
        t,d=t[keep],d[keep]
    return t, d


# ---------------------------------------------------------------------------
# 命令行
# ---------------------------------------------------------------------------
def self_test(path, target=16.45, tol=0.05):
    t, d = read_csv(path)
    shots = analyze_all(t, d, target)
    st = shot_stats(shots, target - tol, target + tol)
    print('文件 %s：%d 行，时长 %.1f s，%d 条射击消息' %
          (path, len(t), t[-1] - t[0] if len(t) else 0, len(shots)))
    print('%-3s %8s %8s %8s %8s %8s %8s %8s %7s %5s %s' % (
        '#', '时刻', '弹速', 'Δ目标', '12点', '4点', '8点', '恢复ms', '峰值A', '就绪', '判定'))
    for i, s in enumerate(shots, 1):
        flag = s['note'] or ('偏高' if s['speed'] > target + tol else
                             '偏低' if s['speed'] < target - tol else 'OK')
        print('%-3d %8.2f %8.3f %+8.3f %8.0f %8.0f %8.0f %8.0f %7.1f %5d %s' % (
            i, s['t'], s['speed'], s['speed'] - target,
            s['mins'][0], s['mins'][1], s['mins'][2], s['recov_ms'],
            max(s['peak_amp']), s['ready'], flag))
    if st:
        print('\n有效 %d 发：均值 %.3f  标准差 %.3f  极差 %.3f  命中 %d/%d' % (
            st['n'], st['mean'], st['std'], st['max'] - st['min'],
            st['hit'], st['n']))
    return shots


def main():
    ap = argparse.ArgumentParser(description='三摩擦轮弹速上位机')
    ap.add_argument('--port', help='串口设备，例如 /dev/cu.usbmodemXXXX')
    ap.add_argument('--monitor', action='store_true', help='实时共享后台采集数据，不占用串口')
    ap.add_argument('--baud', type=int, default=DEFAULT_BAUD)
    ap.add_argument('--replay', help='启动时载入一份 CSV 记录')
    ap.add_argument('--self-test', metavar='CSV', help='不开界面，直接出统计')
    ap.add_argument('--target', type=float, default=16.45)
    ap.add_argument('--tol', type=float, default=0.05)
    ap.add_argument('--wheel-view', action='store_true', help='启动后显示单发三轮变化页面')
    ap.add_argument('--smoke', action='store_true', help='界面自检后自动关闭')
    args = ap.parse_args()

    if args.self_test:
        self_test(args.self_test, args.target, args.tol)
        return

    root = tk.Tk()
    monitor_status = Path(__file__).resolve().parent / 'records/monitor/status.json'
    app = App(root, port=str(monitor_status) if args.monitor else args.port, replay=args.replay)
    app.target_speed = args.target
    app.tol = args.tol
    app.entry_target.delete(0, 'end')
    app.entry_target.insert(0, '%.2f' % args.target)
    app.entry_tol.delete(0, 'end')
    app.entry_tol.insert(0, '%.2f' % args.tol)
    app.apply_target()
    if args.wheel_view:
        app.nb.select(3)
    root.protocol('WM_DELETE_WINDOW', app.on_close)

    def _on_sig(_signum, _frame):
        # 信号里不直接碰 Tk，交给下一拍 _poll 收尾，避免退出时 abort
        app._quit = True

    for _sig in (signal.SIGINT, signal.SIGTERM):
        try:
            signal.signal(_sig, _on_sig)
        except (ValueError, OSError):
            pass
    app.refresh_ports()
    if args.smoke:
        root.after(1500, app.on_close)
    root.mainloop()


if __name__ == '__main__':
    main()
