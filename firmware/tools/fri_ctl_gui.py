#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
三摩擦轮上位机  (12点 / 4点 / 8点)  +  实时曲线

通过板载 USB CDC 虚拟串口下发每轮使能与转速，回读三轮实时数据并绘制曲线。

依赖:  python3 -m pip install pyserial matplotlib
        （用 tools/fri_ctl.sh 启动，它会用工程内的 .venv）

协议 (定义见 Application/booster_task.h)

  PC -> 板   指令帧 12 字节, 包头 0x5A
    0     header        0x5A
    1     enable        bit0=12点  bit1=4点  bit2=8点
    2-3   spd_12H       int16, 幅值×方向, 与固件同量纲
    4-5   spd_4H        int16
    6-7   spd_8H        int16
    8     seq           帧序号, 递增回绕
    9     shot_counter  进弹计数, 每点一次"进弹"+1
                         固件里目标角度被硬夹在"已走完的格 ~ 已走完的格+1"之间(见固件
                         DIAL_LEAD_MAX_SLOT)：**这一发出了(到位容差内)之后点的才有效**，
                         比一格(约130ms)更快的连点会被丢掉。所以"已发"读数比点击次数
                         少是固件主动舍弃，不是丢帧
                        固件按 (新-旧) 差值累加 Dial_Shoot, 丢帧不会丢发弹
    10    zero_byte     拨弹盘零点: 0~255 → 0~45°, 步进 45/256 ≈ 0.176°
                        固件算 实测角度 = 编码器角度 - zero, 用它把弹槽对到进弹口
                        掉电不保存, 所以每帧都要带(反正 50Hz 一直在发)
    11    checksum      XOR(字节 1..10)

  板 -> PC   回传帧 52 字节, 包头 0x5B
    0     header        0x5B
    1     status        bit0=link_ok, bit1..3=生效使能
                        bit4=三轮轮速就绪(都到速并稳住 ≥50ms)
                        bit5=任一速度环输出饱和(≥97% 满量程)
                        bit6=三轮反馈都新鲜(≤20ms)
                        bit7=CAN 错误(ESR 报警/被动/bus-off 或最近 1s TX 失败)
    2     seq_echo      板最近一次接受的指令 seq
    3     jam_retry     这一发已经自动解卡几次; 到 JAM_RETRY_MAX 就停在回拨位不再重试
    4-7   timestamp     uint32 微秒 (DWT)，横轴按它画，不受投递抖动影响
    8..   三轮, 每轮 10 字节: speed, ref, out, cur, temp 各 int16
    38..  拨弹盘, 10 字节:     angle, target, speed, out, cur 各 int16
    48    shots         发弹数 (Dial_Shoot 低 8 位, 按 int8 解, 标定零点后可能为负)
    49    dial_state    bit0=卡弹处理中, bit1=拨弹盘在控
    50    checksum      XOR(字节 1..49)
    51    帧尾          0x0D

卡弹: 板上判定"该动却没动"(目标差 >4° 但转子 <300rpm 持续 120ms) 就算卡弹, 此时
      **进弹请求会被固件直接丢弃**(不入队), 免得解卡后把攒下的几发一次推出去。
      板子会自己反转半格(22.5°)退开, 退到就再顶一次。这样顶不动会来回磨, 所以磨
      JAM_RETRY_MAX 次(默认 3)后就停手: 停在回拨位、红灯不灭、进弹仍然被丢, 等人工
      介入 —— 点"退弹一发"或重新使能, 计数清零重新给满次数。
      本程序在卡弹期间也会把"进弹"按钮置灰, 配合固件一起挡。

注意: 固件侧有 300ms 指令看门狗 —— 本程序停止发帧超过 300ms, 板上会自动刹停三轮。
      macOS 的 App Nap 可能让后台窗口掉帧, 请保持本窗口在前台。
"""

import csv
import datetime
import faulthandler
import logging
import os
import queue
import shutil
import struct
import sys
import threading
import time
import tkinter as tk
import traceback
from pathlib import Path
from tkinter import ttk, messagebox

import numpy as np

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    raise SystemExit(
        "缺少 pyserial —— 说明你现在用的不是工程里的 venv。\n"
        "\n"
        "请用启动脚本运行（它会自动用工程内的 .venv）:\n"
        "\n"
        "    ./tools/fri_ctl.sh\n"
        "\n"
        "或在 tools/ 目录下:\n"
        "\n"
        "    ./fri_ctl.sh\n"
        "    ../.venv/bin/python fri_ctl_gui.py\n"
        "\n"
        "若 .venv 不存在, 重建:\n"
        "\n"
        "    /opt/homebrew/bin/python3.14 -m venv .venv\n"
        "    .venv/bin/pip install pyserial matplotlib\n"
    )

try:
    from matplotlib.figure import Figure
    from matplotlib.backends.backend_tkagg import (
        FigureCanvasTkAgg, NavigationToolbar2Tk)
    import matplotlib
except ImportError:
    raise SystemExit(
        "缺少 matplotlib。请运行:\n\n"
        "    .venv/bin/pip install matplotlib\n"
    )

# ---------------------------------------------------------------- 日志
# 崩溃排查用：应用层日志 + (配合 tools/fri_ctl.sh 的 tee) C 层致命错误也进同一个文件
LOG_PATH = Path(__file__).resolve().with_name("fri_ctl.log")
LOG_MAX_BYTES = 32 * 1024 * 1024      # 单个日志文件超过它就轮转
LOG_KEEP = 3                          # 历史文件保留个数 (.1 最新)
LOG_SCAN_S = 30                       # 多久查一次大小
log = logging.getLogger("fri")
_fault_fh = None      # faulthandler 的输出句柄, 必须一直持有


def rotate_logs():
    """按大小轮转 fri_ctl.log。

    这个文件有三个写入方同时攥着 fd: logging 的 FileHandler、faulthandler 的
    _fault_fh、以及 fri_ctl.sh 里的 tee。三个都是 O_APPEND, 所以只能
    "复制走内容 + 原地截断"; 若像 RotatingFileHandler 那样改名, 它们会继续往
    改名后的旧 inode 写, 新建的 fri_ctl.log 永远空着。
    复制和截断之间那几毫秒的写入会丢 —— 日志而已, 认了。
    """
    try:
        if LOG_PATH.stat().st_size < LOG_MAX_BYTES:
            return
    except OSError:
        return

    # 先删最老的, 再从大到小挪, 顺序反了会把刚挪过去的又删掉
    try:
        Path(f"{LOG_PATH}.{LOG_KEEP}").unlink(missing_ok=True)
        for i in range(LOG_KEEP - 1, 0, -1):
            src = Path(f"{LOG_PATH}.{i}")
            if src.exists():
                src.replace(Path(f"{LOG_PATH}.{i + 1}"))
    except OSError:
        pass

    try:
        shutil.copyfile(LOG_PATH, f"{LOG_PATH}.1")
        with open(LOG_PATH, "r+b") as f:
            f.truncate(0)
    except OSError as exc:
        log.warning("日志轮转失败: %s", exc)
        return
    log.info("日志超过 %d MB, 已轮转; 上一段见 %s.1",
             LOG_MAX_BYTES >> 20, LOG_PATH.name)


def _log_rotator():
    """后台轮转。放线程里是因为复制 32MB 会卡住 Tk 主线程"""
    while True:
        time.sleep(LOG_SCAN_S)
        try:
            rotate_logs()
        except Exception as exc:                       # noqa: BLE001
            log.warning("日志轮转异常: %s", exc)


def setup_logging():
    global _fault_fh
    logging.basicConfig(
        filename=LOG_PATH, filemode="a", level=logging.INFO,
        format="%(asctime)s.%(msecs)03d %(levelname)-7s [%(threadName)s] %(message)s",
        datefmt="%Y-%m-%d %H:%M:%S",
    )
    # 段错误 / SIGABRT / Fatal Python error 这类 C 层崩溃, Python 的 excepthook 抓不到
    # (那种情况下进程直接死, 连 shell 那句 "Segmentation fault" 也不走管道),
    # 用 faulthandler 把当时的调用栈直接落盘。
    _fault_fh = open(LOG_PATH, "a", buffering=1, encoding="utf-8", errors="replace")
    faulthandler.enable(file=_fault_fh, all_threads=True)
    log.info("")
    log.info("=" * 72)
    log.info("启动  pid=%d  python=%s", os.getpid(), sys.version.split()[0])
    log.info("      matplotlib=%s  pyserial=%s  numpy=%s",
             matplotlib.__version__, serial.__version__, np.__version__)
    log.info("      log=%s", LOG_PATH)

    def _hook(exc_type, exc, tb):
        log.critical("未捕获异常\n%s",
                     "".join(traceback.format_exception(exc_type, exc, tb)))
    sys.excepthook = _hook

    def _thread_hook(a):
        log.critical("线程 [%s] 未捕获异常\n%s", a.thread.name,
                     "".join(traceback.format_exception(
                         a.exc_type, a.exc_value, a.exc_traceback)))
    threading.excepthook = _thread_hook


# matplotlib 默认字体 DejaVu Sans 没有中文字形, 不换字体的话所有中文标签都是方块
matplotlib.rcParams["font.sans-serif"] = [
    "Hiragino Sans GB", "PingFang SC", "Arial Unicode MS", "STHeiti",
    "Heiti SC", "DejaVu Sans",
]
matplotlib.rcParams["axes.unicode_minus"] = False   # 负号用 ASCII, 免得也变方块
# 这几款中文字体只有 300 字重, matplotlib 会刷一堆 findfont 警告 —— 静音掉
logging.getLogger("matplotlib.font_manager").setLevel(logging.ERROR)

# ---------------------------------------------------------------- 协议常量
CMD_HEADER    = 0x5A
TELEM_HEADER  = 0x5B
TELEM_TAIL    = 0x0D
CMD_LEN       = 12
TELEM_LEN     = 52

CMD_SHOT_OFF     = 9         # 指令帧: 进弹计数字节
CMD_ZERO_OFF     = 10        # 指令帧: 拨弹盘零点字节 (0~255 → 0~45°)
TELEM_RETRY_OFF  = 3         # 回传帧: 这一发已自动解卡几次
TELEM_DIAL_OFF   = 38        # 回传帧: 拨弹盘块(10 字节)
TELEM_SHOTS_OFF  = 48        # 回传帧: 发弹计数
TELEM_DSTATE_OFF = 49        # 回传帧: 拨弹盘状态

# 回传帧 byte1(status) 位定义，必须与 Application/booster_task.h 的 PC_TELEM_ST_* 一致
ST_LINK       = 0x01
ST_EN_12H     = 0x02
ST_EN_4H      = 0x04
ST_EN_8H      = 0x08
ST_READY      = 0x10         # 三轮都到速并稳住
ST_SAT        = 0x20         # 任一速度环输出打满
ST_FB_OK      = 0x40         # 三轮反馈都新鲜
ST_CAN_ERR    = 0x80         # CAN 报警/被动/bus-off 或最近 1s TX 失败

# CSV 列顺序。三轮和拨盘的值直接沿用回传帧的通道顺序:
#   speed, ref, out, cur, temp / angle, target, speed, out, cur
CSV_HEADER = (
    ["pc_time", "board_t_s", "ts_us", "status",
     "link", "ready", "sat", "fb_ok", "can_err",
     "seq_echo", "jam_retry", "shots", "dial_state"]
    + [f"w{wi}_{name}" for wi in ("12h", "4h", "8h")
       for name in ("speed", "ref", "out", "cur", "temp")]
    + ["dial_angle", "dial_target", "dial_speed", "dial_out", "dial_cur",
       "event"]
)

SEND_PERIOD_MS = 20          # 指令 50 Hz
RX_TIMEOUT_MS  = 200         # 超过这么久没回传就把状态标红
BAUDRATE       = 115200      # USB CDC 下无实际意义, 随便填

ZERO_MAX_DEG = 45.0          # 零点滑块量程 = 一个弹槽, 与固件 DIAL_DEG_PER_SHOT 对齐
JAM_RETRY_MAX = 3            # 与固件 DIAL_JAM_RETRY_MAX 对齐, 到它就停手等人工介入

SPEED_MIN     = 0            # 转速按"幅值"填，方向由每轮的 正/反 决定
SPEED_MAX     = 10000        # 与固件 FRI_SPD_LIMIT 对齐
SPEED_DEFAULT = 5000         # 与旧 front_speed 一致

WHEELS = ("12点 上", "4点 右下", "8点 左下")

# 回传帧里每个轮子的通道顺序: speed, ref, out, cur, temp
CH_SPEED, CH_REF, CH_OUT, CH_CUR, CH_TEMP = 0, 1, 2, 3, 4
CH_PER_WHEEL = 5
NCH = CH_PER_WHEEL * 4       # 三轮 + 拨弹盘

# 拨弹盘块的通道顺序: 角度, 目标角度, 转速, PID输出, 转矩电流
DIAL_BASE = CH_PER_WHEEL * 3
CH_D_ANG, CH_D_TGT, CH_D_SPD, CH_D_OUT, CH_D_CUR = 0, 1, 2, 3, 4

WINDOW_S     = 10.0          # 曲线滚动窗口长度(秒)
HIST_MAX     = 4000          # 环形缓冲容量(约 10s @ 200Hz 有余量)
REDRAW_MS    = 50            # 绘图刷新 20 FPS —— 数据全存, 只降刷新
WHEEL_COLORS = ("#1f77b4", "#ff7f0e", "#2ca02c")


def ch(wheel: int, kind: int) -> int:
    """轮子 i 的第 kind 个通道在环形缓冲里的列号"""
    return wheel * CH_PER_WHEEL + kind


def dch(kind: int) -> int:
    """拨弹盘第 kind 个通道的列号"""
    return DIAL_BASE + kind


def xor_checksum(data) -> int:
    c = 0
    for b in data:
        c ^= b
    return c


def cmd_checksum(frame: bytes) -> int:
    """指令帧校验和 = 字节 1..10 的异或（不含包头；校验和在 [11]）"""
    return xor_checksum(frame[1:11])


def telem_checksum(frame: bytes) -> int:
    """回传帧校验和 = 字节 1..49 的异或（不含包头、校验和本身、帧尾）"""
    return xor_checksum(frame[1:50])


def scan_ports():
    """返回 [(设备名, 下拉框显示文本)]，把机器人的口排在最前面。

    macOS 上优先 cu.*（不等待载波）。描述里带 STM32 的就是机器人的 USB CDC；
    DAPLink 的虚拟串口（描述是 CMSIS-DAP / Horco）排在后面，免得选错：
    连到 DAPLink 上不会报错，但永远收不到回传。
    """
    out = []
    for p in list_ports.comports():
        desc = (p.description or "").strip()
        label = f"{p.device}  –  {desc}" if desc else p.device
        out.append((p.device, label, desc.lower()))
    out.sort(key=lambda e: (
        0 if "stm32" in e[2] else 1,        # 机器人的 CDC 排最前
        0 if "/cu." in e[0] else 1,
        e[0],
    ))
    return [(dev, label) for dev, label, _ in out]


class FrictionApp:
    def __init__(self, root: tk.Tk):
        self.root = root
        root.title("三摩擦轮上位机  (12点 / 4点 / 8点)")
        root.minsize(900, 760)

        self.ser = None
        self.ser_lock = threading.Lock()
        self.rx_thread = None
        self.rx_stop = threading.Event()
        self.rx_queue = queue.Queue()
        self.connected = False
        self._disconnecting = False   # 防 _disconnect <-> _write_frame 互相递归
        self.seq = 0
        self.last_rx_t = 0.0

        # ---- 掉线自动重连 ----
        # USB CDC 会被电机 EMI / 接触抖动打断，板子本身不复位（实测 RCC_CSR 全 0），
        # 主机侧看到的是 "hardware connection lost"，一秒左右就重新枚举回来。
        # 以前掉一次就得手动点连接，还经常赶上设备节点重建的瞬间报 No such file。
        self._conn_id = 0             # 每次成功接管一个新口 +1，用来丢弃旧线程的残留事件
        self._port_info = None        # (设备名, 序列号)：重连时按它找同一个口
        self._reconnect_stop = threading.Event()
        self._reconnect_thread = None
        self._pending_ser = None      # 后台线程只把新口放这儿，由 Tk 线程接管

        self.shot_counter = 0        # 进弹计数, 点一次"进弹" +1
        self.dial_shots = 0          # 板上回传的发弹数(Dial_Shoot 低 8 位)
        self.dial_state = 0          # bit0 卡弹处理中, bit1 拨弹盘在控
        self.zero_byte = 0           # 拨弹盘零点, 随每帧下发 (0~255 → 0~45°)
        self._feed_locked = None     # 进弹按钮上次的置灰状态, 只在变化时 configure

        self._syncing = False        # 防止「总控 <-> 单轮」互相触发成死循环

        self.en_vars   = [tk.BooleanVar(value=False) for _ in WHEELS]
        self.spd_vars  = [tk.IntVar(value=SPEED_DEFAULT) for _ in WHEELS]
        self.dir_vars  = [tk.IntVar(value=1) for _ in WHEELS]   # 1=正转  -1=反转
        self.meas_vars = [tk.StringVar(value="--") for _ in WHEELS]

        self.master_en_var  = tk.BooleanVar(value=False)
        self.master_spd_var = tk.IntVar(value=SPEED_DEFAULT)
        for v in self.en_vars:
            v.trace_add("write", self._sync_master_en)

        self.port_var   = tk.StringVar()
        self.port_map   = {}          # 下拉框显示文本 -> 真实设备名
        self.status_var = tk.StringVar(value="未连接")
        self.link_var   = tk.StringVar(value="")
        self.dial_var   = tk.StringVar(value="")
        self.warn_var   = tk.StringVar(value="")

        # ---- CSV 记录（逐帧落盘，供事后分析每发前后的轮速/电流） ----
        self.recording     = False
        self.rec_btn       = None
        self.rec_var       = tk.StringVar(value="未记录")
        self.rec_path      = None
        self._rec_file     = None
        self._rec_writer   = None
        self._rec_rows     = 0
        self._csv_last_shots = None
        self._csv_last_jam   = None
        self._csv_last_link  = None
        self._csv_last_ready = None

        # 拨弹盘零点: zero_deg_var 是滑块(度), zero_show_var 显示落到字节网格上的真值
        self.zero_deg_var  = tk.DoubleVar(value=0.0)
        self.zero_show_var = tk.StringVar(value=f"0.00° (byte 0)")

        # 曲线显示开关
        self.wheel_vis = [tk.BooleanVar(value=True) for _ in WHEELS]
        self.show_ref  = tk.BooleanVar(value=True)    # 目标转速(虚线)
        self.show_cur  = tk.BooleanVar(value=True)    # 转矩电流
        self.dial_vis  = tk.BooleanVar(value=True)    # 拨弹盘曲线
        self.auto_y    = tk.BooleanVar(value=True)    # Y 轴自适应

        # 曲线数据：环形缓冲
        self.buf      = np.zeros((HIST_MAX, NCH))
        self.tbuf     = np.zeros(HIST_MAX)
        self.wr       = 0             # 写指针
        self.count    = 0
        self.ts_prev  = None          # 上一个 uint32 微秒时间戳(处理回绕)
        self.ts_abs   = 0.0

        self._build_ui()
        self._refresh_ports()

        # 统计(心跳日志用)
        self.n_frames = 0     # 收到的回传帧
        self.n_bad    = 0     # 校验/帧尾不符被丢弃的候选帧
        self.n_sent   = 0     # 发出的指令帧
        self.n_redraw = 0     # 绘图次数
        self.t_start  = time.time()

        root.protocol("WM_DELETE_WINDOW", self._on_close)
        root.report_callback_exception = self._on_tk_error

        # 周期性回调统一走 _every：出异常只记日志, 但一定会排下一轮。
        # 否则一次异常就会让那条 after 链永久断掉(图就冻住了, 但界面还活着)。
        self._every(SEND_PERIOD_MS, self._send_tick)
        self._every(30,   self._poll_rx)
        self._every(500,  self._tick_status)
        self._every(REDRAW_MS, self._redraw)
        self._every(10000, self._heartbeat)

    # ------------------------------------------------------------ 日志/兜底
    def _on_tk_error(self, exc, val, tb):
        log.critical("Tk 回调异常\n%s",
                     "".join(traceback.format_exception(exc, val, tb)))
        try:
            self.warn_var.set("内部异常，详见 tools/fri_ctl.log")
        except Exception:                              # noqa: BLE001
            pass

    def _every(self, ms, fn):
        """包装周期性回调：异常只记录，且一定重新排下一轮"""
        def tick():
            try:
                fn()
            except Exception:                          # noqa: BLE001
                log.exception("%s() 异常", fn.__name__)
            finally:
                self.root.after(ms, tick)
        self.root.after(ms, tick)

    def _heartbeat(self):
        """每 10 秒记一次状态, 崩溃时能从日志看出死前在什么状态"""
        log.info("心跳 运行%.0fs 收帧=%d 丢弃=%d 发出=%d 重绘=%d 队列=%d %s",
                 time.time() - self.t_start, self.n_frames, self.n_bad,
                 self.n_sent, self.n_redraw, self.rx_queue.qsize(),
                 "已连接" if self.connected else "未连接")

    # ------------------------------------------------------------ 界面
    def _build_ui(self):
        pad = dict(padx=6, pady=3)
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(4, weight=1)          # 曲线区可拉伸

        # ---- 连接区
        top = ttk.LabelFrame(self.root, text="连接")
        top.grid(row=0, column=0, sticky="ew", padx=8, pady=(8, 4))

        ttk.Label(top, text="串口").grid(row=0, column=0, **pad)
        self.port_box = ttk.Combobox(top, textvariable=self.port_var,
                                     width=38, state="readonly")
        self.port_box.grid(row=0, column=1, **pad)
        ttk.Button(top, text="刷新", command=self._refresh_ports, width=6
                   ).grid(row=0, column=2, **pad)
        self.connect_btn = ttk.Button(top, text="连接", command=self._toggle_connect,
                                      width=8)
        self.connect_btn.grid(row=0, column=3, **pad)
        ttk.Label(top, textvariable=self.status_var).grid(row=0, column=4, **pad)

        # ---- 总控
        mas = ttk.LabelFrame(self.root, text="总控")
        mas.grid(row=1, column=0, sticky="ew", padx=8, pady=4)

        ttk.Checkbutton(mas, text="总使能（三路同步）", variable=self.master_en_var,
                        command=self._apply_master_en
                        ).grid(row=0, column=0, padx=6, pady=6)
        ttk.Label(mas, text="总转速").grid(row=0, column=1, padx=(16, 4))
        e = ttk.Entry(mas, textvariable=self.master_spd_var, width=8, justify="right")
        e.grid(row=0, column=2, padx=4)
        e.bind("<Return>",   lambda _e: self._apply_master_spd())
        e.bind("<FocusOut>", lambda _e: self._apply_master_spd())
        ttk.Scale(mas, from_=SPEED_MIN, to=SPEED_MAX, orient="horizontal",
                  length=260, variable=self.master_spd_var,
                  command=self._apply_master_spd
                  ).grid(row=0, column=3, padx=6)
        ttk.Label(mas, text="（动它 → 三路同时改）", foreground="#777"
                  ).grid(row=0, column=4, padx=(0, 6))

        # ---- 三轮控件
        mid = ttk.LabelFrame(self.root, text="摩擦轮")
        mid.grid(row=2, column=0, sticky="ew", padx=8, pady=4)

        for i, name in enumerate(WHEELS):
            ttk.Checkbutton(mid, text=name, variable=self.en_vars[i], width=10
                            ).grid(row=i, column=0, sticky="w", padx=6, pady=4)
            e = ttk.Entry(mid, textvariable=self.spd_vars[i], width=8, justify="right")
            e.grid(row=i, column=1, padx=4)
            e.bind("<Return>",   lambda _e, k=i: self._clamp_entry(k))
            e.bind("<FocusOut>", lambda _e, k=i: self._clamp_entry(k))
            ttk.Scale(mid, from_=SPEED_MIN, to=SPEED_MAX, orient="horizontal",
                      length=280, variable=self.spd_vars[i]
                      ).grid(row=i, column=2, padx=6)
            ttk.Radiobutton(mid, text="正", variable=self.dir_vars[i], value=1
                            ).grid(row=i, column=3, padx=(8, 0))
            ttk.Radiobutton(mid, text="反", variable=self.dir_vars[i], value=-1
                            ).grid(row=i, column=4, padx=(0, 8))
            ttk.Label(mid, textvariable=self.meas_vars[i], width=10,
                      anchor="e", font=("Menlo", 11)
                      ).grid(row=i, column=5, padx=6)
            ttk.Label(mid, text="实测", foreground="#777"
                      ).grid(row=i, column=6, padx=(0, 6))

        ttk.Label(mid, text=f"转速填幅值 {SPEED_MIN} ~ {SPEED_MAX}，方向由『正 / 反』决定",
                  foreground="#777"
                  ).grid(row=len(WHEELS), column=0, columnspan=7, sticky="w",
                         padx=6, pady=(0, 6))

        # ---- 底部：停止 + 状态
        bot = ttk.Frame(self.root)
        bot.grid(row=3, column=0, sticky="ew", padx=8, pady=(0, 4))

        # ---- 拨弹盘零点（实时随帧下发，板上掉电不保存）
        zrow = ttk.Frame(bot)
        zrow.pack(fill="x", pady=(0, 4))
        ttk.Label(zrow, text="拨弹盘零点").pack(side="left")
        ttk.Scale(zrow, from_=0.0, to=ZERO_MAX_DEG, orient="horizontal", length=280,
                  variable=self.zero_deg_var, command=self._apply_zero
                  ).pack(side="left", padx=6)
        ttk.Label(zrow, textvariable=self.zero_show_var, width=16, anchor="w",
                  font=("Menlo", 11)).pack(side="left")
        ttk.Label(zrow, text="当前槽位角度（顺时针+）", foreground="#777"
                  ).pack(side="left", padx=(10, 2))
        self.dial_angle_var = tk.StringVar(value="—")
        ttk.Label(zrow, textvariable=self.dial_angle_var, width=9, anchor="w",
                  font=("Menlo", 11)).pack(side="left")
        ttk.Label(bot, text="零点按固件方向下发；角度按顺时针为正显示（仅显示距最近45°格点的偏差）。"
                            "拖到弹槽正对进弹口，byte 值可回填 DIAL_ZERO_DEFAULT_DEG",
                  foreground="#777").pack(anchor="w", pady=(0, 4))

        rec_row = ttk.Frame(bot)
        rec_row.pack(fill="x", pady=(0, 4))
        self.rec_btn = tk.Button(rec_row, text="开始记录 CSV", command=self._toggle_record,
                                 bg="#2a5d8f", fg="white", activebackground="#1d466b",
                                 activeforeground="white", font=("", 11, "bold"))
        self.rec_btn.pack(side="left")
        ttk.Label(rec_row, textvariable=self.rec_var, foreground="#555"
                  ).pack(side="left", padx=8)
        ttk.Label(rec_row, text="（逐帧写 speed/ref/out/cur/temp + 状态位 + 发弹事件）",
                  foreground="#777").pack(side="left")

        row = ttk.Frame(bot)
        row.pack(fill="x", pady=(0, 4))
        self.feed_btn = tk.Button(row, text="进 弹 一 发", command=lambda: self._feed(+1),
                                  bg="#2a7d5a", fg="white", activebackground="#1b5c42",
                                  activeforeground="white", font=("", 13, "bold"))
        self.feed_btn.pack(side="left", expand=True, fill="x", padx=(0, 4))
        self.retract_btn = tk.Button(row, text="退 弹 一 发", command=lambda: self._feed(-1),
                                     bg="#6b5b9a", fg="white", activebackground="#4e4273",
                                     activeforeground="white", font=("", 13, "bold"))
        self.retract_btn.pack(side="left", expand=True, fill="x", padx=(0, 4))
        self.stop_btn = tk.Button(row, text="全 部 停 止", command=self._all_stop,
                                  bg="#d33", fg="white", activebackground="#a11",
                                  activeforeground="white", font=("", 13, "bold"))
        self.stop_btn.pack(side="left", expand=True, fill="x")
        ttk.Label(bot, textvariable=self.link_var).pack(anchor="w")
        ttk.Label(bot, textvariable=self.dial_var).pack(anchor="w")
        ttk.Label(bot, textvariable=self.warn_var, foreground="#c00").pack(anchor="w")

        # ---- 曲线区
        self._build_plot()

    def _build_plot(self):
        pf = ttk.LabelFrame(self.root, text="曲线")
        pf.grid(row=4, column=0, sticky="nsew", padx=8, pady=(4, 8))

        # 通道显隐
        bar = ttk.Frame(pf)
        bar.pack(fill="x", padx=6, pady=(4, 0))
        for i, name in enumerate(WHEELS):
            ttk.Checkbutton(bar, text=name, variable=self.wheel_vis[i]
                            ).pack(side="left", padx=(0, 8))
        ttk.Separator(bar, orient="vertical").pack(side="left", fill="y", padx=6)
        ttk.Checkbutton(bar, text="目标转速(虚线)", variable=self.show_ref
                        ).pack(side="left", padx=(0, 8))
        ttk.Checkbutton(bar, text="转矩电流(点线)", variable=self.show_cur
                        ).pack(side="left", padx=(0, 8))
        ttk.Separator(bar, orient="vertical").pack(side="left", fill="y", padx=6)
        ttk.Checkbutton(bar, text="拨弹盘", variable=self.dial_vis
                        ).pack(side="left", padx=(0, 8))
        ttk.Checkbutton(bar, text="Y 轴自适应", variable=self.auto_y
                        ).pack(side="left", padx=(0, 8))

        self.fig = Figure(figsize=(9, 4.6), dpi=100)
        self.fig.subplots_adjust(left=0.075, right=0.985, top=0.975, bottom=0.09,
                                 hspace=0.18)
        self.ax_spd  = self.fig.add_subplot(3, 1, 1)
        self.ax_trq  = self.fig.add_subplot(3, 1, 2, sharex=self.ax_spd)
        self.ax_dial = self.fig.add_subplot(3, 1, 3, sharex=self.ax_spd)
        self.ax_dial_r = self.ax_dial.twinx()      # 拨弹盘转速/输出走右轴，量纲差太远

        self.ax_spd.set_ylabel("转速")
        self.ax_trq.set_ylabel("输出/电流")
        self.ax_dial.set_ylabel("拨弹盘角度 °（顺时针为正）")
        self.ax_dial_r.set_ylabel("转速 / 输出（顺时针为正）")
        self.ax_dial.set_xlabel("时间 (s)")
        self.ax_spd.tick_params(labelbottom=False)
        self.ax_trq.tick_params(labelbottom=False)
        for ax in (self.ax_spd, self.ax_trq, self.ax_dial):
            ax.grid(True, alpha=0.3, linewidth=0.5)
        self.ax_dial_r.grid(False)

        # 每个轮子一套 Line2D，之后用 set_data 增量更新，不重画整个图
        self.l_spd, self.l_ref, self.l_out, self.l_cur = [], [], [], []
        for i in range(3):
            c = WHEEL_COLORS[i]
            self.l_spd.append(self.ax_spd.plot([], [], color=c, lw=1.3,
                                               label=f"{WHEELS[i]} 实测")[0])
            self.l_ref.append(self.ax_spd.plot([], [], color=c, lw=0.9, ls="--",
                                               alpha=0.85,
                                               label=f"{WHEELS[i]} 目标")[0])
            self.l_out.append(self.ax_trq.plot([], [], color=c, lw=1.3,
                                               label=f"{WHEELS[i]} 输出")[0])
            self.l_cur.append(self.ax_trq.plot([], [], color=c, lw=0.9, ls=":",
                                               alpha=0.85,
                                               label=f"{WHEELS[i]} 电流")[0])
        self.ax_spd.legend(loc="upper right", fontsize=7, ncol=3, framealpha=0.85)
        self.ax_trq.legend(loc="upper right", fontsize=7, ncol=3, framealpha=0.85)

        # 拨弹盘：左轴角度(实测+目标)，右轴转速 + PID 输出
        self.l_dang = self.ax_dial.plot([], [], color="#7f5aa8", lw=1.5,
                                        label="角度 实测")[0]
        self.l_dtgt = self.ax_dial.plot([], [], color="#7f5aa8", lw=0.9, ls="--",
                                        alpha=0.85, label="角度 目标")[0]
        self.l_dspd = self.ax_dial_r.plot([], [], color="#8c564b", lw=1.1,
                                          label="转速")[0]
        self.l_dout = self.ax_dial_r.plot([], [], color="#e377c2", lw=0.9, ls=":",
                                          alpha=0.85, label="PID输出")[0]
        self.ax_dial.legend(loc="upper left", fontsize=7, ncol=2, framealpha=0.85)
        self.ax_dial_r.legend(loc="upper right", fontsize=7, ncol=2, framealpha=0.85)

        self.canvas = FigureCanvasTkAgg(self.fig, master=pf)
        self.canvas.get_tk_widget().pack(fill="both", expand=True, padx=4, pady=4)
        # 工具栏会在鼠标移动时算"光标在哪个坐标系里"(contains_point),
        # 那是目前最可疑的崩溃点。设 FRI_NO_TOOLBAR=1 可以关掉它排查。
        if not os.environ.get("FRI_NO_TOOLBAR"):
            tb = NavigationToolbar2Tk(self.canvas, pf, pack_toolbar=False)
            tb.update()
            tb.pack(fill="x", padx=4, pady=(0, 4))
        else:
            log.info("已按 FRI_NO_TOOLBAR 关闭工具栏")

    # ------------------------------------------------------------ 串口
    def _refresh_ports(self):
        ports = scan_ports()
        self.port_map = {label: dev for dev, label in ports}
        labels = list(self.port_map)
        self.port_box["values"] = labels
        if labels and self.port_var.get() not in labels:
            self.port_var.set(labels[0])
        if not labels:
            self.port_var.set("")

    def _toggle_connect(self):
        if self.connected:
            self._disconnect("已断开")
        else:
            self._connect()

    def _port_identity(self, dev):
        """记住这个口在 USB 上的身份：优先序列号。

        重枚举后设备名一般不变，但节点重建需要时间；按序列号找比按路径可靠
        （STM32 CDC 的序列号来自芯片 UID，插哪个口都一样）。"""
        for p in list_ports.comports():
            if p.device == dev and p.serial_number:
                return (dev, p.serial_number)
        return (dev, None)

    def _attach(self, ser, note):
        """接管一个已经打开的口：起收帧线程、把界面切到"已连接"。

        **收帧线程必须绑定到具体的 ser 实例**，不能在循环里读 self.ser：
        重连之后旧线程如果还活着，会跑去读新口，两个线程抢同一份数据。
        事件里带上 _conn_id，_poll_rx 才能把上一个连接的残留帧丢掉。"""
        self.ser = ser
        self.connected = True
        self.last_rx_t = 0.0
        self.seq = 0
        self._conn_id += 1
        rx_stop = threading.Event()
        self.rx_stop = rx_stop
        self.rx_thread = threading.Thread(
            target=self._rx_loop, args=(ser, rx_stop, self._conn_id), daemon=True)
        self.rx_thread.start()
        self.connect_btn.config(text="断开")
        self.status_var.set(note)
        self.warn_var.set("")

    def _connect(self):
        port = self.port_map.get(self.port_var.get()) or self.port_var.get()
        if not port:
            messagebox.showwarning("提示", "没有可用串口，请插好 USB 后点『刷新』")
            return
        self._stop_reconnect()                         # 手动连接优先，停掉自动重连
        try:
            ser = serial.Serial(port, BAUDRATE, timeout=0.05)
        except Exception as exc:                       # noqa: BLE001
            messagebox.showerror("打开串口失败", f"{port}\n\n{exc}")
            return
        self._port_info = self._port_identity(port)
        self._reset_history()
        self._attach(ser, f"已连接 {port}")
        log.info("已连接 %s", port)

    def _disconnect(self, msg, auto=False):
        """断开。auto=True 是掉线触发的（不发停机帧、断开后自动重连）。
        手动断开会把重连状态一并清掉。"""
        if self._disconnecting:
            return
        self._disconnecting = True
        if self.recording or self._rec_file is not None:
            self._stop_record("记录已停止（断开连接）")
        try:
            # 手动断开给板子发一帧全零；串口已经掉了的时候这帧必然失败，
            # 只会再刷一条重复日志，所以 auto 路径不发
            if self.connected and not auto:
                self._write_frame(0, 0, 0, 0)          # 断开前先发一帧全零
        finally:
            self._disconnecting = False
        self.connected = False
        self.rx_stop.set()
        try:
            if self.ser:
                self.ser.close()
        except Exception:                              # noqa: BLE001
            pass
        self.ser = None
        self.connect_btn.config(text="连接")
        self.status_var.set(msg)
        for v in self.meas_vars:
            v.set("--")
        self.link_var.set("")
        self.dial_var.set("")
        # 断开时把卡弹态清掉，否则按钮会一直灰着，重连后也刷不回来
        # （_feed_locked 置 None 是为了让下一次 _update_readout 一定重设按钮外观）
        self.dial_state = 0
        self._feed_locked = None
        self.feed_btn.configure(state="normal", bg="#2a7d5a")
        log.info("已断开: %s", msg)
        if auto:
            self._start_reconnect()
        else:
            self._stop_reconnect()
            self._port_info = None

    # ------------------------------------------------------------ 自动重连
    def _start_reconnect(self):
        """掉线后在后台等设备重新枚举，成功就把新口交给 Tk 线程接管。

        USB CDC 掉线时 /dev 节点会先消失再出现（macOS 报 Device not configured /
        No such file 都是这一段）；这里每 0.4s 试一次，既不忙等也不会错过。
        板子如果真重启了，固件会在 link_ok=0 的第一帧只同步发弹计数、不补发，
        所以重连不会一次打出好几发。"""
        if self._reconnect_thread and self._reconnect_thread.is_alive():
            return
        if not self._port_info:
            return
        self._reconnect_stop.clear()
        info = self._port_info
        self.status_var.set("串口掉线，自动重连中…")
        self.warn_var.set("串口掉线，正在自动重连…")
        log.info("串口掉线，开始自动重连: %s", info[0])

        def worker():
            while not self._reconnect_stop.is_set():
                ser = self._open_port_info(info)
                if ser is not None:
                    if self._reconnect_stop.is_set():
                        # 等待期间用户手动断开/重连了，这个口作废
                        try:
                            ser.close()
                        except Exception:              # noqa: BLE001
                            pass
                    else:
                        self._pending_ser = ser
                    return
                self._reconnect_stop.wait(0.4)

        self._reconnect_thread = threading.Thread(
            target=worker, name="serial-reconnect", daemon=True)
        self._reconnect_thread.start()

    def _open_port_info(self, info):
        """按序列号找口并打开；找不到返回 None。节点可能刚建好还没就绪，
        打开失败就留给下一轮。"""
        dev, sn = info
        devs = []
        if sn:
            for p in list_ports.comports():
                if p.serial_number == sn:
                    devs.append(p.device)
        if not devs and os.path.exists(dev):
            devs.append(dev)
        for d in devs:
            try:
                return serial.Serial(d, BAUDRATE, timeout=0.05)
            except Exception:                          # noqa: BLE001
                continue
        return None

    def _stop_reconnect(self):
        self._reconnect_stop.set()
        self._reconnect_thread = None
        ser, self._pending_ser = self._pending_ser, None
        if ser is not None:
            try:
                ser.close()
            except Exception:                          # noqa: BLE001
                pass

    def _reconnect_tick(self):
        """Tk 线程里收后台线程的成果 —— 后台线程一律不碰 Tk 控件"""
        ser, self._pending_ser = self._pending_ser, None
        if ser is None or self.connected:
            return
        self._attach(ser, f"已重连 {self._port_info[0]}（自动）")
        log.info("串口自动重连成功: %s", self._port_info[0])

    # ------------------------------------------------------------ 总控
    def _apply_master_en(self):
        """总使能 -> 三路同时跟随"""
        if self._syncing:
            return
        self._syncing = True
        for v in self.en_vars:
            v.set(self.master_en_var.get())
        self._syncing = False

    def _sync_master_en(self, *_):
        """单轮使能变了 -> 总使能只反映「是否三路全开」，不反向覆盖单轮"""
        if self._syncing:
            return
        self._syncing = True
        self.master_en_var.set(all(v.get() for v in self.en_vars))
        self._syncing = False

    def _apply_master_spd(self, *_):
        """总转速 -> 三路同时跟随（滑块拖动时也会被回调, 所以签名要能吃参数）"""
        if self._syncing:
            return
        try:
            v = max(SPEED_MIN, min(SPEED_MAX, int(self.master_spd_var.get())))
        except Exception:                              # noqa: BLE001
            return
        self._syncing = True
        for var in self.spd_vars:
            var.set(v)
        self._syncing = False

    # ------------------------------------------------------------ 发送
    def _mag(self, i):
        """转速幅值。控件里可能是非法文本(用户正在编辑), 取不到就按 0 处理"""
        try:
            v = int(self.spd_vars[i].get())
        except Exception:                              # noqa: BLE001
            return 0
        return max(SPEED_MIN, min(SPEED_MAX, v))

    def _setpoint(self, i):
        """实际下发的带符号转速 = 幅值 × 正反方向"""
        d = -1 if self.dir_vars[i].get() == -1 else 1
        return d * self._mag(i)

    def _clamp_entry(self, i):
        self.spd_vars[i].set(self._mag(i))

    def _apply_zero(self, *_):
        """零点滑块 -> 指令帧 byte10（0~45° → 0~255, 步进 45/256 ≈ 0.176°）。

        滑块是连续的，但板上只收一个字节，所以量化到字节网格再显示回来，
        免得以为下发的是滑块的精度。没连接时改它也不报错，接上就生效。
        """
        try:
            deg = float(self.zero_deg_var.get())
        except Exception:                              # noqa: BLE001
            return                                     # 用户正在编辑，取值失败就保持原值
        deg = max(0.0, min(ZERO_MAX_DEG, deg))
        self.zero_byte = int(round(deg * 256.0 / ZERO_MAX_DEG)) & 0xFF
        real = self.zero_byte * ZERO_MAX_DEG / 256.0
        self.zero_show_var.set(f"{real:.2f}° (byte {self.zero_byte})")

    def _write_frame(self, enable, s0, s1, s2) -> bool:
        if not self.ser:
            return False
        payload = struct.pack("<BBhhhBBB", CMD_HEADER, enable, s0, s1, s2,
                              self.seq, self.shot_counter, self.zero_byte)
        frame = payload + bytes([cmd_checksum(payload)])
        try:
            with self.ser_lock:
                self.ser.write(frame)
            self.seq = (self.seq + 1) & 0xFF
            self.n_sent += 1
            return True
        except Exception as exc:                       # noqa: BLE001
            # 只能记一行。这里是每帧都走的路径, 拔掉 USB 后 *_write_frame 会一直
            # 失败; 打整条 traceback 会瞬间把日志刷到 GB 级。
            log.warning("串口写入失败: %s: %s", type(exc).__name__, exc)
            self.warn_var.set(f"串口写入失败: {exc}")
            self._disconnect("串口异常，已断开", auto=True)
            return False

    def _send_tick(self):
        self._reconnect_tick()                         # 自动重连成功后在这里接管
        if self.connected:
            en = 0
            for i, var in enumerate(self.en_vars):
                if var.get():
                    en |= (1 << i)
            self._write_frame(en,
                              self._setpoint(0),
                              self._setpoint(1),
                              self._setpoint(2))

    def _feed(self, n):
        """进弹(+1) / 退弹(-1)。

        只改计数 —— 固件按 (新-旧) 的差值累加 Dial_Shoot，所以丢帧不会丢发弹；
        退弹就是这个计数减 1，Dial_Shoot 跟着回退一发、拨盘反转。
        链路刚建立时固件会先同步一次计数，不会把历史差值一次性补发出去。

        卡弹时进弹不发：固件那边本来就会丢掉，这边先挡住是为了不动计数 ——
        计数一旦加上去，就算固件不补发，计数和实际发弹数也会越差越多。
        退弹照常，它本来就是手动解卡的手段。
        """
        if n > 0 and (self.dial_state & 0x01):
            log.info("卡弹处理中，进弹已忽略(不改计数)")
            return
        self.shot_counter = (self.shot_counter + n) & 0xFF
        log.info("进弹计数 -> %d (%+d)", self.shot_counter, n)

    def _all_stop(self):
        self._syncing = True
        for var in self.en_vars:
            var.set(False)
        for var in self.spd_vars:
            var.set(0)
        self.master_en_var.set(False)
        self.master_spd_var.set(0)
        self._syncing = False
        self._write_frame(0, 0, 0, 0)          # 立刻发一帧全零, 不等下一个周期

    # ------------------------------------------------------------ CSV 记录
    def _toggle_record(self):
        if self.recording:
            self._stop_record()
        else:
            self._start_record()

    def _start_record(self):
        """开始逐帧记录。板子时间戳 + PC 墙钟时间都写进去:
        横轴用 board_t_s (DWT, 不受 USB 投递抖动影响), 对时/查日志用 pc_time。"""
        if not self.connected:
            messagebox.showwarning("提示", "先连接板子，再开始记录")
            return
        if self._rec_file is not None:          # 防御: 状态不一致时先收尾
            self._stop_record()

        stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        path = Path(__file__).resolve().with_name(f"fri_log_{stamp}.csv")
        try:
            f = open(path, "w", newline="", encoding="utf-8")
        except OSError as exc:
            messagebox.showerror("无法创建 CSV", f"{path}\n\n{exc}")
            return

        self._rec_file = f
        self._rec_writer = csv.writer(f)
        self._rec_writer.writerow(CSV_HEADER)
        self._rec_rows = 0
        self._csv_last_shots = None
        self._csv_last_jam   = None
        self._csv_last_link  = None
        self._csv_last_ready = None
        self.rec_path = path
        self.recording = True
        self.rec_btn.config(text="停止记录 CSV", bg="#a11",
                            activebackground="#811")
        self.rec_var.set(f"记录中 → {path.name}")
        log.info("开始记录 CSV: %s", path)

    def _stop_record(self, note=None):
        was_recording = self.recording
        self.recording = False
        f, self._rec_file = self._rec_file, None
        self._rec_writer = None
        if f is not None:
            try:
                f.flush()
                f.close()
            except Exception as exc:            # noqa: BLE001
                log.warning("关闭 CSV 失败: %s", exc)
                self.warn_var.set(f"关闭 CSV 失败: {exc}")
        if self.rec_btn is not None:
            self.rec_btn.config(text="开始记录 CSV", bg="#2a5d8f",
                                activebackground="#1d466b")
        if note:
            self.rec_var.set(note)
        elif was_recording and self.rec_path is not None:
            self.rec_var.set(f"已停止: {self.rec_path.name} ({self._rec_rows} 行)")
        if was_recording:
            log.info("停止记录 CSV: %s (%d 行)", self.rec_path, self._rec_rows)

    def _write_csv(self, frame):
        if not self.recording or self._rec_writer is None:
            return
        try:
            status  = frame[1]
            shots   = frame[TELEM_SHOTS_OFF]
            shots_i8 = shots - 256 if shots > 127 else shots
            jammed  = bool(frame[TELEM_DSTATE_OFF] & 0x01)
            ready   = bool(status & ST_READY)
            link    = bool(status & ST_LINK)

            events = []
            if self._csv_last_shots is not None:
                # 发弹计数只有 8 位，差值按 int8 回绕算，避免 127 -> -128 记成 -255
                delta = (shots_i8 - self._csv_last_shots + 128) % 256 - 128
                if delta:
                    events.append(f"shot{delta:+d}")
            if self._csv_last_jam is not None and jammed != self._csv_last_jam:
                events.append("jam_start" if jammed else "jam_end")
            if self._csv_last_link is not None and link != self._csv_last_link:
                events.append("link_up" if link else "link_down")
            if self._csv_last_ready is not None and ready != self._csv_last_ready:
                events.append("ready_on" if ready else "ready_off")
            self._csv_last_shots = shots_i8
            self._csv_last_jam   = jammed
            self._csv_last_link  = link
            self._csv_last_ready = ready

            ts_us  = struct.unpack_from("<I", frame, 4)[0]
            wheels = [struct.unpack_from("<hhhhh", frame, 8 + i * 10) for i in range(3)]
            dial   = struct.unpack_from("<hhhhh", frame, TELEM_DIAL_OFF)

            row = [
                datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3],
                f"{self.ts_abs:.6f}",
                ts_us,
                status,
                1 if link else 0,
                1 if ready else 0,
                1 if (status & ST_SAT) else 0,
                1 if (status & ST_FB_OK) else 0,
                1 if (status & ST_CAN_ERR) else 0,
                frame[2],
                frame[TELEM_RETRY_OFF],
                shots_i8,
                frame[TELEM_DSTATE_OFF],
            ]
            for w in wheels:
                row.extend(w)
            row.extend(dial)
            row.append(";".join(events))
            self._rec_writer.writerow(row)
            self._rec_rows += 1
            if self._rec_rows % 166 == 0:       # 约 1s 落盘一次
                self._rec_file.flush()
        except Exception as exc:                # noqa: BLE001
            log.exception("CSV 写入失败")
            self.warn_var.set(f"CSV 写入失败: {exc}")
            self._stop_record("记录已停止（写入失败）")

    # ------------------------------------------------------------ 接收
    def _rx_loop(self, ser, stop_evt, conn_id):
        """后台线程: 按 0x5B 包头 + 40 字节 + 校验 + 帧尾 组帧, 结果丢进队列

        ser / stop_evt / conn_id 都是这次连接专属的。重连会起新线程，
        旧线程即使晚一步退出，也不会再碰新口。"""
        buf = bytearray()
        while not stop_evt.is_set():
            try:
                with self.ser_lock:
                    chunk = ser.read(1024)
            except Exception:                          # noqa: BLE001
                self.rx_queue.put((conn_id, None))     # 通知主线程: 串口掉了
                return
            if not chunk:
                time.sleep(0.002)
                continue
            buf.extend(chunk)
            while True:
                idx = buf.find(TELEM_HEADER)
                if idx < 0:
                    del buf[:-1]                       # 只留最后 1 字节, 可能是半个包头
                    break
                if len(buf) - idx < TELEM_LEN:
                    del buf[:idx]
                    break
                frame = bytes(buf[idx:idx + TELEM_LEN])
                if telem_checksum(frame) == frame[TELEM_LEN - 2] \
                        and frame[TELEM_LEN - 1] == TELEM_TAIL:
                    self.rx_queue.put((conn_id, frame))
                    del buf[:idx + TELEM_LEN]
                else:
                    # 校验/帧尾不符说明对齐错了(包头和校验和可能撞车),
                    # 只前进 1 字节重新找, 否则会按整帧错位一直锁死
                    self.n_bad += 1
                    del buf[:idx + 1]

    def _poll_rx(self):
        latest = None
        while True:
            try:
                conn_id, item = self.rx_queue.get_nowait()
            except queue.Empty:
                break
            if conn_id != self._conn_id:
                continue                               # 上一个连接的残留, 丢掉
            if item is None:                           # 串口掉了
                if self.connected:
                    self.warn_var.set("串口异常，已断开。正在自动重连…")
                    self._disconnect("已断开", auto=True)
                latest = None
                continue
            latest = item
            self._push(item)

        if latest is not None:
            self._update_readout(latest)

    def _update_readout(self, frame):
        self.last_rx_t = time.time()
        status = frame[1]
        for i in range(3):
            spd = struct.unpack_from("<h", frame, 8 + i * 10)[0]
            self.meas_vars[i].set(f"{spd:+d}")
        link = "链路正常" if (status & ST_LINK) else "指令超时(板已停机)"
        flags = ["轮速就绪" if (status & ST_READY) else "轮速未就绪"]
        if status & ST_SAT:
            flags.append("电流饱和")
        if not (status & ST_FB_OK):
            flags.append("轮速反馈缺失")
        if status & ST_CAN_ERR:
            flags.append("CAN异常")
        self.link_var.set(f"板回传: {link}   seq回显={frame[2]}   "
                          + " | ".join(flags))

        ang, tgt = struct.unpack_from("<hh", frame, TELEM_DIAL_OFF)
        shots, dstate = frame[TELEM_SHOTS_OFF], frame[TELEM_DSTATE_OFF]
        jammed = bool(dstate & 0x01)
        retry = frame[TELEM_RETRY_OFF]
        shots_i8 = shots - 256 if shots > 127 else shots      # 只传了低 8 位, 按 int8 解
        if not jammed:
            jam_txt = "正常"
        elif retry >= JAM_RETRY_MAX:
            jam_txt = f"卡弹·已停手(磨了{retry}次, 点退弹解卡)"   # 固件不再自动重试
        else:
            jam_txt = f"卡弹处理中(第{retry}次)"
        # bit2: 拨盘那块 C620 的反馈断了，固件已经切电流（角度不动 = 位置环也没在跑）
        fb_txt = "   [拨盘反馈超时!]" if (dstate & 0x04) else ""
        # 只显示距最近 45° 格点的偏移（顺时针+），多圈累计不能直接用于零点/偏移。
        clockwise = -ang
        slot = clockwise - 45.0 * round(clockwise / 45.0)
        self.dial_angle_var.set(f"{slot:+.1f}°")
        self.dial_var.set(
            f"拨弹盘: 槽位角度(顺时针+) {slot:+.1f}° / 目标 {-tgt}°   已发 {shots_i8} 发"
            f"   零点 {self.zero_byte * ZERO_MAX_DEG / 256.0:.2f}°"
            f"   [{jam_txt}]"
            f"  [{'在控' if (dstate & 0x02) else '失联'}]"
            f"{fb_txt}")

        # 卡弹时置灰"进弹"（固件也会丢弃，但按钮变灰用户才知道为什么点了没反应）
        if jammed != self._feed_locked:
            self._feed_locked = jammed
            self.feed_btn.configure(state="disabled" if jammed else "normal",
                                    bg="#555" if jammed else "#2a7d5a")

    def _push(self, frame):
        """把一帧回传放进环形缓冲 (时间戳来自板子, 微秒, 处理 uint32 回绕)"""
        ts_us = struct.unpack_from("<I", frame, 4)[0]
        if self.ts_prev is None:
            self.ts_abs = 0.0
        else:
            self.ts_abs += ((ts_us - self.ts_prev) & 0xFFFFFFFF) / 1e6
        self.ts_prev = ts_us

        self.tbuf[self.wr] = self.ts_abs
        for i in range(3):
            vals = struct.unpack_from("<hhhhh", frame, 8 + i * 10)
            for k in range(CH_PER_WHEEL):
                self.buf[self.wr, ch(i, k)] = vals[k]
        # 拨弹盘块: 角度, 目标角度, 转速, PID输出, 转矩电流
        dvals = struct.unpack_from("<hhhhh", frame, TELEM_DIAL_OFF)
        for k in range(CH_PER_WHEEL):
            self.buf[self.wr, dch(k)] = dvals[k]
        self.dial_shots = frame[TELEM_SHOTS_OFF]
        self.dial_state = frame[TELEM_DSTATE_OFF]

        # 记录必须放在 ts_abs / dial_shots 更新之后，_write_csv 用的是这些值
        self._write_csv(frame)

        self.wr = (self.wr + 1) % HIST_MAX
        if self.count < HIST_MAX:
            self.count += 1
        self.n_frames += 1

    def _reset_history(self):
        self.wr = 0
        self.count = 0
        self.ts_prev = None
        self.ts_abs = 0.0

    def _snapshot(self):
        """按时间顺序返回 (t, data)。环形缓冲未满时直接切片, 满了就跨接缝拼一次"""
        n = self.count
        if n == 0:
            return np.empty(0), np.empty((0, NCH))
        if n < HIST_MAX:
            return self.tbuf[:n], self.buf[:n]
        i = self.wr
        return (np.concatenate((self.tbuf[i:], self.tbuf[:i])),
                np.concatenate((self.buf[i:], self.buf[:i])))

    def _tick_status(self):
        """回传超时提示; 每 500ms 刷一次"""
        if self.connected:
            age = (time.time() - self.last_rx_t) * 1000.0
            if self.last_rx_t == 0.0 or age > RX_TIMEOUT_MS:
                self.link_var.set("未收到板回传（板子没运行新固件？USB 没枚举成 CDC？）")

    # ------------------------------------------------------------ 绘图
    def _redraw(self):
        t, data = self._snapshot()
        if len(t) >= 2:
            e = np.empty(0)
            nvis = 0
            for i in range(3):
                vis = self.wheel_vis[i].get()
                if vis:
                    nvis += 1
                # 隐藏时 x/y 必须一起清空 —— 只清 y 会变成 (303,) 对 (0,)
                # 的形状不匹配, 渲染时抛 ValueError, 图就卡住不再刷新了
                for line, kind, show in (
                        (self.l_spd[i], CH_SPEED, True),
                        (self.l_ref[i], CH_REF,   self.show_ref.get()),
                        (self.l_out[i], CH_OUT,   True),
                        (self.l_cur[i], CH_CUR,   self.show_cur.get())):
                    if vis and show:
                        line.set_data(t, data[:, ch(i, kind)])
                    else:
                        line.set_data(e, e)

            # 拨弹盘（固件角度以逆时针为正，曲线统一按顺时针为正显示）
            dvis = self.dial_vis.get()
            for line, kind in ((self.l_dang, CH_D_ANG), (self.l_dtgt, CH_D_TGT),
                               (self.l_dspd, CH_D_SPD), (self.l_dout, CH_D_OUT)):
                if dvis:
                    line.set_data(t, -data[:, dch(kind)])
                else:
                    line.set_data(e, e)

            t1 = t[-1]
            self.ax_spd.set_xlim(t1 - WINDOW_S, t1)

            # 全隐藏时没有数据可缩放, autoscale_view 会算出退化区间
            if nvis:
                if self.auto_y.get():
                    for ax in (self.ax_spd, self.ax_trq):
                        ax.relim()
                        ax.autoscale_view(scalex=False, scaley=True)
                else:
                    self.ax_spd.set_ylim(-SPEED_MAX * 1.1, SPEED_MAX * 1.1)
                    self.ax_trq.set_ylim(-17000, 17000)
            if dvis:
                if self.auto_y.get():
                    for ax in (self.ax_dial, self.ax_dial_r):
                        ax.relim()
                        ax.autoscale_view(scalex=False, scaley=True)
                else:
                    self.ax_dial.set_ylim(-2000, 500)
                    self.ax_dial_r.set_ylim(-17000, 17000)

            self.canvas.draw_idle()
        self.n_redraw += 1

    # ------------------------------------------------------------ 退出
    def _on_close(self):
        log.info("关闭窗口, 收帧=%d 丢弃=%d 发出=%d 重绘=%d 运行%.0fs",
                 self.n_frames, self.n_bad, self.n_sent, self.n_redraw,
                 time.time() - self.t_start)
        try:
            if self.connected:
                self._write_frame(0, 0, 0, 0)          # 关窗前先停机
        except Exception:                              # noqa: BLE001
            pass
        if self.recording:
            self._stop_record()
        self._stop_reconnect()
        self.rx_stop.set()
        try:
            if self.ser:
                self.ser.close()
        except Exception:                              # noqa: BLE001
            pass
        self.root.destroy()


def main():
    setup_logging()
    threading.Thread(target=_log_rotator, name="log-rotator",
                     daemon=True).start()
    root = tk.Tk()
    try:
        ttk.Style().theme_use("aqua")                  # macOS 原生外观
    except Exception:                                  # noqa: BLE001
        pass
    FrictionApp(root)
    try:
        root.mainloop()
        log.info("主循环正常退出")
    except BaseException:                              # noqa: BLE001
        log.critical("主循环抛出异常\n%s", traceback.format_exc())
        raise
    finally:
        logging.shutdown()


if __name__ == "__main__":
    main()
