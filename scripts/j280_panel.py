#!/usr/bin/env python3
"""J280 board panel: pick a profile, run it, watch it, read the result.

A thin tkinter front end over j280_balance_board.run_with_cleanup: the same
CONFIG/SWING/GAINS2 sequence, keepalive, ABORT on every exit path and stop
confirmation as the command line tool. There is still no host START: motion
needs DIP 1 ON and SW1 on the board. Every run is recorded under
docs/data/panel-sessions/.

The window shows a live parameter table (angles, codes, reference, error,
output, feedforward, observer states, sensor and input flags), an angle plot
and an applied-duty plot. The 2026-10-20 firmware keeps the v6 wire format;
configs that select the experimental alpha-rate filter use algorithm_id 2..7.
"""
from contextlib import ExitStack
from datetime import datetime
import json
from pathlib import Path
import queue
import sys
import threading
import time

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
sys.path.insert(0, str(HERE))
import j280_balance as profiles  # noqa: E402
import j280_balance_board as board  # noqa: E402
import j280_swing_math as swing_math  # noqa: E402

SESSIONS = ROOT / 'docs/data/panel-sessions'
# One firmware per wire generation; the CONFIG version selects the protocol, so
# the panel flips versions with the profile instead of a separate switch.
FIRMWARES = {
    'v6': ROOT / 'docs/data/swing-firmware-2026-10-24-travel240-45/j280_swing.fs',
    'v6-lean': ROOT / 'docs/data/swing-firmware-2026-10-26-mode16/j280_swing.fs',
    'v6-pip': ROOT / 'docs/data/swing-firmware-2026-10-26-pip/j280_swing.fs',
    'v6-hyb': ROOT / 'docs/data/swing-firmware-2026-10-26-hyb3/j280_swing.fs',
    'v5': ROOT / 'docs/data/swing-firmware-2026-10-18c/j280_swing.fs',
    'v4': ROOT / 'docs/data/balance-firmware-2026-10-18/j280_balance.fs',
}
# The submission bundle keeps the final 973 bitstream at this path; the archive
# paths above are the development records and are absent from that bundle.
BUNDLED_FIRMWARE = ROOT / 'firmware/j280_swing_973_keypad.fs'
FIRMWARE = FIRMWARES['v6'] if FIRMWARES['v6'].exists() else BUNDLED_FIRMWARE
PROGRAM_SCRIPT = ROOT / 'scripts/j280_program.ps1'
# v6 firmware profiles (no time limit; SW3/SW4 step 90 deg, DIP 2 turns). The
# v5/v4 entries are the previous generations, kept for side-by-side comparison;
# each needs its own firmware, which the download button chooses by protocol.
PRESETS = (
    ('v6-973 比赛主候选：自动起摆 + 963 的 K0/K3 + K2×1.5（不限时，SW2 停；34 个会话样本，最均衡）', 'docs/data/obs-2026-10-26/obs-973.json'),
    ('v6-994 973 + travel_limit 300（诊断：确认 240 安全窗是否截断 90° 步进）', 'docs/data/obs-2026-10-26/obs-994.json'),
    ('v6-963 962 比赛序列版：摆臂最静（arm 4.6 count）；180° 后有 α 瞬态', 'docs/data/obs-2026-10-26/obs-963.json'),
    ('v6-962 静态摆臂最静：arm 3.4 count，但 1.5–5 Hz 高频更明显', 'docs/data/obs-2026-10-26/obs-962.json'),
    ('v6-985 动作峰值最小：4×SW3 3.2–4.9°，但静止观感更震', 'docs/data/obs-2026-10-26/obs-985.json'),
    ('v6-990 预扭矩折中：arm 5.5、α 1.23（短测好，完整序列未复现）', 'docs/data/obs-2026-10-26/obs-990.json'),
    ('v6-986 973 + K2×1.75：arm 5.7、α 1.01（实验）', 'docs/data/obs-2026-10-26/obs-986.json'),
)


def firmware_for(config):
    """Firmware matching a loaded profile's wire generation and lean mode."""
    wire = board.wire_of(config)
    algorithm = config.get('values', {}).get('algorithm_id')
    if wire is board.protocol6 and algorithm in (15, 16):
        return FIRMWARES['v6-lean']
    if wire is board.protocol6 and algorithm == 17:
        return FIRMWARES['v6-pip']
    if wire is board.protocol6 and algorithm in (18, 19):
        fw = FIRMWARES['v6-hyb']
        return fw if fw.exists() else BUNDLED_FIRMWARE
    return FIRMWARES['v6' if wire is board.protocol6 else 'v5' if wire is board.protocol5 else 'v4']
REASONS_CN = dict(none='', complete='到时结束', stop_key='按了 SW2', arm_off='拨码拨到了 OFF',
                  host_abort='电脑中止', swing_timeout='起摆超时', angle_limit='摆杆超角度',
                  travel_limit='摆臂超行程', speed_limit='速度超限', sensor_invalid='传感器无效',
                  keepalive_timeout='保活超时', arm_stall='摆臂堵转', blind_limit='死区过长',
                  blind_entry='中段丢角度')


def pendulum_deg(code):
    """Degrees from upright (ADC-increasing positive) from the calibrated full-circle table."""
    return swing_math.angle_of(int(code)) / 2 ** 32 * 360


def hint(row, config):
    """One line telling the person at the board what to do now."""
    if row is None:
        return '等待板子遥测…（12V 和 USB 都接好了吗？）'
    swing = 'swing' in config
    state, loaded = row['state'], row['config_loaded']
    if state == 'FAULT':
        return '板子在故障状态：按一下 SW5 复位'
    if state == 'SAFE' and not loaded:
        if row['arm_on'] and (swing or 'gains2' in config):
            return '请把拨码 1 拨到 OFF（载入需要）'
        return '正在载入配置…'
    if state == 'SAFE':
        return '已载入：拨码 1 拨到 ON（若已是 ON，先拨 OFF 再拨回 ON）'
    if state == 'ARMED':
        return ('摆杆自然下垂静止，摆臂别碰，按 SW1 开始起摆，然后手离开' if swing
                else '一只手把摆杆扶到竖直，另一只手按 SW1')
    if state == 'CAPTURE':
        return '扶住保持竖直，听到高音后保持不动，接管后慢慢松手'
    if state == 'SWING':
        return '起摆中，手离开转动范围；异常就按 SW2'
    if state == 'BALANCE':
        t = f"平衡中 {int(row['state_elapsed_ms']) / 1000:.1f} s"
        if row.get('traj_rotating'):
            return t + '；转圈中，拨码 2 拨到 OFF 停到下一个 90° 位置'
        if row.get('traj_settled') and config.get('traj', {}).get('step_counts'):
            return t + '；SW3/SW4 左右转 90°，拨码 2 ON 连续转圈；异常就按 SW2'
        return t + '；异常就按 SW2'
    return state


def session_paths(config, stamp=None):
    stamp = stamp or datetime.now().strftime('%Y%m%d-%H%M%S')
    base = SESSIONS / f"{stamp}-{config['values']['config_id']}"
    return dict(csv=base.with_suffix('.csv'), raw=base.with_suffix('.bin'),
                summary=base.parent / (base.name + '-summary.json'))


def run_profile(open_port, config, paths, cancel=None, cue=None, on_recorder=None, wait_start_s=300.0,
                observe_s=2.0, keepalive_period_s=0.05, clock=time.monotonic):
    """One recorded session; returns the summary (also written to paths['summary'])."""
    wire = board.wire_of(config)
    paths['csv'].parent.mkdir(parents=True, exist_ok=True)
    summary = {}
    with ExitStack() as stack:
        writer, raw = board.open_outputs(stack, paths['csv'], paths['raw'], board.fields_of(wire))
        recorder = board.Recorder(writer, raw, clock=clock, wire=wire, keep_rows=True)
        if on_recorder:
            on_recorder(recorder)
        try:
            port = stack.enter_context(open_port())
            summary = board.run_with_cleanup(port, recorder, config, wait_start_s, observe_s,
                                             keepalive_period_s, cue=cue, wire=wire, cancel=cancel, clock=clock)
        except (OSError, ValueError, RuntimeError) as error:
            summary = dict(getattr(recorder, 'last_summary', None) or board.summarize(recorder),
                           error=str(error) or type(error).__name__)
        summary['balance_stats'] = board.balance_stats(recorder.kept,
                                                       config['values']['upright_zero_q16'] / 65536)
    paths['summary'].write_text(json.dumps(summary, indent=1, default=str, allow_nan=False), encoding='utf-8')
    return summary


def check_link(open_port, seconds=1.0, clock=time.monotonic, wire=board.protocol6):
    """Read-only look at the board: never writes to the port. Returns a Chinese one-liner."""
    import io
    rec = board.Recorder(board.csv.DictWriter(io.StringIO(), fieldnames=board.fields_of(wire)), None,
                         clock=clock, wire=wire, keep_rows=True)
    try:
        with open_port() as port:
            board.read_for(port, rec, seconds, clock=clock)
    except (OSError, ValueError, RuntimeError) as error:
        return f'连不上串口：{error}'
    if not rec.kept:
        v4 = rec.stream.stats.get('version_errors', 0)
        return ('收到的不是 v6 遥测：需要先“下载固件”' if v4 else
                '没有收到遥测：检查 USB，或先“下载固件”')
    r = rec.kept[-1]
    adc = '角度读数 0，12V 可能没接' if r['adc_raw10'] == 0 and r['adc_valid'] else (
        f"角度读数 {r['adc_raw10']}（下垂约 221）")
    tilt = f"{pendulum_deg(r['adc_raw10']):+.1f}°" if r['adc_valid'] else '—'
    arm_phase = (r['enc_count'] % 1040) * 360 / 1040
    duty = (1 if r['motor_mode'] == 2 else -1 if r['motor_mode'] == 1 else 0) * r['applied_compare'] / 25
    reason = REASONS_CN.get(r['stop_reason'], r['stop_reason'])
    return (f"板子在线：{r['state']}{'（' + reason + '）' if reason else ''}，"
            f"摆杆 {tilt}，摆臂相位 {arm_phase:.1f}°，输出 {duty:+.1f}%，{adc}，"
            f"拨码 1 {'ON' if r['arm_on'] else 'OFF'}，故障位 {r['fault_bits']}"
            f"{'，脱机运行中' if r.get('standalone') else ''}"
            f"{'，SW3 按下' if r.get('sw3_pressed') else ''}"
            f"{'，SW4 按下' if r.get('sw4_pressed') else ''}"
            f"{'，拨码 2 ON' if r.get('dip2_on') else ''}")


def program_firmware(fs=FIRMWARE, run=None):
    """SRAM-program the sealed v6 bitstream through the project script; returns (ok, last lines)."""
    import subprocess
    run = run or subprocess.run
    if not Path(fs).is_file():
        return False, f'找不到固件：{fs}'
    result = run(['powershell', '-NoProfile', '-ExecutionPolicy', 'Bypass', '-File', str(PROGRAM_SCRIPT),
                  '-Fs', str(fs)], capture_output=True, text=True, timeout=180)
    text = (result.stdout or '') + (result.stderr or '')
    lines = [l.strip() for l in text.splitlines() if any(k in l for k in ('User Code', 'Finished', 'rror'))]
    ok = result.returncode == 0 and any('Finished' in l for l in lines)
    return ok, '\n'.join(lines[-3:]) or text[-300:]


def describe(summary):
    """Short Chinese result text."""
    r = summary.get('result') or {}
    final = summary.get('final') or {}
    lines = []
    if summary.get('error'):
        lines.append('出错：' + summary['error'])
    reason = r.get('end_reason') or final.get('stop_reason')
    if reason:
        lines.append(f"结束：{r.get('end_state') or final.get('state')}，{REASONS_CN.get(reason, reason)}")
    if r.get('catch_s') is not None:
        lines.append(f"起摆接住用时 {r['catch_s']} s")
    if r.get('balance_s') is not None:
        lines.append(f"平衡 {r['balance_s']} s")
    s = summary.get('balance_stats') or {}
    if 'pend_std_code' in s:
        lines.append(f"摆杆晃动 {s['pend_std_code']} 码（其中快抖 {s['fast_pend_std_code']} 码），"
                     f"范围 {s['pend_range_code'][0]}…{s['pend_range_code'][1]}")
        lines.append(f"摆臂晃动 {s['arm_std_count']} count，范围 {s['arm_range_count'][0]}…{s['arm_range_count'][1]}")
        lines.append(f"输出均方根 {s['duty_rms_percent']}%，饱和 {s['saturated_fraction'] * 100:.1f}%")
    if summary.get('wire_issues'):
        lines.append('串口有丢帧（数据仍可用）：' + ', '.join(summary['wire_issues']))
    return '\n'.join(lines) or '没有结果'


class Panel:
    """The window. All board I/O runs in one worker thread; the UI polls it."""

    def __init__(self, root):
        import tkinter as tk
        from tkinter import ttk
        self.tk, self.root = tk, root
        root.title('J280 倒立摆面板')
        root.geometry('1280x980')
        root.minsize(1100, 860)
        style = ttk.Style(root)
        try:
            style.theme_use('clam')
        except Exception:
            pass
        style.configure('.', font=('Microsoft YaHei', 10))
        style.configure('TButton', padding=(10, 4))
        style.configure('TLabelframe', padding=8)
        root.configure(background='#f4f6f8')
        root.columnconfigure(0, weight=1)
        root.rowconfigure(0, weight=1)
        self.port = tk.StringVar(value='COM8')
        self.choice = tk.StringVar(value=PRESETS[0][0])
        self.path = None
        self.config = {}
        self.worker = self.recorder = None
        self.stop = threading.Event()
        self.done = queue.Queue()
        self.vars = {}
        top = ttk.Frame(root, padding=10)
        top.grid(sticky='nsew')
        top.columnconfigure(1, weight=1)
        ttk.Label(top, text='串口').grid(row=0, column=0, sticky='w')
        ttk.Entry(top, textvariable=self.port, width=8).grid(row=0, column=1, sticky='w')
        ttk.Label(top, text='配置').grid(row=1, column=0, sticky='w')
        box = ttk.Combobox(top, textvariable=self.choice, values=[p[0] for p in PRESETS], width=64, state='readonly')
        box.grid(row=1, column=1, columnspan=2, sticky='we')
        box.bind('<<ComboboxSelected>>', lambda e: self.pick())
        ttk.Button(top, text='其他文件…', command=self.browse).grid(row=1, column=3)
        ttk.Button(top, text='检查连接', command=self.check).grid(row=0, column=2, sticky='we')
        ttk.Button(top, text='下载固件', command=self.program).grid(row=0, column=3, sticky='we')
        self.purpose = ttk.Label(top, text='', wraplength=1180, foreground='#555')
        self.purpose.grid(row=2, column=0, columnspan=4, sticky='w', pady=(2, 6))
        self.start_button = ttk.Button(top, text='开始（载入并等待）', command=self.start)
        self.start_button.grid(row=3, column=0, columnspan=2, sticky='we')
        self.stop_button = ttk.Button(top, text='停止（电脑中止）', command=self.request_stop, state='disabled')
        self.stop_button.grid(row=3, column=2, columnspan=2, sticky='we')
        self.hint = ttk.Label(top, text='选择配置后点“开始”', font=('Microsoft YaHei', 15, 'bold'),
                              foreground='#0a5', wraplength=1180)
        self.hint.grid(row=4, column=0, columnspan=4, sticky='w', pady=8)
        params = ttk.LabelFrame(top, text='实时参数', padding=(10, 6))
        params.grid(row=5, column=0, columnspan=4, sticky='we', pady=(0, 8))
        params.columnconfigure(1, weight=1)
        params.columnconfigure(3, weight=1)
        important = {'state', 'end', 'config', 'angle', 'arm', 'output', 'feed',
                     'fault', 'inputs', 'motion', 'keypad'}
        shown = 0
        for key, label in (('state', '状态 / 运行'), ('end', '结束原因'),
                           ('config', '配置 / CRC'), ('angle', '摆杆倾角（竖直=0°）'),
                           ('code', '摆杆码值（ADC / 电压）'), ('arm', '摆臂相位 / 绝对角'),
                           ('target', '目标 / 误差'), ('output', '输出占空比'),
                           ('feed', 'u_ff / u_linear'), ('comp', 'u_comp / u_model'),
                           ('obs', '观测 θ / α'), ('rate', '观测速度 θ̇ / α̇'),
                           ('sensor', '角度有效性 / 质量'), ('fault', '故障位 / 控制延迟'),
                           ('inputs', '拨码1 / SW3 / SW4 / 拨码2'), ('motion', '轨迹 / 模式'),
                           ('keypad', '键盘输入（角度/方向/速度/状态）')):
            var = tk.StringVar(value='—')
            self.vars[key] = var
            if key not in important:
                continue
            row, pair = divmod(shown, 2)
            shown += 1
            ttk.Label(params, text=label).grid(row=row, column=pair * 2, sticky='w', padx=(0, 6), pady=1)
            ttk.Label(params, textvariable=var, font=('Consolas', 11), width=36,
                      anchor='w').grid(row=row, column=pair * 2 + 1, sticky='w', padx=(0, 20), pady=1)
        plots = ttk.Frame(top)
        plots.grid(row=6, column=0, columnspan=4, sticky='nsew')
        plots.columnconfigure(0, weight=1)
        plots.rowconfigure(0, weight=1)
        plots.rowconfigure(1, weight=1)
        self.canvas = tk.Canvas(plots, width=1180, height=210, background='white',
                                highlightthickness=1, highlightbackground='#c9d2da')
        self.canvas.grid(row=0, column=0, sticky='nsew')
        self.arm_canvas = tk.Canvas(plots, width=1180, height=210, background='white',
                                    highlightthickness=1, highlightbackground='#c9d2da')
        self.arm_canvas.grid(row=1, column=0, sticky='nsew', pady=(6, 0))
        self.result = tk.Text(top, width=140, height=7, font=('Microsoft YaHei', 10))
        self.result.grid(row=7, column=0, columnspan=4, pady=(8, 0), sticky='we')
        self.pick()
        root.protocol('WM_DELETE_WINDOW', self.close)
        self.plot([])
        self.plot_arm([])
        self.poll()

    def pick(self):
        name = self.choice.get()
        self.path = ROOT / dict(PRESETS)[name] if name in dict(PRESETS) else self.path
        self.show_purpose()

    def browse(self):
        from tkinter import filedialog
        chosen = filedialog.askopenfilename(initialdir=str(ROOT / 'docs/data'), filetypes=[('配置', '*-config.json')])
        if chosen:
            self.path = Path(chosen)
            self.choice.set(self.path.name)
            self.show_purpose()

    def show_purpose(self):
        try:
            self.purpose.config(text=profiles.load_config(self.path)['purpose'])
        except (OSError, ValueError, KeyError) as error:
            self.purpose.config(text=f'无法读取：{error}')

    def show_row(self, row):
        """Refresh the parameter table from the latest telemetry row."""
        if row is None:
            for var in self.vars.values():
                var.set('—')
            return
        cfg = self.config or {}
        zero = cfg.get('values', {}).get('upright_zero_q16', 737 << 16) / 65536
        adc = int(row.get('adc_raw10', 0))
        code = adc - zero
        valid = row.get('adc_valid')
        tilt = pendulum_deg(adc) if valid else float('nan')
        arm_phase = (row.get('enc_count', 0) % 1040) * 360 / 1040
        ref_deg = row.get('theta_ref', 0) * 360 / 1040
        err_deg = (row.get('enc_count', 0) - row.get('theta_ref', 0)) * 360 / 1040
        duty = (1 if row.get('motor_mode') == 2 else -1 if row.get('motor_mode') == 1 else 0) \
            * row.get('applied_compare', 0) / 25
        ul = row.get('u_linear_q16', 0) / 65536 / 25
        uc = row.get('u_compensated_q16', 0) / 65536 / 25
        um = row.get('u_model_q16', 0) / 65536 / 25
        uff = row.get('u_ff_q16', 0) / 65536 / 25
        x = [row.get(f'xhat{i}', 0) / 65536 for i in range(4)]
        alpha_deg = x[1] * 57.29578 / 162.33
        arate_deg = x[3] * 57.29578 / 162.33
        reason = REASONS_CN.get(row.get('stop_reason'), row.get('stop_reason') or '')
        motion = '转圈' if row.get('traj_rotating') else ('移动' if row.get('traj_moving') else '静止')
        if row.get('traj_settled'):
            motion += ' / 已稳定'
        mode = [name for key, name in (('swing_loaded', '起摆'), ('gains2_active', '第二套'),
                                       ('standalone', '脱机')) if row.get(key)]
        inputs = f"{'ON' if row.get('arm_on') else 'OFF'} / " \
                 f"{'●' if row.get('sw3_pressed') else '—'} / {'●' if row.get('sw4_pressed') else '—'} / " \
                 f"{'ON' if row.get('dip2_on') else 'OFF'}"
        v = self.vars
        v['state'].set(f"{row.get('state', '—')}  {row.get('state_elapsed_ms', 0) / 1000:.1f} s")
        v['end'].set(reason or '—')
        v['config'].set(f"{row.get('config_id', 0)} / 0x{row.get('config_crc', 0):04x}")
        v['angle'].set('—' if not valid else f"{tilt:+.2f}°  （{code:+.0f} code）")
        v['code'].set(f"{adc}  /  {adc * 3.3 / 1024:.2f} V")
        v['arm'].set(f"{arm_phase:.2f}°（φ {row.get('enc_count', 0) % 1040}）  "
                     f"[{row.get('enc_count', 0):+d} count]")
        v['target'].set(f"{ref_deg:+.2f}°  /  {err_deg:+.2f}°")
        v['output'].set(f"{duty:+.2f}%  （{row.get('applied_compare', 0)} cmp）")
        v['feed'].set(f"{uff:+.1f}%  /  {ul:+.1f}%")
        v['comp'].set(f"{uc:+.1f}%  /  {um:+.1f}%")
        v['obs'].set(f"{x[0]:+.1f} count  /  {x[1]:+.1f} code（{alpha_deg:+.2f}°）")
        v['rate'].set(f"{x[2]:+.0f} count/s  /  {x[3]:+.0f} code/s（{arate_deg:+.1f}°/s）")
        v['sensor'].set(f"{'有效' if valid else '无效'}，质量 {row.get('raw_estimator_quality', '—')}，"
                        f"年龄 {row.get('adc_age_us', '—')} µs")
        v['fault'].set(f"{row.get('fault_bits', 0)}  /  {row.get('controller_latency_cycles', 0)} clk")
        v['inputs'].set(inputs)
        v['motion'].set(f"{motion}  /  {'+'.join(mode) or '普通'}  /  索引 {row.get('traj_index', '—')}")
        kstate = {0: '空闲', 1: '输入中', 2: '已提交/移动', 3: '错误'}.get(row.get('key_state', 0), '?')
        kdir = '逆时针' if row.get('key_dir') else '顺时针'
        kspd = {0: 30, 1: 60, 2: 90}.get(row.get('key_speed_sel', 1), '?')
        v['keypad'].set(f"{row.get('key_entry', 0)}°  {kdir}  {kspd}°/s  {kstate}"
                        f"  (last {row.get('key_last', 0)})")

    def start(self):
        from tkinter import messagebox
        try:
            config = profiles.load_config(self.path)
        except (OSError, ValueError, KeyError) as error:
            messagebox.showerror('配置无效', str(error))
            return
        cue = None
        try:
            import winsound  # noqa: F401
            cue = board.tone
        except ImportError:
            pass
        import serial
        port_name = self.port.get().strip()
        self.config, self.stop = config, threading.Event()
        paths = session_paths(config)

        def work():
            try:
                summary = run_profile(lambda: serial.Serial(port=port_name, baudrate=230400, timeout=0.01,
                                                            write_timeout=0.2),
                                      config, paths, cancel=self.stop.is_set, cue=cue,
                                      on_recorder=lambda r: setattr(self, 'recorder', r))
            except Exception as error:      # keep the UI alive; the summary file has the details
                summary = dict(error=f'{type(error).__name__}: {error}')
            self.done.put((summary, paths))
        self.recorder = None
        self.result.delete('1.0', 'end')
        self.start_button.config(state='disabled')
        self.stop_button.config(state='normal')
        self.worker = threading.Thread(target=work, daemon=True)
        self.worker.start()

    def background(self, title, job):
        if self.worker is not None:
            return
        self.hint.config(text=title + '…')
        self.start_button.config(state='disabled')

        def work():
            try:
                text = job()
            except Exception as error:
                text = f'{type(error).__name__}: {error}'
            self.done.put(({'_message': text}, None))
        self.worker = threading.Thread(target=work, daemon=True)
        self.worker.start()

    def check(self):
        import serial
        name = self.port.get().strip()
        try:
            config = profiles.load_config(self.path) if self.path else None
            wire = board.wire_of(config) if config else board.protocol6
        except (OSError, ValueError, KeyError):
            wire = board.protocol6
        self.background('正在读取遥测（只读）', lambda: check_link(
            lambda: serial.Serial(port=name, baudrate=230400, timeout=0.01), wire=wire))

    def program(self):
        from tkinter import messagebox
        try:
            config = profiles.load_config(self.path) if self.path else None
        except (OSError, ValueError, KeyError):
            config = None
        fs = firmware_for(config) if config is not None else FIRMWARE
        if not messagebox.askokcancel('下载固件', f'把 {fs.parent.name} 写入 FPGA（断电会丢失）。\n'
                                      '下载期间电机不会动。确认拨码 1 在 OFF。'):
            return

        def job():
            ok, text = program_firmware(fs=fs)
            return ('下载完成：' if ok else '下载失败：') + text + (
                '\n接着：如果 12V 接通后才下载，不用按 SW5；否则接 12V 后按一下 SW5' if ok else '')
        self.background('正在下载固件（约 30 秒）', job)

    def request_stop(self):
        self.stop.set()

    def close(self):
        self.stop.set()
        if self.worker is not None:
            self.worker.join(timeout=3)
        self.root.destroy()

    def poll(self):
        rec = self.recorder
        row = rec.latest if rec is not None else None
        self.show_row(row)
        if self.worker is not None:
            self.hint.config(text=hint(row, self.config))
        if rec is not None and rec.kept:
            self.plot(rec.kept[-1000:])
            self.plot_arm(rec.kept[-1000:])
        else:
            self.plot([])
            self.plot_arm([])
        try:
            summary, paths = self.done.get_nowait()
        except queue.Empty:
            pass
        else:
            self.worker = None
            self.start_button.config(state='normal')
            self.stop_button.config(state='disabled')
            if '_message' in summary:
                self.hint.config(text='就绪')
                self.result.insert('end', summary['_message'] + '\n')
            else:
                self.hint.config(text='本次结束。拨码 1 拨到 OFF；要再来一次就再点“开始”')
                self.result.insert('end', describe(summary) + f"\n记录：{paths['csv'].name}\n")
        self.root.after(100, self.poll)

    def plot(self, rows):
        """Pendulum tilt over the last window (±30°, red)."""
        c = self.canvas
        w = max(c.winfo_width(), 320)
        h = max(c.winfo_height(), 140)
        c.delete('all')
        left, right, top_m, bottom = 52, 14, 24, 18
        x0, x1, y0, y1 = left, w - right, top_m, h - bottom
        mid = (y0 + y1) / 2
        for deg in (-30, -20, -10, 0, 10, 20, 30):
            y = mid - deg / 30 * (y1 - y0) / 2
            c.create_line(x0, y, x1, y, fill='#e8ecef' if deg else '#b8c2cc')
            c.create_text(x0 - 6, y, anchor='e', text=f'{deg:+d}°', fill='#7a8a99',
                          font=('Consolas', 8))
        c.create_text(x0, 4, anchor='nw', text='摆杆倾角（°）', fill='#333',
                      font=('Microsoft YaHei', 9, 'bold'))
        n = len(rows)
        last = None
        if n >= 2:
            pts = []
            for i, r in enumerate(rows):
                x = x0 + (x1 - x0) * i / (n - 1)
                a = pendulum_deg(r['adc_raw10']) if r['adc_valid'] else 0.0
                a = max(-30.0, min(30.0, a))
                pts += [x, mid - a / 30 * (y1 - y0) / 2]
                last = a
            c.create_line(*pts, fill='#c0392b', width=2)
        if last is not None:
            c.create_text(x1, 4, anchor='ne', text=f'摆杆 {last:+.2f}°', fill='#333',
                          font=('Consolas', 9))

    def plot_arm(self, rows):
        """Arm phase 0-360° over the last window; wraps, never leaves the canvas."""
        c = self.arm_canvas
        w = max(c.winfo_width(), 320)
        h = max(c.winfo_height(), 140)
        c.delete('all')
        left, right, top_m, bottom = 52, 14, 24, 18
        x0, x1, y0, y1 = left, w - right, top_m, h - bottom
        for deg in (0, 90, 180, 270, 360):
            y = y1 - (deg / 360) * (y1 - y0)
            c.create_line(x0, y, x1, y, fill='#e8ecef' if deg % 180 == 0 else '#f1f4f6')
            c.create_text(x0 - 6, y, anchor='e', text=f'{deg}°', fill='#7a8a99',
                          font=('Consolas', 8))
        c.create_text(x0, 4, anchor='nw',
                      text='摆臂相位（mod 360°，旋转时自动回绕）', fill='#333',
                      font=('Microsoft YaHei', 9, 'bold'))
        n = len(rows)
        last = None
        if n >= 2:
            seg = []
            prev = None
            for i, r in enumerate(rows):
                x = x0 + (x1 - x0) * i / (n - 1)
                ph = (r['enc_count'] % 1040) * 360 / 1040
                y = y1 - (ph / 360) * (y1 - y0)
                if prev is not None and abs(ph - prev) > 180:
                    if len(seg) >= 4:
                        c.create_line(*seg, fill='#2f6fb3', width=2)
                    seg = [x, y]
                else:
                    seg += [x, y]
                prev = ph
                last = ph
            if len(seg) >= 4:
                c.create_line(*seg, fill='#2f6fb3', width=2)
        if last is not None:
            c.create_text(x1, 4, anchor='ne', text=f'相位 {last:.1f}°', fill='#333',
                          font=('Consolas', 9))


def main():
    import tkinter as tk
    root = tk.Tk()
    Panel(root)
    root.mainloop()


if __name__ == '__main__':
    main()
