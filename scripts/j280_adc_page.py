#!/usr/bin/env python3
"""Live read-only angle-ADC page.

Large raw code, voltage, tilt, validity and a rolling trace for chasing the
intermittent three-wire sensor. Never writes to the serial port. Hanging is
about 221 codes (0.71 V at 3.3 V); upright about 737 (2.38 V).

Run: python scripts/j280_adc_page.py --port COM8
"""
import argparse
from collections import deque
import queue
import sys
import threading
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import j280_balance_board as board  # noqa: E402
import j280_balance_protocol as protocol4  # noqa: E402
import j280_measurement as protocol2  # noqa: E402
import j280_state_telemetry as protocol_state2  # noqa: E402
import j280_swing_math as swing_math  # noqa: E402
import j280_swing_protocol as protocol5  # noqa: E402
import j280_v6_protocol as protocol6  # noqa: E402

TILT_ZERO = 737
HANG_LOW, HANG_HIGH = 150, 240
UPRIGHT_LOW, UPRIGHT_HIGH = 700, 770


def tilt_deg(code):
    """Calibrated tilt from upright (0 at 737, +-180 at hanging)."""
    return swing_math.angle_of(int(code)) / 2 ** 32 * 360


class AdcPage:
    def __init__(self, root, port):
        import tkinter as tk
        from tkinter import ttk
        self.tk, self.root = tk, root
        root.title('J280 角度 ADC 监测（只读）')
        root.geometry('1100x780')
        root.minsize(900, 640)
        root.columnconfigure(0, weight=1)
        root.rowconfigure(3, weight=1)
        self.port = tk.StringVar(value=port)
        self.vref = tk.StringVar(value='3.3')
        self.q = queue.Queue()
        self.stop = threading.Event()
        self.worker = None
        self.window = deque(maxlen=3000)      # last 30 s of (t, adc, valid, state)
        self.samples = 0
        self.invalid = 0
        self.invalid_run = 0
        self.longest_invalid = 0
        self.min_code = 1023
        self.max_code = 0
        self.sum_code = 0
        self.last_t = None
        self.last_row = None
        self.flash_until = 0.0
        self.window_s = 10.0

        top = ttk.Frame(root, padding=10)
        top.grid(sticky='nsew')
        top.columnconfigure(1, weight=1)
        top.rowconfigure(4, weight=1)
        ttk.Label(top, text='串口').grid(row=0, column=0, sticky='w')
        ttk.Entry(top, textvariable=self.port, width=10).grid(row=0, column=1, sticky='w')
        ttk.Label(top, text='ADC 参考电压(V)').grid(row=0, column=2, padx=(12, 0))
        ttk.Entry(top, textvariable=self.vref, width=6).grid(row=0, column=3, sticky='w')
        self.start_button = ttk.Button(top, text='开始监测', command=self.start)
        self.start_button.grid(row=0, column=4, padx=(12, 4))
        self.stop_button = ttk.Button(top, text='停止', command=self.stop_reader, state='disabled')
        self.stop_button.grid(row=0, column=5, padx=4)
        ttk.Button(top, text='清零统计', command=self.reset).grid(row=0, column=6, padx=4)
        ttk.Label(top, text='只读：不会向板子写任何数据。12V 必须接上，否则 ADC 无信号。',
                  foreground='#666').grid(row=1, column=0, columnspan=7, sticky='w', pady=(6, 0))

        big = ttk.Frame(top, padding=(10, 8))
        big.grid(row=2, column=0, columnspan=7, sticky='we')
        big.columnconfigure(1, weight=1)
        self.code = tk.Label(big, text='---', font=('Consolas', 64, 'bold'), width=6,
                             anchor='e', foreground='#222')
        self.code.grid(row=0, column=0, rowspan=2, sticky='w')
        self.valid = tk.Label(big, text='未开始', font=('Microsoft YaHei', 20, 'bold'),
                              width=10, foreground='white', background='#888')
        self.valid.grid(row=0, column=1, sticky='w', padx=(16, 0))
        self.volt = tk.Label(big, text='— V', font=('Consolas', 22))
        self.volt.grid(row=1, column=1, sticky='w', padx=(16, 0))
        info = ttk.LabelFrame(big, text='读数', padding=8)
        info.grid(row=0, column=2, rowspan=2, sticky='e', padx=(20, 0))
        self.fields = {}
        for r, (key, label) in enumerate((('tilt', '倾角（竖直=0°）'), ('zero', '相对零点码值'),
                                          ('enc', '编码器'), ('state', '状态 / 质量'),
                                          ('age', 'ADC 年龄'), ('fault', '故障位'))):
            ttk.Label(info, text=label).grid(row=r, column=0, sticky='w')
            var = tk.StringVar(value='—')
            ttk.Label(info, textvariable=var, font=('Consolas', 12), width=22,
                      anchor='w').grid(row=r, column=1, sticky='w')
            self.fields[key] = var

        stats = ttk.LabelFrame(top, text='统计', padding=8)
        stats.grid(row=3, column=0, columnspan=7, sticky='we')
        for c in range(4):
            stats.columnconfigure(c, weight=1)
        self.stat = {}
        for i, (key, label) in enumerate((('session', '本次会话'), ('window', '最近 10 s'),
                                          ('quality', '有效 / 无效'), ('gap', '最长无效间隔'))):
            ttk.Label(stats, text=label).grid(row=0, column=i, sticky='w')
            var = tk.StringVar(value='—')
            ttk.Label(stats, textvariable=var, font=('Consolas', 11),
                      anchor='w').grid(row=1, column=i, sticky='w')
            self.stat[key] = var

        self.canvas = tk.Canvas(top, width=1060, height=320, background='white',
                                highlightthickness=1, highlightbackground='#bbb')
        self.canvas.grid(row=4, column=0, columnspan=7, sticky='nsew', pady=(8, 6))
        self.diag = tk.Label(top, text='', font=('Microsoft YaHei', 14, 'bold'),
                             foreground='#333', anchor='w')
        self.diag.grid(row=5, column=0, columnspan=7, sticky='we')
        self.footer = ttk.Label(top, text='', foreground='#666')
        self.footer.grid(row=6, column=0, columnspan=7, sticky='w')
        root.protocol('WM_DELETE_WINDOW', self.close)
        self.poll()

    def start(self):
        if self.worker is not None:
            return
        self.reset()
        self.stop.clear()
        port_name = self.port.get().strip()
        self.worker = threading.Thread(target=self.reader, args=(port_name,), daemon=True)
        self.worker.start()
        self.start_button.config(state='disabled')
        self.stop_button.config(state='normal')
        self.footer.config(text=f'正在读取 {port_name} …')

    def stop_reader(self):
        self.stop.set()
        self.worker = None
        self.start_button.config(state='normal')
        self.stop_button.config(state='disabled')
        self.footer.config(text='已停止')

    def reset(self):
        self.window.clear()
        self.samples = self.invalid = self.invalid_run = self.longest_invalid = 0
        self.min_code, self.max_code, self.sum_code = 1023, 0, 0
        self.last_t = None
        self.last_row = None
        self.flash_until = 0.0

    def reader(self, port_name):
        try:
            import serial
            port = serial.Serial(port_name, 230400, timeout=0.02)
        except Exception as error:            # report and keep the window alive
            self.q.put(('error', f'{type(error).__name__}: {error}'))
            self.stop.set()
            return
        # The board can boot the passive v2 firmware (0x499F), the v4/v5 balance
        # firmware, or v6. All four streams validate CRC and version, so feeding
        # the same bytes to each picks the one that actually matches.
        streams = (protocol2.FrameStream(), protocol_state2.FrameStream(),
                   protocol4.FrameStream(), protocol5.FrameStream(),
                   protocol6.FrameStream())
        try:
            while not self.stop.is_set():
                try:
                    chunk = board.read_chunk(port)
                except Exception as error:      # the port can be stolen or unplugged
                    self.q.put(('error', f'串口读取失败（可能被其他程序占用或掉线）：{error}'))
                    break
                if chunk:
                    for stream in streams:
                        for row in stream.feed(chunk):
                            self.q.put(('row', row))
                else:
                    time.sleep(0.002)
        finally:
            try:
                port.close()
            except Exception:
                pass
            self.q.put(('closed', None))

    def push(self, row):
        now = time.monotonic()
        adc = int(row['adc_raw10'])
        valid = bool(row['adc_valid'])
        self.window.append((now, adc, valid, row['state']))
        self.samples += 1
        self.sum_code += adc
        self.min_code = min(self.min_code, adc)
        self.max_code = max(self.max_code, adc)
        if not valid or adc == 0:
            self.invalid += 1
            self.invalid_run += 1
            self.longest_invalid = max(self.longest_invalid, self.invalid_run)
            if not valid and adc != 0:
                self.flash_until = now + 0.4
        else:
            self.invalid_run = 0
        self.last_row = row
        self.last_t = now

    def poll(self):
        drained = 0
        while drained < 500:
            try:
                kind, payload = self.q.get_nowait()
            except queue.Empty:
                break
            drained += 1
            if kind == 'row':
                self.push(payload)
            elif kind == 'error':
                self.stop_reader()
                self.diag.config(text='串口错误：' + payload, foreground='#c00')
                self.valid.config(text='错误', background='#c33')
                self.footer.config(text='检查串口是否被面板/其他程序占用')
            elif kind == 'closed':
                if self.worker is not None:
                    self.stop_reader()
        now = time.monotonic()
        if self.last_row is not None:
            self.show(self.last_row, now)
        self.update_stats(now)
        self.plot()
        self.root.after(50, self.poll)

    def show(self, row, now):
        adc = int(row['adc_raw10'])
        valid = bool(row['adc_valid'])
        try:
            vref = float(self.vref.get())
        except ValueError:
            vref = 3.3
        self.code.config(text=f'{adc:4d}')
        bad = not valid or adc == 0
        if bad:
            self.valid.config(text='无效', background='#c33' if now < self.flash_until else '#a44')
        else:
            self.valid.config(text='有效', background='#2a8')
        self.volt.config(text=f'{adc / 1023 * vref:.3f} V')
        self.fields['tilt'].set('%+.2f°' % tilt_deg(adc) if not bad else '—')
        self.fields['zero'].set(f'{adc - TILT_ZERO:+d}')
        self.fields['enc'].set(str(row['enc_count']))
        quality = row.get('raw_estimator_quality', row.get('quality', '—'))
        self.fields['state'].set(f"{row['state']} / {quality}")
        age = row.get('adc_age_us', '—')
        self.fields['age'].set(f'{age} µs')
        self.fields['fault'].set(str(row.get('fault_bits', 0)))
        self.diag.config(**self.diagnosis())

    def diagnosis(self):
        if self.last_row is None:
            return dict(text='等待遥测…（USB 和 12V 都接好了吗？）', foreground='#666')
        if self.samples < 10:
            return dict(text='刚开始读取…', foreground='#666')
        valid_pct = 100 * (self.samples - self.invalid) / self.samples
        if self.min_code == 0 and self.max_code == 0:
            return dict(text='ADC 一直读到 0：12V 未接，或三芯线/电位器供电断开。', foreground='#c00')
        if valid_pct < 99:
            return dict(text=f'间歇接触：{100 - valid_pct:.1f}% 的帧无效/为 0。逐个弯折线缆和插头定位。',
                        foreground='#c30')
        span = self.max_code - self.min_code
        if span > 60:
            return dict(text=f'信号跳动较大（本次范围 {span} code）；如果摆杆没动，检查接触和噪声。',
                        foreground='#c60')
        return dict(text=f'信号稳定（本次 {self.min_code}…{self.max_code} code）。',
                    foreground='#2a8')

    def update_stats(self, now):
        if self.samples:
            self.stat['session'].set(f'{self.min_code}…{self.max_code}，均值 {self.sum_code / self.samples:.1f}')
            if self.samples:
                self.stat['quality'].set(f'{(self.samples - self.invalid) / self.samples * 100:.1f}% / {self.invalid}')
            self.stat['gap'].set(f'{self.longest_invalid * 0.01:.2f} s（约 {self.longest_invalid} 帧）')
        else:
            self.stat['session'].set('—')
            self.stat['quality'].set('—')
            self.stat['gap'].set('—')
        recent = [x for x in self.window if now - x[0] <= self.window_s]
        if recent:
            codes = [x[1] for x in recent]
            self.stat['window'].set(f'{min(codes)}…{max(codes)}，均值 {sum(codes) / len(codes):.1f}')
        else:
            self.stat['window'].set('—')

    def plot(self):
        c = self.canvas
        w = max(c.winfo_width(), 400)
        h = max(c.winfo_height(), 180)
        c.delete('all')
        left, right, top_m, bottom = 52, 14, 20, 24
        x0, x1, y0, y1 = left, w - right, top_m, h - bottom

        def yp(code):
            return y1 - (max(0, min(1023, code)) / 1023) * (y1 - y0)

        for code in range(0, 1024, 128):
            y = yp(code)
            c.create_line(x0, y, x1, y, fill='#eee')
            c.create_text(x0 - 6, y, anchor='e', text=str(code), fill='#999',
                          font=('Consolas', 8))
        for lo, hi, color, label in ((HANG_LOW, HANG_HIGH, '#2a8', '下垂窗'),
                                     (UPRIGHT_LOW, UPRIGHT_HIGH, '#36c', '竖直窗')):
            c.create_rectangle(x0, yp(hi), x1, yp(lo), outline='', fill=color, stipple='gray25')
            c.create_text(x1 - 4, yp((lo + hi) / 2), anchor='e', text=label, fill=color,
                          font=('Microsoft YaHei', 9))
        c.create_text(x0, 4, anchor='nw', text='ADC 原始码（0–1023），最近 30 s；红点=无效',
                      fill='#444', font=('Microsoft YaHei', 9))
        rows = list(self.window)
        if len(rows) < 2:
            return
        now = rows[-1][0]
        span = max(1e-6, rows[-1][0] - rows[0][0])
        pts, bad = [], []
        for t, adc, valid, _ in rows:
            x = x1 - (now - t) / 30.0 * (x1 - x0)
            if x < x0 - 2:
                continue
            if not valid or adc == 0:
                bad.append((x, yp(adc)))
            else:
                pts += [x, yp(adc)]
        if len(pts) >= 4:
            c.create_line(*pts, fill='#c33', width=1.5)
        for x, y in bad:
            c.create_oval(x - 2, y - 2, x + 2, y + 2, fill='#e00', outline='')
        c.create_text(x0 + 4, y1 + 14, anchor='w', text='← 30 s', fill='#888',
                      font=('Consolas', 8))

    def close(self):
        self.stop.set()
        if self.worker is not None:
            self.worker.join(timeout=2)
        self.root.destroy()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--port', default='COM8')
    args = ap.parse_args()
    import tkinter as tk
    root = tk.Tk()
    page = AdcPage(root, args.port)
    root.after(300, page.start)
    root.lift()
    root.attributes('-topmost', True)
    root.after(1200, lambda: root.attributes('-topmost', False))
    print('ADC page started', flush=True)
    root.mainloop()
    print('ADC page closed', flush=True)


if __name__ == '__main__':
    main()
