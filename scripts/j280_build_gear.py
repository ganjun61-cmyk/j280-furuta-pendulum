"""Build the J280 v6 swing/balance firmware with Gowin gw_sh.

Usage: python scripts/j280_build_gear.py
Set J280_GOWIN to gw_sh.exe if it is not in the default Gowin install path.
"""
import os, shutil, subprocess, time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
BUILD = ROOT / 'build' / 'gear'
NAME = 'j280_swing'
GOWIN = Path(os.environ.get(
    'J280_GOWIN',
    r'C:\Gowin\Gowin_V1.9.12.03_x64\Gowin_V1.9.12.03_x64\IDE\bin\gw_sh.exe'))
DIRS = [ROOT/'rtl/j280_measure', ROOT/'rtl/j280_balance', ROOT/'rtl/j280_state',
        ROOT/'rtl/j280_motor_test', ROOT/'rtl/j280_swing']
FILES = ['j280_adc_capture.v', 'j280_encoder_filtered.v', 'j280_uart_tx.v',
         'j280_signed_divider.v', 'j280_raw_estimator.v', 'j280_state_packet_tx.v',
         'j280_uart_rx.v', 'j280_local_inputs.v', 'j280_balance_cmd_rx.v',
         'j280_balance_core.v', 'j280_balance_pwm.v', 'j280_alpha_svf.v',
         'j280_keypad.v', 'j280_swing_tables.v', 'j280_swing_core.v',
         'j280_traj.v', 'j280_aux_inputs.v',
         'j280_swing_guard.v', 'j280_phase_lut16.v', 'j280_swing_hyb_top.v',
         'j280_swing.cst', 'j280_swing.sdc']


def find(name):
    for d in DIRS:
        p = d / name
        if p.is_file():
            return p
    raise FileNotFoundError(name)


def stage():
    BUILD.mkdir(parents=True, exist_ok=True)
    for old in BUILD.iterdir():
        if old.name in ('impl', 'run.tcl', 'build-console.log'):
            if old.is_dir():
                shutil.rmtree(old, ignore_errors=True)
            else:
                old.unlink()
    for name in FILES:
        shutil.copy2(find(name), BUILD/name)
    tcl = ['set_device -name GW2A-55C GW2A-LV55PG484C8/I7']
    for name in FILES:
        tcl.append('add_file ' + name)
    tcl += ['set_option -top_module j280_swing_hyb_top',
            'set_option -use_sspi_as_gpio 1',
            'set_option -place_option 2',
            'set_option -output_base_name ' + NAME,
            'run all']
    (BUILD/'run.tcl').write_text('\n'.join(tcl) + '\n', encoding='ascii')


def run():
    pnr = BUILD/'impl/pnr'
    log = BUILD/'build-console.log'
    marker = f'{NAME}.tr.html" completed'
    start = time.monotonic(); stuck = False
    with log.open('wb') as sink:
        proc = subprocess.Popen([str(GOWIN), 'run.tcl'], cwd=str(BUILD),
                                stdout=sink, stderr=subprocess.STDOUT)
        while proc.poll() is None:
            time.sleep(2)
            if time.monotonic() - start > 3600:
                proc.kill(); raise SystemExit('gw_sh timeout')
            pl = pnr/f'{NAME}.log'
            if pl.is_file() and marker in pl.read_text(encoding='utf-8', errors='replace'):
                if time.time() - max(p.stat().st_mtime for p in pnr.iterdir()) > 90:
                    proc.kill(); proc.wait(); stuck = True
    text = log.read_text(encoding='utf-8', errors='replace')
    print('rc', proc.returncode, 'stuck', stuck)
    print(text[-2500:])
    fs = pnr/f'{NAME}.fs'
    print('fs', fs, fs.exists(), fs.stat().st_size if fs.exists() else None)
    rpt = pnr/f'{NAME}.rpt.txt'
    if rpt.exists():
        for line in rpt.read_text(encoding='utf-8', errors='replace').splitlines():
            if 'Logic' in line or 'Register' in line or 'DSP' in line:
                print('RPT', line.strip())
    tl = pnr/f'{NAME}.timing_paths'
    if tl.exists():
        print('timing_paths', tl)


if __name__ == '__main__':
    stage(); run()
