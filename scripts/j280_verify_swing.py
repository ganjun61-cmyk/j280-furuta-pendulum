#!/usr/bin/env python3
"""Verify the v6 firmware (swing-up, balance, arm trajectory, standalone) offline,
optionally build and archive. No board access.

Same procedure as j280_verify_balance.py (full Python/HDL regression, UART
evidence re-decoded, Gowin build, Setup/Hold/Recovery/Removal, sealed archive),
run on a private copy of that module with the v6 firmware identity.
"""
import importlib.util
from pathlib import Path
import sys

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
_spec = importlib.util.spec_from_file_location('j280_verify_swing_base', HERE / 'j280_verify_balance.py')
base = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(base)

import j280_builtin  # noqa: E402
import j280_v6_protocol  # noqa: E402

ROOT = base.ROOT
base.NAME = 'j280_swing'
base.RTL = ROOT / 'rtl/j280_swing'
base.SIM = ROOT / 'sim/j280_swing'
base.TEST_PATTERNS = ('test_j280_balance*.py', 'test_j280_swing*.py', 'test_j280_board_session.py',
                      'test_j280_backlash*.py', 'test_j280_panel.py', 'test_j280_traj*.py',
                      'test_j280_builtin.py', 'test_j280_v6*.py', 'test_j280_aux*.py')
base.EVIDENCE_ENV = 'J280_SWING_EVIDENCE_DIR'
base.ARCHIVE = ROOT / 'docs/data/swing-firmware-2026-10-19'
# Four integration scenarios (~15-35 min each) plus the unit suites.
base.TEST_TIMEOUT_S = 4 * 3600
base.FRAME_STREAM = j280_v6_protocol.FrameStream
base.SHARED = base.SHARED + [ROOT / 'rtl/j280_balance' / name for name in
                             ('j280_encoder_filtered.v', 'j280_balance_cmd_rx.v',
                              'j280_balance_core.v', 'j280_balance_pwm.v')]
base.SCENARIOS = {'stop': ('SAFE', 'stop_key', {'SWING', 'BALANCE'}),
                  'dead_zone': ('FAULT', 'angle_limit', {'SWING', 'BALANCE'}),
                  'blind_entry': ('FAULT', 'blind_entry', {'SWING'}),
                  'standalone': ('SAFE', 'stop_key', {'SWING', 'BALANCE'})}
base.PYTHON = base.PYTHON + ('j280_swing_math.py', 'j280_swing_protocol.py', 'j280_swingup_sim.py',
                             'j280_balance_board.py', 'j280_verify_swing.py', 'j280_backlash.py',
                             'j280_panel.py', 'j280_traj.py', 'j280_v6_protocol.py', 'j280_builtin.py')

base.EXTRA_CONFIGS = {j280_builtin.CONFIG_ID: j280_builtin.frames()[2]}
SHARED, SCENARIOS, EVIDENCE_FILES = base.SHARED, base.SCENARIOS, base.EVIDENCE_FILES
source_files, check_evidence, archive_verified_run = base.source_files, base.check_evidence, base.archive_verified_run
main = base.main

if __name__ == '__main__':
    sys.exit(main())
