"""Self-checking v4 supervisor vectors, independent of host packing/math code.

Only Icarus is used. Builds/logs live in the existing ASCII J280 tool root;
there is no serial, programmer, vendor compiler or hardware access.
"""
import os
from pathlib import Path
import shutil
import struct
import subprocess
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]
LOCAL = Path(os.environ.get("LOCALAPPDATA", str(Path.home())))


def _iverilog_bin():
    candidates = [Path(os.environ[k]) for k in ("J280_IVERILOG_BIN", "IVERILOG_BIN")
                  if k in os.environ]
    candidates.append(LOCAL / "J280Tools/iverilog-msys2/ucrt64/bin")
    found = shutil.which("iverilog")
    if found:
        candidates.append(Path(found).parent)
    return next((p for p in candidates if (p / "iverilog.exe").is_file()), candidates[0])


BIN = _iverilog_bin()
BUILD = Path(tempfile.gettempdir()) / "J280Build/balance-guard-unit"
SOURCES = (ROOT / "rtl/j280_balance/j280_balance_guard.v",
           ROOT / "sim/j280_balance/tb_balance_guard.v")
SAFE, ARMED, CAPTURE, BALANCE, FAULT = 0, 1, 2, 3, 255


def config(**changes):
    """Explicit wire offsets (payload starts at wire byte 2), no production imports."""
    values = dict(version=4, command=1, config_id=0x1234, algorithm_id=1,
                  run_ms=1000, keepalive_ms=500, travel=100, speed=5000,
                  adc_speed=10000, maximum=1000, hold_ms=200, wait_ms=1000,
                  zero=500 << 16, enter=9, exit=57, comp=175, blend=25,
                  gain=7 << 20, coefficients=list(range(-16, 16)), sample=2000)
    values.update(changes)
    data = bytearray(188)
    fields = ((0, "B", "version"), (1, "B", "command"), (2, "H", "config_id"),
              (4, "H", "algorithm_id"), (6, "H", "run_ms"), (8, "H", "keepalive_ms"),
              (10, "H", "travel"), (12, "H", "speed"), (14, "H", "adc_speed"),
              (16, "H", "maximum"), (18, "H", "hold_ms"), (20, "H", "wait_ms"),
              (22, "i", "zero"), (26, "H", "enter"), (28, "H", "exit"),
              (30, "H", "comp"), (32, "H", "blend"), (34, "i", "gain"),
              (166, "H", "sample"))
    for offset, fmt, key in fields:
        struct.pack_into("<" + fmt, data, offset, values[key])
    struct.pack_into("<32i", data, 38, *values["coefficients"])
    return data


class Vectors:
    def __init__(self, version=4):
        self.lines = []
        self.version = version

    def add(self, *args):
        self.lines.append(" ".join(map(str, args)))
        return self

    def wait(self, clocks):
        return self.add("W", clocks)

    def set(self, name, value):
        return self.add("I", name, value)

    def command(self, payload, crc=0xBEEF):
        return self.add("C", bytes(payload)[::-1].hex(), f"{crc:04x}")

    def control(self, command=2, dirty=None):
        data = bytearray(188)
        data[:2] = bytes((self.version, command))
        if dirty is not None:
            data[dirty] = 1
        return self.command(data)

    def load(self, **fields):
        return self.command(config(**dict(dict(version=self.version), **fields))).wait(34)

    def expect(self, state=SAFE, reason=0, loaded=1, rx=-1):
        return self.add("E", state, reason, loaded, rx)

    def armed(self, **fields):
        return self.wait(45).load(**fields).set("raw", 1).set("stable", 1).wait(1).expect(ARMED)

    def capture(self, **fields):
        return self.armed(**fields).set("start", 1).wait(1).set("start", 0).expect(CAPTURE)

    def balance(self, **fields):
        return self.capture(**dict(dict(hold_ms=2), **fields)).wait(90).expect(BALANCE)


class GuardHarness(unittest.TestCase):
    """Compiles one guard bench per MIN_HOLD_MS; subclasses choose the bench."""
    TB = "tb_balance_guard"
    SOURCES = SOURCES
    PARAMS = {}

    @classmethod
    def setUpClass(cls):
        for source in cls.SOURCES:
            if not source.is_file():
                raise AssertionError(f"missing guard source (RED): {source}")
        if not (BIN / "iverilog.exe").is_file():
            raise AssertionError(f"missing existing Icarus tool root: {BIN}")
        BUILD.mkdir(parents=True, exist_ok=True)
        cls.run_dir = Path(tempfile.mkdtemp(prefix="vectors-", dir=BUILD))
        if not str(cls.run_dir).isascii():
            raise AssertionError(cls.run_dir)
        cls.env = dict(os.environ, PATH=str(BIN) + os.pathsep + os.environ.get("PATH", ""))
        for source in cls.SOURCES:
            shutil.copy2(source, cls.run_dir / source.name)
        for minimum in (2, 200):
            result = subprocess.run([str(BIN / "iverilog.exe"), "-g2001", "-Wall",
                                     "-s", cls.TB, f"-P{cls.TB}.MIN_HOLD_MS={minimum}",
                                     *[f"-P{cls.TB}.{k}={v}" for k, v in cls.PARAMS.items()],
                                     "-o", f"guard{minimum}.vvp", *[s.name for s in cls.SOURCES]],
                                    cwd=cls.run_dir, env=cls.env, capture_output=True,
                                    text=True, timeout=60)
            log = result.stdout + result.stderr
            (cls.run_dir / f"compile{minimum}.log").write_text(log, encoding="utf-8")
            if result.returncode or log.strip():
                raise AssertionError(f"{cls.run_dir}\n{log}")

    def simulate(self, vectors, minimum=2):
        name = f"{self._testMethodName}-{len(list(self.run_dir.glob('*.vec')))}"
        path = self.run_dir / (name + ".vec")
        path.write_text("\n".join(vectors.lines + ["Q"]) + "\n", encoding="ascii")
        result = subprocess.run([str(BIN / "vvp.exe"), f"guard{minimum}.vvp",
                                 f"+VECTORS={path.name}"], cwd=self.run_dir, env=self.env,
                                capture_output=True, text=True, timeout=30)
        log = result.stdout + result.stderr
        (self.run_dir / (name + ".log")).write_text(log, encoding="utf-8")
        self.assertEqual(result.returncode, 0, f"{path}\n{log}")
        self.assertNotIn("FAIL", log, f"{path}\n{log}")
        self.assertIn(f"PASS {self.TB}", log, f"{path}\n{log}")


class BalanceGuardRTLTests(GuardHarness):
    def test_serial_validation_atomic_commit_full_payload_and_id_crc(self):
        v = Vectors().wait(45).load().expect(rx=1)
        v.add("P", 0x1234, 0xBEEF, 1000, 175, 25, 7 << 20, 500 << 16)
        v.add("K", config()[38:166][::-1].hex())
        replacement = config(config_id=9, maximum=900, comp=0, gain=0,
                             zero=(511 << 16) + 32768, coefficients=[128 << 20] * 31 + [-128 << 20])
        v.command(replacement, 0xCAFE).expect(loaded=0, rx=0)
        v.set("raw", 1).set("stable", 1).wait(32).expect(loaded=0)
        # All old public parameters must remain unchanged throughout validation.
        v.add("P", 0, 0, 1000, 175, 25, 7 << 20, 500 << 16)
        v.wait(1).expect(loaded=1, rx=1)
        v.add("P", 9, 0xCAFE, 900, 0, 25, 0, (511 << 16) + 32768)
        v.add("K", replacement[38:166][::-1].hex()).wait(1).expect(ARMED)
        self.simulate(v)

    def test_all_scalar_bounds_and_compensation_coherence(self):
        bad = [dict(version=3), dict(config_id=0), dict(algorithm_id=0), dict(algorithm_id=2),
               dict(run_ms=0), dict(run_ms=30001), dict(keepalive_ms=0), dict(keepalive_ms=501),
               dict(travel=0), dict(travel=301), dict(speed=0), dict(speed=10001),
               dict(adc_speed=0), dict(adc_speed=30001), dict(maximum=0), dict(maximum=1126),
               dict(hold_ms=199), dict(hold_ms=1001), dict(wait_ms=201), dict(wait_ms=10001),
               dict(zero=-1), dict(zero=(1023 << 16) + 1), dict(enter=10), dict(exit=0),
               dict(exit=58), dict(enter=9, exit=9), dict(comp=251), dict(comp=176),
               dict(maximum=174), dict(blend=24), dict(blend=251), dict(gain=-1),
               dict(gain=(10 << 20) + 1), dict(gain=(7 << 20) + 1), dict(sample=1999)]
        v = Vectors().wait(45)
        for fields in bad:
            v.load().expect(rx=1).command(config(**fields)).expect(loaded=0)
            v.wait(34).expect(loaded=0, rx=5)
        self.simulate(v, minimum=200)

    def test_coefficient_every_index_last_reserved_bits_and_min_int(self):
        v = Vectors().wait(45)
        for index in range(32):
            coeffs = [0] * 32
            coeffs[index] = (128 << 20) + 1 if index % 2 else -(128 << 20) - 1
            v.load().command(config(coefficients=coeffs)).wait(34).expect(loaded=0, rx=5)
        for bad in (-(1 << 31), (1 << 31) - 1):
            v.load(coefficients=[0] * 31 + [bad]).expect(loaded=0, rx=5)
        for offset in range(168, 188):
            data = config()
            data[offset] = 0x80
            v.load().command(data).wait(34).expect(loaded=0, rx=5)
        self.simulate(v)

    def test_inclusive_boundaries_and_signed64_compensation_tolerance(self):
        v = Vectors().wait(45)
        for fields in (dict(enter=0, exit=1, comp=0, gain=0, maximum=1, zero=0),
                       dict(run_ms=30000, travel=200, speed=10000, adc_speed=30000,
                            hold_ms=1000, wait_ms=1002, zero=1023 << 16, enter=9, exit=57),
                       dict(comp=250, blend=25, gain=10 << 20),
                       dict(comp=1, blend=250, gain=4194),
                       dict(comp=1, blend=25, gain=41943),
                       dict(comp=13, blend=25, gain=545259)):
            v.load(**fields).expect(rx=1)
        # 4194*250 - 2**20 = -76 passes (tolerance 125); 4195 gives +174.
        v.load(comp=1, blend=250, gain=4195).expect(loaded=0, rx=5)
        self.simulate(v, minimum=200)

    def test_pending_restart_abort_and_no_mixed_payload(self):
        v = Vectors().wait(45).load().command(config(config_id=2)).wait(15)
        v.command(config(config_id=3, maximum=600)).wait(32).expect(loaded=0)
        v.wait(1).expect(rx=1).add("P", 3, 0xBEEF, 600, 175, 25, 7 << 20, 500 << 16)
        v.command(config(config_id=4)).wait(20).control(3).wait(40).expect(loaded=0, rx=3)
        v.command(config(config_id=5)).wait(32).control(3).wait(2).expect(loaded=0, rx=3)
        v.command(config(config_id=6)).wait(5).control(3, dirty=187).wait(27).expect(rx=1)
        v.add("P", 6, 0xBEEF, 1000, 175, 25, 7 << 20, 500 << 16)
        self.simulate(v)
        # Invalid CONFIG on the otherwise-ready arm edge must invalidate the
        # old configuration immediately and never enter ARMED while checking.
        v = Vectors().wait(45).load().set("raw", 1).set("stable", 1)
        self.simulate(v.command(config(config_id=0)).expect(loaded=0).wait(40).expect(loaded=0, rx=5))

    def test_pending_bus_changes_do_not_change_staged_payload_crc_or_commit_position(self):
        v = Vectors().wait(45).set("position", 123).command(config(config_id=7), 0xACE1)
        # Change the live bus without cmd_valid while staged checks are running.
        v.wait(15).add("U", config(config_id=9, maximum=500)[::-1].hex(), "dead")
        v.set("position", 456).wait(18).expect(rx=1)
        v.add("P", 7, 0xACE1, 1000, 175, 25, 7 << 20, 500 << 16).add("T", 0, 0)
        v.set("position", 556).set("raw", 1).set("stable", 1).wait(2).expect(ARMED)
        self.simulate(v.set("position", 557).wait(1).expect(FAULT, 18, 0))

    def test_non_safe_config_rejected_keeps_valid_parameters(self):
        for stage in (ARMED, CAPTURE, BALANCE):
            v = Vectors()
            if stage == ARMED:
                v.armed()
            elif stage == CAPTURE:
                v.capture()
            else:
                v.balance()
            v.command(config(config_id=2, coefficients=[-(1 << 31)] * 32)).wait(35)
            v.expect(stage, rx=4).add("P", 0x1234, 0xBEEF, 1000, 175, 25, 7 << 20, 500 << 16)
            self.simulate(v)

    def test_control_reserved_entire_payload_frame_counter_and_status(self):
        v = Vectors().armed()
        for command in (2, 3):
            for offset in (2, 100, 187):
                v.control(command, dirty=offset).expect(ARMED, rx=5)
        v.control(9).expect(ARMED, rx=6).set("error", 1).wait(1).set("error", 0)
        v.expect(ARMED, rx=7).control(2).expect(ARMED, rx=2)
        # 1 CONFIG + 6 malformed controls + unknown + KA = 9 CRC-correct frames.
        v.add("H", 9).control(3).expect(SAFE, 4, 0, 3).add("H", 10)
        for _ in range(8):
            v.control(2)
        v.add("H", 2)
        self.simulate(v)

    def test_no_autostart_keys_off_seen_and_stable_on_required(self):
        for name, value in (("released", 0), ("quality", 3), ("age", 101),
                            ("faults", 1), ("stop", 1), ("est_on", 0)):
            v = Vectors().set(name, value).wait(120).load().set("raw", 1).set("stable", 1)
            v.wait(20).expect(loaded=1)
            self.simulate(v)
        v = Vectors().set("raw", 1).set("stable", 1).wait(45).load().wait(10).expect()
        v.set("raw", 0).set("stable", 0).wait(1).set("raw", 1).wait(5).expect()
        v.set("stable", 1).wait(1).expect(ARMED).wait(80).expect(ARMED)
        self.simulate(v)

    def test_200ms_requires_101_snapshots_and_capture_starts_after_sw1(self):
        v = Vectors().armed()
        for _ in range(5):
            v.wait(800).control(2)
        v.expect(ARMED)
        v.set("samples_on", 0).set("start", 1).wait(1).set("start", 0).expect(CAPTURE)
        for index in range(100):
            v.add("N").expect(CAPTURE)
            if index % 20 == 19:
                v.control(2).wait(38)
            else:
                v.wait(39)
        v.add("N").expect(BALANCE).add("T", 0, 0).wait(1)
        self.simulate(v, minimum=200)

    def test_capture_uses_enter_threshold_and_restarts_hold_after_excursion(self):
        v = Vectors().capture(hold_ms=2).set("adc", 510).wait(90).expect(CAPTURE)
        v.set("samples_on", 0).set("adc", 509).add("N").expect(CAPTURE).wait(39)
        v.set("adc", 510).add("N").expect(CAPTURE).wait(39)
        v.set("adc", 491).add("N").expect(CAPTURE).wait(39)
        v.add("N").expect(BALANCE)
        self.simulate(v)

    def test_capture_timeout_arm_timeout_and_complete_consume_config(self):
        self.simulate(Vectors().set("adc", 510).capture(hold_ms=2, wait_ms=4).wait(100).expect(SAFE, 6, 0))
        v = Vectors().armed()
        for _ in range(7):
            v.wait(800).control(2)
        v.wait(500).expect(SAFE, 5, 0)
        self.simulate(v)
        v = Vectors().balance(run_ms=4).wait(100).expect(SAFE, 1, 0)
        v.load(hold_ms=2).wait(100).expect(SAFE, 1, 1)
        v.set("start", 1).wait(1).set("start", 0).expect(SAFE, 1, 1)
        v.set("stable", 0).set("raw", 0).wait(1).set("stable", 1).set("raw", 1)
        v.wait(1).expect(ARMED, 1)
        self.simulate(v)

    def test_travel_safe_reference_distinct_from_takeover_and_signed33(self):
        for stage in (ARMED, CAPTURE, BALANCE):
            v = Vectors().set("position", 1000).armed(hold_ms=2)
            if stage != ARMED:
                v.set("position", 1060).set("start", 1).wait(1).set("start", 0)
                if stage == BALANCE:
                    v.wait(90).expect(BALANCE).add("T", 1060, 60)
            v.set("position", 1100).wait(1).expect(stage)
            v.set("position", 1101).wait(1).expect(FAULT, 18, 0)
            self.simulate(v)
        for start, end in ((2147483647, -2147483648), (-2147483648, 2147483647),
                           (0, -101)):
            v = Vectors().set("position", start).armed().set("position", end).wait(1)
            self.simulate(v.expect(FAULT, 18, 0))

    def test_all_normal_stops_active_and_registered_output_drop(self):
        for stage in (ARMED, CAPTURE, BALANCE):
            for kind, reason in (("stop", 2), ("raw", 3), ("abort", 4)):
                v = Vectors()
                {ARMED: v.armed, CAPTURE: v.capture, BALANCE: v.balance}[stage]()
                if kind == "abort":
                    v.control(3)
                else:
                    v.set(kind, 0 if kind == "raw" else 1).add("A", int(stage == BALANCE)).wait(1)
                self.simulate(v.expect(SAFE, reason, 0))

    def test_direct_sensor_fault_age_and_current_bad_quality(self):
        for stage in (ARMED, CAPTURE, BALANCE):
            for name, value in (("age", 101), ("faults", 1), ("faults", 2), ("faults", 4)):
                v = Vectors()
                {ARMED: v.armed, CAPTURE: v.capture, BALANCE: v.balance}[stage]()
                self.simulate(v.set(name, value).wait(1).expect(FAULT, 16, 0))
        v = Vectors().balance().set("quality", 3).add("J", "est").expect(FAULT, 16, 0)
        self.simulate(v)

    def test_raw_and_core_speed_limits_both_signs_including_int_min(self):
        for source, limit in (("ev", 5000), ("av", 10000), ("cv", 5000), ("ca", 10000)):
            for sign in (-1, 1):
                v = Vectors().balance().set(source, sign * (limit << 16))
                v.add("J", "core" if source.startswith("c") else "est").expect(BALANCE)
                v.set(source, sign * ((limit << 16) + 1))
                v.add("J", "core" if source.startswith("c") else "est").expect(FAULT, 19, 0)
                self.simulate(v)
            v = Vectors().balance().set(source, -(1 << 31))
            self.simulate(v.add("J", "core" if source.startswith("c") else "est").expect(FAULT, 19, 0))
        self.simulate(Vectors().armed().set("ev", 5001 << 16).add("J", "est").expect(FAULT, 19, 0))

    def test_exit_angle_only_in_balance_and_fractional_q16_boundary(self):
        v = Vectors().capture(hold_ms=2).set("adc", 600).wait(90).expect(CAPTURE)
        v.set("adc", 500).wait(90).expect(BALANCE).set("adc", 557).wait(1).expect(BALANCE)
        v.set("adc", 558).wait(1).expect(FAULT, 21, 0)
        self.simulate(v)
        v = Vectors().balance(zero=(500 << 16) + 1).set("adc", 443).wait(1)
        self.simulate(v.expect(FAULT, 21, 0))

    def test_raw_core_stale_independent_and_core_fresh_on_balance_entry(self):
        v = Vectors().armed().wait(160).expect(ARMED).add("F", 1, 1, 0)
        v.set("est_on", 0).wait(101).expect(FAULT, 17, 0)
        self.simulate(v)
        v = Vectors().capture(hold_ms=2).set("core_on", 0).wait(90).expect(BALANCE)
        v.add("F", 1, 1, 1).wait(101).expect(FAULT, 17, 0)
        self.simulate(v)
        v = Vectors().balance().set("core_on", 0).wait(70).add("J", "core")
        v.wait(99).expect(BALANCE).wait(3).expect(FAULT, 17, 0)
        self.simulate(v)

    def test_tick_gap_duplicate_rollback_and_uint64_wrap_fault_without_takeover(self):
        for first, second in ((100000, 104000), (100000, 100000), (100000, 98000),
                              (18446744073709550616, 1000)):
            v = Vectors().set("samples_on", 0).capture(hold_ms=2).add("S", first).expect(CAPTURE)
            v.wait(39).add("S", second).expect(FAULT, 17, 0).add("A", 0)
            self.simulate(v)
        v = Vectors().set("samples_on", 0).armed().add("S", 100000).expect(ARMED).wait(39).add("S", 99999)
        self.simulate(v.expect(FAULT, 17, 0))
        v = Vectors().balance().set("samples_on", 0).add("S", 1)
        self.simulate(v.expect(FAULT, 17, 0))

    def test_every_crc_correct_command_refreshes_ka_but_frame_error_does_not(self):
        for command, dirty in ((2, None), (2, 187), (3, 187), (9, None), (1, None)):
            v = Vectors().armed(keepalive_ms=8).wait(100)
            if command == 1:
                v.command(config(config_id=2))
            else:
                v.control(command, dirty)
            v.wait(100).expect(ARMED).wait(65).expect(FAULT, 20, 0)
            self.simulate(v)
        v = Vectors().balance(keepalive_ms=10).wait(50).set("error", 1).wait(1).set("error", 0)
        self.simulate(v.wait(80).expect(FAULT, 20, 0))

    def test_numeric_overrun_pwm_fault_latch_reset_and_reject_config_in_fault(self):
        for name, reason in (("numeric", 22), ("overrun", 23), ("reject", 24)):
            v = Vectors().balance().set(name, 1).wait(1).expect(FAULT, reason, 0)
            v.set(name, 0).control(3).load().wait(120).expect(FAULT, reason, 0, 4)
            v.set("reset", 0).add("A", 0).wait(2).set("reset", 1).wait(45)
            v.expect(SAFE, 0, 0).load().wait(5).expect()
            self.simulate(v)

    def test_simultaneous_stop_priority_chain(self):
        # Adjacent pairs prove the complete priority order, with fresh KA/frame.
        pairs = [(("stop", 1), ("raw", 0), 2), (("raw", 0), ("faults", 1), 3),
                 (("faults", 1), ("bad_tick", 1), 16),
                 (("bad_tick", 1), ("position", 101), 17),
                 (("position", 101), ("ev", 5001 << 16), 18),
                 (("ev", 5001 << 16), ("adc", 558), 19),
                 (("adc", 558), ("numeric", 1), 21),
                 (("numeric", 1), ("overrun", 1), 23),
                 (("overrun", 1), ("reject", 1), 23),
                 (("reject", 1), ("abort", 1), 24)]
        for a, b, reason in pairs:
            v = Vectors().balance()
            for name, value in (a, b):
                v.set(name, value)
            v.add("J", "all").expect(SAFE if reason in (2, 3) else FAULT, reason, 0)
            self.simulate(v)
        # The state counter reaches a timeout on the preceding edge. These
        # vectors assert the still-active state, then apply competing conditions
        # on the condition clock, so an already-completed stop cannot pass.
        v = Vectors().armed(keepalive_ms=8).wait(125).expect(ARMED)
        v.set("numeric", 1).wait(1)
        self.simulate(v.expect(FAULT, 20, 0))

    def test_keepalive_timeout_latches_until_physical_reset(self):
        v=Vectors().armed(keepalive_ms=8).wait(200).expect(FAULT,20,0)
        v.control(2).load().wait(60).expect(FAULT,20,0,4)
        self.simulate(v)
        self.simulate(Vectors().balance().set("numeric", 1).control(3).expect(FAULT, 22, 0))
        v = Vectors().balance(keepalive_ms=10).wait(74).expect(BALANCE)
        self.simulate(v.set("adc", 558).wait(1).expect(FAULT, 21, 0))
        v = Vectors().balance(run_ms=4).wait(66).expect(BALANCE)
        self.simulate(v.control(3).expect(SAFE, 4, 0))
        v = Vectors().capture(hold_ms=2, wait_ms=4).set("adc", 510).wait(40)
        v.set("samples_on", 0).set("adc", 500).add("N").expect(CAPTURE).wait(39)
        self.simulate(v.add("N").expect(SAFE, 6, 0))

    def test_exact_adc_age_limit_and_snapshot_at_start_does_not_qualify(self):
        v = Vectors().set("age", 100).armed(hold_ms=2).set("samples_on", 0)
        # The snapshot coinciding with SW1 is still in ARMED. Two subsequent
        # qualifying samples are necessary even for the 2ms simulation hold.
        v.set("start", 1).add("N").set("start", 0).expect(CAPTURE).wait(39)
        v.add("N").expect(CAPTURE).wait(39).add("N").expect(BALANCE)
        self.simulate(v.set("age", 101).wait(1).expect(FAULT, 16, 0))

    def test_safety_delta_saturates_diagnostic_and_theta_reference_survives_exit(self):
        v = Vectors().set("position", 100).balance().add("T", 100, 0)
        self.simulate(v.set("position", 201).wait(1).expect(FAULT, 18, 0).add("T", 100, 101))
        for start, end, delta in ((2147483647, -2147483648, -2147483648),
                                  (-2147483648, 2147483647, 2147483647)):
            v = Vectors().set("position", start).armed().set("position", end).wait(1)
            self.simulate(v.expect(FAULT, 18, 0).add("T", 0, delta))

    def test_reset_cancels_pending_and_active_and_requires_fresh_off(self):
        for active in (False, True):
            v = Vectors()
            if active:
                v.balance()
            else:
                v.wait(45).command(config()).wait(10)
            v.set("reset", 0).add("A", 0).wait(2).set("reset", 1).wait(45).expect(loaded=0)
            self.simulate(v)


if __name__ == "__main__":
    unittest.main()
