"""Offline v4 host tests; simulated serial I/O and a module-scoped fake clock."""

import contextlib
import csv
import importlib.util
import io
import json
from pathlib import Path
import sys
import tempfile
import types
import unittest
from unittest import mock

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
sys.path.insert(0, str(ROOT / "tests"))
import j280_balance_math as balance_math
import j280_balance_protocol as protocol
from test_j280_balance_protocol import reference_crc, telemetry_fixture, valid_values

SCRIPT = ROOT / "scripts" / "j280_balance.py"
host = None
if SCRIPT.exists():
    spec = importlib.util.spec_from_file_location("j280_balance", SCRIPT)
    host = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(host)


def configuration(**changes):
    values = valid_values(config_id=1, run_ms=1000, keepalive_ms=200,
                          travel_limit_count=100, speed_limit_count_s=5000,
                          adc_speed_limit_code_s=10000, hold_ms=200, wait_ms=5000,
                          enter_code=8, exit_code=45, comp_compare=175,
                          blend_compare=25, comp_gain_q20=7 << 20)
    result = dict(schema="j280-balance-config-v1", power="isolated",
                  purpose="offline fake-board test", values=values,
                  sources={key: "independent unittest fixture" for key in values})
    result.update(changes)
    return result


class Clock:
    def __init__(self, limit=5):
        self.now, self.limit = 0., limit

    def __call__(self):
        return self.now

    def advance(self, duration):
        self.now += duration
        if self.now > self.limit:
            raise AssertionError("host exceeded virtual deadline")


class ProfileTests(unittest.TestCase):
    def setUp(self):
        self.assertIsNotNone(host, "Implement scripts/j280_balance.py")
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.path = self.root / "profile.json"

    def save(self, config):
        self.path.write_text(json.dumps(config), encoding="utf-8")
        return self.path

    def test_isolated_profile_uses_default_design_and_exact_logic_defaults(self):
        coefficients = balance_math.design_coefficients()
        with mock.patch.object(host.balance_math, "design_coefficients", return_value=coefficients) as design:
            config = host.build_isolated_config(512.5, config_id=123)
        design.assert_called_once_with()
        expected = configuration()["values"]
        expected.update(config_id=123, coefficients=coefficients)
        self.assertEqual(config["values"], expected)
        self.assertEqual(config["schema"], "j280-balance-config-v1")
        self.assertEqual(config["power"], "isolated")
        self.assertEqual(protocol.validate_values(config["values"]), expected)
        self.assertEqual(set(config["sources"]), set(protocol.VALUE_FIELDS))
        self.assertTrue(all(isinstance(source, str) and source.strip()
                            for source in config["sources"].values()))
        self.assertIn("not a measurement", config["purpose"])
        self.assertIn("caller", config["sources"]["upright_zero_q16"])

    def test_upright_is_required_and_fractional_q16_is_explicit(self):
        with self.assertRaises(TypeError):
            host.build_isolated_config()
        with mock.patch.object(host.balance_math, "design_coefficients", return_value=[0] * 32):
            config = host.build_isolated_config(0.5 / 65536)
        self.assertEqual(config["values"]["upright_zero_q16"], 1)
        self.assertEqual(config["values"]["config_id"], 1)
        self.assertEqual(config["power"], "isolated")

    def test_upright_invalid_type_nonfinite_and_out_of_range_are_rejected(self):
        for value in (True, False, None, "512", float("nan"), float("inf"), -1, 1024):
            with self.subTest(value=value), self.assertRaises(ValueError):
                host.build_isolated_config(value)

    def test_builder_delegates_all_values_to_protocol_validation(self):
        with mock.patch.object(host.balance_math, "design_coefficients", return_value=[0] * 32), \
                mock.patch.object(host.protocol, "validate_values", side_effect=ValueError("protocol rejected")) as validate:
            with self.assertRaisesRegex(ValueError, "protocol rejected"):
                host.build_isolated_config(512)
        self.assertEqual(validate.call_count, 1)
        self.assertEqual(set(validate.call_args.args[0]), set(protocol.VALUE_FIELDS))
        for config_id in (True, 0, 65536):
            with mock.patch.object(host.balance_math, "design_coefficients", return_value=[0] * 32):
                with self.assertRaises(ValueError):
                    host.build_isolated_config(512, config_id=config_id)

    def test_load_config_validates_isolated_and_powered_labels(self):
        for power in ("isolated", "powered"):
            config = configuration(power=power)
            self.assertEqual(host.load_config(self.save(config)), config)

    def test_load_config_rejects_invalid_schema_power_purpose_and_root_type(self):
        invalid = ([], None, configuration(schema="v3"), configuration(power="unknown"),
                   configuration(power=True), configuration(purpose="  "), configuration(purpose=1))
        for config in invalid:
            with self.subTest(config=config), self.assertRaises(ValueError):
                host.load_config(self.save(config))

    def test_load_config_requires_exact_sources_for_every_value(self):
        missing = dict(configuration()["sources"])
        del missing["coefficients"]
        empty = dict(configuration()["sources"], upright_zero_q16=" ")
        for sources in (None, [], missing, empty, dict(configuration()["sources"], extra="source")):
            with self.subTest(sources=sources), self.assertRaises(ValueError):
                host.load_config(self.save(configuration(sources=sources)))

    def test_loader_delegates_numeric_validation(self):
        path = self.save(configuration())
        with mock.patch.object(host.protocol, "validate_values", side_effect=ValueError("invalid numbers")):
            with self.assertRaisesRegex(ValueError, "invalid numbers"):
                host.load_config(path)
        config = configuration()
        config["values"]["coefficients"][0] = True
        with self.assertRaises(ValueError):
            host.load_config(self.save(config))


if __name__ == "__main__":
    unittest.main()
