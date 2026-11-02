"""Filtered quadrature counter for the balance firmware (Icarus self-check)."""
from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'tests'))
from test_j280_motor_test_rtl import simulate  # noqa: E402

SOURCES = [ROOT / 'rtl/j280_balance/j280_encoder_filtered.v', ROOT / 'sim/j280_balance/tb_encoder_filtered.v']


class FilteredEncoderTests(unittest.TestCase):
    def test_glitch_rejection_real_edges_and_reset(self):
        simulate('tb_encoder_filtered', SOURCES)

    def test_filter_length_is_respected_at_the_boundary(self):
        # A 120-clock filter: the testbench's glitches stay two clocks shorter than the filter.
        simulate('tb_encoder_filtered', SOURCES, params=dict(FILTER=120))


if __name__ == '__main__':
    unittest.main()
