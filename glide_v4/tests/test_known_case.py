"""Known-case sanity checks."""

import unittest
import numpy as np

from dynamics.constants import R_EARTH
from dynamics.orbit import coe_to_state
from dynamics.frames import eci_to_lvlh
from modes.gates import GateTracker


class TestKnownCase(unittest.TestCase):
    def test_zero_relative(self):
        a = R_EARTH + 500000.0
        r_ref, v_ref = coe_to_state(a, 0.0, np.deg2rad(51.6), 0.0, 0.0, 0.0)
        r_rel = r_ref - r_ref
        v_rel = v_ref - v_ref
        r_lvh, v_lvh = eci_to_lvlh(r_rel, v_rel, r_ref, v_ref)
        self.assertTrue(np.allclose(r_lvh, np.zeros(3), atol=1e-9))
        self.assertTrue(np.allclose(v_lvh, np.zeros(3), atol=1e-9))

    def test_gate_crossing(self):
        thresholds = {"R_ACQ": 250.0, "R_COR": 10.0, "R_PRE": 2.0, "R_LATCH": 0.5}
        speed_limits = {"R_LATCH": {"hard_max": 0.10}}
        gate = GateTracker(thresholds, speed_limits)
        events = gate.update(0.4, 0.05, 0.0)
        self.assertIn("R_LATCH", events)


if __name__ == "__main__":
    unittest.main()
