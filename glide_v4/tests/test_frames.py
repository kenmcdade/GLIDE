"""Unit tests for LVLH frames."""

import unittest
import numpy as np

from dynamics.constants import R_EARTH
from dynamics.orbit import coe_to_state
from dynamics.frames import lvlh_basis, eci_to_lvlh, lvlh_rel_to_eci


class TestFrames(unittest.TestCase):
    def setUp(self):
        a = R_EARTH + 500000.0
        r_ref, v_ref = coe_to_state(a, 0.0, np.deg2rad(51.6), 0.0, 0.0, 0.0)
        self.r_ref = r_ref
        self.v_ref = v_ref

    def test_basis_orthonormal(self):
        C = lvlh_basis(self.r_ref, self.v_ref)
        I = C @ C.T
        self.assertTrue(np.allclose(I, np.eye(3), atol=1e-9))

    def test_round_trip(self):
        r_lvh = np.array([100.0, -50.0, 20.0])
        v_lvh = np.array([0.1, -0.05, 0.02])
        r_eci, v_eci = lvlh_rel_to_eci(r_lvh, v_lvh, self.r_ref, self.v_ref)
        r_lvh_2, v_lvh_2 = eci_to_lvlh(r_eci, v_eci, self.r_ref, self.v_ref)
        self.assertTrue(np.allclose(r_lvh, r_lvh_2, atol=1e-6))
        self.assertTrue(np.allclose(v_lvh, v_lvh_2, atol=1e-6))


if __name__ == "__main__":
    unittest.main()
