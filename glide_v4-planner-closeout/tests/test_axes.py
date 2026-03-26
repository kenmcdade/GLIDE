"""Axis definition and sign tests."""

import unittest
import numpy as np

from dynamics.constants import R_EARTH
from dynamics.orbit import coe_to_state
from dynamics.frames import lvlh_basis, eci_to_lvlh, lvlh_to_eci_vector
from dynamics.drag import drag_accel
from dynamics.medt import medt_accel, dipole_b_field_eci


class TestAxes(unittest.TestCase):
    def setUp(self):
        a = R_EARTH + 500000.0
        self.r_ref, self.v_ref = coe_to_state(a, 0.0, np.deg2rad(51.6), 0.0, 0.0, 0.0)

    def test_rtn_axes_orientation(self):
        C = lvlh_basis(self.r_ref, self.v_ref)
        r_hat = C[0, :]
        t_hat = C[1, :]
        n_hat = C[2, :]
        h = np.cross(self.r_ref, self.v_ref)
        self.assertGreater(np.dot(r_hat, self.r_ref), 0.0)
        self.assertGreater(np.dot(t_hat, self.v_ref), 0.0)
        self.assertGreater(np.dot(n_hat, h), 0.0)

    def test_plus_t_accel_increases_t_position(self):
        dt = 1.0
        a_t = 1.0e-4
        a_lvh = np.array([0.0, a_t, 0.0])
        a_eci = lvlh_to_eci_vector(a_lvh, self.r_ref, self.v_ref)

        # Apply to payload only for a short step.
        r_payload = self.r_ref.copy() + self.v_ref * dt + 0.5 * a_eci * dt * dt
        v_payload = self.v_ref.copy() + a_eci * dt
        r_ref_new = self.r_ref.copy() + self.v_ref * dt
        v_ref_new = self.v_ref.copy()

        r_rel = r_payload - r_ref_new
        v_rel = v_payload - v_ref_new
        r_lvh, v_lvh = eci_to_lvlh(r_rel, v_rel, self.r_ref, self.v_ref)

        # T component of relative position and velocity should be positive.
        self.assertGreater(r_lvh[1], 0.0)
        self.assertGreater(v_lvh[1], 0.0)

    def test_drag_opposes_velocity(self):
        v = np.array([1000.0, 0.0, 0.0])
        a = drag_accel(1.0e-12, v, 2.2, 0.1, 100.0)
        self.assertLess(np.dot(a, v), 0.0)

    def test_medt_perpendicular_to_b(self):
        r = self.r_ref
        a_cmd = np.array([1.0e-4, 2.0e-4, 0.0])
        a, _, _ = medt_accel(a_cmd, r, 150.0, 10.0, 1000.0, 0.5, safe_mode=False)
        B = dipole_b_field_eci(r)
        if np.linalg.norm(a) > 0.0 and np.linalg.norm(B) > 0.0:
            self.assertAlmostEqual(float(np.dot(a, B)), 0.0, places=9)


if __name__ == "__main__":
    unittest.main()
