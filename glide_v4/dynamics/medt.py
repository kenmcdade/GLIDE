"""Simplified mEDT model with dipole B-field."""

import numpy as np
from .constants import R_EARTH, B0_EARTH


def dipole_b_field_eci(r_eci):
    r = np.asarray(r_eci, dtype=float)
    r_norm = np.linalg.norm(r)
    if r_norm < 1e-6:
        return np.zeros(3)
    r_hat = r / r_norm
    m_hat = np.array([0.0, 0.0, 1.0])
    factor = B0_EARTH * (R_EARTH / r_norm) ** 3
    return factor * (3.0 * np.dot(m_hat, r_hat) * r_hat - m_hat)


def medt_accel(a_cmd_eci, r_eci, mass, I_max, L_tether, eta, safe_mode=False):
    if safe_mode:
        return np.zeros(3), 0.0, 0.0

    B_vec = dipole_b_field_eci(r_eci)
    B_mag = np.linalg.norm(B_vec)
    if B_mag < 1e-12:
        return np.zeros(3), 0.0, B_mag

    b_hat = B_vec / B_mag
    # Force is perpendicular to B. Project command onto that plane.
    a_cmd = np.asarray(a_cmd_eci, dtype=float)
    a_perp = a_cmd - np.dot(a_cmd, b_hat) * b_hat
    a_perp_mag = np.linalg.norm(a_perp)

    eta = max(0.0, float(eta))
    a_max = (eta * I_max * L_tether * B_mag) / max(mass, 1e-9)
    if a_perp_mag < 1e-12:
        return np.zeros(3), a_max, B_mag

    a_mag = min(a_perp_mag, a_max)
    a_vec = a_mag * (a_perp / a_perp_mag)
    return a_vec, a_max, B_mag
