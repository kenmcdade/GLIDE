"""Drag model for DDM area modulation."""

import numpy as np


def drag_accel(rho, v_vec, Cd, area, mass):
    v_mag = np.linalg.norm(v_vec)
    if v_mag < 1e-9:
        return np.zeros(3)
    coeff = 0.5 * rho * Cd * area / mass
    return -coeff * v_mag * v_mag * (v_vec / v_mag)


def drag_accel_mag(rho, v_mag, Cd, area, mass):
    coeff = 0.5 * rho * Cd * area / mass
    return coeff * v_mag * v_mag
