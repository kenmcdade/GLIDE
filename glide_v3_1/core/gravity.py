# ============================================================
# File: core/gravity.py
# GLIDE v3.1 – Gravitational Field Model (clean + consistent)
# ============================================================

from __future__ import annotations
import numpy as np


class GravityField:
    """Gravity model.

    Modes:
      • local   : constant acceleration [0, -g0]
      • orbital : two-body field about origin (expects origin at central body)

    Notes:
      If you want an orbital environment while simulating a local tether patch,
      you usually want a gravity-gradient model around a reference radius.
      This code implements the simple two-body field; for your current workflow,
      you're using "local" with g0=local_g.
    """

    def __init__(
        self,
        mu: float = 3.986004418e14,
        g0: float = 9.80665,
        R_earth: float = 6.371e6,
        mode: str = "local",
    ):
        self.mu = float(mu)
        self.g0 = float(g0)
        self.R_earth = float(R_earth)
        self.mode = str(mode).lower().strip()

    def get_acceleration(self, position: np.ndarray) -> np.ndarray:
        pos = np.asarray(position, dtype=float)

        if self.mode == "local":
            return np.array([0.0, -self.g0], dtype=float)

        # Orbital (two-body) about origin
        r2 = float(np.dot(pos, pos))
        if r2 < 1.0:
            return np.zeros(2, dtype=float)

        r = float(np.sqrt(r2))
        return -self.mu * pos / (r ** 3)

    def get_potential(self, position: np.ndarray) -> float:
        pos = np.asarray(position, dtype=float)

        if self.mode == "local":
            # Potential per unit mass up to a constant: U = g*y
            return self.g0 * float(pos[1])

        r2 = float(np.dot(pos, pos))
        if r2 < 1.0:
            return 0.0
        r = float(np.sqrt(r2))
        return -self.mu / r

    def acceleration_difference(self, p1: np.ndarray, p2: np.ndarray) -> np.ndarray:
        return self.get_acceleration(p2) - self.get_acceleration(p1)
