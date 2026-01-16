# ============================================================
# File: core/edt.py
# GLIDE v3.1 – Electrodynamic Tether Model (2D-consistent)
# ============================================================

from __future__ import annotations
import numpy as np


class ElectrodynamicTether:
    """Simple EDT model suitable for a 2D planar sim.

    Your original version forced the Lorentz force to be perpendicular to velocity,
    which can inject/remove energy in non-physical ways.

    This version:
      • Treats B as out-of-plane (Bz) and applies in-plane force ⟂ tether direction
      • Chooses sign so boost tends to make F·v >= 0 and drag tends to make F·v <= 0
      • Power bookkeeping:
           P_orbit = F · v
           P_loss  = I^2 R
           boost: P_net = P_loss + max(0, P_orbit)
           drag : P_net = P_loss + min(0, P_orbit)   (can be negative => generation)
    """

    def __init__(
        self,
        L: float = 100.0,
        B_vec: np.ndarray = np.array([0.0, 0.0, 3.1e-5]),
        R_tether: float = 50.0,
        I_max: float = 2.5,
        mode: str = "off",
    ):
        self.L = float(L)
        self.B_vec = np.asarray(B_vec, dtype=float)
        if self.B_vec.size == 2:
            self.B_vec = np.array([0.0, 0.0, float(self.B_vec[-1])], dtype=float)
        elif self.B_vec.size != 3:
            raise ValueError("B_vec must be length 2 or 3")

        self.R_tether = float(R_tether)
        self.I_max = float(I_max)
        self.mode = str(mode).lower().strip()

        self.I = 0.0
        self.F = np.zeros(2, dtype=float)
        self.P_loss = 0.0
        self.P_orbit = 0.0
        self.P_net = 0.0

    def update(
        self,
        node_pos: np.ndarray,
        payload_pos: np.ndarray,
        velocity_vec: np.ndarray,
        current_cmd: float = 0.0,
    ) -> np.ndarray:
        if self.mode == "off":
            self.I = 0.0
            self.F[:] = 0.0
            self.P_loss = 0.0
            self.P_orbit = 0.0
            self.P_net = 0.0
            return self.F

        self.I = float(np.clip(current_cmd, -1.0, 1.0)) * self.I_max

        L_vec2 = np.asarray(payload_pos, dtype=float) - np.asarray(node_pos, dtype=float)
        L_norm = float(np.linalg.norm(L_vec2))
        if L_norm < 1e-9:
            self.F[:] = 0.0
            self.P_loss = (self.I ** 2) * self.R_tether
            self.P_orbit = 0.0
            self.P_net = self.P_loss
            return self.F

        L_hat = L_vec2 / L_norm

        # Assume B is out-of-plane
        Bz = float(self.B_vec[2])
        F_mag = abs(self.I) * L_norm * abs(Bz)

        perp_L = np.array([-L_hat[1], L_hat[0]], dtype=float)
        F_candidate = F_mag * perp_L

        v = np.asarray(velocity_vec, dtype=float)
        P_candidate = float(np.dot(F_candidate, v))

        if np.linalg.norm(v) > 1e-9:
            if self.mode == "boost":
                sgn = 1.0 if P_candidate >= 0 else -1.0
            else:
                sgn = -1.0 if P_candidate >= 0 else 1.0
        else:
            sgn = 1.0 if self.mode == "boost" else -1.0

        self.F = sgn * F_candidate

        self.P_loss = (self.I ** 2) * self.R_tether
        self.P_orbit = float(np.dot(self.F, v))

        if self.mode == "boost":
            self.P_net = self.P_loss + max(0.0, self.P_orbit)
        else:  # drag
            self.P_net = self.P_loss + min(0.0, self.P_orbit)

        return self.F

    def get_force(self) -> np.ndarray:
        return self.F

    def get_power(self) -> float:
        return float(self.P_net)

    def set_mode(self, mode: str):
        self.mode = str(mode).lower().strip()
