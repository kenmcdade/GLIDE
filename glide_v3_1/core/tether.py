# ============================================================
# File: core/tether.py
# GLIDE v3.1 – Dynamic Tether Physics Engine (stabilized)
# ============================================================

from __future__ import annotations
import numpy as np


class DynamicTether:
    """Multi-segment tether with spring-damper axial elements.

    Stability fixes:
      • No dt-dependent per-step damping.
      • Constraints (optional) do not inject energy (velocity recomputed from corrected positions).
      • Gravity/external fields are applied by the integrator, not here.
    """

    def __init__(
        self,
        N: int = 25,
        L0: float = 200.0,
        E: float = 30e9,
        d: float = 0.005,
        rho_l: float = 1.5,
        damping_ratio: float = 0.05,
        numerical_vel_decay_per_s: float = 0.0,
    ):
        self.N = int(N)
        self.L0 = float(L0)

        self.segment_length = self.L0 / self.N
        self.A = np.pi * (float(d) / 2.0) ** 2
        self.k = float(E) * self.A / self.segment_length  # N/m per segment

        self.rho_l = float(rho_l)
        self.m_segment = self.rho_l * self.segment_length

        self.c = 2.0 * np.sqrt(self.k * self.m_segment) * float(damping_ratio)
        self._vel_decay_per_s = max(0.0, float(numerical_vel_decay_per_s))

        self.positions = np.zeros((self.N + 1, 2), dtype=float)
        self.velocities = np.zeros((self.N + 1, 2), dtype=float)

        for i in range(self.N + 1):
            self.positions[i] = np.array([0.0, -i * self.segment_length], dtype=float)

        self._forces = np.zeros_like(self.positions)
        self._pos_prev = self.positions.copy()

    def anchor_update(self, anchor_pos: np.ndarray, anchor_vel: np.ndarray):
        self.positions[0] = np.asarray(anchor_pos, dtype=float)
        self.velocities[0] = np.asarray(anchor_vel, dtype=float)

    def compute_internal_forces(self) -> np.ndarray:
        F = np.zeros_like(self.positions)

        for i in range(self.N):
            p0, p1 = self.positions[i], self.positions[i + 1]
            v0, v1 = self.velocities[i], self.velocities[i + 1]

            r = p1 - p0
            L = float(np.linalg.norm(r))
            if L < 1e-12:
                continue

            u = r / L
            stretch = L - self.segment_length
            rel_vel = float(np.dot(v1 - v0, u))

            Fs = -self.k * stretch * u - self.c * rel_vel * u

            F[i] += Fs
            F[i + 1] -= Fs

        self._forces = F
        return F

    def add_external_forces(self, F_ext: np.ndarray):
        self._forces += np.asarray(F_ext, dtype=float)

    def apply_force_to_payload(self, F_ext: np.ndarray):
        self._forces[-1] += np.asarray(F_ext, dtype=float)

    def integrate(self, dt: float):
        dt = float(dt)
        if dt <= 0:
            return

        self._pos_prev[:] = self.positions

        a = self._forces / self.m_segment

        self.velocities[1:] += a[1:] * dt
        self.positions[1:] += self.velocities[1:] * dt

        if self._vel_decay_per_s > 0.0:
            self.velocities[1:] *= np.exp(-self._vel_decay_per_s * dt)

    def enforce_constraints(self, dt: float, iterations: int = 2):
        dt = float(dt)
        if dt <= 0:
            return

        iterations = int(max(0, iterations))
        if iterations == 0:
            return

        for _ in range(iterations):
            for i in range(self.N):
                p0, p1 = self.positions[i], self.positions[i + 1]
                delta = p1 - p0
                dist = float(np.linalg.norm(delta))
                if dist < 1e-12:
                    continue

                diff = (dist - self.segment_length) / dist

                if i == 0:
                    self.positions[i + 1] -= delta * diff
                else:
                    corr = 0.5 * delta * diff
                    self.positions[i] += corr
                    self.positions[i + 1] -= corr

            self.positions[0] = self._pos_prev[0]

        # velocity recompute => stops constraint energy injection
        self.velocities[1:] = (self.positions[1:] - self._pos_prev[1:]) / dt

    def get_payload_state(self):
        return self.positions[-1].copy(), self.velocities[-1].copy()

    def get_tension_profile(self) -> np.ndarray:
        tensions = np.zeros(self.N, dtype=float)
        for i in range(self.N):
            p0, p1 = self.positions[i], self.positions[i + 1]
            dist = float(np.linalg.norm(p1 - p0))
            tensions[i] = self.k * max(0.0, dist - self.segment_length)
        return tensions

    def total_energy_mech(self) -> float:
        kinetic = 0.5 * self.m_segment * float(np.sum(np.linalg.norm(self.velocities[1:], axis=1) ** 2))
        elastic = 0.0
        for i in range(self.N):
            p0, p1 = self.positions[i], self.positions[i + 1]
            dist = float(np.linalg.norm(p1 - p0))
            stretch = dist - self.segment_length
            elastic += 0.5 * self.k * (stretch * stretch)
        return kinetic + elastic
