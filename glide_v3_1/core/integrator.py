# ============================================================
# File: core/integrator.py
# GLIDE v3.1 – Unified Physics Integrator (stabilized)
# ============================================================

from __future__ import annotations
import numpy as np

from core.tether import DynamicTether
from core.motor import MotorSystem
from core.gravity import GravityField
from core.edt import ElectrodynamicTether
from core.energy import EnergyTracker


class GLIDEIntegrator:
    """Orchestrates subsystem updates per simulation step.

    Stability features:
      • Optional tether substepping so dt=0.05 doesn't blow up stiff springs
      • Anchor boundary condition applied every step
      • Gravity applied consistently to ALL nodes
    """

    def __init__(self, config: dict):
        self.config = dict(config)
        self.dt = float(self.config.get("dt", 0.01))

        self.anchor_pos = np.array([0.0, 0.0], dtype=float)
        self.anchor_vel = np.zeros(2, dtype=float)

        self.gravity_mode = str(self.config.get("gravity_mode", "local")).lower().strip()
        self.local_g = float(self.config.get("local_g", 9.80665))
        self.gravity = GravityField(
            mode=self.gravity_mode,
            g0=self.local_g,
            mu=float(self.config.get("mu", 3.986004418e14)),
            R_earth=float(self.config.get("R_earth", 6.371e6)),
        )

        self.tether = DynamicTether(
            N=int(self.config.get("N", 25)),
            L0=float(self.config.get("L0", 200.0)),
            E=float(self.config.get("E", 30e9)),
            d=float(self.config.get("d", 0.005)),
            rho_l=float(self.config.get("rho_l", 1.5)),
            damping_ratio=float(self.config.get("tether_damping_ratio", 0.05)),
            numerical_vel_decay_per_s=float(self.config.get("numerical_vel_decay_per_s", 0.0)),
        )

        self.motor = MotorSystem(
            R_spool=float(self.config.get("R_spool", 0.25)),
            J_m=float(self.config.get("J_m", 0.08)),
            tau_max=float(self.config.get("tau_max", 15.0)),
            b_m=float(self.config.get("b_m", 0.02)),
            eta=float(self.config.get("eta", 0.87)),
        )

        self.edt = ElectrodynamicTether(
            L=float(self.config.get("EDT_L", 100.0)),
            B_vec=np.array(self.config.get("B_vec", [0.0, 0.0, 3.1e-5]), dtype=float),
            R_tether=float(self.config.get("EDT_R", 50.0)),
            I_max=float(self.config.get("EDT_Imax", 2.5)),
            mode=str(self.config.get("EDT_mode", "off")),
        )

        self.energy = EnergyTracker(
            battery_capacity_J=float(self.config.get("battery_capacity_J", 5e5)),
            log_interval_s=float(self.config.get("log_interval", 1.0)),
        )

        self.constraint_iterations = int(self.config.get("constraint_iterations", 3))
        self.max_substep_dt = float(self.config.get("tether_max_substep_dt", 0.005))
        self.velocity_limit = float(self.config.get("velocity_limit", 0.0))  # 0 disables

        self.step_count = 0

    def _gravity_accel(self, pos: np.ndarray) -> np.ndarray:
        return self.gravity.get_acceleration(pos)

    def _build_external_forces(self, current_cmd: float) -> np.ndarray:
        F_ext = np.zeros_like(self.tether.positions)

        # Gravity on all non-anchor nodes
        for i in range(1, self.tether.N + 1):
            a_g = self._gravity_accel(self.tether.positions[i])
            F_ext[i] += self.tether.m_segment * a_g

        # EDT on payload only
        payload_pos, payload_vel = self.tether.get_payload_state()
        F_edt = self.edt.update(self.anchor_pos, payload_pos, payload_vel, current_cmd)
        F_ext[-1] += F_edt

        return F_ext

    def step(self, omega_cmd: float = 0.0, current_cmd: float = 0.0):
        # Apply anchor boundary condition
        self.tether.anchor_update(self.anchor_pos, self.anchor_vel)

        # Internal forces
        self.tether.compute_internal_forces()

        # Motor update at control rate
        tensions = self.tether.get_tension_profile()
        T_tip = float(tensions[-1]) if len(tensions) else 0.0
        v_anchor_vertical = float(self.motor.update(self.dt, T_tip, omega_cmd))
        self.anchor_vel = np.array([0.0, v_anchor_vertical], dtype=float)

        # Substep tether dynamics if dt is big
        dt = self.dt
        dt_sub = min(dt, self.max_substep_dt) if self.max_substep_dt > 0 else dt
        n_sub = max(1, int(np.ceil(dt / dt_sub)))
        dt_sub = dt / n_sub

        for _ in range(n_sub):
            self.anchor_pos += self.anchor_vel * dt_sub
            self.tether.anchor_update(self.anchor_pos, self.anchor_vel)

            self.tether.compute_internal_forces()
            F_ext = self._build_external_forces(current_cmd)
            self.tether.add_external_forces(F_ext)

            self.tether.integrate(dt_sub)
            self.tether.enforce_constraints(dt_sub, iterations=self.constraint_iterations)

            if self.velocity_limit and self.velocity_limit > 0:
                v_mag = np.linalg.norm(self.tether.velocities, axis=1)
                too_fast = v_mag > self.velocity_limit
                if np.any(too_fast):
                    self.tether.velocities[too_fast] *= (self.velocity_limit / v_mag[too_fast])[:, None]

            if np.isnan(self.tether.positions).any() or np.isnan(self.tether.velocities).any():
                self.tether.positions = np.nan_to_num(self.tether.positions)
                self.tether.velocities = np.nan_to_num(self.tether.velocities)

        # Energy bookkeeping at control rate
        self.energy.update(self.dt, self.tether, self.motor, self.edt, self.gravity)
        self.step_count += 1

    def run(self, duration: float, omega_profile=None, current_profile=None):
        steps = int(float(duration) / self.dt)
        for i in range(steps):
            t = i * self.dt
            omega_cmd = float(omega_profile(t)) if omega_profile else 0.0
            current_cmd = float(current_profile(t)) if current_profile else 0.0
            self.step(omega_cmd, current_cmd)
        return self.energy.summary()
