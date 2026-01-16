# ============================================================
# File: core/motor.py
# GLIDE v3.1 – Winch Motor Dynamics
# ============================================================

import numpy as np

class MotorSystem:
    """
    MotorSystem (Engineering Mode)
    ------------------------------
    High-fidelity model of a GLIDE winch motor driving tether motion.
    """

    def __init__(
        self,
        R_spool: float = 0.25,
        J_m: float = 0.08,
        tau_max: float = 15.0,
        b_m: float = 0.02,
        eta: float = 0.87,
    ):
        self.R_spool = R_spool
        self.J_m = J_m
        self.tau_max = tau_max
        self.b_m = b_m
        self.eta = eta

        self.omega = 0.0
        self.torque = 0.0
        self.energy_used = 0.0
        self.anchor_length = 0.0

        self.P_mech = 0.0
        self.P_elec = 0.0

    def update(self, dt: float, tension: float, omega_cmd: float = 0.0):
        tau_load = tension * self.R_spool

        kp = 2.0
        tau_cmd = kp * (omega_cmd - self.omega)
        tau_cmd = np.clip(tau_cmd, -self.tau_max, self.tau_max)

        domega = (tau_cmd - tau_load - self.b_m * self.omega) / self.J_m
        domega = np.clip(domega, -100.0, 100.0)
        self.omega = np.clip(self.omega + domega * dt, -50.0, 50.0)
        self.torque = tau_cmd

        v_anchor = self.omega * self.R_spool
        self.anchor_length += v_anchor * dt

        self.P_mech = self.torque * self.omega

        if self.P_mech >= 0:
            self.P_elec = self.P_mech / self.eta
        else:
            self.P_elec = self.P_mech * self.eta

        self.energy_used += self.P_elec * dt
        return v_anchor

    def get_state(self):
        return {
            "omega": self.omega,
            "torque": self.torque,
            "P_mech": self.P_mech,
            "P_elec": self.P_elec,
            "energy_used": self.energy_used,
            "anchor_length": self.anchor_length,
        }

    def reset(self):
        self.omega = 0.0
        self.torque = 0.0
        self.energy_used = 0.0
        self.anchor_length = 0.0
