"""Simple closed-loop guidance in LVLH."""

import numpy as np
from .cw import cw_matrices


class Guidance:
    def __init__(self, cfg):
        self.cfg = cfg
        self.last_update_t = -1.0e9
        self.last_mode = None
        self.last_cmd = np.zeros(3)

    def compute(self, t, r_lvh, v_lvh, mode):
        cadence = float(self.cfg["cadence_s"][mode])
        update_now = (t - self.last_update_t) >= cadence or (mode != self.last_mode)
        if update_now:
            method = self.cfg.get("method", "pd")
            if method == "cw_target":
                n = float(self.cfg["n_rad_s"])
                t_end = float(self.cfg["t_end_s"])
                t_go = max(t_end - t, cadence)
                phi_rr, phi_rv, phi_vr, phi_vv = cw_matrices(n, t_go)
                w_r = float(self.cfg.get("cw_position_weight", 1.0))
                w_v = float(self.cfg.get("cw_speed_weight", 0.0))
                if w_v > 0.0:
                    swr = np.sqrt(max(w_r, 0.0))
                    swv = np.sqrt(max(w_v, 0.0))
                    a = np.vstack((swr * phi_rv, swv * phi_vv))
                    b = np.hstack((swr * (phi_rr @ r_lvh + phi_rv @ v_lvh), swv * (phi_vr @ r_lvh + phi_vv @ v_lvh)))
                    dv0 = -np.linalg.pinv(a) @ b
                else:
                    dv0 = -np.linalg.pinv(phi_rv) @ (phi_rr @ r_lvh + phi_rv @ v_lvh)
                a_cmd = dv0 / max(cadence, 1e-6)
            else:
                gains = self.cfg["gains"][mode]
                kp = float(gains["kp"])
                kd = float(gains["kd"])
                # PD guidance: reduce miss distance and relative speed.
                a_cmd = -kp * r_lvh - kd * v_lvh

            # Speed shaping near gates.
            speed = np.linalg.norm(v_lvh)
            target_speed = self.cfg.get("target_speeds", {}).get(mode)
            if target_speed is not None and speed > 1e-9:
                speed_gain = float(self.cfg.get("speed_gain", 0.0))
                if speed > target_speed:
                    a_cmd -= speed_gain * (speed - target_speed) * (v_lvh / speed)

            # Explicit R_COR entry speed minimization near corridor.
            range_m = np.linalg.norm(r_lvh)
            cor_radius = float(self.cfg.get("r_cor_slowdown_radius_m", 100.0))
            cor_target = float(self.cfg.get("r_cor_target_speed_mps", 0.20))
            cor_gain = float(self.cfg.get("r_cor_speed_gain", 0.0))
            safety_cfg = self.cfg.get("speed_safety_mode", {})
            if safety_cfg.get("enabled", False):
                r_cor_m = float(safety_cfg.get("r_cor_m", 10.0))
                window_k = float(safety_cfg.get("window_k_r_cor", 3.0))
                safety_range = float(safety_cfg.get("range_m", window_k * r_cor_m))
                safety_speed_trigger = float(safety_cfg.get("speed_trigger_mps", 0.20))
                gain_mult_min = float(safety_cfg.get("gain_mult", 1.0))
                gain_mult_max = float(safety_cfg.get("gain_mult_max", gain_mult_min))
                if range_m <= safety_range and speed >= safety_speed_trigger:
                    blend = max(0.0, min(1.0, 1.0 - range_m / max(safety_range, 1e-6)))
                    gain_mult = gain_mult_min + (gain_mult_max - gain_mult_min) * blend
                    cor_gain *= gain_mult
            if cor_gain > 0.0 and range_m <= cor_radius and speed > 1e-9:
                blend = max(0.0, min(1.0, 1.0 - range_m / max(cor_radius, 1e-6)))
                speed_excess = max(0.0, speed - cor_target)
                a_cmd -= cor_gain * blend * speed_excess * (v_lvh / speed)

            max_accel = float(self.cfg.get("max_accel", 1e-3))
            a_mag = np.linalg.norm(a_cmd)
            if a_mag > max_accel:
                a_cmd = a_cmd * (max_accel / a_mag)

            self.last_cmd = a_cmd
            self.last_update_t = t
            self.last_mode = mode

        return self.last_cmd.copy()
