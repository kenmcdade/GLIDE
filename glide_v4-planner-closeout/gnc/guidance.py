"""Simple closed-loop guidance in LVLH."""

import numpy as np
from .cw import cw_matrices


class Guidance:
    def __init__(self, cfg):
        self.cfg = cfg
        self.last_update_t = -1.0e9
        self.last_mode = None
        self.last_cmd = np.zeros(3)

    def _apply_speed_safety_prebrake(self, a_cmd, r_lvh, v_lvh, range_m, speed, safety_cfg):
        """Predict one-step corridor crossing speed and pre-brake if risk is detected."""
        if not safety_cfg.get("enabled", False):
            return a_cmd

        r_cor_m = float(safety_cfg.get("r_cor_m", 10.0))
        range_window_m = float(safety_cfg.get("range_m", np.inf))
        trigger_margin_m = float(max(0.0, safety_cfg.get("prebrake_margin_m", 0.0)))
        if range_m <= r_cor_m or range_m > range_window_m:
            return a_cmd
        if range_m <= 1e-9 or speed <= 1e-12:
            return a_cmd

        u_los = r_lvh / range_m
        v_los = float(np.dot(v_lvh, u_los))
        closing = max(0.0, -v_los)
        if closing <= 1e-12:
            return a_cmd

        dt_pred = float(max(1e-3, safety_cfg.get("predict_dt_s", 1.0)))
        t_to_cor = (range_m - r_cor_m) / closing
        if t_to_cor > dt_pred and range_m > (r_cor_m + trigger_margin_m):
            return a_cmd

        t_eval = float(max(1e-3, min(t_to_cor, dt_pred)))
        v_pred = v_lvh + a_cmd * t_eval
        speed_pred = float(np.linalg.norm(v_pred))
        v_pred_hat = v_pred / max(speed_pred, 1e-12)

        hard_cap_mps = float(safety_cfg.get("hard_cap_mps", 0.35))
        comfort_cap_mps = float(safety_cfg.get("comfort_cap_mps", min(0.30, hard_cap_mps)))
        crossing_target_mps = min(hard_cap_mps, comfort_cap_mps)
        prebrake_gain = float(max(0.0, safety_cfg.get("prebrake_gain", 1.0)))

        brake_cmd = np.zeros(3)

        # Overspeed-at-crossing guard: brake opposite predicted velocity.
        if speed_pred > crossing_target_mps:
            excess = speed_pred - crossing_target_mps
            brake_mag = prebrake_gain * excess / t_eval
            brake_cmd -= brake_mag * v_pred_hat

        # Stopping-distance guard on line-of-sight closing.
        max_accel = float(self.cfg.get("max_accel", 1e-3))
        remaining_m = max(range_m - r_cor_m, 1e-6)
        stopping_m = (closing * closing) / max(2.0 * max_accel, 1e-12)
        if stopping_m > remaining_m:
            urgency = min(2.0, stopping_m / remaining_m)
            brake_cmd += prebrake_gain * urgency * max_accel * u_los

        return a_cmd + brake_cmd

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

            # One-step crossing predictor to reduce discrete-time R_COR overshoot.
            a_cmd = self._apply_speed_safety_prebrake(
                a_cmd,
                r_lvh,
                v_lvh,
                range_m,
                speed,
                safety_cfg,
            )

            max_accel = float(self.cfg.get("max_accel", 1e-3))
            a_mag = np.linalg.norm(a_cmd)
            if a_mag > max_accel:
                a_cmd = a_cmd * (max_accel / a_mag)

            self.last_cmd = a_cmd
            self.last_update_t = t
            self.last_mode = mode

        return self.last_cmd.copy()
