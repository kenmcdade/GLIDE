"""Terminal capture controller with LOS speed governor and tangential damping."""

import numpy as np


def terminal_capture_command(r_lvh, v_lvh, thresholds, speed_limits, cfg, dt):
    # LOS unit vector and LOS speed.
    r = np.asarray(r_lvh, dtype=float)
    v = np.asarray(v_lvh, dtype=float)
    range_m = float(np.linalg.norm(r))
    if range_m < 1e-9:
        return np.zeros(3), {"raw_mag": 0.0, "sat": False, "a_max": 0.0}

    u = r / range_m
    v_los = float(np.dot(v, u))
    v_tan = v - v_los * u
    v_mag = float(np.linalg.norm(v))
    v_hat = v / max(v_mag, 1e-12)

    # LOS speed limits (internal targets).
    margin = float(cfg.get("range_margin", 0.0))
    post_cfg = cfg.get("post_cor_shaping", {})
    post_enabled = bool(post_cfg.get("enabled", False))
    if range_m <= thresholds["R_LATCH"]:
        target_v_los = float(cfg.get("v_los_target_latch", 0.03))
        hard_v_los = float(cfg.get("v_los_max_latch", 0.06))
        next_radius = 0.0
        next_speed_cap = speed_limits["R_LATCH"]["hard_max"]
        los_scale = float(post_cfg.get("v_los_scale_latch", 1.0)) if post_enabled else 1.0
    elif range_m <= thresholds["R_PRE"]:
        target_v_los = float(cfg.get("v_los_target_pre", 0.07))
        hard_v_los = float(cfg.get("v_los_max_pre", 0.12))
        next_radius = thresholds["R_LATCH"]
        next_speed_cap = speed_limits["R_LATCH"]["hard_max"]
        los_scale = float(post_cfg.get("v_los_scale_pre", 1.0)) if post_enabled else 1.0
    else:
        target_v_los = float(cfg.get("v_los_target_cor", 0.15))
        hard_v_los = float(cfg.get("v_los_max_cor", 0.25))
        next_radius = thresholds["R_PRE"]
        next_speed_cap = speed_limits["R_PRE"]["hard_max"]
        los_scale = float(post_cfg.get("v_los_scale_cor", 1.0)) if post_enabled else 1.0

    cap_scale = float(cfg.get("cap_scale", 1.0))
    cap_scale_pre = float(cfg.get("cap_scale_pre", cap_scale))
    cap_scale_latch = float(cfg.get("cap_scale_latch", cap_scale))

    # Enforce conservative caps by region to guarantee gate compliance.
    if range_m <= thresholds["R_PRE"]:
        next_speed_cap = min(next_speed_cap, speed_limits["R_LATCH"]["hard_max"] * cap_scale_latch)
    else:
        next_speed_cap = min(next_speed_cap, speed_limits["R_PRE"]["hard_max"] * cap_scale_pre)

    next_radius_eff = max(0.0, next_radius - margin)

    desired_v_los = -target_v_los * los_scale

    # Range-scheduled gains.
    gain_scale = float(cfg.get("gain_scale_max", 6.0))
    scale = min(gain_scale, max(1.0, thresholds["R_COR"] / max(range_m, 1e-6)))
    kp = float(cfg.get("kp", 2.0e-3)) * scale
    kd = float(cfg.get("kd", 5.0e-2)) * scale
    kt = float(cfg.get("kt", 2.0e-1)) * scale
    if post_enabled:
        kd *= float(post_cfg.get("kd_mult", 1.0))
        kt *= float(post_cfg.get("kt_mult", 1.0))
        kp *= float(post_cfg.get("kp_mult", 1.0))

    # Braking to enforce latch cap based on stopping distance.
    d = max(range_m - next_radius_eff, 1e-3)
    a_max = float(cfg.get("max_accel", 5.0e-4))
    if range_m <= thresholds["R_PRE"]:
        v_cap_latch = speed_limits["R_LATCH"]["hard_max"] * cap_scale_latch
        if v_mag > v_cap_latch:
            a_brake = min(a_max, (v_mag - v_cap_latch) / max(dt, 1e-6))
            a_cmd = -a_brake * v_hat
            return a_cmd, {"raw_mag": float(np.linalg.norm(a_cmd)), "sat": False, "a_max": float(a_max)}

    # Discrete-time braking to enforce total speed cap.
    if v_mag > next_speed_cap:
        a_brake = min(a_max, (v_mag - next_speed_cap) / max(dt, 1e-6))
        a_cmd = -a_brake * v_hat
        return a_cmd, {"raw_mag": float(np.linalg.norm(a_cmd)), "sat": False, "a_max": float(a_max)}

    # Direct velocity targeting in terminal region (discrete-time).
    desired_v_los = np.clip(desired_v_los, -hard_v_los, hard_v_los)
    v_target = desired_v_los * u
    if np.linalg.norm(v_target) > next_speed_cap:
        v_target = v_target / max(np.linalg.norm(v_target), 1e-9) * next_speed_cap
    a_cmd_raw = (v_target - v) / max(dt, 1e-6)
    if post_enabled:
        los_err = v_los - desired_v_los
        range_err = max(range_m - next_radius_eff, 0.0)
        a_cmd_raw += -kd * los_err * u
        a_cmd_raw += -kt * v_tan
        a_cmd_raw += -kp * range_err * u
    raw_mag = float(np.linalg.norm(a_cmd_raw))
    a_cmd = a_cmd_raw
    if raw_mag > a_max and raw_mag > 1e-12:
        a_cmd = a_cmd_raw * (a_max / raw_mag)
    sat = raw_mag > (a_max + 1e-12)

    return a_cmd, {"raw_mag": raw_mag, "sat": sat, "a_max": float(a_max)}
