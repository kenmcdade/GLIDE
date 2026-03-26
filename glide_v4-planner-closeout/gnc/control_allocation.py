"""Control allocation between DDM drag and mEDT."""

import numpy as np
from dynamics.constants import R_EARTH
from dynamics.drag import drag_accel_mag
from dynamics.medt import medt_accel


def allocate_control(a_cmd_eci, state, env, ddm_cfg, medt_cfg, mode):
    r = state[0:3]
    v = state[3:6]
    mass = float(env["mass"])
    Cd = float(env["Cd"])
    atm = env.get("atmosphere")

    v_mag = np.linalg.norm(v)
    # DDM drag authority: aligned opposite velocity.
    if v_mag < 1e-9 or atm is None:
        a_drag = np.zeros(3)
        a_drag_mag = 0.0
        a_drag_min = 0.0
        a_drag_max = 0.0
        A_eff = float(ddm_cfg.get("area_min", 0.0))
    else:
        alt = np.linalg.norm(r) - R_EARTH
        rho = atm.density(alt)
        a_drag_max = drag_accel_mag(rho, v_mag, Cd, ddm_cfg["area_max"], mass)
        a_drag_min = drag_accel_mag(rho, v_mag, Cd, ddm_cfg["area_min"], mass)

        v_hat = v / v_mag
        # Positive desired_along means a deceleration request (against velocity).
        desired_along = -float(np.dot(a_cmd_eci, v_hat))
        if desired_along <= 0.0:
            a_drag_mag = a_drag_min
        else:
            a_drag_mag = float(np.clip(desired_along, a_drag_min, a_drag_max))
        a_drag = -a_drag_mag * v_hat

        if rho > 0.0:
            A_eff = a_drag_mag * mass / (0.5 * rho * Cd * v_mag * v_mag)
            A_eff = float(np.clip(A_eff, ddm_cfg["area_min"], ddm_cfg["area_max"]))
        else:
            A_eff = float(ddm_cfg["area_min"])

    # mEDT authority: perpendicular to B-field, disabled in safe modes.
    safe = mode in medt_cfg.get("safe_modes", [])
    a_remaining = a_cmd_eci - a_drag
    a_medt, a_medt_max, B_mag = medt_accel(
        a_remaining,
        r,
        mass,
        float(medt_cfg["I_max"]),
        float(medt_cfg["L_tether"]),
        float(medt_cfg["eta"]),
        safe_mode=safe,
    )

    a_total = a_drag + a_medt
    allocation_error = float(np.linalg.norm(a_cmd_eci - a_total))

    info = {
        "a_drag_mag": float(a_drag_mag),
        "a_drag_min": float(a_drag_min),
        "a_drag_max": float(a_drag_max),
        "A_eff": float(A_eff),
        "B_mag": float(B_mag),
        "a_medt_max": float(a_medt_max),
        "allocation_error": allocation_error,
        "medt_state": "safe" if safe else "active",
    }
    return a_total, a_drag, a_medt, info
