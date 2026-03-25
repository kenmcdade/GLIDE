"""Metrics and success classification."""

import numpy as np


def compute_metrics(log, gate_tracker, cfg):
    data = log.as_arrays()
    times = data["t"]
    if len(times) > 1:
        dt = float(np.mean(np.diff(times)))
    else:
        dt = 0.0

    a_applied = data.get("a_applied_lvh")
    a_cmd_lvh = data.get("a_cmd_lvh")

    if a_applied is None:
        dv_eq_applied = 0.0
        dv_axis = np.zeros(3)
        a_applied_mag = np.zeros(1)
    else:
        a_applied_mag = np.linalg.norm(a_applied, axis=1)
        dv_eq_applied = float(np.sum(a_applied_mag) * dt)
        dv_axis = np.sum(np.abs(a_applied), axis=0) * dt

    if a_cmd_lvh is None:
        dv_eq_commanded = 0.0
    else:
        a_cmd_mag = np.linalg.norm(a_cmd_lvh, axis=1)
        dv_eq_commanded = float(np.sum(a_cmd_mag) * dt)

    ddm_on = data.get("ddm_on")
    medt_on = data.get("medt_on")
    duty_ddm = float(np.mean(ddm_on)) if ddm_on is not None else 0.0
    duty_medt = float(np.mean(medt_on)) if medt_on is not None else 0.0

    tags = []

    if gate_tracker.violations:
        tags.append("speed_violation")

    if "R_COR" not in gate_tracker.crossings:
        tags.append("miss_radius_violation")

    if "R_LATCH" not in gate_tracker.crossings:
        tags.append("capture_not_reached")

    alloc_err = data.get("allocation_error")
    alloc_tol = float(cfg.get("allocation_error_tol", 1e-6))
    if alloc_err is not None and float(np.max(alloc_err)) > alloc_tol:
        tags.append("actuator_limit")

    a_cmd_lvh = data.get("a_cmd_lvh")
    adcs_limit = cfg.get("adcs_saturation_limit")
    if a_cmd_lvh is not None and adcs_limit is not None:
        if float(np.max(np.linalg.norm(a_cmd_lvh, axis=1))) > float(adcs_limit):
            tags.append("adcs_saturation")

    # Tiered outcomes
    corridor_entry_success = False
    precapture_success = False
    latch_success = False

    if "R_COR" in gate_tracker.crossings:
        speed = gate_tracker.crossings["R_COR"]["speed_mps"]
        corridor_entry_success = speed <= gate_tracker.speed_limits["R_COR"]["hard_max"]
    if "R_PRE" in gate_tracker.crossings:
        speed = gate_tracker.crossings["R_PRE"]["speed_mps"]
        precapture_success = speed <= gate_tracker.speed_limits["R_PRE"]["hard_max"]
    if "R_LATCH" in gate_tracker.crossings:
        speed = gate_tracker.crossings["R_LATCH"]["speed_mps"]
        latch_success = speed <= gate_tracker.speed_limits["R_LATCH"]["hard_max"]

    success = latch_success and ("speed_violation" not in tags)

    peak_applied = float(np.max(a_applied_mag)) if a_applied is not None else 0.0
    avg_applied = float(np.mean(a_applied_mag)) if a_applied is not None else 0.0

    return {
        "success": success,
        "tags": tags,
        "dv_eq_applied": dv_eq_applied,
        "dv_axis": dv_axis,
        "dv_eq_commanded": dv_eq_commanded,
        "peak_applied": peak_applied,
        "avg_applied": avg_applied,
        "duty_ddm": duty_ddm,
        "duty_medt": duty_medt,
        "corridor_entry_success": corridor_entry_success,
        "precapture_success": precapture_success,
        "latch_success": latch_success,
    }
