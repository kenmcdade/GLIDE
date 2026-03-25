"""Feasibility estimator for initial conditions."""

import numpy as np

from dynamics.constants import MU_EARTH, R_EARTH
from dynamics.drag import drag_accel_mag
from dynamics.medt import dipole_b_field_eci
from dynamics.frames import lvlh_basis


def estimate_feasibility(cfg, r_rel_lvh, v_rel_lvh, r_ref, v_ref, env_payload):
    sim_cfg = cfg["simulation"]
    tof = float(sim_cfg["t_end_s"])

    r_rel_lvh = np.asarray(r_rel_lvh, dtype=float)
    v_rel_lvh = np.asarray(v_rel_lvh, dtype=float)

    dv_req_axis = np.abs(r_rel_lvh) / max(tof, 1e-6)
    dv_req_total = float(np.linalg.norm(r_rel_lvh) / max(tof, 1e-6))

    # Max DDM drag authority (along -T axis only).
    atm = env_payload.get("atmosphere")
    r_norm = np.linalg.norm(r_ref)
    alt = r_norm - R_EARTH
    Cd = float(env_payload.get("Cd", 2.2))
    mass = float(env_payload.get("mass", 100.0))

    ddm_cfg = cfg["payload"]
    area_max = float(ddm_cfg["area_m2_max"])

    if atm is None:
        a_drag_max = 0.0
    else:
        rho = atm.density(alt)
        v_mag = np.linalg.norm(v_ref)
        a_drag_max = drag_accel_mag(rho, v_mag, Cd, area_max, mass)

    a_drag_axis = np.array([0.0, a_drag_max, 0.0])

    # Max mEDT authority (perpendicular to B-field).
    medt_cfg = cfg["medt"]
    eta = float(medt_cfg["eta"])
    I_max = float(medt_cfg["I_max"])
    L_tether = float(medt_cfg["L_tether"])

    B_vec = dipole_b_field_eci(r_ref)
    B_mag = np.linalg.norm(B_vec)
    if B_mag < 1e-12 or eta <= 0.0:
        a_medt_max = 0.0
    else:
        a_medt_max = (eta * I_max * L_tether * B_mag) / max(mass, 1e-9)

    # Axis-wise max mEDT component based on B-field orientation.
    C = lvlh_basis(r_ref, v_ref)
    e_R = C.T @ np.array([1.0, 0.0, 0.0])
    e_T = C.T @ np.array([0.0, 1.0, 0.0])
    e_N = C.T @ np.array([0.0, 0.0, 1.0])

    if B_mag < 1e-12:
        medt_axis = np.zeros(3)
    else:
        b_hat = B_vec / B_mag
        def axis_cap(e_i):
            return a_medt_max * np.sqrt(max(0.0, 1.0 - float(np.dot(e_i, b_hat)) ** 2))
        medt_axis = np.array([axis_cap(e_R), axis_cap(e_T), axis_cap(e_N)])

    a_axis_max = a_drag_axis + medt_axis
    dv_eq_applied_axis_max = a_axis_max * tof

    # Combined applied acceleration magnitude (do not sum per-axis maxima).
    v_hat = v_ref / max(np.linalg.norm(v_ref), 1e-12)
    if B_mag < 1e-12:
        sin_theta = 0.0
    else:
        b_hat = B_vec / B_mag
        sin_theta = float(np.sqrt(max(0.0, 1.0 - (np.dot(v_hat, b_hat) ** 2))))
    a_total_max = np.sqrt(a_drag_max ** 2 + a_medt_max ** 2 + 2.0 * a_drag_max * a_medt_max * sin_theta)
    dv_eq_applied_total_max = float(a_total_max * tof)

    infeasible = False
    reasons = []
    if dv_req_total > 0.8 * dv_eq_applied_total_max:
        infeasible = True
        reasons.append("dv_req_total_exceeds")

    for i, axis in enumerate(["R", "T", "N"]):
        if dv_req_axis[i] > dv_eq_applied_axis_max[i]:
            infeasible = True
            reasons.append(f"dv_req_{axis}_exceeds")

    return {
        "tof": tof,
        "dv_req_axis": dv_req_axis,
        "dv_req_total": dv_req_total,
        "dv_eq_axis_max": dv_eq_applied_axis_max,
        "dv_eq_total_max": dv_eq_applied_total_max,
        "infeasible": infeasible,
        "reasons": reasons,
        "a_drag_max": a_drag_max,
        "a_medt_max": a_medt_max,
    }
