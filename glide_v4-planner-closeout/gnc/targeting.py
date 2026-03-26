"""Release targeting-lite utilities."""

import numpy as np
from .cw import cw_matrices, cw_propagate


def solve_dv0_minimize_rT(r0, v0, n, tof, dv0_max=None, dv0_axis_max=None):
    dv0, _ = solve_dv0_minimize_state(
        r0,
        v0,
        n,
        tof,
        dv0_max=dv0_max,
        dv0_axis_max=dv0_axis_max,
        w_r=1.0,
        w_v=0.0,
    )
    phi_rr, phi_rv, _, _ = cw_matrices(n, tof)
    r_pred = phi_rr @ r0 + phi_rv @ v0
    return dv0, r_pred


def solve_dv0_minimize_state(
    r0,
    v0,
    n,
    tof,
    dv0_max=None,
    dv0_axis_max=None,
    w_r=1.0,
    w_v=0.0,
    r_target=None,
    v_target=None,
):
    """Weighted least-squares solve for a release dv that shapes final CW state."""
    phi_rr, phi_rv, phi_vr, phi_vv = cw_matrices(n, tof)
    r_target = np.zeros(3) if r_target is None else np.asarray(r_target, dtype=float)
    v_target = np.zeros(3) if v_target is None else np.asarray(v_target, dtype=float)
    w_r = float(max(w_r, 0.0))
    w_v = float(max(w_v, 0.0))

    rows = []
    rhs = []
    if w_r > 0.0:
        swr = np.sqrt(w_r)
        rows.append(swr * phi_rv)
        rhs.append(swr * (phi_rr @ r0 + phi_rv @ v0 - r_target))
    if w_v > 0.0:
        swv = np.sqrt(w_v)
        rows.append(swv * phi_vv)
        rhs.append(swv * (phi_vr @ r0 + phi_vv @ v0 - v_target))

    if not rows:
        dv0 = np.zeros(3)
    else:
        a = np.vstack(rows)
        b = np.hstack(rhs)
        dv0 = -np.linalg.pinv(a) @ b

    if dv0_axis_max is not None:
        dv0_axis_max = np.asarray(dv0_axis_max, dtype=float)
        dv0 = np.clip(dv0, -dv0_axis_max, dv0_axis_max)

    if dv0_max is not None:
        dv_mag = np.linalg.norm(dv0)
        if dv_mag > dv0_max and dv_mag > 1e-12:
            dv0 = dv0 * (dv0_max / dv_mag)

    r_pred, v_pred = cw_propagate(r0, v0 + dv0, n, tof)
    return dv0, {"r_pred": r_pred, "v_pred": v_pred}


def solve_dt_offset_minimize_rT(r0, v0, n, tof, dt_max, dt_step):
    best = {
        "dt": 0.0,
        "r_pred": None,
        "r_norm": None,
        "r0": r0,
        "v0": v0,
    }
    steps = int(np.floor(dt_max / dt_step))
    for k in range(-steps, steps + 1):
        dt = k * dt_step
        r0_dt, v0_dt = cw_propagate(r0, v0, n, dt)
        r_pred, _ = cw_propagate(r0_dt, v0_dt, n, tof)
        r_norm = float(np.linalg.norm(r_pred))
        if best["r_norm"] is None or r_norm < best["r_norm"]:
            best = {
                "dt": float(dt),
                "r_pred": r_pred,
                "r_norm": r_norm,
                "r0": r0_dt,
                "v0": v0_dt,
            }

    return best
