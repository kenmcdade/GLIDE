"""Run reporting utilities."""

import numpy as np


def _fmt_vec(vec, precision=3):
    return "[" + ", ".join(f"{v:.{precision}f}" for v in vec) + "]"


def print_start_end_summary(cfg, logger, metrics, gate_tracker, label="RUN"):
    data = logger.as_arrays()
    t = data["t"]
    r = data["r_rel_lvh"]
    v = data["v_rel_lvh"]

    t0 = float(t[0]) if len(t) else 0.0
    tf = float(t[-1]) if len(t) else 0.0
    r0 = r[0] if len(r) else np.zeros(3)
    v0 = v[0] if len(v) else np.zeros(3)
    rf = r[-1] if len(r) else np.zeros(3)
    vf = v[-1] if len(v) else np.zeros(3)

    tof = float(cfg["simulation"]["t_end_s"])
    dt = float(cfg["simulation"]["dt_s"])

    dv_axis = metrics.get("dv_axis", np.zeros(3))
    dv_eq_applied = float(metrics.get("dv_eq_applied", 0.0))
    dv_eq_commanded = float(metrics.get("dv_eq_commanded", 0.0))
    peak_applied = float(metrics.get("peak_applied", 0.0))
    avg_applied = float(metrics.get("avg_applied", 0.0))

    duty_ddm = float(metrics.get("duty_ddm", 0.0))
    duty_medt = float(metrics.get("duty_medt", 0.0))

    start_line = (
        f"[{label}] START TOF={tof:.1f}s dt={dt:.2f}s "
        f"r_RTN={_fmt_vec(r0, 3)} |r|={np.linalg.norm(r0):.3f} "
        f"v_RTN={_fmt_vec(v0, 3)} |v|={np.linalg.norm(v0):.3f}"
    )
    end_line = (
        f"[{label}] END t={tf:.1f}s "
        f"r_RTN={_fmt_vec(rf, 3)} |r|={np.linalg.norm(rf):.3f} "
        f"v_RTN={_fmt_vec(vf, 3)} |v|={np.linalg.norm(vf):.3f}"
    )

    print(start_line)
    print(end_line)
    print(
        f"[{label}] TOF={tof:.1f}s dt={dt:.2f}s "
        f"dv_eq_applied={dv_eq_applied:.4f} "
        f"dv_RTN={_fmt_vec(dv_axis, 4)} "
        f"dv_eq_commanded={dv_eq_commanded:.4f}"
    )
    print(
        f"[{label}] peak(|a_applied|)={peak_applied:.6e} "
        f"avg(|a_applied|)={avg_applied:.6e} "
        f"duty_ddm={duty_ddm:.3f} duty_medt={duty_medt:.3f}"
    )
    print(
        f"[{label}] corridor_entry_success={metrics.get('corridor_entry_success', False)} "
        f"precapture_success={metrics.get('precapture_success', False)} "
        f"latch_success={metrics.get('latch_success', False)}"
    )

    if gate_tracker.crossings:
        print(f"[{label}] Gate crossings:")
        for name, info in gate_tracker.crossings.items():
            limits = gate_tracker.speed_limits.get(name, {})
            hard_max = limits.get("hard_max")
            ok = True
            if hard_max is not None and info["speed_mps"] > hard_max:
                ok = False
            status = "OK" if ok else "VIOLATION"
            print(
                f"  {name}: t={info['t']:.1f}s range={info['range_m']:.3f} "
                f"speed={info['speed_mps']:.3f} ({status})"
            )
    else:
        print(f"[{label}] Gate crossings: NONE")


def print_feasibility(feas, label="RUN"):
    print(f"Feasibility Estimate [{label}]")
    print("Axis | dv_req (m/s) | dv_eq_applied_max (m/s)")
    axes = ["R", "T", "N"]
    for i, axis in enumerate(axes):
        print(f" {axis}   | {feas['dv_req_axis'][i]:.5f}      | {feas['dv_eq_axis_max'][i]:.5f}")
    print(f"Total dv_req: {feas['dv_req_total']:.5f} m/s")
    print(f"Total dv_eq_applied_max: {feas['dv_eq_total_max']:.5f} m/s")
    print(f"Infeasible: {feas['infeasible']}  Reasons: {feas['reasons']}")


def print_targeting(report):
    if not report.get("enabled", False):
        print("Release Targeting: disabled")
        return
    r_orig = report.get("r_pred_original")
    r_tgt = report.get("r_pred_targeted")
    if r_orig is not None:
        print(f"Targeting original predicted miss: {np.linalg.norm(r_orig):.3f} m")
    if r_tgt is not None:
        print(f"Targeting targeted predicted miss: {np.linalg.norm(r_tgt):.3f} m")
    method = report.get("method")
    if method == "dv":
        dv0 = report.get("dv0", np.zeros(3))
        print(f"Targeting dv0 applied (RTN): {_fmt_vec(dv0, 4)}")
    elif method == "time":
        dt = report.get("dt", 0.0)
        print(f"Targeting dt offset applied: {dt:.1f} s")
