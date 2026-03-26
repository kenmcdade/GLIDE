"""Characterize baseline failures and apply one-lever improvement test."""

from __future__ import annotations

import csv
from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt

from mc.sampler import sample_config
from sim.runner import apply_release_targeting, run_sim_config
from sim.scenario import load_config, build_reference_state, get_initial_relative_state


CFG_PATH = "configs/dispersion_recovery.yaml"
DT_S = 5.0
BASELINE_RUNS = 1000
BASELINE_SEED = 123
TUNE_PILOT_RUNS = 200
TUNE_FINAL_RUNS = 500


def _stat(values, fn):
    arr = np.asarray(values, dtype=float)
    if arr.size == 0:
        return float("nan")
    return float(fn(arr))


def _stat_median(values):
    return _stat(values, np.median)


def _stat_p95(values):
    return _stat(values, lambda x: np.percentile(x, 95))


def _stat_mean(values):
    return _stat(values, np.mean)


def _stat_max(values):
    return _stat(values, np.max)


def _write_csv(path: Path, rows: list[dict]):
    if not rows:
        return
    with path.open("w", newline="", encoding="ascii") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def _failure_cause(gates, metrics, pred_miss_m, r_disp, v_disp):
    for vio in gates.violations:
        if vio.get("type") == "speed_violation":
            return f"speed_violation_at_{vio.get('gate', 'unknown')}"

    if "R_COR" not in gates.crossings:
        abs_r_t = abs(float(r_disp[1]))
        abs_v_t = abs(float(v_disp[1]))
        if abs_v_t >= 0.035:
            return "no_R_COR_large_alongtrack_velocity_dispersion"
        if abs_r_t >= 150.0:
            return "no_R_COR_large_alongtrack_position_dispersion"
        return "no_R_COR_moderate_coupled_dispersion"

    if "R_PRE" not in gates.crossings:
        return "no_R_PRE_after_R_COR"
    if "R_LATCH" not in gates.crossings:
        if metrics.get("term_sat_frac", 0.0) > 0.10:
            return "no_R_LATCH_terminal_saturation"
        return "no_R_LATCH_terminal_settle"

    return "unknown_failure"


def _bin_failure_rate(values, failures, bins=12):
    vals = np.asarray(values, dtype=float)
    flg = np.asarray(failures, dtype=bool)
    if vals.size == 0:
        return []

    lo = float(np.min(vals))
    hi = float(np.max(vals))
    if hi <= lo:
        edges = np.array([lo, hi + 1e-9], dtype=float)
    else:
        edges = np.linspace(lo, hi, bins + 1)

    rows = []
    for i in range(len(edges) - 1):
        left = float(edges[i])
        right = float(edges[i + 1])
        if i == len(edges) - 2:
            mask = (vals >= left) & (vals <= right)
        else:
            mask = (vals >= left) & (vals < right)
        n = int(np.sum(mask))
        n_fail = int(np.sum(flg[mask])) if n > 0 else 0
        rate = float(n_fail / n) if n > 0 else float("nan")
        rows.append(
            {
                "bin_low": left,
                "bin_high": right,
                "count": n,
                "fail_count": n_fail,
                "fail_rate": rate,
                "bin_center": 0.5 * (left + right),
            }
        )
    return rows


def _plot_failure_rate(rows, title, xlabel, out_path: Path):
    x = [r["bin_center"] for r in rows if not np.isnan(r["fail_rate"])]
    y = [r["fail_rate"] for r in rows if not np.isnan(r["fail_rate"])]
    plt.figure(figsize=(8, 5))
    plt.plot(x, y, marker="o", linewidth=1.5)
    plt.ylim(0.0, 1.0)
    plt.grid(True, alpha=0.3)
    plt.title(title)
    plt.xlabel(xlabel)
    plt.ylabel("Failure rate")
    plt.tight_layout()
    plt.savefig(out_path, dpi=150)
    plt.close()


def run_mc_with_features(runs, seed, disp_scale=1.0, t_end_s_override=None):
    cfg = load_config(CFG_PATH)
    rng = np.random.default_rng(seed)

    r_base = np.asarray(cfg.get("dispersion", {}).get("r_m", [0.0, 0.0, 0.0]), dtype=float)
    v_base = np.asarray(cfg.get("dispersion", {}).get("v_mps", [0.0, 0.0, 0.0]), dtype=float)

    all_rows = []
    failed_rows = []
    cor_speeds = []
    dv_tag_vals = []
    dv_node_vals = []
    cor_to_latch = []
    counts = {k: 0 for k in ["R_ACQ", "R_COR", "R_PRE", "R_LATCH"]}
    vio_counts = {k: 0 for k in ["R_COR", "R_PRE", "R_LATCH"]}

    for trial_idx in range(runs):
        cfg_s = sample_config(cfg, rng)
        cfg_s["simulation"]["save_plots"] = False
        cfg_s["simulation"]["dt_s"] = float(DT_S)
        cfg_s["simulation"]["stop_on_gate"] = None
        cfg_s["terminal_capture"]["enabled"] = True
        cfg_s["terminal_capture"]["extend_mission"] = False
        if t_end_s_override is not None:
            cfg_s["simulation"]["t_end_s"] = float(t_end_s_override)

        cfg_s["dispersion"]["r_m"] = (r_base * disp_scale).tolist()
        cfg_s["dispersion"]["v_mps"] = (v_base * disp_scale).tolist()

        r_ref, v_ref = build_reference_state(cfg_s)
        r0, v0 = get_initial_relative_state(cfg_s)
        r_nom, v_nom, target_report = apply_release_targeting(cfg_s, r_ref, v_ref, r0, v0)
        pred_miss = target_report.get("r_pred_targeted")
        if pred_miss is None:
            pred_miss = target_report.get("r_pred_original")
        pred_miss_m = float(np.linalg.norm(pred_miss)) if pred_miss is not None else float("nan")

        r_bounds = np.asarray(cfg_s["dispersion"].get("r_m", [0.0, 0.0, 0.0]), dtype=float)
        v_bounds = np.asarray(cfg_s["dispersion"].get("v_mps", [0.0, 0.0, 0.0]), dtype=float)
        r_disp = rng.uniform(-r_bounds, r_bounds)
        v_disp = rng.uniform(-v_bounds, v_bounds)

        metrics, gates, _ = run_sim_config(
            cfg_s,
            control_enabled=True,
            guidance_enabled=True,
            r_rel_lvh0=r_nom + r_disp,
            v_rel_lvh0=v_nom + v_disp,
            r_rel_lvh_nom0=r_nom,
            v_rel_lvh_nom0=v_nom,
            fast_mode=True,
        )

        for gate_name in counts:
            if gate_name in gates.crossings:
                counts[gate_name] += 1

        for vio in gates.violations:
            if vio.get("type") != "speed_violation":
                continue
            gate_name = vio.get("gate")
            if gate_name in vio_counts:
                vio_counts[gate_name] += 1

        if "R_COR" in gates.crossings:
            cor_speeds.append(float(gates.crossings["R_COR"]["speed_mps"]))
        if "R_COR" in gates.crossings and "R_LATCH" in gates.crossings:
            cor_to_latch.append(
                float(gates.crossings["R_LATCH"]["t"] - gates.crossings["R_COR"]["t"])
            )

        dv_tag_vals.append(float(metrics.get("dv_tag", 0.0)))
        dv_node_vals.append(float(metrics.get("dv_node_terminal", 0.0)))

        failed = not bool(metrics.get("latch_success", False))
        cause = _failure_cause(gates, metrics, pred_miss_m, r_disp, v_disp) if failed else "success"
        r_cor_t = gates.crossings["R_COR"]["t"] if "R_COR" in gates.crossings else float("nan")
        r_cor_v = gates.crossings["R_COR"]["speed_mps"] if "R_COR" in gates.crossings else float("nan")

        row = {
            "trial": int(trial_idx),
            "failed": bool(failed),
            "cause": cause,
            "disp_r_R_m": float(r_disp[0]),
            "disp_r_T_m": float(r_disp[1]),
            "disp_r_N_m": float(r_disp[2]),
            "disp_v_R_mps": float(v_disp[0]),
            "disp_v_T_mps": float(v_disp[1]),
            "disp_v_N_mps": float(v_disp[2]),
            "disp_r_norm_m": float(np.linalg.norm(r_disp)),
            "disp_v_norm_mps": float(np.linalg.norm(v_disp)),
            "predicted_miss_m": pred_miss_m,
            "t_closest_s": float(metrics.get("t_closest_s", float("nan"))),
            "min_range_m": float(metrics.get("min_range_m", float("nan"))),
            "r_cor_entry_t_s": float(r_cor_t),
            "r_cor_entry_speed_mps": float(r_cor_v),
            "dv_tag_total_mps": float(metrics.get("dv_tag", 0.0)),
            "dv_node_total_mps": float(metrics.get("dv_node_terminal", 0.0)),
            "tag_ddm_duty": float(metrics.get("duty_ddm", float("nan"))),
            "tag_medt_duty": float(metrics.get("duty_medt", float("nan"))),
            "tag_sat_cmd_frac": float(metrics.get("tag_sat_cmd_frac", float("nan"))),
            "terminal_sat_frac": float(metrics.get("term_sat_frac", float("nan"))),
            "terminal_sat_steps": int(metrics.get("term_sat_steps", 0)),
            "terminal_steps": int(metrics.get("term_steps", 0)),
        }

        all_rows.append(row)
        if failed:
            failed_rows.append(row)

    summary = {
        "runs": int(runs),
        "p_r_acq": counts["R_ACQ"] / max(runs, 1),
        "p_r_cor": counts["R_COR"] / max(runs, 1),
        "p_r_pre": counts["R_PRE"] / max(runs, 1),
        "p_r_latch": counts["R_LATCH"] / max(runs, 1),
        "r_cor_speed_mean_mps": _stat_mean(cor_speeds),
        "r_cor_speed_p95_mps": _stat_p95(cor_speeds),
        "r_cor_speed_max_mps": _stat_max(cor_speeds),
        "viol_r_cor_count": int(vio_counts["R_COR"]),
        "viol_r_pre_count": int(vio_counts["R_PRE"]),
        "viol_r_latch_count": int(vio_counts["R_LATCH"]),
        "dv_tag_median_mps": _stat_median(dv_tag_vals),
        "dv_tag_p95_mps": _stat_p95(dv_tag_vals),
        "dv_tag_max_mps": _stat_max(dv_tag_vals),
        "dv_node_median_mps": _stat_median(dv_node_vals),
        "dv_node_p95_mps": _stat_p95(dv_node_vals),
        "dv_node_max_mps": _stat_max(dv_node_vals),
        "t_cor_to_latch_mean_s": _stat_mean(cor_to_latch),
        "t_cor_to_latch_p95_s": _stat_p95(cor_to_latch),
        "failed_count": len(failed_rows),
    }
    return summary, all_rows, failed_rows


def main():
    out_dir = Path("outputs")
    out_dir.mkdir(parents=True, exist_ok=True)

    print("Running baseline failure characterization: N=1000, dt=5s, seed=123")
    baseline_summary, baseline_rows, failed_rows = run_mc_with_features(
        runs=BASELINE_RUNS,
        seed=BASELINE_SEED,
        disp_scale=1.0,
        tag_max_accel=None,
    )

    # Per-failed-trial compact feature vectors.
    failed_csv = out_dir / "baseline_failed_trials_features.csv"
    _write_csv(failed_csv, failed_rows)

    # Ranked failure breakdown (top 3).
    cause_counts = {}
    for row in failed_rows:
        cause_counts[row["cause"]] = cause_counts.get(row["cause"], 0) + 1
    ranked = sorted(cause_counts.items(), key=lambda x: x[1], reverse=True)
    top3_rows = []
    for cause, count in ranked[:3]:
        top3_rows.append(
            {
                "cause": cause,
                "count": int(count),
                "pct_of_failures": float(count / max(len(failed_rows), 1)),
            }
        )
    top3_csv = out_dir / "baseline_failure_breakdown_top3.csv"
    _write_csv(top3_csv, top3_rows)

    # Failure-rate correlations vs |disp_r| and |disp_v|.
    disp_r_norm = [r["disp_r_norm_m"] for r in baseline_rows]
    disp_v_norm = [r["disp_v_norm_mps"] for r in baseline_rows]
    failures = [r["failed"] for r in baseline_rows]

    bins_r = _bin_failure_rate(disp_r_norm, failures, bins=12)
    bins_v = _bin_failure_rate(disp_v_norm, failures, bins=12)

    corr_rows = []
    for row in bins_r:
        corr_rows.append({"axis": "disp_r_norm_m", **row})
    for row in bins_v:
        corr_rows.append({"axis": "disp_v_norm_mps", **row})
    corr_csv = out_dir / "baseline_failure_rate_vs_dispersion.csv"
    _write_csv(corr_csv, corr_rows)

    plot_r = out_dir / "failure_rate_vs_disp_r.png"
    plot_v = out_dir / "failure_rate_vs_disp_v.png"
    _plot_failure_rate(
        bins_r,
        title="Failure Rate vs |disp_r|",
        xlabel="|disp_r| (m)",
        out_path=plot_r,
    )
    _plot_failure_rate(
        bins_v,
        title="Failure Rate vs |disp_v|",
        xlabel="|disp_v| (m/s)",
        out_path=plot_v,
    )

    # One-lever tuning: slightly longer TOF only.
    lever_candidates = [7350.0, 7500.0, 7800.0]
    pilot_rows = []
    print("Running one-lever pilot sweep (simulation.t_end_s): N=200 each")
    for val in lever_candidates:
        summary, _, _ = run_mc_with_features(
            runs=TUNE_PILOT_RUNS,
            seed=BASELINE_SEED,
            disp_scale=1.0,
            t_end_s_override=val,
        )
        summary["simulation_t_end_s"] = val
        pilot_rows.append(summary)
        print(
            f"  t_end_s={val:.0f} P(R_COR)={summary['p_r_cor']:.3f} "
            f"P(R_LATCH)={summary['p_r_latch']:.3f} "
            f"viol={summary['viol_r_cor_count']}/{summary['viol_r_pre_count']}/{summary['viol_r_latch_count']} "
            f"R_COR max={summary['r_cor_speed_max_mps']:.3f}"
        )

    pilot_csv = out_dir / "baseline_tuning_pilot_tof.csv"
    _write_csv(pilot_csv, pilot_rows)

    # Pick smallest candidate that satisfies constraints; else best P(R_COR) with constraints.
    feasible = [
        s
        for s in pilot_rows
        if s["p_r_cor"] >= 0.93
        and s["viol_r_cor_count"] == 0
        and s["viol_r_pre_count"] == 0
        and s["viol_r_latch_count"] == 0
        and s["r_cor_speed_max_mps"] <= 0.35
    ]
    if feasible:
        selected = min(feasible, key=lambda x: x["simulation_t_end_s"])
    else:
        constrained = [
            s
            for s in pilot_rows
            if s["viol_r_cor_count"] == 0
            and s["viol_r_pre_count"] == 0
            and s["viol_r_latch_count"] == 0
            and s["r_cor_speed_max_mps"] <= 0.35
        ]
        if constrained:
            selected = max(constrained, key=lambda x: x["p_r_cor"])
        else:
            selected = max(pilot_rows, key=lambda x: x["p_r_cor"])

    selected_tof = float(selected["simulation_t_end_s"])
    print(f"Selected one-lever change: simulation.t_end_s -> {selected_tof:.0f}")
    tuned_summary, _, _ = run_mc_with_features(
        runs=TUNE_FINAL_RUNS,
        seed=BASELINE_SEED,
        disp_scale=1.0,
        t_end_s_override=selected_tof,
    )
    tuned_summary["simulation_t_end_s"] = selected_tof
    tuned_csv = out_dir / "baseline_tuned_n500.csv"
    _write_csv(tuned_csv, [tuned_summary])

    # Markdown summary.
    summary_md = out_dir / "baseline_failure_mode_summary.md"
    lines = []
    lines.append("# Baseline Failure Mode Characterization")
    lines.append("")
    lines.append("## Baseline Campaign")
    lines.append("- Config: baseline 1x, terminal ON, dt=5s")
    lines.append(f"- Trials: {BASELINE_RUNS}, seed={BASELINE_SEED}")
    lines.append(
        f"- P(R_COR)={baseline_summary['p_r_cor']:.3f}, "
        f"P(R_LATCH)={baseline_summary['p_r_latch']:.3f}, "
        f"failures={baseline_summary['failed_count']}"
    )
    lines.append("")
    lines.append("## Top 3 Failure Causes")
    for i, row in enumerate(top3_rows, start=1):
        lines.append(
            f"{i}. {row['cause']}: count={row['count']} "
            f"({100.0 * row['pct_of_failures']:.1f}% of failures)"
        )
    lines.append("")
    lines.append("## One-Lever Change")
    lines.append("- Lever: `simulation.t_end_s` only (slightly longer TOF)")
    lines.append(f"- Selected value: `{selected_tof:.0f} s` (lead_s kept at 300 s)")
    lines.append(f"- Validation: baseline 1x, terminal ON, dt=5s, N={TUNE_FINAL_RUNS}, seed={BASELINE_SEED}")
    lines.append(
        f"- New P(R_COR)={tuned_summary['p_r_cor']:.3f}, "
        f"P(R_LATCH)={tuned_summary['p_r_latch']:.3f}"
    )
    lines.append(
        f"- R_COR speed mean/p95/max = {tuned_summary['r_cor_speed_mean_mps']:.3f}/"
        f"{tuned_summary['r_cor_speed_p95_mps']:.3f}/"
        f"{tuned_summary['r_cor_speed_max_mps']:.3f} m/s"
    )
    lines.append(
        f"- Gate speed violations (R_COR/R_PRE/R_LATCH) = "
        f"{tuned_summary['viol_r_cor_count']}/"
        f"{tuned_summary['viol_r_pre_count']}/"
        f"{tuned_summary['viol_r_latch_count']}"
    )
    lines.append("")
    lines.append("## Artifacts")
    lines.append(f"- `{failed_csv.as_posix()}`")
    lines.append(f"- `{top3_csv.as_posix()}`")
    lines.append(f"- `{corr_csv.as_posix()}`")
    lines.append(f"- `{plot_r.as_posix()}`")
    lines.append(f"- `{plot_v.as_posix()}`")
    lines.append(f"- `{pilot_csv.as_posix()}`")
    lines.append(f"- `{tuned_csv.as_posix()}`")
    summary_md.write_text("\n".join(lines), encoding="ascii")

    print(f"Wrote {failed_csv}")
    print(f"Wrote {top3_csv}")
    print(f"Wrote {corr_csv}")
    print(f"Wrote {plot_r}")
    print(f"Wrote {plot_v}")
    print(f"Wrote {pilot_csv}")
    print(f"Wrote {tuned_csv}")
    print(f"Wrote {summary_md}")


if __name__ == "__main__":
    main()
