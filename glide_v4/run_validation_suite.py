"""Run GLIDE V4 validation MC campaigns and write summary artifacts."""

from __future__ import annotations

import csv
from concurrent.futures import ProcessPoolExecutor, as_completed
from pathlib import Path
import numpy as np

from mc.sampler import sample_config
from sim.runner import apply_release_targeting, run_sim_config
from sim.scenario import load_config, build_reference_state, get_initial_relative_state


CFG_PATH = "configs/dispersion_recovery.yaml"
DT_S = 5.0
BASELINE_RUNS = 1000
BASELINE_SEEDS = [123, 456, 789]
STRESS_RUNS = 500
STRESS_SEED = 123


def _stat_mean(values):
    arr = np.asarray(values, dtype=float)
    if arr.size == 0:
        return float("nan")
    return float(np.mean(arr))


def _stat_p95(values):
    arr = np.asarray(values, dtype=float)
    if arr.size == 0:
        return float("nan")
    return float(np.percentile(arr, 95))


def _stat_median(values):
    arr = np.asarray(values, dtype=float)
    if arr.size == 0:
        return float("nan")
    return float(np.median(arr))


def _stat_max(values):
    arr = np.asarray(values, dtype=float)
    if arr.size == 0:
        return float("nan")
    return float(np.max(arr))


def run_campaign(seed: int, runs: int, disp_scale: float, campaign: str) -> dict:
    cfg = load_config(CFG_PATH)
    rng = np.random.default_rng(seed)

    thresholds = cfg["gates"]["thresholds"]
    speed_limits = cfg["gates"]["speed_limits"]
    r_base = np.asarray(cfg.get("dispersion", {}).get("r_m", [0.0, 0.0, 0.0]), dtype=float)
    v_base = np.asarray(cfg.get("dispersion", {}).get("v_mps", [0.0, 0.0, 0.0]), dtype=float)

    counts = {k: 0 for k in ["R_ACQ", "R_COR", "R_PRE", "R_LATCH"]}
    violation_counts = {k: 0 for k in ["R_COR", "R_PRE", "R_LATCH"]}
    cor_speeds = []
    dv_tag = []
    dv_node = []
    cor_to_latch_dt = []

    for _ in range(runs):
        cfg_s = sample_config(cfg, rng)
        cfg_s["simulation"]["save_plots"] = False
        cfg_s["simulation"]["dt_s"] = float(DT_S)
        cfg_s["simulation"]["stop_on_gate"] = None
        cfg_s["terminal_capture"]["enabled"] = True
        cfg_s["terminal_capture"]["extend_mission"] = False

        if "dispersion" not in cfg_s:
            cfg_s["dispersion"] = {}
        cfg_s["dispersion"]["r_m"] = (r_base * disp_scale).tolist()
        cfg_s["dispersion"]["v_mps"] = (v_base * disp_scale).tolist()

        r_ref, v_ref = build_reference_state(cfg_s)
        r0, v0 = get_initial_relative_state(cfg_s)
        r_nom, v_nom, _ = apply_release_targeting(cfg_s, r_ref, v_ref, r0, v0)

        r_bounds = np.asarray(cfg_s["dispersion"].get("r_m", [0.0, 0.0, 0.0]), dtype=float)
        v_bounds = np.asarray(cfg_s["dispersion"].get("v_mps", [0.0, 0.0, 0.0]), dtype=float)
        r_act = r_nom + rng.uniform(-r_bounds, r_bounds)
        v_act = v_nom + rng.uniform(-v_bounds, v_bounds)

        metrics, gates, _ = run_sim_config(
            cfg_s,
            control_enabled=True,
            guidance_enabled=True,
            r_rel_lvh0=r_act,
            v_rel_lvh0=v_act,
            r_rel_lvh_nom0=r_nom,
            v_rel_lvh_nom0=v_nom,
            fast_mode=True,
        )

        for gate_name in counts:
            if gate_name in gates.crossings:
                counts[gate_name] += 1

        if "R_COR" in gates.crossings:
            cor_speeds.append(float(gates.crossings["R_COR"]["speed_mps"]))

        for vio in gates.violations:
            if vio.get("type") != "speed_violation":
                continue
            gate_name = vio.get("gate")
            if gate_name in violation_counts:
                violation_counts[gate_name] += 1

        dv_tag.append(float(metrics.get("dv_tag", 0.0)))
        dv_node.append(float(metrics.get("dv_node_terminal", 0.0)))

        if "R_COR" in gates.crossings and "R_LATCH" in gates.crossings:
            dt_rl = float(gates.crossings["R_LATCH"]["t"] - gates.crossings["R_COR"]["t"])
            cor_to_latch_dt.append(dt_rl)

    p_acq = counts["R_ACQ"] / max(runs, 1)
    p_cor = counts["R_COR"] / max(runs, 1)
    p_pre = counts["R_PRE"] / max(runs, 1)
    p_latch = counts["R_LATCH"] / max(runs, 1)

    r_cor_hard_max = float(speed_limits["R_COR"]["hard_max"])
    any_r_cor_hard_cap_violation = bool(
        violation_counts["R_COR"] > 0 or (len(cor_speeds) > 0 and np.max(cor_speeds) > r_cor_hard_max)
    )

    return {
        "campaign": campaign,
        "seed": int(seed),
        "runs": int(runs),
        "dt_s": float(DT_S),
        "dispersion_scale": float(disp_scale),
        "p_r_acq": p_acq,
        "p_r_cor": p_cor,
        "p_r_pre": p_pre,
        "p_r_latch": p_latch,
        "r_cor_speed_mean_mps": _stat_mean(cor_speeds),
        "r_cor_speed_p95_mps": _stat_p95(cor_speeds),
        "r_cor_speed_max_mps": _stat_max(cor_speeds),
        "viol_r_cor_count": int(violation_counts["R_COR"]),
        "viol_r_pre_count": int(violation_counts["R_PRE"]),
        "viol_r_latch_count": int(violation_counts["R_LATCH"]),
        "dv_tag_median_mps": _stat_median(dv_tag),
        "dv_tag_p95_mps": _stat_p95(dv_tag),
        "dv_tag_max_mps": _stat_max(dv_tag),
        "dv_node_median_mps": _stat_median(dv_node),
        "dv_node_p95_mps": _stat_p95(dv_node),
        "dv_node_max_mps": _stat_max(dv_node),
        "t_cor_to_latch_mean_s": _stat_mean(cor_to_latch_dt),
        "t_cor_to_latch_p95_s": _stat_p95(cor_to_latch_dt),
        "any_r_cor_hard_cap_violation": any_r_cor_hard_cap_violation,
        "r_cor_hard_max_mps": r_cor_hard_max,
        "r_acq_m": float(thresholds["R_ACQ"]),
        "r_cor_m": float(thresholds["R_COR"]),
        "r_pre_m": float(thresholds["R_PRE"]),
        "r_latch_m": float(thresholds["R_LATCH"]),
    }


def _write_csv(path: Path, rows: list[dict]):
    if not rows:
        return
    fieldnames = list(rows[0].keys())
    with path.open("w", newline="", encoding="ascii") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def _fmt_pct(x: float) -> str:
    return f"{x:.3f}"


def _fmt_num(x: float) -> str:
    if np.isnan(x):
        return "nan"
    return f"{x:.3f}"


def main():
    out_dir = Path("outputs")
    out_dir.mkdir(parents=True, exist_ok=True)

    baseline_rows = []
    print("Running baseline robustness: 1x dispersion, terminal ON, dt=5s, N=1000 for seeds 123/456/789")
    with ProcessPoolExecutor(max_workers=min(3, len(BASELINE_SEEDS))) as ex:
        futs = {
            ex.submit(run_campaign, seed, BASELINE_RUNS, 1.0, "baseline_1x"): seed
            for seed in BASELINE_SEEDS
        }
        for fut in as_completed(futs):
            row = fut.result()
            baseline_rows.append(row)
            print(
                f"  seed={row['seed']} P(R_COR)={row['p_r_cor']:.3f} "
                f"P(R_LATCH)={row['p_r_latch']:.3f} "
                f"R_COR p95={row['r_cor_speed_p95_mps']:.3f} max={row['r_cor_speed_max_mps']:.3f}"
            )

    baseline_rows.sort(key=lambda r: r["seed"])
    baseline_csv = out_dir / "mc_baseline_seeds.csv"
    _write_csv(baseline_csv, baseline_rows)

    print("Running stress test: 2x dispersion, terminal ON, dt=5s, N=500, seed=123")
    stress_row = run_campaign(STRESS_SEED, STRESS_RUNS, 2.0, "stress_2x")
    stress_csv = out_dir / "mc_2x.csv"
    _write_csv(stress_csv, [stress_row])
    print(
        f"  stress P(R_COR)={stress_row['p_r_cor']:.3f} "
        f"P(R_LATCH)={stress_row['p_r_latch']:.3f} "
        f"R_COR p95={stress_row['r_cor_speed_p95_mps']:.3f} max={stress_row['r_cor_speed_max_mps']:.3f} "
        f"hard_cap_violation={stress_row['any_r_cor_hard_cap_violation']}"
    )

    worst_p_cor = min(baseline_rows, key=lambda r: r["p_r_cor"])
    worst_p_latch = min(baseline_rows, key=lambda r: r["p_r_latch"])

    summary_md = out_dir / "GLIDE_V4_validation_summary.md"
    lines = []
    lines.append("# GLIDE V4 Validation Summary")
    lines.append("")
    lines.append("## Baseline Tuning")
    lines.append("- lead_s = 300")
    lines.append("- cw_speed_weight = 0.0")
    lines.append("- r_cor_speed_gain = 0.5")
    lines.append("- corridor radii and official speed caps unchanged")
    lines.append("")
    lines.append("## Baseline Robustness (1x dispersion, terminal ON, dt=5s, N=1000)")
    lines.append("")
    lines.append("| Seed | P(R_ACQ) | P(R_COR) | P(R_PRE) | P(R_LATCH) | R_COR mean | R_COR p95 | R_COR max | Viol R_COR | Viol R_PRE | Viol R_LATCH | dv_tag med/p95/max | dv_node med/p95/max | t(R_COR->R_LATCH) mean/p95 |")
    lines.append("|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---|---|---|")
    for r in baseline_rows:
        lines.append(
            f"| {r['seed']} | {_fmt_pct(r['p_r_acq'])} | {_fmt_pct(r['p_r_cor'])} | {_fmt_pct(r['p_r_pre'])} | {_fmt_pct(r['p_r_latch'])} | "
            f"{_fmt_num(r['r_cor_speed_mean_mps'])} | {_fmt_num(r['r_cor_speed_p95_mps'])} | {_fmt_num(r['r_cor_speed_max_mps'])} | "
            f"{r['viol_r_cor_count']} | {r['viol_r_pre_count']} | {r['viol_r_latch_count']} | "
            f"{_fmt_num(r['dv_tag_median_mps'])}/{_fmt_num(r['dv_tag_p95_mps'])}/{_fmt_num(r['dv_tag_max_mps'])} | "
            f"{_fmt_num(r['dv_node_median_mps'])}/{_fmt_num(r['dv_node_p95_mps'])}/{_fmt_num(r['dv_node_max_mps'])} | "
            f"{_fmt_num(r['t_cor_to_latch_mean_s'])}/{_fmt_num(r['t_cor_to_latch_p95_s'])} |"
        )
    lines.append("")
    lines.append("Worst-case across baseline seeds:")
    lines.append(
        f"- Worst P(R_COR): {_fmt_pct(worst_p_cor['p_r_cor'])} at seed {worst_p_cor['seed']}"
    )
    lines.append(
        f"- Worst P(R_LATCH): {_fmt_pct(worst_p_latch['p_r_latch'])} at seed {worst_p_latch['seed']}"
    )
    lines.append("")
    lines.append("## Stress Test (2x dispersion, terminal ON, dt=5s, N=500, seed=123)")
    lines.append("")
    lines.append(
        f"- P(R_ACQ)={_fmt_pct(stress_row['p_r_acq'])}, P(R_COR)={_fmt_pct(stress_row['p_r_cor'])}, "
        f"P(R_PRE)={_fmt_pct(stress_row['p_r_pre'])}, P(R_LATCH)={_fmt_pct(stress_row['p_r_latch'])}"
    )
    lines.append(
        f"- R_COR speed mean/p95/max = {_fmt_num(stress_row['r_cor_speed_mean_mps'])}/"
        f"{_fmt_num(stress_row['r_cor_speed_p95_mps'])}/{_fmt_num(stress_row['r_cor_speed_max_mps'])} m/s"
    )
    lines.append(
        f"- Gate speed violations counts (R_COR/R_PRE/R_LATCH) = "
        f"{stress_row['viol_r_cor_count']}/{stress_row['viol_r_pre_count']}/{stress_row['viol_r_latch_count']}"
    )
    lines.append(
        f"- dv_tag med/p95/max = {_fmt_num(stress_row['dv_tag_median_mps'])}/"
        f"{_fmt_num(stress_row['dv_tag_p95_mps'])}/{_fmt_num(stress_row['dv_tag_max_mps'])} m/s"
    )
    lines.append(
        f"- dv_node_terminal med/p95/max = {_fmt_num(stress_row['dv_node_median_mps'])}/"
        f"{_fmt_num(stress_row['dv_node_p95_mps'])}/{_fmt_num(stress_row['dv_node_max_mps'])} m/s"
    )
    lines.append(
        f"- t(R_COR->R_LATCH) mean/p95 = {_fmt_num(stress_row['t_cor_to_latch_mean_s'])}/"
        f"{_fmt_num(stress_row['t_cor_to_latch_p95_s'])} s"
    )
    lines.append(
        f"- Any R_COR hard cap (0.35 m/s) violation: {stress_row['any_r_cor_hard_cap_violation']}"
    )
    lines.append("")
    lines.append("## Artifacts")
    lines.append(f"- `{baseline_csv.as_posix()}`")
    lines.append(f"- `{stress_csv.as_posix()}`")
    lines.append(f"- `{summary_md.as_posix()}`")
    summary_md.write_text("\n".join(lines), encoding="ascii")

    print(f"Wrote {baseline_csv}")
    print(f"Wrote {stress_csv}")
    print(f"Wrote {summary_md}")


if __name__ == "__main__":
    main()
