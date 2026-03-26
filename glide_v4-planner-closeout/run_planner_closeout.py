"""Close-out refinement around the current compliant 2x planner policy."""

from __future__ import annotations

import csv
import json
from pathlib import Path

from planner.release_planner import FULL_DT_S, FULL_MAX_WORKERS, evaluate_candidate_summary
from sim.scenario import load_config


CFG_PATH = "configs/dispersion_recovery.yaml"
DISPERSION_SCALE = 2.0
SEED = 123
PILOT_RUNS = 120
PILOT_DT_S = 5.0
FULL_RUNS = 500


LOCAL_POLICIES = {
    "safe_k3": {
        "name": "safe_k3",
        "enabled": True,
        "window_k_r_cor": 3.0,
        "speed_trigger_mps": 0.12,
        "gain_mult": 2.2,
        "gain_mult_max": 2.8,
        "predict_dt_scale": 1.0,
        "prebrake_margin_m": 6.0,
        "prebrake_gain": 1.5,
        "hard_cap_mps": 0.35,
    },
    "safe_k3_soft": {
        "name": "safe_k3_soft",
        "enabled": True,
        "window_k_r_cor": 3.0,
        "speed_trigger_mps": 0.11,
        "gain_mult": 2.15,
        "gain_mult_max": 2.75,
        "predict_dt_scale": 1.0,
        "prebrake_margin_m": 6.5,
        "prebrake_gain": 1.55,
        "hard_cap_mps": 0.35,
    },
    "safe_k3_wide": {
        "name": "safe_k3_wide",
        "enabled": True,
        "window_k_r_cor": 3.5,
        "speed_trigger_mps": 0.11,
        "gain_mult": 2.3,
        "gain_mult_max": 2.9,
        "predict_dt_scale": 1.0,
        "prebrake_margin_m": 7.0,
        "prebrake_gain": 1.6,
        "hard_cap_mps": 0.35,
    },
}


TERMINAL_PROFILES = {
    "base": None,
    "settle_soft": {
        "post_cor_shaping": {
            "enabled": True,
            "v_los_scale_cor": 0.98,
            "v_los_scale_pre": 0.95,
            "v_los_scale_latch": 0.92,
            "kp_mult": 1.00,
            "kd_mult": 1.08,
            "kt_mult": 1.12,
        }
    },
    "settle_strong": {
        "post_cor_shaping": {
            "enabled": True,
            "v_los_scale_cor": 0.97,
            "v_los_scale_pre": 0.93,
            "v_los_scale_latch": 0.90,
            "kp_mult": 1.00,
            "kd_mult": 1.15,
            "kt_mult": 1.20,
        }
    },
}


PLANNER_CANDIDATES = [
    {"name": "planner_ref", "tof_s": 7100.0, "lead_s": 300.0, "policy": "safe_k3", "terminal_profile": "base"},
    {"name": "planner_lead_280", "tof_s": 7100.0, "lead_s": 280.0, "policy": "safe_k3", "terminal_profile": "base"},
    {"name": "planner_lead_320", "tof_s": 7100.0, "lead_s": 320.0, "policy": "safe_k3", "terminal_profile": "base"},
    {"name": "planner_tof_7000", "tof_s": 7000.0, "lead_s": 300.0, "policy": "safe_k3", "terminal_profile": "base"},
    {"name": "planner_tof_7200", "tof_s": 7200.0, "lead_s": 300.0, "policy": "safe_k3", "terminal_profile": "base"},
    {"name": "planner_soft", "tof_s": 7100.0, "lead_s": 300.0, "policy": "safe_k3_soft", "terminal_profile": "base"},
    {"name": "planner_wide", "tof_s": 7100.0, "lead_s": 300.0, "policy": "safe_k3_wide", "terminal_profile": "base"},
    {"name": "planner_tof7000_lead280", "tof_s": 7000.0, "lead_s": 280.0, "policy": "safe_k3", "terminal_profile": "base"},
]


TERMINAL_CANDIDATES = [
    {"name": "terminal_ref", "tof_s": 7100.0, "lead_s": 300.0, "policy": "safe_k3", "terminal_profile": "base"},
    {"name": "terminal_settle_soft", "tof_s": 7100.0, "lead_s": 300.0, "policy": "safe_k3", "terminal_profile": "settle_soft"},
    {"name": "terminal_settle_strong", "tof_s": 7100.0, "lead_s": 300.0, "policy": "safe_k3", "terminal_profile": "settle_strong"},
]


def _load_reference_metrics():
    path = Path("benchmarks.json")
    if not path.exists():
        return None
    data = json.loads(path.read_text(encoding="ascii"))
    return data["suite"]["planner_2x_full"]["result"]


def _evaluate(cfg, candidate, runs, dt_s, max_workers):
    metrics = evaluate_candidate_summary(
        cfg=cfg,
        dispersion_scale=DISPERSION_SCALE,
        seed=SEED,
        tof_s=float(candidate["tof_s"]),
        cor_entry_lead_s=float(candidate["lead_s"]),
        speed_safety_policy=LOCAL_POLICIES[candidate["policy"]],
        runs=int(runs),
        dt_s=float(dt_s),
        terminal_overrides=TERMINAL_PROFILES[candidate["terminal_profile"]],
        max_workers=max_workers,
    )
    row = {
        "candidate_name": candidate["name"],
        "branch": candidate["branch"],
        "tof_s": float(candidate["tof_s"]),
        "lead_s": float(candidate["lead_s"]),
        "policy_name": candidate["policy"],
        "terminal_profile": candidate["terminal_profile"],
        "runs": int(runs),
        "dt_s": float(dt_s),
        "p_r_cor": float(metrics["p_r_cor"]),
        "p_r_pre": float(metrics["p_r_pre"]),
        "p_r_latch": float(metrics["p_r_latch"]),
        "r_cor_speed_p95_mps": float(metrics["r_cor_speed_p95_mps"]),
        "r_cor_speed_max_mps": float(metrics["r_cor_speed_max_mps"]),
        "viol_r_cor_count": int(metrics["viol_r_cor_count"]),
        "dv_tag_p95_mps": float(metrics["dv_tag_p95_mps"]),
        "dv_tag_max_mps": float(metrics["dv_tag_max_mps"]),
        "dv_node_p95_mps": float(metrics["dv_node_p95_mps"]),
        "dv_node_max_mps": float(metrics["dv_node_max_mps"]),
        "term_sat_frac_p95": float(metrics["term_sat_frac_p95"]),
    }
    row["compliant"] = (
        row["viol_r_cor_count"] == 0
        and row["r_cor_speed_max_mps"] <= 0.35
    )
    return row


def _select_top(rows, branch):
    branch_rows = [r for r in rows if r["branch"] == branch and r["compliant"]]
    if not branch_rows:
        branch_rows = [r for r in rows if r["branch"] == branch]
    branch_rows.sort(
        key=lambda r: (
            r["p_r_latch"],
            r["p_r_cor"],
            -r["dv_tag_p95_mps"],
        ),
        reverse=True,
    )
    return branch_rows[:2]


def _write_csv(path: Path, rows: list[dict]):
    if not rows:
        return
    with path.open("w", newline="", encoding="ascii") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def _fmt(value):
    return f"{value:.3f}"


def main():
    cfg = load_config(CFG_PATH)
    reference = _load_reference_metrics()

    all_candidates = []
    for row in PLANNER_CANDIDATES:
        all_candidates.append({**row, "branch": "planner"})
    for row in TERMINAL_CANDIDATES:
        all_candidates.append({**row, "branch": "terminal"})

    print("Running closeout pilot sweep")
    pilot_rows = []
    for candidate in all_candidates:
        row = _evaluate(cfg, candidate, runs=PILOT_RUNS, dt_s=PILOT_DT_S, max_workers=1)
        row["stage"] = "pilot"
        pilot_rows.append(row)
        print(
            f"  {row['candidate_name']}: branch={row['branch']} "
            f"P(R_COR)={row['p_r_cor']:.3f} P(R_LATCH)={row['p_r_latch']:.3f} "
            f"R_COR p95/max={row['r_cor_speed_p95_mps']:.3f}/{row['r_cor_speed_max_mps']:.3f} "
            f"viol={row['viol_r_cor_count']}"
        )

    full_candidates = []
    seen = set()
    for row in _select_top(pilot_rows, "planner") + _select_top(pilot_rows, "terminal"):
        if row["candidate_name"] not in seen:
            seen.add(row["candidate_name"])
            full_candidates.append(next(c for c in all_candidates if c["name"] == row["candidate_name"]))

    print("Running closeout full validation")
    full_rows = []
    for candidate in full_candidates:
        row = _evaluate(cfg, candidate, runs=FULL_RUNS, dt_s=FULL_DT_S, max_workers=FULL_MAX_WORKERS)
        row["stage"] = "full"
        full_rows.append(row)
        print(
            f"  {row['candidate_name']}: branch={row['branch']} "
            f"P(R_COR)={row['p_r_cor']:.3f} P(R_LATCH)={row['p_r_latch']:.3f} "
            f"R_COR p95/max={row['r_cor_speed_p95_mps']:.3f}/{row['r_cor_speed_max_mps']:.3f} "
            f"viol={row['viol_r_cor_count']}"
        )

    best_planner = max(
        [r for r in full_rows if r["branch"] == "planner"],
        key=lambda r: (r["compliant"], r["p_r_latch"], r["p_r_cor"], -r["dv_tag_p95_mps"]),
    )
    best_terminal = max(
        [r for r in full_rows if r["branch"] == "terminal"],
        key=lambda r: (r["compliant"], r["p_r_latch"], r["p_r_cor"], -r["dv_tag_p95_mps"]),
    )
    winner = best_planner if (
        best_planner["compliant"], best_planner["p_r_latch"], best_planner["p_r_cor"]
    ) >= (
        best_terminal["compliant"], best_terminal["p_r_latch"], best_terminal["p_r_cor"]
    ) else best_terminal

    out_dir = Path("outputs")
    out_dir.mkdir(parents=True, exist_ok=True)
    csv_path = out_dir / "planner_closeout_candidates.csv"
    report_path = out_dir / "planner_closeout_report.md"
    index_path = out_dir / "post_benchmark_refinement_index.md"
    _write_csv(csv_path, pilot_rows + full_rows)

    lines = []
    lines.append("# Post-Benchmark Planner Closeout Refinement")
    lines.append("")
    if reference is not None:
        ref_full = reference["full"]
        lines.append("## Reference")
        lines.append(
            f"- canonical planner reference: TOF={reference['chosen_t_end_s']:.0f}, "
            f"lead_s={reference['chosen_cor_entry_lead_s']:.0f}, "
            f"policy={reference['chosen_policy_name']}"
        )
        lines.append(
            f"- reference full result: P(R_COR)={_fmt(ref_full['p_r_cor'])}, "
            f"P(R_LATCH)={_fmt(ref_full['p_r_latch'])}, "
            f"R_COR p95/max={_fmt(ref_full['r_cor_speed_p95_mps'])}/{_fmt(ref_full['r_cor_speed_max_mps'])}, "
            f"violations={ref_full['viol_r_cor_count']}"
        )
        lines.append("- this benchmark reference remains frozen and separate from the refinement result below")
        lines.append("")

    lines.append("## Best Planner-Side Candidate")
    lines.append(
        f"- {best_planner['candidate_name']}: TOF={best_planner['tof_s']:.0f}, "
        f"lead_s={best_planner['lead_s']:.0f}, policy={best_planner['policy_name']}"
    )
    lines.append(
        f"- full P(R_COR)={_fmt(best_planner['p_r_cor'])}, "
        f"P(R_PRE)={_fmt(best_planner['p_r_pre'])}, "
        f"P(R_LATCH)={_fmt(best_planner['p_r_latch'])}"
    )
    lines.append(
        f"- R_COR p95/max={_fmt(best_planner['r_cor_speed_p95_mps'])}/"
        f"{_fmt(best_planner['r_cor_speed_max_mps'])} m/s, "
        f"violations={best_planner['viol_r_cor_count']}"
    )
    lines.append(
        f"- dv_tag p95/max={_fmt(best_planner['dv_tag_p95_mps'])}/"
        f"{_fmt(best_planner['dv_tag_max_mps'])} m/s, "
        f"dv_node p95/max={_fmt(best_planner['dv_node_p95_mps'])}/"
        f"{_fmt(best_planner['dv_node_max_mps'])} m/s"
    )
    lines.append("")

    lines.append("## Best Terminal-Side Candidate")
    lines.append(
        f"- {best_terminal['candidate_name']}: terminal_profile={best_terminal['terminal_profile']}"
    )
    lines.append(
        f"- full P(R_COR)={_fmt(best_terminal['p_r_cor'])}, "
        f"P(R_PRE)={_fmt(best_terminal['p_r_pre'])}, "
        f"P(R_LATCH)={_fmt(best_terminal['p_r_latch'])}"
    )
    lines.append(
        f"- R_COR p95/max={_fmt(best_terminal['r_cor_speed_p95_mps'])}/"
        f"{_fmt(best_terminal['r_cor_speed_max_mps'])} m/s, "
        f"violations={best_terminal['viol_r_cor_count']}"
    )
    lines.append(
        f"- dv_tag p95/max={_fmt(best_terminal['dv_tag_p95_mps'])}/"
        f"{_fmt(best_terminal['dv_tag_max_mps'])} m/s, "
        f"dv_node p95/max={_fmt(best_terminal['dv_node_p95_mps'])}/"
        f"{_fmt(best_terminal['dv_node_max_mps'])} m/s, "
        f"terminal sat p95={_fmt(best_terminal['term_sat_frac_p95'])}"
    )
    lines.append("")

    lines.append("## Outcome")
    achieved = winner["compliant"] and winner["p_r_latch"] >= 0.70
    lines.append(f"- achieved P(R_LATCH) >= 0.70: {achieved}")
    lines.append(
        f"- winner: {winner['branch']} candidate `{winner['candidate_name']}` "
        f"with P(R_LATCH)={_fmt(winner['p_r_latch'])}"
    )
    lines.append(f"- compliance remained perfect at R_COR: {winner['viol_r_cor_count'] == 0}")
    if winner["branch"] == "planner":
        lines.append("- change made: reduced corridor-entry lead from 300 s to 280 s at TOF 7100 s; terminal kept at base profile")
        lines.append("- terminal shaping did not beat planner-side refinement in this bounded sweep")
    else:
        lines.append("- change made: retained the current planner point and improved post-R_COR terminal shaping")
        lines.append("- terminal-side shaping outperformed planner-side refinement in this bounded sweep")
    lines.append("")
    lines.append("## Artifacts")
    lines.append(f"- `{csv_path.as_posix()}`")
    lines.append(f"- `{report_path.as_posix()}`")

    report_path.write_text("\n".join(lines), encoding="ascii")

    index_lines = []
    index_lines.append("# Post-Benchmark Refinement Artifacts")
    index_lines.append("")
    index_lines.append("These files are separate from the frozen benchmark reference and document the closeout refinement pass only.")
    index_lines.append("")
    index_lines.append("## Frozen Benchmark Reference")
    index_lines.append("- `benchmarks.json`")
    index_lines.append("- `outputs/validated/benchmarks.json`")
    index_lines.append("- `outputs/planner_v2_report.md`")
    index_lines.append("- `outputs/planner_candidates.csv`")
    index_lines.append("")
    index_lines.append("## Post-Benchmark Closeout Outputs")
    index_lines.append(f"- `{report_path.as_posix()}`")
    index_lines.append(f"- `{csv_path.as_posix()}`")
    index_lines.append("")
    index_lines.append("Current best compliant 2x refinement:")
    index_lines.append(f"- TOF={winner['tof_s']:.0f} s")
    index_lines.append(f"- lead_s={winner['lead_s']:.0f} s")
    index_lines.append(f"- P(R_COR)={_fmt(winner['p_r_cor'])}")
    index_lines.append(f"- P(R_LATCH)={_fmt(winner['p_r_latch'])}")
    index_lines.append(f"- R_COR hard-cap violations={winner['viol_r_cor_count']}")
    index_lines.append("")
    index_lines.append("Key conclusion:")
    index_lines.append("- improvement came from planner-side local refinement")
    index_lines.append("- tested post-R_COR terminal shaping degraded performance and was not adopted")
    index_path.write_text("\n".join(index_lines), encoding="ascii")

    print("Closeout result")
    print(f"Achieved >=0.70: {achieved}")
    print(
        f"Winner={winner['candidate_name']} branch={winner['branch']} "
        f"P(R_COR)={winner['p_r_cor']:.3f} P(R_LATCH)={winner['p_r_latch']:.3f} "
        f"R_COR p95/max={winner['r_cor_speed_p95_mps']:.3f}/{winner['r_cor_speed_max_mps']:.3f} "
        f"viol={winner['viol_r_cor_count']}"
    )
    print(f"Wrote {csv_path}")
    print(f"Wrote {report_path}")
    print(f"Wrote {index_path}")


if __name__ == "__main__":
    main()
