"""Canonical benchmark harness for GLIDE V4 regression tracking."""

from __future__ import annotations

import hashlib
import json
from concurrent.futures import ProcessPoolExecutor, as_completed
from datetime import datetime, timezone
from pathlib import Path

import numpy as np

from planner import plan_release_and_tof
from run_validation_suite import run_campaign
from sim.scenario import load_config


CFG_PATH = "configs/dispersion_recovery.yaml"
BASELINE_SEEDS = [123, 456, 789]
BASELINE_RUNS = 1000
STRESS_SEED = 123
STRESS_RUNS = 500
PLANNER_SEED = 123
PLANNER_DISP_SCALE = 2.0


def _stat(values, fn):
    arr = np.asarray(values, dtype=float)
    if arr.size == 0:
        return float("nan")
    return float(fn(arr))


def _safe_float(x):
    if x is None:
        return None
    try:
        xv = float(x)
    except (TypeError, ValueError):
        return None
    if np.isnan(xv):
        return None
    return xv


def _cfg_fingerprint(path: Path) -> str:
    data = path.read_bytes()
    return hashlib.sha256(data).hexdigest()


def _baseline_aggregate(rows: list[dict]) -> dict:
    p_cor = [r["p_r_cor"] for r in rows]
    p_latch = [r["p_r_latch"] for r in rows]
    dv_tag_p95 = [r["dv_tag_p95_mps"] for r in rows]
    r_cor_max = [r["r_cor_speed_max_mps"] for r in rows]
    r_cor_p95 = [r["r_cor_speed_p95_mps"] for r in rows]
    return {
        "p_r_cor_mean": _safe_float(_stat(p_cor, np.mean)),
        "p_r_cor_min": _safe_float(_stat(p_cor, np.min)),
        "p_r_latch_mean": _safe_float(_stat(p_latch, np.mean)),
        "p_r_latch_min": _safe_float(_stat(p_latch, np.min)),
        "r_cor_speed_p95_max_of_seeds_mps": _safe_float(_stat(r_cor_p95, np.max)),
        "r_cor_speed_max_of_seeds_mps": _safe_float(_stat(r_cor_max, np.max)),
        "dv_tag_p95_max_of_seeds_mps": _safe_float(_stat(dv_tag_p95, np.max)),
        "viol_r_cor_total": int(sum(int(r["viol_r_cor_count"]) for r in rows)),
        "viol_r_pre_total": int(sum(int(r["viol_r_pre_count"]) for r in rows)),
        "viol_r_latch_total": int(sum(int(r["viol_r_latch_count"]) for r in rows)),
    }


def _planner_snapshot(plan: dict) -> dict:
    full = plan["full_metrics"]
    predicted = plan["predicted_metrics"]
    chosen = plan["chosen_candidate"]
    return {
        "chosen_t_end_s": float(plan["chosen_t_end_s"]),
        "chosen_cor_entry_lead_s": float(plan["chosen_cor_entry_lead_s"]),
        "chosen_policy_name": str(plan["chosen_policy_name"]),
        "speed_safety_mode_enabled": bool(plan["speed_safety_mode_enabled"]),
        "candidate_count": int(plan["candidate_count"]),
        "predicted": {
            "p_r_cor": _safe_float(predicted["p_r_cor"]),
            "p_r_latch": _safe_float(predicted["p_r_latch"]),
            "compliant_lcb95": _safe_float(predicted["compliant_lcb95"]),
            "r_cor_speed_p95_mps": _safe_float(predicted["r_cor_speed_p95_mps"]),
            "r_cor_speed_max_mps": _safe_float(predicted["r_cor_speed_max_mps"]),
            "viol_r_cor_count": int(predicted["viol_r_cor_count"]),
            "dv_tag_p95_mps": _safe_float(predicted["dv_tag_p95_mps"]),
        },
        "full": {
            "p_r_cor": _safe_float(full["p_r_cor"]),
            "p_r_latch": _safe_float(full["p_r_latch"]),
            "r_cor_speed_p95_mps": _safe_float(full["r_cor_speed_p95_mps"]),
            "r_cor_speed_max_mps": _safe_float(full["r_cor_speed_max_mps"]),
            "viol_r_cor_count": int(full["viol_r_cor_count"]),
            "dv_tag_p95_mps": _safe_float(full["dv_tag_p95_mps"]),
            "dv_tag_max_mps": _safe_float(full["dv_tag_max_mps"]),
        },
    }


def main():
    cfg = load_config(CFG_PATH)
    cfg_path = Path(CFG_PATH)
    cfg_fp = _cfg_fingerprint(cfg_path)

    print("Running canonical benchmark suite")
    print("1) baseline 1x: N=1000 for seeds 123/456/789")
    baseline_rows = []
    with ProcessPoolExecutor(max_workers=min(len(BASELINE_SEEDS), 3)) as ex:
        futs = {
            ex.submit(
                run_campaign,
                seed=seed,
                runs=BASELINE_RUNS,
                disp_scale=1.0,
                campaign="benchmark_baseline_1x",
            ): seed
            for seed in BASELINE_SEEDS
        }
        for fut in as_completed(futs):
            row = fut.result()
            baseline_rows.append(row)
            print(
                f"  seed={row['seed']} P(R_COR)={row['p_r_cor']:.3f} "
                f"P(R_LATCH)={row['p_r_latch']:.3f} "
                f"R_COR p95/max={row['r_cor_speed_p95_mps']:.3f}/{row['r_cor_speed_max_mps']:.3f} "
                f"viol_R_COR={row['viol_r_cor_count']}"
            )
    baseline_rows.sort(key=lambda r: r["seed"])

    print("2) stress 2x: N=500, seed=123")
    stress_row = run_campaign(seed=STRESS_SEED, runs=STRESS_RUNS, disp_scale=2.0, campaign="benchmark_stress_2x")
    print(
        f"  P(R_COR)={stress_row['p_r_cor']:.3f} P(R_LATCH)={stress_row['p_r_latch']:.3f} "
        f"R_COR p95/max={stress_row['r_cor_speed_p95_mps']:.3f}/{stress_row['r_cor_speed_max_mps']:.3f} "
        f"viol_R_COR={stress_row['viol_r_cor_count']}"
    )

    print("3) planner 2x full validation: N=500")
    plan = plan_release_and_tof(
        config_path=CFG_PATH,
        dispersion_scale=PLANNER_DISP_SCALE,
        seed=PLANNER_SEED,
        budget_evals=0,
    )
    plan_snap = _planner_snapshot(plan)
    print(
        f"  TOF={plan_snap['chosen_t_end_s']:.0f} policy={plan_snap['chosen_policy_name']} "
        f"P(R_LATCH)={plan_snap['full']['p_r_latch']:.3f} "
        f"R_COR p95/max={plan_snap['full']['r_cor_speed_p95_mps']:.3f}/{plan_snap['full']['r_cor_speed_max_mps']:.3f} "
        f"viol_R_COR={plan_snap['full']['viol_r_cor_count']}"
    )

    baseline_agg = _baseline_aggregate(baseline_rows)
    out = {
        "meta": {
            "generated_utc": datetime.now(timezone.utc).isoformat(),
            "script": "run_benchmarks.py",
            "config_path": CFG_PATH,
            "config_sha256": cfg_fp,
        },
        "config_used": {
            "frozen_baseline": {
                "simulation.t_end_s": float(cfg["simulation"]["t_end_s"]),
                "targeting.cor_entry_lead_s": float(cfg["targeting"]["cor_entry_lead_s"]),
                "guidance.cw_speed_weight": float(cfg["guidance"]["cw_speed_weight"]),
                "guidance.r_cor_speed_gain": float(cfg["guidance"]["r_cor_speed_gain"]),
            },
            "gates": cfg["gates"]["thresholds"],
            "speed_limits": {
                "R_COR_hard_max_mps": float(cfg["gates"]["speed_limits"]["R_COR"]["hard_max"]),
                "R_PRE_hard_max_mps": float(cfg["gates"]["speed_limits"]["R_PRE"]["hard_max"]),
                "R_LATCH_hard_max_mps": float(cfg["gates"]["speed_limits"]["R_LATCH"]["hard_max"]),
            },
        },
        "suite": {
            "baseline_1x": {
                "runs_per_seed": BASELINE_RUNS,
                "seeds": BASELINE_SEEDS,
                "results": baseline_rows,
                "aggregate": baseline_agg,
            },
            "stress_2x": {
                "runs": STRESS_RUNS,
                "seed": STRESS_SEED,
                "result": stress_row,
            },
            "planner_2x_full": {
                "seed": PLANNER_SEED,
                "dispersion_scale": PLANNER_DISP_SCALE,
                "result": plan_snap,
            },
        },
        "regression_snapshot": {
            "baseline_1x_min_p_r_latch": baseline_agg["p_r_latch_min"],
            "baseline_1x_total_viol_r_cor": baseline_agg["viol_r_cor_total"],
            "stress_2x_viol_r_cor": int(stress_row["viol_r_cor_count"]),
            "stress_2x_dv_tag_p95_mps": _safe_float(stress_row["dv_tag_p95_mps"]),
            "planner_2x_viol_r_cor": int(plan_snap["full"]["viol_r_cor_count"]),
            "planner_2x_p_r_latch": _safe_float(plan_snap["full"]["p_r_latch"]),
            "planner_2x_dv_tag_p95_mps": _safe_float(plan_snap["full"]["dv_tag_p95_mps"]),
        },
    }

    out_root = Path("benchmarks.json")
    out_validated = Path("outputs") / "validated" / "benchmarks.json"
    out_validated.parent.mkdir(parents=True, exist_ok=True)
    text = json.dumps(out, indent=2, sort_keys=True)
    out_root.write_text(text, encoding="ascii")
    out_validated.write_text(text, encoding="ascii")

    print(f"Wrote {out_root}")
    print(f"Wrote {out_validated}")


if __name__ == "__main__":
    main()
