"""Autonomous planner for TOF/release policy under hard-cap constraints."""

from __future__ import annotations

import copy
import csv
import os
from concurrent.futures import ProcessPoolExecutor, as_completed
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np

from mc.sampler import sample_config
from sim.runner import apply_release_targeting, run_sim_config
from sim.scenario import load_config, build_reference_state, get_initial_relative_state

# Pilot/full evaluation settings (module constants so tests can override quickly).
PILOT_RUNS_PER_SEED = 40
PILOT_DT_S = 5.0
PILOT_SEED_OFFSETS = [0, 333, 666]
FULL_RUNS = 500
FULL_DT_S = 5.0
_DEFAULT_WORKERS = max(1, min(8, os.cpu_count() or 1))
PILOT_MAX_WORKERS = max(1, int(os.environ.get("GLIDE_PILOT_MAX_WORKERS", _DEFAULT_WORKERS)))
FULL_MAX_WORKERS = max(1, int(os.environ.get("GLIDE_FULL_MAX_WORKERS", _DEFAULT_WORKERS)))
LOCAL_REFINEMENT_TOF_OFFSETS_S = [-100.0, 0.0, 100.0]
LOCAL_REFINEMENT_LEAD_OFFSETS_S = [-20.0, 0.0, 20.0]

# Conservative pilot rejection buffers (below hard cap 0.35).
R_COR_MAX_BUFFER_MPS = 0.33
R_COR_P95_BUFFER_MPS = 0.30

# Policy family: explicit discrete options (baseline OFF + two safety variants).
POLICY_FAMILIES = [
    {
        "name": "off",
        "enabled": False,
    },
    {
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
    {
        "name": "safe_k5",
        "enabled": True,
        "window_k_r_cor": 5.0,
        "speed_trigger_mps": 0.10,
        "gain_mult": 2.5,
        "gain_mult_max": 3.0,
        "predict_dt_scale": 1.0,
        "prebrake_margin_m": 8.0,
        "prebrake_gain": 1.8,
        "hard_cap_mps": 0.35,
    },
]


@dataclass
class TrialSummary:
    runs: int
    count_r_cor: int
    count_r_pre: int
    count_r_latch: int
    viol_r_cor_count: int
    r_cor_speeds: list[float]
    dv_tag: list[float]
    dv_node: list[float]
    term_sat_frac: list[float]


def _stat(values, fn):
    arr = np.asarray(values, dtype=float)
    if arr.size == 0:
        return float("nan")
    return float(fn(arr))


def _stat_p95(values):
    return _stat(values, lambda x: np.percentile(x, 95))


def _deep_update(dst: dict[str, Any], src: dict[str, Any]):
    for key, value in src.items():
        if isinstance(value, dict) and isinstance(dst.get(key), dict):
            _deep_update(dst[key], value)
        else:
            dst[key] = copy.deepcopy(value)
    return dst


def _build_tof_grid(t_min, t_max, step):
    tofs = []
    t = float(t_min)
    t_max = float(t_max)
    step = float(step)
    while t <= (t_max + 1e-9):
        tofs.append(float(round(t)))
        t += step
    if not tofs:
        tofs = [float(t_min)]
    return tofs


def _apply_budget(grid, budget_evals):
    if budget_evals is None or budget_evals <= 0 or budget_evals >= len(grid):
        return list(grid)
    idx = np.linspace(0, len(grid) - 1, budget_evals)
    idx = sorted(set(int(round(i)) for i in idx))
    return [grid[i] for i in idx]


def _wilson_lcb(successes: int, n: int, z: float = 1.96) -> float:
    if n <= 0:
        return 0.0
    p = successes / n
    z2 = z * z
    denom = 1.0 + z2 / n
    center = p + z2 / (2.0 * n)
    margin = z * np.sqrt((p * (1.0 - p) + z2 / (4.0 * n)) / n)
    return float((center - margin) / denom)


def _normalize_speed_safety(policy: dict[str, Any], dt_s: float, r_cor_m: float) -> dict[str, Any]:
    if not policy.get("enabled", False):
        return {"enabled": False}
    window_k = float(policy.get("window_k_r_cor", 3.0))
    predict_dt_scale = float(policy.get("predict_dt_scale", 1.0))
    return {
        "enabled": True,
        "r_cor_m": float(r_cor_m),
        "window_k_r_cor": window_k,
        "range_m": float(window_k * r_cor_m),
        "speed_trigger_mps": float(policy.get("speed_trigger_mps", 0.12)),
        "gain_mult": float(policy.get("gain_mult", 2.0)),
        "gain_mult_max": float(policy.get("gain_mult_max", 2.5)),
        "predict_dt_s": float(max(1e-6, dt_s * predict_dt_scale)),
        "prebrake_margin_m": float(policy.get("prebrake_margin_m", 6.0)),
        "prebrake_gain": float(policy.get("prebrake_gain", 1.2)),
        "hard_cap_mps": float(policy.get("hard_cap_mps", 0.35)),
    }


def _run_single_seed_summary(
    cfg: dict[str, Any],
    runs: int,
    dt_s: float,
    dispersion_scale: float,
    seed: int,
    t_end_s: float,
    cor_entry_lead_s: float,
    speed_safety_policy: dict[str, Any],
    terminal_overrides: dict[str, Any] | None = None,
) -> TrialSummary:
    rng = np.random.default_rng(seed)
    r_base = np.asarray(cfg.get("dispersion", {}).get("r_m", [0.0, 0.0, 0.0]), dtype=float)
    v_base = np.asarray(cfg.get("dispersion", {}).get("v_mps", [0.0, 0.0, 0.0]), dtype=float)
    r_cor_m = float(cfg.get("gates", {}).get("thresholds", {}).get("R_COR", 10.0))

    count_r_cor = 0
    count_r_pre = 0
    count_r_latch = 0
    viol_r_cor_count = 0
    r_cor_speeds = []
    dv_tag = []
    dv_node = []
    term_sat_frac = []

    for _ in range(runs):
        cfg_s = sample_config(cfg, rng)
        cfg_s["simulation"]["save_plots"] = False
        cfg_s["simulation"]["dt_s"] = float(dt_s)
        cfg_s["simulation"]["t_end_s"] = float(t_end_s)
        cfg_s["simulation"]["stop_on_gate"] = None
        cfg_s["terminal_capture"]["enabled"] = True
        cfg_s["terminal_capture"]["extend_mission"] = False
        if terminal_overrides:
            _deep_update(cfg_s["terminal_capture"], terminal_overrides)

        cfg_s.setdefault("targeting", {})
        cfg_s["targeting"]["cor_entry_lead_s"] = float(cor_entry_lead_s)
        cfg_s.setdefault("guidance", {})
        cfg_s["guidance"]["speed_safety_mode"] = _normalize_speed_safety(
            speed_safety_policy, dt_s, r_cor_m
        )

        cfg_s.setdefault("dispersion", {})
        cfg_s["dispersion"]["r_m"] = (r_base * dispersion_scale).tolist()
        cfg_s["dispersion"]["v_mps"] = (v_base * dispersion_scale).tolist()

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

        if "R_COR" in gates.crossings:
            count_r_cor += 1
            r_cor_speeds.append(float(gates.crossings["R_COR"]["speed_mps"]))
        if "R_PRE" in gates.crossings:
            count_r_pre += 1
        if "R_LATCH" in gates.crossings:
            count_r_latch += 1

        for vio in gates.violations:
            if vio.get("type") == "speed_violation" and vio.get("gate") == "R_COR":
                viol_r_cor_count += 1

        dv_tag.append(float(metrics.get("dv_tag", 0.0)))
        dv_node.append(float(metrics.get("dv_node_terminal", 0.0)))
        term_sat_frac.append(float(metrics.get("term_sat_frac", 0.0)))

    return TrialSummary(
        runs=int(runs),
        count_r_cor=int(count_r_cor),
        count_r_pre=int(count_r_pre),
        count_r_latch=int(count_r_latch),
        viol_r_cor_count=int(viol_r_cor_count),
        r_cor_speeds=r_cor_speeds,
        dv_tag=dv_tag,
        dv_node=dv_node,
        term_sat_frac=term_sat_frac,
    )


def _aggregate_seed_summaries(seed_summaries: list[TrialSummary]):
    total_runs = int(sum(s.runs for s in seed_summaries))
    total_r_cor = int(sum(s.count_r_cor for s in seed_summaries))
    total_r_pre = int(sum(s.count_r_pre for s in seed_summaries))
    total_r_latch = int(sum(s.count_r_latch for s in seed_summaries))
    total_viol_r_cor = int(sum(s.viol_r_cor_count for s in seed_summaries))
    cor_speeds = [v for s in seed_summaries for v in s.r_cor_speeds]
    dv_tag = [v for s in seed_summaries for v in s.dv_tag]
    dv_node = [v for s in seed_summaries for v in s.dv_node]
    term_sat_frac = [v for s in seed_summaries for v in s.term_sat_frac]

    return {
        "runs": total_runs,
        "p_r_cor": total_r_cor / max(total_runs, 1),
        "p_r_pre": total_r_pre / max(total_runs, 1),
        "p_r_latch": total_r_latch / max(total_runs, 1),
        "count_r_latch": total_r_latch,
        "r_cor_speed_p95_mps": _stat_p95(cor_speeds),
        "r_cor_speed_max_mps": _stat(cor_speeds, np.max),
        "viol_r_cor_count": total_viol_r_cor,
        "dv_tag_p95_mps": _stat_p95(dv_tag),
        "dv_tag_max_mps": _stat(dv_tag, np.max),
        "dv_node_p95_mps": _stat_p95(dv_node),
        "dv_node_max_mps": _stat(dv_node, np.max),
        "term_sat_frac_p95": _stat_p95(term_sat_frac),
    }


def _evaluate_candidate(
    cfg: dict[str, Any],
    dispersion_scale: float,
    base_seed: int,
    tof_s: float,
    cor_entry_lead_s: float,
    policy: dict[str, Any],
):
    seed_summaries = []
    for idx, off in enumerate(PILOT_SEED_OFFSETS):
        s = _run_single_seed_summary(
            cfg=cfg,
            runs=PILOT_RUNS_PER_SEED,
            dt_s=PILOT_DT_S,
            dispersion_scale=dispersion_scale,
            seed=int(base_seed + off + int(tof_s) + idx * 7),
            t_end_s=tof_s,
            cor_entry_lead_s=cor_entry_lead_s,
            speed_safety_policy=policy,
        )
        seed_summaries.append(s)

    agg = _aggregate_seed_summaries(seed_summaries)
    rejected = (
        agg["viol_r_cor_count"] > 0
        or np.isnan(agg["r_cor_speed_max_mps"])
        or np.isnan(agg["r_cor_speed_p95_mps"])
        or agg["r_cor_speed_max_mps"] > R_COR_MAX_BUFFER_MPS
        or agg["r_cor_speed_p95_mps"] > R_COR_P95_BUFFER_MPS
    )
    compliant_lcb95 = _wilson_lcb(agg["count_r_latch"], agg["runs"], z=1.96)

    return {
        "tof_s": float(tof_s),
        "policy_name": str(policy["name"]),
        "cor_entry_lead_s": float(cor_entry_lead_s),
        "speed_safety_mode_enabled": bool(policy.get("enabled", False)),
        "pilot_p_r_cor": agg["p_r_cor"],
        "pilot_p_r_latch": agg["p_r_latch"],
        "pilot_compliant_latch_lcb95": compliant_lcb95,
        "pilot_r_cor_speed_p95_mps": agg["r_cor_speed_p95_mps"],
        "pilot_r_cor_speed_max_mps": agg["r_cor_speed_max_mps"],
        "pilot_viol_r_cor_count": int(agg["viol_r_cor_count"]),
        "pilot_dv_tag_p95_mps": agg["dv_tag_p95_mps"],
        "pilot_rejected": bool(rejected),
    }


def _evaluate_candidate_job(job: dict[str, Any]):
    return _evaluate_candidate(**job)


def evaluate_candidate_summary(
    cfg: dict[str, Any],
    dispersion_scale: float,
    seed: int,
    tof_s: float,
    cor_entry_lead_s: float,
    speed_safety_policy: dict[str, Any],
    runs: int,
    dt_s: float,
    terminal_overrides: dict[str, Any] | None = None,
    max_workers: int | None = None,
):
    summaries = []
    runs = int(runs)
    dt_s = float(dt_s)
    worker_count = FULL_MAX_WORKERS if max_workers is None else int(max_workers)

    if worker_count > 1 and runs >= (4 * worker_count):
        chunks = []
        base = runs // worker_count
        rem = runs % worker_count
        for i in range(worker_count):
            chunk_runs = base + (1 if i < rem else 0)
            if chunk_runs > 0:
                chunks.append((i, chunk_runs))
        with ProcessPoolExecutor(max_workers=min(worker_count, len(chunks))) as ex:
            futs = []
            for i, chunk_runs in chunks:
                futs.append(
                    ex.submit(
                        _run_single_seed_summary,
                        cfg=cfg,
                        runs=chunk_runs,
                        dt_s=dt_s,
                        dispersion_scale=dispersion_scale,
                        seed=seed + 999 + i * 100003,
                        t_end_s=tof_s,
                        cor_entry_lead_s=cor_entry_lead_s,
                        speed_safety_policy=speed_safety_policy,
                        terminal_overrides=terminal_overrides,
                    )
                )
            for fut in as_completed(futs):
                summaries.append(fut.result())
    else:
        summaries.append(
            _run_single_seed_summary(
                cfg=cfg,
                runs=runs,
                dt_s=dt_s,
                dispersion_scale=dispersion_scale,
                seed=seed + 999,
                t_end_s=tof_s,
                cor_entry_lead_s=cor_entry_lead_s,
                speed_safety_policy=speed_safety_policy,
                terminal_overrides=terminal_overrides,
            )
        )
    return _aggregate_seed_summaries(summaries)


def _select_candidate(candidates: list[dict[str, Any]]):
    feasible = [c for c in candidates if not c["pilot_rejected"]]
    if feasible:
        return max(
            feasible,
            key=lambda c: (
                c["pilot_compliant_latch_lcb95"],      # conservative compliant success
                c["pilot_p_r_latch"],                   # secondary success
                -c["pilot_dv_tag_p95_mps"],            # tertiary effort
            ),
        )

    # If all rejected, choose least-bad candidate deterministically.
    return min(
        candidates,
        key=lambda c: (
            c["pilot_viol_r_cor_count"],
            max(0.0, c["pilot_r_cor_speed_max_mps"] - R_COR_MAX_BUFFER_MPS)
            if not np.isnan(c["pilot_r_cor_speed_max_mps"])
            else 999.0,
            max(0.0, c["pilot_r_cor_speed_p95_mps"] - R_COR_P95_BUFFER_MPS)
            if not np.isnan(c["pilot_r_cor_speed_p95_mps"])
            else 999.0,
            -c["pilot_compliant_latch_lcb95"],
            c["pilot_dv_tag_p95_mps"] if not np.isnan(c["pilot_dv_tag_p95_mps"]) else 999.0,
        ),
    )


def _refine_around_candidate(
    cfg: dict[str, Any],
    dispersion_scale: float,
    seed: int,
    chosen: dict[str, Any],
):
    policy = next((p for p in POLICY_FAMILIES if p["name"] == chosen["policy_name"]), None)
    if policy is None:
        return chosen

    tof_values = sorted(
        {
            float(max(1.0, chosen["tof_s"] + dt))
            for dt in LOCAL_REFINEMENT_TOF_OFFSETS_S
        }
    )
    lead_values = sorted(
        {
            float(max(0.0, chosen["cor_entry_lead_s"] + dl))
            for dl in LOCAL_REFINEMENT_LEAD_OFFSETS_S
        }
    )

    local_candidates = []
    for tof_s in tof_values:
        for lead_s in lead_values:
            local_candidates.append(
                _evaluate_candidate(
                    cfg=cfg,
                    dispersion_scale=dispersion_scale,
                    base_seed=seed,
                    tof_s=tof_s,
                    cor_entry_lead_s=lead_s,
                    policy=policy,
                )
            )
    local_candidates.append(chosen)
    return _select_candidate(local_candidates)


def _write_candidates_csv(path: Path, rows: list[dict[str, Any]]):
    if not rows:
        return
    with path.open("w", newline="", encoding="ascii") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def _write_report(path: Path, plan: dict[str, Any]):
    cand = plan["chosen_candidate"]
    pilot = plan["predicted_metrics"]
    full = plan["full_metrics"]
    lines = []
    lines.append("# Planner v2 Report")
    lines.append("")
    lines.append("## Selected Plan")
    lines.append(f"- chosen TOF (simulation.t_end_s): {plan['chosen_t_end_s']:.0f} s")
    lines.append(f"- chosen targeting cor_entry_lead_s: {plan['chosen_cor_entry_lead_s']:.0f} s")
    lines.append(f"- chosen policy: {cand['policy_name']}")
    lines.append(f"- speed-safety mode engaged: {plan['speed_safety_mode_enabled']}")
    lines.append("")
    lines.append("## Selection Logic")
    lines.append("- Candidate search: TOF 6500-9500 step 300")
    lines.append(f"- Policies per TOF: {[p['name'] for p in POLICY_FAMILIES]}")
    lines.append(
        f"- Pilot MC: runs/seed={PILOT_RUNS_PER_SEED}, seeds={len(PILOT_SEED_OFFSETS)}, dt={PILOT_DT_S:.0f}s, dispersion=2x"
    )
    lines.append(
        f"- Local refinement after coarse pick: TOF offsets {LOCAL_REFINEMENT_TOF_OFFSETS_S}, "
        f"lead offsets {LOCAL_REFINEMENT_LEAD_OFFSETS_S}"
    )
    lines.append(
        f"- Reject candidate if pilot R_COR max>{R_COR_MAX_BUFFER_MPS:.2f} or "
        f"pilot R_COR p95>{R_COR_P95_BUFFER_MPS:.2f} or any R_COR violation"
    )
    lines.append("- Select by: zero-violation first, then highest compliant LCB95, then lowest dv_tag p95")
    lines.append("")
    lines.append("## Chosen Candidate Pilot Metrics")
    lines.append(f"- pilot P(R_COR): {pilot['p_r_cor']:.3f}")
    lines.append(f"- pilot P(R_LATCH): {pilot['p_r_latch']:.3f}")
    lines.append(f"- pilot compliant LCB95: {pilot['compliant_lcb95']:.3f}")
    lines.append(f"- pilot R_COR speed p95/max: {pilot['r_cor_speed_p95_mps']:.3f}/{pilot['r_cor_speed_max_mps']:.3f} m/s")
    lines.append(f"- pilot R_COR violation count: {pilot['viol_r_cor_count']}")
    lines.append(f"- pilot dv_tag p95: {pilot['dv_tag_p95_mps']:.3f} m/s")
    lines.append("")
    lines.append("## Full Validation Metrics (2x)")
    lines.append(f"- Full MC: N={FULL_RUNS}, dt={FULL_DT_S:.0f}s")
    lines.append(f"- full P(R_COR): {full['p_r_cor']:.3f}")
    lines.append(f"- full P(R_LATCH): {full['p_r_latch']:.3f}")
    lines.append(f"- full R_COR speed p95/max: {full['r_cor_speed_p95_mps']:.3f}/{full['r_cor_speed_max_mps']:.3f} m/s")
    lines.append(f"- full R_COR violation count: {full['viol_r_cor_count']}")
    lines.append(f"- full dv_tag p95/max: {full['dv_tag_p95_mps']:.3f}/{full['dv_tag_max_mps']:.3f} m/s")
    lines.append("")
    lines.append("## Candidate Count")
    lines.append(f"- evaluated candidates: {plan['candidate_count']}")
    lines.append("")
    lines.append("## Artifacts")
    lines.append("- `outputs/planner_candidates.csv`")
    lines.append("- `outputs/planner_v2_report.md`")
    path.write_text("\n".join(lines), encoding="ascii")


def _run_full_validation(
    cfg: dict[str, Any],
    dispersion_scale: float,
    seed: int,
    tof_s: float,
    cor_entry_lead_s: float,
    policy_name: str,
):
    policy = next((p for p in POLICY_FAMILIES if p["name"] == policy_name), None)
    if policy is None:
        raise ValueError(f"Unknown policy: {policy_name}")
    agg = evaluate_candidate_summary(
        cfg=cfg,
        dispersion_scale=dispersion_scale,
        seed=seed,
        tof_s=tof_s,
        cor_entry_lead_s=cor_entry_lead_s,
        speed_safety_policy=policy,
        runs=FULL_RUNS,
        dt_s=FULL_DT_S,
        terminal_overrides=None,
        max_workers=FULL_MAX_WORKERS,
    )
    return {
        "p_r_cor": agg["p_r_cor"],
        "p_r_pre": agg["p_r_pre"],
        "p_r_latch": agg["p_r_latch"],
        "r_cor_speed_p95_mps": agg["r_cor_speed_p95_mps"],
        "r_cor_speed_max_mps": agg["r_cor_speed_max_mps"],
        "viol_r_cor_count": int(agg["viol_r_cor_count"]),
        "dv_tag_p95_mps": agg["dv_tag_p95_mps"],
        "dv_tag_max_mps": agg["dv_tag_max_mps"],
        "dv_node_p95_mps": agg["dv_node_p95_mps"],
        "dv_node_max_mps": agg["dv_node_max_mps"],
        "term_sat_frac_p95": agg["term_sat_frac_p95"],
    }


def plan_release_and_tof(config_path, dispersion_scale=2.0, seed=123, budget_evals=0):
    """Plan TOF/release policy by conservative pilot screening + full validation."""
    cfg = load_config(config_path)
    cfg = copy.deepcopy(cfg)

    tof_grid = _build_tof_grid(6500.0, 9500.0, 300.0)
    tof_grid = _apply_budget(tof_grid, int(budget_evals or 0))
    base_lead = float(cfg.get("targeting", {}).get("cor_entry_lead_s", 300.0))

    jobs = []
    for tof_s in tof_grid:
        for policy in POLICY_FAMILIES:
            jobs.append(
                {
                    "cfg": cfg,
                    "dispersion_scale": dispersion_scale,
                    "base_seed": seed,
                    "tof_s": tof_s,
                    "cor_entry_lead_s": base_lead,
                    "policy": policy,
                }
            )

    candidates = []
    if PILOT_MAX_WORKERS > 1 and len(jobs) > 1:
        with ProcessPoolExecutor(max_workers=min(PILOT_MAX_WORKERS, len(jobs))) as ex:
            futs = [ex.submit(_evaluate_candidate_job, job) for job in jobs]
            for fut in as_completed(futs):
                candidates.append(fut.result())
    else:
        for job in jobs:
            candidates.append(_evaluate_candidate_job(job))

    candidates.sort(key=lambda c: (c["tof_s"], c["policy_name"]))

    chosen = _select_candidate(candidates)
    chosen = _refine_around_candidate(
        cfg=cfg,
        dispersion_scale=dispersion_scale,
        seed=seed,
        chosen=chosen,
    )
    full = _run_full_validation(
        cfg=cfg,
        dispersion_scale=dispersion_scale,
        seed=seed,
        tof_s=chosen["tof_s"],
        cor_entry_lead_s=chosen["cor_entry_lead_s"],
        policy_name=chosen["policy_name"],
    )

    plan = {
        "chosen_t_end_s": float(chosen["tof_s"]),
        "chosen_cor_entry_lead_s": float(chosen["cor_entry_lead_s"]),
        "speed_safety_mode_enabled": bool(chosen["speed_safety_mode_enabled"]),
        "chosen_policy_name": str(chosen["policy_name"]),
        "predicted_metrics": {
            "p_r_cor": chosen["pilot_p_r_cor"],
            "p_r_latch": chosen["pilot_p_r_latch"],
            "compliant_lcb95": chosen["pilot_compliant_latch_lcb95"],
            "r_cor_speed_p95_mps": chosen["pilot_r_cor_speed_p95_mps"],
            "r_cor_speed_max_mps": chosen["pilot_r_cor_speed_max_mps"],
            "viol_r_cor_count": int(chosen["pilot_viol_r_cor_count"]),
            "dv_tag_p95_mps": chosen["pilot_dv_tag_p95_mps"],
        },
        "full_metrics": full,
        "candidate_count": len(candidates),
        "chosen_candidate": chosen,
        "candidates": candidates,
    }

    out_dir = Path("outputs")
    out_dir.mkdir(parents=True, exist_ok=True)
    _write_candidates_csv(out_dir / "planner_candidates.csv", candidates)
    _write_report(out_dir / "planner_v2_report.md", plan)
    # Keep legacy name updated for continuity.
    _write_report(out_dir / "planner_2x_report.md", plan)
    return plan
