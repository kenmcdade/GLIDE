"""Autonomous planner for TOF/release policy under hard-cap constraints."""

from __future__ import annotations

import copy
import csv
from pathlib import Path
from typing import Any

import numpy as np

from mc.sampler import sample_config
from sim.runner import apply_release_targeting, run_sim_config
from sim.scenario import load_config, build_reference_state, get_initial_relative_state

PILOT_RUNS = 150
PILOT_DT_S = 5.0
FULL_RUNS = 500
FULL_DT_S = 5.0
R_COR_MAX_BUFFER_MPS = 0.33
R_COR_P95_BUFFER_MPS = 0.30


def _stat(values, fn):
    arr = np.asarray(values, dtype=float)
    if arr.size == 0:
        return float("nan")
    return float(fn(arr))


def _stat_p95(values):
    return _stat(values, lambda x: np.percentile(x, 95))


def _is_buffer_violation(metrics):
    p95 = metrics["r_cor_speed_p95_mps"]
    vmax = metrics["r_cor_speed_max_mps"]
    if np.isnan(p95) or np.isnan(vmax):
        return True
    return (
        metrics["viol_r_cor_count"] > 0
        or vmax > R_COR_MAX_BUFFER_MPS
        or p95 > R_COR_P95_BUFFER_MPS
    )


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


def _run_mc_summary(
    cfg: dict[str, Any],
    runs: int,
    dt_s: float,
    dispersion_scale: float,
    seed: int,
    t_end_s: float,
    cor_entry_lead_s: float,
    speed_safety_enabled: bool,
):
    rng = np.random.default_rng(seed)
    r_base = np.asarray(cfg.get("dispersion", {}).get("r_m", [0.0, 0.0, 0.0]), dtype=float)
    v_base = np.asarray(cfg.get("dispersion", {}).get("v_mps", [0.0, 0.0, 0.0]), dtype=float)

    counts = {k: 0 for k in ["R_ACQ", "R_COR", "R_PRE", "R_LATCH"]}
    vio_counts = {k: 0 for k in ["R_COR", "R_PRE", "R_LATCH"]}
    cor_speeds = []
    dv_tag = []
    dv_node = []

    for _ in range(runs):
        cfg_s = sample_config(cfg, rng)
        cfg_s["simulation"]["save_plots"] = False
        cfg_s["simulation"]["dt_s"] = float(dt_s)
        cfg_s["simulation"]["t_end_s"] = float(t_end_s)
        cfg_s["simulation"]["stop_on_gate"] = None
        cfg_s["terminal_capture"]["enabled"] = True
        cfg_s["terminal_capture"]["extend_mission"] = False

        cfg_s.setdefault("targeting", {})
        cfg_s["targeting"]["cor_entry_lead_s"] = float(cor_entry_lead_s)
        cfg_s.setdefault("guidance", {})
        r_cor_m = float(cfg_s.get("gates", {}).get("thresholds", {}).get("R_COR", 10.0))
        cfg_s["guidance"]["speed_safety_mode"] = {
            "enabled": bool(speed_safety_enabled),
            "r_cor_m": r_cor_m,
            "window_k_r_cor": 5.0,
            "speed_trigger_mps": 0.12,
            "gain_mult": 2.5,
            "gain_mult_max": 3.0,
        }

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

        for gate_name in counts:
            if gate_name in gates.crossings:
                counts[gate_name] += 1

        if "R_COR" in gates.crossings:
            cor_speeds.append(float(gates.crossings["R_COR"]["speed_mps"]))

        for vio in gates.violations:
            if vio.get("type") == "speed_violation":
                gate_name = vio.get("gate")
                if gate_name in vio_counts:
                    vio_counts[gate_name] += 1

        dv_tag.append(float(metrics.get("dv_tag", 0.0)))
        dv_node.append(float(metrics.get("dv_node_terminal", 0.0)))

    runs_f = max(runs, 1)
    return {
        "runs": int(runs),
        "p_r_acq": counts["R_ACQ"] / runs_f,
        "p_r_cor": counts["R_COR"] / runs_f,
        "p_r_pre": counts["R_PRE"] / runs_f,
        "p_r_latch": counts["R_LATCH"] / runs_f,
        "r_cor_speed_p95_mps": _stat_p95(cor_speeds),
        "r_cor_speed_max_mps": _stat(cor_speeds, np.max),
        "viol_r_cor_count": int(vio_counts["R_COR"]),
        "viol_r_pre_count": int(vio_counts["R_PRE"]),
        "viol_r_latch_count": int(vio_counts["R_LATCH"]),
        "dv_tag_p95_mps": _stat_p95(dv_tag),
        "dv_tag_max_mps": _stat(dv_tag, np.max),
        "dv_node_p95_mps": _stat_p95(dv_node),
        "dv_node_max_mps": _stat(dv_node, np.max),
    }


def _select_candidate(candidates):
    feasible = [c for c in candidates if not c["pilot_rejected"]]
    if feasible:
        return max(feasible, key=lambda c: (c["pilot_p_r_latch"], -c["pilot_dv_tag_p95_mps"]))
    # Fallback: choose least risky pilot if all are rejected.
    return min(
        candidates,
        key=lambda c: (
            c["pilot_viol_r_cor_count"],
            max(0.0, c["pilot_r_cor_speed_max_mps"] - R_COR_MAX_BUFFER_MPS) if not np.isnan(c["pilot_r_cor_speed_max_mps"]) else 999.0,
            max(0.0, c["pilot_r_cor_speed_p95_mps"] - R_COR_P95_BUFFER_MPS) if not np.isnan(c["pilot_r_cor_speed_p95_mps"]) else 999.0,
            -c["pilot_p_r_latch"],
            c["pilot_dv_tag_p95_mps"] if not np.isnan(c["pilot_dv_tag_p95_mps"]) else 999.0,
        ),
    )


def _write_candidates_csv(path: Path, rows):
    if not rows:
        return
    with path.open("w", newline="", encoding="ascii") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def _write_report(path: Path, plan):
    cand = plan["chosen_candidate"]
    pilot = plan["predicted_metrics"]
    full = plan["full_metrics"]
    lines = []
    lines.append("# Planner 2x Report")
    lines.append("")
    lines.append("## Selected Plan")
    lines.append(f"- chosen TOF (simulation.t_end_s): {plan['chosen_t_end_s']:.0f} s")
    lines.append(f"- chosen targeting cor_entry_lead_s: {plan['chosen_cor_entry_lead_s']:.0f} s")
    lines.append(f"- speed-safety mode engaged: {plan['speed_safety_mode_enabled']}")
    lines.append("")
    lines.append("## Selection Rationale")
    lines.append("- Candidate search: TOF 6500-9500 step 300")
    lines.append(f"- Pilot MC: N={PILOT_RUNS}, dt={PILOT_DT_S:.0f}s, dispersion_scale=2x")
    lines.append(
        f"- Reject rule: pilot R_COR max > {R_COR_MAX_BUFFER_MPS:.2f} m/s "
        f"or pilot R_COR p95 > {R_COR_P95_BUFFER_MPS:.2f} m/s or any R_COR violation"
    )
    lines.append("- Objective: maximize pilot P(R_LATCH), tie-break with lower pilot dv_tag p95")
    lines.append("")
    lines.append("## Chosen Candidate Pilot Metrics")
    lines.append(f"- pilot P(R_COR): {pilot['p_r_cor']:.3f}")
    lines.append(f"- pilot P(R_LATCH): {pilot['p_r_latch']:.3f}")
    lines.append(f"- pilot R_COR speed p95/max: {pilot['r_cor_speed_p95_mps']:.3f}/{pilot['r_cor_speed_max_mps']:.3f} m/s")
    lines.append(f"- pilot R_COR violation count: {pilot['viol_r_cor_count']}")
    lines.append(f"- pilot dv_tag p95: {pilot['dv_tag_p95_mps']:.3f} m/s")
    lines.append(f"- pilot rejected by buffers: {cand['pilot_rejected']}")
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
    lines.append("- `outputs/planner_2x_report.md`")
    path.write_text("\n".join(lines), encoding="ascii")


def plan_release_and_tof(config_path, dispersion_scale=2.0, seed=123, budget_evals=0):
    """Plan TOF/release policy by pilot screening and full validation."""
    cfg = load_config(config_path)
    cfg = copy.deepcopy(cfg)

    tof_grid = _build_tof_grid(6500.0, 9500.0, 300.0)
    tof_grid = _apply_budget(tof_grid, int(budget_evals or 0))

    base_lead = float(cfg.get("targeting", {}).get("cor_entry_lead_s", 300.0))
    candidates = []

    for i, tof_s in enumerate(tof_grid):
        # Keep baseline lead policy; planner can still explicitly choose it.
        lead_s = base_lead

        pilot_off = _run_mc_summary(
            cfg,
            runs=PILOT_RUNS,
            dt_s=PILOT_DT_S,
            dispersion_scale=dispersion_scale,
            seed=seed + i * 17 + 1,
            t_end_s=tof_s,
            cor_entry_lead_s=lead_s,
            speed_safety_enabled=False,
        )

        # Safety rerun trigger: explicit p95 risk (>0.33) or any hard-cap risk.
        off_p95 = pilot_off["r_cor_speed_p95_mps"]
        off_vmax = pilot_off["r_cor_speed_max_mps"]
        risk = (
            (not np.isnan(off_p95) and off_p95 > 0.33)
            or pilot_off["viol_r_cor_count"] > 0
            or (not np.isnan(off_vmax) and off_vmax > 0.35)
        )
        if risk:
            pilot = _run_mc_summary(
                cfg,
                runs=PILOT_RUNS,
                dt_s=PILOT_DT_S,
                dispersion_scale=dispersion_scale,
                seed=seed + i * 17 + 2,
                t_end_s=tof_s,
                cor_entry_lead_s=lead_s,
                speed_safety_enabled=True,
            )
            safety_enabled = True
        else:
            pilot = pilot_off
            safety_enabled = False

        rejected = _is_buffer_violation(pilot)

        candidates.append(
            {
                "tof_s": float(tof_s),
                "cor_entry_lead_s": float(lead_s),
                "speed_safety_mode_enabled": bool(safety_enabled),
                "pilot_p_r_cor": pilot["p_r_cor"],
                "pilot_p_r_latch": pilot["p_r_latch"],
                "pilot_r_cor_speed_p95_mps": pilot["r_cor_speed_p95_mps"],
                "pilot_r_cor_speed_max_mps": pilot["r_cor_speed_max_mps"],
                "pilot_viol_r_cor_count": int(pilot["viol_r_cor_count"]),
                "pilot_dv_tag_p95_mps": pilot["dv_tag_p95_mps"],
                "pilot_rejected": bool(rejected),
            }
        )

    chosen = _select_candidate(candidates)
    full = _run_mc_summary(
        cfg,
        runs=FULL_RUNS,
        dt_s=FULL_DT_S,
        dispersion_scale=dispersion_scale,
        seed=seed + 999,
        t_end_s=chosen["tof_s"],
        cor_entry_lead_s=chosen["cor_entry_lead_s"],
        speed_safety_enabled=bool(chosen["speed_safety_mode_enabled"]),
    )

    plan = {
        "chosen_t_end_s": float(chosen["tof_s"]),
        "chosen_cor_entry_lead_s": float(chosen["cor_entry_lead_s"]),
        "speed_safety_mode_enabled": bool(chosen["speed_safety_mode_enabled"]),
        "predicted_metrics": {
            "p_r_cor": chosen["pilot_p_r_cor"],
            "p_r_latch": chosen["pilot_p_r_latch"],
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
    _write_report(out_dir / "planner_2x_report.md", plan)
    return plan
