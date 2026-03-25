"""Grid sweep for lead-time and speed/min-miss tuning."""

import csv
import itertools
import math
import os
from concurrent.futures import ProcessPoolExecutor, as_completed

import numpy as np

from mc.sampler import sample_config
from sim.scenario import load_config, build_reference_state, get_initial_relative_state
from sim.runner import apply_release_targeting, run_sim_config

CFG_PATH = "configs/dispersion_recovery.yaml"
RUNS = 200
DT = 5.0
SEED = 123
LEADS = [300.0, 600.0, 900.0]
CW_SPEED_WEIGHTS = [0.0, 0.1, 0.3, 0.5]
R_COR_SPEED_GAINS = [0.0, 0.5, 1.0, 2.0]
OUT_CSV = "outputs/pareto_lead_cw_gain_sweep.csv"


def evaluate_combo(args):
    idx, lead_s, cw_w, cor_gain = args
    cfg = load_config(CFG_PATH)
    rng = np.random.default_rng(SEED + 1000 * idx)

    count_cor = 0
    count_latch = 0
    cor_speeds = []

    for _ in range(RUNS):
        cfg_s = sample_config(cfg, rng)
        cfg_s["simulation"]["save_plots"] = False
        cfg_s["simulation"]["dt_s"] = float(DT)
        cfg_s["simulation"]["stop_on_gate"] = None
        cfg_s["terminal_capture"]["enabled"] = True
        cfg_s["terminal_capture"]["extend_mission"] = False

        cfg_s["targeting"]["cor_entry_lead_s"] = float(lead_s)
        cfg_s["guidance"]["cw_speed_weight"] = float(cw_w)
        cfg_s["guidance"]["r_cor_speed_gain"] = float(cor_gain)

        r_ref, v_ref = build_reference_state(cfg_s)
        r0, v0 = get_initial_relative_state(cfg_s)
        r_nom, v_nom, _ = apply_release_targeting(cfg_s, r_ref, v_ref, r0, v0)

        disp_cfg = cfg_s.get("dispersion", {})
        r_bounds = np.array(disp_cfg.get("r_m", [0.0, 0.0, 0.0]), dtype=float)
        v_bounds = np.array(disp_cfg.get("v_mps", [0.0, 0.0, 0.0]), dtype=float)

        r_act = r_nom + rng.uniform(-r_bounds, r_bounds)
        v_act = v_nom + rng.uniform(-v_bounds, v_bounds)

        _, gates, _ = run_sim_config(
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
            count_cor += 1
            cor_speeds.append(gates.crossings["R_COR"]["speed_mps"])
        if "R_LATCH" in gates.crossings:
            count_latch += 1

    p_cor = count_cor / max(RUNS, 1)
    p_latch = count_latch / max(RUNS, 1)
    if cor_speeds:
        arr = np.array(cor_speeds, dtype=float)
        p95 = float(np.percentile(arr, 95))
        vmax = float(np.max(arr))
    else:
        p95 = float("nan")
        vmax = float("nan")

    feasible = (not math.isnan(p95)) and p95 <= 0.22 and vmax <= 0.30

    return {
        "lead_s": float(lead_s),
        "cw_speed_weight": float(cw_w),
        "r_cor_speed_gain": float(cor_gain),
        "p_r_cor": p_cor,
        "p_r_latch": p_latch,
        "r_cor_speed_p95": p95,
        "r_cor_speed_max": vmax,
        "feasible_speed": feasible,
    }


def pareto_front(rows):
    front = []
    for a in rows:
        dominated = False
        for b in rows:
            if b is a:
                continue
            if (
                b["p_r_cor"] >= a["p_r_cor"]
                and b["p_r_latch"] >= a["p_r_latch"]
                and (b["p_r_cor"] > a["p_r_cor"] or b["p_r_latch"] > a["p_r_latch"])
            ):
                dominated = True
                break
        if not dominated:
            front.append(a)
    front.sort(key=lambda x: (-x["p_r_cor"], -x["p_r_latch"], x["r_cor_speed_p95"]))
    return front


def main():
    combos = []
    idx = 0
    for lead_s, cw_w, cor_gain in itertools.product(LEADS, CW_SPEED_WEIGHTS, R_COR_SPEED_GAINS):
        combos.append((idx, lead_s, cw_w, cor_gain))
        idx += 1

    max_workers = min(8, max(1, (os.cpu_count() or 4)))
    results = []
    print(f"Running {len(combos)} combos with {max_workers} workers...")

    with ProcessPoolExecutor(max_workers=max_workers) as ex:
        futs = {ex.submit(evaluate_combo, c): c for c in combos}
        done = 0
        for fut in as_completed(futs):
            res = fut.result()
            results.append(res)
            done += 1
            print(
                f"[{done:02d}/{len(combos)}] "
                f"lead={res['lead_s']:.0f} cw={res['cw_speed_weight']:.1f} gain={res['r_cor_speed_gain']:.1f} "
                f"Pcor={res['p_r_cor']:.3f} Platch={res['p_r_latch']:.3f} "
                f"p95={res['r_cor_speed_p95']:.3f} max={res['r_cor_speed_max']:.3f} "
                f"feasible={res['feasible_speed']}"
            )

    results.sort(key=lambda x: (x["lead_s"], x["cw_speed_weight"], x["r_cor_speed_gain"]))
    os.makedirs("outputs", exist_ok=True)
    with open(OUT_CSV, "w", newline="", encoding="ascii") as f:
        writer = csv.DictWriter(
            f,
            fieldnames=[
                "lead_s",
                "cw_speed_weight",
                "r_cor_speed_gain",
                "p_r_cor",
                "p_r_latch",
                "r_cor_speed_p95",
                "r_cor_speed_max",
                "feasible_speed",
            ],
        )
        writer.writeheader()
        writer.writerows(results)

    feasible = [r for r in results if r["feasible_speed"]]
    pareto = pareto_front(feasible)

    print("\nSummary")
    print(f"Total combos: {len(results)}")
    print(f"Feasible speed combos: {len(feasible)}")
    if feasible:
        best = max(feasible, key=lambda r: (r["p_r_cor"], r["p_r_latch"]))
        print(
            "Best feasible by P(R_COR): "
            f"lead={best['lead_s']:.0f} cw={best['cw_speed_weight']:.1f} gain={best['r_cor_speed_gain']:.1f} "
            f"Pcor={best['p_r_cor']:.3f} Platch={best['p_r_latch']:.3f} "
            f"p95={best['r_cor_speed_p95']:.3f} max={best['r_cor_speed_max']:.3f}"
        )
    else:
        print("No feasible combo met speed constraints.")

    print("\nPareto (feasible, by P(R_COR), P(R_LATCH))")
    for r in pareto:
        print(
            f"lead={r['lead_s']:.0f} cw={r['cw_speed_weight']:.1f} gain={r['r_cor_speed_gain']:.1f} "
            f"Pcor={r['p_r_cor']:.3f} Platch={r['p_r_latch']:.3f} "
            f"p95={r['r_cor_speed_p95']:.3f} max={r['r_cor_speed_max']:.3f}"
        )

    print(f"\nSaved full table: {OUT_CSV}")


if __name__ == "__main__":
    main()
