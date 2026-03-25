"""Monte Carlo runner (Phase 2 stub)."""

import copy
import numpy as np
from sim.runner import run_sim_config
from sim.scenario import load_config
from mc.sampler import sample_config


def run_mc(config_path):
    cfg = load_config(config_path)
    mc_cfg = cfg.get("mc", {})
    if not mc_cfg.get("enabled", False):
        return None

    runs = int(mc_cfg.get("runs", 10))
    seed = int(mc_cfg.get("seed", 0))
    rng = np.random.default_rng(seed)

    results = []
    for _ in range(runs):
        cfg_s = sample_config(cfg, rng)
        cfg_s["simulation"]["save_plots"] = False
        metrics, gates, _ = run_sim_config(cfg_s)
        results.append({
            "success": metrics["success"],
            "tags": metrics["tags"],
            "gates": gates.crossings,
        })

    # Corridor entry probability
    corridor_success = 0
    for r in results:
        if "R_COR" in r["gates"]:
            corridor_success += 1
    p_corr = corridor_success / max(len(results), 1)

    sweep = {}
    for tof in mc_cfg.get("tof_sweep_s", []):
        cfg_sweep = copy.deepcopy(cfg)
        cfg_sweep["simulation"]["t_end_s"] = float(tof)
        cfg_sweep["simulation"]["save_plots"] = False
        sweep_results = []
        for _ in range(runs):
            cfg_s = sample_config(cfg_sweep, rng)
            metrics, gates, _ = run_sim_config(cfg_s)
            sweep_results.append("R_COR" in gates.crossings)
        sweep[str(tof)] = float(sum(sweep_results) / max(len(sweep_results), 1))

    return {
        "runs": runs,
        "p_corr": p_corr,
        "p_corr_sweep": sweep,
        "results": results,
    }
