"""Run deterministic demo scenario with debug and feasibility reporting."""

import sys
import numpy as np

from sim.runner import run_sim_config, apply_release_targeting
from sim.scenario import load_config, build_atmosphere, build_reference_state, get_initial_relative_state
from sim.feasibility import estimate_feasibility
from sim.plotter import plot_all
from sim.reporting import print_start_end_summary, print_feasibility, print_targeting


def main():
    config_path = sys.argv[1] if len(sys.argv) > 1 else "configs/demo.yaml"
    cfg = load_config(config_path)

    # Build initial states and feasibility check.
    atm = build_atmosphere(cfg)
    r_ref, v_ref = build_reference_state(cfg)
    r_rel_lvh0, v_rel_lvh0 = get_initial_relative_state(cfg)

    env_payload = {
        "mass": float(cfg["payload"]["mass_kg"]),
        "Cd": float(cfg["payload"]["Cd"]),
        "area": 0.0,
        "atmosphere": atm,
    }

    feas_initial = estimate_feasibility(cfg, r_rel_lvh0, v_rel_lvh0, r_ref, v_ref, env_payload)
    print("Initial Feasibility (Pre-Targeting)")
    print_feasibility(feas_initial, label="INITIAL")

    # Release targeting (lite) - nominal target
    r_rel_lvh_tgt, v_rel_lvh_tgt, targeting_report = apply_release_targeting(
        cfg, r_ref, v_ref, r_rel_lvh0, v_rel_lvh0
    )
    print_targeting(targeting_report)

    # Apply deterministic dispersion if configured (actual initial state).
    disp_cfg = cfg.get("dispersion", {})
    if disp_cfg.get("enabled", False):
        r_disp = disp_cfg.get("r_m", [0.0, 0.0, 0.0])
        v_disp = disp_cfg.get("v_mps", [0.0, 0.0, 0.0])
        r_sign = disp_cfg.get("signs_r", [1.0, 1.0, 1.0])
        v_sign = disp_cfg.get("signs_v", [1.0, 1.0, 1.0])
        r_rel_lvh_act = r_rel_lvh_tgt + (np.array(r_disp) * np.array(r_sign))
        v_rel_lvh_act = v_rel_lvh_tgt + (np.array(v_disp) * np.array(v_sign))
        print(f"Applied dispersion r (RTN): {r_disp} signs {r_sign}")
        print(f"Applied dispersion v (RTN): {v_disp} signs {v_sign}")
    else:
        r_rel_lvh_act = r_rel_lvh_tgt
        v_rel_lvh_act = v_rel_lvh_tgt

    feas_targeted = estimate_feasibility(cfg, r_rel_lvh_act, v_rel_lvh_act, r_ref, v_ref, env_payload)
    print("Feasibility (Post-Targeting)")
    print_feasibility(feas_targeted, label="TARGETED")

    if feas_targeted["infeasible"]:
        print("Run skipped: infeasible_initial_condition")
        return

    debug_cfg = cfg.get("debug", {})
    dual = bool(debug_cfg.get("dual_run", True))

    control_on = bool(cfg["simulation"].get("control_enabled", True))
    guidance_on = bool(cfg["simulation"].get("guidance_enabled", True))

    # Feasibility check per run (same initial for ON/OFF).
    print_feasibility(feas_targeted, label="ON")
    if dual:
        print_feasibility(feas_targeted, label="OFF")

    # Actuation ON run
    metrics_on, gates_on, logger_on = run_sim_config(
        cfg,
        control_enabled=control_on,
        guidance_enabled=guidance_on,
        r_rel_lvh0=r_rel_lvh_act,
        v_rel_lvh0=v_rel_lvh_act,
        r_rel_lvh_nom0=r_rel_lvh_tgt,
        v_rel_lvh_nom0=v_rel_lvh_tgt,
    )

    if dual:
        metrics_off, gates_off, logger_off = run_sim_config(
            cfg,
            control_enabled=False,
            guidance_enabled=guidance_on,
            r_rel_lvh0=r_rel_lvh_act,
            v_rel_lvh0=v_rel_lvh_act,
            r_rel_lvh_nom0=r_rel_lvh_tgt,
            v_rel_lvh_nom0=v_rel_lvh_tgt,
        )
    else:
        metrics_off, gates_off, logger_off = None, None, None

    print("GLIDE V4 Demo Result (Actuation ON)")
    print(f"Success: {metrics_on['success']}")
    print(f"Tags: {metrics_on['tags']}")
    print_start_end_summary(cfg, logger_on, metrics_on, gates_on, label="ON")

    if dual:
        print("GLIDE V4 Demo Result (Actuation OFF)")
        print_start_end_summary(cfg, logger_off, metrics_off, gates_off, label="OFF")

    # Plotting
    if cfg["simulation"].get("save_plots", True):
        data_on = logger_on.as_arrays()
        data_off = logger_off.as_arrays() if logger_off is not None else None
        plot_all(data_on, gates_on, cfg["simulation"]["output_dir"], data_off=data_off)


if __name__ == "__main__":
    main()
