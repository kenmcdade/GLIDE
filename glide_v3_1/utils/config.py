# ============================================================
# File: utils/config.py
# GLIDE v3.1 – Configuration and Simulation Parameters (fixed)
# ============================================================

from __future__ import annotations


def load_config(mode: str = "engineering") -> dict:
    """Load a GLIDE simulation configuration preset.

    Modes:
      • local_demo    – simple ground demo
      • orbital_test  – pseudo-orbital constant-g validation (your current approach)
      • engineering   – subsystem testing
    """

    cfg = {
        # --- Integration ---
        "dt": 0.01,  # [s] control-rate step

        # --- Tether subsystem ---
        "N": 25,
        "L0": 200.0,
        "E": 30e9,
        "d": 0.005,
        "rho_l": 1.5,
        "tether_damping_ratio": 0.05,

        # Substepping (THIS is what makes dt=0.05 viable)
        "tether_max_substep_dt": 0.005,
        "constraint_iterations": 3,

        # Optional numerics-only velocity decay (per second). Leave 0 for pure physics.
        "numerical_vel_decay_per_s": 0.0,

        # --- Motor subsystem ---
        "R_spool": 0.25,
        "J_m": 0.08,
        "tau_max": 15.0,
        "b_m": 0.02,
        "eta": 0.87,

        # --- Gravity ---
        "gravity_mode": "local",
        "local_g": 9.80665,
        # Orbital params (only used if gravity_mode == "orbital")
        "mu": 3.986004418e14,
        "R_earth": 6.371e6,

        # --- EDT ---
        "EDT_L": 100.0,
        # Back-compat: can be length 2 or 3; last element is treated as out-of-plane Bz.
        "B_vec": [0.0, 0.0, 3.1e-5],
        "EDT_R": 50.0,
        "EDT_Imax": 2.5,
        "EDT_mode": "off",

        # --- Battery ---
        "battery_capacity_J": 5e5,

        # --- Debug / Output ---
        "log_interval": 1.0,
        "save_energy_csv": True,

        # Optional safety clamp (m/s). 0 disables.
        "velocity_limit": 0.0,
    }

    mode = str(mode).lower().strip()

    if mode == "orbital_test":
        cfg.update({
            "gravity_mode": "local",   # pseudo-orbital constant-g
            "local_g": 8.70,
            "L0": 300.0,
            "N": 40,
            "E": 40e9,
            "rho_l": 1.8,
            "dt": 0.05,                 # coarse control step
            "tether_max_substep_dt": 0.005,
            "constraint_iterations": 4,
            "EDT_mode": "drag",
            "battery_capacity_J": 1e6,
        })

    elif mode == "engineering":
        cfg.update({
            "L0": 250.0,
            "N": 25,
            "E": 5e9,
            "rho_l": 1.6,
            "EDT_mode": "boost",
            "EDT_Imax": 0.5,
            "tau_max": 8.0,
            "battery_capacity_J": 2e6,
            "dt": 0.005,
            "tether_max_substep_dt": 0.005,
            "constraint_iterations": 3,
        })

    elif mode == "local_demo":
        cfg.update({
            "L0": 100.0,
            "N": 15,
            "gravity_mode": "local",
            "local_g": 9.80665,
            "EDT_mode": "off",
            "dt": 0.005,
            "tether_max_substep_dt": 0.005,
            "constraint_iterations": 2,
        })

    else:
        raise ValueError(f"Unknown configuration mode: {mode}")

    return cfg


def print_config_summary(cfg: dict):
    print("\n================ GLIDE CONFIGURATION ================")
    for k, v in cfg.items():
        if isinstance(v, float):
            print(f"{k:<24s} : {v:>12.6f}")
        else:
            print(f"{k:<24s} : {v}")
    print("=====================================================\n")
