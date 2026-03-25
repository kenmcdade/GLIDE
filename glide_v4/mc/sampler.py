"""Sampling utilities for Monte Carlo runs."""

import copy
import numpy as np


def sample_config(cfg, rng):
    cfg_s = copy.deepcopy(cfg)
    unc = cfg.get("mc", {}).get("uncertainties", {})

    cd_sigma = float(unc.get("cd_sigma", 0.0))
    if cd_sigma > 0.0:
        cd_scale = 1.0 + rng.normal(0.0, cd_sigma)
        cd_scale = max(0.1, cd_scale)
        cfg_s["payload"]["Cd"] *= cd_scale

    eta_sigma = float(unc.get("eta_sigma", 0.0))
    if eta_sigma > 0.0:
        eta_scale = 1.0 + rng.normal(0.0, eta_sigma)
        eta_scale = max(0.0, eta_scale)
        cfg_s["medt"]["eta"] *= eta_scale

    pos_sigma = float(unc.get("initial_pos_sigma_m", 0.0))
    vel_sigma = float(unc.get("initial_vel_sigma_mps", 0.0))
    if pos_sigma > 0.0:
        noise = rng.normal(0.0, pos_sigma, size=3)
        cfg_s["initial_relative_state_lvh_m"]["r_m"] = (
            np.array(cfg_s["initial_relative_state_lvh_m"]["r_m"]) + noise
        ).tolist()
    if vel_sigma > 0.0:
        noise = rng.normal(0.0, vel_sigma, size=3)
        cfg_s["initial_relative_state_lvh_m"]["v_mps"] = (
            np.array(cfg_s["initial_relative_state_lvh_m"]["v_mps"]) + noise
        ).tolist()

    rho_sigma = float(unc.get("rho_scale_sigma", 0.0))
    if rho_sigma > 0.0:
        rho_scale = 1.0 + rng.normal(0.0, rho_sigma)
        cfg_s["environment"]["atmosphere"]["params"]["rho_scale"] = rho_scale

    return cfg_s
