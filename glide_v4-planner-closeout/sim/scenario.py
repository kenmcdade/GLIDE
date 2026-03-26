"""Scenario loading and initialization."""

from pathlib import Path
import numpy as np
import yaml

from dynamics.constants import R_EARTH
from dynamics.orbit import coe_to_state
from dynamics.frames import lvlh_rel_to_eci
from dynamics.atmosphere import AtmosphereModel


def load_config(path):
    path = Path(path)
    with path.open("r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def build_atmosphere(cfg):
    atm_cfg = cfg["environment"]["atmosphere"]
    return AtmosphereModel(atm_cfg["model"], atm_cfg["params"])


def build_reference_state(cfg):
    orbit = cfg["orbit"]
    alt_m = float(orbit["altitude_km"]) * 1000.0
    a = R_EARTH + alt_m
    e = float(orbit.get("ecc", 0.0))
    inc = np.deg2rad(float(orbit["inclination_deg"]))
    raan = np.deg2rad(float(orbit.get("raan_deg", 0.0)))
    argp = np.deg2rad(float(orbit.get("arg_perigee_deg", 0.0)))
    nu = np.deg2rad(float(orbit.get("true_anomaly_deg", 0.0)))
    return coe_to_state(a, e, inc, raan, argp, nu)


def get_initial_relative_state(cfg):
    rel_cfg = cfg["initial_relative_state_lvh_m"]
    r_rel_lvh = np.array(rel_cfg["r_m"], dtype=float)
    v_rel_lvh = np.array(rel_cfg["v_mps"], dtype=float)
    return r_rel_lvh, v_rel_lvh


def payload_state_from_rel(r_ref, v_ref, r_rel_lvh, v_rel_lvh):
    r_rel_eci, v_rel_eci = lvlh_rel_to_eci(r_rel_lvh, v_rel_lvh, r_ref, v_ref)
    r_payload = r_ref + r_rel_eci
    v_payload = v_ref + v_rel_eci
    return r_payload, v_payload


def build_initial_states(cfg):
    r_ref, v_ref = build_reference_state(cfg)
    r_rel_lvh, v_rel_lvh = get_initial_relative_state(cfg)
    r_payload, v_payload = payload_state_from_rel(r_ref, v_ref, r_rel_lvh, v_rel_lvh)
    return (r_ref, v_ref), (r_payload, v_payload)
