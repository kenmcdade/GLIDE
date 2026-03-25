"""Orbit propagation utilities."""

import numpy as np
from .constants import MU_EARTH, R_EARTH, J2
from .drag import drag_accel


def coe_to_state(a, e, inc, raan, argp, nu, mu=MU_EARTH):
    p = a * (1.0 - e * e)
    r_pf = np.array([
        p * np.cos(nu) / (1.0 + e * np.cos(nu)),
        p * np.sin(nu) / (1.0 + e * np.cos(nu)),
        0.0,
    ])
    v_pf = np.array([
        -np.sqrt(mu / p) * np.sin(nu),
        np.sqrt(mu / p) * (e + np.cos(nu)),
        0.0,
    ])

    c_raan = np.cos(raan)
    s_raan = np.sin(raan)
    c_inc = np.cos(inc)
    s_inc = np.sin(inc)
    c_argp = np.cos(argp)
    s_argp = np.sin(argp)

    R3_raan = np.array([
        [c_raan, -s_raan, 0.0],
        [s_raan, c_raan, 0.0],
        [0.0, 0.0, 1.0],
    ])
    R1_inc = np.array([
        [1.0, 0.0, 0.0],
        [0.0, c_inc, -s_inc],
        [0.0, s_inc, c_inc],
    ])
    R3_argp = np.array([
        [c_argp, -s_argp, 0.0],
        [s_argp, c_argp, 0.0],
        [0.0, 0.0, 1.0],
    ])

    Q = R3_raan @ R1_inc @ R3_argp
    r_eci = Q @ r_pf
    v_eci = Q @ v_pf
    return r_eci, v_eci


def accel_gravity(r_eci):
    r = np.asarray(r_eci, dtype=float)
    r_norm = np.linalg.norm(r)
    return -MU_EARTH * r / (r_norm ** 3)


def accel_j2(r_eci):
    r = np.asarray(r_eci, dtype=float)
    x, y, z = r
    r_norm = np.linalg.norm(r)
    if r_norm < 1.0:
        return np.zeros(3)
    z2 = z * z
    r2 = r_norm * r_norm
    factor = 1.5 * J2 * MU_EARTH * (R_EARTH ** 2) / (r_norm ** 5)
    term = 5.0 * z2 / r2 - 1.0
    ax = factor * x * term
    ay = factor * y * term
    az = factor * z * (5.0 * z2 / r2 - 3.0)
    return np.array([ax, ay, az])


def accel_drag_model(r_eci, v_eci, env):
    if env is None:
        return np.zeros(3)
    atm = env.get("atmosphere")
    if atm is None:
        return np.zeros(3)
    alt = np.linalg.norm(r_eci) - R_EARTH
    rho = atm.density(alt)
    Cd = float(env.get("Cd", 2.2))
    area = float(env.get("area", 0.1))
    mass = float(env.get("mass", 100.0))
    return drag_accel(rho, v_eci, Cd, area, mass)


def eom(t, state, env=None, control_accel=None):
    r = state[0:3]
    v = state[3:6]
    a = accel_gravity(r) + accel_j2(r)
    a += accel_drag_model(r, v, env)
    if control_accel is not None:
        a += control_accel
    return np.hstack((v, a))
