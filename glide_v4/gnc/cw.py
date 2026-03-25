"""Clohessy-Wiltshire (Hill) relative motion utilities."""

import numpy as np


def cw_matrices(n, t):
    c = np.cos(n * t)
    s = np.sin(n * t)

    phi_rr = np.array([
        [4.0 - 3.0 * c, 0.0, 0.0],
        [6.0 * (s - n * t), 1.0, 0.0],
        [0.0, 0.0, c],
    ])

    phi_rv = np.array([
        [s / n, 2.0 * (1.0 - c) / n, 0.0],
        [2.0 * (c - 1.0) / n, (4.0 * s - 3.0 * n * t) / n, 0.0],
        [0.0, 0.0, s / n],
    ])

    phi_vr = np.array([
        [3.0 * n * s, 0.0, 0.0],
        [6.0 * n * (c - 1.0), 0.0, 0.0],
        [0.0, 0.0, -n * s],
    ])

    phi_vv = np.array([
        [c, 2.0 * s, 0.0],
        [-2.0 * s, 4.0 * c - 3.0, 0.0],
        [0.0, 0.0, c],
    ])

    return phi_rr, phi_rv, phi_vr, phi_vv


def cw_propagate(r0, v0, n, t):
    phi_rr, phi_rv, phi_vr, phi_vv = cw_matrices(n, t)
    r = phi_rr @ r0 + phi_rv @ v0
    v = phi_vr @ r0 + phi_vv @ v0
    return r, v
