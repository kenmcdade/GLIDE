"""Frame transforms between ECI and LVLH."""

import numpy as np


def lvlh_basis(r_ref, v_ref):
    r_ref = np.asarray(r_ref, dtype=float)
    v_ref = np.asarray(v_ref, dtype=float)
    r_hat = r_ref / np.linalg.norm(r_ref)
    h_hat = np.cross(r_ref, v_ref)
    h_hat = h_hat / np.linalg.norm(h_hat)
    t_hat = np.cross(h_hat, r_hat)
    t_hat = t_hat / np.linalg.norm(t_hat)
    # Rows are basis vectors so that r_lvh = C @ r_eci
    C = np.vstack((r_hat, t_hat, h_hat))
    return C


def lvlh_omega(r_ref, v_ref):
    r_ref = np.asarray(r_ref, dtype=float)
    v_ref = np.asarray(v_ref, dtype=float)
    return np.cross(r_ref, v_ref) / (np.linalg.norm(r_ref) ** 2)


def eci_to_lvlh(r_rel, v_rel, r_ref, v_ref):
    C = lvlh_basis(r_ref, v_ref)
    omega = lvlh_omega(r_ref, v_ref)
    r_l = C @ r_rel
    v_l = C @ (v_rel - np.cross(omega, r_rel))
    return r_l, v_l


def eci_to_lvlh_vector(vec_eci, r_ref, v_ref):
    C = lvlh_basis(r_ref, v_ref)
    return C @ vec_eci


def lvlh_to_eci_vector(vec_lvh, r_ref, v_ref):
    C = lvlh_basis(r_ref, v_ref)
    return C.T @ vec_lvh


def lvlh_rel_to_eci(r_rel_lvh, v_rel_lvh, r_ref, v_ref):
    C = lvlh_basis(r_ref, v_ref)
    omega = lvlh_omega(r_ref, v_ref)
    r_rel_eci = C.T @ r_rel_lvh
    v_rel_eci = C.T @ v_rel_lvh + np.cross(omega, r_rel_eci)
    return r_rel_eci, v_rel_eci
