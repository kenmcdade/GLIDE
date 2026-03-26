"""Estimator stub (pass-through)."""

import numpy as np


class Estimator:
    def __init__(self):
        pass

    def estimate(self, r_lvh, v_lvh):
        return np.asarray(r_lvh, dtype=float), np.asarray(v_lvh, dtype=float)
