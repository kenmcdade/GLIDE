"""Simulation logger."""

import numpy as np


class SimLogger:
    def __init__(self):
        self.data = {}

    def log(self, **kwargs):
        for key, value in kwargs.items():
            if key not in self.data:
                self.data[key] = []
            self.data[key].append(value)

    def as_arrays(self):
        out = {}
        for key, values in self.data.items():
            out[key] = np.array(values)
        return out
