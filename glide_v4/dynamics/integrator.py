"""Simple fixed-step integrators."""

import numpy as np


def rk4_step(fun, t, y, dt):
    k1 = fun(t, y)
    k2 = fun(t + 0.5 * dt, y + 0.5 * dt * k1)
    k3 = fun(t + 0.5 * dt, y + 0.5 * dt * k2)
    k4 = fun(t + dt, y + dt * k3)
    return y + (dt / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4)
