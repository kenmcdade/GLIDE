import numpy as np

class GravityField:
    def __init__(self, params):
        self.enabled = params.get("enable_gravity", False)
        self.g = 8.7e-3  # m/s^2 differential accel typical in LEO

    def apply(self, tether):
        if not self.enabled:
            return
        # apply simple gradient: pulls payload "down"
        gravity_vec = np.array([0.0, -self.g])
        tether.v_payload += gravity_vec * tether.dt
        tether.v_node -= gravity_vec * tether.dt * (tether.m_payload / tether.m_node)
