import numpy as np

class ElectrodynamicTether:
    def __init__(self, params):
        self.enabled = params.get("enable_electrodynamic", False)
        self.F_reboost = 2.0  # N constant

    def apply(self, tether):
        if not self.enabled or not tether.released:
            return
        # thrust back toward origin
        dir_back = -tether.r_node / (np.linalg.norm(tether.r_node) + 1e-9)
        F_recovery = self.F_reboost * dir_back
        a_node = F_recovery / tether.m_node
        tether.v_node += a_node * tether.dt
