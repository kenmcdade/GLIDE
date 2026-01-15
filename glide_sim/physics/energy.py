import numpy as np

class EnergyTracker:
    def __init__(self, params):
        self.enabled = params.get("enable_energy", False)
        self.log = []

    def record(self, t, tether):
        if not self.enabled:
            return
        v2 = 0.5*tether.m_node*np.dot(tether.v_node,tether.v_node) + \
             0.5*tether.m_payload*np.dot(tether.v_payload,tether.v_payload)
        e_tether = 0.5*tether.k*max(np.linalg.norm(tether.r_payload-tether.r_node)-tether.L0,0)**2
        self.log.append([t, v2, e_tether])

    def snapshot(self):
        return self.log[-1] if self.log else [0,0,0]
