import numpy as np

class TetherSystem:
    def __init__(self, params):
        self.m_node = params["m_node"]
        self.m_payload = params["m_payload"]
        self.L0 = params["L0"]
        self.k = params["k_tether"]
        self.c = params["c_damp"]
        self.dt = params["dt"]
        self.released = False
        self.release_time = params["release_time"]

        # initial state
        self.r_node = np.array([0.0, 0.0])
        self.r_payload = np.array([self.L0, 0.0])
        self.v_node = np.zeros(2)
        self.v_payload = np.zeros(2)
        self.current_tension = 0.0

    def update(self, t, motor):
        # 1️⃣ Check for release first (before computing forces)
        if not self.released and t >= self.release_time:
            self.released = True
            print(f"[{t:.2f}s] Tether Released")

        # 2️⃣ Compute relative geometry
        r_rel = self.r_payload - self.r_node
        dist = np.linalg.norm(r_rel)
        u = r_rel / dist if dist != 0 else np.zeros(2)

        # 3️⃣ Force setup based on tether state
        if not self.released:
            stretch = dist - self.L0
            tension = self.k * max(stretch, 0) + self.c * np.dot(self.v_payload - self.v_node, u)
            tension = max(tension, 0)
            self.current_tension = tension

            tangent = np.array([-u[1], u[0]])
            F_ext_payload = motor.get_force(t, tangent)
        else:
            tension = 0.0
            self.current_tension = 0.0
            F_ext_payload = np.zeros(2)

        # 4️⃣ Apply tether forces if attached
        F_tether_payload = -tension * u if not self.released else np.zeros(2)
        F_tether_node = tension * u if not self.released else np.zeros(2)

        # 5️⃣ Integrate motion
        a_node = F_tether_node / self.m_node
        a_payload = (F_tether_payload + F_ext_payload) / self.m_payload

        self.v_node += a_node * self.dt
        self.v_payload += a_payload * self.dt
        self.r_node += self.v_node * self.dt
        self.r_payload += self.v_payload * self.dt
