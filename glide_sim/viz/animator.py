import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

class GlideAnimator:
    def __init__(self, tether, node_traj, payload_traj, params):
        self.tether = tether
        self.node_traj = node_traj
        self.payload_traj = payload_traj
        self.params = params

    def run(self):
        fig, ax = plt.subplots(figsize=(8, 6))
        ax.set_xlim(-self.params["L0"] * 1.5, self.params["L0"] * 1.5)
        ax.set_ylim(-self.params["L0"] * 0.5, self.params["L0"] * 1.5)
        ax.set_aspect("equal")

        line_tether, = ax.plot([], [], "-", color="gray")
        dot_node, = ax.plot([], [], "o", color="orange", markersize=10)
        dot_payload, = ax.plot([], [], "o", color="lime", markersize=8)
        time_text = ax.text(0.02, 0.95, "", transform=ax.transAxes)
        status_text = ax.text(0.5, 0.9, "", transform=ax.transAxes,
                              ha="center", va="center",
                              color="red", fontsize=12, weight="bold")

        release_time = self.params.get("release_time", 5.0)
        dt = self.params.get("dt", 0.01)

        def update(frame):
            xN, yN = self.node_traj[frame]
            xP, yP = self.payload_traj[frame]
            t_now = frame * dt

            # Hide the tether line after release
            if t_now >= release_time:
                line_tether.set_data([], [])
                status_text.set_text("TETHER RELEASED — Momentum Exchange Complete")
            else:
                line_tether.set_data([xN, xP], [yN, yP])
                status_text.set_text("")

            dot_node.set_data([xN], [yN])
            dot_payload.set_data([xP], [yP])
            time_text.set_text(f"t = {t_now:.2f}s")

            return line_tether, dot_node, dot_payload, time_text, status_text

        ani = FuncAnimation(
            fig,
            update,
            frames=len(self.node_traj),
            interval=10,
            blit=False,
            repeat=False
        )

        plt.show()
