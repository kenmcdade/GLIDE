import matplotlib.pyplot as plt
import numpy as np

class DiagnosticPlots:
    def __init__(self, tether, energy_log, tension_log, params):
        self.energy = np.array(energy_log)
        self.tension = np.array(tension_log)
        self.params = params

    def show(self):
        fig, ax = plt.subplots(2,1,figsize=(8,6))
        t = np.arange(0, self.params["t_final"], self.params["dt"])
        if len(self.energy)>0:
            ax[0].plot(t[:len(self.energy)], self.energy[:,1], label='Kinetic')
            ax[0].plot(t[:len(self.energy)], self.energy[:,2], label='Tether')
            ax[0].legend(); ax[0].set_ylabel("Energy [J]")
        ax[1].plot(t[:len(self.tension)], self.tension, color='steelblue')
        ax[1].set_ylabel("Tether Tension [N]"); ax[1].set_xlabel("Time [s]")
        plt.tight_layout(); plt.show()
