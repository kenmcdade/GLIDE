# ============================================================
# GLIDE Simulation Framework (v4)
# Modular version: Node ↔ Payload tether dynamics
# ============================================================

import numpy as np
from physics.tether import TetherSystem
from physics.motor import WinchMotor
from physics.gravity import GravityField
from physics.electrodynamic import ElectrodynamicTether
from physics.energy import EnergyTracker
from viz.animator import GlideAnimator
from viz.plots import DiagnosticPlots

# ------------------------------------------------------------
# GLOBAL PARAMETERS
# ------------------------------------------------------------
params = {
    "m_node": 25.0,
    "m_payload": 150.0,
    "L0": 300.0,
    "k_tether": 1000.0,
    "c_damp": 5.0,
    "F_winch_max": 800.0,
    "dt": 0.01,
    "t_final": 20.0,
    "release_time": 5.0,
    "enable_gravity": True,
    "enable_electrodynamic": True,
    "enable_energy": True
}

# ------------------------------------------------------------
# SUBSYSTEM INITIALIZATION
# ------------------------------------------------------------
tether = TetherSystem(params)
motor = WinchMotor(params)
gravity = GravityField(params)
electro = ElectrodynamicTether(params)
energy = EnergyTracker(params)

# ------------------------------------------------------------
# SIMULATION LOOP
# ------------------------------------------------------------
positions_node, positions_payload, tether_forces = [], [], []
energy_log = []

for t in np.arange(0, params["t_final"], params["dt"]):
    # update subsystems
    tether.update(t, motor)
    gravity.apply(tether)
    electro.apply(tether)
    energy.record(t, tether)

    # record
    positions_node.append(tether.r_node.copy())
    positions_payload.append(tether.r_payload.copy())
    tether_forces.append(tether.current_tension)
    energy_log.append(energy.snapshot())

# ------------------------------------------------------------
# VISUALIZATION
# ------------------------------------------------------------
anim = GlideAnimator(tether, positions_node, positions_payload, params)
anim.run()

if params["enable_energy"]:
    diag = DiagnosticPlots(tether, energy_log, tether_forces, params)
    diag.show()
