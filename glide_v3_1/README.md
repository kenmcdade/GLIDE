GLIDE v3.1 — Tether / EDT Simulation (2D)

This folder contains the GLIDE v3.1 simulator: a 2D multi-segment tether dynamics model with an anchor-side winch/motor boundary condition, optional electrodynamic tether (EDT) forcing on the payload, and energy/power bookkeeping.

Primary goals:

Validate numerical stability across different control timesteps (dt)

Compare scenarios (engineering, orbital_test, local_demo)

Track energy buckets + battery SoC to catch non-physical runaway

Folder layout

glide_v3_1/

main.py

core/

integrator.py

tether.py

motor.py

gravity.py

edt.py

energy.py

utils/

config.py

What it simulates

Tether (core/tether.py)

N axial segments (N+1 nodes)

Spring-damper axial elements

Optional constraint enforcement to keep segment lengths well-behaved

External forces are applied by the integrator (not inside the tether module)

Anchor / winch motor (core/motor.py)

The anchor node is a moving boundary driven by the motor model

Motor responds to tether tension and a commanded angular velocity profile

Anchor motion is applied continuously through the integrator

Gravity (core/gravity.py)

local: constant downward acceleration

orbital: simple two-body field about the origin (only if enabled in config)

EDT (core/edt.py)

Optional force applied to the payload (tip node)

Modes: off, boost, drag

Includes resistive losses and a battery power model

Energy logging (core/energy.py)
Tracks:

Kinetic energy (tether nodes)

Elastic energy (tether strain)

Gravitational energy (relative to an anchor reference)

Battery energy + SoC

Total energy (mechanical + battery)

Optional CSV output:

glide_v3_energy_log.csv

Running

Run from inside the glide_v3_1 folder:

py main.py engineering

Other modes:

py main.py orbital_test
py main.py local_demo

Mode presets live in utils/config.py inside load_config(mode).

Operating / tuning

Edit mode presets (utils/config.py)
Key parameters:

dt: control timestep (outer loop)

tether_max_substep_dt: inner tether substep size (stability control)

constraint_iterations: number of constraint passes per substep

EDT_mode: off / boost / drag

EDT_Imax, EDT_R, B_vec: EDT electrical / field parameters

battery_capacity_J: starting battery energy

Edit command profiles (main.py)

winch_profile(t): motor angular velocity command

current_profile(t): EDT current command (normalized)

Timestep guidance (important)
If you increase stiffness (E), segment count (N), or reduce damping, the tether becomes more numerically stiff.

You can keep dt as a coarse control step, but keep tether_max_substep_dt small (for example 0.005 or below).

If you see oscillations grow, reduce dt and/or tether_max_substep_dt, or reduce constraint_iterations.

Outputs

Console output
Periodic energy summaries print at a cadence set by log_interval in utils/config.py.

CSV output
If save_energy_csv is True, the sim exports glide_v3_energy_log.csv with columns:
time, E_kin, E_elastic, E_grav, E_batt, E_total, SoC

Notes / intended use

This is a numerical validation sandbox: stable dynamics, consistent energy accounting, and tunable forcing.

Drag mode does not automatically mean charging in all setups. If resistive losses dominate and/or orbital baseline motion is not represented, battery power can remain net-negative.

If you want to isolate numerical stability, run orbital_test with EDT_mode set to off first, confirm stability, then enable EDT.

Quick troubleshooting

Import errors (for example: “No module named core”)

Run the script from inside the glide_v3_1 folder (cd glide_v3_1, then py main.py engineering).

If your environment requires it, add empty init.py files to core/ and utils/.

Energy runaway
Check in this order:

Confirm tether substepping is enabled (tether_max_substep_dt is small enough)

Reduce constraint_iterations

Validate stability with EDT_mode = off before enabling EDT
