# GLIDE Simulation Framework

This repository contains early-stage physics and numerical models supporting GLIDE,
an orbital-scale momentum exchange and electrodynamic reboost concept focused on
reducing propellant dependence for in-space maneuvering.

The goal of this project is to explore feasibility, scaling behavior, and energy
constraints of tether-based orbital logistics using simplified but physically
grounded models.

This repository is intended as a research and exploration environment rather than
a flight-ready simulator.

## Repository Contents

- Concept-level physics models for:
  - Electrodynamic tether forces
  - Orbital energy exchange
  - Gravity and basic propulsion assumptions
- Numerical and visualization tools for inspecting system behavior over time

## Numeric Simulation Core (v3.1)

The directory `glide_v3_1/` contains a non-visual, numerically stable simulation core
intended for engineering analysis and energy accounting.

This core focuses on:
- Explicit energy tracking
- Stability under stiff tether dynamics
- Separation of configuration, integration, and subsystem models

See `glide_v3_1/README.md` for details and usage instructions.

## Notes

This codebase is under active development. Assumptions are made explicit and may
change as the model evolves.

