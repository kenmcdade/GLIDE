# GLIDE Simulation Framework

This repository contains early-stage physics, controls, and numerical models supporting **GLIDE**: a propellant-independence concept for in-space maneuvering using tether-based effects (electrodynamic tether authority, momentum exchange) and capture-oriented rendezvous logic.

This codebase is a **research and exploration environment**, not a flight-ready simulator. Assumptions are explicit and evolve as models mature.

## What’s “GLIDE V4” in this repo?

**GLIDE V4** is the current working architecture focused on **propellant-less maneuvering for the payload** via:

- a reusable guidance **Tag** providing midcourse authority (e.g., DDM + mini-EDT modeled authority), and
- **receiver-side terminal capture control** enforcing strict relative-speed limits through corridor entry → pre-capture → latch gates.

V4 work emphasizes **dispersion recovery**, **corridor entry**, and **terminal closure** validated with Monte Carlo campaigns and gate-speed constraint checks.

> NOTE: V4 does *not* imply a full operational system implementation. This repo focuses on the physics/control feasibility and the requirements that fall out of that analysis.

## Repository Contents

High level:
- Concept-level physics models for:
  - Electrodynamic tether forces / Lorentz authority
  - Orbital energy exchange and scaling behavior
  - Gravity and simplified environmental assumptions
- Numerical and visualization tools for inspecting system behavior over time

### Numeric Simulation Core (v3.1)

The directory `glide_v3_1/` contains a non-visual, numerically stable simulation core intended for engineering analysis and energy accounting.

This core focuses on:
- Explicit energy tracking
- Stability under stiff tether dynamics
- Separation of configuration, integration, and subsystem models

See `glide_v3_1/README.md` for details and usage instructions.

## Where to start

If you are new here:
1) Start with the top-level docs in each versioned directory (e.g., `glide_v3_1/README.md`).
2) For V4 work, look for directories/configs/scripts that reference:
   - “tagged transfer”, “dispersion recovery”, “terminal capture”
   - gate terminology: `R_ACQ`, `R_COR`, `R_PRE`, `R_LATCH`
   - Monte Carlo runners / result CSVs / validation summaries

## Notes

This codebase is under active development; assumptions are made explicit and may change as the model evolves.
