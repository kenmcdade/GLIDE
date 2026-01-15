# GLIDE Simulation Framework

This repository contains early-stage physics and visualization models supporting **GLIDE**, an orbital-scale momentum exchange and electrodynamic reboost concept for propellant-minimized spacecraft maneuvering.

The goal of this project is to explore **feasibility, scaling behavior, and energy constraints** of tether-based orbital logistics using simplified but physically grounded models. The emphasis is on conceptual validation and inspection, not flight qualification.

## What This Repository Contains

**Core physics and modeling work for:**
- Electrodynamic tether forces  
- Orbital energy and momentum exchange  
- Gravity and simplified propulsion assumptions  

**Supporting tools for:**
- Numerical integration and scenario exploration  
- Visualization and plotting of orbital trajectories and key metrics  
- Sanity checks on conservation behavior and system trends  

## Repository Structure

- `glide_sim/`  
  Modular simulation framework containing the core physics models, numerical integration logic, and visualization utilities.  
  Includes a runnable driver script (`main.py`) that ties components together.

- `standalone_demos/`  
  Self-contained Python scripts demonstrating specific GLIDE-style dynamics without relying on the internal framework.  
  Intended as quick, inspectable conceptual demonstrations.

## Quick Start

Install dependencies:

    pip install -r requirements.txt

Run the core simulation:

    python glide_sim/main.py

Run a standalone demonstration:

    python standalone_demos/glide_orbit.py

The standalone demo produces an animated orbital trajectory followed by diagnostic plots illustrating key post-event metrics.

## Scope and Limitations

This repository is **not** a flight-ready simulator.

Models are intentionally simplified to:
- isolate dominant physical effects  
- explore scaling behavior  
- support early feasibility analysis  

Environmental effects, structural dynamics, operational constraints, and system-level integration are outside the current scope.

## Roadmap (Near-Term)

- Parameter sweep tooling for scenario exploration  
- Additional standalone demonstrations covering different orbital regimes  
- Improved documentation of modeling assumptions  
- Lightweight validation tests for core numerical components  

## License

Released under the MIT License.

## Contact

Questions, technical feedback, or collaboration interest are welcome.
