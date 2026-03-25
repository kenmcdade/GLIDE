# GLIDE V4 Rendezvous Simulation

Proof-of-concept simulation for GLIDE V4 tagged transfer and terminal capture.

The code validates whether a dispersed release can be guided through fixed corridor gates while respecting hard speed limits, using simplified but parameterized actuators (DDM + mEDT + terminal node control).

## Headline Result

Frozen baseline configuration (`configs/dispersion_recovery.yaml`):
- `t_end_s=7500`
- `cor_entry_lead_s=300`
- `cw_speed_weight=0.0`
- `r_cor_speed_gain=0.5`

Baseline robustness at `1x` dispersions, terminal ON, `dt=5s`, `N=1000`:

| Seed | P(R_COR) | P(R_LATCH) | R_COR max speed |
|---:|---:|---:|---:|
| 123 | 0.974 | 0.974 | 0.327 m/s |
| 456 | 0.964 | 0.964 | 0.319 m/s |
| 789 | 0.986 | 0.986 | 0.323 m/s |

This is the `96-99%` MC performance range.

Source: `outputs/mc_baseline_seeds.csv`.

## Current Status

- Baseline (`1x`) is strong and compliant.
- Stress (`2x`) is still the open challenge:
  - baseline policy can violate `R_COR` hard cap.
  - conservative planner policy can remove hard-cap violations, but currently lowers `P(R_LATCH)` below target.

See:
- `outputs/GLIDE_V4_validation_summary.md`
- `outputs/planner_2x_report.md`
- `docs/STATUS.md`

## Quick Start (Windows PowerShell)

```powershell
py -m venv .venv
.\.venv\Scripts\python -m pip install -r requirements.txt
.\.venv\Scripts\python run_demo.py configs/dispersion_recovery.yaml
```

## Reproduce Key Runs

Baseline/stress validation suite:

```powershell
.\.venv\Scripts\python run_validation_suite.py
```

Planner run (`2x` dispersions):

```powershell
.\.venv\Scripts\python run_planner.py
```

## Repository Layout

- `dynamics/`: orbit propagation, J2, drag, atmosphere, mEDT model, frames.
- `gnc/`: guidance, control allocation, release targeting.
- `modes/`: gate logic and mode/state transitions.
- `sim/`: scenario runner, logging, metrics, terminal capture controller.
- `mc/`: Monte Carlo sampling/harness.
- `planner/`: autonomous TOF/release policy search.
- `configs/`: scenario and actuator settings.
- `tests/`: frame and sanity tests.

## Corridor and Caps (Locked)

- `R_ACQ=250m`
- `R_COR=10m`
- `R_PRE=2m`
- `R_LATCH=0.5m`

Speed hard caps:
- at `R_COR`: `0.35 m/s`
- at `R_PRE`: `0.20 m/s`
- at `R_LATCH`: `0.10 m/s`
