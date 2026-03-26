# GLIDE V4 Rendezvous Simulation

This repo is a proof-of-concept rendezvous and capture simulation for GLIDE V4. It is built to answer a specific question: can a dispersed release be guided through fixed corridor gates and terminal capture without violating hard speed caps, using simplified but parameterized DDM drag, mEDT control, and node-side terminal control.

## What The Sim Covers

- orbital propagation with `2-body + J2 + drag`
- LVLH relative-motion guidance and CW-based release targeting-lite
- fixed corridor gate logic at `R_ACQ`, `R_COR`, `R_PRE`, and `R_LATCH`
- terminal capture control after corridor entry
- Monte Carlo robustness analysis
- autonomous planner-side search over `TOF` and corridor-entry timing

## Locked Corridor And Caps

Gate radii:
- `R_ACQ = 250 m`
- `R_COR = 10 m`
- `R_PRE = 2 m`
- `R_LATCH = 0.5 m`

Hard speed caps:
- at `R_COR`: `0.35 m/s`
- at `R_PRE`: `0.20 m/s`
- at `R_LATCH`: `0.10 m/s`

## Frozen Baseline Reference

Frozen baseline config in [configs/dispersion_recovery.yaml](C:/Users/kenny/Desktop/glide_v4/configs/dispersion_recovery.yaml):
- `simulation.t_end_s = 7500`
- `targeting.cor_entry_lead_s = 300`
- `guidance.cw_speed_weight = 0.0`
- `guidance.r_cor_speed_gain = 0.5`

Frozen `1x` benchmark result:
- `P(R_LATCH) = 0.964` to `0.986` across seeds `123/456/789`
- total `R_COR` hard-cap violations: `0`
- worst `R_COR` max speed: `0.327 m/s`

This is the `96-99%` Monte Carlo result range.

Authoritative frozen benchmark artifacts:
- [benchmarks.json](C:/Users/kenny/Desktop/glide_v4/benchmarks.json)
- [outputs/validated/benchmarks.json](C:/Users/kenny/Desktop/glide_v4/outputs/validated/benchmarks.json)

## Current Best Operating Plan

The frozen benchmark reference is still the regression baseline. The best current compliant `2x` result is a separate post-benchmark refinement result.

Current best compliant `2x` plan:
- `TOF = 7100 s`
- `lead_s = 280 s`
- safety mode: `on`
- planner policy family: `safe_k3`

Current best compliant `2x` result:
- `P(R_COR) = 0.728`
- `P(R_PRE) = 0.728`
- `P(R_LATCH) = 0.728`
- `R_COR` speed `p95/max = 0.236 / 0.264 m/s`
- `R_COR` hard-cap violations: `0`

Why it improved:
- the gain came from planner-side local refinement around the coarse winning policy
- tested post-`R_COR` terminal shaping degraded performance and was not adopted

Post-benchmark refinement artifacts:
- [outputs/planner_closeout_report.md](C:/Users/kenny/Desktop/glide_v4/outputs/planner_closeout_report.md)
- [outputs/planner_closeout_candidates.csv](C:/Users/kenny/Desktop/glide_v4/outputs/planner_closeout_candidates.csv)
- [outputs/post_benchmark_refinement_index.md](C:/Users/kenny/Desktop/glide_v4/outputs/post_benchmark_refinement_index.md)

## Canonical Benchmark Harness

Canonical regression run:

```powershell
.\.venv\Scripts\python run_benchmarks.py
```

This always runs:
- baseline `1x`: `N=1000` for seeds `123`, `456`, `789`
- stress `2x`: `N=500`, seed `123`
- planner `2x` full validate: `N=500`

The outputs from this harness are the frozen regression reference. They should not be treated as the same thing as the later closeout refinement outputs.

## Quick Start

Windows PowerShell:

```powershell
py -m venv .venv
.\.venv\Scripts\python -m pip install -r requirements.txt
.\.venv\Scripts\python run_demo.py configs/dispersion_recovery.yaml
```

## Common Commands

Run the demo:

```powershell
.\.venv\Scripts\python run_demo.py configs/dispersion_recovery.yaml
```

Run the validation suite:

```powershell
.\.venv\Scripts\python run_validation_suite.py
```

Run the planner:

```powershell
.\.venv\Scripts\python run_planner.py
```

Run the planner closeout refinement report:

```powershell
.\.venv\Scripts\python run_planner_closeout.py
```

Run tests:

```powershell
.\.venv\Scripts\python -m unittest discover -s tests -v
```

## Repo Layout

- `dynamics/`: orbital dynamics, frames, drag, mEDT, atmosphere
- `gnc/`: guidance, control allocation, CW targeting
- `modes/`: gate logic and mode/state transitions
- `sim/`: scenario loading, runner, logging, terminal controller, reporting
- `mc/`: Monte Carlo sampling and harness utilities
- `planner/`: planner-side TOF and policy search
- `configs/`: scenario and actuator settings
- `tests/`: unit and lightweight metric regression tests
- `outputs/`: generated plots, reports, benchmark artifacts, and closeout refinement artifacts
- `docs/`: status and handoff documentation

## Current Open Work

The major tuning pass is complete for this phase. What remains open is mostly operational:
- keep frozen benchmark outputs and post-benchmark refinement outputs clearly separated
- preserve the frozen `1x` baseline behavior
- use the current best `2x` compliant operating plan for review, handoff, and external discussion

For the current authoritative status snapshot, see [docs/STATUS.md](C:/Users/kenny/Desktop/glide_v4/docs/STATUS.md).
