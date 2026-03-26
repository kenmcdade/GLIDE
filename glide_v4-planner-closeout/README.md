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

## Canonical Regression Harness

Run:

```powershell
.\.venv\Scripts\python run_benchmarks.py
```

Authoritative machine-readable output:
- `benchmarks.json`

Mirror copy:
- `outputs/validated/benchmarks.json`

These files are the frozen regression reference. Post-benchmark refinement results are documented separately and do not replace them.

## Current Best Operating Plan

Frozen benchmark reference:
- baseline config remains `t_end_s=7500`, `cor_entry_lead_s=300`, `cw_speed_weight=0.0`, `r_cor_speed_gain=0.5`
- canonical `2x` planner benchmark reference is `TOF=7100 s`, `lead_s=300`, `safe_k3`, `P(R_LATCH)=0.690`, `R_COR` violations=`0`

Current best compliant `2x` planner result:
- `TOF=7100 s`
- `lead_s=280 s`
- safety mode on
- `P(R_COR)=0.728`
- `P(R_LATCH)=0.728`
- `R_COR` hard-cap violations=`0`

What changed:
- the improvement came from planner-side local refinement around the coarse winning policy
- tested post-`R_COR` terminal shaping degraded performance and was not adopted

Current best refinement artifacts:
- `outputs/planner_closeout_report.md`
- `outputs/planner_closeout_candidates.csv`
- `outputs/post_benchmark_refinement_index.md`

## Current Status

- Baseline (`1x`) is strong and compliant.
- The frozen benchmark `2x` reference is still the official comparison point.
- The closeout refinement pass reached the compliant `2x` latch target without disturbing the frozen `1x` baseline.
- Remaining work is handoff and review oriented, not major retuning.

See:
- `docs/STATUS.md`
- `benchmarks.json`
- `outputs/planner_closeout_report.md`

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

All tests:

```powershell
.\.venv\Scripts\python -m unittest discover -s tests -v
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
