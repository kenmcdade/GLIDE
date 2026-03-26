# Project Status

## Authoritative Reference

Canonical benchmark harness:
- `run_benchmarks.py`
- frozen machine-readable reference: `benchmarks.json`
- frozen mirror copy: `outputs/validated/benchmarks.json`

These benchmark artifacts remain the regression reference and were not overwritten by the closeout refinement pass.

## Frozen Baseline Reference

From `configs/dispersion_recovery.yaml`:
- `simulation.t_end_s = 7500`
- `targeting.cor_entry_lead_s = 300`
- `guidance.cw_speed_weight = 0.0`
- `guidance.r_cor_speed_gain = 0.5`

Baseline `1x` benchmark (`dt=5s`, terminal ON, `N=1000`, seeds `123/456/789`):
- `P(R_LATCH) = 0.964` to `0.986`
- total `R_COR` hard-cap violations: `0`
- worst `R_COR` max speed: `0.327 m/s`

Source:
- `benchmarks.json`

## Current Best Operating Plan

Current best compliant `2x` planner result is a post-benchmark refinement result, not a replacement for the frozen benchmark reference.

Winning settings:
- `TOF = 7100 s`
- `lead_s = 280 s`
- safety mode: `on`
- planner policy family: `safe_k3`

Current best `2x` compliant result:
- `P(R_COR) = 0.728`
- `P(R_PRE) = 0.728`
- `P(R_LATCH) = 0.728`
- `R_COR` speed `p95/max = 0.236/0.264 m/s`
- `R_COR` hard-cap violations: `0`

Key conclusion:
- the gain came from planner-side local refinement around the coarse winning policy
- tested post-`R_COR` terminal shaping degraded performance and was not adopted

Source:
- `outputs/planner_closeout_report.md`
- `outputs/planner_closeout_candidates.csv`

## Benchmark Snapshot

Frozen benchmark `2x` planner reference:
- `TOF = 7100 s`
- `lead_s = 300 s`
- policy `safe_k3`
- `P(R_COR) = 0.692`
- `P(R_LATCH) = 0.690`
- `R_COR` hard-cap violations: `0`

This remains the canonical benchmark comparison point.

## Regression Tests

Metric regression checks:
- `tests/test_metric_regression.py`

Current guardrails:
- frozen `1x` baseline keeps zero gate violations
- planner `2x` small-N regression keeps `R_COR` cap compliance

Run with:
- `.\.venv\Scripts\python.exe -m unittest discover -s tests -v`

## Remaining Open Work

Current open items are consolidation-oriented rather than tuning-oriented:
- keep benchmark and refinement outputs clearly separated
- preserve the frozen baseline and canonical benchmark reference
- use the refined `2x` operating plan for discussion, review, and planner follow-on work
