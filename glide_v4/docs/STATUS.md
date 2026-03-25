# Project Status

## Scope

GLIDE V4 phased rendezvous/capture simulation with:
- 2-body + J2 + drag propagation
- LVLH guidance and CW-based targeting-lite
- gate-based corridor capture checks
- terminal capture controller
- Monte Carlo and planner layer

## Frozen Baseline

From `configs/dispersion_recovery.yaml`:
- `simulation.t_end_s = 7500`
- `targeting.cor_entry_lead_s = 300`
- `guidance.cw_speed_weight = 0.0`
- `guidance.r_cor_speed_gain = 0.5`

## Validated Performance

Baseline robustness (`1x`, terminal ON, `dt=5s`, `N=1000`):
- `P(R_LATCH) = 0.964` to `0.986` across seeds `123/456/789`
- zero gate speed violations

Source:
- `outputs/mc_baseline_seeds.csv`
- `outputs/GLIDE_V4_validation_summary.md`

## Current Gap

Stress case (`2x`) still trades off between:
- strict `R_COR` hard-cap compliance
- and `P(R_LATCH)` target level

Planner currently enforces conservative pilot rejection buffers and can find compliant plans, but latest full-validate run reached:
- `R_COR violations = 0`
- `P(R_LATCH) = 0.666`

Source:
- `outputs/planner_candidates.csv`
- `outputs/planner_2x_report.md`

## Next Engineering Step

Keep dynamics fixed, continue planner-side policy search:
- jointly tune TOF + conservative speed-safety profile policy selection
- maintain hard-cap compliance
- recover `P(R_LATCH) >= 0.70` at `2x`
