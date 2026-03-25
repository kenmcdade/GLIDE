# Planner 2x Report

## Selected Plan
- chosen TOF (simulation.t_end_s): 6800 s
- chosen targeting cor_entry_lead_s: 300 s
- speed-safety mode engaged: False

## Selection Rationale
- Candidate search: TOF 6500-9500 step 300
- Pilot MC: N=150, dt=5s, dispersion_scale=2x
- Reject rule: pilot R_COR max > 0.33 m/s or pilot R_COR p95 > 0.30 m/s or any R_COR violation
- Objective: maximize pilot P(R_LATCH), tie-break with lower pilot dv_tag p95

## Chosen Candidate Pilot Metrics
- pilot P(R_COR): 0.707
- pilot P(R_LATCH): 0.700
- pilot R_COR speed p95/max: 0.203/0.245 m/s
- pilot R_COR violation count: 0
- pilot dv_tag p95: 1.327 m/s
- pilot rejected by buffers: False

## Full Validation Metrics (2x)
- Full MC: N=500, dt=5s
- full P(R_COR): 0.668
- full P(R_LATCH): 0.666
- full R_COR speed p95/max: 0.202/0.269 m/s
- full R_COR violation count: 0
- full dv_tag p95/max: 1.430/1.815 m/s

## Candidate Count
- evaluated candidates: 11

## Artifacts
- `outputs/planner_candidates.csv`
- `outputs/planner_2x_report.md`