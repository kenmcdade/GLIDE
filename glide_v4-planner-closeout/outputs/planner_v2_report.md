# Planner v2 Report

## Selected Plan
- chosen TOF (simulation.t_end_s): 9500 s
- chosen targeting cor_entry_lead_s: 300 s
- chosen policy: safe_k3
- speed-safety mode engaged: True

## Selection Logic
- Candidate search: TOF 6500-9500 step 300
- Policies per TOF: ['off', 'safe_k3', 'safe_k5']
- Pilot MC: runs/seed=6, seeds=2, dt=10s, dispersion=2x
- Local refinement after coarse pick: TOF offsets [-100.0, 0.0, 100.0], lead offsets [-20.0, 0.0, 20.0]
- Reject candidate if pilot R_COR max>0.33 or pilot R_COR p95>0.30 or any R_COR violation
- Select by: zero-violation first, then highest compliant LCB95, then lowest dv_tag p95

## Chosen Candidate Pilot Metrics
- pilot P(R_COR): 0.500
- pilot P(R_LATCH): 0.500
- pilot compliant LCB95: 0.254
- pilot R_COR speed p95/max: 0.192/0.200 m/s
- pilot R_COR violation count: 0
- pilot dv_tag p95: 1.580 m/s

## Full Validation Metrics (2x)
- Full MC: N=60, dt=5s
- full P(R_COR): 0.550
- full P(R_LATCH): 0.533
- full R_COR speed p95/max: 0.210/0.253 m/s
- full R_COR violation count: 0
- full dv_tag p95/max: 2.288/2.759 m/s

## Candidate Count
- evaluated candidates: 9

## Artifacts
- `outputs/planner_candidates.csv`
- `outputs/planner_v2_report.md`