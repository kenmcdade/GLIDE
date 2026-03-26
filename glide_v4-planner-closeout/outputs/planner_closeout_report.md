# Post-Benchmark Planner Closeout Refinement

## Reference
- canonical planner reference: TOF=7100, lead_s=300, policy=safe_k3
- reference full result: P(R_COR)=0.692, P(R_LATCH)=0.690, R_COR p95/max=0.229/0.255, violations=0
- this benchmark reference remains frozen and separate from the refinement result below

## Best Planner-Side Candidate
- planner_lead_280: TOF=7100, lead_s=280, policy=safe_k3
- full P(R_COR)=0.728, P(R_PRE)=0.728, P(R_LATCH)=0.728
- R_COR p95/max=0.236/0.264 m/s, violations=0
- dv_tag p95/max=1.349/2.151 m/s, dv_node p95/max=0.293/0.424 m/s

## Best Terminal-Side Candidate
- terminal_ref: terminal_profile=base
- full P(R_COR)=0.692, P(R_PRE)=0.690, P(R_LATCH)=0.690
- R_COR p95/max=0.229/0.255 m/s, violations=0
- dv_tag p95/max=1.455/2.143 m/s, dv_node p95/max=0.283/0.368 m/s, terminal sat p95=0.000

## Outcome
- achieved P(R_LATCH) >= 0.70: True
- winner: planner candidate `planner_lead_280` with P(R_LATCH)=0.728
- compliance remained perfect at R_COR: True
- change made: reduced corridor-entry lead from 300 s to 280 s at TOF 7100 s; terminal kept at base profile
- terminal shaping did not beat planner-side refinement in this bounded sweep

## Artifacts
- `outputs/planner_closeout_candidates.csv`
- `outputs/planner_closeout_report.md`
