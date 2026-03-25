# GLIDE V4 Validation Summary

## Baseline Tuning
- lead_s = 300
- cw_speed_weight = 0.0
- r_cor_speed_gain = 0.5
- corridor radii and official speed caps unchanged

## Baseline Robustness (1x dispersion, terminal ON, dt=5s, N=1000)

| Seed | P(R_ACQ) | P(R_COR) | P(R_PRE) | P(R_LATCH) | R_COR mean | R_COR p95 | R_COR max | Viol R_COR | Viol R_PRE | Viol R_LATCH | dv_tag med/p95/max | dv_node med/p95/max | t(R_COR->R_LATCH) mean/p95 |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---|---|---|
| 123 | 1.000 | 0.974 | 0.974 | 0.974 | 0.190 | 0.285 | 0.327 | 0 | 0 | 0 | 0.153/0.343/0.735 | 0.286/0.378/0.418 | 68.383/75.000 |
| 456 | 1.000 | 0.964 | 0.964 | 0.964 | 0.189 | 0.282 | 0.319 | 0 | 0 | 0 | 0.154/0.338/0.766 | 0.284/0.376/0.424 | 68.646/75.000 |
| 789 | 1.000 | 0.986 | 0.986 | 0.986 | 0.192 | 0.285 | 0.323 | 0 | 0 | 0 | 0.163/0.343/0.699 | 0.291/0.383/0.437 | 68.555/75.000 |

Worst-case across baseline seeds:
- Worst P(R_COR): 0.964 at seed 456
- Worst P(R_LATCH): 0.964 at seed 456

## Stress Test (2x dispersion, terminal ON, dt=5s, N=500, seed=123)

- P(R_ACQ)=1.000, P(R_COR)=0.746, P(R_PRE)=0.746, P(R_LATCH)=0.746
- R_COR speed mean/p95/max = 0.234/0.372/0.395 m/s
- Gate speed violations counts (R_COR/R_PRE/R_LATCH) = 35/0/0
- dv_tag med/p95/max = 0.351/1.139/2.522 m/s
- dv_node_terminal med/p95/max = 0.287/0.447/0.489 m/s
- t(R_COR->R_LATCH) mean/p95 = 67.225/70.000 s
- Any R_COR hard cap (0.35 m/s) violation: True

## Artifacts
- `outputs/mc_baseline_seeds.csv`
- `outputs/mc_2x.csv`
- `outputs/GLIDE_V4_validation_summary.md`