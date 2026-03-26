# Baseline Failure Mode Characterization

## Baseline Campaign
- Config: baseline 1x, terminal ON, dt=5s
- Trials: 1000, seed=123
- P(R_COR)=0.913, P(R_LATCH)=0.913, failures=87

## Top 3 Failure Causes
1. no_R_COR_moderate_coupled_dispersion: count=37 (42.5% of failures)
2. no_R_COR_large_alongtrack_velocity_dispersion: count=36 (41.4% of failures)
3. no_R_COR_large_alongtrack_position_dispersion: count=14 (16.1% of failures)

## One-Lever Change
- Lever: `simulation.t_end_s` only (slightly longer TOF)
- Selected value: `7500.0 s` (lead_s policy kept at 300 s)
- Validation: baseline 1x, terminal ON, dt=5s, N=500, seed=123
- New P(R_COR)=0.964, P(R_LATCH)=0.964
- R_COR speed mean/p95/max = 0.190/0.281/0.316 m/s
- Gate speed violations (R_COR/R_PRE/R_LATCH) = 0/0/0

## Artifacts
- `outputs/baseline_failed_trials_features.csv`
- `outputs/baseline_failure_breakdown_top3.csv`
- `outputs/baseline_failure_rate_vs_dispersion.csv`
- `outputs/failure_rate_vs_disp_r.png`
- `outputs/failure_rate_vs_disp_v.png`
- `outputs/baseline_tuning_pilot_tof.csv`
- `outputs/baseline_tuned_n500.csv`
