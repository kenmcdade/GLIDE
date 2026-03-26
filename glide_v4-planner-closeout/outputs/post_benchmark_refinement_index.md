# Post-Benchmark Refinement Artifacts

These files are separate from the frozen benchmark reference and document the closeout refinement pass only.

## Frozen Benchmark Reference

Do not treat these as closeout-tuned outputs:
- `benchmarks.json`
- `outputs/validated/benchmarks.json`
- `outputs/planner_v2_report.md`
- `outputs/planner_candidates.csv`

## Post-Benchmark Closeout Outputs

Current best compliant `2x` refinement artifacts:
- `outputs/planner_closeout_report.md`
- `outputs/planner_closeout_candidates.csv`

Summary of the winning refinement:
- `TOF=7100 s`
- `lead_s=280 s`
- safety mode on
- `P(R_COR)=0.728`
- `P(R_LATCH)=0.728`
- `R_COR` hard-cap violations=`0`

Key conclusion:
- improvement came from planner-side local refinement
- tested post-`R_COR` terminal shaping degraded performance and was not adopted
