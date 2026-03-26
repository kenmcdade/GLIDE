"""Lightweight metric-based regression checks for CI."""

from __future__ import annotations

import unittest

from run_validation_suite import run_campaign
import planner.release_planner as release_planner


class TestMetricRegression(unittest.TestCase):
    def test_baseline_1x_guardrails(self):
        row = run_campaign(
            seed=123,
            runs=80,
            disp_scale=1.0,
            campaign="ci_baseline_1x",
        )
        self.assertEqual(row["viol_r_cor_count"], 0)
        self.assertEqual(row["viol_r_pre_count"], 0)
        self.assertEqual(row["viol_r_latch_count"], 0)
        self.assertGreaterEqual(row["p_r_latch"], 0.90)
        self.assertLessEqual(row["r_cor_speed_max_mps"], 0.35)

    def test_planner_2x_compliance_small(self):
        old = {
            "PILOT_RUNS_PER_SEED": release_planner.PILOT_RUNS_PER_SEED,
            "PILOT_DT_S": release_planner.PILOT_DT_S,
            "PILOT_SEED_OFFSETS": list(release_planner.PILOT_SEED_OFFSETS),
            "FULL_RUNS": release_planner.FULL_RUNS,
            "FULL_DT_S": release_planner.FULL_DT_S,
            "PILOT_MAX_WORKERS": release_planner.PILOT_MAX_WORKERS,
            "FULL_MAX_WORKERS": release_planner.FULL_MAX_WORKERS,
        }
        try:
            # Small-N quick check; full-scale validation stays in run_benchmarks.py.
            release_planner.PILOT_RUNS_PER_SEED = 6
            release_planner.PILOT_DT_S = 10.0
            release_planner.PILOT_SEED_OFFSETS = [0, 211]
            release_planner.FULL_RUNS = 60
            release_planner.FULL_DT_S = 5.0
            release_planner.PILOT_MAX_WORKERS = 1
            release_planner.FULL_MAX_WORKERS = 1

            plan = release_planner.plan_release_and_tof(
                config_path="configs/dispersion_recovery.yaml",
                dispersion_scale=2.0,
                seed=123,
                budget_evals=3,
            )
        finally:
            release_planner.PILOT_RUNS_PER_SEED = old["PILOT_RUNS_PER_SEED"]
            release_planner.PILOT_DT_S = old["PILOT_DT_S"]
            release_planner.PILOT_SEED_OFFSETS = old["PILOT_SEED_OFFSETS"]
            release_planner.FULL_RUNS = old["FULL_RUNS"]
            release_planner.FULL_DT_S = old["FULL_DT_S"]
            release_planner.PILOT_MAX_WORKERS = old["PILOT_MAX_WORKERS"]
            release_planner.FULL_MAX_WORKERS = old["FULL_MAX_WORKERS"]

        full = plan["full_metrics"]
        self.assertGreater(plan["candidate_count"], 0)
        self.assertEqual(full["viol_r_cor_count"], 0)
        self.assertLessEqual(full["r_cor_speed_max_mps"], 0.35)
        self.assertGreaterEqual(full["p_r_latch"], 0.50)


if __name__ == "__main__":
    unittest.main()
