"""Run autonomous planning layer for 2x dispersion scenario."""

from planner import plan_release_and_tof
from sim.scenario import load_config


def main():
    cfg = load_config("configs/dispersion_recovery.yaml")
    baseline_ok = (
        float(cfg["simulation"]["t_end_s"]) == 7500.0
        and float(cfg["targeting"]["cor_entry_lead_s"]) == 300.0
        and float(cfg["guidance"]["cw_speed_weight"]) == 0.0
        and float(cfg["guidance"]["r_cor_speed_gain"]) == 0.5
    )

    plan = plan_release_and_tof(
        config_path="configs/dispersion_recovery.yaml",
        dispersion_scale=2.0,
        seed=123,
        budget_evals=0,
    )

    full = plan["full_metrics"]
    print("Planner result")
    print(f"chosen TOF: {plan['chosen_t_end_s']:.0f} s")
    print(f"chosen cor_entry_lead_s: {plan['chosen_cor_entry_lead_s']:.0f} s")
    print(f"speed-safety mode engaged: {plan['speed_safety_mode_enabled']}")
    print(
        "full 2x: "
        f"P(R_COR)={full['p_r_cor']:.3f}, "
        f"P(R_LATCH)={full['p_r_latch']:.3f}, "
        f"R_COR p95/max={full['r_cor_speed_p95_mps']:.3f}/{full['r_cor_speed_max_mps']:.3f}, "
        f"R_COR violations={full['viol_r_cor_count']}, "
        f"dv_tag p95/max={full['dv_tag_p95_mps']:.3f}/{full['dv_tag_max_mps']:.3f}"
    )
    print(f"baseline config unchanged: {baseline_ok}")


if __name__ == "__main__":
    main()
