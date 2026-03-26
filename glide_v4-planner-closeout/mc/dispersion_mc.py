"""Monte Carlo for dispersion recovery."""

from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt

from sim.scenario import load_config, build_reference_state, get_initial_relative_state
from sim.runner import run_sim_config, apply_release_targeting
from mc.sampler import sample_config


def _hist_plot(values, bins, title, xlabel, out_path):
    plt.figure(figsize=(8, 5))
    plt.hist(values, bins=bins, alpha=0.8)
    plt.title(title)
    plt.xlabel(xlabel)
    plt.ylabel("Count")
    plt.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig(out_path, dpi=150)
    plt.close()


def _failure_reason(gates, terminal_enabled=False):
    for v in gates.violations:
        if v.get("type") == "speed_violation":
            gate = v.get("gate", "unknown")
            return f"speed_violation_at_{gate}"
    if "R_ACQ" not in gates.crossings:
        return "no_R_ACQ"
    if "R_COR" not in gates.crossings:
        return "no_R_COR"
    if terminal_enabled:
        if "R_PRE" not in gates.crossings:
            return "no_R_PRE"
        if "R_LATCH" not in gates.crossings:
            return "no_R_LATCH"
    return "unknown"


def run_dispersion_mc(
    config_path,
    runs=500,
    seed=123,
    output_dir="outputs/dispersion_recovery_mc",
    terminal_enabled=False,
    dt_override=None,
):
    cfg = load_config(config_path)
    rng = np.random.default_rng(seed)

    out_dir = Path(output_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    mc_cfg = cfg.get("mc", {})
    mc_dt = dt_override if dt_override is not None else mc_cfg.get("dt_s", None)

    count_acq = 0
    count_cor = 0
    count_pre = 0
    count_latch = 0
    speeds_cor = []
    max_speed = {"R_ACQ": 0.0, "R_COR": 0.0, "R_PRE": 0.0, "R_LATCH": 0.0}
    dv_eqs = []
    fail_counts = {}

    for _ in range(runs):
        cfg_s = sample_config(cfg, rng)
        cfg_s["simulation"]["save_plots"] = False
        if mc_dt is not None:
            cfg_s["simulation"]["dt_s"] = float(mc_dt)
        if terminal_enabled:
            cfg_s["simulation"]["stop_on_gate"] = None
        else:
            cfg_s["simulation"]["stop_on_gate"] = "R_COR"
        cfg_s["terminal_capture"]["enabled"] = bool(terminal_enabled)

        r_ref, v_ref = build_reference_state(cfg_s)
        r0, v0 = get_initial_relative_state(cfg_s)
        r_nom, v_nom, _ = apply_release_targeting(cfg_s, r_ref, v_ref, r0, v0)

        disp_cfg = cfg_s.get("dispersion", {})
        r_bounds = np.array(disp_cfg.get("r_m", [0.0, 0.0, 0.0]), dtype=float)
        v_bounds = np.array(disp_cfg.get("v_mps", [0.0, 0.0, 0.0]), dtype=float)

        r_disp = rng.uniform(-r_bounds, r_bounds)
        v_disp = rng.uniform(-v_bounds, v_bounds)

        r_act = r_nom + r_disp
        v_act = v_nom + v_disp

        metrics, gates, _ = run_sim_config(
            cfg_s,
            control_enabled=True,
            guidance_enabled=True,
            r_rel_lvh0=r_act,
            v_rel_lvh0=v_act,
            r_rel_lvh_nom0=r_nom,
            v_rel_lvh_nom0=v_nom,
            fast_mode=True,
        )

        dv_eqs.append(metrics["dv_eq_applied"])

        if "R_ACQ" in gates.crossings:
            count_acq += 1
            max_speed["R_ACQ"] = max(max_speed["R_ACQ"], gates.crossings["R_ACQ"]["speed_mps"])
        if "R_COR" in gates.crossings:
            count_cor += 1
            speeds_cor.append(gates.crossings["R_COR"]["speed_mps"])
            max_speed["R_COR"] = max(max_speed["R_COR"], gates.crossings["R_COR"]["speed_mps"])
        if "R_PRE" in gates.crossings:
            count_pre += 1
            max_speed["R_PRE"] = max(max_speed["R_PRE"], gates.crossings["R_PRE"]["speed_mps"])
        if "R_LATCH" in gates.crossings:
            count_latch += 1
            max_speed["R_LATCH"] = max(max_speed["R_LATCH"], gates.crossings["R_LATCH"]["speed_mps"])

        if terminal_enabled:
            failed = not metrics["latch_success"]
        else:
            failed = not metrics["corridor_entry_success"]

        if failed:
            reason = _failure_reason(gates, terminal_enabled=terminal_enabled)
            fail_counts[reason] = fail_counts.get(reason, 0) + 1

    p_acq = count_acq / max(runs, 1)
    p_cor = count_cor / max(runs, 1)
    p_pre = count_pre / max(runs, 1)
    p_latch = count_latch / max(runs, 1)

    print(f"MC runs: {runs}")
    print(f"P(R_ACQ): {p_acq:.3f}")
    print(f"P(R_COR): {p_cor:.3f}")
    if terminal_enabled:
        print(f"P(R_PRE): {p_pre:.3f}")
        print(f"P(R_LATCH): {p_latch:.3f}")

    print("Max speed at gates (m/s):")
    print(f"  R_ACQ: {max_speed['R_ACQ']:.3f}")
    print(f"  R_COR: {max_speed['R_COR']:.3f}")
    print(f"  R_PRE: {max_speed['R_PRE']:.3f}")
    print(f"  R_LATCH: {max_speed['R_LATCH']:.3f}")

    if speeds_cor:
        speeds = np.array(speeds_cor)
        print("R_COR speed stats (m/s):")
        print(f"  min={speeds.min():.3f} mean={speeds.mean():.3f} median={np.median(speeds):.3f} max={speeds.max():.3f}")
        _hist_plot(speeds, bins=30, title="R_COR Entry Speed", xlabel="Speed (m/s)", out_path=out_dir / "r_cor_speed_hist.png")
    else:
        print("No R_COR crossings; speed distribution empty.")

    if dv_eqs:
        dv = np.array(dv_eqs)
        print("dv_eq_applied stats (m/s):")
        print(f"  min={dv.min():.4f} mean={dv.mean():.4f} median={np.median(dv):.4f} max={dv.max():.4f}")
        _hist_plot(dv, bins=30, title="dv_eq_applied", xlabel="dv_eq_applied (m/s)", out_path=out_dir / "dv_eq_applied_hist.png")

    if fail_counts:
        print("Failure breakdown:")
        total_fail = sum(fail_counts.values())
        for reason, count in sorted(fail_counts.items(), key=lambda x: x[1], reverse=True):
            pct = 100.0 * count / max(total_fail, 1)
            print(f"  {reason}: {count} ({pct:.1f}%)")


def run_terminal_sweep(config_path, accel_list, runs=200, dt=5.0, seed=123):
    cfg = load_config(config_path)
    rng = np.random.default_rng(seed)

    for accel in accel_list:
        count_cor = 0
        count_pre = 0
        count_latch = 0
        max_speed = {"R_COR": 0.0, "R_PRE": 0.0, "R_LATCH": 0.0}
        dv_tag = []
        dv_node = []

        for _ in range(runs):
            cfg_s = sample_config(cfg, rng)
            cfg_s["simulation"]["save_plots"] = False
            cfg_s["simulation"]["dt_s"] = float(dt)
            cfg_s["simulation"]["stop_on_gate"] = None
            cfg_s["terminal_capture"]["enabled"] = True
            cfg_s["terminal_capture"]["max_accel"] = float(accel)

            r_ref, v_ref = build_reference_state(cfg_s)
            r0, v0 = get_initial_relative_state(cfg_s)
            r_nom, v_nom, _ = apply_release_targeting(cfg_s, r_ref, v_ref, r0, v0)

            disp_cfg = cfg_s.get("dispersion", {})
            r_bounds = np.array(disp_cfg.get("r_m", [0.0, 0.0, 0.0]), dtype=float)
            v_bounds = np.array(disp_cfg.get("v_mps", [0.0, 0.0, 0.0]), dtype=float)
            r_disp = rng.uniform(-r_bounds, r_bounds)
            v_disp = rng.uniform(-v_bounds, v_bounds)
            r_act = r_nom + r_disp
            v_act = v_nom + v_disp

            metrics, gates, _ = run_sim_config(
                cfg_s,
                control_enabled=True,
                guidance_enabled=True,
                r_rel_lvh0=r_act,
                v_rel_lvh0=v_act,
                r_rel_lvh_nom0=r_nom,
                v_rel_lvh_nom0=v_nom,
                fast_mode=True,
            )

            dv_tag.append(metrics.get("dv_tag", 0.0))
            dv_node.append(metrics.get("dv_node_terminal", 0.0))

            if "R_COR" in gates.crossings:
                count_cor += 1
                max_speed["R_COR"] = max(max_speed["R_COR"], gates.crossings["R_COR"]["speed_mps"])
            if "R_PRE" in gates.crossings:
                count_pre += 1
                max_speed["R_PRE"] = max(max_speed["R_PRE"], gates.crossings["R_PRE"]["speed_mps"])
            if "R_LATCH" in gates.crossings:
                count_latch += 1
                max_speed["R_LATCH"] = max(max_speed["R_LATCH"], gates.crossings["R_LATCH"]["speed_mps"])

        p_cor = count_cor / max(runs, 1)
        p_pre = count_pre / max(runs, 1)
        p_latch = count_latch / max(runs, 1)

        dv_tag_arr = np.array(dv_tag)
        dv_node_arr = np.array(dv_node)

        print(f"\nTerminal sweep max_accel={accel:.3f} m/s^2")
        print(f"P(R_COR)={p_cor:.3f} P(R_PRE)={p_pre:.3f} P(R_LATCH)={p_latch:.3f}")
        print("Max speeds (m/s):")
        print(f"  R_COR: {max_speed['R_COR']:.3f}")
        print(f"  R_PRE: {max_speed['R_PRE']:.3f}")
        print(f"  R_LATCH: {max_speed['R_LATCH']:.3f}")
        print(
            "dv_tag (m/s): "
            f"mean={dv_tag_arr.mean():.4f} "
            f"median={np.median(dv_tag_arr):.4f} "
            f"max={dv_tag_arr.max():.4f}"
        )
        print(
            "dv_node_terminal (m/s): "
            f"mean={dv_node_arr.mean():.4f} "
            f"median={np.median(dv_node_arr):.4f} "
            f"p95={np.percentile(dv_node_arr,95):.4f} "
            f"max={dv_node_arr.max():.4f}"
        )


def run_terminal_binding_check(config_path, accel_list, runs=20, dt=5.0, seed=123):
    cfg = load_config(config_path)
    rng = np.random.default_rng(seed)

    for accel in accel_list:
        total_term_steps = 0
        total_term_sat = 0
        max_cmd_raw = 0.0
        max_applied = 0.0
        latch_dts = []

        for _ in range(runs):
            cfg_s = sample_config(cfg, rng)
            cfg_s["simulation"]["save_plots"] = False
            cfg_s["simulation"]["dt_s"] = float(dt)
            cfg_s["simulation"]["stop_on_gate"] = None
            cfg_s["terminal_capture"]["enabled"] = True
            cfg_s["terminal_capture"]["max_accel"] = float(accel)

            r_ref, v_ref = build_reference_state(cfg_s)
            r0, v0 = get_initial_relative_state(cfg_s)
            r_nom, v_nom, _ = apply_release_targeting(cfg_s, r_ref, v_ref, r0, v0)

            disp_cfg = cfg_s.get("dispersion", {})
            r_bounds = np.array(disp_cfg.get("r_m", [0.0, 0.0, 0.0]), dtype=float)
            v_bounds = np.array(disp_cfg.get("v_mps", [0.0, 0.0, 0.0]), dtype=float)
            r_disp = rng.uniform(-r_bounds, r_bounds)
            v_disp = rng.uniform(-v_bounds, v_bounds)
            r_act = r_nom + r_disp
            v_act = v_nom + v_disp

            metrics, gates, _ = run_sim_config(
                cfg_s,
                control_enabled=True,
                guidance_enabled=True,
                r_rel_lvh0=r_act,
                v_rel_lvh0=v_act,
                r_rel_lvh_nom0=r_nom,
                v_rel_lvh_nom0=v_nom,
                fast_mode=True,
            )

            total_term_steps += int(metrics.get("term_steps", 0))
            total_term_sat += int(metrics.get("term_sat_steps", 0))
            max_cmd_raw = max(max_cmd_raw, float(metrics.get("max_cmd_raw_term", 0.0)))
            max_applied = max(max_applied, float(metrics.get("max_applied_term", 0.0)))

            if "R_COR" in gates.crossings and "R_LATCH" in gates.crossings:
                latch_dts.append(
                    gates.crossings["R_LATCH"]["t"] - gates.crossings["R_COR"]["t"]
                )

        sat_pct = 100.0 * total_term_sat / max(total_term_steps, 1)
        avg_dt = float(np.mean(latch_dts)) if latch_dts else None
        clamp_ok = max_applied <= (float(accel) + 1e-6)

        print(f"\nTerminal binding check max_accel={accel:.3f} m/s^2 (runs={runs})")
        print(f"  max(|a_cmd_raw|) = {max_cmd_raw:.4f} m/s^2")
        print(f"  max(|a_applied|) = {max_applied:.4f} m/s^2")
        print(f"  terminal saturation = {sat_pct:.1f}% of terminal steps")
        if avg_dt is None:
            print("  avg time R_COR->R_LATCH: N/A")
        else:
            print(f"  avg time R_COR->R_LATCH = {avg_dt:.1f} s")
        print(f"  clamp respected: {clamp_ok}")


def _run_terminal_stats(cfg, rng, runs, dt, extra_time_s=None, disp_scale=1.0):
    base_r = np.array(cfg.get("dispersion", {}).get("r_m", [0.0, 0.0, 0.0]), dtype=float)
    base_v = np.array(cfg.get("dispersion", {}).get("v_mps", [0.0, 0.0, 0.0]), dtype=float)

    count_cor = 0
    count_pre = 0
    count_latch = 0
    max_speed = {"R_COR": 0.0, "R_PRE": 0.0, "R_LATCH": 0.0}
    dv_tag = []
    dv_node = []
    latch_dts = []

    for _ in range(runs):
        cfg_s = sample_config(cfg, rng)
        cfg_s["simulation"]["save_plots"] = False
        cfg_s["simulation"]["dt_s"] = float(dt)
        cfg_s["simulation"]["stop_on_gate"] = None
        cfg_s["terminal_capture"]["enabled"] = True
        if extra_time_s is not None:
            cfg_s["terminal_capture"]["extra_time_s"] = float(extra_time_s)

        if "dispersion" not in cfg_s:
            cfg_s["dispersion"] = {}
        cfg_s["dispersion"]["r_m"] = (base_r * disp_scale).tolist()
        cfg_s["dispersion"]["v_mps"] = (base_v * disp_scale).tolist()

        r_ref, v_ref = build_reference_state(cfg_s)
        r0, v0 = get_initial_relative_state(cfg_s)
        r_nom, v_nom, _ = apply_release_targeting(cfg_s, r_ref, v_ref, r0, v0)

        r_bounds = np.array(cfg_s["dispersion"].get("r_m", [0.0, 0.0, 0.0]), dtype=float)
        v_bounds = np.array(cfg_s["dispersion"].get("v_mps", [0.0, 0.0, 0.0]), dtype=float)
        r_disp = rng.uniform(-r_bounds, r_bounds)
        v_disp = rng.uniform(-v_bounds, v_bounds)
        r_act = r_nom + r_disp
        v_act = v_nom + v_disp

        metrics, gates, _ = run_sim_config(
            cfg_s,
            control_enabled=True,
            guidance_enabled=True,
            r_rel_lvh0=r_act,
            v_rel_lvh0=v_act,
            r_rel_lvh_nom0=r_nom,
            v_rel_lvh_nom0=v_nom,
            fast_mode=True,
        )

        dv_tag.append(metrics.get("dv_tag", 0.0))
        dv_node.append(metrics.get("dv_node_terminal", 0.0))

        if "R_COR" in gates.crossings:
            count_cor += 1
            max_speed["R_COR"] = max(max_speed["R_COR"], gates.crossings["R_COR"]["speed_mps"])
        if "R_PRE" in gates.crossings:
            count_pre += 1
            max_speed["R_PRE"] = max(max_speed["R_PRE"], gates.crossings["R_PRE"]["speed_mps"])
        if "R_LATCH" in gates.crossings:
            count_latch += 1
            max_speed["R_LATCH"] = max(max_speed["R_LATCH"], gates.crossings["R_LATCH"]["speed_mps"])

        if "R_COR" in gates.crossings and "R_LATCH" in gates.crossings:
            latch_dts.append(gates.crossings["R_LATCH"]["t"] - gates.crossings["R_COR"]["t"])

    p_cor = count_cor / max(runs, 1)
    p_pre = count_pre / max(runs, 1)
    p_latch = count_latch / max(runs, 1)

    dv_tag_arr = np.array(dv_tag) if dv_tag else np.array([])
    dv_node_arr = np.array(dv_node) if dv_node else np.array([])

    stats = {
        "p_cor": p_cor,
        "p_pre": p_pre,
        "p_latch": p_latch,
        "max_speed": max_speed,
        "dv_tag_median": float(np.median(dv_tag_arr)) if dv_tag_arr.size else None,
        "dv_tag_p95": float(np.percentile(dv_tag_arr, 95)) if dv_tag_arr.size else None,
        "dv_node_median": float(np.median(dv_node_arr)) if dv_node_arr.size else None,
        "dv_node_p95": float(np.percentile(dv_node_arr, 95)) if dv_node_arr.size else None,
        "mean_latch_dt": float(np.mean(latch_dts)) if latch_dts else None,
    }
    return stats


def run_extra_time_sweep(config_path, extra_time_list, runs=200, dt=5.0, seed=123):
    cfg = load_config(config_path)
    rng = np.random.default_rng(seed)

    for extra_time in extra_time_list:
        stats = _run_terminal_stats(cfg, rng, runs, dt, extra_time_s=extra_time, disp_scale=1.0)
        print(f"\nextra_time_s={extra_time:.1f} (runs={runs}, dt={dt})")
        print(f"P(R_COR)={stats['p_cor']:.3f} P(R_PRE)={stats['p_pre']:.3f} P(R_LATCH)={stats['p_latch']:.3f}")
        print(
            "Max speeds (m/s): "
            f"R_COR={stats['max_speed']['R_COR']:.3f} "
            f"R_PRE={stats['max_speed']['R_PRE']:.3f} "
            f"R_LATCH={stats['max_speed']['R_LATCH']:.3f}"
        )
        if stats["dv_tag_median"] is not None:
            print(f"dv_tag median={stats['dv_tag_median']:.4f} p95={stats['dv_tag_p95']:.4f}")
        else:
            print("dv_tag median/p95: N/A")
        if stats["dv_node_median"] is not None:
            print(f"dv_node median={stats['dv_node_median']:.4f} p95={stats['dv_node_p95']:.4f}")
        else:
            print("dv_node median/p95: N/A")
        if stats["mean_latch_dt"] is None:
            print("mean time R_COR->R_LATCH: N/A")
        else:
            print(f"mean time R_COR->R_LATCH: {stats['mean_latch_dt']:.1f} s")


def run_dispersion_scale_sweep(config_path, scales, runs=200, dt=5.0, seed=123):
    cfg = load_config(config_path)
    rng = np.random.default_rng(seed)

    for scale in scales:
        stats = _run_terminal_stats(cfg, rng, runs, dt, extra_time_s=None, disp_scale=scale)
        print(f"\nDispersion scale={scale:.1f}x (runs={runs}, dt={dt})")
        print(f"P(R_COR)={stats['p_cor']:.3f} P(R_PRE)={stats['p_pre']:.3f} P(R_LATCH)={stats['p_latch']:.3f}")
        print(
            "Max speeds (m/s): "
            f"R_COR={stats['max_speed']['R_COR']:.3f} "
            f"R_PRE={stats['max_speed']['R_PRE']:.3f} "
            f"R_LATCH={stats['max_speed']['R_LATCH']:.3f}"
        )
        if stats["dv_tag_median"] is not None:
            print(f"dv_tag median={stats['dv_tag_median']:.4f} p95={stats['dv_tag_p95']:.4f}")
        else:
            print("dv_tag median/p95: N/A")
        if stats["dv_node_median"] is not None:
            print(f"dv_node median={stats['dv_node_median']:.4f} p95={stats['dv_node_p95']:.4f}")
        else:
            print("dv_node median/p95: N/A")
        if stats["mean_latch_dt"] is None:
            print("mean time R_COR->R_LATCH: N/A")
        else:
            print(f"mean time R_COR->R_LATCH: {stats['mean_latch_dt']:.1f} s")


if __name__ == "__main__":
    run_dispersion_mc("configs/dispersion_recovery.yaml", runs=500, seed=123)
