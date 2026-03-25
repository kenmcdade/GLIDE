"""Simulation runner."""

from pathlib import Path
import numpy as np

from dynamics.integrator import rk4_step
from dynamics.orbit import eom
from dynamics.frames import eci_to_lvlh, eci_to_lvlh_vector, lvlh_to_eci_vector
from gnc.guidance import Guidance
from sim.terminal import terminal_capture_command
from gnc.control_allocation import allocate_control
from gnc.estimator import Estimator
from modes.gates import GateTracker
from modes.tag_modes import TagModeManager
from sim.logger import SimLogger
from sim.metrics import compute_metrics
from sim.scenario import (
    load_config,
    build_atmosphere,
    build_reference_state,
    get_initial_relative_state,
    payload_state_from_rel,
)
from gnc.targeting import solve_dv0_minimize_rT, solve_dt_offset_minimize_rT, solve_dv0_minimize_state
from gnc.cw import cw_propagate


def apply_release_targeting(cfg, r_ref, v_ref, r_rel_lvh, v_rel_lvh):
    targeting_cfg = cfg.get("targeting", {})
    enabled = targeting_cfg.get("enabled", False)
    if not enabled:
        return r_rel_lvh, v_rel_lvh, {
            "enabled": False,
            "method": None,
            "r_pred_original": None,
            "r_pred_targeted": None,
        }

    mission_t_end = float(cfg["simulation"]["t_end_s"])
    cor_entry_lead_s = float(targeting_cfg.get("cor_entry_lead_s", 0.0))
    tof = max(1.0, mission_t_end - cor_entry_lead_s)
    mu = cfg.get("constants", {}).get("mu", None)
    if mu is None:
        from dynamics.constants import MU_EARTH
        mu = MU_EARTH
    n = np.sqrt(mu / (np.linalg.norm(r_ref) ** 3))

    method = targeting_cfg.get("method", "dv")
    report = {
        "enabled": True,
        "method": method,
        "r_pred_original": None,
        "r_pred_targeted": None,
        "v_pred_original": None,
        "v_pred_targeted": None,
        "target_time_s": tof,
        "dv0": np.zeros(3),
        "dt": 0.0,
    }

    r_pred_orig, v_pred_orig = cw_propagate(r_rel_lvh, v_rel_lvh, n, tof)
    report["r_pred_original"] = r_pred_orig
    report["v_pred_original"] = v_pred_orig

    if method == "dv":
        dv0_max = targeting_cfg.get("dv0_max_mps", None)
        dv0_axis_max = targeting_cfg.get("dv0_axis_max_mps", None)
        w_r = float(targeting_cfg.get("position_weight", 1.0))
        w_v = float(targeting_cfg.get("speed_weight", 0.0))
        dv0, pred = solve_dv0_minimize_state(
            r_rel_lvh,
            v_rel_lvh,
            n,
            tof,
            dv0_max=dv0_max,
            dv0_axis_max=dv0_axis_max,
            w_r=w_r,
            w_v=w_v,
        )
        r_rel_lvh = r_rel_lvh.copy()
        v_rel_lvh = v_rel_lvh + dv0
        report["dv0"] = dv0
        report["r_pred_targeted"] = pred["r_pred"]
        report["v_pred_targeted"] = pred["v_pred"]
    elif method == "time":
        dt_max = float(targeting_cfg.get("dt_max_s", 0.0))
        dt_step = float(targeting_cfg.get("dt_step_s", 1.0))
        best = solve_dt_offset_minimize_rT(r_rel_lvh, v_rel_lvh, n, tof, dt_max, dt_step)
        r_rel_lvh = best["r0"]
        v_rel_lvh = best["v0"]
        report["dt"] = best["dt"]
        report["r_pred_targeted"] = best["r_pred"]
        _, report["v_pred_targeted"] = cw_propagate(r_rel_lvh, v_rel_lvh, n, tof)
    else:
        raise ValueError(f"Unknown targeting method: {method}")

    return r_rel_lvh, v_rel_lvh, report


def run_sim_config(
    cfg,
    control_enabled=True,
    guidance_enabled=True,
    r_rel_lvh0=None,
    v_rel_lvh0=None,
    r_rel_lvh_nom0=None,
    v_rel_lvh_nom0=None,
    fast_mode=False,
):

    atm = build_atmosphere(cfg)
    r_ref, v_ref = build_reference_state(cfg)
    if r_rel_lvh0 is None or v_rel_lvh0 is None:
        r_rel_lvh0, v_rel_lvh0 = get_initial_relative_state(cfg)
    r_payload, v_payload = payload_state_from_rel(r_ref, v_ref, r_rel_lvh0, v_rel_lvh0)
    if r_rel_lvh_nom0 is None or v_rel_lvh_nom0 is None:
        r_rel_lvh_nom0 = r_rel_lvh0
        v_rel_lvh_nom0 = v_rel_lvh0
    r_payload_nom, v_payload_nom = payload_state_from_rel(r_ref, v_ref, r_rel_lvh_nom0, v_rel_lvh_nom0)

    node_cfg = cfg["receiver_node"]
    payload_cfg = cfg["payload"]

    env_node = {
        "mass": float(node_cfg["mass_kg"]),
        "Cd": float(node_cfg["Cd"]),
        "area": float(node_cfg["area_m2"]),
        "atmosphere": atm,
    }

    # Payload drag is applied via DDM control allocation, so base drag is set to zero here.
    env_payload = {
        "mass": float(payload_cfg["mass_kg"]),
        "Cd": float(payload_cfg["Cd"]),
        "area": 0.0,
        "atmosphere": atm,
    }

    ddm_cfg = {
        "area_min": float(payload_cfg["area_m2_min"]),
        "area_max": float(payload_cfg["area_m2_max"]),
    }
    medt_cfg = cfg["medt"]

    sim_cfg = cfg["simulation"]
    t_end = float(sim_cfg["t_end_s"])
    dt = float(sim_cfg["dt_s"])

    thresholds = cfg["gates"]["thresholds"]
    speed_limits = cfg["gates"]["speed_limits"]
    term_cfg = cfg.get("terminal_capture", {})

    # Inject nominal mean motion into guidance config if needed.
    mu = cfg.get("constants", {}).get("mu", None)
    if mu is None:
        from dynamics.constants import MU_EARTH
        mu = MU_EARTH
    n = np.sqrt(mu / (np.linalg.norm(r_ref) ** 3))
    cfg["guidance"]["n_rad_s"] = float(cfg["guidance"].get("n_rad_s", n))
    cor_entry_lead_s = float(cfg.get("targeting", {}).get("cor_entry_lead_s", 0.0))
    guide_t_end_default = max(float(dt), float(sim_cfg["t_end_s"]) - cor_entry_lead_s)
    cfg["guidance"]["t_end_s"] = float(cfg["guidance"].get("t_end_s", guide_t_end_default))

    guidance = Guidance(cfg["guidance"])
    estimator = Estimator()
    mode_mgr = TagModeManager(thresholds)
    gate_tracker = GateTracker(thresholds, speed_limits)

    output_dir = Path(sim_cfg["output_dir"])
    output_dir.mkdir(parents=True, exist_ok=True)

    logger = None if fast_mode else SimLogger()

    state_ref = np.hstack((r_ref, v_ref))
    state_payload = np.hstack((r_payload, v_payload))

    # Shadow nominal trajectory (no control) for guidance error.
    use_nominal = bool(cfg.get("guidance", {}).get("use_nominal", True))
    state_payload_nom = np.hstack((r_payload_nom, v_payload_nom))

    # Fast-mode accumulators
    if fast_mode:
        sum_applied_mag = 0.0
        sum_cmd_mag = 0.0
        sum_abs_axis = np.zeros(3)
        sum_ddm_on = 0.0
        sum_medt_on = 0.0
        sum_tag_applied = 0.0
        sum_node_applied = 0.0
        sum_tag_steps = 0
        sum_tag_sat_cmd = 0
        peak_applied = 0.0
        max_alloc_error = 0.0
        max_cmd_mag = 0.0
        count_steps = 0
        sum_term_steps = 0
        sum_term_sat = 0
        max_cmd_raw_term = 0.0
        max_applied_term = 0.0
        min_range_m = float("inf")
        t_closest_s = 0.0
        tag_cmd_max_accel = float(cfg.get("guidance", {}).get("max_accel", 1e-3))

    t = 0.0
    prev_range = None
    prev_speed = None
    while t <= t_end:
        # Relative state in ECI and LVLH (receiver node is the reference frame).
        r_rel = state_payload[0:3] - state_ref[0:3]
        v_rel = state_payload[3:6] - state_ref[3:6]

        r_lvh, v_lvh = eci_to_lvlh(r_rel, v_rel, state_ref[0:3], state_ref[3:6])
        range_m = float(np.linalg.norm(r_lvh))
        speed_mps = float(np.linalg.norm(v_lvh))
        if fast_mode and range_m < min_range_m:
            min_range_m = range_m
            t_closest_s = t

        # Tag mode, guidance update cadence, and LVLH command.
        mode = mode_mgr.update(range_m)
        if use_nominal:
            r_rel_nom = state_payload_nom[0:3] - state_ref[0:3]
            v_rel_nom = state_payload_nom[3:6] - state_ref[3:6]
            r_nom, v_nom = eci_to_lvlh(r_rel_nom, v_rel_nom, state_ref[0:3], state_ref[3:6])
            r_err = r_lvh - r_nom
            v_err = v_lvh - v_nom
        else:
            r_err = r_lvh
            v_err = v_lvh

        r_est, v_est = estimator.estimate(r_err, v_err)
        term_cfg = cfg.get("terminal_capture", {})
        terminal_active = False
        term_info = None
        if guidance_enabled:
            a_cmd_lvh = guidance.compute(t, r_est, v_est, mode)
            # Optional terminal capture controller (at/inside corridor).
            if term_cfg.get("enabled", False) and range_m <= thresholds["R_COR"]:
                a_cmd_lvh, term_info = terminal_capture_command(
                    r_lvh,
                    v_lvh,
                    thresholds,
                    speed_limits,
                    term_cfg,
                    dt,
                )
                terminal_active = True
        else:
            a_cmd_lvh = np.zeros(3)
        a_cmd_eci = lvlh_to_eci_vector(a_cmd_lvh, state_ref[0:3], state_ref[3:6])

        # Allocate command between DDM drag and mEDT, or use node terminal control.
        if terminal_active and term_cfg.get("use_node_thruster", False):
            a_total = a_cmd_eci
            a_ddm = np.zeros(3)
            a_medt = np.zeros(3)
            info = {
                "A_eff": float(ddm_cfg["area_min"]),
                "B_mag": 0.0,
                "allocation_error": 0.0,
            }
        elif control_enabled:
            a_total, a_ddm, a_medt, info = allocate_control(
                a_cmd_eci,
                state_payload,
                env_payload,
                ddm_cfg,
                medt_cfg,
                mode,
            )
        else:
            a_total = np.zeros(3)
            a_ddm = np.zeros(3)
            a_medt = np.zeros(3)
            info = {
                "A_eff": float(ddm_cfg["area_min"]),
                "B_mag": 0.0,
                "allocation_error": float(np.linalg.norm(a_cmd_eci)),
            }

        # Gate tracking for corridor compliance.
        events = gate_tracker.update(range_m, speed_mps, t, prev_range=prev_range, prev_speed=prev_speed)
        if term_cfg.get("enabled", False) and term_cfg.get("extend_mission", False):
            if "R_COR" in events and term_cfg.get("extra_time_s", 0.0) > 0.0:
                t_end = max(t_end, t + float(term_cfg["extra_time_s"]))

        ddm_on = 1.0 if info["A_eff"] > (ddm_cfg["area_min"] + 0.1 * (ddm_cfg["area_max"] - ddm_cfg["area_min"])) else 0.0
        medt_on = 1.0 if np.linalg.norm(a_medt) > 1e-12 else 0.0

        a_cmd_lvh_vec = a_cmd_lvh
        a_cmd_mag = float(np.linalg.norm(a_cmd_lvh_vec))
        if fast_mode:
            a_applied_lvh = None
            a_applied_mag = float(np.linalg.norm(a_total))
            alignment = 0.0
        else:
            a_applied_lvh = eci_to_lvlh_vector(a_total, state_ref[0:3], state_ref[3:6])
            a_applied_mag = float(np.linalg.norm(a_applied_lvh))
            if a_applied_mag > 1e-12 and range_m > 1e-9:
                alignment = float(np.dot(a_applied_lvh, -r_lvh) / (a_applied_mag * range_m))
            else:
                alignment = 0.0

        if fast_mode:
            sum_applied_mag += a_applied_mag
            sum_cmd_mag += a_cmd_mag
            if a_applied_lvh is not None:
                sum_abs_axis += np.abs(a_applied_lvh)
            sum_ddm_on += ddm_on
            sum_medt_on += medt_on
            if terminal_active and term_cfg.get("use_node_thruster", False):
                sum_node_applied += a_applied_mag
            else:
                sum_tag_applied += a_applied_mag
            if not terminal_active:
                sum_tag_steps += 1
                if a_cmd_mag >= (0.999 * tag_cmd_max_accel):
                    sum_tag_sat_cmd += 1
            if terminal_active and term_info is not None:
                sum_term_steps += 1
                if term_info.get("sat", False):
                    sum_term_sat += 1
                max_cmd_raw_term = max(max_cmd_raw_term, float(term_info.get("raw_mag", 0.0)))
                max_applied_term = max(max_applied_term, a_applied_mag)
            peak_applied = max(peak_applied, a_applied_mag)
            max_alloc_error = max(max_alloc_error, info["allocation_error"])
            max_cmd_mag = max(max_cmd_mag, a_cmd_mag)
            count_steps += 1
        else:
            logger.log(
                t=t,
                r_rel_lvh=r_lvh,
                v_rel_lvh=v_lvh,
                range_m=range_m,
                speed_mps=speed_mps,
                mode=mode,
                a_cmd_lvh=a_cmd_lvh,
                a_cmd_eci=a_cmd_eci,
                a_applied_lvh=a_applied_lvh,
                a_applied_mag=a_applied_mag,
                a_cmd_mag=a_cmd_mag,
                alignment=alignment,
                a_total_eci=a_total,
                a_ddm_eci=a_ddm,
                a_medt_eci=a_medt,
                A_eff=info["A_eff"],
                B_mag=info["B_mag"],
                allocation_error=info["allocation_error"],
                ddm_on=ddm_on,
                medt_on=medt_on,
            )

        # Termination condition (optional gate or capture gate + speed constraint).
        stop_gate = sim_cfg.get("stop_on_gate", None)
        if stop_gate is not None and stop_gate in gate_tracker.crossings:
            hard_max = speed_limits.get(stop_gate, {}).get("hard_max")
            if hard_max is None or speed_mps <= hard_max:
                break

        if sim_cfg.get("stop_on_success", True):
            if "R_LATCH" in gate_tracker.crossings and speed_mps <= speed_limits["R_LATCH"]["hard_max"]:
                break

        def eom_ref(ti, yi):
            return eom(ti, yi, env_node, control_accel=None)

        def eom_payload(ti, yi):
            return eom(ti, yi, env_payload, control_accel=a_total)

        def eom_payload_nom(ti, yi):
            return eom(ti, yi, env_payload, control_accel=None)

        # Propagate both reference and payload using fixed-step RK4.
        state_ref = rk4_step(eom_ref, t, state_ref, dt)
        state_payload = rk4_step(eom_payload, t, state_payload, dt)
        if use_nominal:
            state_payload_nom = rk4_step(eom_payload_nom, t, state_payload_nom, dt)
        prev_range = range_m
        prev_speed = speed_mps
        t += dt

    if fast_mode:
        allocation_error_tol = float(cfg["simulation"].get("allocation_error_tol", 1e-6))
        adcs_limit = cfg["simulation"].get("adcs_saturation_limit")

        tags = []
        if gate_tracker.violations:
            tags.append("speed_violation")
        if "R_COR" not in gate_tracker.crossings:
            tags.append("miss_radius_violation")
        if "R_LATCH" not in gate_tracker.crossings:
            tags.append("capture_not_reached")
        if max_alloc_error > allocation_error_tol:
            tags.append("actuator_limit")
        if adcs_limit is not None and max_cmd_mag > float(adcs_limit):
            tags.append("adcs_saturation")

        corridor_entry_success = False
        precapture_success = False
        latch_success = False
        if "R_COR" in gate_tracker.crossings:
            speed = gate_tracker.crossings["R_COR"]["speed_mps"]
            corridor_entry_success = speed <= gate_tracker.speed_limits["R_COR"]["hard_max"]
        if "R_PRE" in gate_tracker.crossings:
            speed = gate_tracker.crossings["R_PRE"]["speed_mps"]
            precapture_success = speed <= gate_tracker.speed_limits["R_PRE"]["hard_max"]
        if "R_LATCH" in gate_tracker.crossings:
            speed = gate_tracker.crossings["R_LATCH"]["speed_mps"]
            latch_success = speed <= gate_tracker.speed_limits["R_LATCH"]["hard_max"]

        dv_eq_applied = float(sum_applied_mag * dt)
        dv_axis = sum_abs_axis * dt
        dv_eq_commanded = float(sum_cmd_mag * dt)
        dv_tag = float(sum_tag_applied * dt)
        dv_node_terminal = float(sum_node_applied * dt)
        avg_applied = float(sum_applied_mag / max(count_steps, 1))
        term_steps = sum_term_steps
        term_sat = sum_term_sat
        term_sat_frac = float(term_sat / max(term_steps, 1))
        tag_sat_frac = float(sum_tag_sat_cmd / max(sum_tag_steps, 1))

        metrics = {
            "success": latch_success and ("speed_violation" not in tags),
            "tags": tags,
            "dv_eq_applied": dv_eq_applied,
            "dv_axis": dv_axis,
            "dv_eq_commanded": dv_eq_commanded,
            "peak_applied": float(peak_applied),
            "avg_applied": avg_applied,
            "duty_ddm": float(sum_ddm_on / max(count_steps, 1)),
            "duty_medt": float(sum_medt_on / max(count_steps, 1)),
            "dv_tag": dv_tag,
            "dv_node_terminal": dv_node_terminal,
            "corridor_entry_success": corridor_entry_success,
            "precapture_success": precapture_success,
            "latch_success": latch_success,
            "term_steps": int(term_steps),
            "term_sat_steps": int(term_sat),
            "term_sat_frac": term_sat_frac,
            "max_cmd_raw_term": float(max_cmd_raw_term),
            "max_applied_term": float(max_applied_term),
            "tag_steps": int(sum_tag_steps),
            "tag_sat_cmd_steps": int(sum_tag_sat_cmd),
            "tag_sat_cmd_frac": tag_sat_frac,
            "min_range_m": float(min_range_m),
            "t_closest_s": float(t_closest_s),
        }
    else:
        metrics = compute_metrics(logger, gate_tracker, cfg["simulation"])

    return metrics, gate_tracker, logger


def run_sim(config_path):
    cfg = load_config(config_path)
    return run_sim_config(cfg)
