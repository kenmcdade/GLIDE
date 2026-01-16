# ============================================================
# File: main.py
# GLIDE v3.1 – Simulation Orchestrator
# ============================================================

import numpy as np
import sys
from utils.config import load_config, print_config_summary
from core.integrator import GLIDEIntegrator


def winch_profile(t: float) -> float:
    omega = 8.0 * np.sin(2.0 * np.pi * 0.2 * t)
    return omega


def current_profile(t: float) -> float:
    cycle = int(t // 10) % 2
    return 0.8 if cycle == 0 else -0.8


def main():
    mode = "engineering"
    if len(sys.argv) > 1:
        mode = sys.argv[1]
    cfg = load_config(mode)
    print_config_summary(cfg)

    sim = GLIDEIntegrator(cfg)

    sim_duration = 60.0
    print(f"Starting GLIDE simulation for {sim_duration:.1f} s in mode '{mode}'...\n")

    summary = sim.run(
        duration=sim_duration,
        omega_profile=winch_profile,
        current_profile=current_profile
    )

    print("\n================= SIMULATION COMPLETE =================")
    print(f"Total steps: {int(sim_duration / cfg['dt'])}")
    print(f"Final Time:  {sim_duration:.2f} s")
    print("--------------------------------------------------------")
    print(f"Battery SoC: {summary['SoC']*100:8.5f}%")
    print(f"Total Energy: {summary['E_total']:.2f} J")
    print(f"Elastic Energy: {summary['E_elastic']:.2f} J")
    print(f"Kinetic Energy: {summary['E_kin']:.2f} J")
    print(f"Gravitational: {summary['E_grav']:.2f} J")
    print("--------------------------------------------------------")

    if cfg.get("save_energy_csv", True):
        sim.energy.export_csv("glide_v3_energy_log.csv")
        print("Energy log saved to glide_v3_energy_log.csv")

    print("========================================================\n")
    print("Simulation data ready for visualization or analysis.")

    if sys.stdin.isatty():
        input("\nSimulation complete. Press Enter to exit...")


if __name__ == "__main__":
    import traceback
    try:
        main()
    except Exception:
        print("\n" + "="*70)
        print("🔥 UNCAUGHT ERROR DURING GLIDE SIMULATION:")
        traceback.print_exc()
        print("="*70)
        if sys.stdin.isatty():
            input("\nPress Enter to close...")
        sys.exit(1)
