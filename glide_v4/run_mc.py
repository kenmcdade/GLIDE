"""Run Monte Carlo stub (Phase 2)."""

from mc.runner import run_mc


def main():
    result = run_mc("configs/demo.yaml")
    if result is None:
        print("Monte Carlo disabled in config.")
        return
    print(f"Runs: {result['runs']}")
    print(f"P_corr: {result['p_corr']:.3f}")
    sweep = result.get("p_corr_sweep", {})
    if sweep:
        print("P_corr vs TOF:")
        for tof, p in sweep.items():
            print(f"  TOF {tof}s: {p:.3f}")


if __name__ == "__main__":
    main()
