"""Run Monte Carlo for dispersion recovery."""

from mc.dispersion_mc import run_dispersion_mc


if __name__ == "__main__":
    run_dispersion_mc("configs/dispersion_recovery.yaml", runs=500, seed=123, terminal_enabled=False)
