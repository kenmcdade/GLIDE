"""Plotting utilities."""

from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt


def plot_overlay_range(data_on, data_off, gate_tracker, output_dir):
    out = Path(output_dir)
    out.mkdir(parents=True, exist_ok=True)

    t_on = data_on["t"]
    r_on = np.linalg.norm(data_on["r_rel_lvh"], axis=1)
    t_off = data_off["t"]
    r_off = np.linalg.norm(data_off["r_rel_lvh"], axis=1)

    plt.figure(figsize=(9, 5))
    plt.plot(t_on, r_on, label="Actuation ON")
    plt.plot(t_off, r_off, label="Actuation OFF", alpha=0.7)
    for name, thr in gate_tracker.thresholds.items():
        plt.axhline(thr, linestyle=":", alpha=0.5, label=name)
    plt.xlabel("Time (s)")
    plt.ylabel("Range (m)")
    plt.title("Range vs Time (Overlay)")
    plt.grid(True, alpha=0.3)
    plt.legend()
    plt.tight_layout()
    plt.savefig(out / "range_overlay.png", dpi=150)
    plt.close()


def plot_overlay_rtn(data_on, data_off, output_dir):
    out = Path(output_dir)
    out.mkdir(parents=True, exist_ok=True)

    t_on = data_on["t"]
    t_off = data_off["t"]
    r_on = data_on["r_rel_lvh"]
    r_off = data_off["r_rel_lvh"]

    fig, axes = plt.subplots(3, 1, figsize=(9, 8), sharex=True)
    labels = ["R", "T", "N"]
    for i in range(3):
        axes[i].plot(t_on, r_on[:, i], label="ON")
        axes[i].plot(t_off, r_off[:, i], label="OFF", alpha=0.7)
        axes[i].set_ylabel(f"{labels[i]} (m)")
        axes[i].grid(True, alpha=0.3)
        if i == 0:
            axes[i].legend()
    axes[-1].set_xlabel("Time (s)")
    fig.suptitle("RTN Components (Overlay)")
    fig.tight_layout()
    fig.savefig(out / "rtn_overlay.png", dpi=150)
    plt.close(fig)


def plot_speed(data, output_dir):
    out = Path(output_dir)
    out.mkdir(parents=True, exist_ok=True)

    t = data["t"]
    v_lvh = data["v_rel_lvh"]
    speed = np.linalg.norm(v_lvh, axis=1)
    plt.figure(figsize=(9, 5))
    plt.plot(t, speed, label="Relative speed")
    plt.xlabel("Time (s)")
    plt.ylabel("Relative Speed (m/s)")
    plt.title("Relative Speed vs Time")
    plt.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig(out / "rel_speed.png", dpi=150)
    plt.close()


def plot_velocity_components(data, output_dir):
    out = Path(output_dir)
    out.mkdir(parents=True, exist_ok=True)

    t = data["t"]
    v_lvh = data["v_rel_lvh"]
    plt.figure(figsize=(9, 5))
    plt.plot(t, v_lvh[:, 0], label="v_R")
    plt.plot(t, v_lvh[:, 1], label="v_T")
    plt.plot(t, v_lvh[:, 2], label="v_N")
    plt.xlabel("Time (s)")
    plt.ylabel("Relative Velocity (m/s)")
    plt.title("Relative Velocity in RTN")
    plt.grid(True, alpha=0.3)
    plt.legend()
    plt.tight_layout()
    plt.savefig(out / "v_rtn.png", dpi=150)
    plt.close()


def plot_applied_accel(data, output_dir):
    out = Path(output_dir)
    out.mkdir(parents=True, exist_ok=True)

    t = data["t"]
    a = data["a_applied_lvh"]
    plt.figure(figsize=(9, 5))
    plt.plot(t, a[:, 0], label="a_R")
    plt.plot(t, a[:, 1], label="a_T")
    plt.plot(t, a[:, 2], label="a_N")
    plt.xlabel("Time (s)")
    plt.ylabel("Applied Accel (m/s^2)")
    plt.title("Applied Acceleration in RTN")
    plt.grid(True, alpha=0.3)
    plt.legend()
    plt.tight_layout()
    plt.savefig(out / "a_applied_rtn.png", dpi=150)
    plt.close()


def plot_alignment(data, output_dir):
    out = Path(output_dir)
    out.mkdir(parents=True, exist_ok=True)

    t = data["t"]
    alignment = data["alignment"]
    plt.figure(figsize=(9, 5))
    plt.plot(t, alignment, label="alignment")
    plt.xlabel("Time (s)")
    plt.ylabel("Alignment")
    plt.title("Alignment vs Time")
    plt.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig(out / "alignment.png", dpi=150)
    plt.close()


def plot_all(data_on, gate_tracker, output_dir, data_off=None):
    out = Path(output_dir)
    out.mkdir(parents=True, exist_ok=True)

    if data_off is not None:
        plot_overlay_range(data_on, data_off, gate_tracker, output_dir)
        plot_overlay_rtn(data_on, data_off, output_dir)

    plot_speed(data_on, output_dir)
    plot_velocity_components(data_on, output_dir)
    plot_applied_accel(data_on, output_dir)
    plot_alignment(data_on, output_dir)
