from __future__ import annotations

from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


def save_line_plots(df: pd.DataFrame, output_path: str | Path) -> Path:
    """Save core telemetry plots for altitude, control response, CoM, and stability."""

    output_path = Path(output_path)
    fig, axes = plt.subplots(3, 2, figsize=(14, 12), sharex="col")
    ax_altitude, ax_lateral, ax_pitch, ax_gimbal, ax_com, ax_stability = axes.flatten()

    ax_altitude.plot(df["time_s"], df["altitude_m"], color="tab:blue", linewidth=2.0)
    ax_altitude.set_ylabel("Altitude (m)")
    ax_altitude.set_title("Altitude vs Time")
    ax_altitude.grid(True, alpha=0.3)

    ax_lateral.plot(df["time_s"], df["x_m"], color="tab:purple", linewidth=2.0)
    ax_lateral.set_ylabel("Lateral Position (m)")
    ax_lateral.set_title("Wind Disturbance Response")
    ax_lateral.grid(True, alpha=0.3)

    ax_pitch.plot(df["time_s"], df["pitch_angle_deg"], color="tab:orange", linewidth=2.0, label="True Pitch")
    ax_pitch.plot(
        df["time_s"],
        df["measured_pitch_angle_deg"],
        color="tab:brown",
        linewidth=1.0,
        alpha=0.65,
        label="Measured Pitch",
    )
    ax_pitch.set_ylabel("Pitch (deg)")
    ax_pitch.set_title("Pitch Correction Response")
    ax_pitch.grid(True, alpha=0.3)
    ax_pitch.legend(loc="best")

    ax_gimbal.plot(df["time_s"], df["gimbal_angle_deg"], color="tab:green", linewidth=2.0)
    ax_gimbal.set_ylabel("Gimbal (deg)")
    ax_gimbal.set_title("Gimbal Correction Command")
    ax_gimbal.grid(True, alpha=0.3)

    ax_com.plot(df["time_s"], df["center_of_mass_m"], color="tab:red", linewidth=2.0, label="CoM")
    ax_com.plot(
        df["time_s"],
        df["center_of_pressure_m"],
        color="tab:cyan",
        linewidth=1.5,
        linestyle="--",
        label="CoP",
    )
    ax_com.set_ylabel("Body Position (m)")
    ax_com.set_title("Center of Mass Migration")
    ax_com.grid(True, alpha=0.3)
    ax_com.legend(loc="best")

    stability_min = float(min(df["stability_margin_m"].min(), -1.5))
    stability_max = float(max(df["stability_margin_m"].max(), 0.5))
    ax_stability.axhspan(stability_min, -0.25, color="tab:green", alpha=0.12)
    ax_stability.axhspan(-0.25, 0.0, color="gold", alpha=0.14)
    ax_stability.axhspan(0.0, stability_max, color="tab:red", alpha=0.10)
    ax_stability.plot(df["time_s"], df["stability_margin_m"], color="black", linewidth=2.0)
    ax_stability.set_ylabel("CoP - CoM (m)")
    ax_stability.set_title("Stability Margin vs Time")
    ax_stability.grid(True, alpha=0.3)
    ax_stability.text(df["time_s"].iloc[-1] * 0.78, -0.9, "Stable", color="tab:green")
    ax_stability.text(df["time_s"].iloc[-1] * 0.72, -0.12, "Marginal", color="goldenrod")
    ax_stability.text(df["time_s"].iloc[-1] * 0.76, 0.15, "Unstable", color="tab:red")

    for axis in axes[-1]:
        axis.set_xlabel("Time (s)")

    fig.suptitle("Closed-Loop Rocket Ascent Telemetry", fontsize=14)
    fig.tight_layout()
    fig.savefig(output_path, dpi=150)
    plt.close(fig)
    return output_path
