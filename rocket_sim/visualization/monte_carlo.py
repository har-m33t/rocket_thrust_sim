from __future__ import annotations

from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import pandas as pd


def save_monte_carlo_plots(
    trajectories_df: pd.DataFrame,
    summary_df: pd.DataFrame,
    output_path: str | Path,
) -> Path:
    """Save trajectory spread, altitude distribution, and stability variability plots."""

    output_path = Path(output_path)
    fig, axes = plt.subplots(1, 3, figsize=(16, 5))

    for _, run_df in trajectories_df.groupby("run_id"):
        axes[0].plot(run_df["x_m"], run_df["altitude_m"], alpha=0.35, linewidth=1.0)
    axes[0].set_xlabel("Lateral Position (m)")
    axes[0].set_ylabel("Altitude (m)")
    axes[0].set_title("Trajectory Spread")
    axes[0].grid(True, alpha=0.3)

    axes[1].hist(summary_df["max_altitude_m"], bins=10, color="tab:blue", alpha=0.75, edgecolor="black")
    axes[1].set_xlabel("Max Altitude (m)")
    axes[1].set_ylabel("Count")
    axes[1].set_title("Max Altitude Distribution")
    axes[1].grid(True, alpha=0.3)

    for _, run_df in trajectories_df.groupby("run_id"):
        axes[2].plot(run_df["time_s"], run_df["stability_margin_m"], alpha=0.25, linewidth=1.0)
    stability_mean = trajectories_df.groupby("time_s")["stability_margin_m"].mean()
    axes[2].plot(
        stability_mean.index,
        stability_mean.values,
        color="black",
        linewidth=2.0,
        label="Mean",
    )
    axes[2].set_xlabel("Time (s)")
    axes[2].set_ylabel("CoP - CoM (m)")
    axes[2].set_title("Stability Variability")
    axes[2].grid(True, alpha=0.3)
    axes[2].legend(loc="best")

    fig.suptitle("Monte Carlo Robustness Analysis", fontsize=14)
    fig.tight_layout()
    fig.savefig(output_path, dpi=150)
    plt.close(fig)
    return output_path
