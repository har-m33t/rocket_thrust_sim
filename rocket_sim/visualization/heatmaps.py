from __future__ import annotations

from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


def _build_thrust_distribution_heatmap(
    df: pd.DataFrame,
    time_bins: np.ndarray,
    body_bins: np.ndarray,
) -> np.ndarray:
    """Accumulate thrust across the lever arm between the engine and the current CoM."""

    histogram = np.zeros((len(time_bins) - 1, len(body_bins) - 1), dtype=float)
    for row in df.itertuples(index=False):
        body_positions_m = np.linspace(row.center_of_mass_m, row.engine_position_m, 12)
        thrust_weights_n = np.full(body_positions_m.shape, row.thrust_n / len(body_positions_m))
        step_histogram, _, _ = np.histogram2d(
            np.full(body_positions_m.shape, row.time_s),
            body_positions_m,
            bins=[time_bins, body_bins],
            weights=thrust_weights_n,
        )
        histogram += step_histogram
    return histogram


def save_engine_workload_heatmap(df: pd.DataFrame, output_path: str | Path) -> Path:
    """Save thrust distribution and gimbal workload heatmaps."""

    output_path = Path(output_path)
    time_bins = np.linspace(df["time_s"].min(), df["time_s"].max(), 61)
    body_bins = np.linspace(0.0, max(float(df["engine_position_m"].max()) + 0.2, 4.4), 60)
    gimbal_limit_deg = max(8.5, float(np.ceil(df["gimbal_angle_deg"].abs().max() + 1.0)))
    gimbal_bins = np.linspace(-gimbal_limit_deg, gimbal_limit_deg, 51)

    thrust_histogram = _build_thrust_distribution_heatmap(df, time_bins, body_bins)
    gimbal_histogram, _, _ = np.histogram2d(
        df["time_s"],
        df["gimbal_angle_deg"],
        bins=[time_bins, gimbal_bins],
        weights=df["thrust_n"],
    )

    fig, (ax_thrust, ax_gimbal) = plt.subplots(2, 1, figsize=(12, 9), sharex=True)

    thrust_mesh = ax_thrust.pcolormesh(
        time_bins,
        body_bins,
        thrust_histogram.T,
        shading="auto",
        cmap="magma",
    )
    ax_thrust.plot(df["time_s"], df["center_of_mass_m"], color="cyan", linewidth=1.5, label="CoM")
    ax_thrust.plot(
        df["time_s"],
        df["center_of_pressure_m"],
        color="white",
        linewidth=1.2,
        linestyle="--",
        label="CoP",
    )
    ax_thrust.plot(
        df["time_s"],
        df["engine_position_m"],
        color="lime",
        linewidth=1.0,
        linestyle=":",
        label="Engine",
    )
    ax_thrust.set_ylabel("Body Position (m)")
    ax_thrust.set_title("Thrust Distribution Along the Body")
    ax_thrust.legend(loc="upper left")
    fig.colorbar(thrust_mesh, ax=ax_thrust, label="Weighted Thrust (N)")

    gimbal_mesh = ax_gimbal.pcolormesh(
        time_bins,
        gimbal_bins,
        gimbal_histogram.T,
        shading="auto",
        cmap="viridis",
    )
    ax_gimbal.set_xlabel("Time (s)")
    ax_gimbal.set_ylabel("Gimbal Angle (deg)")
    ax_gimbal.set_title("Gimbal Angle vs Time Weighted by Thrust")
    fig.colorbar(gimbal_mesh, ax=ax_gimbal, label="Weighted Thrust (N)")

    fig.suptitle("Engine Workload and Thrust Distribution", fontsize=14)
    fig.tight_layout()
    fig.savefig(output_path, dpi=150)
    plt.close(fig)
    return output_path
