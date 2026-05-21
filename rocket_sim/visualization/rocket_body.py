from __future__ import annotations

from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from matplotlib.patches import Polygon, Rectangle

from rocket_sim.data.rocket_config import RocketConfig


def _draw_single_snapshot(ax: plt.Axes, row: pd.Series, config: RocketConfig) -> None:
    """Draw one simplified rocket body with CoM, CoP, engine, and thrust vector markers."""

    body = Rectangle(
        (-config.body_radius_m, 0.25),
        2.0 * config.body_radius_m,
        config.body_length_m - 0.5,
        fill=False,
        linewidth=2.0,
        edgecolor="black",
    )
    nose = Polygon(
        [
            (-config.body_radius_m, 0.25),
            (0.0, 0.0),
            (config.body_radius_m, 0.25),
        ],
        closed=True,
        fill=False,
        linewidth=2.0,
        edgecolor="black",
    )
    ax.add_patch(body)
    ax.add_patch(nose)

    ax.scatter(0.0, row["center_of_mass_m"], color="tab:red", s=60, label="CoM")
    ax.scatter(0.0, row["center_of_pressure_m"], color="tab:blue", s=60, marker="s", label="CoP")
    ax.scatter(0.0, row["engine_position_m"], color="black", s=70, marker="v", label="Engine")

    arrow_length_m = 0.7
    gimbal_angle_rad = np.radians(row["gimbal_angle_deg"])
    thrust_dx = arrow_length_m * np.sin(gimbal_angle_rad)
    thrust_dy = arrow_length_m * np.cos(gimbal_angle_rad)
    ax.arrow(
        0.0,
        row["engine_position_m"],
        thrust_dx,
        -thrust_dy,
        width=0.015,
        head_width=0.09,
        head_length=0.12,
        length_includes_head=True,
        color="tab:green",
    )

    ax.set_xlim(-1.0, 1.0)
    ax.set_ylim(config.body_length_m + 0.2, -0.2)
    ax.set_aspect("equal")
    ax.set_xticks([])
    ax.set_ylabel("Body Position (m)")
    ax.set_title(
        f"t={row['time_s']:.1f}s | pitch={row['pitch_angle_deg']:.2f} deg\n"
        f"CoM={row['center_of_mass_m']:.2f} m | margin={row['stability_margin_m']:.2f} m"
    )


def save_rocket_body_visualization(
    df: pd.DataFrame,
    config: RocketConfig,
    output_path: str | Path,
) -> Path:
    """Save a multi-snapshot rocket body visualization across the simulation."""

    output_path = Path(output_path)
    sample_indices = np.unique(np.linspace(0, len(df) - 1, 6, dtype=int))
    sample_df = df.iloc[sample_indices].reset_index(drop=True)

    fig, axes = plt.subplots(2, 3, figsize=(12, 10))
    for axis, (_, row) in zip(axes.flatten(), sample_df.iterrows()):
        _draw_single_snapshot(axis, row, config)

    handles, labels = axes.flatten()[0].get_legend_handles_labels()
    fig.legend(handles, labels, loc="upper center", ncol=4, bbox_to_anchor=(0.5, 0.98))
    fig.suptitle(
        "Rocket Body Visualization\nCoM migration, fixed CoP estimate, and thrust-vector deflection",
        fontsize=14,
    )
    fig.tight_layout(rect=(0, 0, 1, 0.95))
    fig.savefig(output_path, dpi=150)
    plt.close(fig)
    return output_path
