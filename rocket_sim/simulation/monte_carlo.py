from __future__ import annotations

from dataclasses import replace

import numpy as np
import pandas as pd

from rocket_sim.data.rocket_config import RocketConfig
from rocket_sim.simulation.simulation_loop import SimulationLoop


class MonteCarloRunner:
    """Runs multiple perturbed simulations to visualize robustness and variability."""

    def __init__(self, base_config: RocketConfig) -> None:
        self.base_config = base_config
        self.rng = np.random.default_rng(base_config.random_seed + 1_000)

    def run(self, runs: int | None = None) -> tuple[pd.DataFrame, pd.DataFrame]:
        """Run Monte Carlo samples and return time-series and summary data."""

        run_count = self.base_config.monte_carlo_runs if runs is None else runs
        trajectory_frames: list[pd.DataFrame] = []
        summary_rows: list[dict[str, float]] = []

        for run_id in range(run_count):
            drag_scale = max(0.7, float(self.rng.normal(1.0, self.base_config.monte_carlo_drag_sigma)))
            thrust_scale = max(0.8, float(self.rng.normal(1.0, self.base_config.monte_carlo_thrust_sigma)))
            wind_base_force_n = float(
                self.rng.normal(
                    self.base_config.wind_base_force_n,
                    self.base_config.monte_carlo_wind_sigma_n,
                )
            )

            run_config = replace(
                self.base_config,
                drag_scale=drag_scale,
                thrust_scale=thrust_scale,
                wind_base_force_n=wind_base_force_n,
            )
            run_seed = self.base_config.random_seed + run_id + 1
            df = SimulationLoop(run_config, seed=run_seed).run()
            df["run_id"] = run_id
            trajectory_frames.append(
                df[["run_id", "time_s", "x_m", "altitude_m", "stability_margin_m"]].copy()
            )

            summary_rows.append(
                {
                    "run_id": run_id,
                    "drag_scale": drag_scale,
                    "thrust_scale": thrust_scale,
                    "wind_base_force_n": wind_base_force_n,
                    "max_altitude_m": float(df["altitude_m"].max()),
                    "final_lateral_position_m": float(df["x_m"].iloc[-1]),
                    "max_abs_pitch_deg": float(df["pitch_angle_deg"].abs().max()),
                    "min_stability_margin_m": float(df["stability_margin_m"].min()),
                }
            )

        return pd.concat(trajectory_frames, ignore_index=True), pd.DataFrame(summary_rows)
