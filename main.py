from __future__ import annotations

from pathlib import Path

from rocket_sim.data.rocket_config import RocketConfig
from rocket_sim.simulation.monte_carlo import MonteCarloRunner
from rocket_sim.simulation.simulation_loop import SimulationLoop
from rocket_sim.telemetry.exporter import export_dataframe_to_csv
from rocket_sim.visualization.heatmaps import save_engine_workload_heatmap
from rocket_sim.visualization.monte_carlo import save_monte_carlo_plots
from rocket_sim.visualization.plots import save_line_plots
from rocket_sim.visualization.rocket_body import save_rocket_body_visualization


def main() -> None:
    """Run the rocket analysis workflow and save telemetry plus analysis figures."""

    config = RocketConfig()
    simulation_df = SimulationLoop(config).run()
    telemetry_csv_path = export_dataframe_to_csv(
        simulation_df,
        Path("telemetry") / "simulation_output.csv",
    )
    line_plot_path = save_line_plots(simulation_df, Path("telemetry_plots.png"))
    body_plot_path = save_rocket_body_visualization(
        simulation_df,
        config,
        Path("rocket_body_visualization.png"),
    )
    heatmap_path = save_engine_workload_heatmap(
        simulation_df,
        Path("engine_workload_and_thrust_distribution.png"),
    )

    monte_carlo_runner = MonteCarloRunner(config)
    trajectories_df, monte_carlo_summary_df = monte_carlo_runner.run()
    monte_carlo_plot_path = save_monte_carlo_plots(
        trajectories_df,
        monte_carlo_summary_df,
        Path("monte_carlo_analysis.png"),
    )

    final_row = simulation_df.iloc[-1]
    print("Rocket systems analysis run complete.")
    print(
        f"Final altitude: {final_row['altitude_m']:.1f} m | "
        f"Final pitch: {final_row['pitch_angle_deg']:.3f} deg | "
        f"Final CoM: {final_row['center_of_mass_m']:.3f} m | "
        f"Stability margin: {final_row['stability_margin_m']:.3f} m"
    )
    print(
        f"Mass change: {simulation_df['total_mass_kg'].iloc[0]:.2f} kg -> "
        f"{simulation_df['total_mass_kg'].iloc[-1]:.2f} kg"
    )
    print(
        f"Monte Carlo max altitude range: "
        f"{monte_carlo_summary_df['max_altitude_m'].min():.1f} m to "
        f"{monte_carlo_summary_df['max_altitude_m'].max():.1f} m"
    )
    print(f"Saved telemetry CSV to: {telemetry_csv_path}")
    print(f"Saved line plots to: {line_plot_path}")
    print(f"Saved rocket body visualization to: {body_plot_path}")
    print(f"Saved heatmaps to: {heatmap_path}")
    print(f"Saved Monte Carlo analysis to: {monte_carlo_plot_path}")


if __name__ == "__main__":
    main()
