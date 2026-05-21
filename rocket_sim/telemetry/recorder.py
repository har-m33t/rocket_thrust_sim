from __future__ import annotations

from typing import Dict, List

import numpy as np
import pandas as pd

from rocket_sim.control.sensors import AttitudeMeasurement
from rocket_sim.physics.rocket_state import RocketState


class TelemetryRecorder:
    """Collects per-step state, controller, and analysis data."""

    def __init__(self) -> None:
        self._rows: List[Dict[str, float]] = []

    def record(
        self,
        state: RocketState,
        measurement: AttitudeMeasurement,
        desired_gimbal_angle_rad: float,
        actual_gimbal_angle_rad: float,
        diagnostics: Dict[str, float],
    ) -> None:
        """Store the telemetry for the current timestep."""

        self._rows.append(
            {
                "time_s": state.time_s,
                "x_m": state.x_m,
                "altitude_m": state.y_m,
                "vx_mps": state.vx_mps,
                "vy_mps": state.vy_mps,
                "speed_mps": diagnostics["speed_mps"],
                "pitch_angle_deg": float(np.degrees(state.pitch_rad)),
                "measured_pitch_angle_deg": float(np.degrees(measurement.pitch_rad)),
                "angular_velocity_deg_s": float(np.degrees(state.angular_velocity_rad_s)),
                "measured_angular_velocity_deg_s": float(
                    np.degrees(measurement.angular_velocity_rad_s)
                ),
                "desired_gimbal_angle_deg": float(np.degrees(desired_gimbal_angle_rad)),
                "gimbal_angle_deg": float(np.degrees(actual_gimbal_angle_rad)),
                "thrust_n": diagnostics["thrust_n"],
                "thrust_direction_deg": diagnostics["thrust_direction_deg"],
                "drag_n": diagnostics["drag_n"],
                "drag_coefficient": diagnostics["cd"],
                "angle_of_attack_deg": diagnostics["alpha_deg"],
                "wind_force_n": diagnostics["wind_force_n"],
                "dry_mass_kg": state.dry_mass_kg,
                "propellant_mass_kg": state.propellant_mass_kg,
                "total_mass_kg": state.total_mass_kg,
                "center_of_mass_m": state.center_of_mass_m,
                "center_of_pressure_m": diagnostics["center_of_pressure_m"],
                "engine_position_m": diagnostics["engine_position_m"],
                "engine_to_com_m": diagnostics["engine_to_com_m"],
                "stability_margin_m": diagnostics["stability_margin_m"],
                "ax_mps2": diagnostics["ax_mps2"],
                "ay_mps2": diagnostics["ay_mps2"],
                "control_torque_nm": diagnostics["control_torque_nm"],
                "wind_torque_nm": diagnostics["wind_torque_nm"],
                "damping_torque_nm": diagnostics["damping_torque_nm"],
                "pitch_inertia_kgm2": diagnostics["pitch_inertia_kgm2"],
            }
        )

    def to_dataframe(self) -> pd.DataFrame:
        """Return the full telemetry history as a pandas DataFrame."""

        return pd.DataFrame(self._rows)
