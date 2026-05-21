from __future__ import annotations

from typing import Dict, Tuple

import numpy as np

from rocket_sim.data.rocket_config import RocketConfig
from rocket_sim.physics.aerodynamics import Aerodynamics
from rocket_sim.physics.mass_properties import MassProperties
from rocket_sim.physics.rocket_state import RocketState
from rocket_sim.physics.thrust_curve import ThrustCurve
from rocket_sim.physics.wind_model import WindModel


class PhysicsEngine:
    """Advances the rocket state one timestep using intentionally simplified 2D dynamics."""

    def __init__(
        self,
        config: RocketConfig,
        thrust_curve: ThrustCurve,
        aerodynamics: Aerodynamics,
        mass_properties: MassProperties,
        wind_model: WindModel,
    ) -> None:
        self.config = config
        self.thrust_curve = thrust_curve
        self.aerodynamics = aerodynamics
        self.mass_properties = mass_properties
        self.wind_model = wind_model

    def initial_state(self) -> RocketState:
        """Create the launch state with a small initial pitch disturbance."""

        propellant_mass_kg = self.config.initial_propellant_mass_kg
        total_mass_kg = self.mass_properties.total_mass_kg(propellant_mass_kg)
        center_of_mass_m = self.mass_properties.center_of_mass_m(propellant_mass_kg)
        return RocketState(
            time_s=0.0,
            x_m=0.0,
            y_m=0.0,
            vx_mps=0.0,
            vy_mps=0.0,
            pitch_rad=np.radians(self.config.initial_pitch_deg),
            angular_velocity_rad_s=0.0,
            dry_mass_kg=self.config.dry_mass_kg,
            propellant_mass_kg=propellant_mass_kg,
            total_mass_kg=total_mass_kg,
            center_of_mass_m=center_of_mass_m,
            thrust_n=self.thrust_curve.thrust_at(0.0),
            gimbal_angle_rad=0.0,
        )

    def update(
        self,
        state: RocketState,
        gimbal_angle_rad: float,
        dt_s: float,
    ) -> Tuple[RocketState, Dict[str, float]]:
        """Advance the state by one Forward Euler timestep."""

        thrust_n = self.thrust_curve.thrust_at(state.time_s)
        drag = self.aerodynamics.drag_force(state.vx_mps, state.vy_mps, state.pitch_rad)
        pitch_inertia_kgm2 = self.mass_properties.pitch_inertia_kgm2(state.propellant_mass_kg)
        wind_force_n = self.wind_model.force_at(state.time_s)

        thrust_direction_rad = state.pitch_rad + gimbal_angle_rad
        thrust_fx_n = thrust_n * np.sin(thrust_direction_rad)
        thrust_fy_n = thrust_n * np.cos(thrust_direction_rad)

        net_fx_n = thrust_fx_n + drag["drag_fx_n"] + wind_force_n
        net_fy_n = thrust_fy_n + drag["drag_fy_n"] - state.total_mass_kg * self.config.gravity_mps2

        ax_mps2 = net_fx_n / state.total_mass_kg
        ay_mps2 = net_fy_n / state.total_mass_kg

        engine_to_com_m = self.config.engine_position_m - state.center_of_mass_m
        control_torque_nm = thrust_n * np.sin(gimbal_angle_rad) * engine_to_com_m

        # The wind side load is applied near the upper body so the controller has a clean,
        # explainable disturbance to reject. The torque is intentionally scaled down because
        # this is not a high-fidelity aero moment model, only a readable disturbance source.
        wind_moment_arm_m = max(state.center_of_mass_m - self.config.center_of_pressure_m, 0.0)
        wind_torque_nm = wind_force_n * wind_moment_arm_m * self.config.wind_torque_scale

        damping_torque_nm = -self.config.rotational_damping_nms * state.angular_velocity_rad_s
        angular_acceleration_rad_s2 = (
            control_torque_nm + wind_torque_nm + damping_torque_nm
        ) / pitch_inertia_kgm2

        next_x_m = state.x_m + state.vx_mps * dt_s
        next_y_m = state.y_m + state.vy_mps * dt_s
        next_vx_mps = state.vx_mps + ax_mps2 * dt_s
        next_vy_mps = state.vy_mps + ay_mps2 * dt_s

        if next_y_m < 0.0 and next_vy_mps < 0.0:
            next_y_m = 0.0
            next_vy_mps = 0.0

        next_propellant_mass_kg = self.mass_properties.deplete_propellant(
            state.propellant_mass_kg,
            thrust_n,
            dt_s,
        )
        next_total_mass_kg = self.mass_properties.total_mass_kg(next_propellant_mass_kg)
        next_center_of_mass_m = self.mass_properties.center_of_mass_m(next_propellant_mass_kg)

        next_state = RocketState(
            time_s=state.time_s + dt_s,
            x_m=next_x_m,
            y_m=next_y_m,
            vx_mps=next_vx_mps,
            vy_mps=next_vy_mps,
            pitch_rad=state.pitch_rad + state.angular_velocity_rad_s * dt_s,
            angular_velocity_rad_s=state.angular_velocity_rad_s + angular_acceleration_rad_s2 * dt_s,
            dry_mass_kg=state.dry_mass_kg,
            propellant_mass_kg=next_propellant_mass_kg,
            total_mass_kg=next_total_mass_kg,
            center_of_mass_m=next_center_of_mass_m,
            thrust_n=self.thrust_curve.thrust_at(state.time_s + dt_s),
            gimbal_angle_rad=gimbal_angle_rad,
        )

        diagnostics = {
            "thrust_n": thrust_n,
            "drag_n": drag["drag_n"],
            "speed_mps": drag["speed_mps"],
            "alpha_deg": drag["alpha_deg"],
            "cd": drag["cd"],
            "ax_mps2": float(ax_mps2),
            "ay_mps2": float(ay_mps2),
            "wind_force_n": float(wind_force_n),
            "control_torque_nm": float(control_torque_nm),
            "wind_torque_nm": float(wind_torque_nm),
            "damping_torque_nm": float(damping_torque_nm),
            "pitch_inertia_kgm2": float(pitch_inertia_kgm2),
            "center_of_pressure_m": self.config.center_of_pressure_m,
            "engine_position_m": self.config.engine_position_m,
            "engine_to_com_m": float(engine_to_com_m),
            "stability_margin_m": float(self.config.center_of_pressure_m - state.center_of_mass_m),
            "thrust_direction_deg": float(np.degrees(thrust_direction_rad)),
        }
        return next_state, diagnostics
