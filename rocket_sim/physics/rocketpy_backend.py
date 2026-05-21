from __future__ import annotations

from typing import Dict, Tuple

import numpy as np

from rocket_sim.data.rocket_config import RocketConfig
from rocket_sim.physics.rocket_state import RocketState
from rocketpy import Environment, Flight, GenericMotor, Rocket


_THRUST_CURVE_POINTS = [
    (0.0, 400.0),
    (0.5, 800.0),
    (2.0, 1200.0),
    (8.0, 1200.0),
    (8.5, 0.0),
]
_BURN_TIME_S = 8.5


class RocketPyBackend:
    """Thin adapter that replaces the custom physics engine with RocketPy as the state source."""

    def __init__(self, config: RocketConfig) -> None:
        self.config = config

        environment = Environment(latitude=0.0, longitude=0.0, elevation=0.0)
        environment.set_atmospheric_model(type="standard_atmosphere")
        if config.wind_base_force_n != 0.0:
            wind_speed_mps = float(
                np.sign(config.wind_base_force_n)
                * np.sqrt(
                    2.0 * abs(config.wind_base_force_n)
                    / max(config.air_density_kgpm3 * config.reference_area_m2, 1e-9)
                )
            )
            environment.set_atmospheric_model(
                type="custom_atmosphere",
                wind_u=wind_speed_mps,
                wind_v=0.0,
            )
        self.environment = environment

        scaled_thrust_points = [(t, F * config.thrust_scale) for t, F in _THRUST_CURVE_POINTS]
        tank_height_m = max(config.tank_bottom_m - config.tank_top_m, 0.1)
        tank_center_m = 0.5 * (config.tank_top_m + config.tank_bottom_m)
        nozzle_offset_m = config.engine_position_m - tank_center_m
        motor = GenericMotor(
            thrust_source=scaled_thrust_points,
            burn_time=(0.0, _BURN_TIME_S),
            chamber_radius=config.body_radius_m * 0.6,
            chamber_height=tank_height_m,
            chamber_position=0.0,
            propellant_initial_mass=config.initial_propellant_mass_kg,
            nozzle_radius=config.body_radius_m * 0.3,
            nozzle_position=nozzle_offset_m,
            dry_mass=0.5,
            dry_inertia=(0.01, 0.01, 0.001),
            center_of_dry_mass_position=0.0,
            coordinate_system_orientation="combustion_chamber_to_nozzle",
        )
        self.motor = motor

        rocket_dry_mass_kg = max(config.dry_mass_kg - 0.5, 0.1)
        rocket = Rocket(
            radius=config.body_radius_m,
            mass=rocket_dry_mass_kg,
            inertia=(
                config.dry_pitch_inertia_kgm2,
                config.dry_pitch_inertia_kgm2,
                0.5 * config.dry_pitch_inertia_kgm2 * (config.body_radius_m ** 2),
            ),
            power_off_drag=0.3 * config.drag_scale,
            power_on_drag=0.3 * config.drag_scale,
            center_of_mass_without_motor=config.dry_mass_position_m,
            coordinate_system_orientation="nose_to_tail",
        )
        rocket.add_motor(motor, position=tank_center_m)
        rocket.add_nose(length=0.25, kind="vonKarman", position=0.0)
        rocket.add_trapezoidal_fins(
            n=3,
            root_chord=0.2,
            tip_chord=0.1,
            span=0.15,
            position=config.body_length_m - 0.3,
        )
        self.rocket = rocket

        flight = Flight(
            rocket=rocket,
            environment=environment,
            rail_length=0.5,
            inclination=90.0 - config.initial_pitch_deg,
            heading=0.0,
            terminate_on_apogee=False,
            max_time=config.duration_s + 1.0,
            verbose=False,
        )
        self.flight = flight
        self._t_final = float(flight.t_final)

    def _clamp_time(self, t: float) -> float:
        return float(max(0.0, min(t, self._t_final)))

    def _query(self, t: float) -> Tuple[RocketState, Dict[str, float]]:
        cfg = self.config
        flight = self.flight
        t_query = self._clamp_time(t)

        x_m = float(flight.x(t_query))
        altitude_m = float(flight.z(t_query))
        vx_mps = float(flight.vx(t_query))
        vy_mps = float(flight.vz(t_query))
        ax_mps2 = float(flight.ax(t_query))
        ay_mps2 = float(flight.az(t_query))

        attitude_deg = float(flight.attitude_angle(t_query))
        pitch_rad = float(np.radians(90.0 - attitude_deg))

        angular_velocity_rad_s = float(flight.w2(t_query))

        total_mass_kg = float(self.rocket.total_mass(t_query))
        propellant_mass_kg = float(self.motor.propellant_mass(t_query))
        center_of_mass_m = float(self.rocket.center_of_mass(t_query))
        thrust_n = float(self.motor.thrust(t_query))

        speed_mps = float(flight.speed(t_query))
        alpha_deg = float(flight.angle_of_attack(t_query))
        drag_n = float(flight.aerodynamic_drag(t_query))

        state = RocketState(
            time_s=t,
            x_m=x_m,
            y_m=altitude_m,
            vx_mps=vx_mps,
            vy_mps=vy_mps,
            pitch_rad=pitch_rad,
            angular_velocity_rad_s=angular_velocity_rad_s,
            dry_mass_kg=cfg.dry_mass_kg,
            propellant_mass_kg=propellant_mass_kg,
            total_mass_kg=total_mass_kg,
            center_of_mass_m=center_of_mass_m,
            thrust_n=thrust_n,
            gimbal_angle_rad=0.0,
        )

        pitch_inertia_kgm2 = (
            cfg.dry_pitch_inertia_kgm2
            + cfg.propellant_pitch_inertia_per_kg * propellant_mass_kg
        )
        diagnostics = {
            "thrust_n": thrust_n,
            "drag_n": drag_n,
            "speed_mps": speed_mps,
            "alpha_deg": alpha_deg,
            "cd": 0.3 * cfg.drag_scale,
            "ax_mps2": ax_mps2,
            "ay_mps2": ay_mps2,
            "wind_force_n": float(cfg.wind_base_force_n),
            "control_torque_nm": 0.0,
            "wind_torque_nm": 0.0,
            "damping_torque_nm": 0.0,
            "pitch_inertia_kgm2": float(pitch_inertia_kgm2),
            "center_of_pressure_m": cfg.center_of_pressure_m,
            "engine_position_m": cfg.engine_position_m,
            "engine_to_com_m": float(cfg.engine_position_m - center_of_mass_m),
            "stability_margin_m": float(cfg.center_of_pressure_m - center_of_mass_m),
            "thrust_direction_deg": float(np.degrees(pitch_rad)),
        }
        return state, diagnostics

    def initial_state(self) -> RocketState:
        state, _ = self._query(0.0)
        return state

    def get_state(self, t: float) -> RocketState:
        state, _ = self._query(t)
        return state

    def update(
        self,
        state: RocketState,
        gimbal_angle_rad: float,
        dt_s: float,
    ) -> Tuple[RocketState, Dict[str, float]]:
        next_state, diagnostics = self._query(state.time_s + dt_s)
        next_state.gimbal_angle_rad = gimbal_angle_rad
        diagnostics["thrust_direction_deg"] = float(
            np.degrees(next_state.pitch_rad + gimbal_angle_rad)
        )
        return next_state, diagnostics
