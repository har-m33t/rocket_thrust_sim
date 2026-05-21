from __future__ import annotations

import numpy as np
import pandas as pd

from rocket_sim.control.gimbal_actuator import GimbalActuator
from rocket_sim.control.pid_controller import FlightController
from rocket_sim.control.sensors import AttitudeSensor
from rocket_sim.data.rocket_config import RocketConfig
from rocket_sim.physics.aerodynamics import Aerodynamics
from rocket_sim.physics.mass_properties import MassProperties
from rocket_sim.physics.physics_engine import PhysicsEngine
from rocket_sim.physics.thrust_curve import ThrustCurve
from rocket_sim.physics.wind_model import WindModel
from rocket_sim.telemetry.recorder import TelemetryRecorder


class SimulationLoop:
    """Owns the controller, actuator, physics engine, and timestep orchestration."""

    def __init__(self, config: RocketConfig, seed: int | None = None) -> None:
        self.config = config
        self.rng = np.random.default_rng(config.random_seed if seed is None else seed)

        thrust_curve = ThrustCurve(thrust_scale=config.thrust_scale)
        aerodynamics = Aerodynamics(
            air_density_kgpm3=config.air_density_kgpm3,
            reference_area_m2=config.reference_area_m2,
            drag_scale=config.drag_scale,
        )
        mass_properties = MassProperties(config)
        wind_model = WindModel(
            base_force_n=config.wind_base_force_n,
            gust_std_n=config.wind_gust_std_n,
            gust_interval_s=config.gust_interval_s,
            enable_random_gusts=config.enable_random_gusts,
            rng=self.rng,
        )

        self.physics = PhysicsEngine(
            config=config,
            thrust_curve=thrust_curve,
            aerodynamics=aerodynamics,
            mass_properties=mass_properties,
            wind_model=wind_model,
        )
        self.controller = FlightController(
            target_pitch_deg=config.target_pitch_deg,
            kp=config.pid_kp,
            ki=config.pid_ki,
            kd=config.pid_kd,
            integral_limit_rad=config.pid_integral_limit_rad,
        )
        self.actuator = GimbalActuator(
            max_angle_deg=config.max_gimbal_deg,
            max_rate_deg_per_s=config.max_gimbal_rate_deg_per_s,
        )
        self.sensor = AttitudeSensor(
            pitch_noise_std_deg=config.sensor_pitch_noise_std_deg,
            rate_noise_std_deg_s=config.sensor_rate_noise_std_deg_s,
            rng=self.rng,
        )
        self.telemetry = TelemetryRecorder()

    def run(self) -> pd.DataFrame:
        """Run the closed-loop simulation and return the collected telemetry."""

        state = self.physics.initial_state()
        num_steps = int(self.config.duration_s / self.config.dt_s)

        for _ in range(num_steps):
            # 1. Read the current vehicle state through a noisy sensor abstraction.
            measurement = self.sensor.measure(state)

            # 2. Ask the controller how much gimbal deflection it wants to reduce pitch error.
            desired_gimbal_angle_rad = self.controller.command(measurement, self.config.dt_s)

            # 3. Pass the request through the actuator so the command respects hardware limits.
            actual_gimbal_angle_rad = self.actuator.apply(
                desired_gimbal_angle_rad,
                self.config.dt_s,
            )

            # 4. Advance the physics using the realizable actuator command and disturbances.
            next_state, diagnostics = self.physics.update(
                state,
                actual_gimbal_angle_rad,
                self.config.dt_s,
            )

            # 5. Store telemetry before moving the simulation clock forward.
            self.telemetry.record(
                state=state,
                measurement=measurement,
                desired_gimbal_angle_rad=desired_gimbal_angle_rad,
                actual_gimbal_angle_rad=actual_gimbal_angle_rad,
                diagnostics=diagnostics,
            )
            state = next_state

        return self.telemetry.to_dataframe()
