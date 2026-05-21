from __future__ import annotations

import numpy as np

from rocket_sim.control.sensors import AttitudeMeasurement


class FlightController:
    """PID-based flight controller that requests a pitch-correcting gimbal angle."""

    def __init__(
        self,
        target_pitch_deg: float,
        kp: float,
        ki: float,
        kd: float,
        integral_limit_rad: float,
    ) -> None:
        self.target_pitch_rad = np.radians(target_pitch_deg)
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral_limit_rad = integral_limit_rad
        self.integral_error = 0.0

    def command(self, measurement: AttitudeMeasurement, dt_s: float) -> float:
        """Return the desired gimbal angle before actuator limits are applied."""

        error_rad = self.target_pitch_rad - measurement.pitch_rad
        self.integral_error += error_rad * dt_s
        self.integral_error = float(
            np.clip(
                self.integral_error,
                -self.integral_limit_rad,
                self.integral_limit_rad,
            )
        )

        # The D-term is driven by measured angular velocity instead of a perfect derivative.
        # That keeps the controller logic close to what a real attitude loop would consume.
        desired_gimbal_angle_rad = (
            self.kp * error_rad
            + self.ki * self.integral_error
            - self.kd * measurement.angular_velocity_rad_s
        )
        return float(desired_gimbal_angle_rad)
