from __future__ import annotations

import numpy as np


class GimbalActuator:
    """Applies rate and saturation limits to the requested gimbal angle."""

    def __init__(self, max_angle_deg: float, max_rate_deg_per_s: float) -> None:
        self.max_angle_rad = np.radians(max_angle_deg)
        self.max_rate_rad_s = np.radians(max_rate_deg_per_s)
        self.current_angle_rad = 0.0

    def apply(self, desired_angle_rad: float, dt_s: float) -> float:
        """Return the realizable gimbal angle after actuator constraints are enforced."""

        saturated_angle_rad = float(
            np.clip(desired_angle_rad, -self.max_angle_rad, self.max_angle_rad)
        )
        max_step_rad = self.max_rate_rad_s * dt_s
        next_angle_rad = float(
            np.clip(
                saturated_angle_rad,
                self.current_angle_rad - max_step_rad,
                self.current_angle_rad + max_step_rad,
            )
        )
        self.current_angle_rad = next_angle_rad
        return next_angle_rad
