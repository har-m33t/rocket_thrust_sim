from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from rocket_sim.physics.rocket_state import RocketState


@dataclass
class AttitudeMeasurement:
    """Noisy measurement bundle provided to the flight controller."""

    pitch_rad: float
    angular_velocity_rad_s: float


class AttitudeSensor:
    """Produces noisy attitude measurements for the controller."""

    def __init__(
        self,
        pitch_noise_std_deg: float,
        rate_noise_std_deg_s: float,
        rng: np.random.Generator,
    ) -> None:
        self.pitch_noise_std_rad = np.radians(pitch_noise_std_deg)
        self.rate_noise_std_rad_s = np.radians(rate_noise_std_deg_s)
        self.rng = rng

    def measure(self, state: RocketState) -> AttitudeMeasurement:
        """Return a noisy estimate of pitch angle and angular velocity.

        Real flight computers never read perfect state values. They see sensors with
        noise, bias, and latency, so even a toy simulator benefits from showing that
        the controller is reacting to measurements rather than omniscient truth.
        """

        measured_pitch_rad = state.pitch_rad + float(
            self.rng.normal(0.0, self.pitch_noise_std_rad)
        )
        measured_angular_velocity_rad_s = state.angular_velocity_rad_s + float(
            self.rng.normal(0.0, self.rate_noise_std_rad_s)
        )
        return AttitudeMeasurement(
            pitch_rad=measured_pitch_rad,
            angular_velocity_rad_s=measured_angular_velocity_rad_s,
        )
