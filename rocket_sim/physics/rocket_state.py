from __future__ import annotations

from dataclasses import dataclass


@dataclass
class RocketState:
    """Single source of truth for the vehicle at one discrete simulation step."""

    time_s: float
    x_m: float
    y_m: float
    vx_mps: float
    vy_mps: float
    pitch_rad: float
    angular_velocity_rad_s: float
    dry_mass_kg: float
    propellant_mass_kg: float
    total_mass_kg: float
    center_of_mass_m: float
    thrust_n: float
    gimbal_angle_rad: float
