from __future__ import annotations

from typing import Dict

import numpy as np


class Aerodynamics:
    """Computes a simple drag force with an angle-dependent drag coefficient."""

    def __init__(
        self,
        air_density_kgpm3: float,
        reference_area_m2: float,
        drag_scale: float = 1.0,
    ) -> None:
        self.air_density_kgpm3 = air_density_kgpm3
        self.reference_area_m2 = reference_area_m2
        self.drag_scale = drag_scale
        self.angles_deg = np.array([0.0, 15.0, 45.0, 90.0], dtype=float)
        self.cd_values = np.array([0.3, 0.4, 0.8, 1.2], dtype=float)

    def angle_of_attack_deg(self, vx_mps: float, vy_mps: float, pitch_rad: float) -> float:
        """Estimate angle of attack from body pitch and velocity direction."""

        speed_mps = float(np.hypot(vx_mps, vy_mps))
        if speed_mps < 1e-9:
            return 0.0

        velocity_angle_rad = float(np.arctan2(vx_mps, vy_mps))
        angle_error_rad = float(
            np.arctan2(
                np.sin(pitch_rad - velocity_angle_rad),
                np.cos(pitch_rad - velocity_angle_rad),
            )
        )
        return float(np.clip(abs(np.degrees(angle_error_rad)), 0.0, 90.0))

    def drag_force(self, vx_mps: float, vy_mps: float, pitch_rad: float) -> Dict[str, float]:
        """Return drag force components and the intermediate aerodynamic terms."""

        speed_mps = float(np.hypot(vx_mps, vy_mps))
        if speed_mps < 1e-9:
            return {
                "drag_fx_n": 0.0,
                "drag_fy_n": 0.0,
                "drag_n": 0.0,
                "speed_mps": 0.0,
                "alpha_deg": 0.0,
                "cd": self.cd_values[0] * self.drag_scale,
            }

        alpha_deg = self.angle_of_attack_deg(vx_mps, vy_mps, pitch_rad)
        cd = float(np.interp(alpha_deg, self.angles_deg, self.cd_values)) * self.drag_scale
        dynamic_pressure_pa = 0.5 * self.air_density_kgpm3 * speed_mps**2
        drag_n = dynamic_pressure_pa * cd * self.reference_area_m2

        return {
            "drag_fx_n": float(-drag_n * vx_mps / speed_mps),
            "drag_fy_n": float(-drag_n * vy_mps / speed_mps),
            "drag_n": float(drag_n),
            "speed_mps": speed_mps,
            "alpha_deg": alpha_deg,
            "cd": cd,
        }
