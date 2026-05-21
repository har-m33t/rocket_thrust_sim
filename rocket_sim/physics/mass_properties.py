from __future__ import annotations

from rocket_sim.data.rocket_config import RocketConfig


class MassProperties:
    """Tracks mass depletion, center-of-mass migration, and scalar pitch inertia."""

    def __init__(self, config: RocketConfig) -> None:
        self.config = config

    def total_mass_kg(self, propellant_mass_kg: float) -> float:
        """Return dry mass plus remaining propellant mass."""

        return self.config.dry_mass_kg + propellant_mass_kg

    def propellant_centroid_m(self, propellant_mass_kg: float) -> float:
        """Return the centroid of the remaining liquid propellant column."""

        tank_height_m = self.config.tank_bottom_m - self.config.tank_top_m
        if self.config.initial_propellant_mass_kg <= 0.0 or tank_height_m <= 0.0:
            return self.config.tank_bottom_m

        fill_fraction = max(
            0.0,
            min(1.0, propellant_mass_kg / self.config.initial_propellant_mass_kg),
        )
        if fill_fraction <= 1e-9:
            return self.config.tank_bottom_m

        # As liquid propellant drains, the remaining mass stays pooled lower in the tank.
        # That slides the fuel centroid toward the engine and shifts the full-vehicle CoM.
        filled_height_m = tank_height_m * fill_fraction
        return self.config.tank_bottom_m - 0.5 * filled_height_m

    def center_of_mass_m(self, propellant_mass_kg: float) -> float:
        """Compute the weighted-average center of mass of the vehicle."""

        propellant_centroid_m = self.propellant_centroid_m(propellant_mass_kg)
        total_mass_kg = self.total_mass_kg(propellant_mass_kg)
        if total_mass_kg <= 0.0:
            return self.config.dry_mass_position_m

        # Rockets care about CoM migration because it changes both passive stability intuition
        # and control leverage. As fuel burns away, the same gimbal angle acts on a slightly
        # different mass distribution than it did at liftoff.
        weighted_sum = (
            self.config.dry_mass_kg * self.config.dry_mass_position_m
            + propellant_mass_kg * propellant_centroid_m
        )
        return weighted_sum / total_mass_kg

    def pitch_inertia_kgm2(self, propellant_mass_kg: float) -> float:
        """Return a simplified scalar pitch inertia."""

        return (
            self.config.dry_pitch_inertia_kgm2
            + self.config.propellant_pitch_inertia_per_kg * propellant_mass_kg
        )

    def mass_flow_rate_kgps(self, thrust_n: float) -> float:
        """Convert thrust into fuel burn rate with a simple Isp approximation."""

        return thrust_n / (self.config.isp_s * self.config.gravity_mps2)

    def deplete_propellant(self, propellant_mass_kg: float, thrust_n: float, dt_s: float) -> float:
        """Integrate propellant burn using Forward Euler."""

        next_propellant_mass_kg = propellant_mass_kg - self.mass_flow_rate_kgps(thrust_n) * dt_s
        return max(0.0, next_propellant_mass_kg)
