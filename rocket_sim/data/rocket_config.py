from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class RocketConfig:
    """Configuration for the simplified rocket analysis framework."""

    dt_s: float = 0.01
    duration_s: float = 10.0
    random_seed: int = 42

    gravity_mps2: float = 9.81
    air_density_kgpm3: float = 1.225
    reference_area_m2: float = 0.05
    isp_s: float = 250.0

    dry_mass_kg: float = 15.0
    initial_propellant_mass_kg: float = 10.0

    # Body-axis distances are measured from the nose toward the engine.
    body_length_m: float = 4.4
    body_radius_m: float = 0.12
    dry_mass_position_m: float = 2.0
    tank_top_m: float = 1.4
    tank_bottom_m: float = 3.6
    center_of_pressure_m: float = 1.1
    engine_position_m: float = 4.2

    dry_pitch_inertia_kgm2: float = 14.0
    propellant_pitch_inertia_per_kg: float = 0.55
    rotational_damping_nms: float = 22.0

    initial_pitch_deg: float = 5.0
    target_pitch_deg: float = 0.0

    pid_kp: float = 0.18
    pid_ki: float = 0.06
    pid_kd: float = 0.11
    pid_integral_limit_rad: float = 0.75

    max_gimbal_deg: float = 8.0
    max_gimbal_rate_deg_per_s: float = 25.0

    thrust_scale: float = 1.0
    drag_scale: float = 1.0

    # Wind is modeled as a lateral body disturbance, not a full atmospheric model.
    wind_base_force_n: float = 5.0
    wind_gust_std_n: float = 1.0
    gust_interval_s: float = 0.5
    enable_random_gusts: bool = True
    wind_torque_scale: float = 0.05

    # Real avionics never receive perfect state values, so the controller reads noisy sensors.
    sensor_pitch_noise_std_deg: float = 0.15
    sensor_rate_noise_std_deg_s: float = 0.6

    monte_carlo_runs: int = 25
    monte_carlo_drag_sigma: float = 0.06
    monte_carlo_thrust_sigma: float = 0.05
    monte_carlo_wind_sigma_n: float = 2.0
