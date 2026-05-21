# Rocket Systems Analysis Framework

This project is a control-oriented 2D rocket simulation and analysis framework. It is designed for software engineering and student rocketry interviews where the goal is to demonstrate clean architecture, timestep simulation thinking, telemetry analysis, and control-loop reasoning without drifting into full aerospace research code.

The simulator is intentionally simplified. It focuses on:

- closed-loop pitch stabilization
- timestep-based state updates
- thrust vector control
- center-of-mass migration during tank drain
- simplified stability intuition
- wind disturbance rejection
- noisy sensor-driven control
- telemetry export and engineering visualizations
- Monte Carlo robustness analysis

## Project Structure

```text
.
├── main.py
├── README.md
├── requirements.txt
├── rocket_sim/
│   ├── control/
│   │   ├── gimbal_actuator.py
│   │   ├── pid_controller.py
│   │   └── sensors.py
│   ├── data/
│   │   └── rocket_config.py
│   ├── physics/
│   │   ├── aerodynamics.py
│   │   ├── mass_properties.py
│   │   ├── physics_engine.py
│   │   ├── rocket_state.py
│   │   ├── thrust_curve.py
│   │   └── wind_model.py
│   ├── simulation/
│   │   ├── monte_carlo.py
│   │   └── simulation_loop.py
│   ├── telemetry/
│   │   ├── exporter.py
│   │   └── recorder.py
│   └── visualization/
│       ├── heatmaps.py
│       ├── monte_carlo.py
│       ├── plots.py
│       └── rocket_body.py
└── telemetry/
    └── simulation_output.csv
```

## How To Run

```bash
pip install -r requirements.txt
python main.py
```

The run generates:

- `telemetry/simulation_output.csv`
- `telemetry_plots.png`
- `rocket_body_visualization.png`
- `engine_workload_and_thrust_distribution.png`
- `monte_carlo_analysis.png`

## Architecture Diagram

```text
RocketState
    |
    v
AttitudeSensor ----> FlightController ----> GimbalActuator
    |                                         |
    +---------------- noisy measurements -----+
                                              |
                                              v
                                     PhysicsEngine
                                              |
                                              v
                                    TelemetryRecorder
                                              |
                                              v
                              CSV export + analysis visualizations
```

## Timestep Simulation Explanation

The simulation runs at:

- `dt = 0.01 s`
- `duration = 10.0 s`
- Forward Euler integration

Each timestep follows the same clear sequence:

1. Read the current state through a noisy sensor layer
2. Compute a pitch correction command with the PID controller
3. Clamp the command with actuator rate and angle limits
4. Advance the simplified physics by one timestep
5. Record telemetry for analysis and export

This is intentionally explicit because the timestep loop is the core systems-engineering talking point.

## Control Loop Explanation

The rocket starts with a small pitch disturbance and sees lateral wind loading during ascent. The controller receives noisy pitch and angular-rate measurements rather than perfect truth values. It then requests a gimbal correction to drive pitch back toward vertical.

That separation matters:

- the sensor layer represents avionics uncertainty
- the controller represents guidance and control logic
- the actuator represents hardware limitations
- the physics engine represents the plant being controlled

## Simplified Physics Assumptions

This project is intentionally not a full-fidelity rocket flight model. It uses:

- 2D translational motion
- one pitch angle and one angular rate
- a scalar pitch inertia
- a fixed-center approximate CoP for stability intuition
- a simple drag equation with angle-dependent `Cd`
- a fixed thrust curve interpolated over time
- a lateral wind disturbance model with optional gusts
- a simple rotational damping term

It deliberately does not include:

- full 6DOF rigid body dynamics
- quaternion math
- CFD-grade aerodynamics
- tensor inertia formulations
- detailed aerodynamic moment coefficients
- rail departure modeling
- state estimation or navigation filters

## Center of Mass and Stability Intuition

The dry structure stays fixed, but the fuel is modeled as a draining liquid column. As the tank empties, the fuel centroid moves lower toward the engine. That shifts the overall center of mass over time.

This matters because:

- the same thrust vector does not act on the exact same mass distribution throughout flight
- engine-to-CoM lever arm changes control effectiveness
- CoM movement changes the relationship between CoM and the approximate CoP

The project uses a simplified stability metric:

`stability_margin = CoP - CoM`

Because body positions are measured from the nose toward the engine, negative values mean the CoP is above the CoM, which is the usual intuitively stable arrangement for a rocket.

## Visualizations

The framework emphasizes engineering analysis outputs:

- multi-panel time-series telemetry plots
- rocket body snapshots showing CoM, CoP, engine, and thrust-vector direction
- a thrust distribution heatmap along the vehicle body
- a gimbal workload heatmap weighted by thrust
- Monte Carlo trajectory spread and stability variability plots

## Where Real Rocket Data Normally Comes From

In a higher-fidelity program, these model inputs would usually come from:

- static fire tests
- manufacturer thrust curves
- OpenRocket
- RASAero
- CFD tools
- telemetry logs from real flights
- measured mass property and balance data

This project uses compact hardcoded approximations so the software structure stays easy to inspect.

## Future Improvements

Reasonable next steps, while still preserving readability, would include:

- importing thrust curves and aero tables from OpenRocket exports
- higher-fidelity aerodynamic models
- a state estimator or EKF for noisy sensors
- controller gain scheduling as mass changes
- full 6DOF rigid body dynamics
- actuator lag and servo current models
- flight replay overlays using real telemetry logs

## Positioning

The right way to describe this project is:

> a control-oriented rocket systems analysis and visualization framework

That is the intent. The code emphasizes readable architecture, explainable control loops, and engineering intuition rather than claiming research-grade flight prediction.
