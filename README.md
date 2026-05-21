# Rocket Systems Analysis Framework

A control-oriented rocket simulation and analysis framework. The flight dynamics are powered by [RocketPy](https://github.com/RocketPy-Team/RocketPy) — an open-source 6-DOF rocket trajectory simulator — and a thin adapter layer maps RocketPy's state into the existing closed-loop control, sensor, telemetry, and visualization pipeline.

The project demonstrates clean architecture, timestep simulation thinking, telemetry analysis, and control-loop reasoning while delegating the physics to a production-grade trajectory solver.

It focuses on:

- closed-loop pitch stabilization with a PID controller
- timestep-based telemetry collection driven by RocketPy state queries
- thrust vector control commands through a rate-limited gimbal actuator
- center-of-mass migration during propellant burn (via RocketPy's mass model)
- simplified stability intuition (CoP - CoM margin)
- noisy sensor-driven control
- telemetry export and engineering visualizations
- Monte Carlo robustness analysis across drag, thrust, and wind variations

## Project Structure

```text
.
├── main.py                       # entry point: runs the simulation, Monte Carlo, and saves all artifacts
├── README.md
├── requirements.txt
├── rocket_sim/
│   ├── control/
│   │   ├── gimbal_actuator.py    # angle + rate-limited gimbal actuator
│   │   ├── pid_controller.py     # PID attitude controller
│   │   └── sensors.py            # noisy attitude sensor
│   ├── data/
│   │   └── rocket_config.py      # all tunable rocket and simulation parameters
│   ├── physics/
│   │   ├── rocket_state.py       # state dataclass shared by every module
│   │   └── rocketpy_backend.py   # thin adapter: builds RocketPy Environment/Motor/Rocket/Flight and queries state
│   ├── simulation/
│   │   ├── monte_carlo.py        # Monte Carlo runner over drag/thrust/wind perturbations
│   │   └── simulation_loop.py    # closed-loop timestep loop wiring sensor → controller → actuator → physics → telemetry
│   ├── telemetry/
│   │   ├── exporter.py           # CSV exporter
│   │   └── recorder.py           # per-step telemetry collector
│   └── visualization/
│       ├── heatmaps.py           # engine workload / thrust distribution heatmaps
│       ├── monte_carlo.py        # trajectory spread + altitude distribution + stability variability plots
│       ├── plots.py              # multi-panel telemetry line plots
│       └── rocket_body.py        # rocket body snapshots with CoM, CoP, engine, thrust vector
└── telemetry/
    └── simulation_output.csv     # generated per run
```

## How To Run

```bash
pip install -r requirements.txt
python main.py
```

The run generates:

- `telemetry/simulation_output.csv` — per-step telemetry
- `telemetry_plots.png` — closed-loop ascent telemetry
- `rocket_body_visualization.png` — body snapshots with CoM/CoP/engine/thrust vector
- `engine_workload_and_thrust_distribution.png` — thrust and gimbal workload heatmaps
- `monte_carlo_analysis.png` — Monte Carlo trajectory and stability spreads

## Architecture Diagram

```text
                            ┌─────────────────────────────┐
                            │      RocketConfig           │
                            └──────────────┬──────────────┘
                                           │
                                           v
┌──────────────────┐         ┌─────────────────────────────┐
│  RocketPyBackend │ ◄─────► │  Environment / Motor /      │
│  (adapter)       │         │  Rocket / Flight (RocketPy) │
└────────┬─────────┘         └─────────────────────────────┘
         │ get_state(t), update(state, gimbal, dt)
         v
   RocketState  ────►  AttitudeSensor  ────►  FlightController  ────►  GimbalActuator
                                                                              │
                                                                              v
                                                                       TelemetryRecorder
                                                                              │
                                                                              v
                                                          CSV export + analysis visualizations
```

## Closed-Loop Timestep

The simulation runs at:

- `dt = 0.01 s`
- `duration = 10.0 s`

Each timestep follows the same explicit sequence:

1. Read the current state through a noisy sensor layer
2. Compute a pitch correction command with the PID controller
3. Clamp the command with actuator rate and angle limits
4. Query the RocketPy-backed physics at `t + dt` and map the result into `RocketState`
5. Record telemetry for analysis and export

RocketPy integrates the full 6-DOF trajectory up front; the loop then samples that solution at each timestep and threads it through the controller, sensor, actuator, and telemetry layers.

## Control Loop

The rocket starts with a small pitch disturbance and sees aerodynamic and wind loading during ascent. The controller receives noisy pitch and angular-rate measurements rather than perfect truth values. It then requests a gimbal correction to drive pitch back toward vertical.

That separation matters:

- the sensor layer represents avionics uncertainty
- the controller represents guidance and control logic
- the actuator represents hardware limitations
- the physics backend represents the plant being controlled

## Physics Backend (RocketPy)

The `RocketPyBackend` class is the only physics code in the project. It:

- constructs a RocketPy `Environment` (standard atmosphere, optional lateral wind)
- constructs a RocketPy `GenericMotor` from the configured thrust curve and propellant mass
- constructs a RocketPy `Rocket` with the configured radius, mass, inertia, drag coefficients, and aero surfaces
- runs a single `Flight` integration covering the configured duration
- exposes `initial_state()`, `get_state(t)`, and `update(state, gimbal_angle_rad, dt_s)` to the rest of the codebase

Everything previously handled by hand — thrust computation, drag, acceleration updates, rotational dynamics, position/velocity integration, mass depletion, timestep propagation — is delegated to RocketPy.

## Center of Mass and Stability Intuition

RocketPy tracks center-of-mass migration as propellant burns. The project still surfaces a simplified stability metric for visualization:

`stability_margin = CoP - CoM`

Because body positions are measured from the nose toward the engine, negative values mean the CoP is above the CoM, which is the usual intuitively stable arrangement for a rocket.

## Visualizations

The framework emphasizes engineering analysis outputs:

- multi-panel time-series telemetry plots
- rocket body snapshots showing CoM, CoP, engine, and thrust-vector direction
- a thrust distribution heatmap along the vehicle body
- a gimbal workload heatmap weighted by thrust
- Monte Carlo trajectory spread and stability variability plots

## Monte Carlo Analysis

The Monte Carlo runner perturbs:

- drag scale
- thrust scale
- base wind force

across multiple runs, each with a different RNG seed. For each run it captures the full trajectory and a summary row (max altitude, final lateral position, max pitch, minimum stability margin). The visualization layer plots the trajectory spread, altitude distribution, and stability variability.

## Future Improvements

Reasonable next steps include:

- closing the control loop back into RocketPy via its controller API so gimbal commands actually drive the integrated trajectory
- importing thrust curves and aero tables directly from OpenRocket / RASAero exports
- a state estimator or EKF for noisy sensors
- controller gain scheduling as mass changes
- actuator lag and servo current models
- flight replay overlays using real telemetry logs

## Positioning

The right way to describe this project is:

> a control-oriented rocket systems analysis and visualization framework on top of RocketPy

The code emphasizes readable architecture, explainable control loops, and engineering intuition while leaning on RocketPy for trajectory fidelity.
