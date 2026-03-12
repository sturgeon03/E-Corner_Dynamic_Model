# vehicle_sim package

`vehicle_sim` is the actual code package in this repository. It is organized around a full-vehicle model (`VehicleBody`) and an integrated corner model (`ECorner`).

## Package exports

The package-level API currently exposes:

```python
from vehicle_sim import VehicleBody, ECorner
from vehicle_sim.controllers import (
    ActiveAntiRollBarController,
    ActiveAntiRollBarGains,
    LateralYawRateTorqueController,
    create_lateral_torque_stepper,
)
from vehicle_sim import scenarios
```

Package version: `0.1.0`

## Core architecture

The model hierarchy is:

1. `VehicleBody` owns four corners labeled `FL`, `FR`, `RL`, `RR`.
2. Each `ECorner` owns six submodels:
   - `SteeringModel`
   - `BrakeModel`
   - `DriveModel`
   - `SuspensionModel`
   - `LongitudinalTireModel`
   - `LateralTireModel`
3. `VehicleBody.update(...)` computes wheel-center kinematics from the current body state.
4. Each corner consumes actuator commands, wheel-center velocities, body motion, and road input.
5. The four corner outputs are assembled into total body forces and moments, then integrated back into the vehicle state.

In practical terms, the data flow is:

- steering torque command -> steering angle
- brake torque command -> clamp force
- drive torque command + previous tire force + brake clamp force -> wheel speed
- body motion + active suspension torque + road profile -> suspension force and vertical tire load
- wheel speed + local wheel velocities + vertical load -> longitudinal and lateral tire forces
- four-corner force set -> full-vehicle accelerations, rates, and positions

## Main modules

### `models/vehicle_body/vehicle_body.py`

The full-vehicle model integrates a 12-state rigid-body representation:

- position: `x`, `y`, `heave`
- attitude: `roll`, `pitch`, `yaw`
- linear velocity: `velocity_x`, `velocity_y`, `heave_dot`
- angular velocity: `roll_rate`, `pitch_rate`, `yaw_rate`

Important behavior:

- loads default parameters from YAML when no explicit parameter object is passed
- computes corner positions and wheel-center velocities from body motion
- calls all four `ECorner` instances every step
- assembles total forces and moments, including gravity handling and rotational dynamics
- exposes `get_state_vector()` and `set_state_vector(...)` for direct state access

### `models/e_corner/e_corner.py`

`ECorner` is the integration point for one wheel station. Its update contract is:

```python
F_s, F_x_tire, F_y_tire = corner.update(
    dt,
    T_steer,
    T_brk,
    T_Drv,
    T_susp,
    V_wheel_x,
    V_wheel_y,
    X_body,
    z_road=0.0,
    z_road_dot=0.0,
    direction=1,
)
```

Where:

- `X_body` is `[heave, roll, pitch, heave_dot, roll_rate, pitch_rate]`
- `direction=1` means forward and `direction=-1` means reverse
- returned values are suspension force, longitudinal tire force, and lateral tire force for that corner

The corner state stores:

- `F_s`
- `F_x_tire`
- `F_y_tire`
- `F_z`
- `steering_angle`
- `omega_wheel`

### `models/e_corner/steering/steering_model.py`

Steering is modeled as a dynamic actuator with:

- inertia `J_cq`
- damping `B_cq`
- gear ratio from motor torque to steering-axis torque
- asymmetric left/right wheel-angle limits loaded from YAML
- steering-rate limits

The model also feeds back self-aligning torque from the lateral tire model.

### `models/e_corner/drive/`

Drive and brake are split into two models:

- `BrakeModel`: converts brake torque command into clamp force with actuator dynamics
- `DriveModel`: updates wheel angular speed using drive torque, longitudinal tire force, wheel inertia, wheel damping, and braking effect

The wheel and brake parameterization comes from the shared YAML file, including effective tire radius, wheel inertia, viscous drag, and brake hardware constants.

### `models/e_corner/suspension/suspension_model.py`

The suspension model carries the vertical dynamics around one corner, including:

- sprung/unsprung interaction
- spring force
- split rebound/compression damper coefficients
- active suspension force derived from suspension actuator torque
- vertical tire stiffness and damping
- stroke and tire-deflection limiting logic

This module is also where the corner's static equilibrium quantities are established and reset.

### `models/e_corner/tire/`

Two separate tire models are used:

- `longitudinal/longitudinal_tire.py`
  - computes slip ratio from wheel angular speed and local wheel longitudinal velocity
  - computes longitudinal force with a stiffness term and friction saturation
- `lateral/lateral_tire.py`
  - computes slip angle from local wheel velocities
  - computes lateral force with cornering stiffness and friction saturation
  - computes self-aligning torque via pneumatic trail

### `controllers/active_anti_roll_bar_controller.py`

This controller:

- takes left/right suspension stroke and stroke-rate differences at front and rear
- computes anti-roll moment with PD-like gains
- maps the result to either per-corner force or suspension actuator torque

### `controllers/lateral_feature/`

This package now also contains an integrated lateral controller chain:

- yaw-rate command -> yaw moment feedforward + PID feedback
- yaw moment -> per-wheel lateral force allocation
- per-wheel lateral force -> steering-angle feedforward
- steering-angle tracking PID -> per-wheel steering torque command

For simplified usage, use:

- `LateralYawRateTorqueController` (object API)
- `create_lateral_torque_stepper(...)` (one-line step function API)

### `scenarios/`

Despite the directory name, the current `scenarios` package is not yet a full scenario-runner framework. What is already implemented is:

- `base_scenario.py`
  - `TrajectoryPoint`
  - `Trajectory`
  - `path_to_trajectory(...)`
- `paths/sine_path.py`
  - sine-path generation
  - curvature and curvature-rate visualization helpers

This is useful for path definition and offline trajectory generation, but it does not yet replace a full simulation orchestration layer.

### `utils/`

Current utility status:

- `config_loader.py`: implemented and used across the package
- `math_utils.py`: placeholder
- `coordinate_transform.py`: placeholder

## Configuration model

Most constructors accept `config_path`. If omitted, they load:

- `vehicle_sim/models/params/vehicle_standard.yaml`

The central YAML file currently contains:

- `vehicle_spec`
  - wheel geometry and inertia
  - wheelbase, track width, axle load split
  - corner offsets relative to CG
- `brake`
- `drive`
- `suspension`
- `steering`
- `tire`
- `unsprung`
- `physics`
- `vehicle_body`

This makes the repository parameter-driven: most model instances can be rebuilt against a custom YAML file without editing Python code.

## Minimal examples

### Full vehicle step

```python
from vehicle_sim import VehicleBody

vehicle = VehicleBody()

corner_inputs = {
    label: {
        "T_steer": 0.0,
        "T_brk": 0.0,
        "T_Drv": 0.0,
        "T_susp": 0.0,
        "z_road": 0.0,
        "z_road_dot": 0.0,
    }
    for label in vehicle.wheel_labels
}

vehicle.update(dt=0.001, corner_inputs=corner_inputs)
print(vehicle.get_state_vector())
```

### Single-corner step

```python
import numpy as np

from vehicle_sim import ECorner

corner = ECorner(corner_id="FL")

X_body = np.zeros(6)

F_s, F_x, F_y = corner.update(
    dt=0.001,
    T_steer=0.0,
    T_brk=0.0,
    T_Drv=0.0,
    T_susp=0.0,
    V_wheel_x=10.0,
    V_wheel_y=0.0,
    X_body=X_body,
    z_road=0.0,
    z_road_dot=0.0,
    direction=1,
)

print(F_s, F_x, F_y)
print(corner.get_state())
```

### One-line lateral controller step

```python
from vehicle_sim import VehicleBody, create_lateral_torque_stepper

vehicle = VehicleBody()
step_lateral = create_lateral_torque_stepper(vehicle_body=vehicle, dt=0.001)

# One-line control call (returns {"FL": T, "FR": T, "RR": T, "RL": T} in N*m)
T_steer_cmd = step_lateral(yaw_rate_cmd=0.15)
```

## Conventions and assumptions

- corner labels use `FL`, `FR`, `RL`, `RR`
- vehicle geometry in the YAML uses body-frame corner offsets with `x` forward and `y` left
- steering limits are asymmetric and depend on left/right side
- several models rely on the central YAML as the source of truth rather than constructor-time hardcoding

## Current limitations

The codebase contains both implemented models and unfinished integration work. Important caveats:

- `vehicle_sim/main.py` still refers to `VehicleSimulator`, `DriverController`, and scenario classes that are not present in the current tree
- older README content in the repository described files and flows that no longer match the codebase
- `models/e_corner/config/corner_config.py` is a scaffold and is not wired into the active model path
- `utils/math_utils.py` and `utils/coordinate_transform.py` are TODO-only placeholders
- many validation assets live in `test_debug/` notebooks rather than in a formal automated test suite

## Suggested next documentation targets

If this package keeps growing, the next useful documentation split would be:

1. `models/README.md` for model equations and state conventions
2. `scenarios/README.md` once scenario execution becomes a real subsystem
3. `docs/` for derivations, validation reports, and parameter provenance
