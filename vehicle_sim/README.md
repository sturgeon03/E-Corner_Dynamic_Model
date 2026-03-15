# vehicle_sim package

`vehicle_sim` is the Python package in this repository for E-corner-based vehicle dynamics simulation.

## Public package exports

The package-level API currently exports:

```python
from vehicle_sim import (
    VehicleBody,
    ECorner,
    ActiveAntiRollBarController,
    ActiveAntiRollBarGains,
    scenarios,
)
```

## Current package layout

```text
vehicle_sim/
  __init__.py
  models/
    vehicle_body/
    e_corner/
    params/vehicle_standard.yaml
  controllers/
    anti_roll_bar_controll/
    yaw_rate_steering_controller/
  scenarios/
    sinesweep/
  utils/
```

## Core modules

### models

- `VehicleBody`: full-vehicle rigid-body integration and 4-corner coupling.
- `ECorner`: integrated corner model (steering, drive, brake, suspension, tire).
- Shared default parameters are loaded from `models/params/vehicle_standard.yaml`.

### controllers

- `anti_roll_bar_controll/active_anti_roll_bar_controller.py`
  - Active anti-roll bar controller with gain dataclass:
    - `ActiveAntiRollBarController`
    - `ActiveAntiRollBarGains`
- `yaw_rate_steering_controller/`
  - Yaw-rate-based steering controller package with:
    - instance API (`YawRateSteeringController`)
    - convenience API (`compute_steering_torque`, `compute_steering_angle`)
    - config adapter and estimator/control-block modules
    - example and smoke tests

### scenarios

- `scenarios/sinesweep/` contains logged-input sine-sweep helpers.
- `from vehicle_sim import scenarios` exposes the scenarios package namespace.

### utils

- `utils/config_loader.py` is the shared YAML configuration loader.

## Yaw-rate steering quick start

Run from repository root:

```bash
python -m vehicle_sim.controllers.yaw_rate_steering_controller.examples.controller_usage_demo
python vehicle_sim/controllers/yaw_rate_steering_controller/examples/controller_usage_demo.py
python -m vehicle_sim.controllers.yaw_rate_steering_controller.tests.test_public_api
```

Recommended import style:

```python
from vehicle_sim.controllers.yaw_rate_steering_controller import (
    YawRateSteeringController,
    YawRateSteeringControllerOptions,
    compute_steering_torque,
    compute_steering_angle,
)
```

## Notes

- This README reflects the code currently present in this branch.
- Deprecated references to removed controller modules were intentionally dropped.
