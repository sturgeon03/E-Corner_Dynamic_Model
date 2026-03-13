# E-Corner Dynamic Model

Physics-oriented Python package for studying an e-corner vehicle architecture at two levels:

- a full-vehicle rigid-body model (`VehicleBody`)
- a per-corner integrated module (`ECorner`) that couples steering, braking, drive, suspension, and tire dynamics

The repository already contains the core model hierarchy and a central YAML parameter set. It is not yet a finished end-to-end simulator application; some orchestration, helper utilities, and old example entrypoints are still under construction.

## What is in the repository

- `vehicle_sim/`: the installable Python package
- `vehicle_sim/models/vehicle_body/vehicle_body.py`: 6-DOF vehicle body model that owns four corners
- `vehicle_sim/models/e_corner/e_corner.py`: integrated corner model
- `vehicle_sim/models/params/vehicle_standard.yaml`: default parameter source for the package
- `vehicle_sim/README.md`: detailed package and architecture documentation

## Implemented model scope

- 4-corner vehicle body integration with body-frame force and moment assembly
- corner-level steering, brake, drive, suspension, and tire submodels
- longitudinal tire force from slip ratio
- lateral tire force and aligning torque from slip angle
- YAML-based parameter loading shared across modules
- trajectory primitives and sine-path generation utilities under `vehicle_sim/scenarios`
- an active anti-roll-bar controller under `vehicle_sim/controllers`
- an integrated lateral controller chain (`yawrate -> steering torque`) under `vehicle_sim/controllers/lateral_feature`

## Installation

```bash
python -m venv .venv
.venv\Scripts\activate
pip install -e .
```

Dependencies declared by the package:

- `numpy`
- `pyyaml`
- `matplotlib`

## Minimal usage

The safest current entrypoint is the package API itself, not `vehicle_sim/main.py`.

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
state_vector = vehicle.get_state_vector()
print(state_vector)
```

Lateral controller one-line step:

```python
from vehicle_sim import VehicleBody, create_lateral_torque_stepper

vehicle = VehicleBody()
step_lateral = create_lateral_torque_stepper(vehicle_body=vehicle, dt=0.001)
T_steer_cmd = step_lateral(
    yaw_rate_cmd=0.15,
    yaw_rate=vehicle.state.yaw_rate,
    ay=vehicle.state.ay_prev,
    vx=vehicle.state.velocity_x,
    steer_angle={
        label: vehicle.corners[label].state.steering_angle
        for label in vehicle.wheel_labels
    },
)  # {"FL":..., "FR":..., "RR":..., "RL":...}
```

For a deeper breakdown of the package, model interfaces, configuration layout, and known limitations, see `vehicle_sim/README.md`.

## Current status and caveats

- `vehicle_sim/main.py` still references simulator/controller/scenario glue that is not present in the current tree.
- `vehicle_sim/scenarios` currently provides trajectory data structures and path-generation utilities, not a full scenario runner.
- `vehicle_sim/utils/math_utils.py`, `vehicle_sim/utils/coordinate_transform.py`, and `vehicle_sim/models/e_corner/config/corner_config.py` are placeholders with TODOs.
- Several `test_debug/` directories contain exploratory notebooks and scripts for component-level investigation.

## Recommended reading order

1. `vehicle_sim/README.md`
2. `vehicle_sim/models/params/vehicle_standard.yaml`
3. `vehicle_sim/models/e_corner/e_corner.py`
4. `vehicle_sim/models/vehicle_body/vehicle_body.py`
