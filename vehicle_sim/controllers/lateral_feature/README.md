# Lateral Feature

`lateral_feature` provides an integrated controller chain:

`yaw_rate_cmd -> yaw moment (FF + PID) -> wheel Fy allocation -> steering angle FF -> steering torque PID`

## Main API (Refactored)

Use a structured real-car style interface:

- `LateralSensorFrame`: measured signals for one control tick
- `LateralRequest`: command inputs for one control tick
- `LateralOutput`: controller output bundle
- `LateralYawRateTorqueController.update(sensor, request)`

## Minimal Usage

```python
from vehicle_sim import VehicleBody
from vehicle_sim.controllers import (
    LateralYawRateTorqueController,
    LateralSensorFrame,
    LateralRequest,
)

vehicle = VehicleBody()
controller = LateralYawRateTorqueController(vehicle_body=vehicle, dt=0.001)

sensor = LateralSensorFrame(
    yaw_rate=vehicle.state.yaw_rate,  # IMU
    ay=vehicle.state.ay_prev,         # IMU
    vx=vehicle.state.velocity_x,      # velocity estimate
    steer_angle={
        label: vehicle.corners[label].state.steering_angle
        for label in vehicle.wheel_labels
    },
    wheel_speed={
        label: vehicle.corners[label].state.omega_wheel
        for label in vehicle.wheel_labels
    },
    fx_body=None,   # optional per-wheel Fx estimate map
    timestamp=None, # optional
    valid=True,     # set False to force safe zero output
)

request = LateralRequest(
    yaw_rate_cmd=0.15,
    yaw_accel_cmd=None,  # optional
    vy_cmd=None,         # optional
    fy_total_cmd=None,   # optional
)

output = controller.update(sensor=sensor, request=request)
T_steer_cmd = output.steer_torque_cmd  # {"FL":..., "FR":..., "RR":..., "RL":...}
debug = output.debug
```

## One-Line Convenience

If you want a very short call, keep using:

```python
from vehicle_sim import VehicleBody, create_lateral_torque_stepper

vehicle = VehicleBody()
step_lateral = create_lateral_torque_stepper(vehicle_body=vehicle, dt=0.001)
T_steer_cmd = step_lateral(yaw_rate_cmd=0.15)
```

Internally, this uses `LateralYawRateTorqueController.step_from_vehicle(...)`.

## Typical Simulation Loop

```python
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

for _ in range(1000):
    sensor = controller.sensor_from_vehicle()
    request = LateralRequest(yaw_rate_cmd=0.15)
    output = controller.update(sensor=sensor, request=request)

    for label in vehicle.wheel_labels:
        corner_inputs[label]["T_steer"] = output.steer_torque_cmd[label]

    vehicle.update(dt=0.001, corner_inputs=corner_inputs)
```

## YAML Configuration

Default config source:

- `vehicle_sim/models/params/vehicle_standard.yaml`
- key: `lateral_controller`

You can pass a custom file:

```python
controller = LateralYawRateTorqueController(
    vehicle_body=vehicle,
    dt=0.001,
    config_path="path/to/custom_lateral_config.yaml",
)
```

Supported config format:

- top-level `lateral_controller: {...}`
- or whole file as the controller dict itself

## Compatibility Notes

- `step(...)`, `step_with_debug(...)`, and `step_from_vehicle(...)` are still available for backward compatibility.
- Preferred new interface for deployment is `update(sensor, request)`.
