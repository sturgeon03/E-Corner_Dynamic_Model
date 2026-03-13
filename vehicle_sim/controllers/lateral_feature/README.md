# Lateral Feature

`lateral_feature`는 아래 통합 제어 체인을 제공합니다.

`yaw_rate_cmd -> yaw moment (FF + PID) -> wheel Fy allocation -> steering angle FF -> steering torque PID`

## 메인 API (리팩토링 버전)

실차 형태의 구조화된 인터페이스를 권장합니다.

- `LateralSensorFrame`: 한 제어 주기의 센서 입력
- `LateralRequest`: 한 제어 주기의 명령 입력
- `LateralOutput`: 제어기 출력 묶음
- `LateralYawRateTorqueController.update(sensor, request)`

## 기본 사용 예

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
    vx=vehicle.state.velocity_x,      # 차속 추정값
    steer_angle={
        label: vehicle.corners[label].state.steering_angle
        for label in vehicle.wheel_labels
    },
    wheel_speed={
        label: vehicle.corners[label].state.omega_wheel
        for label in vehicle.wheel_labels
    },
    fx_body=None,   # 선택: 휠별 Fx 추정 맵
    timestamp=None, # 선택
    valid=True,     # False면 안전하게 0 토크 출력
)

request = LateralRequest(
    yaw_rate_cmd=0.15,
    yaw_accel_cmd=None,  # 선택
    vy_cmd=None,         # 선택
    fy_total_cmd=None,   # 선택
)

output = controller.update(sensor=sensor, request=request)
T_steer_cmd = output.steer_torque_cmd  # {"FL":..., "FR":..., "RR":..., "RL":...}
debug = output.debug
```

## 원라인 래퍼 (필수 입력만)

`create_lateral_torque_stepper(...)`는 실행 시 필수 입력만 받습니다.

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
)
```

차량 상태를 내부에서 자동으로 읽는 초간단 래퍼가 필요하면 아래를 사용하세요.

```python
from vehicle_sim import create_lateral_torque_stepper_from_vehicle

step_lateral_short = create_lateral_torque_stepper_from_vehicle(vehicle_body=vehicle, dt=0.001)
T_steer_cmd = step_lateral_short(yaw_rate_cmd=0.15)
```

## 시뮬레이션 루프 예시

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

## YAML 설정

기본 설정 파일:

- `vehicle_sim/models/params/vehicle_standard.yaml`
- key: `lateral_controller`

커스텀 설정 파일 사용:

```python
controller = LateralYawRateTorqueController(
    vehicle_body=vehicle,
    dt=0.001,
    config_path="path/to/custom_lateral_config.yaml",
)
```

지원 형식:

- top-level `lateral_controller: {...}`
- 또는 파일 전체를 컨트롤러 설정 dict로 사용

## 호환성 참고

- `step(...)`, `step_with_debug(...)`, `step_from_vehicle(...)`는 하위 호환을 위해 유지됩니다.
- 배포/실차 연동에서는 `update(sensor, request)` 인터페이스를 권장합니다.
