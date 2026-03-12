# Lateral Feature Usage

`lateral_feature`는 yaw rate 명령을 받아 wheel별 steering torque 명령으로 변환하는 통합 제어 체인을 제공합니다.

체인 구성:

`yaw_rate_cmd -> yaw moment (feedforward + PID) -> wheel Fy allocation -> steering angle FF -> steering torque PID`

핵심 API는 아래 2가지입니다.

- `create_lateral_torque_stepper(...)`: 원라인 호출용 함수
- `LateralYawRateTorqueController`: 디버그/세부 입력이 필요한 객체 API

## 1) 원라인 사용 (권장)

```python
from vehicle_sim import VehicleBody, create_lateral_torque_stepper

vehicle = VehicleBody()  # 기본 YAML: vehicle_sim/models/params/vehicle_standard.yaml
step_lateral = create_lateral_torque_stepper(vehicle_body=vehicle, dt=0.001)

# One-line call: yaw rate command -> per-wheel steering torque [N*m]
T_steer_cmd = step_lateral(yaw_rate_cmd=0.15)
```

반환값 예시:

```python
{
    "FL": ...,
    "FR": ...,
    "RR": ...,
    "RL": ...,
}
```

## 2) VehicleBody.update(...)에 바로 연결

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
    T_steer_cmd = step_lateral(yaw_rate_cmd=0.15)

    for label in vehicle.wheel_labels:
        corner_inputs[label]["T_steer"] = T_steer_cmd[label]

    vehicle.update(dt=0.001, corner_inputs=corner_inputs)
```

## 3) 객체 API (세부 입력/디버그)

```python
from vehicle_sim.controllers import LateralYawRateTorqueController

controller = LateralYawRateTorqueController(vehicle_body=vehicle, dt=0.001)

torque_cmd = controller.step(
    yaw_rate_cmd=0.15,
    yaw_rate_meas=vehicle.state.yaw_rate,      # optional
    vx_meas=vehicle.state.velocity_x,          # optional
    ay_meas=vehicle.state.ay_prev,             # optional
    yaw_accel_cmd=None,                        # optional
    fx_body=None,                              # optional per-wheel Fx map
    fy_total_cmd=None,                         # optional total Fy command
    delta_meas=None,                           # optional per-wheel steering angle map
    vy_cmd=None,                               # optional lateral speed command
)

torque_cmd_dbg, debug = controller.step_with_debug(yaw_rate_cmd=0.15)
```

## 4) YAML 설정

기본적으로 아래 섹션을 읽습니다.

- `vehicle_sim/models/params/vehicle_standard.yaml`
- top-level key: `lateral_controller`

주요 키:

- `yaw_rate_pid`: yaw-rate feedback PID gains
- `steer_torque_pid`: steering-angle tracking PID gains
- `yaw_moment_feedforward`: yaw accel 기반 feedforward 제한
- `yaw_moment_allocator`: 모멘트 분배 옵션
- `lateral_force_estimator`: ay 기반 total lateral force estimator 옵션
- `slip_angle_estimator`: vy/slip angle estimator 옵션
- `steering_feedforward`: Fy clamp, angle unwrap 옵션
- `behavior`: estimator 기반 bias/reference 사용 여부
- `output_limits`: yaw moment / steering torque 제한

별도 YAML을 쓰고 싶으면:

```python
step_lateral = create_lateral_torque_stepper(
    vehicle_body=vehicle,
    dt=0.001,
    config_path="path/to/custom_lateral_config.yaml",
)
```

`custom_lateral_config.yaml`은 아래 둘 다 지원합니다.

- 파일 전체가 `lateral_controller` 설정 dict
- top-level에 `lateral_controller:` 섹션 포함

## 5) 참고 사항

- 출력 토크 단위: `[N*m]`
- `step_from_vehicle`/`create_lateral_torque_stepper`는 내부적으로 차량 상태를 읽습니다:
  - `vehicle.state.yaw_rate`
  - `vehicle.state.velocity_x`
  - `vehicle.state.ay_prev`
  - `vehicle.corners[label].state.steering_angle`
- 필요 시 `controller.reset()`으로 PID/estimator 내부 상태를 초기화할 수 있습니다.
