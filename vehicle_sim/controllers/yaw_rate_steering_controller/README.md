# Yaw Rate Steering Controller

목표 yaw rate를 입력으로 받아 yaw moment, 타이어 횡력, 조향각, 조향 모터 토크를 순차적으로 계산하는 통합 조향 제어기.

**예제 코드**: [controller_usage_demo.py](examples/controller_usage_demo.py)
**테스트 코드**: [test_public_api.py](tests/test_public_api.py)
**파라미터**: [controller_gains.yaml](config/controller_gains.yaml), [controller_options.example.yaml](config/controller_options.example.yaml)

---

## 제어 흐름

```text
yaw_rate_ref
 -> yaw moment feedforward + feedback
 -> per-wheel lateral force allocation
 -> per-wheel steering angle command
 -> steering motor torque command
```

대표 계산 흐름:

```text
Mz_cmd = Mz_ff + PID(yaw_rate_ref - yaw_rate_meas)
Fy_total_cmd = m * vx_cmd * yaw_rate_ref
delta_cmd = steering_feedforward(Fy_cmd, vx_cmd, yaw_rate_ref, vy_cmd)
T_steer_cmd = steering_ff(delta_cmd) + steering_pid(delta_cmd - delta_meas)
```

---

## 모드 및 옵션 (`controller_options.example.yaml`)

| 항목 | 기본값 | 설명 |
|---|---|---|
| `mode` | `ff_fb_ls` | 기본 운용 모드 |
| `ff` | - | feedforward만 사용 |
| `ff_fb` | - | yaw/Fy/steer feedback 사용 |
| `ff_fb_ls` | - | feedback + slip/B/C_alpha 추정 사용 |
| `dt` | 0.01 | 제어 주기 |
| `fy_feedback_source` | `estimate` | Fy feedback 입력원 |
| `enable_estimator` | `true` | 파라미터 추정기 사용 여부 |

---

## 초기화

```python
from vehicle_sim.controllers.yaw_rate_steering_controller import (
    YawRateSteeringController,
    YawRateSteeringControllerOptions,
)

options = YawRateSteeringControllerOptions(
    dt=0.01,
    enable_yaw_feedback=True,
    enable_fy_feedback=False,
    enable_steer_feedback=True,
    enable_estimator=False,
)

controller = YawRateSteeringController(options)
```

YAML 런타임 설정 사용:

```python
from vehicle_sim.controllers.yaw_rate_steering_controller import (
    load_controller_runtime_config,
    YawRateSteeringController,
)

runtime_cfg = load_controller_runtime_config(
    "vehicle_sim/controllers/yaw_rate_steering_controller/config/controller_options.example.yaml"
)

controller = YawRateSteeringController(
    runtime_cfg.options,
    vehicle_config_path=runtime_cfg.vehicle_config_path,
    gains_path=runtime_cfg.gains_path,
)
```

---

## 메서드

### `compute_torque_command(state, ref) -> Dict[str, float]`

매 스텝 조향 모터 토크 명령 계산.

| 인자 | 타입 | 설명 |
|---|---|---|
| `state` | `dict` | 현재 차량 상태 입력 |
| `ref` | `dict` | yaw rate 기준 입력 |

필수 `state`:
- `yaw_rate`
- `vx`
- `steering_angle`

필수 `ref`:
- `yaw_rate`

**출력**: 바퀴별 조향 모터 토크 `[N*m]`
