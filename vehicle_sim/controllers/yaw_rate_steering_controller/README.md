# Yaw Rate Steering Controller

사용자는 **패키지 루트 import만 사용**하세요.
- 권장: `from vehicle_sim.controllers.yaw_rate_steering_controller import ...`
- 비권장: 내부 구현 파일 직접 import

공식 진입점은 [`controller.py`](./controller.py) 입니다.

## 폴더 구조
- 루트(공개 API)
  - `controller.py`: 메인 제어기
  - `config_adapter.py`: 옵션 YAML -> 런타임 옵션
  - `__init__.py`: 공개 API re-export
  - `README.md`
- 구현 상세
  - `control_blocks/`: `steer_*`, `yaw_moment_*`, `pid_controller.py`
  - `estimators/`: `lateral_force_estimator.py`, `slip_angle_estimator.py`, `steering_param_estimator.py`
  - `config/`: `controller_gains.yaml`, `controller_options.example.yaml`
  - `examples/`: `controller_usage_demo.py`
  - `tests/`: `test_public_api.py`

## 권장 사용법
```python
from vehicle_sim.controllers.yaw_rate_steering_controller import (
    YawRateSteeringController,
    YawRateSteeringControllerOptions,
)

options = YawRateSteeringControllerOptions(dt=0.001)
controller = YawRateSteeringController(options)

torque_cmd = controller.compute_torque_command(state, ref)
angle_cmd = controller.compute_angle_command(state, ref)
```

디버그:
```python
torque_cmd, debug = controller.compute_torque_with_debug(state, ref)
```

간단 함수 API:
```python
from vehicle_sim.controllers.yaw_rate_steering_controller import (
    compute_steering_torque,
    compute_steering_angle,
)

torque_cmd = compute_steering_torque(state, ref, reset=True)
angle_cmd = compute_steering_angle(state, ref, reset=True)
```

## 입력/출력 계약
필수 `state`:
- `yaw_rate` [rad/s]
- `vx` [m/s]
- `steering_angle` (`FL/FR/RR/RL` -> [rad])

필수 `ref`:
- `yaw_rate` [rad/s]

선택 `state`:
- `ay`, `delta_dot`, `fy_tire`, `steering_torque_axis`, `alpha`

출력:
- 토크 API: 바퀴별 조향 모터 토크 [N*m]
- 각도 API: 바퀴별 조향각 명령 [rad]

## 모드와 기본 동작
옵션 템플릿: `config/controller_options.example.yaml`
- `ff`
- `ff_fb`
- `ff_fb_ls` (권장 기본)

중요 기본값:
- `fy_feedback_source = estimate`
- `use_lateral_force_estimator = true`
- `compute_angle_command()` 경로는 `C_alpha` 추정은 수행, `B` 추정은 스킵

## YAML 로딩 예시
```bash
python SeohanModel/vehicle_sim/scenarios/yaw_rate_study/quick_plot_controller.py \
  --controller-config SeohanModel/vehicle_sim/controllers/yaw_rate_steering_controller/config/controller_options.example.yaml
```

## 실행
```bash
cd SeohanModel
python -m vehicle_sim.controllers.yaw_rate_steering_controller.examples.controller_usage_demo
python -m vehicle_sim.controllers.yaw_rate_steering_controller.tests.test_public_api
```
