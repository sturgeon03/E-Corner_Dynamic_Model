# Controllers

차량 제어기 모듈 모음.

---

## 디렉토리 구조

```text
controllers/
├── anti_roll_bar_controll/
│   ├── active_anti_roll_bar_controller.py   # 제어기 본체
│   ├── param/
│   │   └── aarb_param.yaml                  # 게인 파라미터
│   └── test_debug/
│       └── aarb_comparison.ipynb            # ON/OFF 비교 검증 노트북
└── yaw_rate_steering_controller/
    ├── controller.py                        # 메인 제어기
    ├── config_adapter.py                    # YAML -> 런타임 옵션 변환
    ├── config/
    │   ├── controller_gains.yaml            # PID 게인
    │   └── controller_options.example.yaml  # 모드/옵션 템플릿
    ├── control_blocks/                      # 제어 블록
    ├── estimators/                          # 추정기
    ├── examples/
    │   └── controller_usage_demo.py         # 사용 예제
    ├── tests/
    │   └── test_public_api.py               # 공개 API 스모크 테스트
    └── README.md
```

---

## Active Anti-Roll Bar Controller

좌우 서스펜션 스트로크 차이를 입력으로 받아 코너별 안티롤 토크를 출력하는 PD 제어기.

**예제 노트북**: [aarb_comparison.ipynb](anti_roll_bar_controll/test_debug/aarb_comparison.ipynb)
**테스트 코드**: [active_anti_roll_bar_controller.py](anti_roll_bar_controll/active_anti_roll_bar_controller.py)
**파라미터**: [aarb_param.yaml](anti_roll_bar_controll/param/aarb_param.yaml)

---

### 제어 법칙

```text
delta_s_front = delta_s_FL - delta_s_FR
delta_s_rear  = delta_s_RL - delta_s_RR

M_arb = k_arb * delta_s + c_arb * delta_s_dot

F_L = -M_arb / track_width
F_R = +M_arb / track_width
```

볼스크류 토크 변환:

```text
T = F * lead / (2 * pi * efficiency)
```

---

### 파라미터 (`aarb_param.yaml`)

| 파라미터 | 기본값 | 단위 | 설명 |
|---|---|---|---|
| `k_arb_front` | 30000 | N/m | 전륜 ARB 강성 |
| `c_arb_front` | 2000 | N*s/m | 전륜 ARB 감쇠 |
| `k_arb_rear` | 25000 | N/m | 후륜 ARB 강성 |
| `c_arb_rear` | 1500 | N*s/m | 후륜 ARB 감쇠 |
| `track_width` | 1.634 | m | 트랙 폭 |
| `lead` | 0.01 | m/rev | 볼스크류 리드 |
| `efficiency` | 0.9 | - | 액추에이터 효율 |

---

### 초기화

```python
from vehicle_sim.controllers.anti_roll_bar_controll.active_anti_roll_bar_controller import ActiveAntiRollBarController

# YAML 파라미터 사용
aarb = ActiveAntiRollBarController(config_path="path/to/aarb_param.yaml")

# 직접 게인 주입
from vehicle_sim.controllers.anti_roll_bar_controll.active_anti_roll_bar_controller import ActiveAntiRollBarGains

gains = ActiveAntiRollBarGains(k_arb_front=30000.0, c_arb_front=2000.0)
aarb = ActiveAntiRollBarController(gains=gains)
```

---

### 메서드

#### `update(delta_s, delta_s_dot, output_type="torque") -> Dict[str, float]`

매 스텝 제어 토크/힘 계산.

| 인자 | 타입 | 단위 | 설명 |
|---|---|---|---|
| `delta_s` | `{"FL", "FR", "RL", "RR": float}` | m | 코너별 서스펜션 스트로크 |
| `delta_s_dot` | `{"FL", "FR", "RL", "RR": float}` | m/s | 코너별 스트로크 변화율 |
| `output_type` | `"torque"` \| `"force"` | - | 출력 단위 선택 |

**출력**: 코너별 토크 `[N*m]` 또는 힘 `[N]`

```python
T_susp = aarb.update(
    delta_s={"FL": ..., "FR": ..., "RL": ..., "RR": ...},
    delta_s_dot={"FL": ..., "FR": ..., "RL": ..., "RR": ...},
    output_type="torque",
)
# {"FL": T_FL, "FR": T_FR, "RL": T_RL, "RR": T_RR}
```

#### `get_state() -> Dict[str, float]`

현재 제어기 상태 조회.

| 키 | 단위 | 설명 |
|---|---|---|
| `M_arb_front` | N*m | 전륜 ARB 모멘트 |
| `M_arb_rear` | N*m | 후륜 ARB 모멘트 |
| `delta_s_front` | m | 전륜 좌우 스트로크 차 |
| `delta_s_rear` | m | 후륜 좌우 스트로크 차 |

#### `reset()`

내부 상태를 0으로 초기화.

---

### 시뮬레이션 루프 예시

```python
for i in range(1, n):
    dt = time[i] - time[i - 1]

    # 서스펜션 스트로크 읽기
    sus = {c: body.corners[c].suspension.state for c in ["FL", "FR", "RL", "RR"]}

    # AARB 토크 계산
    T_susp = aarb.update(
        delta_s={c: sus[c].delta_s for c in ["FL", "FR", "RL", "RR"]},
        delta_s_dot={c: sus[c].delta_s_dot for c in ["FL", "FR", "RL", "RR"]},
    )

    # 차량 모델 업데이트
    body.update(dt, scenario.corner_inputs(idx=i, body=body, t_susp=T_susp))
```

---

## Yaw Rate Steering Controller

목표 yaw rate를 입력으로 받아 yaw moment, 타이어 횡력, 조향각, 조향 모터 토크를 순차적으로 계산하는 통합 조향 제어기.

**예제 코드**: [controller_usage_demo.py](yaw_rate_steering_controller/examples/controller_usage_demo.py)
**테스트 코드**: [test_public_api.py](yaw_rate_steering_controller/tests/test_public_api.py)
**파라미터**: [controller_gains.yaml](yaw_rate_steering_controller/config/controller_gains.yaml), [controller_options.example.yaml](yaw_rate_steering_controller/config/controller_options.example.yaml)

---

### 제어 흐름

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

### 모드 및 옵션 (`controller_options.example.yaml`)

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

### 초기화

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

### 메서드

#### `compute_torque_command(state, ref) -> Dict[str, float]`

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

#### `compute_angle_command(state, ref) -> Dict[str, float]`

조향각 명령만 계산하는 경량 경로.

**출력**: 바퀴별 조향각 명령 `[rad]`

#### `compute_torque_with_debug(state, ref) -> Tuple[Dict[str, float], Dict[str, Any]]`

조향 모터 토크와 디버그 정보를 함께 반환.

주요 디버그 항목:
- `Mz_cmd`
- `Fy_cmd`
- `delta_cmd`
- `T_steer_ff_motor`
- `T_steer_ff_axis`
- `estimator`

#### `control(...)`, `control_torque(...)`, `control_angle(...)`

기존 연동 코드를 위한 backward-compatible alias.

#### `compute_steering_torque(...)`, `compute_steering_angle(...)`

공유 기본 인스턴스를 사용하는 간단 함수 API.

---

### 실행 예시

```bash
python -m vehicle_sim.controllers.yaw_rate_steering_controller.examples.controller_usage_demo
python vehicle_sim/controllers/yaw_rate_steering_controller/examples/controller_usage_demo.py
python -m vehicle_sim.controllers.yaw_rate_steering_controller.tests.test_public_api
```

시뮬레이션 루프 예시:

```python
state = {
    "yaw_rate": body.state.yaw_rate,
    "vx": body.state.velocity_x,
    "steering_angle": {c: body.corners[c].state.steering_angle for c in ["FL", "FR", "RR", "RL"]},
    "fy_tire": {c: body.corners[c].state.F_y_tire for c in ["FL", "FR", "RR", "RL"]},
    "fx_tire": {c: body.corners[c].state.F_x_tire for c in ["FL", "FR", "RR", "RL"]},
    "ay": body.state.ay_prev,
}

ref = {
    "yaw_rate": yaw_rate_ref,
    "yaw_accel": yaw_accel_ref,
    "vx": body.state.velocity_x,
    "vy": body.state.velocity_y,
}

T_steer = controller.compute_torque_command(state, ref)
```
