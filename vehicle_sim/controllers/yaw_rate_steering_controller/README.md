# Yaw Rate Steering Controller

차량 상태와 요레이트 지령을 입력으로 받아 코너별 조향각·조향 토크를 계산하는 제어기.

**예제 노트북**: [yawrate_tracking_test.ipynb](test_debug/yawrate_tracking_test.ipynb)
**제어기 코드**: [controller.py](controller.py)
**게인 파라미터**: [controller_gains.yaml](param/controller_gains.yaml)
**옵션 파라미터**: [controller_options.example.yaml](param/controller_options.example.yaml)

---

## 메서드

### `update(state, ref) -> Dict[str, float]`

매 스텝 제어 출력 계산. 생성 시 `output_mode`에 따라 조향각 또는 조향 토크를 반환한다.

| 인자 | 타입 | 설명 |
|---|---|---|
| `state` | `dict` | 차량 상태 (yaw rate, 속도, 조향각, 타이어 힘 등) |
| `ref` | `dict` | 요레이트 지령 (`yaw_rate` [rad/s]) |

**출력 — `output_mode="torque"` (기본)**: 코너별 조향 토크 `[N·m]`

**출력 — `output_mode="steer"`**: 코너별 조향각 지령 `[rad]`

```python
state = {
    "yaw_rate":             float,                                       # 현재 yaw rate [rad/s]
    "vx":                   float,                                       # 종방향 속도 [m/s]
    "steering_angle":       {"FL": ..., "FR": ..., "RL": ..., "RR": ...},  # [rad]
    "fy_tire":              {"FL": ..., "FR": ..., "RL": ..., "RR": ...},  # [N]
    "fx_tire":              {"FL": ..., "FR": ..., "RL": ..., "RR": ...},  # [N]
    "fz":                   {"FL": ..., "FR": ..., "RL": ..., "RR": ...},  # [N]
    "ay":                   float,                                       # 횡방향 가속도 [m/s²]
    "delta_dot":            {"FL": ..., "FR": ..., "RL": ..., "RR": ...},  # [rad/s]
    "steering_torque_axis": {"FL": ..., "FR": ..., "RL": ..., "RR": ...},  # [N·m]
    "alpha":                {"FL": ..., "FR": ..., "RL": ..., "RR": ...},  # slip angle [rad]
}
reference = {"yaw_rate": 0.1}

steer = YawRateSteeringController(..., output_mode="steer")
trq   = YawRateSteeringController(..., output_mode="torque")

delta_cmd = steer.update(state, reference)  # {"FL": δ_FL, "FR": δ_FR, "RL": δ_RL, "RR": δ_RR}
steer_trq = trq.update(state, reference)    # {"FL": T_FL, "FR": T_FR, "RL": T_RL, "RR": T_RR}
```

### `compute_steer_command(state, ref) -> Dict[str, float]`

조향각 지령만 계산 (토크 변환 없이).

**출력**: 코너별 조향각 `[rad]`

### `compute_torque_command(state, ref) -> Dict[str, float]`

조향 토크 지령 계산.

**출력**: 코너별 조향 토크 `[N·m]`

### `reset()`

내부 PID·추정기 상태를 0으로 초기화.

---

## 루프 구조

| 단계 | 내용 |
|---|---|
| 센서 읽기 (`state`) | 차량 상태 → 제어기 입력 형식으로 변환 |
| 지령 (`reference`) | 시나리오에서 요레이트 목표 조회 |
| 제어기 업데이트 | `steer.update(state, reference)` / `trq.update(state, reference)` |
| 모델 업데이트 | 차량 물리 모델 진행 |

---

## 파라미터

### 게인 (`controller_gains.yaml`)

| 파라미터 | 기본값 | 단위 | 설명 |
|---|---|---|---|
| `yaw_rate_pid.kp` | 100.0 | N·m/(rad/s) | yaw rate → Mz 비례 게인 |
| `yaw_rate_pid.ki` | 50.0 | N·m/(rad/s·s) | yaw rate → Mz 적분 게인 |
| `yaw_rate_pid.kd` | 10.0 | N·m·s/(rad/s) | yaw rate → Mz 미분 게인 |
| `fy_pid.kp` | 4.4e-9 | rad/N | Fy 오차 → 조향각 보정 비례 게인 |
| `steering_pid.kp` | 100.0 | N·m/rad | 조향각 → 토크 비례 게인 |
| `steering_pid.ki` | 50.0 | N·m/(rad·s) | 조향각 → 토크 적분 게인 |
| `steering_pid.kd` | 10.0 | N·m·s/rad | 조향각 → 토크 미분 게인 |

### 제어기 모드 (`controller_options.example.yaml`)

| 모드 | 설명 |
|---|---|
| `ff` | Feedforward only |
| `ff_fb` | Feedforward + yaw/Fy/steer feedback |
| `ff_fb_ls` | Feedback + B·C_α 추정기 (최고 성능) |

---

## 제어 수식

> 추후 업데이트 예정.

### 변수 정의

| 기호 | 변수명 | 단위 | 설명 |
|---|---|---|---|
| $r$ | `yaw_rate` | rad/s | 현재 요레이트 |
| $r_\text{ref}$ | `reference["yaw_rate"]` | rad/s | 목표 요레이트 |
| $e_r$ | — | rad/s | 요레이트 오차 |
| $I_z$ | — | kg·m² | 차량 요 관성 모멘트 |
| $M_{z,\text{ff}}$ | — | N·m | 피드포워드 요 모멘트 |
| $M_{z,\text{fb}}$ | — | N·m | 피드백 요 모멘트 |
| $M_{z,\text{cmd}}$ | — | N·m | 합산 요 모멘트 지령 |
| $x_i$ | — | m | 코너 $i$ 종방향 모멘트 암 (CG 기준) |
| $y_i$ | — | m | 코너 $i$ 횡방향 모멘트 암 (CG 기준) |
| $F_{x,i}$ | `fx_tire[i]` | N | 코너 $i$ 종력 |
| $F_{y,i,\text{cmd}}$ | `delta_cmd` 계산에 사용 | N | 코너 $i$ 횡력 지령 |
| $C_{\alpha,i}$ | — | N/rad | 코너 $i$ 코너링 스티프니스 |
| $\delta_{i,\text{cmd}}$ | `delta_cmd[i]` | rad | 코너 $i$ 조향각 지령 |
| $\delta_{i,\text{meas}}$ | `steering_angle[i]` | rad | 코너 $i$ 조향각 측정값 |
| $T_{\text{steer},i}$ | `steer_trq[i]` | N·m | 코너 $i$ 조향 토크 지령 |

---

**Feedforward — yaw moment**

$$
M_{z,\text{ff}} = I_z \cdot \dot{r}_\text{ref}
$$

**Feedback — yaw rate PID**

$$
e_r = r_\text{ref} - r
$$

$$
M_{z,\text{fb}} = k_p \cdot e_r + k_i \int e_r \, dt + k_d \dot{e}_r
$$

**합산 yaw moment → 코너별 횡력 배분**

$$
M_{z,\text{cmd}} = M_{z,\text{ff}} + M_{z,\text{fb}}
$$

yaw moment 관계식 $M_z = \sum_i (x_i \cdot F_{y,i} - y_i \cdot F_{x,i})$ 에서 각 코너에 균등 분할하면:

$$
F_{y,i,\text{cmd}} = \frac{M_{z,\text{cmd}} / N_\text{wheels} + y_i \cdot F_{x,i}}{x_i}
$$

**횡력 → 조향각 변환 (선형 타이어)**

$$
\delta_{i,\text{cmd}} = \frac{F_{y,i,\text{cmd}}}{C_{\alpha,i}}
$$

**조향각 → 토크 (steering PID)**

$$
e_{\delta,i} = \delta_{i,\text{cmd}} - \delta_{i,\text{meas}}
$$

$$
T_{\text{steer},i} = k_p \cdot e_{\delta,i} + k_i \int e_{\delta,i} \, dt + k_d \dot{e}_{\delta,i}
$$

```text
e_r          = r_ref - r                        [rad/s]
Mz_ff        = Iz * r_ref_dot                   [N·m]
Mz_fb        = yaw_rate_pid(e_r)                [N·m]
Mz_cmd       = Mz_ff + Mz_fb                    [N·m]

Fy_cmd[i]    = (Mz_cmd/N + y_i*Fx_i) / x_i      [N]   ← N·m/m = N
delta_cmd[i] = Fy_cmd[i] / C_alpha[i]           [rad] ← N/(N/rad) = rad

e_delta[i]   = delta_cmd[i] - delta_meas[i]     [rad]
T_steer[i]   = steering_pid(e_delta[i])         [N·m]
```
