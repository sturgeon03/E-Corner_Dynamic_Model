# Yaw Rate Steering Controller

차량 상태와 요레이트 지령을 입력으로 받아 코너별 조향 토크를 계산하는 제어기.

**예제 노트북**: [controller_symmetric_internal_signals_test.ipynb](test_debug/controller_symmetric_internal_signals_test.ipynb)
**제어기 코드**: [controller.py](controller.py)
**게인 파라미터**: [controller_gains.yaml](param/controller_gains.yaml)
**옵션 파라미터**: [controller_options.example.yaml](param/controller_options.example.yaml)

---

## 메서드

### `compute_torque_command(state, ref) -> Dict[str, float]`

매 스텝 조향 토크 계산.

| 인자 | 타입 | 설명 |
|---|---|---|
| `state` | `dict` | 차량 상태 (yaw rate, 속도, 조향각, 타이어 힘 등) |
| `ref` | `dict` | 요레이트 지령 (`yaw_rate` [rad/s]) |

**출력**: 코너별 조향 토크 `{"FL": ..., "FR": ..., "RL": ..., "RR": ...}` [N·m]

```python
state = {
    "yaw_rate":       float,                              # 현재 yaw rate [rad/s]
    "vx":             float,                              # 종방향 속도 [m/s]
    "steering_angle": {"FL": ..., "FR": ..., "RL": ..., "RR": ...},  # [rad]
    "fy_tire":        {"FL": ..., "FR": ..., "RL": ..., "RR": ...},  # [N]
    "fx_tire":        {"FL": ..., "FR": ..., "RL": ..., "RR": ...},  # [N]
    "fz":             {"FL": ..., "FR": ..., "RL": ..., "RR": ...},  # [N]
    "ay":             float,                              # 횡방향 가속도 [m/s²]
    "delta_dot":      {"FL": ..., "FR": ..., "RL": ..., "RR": ...},  # [rad/s]
    "steering_torque_axis": {"FL": ..., "FR": ..., "RL": ..., "RR": ...},  # [N·m]
    "alpha":          {"FL": ..., "FR": ..., "RL": ..., "RR": ...},  # slip angle [rad]
}
reference = {"yaw_rate": 0.1}

steer_trq = controller.compute_torque_command(state, reference)
# {"FL": T_FL, "FR": T_FR, "RL": T_RL, "RR": T_RR}
```

### `compute_angle_command(state, ref) -> Dict[str, float]`

조향각 지령만 계산 (토크 변환 없이).

**출력**: 코너별 조향각 `[rad]`

### `reset()`

내부 PID·추정기 상태를 0으로 초기화.

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

**Feedforward — yaw moment**

$$
M_{z,\text{ff}} = I_z \cdot \dot{r}_\text{ref}
$$

**Feedback — yaw rate PID**

$$
M_{z,\text{fb}} = k_p \cdot e_r + k_i \int e_r \, dt + k_d \dot{e}_r, \quad e_r = r_\text{ref} - r
$$

**합산 yaw moment → 코너별 횡력 배분**

$$
M_{z,\text{cmd}} = M_{z,\text{ff}} + M_{z,\text{fb}}
$$

$$
F_{y,i} = \frac{M_{z,\text{cmd}}}{N_\text{wheels}} \cdot w_i
$$

**횡력 → 조향각 변환 (선형 타이어)**

$$
\delta_i = \frac{F_{y,i}}{C_{\alpha,i}}
$$

**조향각 → 토크 (steering PID)**

$$
T_{\text{steer},i} = k_p \cdot (\delta_{i,\text{cmd}} - \delta_{i,\text{meas}}) + k_i \int (\cdot) \, dt + k_d \frac{d(\cdot)}{dt}
$$

```text
e_r        = yaw_rate_ref - yaw_rate_meas
Mz_fb      = yaw_rate_pid(e_r)
Mz_cmd     = Mz_ff + Mz_fb

Fy_cmd[i]  = yaw_moment_allocator(Mz_cmd, state)
delta_cmd[i] = Fy_cmd[i] / C_alpha[i]

e_delta[i]   = delta_cmd[i] - steering_angle[i]
T_steer[i]   = steering_pid(e_delta[i])
```
