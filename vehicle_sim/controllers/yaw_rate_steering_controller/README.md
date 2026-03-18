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

센서가 적은 경우에는 아래 **최소 `state`** 만으로도 동작한다.

```python
minimal_state = {
    "yaw_rate": 0.02,  # [rad/s]
    "vx": 8.33,        # [m/s]
    "steering_angle": {
        "FL": 0.0, "FR": 0.0, "RL": 0.0, "RR": 0.0,
    },
}
reference = {"yaw_rate": 0.1}

delta_cmd = steer.update(minimal_state, reference)
steer_trq = trq.update(minimal_state, reference)
```

최소 `state` 사용 시 내부 처리:

- `fz` 미입력: 정하중 근사값 사용
- `delta_dot` 미입력: 이전 조향각 이력으로 내부 차분 추정
- `fy_tire`, `fx_tire`, `ay`, `steering_torque_axis`, `alpha` 미입력: 관련 feedback/estimator 성능은 제한되지만 제어기는 동작

즉, 센서가 부족한 초기 단계에는 최소 `state`로 시작하고, 사용 가능한 센서가 늘어나면 선택 입력을 추가하는 방식으로 확장하면 된다.

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

아래 식은 설명용 연속시간 표기이며, 실제 코드는 이산시간으로 동작한다.

**1. Yaw moment 생성**

$$
e_r = r_\text{ref} - r
$$

$$
M_{z,\text{ff}} = I_z \dot{r}_\text{ref}
$$

$$
M_{z,\text{fb}} = k_p e_r + k_i \int e_r \, dt + k_d \dot{e}_r
$$

$$
M_{z,\text{cmd}} = M_{z,\text{ff}} + M_{z,\text{fb}}
$$

**2. Yaw moment → 앞/뒤축 횡력 분배**

$$
F_{y,\text{total,cmd}} = m v_x r_\text{ref}
$$

$$
M_{z,\text{lat,des}} = M_{z,\text{cmd}} - \sum_i \left(-y_i F_{x,i}\right)
$$

현재 기본 경로에서는 allocator 호출 시 `Fx_body=None`이므로
$M_{z,\text{lat,des}} = M_{z,\text{cmd}}$ 이다.

앞/뒤축 횡력은 아래 두 조건을 만족하도록 먼저 계산한다.

$$
F_{y,f} + F_{y,r} = F_{y,\text{total,cmd}}
$$

$$
l_f F_{y,f} - l_r F_{y,r} = M_{z,\text{lat,des}}
$$

연립하면:

$$
F_{y,f} = \frac{M_{z,\text{lat,des}} + l_r F_{y,\text{total,cmd}}}{l_f + l_r}
$$

$$
F_{y,r} = \frac{l_f F_{y,\text{total,cmd}} - M_{z,\text{lat,des}}}{l_f + l_r}
$$

현재 구현은 각 축 내부 좌/우 하중이동은 고려하지 않고 좌우를 반반으로 나눈다.

$$
F_{y,\mathrm{FL}} = F_{y,\mathrm{FR}} = \frac{F_{y,f}}{2}
$$

$$
F_{y,\mathrm{RL}} = F_{y,\mathrm{RR}} = \frac{F_{y,r}}{2}
$$

**3. 횡력 → 조향각**

$$
\beta_{\text{ref},i} =
\text{atan2}(v_{y,\text{cmd}} + r_\text{ref} x_i,\; v_{x,\text{cmd}} - r_\text{ref} y_i)
$$

$$
F_{y,i}^{\text{clip}} = \text{clip}(F_{y,i}, -\mu_i F_{z,i}, \mu_i F_{z,i})
$$

$$
\alpha_{\text{cmd},i} = -\frac{F_{y,i}^{\text{clip}}}{C_{\alpha,i}}
$$

$$
\delta_{\text{ff},i} = \beta_{\text{ref},i} - \alpha_{\text{cmd},i}
$$

Fy feedback 사용 시:

$$
e_{Fy,i} = F_{y,i} - F_{y,i}^{\text{actual}}, \qquad
\delta_{\text{cmd},i} = \delta_{\text{ff},i} + PID_{Fy,i}(e_{Fy,i})
$$

Fy feedback 미사용 시:

$$
\delta_{\text{cmd},i} = \delta_{\text{ff},i}
$$

**4. 조향각 → 모터 토크**

$$
T_{\text{align},i} = trail_i F_{y,i}^{\text{clip}}
$$

$$
T_{\text{ff,axis},i} = J_{cq} \ddot{\delta}_{\text{cmd},i}
+ B_{cq} \dot{\delta}_{\text{cmd},i}
+ T_{\text{align},i}
$$

$$
e_{\delta,i} = \delta_{\text{cmd},i} - \delta_{\text{meas},i}
$$

$$
T_{\text{fb,axis},i} = PID_{\delta,i}(e_{\delta,i})
$$

$$
T_{\text{motor},i} = \frac{T_{\text{ff,axis},i} + T_{\text{fb,axis},i}}{g_i}
$$

## 기호 설명

| 기호 | 의미 | 단위 | 비고 |
|---|---|---|---|
| $r$ | 실제 yaw rate | rad/s | `state["yaw_rate"]` |
| $r_\text{ref}$ | yaw rate 지령 | rad/s | `ref["yaw_rate"]` |
| $e_r$ | yaw rate 오차 | rad/s | `r_ref - r` |
| $I_z$ | 차량 yaw 축 관성모멘트 | kg·m² | vehicle body의 `Izz` |
| $M_{z,\text{ff}}$ | feedforward yaw moment | N·m | yaw rate 지령 변화율 기반 |
| $M_{z,\text{fb}}$ | feedback yaw moment | N·m | yaw PID 출력 |
| $M_{z,\text{cmd}}$ | 최종 yaw moment 명령 | N·m | `M_ff + M_fb` |
| $m$ | 차량 질량 | kg | vehicle body의 `m` |
| $v_x$ | 종방향 속도 | m/s | `state["vx"]` |
| $v_{x,\text{cmd}}$ | 조향각 계산에 쓰는 종방향 속도 | m/s | 현재 구현에서는 $v_x$와 동일 |
| $v_{y,\text{cmd}}$ | 조향각 계산에 쓰는 횡방향 속도 목표 | m/s | 현재 구현에서는 `0` |
| $F_{y,\text{total,cmd}}$ | 총 횡력 목표 | N | $m v_x r_\text{ref}$ |
| $M_{z,\text{lat,des}}$ | 횡력으로 만들어야 할 목표 yaw moment | N·m | 종력으로 이미 생성된 yaw moment를 제외한 잔여 모멘트 |
| $l_f$ | CG → 앞축 거리 | m | front axle lever arm |
| $l_r$ | CG → 뒤축 거리 | m | rear axle lever arm |
| $x_i$ | CG 기준 i번 바퀴의 x 위치 | m | 전방 `+`, 후방 `-` |
| $y_i$ | CG 기준 i번 바퀴의 y 위치 | m | 좌측 `+`, 우측 `-` |
| $F_{x,i}$ | i번 바퀴 종력 | N | allocator에 종력이 주어지면 $M_{z,\text{lat,des}}$ 계산에 반영 |
| $F_{y,f}$ | 앞축 총 횡력 명령 | N | 앞축 좌/우 합 |
| $F_{y,r}$ | 뒤축 총 횡력 명령 | N | 뒤축 좌/우 합 |
| $F_{y,i}$ | i번 바퀴 횡력 명령 | N | wheel frame 기준 |
| $F_{y,i}^\text{actual}$ | Fy feedback용 실제/추정 횡력 | N | measured 또는 estimated |
| $F_{z,i}$ | i번 바퀴 수직하중 | N | `state["fz"]` 또는 정하중 근사 |
| $\beta_{\text{ref},i}$ | i번 바퀴 위치에서의 목표 진행방향 각 | rad | `atan2(v_y_ref, v_x_ref)` |
| $\mu_i$ | i번 타이어 마찰계수 | - | lateral tire parameter |
| $F_{y,i}^\text{clip}$ | 마찰한계로 clamp한 횡력 | N | $\text{clip}(F_y, -\mu F_z, \mu F_z)$ |
| $C_{\alpha,i}$ | i번 코너의 cornering stiffness | N/rad | 기본값 또는 추정값 |
| $\alpha_{\text{cmd},i}$ | 횡력에 대응하는 slip angle 명령 | rad | $-F_y^\text{clip}/C_\alpha$ |
| $\delta_{\text{ff},i}$ | feedforward 조향각 | rad | $\beta_\text{ref} - \alpha_\text{cmd}$ |
| $\delta_{\text{cmd},i}$ | 최종 조향각 명령 | rad | $\delta_\text{ff}$ + Fy feedback |
| $\delta_{\text{meas},i}$ | 실제 조향각 | rad | `state["steering_angle"][i]` |
| $e_{Fy,i}$ | 횡력 오차 | N | `F_y_cmd - F_y_actual` |
| $e_{\delta,i}$ | 조향각 오차 | rad | `delta_cmd - delta_meas` |
| $\dot{\delta}_{\text{cmd},i}$ | 조향각 명령 속도 | rad/s | 명령 이력 차분 |
| $\ddot{\delta}_{\text{cmd},i}$ | 조향각 명령 가속도 | rad/s² | 명령 속도 차분 |
| $\text{trail}_i$ | pneumatic / mechanical trail 등가값 | m | 구현에서는 lateral tire `trail` |
| $T_{\text{align},i}$ | self-aligning torque 근사값 | N·m | $\text{trail}_i \cdot F_y^\text{clip}$ |
| $J_{cq}$ | 조향축 등가 관성 | N·m·s²/rad | steering FF 파라미터 |
| $B_{cq}$ | 조향축 등가 점성계수 | N·m·s/rad | steering FF 파라미터 |
| $T_{\text{ff,axis},i}$ | 조향축 기준 feedforward 토크 | N·m | 관성 + 감쇠 + aligning |
| $PID_{\delta,i}$ | 조향각 feedback PID | N·m | 축 토크 기준 출력 |
| $T_{\text{fb,axis},i}$ | 조향각 feedback 축 토크 | N·m | `PID_delta(e_delta)` |
| $g_i$ | 조향 gear ratio | - | axis torque → motor torque 변환 |
| $T_{\text{motor},i}$ | 최종 모터 토크 명령 | N·m | $(T_\text{ff,axis} + T_\text{fb,axis}) / g_i$ |

※ 제어기 추후 추가 업데이트 예정
