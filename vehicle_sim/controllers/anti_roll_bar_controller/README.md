# Active Anti-Roll Bar Controller

좌우 서스펜션 스트로크 차이를 입력으로 받아 코너별 안티롤 토크를 출력하는 제어기.

**예제 노트북**: [aarb_onoff_test.ipynb](test_debug/aarb_onoff_test.ipynb)
**제어기 코드**: [controller.py](controller.py)
**파라미터**: [aarb_param.yaml](param/aarb_param.yaml)

---

## 메서드

### `update(delta_s, delta_s_dot, output_type="torque") -> Dict[str, float]`

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

### `get_state() -> Dict[str, float]`

현재 제어기 상태 조회.

| 키 | 단위 | 설명 |
|---|---|---|
| `M_arb_front` | N*m | 전륜 ARB 모멘트 |
| `M_arb_rear` | N*m | 후륜 ARB 모멘트 |
| `delta_s_front` | m | 전륜 좌우 스트로크 차 |
| `delta_s_rear` | m | 후륜 좌우 스트로크 차 |

### `reset()`

내부 상태를 0으로 초기화.

---

## 파라미터 (`aarb_param.yaml`)

| 파라미터 | 수식 기호 | 기본값 | 단위 | 설명 |
|---|---|---|---|---|
| `k_arb_front` | $k_\text{arb,f}$ | 30000 | N/m | 전륜 ARB 강성 |
| `c_arb_front` | $c_\text{arb,f}$ | 2000 | N·s/m | 전륜 ARB 감쇠 |
| `k_arb_rear` | $k_\text{arb,r}$ | 25000 | N/m | 후륜 ARB 강성 |
| `c_arb_rear` | $c_\text{arb,r}$ | 1500 | N·s/m | 후륜 ARB 감쇠 |
| `track_width` | $t_w$ | 1.634 | m | 트랙 폭 |
| `lead` | $l$ | 0.01 | m/rev | 볼스크류 리드 |
| `efficiency` | $\eta$ | 0.9 | - | 액추에이터 효율 |

---

## 제어 수식

$$
\Delta s_\text{front} = \Delta s_{FL} - \Delta s_{FR}, \quad
\Delta s_\text{rear}  = \Delta s_{RL} - \Delta s_{RR}
$$

$$
M_\text{arb} = k_\text{arb} \cdot \Delta s + c_\text{arb} \cdot \dot{\Delta s}
$$

$$
F_L = -\frac{M_\text{arb}}{t_w}, \quad F_R = +\frac{M_\text{arb}}{t_w}
$$

볼스크류 토크 변환:

$$
T = \frac{F \cdot l}{2\pi \cdot \eta}
$$

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
