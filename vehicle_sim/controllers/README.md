# Controllers

차량 제어기 모듈 모음.

---

## 디렉토리 구조

```
controllers/
└── anti_roll_bar_controll/
    ├── active_anti_roll_bar_controller.py   # 제어기 본체
    ├── param/
    │   └── aarb_param.yaml                  # 게인 파라미터
    └── test_debug/
        └── aarb_comparison.ipynb            # ON/OFF 비교 검증 노트북
```

---

## Active Anti-Roll Bar Controller

좌우 서스펜션 스트로크 차이를 입력으로 받아 코너별 안티롤 토크를 출력하는 PD 제어기.

**예제 노트북**: [aarb_comparison.ipynb](anti_roll_bar_controll/test_debug/aarb_comparison.ipynb)
**소스 코드**: [active_anti_roll_bar_controller.py](anti_roll_bar_controll/active_anti_roll_bar_controller.py)
**파라미터**: [aarb_param.yaml](anti_roll_bar_controll/param/aarb_param.yaml)

---

### 제어 법칙

```
Δs_front = delta_s_FL - delta_s_FR
Δs_rear  = delta_s_RL - delta_s_RR

M_arb = k_arb * Δs + c_arb * Δs_dot

F_L = -M_arb / track_width
F_R = +M_arb / track_width
```

힘 → 토크 변환 (볼 스크류):

```
T = F * lead / (2π * efficiency)
```

---

### 파라미터 (`aarb_param.yaml`)

| 파라미터 | 기본값 | 단위 | 설명 |
|---|---|---|---|
| `k_arb_front` | 30000 | N/m | 전륜 ARB 강성 |
| `c_arb_front` | 2000 | N·s/m | 전륜 ARB 댐핑 |
| `k_arb_rear` | 25000 | N/m | 후륜 ARB 강성 |
| `c_arb_rear` | 1500 | N·s/m | 후륜 ARB 댐핑 |
| `track_width` | 1.634 | m | 트랙 폭 |
| `lead` | 0.01 | m/rev | 볼 스크류 리드 |
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
aarb  = ActiveAntiRollBarController(gains=gains)
```

---

### 메서드

#### `update(delta_s, delta_s_dot, output_type="torque") → Dict[str, float]`

매 스텝 제어 토크/힘 계산.

| 인자 | 타입 | 단위 | 설명 |
|---|---|---|---|
| `delta_s` | `{"FL", "FR", "RL", "RR": float}` | m | 코너별 서스펜션 스트로크 |
| `delta_s_dot` | `{"FL", "FR", "RL", "RR": float}` | m/s | 코너별 스트로크 변화율 |
| `output_type` | `"torque"` \| `"force"` | - | 출력 단위 선택 |

**출력**: 코너별 토크 `[N·m]` 또는 힘 `[N]`

```python
T_susp = aarb.update(
    delta_s    ={"FL": ..., "FR": ..., "RL": ..., "RR": ...},
    delta_s_dot={"FL": ..., "FR": ..., "RL": ..., "RR": ...},
    output_type="torque",
)
# {"FL": T_FL, "FR": T_FR, "RL": T_RL, "RR": T_RR}
```

#### `get_state() → Dict[str, float]`

현재 내부 상태 조회 (진단용).

| 키 | 단위 | 설명 |
|---|---|---|
| `M_arb_front` | N·m | 전륜 ARB 모멘트 |
| `M_arb_rear` | N·m | 후륜 ARB 모멘트 |
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
        delta_s    ={c: sus[c].delta_s     for c in ["FL", "FR", "RL", "RR"]},
        delta_s_dot={c: sus[c].delta_s_dot for c in ["FL", "FR", "RL", "RR"]},
    )

    # 차량 모델 업데이트
    body.update(dt, scenario.corner_inputs(idx=i, body=body, t_susp=T_susp))
```