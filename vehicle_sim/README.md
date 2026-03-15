# vehicle_sim

E-Corner 차량 동역학 시뮬레이션 패키지.

---

## 시스템 구조

![E-Corner Block Diagram](docs/ecorner_block_diagram.png)

각 E-Corner는 독립적으로 동작하며, 4개 코너의 힘과 모멘트를 VehicleBody가 통합한다.

---

## 구성요소 입출력 요약

| 그룹 | 구성요소 | 입력 | 상태 | 출력 |
|---|---|---|---|---|
| Actuation | Brake Actuator | `T_brk` | `F*_clamp` | `F_clamp` |
| Actuation | Drive System | `T_drv`, `F_clamp` | `ω` | `ω` |
| Actuation | Steer System | `T_str`, `T_align` | `δ`, `δ̇` | `δ` |
| Suspension | Suspension System | `T_susp`, `z_road` | `z_u`, `ż_u` | `F_s`, `F_z` |
| Tire models | Longitudinal Tire Dynamics | `ω`, `V_wx`, `F_z` | — | `F_x` |
| Tire models | Lateral Tire Dynamics | `V_wx`, `V_wy`, `F_z` | — | `F_y`, `M_z` |
| Vehicle body | Vehicle Body Dynamics | `F_x`, `F_y`, `F_s`, `δ` | `X_body` | `X_body`, `V_wx`, `V_wy` |

차체 상태 벡터: `X_body = [z, φ, θ, ż, φ̇, θ̇]`

---

## 패키지 구조

```text
vehicle_sim/
├── models/
│   ├── params/
│   │   └── vehicle_standard.yaml   # 공용 파라미터
│   ├── vehicle_body/
│   │   └── vehicle_body.py         # 전체 차량 바디 모델
│   └── e_corner/
│       ├── e_corner.py             # 코너 통합 모델
│       ├── steering/               # 조향 모델
│       ├── drive/                  # 구동/제동 모델
│       ├── suspension/             # 서스펜션 모델
│       └── tire/                   # 타이어 모델 (종/횡)
├── controllers/                    # 제어기 → controllers/README.md 참고
├── scenarios/                      # 시나리오 입력 데이터
├── utils/                          # 공용 유틸리티
└── docs/                           # 문서용 이미지
```

---

## models

### VehicleBody

4개 코너를 소유하는 전체 차량 강체 모델.

- **입력**: 코너별 `T_steer`, `T_brk`, `T_Drv`, `T_susp`, `z_road`
- **출력**: 6-DOF 차체 상태 (heave, roll, pitch, 속도, 가속도 등)
- 각 코너의 서스펜션 수직력을 받아 차체 운동방정식 적분
- 파라미터 출처: `models/params/vehicle_standard.yaml`

```python
from vehicle_sim import VehicleBody

vehicle = VehicleBody()
vehicle.update(dt=0.001, corner_inputs={...})
state = vehicle.get_state_vector()
```

### ECorner

코너 단위 통합 모델. 조향 → 구동/제동 → 서스펜션 → 타이어 순서로 업데이트.

| 서브모델 | 파일 | 역할 |
|---|---|---|
| SteeringModel | `steering/steering_model.py` | 조향 토크 → 조향각 |
| DriveModel | `drive/drive_model.py` | 구동 토크 → 휠 각속도 |
| BrakeModel | `drive/brake_model.py` | 제동 토크 → 클램핑력 |
| SuspensionModel | `suspension/suspension_model.py` | 차체 자세 → 수직력 |
| LongitudinalTireModel | `tire/longitudinal/longitudinal_tire.py` | 슬립률 → 종방향 힘 |
| LateralTireModel | `tire/lateral/lateral_tire.py` | 슬립각 → 횡방향 힘 |

```python
from vehicle_sim import ECorner

corner = ECorner(corner_id="FL")
corner.update(dt=0.001, T_steer=0.0, T_brk=0.0, T_Drv=0.0,
              T_susp=0.0, X_body=X_body, z_road=0.0)
```

### 파라미터 (`vehicle_standard.yaml`)

모든 서브모델이 공유하는 기본 파라미터 파일.
`corner_offsets`로 FL/FR/RL/RR 각 코너의 CG 기준 위치(x, y)를 정의.

---

## scenarios

```text
scenarios/
└── sinesweep/
    ├── sinesweep.py                    # 사인스윕 입력 생성기
    └── Log_Data/
        └── CM_Body_sinesweep.csv       
```

사인스윕 시나리오 입력을 생성하거나 로그 데이터를 불러오는 헬퍼 모듈.

---

## utils

```text
utils/
└── config_loader.py    # YAML 파라미터 로더
```

YAML 파라미터 파일을 읽어 dataclass로 변환하는 공용 로더.
모든 모델이 파라미터 초기화에 내부적으로 사용.

---

## controllers

제어기 상세 설명은 [controllers/README.md](controllers/README.md) 참고.
