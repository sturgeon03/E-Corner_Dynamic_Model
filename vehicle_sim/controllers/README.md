# Controllers

차량 제어기 모듈 모음.

---

## 디렉토리 구조

```text
controllers/
├── anti_roll_bar_controller/
│   ├── controller.py                        # 제어기 본체
│   ├── param/
│   │   └── aarb_param.yaml                  # 게인 파라미터
│   ├── test_debug/
│   │   └── aarb_onoff_test.ipynb            # ON/OFF 비교 검증 노트북
│   └── README.md
└── yaw_rate_steering_controller/
    ├── controller.py                        # 메인 제어기
    ├── control_blocks/                      # 제어 블록 (PID, FF, allocator 등)
    ├── estimators/                          # 추정기 (lateral force, slip angle 등)
    ├── param/
    │   ├── controller_gains.yaml            # PID 게인
    │   └── controller_options.example.yaml  # 모드/옵션 템플릿
    ├── test_debug/
    │   └── yawrate_tracking_test.ipynb      # 요레이트 추종 검증 노트북
    └── README.md
```

> `config_adapter.py` (YAML → 런타임 옵션 변환)는 `vehicle_sim/utils/config_adapter.py`로 관리됨.

---

## 제어기 목록

| 제어기 | 설명 | 상세 |
|---|---|---|
| Active Anti-Roll Bar Controller | 좌우 서스펜션 스트로크 차 기반 안티롤 토크 제어기 | [README](anti_roll_bar_controller/README.md) |
| Yaw Rate Steering Controller | 목표 yaw rate 기반 통합 조향 제어기 | [README](yaw_rate_steering_controller/README.md) |
