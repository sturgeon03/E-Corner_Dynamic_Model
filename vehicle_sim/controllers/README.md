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

## 제어기 목록

| 제어기 | 설명 | 상세 |
|---|---|---|
| Active Anti-Roll Bar Controller | 좌우 서스펜션 스트로크 차 기반 안티롤 토크 제어기 | [README](anti_roll_bar_controller/README.md) |
| Yaw Rate Steering Controller | 목표 yaw rate 기반 통합 조향 제어기 | [README](yaw_rate_steering_controller/README.md) |
