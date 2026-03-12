"""
Sinesweep2 시나리오 : 합성 토크 입력 생성
- 구동 토크 : 50 Nm (전 코너 고정)
- 조향 토크 : ±1 Nm, 2초마다 방향 전환 (전륜)
- 브레이크/노면 : 0
모든 출력이 토크이므로 body.update() 직접 사용 가능
"""

import numpy as np

CORNERS = ['FL', 'FR', 'RL', 'RR']


def generate(dt: float = 0.001, duration: float = 20.0) -> dict:
    """
    토크 기반 시나리오 데이터 생성

    Returns:
        data: {
            'time'      : [s]
            'drive_trq' : {corner: array [N·m]}  - 50 Nm 고정
            'steer_trq' : {corner: array [N·m]}  - ±1 Nm, 2s 주기 (전륜만)
            'brake_trq' : {corner: array [N·m]}  - 0
            'road_z'    : {corner: array [m]}    - 0
        }
    """
    t = np.arange(0, duration, dt)

    # 조향 토크 : 2초마다 +1 / -1 반복
    steer = np.where((t % 4.0) < 2.0, 1.0, -1.0)

    data = dict(
        time      = t,
        drive_trq = {c: np.full(len(t), 50.0) for c in CORNERS},
        steer_trq = {'FL': steer, 'FR': steer,
                     'RL': np.zeros(len(t)), 'RR': np.zeros(len(t))},
        brake_trq = {c: np.zeros(len(t)) for c in CORNERS},
        road_z    = {c: np.zeros(len(t)) for c in CORNERS},
    )

    print(f'시나리오 생성: dt={dt}s, duration={duration}s, steps={len(t)}')
    return data
