"""사인 파형 요레이트 지령 시나리오."""

from __future__ import annotations

from typing import Optional

import numpy as np

_KPH = 1000.0 / 3600.0


def _half_cosine_ramp(t: float, ramp_time: float) -> float:
    """0→1 반코사인 램프 윈도우 함수."""
    if ramp_time <= 0.0:
        return 1.0
    if t <= 0.0:
        return 0.0
    if t >= ramp_time:
        return 1.0
    return float(0.5 * (1.0 - np.cos(np.pi * t / ramp_time)))


def _yaw_rate_wave(
    t: float,
    amp: float,
    freq_hz: float,
    start_delay: float,
    ramp_time: float,
) -> float:
    """사인 파형 요레이트 지령 — 시작 지연 후 반코사인 램프로 진폭을 서서히 증가시킨다."""
    if t < start_delay:
        return 0.0
    t_rel = float(t - start_delay)
    win = _half_cosine_ramp(t_rel, ramp_time)
    return float(amp * win * np.sin(2.0 * np.pi * freq_hz * t_rel))


def generate(
    dt: float,
    target_kph: float = 30.0,
    amp: float = 0.05,
    freq_hz: float = 0.25,
    start_delay: float = 1.0,
    ramp_time: float = 1.2,
    duration: float = 5.5,
) -> "YawRateSineScenario":
    """
    사인 파형 요레이트 지령 시나리오를 생성한다.

    Args:
        dt          : 시뮬레이션 시간 간격 [s] (제어기 dt에 맞춤)
        target_kph  : 목표 종방향 속도 [kph]
        amp         : 요레이트 지령 진폭 [rad/s]
        freq_hz     : 요레이트 지령 주파수 [Hz]
        start_delay : 지령 시작 지연 [s]
        ramp_time   : 진폭 램프 시간 [s]
        duration    : 시뮬레이션 총 시간 [s]

    Returns:
        YawRateSineScenario
    """
    t = np.arange(int(round(duration / dt))) * dt
    yaw_rate_ref = np.array(
        [_yaw_rate_wave(ti, amp, freq_hz, start_delay, ramp_time) for ti in t]
    )

    data = dict(
        time=t,
        target_mps=float(target_kph * _KPH),
        yaw_rate_ref=yaw_rate_ref,
    )

    print(
        f"시나리오 생성 완료: steps={len(t)}, t_end={float(t[-1]):.3f}s, "
        f"target={target_kph:.1f} kph, amp={amp:.3f} rad/s, freq={freq_hz:.2f} Hz"
    )
    return YawRateSineScenario(data)


class YawRateSineScenario(dict):
    """요레이트 사인 파형 시나리오 컨테이너."""

    def yaw_rate_ref(self, idx: int) -> float:
        """인덱스에 해당하는 요레이트 지령 [rad/s]을 반환한다."""
        return float(self["yaw_rate_ref"][idx])
