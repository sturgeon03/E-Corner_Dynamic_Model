"""
Active Anti-Roll Bar (AARB) Controller
코너별 서스펜션 스트로크 높이를 입력으로 받아 좌우 롤 모멘트를 억제하는 제어기
"""

from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Optional

import numpy as np
import yaml


@dataclass
class ActiveAntiRollBarGains:
    """Active Anti-Roll Bar PD 게인 (축별)"""
    # Front ARB
    k_arb_front: float = 30000.0    # 전륜 ARB 강성 [N/m]
    c_arb_front: float = 2000.0     # 전륜 ARB 댐핑 [N*s/m]

    # Rear ARB
    k_arb_rear: float = 25000.0     # 후륜 ARB 강성 [N/m]
    c_arb_rear: float = 1500.0      # 후륜 ARB 댐핑 [N*s/m]

    # Track width (좌우 간격)
    track_width: float = 1.634      # 트랙 폭 [m]

    # 힘→토크 변환 파라미터
    lead:       float = 0.01        # 스크루 리드 [m/rev]
    efficiency: float = 0.9         # 액추에이터 효율 [-]


class ActiveAntiRollBarController:
    """
    Active Anti-Roll Bar (AARB) Controller

    코너별 서스펜션 스트로크 높이(delta_s)를 입력으로 받아
    전·후륜 좌우 높이차로부터 anti-roll 모멘트를 계산하고
    각 코너에 힘(또는 토크)으로 분배한다.

    제어 법칙 (축별):
        Δs_front = delta_s_FL - delta_s_FR
        Δs_rear  = delta_s_RL - delta_s_RR

        M_arb_front = k_arb_front * Δs_front + c_arb_front * Δs_front_dot
        M_arb_rear  = k_arb_rear  * Δs_rear  + c_arb_rear  * Δs_rear_dot

    코너 힘 분배:
        F_R = +M_arb / track_width
        F_L = -M_arb / track_width

    입력: 코너별 SuspensionState (delta_s [m], delta_s_dot [m/s])
    출력: 코너별 F_arb [N] 또는 T_susp [N*m]
    """

    def __init__(
        self,
        gains: Optional[ActiveAntiRollBarGains] = None,
        config_path: Optional[str] = None,
    ):
        if config_path is not None:
            with open(config_path, 'r', encoding='utf-8') as f:
                cfg = yaml.safe_load(f)['aarb']
            self.gains = ActiveAntiRollBarGains(
                k_arb_front = float(cfg['k_arb_front']),
                c_arb_front = float(cfg['c_arb_front']),
                k_arb_rear  = float(cfg['k_arb_rear']),
                c_arb_rear  = float(cfg['c_arb_rear']),
                track_width = float(cfg['track_width']),
                lead        = float(cfg['lead']),
                efficiency  = float(cfg['efficiency']),
            )
        else:
            self.gains = gains if gains is not None else ActiveAntiRollBarGains()

        # 내부 상태 (진단용)
        self.M_arb_front: float = 0.0  # 전륜 ARB 모멘트 [N*m]
        self.M_arb_rear: float = 0.0   # 후륜 ARB 모멘트 [N*m]
        self.delta_s_front: float = 0.0  # 전륜 스트로크 차 [m]
        self.delta_s_rear: float = 0.0   # 후륜 스트로크 차 [m]

    def reset(self) -> None:
        """제어기 상태 리셋"""
        self.M_arb_front = 0.0
        self.M_arb_rear = 0.0
        self.delta_s_front = 0.0
        self.delta_s_rear = 0.0

    def update(
        self,
        state: Dict[str, object],
        output_type: str = "torque",
        enabled: bool = True,
    ) -> Dict[str, float]:
        """
        코너별 스트로크 높이로부터 ARB 힘/토크를 계산한다.

        Args:
            state:       코너별 스트로크 변위·속도
                         {"FL": {"stroke": [m], "stroke_rate": [m/s]}, "FR": ..., ...}
            output_type: 출력 단위 — "force" [N] 또는 "torque" [N*m]
            enabled:     False 이면 토크를 출력하지 않고 0을 반환

        Returns:
            {"FL": value, "FR": value, "RL": value, "RR": value}
        """
        if not enabled:
            self.reset()
            return {c: 0.0 for c in ("FL", "FR", "RL", "RR")}

        # 전·후륜 좌우 스트로크 높이차 및 속도차
        self.delta_s_front = state["FL"]["stroke"] - state["FR"]["stroke"]
        self.delta_s_rear  = state["RL"]["stroke"] - state["RR"]["stroke"]

        delta_s_dot_front = state["FL"]["stroke_rate"] - state["FR"]["stroke_rate"]
        delta_s_dot_rear  = state["RL"]["stroke_rate"] - state["RR"]["stroke_rate"]

        # 높이차·속도차로부터 축별 ARB 모멘트 계산 (M = k·Δs + c·Δs_dot)
        self.M_arb_front = (
            self.gains.k_arb_front * self.delta_s_front
            + self.gains.c_arb_front * delta_s_dot_front
        )
        self.M_arb_rear = (
            self.gains.k_arb_rear * self.delta_s_rear
            + self.gains.c_arb_rear * delta_s_dot_rear
        )

        # 모멘트를 트랙 폭으로 나눠 좌우 코너 힘으로 분배 (F = ±M / track)
        track = self.gains.track_width

        F_arb = {
            "FL": -self.M_arb_front / track,
            "FR": +self.M_arb_front / track,
            "RL": -self.M_arb_rear / track,
            "RR": +self.M_arb_rear / track,
        }

        if output_type == "force":
            return F_arb
        elif output_type == "torque":
            return {corner: self._force_to_torque(F) for corner, F in F_arb.items()}
        else:
            raise ValueError(f"Invalid output_type: {output_type}. Must be 'force' or 'torque'.")

    def _force_to_torque(self, F_act: float) -> float:
        """F [N] → T [N*m] : T = F * lead / (2π * efficiency)"""
        return float(F_act * self.gains.lead / (2.0 * np.pi * self.gains.efficiency))

    def get_state(self) -> Dict[str, float]:
        """
        현재 제어기 상태 조회

        Returns:
            state: {
                "M_arb_front": 전륜 ARB 모멘트 [N*m],
                "M_arb_rear": 후륜 ARB 모멘트 [N*m],
                "delta_s_front": 전륜 스트로크 차 [m],
                "delta_s_rear": 후륜 스트로크 차 [m]
            }
        """
        return {
            "M_arb_front": self.M_arb_front,
            "M_arb_rear": self.M_arb_rear,
            "delta_s_front": self.delta_s_front,
            "delta_s_rear": self.delta_s_rear,
        }
