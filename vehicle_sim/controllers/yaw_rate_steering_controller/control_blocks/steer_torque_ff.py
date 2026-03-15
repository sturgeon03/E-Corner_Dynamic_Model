"""Steering motor torque feedforward using explicit parameters."""

from dataclasses import dataclass
from typing import Dict, Iterable, Optional

import numpy as np


@dataclass
class SteeringTorqueFFOptions:
    max_accel: Optional[float] = None
    torque_limit: Optional[float] = None


@dataclass
class SteeringFFParams:
    J_cq: float
    B_cq: float
    gear_ratio: float
    max_rate: float
    max_angle_pos: float
    max_angle_neg: float


class SteeringMotorTorqueFF:
    """Compute motor torque feedforward from steering commands."""

    def __init__(self, dt: float, options: Optional[SteeringTorqueFFOptions] = None) -> None:
        if dt <= 0.0:
            raise ValueError("dt must be positive")
        self.dt = float(dt)
        self.options = options or SteeringTorqueFFOptions()
        self._prev_delta_cmd: Dict[str, float] = {}
        self._prev_delta_dot_cmd: Dict[str, float] = {}

    def reset(self) -> None:
        self._prev_delta_cmd.clear()
        self._prev_delta_dot_cmd.clear()

    def compute_torque(
        self,
        labels: Iterable[str],
        delta_cmd: Dict[str, float],
        align_cmd: Dict[str, float],
        params_map: Dict[str, SteeringFFParams],
        delta_dot_cmd: Optional[Dict[str, float]] = None,
        delta_ddot_cmd: Optional[Dict[str, float]] = None,
    ) -> tuple[Dict[str, float], Dict[str, float]]:
        """
        Returns:
            (motor_torque, axis_torque) per wheel
        """
        labels = list(labels)
        motor_torque: Dict[str, float] = {}
        axis_torque: Dict[str, float] = {}

        for label in labels:
            params = params_map[label]
            delta_cmd_i = float(delta_cmd.get(label, 0.0))
            delta_cmd_i = self._clip_angle(delta_cmd_i, params.max_angle_neg, params.max_angle_pos)

            prev_delta = self._prev_delta_cmd.get(label, delta_cmd_i)
            has_prev_delta_dot = label in self._prev_delta_dot_cmd
            prev_delta_dot = self._prev_delta_dot_cmd.get(label, 0.0)

            if delta_dot_cmd is None:
                delta_dot_cmd_i = (delta_cmd_i - prev_delta) / self.dt
            else:
                delta_dot_cmd_i = float(delta_dot_cmd.get(label, 0.0))
            delta_dot_cmd_i = float(np.clip(delta_dot_cmd_i, -params.max_rate, params.max_rate))

            if delta_ddot_cmd is None:
                if not has_prev_delta_dot:
                    delta_ddot_cmd_i = 0.0
                else:
                    delta_ddot_cmd_i = (delta_dot_cmd_i - prev_delta_dot) / self.dt
            else:
                delta_ddot_cmd_i = float(delta_ddot_cmd.get(label, 0.0))

            if self.options.max_accel is not None:
                delta_ddot_cmd_i = float(
                    np.clip(delta_ddot_cmd_i, -self.options.max_accel, self.options.max_accel)
                )

            t_align = float(align_cmd.get(label, 0.0))
            t_axis = params.J_cq * delta_ddot_cmd_i + params.B_cq * delta_dot_cmd_i + t_align

            if self.options.torque_limit is not None:
                t_axis = float(np.clip(t_axis, -self.options.torque_limit, self.options.torque_limit))

            axis_torque[label] = float(t_axis)
            motor_torque[label] = float(self._axis_to_motor_torque(t_axis, params.gear_ratio))

            self._prev_delta_cmd[label] = delta_cmd_i
            self._prev_delta_dot_cmd[label] = delta_dot_cmd_i

        return motor_torque, axis_torque

    @staticmethod
    def _axis_to_motor_torque(torque_axis: float, gear_ratio: float) -> float:
        scale = float(gear_ratio)
        if abs(scale) < 1e-6:
            return float(torque_axis)
        return float(torque_axis) / scale

    @staticmethod
    def _clip_angle(angle: float, neg: float, pos: float) -> float:
        lower = min(neg, pos)
        upper = max(neg, pos)
        return float(np.clip(angle, lower, upper))
