"""
Steering feedforward from per-wheel lateral-force commands.
"""

from dataclasses import dataclass
from typing import Dict, Optional, Tuple

import numpy as np


@dataclass
class SteeringFeedforwardOptions:
    clamp_fy: bool = True
    unwrap_delta: bool = True


class SteeringFeedforwardController:
    """
    Convert wheel-frame lateral-force commands into steering-angle commands via a tire inverse model.

    alpha_cmd = -Fy_wheel_cmd / C_alpha
    beta_ref = atan2(Vy_ref_body, Vx_ref_body)
    delta_cmd = beta_ref - alpha_cmd
    """

    def __init__(self, options: Optional[SteeringFeedforwardOptions] = None) -> None:
        self.options = options or SteeringFeedforwardOptions()
        self._prev_delta_cmd: Dict[str, float] = {}

    def reset(self) -> None:
        """Reset angle unwrapping memory."""
        self._prev_delta_cmd.clear()

    @staticmethod
    def _wrap_to_pi(angle: float) -> float:
        return float((angle + np.pi) % (2.0 * np.pi) - np.pi)

    @classmethod
    def _unwrap_near(cls, angle: float, reference: float) -> float:
        return float(reference + cls._wrap_to_pi(angle - reference))

    @staticmethod
    def _resolve_wheel_xy(vehicle_body, label: str) -> Tuple[float, float]:
        """Resolve wheel center coordinates in the body frame."""
        if hasattr(vehicle_body, "corner_offsets") and label in vehicle_body.corner_offsets:
            offset = vehicle_body.corner_offsets[label]
            return float(offset["x"]), float(offset["y"])

        # Backward compatibility for older geometry conventions.
        if (
            hasattr(vehicle_body, "corner_signs")
            and hasattr(vehicle_body, "params")
            and hasattr(vehicle_body.params, "L_wheelbase")
            and hasattr(vehicle_body.params, "L_track")
        ):
            signs = vehicle_body.corner_signs[label]
            x_i = (vehicle_body.params.L_wheelbase / 2.0) * signs["pitch"]
            y_i = (vehicle_body.params.L_track / 2.0) * signs["roll"]
            return float(x_i), float(y_i)

        raise AttributeError(
            "vehicle_body must provide corner_offsets or corner_signs/L_wheelbase/L_track"
        )

    def compute_delta_cmd(
        self,
        vehicle_body,
        fy_wheel_cmd: Dict[str, float],
        vx_cmd: float,
        yaw_rate_cmd: float = 0.0,
        vy_cmd: float = 0.0,
        c_alpha_override: Optional[Dict[str, float]] = None,
    ) -> Dict[str, float]:
        """
        Args:
            vehicle_body: VehicleBody instance.
            fy_wheel_cmd: per-wheel lateral-force commands [N] in wheel frame.
            vx_cmd: reference longitudinal speed [m/s] in body frame.
            yaw_rate_cmd: reference yaw rate [rad/s].
            vy_cmd: reference lateral speed [m/s] in body frame.
            c_alpha_override: optional per-wheel C_alpha overrides [N/rad].

        Returns:
            Per-wheel steering-angle commands [rad].
        """
        delta_cmd, _ = self._compute_delta_cmd_impl(
            vehicle_body=vehicle_body,
            fy_wheel_cmd=fy_wheel_cmd,
            vx_cmd=vx_cmd,
            yaw_rate_cmd=yaw_rate_cmd,
            vy_cmd=vy_cmd,
            c_alpha_override=c_alpha_override,
            with_debug=False,
        )
        return delta_cmd

    def compute_delta_cmd_with_debug(
        self,
        vehicle_body,
        fy_wheel_cmd: Dict[str, float],
        vx_cmd: float,
        yaw_rate_cmd: float = 0.0,
        vy_cmd: float = 0.0,
        c_alpha_override: Optional[Dict[str, float]] = None,
    ) -> Tuple[Dict[str, float], Dict[str, Dict[str, float]]]:
        """
        Returns:
            (delta_cmd, debug) where debug has keys: alpha_cmd, beta_ref, fy_wheel_clamped.
        """
        return self._compute_delta_cmd_impl(
            vehicle_body=vehicle_body,
            fy_wheel_cmd=fy_wheel_cmd,
            vx_cmd=vx_cmd,
            yaw_rate_cmd=yaw_rate_cmd,
            vy_cmd=vy_cmd,
            c_alpha_override=c_alpha_override,
            with_debug=True,
        )

    def _compute_delta_cmd_impl(
        self,
        vehicle_body,
        fy_wheel_cmd: Dict[str, float],
        vx_cmd: float,
        yaw_rate_cmd: float = 0.0,
        vy_cmd: float = 0.0,
        c_alpha_override: Optional[Dict[str, float]] = None,
        with_debug: bool = False,
    ) -> Tuple[Dict[str, float], Dict[str, Dict[str, float]]]:
        vx_cmd = float(vx_cmd)
        yaw_rate_cmd = float(yaw_rate_cmd)
        vy_cmd = float(vy_cmd)

        delta_cmd: Dict[str, float] = {}
        alpha_cmd_map: Dict[str, float] = {}
        beta_ref_map: Dict[str, float] = {}
        fy_clamped_map: Dict[str, float] = {}
        labels = list(vehicle_body.wheel_labels)

        if set(self._prev_delta_cmd.keys()) != set(labels):
            self._prev_delta_cmd = {}

        for label in labels:
            corner = vehicle_body.corners[label]
            fy_in = float(fy_wheel_cmd.get(label, 0.0))
            ref = float(self._prev_delta_cmd.get(label, 0.0))

            def _clamp_fy_wheel(fy_wheel_value: float) -> float:
                if not self.options.clamp_fy:
                    return float(fy_wheel_value)
                fz = float(corner.state.F_z)
                if fz <= 0.0:
                    return float(fy_wheel_value)
                mu = float(corner.lateral_tire.params.mu)
                return float(np.clip(fy_wheel_value, -mu * abs(fz), mu * abs(fz)))

            if c_alpha_override is not None and label in c_alpha_override:
                c_alpha = float(c_alpha_override.get(label, 0.0))
            else:
                c_alpha = float(corner.lateral_tire.params.C_alpha)
            if c_alpha == 0.0:
                raise ValueError("C_alpha must be non-zero for feedforward steering")

            x_i, y_i = self._resolve_wheel_xy(vehicle_body, label)
            vx_ref_body = vx_cmd - yaw_rate_cmd * y_i
            vy_ref_body = vy_cmd + yaw_rate_cmd * x_i
            beta_ref = float(np.arctan2(vy_ref_body, vx_ref_body))

            fy_wheel = _clamp_fy_wheel(float(fy_in))
            alpha_cmd_value = -fy_wheel / c_alpha
            delta_raw = float(beta_ref - alpha_cmd_value)

            if self.options.unwrap_delta:
                delta_raw = self._unwrap_near(delta_raw, ref)

            delta_cmd[label] = float(delta_raw)
            self._prev_delta_cmd[label] = float(delta_raw)

            if with_debug:
                alpha_cmd_map[label] = float(alpha_cmd_value)
                beta_ref_map[label] = float(beta_ref)
                fy_clamped_map[label] = float(fy_wheel)

        debug: Dict[str, Dict[str, float]] = {}
        if with_debug:
            debug = {
                "alpha_cmd": alpha_cmd_map,
                "beta_ref": beta_ref_map,
                "fy_wheel_clamped": fy_clamped_map,
            }
        return delta_cmd, debug
