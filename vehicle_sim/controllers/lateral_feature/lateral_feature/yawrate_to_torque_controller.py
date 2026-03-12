"""
Integrated lateral controller: yaw-rate command to per-wheel steering torque commands.
"""

from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable, Dict, Optional, Tuple

import numpy as np
import yaml

from vehicle_sim.utils.config_loader import load_param

from .lateral_force_estimator import LateralForceEstimator, LateralForceEstimatorOptions
from .pid_controller import PIDController, PIDGains
from .slip_angle_estimator import SlipAngleEstimator, SlipAngleEstimatorOptions
from .steer_angle_ff import SteeringFeedforwardController, SteeringFeedforwardOptions
from .yaw_moment_allocator import YawMomentAllocator
from .yaw_moment_feedforward_controller import (
    YawMomentFeedforwardController,
    YawMomentFeedforwardOptions,
)


@dataclass
class LateralTorqueControllerBehavior:
    use_estimated_fy_bias: bool = False
    use_estimated_vy_ref: bool = False


@dataclass
class LateralSensorFrame:
    """Sensor snapshot used by the lateral controller."""

    yaw_rate: float
    ay: float
    vx: float
    steer_angle: Dict[str, float]
    wheel_speed: Dict[str, float] = field(default_factory=dict)
    fx_body: Optional[Dict[str, float]] = None
    timestamp: Optional[float] = None
    valid: bool = True


@dataclass
class LateralRequest:
    """Control request for one step."""

    yaw_rate_cmd: float
    yaw_accel_cmd: Optional[float] = None
    vy_cmd: Optional[float] = None
    fy_total_cmd: Optional[float] = None


@dataclass
class LateralOutput:
    """Controller output bundle for one step."""

    steer_torque_cmd: Dict[str, float]
    valid: bool = True
    debug: Dict[str, object] = field(default_factory=dict)


class LateralYawRateTorqueController:
    """
    End-to-end controller chain:
    yaw-rate command -> yaw moment (FF + PID) -> wheel Fy -> steering angle -> steering torque.
    """

    def __init__(
        self,
        vehicle_body,
        dt: float,
        config: Optional[Dict] = None,
        config_path: Optional[str] = None,
    ) -> None:
        if dt <= 0.0:
            raise ValueError("dt must be positive")

        self.vehicle_body = vehicle_body
        self.dt = float(dt)
        self.config = self._resolve_config(config=config, config_path=config_path)

        yaw_rate_pid_cfg = self.config.get("yaw_rate_pid", {})
        yaw_rate_pid_gains = PIDGains(
            kp=float(yaw_rate_pid_cfg.get("kp", 0.0)),
            ki=float(yaw_rate_pid_cfg.get("ki", 0.0)),
            kd=float(yaw_rate_pid_cfg.get("kd", 0.0)),
        )
        self.yaw_rate_pid = PIDController(dt=self.dt, gains=yaw_rate_pid_gains)

        steer_pid_cfg = self.config.get("steer_torque_pid", {})
        base_steer_gains = PIDGains(
            kp=float(steer_pid_cfg.get("kp", 0.0)),
            ki=float(steer_pid_cfg.get("ki", 0.0)),
            kd=float(steer_pid_cfg.get("kd", 0.0)),
        )
        per_wheel_cfg = steer_pid_cfg.get("per_wheel", {})
        self.steer_torque_pid: Dict[str, PIDController] = {}
        for label in self.vehicle_body.wheel_labels:
            wheel_cfg = per_wheel_cfg.get(label, {}) if isinstance(per_wheel_cfg, dict) else {}
            wheel_gains = PIDGains(
                kp=float(wheel_cfg.get("kp", base_steer_gains.kp)),
                ki=float(wheel_cfg.get("ki", base_steer_gains.ki)),
                kd=float(wheel_cfg.get("kd", base_steer_gains.kd)),
            )
            self.steer_torque_pid[label] = PIDController(dt=self.dt, gains=wheel_gains)

        yaw_ff_cfg = self.config.get("yaw_moment_feedforward", {})
        yaw_ff_options = YawMomentFeedforwardOptions(
            max_yaw_accel=self._to_optional_float(yaw_ff_cfg.get("max_yaw_accel")),
            torque_limit=self._to_optional_float(yaw_ff_cfg.get("torque_limit")),
        )
        self.yaw_moment_ff = YawMomentFeedforwardController(dt=self.dt, options=yaw_ff_options)

        allocator_cfg = self.config.get("yaw_moment_allocator", {})
        self.yaw_moment_allocator = YawMomentAllocator(
            min_abs_x=float(allocator_cfg.get("min_abs_x", 1e-6))
        )

        lateral_force_cfg = self.config.get("lateral_force_estimator", {})
        lateral_force_options = LateralForceEstimatorOptions(
            ay_bias=float(lateral_force_cfg.get("ay_bias", 0.0)),
            lowpass_tau=self._to_optional_float(lateral_force_cfg.get("lowpass_tau")),
            max_abs_ay=self._to_optional_float(lateral_force_cfg.get("max_abs_ay")),
        )
        self.lateral_force_estimator = LateralForceEstimator(
            dt=self.dt,
            mass=self._resolve_mass(vehicle_body),
            options=lateral_force_options,
        )

        slip_cfg = self.config.get("slip_angle_estimator", {})
        slip_options = SlipAngleEstimatorOptions(
            ay_bias=float(slip_cfg.get("ay_bias", 0.0)),
            lowpass_tau=self._to_optional_float(slip_cfg.get("lowpass_tau")),
            vy_init=float(slip_cfg.get("vy_init", 0.0)),
            vy_limit=self._to_optional_float(slip_cfg.get("vy_limit")),
            leak_tau=self._to_optional_float(slip_cfg.get("leak_tau")),
            min_vx=float(slip_cfg.get("min_vx", 0.1)),
        )
        self.slip_angle_estimator = SlipAngleEstimator(
            dt=self.dt,
            wheel_xy=self._build_wheel_xy(vehicle_body),
            options=slip_options,
        )

        steer_ff_cfg = self.config.get("steering_feedforward", {})
        steer_ff_options = SteeringFeedforwardOptions(
            clamp_fy=bool(steer_ff_cfg.get("clamp_fy", True)),
            unwrap_delta=bool(steer_ff_cfg.get("unwrap_delta", True)),
        )
        self.steering_ff = SteeringFeedforwardController(options=steer_ff_options)

        behavior_cfg = self.config.get("behavior", {})
        self.behavior = LateralTorqueControllerBehavior(
            use_estimated_fy_bias=bool(behavior_cfg.get("use_estimated_fy_bias", False)),
            use_estimated_vy_ref=bool(behavior_cfg.get("use_estimated_vy_ref", False)),
        )

        limit_cfg = self.config.get("output_limits", {})
        self.yaw_moment_limit = self._to_optional_float(limit_cfg.get("yaw_moment"))
        self.steer_torque_limit = self._to_optional_float(limit_cfg.get("steer_torque"))

        self.last_debug: Dict[str, object] = {}

    @staticmethod
    def _resolve_mass(vehicle_body) -> float:
        if hasattr(vehicle_body, "params") and hasattr(vehicle_body.params, "m_total"):
            mass = float(vehicle_body.params.m_total)
        elif hasattr(vehicle_body, "params") and hasattr(vehicle_body.params, "m"):
            mass = float(vehicle_body.params.m)
        else:
            raise AttributeError("vehicle_body.params must provide m_total or m")

        if mass <= 0.0:
            raise ValueError("vehicle mass must be positive")
        return mass

    @staticmethod
    def _build_wheel_xy(vehicle_body) -> Dict[str, Tuple[float, float]]:
        if hasattr(vehicle_body, "corner_offsets"):
            return {
                label: (
                    float(vehicle_body.corner_offsets[label]["x"]),
                    float(vehicle_body.corner_offsets[label]["y"]),
                )
                for label in vehicle_body.wheel_labels
            }

        if (
            hasattr(vehicle_body, "corner_signs")
            and hasattr(vehicle_body.params, "L_wheelbase")
            and hasattr(vehicle_body.params, "L_track")
        ):
            wheel_xy = {}
            for label in vehicle_body.wheel_labels:
                signs = vehicle_body.corner_signs[label]
                x_i = (vehicle_body.params.L_wheelbase / 2.0) * signs["pitch"]
                y_i = (vehicle_body.params.L_track / 2.0) * signs["roll"]
                wheel_xy[label] = (float(x_i), float(y_i))
            return wheel_xy

        raise AttributeError(
            "vehicle_body must provide corner_offsets or corner_signs/L_wheelbase/L_track"
        )

    @staticmethod
    def _to_optional_float(value) -> Optional[float]:
        if value is None:
            return None
        return float(value)

    @staticmethod
    def _resolve_config(
        config: Optional[Dict],
        config_path: Optional[str],
    ) -> Dict:
        if config is not None:
            if not isinstance(config, dict):
                raise TypeError("config must be a dict")
            if "lateral_controller" in config and isinstance(config["lateral_controller"], dict):
                return dict(config["lateral_controller"])
            return dict(config)

        if config_path is None:
            loaded = load_param("lateral_controller", config_path=None)
            return loaded if isinstance(loaded, dict) else {}

        yaml_path = Path(config_path)
        if not yaml_path.exists():
            raise FileNotFoundError(f"Config file not found: {yaml_path}")
        with open(yaml_path, "r", encoding="utf-8") as f:
            loaded = yaml.safe_load(f) or {}
        if not isinstance(loaded, dict):
            raise ValueError("YAML config must be a mapping at top level")
        if "lateral_controller" in loaded and isinstance(loaded["lateral_controller"], dict):
            return dict(loaded["lateral_controller"])
        return dict(loaded)

    def reset(self) -> None:
        """Reset all internal controller and estimator states."""
        self.yaw_rate_pid.reset()
        for pid in self.steer_torque_pid.values():
            pid.reset()
        self.yaw_moment_ff.reset()
        self.lateral_force_estimator.reset()
        self.slip_angle_estimator.reset()
        self.steering_ff.reset()
        self.last_debug = {}

    def sensor_from_vehicle(
        self,
        fx_body: Optional[Dict[str, float]] = None,
        timestamp: Optional[float] = None,
    ) -> LateralSensorFrame:
        """Build a sensor frame from the current VehicleBody state."""
        return LateralSensorFrame(
            yaw_rate=float(self.vehicle_body.state.yaw_rate),
            ay=float(getattr(self.vehicle_body.state, "ay_prev", 0.0)),
            vx=float(self.vehicle_body.state.velocity_x),
            steer_angle={
                label: float(self.vehicle_body.corners[label].state.steering_angle)
                for label in self.vehicle_body.wheel_labels
            },
            wheel_speed={
                label: float(self.vehicle_body.corners[label].state.omega_wheel)
                for label in self.vehicle_body.wheel_labels
            },
            fx_body=fx_body,
            timestamp=timestamp,
            valid=True,
        )

    def _resolve_delta_map(self, steer_angle: Optional[Dict[str, float]]) -> Dict[str, float]:
        steer_angle = steer_angle or {}
        delta_map: Dict[str, float] = {}
        for label in self.vehicle_body.wheel_labels:
            if label in steer_angle:
                delta_map[label] = float(steer_angle[label])
            else:
                delta_map[label] = float(self.vehicle_body.corners[label].state.steering_angle)
        return delta_map

    def _compute_control(
        self,
        yaw_rate_cmd: float,
        yaw_rate_meas: float,
        vx_meas: float,
        ay_meas: float,
        yaw_accel_cmd: Optional[float],
        fx_body: Optional[Dict[str, float]],
        fy_total_cmd: Optional[float],
        delta_map: Dict[str, float],
        vy_cmd: Optional[float],
    ) -> Tuple[Dict[str, float], Dict[str, object]]:
        yaw_rate_cmd = float(yaw_rate_cmd)
        yaw_rate_meas = float(yaw_rate_meas)
        vx_meas = float(vx_meas)
        ay_meas = float(ay_meas)

        yaw_error = float(yaw_rate_cmd - yaw_rate_meas)
        mz_fb = float(self.yaw_rate_pid.update(yaw_error))
        mz_ff = float(
            self.yaw_moment_ff.compute_moment(
                vehicle_body=self.vehicle_body,
                yaw_rate_cmd=yaw_rate_cmd,
                yaw_accel_cmd=yaw_accel_cmd,
            )
        )
        mz_cmd = float(mz_ff + mz_fb)
        if self.yaw_moment_limit is not None:
            mz_cmd = float(np.clip(mz_cmd, -self.yaw_moment_limit, self.yaw_moment_limit))

        fy_total_est = float(self.lateral_force_estimator.update(ay_meas))
        alpha_est, vy_est = self.slip_angle_estimator.update(
            vx=vx_meas,
            yaw_rate=yaw_rate_meas,
            ay_meas=ay_meas,
            delta_map=delta_map,
        )

        fy_total_for_allocator = fy_total_cmd
        if fy_total_for_allocator is None and self.behavior.use_estimated_fy_bias:
            fy_total_for_allocator = fy_total_est

        vy_ref = (
            float(vy_cmd)
            if vy_cmd is not None
            else (float(vy_est) if self.behavior.use_estimated_vy_ref else 0.0)
        )

        fy_wheel_cmd = self.yaw_moment_allocator.allocate(
            vehicle_body=self.vehicle_body,
            Mz_d=mz_cmd,
            Fx_body=fx_body,
            Fy_total_cmd=fy_total_for_allocator,
        )
        delta_cmd = self.steering_ff.compute_delta_cmd(
            vehicle_body=self.vehicle_body,
            fy_wheel_cmd=fy_wheel_cmd,
            vx_cmd=vx_meas,
            yaw_rate_cmd=yaw_rate_cmd,
            vy_cmd=vy_ref,
            c_alpha_override=None,
        )

        torque_cmd: Dict[str, float] = {}
        for label in self.vehicle_body.wheel_labels:
            steer_error = float(delta_cmd.get(label, 0.0) - delta_map.get(label, 0.0))
            torque = float(self.steer_torque_pid[label].update(steer_error))
            if self.steer_torque_limit is not None:
                torque = float(np.clip(torque, -self.steer_torque_limit, self.steer_torque_limit))
            torque_cmd[label] = torque

        debug = {
            "yaw_error": yaw_error,
            "mz_ff": mz_ff,
            "mz_fb": mz_fb,
            "mz_cmd": mz_cmd,
            "fy_total_est": fy_total_est,
            "fy_total_for_allocator": fy_total_for_allocator,
            "alpha_est": alpha_est,
            "vy_est": float(vy_est),
            "vy_ref": vy_ref,
            "fy_wheel_cmd": fy_wheel_cmd,
            "delta_cmd": delta_cmd,
            "delta_meas": delta_map,
        }
        self.last_debug = debug
        return torque_cmd, debug

    def update(self, sensor: LateralSensorFrame, request: LateralRequest) -> LateralOutput:
        """
        Real-car style update API using structured sensor/request messages.
        """
        if not sensor.valid:
            zero_cmd = {label: 0.0 for label in self.vehicle_body.wheel_labels}
            debug = {"reason": "invalid_sensor_frame", "sensor_timestamp": sensor.timestamp}
            self.last_debug = debug
            return LateralOutput(steer_torque_cmd=zero_cmd, valid=False, debug=debug)

        delta_map = self._resolve_delta_map(sensor.steer_angle)
        torque_cmd, debug = self._compute_control(
            yaw_rate_cmd=float(request.yaw_rate_cmd),
            yaw_rate_meas=float(sensor.yaw_rate),
            vx_meas=float(sensor.vx),
            ay_meas=float(sensor.ay),
            yaw_accel_cmd=request.yaw_accel_cmd,
            fx_body=sensor.fx_body,
            fy_total_cmd=request.fy_total_cmd,
            delta_map=delta_map,
            vy_cmd=request.vy_cmd,
        )
        debug["sensor_timestamp"] = sensor.timestamp
        debug["sensor_wheel_speed"] = dict(sensor.wheel_speed)
        return LateralOutput(steer_torque_cmd=torque_cmd, valid=True, debug=debug)

    def step(
        self,
        yaw_rate_cmd: float,
        yaw_rate_meas: Optional[float] = None,
        vx_meas: Optional[float] = None,
        ay_meas: Optional[float] = None,
        yaw_accel_cmd: Optional[float] = None,
        fx_body: Optional[Dict[str, float]] = None,
        fy_total_cmd: Optional[float] = None,
        delta_meas: Optional[Dict[str, float]] = None,
        vy_cmd: Optional[float] = None,
    ) -> Dict[str, float]:
        """
        Backward-compatible wrapper around update(...).
        """
        sensor = LateralSensorFrame(
            yaw_rate=(
                float(yaw_rate_meas)
                if yaw_rate_meas is not None
                else float(self.vehicle_body.state.yaw_rate)
            ),
            ay=(
                float(ay_meas)
                if ay_meas is not None
                else float(getattr(self.vehicle_body.state, "ay_prev", 0.0))
            ),
            vx=(
                float(vx_meas)
                if vx_meas is not None
                else float(self.vehicle_body.state.velocity_x)
            ),
            steer_angle=self._resolve_delta_map(delta_meas),
            wheel_speed={},
            fx_body=fx_body,
            valid=True,
        )
        request = LateralRequest(
            yaw_rate_cmd=float(yaw_rate_cmd),
            yaw_accel_cmd=yaw_accel_cmd,
            vy_cmd=vy_cmd,
            fy_total_cmd=fy_total_cmd,
        )
        return self.update(sensor=sensor, request=request).steer_torque_cmd

    def step_with_debug(
        self,
        yaw_rate_cmd: float,
        yaw_rate_meas: Optional[float] = None,
        vx_meas: Optional[float] = None,
        ay_meas: Optional[float] = None,
        yaw_accel_cmd: Optional[float] = None,
        fx_body: Optional[Dict[str, float]] = None,
        fy_total_cmd: Optional[float] = None,
        delta_meas: Optional[Dict[str, float]] = None,
        vy_cmd: Optional[float] = None,
    ) -> Tuple[Dict[str, float], Dict[str, object]]:
        """
        Backward-compatible debug wrapper around update(...).
        """
        sensor = LateralSensorFrame(
            yaw_rate=(
                float(yaw_rate_meas)
                if yaw_rate_meas is not None
                else float(self.vehicle_body.state.yaw_rate)
            ),
            ay=(
                float(ay_meas)
                if ay_meas is not None
                else float(getattr(self.vehicle_body.state, "ay_prev", 0.0))
            ),
            vx=(
                float(vx_meas)
                if vx_meas is not None
                else float(self.vehicle_body.state.velocity_x)
            ),
            steer_angle=self._resolve_delta_map(delta_meas),
            wheel_speed={},
            fx_body=fx_body,
            valid=True,
        )
        request = LateralRequest(
            yaw_rate_cmd=float(yaw_rate_cmd),
            yaw_accel_cmd=yaw_accel_cmd,
            vy_cmd=vy_cmd,
            fy_total_cmd=fy_total_cmd,
        )
        out = self.update(sensor=sensor, request=request)
        return out.steer_torque_cmd, out.debug

    def step_from_vehicle(
        self,
        yaw_rate_cmd: float,
        yaw_accel_cmd: Optional[float] = None,
        fx_body: Optional[Dict[str, float]] = None,
        fy_total_cmd: Optional[float] = None,
        vy_cmd: Optional[float] = None,
        timestamp: Optional[float] = None,
    ) -> Dict[str, float]:
        """
        Convenience one-liner step:
        uses yaw rate, longitudinal speed, lateral acceleration, and steering angles from vehicle_body.
        """
        sensor = self.sensor_from_vehicle(fx_body=fx_body, timestamp=timestamp)
        request = LateralRequest(
            yaw_rate_cmd=float(yaw_rate_cmd),
            yaw_accel_cmd=yaw_accel_cmd,
            vy_cmd=vy_cmd,
            fy_total_cmd=fy_total_cmd,
        )
        return self.update(sensor=sensor, request=request).steer_torque_cmd


def build_lateral_torque_controller(
    vehicle_body,
    dt: float,
    config: Optional[Dict] = None,
    config_path: Optional[str] = None,
) -> LateralYawRateTorqueController:
    """Factory for the integrated yawrate-to-steering-torque controller."""
    return LateralYawRateTorqueController(
        vehicle_body=vehicle_body,
        dt=dt,
        config=config,
        config_path=config_path,
    )


def create_lateral_torque_stepper(
    vehicle_body,
    dt: float,
    config: Optional[Dict] = None,
    config_path: Optional[str] = None,
) -> Callable[..., Dict[str, float]]:
    """
    Return a one-line callable:
    torque_cmd = stepper(yaw_rate_cmd=...)
    """
    controller = build_lateral_torque_controller(
        vehicle_body=vehicle_body,
        dt=dt,
        config=config,
        config_path=config_path,
    )
    return controller.step_from_vehicle
