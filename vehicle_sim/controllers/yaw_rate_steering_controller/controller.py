"""User-friendly yaw-rate steering controller.

This module exposes a stateful controller with two output modes:

    steering_motor_torque_cmd = controller.compute_torque_command(state, ref)
    steering_angle_cmd = controller.compute_angle_command(state, ref)

For production use, create one controller instance per controlled vehicle or
control loop and use one output mode consistently on that instance.
"""

from __future__ import annotations

from copy import deepcopy
from dataclasses import dataclass
from pathlib import Path
from types import SimpleNamespace
from typing import Any, Dict, Mapping, Optional, Tuple

import numpy as np

from vehicle_sim.utils.config_loader import load_param

from .control_blocks.pid_controller import PIDController, PIDGains
from .control_blocks.steer_angle_ff import SteeringFeedforwardController
from .control_blocks.steer_torque_ff import SteeringFFParams, SteeringMotorTorqueFF, SteeringTorqueFFOptions
from .control_blocks.yaw_moment_allocator import YawMomentAllocator
from .control_blocks.yaw_moment_feedforward_controller import YawMomentFeedforwardController
from .estimators.lateral_force_estimator import LateralForceEstimator, LateralForceEstimatorOptions
from .estimators.slip_angle_estimator import SlipAngleEstimator, SlipAngleEstimatorOptions
from .estimators.steering_param_estimator import ScalarClamp, ScalarRLS


WHEEL_LABELS: Tuple[str, ...] = ("FL", "FR", "RR", "RL")
WHEEL_SIGNS: Dict[str, Dict[str, int]] = {
    "FL": {"roll": 1, "pitch": 1},
    "FR": {"roll": -1, "pitch": 1},
    "RR": {"roll": -1, "pitch": -1},
    "RL": {"roll": 1, "pitch": -1},
}


@dataclass
class YawRateSteeringControllerOptions:
    dt: float = 0.01
    enable_yaw_feedback: bool = True
    enable_fy_feedback: bool = False
    fy_feedback_source: str = "estimate"  # measured | estimate
    enable_steer_feedback: bool = True
    enable_estimator: bool = False
    enable_b_estimator: bool = True
    enable_c_alpha_estimator: bool = True
    estimator_forgetting_factor: float = 0.995
    estimator_p0: float = 50.0
    b_estimator_forgetting_factor: Optional[float] = None
    b_estimator_p0: Optional[float] = None
    b_estimator_min_value: Optional[float] = None
    b_estimator_max_value: Optional[float] = None
    b_estimator_start_time: float = 0.0
    b_estimator_min_samples: int = 0
    b_estimator_sample_decimation: int = 1
    b_estimator_dot_min_abs: float = 0.0
    b_estimator_angle_margin_rad: float = 0.0
    b_estimator_rate_margin_rad_s: float = 0.0
    b_estimator_use_torque_fallback: bool = False
    c_alpha_estimator_forgetting_factor: Optional[float] = None
    c_alpha_estimator_p0: Optional[float] = None
    c_alpha_estimator_min_value: Optional[float] = None
    c_alpha_estimator_max_value: Optional[float] = None
    c_alpha_estimator_start_time: float = 0.0
    c_alpha_estimator_min_samples: int = 0
    c_alpha_estimator_sample_decimation: int = 1
    c_alpha_estimator_alpha_min_abs: float = 0.0
    c_alpha_estimator_fz_min: float = 0.0
    c_alpha_estimator_fy_saturation_margin: float = 0.0
    use_lateral_force_estimator: bool = True
    use_slip_angle_estimator: bool = True
    steer_ff_max_accel: Optional[float] = None
    steer_ff_torque_limit: Optional[float] = None


@dataclass
class _VehicleConfig:
    mass: float
    izz: float
    wheelbase: float
    track: float
    corner_xy: Dict[str, Tuple[float, float]]
    base_j: float
    base_b: float
    base_c_alpha: float
    base_mu: float
    base_trail: float
    gear_ratio: float
    max_rate: float
    max_angle_left_pos: float
    max_angle_left_neg: float
    max_angle_right_pos: float
    max_angle_right_neg: float


class YawRateSteeringController:
    """Stateful yaw-rate steering controller.

    Recommended usage:
    - Create one controller instance per vehicle/control loop.
    - Use either `compute_torque_command()` or `compute_angle_command()` consistently on that
      instance.
    - If error-only diagnostics are needed, call `compute_error_debug()`
      or `get_last_error_debug()`.

    Avoid calling torque/angle methods back-to-back on the
    same controller instance for the same sample. Both methods advance internal
    controller states.

    Required `state` keys:
    - `yaw_rate`: measured yaw rate [rad/s]
    - `vx`: measured longitudinal speed [m/s]
    - `steering_angle`: per-wheel steering angle map [rad]

    Required `ref` keys:
    - `yaw_rate`: yaw-rate target [rad/s]

    Optional `state` keys:
    - `fy_tire`, `fx_tire`, `fz`: per-wheel measured maps
    - `ay`: lateral acceleration [m/s^2] (for force estimator)
    - `delta_dot`: per-wheel steering rate [rad/s]
    - `steering_torque_axis`: per-wheel axis torque [N*m] (for B estimator)
    - `alpha`: per-wheel slip angle [rad] (for C_alpha estimator)

    `ref` must contain only `yaw_rate`.
    """

    def __init__(
        self,
        options: Optional[YawRateSteeringControllerOptions] = None,
        *,
        vehicle_config_path: Optional[str] = None,
        gains_path: Optional[str] = None,
    ) -> None:
        self.options = options or YawRateSteeringControllerOptions()
        if self.options.dt <= 0.0:
            raise ValueError("options.dt must be positive")

        self.vehicle_cfg = self._load_vehicle_config(vehicle_config_path)
        self.gains_path = Path(gains_path) if gains_path else self._default_gains_path()

        self._vehicle = self._build_vehicle_proxy(self.vehicle_cfg)
        self._yaw_moment_ff = YawMomentFeedforwardController(self.options.dt)
        self._allocator = YawMomentAllocator()
        self._steering_ff = SteeringFeedforwardController()
        self._steer_torque_ff = SteeringMotorTorqueFF(
            self.options.dt,
            SteeringTorqueFFOptions(
                max_accel=self.options.steer_ff_max_accel,
                torque_limit=self.options.steer_ff_torque_limit,
            ),
        )

        self._yaw_pid = PIDController(self.options.dt, self._load_pid_gains("yaw_rate_pid"))
        fy_gains = self._load_pid_gains("fy_pid")
        steer_gains = self._load_pid_gains("steering_pid")
        self._fy_pids = {label: PIDController(self.options.dt, fy_gains) for label in WHEEL_LABELS}
        self._steer_pids = {label: PIDController(self.options.dt, steer_gains) for label in WHEEL_LABELS}

        self._lateral_force_estimator: Optional[LateralForceEstimator] = None
        self._slip_estimator: Optional[SlipAngleEstimator] = None

        if self.options.use_lateral_force_estimator:
            self._lateral_force_estimator = LateralForceEstimator(
                dt=self.options.dt,
                mass=self.vehicle_cfg.mass,
                options=LateralForceEstimatorOptions(),
            )

        if self.options.use_slip_angle_estimator:
            wheel_xy = self._wheel_xy_from_geometry(self.vehicle_cfg)
            self._slip_estimator = SlipAngleEstimator(
                dt=self.options.dt,
                wheel_xy=wheel_xy,
                options=SlipAngleEstimatorOptions(),
            )

        self._b_estimators: Dict[str, ScalarRLS] = {}
        self._c_estimators: Dict[str, ScalarRLS] = {}
        if self.options.enable_estimator:
            b_clamp = ScalarClamp(
                min_value=(
                    self.options.b_estimator_min_value
                    if self.options.b_estimator_min_value is not None
                    else 0.0
                ),
                max_value=self.options.b_estimator_max_value,
            )
            c_clamp = ScalarClamp(
                min_value=(
                    self.options.c_alpha_estimator_min_value
                    if self.options.c_alpha_estimator_min_value is not None
                    else 1.0
                ),
                max_value=self.options.c_alpha_estimator_max_value,
            )
            if self.options.enable_b_estimator:
                b_forgetting_factor = (
                    self.options.b_estimator_forgetting_factor
                    if self.options.b_estimator_forgetting_factor is not None
                    else self.options.estimator_forgetting_factor
                )
                b_p0 = (
                    self.options.b_estimator_p0
                    if self.options.b_estimator_p0 is not None
                    else self.options.estimator_p0
                )
                for label in WHEEL_LABELS:
                    self._b_estimators[label] = ScalarRLS(
                        init_value=self.vehicle_cfg.base_b,
                        forgetting_factor=b_forgetting_factor,
                        p0=b_p0,
                        clamp=b_clamp,
                    )
            if self.options.enable_c_alpha_estimator:
                c_alpha_forgetting_factor = (
                    self.options.c_alpha_estimator_forgetting_factor
                    if self.options.c_alpha_estimator_forgetting_factor is not None
                    else self.options.estimator_forgetting_factor
                )
                c_alpha_p0 = (
                    self.options.c_alpha_estimator_p0
                    if self.options.c_alpha_estimator_p0 is not None
                    else self.options.estimator_p0
                )
                for label in WHEEL_LABELS:
                    self._c_estimators[label] = ScalarRLS(
                        init_value=self.vehicle_cfg.base_c_alpha,
                        forgetting_factor=c_alpha_forgetting_factor,
                        p0=c_alpha_p0,
                        clamp=c_clamp,
                    )

        self._prev_delta_meas = {label: 0.0 for label in WHEEL_LABELS}
        self._prev_delta_dot_meas = {label: 0.0 for label in WHEEL_LABELS}
        self._sample_index = 0
        self._last_error_debug: Dict[str, Any] = {}

    def reset(self) -> None:
        """Reset all internal controller states."""
        self._yaw_moment_ff.reset()
        self._steer_torque_ff.reset()
        self._yaw_pid.reset()
        for pid in self._fy_pids.values():
            pid.reset()
        for pid in self._steer_pids.values():
            pid.reset()
        if self._lateral_force_estimator is not None:
            self._lateral_force_estimator.reset()
        if self._slip_estimator is not None:
            self._slip_estimator.reset()
        self._sample_index = 0
        self._last_error_debug = {}

    def __call__(self, state: Mapping[str, Any], ref: Mapping[str, Any]) -> Dict[str, float]:
        return self.compute_torque_command(state, ref)

    def compute_torque_command(self, state: Mapping[str, Any], ref: Mapping[str, Any]) -> Dict[str, float]:
        """Advance one control step and return steering motor torque command."""
        stage = self._compute_delta_stage(state, ref)
        torque_cmd, est_debug = self._compute_torque_stage(state, stage)
        self._last_error_debug = self._build_error_debug(stage, est_debug)
        self._sample_index += 1
        return torque_cmd

    def compute_angle_command(self, state: Mapping[str, Any], ref: Mapping[str, Any]) -> Dict[str, float]:
        """Advance one control step and return steering-angle command only.

        Notes:
        - This path updates only C_alpha estimator terms and intentionally skips
          B (viscous) estimation because steering torque terms are not produced.
        """
        stage = self._compute_delta_stage(state, ref)
        est_debug = self._update_estimators(
            state,
            stage["Fy_cmd"],
            t_ff_axis=None,
            update_b=False,
            update_c_alpha=True,
        )
        self._last_error_debug = self._build_error_debug(stage, est_debug)
        self._sample_index += 1
        return {label: float(stage["delta_cmd"].get(label, 0.0)) for label in WHEEL_LABELS}

    def compute_error_debug(
        self,
        state: Mapping[str, Any],
        ref: Mapping[str, Any],
    ) -> Dict[str, Any]:
        """Advance one control step and return error-only diagnostics."""
        self.compute_torque_command(state, ref)
        return self.get_last_error_debug()

    def get_last_error_debug(self) -> Dict[str, Any]:
        """Return latest error-only diagnostics from the most recent control step."""
        return deepcopy(self._last_error_debug)

    def _compute_delta_stage(
        self,
        state: Mapping[str, Any],
        ref: Mapping[str, Any],
    ) -> Dict[str, Any]:
        """Compute upstream control stage through steering-angle command."""
        self._ingest_state(state)

        yaw_rate_ref = self._extract_yaw_rate_ref(ref)
        yaw_accel_ref = None
        vx_cmd = float(state["vx"])
        vy_cmd = 0.0

        yaw_error = yaw_rate_ref - float(self._vehicle.state.yaw_rate)
        mz_fb = self._yaw_pid.update(yaw_error) if self.options.enable_yaw_feedback else 0.0
        mz_ff = self._yaw_moment_ff.compute_moment(self._vehicle, yaw_rate_ref, yaw_accel_ref)
        mz_cmd = float(mz_ff + mz_fb)

        fy_total_cmd = float(self.vehicle_cfg.mass * vx_cmd * yaw_rate_ref)
        fy_cmd = self._allocator.allocate(self._vehicle, mz_cmd, Fx_body=None, Fy_total_cmd=fy_total_cmd)

        c_alpha_map = self._active_c_alpha_map()
        delta_cmd, steer_debug = self._steering_ff.compute_delta_cmd_with_debug(
            self._vehicle,
            fy_cmd,
            vx_cmd=vx_cmd,
            yaw_rate_cmd=yaw_rate_ref,
            vy_cmd=vy_cmd,
            c_alpha_override=c_alpha_map,
        )

        fy_actual = self._resolve_fy_feedback_map(state, fy_cmd)
        if self.options.enable_fy_feedback:
            for label in WHEEL_LABELS:
                fy_err = float(fy_cmd.get(label, 0.0)) - float(fy_actual.get(label, 0.0))
                delta_cmd[label] = float(delta_cmd.get(label, 0.0)) + float(self._fy_pids[label].update(fy_err))

        return {
            "yaw_error": float(yaw_error),
            "Mz_ff": float(mz_ff),
            "Mz_fb": float(mz_fb),
            "Mz_cmd": float(mz_cmd),
            "Fy_total_cmd": float(fy_total_cmd),
            "Fy_cmd": {label: float(fy_cmd.get(label, 0.0)) for label in WHEEL_LABELS},
            "Fy_actual_for_fb": {label: float(fy_actual.get(label, 0.0)) for label in WHEEL_LABELS},
            "delta_cmd": {label: float(delta_cmd.get(label, 0.0)) for label in WHEEL_LABELS},
            "steer_debug": dict(steer_debug),
        }

    @staticmethod
    def _extract_yaw_rate_ref(ref: Mapping[str, Any]) -> float:
        if "yaw_rate" not in ref:
            raise KeyError("ref must include 'yaw_rate'")
        extra_keys = [str(k) for k in ref.keys() if str(k) != "yaw_rate"]
        if extra_keys:
            extra_keys_str = ", ".join(sorted(extra_keys))
            raise KeyError(
                "ref supports only 'yaw_rate'; remove unsupported key(s): "
                f"{extra_keys_str}"
            )
        return float(ref["yaw_rate"])

    def _compute_torque_stage(
        self,
        state: Mapping[str, Any],
        stage: Mapping[str, Any],
    ) -> Tuple[Dict[str, float], Dict[str, Any]]:
        fy_cmd = dict(stage["Fy_cmd"])
        delta_cmd = dict(stage["delta_cmd"])
        align_cmd = self._compute_align_cmd(fy_cmd)
        ff_params = self._build_ff_params(self._active_b_map())
        t_ff_motor, t_ff_axis = self._steer_torque_ff.compute_torque(
            WHEEL_LABELS,
            delta_cmd=delta_cmd,
            align_cmd=align_cmd,
            params_map=ff_params,
        )

        torque_cmd = dict(t_ff_motor)
        if self.options.enable_steer_feedback:
            for label in WHEEL_LABELS:
                delta_err = float(delta_cmd.get(label, 0.0)) - float(self._vehicle.corners[label].state.steering_angle)
                corr_axis = float(self._steer_pids[label].update(delta_err))
                gear = float(self._vehicle.corners[label].steering.params.gear_ratio)
                corr_motor = corr_axis if abs(gear) < 1e-6 else corr_axis / gear
                torque_cmd[label] = float(torque_cmd.get(label, 0.0)) + float(corr_motor)

        est_debug = self._update_estimators(
            state,
            fy_cmd,
            t_ff_axis=t_ff_axis,
            update_b=True,
            update_c_alpha=True,
        )
        return (
            {label: float(torque_cmd.get(label, 0.0)) for label in WHEEL_LABELS},
            est_debug,
        )

    def _build_error_debug(
        self,
        stage: Mapping[str, Any],
        est_debug: Mapping[str, Any],
    ) -> Dict[str, Any]:
        fy_cmd = self._normalize_map(stage.get("Fy_cmd", {}), 0.0)
        fy_actual = self._normalize_map(stage.get("Fy_actual_for_fb", {}), 0.0)
        delta_cmd = self._normalize_map(stage.get("delta_cmd", {}), 0.0)

        fy_error = {}
        delta_error = {}
        for label in WHEEL_LABELS:
            fy_error[label] = float(fy_cmd[label]) - float(fy_actual[label])
            delta_error[label] = float(delta_cmd[label]) - float(self._vehicle.corners[label].state.steering_angle)

        return {
            "yaw_rate_error": float(stage.get("yaw_error", 0.0)),
            "fy_error": fy_error,
            "steering_angle_error": delta_error,
            "estimator": dict(est_debug),
        }

    def _ingest_state(self, state: Mapping[str, Any]) -> None:
        if "yaw_rate" not in state:
            raise KeyError("state must include 'yaw_rate'")
        if "vx" not in state:
            raise KeyError("state must include 'vx'")
        if "steering_angle" not in state:
            raise KeyError("state must include 'steering_angle'")

        self._vehicle.state.yaw_rate = float(state["yaw_rate"])

        steering_map = self._normalize_map(state.get("steering_angle", {}), 0.0)
        fx_map = self._normalize_map(state.get("fx_tire", {}), 0.0)
        fy_map = self._normalize_map(state.get("fy_tire", {}), 0.0)

        if "fz" in state:
            fz_map = self._normalize_map(state.get("fz", {}), self._static_fz())
        else:
            fz_map = {label: self._static_fz() for label in WHEEL_LABELS}

        for label in WHEEL_LABELS:
            corner = self._vehicle.corners[label]
            corner.state.steering_angle = float(steering_map[label])
            corner.state.F_x_tire = float(fx_map[label])
            corner.state.F_y_tire = float(fy_map[label])
            corner.state.F_z = float(fz_map[label])

    def _resolve_fy_feedback_map(
        self,
        state: Mapping[str, Any],
        fy_cmd: Mapping[str, float],
    ) -> Dict[str, float]:
        source = str(self.options.fy_feedback_source).strip().lower()
        if source == "estimate":
            if self._lateral_force_estimator is None:
                return self._normalize_map(state.get("fy_tire", {}), 0.0)
            ay = float(state.get("ay", 0.0))
            fy_total = float(self._lateral_force_estimator.update(ay))
            return self._distribute_total(fy_total, fy_cmd)
        return self._normalize_map(state.get("fy_tire", {}), 0.0)

    def _update_estimators(
        self,
        state: Mapping[str, Any],
        fy_cmd: Mapping[str, float],
        t_ff_axis: Optional[Mapping[str, float]] = None,
        *,
        update_b: bool = True,
        update_c_alpha: bool = True,
    ) -> Dict[str, Any]:
        if not self.options.enable_estimator:
            return {
                "B_hat": self._active_b_map(),
                "C_alpha_hat": self._active_c_alpha_map(),
            }

        delta_map = self._normalize_map(state.get("steering_angle", {}), 0.0)
        delta_dot_map = (
            self._normalize_map(state.get("delta_dot", {}), 0.0)
            if "delta_dot" in state
            else self._estimate_delta_dot(delta_map)
        )
        delta_ddot_map = self._estimate_delta_ddot(delta_dot_map)
        current_time = float(self._sample_index) * float(self.options.dt)
        step_index = int(self._sample_index)

        b_updated = False
        c_alpha_updated = False

        if update_b and self.options.enable_b_estimator and self._b_estimators:
            torque_axis_map: Optional[Dict[str, float]]
            if "steering_torque_axis" in state:
                torque_axis_map = self._normalize_map(state.get("steering_torque_axis", {}), 0.0)
            elif self.options.b_estimator_use_torque_fallback and t_ff_axis is not None:
                torque_axis_map = {label: float(t_ff_axis.get(label, 0.0)) for label in WHEEL_LABELS}
            else:
                torque_axis_map = None

            if torque_axis_map is not None:
                align_cmd = self._compute_align_cmd(fy_cmd)
                for label in WHEEL_LABELS:
                    y_b = (
                        float(torque_axis_map[label])
                        - float(align_cmd[label])
                        - float(self.vehicle_cfg.base_j) * float(delta_ddot_map[label])
                    )
                    if self._should_update_b_estimator(
                        label=label,
                        current_time=current_time,
                        step_index=step_index,
                        delta=float(delta_map[label]),
                        delta_dot=float(delta_dot_map[label]),
                        delta_ddot=float(delta_ddot_map[label]),
                    ):
                        self._b_estimators[label].update(y_b, float(delta_dot_map[label]))
                        b_updated = True

        if update_c_alpha and self.options.enable_c_alpha_estimator and self._c_estimators:
            if self._slip_estimator is not None:
                alpha_map, _ = self._slip_estimator.update(
                    vx=float(state.get("vx", 0.0)),
                    yaw_rate=float(state.get("yaw_rate", 0.0)),
                    ay_meas=float(state.get("ay", 0.0)),
                    delta_map=delta_map,
                )
            else:
                alpha_map = self._normalize_map(state.get("alpha", {}), 0.0)

            fy_for_c = self._normalize_map(state.get("fy_tire", {}), 0.0)
            for label in WHEEL_LABELS:
                alpha = float(alpha_map[label])
                fy_val = float(fy_for_c[label])
                fz_val = float(self._vehicle.corners[label].state.F_z)
                if self._should_update_c_alpha_estimator(
                    label=label,
                    current_time=current_time,
                    step_index=step_index,
                    alpha=alpha,
                    fy=fy_val,
                    fz=fz_val,
                ):
                    self._c_estimators[label].update(-fy_val, alpha)
                    c_alpha_updated = True

        return {
            "B_hat": self._active_b_map(),
            "C_alpha_hat": self._active_c_alpha_map(),
            "delta_dot": delta_dot_map,
            "delta_ddot": delta_ddot_map,
            "B_updated": bool(b_updated),
            "C_alpha_updated": bool(c_alpha_updated),
            "B_samples": {
                label: int(self._b_estimators[label].sample_count) for label in self._b_estimators
            },
            "C_alpha_samples": {
                label: int(self._c_estimators[label].sample_count) for label in self._c_estimators
            },
        }

    def _estimate_delta_dot(self, delta_map: Mapping[str, float]) -> Dict[str, float]:
        dt = float(self.options.dt)
        out = {}
        for label in WHEEL_LABELS:
            out[label] = (float(delta_map[label]) - float(self._prev_delta_meas[label])) / dt
            self._prev_delta_meas[label] = float(delta_map[label])
        return out

    def _estimate_delta_ddot(self, delta_dot_map: Mapping[str, float]) -> Dict[str, float]:
        dt = float(self.options.dt)
        out = {}
        for label in WHEEL_LABELS:
            out[label] = (float(delta_dot_map[label]) - float(self._prev_delta_dot_meas[label])) / dt
            self._prev_delta_dot_meas[label] = float(delta_dot_map[label])
        return out

    def _should_update_b_estimator(
        self,
        *,
        label: str,
        current_time: float,
        step_index: int,
        delta: float,
        delta_dot: float,
        delta_ddot: float,
    ) -> bool:
        if current_time < float(self.options.b_estimator_start_time):
            return False
        if step_index % max(1, int(self.options.b_estimator_sample_decimation)) != 0:
            return False
        if abs(delta_dot) < float(self.options.b_estimator_dot_min_abs):
            return False
        if not np.isfinite(delta) or not np.isfinite(delta_dot) or not np.isfinite(delta_ddot):
            return False

        params = self._vehicle.corners[label].steering.params
        lower = min(float(params.max_angle_neg), float(params.max_angle_pos))
        upper = max(float(params.max_angle_neg), float(params.max_angle_pos))
        angle_margin = float(self.options.b_estimator_angle_margin_rad)
        rate_margin = float(self.options.b_estimator_rate_margin_rad_s)
        if delta <= lower + angle_margin or delta >= upper - angle_margin:
            return False
        if abs(delta_dot) >= float(params.max_rate) - rate_margin:
            return False
        return True

    def _should_update_c_alpha_estimator(
        self,
        *,
        label: str,
        current_time: float,
        step_index: int,
        alpha: float,
        fy: float,
        fz: float,
    ) -> bool:
        if current_time < float(self.options.c_alpha_estimator_start_time):
            return False
        if step_index % max(1, int(self.options.c_alpha_estimator_sample_decimation)) != 0:
            return False
        if abs(alpha) < float(self.options.c_alpha_estimator_alpha_min_abs):
            return False
        if abs(fz) < float(self.options.c_alpha_estimator_fz_min):
            return False
        if not np.isfinite(alpha) or not np.isfinite(fy) or not np.isfinite(fz):
            return False

        mu = float(self._vehicle.corners[label].lateral_tire.params.mu)
        fy_max = abs(mu * fz)
        if fy_max <= 0.0:
            return False
        sat_margin = float(self.options.c_alpha_estimator_fy_saturation_margin)
        if abs(fy) >= fy_max * (1.0 - sat_margin):
            return False
        return True

    def _build_ff_params(self, b_map: Mapping[str, float]) -> Dict[str, SteeringFFParams]:
        out: Dict[str, SteeringFFParams] = {}
        for label in WHEEL_LABELS:
            steer_params = self._vehicle.corners[label].steering.params
            out[label] = SteeringFFParams(
                J_cq=float(self.vehicle_cfg.base_j),
                B_cq=float(b_map[label]),
                gear_ratio=float(steer_params.gear_ratio),
                max_rate=float(steer_params.max_rate),
                max_angle_pos=float(steer_params.max_angle_pos),
                max_angle_neg=float(steer_params.max_angle_neg),
            )
        return out

    def _compute_align_cmd(self, fy_cmd: Mapping[str, float]) -> Dict[str, float]:
        align = {}
        for label in WHEEL_LABELS:
            corner = self._vehicle.corners[label]
            fy = float(fy_cmd.get(label, 0.0))
            fz = float(corner.state.F_z)
            if fz > 0.0:
                mu = float(corner.lateral_tire.params.mu)
                fy = float(np.clip(fy, -mu * abs(fz), mu * abs(fz)))
            trail = float(corner.lateral_tire.params.trail)
            align[label] = float(trail * fy)
        return align

    def _active_b_map(self) -> Dict[str, float]:
        if (
            (not self.options.enable_estimator)
            or (not self.options.enable_b_estimator)
            or (not self._b_estimators)
        ):
            return {label: float(self.vehicle_cfg.base_b) for label in WHEEL_LABELS}
        return {
            label: (
                float(self._b_estimators[label].get_value())
                if self._b_estimators[label].sample_count >= int(self.options.b_estimator_min_samples)
                else float(self.vehicle_cfg.base_b)
            )
            for label in WHEEL_LABELS
        }

    def _active_c_alpha_map(self) -> Dict[str, float]:
        if (
            (not self.options.enable_estimator)
            or (not self.options.enable_c_alpha_estimator)
            or (not self._c_estimators)
        ):
            return {label: float(self.vehicle_cfg.base_c_alpha) for label in WHEEL_LABELS}
        return {
            label: (
                float(self._c_estimators[label].get_value())
                if self._c_estimators[label].sample_count >= int(self.options.c_alpha_estimator_min_samples)
                else float(self.vehicle_cfg.base_c_alpha)
            )
            for label in WHEEL_LABELS
        }

    def _normalize_map(self, value: Any, default: float) -> Dict[str, float]:
        if isinstance(value, Mapping):
            return {label: float(value.get(label, default)) for label in WHEEL_LABELS}
        return {label: float(default) for label in WHEEL_LABELS}

    def _distribute_total(
        self,
        total: float,
        weight_map: Mapping[str, float],
    ) -> Dict[str, float]:
        denom = float(sum(weight_map.values()))
        if abs(denom) < 1e-9:
            share = float(total) / float(len(WHEEL_LABELS))
            return {label: share for label in WHEEL_LABELS}
        return {
            label: float(total) * float(weight_map.get(label, 0.0)) / denom
            for label in WHEEL_LABELS
        }

    def _static_fz(self) -> float:
        gravity = float(load_param("physics", None).get("g", 9.81))
        return float(self.vehicle_cfg.mass * gravity / float(len(WHEEL_LABELS)))

    @staticmethod
    def _default_gains_path() -> Path:
        return Path(__file__).with_name("param").joinpath("controller_gains.yaml")

    def _load_pid_gains(self, section: str) -> PIDGains:
        cfg = load_param(section, str(self.gains_path))
        return PIDGains(
            kp=float(cfg.get("kp", 0.0)),
            ki=float(cfg.get("ki", 0.0)),
            kd=float(cfg.get("kd", 0.0)),
        )

    def _load_vehicle_config(self, config_path: Optional[str]) -> _VehicleConfig:
        vehicle_body = load_param("vehicle_body", config_path)
        vehicle_spec = load_param("vehicle_spec", config_path)
        geometry = vehicle_spec.get("geometry", {})
        corner_offsets = geometry.get("corner_offsets", {}) if isinstance(geometry, dict) else {}
        tire = load_param("tire", config_path)
        lateral = tire.get("lateral", {}) if isinstance(tire, dict) else {}
        steering = load_param("steering", config_path)

        left = steering.get("left", {}) if isinstance(steering, dict) else {}
        right = steering.get("right", {}) if isinstance(steering, dict) else {}
        inertia = vehicle_body.get("inertia", {}) if isinstance(vehicle_body, dict) else {}

        wheelbase = float(geometry.get("L_wheelbase", 2.8))
        track = float(geometry.get("L_track", 1.6))
        corner_xy: Dict[str, Tuple[float, float]] = {}
        for label in WHEEL_LABELS:
            offset = corner_offsets.get(label, {}) if isinstance(corner_offsets, Mapping) else {}
            if isinstance(offset, Mapping) and "x" in offset and "y" in offset:
                x_i = float(offset["x"])
                y_i = float(offset["y"])
            else:
                signs = WHEEL_SIGNS[label]
                x_i = (wheelbase / 2.0) * signs["pitch"]
                y_i = (track / 2.0) * signs["roll"]
            corner_xy[label] = (float(x_i), float(y_i))

        return _VehicleConfig(
            mass=float(vehicle_body.get("m", 1500.0)),
            izz=float(inertia.get("Izz", 2800.0)),
            wheelbase=wheelbase,
            track=track,
            corner_xy=corner_xy,
            base_j=float(steering.get("J_cq", 0.05)),
            base_b=float(steering.get("B_cq", 0.5)),
            base_c_alpha=float(lateral.get("C_alpha", 80000.0)),
            base_mu=float(tire.get("mu", 1.0)),
            base_trail=float(lateral.get("trail", 0.05)),
            gear_ratio=float(steering.get("gear_ratio", 118.0)),
            max_rate=float(steering.get("max_rate", np.deg2rad(360.0))),
            max_angle_left_pos=float(left.get("max_angle_pos", 0.5)),
            max_angle_left_neg=float(left.get("max_angle_neg", -0.5)),
            max_angle_right_pos=float(right.get("max_angle_pos", 0.5)),
            max_angle_right_neg=float(right.get("max_angle_neg", -0.5)),
        )

    @staticmethod
    def _wheel_xy_from_geometry(cfg: _VehicleConfig) -> Dict[str, Tuple[float, float]]:
        return {label: (float(x_i), float(y_i)) for label, (x_i, y_i) in cfg.corner_xy.items()}

    @staticmethod
    def _build_vehicle_proxy(cfg: _VehicleConfig):
        params = SimpleNamespace(
            m=float(cfg.mass),
            Izz=float(cfg.izz),
            L_wheelbase=float(cfg.wheelbase),
            L_track=float(cfg.track),
            corner_xy={label: (float(x_i), float(y_i)) for label, (x_i, y_i) in cfg.corner_xy.items()},
        )
        state = SimpleNamespace(yaw_rate=0.0)
        corners: Dict[str, Any] = {}
        for label in WHEEL_LABELS:
            is_left = label in {"FL", "RL"}
            max_pos = cfg.max_angle_left_pos if is_left else cfg.max_angle_right_pos
            max_neg = cfg.max_angle_left_neg if is_left else cfg.max_angle_right_neg
            steering_params = SimpleNamespace(
                gear_ratio=float(cfg.gear_ratio),
                max_rate=float(cfg.max_rate),
                max_angle_pos=float(max_pos),
                max_angle_neg=float(max_neg),
            )
            lateral_params = SimpleNamespace(
                C_alpha=float(cfg.base_c_alpha),
                mu=float(cfg.base_mu),
                trail=float(cfg.base_trail),
            )
            corner_state = SimpleNamespace(
                steering_angle=0.0,
                F_x_tire=0.0,
                F_y_tire=0.0,
                F_z=0.0,
            )
            corners[label] = SimpleNamespace(
                state=corner_state,
                steering=SimpleNamespace(params=steering_params),
                lateral_tire=SimpleNamespace(params=lateral_params),
            )

        return SimpleNamespace(
            params=params,
            state=state,
            wheel_labels=list(WHEEL_LABELS),
            corner_signs=WHEEL_SIGNS,
            corner_xy={label: (float(x_i), float(y_i)) for label, (x_i, y_i) in cfg.corner_xy.items()},
            corners=corners,
        )


# Shared default controller used by one-line convenience functions below.
_DEFAULT_CONTROLLER: Optional[YawRateSteeringController] = None


def _resolve_controller(
    controller: Optional[YawRateSteeringController],
    reset: bool,
) -> YawRateSteeringController:
    global _DEFAULT_CONTROLLER
    if controller is not None:
        if reset:
            controller.reset()
        return controller

    if _DEFAULT_CONTROLLER is None or reset:
        _DEFAULT_CONTROLLER = YawRateSteeringController()
    return _DEFAULT_CONTROLLER


def compute_steering_torque(
    state: Mapping[str, Any],
    ref: Mapping[str, Any],
    *,
    controller: Optional[YawRateSteeringController] = None,
    reset: bool = False,
) -> Dict[str, float]:
    """One-line torque API using a shared default controller instance.

    This helper is fine for smoke tests or notebooks. For production use,
    prefer creating an explicit `YawRateSteeringController` instance.
    """
    instance = _resolve_controller(controller, reset)
    return instance.compute_torque_command(state, ref)


def compute_steering_angle(
    state: Mapping[str, Any],
    ref: Mapping[str, Any],
    *,
    controller: Optional[YawRateSteeringController] = None,
    reset: bool = False,
) -> Dict[str, float]:
    """One-line angle API using a shared default controller instance."""
    instance = _resolve_controller(controller, reset)
    return instance.compute_angle_command(state, ref)


# Backward-compatible names kept for existing integrations.
BlockControllerOptions = YawRateSteeringControllerOptions
YawRateSteerTorqueBlockController = YawRateSteeringController
