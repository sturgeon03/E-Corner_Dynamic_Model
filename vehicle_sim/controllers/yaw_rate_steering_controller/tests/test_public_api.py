"""Simple smoke tests for yaw-rate steering controller.

Run:
    python -m vehicle_sim.controllers.yaw_rate_steering_controller.tests.test_public_api
"""

from __future__ import annotations

import math
from pathlib import Path

import vehicle_sim.controllers.yaw_rate_steering_controller as api
from vehicle_sim.controllers.yaw_rate_steering_controller import (
    ControllerRuntimeConfig,
    YawRateSteeringControllerOptions,
    YawRateSteeringController,
    compute_steering_angle,
    compute_steering_torque,
    control_angle,
    control,
    control_torque,
    load_controller_runtime_config,
)


WHEELS = ("FL", "FR", "RR", "RL")


def _state() -> dict:
    return {
        "yaw_rate": 0.08,
        "vx": 10.0,
        "steering_angle": {label: 0.0 for label in WHEELS},
        "fy_tire": {label: 0.0 for label in WHEELS},
        "fx_tire": {label: 0.0 for label in WHEELS},
        "ay": 0.1,
    }


def _ref() -> dict:
    return {
        "yaw_rate": 0.25,
        "yaw_accel": 0.0,
        "vx": 10.0,
    }


def test_api_exports_snapshot() -> None:
    expected = [
        "YawRateSteeringControllerOptions",
        "YawRateSteeringController",
        "compute_steering_torque",
        "compute_steering_angle",
        "BlockControllerOptions",
        "ControllerRuntimeConfig",
        "YawRateSteerTorqueBlockController",
        "load_controller_runtime_config",
        "control",
        "control_torque",
        "control_angle",
    ]
    assert list(api.__all__) == expected


def test_default_yaml_paths_and_runtime_config_load() -> None:
    controller = YawRateSteeringController(YawRateSteeringControllerOptions(dt=0.01))
    package_root = Path(__file__).resolve().parents[1]
    expected_gains = package_root / "config" / "controller_gains.yaml"
    assert controller.gains_path.resolve() == expected_gains.resolve()

    options_yaml = package_root / "config" / "controller_options.example.yaml"
    runtime_cfg = load_controller_runtime_config(options_yaml)
    assert isinstance(runtime_cfg, ControllerRuntimeConfig)
    assert runtime_cfg.mode == "ff_fb_ls"


def test_callable_api() -> None:
    controller = YawRateSteeringController(YawRateSteeringControllerOptions(dt=0.01))
    out = controller(_state(), _ref())
    assert isinstance(out, dict)
    assert set(out.keys()) == set(WHEELS)
    assert all(math.isfinite(float(v)) for v in out.values())


def test_debug_api() -> None:
    controller = YawRateSteeringController(YawRateSteeringControllerOptions(dt=0.01))
    out, debug = controller.compute_torque_with_debug(_state(), _ref())
    assert set(out.keys()) == set(WHEELS)
    assert "Mz_cmd" in debug
    assert "Fy_cmd" in debug
    assert "delta_cmd" in debug


def test_angle_only_api() -> None:
    controller = YawRateSteeringController(YawRateSteeringControllerOptions(dt=0.01))
    out = controller.compute_angle_command(_state(), _ref())
    assert set(out.keys()) == set(WHEELS)
    assert all(math.isfinite(float(v)) for v in out.values())


def test_angle_does_not_use_torque_stage() -> None:
    controller = YawRateSteeringController(YawRateSteeringControllerOptions(dt=0.01))
    controller._steer_torque_ff.compute_torque = lambda *args, **kwargs: (_ for _ in ()).throw(
        AssertionError("torque stage must not be called in control_angle")
    )
    delta_cmd = controller.compute_angle_command(_state(), _ref())
    assert set(delta_cmd.keys()) == set(WHEELS)
    assert all(math.isfinite(float(v)) for v in delta_cmd.values())


def test_angle_updates_c_alpha_but_skips_b_estimator() -> None:
    controller = YawRateSteeringController(
        YawRateSteeringControllerOptions(
            dt=0.01,
            enable_estimator=True,
            enable_b_estimator=True,
            enable_c_alpha_estimator=True,
            use_slip_angle_estimator=False,
        )
    )
    initial_b = controller._active_b_map()
    initial_c = controller._active_c_alpha_map()

    state = _state()
    state["alpha"] = {label: 0.05 for label in WHEELS}
    state["fy_tire"] = {label: 1200.0 for label in WHEELS}
    state["steering_torque_axis"] = {label: 30.0 for label in WHEELS}
    state["delta_dot"] = {label: 1.0 for label in WHEELS}

    controller.compute_angle_command(state, _ref())

    b_after = controller._active_b_map()
    c_after = controller._active_c_alpha_map()
    assert b_after == initial_b
    assert any(abs(float(c_after[label]) - float(initial_c[label])) > 1e-12 for label in WHEELS)


def test_one_line_function() -> None:
    out = control(_state(), _ref(), reset=True)
    assert set(out.keys()) == set(WHEELS)
    out_torque = control_torque(_state(), _ref(), reset=True)
    assert set(out_torque.keys()) == set(WHEELS)
    out_angle = control_angle(_state(), _ref(), reset=True)
    assert set(out_angle.keys()) == set(WHEELS)
    out_torque_v2 = compute_steering_torque(_state(), _ref(), reset=True)
    assert set(out_torque_v2.keys()) == set(WHEELS)
    out_angle_v2 = compute_steering_angle(_state(), _ref(), reset=True)
    assert set(out_angle_v2.keys()) == set(WHEELS)


def main() -> None:
    test_api_exports_snapshot()
    test_default_yaml_paths_and_runtime_config_load()
    test_callable_api()
    test_debug_api()
    test_angle_only_api()
    test_angle_does_not_use_torque_stage()
    test_angle_updates_c_alpha_but_skips_b_estimator()
    test_one_line_function()
    print("All controller tests passed.")


if __name__ == "__main__":
    main()
