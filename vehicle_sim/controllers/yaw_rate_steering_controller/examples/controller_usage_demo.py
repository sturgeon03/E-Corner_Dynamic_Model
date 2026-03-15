"""Minimal example for production-style yaw-rate steering usage.

Run:
    python vehicle_sim/controllers/yaw_rate_steering_controller/examples/controller_usage_demo.py
    python -m vehicle_sim.controllers.yaw_rate_steering_controller.examples.controller_usage_demo
"""

from __future__ import annotations

import sys
from pathlib import Path

# Support direct script execution without requiring editable install.
if __package__ in (None, ""):
    repo_root = Path(__file__).resolve().parents[4]
    if str(repo_root) not in sys.path:
        sys.path.insert(0, str(repo_root))

from vehicle_sim.controllers.yaw_rate_steering_controller import (
    YawRateSteeringControllerOptions,
    YawRateSteeringController,
    compute_steering_angle,
    compute_steering_torque,
)


def main() -> None:
    options = YawRateSteeringControllerOptions(
        dt=0.01,
        enable_yaw_feedback=True,
        enable_fy_feedback=False,
        enable_steer_feedback=True,
        enable_estimator=False,
    )
    torque_controller = YawRateSteeringController(options)
    angle_controller = YawRateSteeringController(options)
    debug_controller = YawRateSteeringController(options)

    vehicle_state = {
        "yaw_rate": 0.05,
        "vx": 8.0,
        "steering_angle": {"FL": 0.0, "FR": 0.0, "RR": 0.0, "RL": 0.0},
        "fy_tire": {"FL": 0.0, "FR": 0.0, "RR": 0.0, "RL": 0.0},
        "fx_tire": {"FL": 0.0, "FR": 0.0, "RR": 0.0, "RL": 0.0},
        "ay": 0.2,
    }

    reference = {
        "yaw_rate": 0.2,
        "yaw_accel": 0.0,
        "vx": 8.0,
        "vy": 0.0,
    }

    steering_motor_torque_cmd = torque_controller.compute_torque_command(vehicle_state, reference)
    print("Recommended instance API: compute_torque_command(...) =>")
    print(steering_motor_torque_cmd)

    steering_angle_cmd = angle_controller.compute_angle_command(vehicle_state, reference)
    print("\nRecommended instance API: compute_angle_command(...) =>")
    print(steering_angle_cmd)

    shared_default_torque_cmd = compute_steering_torque(vehicle_state, reference, reset=True)
    print("\nQuick-check helper: compute_steering_torque(...) =>")
    print(shared_default_torque_cmd)
    shared_default_angle_cmd = compute_steering_angle(vehicle_state, reference, reset=True)
    print("\nQuick-check helper: compute_steering_angle(...) =>")
    print(shared_default_angle_cmd)

    steering_motor_torque_cmd_dbg, debug = debug_controller.compute_torque_with_debug(
        vehicle_state,
        reference,
    )
    print("\nDebug API: compute_torque_with_debug(...) =>")
    print("Mz_cmd =", debug["Mz_cmd"])
    print("Fy_cmd =", debug["Fy_cmd"])
    print("delta_cmd =", debug["delta_cmd"])
    print("steering_motor_torque_cmd =", steering_motor_torque_cmd_dbg)


if __name__ == "__main__":
    main()
