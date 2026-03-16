from __future__ import annotations

from copy import deepcopy

import numpy as np

from vehicle_sim.models.vehicle_body.vehicle_body import VehicleBody
from vehicle_sim.controllers.yaw_rate_steering_controller import YawRateSteeringController
def set_initial_speed(vehicle: VehicleBody, speed_mps: float) -> None:
    """차량 초기 속도 설정 — 차체 및 전 코너 휠 속도를 초기화한다."""
    vehicle.state.velocity_x = float(speed_mps)
    for label in vehicle.wheel_labels:
        corner = vehicle.corners[label]
        wheel_r = float(corner.drive.params.R_wheel)
        if wheel_r > 0.0:
            wheel_speed = float(speed_mps / wheel_r)
            corner.drive.state.wheel_speed = wheel_speed
            corner.state.omega_wheel = wheel_speed


def half_cosine_ramp(t: float, ramp_time: float) -> float:
    """0→1 반코사인 램프 윈도우 함수"""
    if ramp_time <= 0.0:
        return 1.0
    if t <= 0.0:
        return 0.0
    if t >= ramp_time:
        return 1.0
    return float(0.5 * (1.0 - np.cos(np.pi * t / ramp_time)))


def yaw_rate_reference_wave(
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
    win = half_cosine_ramp(t_rel, ramp_time)
    return float(amp * win * np.sin(2.0 * np.pi * float(freq_hz) * t_rel))


def build_controller_state(vehicle: VehicleBody) -> dict:
    """차량 현재 상태를 제어기 입력 형식으로 변환한다."""
    wheels = tuple(vehicle.wheel_labels)
    fz_map = {w: float(vehicle.corners[w].state.F_z) for w in wheels}
    if all(abs(v) < 1e-9 for v in fz_map.values()):
        static_fz = float(vehicle.params.m_total * vehicle.params.g / 4.0)
        fz_map = {w: static_fz for w in wheels}
    return {
        'yaw_rate':            float(vehicle.state.yaw_rate),
        'vx':                  float(vehicle.state.velocity_x),
        'steering_angle':      {w: float(vehicle.corners[w].state.steering_angle) for w in wheels},
        'fy_tire':             {w: float(vehicle.corners[w].state.F_y_tire) for w in wheels},
        'fx_tire':             {w: float(vehicle.corners[w].state.F_x_tire) for w in wheels},
        'fz':                  fz_map,
        'ay':                  float(getattr(vehicle.state, 'ay_prev', 0.0)),
        'delta_dot':           {w: float(vehicle.corners[w].steering.state.steering_rate) for w in wheels},
        'steering_torque_axis':{w: float(vehicle.corners[w].steering.state.steering_torque) for w in wheels},
        'alpha':               {w: float(vehicle.corners[w].lateral_tire.state.slip_angle) for w in wheels},
    }


def build_corner_inputs(vehicle: VehicleBody, steer_torque_cmd: dict, drive_torque_cmd: float) -> dict:
    """코너별 제어 입력 딕셔너리를 생성한다."""
    return {
        label: {
            'T_steer':    float(steer_torque_cmd.get(label, 0.0)),
            'T_brk':      0.0,
            'T_Drv':      float(drive_torque_cmd),
            'T_susp':     0.0,
            'z_road':     0.0,
            'z_road_dot': 0.0,
        }
        for label in vehicle.wheel_labels
    }


def measure_vehicle_outputs(vehicle: VehicleBody) -> dict:
    """차량 실제 출력(Fy, Mz)을 계산하여 반환한다."""
    corner_outputs = {
        label: (
            float(vehicle.corners[label].state.F_s),
            float(vehicle.corners[label].state.F_x_tire),
            float(vehicle.corners[label].state.F_y_tire),
        )
        for label in vehicle.wheel_labels
    }
    forces, moments = vehicle.assemble_forces_moments(corner_outputs)
    return {
        'Fy_total_actual': float(forces[1]),
        'Mz_actual':       float(moments[2]),
        'Fy_actual': {
            label: float(vehicle.corners[label].state.F_y_tire)
            for label in vehicle.wheel_labels
        },
    }


def build_yaw_sim(runtime_cfg, vehicle_config_path: str, scenario) -> tuple:
    """제어기·차량 모델을 초기화하고 반환한다.

    Returns:
        (controller, body, wheels)
    """
    controller = YawRateSteeringController(
        options=deepcopy(runtime_cfg.options),
        vehicle_config_path=vehicle_config_path,
        gains_path=runtime_cfg.gains_path,
    )
    body = VehicleBody(config_path=vehicle_config_path)
    set_initial_speed(body, float(scenario['target_mps']))
    wheels = tuple(body.wheel_labels)
    return controller, body, wheels


def init_sim_log(n: int, t, wheels) -> dict:
    """시뮬레이션 로그 딕셔너리를 초기화하여 반환한다."""
    return {
        'time':            t,
        'yaw_ref':         np.zeros(n),
        'yaw_meas':        np.zeros(n),
        'mz_actual':       np.zeros(n),
        'fy_total_actual': np.zeros(n),
        'delta_meas':      {w: np.zeros(n) for w in wheels},
        'steer_trq':       {w: np.zeros(n) for w in wheels},
    }


def log_step(
    log: dict,
    i: int,
    body: VehicleBody,
    yaw_ref: float,
    out: dict,
    steer_trq: dict,
    wheels,
) -> None:
    """한 스텝 시뮬레이션 결과를 로그에 기록한다."""
    log['yaw_ref'][i]         = yaw_ref
    log['yaw_meas'][i]        = float(body.state.yaw_rate)
    log['mz_actual'][i]       = float(out['Mz_actual'])
    log['fy_total_actual'][i] = float(out['Fy_total_actual'])
    for w in wheels:
        log['delta_meas'][w][i] = float(body.corners[w].state.steering_angle)
        log['steer_trq'][w][i]  = float(steer_trq[w])
