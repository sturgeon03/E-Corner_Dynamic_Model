"""
Logged body-input scenario and steer-reference tracking controller.
"""

from pathlib import Path
from typing import Dict, Optional, Tuple

import numpy as np

CORNERS = ["FL", "FR", "RL", "RR"]
DEFAULT_LOG_PATH = Path(__file__).resolve().parent / "Log_Data" / "CM_Body_sinesweep.csv"

_DRIVE_TORQUE_COLUMNS = {
    "FL": "cm_DriveTrqFL",
    "FR": "cm_DriveTrqFR",
    "RL": "cm_DriveTrqRL",
    "RR": "cm_DriveTrqRR",
}

_STEER_REFERENCE_COLUMNS = {
    "FL": "cm_SteerAngleFL",
    "FR": "cm_SteerAngleFR",
    "RL": "cm_SteerAngleRL",
    "RR": "cm_SteerAngleRR",
}

_WHEEL_SPEED_COLUMNS = {
    "FL": "cm_WheelSpd_FL",
    "FR": "cm_WheelSpd_FR",
    "RL": "cm_WheelSpd_RL",
    "RR": "cm_WheelSpd_RR",
}

_UNSPRUNG_Z_COLUMNS = {
    "FL": "cm_UnsprungZFL",
    "FR": "cm_UnsprungZFR",
    "RL": "cm_UnsprungZRL",
    "RR": "cm_UnsprungZRR",
}

_SPRUNG_Z_COLUMNS = {
    "FL": "cm_SprungZFL",
    "FR": "cm_SprungZFR",
    "RL": "cm_SprungZRL",
    "RR": "cm_SprungZRR",
}

_SPRUNG_Z_DOT_COLUMNS = {
    "FL": "cm_SprungZDdotFL",
    "FR": "cm_SprungZDdotFR",
    "RL": "cm_SprungZDdotRL",
    "RR": "cm_SprungZDdotRR",
}


def _load_logged_inputs(
    csv_path: Optional[str] = None,
) -> Tuple[np.ndarray, Dict[str, np.ndarray], Dict[str, np.ndarray]]:
    """Load logged drive torque and steering references from CSV."""
    path = Path(csv_path) if csv_path is not None else DEFAULT_LOG_PATH
    if not path.exists():
        raise FileNotFoundError(f"Logged scenario file not found: {path}")

    raw = np.genfromtxt(
        path,
        delimiter=",",
        names=True,
        dtype=float,
        encoding="utf-8",
    )
    if raw.size == 0:
        raise ValueError(f"Logged scenario file is empty: {path}")
    if raw.dtype.names is None or "Time" not in raw.dtype.names:
        raise KeyError(f"'Time' column is missing in logged scenario file: {path}")

    time = np.atleast_1d(np.asarray(raw["Time"], dtype=float))
    if time.size < 2:
        raise ValueError(f"Logged scenario file must have at least 2 rows: {path}")

    drive_trq = {
        corner: np.atleast_1d(np.asarray(raw[column], dtype=float))
        for corner, column in _DRIVE_TORQUE_COLUMNS.items()
    }
    steer_ref = {
        corner: np.atleast_1d(np.asarray(raw[column], dtype=float))
        for corner, column in _STEER_REFERENCE_COLUMNS.items()
    }

    return time, drive_trq, steer_ref


def _load_initial_rows(csv_path: Optional[str] = None) -> np.ndarray:
    """Load the first one or two rows from the logged scenario file."""
    path = Path(csv_path) if csv_path is not None else DEFAULT_LOG_PATH
    rows = np.genfromtxt(
        path,
        delimiter=",",
        names=True,
        dtype=float,
        encoding="utf-8",
        max_rows=2,
    )
    rows = np.atleast_1d(rows)
    if rows.size == 0:
        raise ValueError(f"Logged scenario file is empty: {path}")
    return rows


def _first_step_rate(rows: np.ndarray, column: str) -> float:
    """Compute a first-step finite-difference rate from the log."""
    if rows.size < 2:
        return 0.0

    dt = float(rows[1]["Time"] - rows[0]["Time"])
    if abs(dt) < 1e-9:
        return 0.0

    return float((rows[1][column] - rows[0][column]) / dt)


def _solve_body_plane_state(body, row: np.void, columns: Dict[str, str], subtract_z0: bool) -> np.ndarray:
    """Solve [heave, roll, pitch] or their rates from corner-height traces."""
    matrix = []
    values = []
    for corner in CORNERS:
        x_i = float(body.corner_offsets[corner]["x"])
        y_i = float(body.corner_offsets[corner]["y"])
        matrix.append([1.0, y_i, -x_i])

        value = float(row[columns[corner]])
        if subtract_z0:
            value -= float(body.params.h_CG)
        values.append(value)

    solution, *_ = np.linalg.lstsq(np.asarray(matrix, dtype=float), np.asarray(values, dtype=float), rcond=None)
    return solution


def generate(
    dt: Optional[float] = None,
    duration: Optional[float] = None,
    log_csv_path: Optional[str] = None,
) -> "LoggedSineSweepScenario":
    """
    Generate a scenario from the logged drive torque and steer references.
    """
    log_time, log_drive_trq, log_steer_ref = _load_logged_inputs(log_csv_path)

    if dt is None and duration is None:
        time = log_time.copy()
        drive_trq = {corner: values.copy() for corner, values in log_drive_trq.items()}
        steer_ref = {corner: values.copy() for corner, values in log_steer_ref.items()}
    else:
        step = float(dt) if dt is not None else float(np.median(np.diff(log_time)))
        total_time = float(duration) if duration is not None else float(log_time[-1])
        if total_time > float(log_time[-1]) + 1e-9:
            raise ValueError(
                "Requested duration exceeds the logged scenario time range: "
                f"{total_time:.6f} > {float(log_time[-1]):.6f}"
            )

        time = np.arange(float(log_time[0]), total_time, step)
        drive_trq = {
            corner: np.interp(time, log_time, values)
            for corner, values in log_drive_trq.items()
        }
        steer_ref = {
            corner: np.interp(time, log_time, values)
            for corner, values in log_steer_ref.items()
        }

    resolved_log_path = str(Path(log_csv_path) if log_csv_path is not None else DEFAULT_LOG_PATH)

    data = dict(
        time=time,
        drive_trq=drive_trq,
        steer_ref=steer_ref,
        steer_trq={corner: np.zeros(time.shape, dtype=float) for corner in CORNERS},
        brake_trq={corner: np.zeros(time.shape, dtype=float) for corner in CORNERS},
        road_z={corner: np.zeros(time.shape, dtype=float) for corner in CORNERS},
        log_csv_path=resolved_log_path,
    )

    print(
        f"Scenario generated from log: {Path(log_csv_path).name if log_csv_path else DEFAULT_LOG_PATH.name}, "
        f"steps={len(time)}, t_end={float(time[-1]):.3f}s"
    )
    return LoggedSineSweepScenario(data)


class Driver:
    """
    Logged steer-reference tracking PD controller.
    """

    def __init__(
        self,
        data: dict,
        kp: float = 6.0,
        kd: float = 0.3,
        torque_limit: Optional[float] = None,
    ):
        self.kp = float(kp)
        self.kd = float(kd)
        self.torque_limit = torque_limit

        time = np.asarray(data["time"], dtype=float)
        self._steer_ref = {
            corner: np.asarray(data["steer_ref"][corner], dtype=float)
            for corner in CORNERS
        }
        edge_order = 2 if time.size > 2 else 1
        self._steer_ref_dot = {
            corner: np.gradient(values, time, edge_order=edge_order)
            for corner, values in self._steer_ref.items()
        }

    def reference(self, idx: int) -> Dict[str, float]:
        """Return the logged steering-angle reference at the given time index."""
        return {corner: float(self._steer_ref[corner][idx]) for corner in CORNERS}

    def steer(
        self,
        idx: int,
        steering_angle: Dict[str, float],
        steering_rate: Dict[str, float],
    ) -> Dict[str, float]:
        """Compute per-corner steering torque commands."""
        torque_cmd = {}
        for corner in CORNERS:
            error = self._steer_ref[corner][idx] - float(steering_angle[corner])
            rate_error = self._steer_ref_dot[corner][idx] - float(steering_rate[corner])
            torque = self.kp * error + self.kd * rate_error

            if self.torque_limit is not None:
                torque = float(np.clip(torque, -self.torque_limit, self.torque_limit))

            torque_cmd[corner] = float(torque)

        return torque_cmd


class LoggedSineSweepScenario(dict):
    """
    Scenario container that can generate full per-corner inputs online.

    The steering torque cannot be fixed at load time because it depends on the
    current steering state. This wrapper keeps the logged references and the
    controller together so callers can request one-step inputs directly from the
    scenario module.
    """

    def __init__(self, data: Dict):
        super().__init__(data)
        self.driver = Driver(data)
        self.log_csv_path = data.get("log_csv_path")

    def apply_initial_conditions(self, body) -> None:
        """Align the model to the first logged operating point."""
        apply_initial_conditions(body, log_csv_path=self.log_csv_path)

    def corner_inputs(
        self,
        idx: int,
        body,
        t_susp: Optional[Dict[str, float]] = None,
    ) -> Dict[str, Dict[str, float]]:
        """
        Build full per-corner model inputs for the current simulation step.
        """
        steer_cmd = self.driver.steer(
            idx=idx,
            steering_angle={corner: body.corners[corner].state.steering_angle for corner in CORNERS},
            steering_rate={corner: body.corners[corner].steering.state.steering_rate for corner in CORNERS},
        )
        t_susp = t_susp or {corner: 0.0 for corner in CORNERS}

        for corner in CORNERS:
            self["steer_trq"][corner][idx] = steer_cmd[corner]

        return {
            corner: {
                "T_steer": steer_cmd[corner],
                "T_brk": self["brake_trq"][corner][idx],
                "T_Drv": self["drive_trq"][corner][idx],
                "T_susp": t_susp[corner],
                "z_road": self["road_z"][corner][idx],
            }
            for corner in CORNERS
        }


def apply_initial_conditions(body, log_csv_path: Optional[str] = None) -> None:
    """
    Align the model state to the first logged operating point.

    This reduces the large start-up transient caused by feeding a moving-vehicle
    log into a zero-state simulation.
    """
    rows = _load_initial_rows(log_csv_path)
    row0 = rows[0]

    heave, roll, pitch = _solve_body_plane_state(body, row0, _SPRUNG_Z_COLUMNS, subtract_z0=True)
    heave_dot, roll_rate, pitch_rate = _solve_body_plane_state(
        body,
        row0,
        _SPRUNG_Z_DOT_COLUMNS,
        subtract_z0=False,
    )

    body.state.heave = float(heave)
    body.state.roll = float(roll)
    body.state.pitch = float(pitch)
    body.state.yaw = float(row0["cm_Yaw"])
    body.state.velocity_x = float(row0["cm_vx"])
    body.state.velocity_y = float(row0["cm_vy"])
    body.state.heave_dot = float(heave_dot)
    body.state.roll_rate = float(roll_rate)
    body.state.pitch_rate = float(pitch_rate)
    body.state.yaw_rate = float(row0["cm_YawRate"])
    body.state.ax_prev = float(row0["cm_ax"])
    body.state.ay_prev = float(row0["cm_ay"])

    for corner in CORNERS:
        corner_model = body.corners[corner]

        steering_angle = float(row0[_STEER_REFERENCE_COLUMNS[corner]])
        steering_rate = _first_step_rate(rows, _STEER_REFERENCE_COLUMNS[corner])
        wheel_speed = float(row0[_WHEEL_SPEED_COLUMNS[corner]])
        z_u_abs = float(row0[_UNSPRUNG_Z_COLUMNS[corner]])
        z_u_dot = _first_step_rate(rows, _UNSPRUNG_Z_COLUMNS[corner])

        corner_model.steering.state.steering_angle = steering_angle
        corner_model.steering.state.steering_rate = steering_rate
        corner_model.steering.state.steering_torque = 0.0
        corner_model.steering.state.self_aligning_torque = 0.0

        corner_model.drive.state.wheel_speed = wheel_speed

        z_cg = float(corner_model.suspension.params.z_CG0 + body.state.heave)
        z_body_abs = float(corner_model.suspension._calculate_body_height(z_cg, body.state.roll, body.state.pitch))
        z_body_dot = float(
            body.state.heave_dot
            + body.state.roll_rate * corner_model.suspension._y_i
            - body.state.pitch_rate * corner_model.suspension._x_i
        )

        delta_s = float((z_body_abs - z_u_abs) - corner_model.suspension.params.L_s0)
        delta_s_dot = float(z_body_dot - z_u_dot)
        delta_t = float(max(0.0, corner_model.suspension.params.R_w - z_u_abs))
        delta_t_dot = float(-z_u_dot)

        if delta_s_dot < 0.0:
            damper = corner_model.suspension.params.C_damper_compression
        else:
            damper = corner_model.suspension.params.C_damper_rebound

        f_spring = float(-corner_model.suspension.params.K_spring * delta_s)
        f_damper = float(-damper * delta_s_dot)
        f_s = float(f_spring + f_damper)
        f_z = float(corner_model.suspension._calculate_tire_force(delta_t, delta_t_dot))

        corner_model.suspension.state.z_u_abs = z_u_abs
        corner_model.suspension.state.z_u_dot = z_u_dot
        corner_model.suspension.state.z_body_abs = z_body_abs
        corner_model.suspension.state.delta_s = delta_s
        corner_model.suspension.state.delta_s_dot = delta_s_dot
        corner_model.suspension.state.delta_t = delta_t
        corner_model.suspension.state.F_active = 0.0
        corner_model.suspension.state.F_spring = f_spring
        corner_model.suspension.state.F_damper = f_damper
        corner_model.suspension.state.F_s = f_s
        corner_model.suspension.state.F_z = f_z

        corner_model.state.steering_angle = steering_angle
        corner_model.state.omega_wheel = wheel_speed
        corner_model.state.F_s = f_s
        corner_model.state.F_z = f_z

    corner_inputs = body.get_corner_inputs()
    for corner in CORNERS:
        corner_model = body.corners[corner]
        steer_angle = float(corner_model.state.steering_angle)
        vx_body, vy_body = corner_inputs["wheel_velocities"][corner]
        cos_delta = float(np.cos(steer_angle))
        sin_delta = float(np.sin(steer_angle))
        vx_local = float(cos_delta * vx_body + sin_delta * vy_body)
        vy_local = float(-sin_delta * vx_body + cos_delta * vy_body)

        f_z = float(corner_model.suspension.state.F_z)
        alpha = float(corner_model.lateral_tire.calculate_slip_angle(vx_local, vy_local))
        kappa = float(corner_model.longitudinal_tire.calculate_slip_ratio(corner_model.drive.state.wheel_speed, vx_local))
        f_x = float(corner_model.longitudinal_tire.calculate_force(kappa, f_z))
        f_y = float(corner_model.lateral_tire.calculate_force(alpha, f_z))
        m_align = float(corner_model.lateral_tire.calculate_aligning_torque(alpha, f_z, Fy_override=f_y))

        corner_model.longitudinal_tire.state.slip_ratio = kappa
        corner_model.longitudinal_tire.state.longitudinal_force = f_x
        corner_model.lateral_tire.state.slip_angle = alpha
        corner_model.lateral_tire.state.lateral_force = f_y
        corner_model.lateral_tire.state.aligning_torque = m_align

        corner_model.steering.state.self_aligning_torque = m_align
        corner_model.state.F_x_tire = f_x
        corner_model.state.F_y_tire = f_y
