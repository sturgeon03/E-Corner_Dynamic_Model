"""Recursive least squares estimator for steering parameters."""

from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np


@dataclass
class ScalarClamp:
    min_value: Optional[float] = None
    max_value: Optional[float] = None


@dataclass
class EstimatorClamp:
    j_min: Optional[float] = None
    j_max: Optional[float] = None
    b_min: Optional[float] = None
    b_max: Optional[float] = None


class SteeringRLS:
    """Estimate [J_cq, B_cq] from y = J*ddot(delta) + B*dot(delta)."""

    def __init__(
        self,
        init_j: float,
        init_b: float,
        forgetting_factor: float,
        p0: float,
        clamp: Optional[EstimatorClamp] = None,
    ) -> None:
        if not (0.0 < forgetting_factor <= 1.0):
            raise ValueError("forgetting_factor must be in (0, 1]")
        if p0 <= 0.0:
            raise ValueError("p0 must be positive")
        self.theta = np.array([float(init_j), float(init_b)], dtype=float)
        self.P = float(p0) * np.eye(2)
        self.lam = float(forgetting_factor)
        self.clamp = clamp or EstimatorClamp()
        self.sample_count = 0

    def update(self, y: float, phi_ddot: float, phi_dot: float) -> Tuple[float, float]:
        phi = np.array([float(phi_ddot), float(phi_dot)], dtype=float).reshape(2, 1)
        denom = self.lam + float(phi.T @ self.P @ phi)
        if denom <= 0.0 or not np.isfinite(denom):
            return self.theta[0], self.theta[1]
        K = (self.P @ phi) / denom
        err = float(y) - float(phi.T @ self.theta.reshape(2, 1))
        self.theta = self.theta + (K.flatten() * err)
        self.P = (self.P - K @ phi.T @ self.P) / self.lam
        self._apply_clamp()
        self.sample_count += 1
        return self.theta[0], self.theta[1]

    def _apply_clamp(self) -> None:
        j_min = self.clamp.j_min
        j_max = self.clamp.j_max
        b_min = self.clamp.b_min
        b_max = self.clamp.b_max
        if j_min is not None or j_max is not None:
            self.theta[0] = float(np.clip(self.theta[0], j_min, j_max))
        if b_min is not None or b_max is not None:
            self.theta[1] = float(np.clip(self.theta[1], b_min, b_max))

    def get_params(self) -> Tuple[float, float]:
        return float(self.theta[0]), float(self.theta[1])


class ScalarRLS:
    """Estimate a single parameter from y = theta * phi."""

    def __init__(
        self,
        init_value: float,
        forgetting_factor: float,
        p0: float,
        clamp: Optional[ScalarClamp] = None,
    ) -> None:
        if not (0.0 < forgetting_factor <= 1.0):
            raise ValueError("forgetting_factor must be in (0, 1]")
        if p0 <= 0.0:
            raise ValueError("p0 must be positive")
        self.theta = float(init_value)
        self.P = float(p0)
        self.lam = float(forgetting_factor)
        self.clamp = clamp or ScalarClamp()
        self.sample_count = 0

    def update(self, y: float, phi: float) -> float:
        phi = float(phi)
        denom = self.lam + self.P * phi * phi
        if denom <= 0.0 or not np.isfinite(denom):
            return float(self.theta)
        K = (self.P * phi) / denom
        err = float(y) - phi * float(self.theta)
        self.theta = float(self.theta + K * err)
        self.P = float((self.P - K * phi * self.P) / self.lam)
        self._apply_clamp()
        self.sample_count += 1
        return float(self.theta)

    def _apply_clamp(self) -> None:
        min_value = self.clamp.min_value
        max_value = self.clamp.max_value
        if min_value is not None or max_value is not None:
            self.theta = float(np.clip(self.theta, min_value, max_value))

    def get_value(self) -> float:
        return float(self.theta)
