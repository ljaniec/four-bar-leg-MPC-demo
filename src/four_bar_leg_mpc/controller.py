"""Kinematic Model Predictive Control for one four-bar leg."""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np
from numpy.typing import NDArray
from scipy.optimize import minimize

from .geometry import DEFAULT_LEG_PARAMETERS, LegParameters, foot_position
from .safety import DEFAULT_SAFETY_PARAMETERS, SafetyParameters, safety_margins

FloatArray = NDArray[np.float64]


@dataclass(frozen=True)
class MPCParameters:
    """Numerical settings and cost weights for the demonstration controller."""

    dt: float = 0.08
    horizon: int = 8
    velocity_limit: FloatArray = field(
        default_factory=lambda: np.array([1.20, 1.20, 1.50], dtype=float)
    )
    foot_tracking_weight: float = 800.0
    posture_weight: float = 0.40
    velocity_weight: float = 0.03
    velocity_change_weight: float = 0.20
    terminal_foot_weight: float = 2000.0
    max_iterations: int = 60
    tolerance: float = 1e-5


DEFAULT_MPC_PARAMETERS = MPCParameters()


def rollout(
    q0: FloatArray,
    controls: FloatArray,
    mpc_parameters: MPCParameters = DEFAULT_MPC_PARAMETERS,
) -> FloatArray:
    """Roll out ``q[k+1] = q[k] + dt * u[k]`` over the prediction horizon."""
    q = np.asarray(q0, dtype=float).copy()
    if q.shape != (3,):
        raise ValueError(f"q0 must have shape (3,), received {q.shape}.")

    controls = np.asarray(controls, dtype=float).reshape(mpc_parameters.horizon, 3)
    trajectory = [q.copy()]
    for velocity in controls:
        q = q + mpc_parameters.dt * velocity
        trajectory.append(q.copy())
    return np.asarray(trajectory, dtype=float)


def objective(
    control_flat: FloatArray,
    q0: FloatArray,
    foot_target: FloatArray,
    q_nominal: FloatArray,
    previous_velocity: FloatArray,
    leg_parameters: LegParameters = DEFAULT_LEG_PARAMETERS,
    mpc_parameters: MPCParameters = DEFAULT_MPC_PARAMETERS,
) -> float:
    """Evaluate the finite-horizon tracking, posture, and smoothness cost."""
    controls = np.asarray(control_flat, dtype=float).reshape(mpc_parameters.horizon, 3)
    q_prediction = rollout(q0, controls, mpc_parameters)

    cost = 0.0
    previous = np.asarray(previous_velocity, dtype=float)
    for index in range(mpc_parameters.horizon):
        q_next = q_prediction[index + 1]
        foot_error = foot_position(q_next, leg_parameters) - foot_target
        posture_error = q_next - q_nominal
        velocity = controls[index]
        velocity_change = velocity - previous

        cost += mpc_parameters.foot_tracking_weight * float(foot_error @ foot_error)
        cost += mpc_parameters.posture_weight * float(posture_error @ posture_error)
        cost += mpc_parameters.velocity_weight * float(velocity @ velocity)
        cost += mpc_parameters.velocity_change_weight * float(
            velocity_change @ velocity_change
        )
        previous = velocity

    terminal_error = foot_position(q_prediction[-1], leg_parameters) - foot_target
    cost += mpc_parameters.terminal_foot_weight * float(
        terminal_error @ terminal_error
    )
    return float(cost)


def constraint_residuals(
    control_flat: FloatArray,
    q0: FloatArray,
    leg_parameters: LegParameters = DEFAULT_LEG_PARAMETERS,
    safety_parameters: SafetyParameters = DEFAULT_SAFETY_PARAMETERS,
    mpc_parameters: MPCParameters = DEFAULT_MPC_PARAMETERS,
) -> FloatArray:
    """Return nonlinear inequality residuals; feasible values are nonnegative."""
    q_prediction = rollout(
        q0,
        np.asarray(control_flat, dtype=float).reshape(mpc_parameters.horizon, 3),
        mpc_parameters,
    )
    residuals: list[float] = []
    for q in q_prediction[1:]:
        residuals.extend((q - leg_parameters.q_min).tolist())
        residuals.extend((leg_parameters.q_max - q).tolist())
        margins = safety_margins(q, leg_parameters, safety_parameters)
        residuals.append(margins["lateral_spread"])
        residuals.append(margins["four_bar_toggle"])
    return np.asarray(residuals, dtype=float)


def solve_mpc(
    q: FloatArray,
    foot_target: FloatArray,
    q_nominal: FloatArray,
    warm_start: FloatArray | None = None,
    previous_velocity: FloatArray | None = None,
    leg_parameters: LegParameters = DEFAULT_LEG_PARAMETERS,
    safety_parameters: SafetyParameters = DEFAULT_SAFETY_PARAMETERS,
    mpc_parameters: MPCParameters = DEFAULT_MPC_PARAMETERS,
) -> tuple[FloatArray, FloatArray]:
    """Solve one constrained MPC problem and return first/all controls."""
    q = np.asarray(q, dtype=float)
    foot_target = np.asarray(foot_target, dtype=float)
    q_nominal = np.asarray(q_nominal, dtype=float)
    if warm_start is None:
        warm_start = np.zeros((mpc_parameters.horizon, 3), dtype=float)
    if previous_velocity is None:
        previous_velocity = np.zeros(3, dtype=float)

    bounds = [
        (-float(mpc_parameters.velocity_limit[j]), float(mpc_parameters.velocity_limit[j]))
        for _ in range(mpc_parameters.horizon)
        for j in range(3)
    ]

    result = minimize(
        objective,
        np.asarray(warm_start, dtype=float).reshape(-1),
        args=(
            q,
            foot_target,
            q_nominal,
            np.asarray(previous_velocity, dtype=float),
            leg_parameters,
            mpc_parameters,
        ),
        method="SLSQP",
        bounds=bounds,
        constraints={
            "type": "ineq",
            "fun": lambda controls: constraint_residuals(
                controls,
                q,
                leg_parameters,
                safety_parameters,
                mpc_parameters,
            ),
        },
        options={
            "maxiter": mpc_parameters.max_iterations,
            "ftol": mpc_parameters.tolerance,
            "disp": False,
        },
    )

    minimum_constraint = float(
        np.min(
            constraint_residuals(
                result.x,
                q,
                leg_parameters,
                safety_parameters,
                mpc_parameters,
            )
        )
    )
    if not result.success or minimum_constraint < -1e-6:
        raise RuntimeError(
            "MPC solver failed: "
            f"{result.message}; minimum constraint residual={minimum_constraint:.3e}."
        )

    controls = np.asarray(result.x, dtype=float).reshape(mpc_parameters.horizon, 3)
    return controls[0].copy(), controls
