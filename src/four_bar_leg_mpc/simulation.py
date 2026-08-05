"""Closed-loop set-point-regulation simulation for the one-leg MPC demo."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from numpy.typing import NDArray

from .controller import DEFAULT_MPC_PARAMETERS, MPCParameters, rollout, solve_mpc
from .geometry import DEFAULT_LEG_PARAMETERS, LegParameters, foot_position
from .safety import DEFAULT_SAFETY_PARAMETERS, SafetyParameters, safety_margins

FloatArray = NDArray[np.float64]


@dataclass(frozen=True)
class SimulationResult:
    """Closed-loop histories for Cartesian foot set-point regulation."""

    q: FloatArray
    foot: FloatArray
    controls: FloatArray
    safety: dict[str, FloatArray]
    predictions: list[FloatArray]
    foot_setpoint: FloatArray
    q_nominal: FloatArray


def run_simulation(
    steps: int = 35,
    initial_q: FloatArray | None = None,
    nominal_q: FloatArray | None = None,
    leg_parameters: LegParameters = DEFAULT_LEG_PARAMETERS,
    safety_parameters: SafetyParameters = DEFAULT_SAFETY_PARAMETERS,
    mpc_parameters: MPCParameters = DEFAULT_MPC_PARAMETERS,
) -> SimulationResult:
    """Regulate the foot from its initial position to one fixed Cartesian set-point.

    The set-point is computed once from ``nominal_q`` and remains constant for
    every closed-loop MPC solve. The predicted point sequences are optimizer
    rollouts, not a desired path and not a time-indexed reference trajectory.
    """
    if steps <= 0:
        raise ValueError("steps must be positive.")

    q = (
        np.array([0.35, 3.50, 2.00], dtype=float)
        if initial_q is None
        else np.asarray(initial_q, dtype=float).copy()
    )
    q_nominal = (
        np.array([-0.20, 2.70, 4.20], dtype=float)
        if nominal_q is None
        else np.asarray(nominal_q, dtype=float).copy()
    )
    if q.shape != (3,) or q_nominal.shape != (3,):
        raise ValueError("initial_q and nominal_q must have shape (3,).")

    foot_setpoint = foot_position(q_nominal, leg_parameters)
    warm_start = np.zeros((mpc_parameters.horizon, 3), dtype=float)
    previous_velocity = np.zeros(3, dtype=float)

    q_history: list[FloatArray] = []
    foot_history: list[FloatArray] = []
    control_history: list[FloatArray] = []
    predictions: list[FloatArray] = []
    safety_history: dict[str, list[float]] = {
        key: []
        for key in safety_margins(q, leg_parameters, safety_parameters)
    }

    for _ in range(steps):
        velocity, controls = solve_mpc(
            q=q,
            foot_setpoint=foot_setpoint,
            q_nominal=q_nominal,
            warm_start=warm_start,
            previous_velocity=previous_velocity,
            leg_parameters=leg_parameters,
            safety_parameters=safety_parameters,
            mpc_parameters=mpc_parameters,
        )
        q_prediction = rollout(q, controls, mpc_parameters)
        predicted_foot_motion = np.asarray(
            [foot_position(q_pred, leg_parameters) for q_pred in q_prediction],
            dtype=float,
        )

        q_history.append(q.copy())
        foot_history.append(foot_position(q, leg_parameters))
        control_history.append(velocity.copy())
        predictions.append(predicted_foot_motion)

        for key, value in safety_margins(
            q,
            leg_parameters,
            safety_parameters,
        ).items():
            safety_history[key].append(value)

        q = q + mpc_parameters.dt * velocity
        previous_velocity = velocity
        warm_start = np.vstack([controls[1:], controls[-1]])

    q_history.append(q.copy())
    foot_history.append(foot_position(q, leg_parameters))
    for key, value in safety_margins(
        q,
        leg_parameters,
        safety_parameters,
    ).items():
        safety_history[key].append(value)

    return SimulationResult(
        q=np.asarray(q_history, dtype=float),
        foot=np.asarray(foot_history, dtype=float),
        controls=np.asarray(control_history, dtype=float),
        safety={
            key: np.asarray(values, dtype=float)
            for key, values in safety_history.items()
        },
        predictions=predictions,
        foot_setpoint=foot_setpoint,
        q_nominal=q_nominal,
    )
