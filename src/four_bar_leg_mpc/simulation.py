"""Closed-loop simulation utilities for the one-leg MPC demonstration."""

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
    """Closed-loop state, foot, control, safety, and prediction histories."""

    q: FloatArray
    foot: FloatArray
    controls: FloatArray
    safety: dict[str, FloatArray]
    predictions: list[FloatArray]
    target: FloatArray
    q_goal: FloatArray


def run_simulation(
    steps: int = 35,
    initial_q: FloatArray | None = None,
    goal_q: FloatArray | None = None,
    leg_parameters: LegParameters = DEFAULT_LEG_PARAMETERS,
    safety_parameters: SafetyParameters = DEFAULT_SAFETY_PARAMETERS,
    mpc_parameters: MPCParameters = DEFAULT_MPC_PARAMETERS,
) -> SimulationResult:
    """Run the receding-horizon controller against its kinematic model."""
    if steps <= 0:
        raise ValueError("steps must be positive.")

    q = (
        np.array([0.35, 3.50, 2.00], dtype=float)
        if initial_q is None
        else np.asarray(initial_q, dtype=float).copy()
    )
    q_goal = (
        np.array([-0.20, 2.70, 4.20], dtype=float)
        if goal_q is None
        else np.asarray(goal_q, dtype=float).copy()
    )
    if q.shape != (3,) or q_goal.shape != (3,):
        raise ValueError("initial_q and goal_q must have shape (3,).")

    target = foot_position(q_goal, leg_parameters)
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
            foot_target=target,
            q_nominal=q_goal,
            warm_start=warm_start,
            previous_velocity=previous_velocity,
            leg_parameters=leg_parameters,
            safety_parameters=safety_parameters,
            mpc_parameters=mpc_parameters,
        )
        q_prediction = rollout(q, controls, mpc_parameters)
        predicted_feet = np.asarray(
            [foot_position(q_pred, leg_parameters) for q_pred in q_prediction],
            dtype=float,
        )

        q_history.append(q.copy())
        foot_history.append(foot_position(q, leg_parameters))
        control_history.append(velocity.copy())
        predictions.append(predicted_feet)

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
        target=target,
        q_goal=q_goal,
    )
