"""Safety margins used by the kinematic MPC demonstration."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from numpy.typing import NDArray

from .geometry import DEFAULT_LEG_PARAMETERS, LegParameters, foot_position, passive_joint_angles

FloatArray = NDArray[np.float64]


@dataclass(frozen=True)
class SafetyParameters:
    """Heuristic presentation-level safety-set parameters."""

    lateral_center: float = 0.081
    max_lateral_deviation: float = 0.145
    min_toggle_sine: float = 0.12


DEFAULT_SAFETY_PARAMETERS = SafetyParameters()


def safety_margins(
    q: FloatArray,
    leg_parameters: LegParameters = DEFAULT_LEG_PARAMETERS,
    safety_parameters: SafetyParameters = DEFAULT_SAFETY_PARAMETERS,
) -> dict[str, float]:
    """Return positive-inside safety margins for a joint configuration.

    These margins support a didactic constrained-MPC example. They are not a
    validated hardware safety specification or a complete Control Barrier
    Function design.
    """
    q = np.asarray(q, dtype=float)
    if q.shape != (3,):
        raise ValueError(f"q must have shape (3,), received {q.shape}.")

    foot = foot_position(q, leg_parameters)
    passive = passive_joint_angles(float(q[2]), leg_parameters)

    lower_joint_margin = float(np.min(q - leg_parameters.q_min))
    upper_joint_margin = float(np.min(leg_parameters.q_max - q))

    lateral_offset = foot[1] - safety_parameters.lateral_center
    lateral_margin = float(
        safety_parameters.max_lateral_deviation**2 - lateral_offset**2
    )
    toggle_margin = float(
        np.sin(passive.gamma) ** 2 - safety_parameters.min_toggle_sine**2
    )

    return {
        "joint_lower": lower_joint_margin,
        "joint_upper": upper_joint_margin,
        "lateral_spread": lateral_margin,
        "four_bar_toggle": toggle_margin,
    }


def minimum_safety_margin(
    q: FloatArray,
    leg_parameters: LegParameters = DEFAULT_LEG_PARAMETERS,
    safety_parameters: SafetyParameters = DEFAULT_SAFETY_PARAMETERS,
) -> float:
    """Return the minimum of all demonstration safety margins."""
    return min(safety_margins(q, leg_parameters, safety_parameters).values())
