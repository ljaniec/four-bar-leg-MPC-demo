"""Constrained kinematic MPC for one-leg Cartesian foot set-point regulation."""

from .controller import MPCParameters, solve_mpc
from .geometry import LegParameters, foot_position, leg_points, passive_joint_angles
from .safety import SafetyParameters, minimum_safety_margin, safety_margins
from .simulation import SimulationResult, run_simulation

__all__ = [
    "LegParameters",
    "MPCParameters",
    "SafetyParameters",
    "SimulationResult",
    "foot_position",
    "leg_points",
    "minimum_safety_margin",
    "passive_joint_angles",
    "run_simulation",
    "safety_margins",
    "solve_mpc",
]

__version__ = "0.1.1"
