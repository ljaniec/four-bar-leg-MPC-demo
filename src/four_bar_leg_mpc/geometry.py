"""Kinematic model of the front-left four-bar leg.

The geometry follows the public ROS 2 model in ``delipl/quadruped_ros2``.
The passive-joint reconstruction is a Python adaptation of the loop-closure
calculation used by ``quadruped_controller::Leg::update_passive_joints``.

This module intentionally models one leg at the kinematic level. It is not a
calibrated rigid-body, motor, gearbox, or contact model.
"""

from __future__ import annotations

from collections.abc import Iterable
from dataclasses import dataclass, field

import numpy as np
from numpy.typing import NDArray

FloatArray = NDArray[np.float64]


def translation(x: float, y: float, z: float) -> FloatArray:
    """Return a homogeneous translation transform."""
    transform = np.eye(4, dtype=float)
    transform[:3, 3] = [x, y, z]
    return transform


def rotation_x(angle: float) -> FloatArray:
    """Return a homogeneous rotation around the x axis."""
    cosine, sine = np.cos(angle), np.sin(angle)
    transform = np.eye(4, dtype=float)
    transform[:3, :3] = np.array(
        [
            [1.0, 0.0, 0.0],
            [0.0, cosine, -sine],
            [0.0, sine, cosine],
        ],
        dtype=float,
    )
    return transform


def rotation_z(angle: float) -> FloatArray:
    """Return a homogeneous rotation around the z axis."""
    cosine, sine = np.cos(angle), np.sin(angle)
    transform = np.eye(4, dtype=float)
    transform[:3, :3] = np.array(
        [
            [cosine, -sine, 0.0],
            [sine, cosine, 0.0],
            [0.0, 0.0, 1.0],
        ],
        dtype=float,
    )
    return transform


def transformed_point(
    transform: FloatArray,
    xyz: Iterable[float] = (0.0, 0.0, 0.0),
) -> FloatArray:
    """Apply a homogeneous transform to a three-dimensional point."""
    homogeneous = np.r_[np.asarray(tuple(xyz), dtype=float), 1.0]
    return np.asarray((transform @ homogeneous)[:3], dtype=float)


@dataclass(frozen=True)
class LegParameters:
    """Geometric and joint-limit parameters for one front-left leg."""

    short_link: float = 0.125
    long_link: float = 0.210

    first_joint_origin: FloatArray = field(
        default_factory=lambda: np.array([0.031, 0.0, 0.0], dtype=float)
    )
    second_joint_origin: FloatArray = field(
        default_factory=lambda: np.array([0.064, 0.0, 0.023], dtype=float)
    )
    active_pivot_origin: FloatArray = field(
        default_factory=lambda: np.array([0.0, 0.0, 0.0555], dtype=float)
    )
    passive_pivot_origin: FloatArray = field(
        default_factory=lambda: np.array([-0.125, 0.0, 0.058], dtype=float)
    )
    distal_joint_origin: FloatArray = field(
        default_factory=lambda: np.array([0.125, 0.0, 0.0255], dtype=float)
    )

    second_joint_fixed_yaw: float = np.pi + 0.2793
    third_joint_gear_correction: float = -1.117 + np.pi

    # The upstream URDF has broad limits. The hip range is narrowed here to a
    # useful demonstration envelope, not asserted as a hardware safety limit.
    q_min: FloatArray = field(
        default_factory=lambda: np.array([-0.80, 1.57, 0.425], dtype=float)
    )
    q_max: FloatArray = field(
        default_factory=lambda: np.array([0.80, 4.71, 5.80], dtype=float)
    )


DEFAULT_LEG_PARAMETERS = LegParameters()


@dataclass(frozen=True)
class PassiveAngles:
    """Passive four-bar joint angles and internal toggle angle."""

    fourth_joint: float
    fifth_joint: float
    gamma: float


@dataclass(frozen=True)
class LegPoints:
    """Key points used for visualization and foot-position calculation."""

    base: FloatArray
    hip_axis: FloatArray
    planar_axis: FloatArray
    active_pivot: FloatArray
    active_short_tip: FloatArray
    passive_pivot: FloatArray
    active_chain_end: FloatArray
    passive_chain_end: FloatArray
    foot: FloatArray


def passive_joint_angles(
    q_extend: float,
    parameters: LegParameters = DEFAULT_LEG_PARAMETERS,
) -> PassiveAngles:
    """Reconstruct the two passive angles from the four-bar loop closure.

    The sign convention is for the front-left leg in the upstream controller.
    A ``ValueError`` is raised if the requested active angle leaves the real
    assembly domain.
    """
    l1 = parameters.short_link
    l2 = parameters.short_link
    l3 = parameters.long_link
    l4 = parameters.long_link
    passive_side_multiplier = -1.0

    theta2 = (
        np.pi
        - passive_side_multiplier * q_extend
        + parameters.third_joint_gear_correction
    )
    cosine2 = np.cos(theta2)
    sine2 = np.sin(theta2)

    diagonal_squared = l1 * l1 + l2 * l2 - 2.0 * l1 * l2 * cosine2
    if diagonal_squared < -1e-12:
        raise ValueError("Four-bar closure produced a negative squared distance.")
    diagonal = np.sqrt(max(0.0, diagonal_squared))

    gamma_argument = (l3 * l3 + l4 * l4 - diagonal * diagonal) / (2.0 * l3 * l4)
    if not -1.000001 <= gamma_argument <= 1.000001:
        raise ValueError("Four-bar closure is outside the real assembly domain.")
    gamma = np.arccos(np.clip(gamma_argument, -1.0, 1.0))

    sine_gamma = np.sin(gamma)
    cosine_gamma = np.cos(gamma)
    theta1 = 2.0 * np.arctan2(
        l2 * sine2 - l3 * sine_gamma,
        l2 * cosine2 + l4 - l1 - l3 * cosine_gamma,
    )
    theta3 = 2.0 * np.arctan2(
        l4 * sine_gamma - l2 * sine2,
        l1 + l3 - l2 * cosine2 - l4 * cosine_gamma,
    )

    fifth_joint = -passive_side_multiplier * (np.pi - theta2 + theta3)
    fourth_joint = passive_side_multiplier * (np.pi - theta1)
    return PassiveAngles(
        fourth_joint=float(fourth_joint),
        fifth_joint=float(fifth_joint),
        gamma=float(gamma),
    )


def leg_points(
    q: FloatArray,
    parameters: LegParameters = DEFAULT_LEG_PARAMETERS,
) -> LegPoints:
    """Return a schematic, URDF-consistent set of front-left leg points.

    ``q[0]`` is hip abduction/adduction, ``q[1]`` rotates the planar leg
    mechanism, and ``q[2]`` drives one short four-bar link and therefore changes
    the effective leg length.
    """
    q = np.asarray(q, dtype=float)
    if q.shape != (3,):
        raise ValueError(f"q must have shape (3,), received {q.shape}.")
    if not np.all(np.isfinite(q)):
        raise ValueError("q must contain finite values.")

    q_hip, q_sweep, q_extend = q
    passive = passive_joint_angles(float(q_extend), parameters)

    first_frame = rotation_x(-np.pi / 2.0)
    base = transformed_point(first_frame)

    second_frame = (
        first_frame
        @ translation(*parameters.first_joint_origin)
        @ rotation_x(float(q_hip))
    )
    hip_axis = transformed_point(second_frame)

    third_frame = (
        second_frame
        @ translation(*parameters.second_joint_origin)
        @ rotation_z(parameters.second_joint_fixed_yaw)
        @ rotation_z(float(q_sweep))
    )
    planar_axis = transformed_point(third_frame)

    active_frame = (
        third_frame
        @ translation(*parameters.active_pivot_origin)
        @ rotation_z(parameters.third_joint_gear_correction)
        @ rotation_z(float(q_extend))
    )
    active_pivot = transformed_point(active_frame)
    active_short_tip = transformed_point(
        active_frame,
        (parameters.short_link, 0.0, 0.0),
    )

    distal_frame = (
        active_frame
        @ translation(*parameters.distal_joint_origin)
        @ rotation_z(np.pi)
        @ rotation_z(-passive.fourth_joint)
    )
    active_chain_end = transformed_point(
        distal_frame,
        (parameters.long_link, 0.0, -0.0115),
    )

    passive_frame = (
        third_frame
        @ translation(*parameters.passive_pivot_origin)
        @ rotation_z(-passive.fifth_joint)
    )
    passive_pivot = transformed_point(passive_frame)
    passive_chain_end = transformed_point(
        passive_frame,
        (parameters.long_link, 0.0, 0.0),
    )

    # The source URDF closes the mechanism with a small out-of-plane offset.
    # The midpoint is a stable presentation-level end-effector definition.
    foot = 0.5 * (active_chain_end + passive_chain_end)

    return LegPoints(
        base=base,
        hip_axis=hip_axis,
        planar_axis=planar_axis,
        active_pivot=active_pivot,
        active_short_tip=active_short_tip,
        passive_pivot=passive_pivot,
        active_chain_end=active_chain_end,
        passive_chain_end=passive_chain_end,
        foot=foot,
    )


def foot_position(
    q: FloatArray,
    parameters: LegParameters = DEFAULT_LEG_PARAMETERS,
) -> FloatArray:
    """Return the presentation-level foot point for active coordinates ``q``."""
    return leg_points(q, parameters).foot
