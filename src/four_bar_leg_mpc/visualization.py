"""Static and animated visualizations for the four-bar leg set-point demo."""

from __future__ import annotations

from pathlib import Path
from typing import Literal

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation, PillowWriter
from matplotlib.axes import Axes

from .geometry import (
    DEFAULT_LEG_PARAMETERS,
    LegParameters,
    leg_points,
    passive_joint_angles,
)
from .safety import safety_margins
from .simulation import SimulationResult, run_simulation

Language = Literal["en", "pl"]

_LABELS: dict[Language, dict[str, str]] = {
    "en": {
        "x": "x [m]",
        "y": "y [m]",
        "z": "z [m]",
        "hip": "Motor 1: hip motion inward and outward",
        "sweep": "Motor 2: rotation of the complete planar leg",
        "extend": "Motor 3: four-bar actuation and effective leg extension",
        "setpoint_title": "Kinematic MPC: Cartesian foot set-point regulation",
        "motion_history": "closed-loop foot motion (result, not a reference path)",
        "setpoint": "fixed Cartesian set-point",
        "states": "System state: three actuated coordinates",
        "time": "time [s]",
        "angle": "angle [rad]",
        "q_hip": "q_hip: inward / outward",
        "q_sweep": "q_sweep: complete-leg rotation",
        "q_extend": "q_extend: effective extension",
        "margins": "Constraint margins: positive values are feasible",
        "margin_value": "margin value",
        "prediction": "predicted motion over the MPC horizon",
        "executed": "closed-loop motion history",
        "animation": "Foot set-point regulation: state and MPC prediction",
        "minimum_margin": "minimum margin",
        "side_view": (
            "Motor 3: short-link rotation changes foot position and effective length"
        ),
        "plane_coordinate": "mechanism-plane coordinate [m]",
        "roles": "One leg: three actuators and a closed mechanism",
        "roles_text": (
            "1 - hip rotation: inward / outward\n"
            "2 - complete-leg rotation in its plane\n"
            "3 - short-link drive: extension / contraction\n"
            "F - presentation-level foot point"
        ),
    },
    "pl": {
        "x": "x [m]",
        "y": "y [m]",
        "z": "z [m]",
        "hip": "Silnik 1: ruch biodra do wewnątrz i na zewnątrz",
        "sweep": "Silnik 2: obrót całej nogi w jej płaszczyźnie",
        "extend": "Silnik 3: napęd czworoboku i efektywna zmiana długości nogi",
        "setpoint_title": (
            "Kinematyczne MPC: regulacja położenia stopy do stałego punktu"
        ),
        "motion_history": "wykonany ruch stopy (wynik, nie ścieżka zadana)",
        "setpoint": "stały punkt zadany",
        "states": "Stan układu: trzy współrzędne napędowe",
        "time": "czas [s]",
        "angle": "kąt [rad]",
        "q_hip": "q_hip: do wewnątrz / na zewnątrz",
        "q_sweep": "q_sweep: obrót całej nogi",
        "q_extend": "q_extend: zmiana długości",
        "margins": (
            "Marginesy ograniczeń: wartości dodatnie oznaczają obszar dopuszczalny"
        ),
        "margin_value": "wartość marginesu",
        "prediction": "przewidywany ruch w horyzoncie MPC",
        "executed": "historia wykonanego ruchu",
        "animation": "Regulacja do punktu: stan mechanizmu i predykcja MPC",
        "minimum_margin": "minimalny margines",
        "side_view": (
            "Silnik 3: obrót krótkiego pręta zmienia położenie stopy i długość nogi"
        ),
        "plane_coordinate": "współrzędna w płaszczyźnie mechanizmu [m]",
        "roles": "Jedna noga: trzy napędy i zamknięty mechanizm",
        "roles_text": (
            "1 - obrót biodra: noga do wewnątrz / na zewnątrz\n"
            "2 - obrót całej nogi w jej płaszczyźnie\n"
            "3 - napęd krótkiego pręta: wydłużanie / skracanie\n"
            "F - umowny punkt stopy"
        ),
    },
}


def _labels(language: Language) -> dict[str, str]:
    try:
        return _LABELS[language]
    except KeyError as error:
        raise ValueError(f"Unsupported language: {language!r}.") from error


def draw_leg(
    axes: Axes,
    q: np.ndarray,
    linewidth: float = 2.5,
    parameters: LegParameters = DEFAULT_LEG_PARAMETERS,
) -> None:
    """Draw the proximal chain and both branches of the closed mechanism."""
    leg = leg_points(q, parameters)
    for points in (
        np.vstack([leg.base, leg.hip_axis, leg.planar_axis]),
        np.vstack([leg.active_pivot, leg.passive_pivot]),
        np.vstack([leg.active_pivot, leg.active_short_tip, leg.active_chain_end]),
        np.vstack([leg.passive_pivot, leg.passive_chain_end]),
    ):
        axes.plot(
            points[:, 0],
            points[:, 1],
            points[:, 2],
            marker="o",
            linewidth=linewidth,
        )
    axes.scatter(
        [leg.foot[0]],
        [leg.foot[1]],
        [leg.foot[2]],
        marker="x",
        s=70,
    )


def configure_3d_axis(axes: Axes, title: str, language: Language) -> None:
    labels = _labels(language)
    axes.set_title(title)
    axes.set_xlabel(labels["x"])
    axes.set_ylabel(labels["y"])
    axes.set_zlabel(labels["z"])
    axes.set_xlim(-0.16, 0.18)
    axes.set_ylim(-0.12, 0.20)
    axes.set_zlim(0.00, 0.36)
    axes.invert_zaxis()
    axes.set_box_aspect((1.0, 1.0, 1.1))
    axes.view_init(elev=24, azim=-56)


def save_motor_role_figure(
    output: Path,
    q_values: list[np.ndarray],
    title: str,
    language: Language,
) -> None:
    figure = plt.figure(figsize=(7.2, 6.0), constrained_layout=True)
    axes = figure.add_subplot(111, projection="3d")
    for q in q_values:
        draw_leg(axes, q, linewidth=2.0)
    configure_3d_axis(axes, title, language)
    figure.savefig(output, dpi=180)
    plt.close(figure)


def save_annotated_roles_figure(output: Path, language: Language) -> None:
    labels = _labels(language)
    q_nominal = np.array([0.0, 3.1, 3.3], dtype=float)
    points = leg_points(q_nominal)
    figure = plt.figure(figsize=(7.8, 6.4), constrained_layout=True)
    axes = figure.add_subplot(111, projection="3d")
    draw_leg(axes, q_nominal, linewidth=3.0)
    configure_3d_axis(axes, labels["roles"], language)
    axes.text(*points.hip_axis, "1", fontsize=14)
    axes.text(*points.planar_axis, "2", fontsize=14)
    axes.text(*points.active_pivot, "3", fontsize=14)
    axes.text(*points.foot, "F", fontsize=14)
    axes.text2D(
        0.02,
        0.02,
        labels["roles_text"],
        transform=axes.transAxes,
        bbox={"boxstyle": "round", "alpha": 0.85},
    )
    figure.savefig(output, dpi=180)
    plt.close(figure)


def save_four_bar_side_view(output: Path, language: Language) -> None:
    labels = _labels(language)
    figure, axes = plt.subplots(figsize=(8.0, 5.4), constrained_layout=True)
    parameters = DEFAULT_LEG_PARAMETERS
    for q_extend in [2.2, 3.3, 4.4]:
        passive = passive_joint_angles(q_extend, parameters)
        active_angle = parameters.third_joint_gear_correction + q_extend
        active_pivot = np.array([0.0, 0.0])
        passive_pivot = np.array([-parameters.short_link, 0.0])
        short_tip = active_pivot + parameters.short_link * np.array(
            [np.cos(active_angle), np.sin(active_angle)]
        )
        active_end = short_tip + parameters.long_link * np.array(
            [
                np.cos(active_angle + np.pi - passive.fourth_joint),
                np.sin(active_angle + np.pi - passive.fourth_joint),
            ]
        )
        passive_end = passive_pivot + parameters.long_link * np.array(
            [np.cos(-passive.fifth_joint), np.sin(-passive.fifth_joint)]
        )
        foot = 0.5 * (active_end + passive_end)
        axes.plot(*np.vstack([passive_pivot, active_pivot]).T, marker="o")
        axes.plot(
            *np.vstack([active_pivot, short_tip, foot]).T,
            marker="o",
            label=rf"$q_{{\mathrm{{extend}}}}={q_extend:.1f}$ rad",
        )
        axes.plot(*np.vstack([passive_pivot, foot]).T, marker="o")
        axes.scatter([foot[0]], [foot[1]], marker="x", s=70)
    axes.set_aspect("equal", adjustable="box")
    axes.set_title(labels["side_view"])
    axes.set_xlabel(labels["plane_coordinate"])
    axes.set_ylabel(labels["plane_coordinate"])
    axes.grid(True)
    axes.legend()
    figure.savefig(output, dpi=180)
    plt.close(figure)


def save_setpoint_regulation_figure(
    result: SimulationResult,
    output: Path,
    language: Language,
) -> None:
    """Plot the resulting closed-loop motion toward one fixed foot set-point."""
    labels = _labels(language)
    figure = plt.figure(figsize=(7.6, 6.3), constrained_layout=True)
    axes = figure.add_subplot(111, projection="3d")
    selected = np.linspace(0, len(result.q) - 1, 6, dtype=int)
    for index in selected:
        draw_leg(axes, result.q[index], linewidth=1.7)
    axes.plot(
        result.foot[:, 0],
        result.foot[:, 1],
        result.foot[:, 2],
        marker=".",
        label=labels["motion_history"],
    )
    axes.scatter(
        [result.foot_setpoint[0]],
        [result.foot_setpoint[1]],
        [result.foot_setpoint[2]],
        marker="*",
        s=170,
        label=labels["setpoint"],
    )
    configure_3d_axis(axes, labels["setpoint_title"], language)
    axes.legend(loc="upper left")
    figure.savefig(output, dpi=180)
    plt.close(figure)


def save_joint_state_figure(
    result: SimulationResult,
    output: Path,
    language: Language,
) -> None:
    labels = _labels(language)
    time = np.arange(len(result.q)) * 0.08
    figure, axes = plt.subplots(figsize=(8.4, 4.7), constrained_layout=True)
    axes.plot(time, result.q[:, 0], label=labels["q_hip"])
    axes.plot(time, result.q[:, 1], label=labels["q_sweep"])
    axes.plot(time, result.q[:, 2], label=labels["q_extend"])
    axes.set_title(labels["states"])
    axes.set_xlabel(labels["time"])
    axes.set_ylabel(labels["angle"])
    axes.grid(True)
    axes.legend()
    figure.savefig(output, dpi=180)
    plt.close(figure)


def save_safety_figure(
    result: SimulationResult,
    output: Path,
    language: Language,
) -> None:
    labels = _labels(language)
    time = np.arange(len(result.q)) * 0.08
    figure, axes = plt.subplots(figsize=(8.4, 4.7), constrained_layout=True)
    for name, values in result.safety.items():
        axes.plot(time, values, label=name.replace("_", " "))
    axes.axhline(0.0)
    axes.set_title(labels["margins"])
    axes.set_xlabel(labels["time"])
    axes.set_ylabel(labels["margin_value"])
    axes.grid(True)
    axes.legend()
    figure.savefig(output, dpi=180)
    plt.close(figure)


def save_animation(
    result: SimulationResult,
    output: Path,
    language: Language,
) -> None:
    labels = _labels(language)
    figure = plt.figure(figsize=(7.4, 6.4), constrained_layout=True)
    axes = figure.add_subplot(111, projection="3d")

    def update(frame: int) -> None:
        axes.clear()
        q = result.q[frame]
        draw_leg(axes, q, linewidth=2.8)
        if frame < len(result.predictions):
            prediction = result.predictions[frame]
            axes.plot(
                prediction[:, 0],
                prediction[:, 1],
                prediction[:, 2],
                marker=".",
                label=labels["prediction"],
            )
        axes.plot(
            result.foot[: frame + 1, 0],
            result.foot[: frame + 1, 1],
            result.foot[: frame + 1, 2],
            label=labels["executed"],
        )
        axes.scatter(
            [result.foot_setpoint[0]],
            [result.foot_setpoint[1]],
            [result.foot_setpoint[2]],
            marker="*",
            s=170,
            label=labels["setpoint"],
        )
        configure_3d_axis(axes, labels["animation"], language)
        axes.legend(loc="upper left", fontsize=8)
        margins = safety_margins(q)
        annotation = (
            f"q_hip = {q[0]: .2f} rad\n"
            f"q_sweep = {q[1]: .2f} rad\n"
            f"q_extend = {q[2]: .2f} rad\n"
            f"{labels['minimum_margin']} = {min(margins.values()): .3f}"
        )
        axes.text2D(0.02, 0.02, annotation, transform=axes.transAxes)

    animation = FuncAnimation(
        figure,
        update,
        frames=len(result.q),
        interval=110,
        repeat=False,
    )
    animation.save(output, writer=PillowWriter(fps=9))
    plt.close(figure)


def generate_outputs(
    output_dir: Path,
    language: Language = "en",
    steps: int = 35,
    include_animation: bool = True,
) -> SimulationResult:
    """Generate all figures for the fixed Cartesian set-point regulation demo."""
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    labels = _labels(language)
    nominal = np.array([0.0, 3.1, 3.3], dtype=float)

    save_motor_role_figure(
        output_dir / "motor_1_hip_motion.png",
        [
            nominal + np.array([-0.45, 0.0, 0.0]),
            nominal,
            nominal + np.array([0.45, 0.0, 0.0]),
        ],
        labels["hip"],
        language,
    )
    save_motor_role_figure(
        output_dir / "motor_2_whole_leg_rotation.png",
        [
            nominal + np.array([0.0, -0.55, 0.0]),
            nominal,
            nominal + np.array([0.0, 0.55, 0.0]),
        ],
        labels["sweep"],
        language,
    )
    save_motor_role_figure(
        output_dir / "motor_3_four_bar_extension.png",
        [
            nominal + np.array([0.0, 0.0, -1.10]),
            nominal,
            nominal + np.array([0.0, 0.0, 1.10]),
        ],
        labels["extend"],
        language,
    )
    save_annotated_roles_figure(
        output_dir / "leg_three_motor_roles_annotated.png",
        language,
    )
    save_four_bar_side_view(
        output_dir / "four_bar_extension_side_view.png",
        language,
    )

    result = run_simulation(steps=steps)
    save_setpoint_regulation_figure(
        result,
        output_dir / "mpc_foot_setpoint_regulation.png",
        language,
    )
    save_joint_state_figure(result, output_dir / "mpc_joint_states.png", language)
    save_safety_figure(result, output_dir / "mpc_safety_margins.png", language)
    if include_animation:
        save_animation(result, output_dir / "mpc_leg_animation.gif", language)
    return result
