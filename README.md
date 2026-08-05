# Four-Bar Leg MPC Demo

A small, reproducible Python package that demonstrates constrained **Model Predictive Control (MPC)** for one leg of a quadruped robot with a closed four-bar mechanism.

The model is based on the public ROS 2 robot description and controller code in [`delipl/quadruped_ros2`](https://github.com/delipl/quadruped_ros2). It distinguishes three active coordinates:

- `q_hip`: hip abduction/adduction, moving the leg inward and outward from the body;
- `q_sweep`: rotation of the complete planar leg mechanism;
- `q_extend`: active rotation of a short four-bar link, changing the effective leg length.

Two passive joint angles are reconstructed from the loop-closure equations. The controller is deliberately kinematic: its inputs are joint velocities, not motor torques.

## Repository layout

```text
.
├── src/four_bar_leg_mpc/   # Installable Python package
├── tests/                  # Local unit tests
├── scripts/                # Local setup, validation, and presentation scripts
├── docs/                   # English technical notes
└── presentation/           # Polish Beamer presentation source
```

The repository intentionally contains no GitHub Actions workflows. All validation commands are available locally.

## Installation

```bash
python -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install -e ".[dev]"
```

## Run the demo

```bash
four-bar-leg-mpc-demo --output-dir artifacts
```

Equivalent module invocation:

```bash
python -m four_bar_leg_mpc --output-dir artifacts
```

Useful options:

```bash
four-bar-leg-mpc-demo --help
four-bar-leg-mpc-demo --language pl --no-animation
```

## Build and test the package

```bash
./scripts/check.sh
python -m build
```

## Build the Polish presentation

The `presentation/` directory contains a 30-slide Polish Beamer deck. The PDF and all plot images are reproducible build artifacts.

```bash
./scripts/build_presentation.sh
```

To regenerate the Python-produced figures before compiling the deck:

```bash
./scripts/regenerate_presentation_figures.sh
./scripts/build_presentation.sh
```

The build requires XeLaTeX and common Beamer/TikZ packages.

## Scope and limitations

This repository is an educational and research-starting-point demonstrator. It does **not** yet provide:

- calibrated rigid-body dynamics;
- motor, gearbox, backlash, or joint-friction models;
- ground contact and friction dynamics;
- torque-level MPC;
- hardware-validated Control Barrier Functions;
- whole-body quadruped stabilization.

The upstream C++ forward kinematics also contains a TODO about verification after parameter changes. This package therefore treats the geometry as a transparent, testable working model rather than a certified representation of the physical robot.

## Attribution and licensing

The Python adaptation of the passive four-bar closure and the geometric constants are derived from the Apache-2.0-licensed `delipl/quadruped_ros2` repository. See [NOTICE.md](NOTICE.md).

The package code and the original presentation source are licensed under Apache-2.0. The deck uses reproducible diagrams and figures generated from this repository.
