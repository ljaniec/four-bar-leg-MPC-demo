"""Command-line interface for the four-bar leg MPC demo."""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np

from .visualization import generate_outputs


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="four-bar-leg-mpc-demo",
        description=(
            "Run the kinematic MPC demonstration for one four-bar quadruped leg "
            "and generate figures."
        ),
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path("artifacts"),
        help="Directory for generated figures (default: artifacts).",
    )
    parser.add_argument(
        "--steps",
        type=int,
        default=35,
        help="Number of closed-loop MPC steps (default: 35).",
    )
    parser.add_argument(
        "--language",
        choices=("en", "pl"),
        default="en",
        help="Language used in generated figures (default: en).",
    )
    parser.add_argument(
        "--no-animation",
        action="store_true",
        help="Skip GIF generation.",
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    if args.steps <= 0:
        raise SystemExit("--steps must be positive.")

    result = generate_outputs(
        output_dir=args.output_dir,
        language=args.language,
        steps=args.steps,
        include_animation=not args.no_animation,
    )
    final_error = float(np.linalg.norm(result.foot[-1] - result.target))
    print(f"Outputs written to: {args.output_dir.resolve()}")
    print(f"Final foot-position error: {final_error:.6f} m")
    print(f"Final joint configuration: {result.q[-1]}")
    return 0
