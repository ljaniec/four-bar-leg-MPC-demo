#!/usr/bin/env bash
set -euo pipefail

OUTPUT_DIR="${1:-artifacts}"
python -m four_bar_leg_mpc --output-dir "$OUTPUT_DIR"
