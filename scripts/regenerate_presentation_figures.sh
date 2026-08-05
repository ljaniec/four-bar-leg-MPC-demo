#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT_DIR"

PYTHONPATH=src python -m four_bar_leg_mpc \
  --output-dir presentation/figures \
  --language pl \
  --no-animation
