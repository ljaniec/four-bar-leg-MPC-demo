#!/usr/bin/env bash
set -euo pipefail

python -m ruff check src tests
python -m pytest
python -m build
