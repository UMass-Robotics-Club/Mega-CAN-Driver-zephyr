#!/usr/bin/env bash
#
# Build the mega_can blinky and program it. Then POWER-CYCLE the board to run
# (the factory boot ROM halts while the probe is attached -- see README).
# Any extra args are passed to `west build` (e.g. -p always).
#
set -euo pipefail

cd "$HOME/mega-can-fw"
# shellcheck disable=SC1091
source .venv/bin/activate
export PATH="$HOME/.local/bin:/opt/homebrew/bin:$PATH"
export ZEPHYR_BASE="$HOME/mega-can-fw/zephyr"

west build -b mega_can app "$@"
./flash.py
