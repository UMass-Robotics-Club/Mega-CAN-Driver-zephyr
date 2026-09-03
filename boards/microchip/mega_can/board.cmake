# Copyright (c) 2026
# SPDX-License-Identifier: Apache-2.0

# Flash/debug with pyOCD using the Microchip DFP pack for this SoC.
# The --pack is passed as a tool-opt so it applies to flash, gdbserver and debug.
board_runner_args(pyocd "--target=pic32cz8110ca80208")
board_runner_args(pyocd "--tool-opt=--pack=/Users/asritha/Downloads/Microchip.PIC32CZ-CA80_DFP.1.7.219.pack")

include(${ZEPHYR_BASE}/boards/common/pyocd.board.cmake)
