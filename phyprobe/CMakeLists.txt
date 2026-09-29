# Copyright (c) 2026
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

# Make the custom board (../boards/microchip/mega_can) discoverable so that
# `west build -b mega_can phyprobe` works without passing -DBOARD_ROOT.
list(APPEND BOARD_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/..)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(mega_can_phyprobe)

target_sources(app PRIVATE src/main.c)
