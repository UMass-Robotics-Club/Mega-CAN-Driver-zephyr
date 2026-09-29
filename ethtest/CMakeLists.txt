# Copyright (c) 2026
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

# Custom board (../boards/microchip/mega_can) and the repo's Zephyr module
# (../module: the PIC32CZ Ethernet driver), without west manifest changes.
list(APPEND BOARD_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/..)
list(APPEND ZEPHYR_EXTRA_MODULES ${CMAKE_CURRENT_SOURCE_DIR}/../module)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(mega_can_ethtest)

target_sources(app PRIVATE src/main.c)
