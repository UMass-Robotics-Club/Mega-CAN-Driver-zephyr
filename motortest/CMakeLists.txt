# Copyright (c) 2026
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

list(APPEND BOARD_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/..)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(mega_can_motortest)

# robstride.c is shared with the real firmware in ../app
target_include_directories(app PRIVATE ../app/src)
target_sources(app PRIVATE src/main.c ../app/src/robstride.c)
