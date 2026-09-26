# Copyright (c) 2026 Rtone
# Copyright (c) 2026 Mathis Lécrivain <lecrivain.mathis@gmail.com>
# SPDX-License-Identifier: Apache-2.0

zephyr_library()

zephyr_library_sources_ifdef(CONFIG_MOTOR_SHELL motor_shell.c)

add_subdirectory_ifdef(CONFIG_MOTOR_BLDC_6STEP_BRIDGE_HALL bldc_6step_bridge_hall)
add_subdirectory_ifdef(CONFIG_MOTOR_FEEDBACK feedback)
