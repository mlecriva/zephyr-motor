# Copyright (c) 2026 Rtone
# Copyright (c) 2026 Mathis Lécrivain <lecrivain.mathis@gmail.com>
# SPDX-License-Identifier: Apache-2.0

zephyr_library()

zephyr_library_sources(motor_control.c)
zephyr_library_sources_ifdef(CONFIG_MOTOR_CONTROL_SHELL shell.c)
