# Copyright (c) 2026 Rtone
# Copyright (c) 2026 Mathis Lécrivain <lecrivain.mathis@gmail.com>
# SPDX-License-Identifier: Apache-2.0

zephyr_library()

zephyr_library_sources(bldc_6step_bridge_hall.c)
zephyr_library_sources(bldc_6step_motion.c)
zephyr_library_sources(bldc_6step_current.c)
