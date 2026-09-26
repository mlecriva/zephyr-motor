# Copyright (c) 2026 Rtone
# Copyright (c) 2026 Mathis Lécrivain <lecrivain.mathis@gmail.com>
# SPDX-License-Identifier: Apache-2.0

zephyr_library()

zephyr_library_sources_ifdef(CONFIG_PWM_BRIDGE_STM32 pwm_bridge_stm32.c)
