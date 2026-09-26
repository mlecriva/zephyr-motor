# Copyright (c) 2026 Rtone
# Copyright (c) 2026 Mathis Lécrivain <lecrivain.mathis@gmail.com>
# SPDX-License-Identifier: Apache-2.0

zephyr_library()

zephyr_library_sources_ifdef(CONFIG_BLDC_HALL_STM32 bldc_hall_xor_stm32.c)
