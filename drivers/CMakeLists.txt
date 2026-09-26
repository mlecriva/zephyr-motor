# Copyright (c) 2026 Rtone
# Copyright (c) 2026 Mathis Lécrivain <lecrivain.mathis@gmail.com>
# SPDX-License-Identifier: Apache-2.0

add_subdirectory_ifdef(CONFIG_BLDC_ADC bldc_adc)
add_subdirectory_ifdef(CONFIG_BLDC_HALL bldc_hall)
add_subdirectory_ifdef(CONFIG_MOTOR motor)
add_subdirectory_ifdef(CONFIG_PWM_BRIDGE pwm_bridge)
