/*
 * Copyright (c) 2026 Rtone
 * Copyright (c) 2026 Mathis Lécrivain <lecrivain.mathis@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_APPS_INCLUDE_DRIVERS_SENSOR_MOTOR_FEEDBACK_H_
#define ZEPHYR_APPS_INCLUDE_DRIVERS_SENSOR_MOTOR_FEEDBACK_H_

#include <zephyr/drivers/sensor.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Private channels of the zephyr,motor-feedback sensor.
 *
 * One per phase current, in phase order. Each is served only when the motor
 * senses that phase; reading it otherwise fails with -ENOTSUP.
 */
enum sensor_channel_motor_feedback {
	/** Current of the first phase (U), in amps, signed. */
	SENSOR_CHAN_MOTOR_PHASE_CURRENT_U = SENSOR_CHAN_PRIV_START,
	/** Current of the second phase (V), in amps, signed. */
	SENSOR_CHAN_MOTOR_PHASE_CURRENT_V,
	/** Current of the third phase (W), in amps, signed. */
	SENSOR_CHAN_MOTOR_PHASE_CURRENT_W,
};

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_APPS_INCLUDE_DRIVERS_SENSOR_MOTOR_FEEDBACK_H_ */
