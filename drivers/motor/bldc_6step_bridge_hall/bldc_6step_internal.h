/*
 * Copyright (c) 2026 Rtone
 * Copyright (c) 2026 Mathis Lécrivain <lecrivain.mathis@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_APPS_DRIVERS_MOTOR_BLDC_6STEP_BRIDGE_HALL_INTERNAL_H_
#define ZEPHYR_APPS_DRIVERS_MOTOR_BLDC_6STEP_BRIDGE_HALL_INTERNAL_H_

#include <stdbool.h>
#include <stdint.h>

#include <zephyr/device.h>
#include <zephyr/dsp/types.h>

#include "bldc_6step_current.h"
#include "bldc_6step_motion.h"

/**
 * @brief 3-bit Hall state codes, (U << 2) | (V << 1) | W.
 *
 * The six valid ones form the forward commutation sequence
 * 5 -> 4 -> 6 -> 2 -> 3 -> 1 -> 5.
 */
enum bldc_hall_state {
	/** All lines low, physically impossible: a sensor fault. */
	BLDC_HALL_STATE_FAULT_LOW = 0,
	BLDC_HALL_STATE_1 = 1,
	BLDC_HALL_STATE_2 = 2,
	BLDC_HALL_STATE_3 = 3,
	BLDC_HALL_STATE_4 = 4,
	BLDC_HALL_STATE_5 = 5,
	BLDC_HALL_STATE_6 = 6,
	/** All lines high, physically impossible: a sensor fault. */
	BLDC_HALL_STATE_FAULT_HIGH = 7,
	/** Codes a 3-bit state can take, faults included. */
	BLDC_HALL_STATE_COUNT = 8,
};

/**
 * @brief Per-instance device tree configuration.
 *
 * The devices below own the hardware; none knows about the motor.
 */
struct bldc_6step_config {
	const struct device *pwm;  /**< pwm_bridge, takes leg patterns. */
	const struct device *hall; /**< bldc_hall, raw edges: 3-bit state + interval. */
	const struct device *adc;  /**< bldc_adc, raw phase-current counts; NULL when unset. */

	struct bldc_motion_config motion;   /**< Hall edges -> direction, angle, speed. */
	struct bldc_current_config current; /**< Raw counts -> phase currents. */
};

/**
 * @brief Runtime state.
 *
 * running and duty are commanded by threads under irq_lock(); hall_state is
 * written by the Hall ISR and read by set_voltage() under the same lock. The
 * motion is written by the Hall ISR and the current by the sampler ISR;
 * bldc_6step_get_feedback() copies each under irq_lock() so its fields stay
 * consistent with each other.
 */
struct bldc_6step_data {
	bool running;              /**< Bridge driven, toggled by start/stop. */
	q31_t duty;                /**< Q31 -1.0..1.0, sign = commanded direction. */
	uint8_t hall_state;        /**< Last seen, what a re-apply commutates from. */
	uint32_t max_pulse_cycles; /**< 100 % pulse width, cached from the bridge. */

	struct bldc_motion_data motion;   /**< Direction, angle and speed, raw. */
	struct bldc_current_data current; /**< Last phase-current sample, raw. */
};

#endif /* ZEPHYR_APPS_DRIVERS_MOTOR_BLDC_6STEP_BRIDGE_HALL_INTERNAL_H_ */
