/*
 * Copyright (c) 2026 Rtone
 * Copyright (c) 2026 Mathis Lécrivain <lecrivain.mathis@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_APPS_DRIVERS_MOTOR_BLDC_6STEP_BRIDGE_HALL_MOTION_H_
#define ZEPHYR_APPS_DRIVERS_MOTOR_BLDC_6STEP_BRIDGE_HALL_MOTION_H_

#include <stdint.h>

#include <zephyr/device.h>

#include <drivers/motor.h>

/**
 * @brief Per-instance device tree configuration of the motion.
 */
struct bldc_motion_config {
	uint16_t pole_pairs; /**< motor-pole-pairs, electrical to mechanical. */
};

/**
 * @brief Direction, angle and speed state, written by the Hall ISR.
 *
 * @note Raw: nothing is averaged or filtered, the consumer does it at its own rate.
 */
struct bldc_motion_data {
	uint8_t prev_hall_state;  /**< Origin of the direction lookup. */
	int8_t direction;         /**< +1 or -1, measured from the Hall state pair. */
	int32_t hall_steps;       /**< Signed, 60 electrical degrees each, since init. */
	uint32_t hall_period;     /**< Ticks between the last two timed edges, 0 if none. */
	uint32_t counter_freq_hz; /**< Hall counter tick rate, for ticks -> RPM. */
	uint8_t edges_per_rev;    /**< Timed Hall edges per electrical revolution. */
};

/**
 * @brief Arm the motion: Hall constants and direction seed.
 *
 * @note Runs before the Hall edge handler is registered.
 *
 * @param dev motor device.
 *
 * @return 0 on success, negative errno otherwise.
 */
int bldc_motion_init(const struct device *dev);

/**
 * @brief Fold a Hall edge into direction, angle and speed (ISR).
 *
 * @param dev motor device.
 * @param state Hall state at the edge.
 * @param period_ticks Ticks since the previous timed edge, 0 when untimed.
 */
void bldc_motion_on_edge(const struct device *dev, uint8_t state, uint32_t period_ticks);

/**
 * @brief Fill the shaft speed and angle of a feedback, from the Hall edges.
 *
 * @note Copies the ISR state under irq_lock(), then converts it.
 *
 * @param dev motor device.
 * @param fb Output; only milli_rpm and micro_deg are written.
 *
 * @retval 0 on success.
 * @retval -errno Other negative errno code from the Hall device.
 */
int bldc_motion_get(const struct device *dev, struct motor_feedback *fb);

#endif /* ZEPHYR_APPS_DRIVERS_MOTOR_BLDC_6STEP_BRIDGE_HALL_MOTION_H_ */
