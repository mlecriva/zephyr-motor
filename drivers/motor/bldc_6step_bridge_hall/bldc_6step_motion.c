/*
 * Copyright (c) 2026 Rtone
 * Copyright (c) 2026 Mathis Lécrivain <lecrivain.mathis@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include "bldc_6step_internal.h"
#include "bldc_6step_motion.h"
#include <drivers/bldc_hall.h>

LOG_MODULE_DECLARE(bldc_6step_bridge_hall, CONFIG_MOTOR_LOG_LEVEL);

/** Milli-RPM per revolution per second: 60 s/min * 1000. */
#define BLDC_MILLI_RPM_PER_RPS 60000ULL

/** Micro-degrees per Hall step: 6 steps per electrical revolution. */
#define BLDC_MICRO_DEG_PER_STEP 60000000LL

/**
 * @brief Direction from a Hall state pair, indexed [prev][new].
 *
 * @note +1 is forward, -1 reverse, 0 invalid (sensor fault, missed edge, or state 0/7). Follows the
 * forward sequence of the commutation table.
 */
static const int8_t bldc_direction_table[BLDC_HALL_STATE_COUNT][BLDC_HALL_STATE_COUNT] = {
	[BLDC_HALL_STATE_5] = {[BLDC_HALL_STATE_4] = +1, [BLDC_HALL_STATE_1] = -1},
	[BLDC_HALL_STATE_4] = {[BLDC_HALL_STATE_6] = +1, [BLDC_HALL_STATE_5] = -1},
	[BLDC_HALL_STATE_6] = {[BLDC_HALL_STATE_2] = +1, [BLDC_HALL_STATE_4] = -1},
	[BLDC_HALL_STATE_2] = {[BLDC_HALL_STATE_3] = +1, [BLDC_HALL_STATE_6] = -1},
	[BLDC_HALL_STATE_3] = {[BLDC_HALL_STATE_1] = +1, [BLDC_HALL_STATE_2] = -1},
	[BLDC_HALL_STATE_1] = {[BLDC_HALL_STATE_5] = +1, [BLDC_HALL_STATE_3] = -1},
};

/**
 * @brief Convert a Hall interval to signed mechanical milli-RPM.
 *
 * @param dev motor device.
 * @param ticks Interval between two timed edges; 0 reads as stopped.
 * @param direction +1 or -1.
 *
 * @return Shaft speed in milli-RPM, saturated at INT32_MAX in magnitude.
 */
static int32_t bldc_ticks_to_milli_rpm(const struct device *dev, uint32_t ticks, int8_t direction)
{
	const struct bldc_6step_config *cfg = dev->config;
	const struct bldc_motion_data *m = &((const struct bldc_6step_data *)dev->data)->motion;
	uint64_t den = (uint64_t)ticks * m->edges_per_rev * cfg->motion.pole_pairs;
	uint64_t mag;

	if (den == 0U) {
		return 0;
	}

	/* A glitch can time a very short interval: saturate rather than wrap. */
	mag = MIN(BLDC_MILLI_RPM_PER_RPS * m->counter_freq_hz / den, (uint64_t)INT32_MAX);

	return (int32_t)mag * direction;
}

/**
 * @brief Convert a signed Hall step count to cumulative mechanical micro-degrees.
 *
 * @param dev motor device.
 * @param steps Hall steps since init.
 *
 * @return Shaft angle since init, in micro-degrees.
 */
static int64_t bldc_steps_to_micro_deg(const struct device *dev, int32_t steps)
{
	const struct bldc_6step_config *cfg = dev->config;

	/* Computed from the count every time, so the non-integer step never accumulates error. */
	return (int64_t)steps * BLDC_MICRO_DEG_PER_STEP / cfg->motion.pole_pairs;
}

void bldc_motion_on_edge(const struct device *dev, uint8_t state, uint32_t period_ticks)
{
	struct bldc_motion_data *m = &((struct bldc_6step_data *)dev->data)->motion;
	int8_t dir = 0;

	if (state < BLDC_HALL_STATE_COUNT) {
		dir = bldc_direction_table[m->prev_hall_state][state];
		m->prev_hall_state = state;
	}

	/* An invalid pair leaves the direction as it was: it must not read as a reversal. */
	if (dir != 0) {
		m->direction = dir;
		m->hall_steps += dir;
	}
	m->hall_period = period_ticks;
}

int bldc_motion_init(const struct device *dev)
{
	const struct bldc_6step_config *cfg = dev->config;
	struct bldc_motion_data *m = &((struct bldc_6step_data *)dev->data)->motion;
	uint8_t state;
	int ret;

	m->direction = 1;
	m->hall_steps = 0;
	m->hall_period = 0;

	ret = bldc_hall_get_counter_freq_hz(cfg->hall, &m->counter_freq_hz);
	if (ret < 0) {
		LOG_ERR("[%s] Failed to read the Hall counter rate: %d", dev->name, ret);
		return ret;
	}

	ret = bldc_hall_get_edges_per_rev(cfg->hall, &m->edges_per_rev);
	if (ret < 0) {
		LOG_ERR("[%s] Failed to read the Hall edge count: %d", dev->name, ret);
		return ret;
	}

	/* Seed the origin of the direction lookup, so the first edge does not read as a spurious
	 * direction. A failed read leaves 0, whose lookups are all invalid: harmless.
	 */
	m->prev_hall_state = 0;
	if (bldc_hall_get_state(cfg->hall, &state) == 0 && state < BLDC_HALL_STATE_COUNT) {
		m->prev_hall_state = state;
	}

	return 0;
}

int bldc_motion_get(const struct device *dev, struct motor_feedback *fb)
{
	const struct bldc_6step_config *cfg = dev->config;
	const struct bldc_motion_data *m = &((const struct bldc_6step_data *)dev->data)->motion;
	uint32_t elapsed;
	uint32_t period;
	int32_t steps;
	int8_t direction;
	int ret;

	unsigned int key = irq_lock();

	steps = m->hall_steps;
	period = m->hall_period;
	direction = m->direction;
	irq_unlock(key);

	/* No edge means no update: the time since the last one bounds the speed from above, so a
	 * stopping rotor reads slower and slower instead of frozen at its last interval.
	 */
	ret = bldc_hall_get_elapsed_ticks(cfg->hall, &elapsed);
	if (ret < 0) {
		return ret;
	}

	fb->micro_deg = bldc_steps_to_micro_deg(dev, steps);
	/* UINT32_MAX: no valid interval, the rotor is stalled. */
	fb->milli_rpm = (period == 0U || elapsed == UINT32_MAX)
				? 0
				: bldc_ticks_to_milli_rpm(dev, MAX(period, elapsed), direction);

	return 0;
}
