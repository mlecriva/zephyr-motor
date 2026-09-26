/*
 * Copyright (c) 2026 Rtone
 * Copyright (c) 2026 Mathis Lécrivain <lecrivain.mathis@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_APPS_INCLUDE_DRIVERS_BLDC_HALL_H_
#define ZEPHYR_APPS_INCLUDE_DRIVERS_BLDC_HALL_H_

#include <errno.h>
#include <stdint.h>

#include <zephyr/device.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Number of Hall lines (U, V, W). */
#define BLDC_HALL_LINE_COUNT 3

/**
 * @brief Per-edge handler, called in ISR context.
 *
 * @param dev          Device that detected the edge.
 * @param state        3-bit U/V/W state read at the edge.
 * @param period_ticks Ticks since the previous timed edge, or 0 if this edge carries no usable
 *                     interval.
 * @param user_data    Opaque pointer passed at registration.
 */
typedef void (*bldc_hall_edge_cb_t)(const struct device *dev, uint8_t state, uint32_t period_ticks,
				    void *user_data);

/**
 * @brief Driver-side vtable. Filled by each concrete bldc_hall driver.
 *
 * All ops are mandatory.
 */
__subsystem struct bldc_hall_driver_api {
	int (*get_state)(const struct device *dev, uint8_t *state);
	int (*get_counter_freq_hz)(const struct device *dev, uint32_t *freq_hz);
	int (*get_elapsed_ticks)(const struct device *dev, uint32_t *ticks);
	int (*get_edges_per_rev)(const struct device *dev, uint8_t *edges_per_rev);
	int (*set_edge_handler)(const struct device *dev, bldc_hall_edge_cb_t cb, void *user_data);
};

/**
 * @brief Read the instantaneous 3-bit U/V/W state.
 *
 * @retval 0 on success.
 * @retval -EINVAL if @p dev or @p state is NULL.
 * @retval -EIO if a line could not be read.
 */
static inline int bldc_hall_get_state(const struct device *dev, uint8_t *state)
{
	if (dev == NULL || state == NULL) {
		return -EINVAL;
	}

	return DEVICE_API_GET(bldc_hall, dev)->get_state(dev, state);
}

/**
 * @brief Read the counter frequency periods and elapsed ticks are counted at.
 *
 * @retval 0 on success.
 * @retval -EINVAL if @p dev or @p freq_hz is NULL.
 */
static inline int bldc_hall_get_counter_freq_hz(const struct device *dev, uint32_t *freq_hz)
{
	if (dev == NULL || freq_hz == NULL) {
		return -EINVAL;
	}

	return DEVICE_API_GET(bldc_hall, dev)->get_counter_freq_hz(dev, freq_hz);
}

/**
 * @brief Read the ticks elapsed since the last timed edge.
 *
 * Reads UINT32_MAX when no valid interval exists: before the first timed edge, or once the
 * counter wrapped with no edge.
 *
 * @retval 0 on success.
 * @retval -EINVAL if @p dev or @p ticks is NULL.
 */
static inline int bldc_hall_get_elapsed_ticks(const struct device *dev, uint32_t *ticks)
{
	if (dev == NULL || ticks == NULL) {
		return -EINVAL;
	}

	return DEVICE_API_GET(bldc_hall, dev)->get_elapsed_ticks(dev, ticks);
}

/**
 * @brief Read how many timed edges span one electrical revolution.
 *
 * @retval 0 on success.
 * @retval -EINVAL if @p dev or @p edges_per_rev is NULL.
 */
static inline int bldc_hall_get_edges_per_rev(const struct device *dev, uint8_t *edges_per_rev)
{
	if (dev == NULL || edges_per_rev == NULL) {
		return -EINVAL;
	}

	return DEVICE_API_GET(bldc_hall, dev)->get_edges_per_rev(dev, edges_per_rev);
}

/**
 * @brief Register the per-edge handler and arm the interrupt.
 *
 * @note Pass @c NULL to disarm.
 *
 * @retval 0 on success.
 * @retval -EINVAL if @p dev is NULL.
 */
static inline int bldc_hall_set_edge_handler(const struct device *dev, bldc_hall_edge_cb_t cb,
					     void *user_data)
{
	if (dev == NULL) {
		return -EINVAL;
	}

	return DEVICE_API_GET(bldc_hall, dev)->set_edge_handler(dev, cb, user_data);
}

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_APPS_INCLUDE_DRIVERS_BLDC_HALL_H_ */
