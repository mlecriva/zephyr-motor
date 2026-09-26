/*
 * Copyright (c) 2026 Rtone
 * Copyright (c) 2026 Mathis Lécrivain <lecrivain.mathis@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_APPS_INCLUDE_DRIVERS_BLDC_ADC_H_
#define ZEPHYR_APPS_INCLUDE_DRIVERS_BLDC_ADC_H_

#include <errno.h>
#include <stdint.h>

#include <zephyr/device.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Number of channels sampled per triggered sequence. */
#define BLDC_ADC_CHANNEL_COUNT 3

/**
 * @brief Per-period sample handler, called in ISR context.
 *
 * @param dev       Device that produced the samples.
 * @param counts    Raw conversion results, in devicetree channel order.
 * @param user_data Opaque pointer passed at registration.
 */
typedef void (*bldc_adc_sample_cb_t)(const struct device *dev,
				     const uint16_t counts[BLDC_ADC_CHANNEL_COUNT],
				     void *user_data);

/**
 * @brief Driver-side vtable. Filled by each concrete bldc_adc driver.
 *
 * All three ops are mandatory.
 */
__subsystem struct bldc_adc_driver_api {
	int (*read)(const struct device *dev, uint16_t counts[BLDC_ADC_CHANNEL_COUNT]);
	int (*get_vref_mv)(const struct device *dev, uint32_t *vref_mv);
	int (*set_sample_handler)(const struct device *dev, bldc_adc_sample_cb_t cb,
				  void *user_data);
};

/**
 * @brief Wait for the next triggered sequence and return its results.
 *
 * Discards any sequence that completed earlier, so the caller always gets a
 * fresh conversion. Blocks by polling, meant for init-time calibration, not
 * for the steady-state path.
 *
 * @retval 0 on success.
 * @retval -EINVAL if @p dev or @p counts is NULL.
 * @retval -EBUSY if a sample handler is registered; it owns the sequence flag.
 * @retval -ETIMEDOUT if no sequence completed, i.e. the bridge is not
 *         triggering.
 */
static inline int bldc_adc_read(const struct device *dev, uint16_t counts[BLDC_ADC_CHANNEL_COUNT])
{
	if (dev == NULL || counts == NULL) {
		return -EINVAL;
	}

	return DEVICE_API_GET(bldc_adc, dev)->read(dev, counts);
}

/**
 * @brief Read the reference voltage the counts are scaled against.
 *
 * Measured once at init from the internal band-gap reference, so it tracks the
 * real VREF+ rather than a nominal value. Use it to turn counts into volts.
 *
 * @retval 0 on success.
 * @retval -EINVAL if @p dev or @p vref_mv is NULL.
 */
static inline int bldc_adc_get_vref_mv(const struct device *dev, uint32_t *vref_mv)
{
	if (dev == NULL || vref_mv == NULL) {
		return -EINVAL;
	}

	return DEVICE_API_GET(bldc_adc, dev)->get_vref_mv(dev, vref_mv);
}

/**
 * @brief Register the per-period sample handler and arm the interrupt.
 *
 * Pass @c NULL to disarm and hand the sequence flag back to bldc_adc_read().
 *
 * @retval 0 on success.
 * @retval -EINVAL if @p dev is NULL.
 */
static inline int bldc_adc_set_sample_handler(const struct device *dev, bldc_adc_sample_cb_t cb,
					      void *user_data)
{
	if (dev == NULL) {
		return -EINVAL;
	}

	return DEVICE_API_GET(bldc_adc, dev)->set_sample_handler(dev, cb, user_data);
}

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_APPS_INCLUDE_DRIVERS_BLDC_ADC_H_ */
