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

#include "bldc_6step_current.h"
#include "bldc_6step_internal.h"
#include <drivers/bldc_adc.h>

LOG_MODULE_DECLARE(bldc_6step_bridge_hall, CONFIG_MOTOR_LOG_LEVEL);

/** 12-bit conversions, which the bldc_adc class does not report: counts span [0, full scale). */
#define BLDC_6STEP_ADC_FULL_SCALE 4096U

/** Conversions averaged per phase for the boot zero-current calibration. */
#define BLDC_6STEP_ADC_CALIB_SAMPLES CONFIG_MOTOR_BLDC_6STEP_BRIDGE_HALL_ADC_CALIB_SAMPLES

BUILD_ASSERT(BLDC_ADC_CHANNEL_COUNT <= MOTOR_MAX_PHASES,
	     "a bldc_adc sample carries more channels than a motor feedback has phases");

/**
 * @brief Convert a signed ADC count deviation to signed milliamps.
 *
 * @param c Current state, holds the count -> mA ratio.
 * @param delta Deviation from the zero-current offset, in counts.
 *
 * @return Current in milliamps, saturated at INT32_MAX in magnitude.
 */
static inline int32_t bldc_delta_count_to_ma(const struct bldc_current_data *c, int32_t delta)
{
	uint32_t mag = (uint32_t)(delta < 0 ? -delta : delta);
	uint64_t ma = MIN((uint64_t)mag * c->lsb_ma_num / c->lsb_ma_den, (uint64_t)INT32_MAX);

	return (delta < 0) ? -(int32_t)ma : (int32_t)ma;
}

/**
 * @brief Per-period sample handler (ISR): keep the last raw counts.
 *
 * @note The three counts are stored under irq_lock(), so a reader preempting from a higher
 * priority never sees them half-updated.
 *
 * @param adc_dev bldc_adc device, unused.
 * @param counts Raw per-phase counts, U/V/W order.
 * @param user_data The motor device.
 */
static void bldc_on_adc_sample(const struct device *adc_dev,
			       const uint16_t counts[BLDC_ADC_CHANNEL_COUNT], void *user_data)
{
	const struct device *dev = user_data;
	struct bldc_current_data *c = &((struct bldc_6step_data *)dev->data)->current;

	ARG_UNUSED(adc_dev);

	unsigned int key = irq_lock();

	for (int i = 0; i < BLDC_ADC_CHANNEL_COUNT; i++) {
		c->counts[i] = counts[i];
	}
	irq_unlock(key);
}

int bldc_current_init(const struct device *dev)
{
	const struct bldc_6step_config *cfg = dev->config;
	struct bldc_current_data *c = &((struct bldc_6step_data *)dev->data)->current;
	uint32_t accum[BLDC_ADC_CHANNEL_COUNT] = {0};
	uint32_t vref_mv;
	int ret;

	ret = bldc_adc_get_vref_mv(cfg->adc, &vref_mv);
	if (ret < 0) {
		LOG_ERR("[%s] Failed to read VREF+: %d", dev->name, ret);
		return ret;
	}

	/* Count -> mA as an exact num/den ratio:
	 *   I[mA] = delta_count * (vref_mv * 1e9) / (full_scale * gain_milli * shunt_uohm)
	 */
	c->lsb_ma_num = (uint64_t)vref_mv * 1000ULL * 1000000ULL;
	c->lsb_ma_den = (uint64_t)BLDC_6STEP_ADC_FULL_SCALE * cfg->current.gain_milli *
			cfg->current.shunt_uohm;

	/* The bridge already runs its time base with the outputs disabled, so these sequences are
	 * taken at rest. A missing sample aborts: there is no fallback offset.
	 */
	for (uint32_t s = 0; s < BLDC_6STEP_ADC_CALIB_SAMPLES; s++) {
		uint16_t raw[BLDC_ADC_CHANNEL_COUNT];

		ret = bldc_adc_read(cfg->adc, raw);
		if (ret < 0) {
			LOG_ERR("[%s] Zero-current calibration failed after %u/%u samples: %d",
				dev->name, s, BLDC_6STEP_ADC_CALIB_SAMPLES, ret);
			return ret;
		}
		for (int i = 0; i < BLDC_ADC_CHANNEL_COUNT; i++) {
			accum[i] += raw[i];
		}
	}

	for (int i = 0; i < BLDC_ADC_CHANNEL_COUNT; i++) {
		c->zero_offset[i] = (uint16_t)(accum[i] / BLDC_6STEP_ADC_CALIB_SAMPLES);
		/* Seed the last sample at the zero-current point, so a read before the first one
		 * gives 0 mA.
		 */
		c->counts[i] = c->zero_offset[i];
	}

	/* Registering the handler is what arms the sampler interrupt, so it cannot race the
	 * calibration loop above for the same sequences.
	 */
	ret = bldc_adc_set_sample_handler(cfg->adc, bldc_on_adc_sample, (void *)dev);
	if (ret < 0) {
		LOG_ERR("[%s] Failed to register the sample handler: %d", dev->name, ret);
		return ret;
	}

	LOG_DBG("[%s] Current sense: VREF+=%u mV, offset {%u,%u,%u}", dev->name, vref_mv,
		c->zero_offset[0], c->zero_offset[1], c->zero_offset[2]);

	return 0;
}

void bldc_current_get(const struct device *dev, struct motor_feedback *fb)
{
	const struct bldc_current_data *c = &((const struct bldc_6step_data *)dev->data)->current;
	uint16_t counts[BLDC_ADC_CHANNEL_COUNT];

	unsigned int key = irq_lock();

	for (int i = 0; i < BLDC_ADC_CHANNEL_COUNT; i++) {
		counts[i] = c->counts[i];
	}
	irq_unlock(key);

	for (int i = 0; i < BLDC_ADC_CHANNEL_COUNT; i++) {
		fb->phase_milli_amp[i] =
			bldc_delta_count_to_ma(c, (int32_t)counts[i] - (int32_t)c->zero_offset[i]);
	}
	fb->phase_count = BLDC_ADC_CHANNEL_COUNT;
}
