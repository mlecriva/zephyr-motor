/*
 * Copyright (c) 2026 Rtone
 * Copyright (c) 2026 Mathis Lécrivain <lecrivain.mathis@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_APPS_DRIVERS_MOTOR_BLDC_6STEP_BRIDGE_HALL_CURRENT_H_
#define ZEPHYR_APPS_DRIVERS_MOTOR_BLDC_6STEP_BRIDGE_HALL_CURRENT_H_

#include <stdint.h>

#include <zephyr/device.h>

#include <drivers/bldc_adc.h>
#include <drivers/motor.h>

/**
 * @brief Per-instance device tree configuration of the current sensing.
 *
 * @note The analog front-end only. VREF+ is measured at boot by the sampler, not a DT value.
 */
struct bldc_current_config {
	uint32_t shunt_uohm; /**< Shunt value, one half of the count -> mA scale. */
	uint32_t gain_milli; /**< Amplifier gain, the other half. */
};

/**
 * @brief Phase-current state, written by the sampler ISR.
 *
 * @note Raw: the last sample only, nothing is averaged or filtered.
 */
struct bldc_current_data {
	uint16_t counts[BLDC_ADC_CHANNEL_COUNT];      /**< Last sample, U/V/W order. */
	uint16_t zero_offset[BLDC_ADC_CHANNEL_COUNT]; /**< Zero-current count, from boot calib. */
	uint64_t lsb_ma_num;                          /**< Count -> mA scale, numerator. */
	uint64_t lsb_ma_den;                          /**< Count -> mA scale, denominator. */
};

/**
 * @brief Compute the count -> mA scale, calibrate the zero-current offsets, arm the sampler.
 *
 * @note Only called when an adc is bound. Runs before the sample handler is registered, which
 * would refuse the calibration reads.
 *
 * @param dev motor device.
 *
 * @return 0 on success, negative errno otherwise.
 */
int bldc_current_init(const struct device *dev);

/**
 * @brief Fill the phase currents of a feedback, from the last sample.
 *
 * @note Copies the ISR state under irq_lock(), then converts it.
 *
 * @param dev motor device.
 * @param fb Output; only phase_milli_amp and phase_count are written.
 */
void bldc_current_get(const struct device *dev, struct motor_feedback *fb);

#endif /* ZEPHYR_APPS_DRIVERS_MOTOR_BLDC_6STEP_BRIDGE_HALL_CURRENT_H_ */
