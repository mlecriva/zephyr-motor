/*
 * Copyright (c) 2026 Rtone
 * Copyright (c) 2026 Mathis Lécrivain <lecrivain.mathis@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_APPS_INCLUDE_DRIVERS_MOTOR_H_
#define ZEPHYR_APPS_INCLUDE_DRIVERS_MOTOR_H_

#include <errno.h>
#include <stdint.h>

#include <zephyr/device.h>
#include <zephyr/dsp/types.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Most phase currents a feedback carries, one per phase of a three-phase motor. */
#define MOTOR_MAX_PHASES 3

/**
 * @brief Raw measurements of a motor, on the shaft and in its phases.
 *
 * Nothing is averaged or filtered, since the right filter depends on the rate
 * of the loop that consumes it. The angle and the speed are mechanical: an
 * implementation measuring electrical quantities divides them by the rotor
 * pole pairs. The phase currents are the last sample, in phase order.
 */
struct motor_feedback {
	/** Instantaneous shaft speed in milli-RPM, signed by the measured direction. */
	int32_t milli_rpm;
	/** Shaft angle traveled since init in micro-degrees, cumulative and signed. */
	int64_t micro_deg;
	/** Phase currents in milliamps, signed; only the first phase_count are valid. */
	int32_t phase_milli_amp[MOTOR_MAX_PHASES];
	/** Phase currents measured, 0 when the motor senses no current. */
	uint8_t phase_count;
};

/**
 * @brief Driver-side vtable. Filled by each concrete motor driver.
 *
 * start, stop and set_voltage are mandatory. get_feedback is optional: a
 * driver that measures nothing leaves it NULL.
 */
__subsystem struct motor_driver_api {
	/** @see motor_start() */
	int (*start)(const struct device *dev);
	/** @see motor_stop() */
	int (*stop)(const struct device *dev);
	/** @see motor_set_voltage() */
	int (*set_voltage)(const struct device *dev, q31_t voltage);
	/** @see motor_get_feedback() */
	int (*get_feedback)(const struct device *dev, struct motor_feedback *fb);
};

/**
 * @brief Start the motor (enable the output stage).
 *
 * The voltage last set applies; it may be set before starting.
 *
 * @param dev motor device.
 *
 * @retval 0 on success.
 * @retval -EINVAL if @p dev is NULL.
 * @retval -errno Other negative errno code from the driver.
 */
static inline int motor_start(const struct device *dev)
{
	if (dev == NULL) {
		return -EINVAL;
	}

	return DEVICE_API_GET(motor, dev)->start(dev);
}

/**
 * @brief Stop the motor (disable the output stage and release every phase).
 *
 * This is the only call guaranteed to leave the rotor free.
 *
 * @param dev motor device.
 *
 * @retval 0 on success.
 * @retval -EINVAL if @p dev is NULL.
 * @retval -errno Other negative errno code from the driver.
 */
static inline int motor_stop(const struct device *dev)
{
	if (dev == NULL) {
		return -EINVAL;
	}

	return DEVICE_API_GET(motor, dev)->stop(dev);
}

/**
 * @brief Apply a signed voltage.
 *
 * The voltage is a Q31 fraction of the available supply, -1.0..1.0: the
 * magnitude is the ratio applied along the torque-producing axis and the sign
 * encodes the rotation direction. A 6-step implementation applies it as the
 * duty of the chopped leg; a field-oriented one as the q-axis voltage, with no
 * d-axis voltage.
 *
 * A zero voltage is not necessarily an idle: on a bridge driven as
 * complementary pairs it applies a null voltage vector, which shorts the
 * winding and brakes the rotor. Use motor_stop() to release the motor.
 *
 * @param dev motor device.
 * @param voltage Signed voltage in Q31. INT32_MIN (-1.0 exactly) is out of
 *        range, so both directions share the same full scale, +/-INT32_MAX.
 *
 * @retval 0 on success.
 * @retval -EINVAL if @p dev is NULL or @p voltage is INT32_MIN.
 * @retval -errno Other negative errno code from the driver.
 */
static inline int motor_set_voltage(const struct device *dev, q31_t voltage)
{
	if (dev == NULL || voltage == INT32_MIN) {
		return -EINVAL;
	}

	return DEVICE_API_GET(motor, dev)->set_voltage(dev, voltage);
}

/**
 * @brief Read the latest feedback.
 *
 * The speed of a stopping rotor decays toward 0 instead of freezing at its
 * last value. The phase currents are filled only when the motor senses them,
 * see @ref motor_feedback::phase_count. Does not block, so it may be called
 * from any context, ISR included.
 *
 * @param dev motor device.
 * @param fb Output, filled on success.
 *
 * @retval 0 on success.
 * @retval -EINVAL if @p dev or @p fb is NULL.
 * @retval -ENOSYS if the driver reports no feedback.
 * @retval -errno Other negative errno code from the driver.
 */
static inline int motor_get_feedback(const struct device *dev, struct motor_feedback *fb)
{
	if (dev == NULL || fb == NULL) {
		return -EINVAL;
	}

	const struct motor_driver_api *api = DEVICE_API_GET(motor, dev);

	if (api->get_feedback == NULL) {
		return -ENOSYS;
	}

	return api->get_feedback(dev, fb);
}

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_APPS_INCLUDE_DRIVERS_MOTOR_H_ */
