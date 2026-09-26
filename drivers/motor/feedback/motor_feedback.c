/*
 * Copyright (c) 2026 Rtone
 * Copyright (c) 2026 Mathis Lécrivain <lecrivain.mathis@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT zephyr_motor_feedback

#include <errno.h>

#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>

#include <drivers/motor.h>
#include <drivers/sensor/motor_feedback.h>

LOG_MODULE_REGISTER(motor_feedback, CONFIG_SENSOR_LOG_LEVEL);

BUILD_ASSERT(SENSOR_CHAN_MOTOR_PHASE_CURRENT_W - SENSOR_CHAN_MOTOR_PHASE_CURRENT_U + 1 ==
		     MOTOR_MAX_PHASES,
	     "one phase-current channel per phase a motor feedback carries");

/**
 * @brief Per-instance device tree configuration.
 */
struct motor_feedback_config {
	const struct device *motor; /**< Parent motor. */
};

/**
 * @brief Runtime state.
 */
struct motor_feedback_data {
	struct motor_feedback latched; /**< sample_fetch() -> channel_get(). */
};

/**
 * @brief Poll the parent motor and latch its feedback.
 *
 * @note Every supported channel latches every value, from the same read.
 *
 * @param dev motor_feedback device.
 * @param chan SENSOR_CHAN_ALL, SENSOR_CHAN_RPM, SENSOR_CHAN_ROTATION or a phase-current channel.
 *
 * @retval 0 on success.
 * @retval -ENOTSUP if @p chan is none of the above.
 * @retval -errno Other negative errno code from the motor.
 */
static int motor_feedback_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
	const struct motor_feedback_config *cfg = dev->config;
	struct motor_feedback_data *data = dev->data;

	/* int: the phase-current channels are private, outside enum sensor_channel. */
	switch ((int)chan) {
	case SENSOR_CHAN_ALL:
	case SENSOR_CHAN_RPM:
	case SENSOR_CHAN_ROTATION:
	case SENSOR_CHAN_MOTOR_PHASE_CURRENT_U:
	case SENSOR_CHAN_MOTOR_PHASE_CURRENT_V:
	case SENSOR_CHAN_MOTOR_PHASE_CURRENT_W:
		return motor_get_feedback(cfg->motor, &data->latched);
	default:
		return -ENOTSUP;
	}
}

/**
 * @brief Return one latched value as a sensor value.
 *
 * @param dev motor_feedback device.
 * @param chan SENSOR_CHAN_RPM, SENSOR_CHAN_ROTATION or a phase-current channel.
 * @param val Output, filled on success.
 *
 * @retval 0 on success.
 * @retval -EINVAL if @p val is NULL.
 * @retval -ENOTSUP if @p chan is none of the above, or a phase the motor does not sense.
 */
static int motor_feedback_channel_get(const struct device *dev, enum sensor_channel chan,
				      struct sensor_value *val)
{
	const struct motor_feedback *fb = &((const struct motor_feedback_data *)dev->data)->latched;
	int phase;

	if (val == NULL) {
		return -EINVAL;
	}

	/* int: the phase-current channels are private, outside enum sensor_channel. */
	switch ((int)chan) {
	case SENSOR_CHAN_RPM:
		/* val1 = RPM, val2 = µRPM, same sign. */
		val->val1 = fb->milli_rpm / 1000;
		val->val2 = (fb->milli_rpm % 1000) * 1000;
		return 0;
	case SENSOR_CHAN_ROTATION:
		/* val1 = degrees, val2 = µdeg, same sign. val1 wraps at int32_t. */
		val->val1 = (int32_t)(fb->micro_deg / 1000000);
		val->val2 = (int32_t)(fb->micro_deg % 1000000);
		return 0;
	case SENSOR_CHAN_MOTOR_PHASE_CURRENT_U:
	case SENSOR_CHAN_MOTOR_PHASE_CURRENT_V:
	case SENSOR_CHAN_MOTOR_PHASE_CURRENT_W:
		phase = (int)chan - SENSOR_CHAN_MOTOR_PHASE_CURRENT_U;
		if (phase >= fb->phase_count) {
			return -ENOTSUP;
		}
		/* val1 = A, val2 = µA, same sign. */
		val->val1 = fb->phase_milli_amp[phase] / 1000;
		val->val2 = (fb->phase_milli_amp[phase] % 1000) * 1000;
		return 0;
	default:
		return -ENOTSUP;
	}
}

static DEVICE_API(sensor, motor_feedback_api) = {
	.sample_fetch = motor_feedback_sample_fetch,
	.channel_get = motor_feedback_channel_get,
};

/**
 * @brief Check that the parent motor is ready and reports feedback.
 *
 * @param dev motor_feedback device.
 *
 * @retval 0 on success.
 * @retval -ENODEV if the motor is not ready.
 * @retval -ENOTSUP if the motor reports no feedback.
 * @retval -errno Other negative errno code from the motor.
 */
static int motor_feedback_init(const struct device *dev)
{
	const struct motor_feedback_config *cfg = dev->config;
	struct motor_feedback_data *data = dev->data;
	int ret;

	if (!device_is_ready(cfg->motor)) {
		LOG_ERR("[%s] Motor %s not ready", dev->name, cfg->motor->name);
		return -ENODEV;
	}

	/* A motor that measures nothing has no feedback to serve. */
	ret = motor_get_feedback(cfg->motor, &data->latched);
	if (ret == -ENOSYS) {
		LOG_ERR("[%s] Motor %s reports no feedback", dev->name, cfg->motor->name);
		return -ENOTSUP;
	}
	if (ret < 0) {
		LOG_ERR("[%s] Failed to read the feedback of %s: %d", dev->name, cfg->motor->name,
			ret);
		return ret;
	}

	return 0;
}

/** Instantiate one motor_feedback sensor from DT instance @p inst. */
#define MOTOR_FEEDBACK_INST(inst)                                                                  \
	BUILD_ASSERT(DT_NODE_HAS_COMPAT(DT_INST_PARENT(inst), zephyr_motor),                       \
		     "instance " #inst ": parent must be a \"zephyr,motor\" node");                \
                                                                                                   \
	static const struct motor_feedback_config motor_feedback_cfg_##inst = {                    \
		.motor = DEVICE_DT_GET(DT_INST_PARENT(inst)),                                      \
	};                                                                                         \
                                                                                                   \
	static struct motor_feedback_data motor_feedback_data_##inst;                              \
                                                                                                   \
	SENSOR_DEVICE_DT_INST_DEFINE(inst, motor_feedback_init, NULL, &motor_feedback_data_##inst, \
				     &motor_feedback_cfg_##inst, POST_KERNEL,                      \
				     CONFIG_SENSOR_INIT_PRIORITY, &motor_feedback_api);

DT_INST_FOREACH_STATUS_OKAY(MOTOR_FEEDBACK_INST)
