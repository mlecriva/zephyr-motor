/*
 * Copyright (c) 2026 Rtone
 * Copyright (c) 2026 Mathis Lécrivain <lecrivain.mathis@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT zephyr_bldc_6step_bridge_hall

#include <errno.h>

#include <zephyr/device.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include "bldc_6step_current.h"
#include "bldc_6step_internal.h"
#include "bldc_6step_motion.h"
#include <drivers/bldc_hall.h>
#include <drivers/motor.h>
#include <drivers/pwm_bridge.h>

LOG_MODULE_REGISTER(bldc_6step_bridge_hall, CONFIG_MOTOR_LOG_LEVEL);

/** Number of bridge legs this driver commutates (phases U, V, W). */
#define BLDC_6STEP_LEG_COUNT 3U

/** Leg states the commutation table uses; the bridge must support all three. */
#define BLDC_6STEP_LEG_STATES                                                                      \
	(PWM_BRIDGE_STATE_BIT(PWM_BRIDGE_LEG_HI_Z) | PWM_BRIDGE_STATE_BIT(PWM_BRIDGE_LEG_PWM) |    \
	 PWM_BRIDGE_STATE_BIT(PWM_BRIDGE_LEG_FORCE_LOW))

#define HIGH PWM_BRIDGE_LEG_PWM       /**< Chopped at the commanded duty. */
#define LOW  PWM_BRIDGE_LEG_FORCE_LOW /**< Low side conducting the whole period. */
#define OFF  PWM_BRIDGE_LEG_HI_Z      /**< Released. */

/**
 * @brief Forward commutation table: leg state of U, V, W indexed by the Hall state.
 *
 * @note Both active legs are driven as complementary pairs: the chopped leg gets synchronous
 * rectification, and the grounded one keeps its pair enabled so the dead-time generator is already
 * in the path at the next commutation. States 0 and 7 are impossible and force high-Z. Reverse
 * swaps HIGH <-> LOW.
 */
static const pwm_bridge_leg_state_t
	bldc_commutation_table_fwd[BLDC_HALL_STATE_COUNT][BLDC_6STEP_LEG_COUNT] = {
		[BLDC_HALL_STATE_FAULT_LOW] = {OFF, OFF, OFF},
		[BLDC_HALL_STATE_5] = {HIGH, LOW, OFF},
		[BLDC_HALL_STATE_4] = {HIGH, OFF, LOW},
		[BLDC_HALL_STATE_6] = {OFF, HIGH, LOW},
		[BLDC_HALL_STATE_2] = {LOW, HIGH, OFF},
		[BLDC_HALL_STATE_3] = {LOW, OFF, HIGH},
		[BLDC_HALL_STATE_1] = {OFF, LOW, HIGH},
		[BLDC_HALL_STATE_FAULT_HIGH] = {OFF, OFF, OFF},
};

/**
 * @brief Swap the chopped and grounded roles of a leg, which reverses the rotation.
 *
 * @param s Leg state from the forward table.
 *
 * @return The leg state for the reverse direction; a released leg stays released.
 */
static inline pwm_bridge_leg_state_t bldc_leg_invert(pwm_bridge_leg_state_t s)
{
	switch (s) {
	case PWM_BRIDGE_LEG_PWM:
		return PWM_BRIDGE_LEG_FORCE_LOW;
	case PWM_BRIDGE_LEG_FORCE_LOW:
		return PWM_BRIDGE_LEG_PWM;
	default:
		return s;
	}
}

/**
 * @brief Push the pattern for @p hall_state, at the current duty, to the bridge.
 *
 * @note Also called from the Hall edge ISR.
 *
 * @param dev motor device.
 * @param hall_state Commutation index; fault codes force high-Z.
 *
 * @return 0 on success, negative errno otherwise.
 */
static int bldc_apply_pattern(const struct device *dev, uint8_t hall_state)
{
	const struct bldc_6step_config *cfg = dev->config;
	const struct bldc_6step_data *data = dev->data;
	/* Zero-initialized: all legs high-Z, no duty — the fault pattern. */
	pwm_bridge_leg_t legs[BLDC_6STEP_LEG_COUNT] = {0};

	if (hall_state == BLDC_HALL_STATE_FAULT_LOW || hall_state == BLDC_HALL_STATE_FAULT_HIGH) {
		return pwm_bridge_set_pattern(cfg->pwm, legs, BLDC_6STEP_LEG_COUNT);
	}

	/* INT32_MIN is rejected by motor_set_voltage(), so the magnitude cannot overflow. */
	uint32_t duty_mag = (uint32_t)(data->duty < 0 ? -data->duty : data->duty);
	bool invert = data->duty < 0;
	/* Q31 -> ticks, rounded so a full-scale duty reaches max_pulse_cycles. */
	uint32_t duty_ticks =
		(uint32_t)(((uint64_t)duty_mag * data->max_pulse_cycles + BIT64(30)) >> 31);

	for (uint8_t i = 0; i < BLDC_6STEP_LEG_COUNT; i++) {
		pwm_bridge_leg_state_t leg = bldc_commutation_table_fwd[hall_state][i];

		legs[i].state = invert ? bldc_leg_invert(leg) : leg;
		legs[i].pulse_cycles = duty_ticks;
	}

	return pwm_bridge_set_pattern(cfg->pwm, legs, BLDC_6STEP_LEG_COUNT);
}

/**
 * @brief Read the instantaneous 3-bit Hall state, (U << 2) | (V << 1) | W.
 *
 * @param dev motor device.
 *
 * @return The Hall state, or a fault code (0) if the read fails, so the caller treats it as one.
 */
static uint8_t bldc_read_hall_state(const struct device *dev)
{
	const struct bldc_6step_config *cfg = dev->config;
	uint8_t state;
	int ret;

	ret = bldc_hall_get_state(cfg->hall, &state);
	if (ret < 0) {
		/* Report a sensor fault: callers force high-Z on those codes. */
		return BLDC_HALL_STATE_FAULT_LOW;
	}
	return state;
}

/**
 * @brief Hall edge handler (ISR): commutate to the new Hall state, then update the motion.
 *
 * @note The commutation latency is the table lookup and one bridge commit: the motion only runs
 * after it, still under the lock, so a reader never sees it half-updated.
 *
 * @param hall_dev bldc_hall device, unused.
 * @param state Hall state at the edge.
 * @param period_ticks Ticks since the previous timed edge, 0 when untimed.
 * @param user_data The motor device.
 */
static void bldc_on_hall_edge(const struct device *hall_dev, uint8_t state, uint32_t period_ticks,
			      void *user_data)
{
	const struct device *dev = user_data;
	struct bldc_6step_data *data = dev->data;

	ARG_UNUSED(hall_dev);

	unsigned int key = irq_lock();

	data->hall_state = state;

	if (data->running) {
		int ret = bldc_apply_pattern(dev, state);

		if (ret < 0) {
			LOG_ERR("[%s] Commutation failed at Hall state 0x%x: %d", dev->name, state,
				ret);
		}
	}

	bldc_motion_on_edge(dev, state, period_ticks);

	irq_unlock(key);
}

/**
 * @brief Seed the commutation from the current Hall state and enable the bridge.
 *
 * @param dev motor device.
 *
 * @retval 0 on success.
 * @retval -EIO if the Hall state is a sensor fault.
 * @retval -errno Other negative errno code from the bridge.
 */
static int bldc_6step_start(const struct device *dev)
{
	const struct bldc_6step_config *cfg = dev->config;
	struct bldc_6step_data *data = dev->data;
	int ret;

	if (data->running) {
		return 0;
	}

	unsigned int key = irq_lock();
	uint8_t state = bldc_read_hall_state(dev);

	if (state == BLDC_HALL_STATE_FAULT_LOW || state == BLDC_HALL_STATE_FAULT_HIGH) {
		irq_unlock(key);
		LOG_ERR("[%s] Cannot start: invalid Hall state 0x%x (sensor fault)", dev->name,
			state);
		return -EIO;
	}

	data->hall_state = state;
	data->running = true;

	ret = bldc_apply_pattern(dev, state);
	irq_unlock(key);

	if (ret < 0) {
		LOG_ERR("[%s] Cannot start: initial commutation failed: %d", dev->name, ret);
		data->running = false;
		return ret;
	}

	ret = pwm_bridge_set_outputs(cfg->pwm, true);
	if (ret < 0) {
		LOG_ERR("[%s] Cannot start: enabling the bridge failed: %d", dev->name, ret);
		data->running = false;
		return ret;
	}

	LOG_DBG("[%s] Motor started (Q31 duty %d)", dev->name, data->duty);
	return 0;
}

/**
 * @brief Disable the bridge and release every phase.
 *
 * @param dev motor device.
 *
 * @return 0 on success, negative errno otherwise.
 */
static int bldc_6step_stop(const struct device *dev)
{
	const struct bldc_6step_config *cfg = dev->config;
	struct bldc_6step_data *data = dev->data;
	/* All legs high-Z, no duty — the bridge is released, not braked. */
	const pwm_bridge_leg_t idle[BLDC_6STEP_LEG_COUNT] = {0};
	int ret;

	if (!data->running) {
		return 0;
	}

	/* Both steps stay atomic against a Hall edge. */
	unsigned int key = irq_lock();

	data->running = false;
	ret = pwm_bridge_set_outputs(cfg->pwm, false);
	if (ret == 0) {
		ret = pwm_bridge_set_pattern(cfg->pwm, idle, BLDC_6STEP_LEG_COUNT);
	}
	irq_unlock(key);

	if (ret < 0) {
		LOG_ERR("[%s] Failed to stop the bridge: %d", dev->name, ret);
		return ret;
	}

	LOG_DBG("[%s] Motor stopped", dev->name);
	return 0;
}

/**
 * @brief Set the signed voltage; the sign selects the rotation direction.
 *
 * @note 6-step applies the voltage ratio as the duty of the chopped leg, so the value is stored and
 * used as that duty.
 *
 * @param dev motor device.
 * @param duty Q31, -1.0..1.0; motor_set_voltage() already rejected INT32_MIN.
 *
 * @return 0 on success, negative errno otherwise.
 */
static int bldc_6step_set_voltage(const struct device *dev, q31_t duty)
{
	struct bldc_6step_data *data = dev->data;
	int ret = 0;

	unsigned int key = irq_lock();

	data->duty = duty;

	if (data->running) {
		/* Re-apply so the new duty, and a direction flip, reach the bridge. */
		ret = bldc_apply_pattern(dev, data->hall_state);
	}
	irq_unlock(key);

	if (ret < 0) {
		LOG_ERR("[%s] Failed to apply Q31 duty %d: %d", dev->name, duty, ret);
	}
	return ret;
}

/**
 * @brief Report the raw feedback: shaft angle and speed, plus the phase currents when an adc is
 * bound.
 *
 * @param dev motor device.
 * @param fb Output, filled on success.
 *
 * @return 0 on success, negative errno otherwise.
 */
static int bldc_6step_get_feedback(const struct device *dev, struct motor_feedback *fb)
{
	const struct bldc_6step_config *cfg = dev->config;
	int ret;

	ret = bldc_motion_get(dev, fb);
	if (ret < 0) {
		return ret;
	}

	if (cfg->adc != NULL) {
		bldc_current_get(dev, fb);
	} else {
		fb->phase_count = 0;
	}

	return 0;
}

static DEVICE_API(motor, bldc_6step_motor_api) = {
	.start = bldc_6step_start,
	.stop = bldc_6step_stop,
	.set_voltage = bldc_6step_set_voltage,
	.get_feedback = bldc_6step_get_feedback,
};

/**
 * @brief Check the bound devices, seed the Hall state, then arm the Hall edges with the motor
 * stopped.
 *
 * @note Registering the edge handler is what arms the Hall interrupt, so it comes last.
 *
 * @param dev motor device.
 *
 * @return 0 on success, negative errno otherwise.
 */
static int bldc_6step_init(const struct device *dev)
{
	const struct bldc_6step_config *cfg = dev->config;
	struct bldc_6step_data *data = dev->data;
	pwm_bridge_caps_t caps;
	int ret;

	data->running = false;
	data->duty = 0;
	data->hall_state = 0;

	if (!device_is_ready(cfg->pwm) || !device_is_ready(cfg->hall) ||
	    (cfg->adc != NULL && !device_is_ready(cfg->adc))) {
		LOG_ERR("[%s] A bound device is not ready", dev->name);
		return -ENODEV;
	}

	ret = pwm_bridge_get_caps(cfg->pwm, &caps);
	if (ret < 0) {
		LOG_ERR("[%s] Failed to read the bridge capabilities: %d", dev->name, ret);
		return ret;
	}

	/* set_pattern() takes exactly leg_count legs, one per phase here. */
	if (caps.leg_count != BLDC_6STEP_LEG_COUNT) {
		LOG_ERR("[%s] Bridge drives %u legs, 6-step needs %u", dev->name, caps.leg_count,
			BLDC_6STEP_LEG_COUNT);
		return -EINVAL;
	}

	if ((caps.state_mask & BLDC_6STEP_LEG_STATES) != BLDC_6STEP_LEG_STATES) {
		LOG_ERR("[%s] Bridge lacks a leg state 6-step needs (mask 0x%x)", dev->name,
			caps.state_mask);
		return -ENOTSUP;
	}

	/* Cached once: every duty conversion scales against it. */
	data->max_pulse_cycles = caps.max_pulse_cycles;

	/* Seed the Hall state so a read before start() works. */
	data->hall_state = bldc_read_hall_state(dev);

	ret = bldc_motion_init(dev);
	if (ret < 0) {
		return ret;
	}

	if (cfg->adc != NULL) {
		ret = bldc_current_init(dev);
		if (ret < 0) {
			return ret;
		}
	}

	/* Registering the handler is what arms the Hall interrupt, so nothing
	 * commutates, and no step is counted, until the seed states above are in place.
	 */
	ret = bldc_hall_set_edge_handler(cfg->hall, bldc_on_hall_edge, (void *)dev);
	if (ret < 0) {
		LOG_ERR("[%s] Failed to register the Hall edge handler: %d", dev->name, ret);
		return ret;
	}

	LOG_DBG("[%s] BLDC 6-step driver ready (hall state=0x%x)", dev->name, data->hall_state);
	return 0;
}

/** The bldc_adc named by the adc property, or NULL when it is unset. */
#define BLDC_6STEP_ADC(idx)                                                                        \
	COND_CODE_1(DT_INST_NODE_HAS_PROP(idx, adc), (DEVICE_DT_GET(DT_INST_PHANDLE(idx, adc))),   \
		    (NULL))

/**
 * True unless the adc property names a sampler synchronized to another bridge than this motor's
 * own, whose currents would be latched against someone else's switching cycle.
 */
#define BLDC_6STEP_ADC_SYNCED(idx)                                                                 \
	COND_CODE_1(DT_INST_NODE_HAS_PROP(idx, adc),                                               \
		    (DT_SAME_NODE(DT_INST_PHANDLE(idx, pwm),                                       \
				  DT_PHANDLE(DT_INST_PHANDLE(idx, adc), pwm))),                    \
		    (1))

/** True unless an adc is bound without the front-end values that scale its counts to milliamps. */
#define BLDC_6STEP_ADC_SCALED(idx)                                                                 \
	COND_CODE_1(DT_INST_NODE_HAS_PROP(idx, adc),                                               \
		    (DT_INST_PROP_OR(idx, shunt_resistance_uohm, 0) > 0 &&                         \
		     DT_INST_PROP_OR(idx, current_gain_milli, 0) > 0),                             \
		    (1))

/** Instantiate one motor device from DT instance @p idx. */
#define BLDC_6STEP_INST(idx)                                                                       \
	BUILD_ASSERT(DT_NODE_HAS_COMPAT(DT_DRV_INST(idx), zephyr_motor),                           \
		     "instance " #idx ": node must also list the \"zephyr,motor\" compatible");    \
	BUILD_ASSERT(DT_INST_PROP(idx, motor_pole_pairs) > 0,                                      \
		     "instance " #idx ": motor-pole-pairs must be at least 1");                    \
	BUILD_ASSERT(BLDC_6STEP_ADC_SYNCED(idx),                                                   \
		     "instance " #idx ": adc must be synced to this motor's own pwm bridge");      \
	BUILD_ASSERT(BLDC_6STEP_ADC_SCALED(idx),                                                   \
		     "instance " #idx ": adc needs shunt-resistance-uohm and current-gain-milli"); \
                                                                                                   \
	static const struct bldc_6step_config bldc_6step_config_##idx = {                          \
		.pwm = DEVICE_DT_GET(DT_INST_PHANDLE(idx, pwm)),                                   \
		.hall = DEVICE_DT_GET(DT_INST_PHANDLE(idx, hall)),                                 \
		.adc = BLDC_6STEP_ADC(idx),                                                        \
		.motion =                                                                          \
			{                                                                          \
				.pole_pairs = DT_INST_PROP(idx, motor_pole_pairs),                 \
			},                                                                         \
		.current =                                                                         \
			{                                                                          \
				.shunt_uohm = DT_INST_PROP_OR(idx, shunt_resistance_uohm, 0),      \
				.gain_milli = DT_INST_PROP_OR(idx, current_gain_milli, 0),         \
			},                                                                         \
	};                                                                                         \
                                                                                                   \
	static struct bldc_6step_data bldc_6step_data_##idx;                                       \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(idx, bldc_6step_init, NULL, &bldc_6step_data_##idx,                  \
			      &bldc_6step_config_##idx, POST_KERNEL, CONFIG_MOTOR_INIT_PRIORITY,   \
			      &bldc_6step_motor_api);

DT_INST_FOREACH_STATUS_OKAY(BLDC_6STEP_INST)
