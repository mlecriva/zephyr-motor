/*
 * Copyright (c) 2026 Rtone
 * Copyright (c) 2026 Mathis Lécrivain <lecrivain.mathis@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_APPS_INCLUDE_DRIVERS_PWM_BRIDGE_H_
#define ZEPHYR_APPS_INCLUDE_DRIVERS_PWM_BRIDGE_H_

#include <errno.h>
#include <stdbool.h>
#include <stdint.h>

#include <zephyr/device.h>
#include <zephyr/sys/util.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Output state of one half-bridge leg.
 */
typedef enum {
	/** Both switches off. Outputs released, not driven. */
	PWM_BRIDGE_LEG_HI_Z = 0,
	/** Chopped at the requested pulse width, low side complementary with dead-time. */
	PWM_BRIDGE_LEG_PWM,
	/** Low side conducting the whole period, high side actively held off. */
	PWM_BRIDGE_LEG_FORCE_LOW,
	/** High side conducting the whole period, low side actively held off. */
	PWM_BRIDGE_LEG_FORCE_HIGH,
} pwm_bridge_leg_state_t;

/** Highest valid leg state; set_pattern() rejects anything above it. */
#define PWM_BRIDGE_LEG_STATE_MAX PWM_BRIDGE_LEG_FORCE_HIGH

/** Bit standing for @p state in @ref pwm_bridge_caps_t::state_mask. */
#define PWM_BRIDGE_STATE_BIT(state) BIT(state)

/** State of one leg within a pattern. */
typedef struct {
	/** What the two switches of the leg do. */
	pwm_bridge_leg_state_t state;
	/** On-time in carrier cycles; ignored unless the leg is in PWM state. */
	uint32_t pulse_cycles;
} pwm_bridge_leg_t;

/**
 * @brief What an instance can do. Fixed for the lifetime of the device.
 */
typedef struct {
	/** Legs this bridge drives; the length set_pattern() expects. */
	uint8_t leg_count;
	/** Supported leg states, as @ref PWM_BRIDGE_STATE_BIT bits. */
	uint8_t state_mask;
	/** Pulse width that stands for 100 % duty. Not a duration. */
	uint32_t max_pulse_cycles;
	/** Carrier frequency actually programmed, in Hz. */
	uint32_t carrier_hz;
	/** Carrier counts up then down, so one period spans max_pulse_cycles twice. */
	bool center_aligned;
} pwm_bridge_caps_t;

/**
 * @brief Driver-side vtable. Filled by each concrete pwm_bridge driver.
 *
 * All three ops are mandatory.
 */
__subsystem struct pwm_bridge_driver_api {
	/** @see pwm_bridge_set_pattern() */
	int (*set_pattern)(const struct device *dev, const pwm_bridge_leg_t *legs, uint8_t count);
	/** @see pwm_bridge_set_outputs() */
	int (*set_outputs)(const struct device *dev, bool enable);
	/** @see pwm_bridge_get_caps() */
	int (*get_caps)(const struct device *dev, pwm_bridge_caps_t *caps);
};

/**
 * @brief Apply a bridge pattern, committed atomically.
 *
 * Every leg switches on the same carrier cycle. Safe to call from an ISR.
 *
 * @param dev pwm_bridge device.
 * @param legs One entry per leg, in the order the binding defines them.
 * @param count Entries in @p legs; must be equal to @ref pwm_bridge_caps_t::leg_count.
 *
 * @retval 0 on success.
 * @retval -EINVAL if @p dev or @p legs is NULL, @p count does not match the
 *         bridge, or a leg state is unknown; the bridge is left untouched.
 * @retval -ENOTSUP if a requested state is not in the capability mask.
 */
static inline int pwm_bridge_set_pattern(const struct device *dev, const pwm_bridge_leg_t *legs,
					 uint8_t count)
{
	if (dev == NULL || legs == NULL) {
		return -EINVAL;
	}

	return DEVICE_API_GET(pwm_bridge, dev)->set_pattern(dev, legs, count);
}

/**
 * @brief Enable or disable the bridge output stage.
 *
 * Master gate on every switch. Disabling drives them to their idle state
 * without touching the pattern or the time base.
 *
 * @param dev pwm_bridge device.
 * @param enable true to arm the output stage, false to idle it.
 *
 * @retval 0 on success.
 * @retval -EINVAL if @p dev is NULL.
 */
static inline int pwm_bridge_set_outputs(const struct device *dev, bool enable)
{
	if (dev == NULL) {
		return -EINVAL;
	}

	return DEVICE_API_GET(pwm_bridge, dev)->set_outputs(dev, enable);
}

/**
 * @brief Read what the bridge is and what it can do.
 *
 * Meant for a consumer's init: check the leg count against the topology it
 * drives, the state mask against the states it emits, and keep
 * @ref pwm_bridge_caps_t::max_pulse_cycles as its duty full scale.
 *
 * @param dev pwm_bridge device.
 * @param caps Output, filled on success.
 *
 * @retval 0 on success.
 * @retval -EINVAL if @p dev or @p caps is NULL.
 */
static inline int pwm_bridge_get_caps(const struct device *dev, pwm_bridge_caps_t *caps)
{
	if (dev == NULL || caps == NULL) {
		return -EINVAL;
	}

	return DEVICE_API_GET(pwm_bridge, dev)->get_caps(dev, caps);
}

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_APPS_INCLUDE_DRIVERS_PWM_BRIDGE_H_ */
