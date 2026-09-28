/*
 * Copyright (c) 2026 Rtone
 * Copyright (c) 2026 Mathis Lécrivain <lecrivain.mathis@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_APPS_INCLUDE_LIB_RAMP_H_
#define ZEPHYR_APPS_INCLUDE_LIB_RAMP_H_

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Configuration of a ramp.
 */
struct ramp_config {
	/** Slew rate away from 0, in units per second. */
	float accel;
	/** Slew rate toward 0, in units per second. */
	float decel;
	/** Step period, in seconds. */
	float dt;
};

/**
 * @brief Ramp: its configuration and its state.
 *
 */
struct ramp {
	/** Configuration applied at init. */
	struct ramp_config cfg;
	/** Value the output moves toward. */
	float target;
	/** Output of the last step. */
	float value;
};

/**
 * @brief Initialize a ramp at rest.
 *
 * @param ramp Ramp.
 * @param cfg Configuration to apply.
 *
 * @retval 0 on success.
 * @retval -EINVAL if @p ramp or @p cfg is NULL, or if a rate or the period is
 *         not strictly positive.
 */
int ramp_init(struct ramp *ramp, const struct ramp_config *cfg);

/**
 * @brief Jump the output to a value, and hold it there.
 *
 * @param ramp Ramp.
 * @param value New output and target.
 *
 * @retval 0 on success.
 * @retval -EINVAL if @p ramp is NULL.
 */
int ramp_reset(struct ramp *ramp, float value);

/**
 * @brief Update the target.
 *
 * @param ramp Ramp.
 * @param target New target.
 *
 * @retval 0 on success.
 * @retval -EINVAL if @p ramp is NULL.
 */
int ramp_set_target(struct ramp *ramp, float target);

/**
 * @brief Run one step: move the output toward the target by one period.
 *
 * @param ramp Ramp.
 * @param out Output, the new output.
 *
 * @retval 0 on success.
 * @retval -EINVAL if @p ramp or @p out is NULL.
 */
int ramp_step(struct ramp *ramp, float *out);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_APPS_INCLUDE_LIB_RAMP_H_ */
