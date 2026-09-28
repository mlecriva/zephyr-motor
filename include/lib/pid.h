/*
 * Copyright (c) 2026 Rtone
 * Copyright (c) 2026 Mathis Lécrivain <lecrivain.mathis@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_APPS_INCLUDE_LIB_PID_H_
#define ZEPHYR_APPS_INCLUDE_LIB_PID_H_

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Configuration of a PID regulator.
 */
struct pid_config {
	/** Proportional gain, output per unit of error. */
	float kp;
	/** Integral gain, output per unit of error and per second. */
	float ki;
	/** Derivative gain, output per unit of error change per second. */
	float kd;
	/** Regulation period, in seconds. */
	float dt;
	/** Lower bound of the output. */
	float output_min;
	/** Upper bound of the output. */
	float output_max;
	/** Lower bound of the integral accumulator. */
	float integral_min;
	/** Upper bound of the integral accumulator. */
	float integral_max;
};

/**
 * @brief PID regulator: its configuration and its runtime state.
 */
struct pid {
	/** Configuration, updated by the setters. */
	struct pid_config cfg;
	/** Integral accumulator, in output units. */
	float integral;
	/** Error of the previous step, for the derivative term. */
	float prev_error;
};

/**
 * @brief Initialize a PID regulator.
 *
 * @param pid Regulator.
 * @param cfg Configuration to apply.
 *
 * @retval 0 on success.
 * @retval -EINVAL if @p pid or @p cfg is NULL, if the period is not strictly
 *         positive, or if a lower limit is above its upper limit.
 */
int pid_init(struct pid *pid, const struct pid_config *cfg);

/**
 * @brief Reset the runtime state.
 *
 * @param pid Regulator.
 *
 * @retval 0 on success.
 * @retval -EINVAL if @p pid is NULL.
 */
int pid_reset(struct pid *pid);

/**
 * @brief Run one regulation step.
 *
 * @param pid Initialized regulator.
 * @param setpoint Desired value of the controlled variable.
 * @param feedback Measured value of the controlled variable.
 * @param out Output, the command within the output limits.
 *
 * @retval 0 on success.
 * @retval -EINVAL if @p pid or @p out is NULL.
 */
int pid_update(struct pid *pid, float setpoint, float feedback, float *out);

/**
 * @brief Update the proportional gain.
 *
 * @param pid Regulator.
 * @param kp New proportional gain.
 *
 * @retval 0 on success.
 * @retval -EINVAL if @p pid is NULL.
 */
int pid_set_kp(struct pid *pid, float kp);

/**
 * @brief Update the integral gain.
 *
 * @param pid Regulator.
 * @param ki New integral gain.
 *
 * @retval 0 on success.
 * @retval -EINVAL if @p pid is NULL.
 */
int pid_set_ki(struct pid *pid, float ki);

/**
 * @brief Update the derivative gain.
 *
 * @param pid Regulator.
 * @param kd New derivative gain.
 *
 * @retval 0 on success.
 * @retval -EINVAL if @p pid is NULL.
 */
int pid_set_kd(struct pid *pid, float kd);

/**
 * @brief Update the output limits.
 *
 * @param pid Regulator.
 * @param min New lower bound of the output.
 * @param max New upper bound of the output.
 *
 * @retval 0 on success.
 * @retval -EINVAL if @p pid is NULL.
 */
int pid_set_output_limits(struct pid *pid, float min, float max);

/**
 * @brief Update the integral accumulator limits.
 *
 * @param pid Regulator.
 * @param min New lower bound of the integral accumulator.
 * @param max New upper bound of the integral accumulator.
 *
 * @retval 0 on success.
 * @retval -EINVAL if @p pid is NULL.
 */
int pid_set_integral_limits(struct pid *pid, float min, float max);

/**
 * @brief Update the regulation period.
 *
 * @param pid Regulator.
 * @param dt New regulation period, in seconds.
 *
 * @retval 0 on success.
 * @retval -EINVAL if @p pid is NULL.
 */
int pid_set_dt(struct pid *pid, float dt);

/**
 * @brief Force the integral accumulator.
 *
 * @param pid Regulator.
 * @param value New integral accumulator.
 *
 * @retval 0 on success.
 * @retval -EINVAL if @p pid is NULL.
 */
int pid_set_integral(struct pid *pid, float value);

/**
 * @brief Read the integral accumulator.
 *
 * @param pid Regulator.
 * @param out Output, the integral accumulator.
 *
 * @retval 0 on success.
 * @retval -EINVAL if @p pid or @p out is NULL.
 */
int pid_get_integral(const struct pid *pid, float *out);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_APPS_INCLUDE_LIB_PID_H_ */
