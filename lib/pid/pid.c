/*
 * Copyright (c) 2026 Rtone
 * Copyright (c) 2026 Mathis Lécrivain <lecrivain.mathis@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <stdbool.h>

#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include <lib/pid.h>

LOG_MODULE_REGISTER(pid, CONFIG_PID_LOG_LEVEL);

/**
 * @brief Check a configuration before pid_init() applies it.
 *
 * @param cfg Configuration to check.
 *
 * @return true if the period is strictly positive and each limit pair is ordered.
 */
static bool config_is_valid(const struct pid_config *cfg)
{
	if (cfg->dt <= 0.0f) {
		return false;
	}

	if (cfg->output_min > cfg->output_max) {
		return false;
	}

	if (cfg->integral_min > cfg->integral_max) {
		return false;
	}

	return true;
}

int pid_init(struct pid *pid, const struct pid_config *cfg)
{
	if (pid == NULL || cfg == NULL) {
		return -EINVAL;
	}

	if (!config_is_valid(cfg)) {
		LOG_ERR("Invalid PID config (dt=%f, out=[%f,%f], int=[%f,%f])", (double)cfg->dt,
			(double)cfg->output_min, (double)cfg->output_max, (double)cfg->integral_min,
			(double)cfg->integral_max);
		return -EINVAL;
	}

	pid->cfg = *cfg;
	pid->integral = 0.0f;
	pid->prev_error = 0.0f;

	return 0;
}

int pid_reset(struct pid *pid)
{
	if (pid == NULL) {
		return -EINVAL;
	}

	pid->integral = 0.0f;
	pid->prev_error = 0.0f;

	return 0;
}

int pid_update(struct pid *pid, float setpoint, float feedback, float *out)
{
	if (pid == NULL || out == NULL) {
		return -EINVAL;
	}

	const struct pid_config *cfg = &pid->cfg;
	const float error = setpoint - feedback;
	const float p_term = cfg->kp * error;
	/* Clamped first, so the integral cannot grow unbounded while the actuator saturates. */
	const float i_term = CLAMP(pid->integral + cfg->ki * error * cfg->dt, cfg->integral_min,
				   cfg->integral_max);
	const float d_term = cfg->kd * (error - pid->prev_error) / cfg->dt;
	const float out_raw = p_term + i_term + d_term;
	const float out_val = CLAMP(out_raw, cfg->output_min, cfg->output_max);

	/* Back-calculation: the part the output saturation cut is taken back from the integral,
	 * so it stops accumulating against the actuator limit.
	 */
	pid->integral = CLAMP(i_term + (out_val - out_raw), cfg->integral_min, cfg->integral_max);
	pid->prev_error = error;

	*out = out_val;

	return 0;
}

int pid_set_kp(struct pid *pid, float kp)
{
	if (pid == NULL) {
		return -EINVAL;
	}

	pid->cfg.kp = kp;

	return 0;
}

int pid_set_ki(struct pid *pid, float ki)
{
	if (pid == NULL) {
		return -EINVAL;
	}

	pid->cfg.ki = ki;

	return 0;
}

int pid_set_kd(struct pid *pid, float kd)
{
	if (pid == NULL) {
		return -EINVAL;
	}

	pid->cfg.kd = kd;

	return 0;
}

int pid_set_output_limits(struct pid *pid, float min, float max)
{
	if (pid == NULL) {
		return -EINVAL;
	}

	pid->cfg.output_min = min;
	pid->cfg.output_max = max;

	return 0;
}

int pid_set_integral_limits(struct pid *pid, float min, float max)
{
	if (pid == NULL) {
		return -EINVAL;
	}

	pid->cfg.integral_min = min;
	pid->cfg.integral_max = max;

	return 0;
}

int pid_set_dt(struct pid *pid, float dt)
{
	if (pid == NULL) {
		return -EINVAL;
	}

	pid->cfg.dt = dt;

	return 0;
}

int pid_set_integral(struct pid *pid, float value)
{
	if (pid == NULL) {
		return -EINVAL;
	}

	pid->integral = CLAMP(value, pid->cfg.integral_min, pid->cfg.integral_max);

	return 0;
}

int pid_get_integral(const struct pid *pid, float *out)
{
	if (pid == NULL || out == NULL) {
		return -EINVAL;
	}

	*out = pid->integral;

	return 0;
}
