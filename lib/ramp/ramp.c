/*
 * Copyright (c) 2026 Rtone
 * Copyright (c) 2026 Mathis Lécrivain <lecrivain.mathis@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <math.h>
#include <stdbool.h>
#include <stddef.h>

#include <lib/ramp.h>

int ramp_init(struct ramp *ramp, const struct ramp_config *cfg)
{
	if (ramp == NULL || cfg == NULL) {
		return -EINVAL;
	}

	if (cfg->accel <= 0.0f || cfg->decel <= 0.0f || cfg->dt <= 0.0f) {
		return -EINVAL;
	}

	ramp->cfg = *cfg;
	ramp->target = 0.0f;
	ramp->value = 0.0f;

	return 0;
}

int ramp_reset(struct ramp *ramp, float value)
{
	if (ramp == NULL) {
		return -EINVAL;
	}

	ramp->target = value;
	ramp->value = value;

	return 0;
}

int ramp_set_target(struct ramp *ramp, float target)
{
	if (ramp == NULL) {
		return -EINVAL;
	}

	ramp->target = target;

	return 0;
}

int ramp_step(struct ramp *ramp, float *out)
{
	if (ramp == NULL || out == NULL) {
		return -EINVAL;
	}

	const float delta = ramp->target - ramp->value;
	const bool toward_zero = (ramp->value * delta) < 0.0f;
	const float step = (toward_zero ? ramp->cfg.decel : ramp->cfg.accel) * ramp->cfg.dt;

	if (fabsf(delta) <= step) {
		ramp->value = ramp->target;
	} else if (toward_zero && fabsf(ramp->value) <= step) {
		ramp->value = 0.0f;
	} else {
		ramp->value += (delta > 0.0f) ? step : -step;
	}

	*out = ramp->value;

	return 0;
}
