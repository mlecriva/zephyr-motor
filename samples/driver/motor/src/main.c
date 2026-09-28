/*
 * Copyright (c) 2026 Rtone
 * Copyright (c) 2026 Mathis Lécrivain <lecrivain.mathis@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(main, CONFIG_APP_LOG_LEVEL);

static const struct device *const motor = DEVICE_DT_GET(DT_NODELABEL(motor0));

/**
 * @brief Check the motor came up, then leave the shell in charge.
 *
 * @return 0 on success, -ENODEV if the motor driver failed its own init.
 */
int main(void)
{
	if (!device_is_ready(motor)) {
		LOG_ERR("%s failed to initialize", motor->name);
		return -ENODEV;
	}

	LOG_INF("%s ready", motor->name);

	return 0;
}
