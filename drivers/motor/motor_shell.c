/*
 * Copyright (c) 2026 Rtone
 * Copyright (c) 2026 Mathis Lécrivain <lecrivain.mathis@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/shell/shell.h>

#include <drivers/motor.h>

/** One enabled motor, as a @ref motor_devs initializer. */
#define MOTOR_DEV_REF(node_id)    DEVICE_DT_GET(node_id),
/** Its implementation compatible, listed first, "zephyr,motor" after it. */
#define MOTOR_COMPAT_REF(node_id) DT_PROP_BY_IDX(node_id, compatible, 0),

/** Every enabled "zephyr,motor" node, whatever the implementation. */
static const struct device *const motor_devs[] = {
	DT_FOREACH_STATUS_OKAY(zephyr_motor, MOTOR_DEV_REF)};

/** Implementation compatible of each entry of @ref motor_devs. */
static const char *const motor_compats[] = {DT_FOREACH_STATUS_OKAY(zephyr_motor, MOTOR_COMPAT_REF)};

/** Entries in @ref motor_devs. */
#define MOTOR_COUNT ARRAY_SIZE(motor_devs)

/**
 * @brief Resolve a motor name to its device.
 *
 * @note The name is optional when only one motor is instantiated, required otherwise.
 *
 * @param sh Shell the errors are printed on.
 * @param name Device name, or NULL for the only motor.
 *
 * @return The motor device, or NULL if it cannot be resolved.
 */
static const struct device *motor_lookup(const struct shell *sh, const char *name)
{
	if (name == NULL) {
		if (MOTOR_COUNT == 1) {
			return motor_devs[0];
		}
		shell_error(sh, "motor name required (try 'motor list')");
		return NULL;
	}
	for (size_t i = 0; i < MOTOR_COUNT; i++) {
		if (strcmp(motor_devs[i]->name, name) == 0) {
			return motor_devs[i];
		}
	}
	shell_error(sh, "unknown motor '%s'", name);
	return NULL;
}

/**
 * @brief Tab-completion source for the optional motor name argument.
 *
 * @param idx Entry the shell asks for.
 * @param entry Output, the name of motor @p idx, or a NULL syntax past the last one.
 */
static void motor_name_get(size_t idx, struct shell_static_entry *entry)
{
	entry->syntax = (idx < MOTOR_COUNT) ? motor_devs[idx]->name : NULL;
	entry->handler = NULL;
	entry->subcmd = NULL;
	entry->help = NULL;
}

SHELL_DYNAMIC_CMD_CREATE(dsub_motor_name, motor_name_get);

/**
 * @brief Resolve the motor of a command taking no value (start, stop).
 *
 * @note argc == 1 is the single-motor form; argc == 2 names the motor in argv[1].
 *
 * @param sh Shell the errors are printed on.
 * @param argc Argument count, command included.
 * @param argv Arguments, command included.
 *
 * @return The motor device, or NULL if it cannot be resolved.
 */
static const struct device *parse_dev_only(const struct shell *sh, size_t argc, char **argv)
{
	return motor_lookup(sh, argc >= 2 ? argv[1] : NULL);
}

/**
 * @brief Resolve the motor and the value of a command taking one value (voltage).
 *
 * @note argc == 2 is the single-motor form, argv[1] being the value; argc == 3 names the motor in
 * argv[1] and gives the value in argv[2].
 *
 * @param sh Shell the errors are printed on.
 * @param argc Argument count, command included.
 * @param argv Arguments, command included.
 * @param value_out Output, the value string; set whenever the argument count is valid.
 *
 * @return The motor device, or NULL on a wrong argument count or a motor that cannot be resolved.
 */
static const struct device *parse_dev_with_value(const struct shell *sh, size_t argc, char **argv,
						 const char **value_out)
{
	const char *name = NULL;
	const char *value;

	if (argc == 2) {
		value = argv[1];
	} else if (argc == 3) {
		name = argv[1];
		value = argv[2];
	} else {
		shell_error(sh, "wrong number of arguments");
		return NULL;
	}
	*value_out = value;
	return motor_lookup(sh, name);
}

/**
 * @brief motor list: print every motor with its implementation compatible.
 *
 * @param sh Shell to print on.
 * @param argc Unused.
 * @param argv Unused.
 *
 * @retval 0 Always.
 */
static int cmd_list(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	shell_print(sh, "%-20s %s", "name", "implementation");
	for (size_t i = 0; i < MOTOR_COUNT; i++) {
		shell_print(sh, "%-20s %s", motor_devs[i]->name, motor_compats[i]);
	}
	return 0;
}

/**
 * @brief motor start [name]: start the motor.
 *
 * @param sh Shell to print on.
 * @param argc Argument count, command included.
 * @param argv Arguments, command included.
 *
 * @retval 0 on success.
 * @retval -ENODEV if the motor cannot be resolved.
 * @retval -errno Other negative errno code from the motor.
 */
static int cmd_start(const struct shell *sh, size_t argc, char **argv)
{
	const struct device *dev = parse_dev_only(sh, argc, argv);

	if (dev == NULL) {
		return -ENODEV;
	}

	int ret = motor_start(dev);

	if (ret < 0) {
		shell_error(sh, "start failed: %d", ret);
		return ret;
	}
	return 0;
}

/**
 * @brief motor stop [name]: stop the motor and release every phase.
 *
 * @param sh Shell to print on.
 * @param argc Argument count, command included.
 * @param argv Arguments, command included.
 *
 * @retval 0 on success.
 * @retval -ENODEV if the motor cannot be resolved.
 * @retval -errno Other negative errno code from the motor.
 */
static int cmd_stop(const struct shell *sh, size_t argc, char **argv)
{
	const struct device *dev = parse_dev_only(sh, argc, argv);

	if (dev == NULL) {
		return -ENODEV;
	}

	int ret = motor_stop(dev);

	if (ret < 0) {
		shell_error(sh, "stop failed: %d", ret);
		return ret;
	}
	return 0;
}

/**
 * @brief Convert a per-mille string to Q31.
 *
 * @note +/-1000 maps to +/-INT32_MAX, never to INT32_MIN. The whole string must be an integer:
 * anything else is rejected rather than read as 0, which would brake the motor.
 *
 * @param sh Shell the errors are printed on.
 * @param str Per-mille value, -1000..1000.
 * @param out Output, the Q31 value; untouched on failure.
 *
 * @retval 0 on success.
 * @retval -EINVAL if the value is not an integer or is out of range.
 */
static int parse_milli_q31(const struct shell *sh, const char *str, q31_t *out)
{
	int err = 0;
	long v = shell_strtol(str, 0, &err);

	if (err != 0 || v < -1000 || v > 1000) {
		shell_error(sh, "invalid value '%s' (integer -1000..1000)", str);
		return -EINVAL;
	}

	*out = (q31_t)(((int64_t)v * INT32_MAX) / 1000);
	return 0;
}

/**
 * @brief motor voltage [name] <-1000..1000>: apply a signed voltage, in per-mille.
 *
 * @param sh Shell to print on.
 * @param argc Argument count, command included.
 * @param argv Arguments, command included.
 *
 * @retval 0 on success.
 * @retval -ENODEV if the arguments or the motor cannot be resolved.
 * @retval -EINVAL if the value is not an integer or is out of range.
 * @retval -errno Other negative errno code from the motor.
 */
static int cmd_voltage(const struct shell *sh, size_t argc, char **argv)
{
	const char *value_str;
	const struct device *dev = parse_dev_with_value(sh, argc, argv, &value_str);
	q31_t voltage;

	if (dev == NULL) {
		return -ENODEV;
	}

	int ret = parse_milli_q31(sh, value_str, &voltage);

	if (ret < 0) {
		return ret;
	}

	ret = motor_set_voltage(dev, voltage);
	if (ret < 0) {
		shell_error(sh, "set_voltage failed: %d", ret);
		return ret;
	}
	return 0;
}

SHELL_SUBCMD_SET_CREATE(motor_cmds, (motor));

SHELL_SUBCMD_ADD((motor), list, NULL, "List motor instances", cmd_list, 1, 0);
SHELL_SUBCMD_ADD((motor), start, &dsub_motor_name, "[name] — Start the motor", cmd_start, 1, 1);
SHELL_SUBCMD_ADD((motor), stop, &dsub_motor_name, "[name] — Stop the motor", cmd_stop, 1, 1);
SHELL_SUBCMD_ADD((motor), voltage, &dsub_motor_name,
		 "[name] <-1000..1000> — Set signed voltage, per mille (sign = direction)",
		 cmd_voltage, 2, 1);

SHELL_CMD_REGISTER(motor, &motor_cmds, "Motor control", NULL);
