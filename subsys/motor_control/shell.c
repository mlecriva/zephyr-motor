/*
 * Copyright (c) 2026 Rtone
 * Copyright (c) 2026 Mathis Lécrivain <lecrivain.mathis@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <stdlib.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/shell/shell.h>

#include <subsys/motor_control/motor_control.h>

/** One enabled controller, as a @ref mc_devs initializer. */
#define MC_DEV_REF(node_id) DEVICE_DT_GET(node_id),

/** Every enabled "zephyr,motor-control" node. */
static const struct device *const mc_devs[] = {
	DT_FOREACH_STATUS_OKAY(zephyr_motor_control, MC_DEV_REF)};

/** Entries in @ref mc_devs. */
#define MC_COUNT ARRAY_SIZE(mc_devs)

/**
 * @brief Resolve a controller name to its device.
 *
 * @note The name is optional when only one controller is instantiated, required otherwise.
 *
 * @param sh Shell the errors are printed on.
 * @param name Device name, or NULL for the only controller.
 *
 * @return The controller device, or NULL if it cannot be resolved.
 */
static const struct device *mc_lookup(const struct shell *sh, const char *name)
{
	if (name == NULL) {
		if (MC_COUNT == 1) {
			return mc_devs[0];
		}
		shell_error(sh, "controller name required (try 'mc list')");
		return NULL;
	}
	for (size_t i = 0; i < MC_COUNT; i++) {
		if (strcmp(mc_devs[i]->name, name) == 0) {
			return mc_devs[i];
		}
	}
	shell_error(sh, "unknown controller '%s'", name);
	return NULL;
}

/**
 * @brief Tab-completion source for the optional controller name argument.
 *
 * @param idx Entry the shell asks for.
 * @param entry Output, the name of controller @p idx, or a NULL syntax past the last one.
 */
static void mc_name_get(size_t idx, struct shell_static_entry *entry)
{
	entry->syntax = (idx < MC_COUNT) ? mc_devs[idx]->name : NULL;
	entry->handler = NULL;
	entry->subcmd = NULL;
	entry->help = NULL;
}

SHELL_DYNAMIC_CMD_CREATE(dsub_mc_name, mc_name_get);

/**
 * @brief Resolve the controller of a command taking no value (list, start, stop, reset).
 *
 * @note argc == 1 is the single-controller form; argc == 2 names the controller in argv[1].
 *
 * @param sh Shell the errors are printed on.
 * @param argc Argument count, command included.
 * @param argv Arguments, command included.
 *
 * @return The controller device, or NULL if it cannot be resolved.
 */
static const struct device *parse_dev_only(const struct shell *sh, size_t argc, char **argv)
{
	return mc_lookup(sh, argc >= 2 ? argv[1] : NULL);
}

/**
 * @brief Resolve the controller and the value of a command taking one value (target, gains).
 *
 * @note argc == 2 is the single-controller form, argv[1] being the value; argc == 3 names the
 * controller in argv[1] and gives the value in argv[2].
 *
 * @param sh Shell the errors are printed on.
 * @param argc Argument count, command included.
 * @param argv Arguments, command included.
 * @param value_out Output, the value string; set whenever the argument count is valid.
 *
 * @return The controller device, or NULL on a wrong argument count or a controller that cannot be
 * resolved.
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
	return mc_lookup(sh, name);
}

/**
 * @brief Parse a speed in RPM.
 *
 * @note The whole string must be an integer: anything else is rejected rather than read as 0.
 *
 * @param sh Shell the errors are printed on.
 * @param str Signed speed, in RPM.
 * @param out Output, the speed; untouched on failure.
 *
 * @retval 0 on success.
 * @retval -EINVAL if the value is not an integer or does not fit in 32 bits.
 */
static int parse_rpm(const struct shell *sh, const char *str, int32_t *out)
{
	int err = 0;
	long v = shell_strtol(str, 0, &err);

	if (err != 0 || v < INT32_MIN || v > INT32_MAX) {
		shell_error(sh, "invalid speed '%s' (integer RPM)", str);
		return -EINVAL;
	}

	*out = (int32_t)v;
	return 0;
}

/**
 * @brief Parse a PID gain.
 *
 * @note The whole string must be a number: anything else is rejected rather than read as 0.
 *
 * @param sh Shell the errors are printed on.
 * @param str Gain, as a decimal number.
 * @param out Output, the gain; untouched on failure.
 *
 * @retval 0 on success.
 * @retval -EINVAL if the value is not a number.
 */
static int parse_gain(const struct shell *sh, const char *str, float *out)
{
	char *end;
	float v;

	errno = 0;
	v = strtof(str, &end);
	if (errno != 0 || end == str || *end != '\0') {
		shell_error(sh, "invalid gain '%s' (decimal number)", str);
		return -EINVAL;
	}

	*out = v;
	return 0;
}

/**
 * @brief Print the speed-loop gains of one controller as a @ref cmd_list row.
 *
 * @param sh Shell to print on.
 * @param dev Controller device.
 */
static void print_gains(const struct shell *sh, const struct device *dev)
{
	float kp = 0.0f;
	float ki = 0.0f;
	float kd = 0.0f;

	(void)motor_control_get_speed_gains(dev, &kp, &ki, &kd);

	shell_print(sh, "%-20s %12.6f %12.6f %12.6f", dev->name, (double)kp, (double)ki,
		    (double)kd);
}

/**
 * @brief mc list [name]: print the speed-loop gains of one controller, or of every one.
 *
 * @param sh Shell to print on.
 * @param argc Argument count, command included.
 * @param argv Arguments, command included.
 *
 * @retval 0 on success.
 * @retval -ENODEV if the named controller cannot be resolved.
 */
static int cmd_list(const struct shell *sh, size_t argc, char **argv)
{
	const struct device *dev = NULL;

	if (argc >= 2) {
		dev = parse_dev_only(sh, argc, argv);
		if (dev == NULL) {
			return -ENODEV;
		}
	}

	shell_print(sh, "%-20s %12s %12s %12s", "name", "kp", "ki", "kd");
	if (dev != NULL) {
		print_gains(sh, dev);
		return 0;
	}
	for (size_t i = 0; i < MC_COUNT; i++) {
		print_gains(sh, mc_devs[i]);
	}
	return 0;
}

/**
 * @brief mc start [name]: start the speed loop, from a 0 RPM setpoint.
 *
 * @param sh Shell to print on.
 * @param argc Argument count, command included.
 * @param argv Arguments, command included.
 *
 * @retval 0 on success.
 * @retval -ENODEV if the controller cannot be resolved.
 * @retval -errno Other negative errno code from the controller.
 */
static int cmd_start(const struct shell *sh, size_t argc, char **argv)
{
	const struct device *dev = parse_dev_only(sh, argc, argv);

	if (dev == NULL) {
		return -ENODEV;
	}

	int ret = motor_control_start(dev);

	if (ret < 0) {
		shell_error(sh, "start failed: %d", ret);
		return ret;
	}
	return 0;
}

/**
 * @brief mc stop [name]: stop the speed loop and release the motor.
 *
 * @param sh Shell to print on.
 * @param argc Argument count, command included.
 * @param argv Arguments, command included.
 *
 * @retval 0 on success.
 * @retval -ENODEV if the controller cannot be resolved.
 * @retval -errno Other negative errno code from the controller.
 */
static int cmd_stop(const struct shell *sh, size_t argc, char **argv)
{
	const struct device *dev = parse_dev_only(sh, argc, argv);

	if (dev == NULL) {
		return -ENODEV;
	}

	int ret = motor_control_stop(dev);

	if (ret < 0) {
		shell_error(sh, "stop failed: %d", ret);
		return ret;
	}
	return 0;
}

/**
 * @brief mc reset [name]: reset the speed PID state.
 *
 * @param sh Shell to print on.
 * @param argc Argument count, command included.
 * @param argv Arguments, command included.
 *
 * @retval 0 on success.
 * @retval -ENODEV if the controller cannot be resolved.
 * @retval -errno Other negative errno code from the controller.
 */
static int cmd_reset(const struct shell *sh, size_t argc, char **argv)
{
	const struct device *dev = parse_dev_only(sh, argc, argv);

	if (dev == NULL) {
		return -ENODEV;
	}

	int ret = motor_control_reset(dev);

	if (ret < 0) {
		shell_error(sh, "reset failed: %d", ret);
		return ret;
	}
	return 0;
}

/**
 * @brief mc target [name] <rpm>: set the speed setpoint, reached through the ramp.
 *
 * @param sh Shell to print on.
 * @param argc Argument count, command included.
 * @param argv Arguments, command included.
 *
 * @retval 0 on success.
 * @retval -ENODEV if the arguments or the controller cannot be resolved.
 * @retval -EINVAL if the value is not an integer.
 * @retval -errno Other negative errno code from the controller.
 */
static int cmd_target(const struct shell *sh, size_t argc, char **argv)
{
	const char *value_str;
	const struct device *dev = parse_dev_with_value(sh, argc, argv, &value_str);
	int32_t rpm;

	if (dev == NULL) {
		return -ENODEV;
	}

	int ret = parse_rpm(sh, value_str, &rpm);

	if (ret < 0) {
		return ret;
	}

	ret = motor_control_set_target_rpm(dev, rpm);
	if (ret < 0) {
		shell_error(sh, "set_target_rpm failed: %d", ret);
		return ret;
	}
	return 0;
}

/**
 * @brief Shared body of the mc speed kp|ki|kd [name] <gain> commands.
 *
 * @param sh Shell to print on.
 * @param argc Argument count, command included.
 * @param argv Arguments, command included.
 * @param set Controller setter of the gain.
 *
 * @retval 0 on success.
 * @retval -ENODEV if the arguments or the controller cannot be resolved.
 * @retval -EINVAL if the value is not a number.
 * @retval -errno Other negative errno code from the controller.
 */
static int set_gain(const struct shell *sh, size_t argc, char **argv,
		    int (*set)(const struct device *dev, float gain))
{
	const char *value_str;
	const struct device *dev = parse_dev_with_value(sh, argc, argv, &value_str);
	float gain;

	if (dev == NULL) {
		return -ENODEV;
	}

	int ret = parse_gain(sh, value_str, &gain);

	if (ret < 0) {
		return ret;
	}

	ret = set(dev, gain);
	if (ret < 0) {
		shell_error(sh, "setting the gain failed: %d", ret);
		return ret;
	}
	return 0;
}

/**
 * @brief mc speed kp [name] <gain>: set the proportional gain of the speed loop.
 *
 * @param sh Shell to print on.
 * @param argc Argument count, command included.
 * @param argv Arguments, command included.
 *
 * @return See set_gain().
 */
static int cmd_speed_kp(const struct shell *sh, size_t argc, char **argv)
{
	return set_gain(sh, argc, argv, motor_control_set_speed_kp);
}

/**
 * @brief mc speed ki [name] <gain>: set the integral gain of the speed loop.
 *
 * @param sh Shell to print on.
 * @param argc Argument count, command included.
 * @param argv Arguments, command included.
 *
 * @return See set_gain().
 */
static int cmd_speed_ki(const struct shell *sh, size_t argc, char **argv)
{
	return set_gain(sh, argc, argv, motor_control_set_speed_ki);
}

/**
 * @brief mc speed kd [name] <gain>: set the derivative gain of the speed loop.
 *
 * @param sh Shell to print on.
 * @param argc Argument count, command included.
 * @param argv Arguments, command included.
 *
 * @return See set_gain().
 */
static int cmd_speed_kd(const struct shell *sh, size_t argc, char **argv)
{
	return set_gain(sh, argc, argv, motor_control_set_speed_kd);
}

SHELL_STATIC_SUBCMD_SET_CREATE(
	mc_speed_cmds,
	SHELL_CMD_ARG(kp, &dsub_mc_name, "[name] <gain> — Set the proportional gain (‰ per RPM)",
		      cmd_speed_kp, 2, 1),
	SHELL_CMD_ARG(ki, &dsub_mc_name, "[name] <gain> — Set the integral gain (‰ per RPM·s)",
		      cmd_speed_ki, 2, 1),
	SHELL_CMD_ARG(kd, &dsub_mc_name, "[name] <gain> — Set the derivative gain (‰ per RPM/s)",
		      cmd_speed_kd, 2, 1),
	SHELL_SUBCMD_SET_END);

SHELL_SUBCMD_SET_CREATE(mc_cmds, (mc));

SHELL_SUBCMD_ADD((mc), list, &dsub_mc_name, "[name] — List controllers and their speed gains",
		 cmd_list, 1, 1);
SHELL_SUBCMD_ADD((mc), start, &dsub_mc_name, "[name] — Start the speed loop", cmd_start, 1, 1);
SHELL_SUBCMD_ADD((mc), stop, &dsub_mc_name, "[name] — Stop the speed loop", cmd_stop, 1, 1);
SHELL_SUBCMD_ADD((mc), reset, &dsub_mc_name, "[name] — Reset the speed PID state", cmd_reset, 1, 1);
SHELL_SUBCMD_ADD((mc), target, &dsub_mc_name, "[name] <rpm> — Set the speed setpoint (signed)",
		 cmd_target, 2, 1);
SHELL_SUBCMD_ADD((mc), speed, &mc_speed_cmds, "Tune the speed-loop gains", NULL, 0, 0);

SHELL_CMD_REGISTER(mc, &mc_cmds, "Motor speed control", NULL);
