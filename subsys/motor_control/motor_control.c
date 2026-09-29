/*
 * Copyright (c) 2026 Rtone
 * Copyright (c) 2026 Mathis Lécrivain <lecrivain.mathis@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT zephyr_motor_control

#include <errno.h>
#include <stdbool.h>
#include <stdint.h>

#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <drivers/motor.h>
#include <lib/pid.h>
#include <lib/ramp.h>
#include <subsys/motor_control/motor_control.h>

LOG_MODULE_REGISTER(motor_control, CONFIG_MOTOR_CONTROL_LOG_LEVEL);

/** Full-scale speed-loop output, in per-mille of the supply voltage. */
#define MC_VOLTAGE_FULL_SCALE_MILLI 1000

/**
 * @brief Per-instance device tree configuration.
 */
struct mc_config {
	const struct device *motor;              /**< Motor driven. */
	const struct device *motor_sensor;       /**< Sensor serving the motor speed. */
	const struct pid_config *speed_init_cfg; /**< Speed PID applied at init. */
	const struct ramp_config *ramp_cfg;      /**< Setpoint ramp applied at init. */
	uint32_t control_period_ms;              /**< Regulation period. */
};

/**
 * @brief Runtime state.
 */
struct mc_data {
	const struct device *dev; /**< Back-pointer for the timer and work handlers. */
	struct k_mutex lock;      /**< Serializes the tick against the API. */
	struct ramp ramp;         /**< Requested setpoint in, ramped setpoint out, in RPM. */
	struct pid speed_pid;     /**< RPM in, per-mille of the supply voltage out. */
	bool running;             /**< Between a start and a stop. */
	struct k_timer timer;     /**< Fires once per regulation period. */
	struct k_work work;       /**< Regulation tick, on @ref mc_workq. */
};

/** Stack of @ref mc_workq. */
static K_THREAD_STACK_DEFINE(mc_workq_stack, CONFIG_MOTOR_CONTROL_WORKQ_STACK_SIZE);

/** Work queue running the regulation tick of every controller. */
static struct k_work_q mc_workq;

/**
 * @brief Fetch one channel of a sensor and read it as a float.
 *
 * @param sensor Sensor device.
 * @param chan Channel to fetch and read.
 * @param out Output, the channel value; untouched on failure.
 *
 * @retval 0 on success.
 * @retval -errno Negative errno code from the sensor.
 */
static int mc_read_channel(const struct device *sensor, enum sensor_channel chan, float *out)
{
	struct sensor_value val;
	int ret;

	ret = sensor_sample_fetch_chan(sensor, chan);
	if (ret < 0) {
		return ret;
	}

	ret = sensor_channel_get(sensor, chan, &val);
	if (ret < 0) {
		return ret;
	}

	*out = sensor_value_to_float(&val);
	return 0;
}

/**
 * @brief Convert a speed-loop output to a motor voltage.
 *
 * @note +/-1000 maps to +/-INT32_MAX, never to INT32_MIN; the output limits are checked at build
 * time to stay within that range.
 *
 * @param milli Voltage in per-mille of the supply, -1000..1000.
 *
 * @return The voltage in Q31.
 */
static q31_t mc_milli_to_q31(int32_t milli)
{
	return (q31_t)(((int64_t)milli * INT32_MAX) / MC_VOLTAGE_FULL_SCALE_MILLI);
}

/**
 * @brief Regulation tick: ramp the setpoint, then close the speed loop.
 *
 * @note A failed speed read or PID update applies a zero voltage, which brakes the rotor. Bails
 * out if a stop raced with the tick. Holds the lock for the whole tick.
 *
 * @param work @ref mc_data::work of the controller.
 */
static void mc_work_handler(struct k_work *work)
{
	struct mc_data *data = CONTAINER_OF(work, struct mc_data, work);
	const struct device *dev = data->dev;
	const struct mc_config *cfg = dev->config;
	float setpoint_rpm;
	float measured_rpm;
	float voltage_milli = 0.0f;
	int ret;

	k_mutex_lock(&data->lock, K_FOREVER);

	if (!data->running) {
		k_mutex_unlock(&data->lock);
		return;
	}

	(void)ramp_step(&data->ramp, &setpoint_rpm);

	/* A failed read must not reach the PID as 0 RPM: on a spinning rotor the loop would see a
	 * huge error and command full voltage.
	 */
	ret = mc_read_channel(cfg->motor_sensor, SENSOR_CHAN_RPM, &measured_rpm);
	if (ret < 0) {
		LOG_WRN("[%s] Failed to read the speed: %d", dev->name, ret);
		goto apply;
	}

	ret = pid_update(&data->speed_pid, setpoint_rpm, measured_rpm, &voltage_milli);
	if (ret < 0) {
		LOG_WRN("[%s] Speed PID update failed: %d", dev->name, ret);
		voltage_milli = 0.0f;
	}

apply:
	ret = motor_set_voltage(cfg->motor, mc_milli_to_q31((int32_t)voltage_milli));
	if (ret < 0) {
		LOG_WRN("[%s] Failed to set the motor voltage: %d", dev->name, ret);
	}

	k_mutex_unlock(&data->lock);
}

/**
 * @brief Regulation timer expiry (ISR): hand the tick to the work queue.
 *
 * @param timer @ref mc_data::timer of the controller.
 */
static void mc_timer_expiry(struct k_timer *timer)
{
	struct mc_data *data = CONTAINER_OF(timer, struct mc_data, timer);

	(void)k_work_submit_to_queue(&mc_workq, &data->work);
}

int motor_control_start(const struct device *dev)
{
	if (dev == NULL) {
		return -EINVAL;
	}

	const struct mc_config *cfg = dev->config;
	struct mc_data *data = dev->data;
	int ret;

	k_mutex_lock(&data->lock, K_FOREVER);

	if (data->running) {
		LOG_WRN("[%s] Already started", dev->name);
		k_mutex_unlock(&data->lock);
		return 0;
	}

	(void)ramp_reset(&data->ramp, 0.0f);
	(void)pid_reset(&data->speed_pid);

	ret = motor_start(cfg->motor);
	if (ret < 0) {
		LOG_ERR("[%s] Failed to start the motor: %d", dev->name, ret);
		k_mutex_unlock(&data->lock);
		return ret;
	}

	data->running = true;
	k_timer_start(&data->timer, K_MSEC(cfg->control_period_ms), K_MSEC(cfg->control_period_ms));

	k_mutex_unlock(&data->lock);

	return 0;
}

int motor_control_stop(const struct device *dev)
{
	if (dev == NULL) {
		return -EINVAL;
	}

	const struct mc_config *cfg = dev->config;
	struct mc_data *data = dev->data;
	int ret;

	k_mutex_lock(&data->lock, K_FOREVER);

	if (!data->running) {
		k_mutex_unlock(&data->lock);
		return 0;
	}
	data->running = false;

	k_mutex_unlock(&data->lock);

	/* Outside the lock: a tick in flight needs it to bail out. */
	k_timer_stop(&data->timer);
	if (k_work_cancel_sync(&data->work, &(struct k_work_sync){0})) {
		LOG_WRN("[%s] Regulation tick still pending at stop", dev->name);
	}

	k_mutex_lock(&data->lock, K_FOREVER);

	/* The motor keeps its voltage across a stop: zero it, so the next start begins at rest. */
	ret = motor_set_voltage(cfg->motor, 0);
	if (ret < 0) {
		LOG_WRN("[%s] Failed to zero the motor voltage: %d", dev->name, ret);
	}

	ret = motor_stop(cfg->motor);
	if (ret < 0) {
		LOG_WRN("[%s] Failed to stop the motor: %d", dev->name, ret);
	}

	k_mutex_unlock(&data->lock);

	return ret;
}

int motor_control_reset(const struct device *dev)
{
	if (dev == NULL) {
		return -EINVAL;
	}

	struct mc_data *data = dev->data;

	k_mutex_lock(&data->lock, K_FOREVER);
	(void)pid_reset(&data->speed_pid);
	k_mutex_unlock(&data->lock);

	return 0;
}

int motor_control_set_target_rpm(const struct device *dev, int32_t rpm)
{
	if (dev == NULL) {
		return -EINVAL;
	}

	struct mc_data *data = dev->data;

	k_mutex_lock(&data->lock, K_FOREVER);
	(void)ramp_set_target(&data->ramp, (float)rpm);
	k_mutex_unlock(&data->lock);

	return 0;
}

int motor_control_set_speed_kp(const struct device *dev, float kp)
{
	if (dev == NULL) {
		return -EINVAL;
	}

	struct mc_data *data = dev->data;

	k_mutex_lock(&data->lock, K_FOREVER);
	(void)pid_set_kp(&data->speed_pid, kp);
	k_mutex_unlock(&data->lock);

	return 0;
}

int motor_control_set_speed_ki(const struct device *dev, float ki)
{
	if (dev == NULL) {
		return -EINVAL;
	}

	struct mc_data *data = dev->data;

	k_mutex_lock(&data->lock, K_FOREVER);
	(void)pid_set_ki(&data->speed_pid, ki);
	k_mutex_unlock(&data->lock);

	return 0;
}

int motor_control_set_speed_kd(const struct device *dev, float kd)
{
	if (dev == NULL) {
		return -EINVAL;
	}

	struct mc_data *data = dev->data;

	k_mutex_lock(&data->lock, K_FOREVER);
	(void)pid_set_kd(&data->speed_pid, kd);
	k_mutex_unlock(&data->lock);

	return 0;
}

int motor_control_get_speed_gains(const struct device *dev, float *kp, float *ki, float *kd)
{
	if (dev == NULL) {
		return -EINVAL;
	}

	struct mc_data *data = dev->data;

	k_mutex_lock(&data->lock, K_FOREVER);
	if (kp != NULL) {
		*kp = data->speed_pid.cfg.kp;
	}
	if (ki != NULL) {
		*ki = data->speed_pid.cfg.ki;
	}
	if (kd != NULL) {
		*kd = data->speed_pid.cfg.kd;
	}
	k_mutex_unlock(&data->lock);

	return 0;
}

/**
 * @brief Check that the motor and its sensor are ready, and set up the speed loop.
 *
 * @param dev motor controller device.
 *
 * @retval 0 on success.
 * @retval -ENODEV if the motor or the sensor is not ready.
 * @retval -EINVAL if the ramp or the speed PID configuration is invalid.
 */
static int mc_init(const struct device *dev)
{
	const struct mc_config *cfg = dev->config;
	struct mc_data *data = dev->data;
	int ret;

	if (!device_is_ready(cfg->motor)) {
		LOG_ERR("[%s] Motor %s not ready", dev->name, cfg->motor->name);
		return -ENODEV;
	}

	if (!device_is_ready(cfg->motor_sensor)) {
		LOG_ERR("[%s] Sensor %s not ready", dev->name, cfg->motor_sensor->name);
		return -ENODEV;
	}

	ret = ramp_init(&data->ramp, cfg->ramp_cfg);
	if (ret < 0) {
		LOG_ERR("[%s] Invalid ramp configuration: %d", dev->name, ret);
		return ret;
	}

	ret = pid_init(&data->speed_pid, cfg->speed_init_cfg);
	if (ret < 0) {
		LOG_ERR("[%s] Invalid speed PID configuration: %d", dev->name, ret);
		return ret;
	}

	data->dev = dev;
	k_mutex_init(&data->lock);
	k_timer_init(&data->timer, mc_timer_expiry, NULL);
	k_work_init(&data->work, mc_work_handler);

	LOG_DBG("[%s] Ready (motor %s, sensor %s, period %u ms)", dev->name, cfg->motor->name,
		cfg->motor_sensor->name, cfg->control_period_ms);

	return 0;
}

/**
 * @brief Start the work queue shared by every controller, before any controller init.
 *
 * @retval 0 Always.
 */
static int mc_workq_init(void)
{
	const struct k_work_queue_config cfg = {
		.name = "motor_control",
		.no_yield = false,
	};

	k_work_queue_start(&mc_workq, mc_workq_stack, K_THREAD_STACK_SIZEOF(mc_workq_stack),
			   CONFIG_MOTOR_CONTROL_WORKQ_PRIORITY, &cfg);

	return 0;
}

SYS_INIT(mc_workq_init, POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);

/**
 * @brief Read a devicetree int property as a signed float.
 *
 * @note Devicetree emits cells as unsigned values, so a negative literal such as <(-1000)> reaches
 * C as its two's complement: the cast through int32_t restores the sign.
 */
#define MC_DT_SIGNED(inst, prop) ((int32_t)DT_INST_PROP(inst, prop))

/** Devicetree int property @p prop of instance @p inst, divided by @p divisor, as a float. */
#define MC_DT_SIGNED_FLOAT(inst, prop, divisor) ((float)MC_DT_SIGNED(inst, prop) / (float)(divisor))

/** Regulation period of instance @p inst, in seconds. */
#define MC_DT_PERIOD_S(inst) ((float)DT_INST_PROP(inst, control_period_ms) / 1000.0f)

/** Instantiate one motor controller from DT instance @p inst. */
#define MC_INST_DEFINE(inst)                                                                       \
	BUILD_ASSERT(DT_NODE_HAS_COMPAT(DT_INST_PHANDLE(inst, motor), zephyr_motor),               \
		     "instance " #inst ": motor must be a \"zephyr,motor\" node");                 \
	BUILD_ASSERT(-MC_VOLTAGE_FULL_SCALE_MILLI <= MC_DT_SIGNED(inst, output_min_milli) &&       \
			     MC_DT_SIGNED(inst, output_min_milli) <=                               \
				     MC_DT_SIGNED(inst, output_max_milli) &&                       \
			     MC_DT_SIGNED(inst, output_max_milli) <= MC_VOLTAGE_FULL_SCALE_MILLI,  \
		     "instance " #inst ": output limits must hold -1000 <= min <= max <= 1000");   \
                                                                                                   \
	static const struct pid_config mc_speed_cfg_##inst = {                                     \
		.kp = MC_DT_SIGNED_FLOAT(inst, speed_pid_kp_micro, 1000000),                       \
		.ki = MC_DT_SIGNED_FLOAT(inst, speed_pid_ki_micro, 1000000),                       \
		.kd = MC_DT_SIGNED_FLOAT(inst, speed_pid_kd_micro, 1000000),                       \
		.dt = MC_DT_PERIOD_S(inst),                                                        \
		.output_min = MC_DT_SIGNED_FLOAT(inst, output_min_milli, 1),                       \
		.output_max = MC_DT_SIGNED_FLOAT(inst, output_max_milli, 1),                       \
		.integral_min = MC_DT_SIGNED_FLOAT(inst, speed_pid_integral_min_milli, 1000),      \
		.integral_max = MC_DT_SIGNED_FLOAT(inst, speed_pid_integral_max_milli, 1000),      \
	};                                                                                         \
                                                                                                   \
	static const struct ramp_config mc_ramp_cfg_##inst = {                                     \
		.accel = (float)DT_INST_PROP(inst, ramp_acceleration_rpm_per_s),                   \
		.decel = (float)DT_INST_PROP_OR(inst, ramp_deceleration_rpm_per_s,                 \
						DT_INST_PROP(inst, ramp_acceleration_rpm_per_s)),  \
		.dt = MC_DT_PERIOD_S(inst),                                                        \
	};                                                                                         \
                                                                                                   \
	static const struct mc_config mc_cfg_##inst = {                                            \
		.motor = DEVICE_DT_GET(DT_INST_PHANDLE(inst, motor)),                              \
		.motor_sensor = DEVICE_DT_GET(DT_INST_PHANDLE(inst, motor_sensor)),                \
		.speed_init_cfg = &mc_speed_cfg_##inst,                                            \
		.ramp_cfg = &mc_ramp_cfg_##inst,                                                   \
		.control_period_ms = DT_INST_PROP(inst, control_period_ms),                        \
	};                                                                                         \
                                                                                                   \
	static struct mc_data mc_data_##inst;                                                      \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(inst, mc_init, NULL, &mc_data_##inst, &mc_cfg_##inst, POST_KERNEL,   \
			      CONFIG_MOTOR_CONTROL_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(MC_INST_DEFINE)
