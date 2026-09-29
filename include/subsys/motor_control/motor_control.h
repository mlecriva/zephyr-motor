/*
 * Copyright (c) 2026 Rtone
 * Copyright (c) 2026 Mathis Lécrivain <lecrivain.mathis@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_APPS_INCLUDE_SUBSYS_MOTOR_CONTROL_MOTOR_CONTROL_H_
#define ZEPHYR_APPS_INCLUDE_SUBSYS_MOTOR_CONTROL_MOTOR_CONTROL_H_

#include <stdint.h>

#include <zephyr/device.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Start the speed loop.
 *
 * Resets the setpoint to 0 RPM and the speed PID, starts the motor, then
 * schedules the first regulation tick. Idempotent: a running controller is left
 * as it is.
 *
 * @param dev motor controller device.
 *
 * @retval 0 on success, or if already running.
 * @retval -EINVAL if @p dev is NULL.
 * @retval -errno Other negative errno code from the motor.
 */
int motor_control_start(const struct device *dev);

/**
 * @brief Stop the speed loop and release the motor.
 *
 * Cancels the regulation tick, waiting for one in flight, applies a zero
 * voltage so the next start does not reuse the last one, then stops the motor.
 * Idempotent.
 *
 * @param dev motor controller device.
 *
 * @retval 0 on success, or if already stopped.
 * @retval -EINVAL if @p dev is NULL.
 * @retval -errno Other negative errno code from stopping the motor.
 */
int motor_control_stop(const struct device *dev);

/**
 * @brief Reset the speed PID.
 *
 * Clears its integral accumulator and previous error, e.g. after a large
 * setpoint change. Gains, limits, period and setpoint are kept.
 *
 * @param dev motor controller device.
 *
 * @retval 0 on success.
 * @retval -EINVAL if @p dev is NULL.
 */
int motor_control_reset(const struct device *dev);

/**
 * @brief Update the speed setpoint.
 *
 * The ramp moves toward it from the next regulation tick. Starting the
 * controller resets it to 0 RPM, so a setpoint set while stopped is lost.
 *
 * @param dev motor controller device.
 * @param rpm Target shaft speed in RPM, signed: the sign sets the direction.
 *
 * @retval 0 on success.
 * @retval -EINVAL if @p dev is NULL.
 */
int motor_control_set_target_rpm(const struct device *dev, int32_t rpm);

/**
 * @brief Update the proportional gain of the speed loop.
 *
 * Applies from the next regulation tick; the other gains and the loop state are
 * kept.
 *
 * @param dev motor controller device.
 * @param kp Gain, in per-mille of the supply voltage per RPM of error.
 *
 * @retval 0 on success.
 * @retval -EINVAL if @p dev is NULL.
 */
int motor_control_set_speed_kp(const struct device *dev, float kp);

/**
 * @brief Update the integral gain of the speed loop.
 *
 * Applies from the next regulation tick; the other gains and the loop state are
 * kept.
 *
 * @param dev motor controller device.
 * @param ki Gain, in per-mille of the supply voltage per RPM of error and per
 *        second.
 *
 * @retval 0 on success.
 * @retval -EINVAL if @p dev is NULL.
 */
int motor_control_set_speed_ki(const struct device *dev, float ki);

/**
 * @brief Update the derivative gain of the speed loop.
 *
 * Applies from the next regulation tick; the other gains and the loop state are
 * kept.
 *
 * @param dev motor controller device.
 * @param kd Gain, in per-mille of the supply voltage per RPM/s of error change.
 *
 * @retval 0 on success.
 * @retval -EINVAL if @p dev is NULL.
 */
int motor_control_set_speed_kd(const struct device *dev, float kd);

/**
 * @brief Read the gains of the speed loop.
 *
 * @param dev motor controller device.
 * @param kp Output, the proportional gain; may be NULL.
 * @param ki Output, the integral gain; may be NULL.
 * @param kd Output, the derivative gain; may be NULL.
 *
 * @retval 0 on success.
 * @retval -EINVAL if @p dev is NULL.
 */
int motor_control_get_speed_gains(const struct device *dev, float *kp, float *ki, float *kd);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_APPS_INCLUDE_SUBSYS_MOTOR_CONTROL_MOTOR_CONTROL_H_ */
