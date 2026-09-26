/*
 * Copyright (c) 2026 Rtone
 * Copyright (c) 2026 Mathis Lécrivain <lecrivain.mathis@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT st_stm32_bldc_adc

#include <zephyr/device.h>
#include <zephyr/drivers/clock_control/stm32_clock_control.h>
#include <zephyr/init.h>
#include <zephyr/irq.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>
#include <soc.h>

#include <stm32_ll_adc.h>

#include <drivers/bldc_adc.h>

LOG_MODULE_REGISTER(bldc_adc_stm32, CONFIG_BLDC_ADC_LOG_LEVEL);

/** Per-channel sampling time, ~5.8 us at the ~42.5 MHz max ADC clock. */
#define BLDC_ADC_SAMPLING_TIME LL_ADC_SAMPLINGTIME_247CYCLES_5

/** Deadline for each blocking init step: calibration, ADRDY, VREFINT; a few us in practice. */
#define BLDC_ADC_INIT_TIMEOUT_US 1000U

/** Longest bldc_adc_read() waits for the next triggered sequence. */
#define BLDC_ADC_READ_TIMEOUT_US CONFIG_BLDC_ADC_STM32_READ_TIMEOUT_US

/**
 * @brief Injected rank of each channel, in channels order.
 */
static const uint32_t bldc_adc_ranks[BLDC_ADC_CHANNEL_COUNT] = {
	LL_ADC_INJ_RANK_1,
	LL_ADC_INJ_RANK_2,
	LL_ADC_INJ_RANK_3,
};

/**
 * @brief Per-instance device tree configuration.
 */
struct bldc_adc_stm32_config {
	ADC_TypeDef *adc;                         /**< ADC instance, the parent node. */
	struct stm32_pclken pclken;               /**< ADC bus clock gate. */
	uint8_t channels[BLDC_ADC_CHANNEL_COUNT]; /**< ADC channel numbers, in channels order. */
	uint32_t common_clock;                    /**< CKMODE: PCLK / st,adc-prescaler. */
	uint32_t inj_trigger;                     /**< JEXTSEL, the bridge timer's TRGO2. */
	uint32_t settle_ms;                       /**< Uptime to reach before measuring VREF+. */
	void (*irq_config)(void);                 /**< Connects and enables the ADC IRQ. */
};

/**
 * @brief Runtime state: the registered sample handler and the reference measured at init.
 */
struct bldc_adc_stm32_data {
	bldc_adc_sample_cb_t cb; /**< Sample handler, NULL while bldc_adc_read() owns JEOS. */
	void *user_data;         /**< Passed back to @ref cb. */
	uint32_t vref_mv;        /**< VREF+ (mV), measured once at init. */
};

/**
 * @brief Read the results of the injected sequence, one per rank.
 *
 * @param adc ADC instance.
 * @param out Output, conversion results in channels order.
 */
static void bldc_adc_read_ranks(ADC_TypeDef *adc, uint16_t out[BLDC_ADC_CHANNEL_COUNT])
{
	for (size_t i = 0; i < BLDC_ADC_CHANNEL_COUNT; i++) {
		out[i] = LL_ADC_INJ_ReadConversionData12(adc, bldc_adc_ranks[i]);
	}
}

/**
 * @brief Hand each completed injected sequence to the registered sample handler.
 *
 * @param arg bldc_adc device.
 */
static void bldc_adc_stm32_isr(const void *arg)
{
	const struct device *dev = arg;
	const struct bldc_adc_stm32_config *cfg = dev->config;
	struct bldc_adc_stm32_data *data = dev->data;
	uint16_t counts[BLDC_ADC_CHANNEL_COUNT];

	if (!LL_ADC_IsActiveFlag_JEOS(cfg->adc)) {
		return;
	}

	LL_ADC_ClearFlag_JEOS(cfg->adc);
	bldc_adc_read_ranks(cfg->adc, counts);

	if (data->cb != NULL) {
		data->cb(dev, counts, data->user_data);
	}
}

/**
 * @brief Wait for the next triggered sequence and return its results.
 *
 * @note JEOS is cleared first, so a sequence completed before the call is discarded.
 *
 * @param dev bldc_adc device.
 * @param counts Output, conversion results in channels order.
 *
 * @retval 0 on success.
 * @retval -EBUSY if a sample handler is registered; it owns JEOS.
 * @retval -ETIMEDOUT if no sequence completed, i.e. the bridge is not triggering.
 */
static int bldc_adc_stm32_read(const struct device *dev, uint16_t counts[BLDC_ADC_CHANNEL_COUNT])
{
	const struct bldc_adc_stm32_config *cfg = dev->config;
	struct bldc_adc_stm32_data *data = dev->data;

	if (data->cb != NULL) {
		return -EBUSY;
	}

	LL_ADC_ClearFlag_JEOS(cfg->adc);

	if (!WAIT_FOR(LL_ADC_IsActiveFlag_JEOS(cfg->adc), BLDC_ADC_READ_TIMEOUT_US, NULL)) {
		return -ETIMEDOUT;
	}

	LL_ADC_ClearFlag_JEOS(cfg->adc);
	bldc_adc_read_ranks(cfg->adc, counts);

	return 0;
}

/**
 * @brief Report VREF+ as measured at init.
 *
 * @param dev bldc_adc device.
 * @param vref_mv Output, VREF+ in millivolts.
 *
 * @retval 0 Always.
 */
static int bldc_adc_stm32_get_vref_mv(const struct device *dev, uint32_t *vref_mv)
{
	const struct bldc_adc_stm32_data *data = dev->data;

	*vref_mv = data->vref_mv;

	return 0;
}

/**
 * @brief Register the sample handler and arm or disarm the JEOS interrupt with it.
 *
 * @note The IRQ lock keeps the ISR from ever seeing a handler without its user data.
 *
 * @param dev bldc_adc device.
 * @param cb Handler called from the ISR, or NULL to hand JEOS back to bldc_adc_read().
 * @param user_data Passed back to @p cb.
 *
 * @retval 0 Always.
 */
static int bldc_adc_stm32_set_sample_handler(const struct device *dev, bldc_adc_sample_cb_t cb,
					     void *user_data)
{
	const struct bldc_adc_stm32_config *cfg = dev->config;
	struct bldc_adc_stm32_data *data = dev->data;
	unsigned int key = irq_lock();

	data->cb = cb;
	data->user_data = user_data;

	if (cb != NULL) {
		LL_ADC_EnableIT_JEOS(cfg->adc);
	} else {
		LL_ADC_DisableIT_JEOS(cfg->adc);
	}

	irq_unlock(key);

	return 0;
}

static DEVICE_API(bldc_adc, bldc_adc_stm32_driver_api) = {
	.read = bldc_adc_stm32_read,
	.get_vref_mv = bldc_adc_stm32_get_vref_mv,
	.set_sample_handler = bldc_adc_stm32_set_sample_handler,
};

/**
 * @brief Clock the ADC, power its analog block up and run the single-ended self-calibration.
 *
 * @param dev bldc_adc device.
 *
 * @return 0 on success, negative errno otherwise.
 */
static int bldc_adc_stm32_analog_init(const struct device *dev)
{
	const struct device *clk = DEVICE_DT_GET(STM32_CLOCK_CONTROL_NODE);
	const struct bldc_adc_stm32_config *cfg = dev->config;

	if (!device_is_ready(clk)) {
		return -ENODEV;
	}

	int ret = clock_control_on(clk, (clock_control_subsys_t)&cfg->pclken);
	if (ret != 0) {
		LOG_ERR("[%s] failed to enable ADC clock: %d", dev->name, ret);
		return ret;
	}

	LL_ADC_SetCommonClock(__LL_ADC_COMMON_INSTANCE(cfg->adc), cfg->common_clock);

	LL_ADC_DisableDeepPowerDown(cfg->adc);
	LL_ADC_EnableInternalRegulator(cfg->adc);
	k_busy_wait(LL_ADC_DELAY_INTERNAL_REGUL_STAB_US + 1U);

	LL_ADC_StartCalibration(cfg->adc, LL_ADC_SINGLE_ENDED);
	if (!WAIT_FOR(!LL_ADC_IsCalibrationOnGoing(cfg->adc), BLDC_ADC_INIT_TIMEOUT_US, NULL)) {
		return -ETIMEDOUT;
	}

	/* RM0440: wait LL_ADC_DELAY_CALIB_ENABLE_ADC_CYCLES ADC clock cycles
	 * before enabling the ADC. One microsecond covers it at any supported
	 * ADC clock.
	 */
	k_busy_wait(1U);

	return 0;
}

/**
 * @brief Program the injected sequence, one rank per channel, triggered on the TRGO2 rising edge.
 *
 * @param dev bldc_adc device.
 */
static void bldc_adc_stm32_inj_init(const struct device *dev)
{
	const struct bldc_adc_stm32_config *cfg = dev->config;

	LL_ADC_INJ_SetTriggerSource(cfg->adc, cfg->inj_trigger);
	LL_ADC_INJ_SetTriggerEdge(cfg->adc, LL_ADC_INJ_TRIG_EXT_RISING);
	LL_ADC_INJ_SetSequencerLength(cfg->adc, LL_ADC_INJ_SEQ_SCAN_ENABLE_3RANKS);

	for (size_t i = 0; i < BLDC_ADC_CHANNEL_COUNT; i++) {
		uint32_t channel = __LL_ADC_DECIMAL_NB_TO_CHANNEL(cfg->channels[i]);

		LL_ADC_INJ_SetSequencerRanks(cfg->adc, bldc_adc_ranks[i], channel);
		LL_ADC_SetChannelSamplingTime(cfg->adc, channel, BLDC_ADC_SAMPLING_TIME);
	}
}

/**
 * @brief Enable the ADC and wait until it is ready to convert.
 *
 * @param dev bldc_adc device.
 *
 * @retval 0 on success.
 * @retval -ETIMEDOUT if ADRDY does not rise.
 */
static int bldc_adc_stm32_enable(const struct device *dev)
{
	const struct bldc_adc_stm32_config *cfg = dev->config;

	LL_ADC_ClearFlag_ADRDY(cfg->adc);
	LL_ADC_Enable(cfg->adc);

	if (!WAIT_FOR(LL_ADC_IsActiveFlag_ADRDY(cfg->adc), BLDC_ADC_INIT_TIMEOUT_US, NULL)) {
		return -ETIMEDOUT;
	}

	return 0;
}

/**
 * @brief Sleep until the uptime reaches frontend-settle-ms.
 *
 * @note Measured from boot rather than per device, so several instances on the same supply pay it
 * once.
 *
 * @param dev bldc_adc device.
 */
static void bldc_adc_stm32_settle(const struct device *dev)
{
	const struct bldc_adc_stm32_config *cfg = dev->config;
	int64_t remaining = (int64_t)cfg->settle_ms - k_uptime_get();

	if (remaining > 0) {
		k_msleep((int32_t)remaining);
	}
}

/**
 * @brief Convert VREFINT once and derive VREF+ from its factory calibration.
 *
 * @param dev bldc_adc device.
 *
 * @retval 0 on success.
 * @retval -ETIMEDOUT if the conversion does not complete.
 */
static int bldc_adc_stm32_measure_vref(const struct device *dev)
{
	const struct bldc_adc_stm32_config *cfg = dev->config;
	struct bldc_adc_stm32_data *data = dev->data;

	k_busy_wait(LL_ADC_DELAY_VREFINT_STAB_US + 1U);

	LL_ADC_REG_SetSequencerLength(cfg->adc, LL_ADC_REG_SEQ_SCAN_DISABLE);
	LL_ADC_REG_SetSequencerRanks(cfg->adc, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_VREFINT);
	LL_ADC_SetChannelSamplingTime(cfg->adc, LL_ADC_CHANNEL_VREFINT, BLDC_ADC_SAMPLING_TIME);
	LL_ADC_REG_SetTriggerSource(cfg->adc, LL_ADC_REG_TRIG_SOFTWARE);

	LL_ADC_REG_StartConversion(cfg->adc);
	if (!WAIT_FOR(LL_ADC_IsActiveFlag_EOC(cfg->adc), BLDC_ADC_INIT_TIMEOUT_US, NULL)) {
		return -ETIMEDOUT;
	}

	data->vref_mv = __LL_ADC_CALC_VREFANALOG_VOLTAGE(LL_ADC_REG_ReadConversionData12(cfg->adc),
							 LL_ADC_RESOLUTION_12B);

	return 0;
}

/**
 * @brief Bring the ADC up with its injected sequence armed on the bridge's sync trigger.
 *
 * @param dev bldc_adc device.
 *
 * @return 0 on success, negative errno otherwise.
 */
static int bldc_adc_stm32_init(const struct device *dev)
{
	const struct bldc_adc_stm32_config *cfg = dev->config;
	int ret;

	ret = bldc_adc_stm32_analog_init(dev);
	if (ret != 0) {
		return ret;
	}

	bldc_adc_stm32_inj_init(dev);

	LL_ADC_SetCommonPathInternalCh(__LL_ADC_COMMON_INSTANCE(cfg->adc),
				       LL_ADC_PATH_INTERNAL_VREFINT);

	ret = bldc_adc_stm32_enable(dev);
	if (ret != 0) {
		return ret;
	}

	bldc_adc_stm32_settle(dev);

	ret = bldc_adc_stm32_measure_vref(dev);
	if (ret != 0) {
		return ret;
	}

	LL_ADC_ClearFlag_JEOS(cfg->adc);
	LL_ADC_INJ_StartConversion(cfg->adc);

	/* Connects the NVIC line, but JEOS itself stays masked until a
	 * consumer registers a sample handler: until then, bldc_adc_read()
	 * polls and owns the flag.
	 */
	cfg->irq_config();

	LOG_INF("channels %u,%u,%u, VREF+ %u mV", cfg->channels[0], cfg->channels[1],
		cfg->channels[2], ((struct bldc_adc_stm32_data *)dev->data)->vref_mv);

	return 0;
}

/** The hardware lives on the parent st,stm32-adc node. */
#define BLDC_ADC_ADC(idx) DT_INST_PARENT(idx)

/** Timer node of the bridge named by the pwm phandle. */
#define BLDC_ADC_BRIDGE_TIMER(idx) DT_PARENT(DT_INST_PHANDLE(idx, pwm))

/** JEXTSEL selecting the TRGO2 of the bridge timer, TIM1 or TIM8. */
#define BLDC_ADC_INJ_TRIGGER(idx)                                                                  \
	(DT_REG_ADDR(BLDC_ADC_BRIDGE_TIMER(idx)) == TIM1_BASE ? LL_ADC_INJ_TRIG_EXT_TIM1_TRGO2     \
							      : LL_ADC_INJ_TRIG_EXT_TIM8_TRGO2)

/** Synchronous ADC clock, PCLK divided by st,adc-prescaler. */
#define BLDC_ADC_COMMON_CLOCK(idx)                                                                 \
	CONCAT(LL_ADC_CLOCK_SYNC_PCLK_DIV, DT_PROP(BLDC_ADC_ADC(idx), st_adc_prescaler))

/** Instantiate one bldc_adc device from DT instance @p idx. */
#define BLDC_ADC_INST(idx)                                                                         \
	BUILD_ASSERT(DT_ENUM_IDX(BLDC_ADC_ADC(idx), st_adc_clock_source) == 0,                     \
		     "bldc_adc only supports the SYNC ADC clock source");                          \
	BUILD_ASSERT(DT_PROP(BLDC_ADC_ADC(idx), st_adc_prescaler) == 1 ||                          \
			     DT_PROP(BLDC_ADC_ADC(idx), st_adc_prescaler) == 2 ||                  \
			     DT_PROP(BLDC_ADC_ADC(idx), st_adc_prescaler) == 4,                    \
		     "bldc_adc requires a SYNC ADC clock prescaler of 1, 2 or 4");                 \
	BUILD_ASSERT(DT_REG_ADDR(BLDC_ADC_BRIDGE_TIMER(idx)) == TIM1_BASE ||                       \
			     DT_REG_ADDR(BLDC_ADC_BRIDGE_TIMER(idx)) == TIM8_BASE,                 \
		     "bldc_adc requires a TIM1 or TIM8 bridge timer");                             \
	BUILD_ASSERT(DT_PROP(DT_INST_PHANDLE(idx, pwm), sync_trigger_enable),                      \
		     "bldc_adc requires the bridge's sync trigger to be enabled");                 \
	BUILD_ASSERT(DT_INST_PROP_LEN(idx, channels) == BLDC_ADC_CHANNEL_COUNT,                    \
		     "bldc_adc requires exactly " STRINGIFY(BLDC_ADC_CHANNEL_COUNT) " channels");  \
                                                                                                   \
	static void bldc_adc_irq_config_##idx(void)                                                \
	{                                                                                          \
		IRQ_CONNECT(DT_IRQN(BLDC_ADC_ADC(idx)), DT_IRQ(BLDC_ADC_ADC(idx), priority),       \
			    bldc_adc_stm32_isr, DEVICE_DT_INST_GET(idx), 0);                       \
		irq_enable(DT_IRQN(BLDC_ADC_ADC(idx)));                                            \
	}                                                                                          \
                                                                                                   \
	static const struct bldc_adc_stm32_config bldc_adc_config_##idx = {                        \
		.adc = (ADC_TypeDef *)DT_REG_ADDR(BLDC_ADC_ADC(idx)),                              \
		.pclken =                                                                          \
			{                                                                          \
				.bus = DT_CLOCKS_CELL(BLDC_ADC_ADC(idx), bus),                     \
				.enr = DT_CLOCKS_CELL(BLDC_ADC_ADC(idx), bits),                    \
			},                                                                         \
		.channels = DT_INST_PROP(idx, channels),                                           \
		.common_clock = BLDC_ADC_COMMON_CLOCK(idx),                                        \
		.inj_trigger = BLDC_ADC_INJ_TRIGGER(idx),                                          \
		.settle_ms = DT_INST_PROP(idx, frontend_settle_ms),                                \
		.irq_config = bldc_adc_irq_config_##idx,                                           \
	};                                                                                         \
                                                                                                   \
	static struct bldc_adc_stm32_data bldc_adc_data_##idx;                                     \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(idx, bldc_adc_stm32_init, NULL, &bldc_adc_data_##idx,                \
			      &bldc_adc_config_##idx, POST_KERNEL, CONFIG_BLDC_ADC_INIT_PRIORITY,  \
			      &bldc_adc_stm32_driver_api);

DT_INST_FOREACH_STATUS_OKAY(BLDC_ADC_INST)
