/*
 * Copyright (c) 2026 Rtone
 * Copyright (c) 2026 Mathis Lécrivain <lecrivain.mathis@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT st_stm32_bldc_hall_xor

#include <zephyr/device.h>
#include <zephyr/drivers/clock_control/stm32_clock_control.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/init.h>
#include <zephyr/irq.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include <stm32_ll_tim.h>

#include <drivers/bldc_hall.h>

LOG_MODULE_REGISTER(bldc_hall_xor_stm32, CONFIG_BLDC_HALL_LOG_LEVEL);

/** The XOR toggles on every commutation, hence six captures per revolution. */
#define BLDC_HALL_XOR_EDGES_PER_REV 6U

/**
 * @brief Per-instance device tree configuration.
 */
struct bldc_hall_xor_stm32_config {
	TIM_TypeDef *timer;                              /**< Timer fed by the three lines. */
	struct stm32_pclken pclken;                      /**< Timer clock gate. */
	uint32_t prescaler;                              /**< PSC, from the parent timer node. */
	const struct pinctrl_dev_config *pcfg;           /**< Pin configuration of the lines. */
	struct gpio_dt_spec lines[BLDC_HALL_LINE_COUNT]; /**< Lines in U/V/W order. */
	void (*irq_config)(void);                        /**< Connects the timer NVIC line. */
};

/**
 * @brief Runtime state.
 *
 */
struct bldc_hall_xor_stm32_data {
	uint32_t counter_freq_hz; /**< Counter clock, i.e. the unit of a capture. */
	bldc_hall_edge_cb_t cb;   /**< Consumer edge callback, NULL while unregistered. */
	void *user_data;          /**< Opaque argument of @ref cb. */
	bool timed; /**< Cleared on overflow, set on capture: false means CNT is meaningless. */
};

/**
 * @brief Counter width, i.e. the ARR to load on a free-running counter.
 *
 * @param tim Timer instance.
 *
 * @return UINT32_MAX on a 32-bit counter, UINT16_MAX otherwise.
 */
static inline uint32_t bldc_hall_xor_counter_max(const TIM_TypeDef *tim)
{
	return IS_TIM_32B_COUNTER_INSTANCE(tim) ? UINT32_MAX : UINT16_MAX;
}

/**
 * @brief Sample the three Hall lines into a U/V/W bitfield.
 *
 * @param cfg Instance configuration.
 * @param state Output, bit 2 = U, bit 1 = V, bit 0 = W.
 *
 * @return 0 on success, negative errno otherwise.
 */
static int bldc_hall_xor_read_lines(const struct bldc_hall_xor_stm32_config *cfg, uint8_t *state)
{
	uint8_t value = 0;

	for (int i = 0; i < BLDC_HALL_LINE_COUNT; i++) {
		int level = gpio_pin_get_raw(cfg->lines[i].port, cfg->lines[i].pin);
		if (level < 0) {
			return level;
		}

		if (level > 0) {
			value |= BIT(2 - i);
		}
	}

	*state = value;

	return 0;
}

/**
 * @brief Timer interrupt: report a commutation, or invalidate the counter.
 *
 * @param arg bldc_hall device, passed by the per-instance IRQ thunk.
 */
static void bldc_hall_xor_stm32_isr(const void *arg)
{
	const struct device *dev = arg;
	const struct bldc_hall_xor_stm32_config *cfg = dev->config;
	struct bldc_hall_xor_stm32_data *data = dev->data;
	TIM_TypeDef *tim = cfg->timer;

	if (LL_TIM_IsActiveFlag_UPDATE(tim)) {
		LL_TIM_ClearFlag_UPDATE(tim);
		/* URS = 1, so this is a real overflow: no edge arrived within a full counter span
		 * and CNT no longer measures anything. Handled before the capture, since an edge
		 * pending along with it came after the overflow.
		 */
		data->timed = false;
	}

	if (LL_TIM_IsActiveFlag_CC1(tim)) {
		/* Reading CCR1 clears CC1IF. */
		uint32_t capture = LL_TIM_IC_GetCaptureCH1(tim);
		/* Before the first edge or after an overflow, the capture is no interval. */
		uint32_t period = data->timed ? capture : 0U;
		uint8_t state = 0;

		data->timed = true;

		if (data->cb != NULL && bldc_hall_xor_read_lines(cfg, &state) == 0) {
			data->cb(dev, state, period, data->user_data);
		}
	}
}

/**
 * @brief Read the current Hall state.
 *
 * @param dev bldc_hall device.
 * @param state Output, bit 2 = U, bit 1 = V, bit 0 = W.
 *
 * @return 0 on success, negative errno otherwise.
 */
static int bldc_hall_xor_stm32_get_state(const struct device *dev, uint8_t *state)
{
	return bldc_hall_xor_read_lines(dev->config, state);
}

/**
 * @brief Report the clock the captures and elapsed ticks are counted in.
 *
 * @param dev bldc_hall device.
 * @param freq_hz Output, counter clock in Hz.
 *
 * @retval 0 Always; the value is fixed at init.
 */
static int bldc_hall_xor_stm32_get_counter_freq_hz(const struct device *dev, uint32_t *freq_hz)
{
	const struct bldc_hall_xor_stm32_data *data = dev->data;

	*freq_hz = data->counter_freq_hz;

	return 0;
}

/**
 * @brief Report the time since the last commutation.
 *
 * @param dev bldc_hall device.
 * @param ticks Output, counter ticks, UINT32_MAX once an overflow made the
 *              counter meaningless.
 *
 * @retval 0 Always.
 */
static int bldc_hall_xor_stm32_get_elapsed_ticks(const struct device *dev, uint32_t *ticks)
{
	const struct bldc_hall_xor_stm32_config *cfg = dev->config;
	const struct bldc_hall_xor_stm32_data *data = dev->data;

	/* Slave-mode RESET restarts the counter on every edge, so CNT is the time since the last
	 * one.
	 */
	*ticks = data->timed ? LL_TIM_GetCounter(cfg->timer) : UINT32_MAX;

	return 0;
}

/**
 * @brief Report how many timed edges make one electrical revolution.
 *
 * @param dev bldc_hall device.
 * @param edges_per_rev Output, timed edges per electrical revolution.
 *
 * @retval 0 Always; the XOR gives all six commutations.
 */
static int bldc_hall_xor_stm32_get_edges_per_rev(const struct device *dev, uint8_t *edges_per_rev)
{
	ARG_UNUSED(dev);

	*edges_per_rev = BLDC_HALL_XOR_EDGES_PER_REV;

	return 0;
}

/**
 * @brief Register the edge consumer, and arm the interrupts for it.
 *
 * @param dev bldc_hall device.
 * @param cb Callback invoked on every commutation, NULL to unregister and mask
 *           the interrupts again.
 * @param user_data Opaque argument handed back to @p cb.
 *
 * @retval 0 Always.
 */
static int bldc_hall_xor_stm32_set_edge_handler(const struct device *dev, bldc_hall_edge_cb_t cb,
						void *user_data)
{
	const struct bldc_hall_xor_stm32_config *cfg = dev->config;
	struct bldc_hall_xor_stm32_data *data = dev->data;
	TIM_TypeDef *tim = cfg->timer;

	/* Publish the pair atomically against the ISR. */
	unsigned int key = irq_lock();
	data->cb = cb;
	data->user_data = user_data;
	irq_unlock(key);

	if (cb != NULL) {
		LL_TIM_EnableIT_CC1(tim);
		LL_TIM_EnableIT_UPDATE(tim);
	} else {
		LL_TIM_DisableIT_CC1(tim);
		LL_TIM_DisableIT_UPDATE(tim);
	}

	return 0;
}

static DEVICE_API(bldc_hall, bldc_hall_xor_stm32_driver_api) = {
	.get_state = bldc_hall_xor_stm32_get_state,
	.get_counter_freq_hz = bldc_hall_xor_stm32_get_counter_freq_hz,
	.get_elapsed_ticks = bldc_hall_xor_stm32_get_elapsed_ticks,
	.get_edges_per_rev = bldc_hall_xor_stm32_get_edges_per_rev,
	.set_edge_handler = bldc_hall_xor_stm32_set_edge_handler,
};

/**
 * @brief Enable the timer clock and derive the counter frequency from it.
 *
 * @param dev bldc_hall device.
 *
 * @return 0 on success, negative errno otherwise.
 */
static int bldc_hall_xor_stm32_clock_init(const struct device *dev)
{
	const struct bldc_hall_xor_stm32_config *cfg = dev->config;
	struct bldc_hall_xor_stm32_data *data = dev->data;
	const struct device *clk = DEVICE_DT_GET(STM32_CLOCK_CONTROL_NODE);
	uint32_t timer_clk_hz = 0;

	if (!device_is_ready(clk)) {
		return -ENODEV;
	}

	int ret = clock_control_on(clk, (clock_control_subsys_t)&cfg->pclken);
	if (ret < 0) {
		LOG_ERR("[%s] failed to enable timer clock: %d", dev->name, ret);
		return ret;
	}

	ret = clock_control_get_rate(clk, (clock_control_subsys_t)&cfg->pclken, &timer_clk_hz);
	if (ret < 0) {
		LOG_ERR("[%s] failed to read timer clock rate: %d", dev->name, ret);
		return ret;
	}

	data->counter_freq_hz = timer_clk_hz / (cfg->prescaler + 1U);

	return 0;
}

/**
 * @brief Wire the XOR into a reset-on-edge capture, and start the counter.
 *
 * @param dev bldc_hall device.
 */
static void bldc_hall_xor_stm32_timer_init(const struct device *dev)
{
	const struct bldc_hall_xor_stm32_config *cfg = dev->config;
	TIM_TypeDef *tim = cfg->timer;

	/* Hall sensor XOR mode: CH1/CH2/CH3 XOR feeds TI1. */
	SET_BIT(tim->CR2, TIM_CR2_TI1S);

	/* AN4013 4.3.4: the prescaler must yield a counter period longer than the maximum time
	 * between two Hall edges at the slowest expected RPM, otherwise the capture overflows and
	 * the speed reading wraps. It comes from the parent timer node, sized for the target motor.
	 */
	LL_TIM_SetPrescaler(tim, cfg->prescaler);
	LL_TIM_SetAutoReload(tim, bldc_hall_xor_counter_max(tim));

	/* IC1 captures the trigger signal, synchronized with the slave reset. */
	LL_TIM_IC_SetActiveInput(tim, LL_TIM_CHANNEL_CH1, LL_TIM_ACTIVEINPUT_TRC);
	LL_TIM_IC_SetFilter(tim, LL_TIM_CHANNEL_CH1, LL_TIM_IC_FILTER_FDIV1);
	LL_TIM_IC_SetPrescaler(tim, LL_TIM_CHANNEL_CH1, LL_TIM_ICPSC_DIV1);
	LL_TIM_IC_SetPolarity(tim, LL_TIM_CHANNEL_CH1, LL_TIM_IC_POLARITY_RISING);

	/* Slave mode: reset the counter on every TI1 edge. */
	LL_TIM_SetSlaveMode(tim, LL_TIM_SLAVEMODE_RESET);
	LL_TIM_SetTriggerInput(tim, LL_TIM_TS_TI1F_ED);

	/* URS = 1: only an over/underflow generates an update event. Otherwise  the slave-mode
	 * RESET on every Hall edge would raise one too, and the overflow would stop meaning "no
	 * edge for a whole counter span".
	 */
	LL_TIM_SetUpdateSource(tim, LL_TIM_UPDATESOURCE_COUNTER);

	LL_TIM_ClearFlag_UPDATE(tim);
	LL_TIM_ClearFlag_CC1(tim);
	LL_TIM_CC_EnableChannel(tim, LL_TIM_CHANNEL_CH1);

	LL_TIM_EnableCounter(tim);
}

/**
 * @brief Bring the timer up with the edge interrupts still masked.
 *
 * @param dev bldc_hall device.
 *
 * @return 0 on success, negative errno otherwise.
 */
static int bldc_hall_xor_stm32_init(const struct device *dev)
{
	int ret;

	const struct bldc_hall_xor_stm32_config *cfg = dev->config;
	struct bldc_hall_xor_stm32_data *data = dev->data;

	/* The lines stay in alternate function for the timer; only their input register is read,
	 * which works in that mode.
	 */
	for (int i = 0; i < BLDC_HALL_LINE_COUNT; i++) {
		if (!gpio_is_ready_dt(&cfg->lines[i])) {
			LOG_ERR("[%s] Hall line %d controller not ready", dev->name, i);
			return -ENODEV;
		}
	}

	ret = bldc_hall_xor_stm32_clock_init(dev);
	if (ret < 0) {
		return ret;
	}

	ret = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
	if (ret < 0) {
		LOG_ERR("[%s] pinctrl setup failed: %d", dev->name, ret);
		return ret;
	}

	bldc_hall_xor_stm32_timer_init(dev);

	/* Connect the NVIC line, but leave the timer interrupts masked: set_edge_handler() unmasks
	 * them once a consumer is ready.
	 */
	cfg->irq_config();

	LOG_DBG("[%s] counter %u Hz, %u edges per electrical revolution", dev->name,
		data->counter_freq_hz, BLDC_HALL_XOR_EDGES_PER_REV);

	return 0;
}

/** The timer lives on the parent st,stm32-timers node. */
#define BLDC_HALL_XOR_TIMER(idx) DT_INST_PARENT(idx)

/** Instantiate one bldc_hall device from DT instance @p idx. */
#define BLDC_HALL_XOR_INST(idx)                                                                    \
	BUILD_ASSERT(DT_IRQ_HAS_NAME(BLDC_HALL_XOR_TIMER(idx), global),                            \
		     "instance " #idx ": parent timer must have a single global interrupt");       \
	BUILD_ASSERT(DT_INST_PROP_LEN(idx, hall_gpios) == BLDC_HALL_LINE_COUNT,                    \
		     "instance " #idx ": hall-gpios must list exactly three lines");               \
	BUILD_ASSERT((DT_INST_GPIO_FLAGS_BY_IDX(idx, hall_gpios, 0) |                              \
		      DT_INST_GPIO_FLAGS_BY_IDX(idx, hall_gpios, 1) |                              \
		      DT_INST_GPIO_FLAGS_BY_IDX(idx, hall_gpios, 2)) == 0,                         \
		     "instance " #idx ": hall-gpios are set up by pinctrl, their flags "           \
		     "must be 0");                                                                 \
                                                                                                   \
	PINCTRL_DT_INST_DEFINE(idx);                                                               \
                                                                                                   \
	static void bldc_hall_xor_irq_config_##idx(void)                                           \
	{                                                                                          \
		IRQ_CONNECT(DT_IRQN(BLDC_HALL_XOR_TIMER(idx)),                                     \
			    DT_IRQ(BLDC_HALL_XOR_TIMER(idx), priority), bldc_hall_xor_stm32_isr,   \
			    DEVICE_DT_INST_GET(idx), 0);                                           \
		irq_enable(DT_IRQN(BLDC_HALL_XOR_TIMER(idx)));                                     \
	}                                                                                          \
                                                                                                   \
	static const struct bldc_hall_xor_stm32_config bldc_hall_xor_config_##idx = {              \
		.timer = (TIM_TypeDef *)DT_REG_ADDR(BLDC_HALL_XOR_TIMER(idx)),                     \
		.pclken =                                                                          \
			{                                                                          \
				.bus = DT_CLOCKS_CELL(BLDC_HALL_XOR_TIMER(idx), bus),              \
				.enr = DT_CLOCKS_CELL(BLDC_HALL_XOR_TIMER(idx), bits),             \
			},                                                                         \
		.prescaler = DT_PROP(BLDC_HALL_XOR_TIMER(idx), st_prescaler),                      \
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(idx),                                       \
		.lines =                                                                           \
			{                                                                          \
				GPIO_DT_SPEC_INST_GET_BY_IDX(idx, hall_gpios, 0),                  \
				GPIO_DT_SPEC_INST_GET_BY_IDX(idx, hall_gpios, 1),                  \
				GPIO_DT_SPEC_INST_GET_BY_IDX(idx, hall_gpios, 2),                  \
			},                                                                         \
		.irq_config = bldc_hall_xor_irq_config_##idx,                                      \
	};                                                                                         \
                                                                                                   \
	static struct bldc_hall_xor_stm32_data bldc_hall_xor_data_##idx;                           \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(idx, bldc_hall_xor_stm32_init, NULL, &bldc_hall_xor_data_##idx,      \
			      &bldc_hall_xor_config_##idx, POST_KERNEL,                            \
			      CONFIG_BLDC_HALL_INIT_PRIORITY, &bldc_hall_xor_stm32_driver_api);

DT_INST_FOREACH_STATUS_OKAY(BLDC_HALL_XOR_INST)
