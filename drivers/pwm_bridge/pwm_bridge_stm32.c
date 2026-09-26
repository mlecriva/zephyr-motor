/*
 * Copyright (c) 2026 Rtone
 * Copyright (c) 2026 Mathis Lécrivain <lecrivain.mathis@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT st_stm32_pwm_bridge

#include <zephyr/device.h>
#include <zephyr/drivers/clock_control/stm32_clock_control.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/reset.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>
#include <soc.h>

#include <stm32_ll_tim.h>

#include <drivers/pwm_bridge.h>

LOG_MODULE_REGISTER(pwm_bridge_stm32, CONFIG_PWM_BRIDGE_LOG_LEVEL);

/** TIM1/TIM8 are 16-bit. */
#define PWM_BRIDGE_MAX_ARR UINT16_MAX

/** Center-aligned: the counter spans ARR twice per PWM period. */
#define PWM_BRIDGE_RAMPS_PER_PERIOD 2U

/** One update event, hence one CCRx reload, per PWM period. */
#define PWM_BRIDGE_UEV_PER_PERIOD 1U

/** t_DTS = 1 / f_tim, which the dead-time conversion below assumes. */
#define PWM_BRIDGE_CLOCK_DIVISION LL_TIM_CLOCKDIVISION_DIV1

/** Channels 1..4 are the ones with both a complementary output and a pin. */
#define PWM_BRIDGE_MAX_LEGS 4U

/** Every leg state of the class is reachable on this hardware. */
#define PWM_BRIDGE_STATE_MASK                                                                      \
	(PWM_BRIDGE_STATE_BIT(PWM_BRIDGE_LEG_HI_Z) | PWM_BRIDGE_STATE_BIT(PWM_BRIDGE_LEG_PWM) |    \
	 PWM_BRIDGE_STATE_BIT(PWM_BRIDGE_LEG_FORCE_LOW) |                                          \
	 PWM_BRIDGE_STATE_BIT(PWM_BRIDGE_LEG_FORCE_HIGH))

/**
 * @brief One DTG encoding range: DT = (offset + DTG[n:0]) * step * t_DTS.
 */
static const struct {
	uint8_t sel_mask; /**< DTG bits identifying the range. */
	uint8_t sel_val;  /**< Their value for this range. */
	uint8_t val_mask; /**< DTG bits carrying the count. */
	uint8_t offset;   /**< Count the range itself stands for. */
	uint8_t step;     /**< t_DTS ticks per count. */
} pwm_bridge_dtg_ranges[] = {
	{0x80U, 0x00U, 0x7FU, 0U, 1U},   /* DTG[7:5] = 0xx, 0..127 ticks */
	{0xC0U, 0x80U, 0x3FU, 64U, 2U},  /* DTG[7:5] = 10x, 128..254 */
	{0xE0U, 0xC0U, 0x1FU, 32U, 8U},  /* DTG[7:5] = 110, 256..504 */
	{0xE0U, 0xE0U, 0x1FU, 32U, 16U}, /* DTG[7:5] = 111, 512..1008 */
};

/**
 * @brief One leg, i.e. one timer channel and its complementary output.
 *
 * @note Unlike the enable masks there is no CHxN compare setter: LL_TIM_OC_SetMode and
 * LL_TIM_OC_EnablePreload address CHx and CCMR drives CHxN from it.
 */
struct pwm_bridge_leg_def {
	uint32_t ch;                                  /**< High side, CCxE. */
	uint32_t chn;                                 /**< Low side, CCxNE. */
	void (*set_compare)(TIM_TypeDef *, uint32_t); /**< CCRx write. */
};

/**
 * @brief Per-instance device tree configuration.
 *
 */
struct pwm_bridge_stm32_config {
	TIM_TypeDef *timer;                    /**< Advanced-control timer, TIM1 or TIM8. */
	const struct pwm_bridge_leg_def *legs; /**< Legs, in st,leg-channels order. */
	uint8_t leg_count;                     /**< Entries in @ref legs. */
	uint32_t prescaler;                    /**< PSC, from the parent timer node. */
	uint32_t freq_hz;                      /**< Requested carrier frequency. */
	uint32_t deadtime_ns;                  /**< Inserted between the two sides of a leg. */
	bool sync_trigger;                     /**< Drive TRGO2 from the update event. */
	bool break_input;                      /**< Arm BKIN, the hardware cutoff. */
	const struct stm32_pclken *pclken;     /**< Timer clock gate, then clock source. */
	size_t pclk_len;                       /**< Entries in @ref pclken. */
	const struct pinctrl_dev_config *pcfg; /**< Pin configuration of the leg outputs. */
};

/**
 * @brief Runtime state, all of it derived from the configuration at init.
 */
struct pwm_bridge_stm32_data {
	struct reset_dt_spec reset; /**< Timer reset line. */
	uint32_t tim_clk;           /**< Timer input clock (Hz), before the prescaler. */
	uint32_t arr;               /**< Auto-reload, i.e. the counter value at the peak. */
	uint32_t carrier_hz;        /**< Carrier the auto-reload actually yields. */
};

/**
 * @brief Counter clock (Hz), i.e. the timer input clock after the prescaler.
 *
 * @param dev pwm_bridge device.
 *
 * @return Rate at which CNT increments; one tick is one ARR/CCRx unit.
 */
static inline uint32_t pwm_bridge_counter_clk(const struct device *dev)
{
	const struct pwm_bridge_stm32_config *cfg = dev->config;
	const struct pwm_bridge_stm32_data *data = dev->data;

	return data->tim_clk / (cfg->prescaler + 1U);
}

/**
 * @brief Preload the output configuration of one leg.
 *
 * @note CCRx is double-buffered (OCxPE = 1) so duty changes apply on the next UEV without glitching
 * the current period. OCxM and the CCER bits are preloaded (CCPC = 1) and commit together on COM.
 *
 * @param tim Timer instance.
 * @param leg Leg to configure.
 * @param state Requested leg state.
 * @param pulse_cycles On-time in counter cycles; only reaches the pins while the leg is chopped.
 */
static void pwm_bridge_apply_leg(TIM_TypeDef *tim, const struct pwm_bridge_leg_def *leg,
				 pwm_bridge_leg_state_t state, uint32_t pulse_cycles)
{
	/* Set leg compare value */
	leg->set_compare(tim, pulse_cycles);

	switch (state) {
	case PWM_BRIDGE_LEG_HI_Z:
		/* Neither output enabled, so OSSR does not apply. Its inactive level only reaches a
		 * pin "as soon as CCxE=1 or CCxNE=1". Both pins are released, and their level is
		 * whatever the gate driver does with an undriven input.
		 */
		LL_TIM_OC_SetMode(tim, leg->ch, LL_TIM_OCMODE_PWM1);
		LL_TIM_CC_DisableChannel(tim, leg->ch | leg->chn);
		break;

	case PWM_BRIDGE_LEG_PWM:
		/* Both sides enabled: the dead-time generator derives OCxN from  the same OCxREF,
		 * complemented, so the low side conducts the freewheeling current instead of the
		 * body diode.
		 */
		LL_TIM_OC_SetMode(tim, leg->ch, LL_TIM_OCMODE_PWM1);
		LL_TIM_CC_EnableChannel(tim, leg->ch | leg->chn);
		break;

	case PWM_BRIDGE_LEG_FORCE_LOW:
		/* Same enabled pair, so the dead-time generator stays in the path for the
		 * commutation that turns this leg into PWM. Forcing OCxREF inactive gives OCx = 0
		 * and OCxN = 1.
		 */
		LL_TIM_OC_SetMode(tim, leg->ch, LL_TIM_OCMODE_FORCED_INACTIVE);
		LL_TIM_CC_EnableChannel(tim, leg->ch | leg->chn);
		break;

	case PWM_BRIDGE_LEG_FORCE_HIGH:
		/* Mirror of FORCE_LOW: OCxREF forced active gives OCx = 1 and OCxN = 0. A
		 * bootstrapped gate driver only holds this for as long as its capacitor lasts.
		 */
		LL_TIM_OC_SetMode(tim, leg->ch, LL_TIM_OCMODE_FORCED_ACTIVE);
		LL_TIM_CC_EnableChannel(tim, leg->ch | leg->chn);
		break;
	}
}

/**
 * @brief Switch every leg to a new pattern, all on the same timer cycle.
 *
 * @param dev pwm_bridge device.
 * @param legs Per-leg states and on-times.
 * @param count Entries in @p legs.
 *
 * @retval 0 on success.
 * @retval -EINVAL if @p count does not match the bridge or a leg state is out of range; the bridge
 * is left untouched.
 */
static int pwm_bridge_stm32_set_pattern(const struct device *dev, const pwm_bridge_leg_t *legs,
					uint8_t count)
{
	const struct pwm_bridge_stm32_config *cfg = dev->config;
	TIM_TypeDef *tim = cfg->timer;

	if (count != cfg->leg_count) {
		return -EINVAL;
	}

	/* Validate the whole pattern: a rejected leg must not leave the bridge half-updated. */
	for (uint8_t i = 0; i < count; i++) {
		if (legs[i].state > PWM_BRIDGE_LEG_STATE_MAX) {
			return -EINVAL;
		}
	}

	/* Prepare all legs before applying it */
	for (uint8_t i = 0; i < count; i++) {
		pwm_bridge_apply_leg(tim, &cfg->legs[i], legs[i].state, legs[i].pulse_cycles);
	}

	/* Atomic load of the new CCMR/CCER set. */
	LL_TIM_GenerateEvent_COM(tim);

	return 0;
}

/**
 * @brief Arm or release the output stage through MOE.
 *
 * @param dev pwm_bridge device.
 * @param enable true to let the preloaded pattern reach the pins, false to idle the bridge
 * (OSSI/OIS drive every gate low).
 *
 * @retval 0 Always; MOE is a single register bit.
 */
static int pwm_bridge_stm32_set_outputs(const struct device *dev, bool enable)
{
	const struct pwm_bridge_stm32_config *cfg = dev->config;

	if (enable) {
		LL_TIM_EnableAllOutputs(cfg->timer); /* MOE = 1 */
	} else {
		LL_TIM_DisableAllOutputs(cfg->timer); /* MOE = 0 */
	}

	return 0;
}

/**
 * @brief Report the shape of the bridge and its duty full scale.
 *
 * @param dev pwm_bridge device.
 * @param caps Output, filled from values fixed at init.
 *
 * @retval 0 Always.
 */
static int pwm_bridge_stm32_get_caps(const struct device *dev, pwm_bridge_caps_t *caps)
{
	const struct pwm_bridge_stm32_config *cfg = dev->config;
	const struct pwm_bridge_stm32_data *data = dev->data;

	caps->leg_count = cfg->leg_count;
	caps->state_mask = PWM_BRIDGE_STATE_MASK;
	caps->max_pulse_cycles = data->arr;
	caps->carrier_hz = data->carrier_hz;
	caps->center_aligned = true;

	return 0;
}

static DEVICE_API(pwm_bridge, pwm_bridge_stm32_driver_api) = {
	.set_pattern = pwm_bridge_stm32_set_pattern,
	.set_outputs = pwm_bridge_stm32_set_outputs,
	.get_caps = pwm_bridge_stm32_get_caps,
};

/**
 * @brief Enable the timer clock and store the rate its counter derives from.
 *
 * @param dev pwm_bridge device.
 *
 * @return 0 on success, negative errno otherwise.
 */
static int pwm_bridge_stm32_clock_init(const struct device *dev)
{
	const struct device *clk = DEVICE_DT_GET(STM32_CLOCK_CONTROL_NODE);
	const struct pwm_bridge_stm32_config *cfg = dev->config;
	struct pwm_bridge_stm32_data *data = dev->data;

	if (!device_is_ready(clk)) {
		return -ENODEV;
	}

	int ret = clock_control_on(clk, (clock_control_subsys_t)&cfg->pclken[0]);
	if (ret < 0) {
		LOG_ERR("[%s] failed to enable timer clock: %d", dev->name, ret);
		return ret;
	}

	if (cfg->pclk_len <= 1) {
		LOG_ERR("[%s] timer clock source is not specified", dev->name);
		return -EINVAL;
	}

	ret = clock_control_configure(clk, (clock_control_subsys_t)&cfg->pclken[1], NULL);
	if (ret < 0) {
		LOG_ERR("[%s] failed to configure timer clock source: %d", dev->name, ret);
		return ret;
	}

	ret = clock_control_get_rate(clk, (clock_control_subsys_t)&cfg->pclken[1], &data->tim_clk);
	if (ret < 0) {
		LOG_ERR("[%s] failed to read timer clock rate: %d", dev->name, ret);
		return ret;
	}

	return 0;
}

/**
 * @brief Reject a leg list that names the same channel twice.
 *
 * @param dev pwm_bridge device.
 *
 * @retval 0 on success.
 * @retval -EINVAL if two legs share a timer channel.
 */
static int pwm_bridge_stm32_check_legs(const struct device *dev)
{
	const struct pwm_bridge_stm32_config *cfg = dev->config;
	uint32_t seen = 0U;

	for (uint8_t i = 0; i < cfg->leg_count; i++) {
		if ((seen & cfg->legs[i].ch) != 0U) {
			LOG_ERR("[%s] st,leg-channels names the same channel twice", dev->name);
			return -EINVAL;
		}
		seen |= cfg->legs[i].ch;
	}

	return 0;
}

/**
 * @brief Program the time base and every leg into its idle configuration.
 *
 * @param dev pwm_bridge device.
 *
 * @retval 0 on success.
 * @retval -EINVAL if the requested frequency is out of reach for st,prescaler.
 */
static int pwm_bridge_stm32_timer_init(const struct device *dev)
{
	const struct pwm_bridge_stm32_config *cfg = dev->config;
	struct pwm_bridge_stm32_data *data = dev->data;
	TIM_TypeDef *tim = cfg->timer;

	/* Time base stopped and bridge idle while the registers are rewritten. */
	LL_TIM_DisableCounter(tim);
	LL_TIM_DisableAllOutputs(tim);

	/* CCPC = 0 during init so CCMR/CCER writes apply immediately. */
	CLEAR_BIT(tim->CR2, TIM_CR2_CCPC);

	/* PSC first: the counter clock it yields is what ARR is derived from. */
	LL_TIM_SetPrescaler(tim, cfg->prescaler);

	/* A carrier above the counter clock has no representable period. */
	uint32_t counter_clk = pwm_bridge_counter_clk(dev);
	if (cfg->freq_hz == 0U || cfg->freq_hz > counter_clk) {
		LOG_ERR("[%s] invalid PWM frequency %u Hz (counter clock %u Hz)", dev->name,
			cfg->freq_hz, counter_clk);
		return -EINVAL;
	}

	/* Center-aligned: one PWM period is two ramps, so ARR is half of them. */
	data->arr = counter_clk / (cfg->freq_hz * PWM_BRIDGE_RAMPS_PER_PERIOD);
	if (data->arr < 2U || data->arr > PWM_BRIDGE_MAX_ARR) {
		LOG_ERR("[%s] auto-reload %u out of range for %u Hz — adjust st,prescaler",
			dev->name, data->arr, cfg->freq_hz);
		return -EINVAL;
	}

	/* Truncating ARR moves the carrier; report what the hardware runs at. */
	data->carrier_hz = counter_clk / (data->arr * PWM_BRIDGE_RAMPS_PER_PERIOD);

	/* Time base: center-aligned, one UEV per period, ARR double-buffered. */
	LL_TIM_SetAutoReload(tim, data->arr);
	LL_TIM_SetCounterMode(tim, LL_TIM_COUNTERMODE_CENTER_UP);
	LL_TIM_SetRepetitionCounter(tim, PWM_BRIDGE_UEV_PER_PERIOD);
	LL_TIM_SetClockDivision(tim, PWM_BRIDGE_CLOCK_DIVISION);
	LL_TIM_EnableARRPreload(tim);

	/* All: PWM mode 1, output preload, both sides disabled and active-high, idling low. */
	for (uint8_t i = 0; i < cfg->leg_count; i++) {
		const struct pwm_bridge_leg_def *leg = &cfg->legs[i];

		LL_TIM_OC_SetMode(tim, leg->ch, LL_TIM_OCMODE_PWM1);
		LL_TIM_OC_EnablePreload(tim, leg->ch);
		LL_TIM_OC_DisableFast(tim, leg->ch);
		LL_TIM_OC_SetPolarity(tim, leg->ch, LL_TIM_OCPOLARITY_HIGH);
		LL_TIM_OC_SetPolarity(tim, leg->chn, LL_TIM_OCPOLARITY_HIGH);
		LL_TIM_OC_SetIdleState(tim, leg->ch, LL_TIM_OCIDLESTATE_LOW);
		LL_TIM_OC_SetIdleState(tim, leg->chn, LL_TIM_OCIDLESTATE_LOW);

		leg->set_compare(tim, 0U);
	}

	/* OSSR = 1: when MOE = 1 and a side is disabled, force its OIS level instead of releasing
	 * the output. Combined with OISxN = 0 this keeps the low-side gate low while only the high
	 * side is chopped. OSSI = 1 with OIS = 0 makes a disabled output stage drive every gate
	 * low, i.e. the motor coasts.
	 */
	LL_TIM_SetOffStates(tim, LL_TIM_OSSI_ENABLE, LL_TIM_OSSR_ENABLE);

	return 0;
}

/**
 * @brief Encode a dead-time, in t_DTS ticks, into the DTG field.
 *
 * @note Rounds up: a board asks for a minimum separation, so the programmed value is the smallest
 * DTG that reaches it.
 *
 * @param ticks Requested dead-time in t_DTS ticks.
 * @param dtg Output, DTG field; untouched on failure.
 *
 * @retval 0 on success.
 * @retval -ERANGE if no range reaches @p ticks.
 */
static int pwm_bridge_dtg_encode(uint32_t ticks, uint8_t *dtg)
{
	ARRAY_FOR_EACH_PTR(pwm_bridge_dtg_ranges, r) {
		if (ticks > (uint32_t)(r->offset + r->val_mask) * r->step) {
			continue;
		}

		uint32_t count = DIV_ROUND_UP(ticks, r->step);
		count = (count > r->offset) ? count - r->offset : 0U;
		*dtg = r->sel_val | (uint8_t)count;

		return 0;
	}

	return -ERANGE;
}

/**
 * @brief Dead-time a DTG field stands for, in t_DTS ticks.
 *
 * @param dtg DTG field.
 *
 * @return Dead-time in t_DTS ticks.
 */
static uint32_t pwm_bridge_dtg_decode(uint8_t dtg)
{
	ARRAY_FOR_EACH_PTR(pwm_bridge_dtg_ranges, r) {
		if ((dtg & r->sel_mask) == r->sel_val) {
			return (uint32_t)(r->offset + (dtg & r->val_mask)) * r->step;
		}
	}

	return 0U;
}

/**
 * @brief Program the dead-time generator from deadtime-ns.
 *
 * @param dev pwm_bridge device.
 *
 * @retval 0 on success.
 * @retval -EINVAL if the request exceeds what DTG can encode.
 */
static int pwm_bridge_stm32_deadtime_init(const struct device *dev)
{
	const struct pwm_bridge_stm32_config *cfg = dev->config;
	const struct pwm_bridge_stm32_data *data = dev->data;

	uint32_t ticks = DIV_ROUND_UP((uint64_t)cfg->deadtime_ns * data->tim_clk, NSEC_PER_SEC);
	uint8_t dtg;

	int ret = pwm_bridge_dtg_encode(ticks, &dtg);
	if (ret < 0) {
		LOG_ERR("[%s] deadtime %u ns (%u ticks) exceeds the DTG range at f_tim=%u Hz",
			dev->name, cfg->deadtime_ns, ticks, data->tim_clk);
		return -EINVAL;
	}

	LL_TIM_OC_SetDeadTime(cfg->timer, dtg);

	ticks = pwm_bridge_dtg_decode(dtg);

	LOG_DBG("[%s] f_tim=%u Hz, arr=%u, carrier=%u Hz, deadtime=%u ns (DTG=0x%02x, %u ticks)",
		dev->name, data->tim_clk, data->arr, data->carrier_hz,
		(uint32_t)(((uint64_t)ticks * NSEC_PER_SEC) / data->tim_clk), dtg, ticks);

	return 0;
}

/**
 * @brief Set up the period-synchronous trigger, unless sync-trigger-enable is unset.
 *
 * @note TRGO2 is driven straight from the update event, which a center-aligned counter reaches once
 * per period at CNT = 0. A fixed point in the cycle, common to every leg. A consumer can samples
 * the winding current there.
 *
 * @param dev pwm_bridge device.
 */
static void pwm_bridge_stm32_sync_trigger_init(const struct device *dev)
{
	const struct pwm_bridge_stm32_config *cfg = dev->config;
	TIM_TypeDef *tim = cfg->timer;

	if (!cfg->sync_trigger) {
		return;
	}

	LL_TIM_SetTriggerOutput2(tim, LL_TIM_TRGO2_UPDATE);

	LOG_DBG("[%s] sync trigger on TRGO2 (update event, counter trough)", dev->name);
}

/**
 * @brief Bring the timer up with the bridge left idle (MOE = 0).
 *
 * @param dev pwm_bridge device.
 *
 * @return 0 on success, negative errno otherwise.
 */
static int pwm_bridge_stm32_init(const struct device *dev)
{
	const struct pwm_bridge_stm32_config *cfg = dev->config;
	struct pwm_bridge_stm32_data *data = dev->data;
	TIM_TypeDef *tim = cfg->timer;

	int ret = pwm_bridge_stm32_check_legs(dev);
	if (ret < 0) {
		return ret;
	}

	ret = pwm_bridge_stm32_clock_init(dev);
	if (ret < 0) {
		return ret;
	}

	ret = reset_line_toggle_dt(&data->reset);
	if (ret < 0) {
		LOG_ERR("[%s] failed to reset timer: %d", dev->name, ret);
		return ret;
	}

	ret = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
	if (ret < 0) {
		LOG_ERR("[%s] pinctrl setup failed: %d", dev->name, ret);
		return ret;
	}

	ret = pwm_bridge_stm32_timer_init(dev);
	if (ret < 0) {
		return ret;
	}

	ret = pwm_bridge_stm32_deadtime_init(dev);
	if (ret < 0) {
		return ret;
	}

	/* Break input: BKE = 1, active low. The hardware cuts every output */
	if (cfg->break_input) {
		LL_TIM_EnableBRK(tim);
		LL_TIM_ConfigBRK(tim, LL_TIM_BREAK_POLARITY_LOW, LL_TIM_BREAK_FILTER_FDIV1,
				 LL_TIM_BREAK_AFMODE_INPUT);
	} else {
		LL_TIM_DisableBRK(tim);
	}

	pwm_bridge_stm32_sync_trigger_init(dev);

	/* Force-load the shadow registers from CCRx/ARR/PSC/RCR, then arm channel preload so later
	 * writes to CCxE/CCxNE/OCxM batch until COM.
	 */
	LL_TIM_CC_DisablePreload(tim);
	LL_TIM_GenerateEvent_UPDATE(tim);
	LL_TIM_CC_EnablePreload(tim);

	/* CCUS = 0: a commutation happens only when software sets COMG. */
	LL_TIM_CC_SetUpdate(tim, LL_TIM_CCUPDATESOURCE_COMG_ONLY);

	/* The time base runs from init so the sync trigger fires even while the bridge is idle: a
	 * consumer sampling through it keeps reading a zero-load reference.
	 */
	LL_TIM_EnableCounter(tim);

	return 0;
}

/** The hardware lives on the parent st,stm32-timers node. */
#define PWM_BRIDGE_TIMER(idx) DT_INST_PARENT(idx)

/** One term per enabled child of the timer node, folded into a sibling count below. */
#define PWM_BRIDGE_COUNT_ENABLED(node_id) 1 +

/** Enabled children of the parent timer node, this bridge instance included. */
#define PWM_BRIDGE_SIBLING_COUNT(idx)                                                              \
	(DT_FOREACH_CHILD_STATUS_OKAY(PWM_BRIDGE_TIMER(idx), PWM_BRIDGE_COUNT_ENABLED) 0)

/** Channel @p n as a leg: its two CCER enables and its compare setter. */
#define PWM_BRIDGE_LEG_DEF(n)                                                                      \
	{                                                                                          \
		.ch = CONCAT(LL_TIM_CHANNEL_CH, n),                                                \
		.chn = CONCAT(LL_TIM_CHANNEL_CH, n, N),                                            \
		.set_compare = CONCAT(LL_TIM_OC_SetCompareCH, n),                                  \
	}

/** One st,leg-channels element, as a @ref pwm_bridge_leg_def initializer. */
#define PWM_BRIDGE_LEG_ENTRY(node_id, prop, i) PWM_BRIDGE_LEG_DEF(DT_PROP_BY_IDX(node_id, prop, i))

/** Compile-time bounds on one st,leg-channels element. */
#define PWM_BRIDGE_LEG_ASSERT(node_id, prop, i)                                                    \
	BUILD_ASSERT(DT_PROP_BY_IDX(node_id, prop, i) >= 1 &&                                      \
			     DT_PROP_BY_IDX(node_id, prop, i) <= PWM_BRIDGE_MAX_LEGS,              \
		     "st,leg-channels: a leg must be timer channel 1..4");

/** Instantiate one pwm_bridge device from DT instance @p idx. */
#define PWM_BRIDGE_INST(idx)                                                                       \
	BUILD_ASSERT(DT_REG_ADDR(PWM_BRIDGE_TIMER(idx)) == TIM1_BASE ||                            \
			     DT_REG_ADDR(PWM_BRIDGE_TIMER(idx)) == TIM8_BASE,                      \
		     "instance " #idx ": parent timer must be TIM1 or TIM8 "                       \
		     "(complementary outputs, dead-time and TRGO2)");                              \
                                                                                                   \
	BUILD_ASSERT(DT_INST_PROP_LEN(idx, st_leg_channels) >= 1 &&                                \
			     DT_INST_PROP_LEN(idx, st_leg_channels) <= PWM_BRIDGE_MAX_LEGS,        \
		     "instance " #idx ": a bridge on one timer carries 1 to 4 legs");              \
                                                                                                   \
	BUILD_ASSERT(PWM_BRIDGE_SIBLING_COUNT(idx) == 1,                                           \
		     "instance " #idx ": no sibling node under the same timers node "              \
		     "(pwm, counter, qdec) may be enabled");                                       \
                                                                                                   \
	DT_INST_FOREACH_PROP_ELEM(idx, st_leg_channels, PWM_BRIDGE_LEG_ASSERT)                     \
                                                                                                   \
	PINCTRL_DT_INST_DEFINE(idx);                                                               \
                                                                                                   \
	static const struct stm32_pclken pwm_bridge_pclken_##idx[] =                               \
		STM32_DT_CLOCKS(PWM_BRIDGE_TIMER(idx));                                            \
                                                                                                   \
	static const struct pwm_bridge_leg_def pwm_bridge_legs_##idx[] = {                         \
		DT_INST_FOREACH_PROP_ELEM_SEP(idx, st_leg_channels, PWM_BRIDGE_LEG_ENTRY, (, ))};  \
                                                                                                   \
	static const struct pwm_bridge_stm32_config pwm_bridge_config_##idx = {                    \
		.timer = (TIM_TypeDef *)DT_REG_ADDR(PWM_BRIDGE_TIMER(idx)),                        \
		.legs = pwm_bridge_legs_##idx,                                                     \
		.leg_count = ARRAY_SIZE(pwm_bridge_legs_##idx),                                    \
		.prescaler = DT_PROP(PWM_BRIDGE_TIMER(idx), st_prescaler),                         \
		.freq_hz = DT_INST_PROP(idx, pwm_frequency_hz),                                    \
		.deadtime_ns = DT_INST_PROP(idx, deadtime_ns),                                     \
		.sync_trigger = DT_INST_PROP(idx, sync_trigger_enable),                            \
		.break_input = DT_INST_PROP(idx, break_input),                                     \
		.pclken = pwm_bridge_pclken_##idx,                                                 \
		.pclk_len = DT_NUM_CLOCKS(PWM_BRIDGE_TIMER(idx)),                                  \
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(idx),                                       \
	};                                                                                         \
                                                                                                   \
	static struct pwm_bridge_stm32_data pwm_bridge_data_##idx = {                              \
		.reset = RESET_DT_SPEC_GET(PWM_BRIDGE_TIMER(idx)),                                 \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(idx, pwm_bridge_stm32_init, NULL, &pwm_bridge_data_##idx,            \
			      &pwm_bridge_config_##idx, POST_KERNEL,                               \
			      CONFIG_PWM_BRIDGE_INIT_PRIORITY, &pwm_bridge_stm32_driver_api);

DT_INST_FOREACH_STATUS_OKAY(PWM_BRIDGE_INST)
