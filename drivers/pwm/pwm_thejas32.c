/*
 * Copyright (c) 2026 Anuj Deshpande
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT cdac_thejas32_pwm

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/sys/barrier.h>
#include <zephyr/sys/sys_io.h>

#define THEJAS32_PWM_CHANNELS 8

/* Per-channel registers, 0x10 apart */
#define THEJAS32_PWM_CTRL(ch)   ((ch) * 0x10 + 0x00)
#define THEJAS32_PWM_STATUS(ch) ((ch) * 0x10 + 0x04)
#define THEJAS32_PWM_PERIOD(ch) ((ch) * 0x10 + 0x08)
#define THEJAS32_PWM_ONOFF(ch)  ((ch) * 0x10 + 0x0c)

/* Global control register */
#define THEJAS32_PWM_GCR     0x80
#define THEJAS32_PWM_GCR_GPE BIT(0) /* global PWM enable */
#define THEJAS32_PWM_GCR_GIE BIT(1) /* global interrupt enable */

#define THEJAS32_PWM_CTRL_MODE_IDLE       0
#define THEJAS32_PWM_CTRL_MODE_CONTINUOUS 2
/* Alignment: left starts the cycle with the on-time, right with the off-time */
#define THEJAS32_PWM_CTRL_ALIGN_RIGHT     BIT(2)
#define THEJAS32_PWM_CTRL_IE              BIT(4)
/* Output level a channel drives while idle */
#define THEJAS32_PWM_CTRL_OPC_HIGH        BIT(5)

struct pwm_thejas32_config {
	mem_addr_t base;
	uint32_t clock_frequency;
};

static void reg_write(const struct device *dev, uint32_t off, uint32_t val)
{
	const struct pwm_thejas32_config *cfg = dev->config;

	sys_write32(val, cfg->base + off);
	barrier_dmem_fence_full();
}

static int pwm_thejas32_set_cycles(const struct device *dev, uint32_t channel,
				   uint32_t period_cycles, uint32_t pulse_cycles, pwm_flags_t flags)
{
	bool inverted = (flags & PWM_POLARITY_MASK) == PWM_POLARITY_INVERTED;

	if (channel >= THEJAS32_PWM_CHANNELS) {
		return -EINVAL;
	}

	if (pulse_cycles == 0U || pulse_cycles >= period_cycles) {
		/*
		 * Constant output: park the channel in idle mode with the
		 * requested level on its idle-output-control bit.
		 */
		bool active = pulse_cycles != 0U;
		bool high = active != inverted;

		reg_write(dev, THEJAS32_PWM_CTRL(channel),
			  THEJAS32_PWM_CTRL_MODE_IDLE | (high ? THEJAS32_PWM_CTRL_OPC_HIGH : 0));
		return 0;
	}

	reg_write(dev, THEJAS32_PWM_PERIOD(channel), period_cycles);
	reg_write(dev, THEJAS32_PWM_ONOFF(channel), pulse_cycles);

	/*
	 * Left alignment outputs the on-time first, giving normal polarity.
	 * Right alignment outputs the off-time first, and the on/off register
	 * then holds the off-time, giving an active-low pulse of the same
	 * length: inverted polarity.
	 */
	reg_write(dev, THEJAS32_PWM_CTRL(channel),
		  THEJAS32_PWM_CTRL_MODE_CONTINUOUS |
			  (inverted ? THEJAS32_PWM_CTRL_ALIGN_RIGHT : 0));

	return 0;
}

static int pwm_thejas32_get_cycles_per_sec(const struct device *dev, uint32_t channel,
					   uint64_t *cycles)
{
	const struct pwm_thejas32_config *cfg = dev->config;

	if (channel >= THEJAS32_PWM_CHANNELS) {
		return -EINVAL;
	}

	*cycles = cfg->clock_frequency;

	return 0;
}

static int pwm_thejas32_init(const struct device *dev)
{
	for (uint32_t ch = 0; ch < THEJAS32_PWM_CHANNELS; ch++) {
		reg_write(dev, THEJAS32_PWM_CTRL(ch), THEJAS32_PWM_CTRL_MODE_IDLE);
	}
	reg_write(dev, THEJAS32_PWM_GCR, THEJAS32_PWM_GCR_GPE);

	return 0;
}

static DEVICE_API(pwm, pwm_thejas32_api) = {
	.set_cycles = pwm_thejas32_set_cycles,
	.get_cycles_per_sec = pwm_thejas32_get_cycles_per_sec,
};

#define PWM_THEJAS32_INIT(n)                                                                       \
	static const struct pwm_thejas32_config pwm_thejas32_config_##n = {                        \
		.base = DT_INST_REG_ADDR(n),                                                       \
		.clock_frequency = DT_INST_PROP(n, clock_frequency),                               \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(n, &pwm_thejas32_init, NULL, NULL, &pwm_thejas32_config_##n,         \
			      POST_KERNEL, CONFIG_PWM_INIT_PRIORITY, &pwm_thejas32_api);

DT_INST_FOREACH_STATUS_OKAY(PWM_THEJAS32_INIT)
