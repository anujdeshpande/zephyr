/*
 * Copyright (c) 2026 Anuj Deshpande
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT cdac_thejas32_spi

#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/sys/barrier.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/sys/util.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(spi_thejas32, CONFIG_SPI_LOG_LEVEL);

#include "spi_context.h"

#define THEJAS32_SPI_CTRL   0x00 /* 16-bit control register */
#define THEJAS32_SPI_STATUS 0x04
#define THEJAS32_SPI_BAUD   0x08
#define THEJAS32_SPI_TXDATA 0x0c
#define THEJAS32_SPI_RXDATA 0x10

#define THEJAS32_SPI_CTRL_PCS(cs)   ((cs) & 0x3) /* peripheral chip select */
#define THEJAS32_SPI_CTRL_LSB_FIRST BIT(3)
#define THEJAS32_SPI_CTRL_MODE(m)   (((m) & 0x3) << 4) /* CPOL/CPHA */
#define THEJAS32_SPI_CTRL_CSAAT     BIT(8)             /* CS active after transfer */
#define THEJAS32_SPI_CTRL_DBITS(b)  (((b) & 0xf) << 9) /* bits per word */

#define THEJAS32_SPI_STATUS_BUSY    BIT(4)
#define THEJAS32_SPI_STATUS_OVERRUN BIT(5)
#define THEJAS32_SPI_STATUS_RXC     BIT(6) /* receive complete */
#define THEJAS32_SPI_STATUS_TXHE    BIT(7) /* TX holding register empty */

/* Baud register bits [7:4] select SCK = input clock / (4 << code), code 0..9 */
#define THEJAS32_SPI_BAUD_CODE(c)  (((c) & 0xf) << 4)
#define THEJAS32_SPI_BAUD_CODE_MAX 9

#define THEJAS32_SPI_TIMEOUT_US 20000

struct spi_thejas32_config {
	mem_addr_t base;
	uint32_t clock_frequency;
};

struct spi_thejas32_data {
	struct spi_context ctx;
	uint16_t ctrl;
};

static bool status_wait(const struct device *dev, uint32_t mask, uint32_t val)
{
	const struct spi_thejas32_config *cfg = dev->config;

	return WAIT_FOR((sys_read32(cfg->base + THEJAS32_SPI_STATUS) & mask) == val,
			THEJAS32_SPI_TIMEOUT_US, k_busy_wait(1));
}

static void ctrl_write(const struct device *dev, uint16_t val)
{
	const struct spi_thejas32_config *cfg = dev->config;

	sys_write16(val, cfg->base + THEJAS32_SPI_CTRL);
	barrier_dmem_fence_full();
}

static int spi_thejas32_configure(const struct device *dev, const struct spi_config *config)
{
	const struct spi_thejas32_config *cfg = dev->config;
	struct spi_thejas32_data *data = dev->data;
	uint32_t code;
	uint16_t ctrl;
	uint8_t mode = 0;

	if (spi_context_configured(&data->ctx, config)) {
		return 0;
	}

	if (config->operation & SPI_OP_MODE_SLAVE) {
		LOG_ERR("Slave mode is not supported");
		return -ENOTSUP;
	}

	if (config->operation &
	    (SPI_MODE_LOOP | SPI_LINES_DUAL | SPI_LINES_QUAD | SPI_LINES_OCTAL)) {
		LOG_ERR("Unsupported operation flags 0x%x", config->operation);
		return -ENOTSUP;
	}

	if (SPI_WORD_SIZE_GET(config->operation) != 8) {
		LOG_ERR("Only 8-bit words are supported");
		return -ENOTSUP;
	}

	if (!spi_cs_is_gpio(config) && (config->slave > 3)) {
		LOG_ERR("Hardware chip select is limited to 0..3");
		return -EINVAL;
	}

	/* Smallest divider (4 << code) that stays at or below the requested rate */
	for (code = 0; code < THEJAS32_SPI_BAUD_CODE_MAX; code++) {
		if (cfg->clock_frequency / (4U << code) <= config->frequency) {
			break;
		}
	}
	sys_write32(THEJAS32_SPI_BAUD_CODE(code), cfg->base + THEJAS32_SPI_BAUD);
	barrier_dmem_fence_full();

	if (config->operation & SPI_MODE_CPOL) {
		mode |= 2;
	}
	if (config->operation & SPI_MODE_CPHA) {
		mode |= 1;
	}

	ctrl = THEJAS32_SPI_CTRL_DBITS(8) | THEJAS32_SPI_CTRL_MODE(mode);
	if (config->operation & SPI_TRANSFER_LSB) {
		ctrl |= THEJAS32_SPI_CTRL_LSB_FIRST;
	}
	if (!spi_cs_is_gpio(config)) {
		ctrl |= THEJAS32_SPI_CTRL_PCS(config->slave);
	}

	data->ctrl = ctrl;
	ctrl_write(dev, ctrl);
	data->ctx.config = config;

	return 0;
}

static int spi_thejas32_xfer(const struct device *dev)
{
	const struct spi_thejas32_config *cfg = dev->config;
	struct spi_thejas32_data *data = dev->data;
	struct spi_context *ctx = &data->ctx;
	int ret = 0;

	/* Keep the hardware chip select asserted for the whole transfer */
	ctrl_write(dev, data->ctrl | THEJAS32_SPI_CTRL_CSAAT);
	spi_context_cs_control(ctx, true);

	while (spi_context_tx_buf_on(ctx) || spi_context_rx_buf_on(ctx)) {
		uint8_t tx = spi_context_tx_buf_on(ctx) ? *ctx->tx_buf : 0U;
		uint8_t rx;

		if (!status_wait(dev, THEJAS32_SPI_STATUS_BUSY, 0) ||
		    !status_wait(dev, THEJAS32_SPI_STATUS_TXHE, THEJAS32_SPI_STATUS_TXHE)) {
			ret = -ETIMEDOUT;
			break;
		}

		sys_write32(tx, cfg->base + THEJAS32_SPI_TXDATA);
		barrier_dmem_fence_full();

		if (!status_wait(dev, THEJAS32_SPI_STATUS_RXC, THEJAS32_SPI_STATUS_RXC)) {
			ret = -ETIMEDOUT;
			break;
		}
		rx = sys_read32(cfg->base + THEJAS32_SPI_RXDATA);

		if (spi_context_rx_buf_on(ctx)) {
			*ctx->rx_buf = rx;
		}
		spi_context_update_tx(ctx, 1, 1);
		spi_context_update_rx(ctx, 1, 1);
	}

	spi_context_cs_control(ctx, false);
	ctrl_write(dev, data->ctrl);
	status_wait(dev, THEJAS32_SPI_STATUS_BUSY, 0);

	return ret;
}

static int spi_thejas32_transceive(const struct device *dev, const struct spi_config *config,
				   const struct spi_buf_set *tx_bufs,
				   const struct spi_buf_set *rx_bufs)
{
	struct spi_thejas32_data *data = dev->data;
	int ret;

	spi_context_lock(&data->ctx, false, NULL, NULL, config);

	ret = spi_thejas32_configure(dev, config);
	if (ret == 0) {
		spi_context_buffers_setup(&data->ctx, tx_bufs, rx_bufs, 1);
		ret = spi_thejas32_xfer(dev);
	}

	spi_context_release(&data->ctx, ret);

	return ret;
}

static int spi_thejas32_release(const struct device *dev, const struct spi_config *config)
{
	struct spi_thejas32_data *data = dev->data;

	spi_context_unlock_unconditionally(&data->ctx);

	return 0;
}

static int spi_thejas32_init(const struct device *dev)
{
	struct spi_thejas32_data *data = dev->data;
	int ret;

	ret = spi_context_cs_configure_all(&data->ctx);
	if (ret < 0) {
		return ret;
	}

	spi_context_unlock_unconditionally(&data->ctx);

	return 0;
}

static DEVICE_API(spi, spi_thejas32_api) = {
	.transceive = spi_thejas32_transceive,
	.release = spi_thejas32_release,
};

#define SPI_THEJAS32_INIT(n)                                                                       \
	static const struct spi_thejas32_config spi_thejas32_config_##n = {                        \
		.base = DT_INST_REG_ADDR(n),                                                       \
		.clock_frequency = DT_INST_PROP(n, clock_frequency),                               \
	};                                                                                         \
	static struct spi_thejas32_data spi_thejas32_data_##n = {                                  \
		SPI_CONTEXT_INIT_LOCK(spi_thejas32_data_##n, ctx),                                 \
		SPI_CONTEXT_INIT_SYNC(spi_thejas32_data_##n, ctx),                                 \
		SPI_CONTEXT_CS_GPIOS_INITIALIZE(DT_DRV_INST(n), ctx)};                             \
	SPI_DEVICE_DT_INST_DEFINE(n, spi_thejas32_init, NULL, &spi_thejas32_data_##n,              \
				  &spi_thejas32_config_##n, POST_KERNEL, CONFIG_SPI_INIT_PRIORITY, \
				  &spi_thejas32_api);

DT_INST_FOREACH_STATUS_OKAY(SPI_THEJAS32_INIT)
