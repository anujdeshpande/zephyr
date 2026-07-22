/*
 * Copyright (c) 2026 Anuj Deshpande
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT cdac_thejas32_i2c

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/sys/barrier.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/sys/util.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(i2c_thejas32, CONFIG_I2C_LOG_LEVEL);

#include "i2c-priv.h"

/* Byte-wide registers */
#define THEJAS32_I2C_CR    0x00 /* control */
#define THEJAS32_I2C_SR0   0x01 /* status 0 */
#define THEJAS32_I2C_SR1   0x02 /* status 1 */
#define THEJAS32_I2C_IER   0x03 /* interrupt enable */
#define THEJAS32_I2C_TXFF  0x04 /* TX FIFO */
#define THEJAS32_I2C_RXFF  0x05 /* RX FIFO */
#define THEJAS32_I2C_CHL   0x06 /* clock half period, low byte */
#define THEJAS32_I2C_CHH   0x07 /* clock half period, high byte */
#define THEJAS32_I2C_CHHL  0x08 /* clock quarter period, low byte */
#define THEJAS32_I2C_CHHH  0x09 /* clock quarter period, high byte */
#define THEJAS32_I2C_TXCLR 0x0a /* TX FIFO clear */

#define THEJAS32_I2C_CR_START    BIT(0)
#define THEJAS32_I2C_CR_STOP     BIT(1)
/* Number of bytes the controller reads on its own after the address phase */
#define THEJAS32_I2C_CR_RDLEN(n) (((n) & 0x1f) << 2)

#define THEJAS32_I2C_SR0_STAS  BIT(0) /* start condition sent */
#define THEJAS32_I2C_SR0_STPS  BIT(1) /* stop condition sent */
#define THEJAS32_I2C_SR0_TXFFF BIT(2) /* TX FIFO full */
#define THEJAS32_I2C_SR0_TXFFE BIT(3) /* TX FIFO empty */
#define THEJAS32_I2C_SR0_TXC   BIT(4) /* transmit complete */
#define THEJAS32_I2C_SR0_RXFFF BIT(5) /* RX FIFO full */
#define THEJAS32_I2C_SR0_RXFFE BIT(6) /* RX FIFO empty */
#define THEJAS32_I2C_SR0_RXC   BIT(7) /* receive complete */

#define THEJAS32_I2C_SR1_NACK BIT(0)

/* The RX FIFO is 16 bytes deep and RDLEN caps a single read at that */
#define THEJAS32_I2C_MAX_READ 16

#define THEJAS32_I2C_TIMEOUT_US 20000

struct i2c_thejas32_config {
	mem_addr_t base;
	uint32_t input_clock;
	uint32_t default_bitrate;
};

struct i2c_thejas32_data {
	struct k_mutex lock;
	uint32_t dev_config;
};

static uint8_t reg_read(const struct device *dev, uint32_t off)
{
	const struct i2c_thejas32_config *cfg = dev->config;

	return sys_read8(cfg->base + off);
}

static void reg_write(const struct device *dev, uint32_t off, uint8_t val)
{
	const struct i2c_thejas32_config *cfg = dev->config;

	sys_write8(val, cfg->base + off);
	barrier_dmem_fence_full();
}

static bool sr0_wait(const struct device *dev, uint8_t mask, uint8_t val)
{
	return WAIT_FOR((reg_read(dev, THEJAS32_I2C_SR0) & mask) == val, THEJAS32_I2C_TIMEOUT_US,
			k_busy_wait(1));
}

static int i2c_thejas32_configure(const struct device *dev, uint32_t dev_config)
{
	const struct i2c_thejas32_config *cfg = dev->config;
	struct i2c_thejas32_data *data = dev->data;
	uint32_t bitrate;
	uint16_t half, quarter;

	if (!(dev_config & I2C_MODE_CONTROLLER)) {
		return -ENOTSUP;
	}

	switch (I2C_SPEED_GET(dev_config)) {
	case I2C_SPEED_STANDARD:
		bitrate = I2C_BITRATE_STANDARD;
		break;
	case I2C_SPEED_FAST:
		bitrate = I2C_BITRATE_FAST;
		break;
	default:
		return -ENOTSUP;
	}

	half = cfg->input_clock / (2U * bitrate);
	quarter = cfg->input_clock / (4U * bitrate);

	k_mutex_lock(&data->lock, K_FOREVER);
	reg_write(dev, THEJAS32_I2C_TXCLR, 0xff);
	reg_write(dev, THEJAS32_I2C_CHL, half & 0xff);
	reg_write(dev, THEJAS32_I2C_CHH, half >> 8);
	reg_write(dev, THEJAS32_I2C_CHHL, quarter & 0xff);
	reg_write(dev, THEJAS32_I2C_CHHH, quarter >> 8);
	data->dev_config = dev_config;
	k_mutex_unlock(&data->lock);

	return 0;
}

static int i2c_thejas32_get_config(const struct device *dev, uint32_t *dev_config)
{
	struct i2c_thejas32_data *data = dev->data;

	*dev_config = data->dev_config;

	return 0;
}

/* Ends the current transaction; on entry a byte may still be shifting out */
static int stop(const struct device *dev)
{
	reg_write(dev, THEJAS32_I2C_CR, THEJAS32_I2C_CR_STOP);
	if (!sr0_wait(dev, THEJAS32_I2C_SR0_STPS, THEJAS32_I2C_SR0_STPS)) {
		return -ETIMEDOUT;
	}

	return 0;
}

/* Waits out the address/data byte just queued and checks its acknowledge */
static int wait_byte_acked(const struct device *dev)
{
	if (!sr0_wait(dev, THEJAS32_I2C_SR0_TXC, THEJAS32_I2C_SR0_TXC)) {
		return -ETIMEDOUT;
	}

	if (reg_read(dev, THEJAS32_I2C_SR1) & THEJAS32_I2C_SR1_NACK) {
		/* The controller raises stop by itself on a NACK */
		sr0_wait(dev, THEJAS32_I2C_SR0_STPS, THEJAS32_I2C_SR0_STPS);
		return -EIO;
	}

	return 0;
}

static int write_msg(const struct device *dev, uint16_t addr, const struct i2c_msg *msg)
{
	int ret;

	reg_write(dev, THEJAS32_I2C_TXCLR, 0xff);
	if (!sr0_wait(dev, THEJAS32_I2C_SR0_TXFFE | THEJAS32_I2C_SR0_TXC,
		      THEJAS32_I2C_SR0_TXFFE | THEJAS32_I2C_SR0_TXC)) {
		return -ETIMEDOUT;
	}

	reg_write(dev, THEJAS32_I2C_CR, THEJAS32_I2C_CR_START);
	if (!sr0_wait(dev, THEJAS32_I2C_SR0_STAS, THEJAS32_I2C_SR0_STAS)) {
		return -ETIMEDOUT;
	}

	reg_write(dev, THEJAS32_I2C_TXFF, addr << 1);
	ret = wait_byte_acked(dev);
	if (ret < 0) {
		return ret;
	}

	for (uint32_t i = 0; i < msg->len; i++) {
		if (!sr0_wait(dev, THEJAS32_I2C_SR0_TXFFF, 0)) {
			return -ETIMEDOUT;
		}
		reg_write(dev, THEJAS32_I2C_TXFF, msg->buf[i]);
		ret = wait_byte_acked(dev);
		if (ret < 0) {
			return ret;
		}
	}

	return stop(dev);
}

static int read_chunk(const struct device *dev, uint16_t addr, uint8_t *buf, uint8_t len)
{
	int ret;

	/* Preload the read length; the controller clocks the bytes in itself */
	reg_write(dev, THEJAS32_I2C_CR, THEJAS32_I2C_CR_RDLEN(len) | THEJAS32_I2C_CR_START);
	if (!sr0_wait(dev, THEJAS32_I2C_SR0_TXFFF, 0)) {
		return -ETIMEDOUT;
	}

	reg_write(dev, THEJAS32_I2C_TXFF, (addr << 1) | 1U);
	ret = wait_byte_acked(dev);
	if (ret < 0) {
		return ret;
	}

	if (!sr0_wait(dev, THEJAS32_I2C_SR0_RXC, THEJAS32_I2C_SR0_RXC)) {
		return -ETIMEDOUT;
	}

	for (uint8_t i = 0; i < len; i++) {
		if (!sr0_wait(dev, THEJAS32_I2C_SR0_RXFFE, 0)) {
			return -ETIMEDOUT;
		}
		buf[i] = reg_read(dev, THEJAS32_I2C_RXFF);
	}

	if (!sr0_wait(dev, THEJAS32_I2C_SR0_STPS, THEJAS32_I2C_SR0_STPS)) {
		return -ETIMEDOUT;
	}

	return stop(dev);
}

static int read_msg(const struct device *dev, uint16_t addr, const struct i2c_msg *msg)
{
	uint32_t done = 0;

	if (msg->len == 0) {
		/* Address-only probe: run it as an empty write */
		struct i2c_msg probe = {.len = 0};

		return write_msg(dev, addr, &probe);
	}

	/*
	 * The read length register caps a transaction at the RX FIFO depth,
	 * so longer reads are issued as repeated addressed transactions.
	 */
	while (done < msg->len) {
		uint8_t chunk = MIN(msg->len - done, THEJAS32_I2C_MAX_READ);
		int ret = read_chunk(dev, addr, &msg->buf[done], chunk);

		if (ret < 0) {
			return ret;
		}
		done += chunk;
	}

	return 0;
}

static int i2c_thejas32_transfer(const struct device *dev, struct i2c_msg *msgs, uint8_t num_msgs,
				 uint16_t addr)
{
	struct i2c_thejas32_data *data = dev->data;
	int ret = 0;

	k_mutex_lock(&data->lock, K_FOREVER);

	for (uint8_t i = 0; i < num_msgs; i++) {
		if (msgs[i].flags & I2C_MSG_ADDR_10_BITS) {
			ret = -ENOTSUP;
			break;
		}

		/*
		 * The controller generates a stop after every transaction and
		 * cannot hold the bus, so each message is a self-contained
		 * start/stop transfer and repeated start is not available.
		 */
		if (msgs[i].flags & I2C_MSG_READ) {
			ret = read_msg(dev, addr, &msgs[i]);
		} else {
			ret = write_msg(dev, addr, &msgs[i]);
		}

		if (ret < 0) {
			break;
		}
	}

	k_mutex_unlock(&data->lock);

	return ret;
}

static int i2c_thejas32_init(const struct device *dev)
{
	const struct i2c_thejas32_config *cfg = dev->config;
	struct i2c_thejas32_data *data = dev->data;

	k_mutex_init(&data->lock);

	return i2c_thejas32_configure(dev, I2C_MODE_CONTROLLER |
						   i2c_map_dt_bitrate(cfg->default_bitrate));
}

static DEVICE_API(i2c, i2c_thejas32_api) = {
	.configure = i2c_thejas32_configure,
	.get_config = i2c_thejas32_get_config,
	.transfer = i2c_thejas32_transfer,
#ifdef CONFIG_I2C_RTIO
	.iodev_submit = i2c_iodev_submit_fallback,
#endif
};

#define I2C_THEJAS32_INIT(n)                                                                       \
	static const struct i2c_thejas32_config i2c_thejas32_config_##n = {                        \
		.base = DT_INST_REG_ADDR(n),                                                       \
		.input_clock = DT_INST_PROP(n, input_clock_frequency),                             \
		.default_bitrate = DT_INST_PROP_OR(n, clock_frequency, I2C_BITRATE_STANDARD),      \
	};                                                                                         \
	static struct i2c_thejas32_data i2c_thejas32_data_##n;                                     \
	I2C_DEVICE_DT_INST_DEFINE(n, i2c_thejas32_init, NULL, &i2c_thejas32_data_##n,              \
				  &i2c_thejas32_config_##n, POST_KERNEL, CONFIG_I2C_INIT_PRIORITY, \
				  &i2c_thejas32_api);

DT_INST_FOREACH_STATUS_OKAY(I2C_THEJAS32_INIT)
