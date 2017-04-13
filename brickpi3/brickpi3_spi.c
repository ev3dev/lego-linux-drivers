/*
 * Dexter Industries BrickPi3 driver
 *
 * Copyright (C) 2017 David Lechner <david@lechnology.com>
 *
 * Based on brickpi3.py by:
 * Copyright (c) 2016 Dexter Industries
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.

 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/delay.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/spi/spi.h>
#include <linux/string.h>

#include "brickpi3.h"

#define BRICKPI3_REQUIRED_FIRMWARE_VERSION	1004000 /* 1.4.x */
#define BRICKPI3_HEADER_SIZE		4
#define BRICKPI3_ID_MSG_SIZE		16
#define BRICKPI3_STRING_MSG_SIZE	20
#define BRICKPI3_MIN_ADDRESS		1
/* technically max address is 255, but we want a reasonable number to probe */
#define BRICKPI3_MAX_ADDRESS		4
#define BRICKPI3_MAX_MSG_SIZE (BRICKPI3_HEADER_SIZE + BRICKPI3_STRING_MSG_SIZE)

#define BRICKPI3_READ_FAILED(b)	((b)[3] != 0xA5)

struct brickpi3 {
	struct spi_device *spi;
	u8 buf[BRICKPI3_MAX_MSG_SIZE];
	struct spi_message msg;
	struct spi_transfer xfer;
	struct mutex xfer_lock;
};

/**
 * brickpi3_write_u8 - Write message with one byte of bp
 *
 * @bp: The private driver bp
 * @address: The BrickPi3 address
 * @msg: The command to send
 * @value: The message bp
 *
 * Returns 0 on success or negative error code.
 */
int brickpi3_write_u8(struct brickpi3 *bp, u8 address, enum brickpi3_message msg,
		      u8 value)
{
	int ret;

	mutex_lock(&bp->xfer_lock);

	bp->buf[0] = address;
	bp->buf[1] = msg;
	bp->buf[2] = value;
	bp->xfer.len = 3;

	ret = spi_sync(bp->spi, &bp->msg);

	mutex_unlock(&bp->xfer_lock);

	return ret;
}

/**
 * brickpi3_read_u16 - Read message with two bytes of bp
 *
 * @bp: The private driver bp
 * @address: The BrickPi3 address
 * @msg: The command to send
 * @value: The returned message bp
 *
 * Returns 0 on success or negative error code.
 */
int brickpi3_read_u16(struct brickpi3 *bp, u8 address, enum brickpi3_message msg,
		      u16 *value)
{
	int ret = 0;

	mutex_lock(&bp->xfer_lock);

	bp->buf[0] = address;
	bp->buf[1] = msg;
	bp->buf[2] = 0;
	bp->buf[3] = 0;
	bp->buf[4] = 0;
	bp->buf[5] = 0;
	bp->xfer.len = 6;

	ret = spi_sync(bp->spi, &bp->msg);
	if (ret < 0)
		goto out;

	if (BRICKPI3_READ_FAILED(bp->buf)) {
		ret = -EIO;
		goto out;
	}

	*value = (bp->buf[4] << 8) | bp->buf[5];

out:
	mutex_unlock(&bp->xfer_lock);

	return ret;
}

/**
 * brickpi3_write_u16 - Write message with two bytes of bp
 *
 * @bp: The private driver bp
 * @address: The BrickPi3 address
 * @msg: The command to send
 * @value: The message bp
 *
 * Returns 0 on success or negative error code.
 */
int brickpi3_write_u16(struct brickpi3 *bp, u8 address,
		       enum brickpi3_message msg, u16 value)
{
	int ret;

	mutex_lock(&bp->xfer_lock);

	bp->buf[0] = address;
	bp->buf[1] = msg;
	bp->buf[2] = (value >> 8) & 0xff;
	bp->buf[3] = value & 0xff;
	bp->xfer.len = 4;

	ret = spi_sync(bp->spi, &bp->msg);

	mutex_unlock(&bp->xfer_lock);

	return ret;
}

int brickpi3_write_u8_u16(struct brickpi3 *bp, u8 address,
			  enum brickpi3_message msg, u8 value1, u16 value2)
{
	int ret;

	mutex_lock(&bp->xfer_lock);

	bp->buf[0] = address;
	bp->buf[1] = msg;
	bp->buf[2] = value1;
	bp->buf[3] = (value2 >> 8) & 0xff;
	bp->buf[4] = value2 & 0xff;
	bp->xfer.len = 5;

	ret = spi_sync(bp->spi, &bp->msg);

	mutex_unlock(&bp->xfer_lock);

	return ret;
}

static int brickpi3_set_address(struct brickpi3 *bp, u8 address, u8 id[16])
{
	int ret;

	mutex_lock(&bp->xfer_lock);

	bp->buf[0] = 0;
	bp->buf[1] = BRICKPI3_MSG_SET_ADDRESS;
	bp->buf[2] = address;
	strncpy(&bp->buf[3], id, 16);
	bp->xfer.len = 19;

	ret = spi_sync(bp->spi, &bp->msg);

	mutex_unlock(&bp->xfer_lock);

	return ret;
}

int brickpi3_set_motor_limits(struct brickpi3 *bp, u8 address,
			      enum brickpi3_output_port port,
			      u8 duty_cycle_sp, u16 speed)
{
	int ret;

	mutex_lock(&bp->xfer_lock);

	bp->buf[0] = address;
	bp->buf[1] = BRICKPI3_MSG_SET_MOTOR_LIMITS;
	bp->buf[2] = BIT(port);
	bp->buf[3] = duty_cycle_sp;
	bp->buf[4] = (speed >> 8) & 0xff;
	bp->buf[5] = speed & 0xff;
	bp->xfer.len = 6;

	ret = spi_sync(bp->spi, &bp->msg);

	mutex_unlock(&bp->xfer_lock);

	return ret;
}

int brickpi3_write_u8_u8(struct brickpi3 *bp, u8 address,
			 enum brickpi3_message msg, u8 value1, u8 value2)
{
	int ret;

	mutex_lock(&bp->xfer_lock);

	bp->buf[0] = address;
	bp->buf[1] = msg;
	bp->buf[2] = value1;
	bp->buf[3] = value2;
	bp->xfer.len = 4;

	ret = spi_sync(bp->spi, &bp->msg);

	mutex_unlock(&bp->xfer_lock);

	return ret;
}

/**
 * brickpi3_write_u24 - Write message with three bytes of bp
 *
 * @bp: The private driver bp
 * @address: The BrickPi3 address
 * @msg: The command to send
 * @value: The message bp
 *
 * Returns 0 on success or negative error code.
 */
int brickpi3_write_u24(struct brickpi3 *bp, u8 address,
		       enum brickpi3_message msg, u32 value)
{
	int ret;

	mutex_lock(&bp->xfer_lock);

	bp->buf[0] = address;
	bp->buf[1] = msg;
	bp->buf[2] = (value >> 16) & 0xff;
	bp->buf[3] = (value >> 8) & 0xff;
	bp->buf[4] = value & 0xff;
	bp->xfer.len = 5;

	ret = spi_sync(bp->spi, &bp->msg);

	mutex_unlock(&bp->xfer_lock);

	return ret;
}

/**
 * brickpi3_read_u32 - Read message with four bytes of bp
 *
 * @bp: The private driver bp
 * @address: The BrickPi3 address
 * @msg: The command to send
 * @value: The returned message bp
 *
 * Returns 0 on success or negative error code.
 */
int brickpi3_read_u32(struct brickpi3 *bp, u8 address,
		      enum brickpi3_message msg, u32 *value)
{
	int ret = 0;

	mutex_lock(&bp->xfer_lock);

	bp->buf[0] = address;
	bp->buf[1] = msg;
	bp->buf[2] = 0;
	bp->buf[3] = 0;
	bp->buf[4] = 0;
	bp->buf[5] = 0;
	bp->buf[6] = 0;
	bp->buf[7] = 0;
	bp->xfer.len = 8;

	ret = spi_sync(bp->spi, &bp->msg);
	if (ret < 0)
		goto out;

	if (BRICKPI3_READ_FAILED(bp->buf)) {
		ret = -EIO;
		goto out;
	}

	*value = (bp->buf[4] << 24) | (bp->buf[5] << 16) |
		 (bp->buf[6] << 8) | bp->buf[7];

out:
	mutex_unlock(&bp->xfer_lock);

	return ret;
}

/**
 * brickpi3_write_u32 - Write message with four bytes of bp
 *
 * @bp: The private driver bp
 * @address: The BrickPi3 address
 * @msg: The command to send
 * @value: The message bp
 *
 * Returns 0 on success or negative error code.
 */
int brickpi3_write_u32(struct brickpi3 *bp, u8 address,
		       enum brickpi3_message msg, u32 value)
{
	int ret;

	mutex_lock(&bp->xfer_lock);

	bp->buf[0] = address;
	bp->buf[1] = msg;
	bp->buf[2] = (value >> 24) & 0xff;
	bp->buf[3] = (value >> 16) & 0xff;
	bp->buf[4] = (value >> 8) & 0xff;
	bp->buf[5] = value & 0xff;
	bp->xfer.len = 6;

	ret = spi_sync(bp->spi, &bp->msg);

	mutex_unlock(&bp->xfer_lock);

	return ret;
}

int brickpi3_write_u8_u32(struct brickpi3 *bp, u8 address,
			  enum brickpi3_message msg, u8 value1, u32 value2)
{
	int ret;

	mutex_lock(&bp->xfer_lock);

	bp->buf[0] = address;
	bp->buf[1] = msg;
	bp->buf[2] = value1;
	bp->buf[3] = (value2 >> 24) & 0xff;
	bp->buf[4] = (value2 >> 16) & 0xff;
	bp->buf[5] = (value2 >> 8) & 0xff;
	bp->buf[6] = value2 & 0xff;
	bp->xfer.len = 7;

	ret = spi_sync(bp->spi, &bp->msg);

	mutex_unlock(&bp->xfer_lock);

	return ret;
}

/**
 * brickpi3_read_string - Read message with arbitrary bytes of bp
 *
 * @bp: The private driver bp
 * @address: The BrickPi3 address
 * @msg: The command to send
 * @value: Caller-allocated array to hold the returned message bp
 * @len: The length of the array - must be <= BRICKPI3_STRING_MSG_SIZE
 *
 * Returns 0 on success or negative error code.
 */
int brickpi3_read_string(struct brickpi3 *bp, u8 address,
			 enum brickpi3_message msg, char *value, size_t len)
{
	int ret = 0;

	mutex_lock(&bp->xfer_lock);

	bp->buf[0] = address;
	bp->buf[1] = msg;
	bp->buf[2] = 0;
	bp->buf[3] = 0;
	memset(&bp->buf[BRICKPI3_HEADER_SIZE], 0, len);
	bp->xfer.len = BRICKPI3_HEADER_SIZE + len;

	ret = spi_sync(bp->spi, &bp->msg);
	if (ret < 0)
		goto out;

	if (BRICKPI3_READ_FAILED(bp->buf)) {
		ret = -EIO;
		goto out;
	}

	memcpy(value, &bp->buf[BRICKPI3_HEADER_SIZE], len);

out:
	mutex_unlock(&bp->xfer_lock);

	return ret;
}

/**
 * brickpi3_read_sensor - Read sensor bp
 *
 * @bp: The private driver bp
 * @address: The BrickPi3 address
 * @port: The input port index
 * @type: The input port type
 * @value: Caller-allocated array to hold the returned message bp
 * @len: The length of the array - must be <= BRICKPI3_MAX_MSG_SIZE - 6
 *
 * Returns 0 on success or negative error code.
 */
int brickpi3_read_sensor(struct brickpi3 *bp, u8 address,
			 enum brickpi3_input_port port,
			 enum brickpi3_sensor_type type, char *value,
			 size_t len)
{
	int ret = 0;

	mutex_lock(&bp->xfer_lock);

	bp->buf[0] = address;
	bp->buf[1] = BRICKPI3_MSG_GET_SENSOR + port;
	bp->buf[2] = 0;
	bp->buf[3] = 0;
	bp->buf[4] = 0;
	bp->buf[5] = 0;
	memset(&bp->buf[6], 0, len);
	bp->xfer.len = 6 + len;

	ret = spi_sync(bp->spi, &bp->msg);
	if (ret < 0)
		goto out;

	if (BRICKPI3_READ_FAILED(bp->buf)) {
		ret = -EIO;
		goto out;
	}

	if (bp->buf[4] != type) {
		ret = -EIO;
		goto out;
	}

	if (bp->buf[5] != BRICKPI3_SENSOR_STATE_VALID_DATA) {
		ret = -EIO;
		goto out;
	}

	memcpy(value, &bp->buf[6], len);

out:
	mutex_unlock(&bp->xfer_lock);

	return ret;
}

/**
 * brickpi3_set_sensor_custom - Sets input port to custom type
 *
 * @bp: The private driver bp
 * @address: The BrickPi3 address
 * @port: The index of the input port
 * @flags: Flags for configuring the input port
 *
 * Returns 0 on success or negative error code.
 */
int brickpi3_set_sensor_custom(struct brickpi3 *bp, u8 address,
			       enum brickpi3_input_port port,
			       enum brickpi3_sensor_pin_flags flags)
{
	int ret = 0;

	mutex_lock(&bp->xfer_lock);

	bp->buf[0] = address;
	bp->buf[1] = BRICKPI3_MSG_SET_SENSOR_TYPE;
	bp->buf[2] = BIT(port);
	bp->buf[3] = BRICKPI3_SENSOR_TYPE_CUSTOM;
	bp->buf[4] = (flags >> 8) & 0xff;
	bp->buf[5] = flags & 0xff;
	bp->xfer.len = 6;

	ret = spi_sync(bp->spi, &bp->msg);

	mutex_unlock(&bp->xfer_lock);

	return ret;
}

/**
 * brickpi3_set_sensor_i2c - Sets input port to I2C type
 *
 * @bp: The private driver bp
 * @address: The BrickPi3 address
 * @port: The index of the input port
 * @flags: Flags for configuring the input port
 * @speed: The I2C bus speed (in microseconds???)
 *
 * Returns 0 on success or negative error code.
 */
int brickpi3_set_sensor_i2c(struct brickpi3 *bp, u8 address,
			    enum brickpi3_input_port port,
			    enum brickpi3_i2c_flags flags,
			    u8 speed)
{
	int ret = 0;

	mutex_lock(&bp->xfer_lock);

	bp->buf[0] = address;
	bp->buf[1] = BRICKPI3_MSG_SET_SENSOR_TYPE;
	bp->buf[2] = BIT(port);
	bp->buf[3] = BRICKPI3_SENSOR_TYPE_I2C;
	bp->buf[4] = flags;
	bp->buf[5] = speed;
	bp->xfer.len = 6;
	/* TODO: handle extra params for (flags & BRICKPI3_I2C_SAME) */

	ret = spi_sync(bp->spi, &bp->msg);

	mutex_unlock(&bp->xfer_lock);

	return ret;
}

/**
 * brickpi3_i2c_transact - Do and I2C transaction
 *
 * @bp: The private driver bp
 * @address: The BrickPi3 address
 * @port: The input port
 * @i2c_addr: The I2C address
 * @write_buf: Array of bp to write
 * @write_size: The number of bytes to write (<= BRICKPI3_I2C_MAX_WRITE_SIZE)
 * @read_buf: The buffer to store the read bp
 * @read_size: The number of bytes to read (<= BRICKPI3_I2C_MAX_READ_SIZE)
 *
 * Returns 0 on success or negative error code.
 */
int brickpi3_i2c_transact(struct brickpi3 *bp, u8 address,
			  enum brickpi3_input_port port, u8 i2c_addr,
			  u8 *write_buf, u8 write_size,
			  u8 *read_buf, u8 read_size)
{
	int ret = 0;

	if (read_size > BRICKPI3_I2C_MAX_READ_SIZE)
		return -EINVAL;
	if (write_size > BRICKPI3_I2C_MAX_WRITE_SIZE)
		return -EINVAL;

	mutex_lock(&bp->xfer_lock);

	/*
	 * TODO: It might be better to error early if we know the port is not
	 * already in I2C mode. For now, we will return -EBUSY when we try to
	 * read back the response if the mode was not set.
	 */

	bp->buf[0] = address;
	bp->buf[1] = BRICKPI3_MSG_I2C_TRANSACT + port;
	bp->buf[2] = i2c_addr << 1;
	bp->buf[3] = read_size;
	bp->buf[4] = write_size;
	memcpy(&bp->buf[5], write_buf, write_size);
	bp->xfer.len = 5 + write_size;

	ret = spi_sync(bp->spi, &bp->msg);
	if (ret < 0)
		goto out;

	if (BRICKPI3_READ_FAILED(bp->buf)) {
		ret = -EIO;
		goto out;
	}

	if (read_buf && read_size) {
		/*
		 * FIXME: msleep is never a good idea. Perhaps we could keep
		 * polling until spi_sync() returns BRICKPI3_SENSOR_STATE_VALID_DATA
		 * along with a timeout mechanism.
		 *
		 * For now, we are estimating one msec per byte (I2C operates
		 * at 10kHz)
		 */
		msleep(write_size + read_size + 2);

		bp->buf[0] = address;
		bp->buf[1] = BRICKPI3_MSG_GET_SENSOR + port;
		memset(&bp->buf[2], 0, 4 + read_size);
		bp->xfer.len = 6 + read_size;

		ret = spi_sync(bp->spi, &bp->msg);
		if (ret < 0)
			goto out;

		if (BRICKPI3_READ_FAILED(bp->buf)) {
			ret = -EIO;
			goto out;
		}
		if (bp->buf[4] != BRICKPI3_SENSOR_TYPE_I2C) {
			ret = -EBUSY;
			goto out;
		}
		if (bp->buf[5] != BRICKPI3_SENSOR_STATE_VALID_DATA) {
			ret = -EIO;
			goto out;
		}

		memcpy(read_buf, &bp->buf[6], read_size);
	}

out:
	mutex_unlock(&bp->xfer_lock);

	return ret;
}

static int brickpi3_detect(struct brickpi3 *bp, u8 address)
{
	struct device *dev = &bp->spi->dev;
	char string[BRICKPI3_STRING_MSG_SIZE + 1];
	u32 value;
	int ret;

	/* ensure null terminator */
	string[BRICKPI3_STRING_MSG_SIZE] = 0;

	ret = brickpi3_read_string(bp, address, BRICKPI3_MSG_GET_MANUFACTURER,
				   string, BRICKPI3_STRING_MSG_SIZE);
	if (ret < 0)
		return ret;

	dev_info(dev, "Address: %u\n", address);
	dev_info(dev, "Mfg: %s\n", string);
	if (strncmp(string, "Dexter Industries", BRICKPI3_STRING_MSG_SIZE) != 0)
		return -EINVAL;

	ret = brickpi3_read_string(bp, address, BRICKPI3_MSG_GET_NAME,
				   string, BRICKPI3_STRING_MSG_SIZE);
	if (ret < 0)
		return ret;

	dev_info(dev, "Board: %s\n", string);
	if (strncmp(string, "BrickPi3", BRICKPI3_STRING_MSG_SIZE) != 0)
		return -EINVAL;

	ret = brickpi3_read_u32(bp, address, BRICKPI3_MSG_GET_HARDWARE_VERSION,
				&value);
	if (ret < 0)
		return ret;

	dev_info(dev, "HW: %u.%u.%u\n", value / 1000000 % 1000000,
		 value / 1000 % 1000, value % 1000);

	ret = brickpi3_read_u32(bp, address, BRICKPI3_MSG_GET_FIRMWARE_VERSION,
				&value);
	if (ret < 0)
		return ret;

	dev_info(dev, "FW: %u.%u.%u\n", value / 1000000 % 1000000,
		 value / 1000 % 1000, value % 1000);
	if (value < BRICKPI3_REQUIRED_FIRMWARE_VERSION ||
	    value > BRICKPI3_REQUIRED_FIRMWARE_VERSION + 999) {
		dev_err(dev, "Unsupported firmware version, expecting %u.%u.x\n",
			BRICKPI3_REQUIRED_FIRMWARE_VERSION / 1000000 % 1000000,
			BRICKPI3_REQUIRED_FIRMWARE_VERSION / 1000 % 1000);
		return -EINVAL;
	}
	ret = brickpi3_read_string(bp, address, BRICKPI3_MSG_GET_ID, string,
				   BRICKPI3_ID_MSG_SIZE);
	if (ret < 0)
		return ret;

	dev_info(dev, "ID: %02X%02X%02X%02X%02X%02X%02X%02X"
		 "%02X%02X%02X%02X%02X%02X%02X%02X\n",
		 string[0], string[1], string[2], string[3],
		 string[4], string[5], string[6], string[7],
		 string[8], string[9], string[10], string[11],
		 string[12], string[13], string[14], string[15]);

	return 0;
}

static void brickpi3_set_addresses(struct brickpi3 *bp) {
	struct device *dev = &bp->spi->dev;
	char id_property[20];
	const char *id;
	char ids[3];
	u8 idn[16];
	int i, j, ret;

	for (i = BRICKPI3_MIN_ADDRESS; i <= BRICKPI3_MAX_ADDRESS; i++) {
		snprintf(id_property, 20, "ev3dev,brickpi3-id%d", i);

		ret = of_property_read_string(dev->of_node, id_property, &id);
		if (ret < 0)
			continue;

		if (strlen(id) != 32) {
			dev_warn(dev, "id%d must be 32 chars, skipping\n", i);
			continue;
		}

		for (j = 0; j < 16; j++) {
			ids[0] = id[2 * j];
			ids[1] = id[2 * j + 1];
			ids[2] = '\0';
			ret = sscanf(ids, "%hhX", &idn[j]);
			if (ret != 1) {
				dev_warn(dev, "Bad id%d: %s, skipping\n", i, id);
				break;
			}
		}
		if (j != 16)
			continue;

		brickpi3_set_address(bp, i, idn);
	}
}

static int brickpi3_probe(struct spi_device *spi)
{
	struct device *dev = &spi->dev;
	struct brickpi3 *bp;
	int i, ret;
	bool ok = false;

	bp = devm_kzalloc(dev, sizeof(*bp), GFP_KERNEL);
	if (!bp)
		return -ENOMEM;

	dev_set_drvdata(dev, bp);

	bp->spi = spi;
	bp->xfer.tx_buf = bp->buf;
	bp->xfer.rx_buf = bp->buf;
	spi_message_init_with_transfers(&bp->msg, &bp->xfer, 1);
	mutex_init(&bp->xfer_lock);

	brickpi3_set_addresses(bp);

	for (i = BRICKPI3_MIN_ADDRESS; i <= BRICKPI3_MAX_ADDRESS; i++) {
		ret = brickpi3_detect(bp, i);
		if (ret < 0)
			continue;

		ret = devm_brickpi3_register_leds(dev, bp, i);
		if (ret < 0)
			return ret;

		ret = devm_brickpi3_register_iio(dev, bp, i);
		if (ret < 0)
			return ret;

		ret = devm_brickpi3_register_i2c(dev, bp, i);
		if (ret < 0)
			return ret;

		ret = devm_brickpi3_register_in_ports(dev, bp, i);
		if (ret < 0)
			return ret;

		ret = devm_brickpi3_register_out_ports(dev, bp, i);
		if (ret < 0)
			return ret;

		ok = true;
	}

	return ok ? 0 : -ENODEV;
}

const static struct of_device_id brickpi3_of_match_table[] = {
	{ .compatible = "ev3dev,brickpi3", },
	{ }
};
MODULE_DEVICE_TABLE(of, brickpi3_of_match_table);

static struct spi_driver brickpi3_driver = {
	.driver	= {
		.name		= "brickpi3",
		.of_match_table	= brickpi3_of_match_table,
	},
	.probe	= brickpi3_probe,
};
module_spi_driver(brickpi3_driver);

MODULE_DESCRIPTION("Dexter Industries BrickPi3");
MODULE_AUTHOR("David Lechner <david@lechnology.com>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("spi:brickpi3");
