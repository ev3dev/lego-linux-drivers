/*
 * Dexter Industries BrickPi3 I2C adapter driver
 *
 * Copyright (C) 2017 David Lechner <david@lechnology.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.

 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

/**
 * DOC: userspace
 *
 * The ``brickpi3`` modules creates one I2C adapter for each input port. These
 * are available at ``/dev/i2c-X`` where ``X`` is the number of the input port
 * plus 2. The adapters can only be used when the port is set to ``nxt-i2c``
 * mode. Only SMBUS messages are supported and are limited to 16 bytes.
 */

#include <linux/device.h>
#include <linux/i2c.h>

#include "brickpi3.h"


struct brickpi3_i2c {
	struct brickpi3 *bp;
	struct i2c_adapter adap;
	enum brickpi3_input_port port;
};

static int brickpi3_i2c_master_xfer(struct i2c_adapter *adap,
				    struct i2c_msg *msgs, int num)
{
	struct brickpi3_i2c *data = adap->algo_data;
	u8 write_size, read_size;
	int ret;

	/*
	 * Hopefully i2c core is doing lots of checking for us based on
	 * I2C_AQ_COMB_WRITE_THEN_READ. We only support one or two messages,
	 * the first is a write and the second is a read. Both are limited
	 * in size to < SMBUS standard.
	 */
	write_size = msgs[0].len;
	read_size = num == 2 ? msgs[1].len : 0;

	ret = brickpi3_i2c_transact(data->bp, data->port, msgs[0].addr,
				    msgs[0].buf, write_size,
				    num == 2 ? msgs[1].buf : NULL, read_size);
	if (ret < 0)
		return ret;

	return num;
}

static u32 brickpi3_i2c_functionality(struct i2c_adapter *adap)
{
	return I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL;
}

static const struct i2c_algorithm brickpi3_i2c_algo = {
	.master_xfer	= brickpi3_i2c_master_xfer,
	.functionality	= brickpi3_i2c_functionality,
};

static const struct i2c_adapter_quirks brickpi3_i2c_quirks = {
	.flags			= I2C_AQ_COMB_WRITE_THEN_READ,
	.max_write_len		= BRICKPI3_I2C_MAX_WRITE_SIZE,
	.max_read_len		= BRICKPI3_I2C_MAX_READ_SIZE,
};

static void brickpi3_i2c_release(struct device *dev, void *res)
{
	struct brickpi3_i2c *data = res;

	i2c_del_adapter(&data->adap);
}

static int devm_brickpi3_i2c_register_one(struct device *dev,
					  struct brickpi3 *bp,
					  enum brickpi3_input_port port)
{
	struct brickpi3_i2c *data;
	int ret;

	data = devres_alloc(brickpi3_i2c_release, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->bp = bp;
	data->port = port;

	data->adap.owner = THIS_MODULE;
	data->adap.algo = &brickpi3_i2c_algo;
	data->adap.algo_data = data;
	data->adap.timeout = HZ; /* 1 second */
	data->adap.dev.parent = dev;
	data->adap.nr = -1;
	snprintf(data->adap.name, sizeof(data->adap.name), "brickpi3-i2c%d", port);
	data->adap.quirks = &brickpi3_i2c_quirks;

	ret = i2c_add_numbered_adapter(&data->adap);
	if (ret < 0)
		return ret;

	devres_add(dev, data);

	return 0;
}

int devm_brickpi3_register_i2c(struct device *dev, struct brickpi3 *bp)
{
	int i, ret;

	for (i = 0; i < NUM_BRICKPI3_INPUT_PORTS; i++) {
		ret = devm_brickpi3_i2c_register_one(dev, bp, i);
		if (ret < 0)
			return ret;
	}

	return 0;
}
