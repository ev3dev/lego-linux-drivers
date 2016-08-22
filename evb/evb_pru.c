/*
 * Software I2C PRU driver for Fatcatlab EVB
 *
 * Copyright (C) 2016 David Lechner <david@lechnology.com>
 *
 * Based on i2c-gpio.c:
 * Copyright (C) 2007 Atmel Corporation
 *
 * and davinci_iic.c from lms2012
 * the file does not contain a copyright, but comes from the LEGO Group
 *
 * Also based on: rpmsg_client_sample.c
 *
 * Remote processor messaging - sample client driver
 *
 * Copyright (C) 2011 Texas Instruments, Inc.
 * Copyright (C) 2011 Google, Inc.
 *
 * Ohad Ben-Cohen <ohad@wizery.com>
 * Brian Swetland <swetland@google.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.

 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/completion.h>
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/rpmsg.h>

/*
 * Everything between here and "END PRU DATA STRUCTS" must exactly match the PRU
 * firmware. These structs are used to pass info to and from the PRU.
 */

#define MAX_BUF_SIZE	128
#define MESSAGE_LIMIT	2

/*
 * This is identical to i2c_msg other than the buffer is included in the struct
 * (which limits it to MAX_BUF_SIZE bytes).
 */
struct pru_i2c_msg {
	__u16 addr;
	__u16 flags;
	__u16 len;
	__u8 buf[MAX_BUF_SIZE];
};

struct evb_pru_i2c_msg_data {
	s32 xfer_result;
	u32 num_msgs;
	struct pru_i2c_msg msgs[MESSAGE_LIMIT];
};

/* END PRU DATA STRUCTS */

/* private driver data */
struct evb_pru_i2c_algo_data {
	struct rpmsg_channel *rpdev;
	struct i2c_adapter *adap;
	struct completion done;
	u8 buf[MESSAGE_LIMIT][MAX_BUF_SIZE];
	int xfer_result;
	int rx_count;
};

/**
 * evb_pru_i2c_master_xfer - i2c algo master_xfer callback
 */
static int evb_pru_i2c_master_xfer(struct i2c_adapter *i2c_adap,
				   struct i2c_msg *msgs, int num)
{
	struct evb_pru_i2c_algo_data *adata = i2c_adap->algo_data;
	struct evb_pru_i2c_msg_data msg_data;
	int i, ret;

	reinit_completion(&adata->done);

	if (num > MESSAGE_LIMIT)
		return -EINVAL;

	/* serialize the i2c msgs for sending to the PRU */
	msg_data.num_msgs = num;
	for (i = 0; i < num; i++) {
		if (msgs[i].len > MAX_BUF_SIZE)
			return -EINVAL;
		memcpy(&msg_data.msgs[i], &msgs[i], sizeof(struct i2c_msg));
		memcpy(&msg_data.msgs[i].buf, msgs[i].buf, msgs[i].len);
	}

	ret = rpmsg_trysend(adata->rpdev, &msg_data, sizeof(msg_data));
	if (ret == -ENOMEM)
		return -EAGAIN;
	if (ret < 0)
		return ret;

	ret = wait_for_completion_timeout(&adata->done, i2c_adap->timeout);
	if (!ret) {
		/* sending num_msgs == 0 puts the port in idle state */
		msg_data.num_msgs = 0;
		rpmsg_send(adata->rpdev, &msg_data, sizeof(msg_data));

		return -ETIMEDOUT;
	}

	/* copy back updated buffers for read messages */
	for (i = 0; i < num; i++) {
		if (msgs[i].flags & I2C_M_RD)
			memcpy(msgs[i].buf, adata->buf[i], msgs[i].len);
	}

	return adata->xfer_result;
}

/**
 * evb_pru_i2c_functionality - i2c algo functionality callback
 */
static u32 evb_pru_i2c_functionality(struct i2c_adapter *adap)
{
	return I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL |
	       I2C_FUNC_SMBUS_READ_BLOCK_DATA |
	       I2C_FUNC_SMBUS_BLOCK_PROC_CALL;
}

const struct i2c_algorithm evb_pru_i2c_algo = {
	.master_xfer	= evb_pru_i2c_master_xfer,
	.functionality	= evb_pru_i2c_functionality,
};
EXPORT_SYMBOL_GPL(evb_pru_i2c_algo);

static const struct i2c_adapter_quirks evb_pru_i2c_quirks = {
	.flags			= I2C_AQ_NO_CLK_STRETCH,
	.max_num_msgs		= MESSAGE_LIMIT,
	.max_write_len		= MAX_BUF_SIZE,
	.max_read_len		= MAX_BUF_SIZE,
	.max_comb_1st_msg_len	= MAX_BUF_SIZE,
	.max_comb_2nd_msg_len	= MAX_BUF_SIZE,
};

/**
 * evb_pru_i2c_cb - rpmsg_channel message received callback
 */
static void evb_pru_i2c_cb(struct rpmsg_channel *rpdev, void *data, int len,
			   void *priv, u32 src)
{
	struct evb_pru_i2c_algo_data *adata = dev_get_drvdata(&rpdev->dev);
	struct evb_pru_i2c_msg_data *msg_data = data;
	int i;

	/* If len is wrong, we probably have bad firmware - just ignore it */
	if (len != sizeof(*msg_data))
		return;

	adata->xfer_result = msg_data->xfer_result;
	/* copy updated buffers for read messages */
	for (i = 0; i < msg_data->num_msgs; i++) {
		if (msg_data->msgs[i].flags & I2C_M_RD)
			memcpy(adata->buf[i], msg_data->msgs[i].buf,
			       msg_data->msgs[i].len);
	}
	complete(&adata->done);
}

static int evb_pru_i2c_probe(struct rpmsg_channel *rpdev)
{
	struct evb_pru_i2c_algo_data *adata;
	struct i2c_adapter *adap;
	int ret;

	adata = devm_kzalloc(&rpdev->dev, sizeof(*adata), GFP_KERNEL);
	if (!adata)
		return -ENOMEM;

	adap = devm_kzalloc(&rpdev->dev, sizeof(*adap), GFP_KERNEL);
	if (!adap)
		return -ENOMEM;

	adap->owner = THIS_MODULE;
	adap->algo = &evb_pru_i2c_algo;
	adap->algo_data = adata;
	adap->timeout = HZ; /* 1 second */
	adap->dev.parent = &rpdev->dev;
	snprintf(adap->name, sizeof(adap->name), "evb-pru-i2c%d", rpdev->dst);
	adap->quirks = &evb_pru_i2c_quirks;

	adata->rpdev = rpdev;
	adata->adap = adap;
	init_completion(&adata->done);

	dev_set_drvdata(&rpdev->dev, adata);

	ret = i2c_add_adapter(adap);
	if (ret < 0)
		return ret;

	dev_info(&rpdev->dev, "Registered i2c adapter %s\n", adap->name);

	return 0;
}

static void evb_pru_i2c_remove(struct rpmsg_channel *rpdev)
{
	struct evb_pru_i2c_algo_data *adata = dev_get_drvdata(&rpdev->dev);

	i2c_del_adapter(adata->adap);

	dev_info(&rpdev->dev, "Removed i2c adapter %s\n", adata->adap->name);
}

static struct rpmsg_device_id evb_pru_i2c_driver_id_table[] = {
	{ .name	= "evb-pru-i2c" },
	{ },
};
MODULE_DEVICE_TABLE(rpmsg, evb_pru_i2c_driver_id_table);

static struct rpmsg_driver evb_pru_i2c_client = {
	.drv.name	= KBUILD_MODNAME,
	.id_table	= evb_pru_i2c_driver_id_table,
	.probe		= evb_pru_i2c_probe,
	.callback	= evb_pru_i2c_cb,
	.remove		= evb_pru_i2c_remove,
};
module_rpmsg_driver(evb_pru_i2c_client);

MODULE_DESCRIPTION("Software I2C driver for Fatcatlab EVB on BeagleBone");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("rpmsg:evb-pru-i2c");
