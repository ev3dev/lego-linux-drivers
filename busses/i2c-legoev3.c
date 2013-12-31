/*
 * I2C bus driver for LEGO Mindstorms EV3
 * Copyright (C) 2013 David Lechner <david@lechnology.com>
 *
 * Based on i2c-gpio.c:
 * Copyright (C) 2007 Atmel Corporation
 *
 * and davinci_iic.c from lms2012
 * the file does not contain a copyright, but comes from the LEGO Group
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/i2c.h>
#include <linux/i2c-legoev3.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/hrtimer.h>
#include <linux/completion.h>
#include <linux/platform_device.h>

#include <asm/gpio.h>

enum transfer_states {
	TRANSFER_START,
	TRANSFER_START2,
	TRANSFER_ADDR,
	TRANSFER_WRITE,
	TRANSFER_READ,
	TRANSFER_WBIT,
	TRANSFER_RBIT,
	TRANSFER_WACK,
	TRANSFER_RACK,
	TRANSFER_STOP,
	TRANSFER_STOP2,
	TRANSFER_STOP3,
	TRANSFER_RESTART,
	TRANSFER_WAIT,
	TRANSFER_COMPLETE,
};

struct i2c_legoev3_algo_data {
	struct i2c_legoev3_platform_data *pdata;
	struct i2c_msg *msgs;
	unsigned num_msg;
	unsigned cur_msg;
	struct hrtimer timer;
	struct completion done;
	int xfer_result;
	unsigned udelay;
	unsigned wait_cycles;
	u16 buf_offset;
	u8 bit_mask;
	u8 data_byte;
	enum transfer_states transfer_state;
	unsigned clock_state:1;
	unsigned nacked:1;
};

static enum hrtimer_restart i2c_legoev3_timer_callback(struct hrtimer *timer)
{
	struct i2c_legoev3_algo_data *adata =
		container_of(timer, struct i2c_legoev3_algo_data, timer);
	struct i2c_msg *msg = &adata->msgs[adata->cur_msg];

	hrtimer_forward_now(timer, ktime_set(0, adata->udelay * 1000));

	gpio_set_value(adata->pdata->scl_pin, adata->clock_state);

	switch (adata->transfer_state)
	{
	case TRANSFER_START:
		/*
		 * Make sure to SYNC into Timer settings
		 * to ensure first bit time having full length
		 */
		adata->cur_msg = 0;
		adata->xfer_result = 0;
		adata->transfer_state = TRANSFER_START2;
		break;

	case TRANSFER_START2:
		/* Generate start condition - sda low to high while clk high */
		gpio_direction_output(adata->pdata->sda_pin, 0);
		adata->clock_state = 0;
		adata->nacked = false;
		adata->transfer_state = TRANSFER_ADDR;
		break;

	case TRANSFER_ADDR:
		adata->data_byte = (msg->addr << 1);
		if (msg->flags & I2C_M_RD)
			adata->data_byte |= 1;
		adata->buf_offset = 0;

	case TRANSFER_WRITE:
		if (adata->transfer_state == TRANSFER_WRITE)
			adata->data_byte = msg->buf[adata->buf_offset++];
		adata->transfer_state = TRANSFER_WBIT;
		adata->bit_mask  = 0x80;

	case TRANSFER_WBIT:
		if (!adata->clock_state) {
			gpio_direction_output(adata->pdata->sda_pin,
					      adata->data_byte & adata->bit_mask);
			adata->bit_mask >>= 1;
		}

		if (!adata->bit_mask && adata->clock_state)
			adata->transfer_state = TRANSFER_RACK;
		adata->clock_state ^= 1;
		break;

	case TRANSFER_READ:
		gpio_direction_input(adata->pdata->sda_pin);
		adata->transfer_state = TRANSFER_RBIT;
		adata->bit_mask  = 0x80;
		adata->data_byte = 0;

	case TRANSFER_RBIT:
		if (adata->clock_state) {
			adata->data_byte |= gpio_get_value(adata->pdata->sda_pin)
					    ? adata->bit_mask : 0;
			adata->bit_mask >>= 1;

			if (!adata->bit_mask) {
				msg->buf[adata->buf_offset++] = adata->data_byte;
				adata->transfer_state = TRANSFER_WACK;
			}
		}
		adata->clock_state ^= 1;
		break;

	case TRANSFER_RACK:
		if (!adata->clock_state)
			gpio_direction_input(adata->pdata->sda_pin);
		else {
			if (!gpio_get_value(adata->pdata->sda_pin)) {
				if (adata->buf_offset < msg->len) {
					adata->wait_cycles = 4;
					adata->transfer_state = TRANSFER_WAIT;
				}
				else
					adata->transfer_state = TRANSFER_STOP;
			} else {
				adata->nacked = true;
				adata->xfer_result = -ENXIO;
				adata->transfer_state = TRANSFER_STOP;
			}
		}
		adata->clock_state ^= 1;
		break;

	case TRANSFER_WACK:
		if (!adata->clock_state)
			/* ACK (or NACK the last byte read) */
			gpio_direction_output(adata->pdata->sda_pin,
					      adata->buf_offset == msg->len);
		else {
			if (adata->buf_offset < msg->len) {
				adata->wait_cycles = 2;
				adata->transfer_state = TRANSFER_WAIT;
			} else
				adata->transfer_state = TRANSFER_STOP;
		}
		adata->clock_state ^= 1;
		break;

	case TRANSFER_WAIT:
		if (adata->wait_cycles--)
			break;
		else if (msg->flags & I2C_M_RD)
			adata->transfer_state = TRANSFER_READ;
		else
			adata->transfer_state = TRANSFER_WRITE;
		break;

	case TRANSFER_STOP:
		if ((msg->flags & I2C_M_RD)
		    && adata->buf_offset < msg->len)
			gpio_direction_output(adata->pdata->sda_pin, 0);
		else
			gpio_direction_input(adata->pdata->sda_pin);

		if (adata->clock_state)
			adata->transfer_state = TRANSFER_STOP2;
		adata->clock_state = 1;
		break;

	case TRANSFER_STOP2:
		if ((adata->cur_msg + 1) < adata->num_msg && !adata->nacked)
		{
			/*
			 * This is some non-standard i2c weirdness for
			 * compatibility with the NXT ultrasonic sensor.
			 *
			 * Normal i2c would just send a restart (sda high
			 * to low while clk is high) between writing the
			 * address and reading the data. Instead, we send
			 * a stop (sda low to high while clk is high) and
			 * then do an extra clock cycle (low then high)
			 * before sending a start and reading the data.
			 */
			gpio_direction_output(adata->pdata->sda_pin, 1);
			adata->clock_state ^= 1;
			if  (adata->clock_state) {
				adata->cur_msg++;
				adata->transfer_state = TRANSFER_RESTART;
			}
		} else
			adata->transfer_state = TRANSFER_STOP3;
		break;

	case TRANSFER_RESTART:
		adata->transfer_state = TRANSFER_START2;
		break;

	case TRANSFER_STOP3:
		gpio_direction_input(adata->pdata->sda_pin);
		adata->transfer_state = TRANSFER_COMPLETE;

	case TRANSFER_COMPLETE:
		complete(&adata->done);
		return HRTIMER_NORESTART;

	default:
		break;
	}

	return HRTIMER_RESTART;
}

static int i2c_legoev3_xfer(struct i2c_adapter *i2c_adap,
			    struct i2c_msg msgs[], int num)
{
	struct i2c_legoev3_algo_data *adata = i2c_adap->algo_data;

	adata->msgs = msgs;
	adata->num_msg = num;
	adata->transfer_state = TRANSFER_START;
	INIT_COMPLETION(adata->done);
	hrtimer_start(&adata->timer, ktime_set(0, adata->udelay * 1000), HRTIMER_MODE_REL);
	wait_for_completion_interruptible(&adata->done);

	return adata->xfer_result;
}

static u32 i2c_legoev3_func(struct i2c_adapter *adap)
{
	return I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL |
	       I2C_FUNC_SMBUS_READ_BLOCK_DATA |
	       I2C_FUNC_SMBUS_BLOCK_PROC_CALL;
}

static const struct i2c_algorithm i2c_legoev3_algo = {
	.master_xfer	= i2c_legoev3_xfer,
	.functionality	= i2c_legoev3_func,
};

static int __devinit i2c_legoev3_probe(struct platform_device *pdev)
{
	struct i2c_legoev3_platform_data *pdata;
	struct i2c_legoev3_algo_data *adata;
	struct i2c_adapter *adap;
	int ret;

	pdata = pdev->dev.platform_data;
	if (!pdata)
		return -ENXIO;

	ret = -ENOMEM;
	adap = kzalloc(sizeof(struct i2c_adapter), GFP_KERNEL);
	if (!adap)
		goto err_alloc_adap;
	adata = kzalloc(sizeof(struct i2c_legoev3_algo_data), GFP_KERNEL);
	if (!adata)
		goto err_alloc_adata;

	gpio_direction_output(pdata->sda_pin, 1);
	gpio_direction_output(pdata->scl_pin, 1);

	adata->udelay = 50;			/* 10 kHz */
//	adata->udelay = 5;			/* 100 kHz */

	adata->pdata = pdata;
	adata->clock_state = 1;

	adap->owner = THIS_MODULE;
	adap->class = I2C_CLASS_LEGOEV3;
	adap->algo = &i2c_legoev3_algo;
	adap->algo_data = adata;
	adap->timeout = HZ / 100;				/* 10 ms */
	adap->dev.parent = &pdev->dev;
	adap->nr = pdev->id;
	snprintf(adap->name, sizeof(adap->name), "i2c-legoev3%d", pdev->id);

	hrtimer_init(&adata->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	adata->timer.function = i2c_legoev3_timer_callback;
	init_completion(&adata->done);

	ret = i2c_add_numbered_adapter(adap);
	if (ret)
		goto err_i2c_add_numbered_adapter;

	platform_set_drvdata(pdev, adap);

	dev_info(&pdev->dev, "using pins %u (SDA) and %u (SCL)\n",
		 pdata->sda_pin, pdata->scl_pin);

	return 0;

err_i2c_add_numbered_adapter:
	kfree(adata);
err_alloc_adata:
	kfree(adap);
err_alloc_adap:
	return ret;
}

static int __devexit i2c_legoev3_remove(struct platform_device *pdev)
{
	struct i2c_adapter *adap = platform_get_drvdata(pdev);
	struct i2c_legoev3_algo_data *adata = adap->algo_data;

	hrtimer_cancel(&adata->timer);
	i2c_del_adapter(adap);
	platform_set_drvdata(pdev, NULL);
	kfree(adata);
	kfree(adap);

	return 0;
}

static struct platform_driver i2c_legoev3_driver = {
	.driver		= {
		.name	= "i2c-legoev3",
		.owner	= THIS_MODULE,
	},
	.probe		= i2c_legoev3_probe,
	.remove		= __devexit_p(i2c_legoev3_remove),
};
module_platform_driver(i2c_legoev3_driver);

MODULE_AUTHOR("David Lechner <david@lechnology.com>");
MODULE_DESCRIPTION("LEGO Mindstorms EV3 I2C driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:i2c-legoev3");
