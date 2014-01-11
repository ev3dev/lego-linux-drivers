/*
 * I2C bus driver for LEGO Mindstorms EV3
 * Copyright (C) 2013-2014 David Lechner <david@lechnology.com>
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
#include <linux/gpio.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/completion.h>
#include <linux/platform_device.h>

#include <mach/legoev3-fiq.h>

struct i2c_legoev3_algo_data {
	struct i2c_legoev3_platform_data *pdata;
	struct completion done;
	int xfer_result;
};

static void i2c_legoev3_complete(int xfer_result, void *context)
{
	struct i2c_legoev3_algo_data *adata = context;

	adata->xfer_result = xfer_result;
	complete(&adata->done);
}

static int i2c_legoev3_xfer(struct i2c_adapter *i2c_adap,
			    struct i2c_msg *msgs, int num)
{
	struct i2c_legoev3_algo_data *adata = i2c_adap->algo_data;
	int err;

	INIT_COMPLETION(adata->done);
	err = legoev3_fiq_start_xfer(adata->pdata->port_id, msgs, num,
				     i2c_legoev3_complete, adata);
	if (err)
		return err;

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

	adata->pdata = pdata;

	adap->owner = THIS_MODULE;
	adap->class = I2C_CLASS_LEGOEV3;
	adap->algo = &i2c_legoev3_algo;
	adap->algo_data = adata;
	adap->timeout = HZ / 100; /* 10 ms - not implemented */
	adap->dev.parent = &pdev->dev;
	adap->dev.platform_data = pdata;
	adap->nr = pdev->id;
	snprintf(adap->name, sizeof(adap->name), "i2c-legoev3%d", pdev->id);

	init_completion(&adata->done);

	ret = legoev3_fiq_request_port(pdata->port_id, pdata->sda_pin,
				       pdata->scl_pin);
	if (ret) {
		dev_err(&pdev->dev, "Requesting FIQ port failed.\n");
		goto err_legoev3_fiq_request_port;
	}

	ret = i2c_add_numbered_adapter(adap);
	if (ret)
		goto err_i2c_add_numbered_adapter;

	platform_set_drvdata(pdev, adap);

	dev_info(&pdev->dev, "registered on input port %d\n", pdata->port_id + 1);

	return 0;

err_i2c_add_numbered_adapter:
	legoev3_fiq_release_port(pdata->port_id);
err_legoev3_fiq_request_port:
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
	struct i2c_legoev3_platform_data *pdata = pdev->dev.platform_data;

	i2c_del_adapter(adap);
	legoev3_fiq_release_port(pdata->port_id);
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
