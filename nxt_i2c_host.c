/*
 * NXT I2C host driver for LEGO Mindstorms EV3
 *
 * Copyright (C) 2014 David Lechner <david@lechnology.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.

 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

/*
 * -----------------------------------------------------------------------------
 * This driver just calls back to the input port driver to register the I2C
 * adapter for the port.
 * -----------------------------------------------------------------------------
 */

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/legoev3/legoev3_ports.h>
#include <linux/legoev3/ev3_input_port.h>

struct nxt_i2c_host_data {
	struct legoev3_port_device *in_port;
};

static int nxt_i2c_host_probe(struct legoev3_port_device *sensor)
{
	struct nxt_i2c_host_data *nxt_i2c;
	struct ev3_sensor_platform_data *pdata = sensor->dev.platform_data;
	int err;

	if (WARN_ON(!pdata))
		return -EINVAL;

	nxt_i2c = kzalloc(sizeof(struct nxt_i2c_host_data), GFP_KERNEL);
	if (!nxt_i2c)
		return -ENOMEM;

	nxt_i2c->in_port = pdata->in_port;

	err = ev3_input_port_register_i2c(nxt_i2c->in_port, &sensor->dev);
	if (err)
		goto register_i2c_sensor_fail;

	err = dev_set_drvdata(&sensor->dev, nxt_i2c);
	if (err)
		goto dev_set_drvdata_fail;

	return 0;

dev_set_drvdata_fail:
	ev3_input_port_unregister_i2c(nxt_i2c->in_port);
register_i2c_sensor_fail:
	kfree(nxt_i2c);

	return err;
}

static int nxt_i2c_host_remove(struct legoev3_port_device *sensor)
{
	struct nxt_i2c_host_data *nxt_i2c = dev_get_drvdata(&sensor->dev);

	ev3_input_port_unregister_i2c(nxt_i2c->in_port);
	dev_set_drvdata(&sensor->dev, NULL);
	kfree(nxt_i2c);
	return 0;
}

struct legoev3_port_driver nxt_i2c_host_driver = {
	.probe	= nxt_i2c_host_probe,
	.remove	= nxt_i2c_host_remove,
	.driver = {
		.name	= "nxt-i2c-host",
		.owner	= THIS_MODULE,
	},
};
EXPORT_SYMBOL_GPL(nxt_i2c_host_driver);
legoev3_port_driver(nxt_i2c_host_driver);

MODULE_DESCRIPTION("NXT I2C host driver for LEGO Mindstorms EV3");
MODULE_AUTHOR("David Lechner <david@lechnology.com>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("legoev3:nxt-i2c-host");
