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

static int nxt_i2c_host_probe(struct legoev3_port_device *host)
{
	int err;

	err = ev3_input_port_register_i2c(host->port, &host->dev);
	if (err)
		return err;

	return 0;
}

static int nxt_i2c_host_remove(struct legoev3_port_device *host)
{
	ev3_input_port_unregister_i2c(host->port);

	return 0;
}

struct legoev3_port_device_driver nxt_i2c_host_driver = {
	.probe	= nxt_i2c_host_probe,
	.remove	= nxt_i2c_host_remove,
	.driver = {
		.name	= "nxt-i2c-host",
		.owner	= THIS_MODULE,
	},
};
EXPORT_SYMBOL_GPL(nxt_i2c_host_driver);
legoev3_port_device_driver(nxt_i2c_host_driver);

MODULE_DESCRIPTION("NXT I2C host driver for LEGO Mindstorms EV3");
MODULE_AUTHOR("David Lechner <david@lechnology.com>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("legoev3:nxt-i2c-host");
