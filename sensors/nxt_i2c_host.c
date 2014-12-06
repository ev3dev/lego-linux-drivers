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
 * Note: The comment block below is used to generate docs on the ev3dev website.
 * Use kramdown (markdown) format. Use a '.' as a placeholder when blank lines
 * or leading whitespace is important for the markdown syntax.
 */

/**
 * DOC: website
 *
 * NXT I2C Host Driver
 *
 * This driver tells an [EV3 input port] to configure itself for I2C
 * communications (same for both I2C/NXT and I2C/Other). This includes
 * registering an I2C adapter for the port. The adapter can be found at
 * `/sys/bus/i2c/devices/i2c-<N>`, where `<N>` is the number of the input
 * port plus 2 (3 to 6).[^i2c-adapter-numbering] More likely, you will be
 * interested in the device node which is `/dev/i2c-in<N>`, where `<N>` is
 * the number of the EV3 input port.[^i2c-device-node-symlinks]
 * .
 * Unlike the analog host drivers, this host does not have anything to do with
 * the [I2C sensor drivers]. The I2C sensor drivers are proper Linux I2C
 * drivers and I2C client devices are loaded by the I2C adapter driver. See
 * [Using I2C Sensors].
 * .
 * ### sysfs Attributes
 * .
 *  * You can find this device at `/sys/bus/legoev3/devices/in<N>:nxt-i2c-host`
 * where `<N>` is the number of an input port (1 to 4).
 * .
 * `device_type` (read-only)
 * : Returns `ev3-analog-host`
 * .
 * `port_name` (read-only)
 * : Returns the name of the port this host is connected to (e.g. `in1`).
 * .
 * <hr />
 * .
 * [^i2c-adapter-numbering]: The reason for the offset is because the CPU has
 * .    two built-in I2C ports that take the numbers 0 and 1.
 * .
 * [^i2c-device-node-symlinks]: `/dev/i2c-in<N>` is a symlink to `/dev/i2c-<N+2>`
 * .
 * [EV3 input port]: ../ev3-input-port
 * [I2C sensor drivers]: ../nxt-i2c-sensor
 * [Using I2C Sensors]: ../using-i2c-sensors
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
