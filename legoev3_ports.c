/*
 * Support for the input and output ports on the LEGO Mindstorms EV3
 *
 * Copyright (C) 2013 David Lechner <david@lechnology.com>
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
 * This module provides components for interfacing with the input and output
 * ports on the EV3 brick.
 *
 * There is a bus called "legoev3". It is used to match the devices plugged into
 * the ports to drivers for those devices.
 *
 * Each input and output port has its own device node. They preform device
 * discovery similar to Device3 in d_analog.c in the LMS2012 code and notify
 * the legoev3 bus when a device has been connected or disconnected.
 * -----------------------------------------------------------------------------
 */

#include <linux/module.h>
#include <linux/string.h>
#include <linux/legoev3/legoev3_ports.h>

struct device_type legoev3_input_port_device_type = {
	.name	= "legoev3-input-port",
};
EXPORT_SYMBOL_GPL(legoev3_input_port_device_type);

int legoev3_register_input_port(struct legoev3_input_port_device *ip)
{
	if (!ip)
		return -EINVAL;
	if (ip->id >= LEGOEV3_NUM_PORT_IN)
		return -EINVAL;

	device_initialize(&ip->dev);
	ip->dev.parent = &legoev3_port.dev;
	ip->dev.bus = &legoev3_bus_type;
	ip->dev.type = &legoev3_input_port_device_type;
	dev_set_name(&ip->dev, "in%c", '1' + ip->id);

	return device_add(&ip->dev);
}

void legoev3_unregister_input_port(struct legoev3_input_port_device *ip)
{
	put_device(&ip->dev);
}
EXPORT_SYMBOL_GPL(legoev3_unregister_input_port);

int legoev3_register_input_ports(struct legoev3_input_port_device *ip,
				 unsigned len)
{
	int err, i;

	i = 0;
	do {
		err = legoev3_register_input_port(&ip[i]);
		if (err)
			goto legoev3_register_input_port_fail;
	} while (++i < len);

	return 0;

legoev3_register_input_port_fail:
	while (i--)
		legoev3_unregister_input_port(&ip[i]);

	return err;
}
EXPORT_SYMBOL_GPL(legoev3_register_input_ports);

void legoev3_unregister_input_ports(struct legoev3_input_port_device *ip,
				    unsigned len)
{
	int i = len;

	while (i--)
		legoev3_unregister_input_port(&ip[i]);
}
EXPORT_SYMBOL_GPL(legoev3_unregister_input_ports);

struct platform_device legoev3_port = {
	.name	= "legoev3-port",
	.id	= -1,
};

int legoev3_register_port_driver(struct legoev3_port_driver *drv)
{
	drv->driver.bus = &legoev3_bus_type;
	return driver_register(&drv->driver);
}
EXPORT_SYMBOL_GPL(legoev3_register_port_driver);

void legoev3_unregister_port_driver(struct legoev3_port_driver *drv)
{
	driver_unregister(&drv->driver);
}
EXPORT_SYMBOL_GPL(legoev3_unregister_port_driver);

static int legoev3_bus_match(struct device *dev, struct device_driver *drv)
{
	return !strcmp(dev->type->name, drv->name);
}

static int legoev3_bus_uevent(struct device *dev, struct kobj_uevent_env *env)
{
	/* TODO: add uevent vars */
	return 0;
}

struct bus_type legoev3_bus_type = {
	.name		= "legoev3",
	.match		= legoev3_bus_match,
	.uevent		= legoev3_bus_uevent,
};
EXPORT_SYMBOL_GPL(legoev3_bus_type);

static int __init legoev3_ports_init(void)
{
	int err;

	err = platform_device_register(&legoev3_port);
	if (err)
		return err;

	err = bus_register(&legoev3_bus_type);
	if (err)
		goto bus_register_fail;

	return 0;

bus_register_fail:
	platform_device_unregister(&legoev3_port);

	return err;
}
postcore_initcall(legoev3_ports_init);

MODULE_DESCRIPTION("Support for LEGO Mindstorms EV3 input and output ports.");
MODULE_AUTHOR("David Lechner <david@lechnology.com>");
MODULE_LICENSE("GPL");
