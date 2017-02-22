/*
 * RCX/Power Functions Motor device driver
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

/**
 * DOC: userspace
 *
 * This driver provides a [dc-motor] interface for RCX motors, Power Functions
 * motors or any other 9V rated DC motor connected to an output port. You can
 * find the devices bound to this driver in the directory
 * ``/sys/bus/lego/drivers/rcx-motor``. There is not much of interest there
 * though - all the useful stuff is in the `dc-motor class`_.
 * .
 * This device is loaded when an `legoev3-output-port`_ is set to ``rcx-motor``
 * mode. It is not automatically detected.
 */

#include <linux/device.h>
#include <linux/module.h>
#include <linux/slab.h>

#include <lego.h>
#include <dc_motor_class.h>
#include <lego_port_class.h>

struct rcx_motor_data {
	struct dc_motor_device motor;
};

static int rcx_motor_probe(struct lego_device *motor)
{
	struct rcx_motor_data *data;
	int err;

	if (WARN_ON(!motor->port->dc_motor_ops))
		return -EINVAL;

	data = kzalloc(sizeof(struct rcx_motor_data), GFP_KERNEL);
	if (IS_ERR(data))
		return -ENOMEM;

	data->motor.name = motor->name;
	data->motor.address = motor->port->address;
	data->motor.ops = motor->port->dc_motor_ops;
	data->motor.context = motor->port->context;

	err = register_dc_motor(&data->motor, &motor->dev);
	if (err)
		goto err_register_dc_motor;

	dev_set_drvdata(&motor->dev, data);

	return 0;

err_register_dc_motor:
	kfree(data);

	return err;
}

static int rcx_motor_remove(struct lego_device *motor)
{
	struct rcx_motor_data *data = dev_get_drvdata(&motor->dev);

	unregister_dc_motor(&data->motor);
	dev_set_drvdata(&motor->dev, NULL);
	kfree(data);

	return 0;
}

struct lego_device_driver rcx_motor_driver = {
	.probe	= rcx_motor_probe,
	.remove	= rcx_motor_remove,
	.driver = {
		.name	= "rcx-motor",
		.owner	= THIS_MODULE,
	},
};
lego_device_driver(rcx_motor_driver);

MODULE_DESCRIPTION("RCX/Power Functions motor driver");
MODULE_AUTHOR("David Lechner <david@lechnology.com>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("lego:rcx-motor");

