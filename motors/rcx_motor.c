/*
 * RCX/Power Functions Motor device driver for LEGO MINDSTORMS EV3
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
 * RCX/Power Functions Motor Driver
 *
 * This driver provides a [dc-motor] interface for RCX motors, Power Functions
 * motors or any other 9V rated DC motor connected to an output port. You can
 * find the sysfs device at `/sys/bus/legoev3/devices/<port>:rcx-motor` where
 * `<port>` is the the name of the output port the motor is connected to (e.g.
 * `outA`). There is not much of interest there though - all the useful stuff
 * is in the [dc-motor] class.
 * .
 * This device is loaded when an [ev3-output-port] is set to `rcx-motor` mode.
 * It is not automatically detected.
 * .
 * [dc-motor]: ../dc-motor-class
 * [ev3-output-port]: ../ev3-output-port
 */

#include <linux/device.h>
#include <linux/module.h>
#include <linux/slab.h>

#include <dc_motor_class.h>

#include "../ev3/legoev3_ports.h"
#include "../ev3/ev3_output_port.h"

struct rcx_motor_data {
	struct dc_motor_device motor;
};

static int rcx_motor_probe(struct legoev3_port_device *motor)
{
	struct rcx_motor_data *data;
	struct ev3_motor_platform_data *pdata = motor->dev.platform_data;
	int err;

	if (WARN_ON(!pdata))
		return -EINVAL;

	data = kzalloc(sizeof(struct rcx_motor_data), GFP_KERNEL);
	if (IS_ERR(data))
		return -ENOMEM;

	strncpy(data->motor.name, dev_name(&motor->dev), DC_MOTOR_NAME_SIZE);
	strncpy(data->motor.port_name, dev_name(&pdata->out_port->dev),
							DC_MOTOR_NAME_SIZE);
	memcpy(&data->motor.ops, &pdata->motor_ops, sizeof(struct dc_motor_ops));

	err = register_dc_motor(&data->motor, &motor->dev);
	if (err)
		goto err_register_dc_motor;

	dev_set_drvdata(&motor->dev, data);

	return 0;

err_register_dc_motor:
	kfree(data);

	return err;
}

static int rcx_motor_remove(struct legoev3_port_device *motor)
{
	struct rcx_motor_data *data = dev_get_drvdata(&motor->dev);

	unregister_dc_motor(&data->motor);
	dev_set_drvdata(&motor->dev, NULL);
	kfree(data);

	return 0;
}

struct legoev3_port_device_driver rcx_motor_driver = {
	.probe	= rcx_motor_probe,
	.remove	= rcx_motor_remove,
	.driver = {
		.name	= "rcx-motor",
		.owner	= THIS_MODULE,
	},
};
legoev3_port_device_driver(rcx_motor_driver);

MODULE_DESCRIPTION("RCX/Power Functions motor driver for LEGO MINDSTORMS EV3");
MODULE_AUTHOR("David Lechner <david@lechnology.com>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("legoev3:rcx-motor");

