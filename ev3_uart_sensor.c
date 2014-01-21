/*
 * EV3 UART sensor device driver for LEGO Mindstorms EV3
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

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/legoev3/legoev3_ports.h>
#include <linux/legoev3/ev3_input_port.h>

struct ev3_uart_sensor_data {
	struct legoev3_port_device *in_port;
};

static int __devinit ev3_uart_sensor_probe(struct legoev3_port_device *sensor)
{
	struct ev3_uart_sensor_data *ev3_uart;
	struct ev3_sensor_platform_data *pdata = sensor->dev.platform_data;
	int err;

	if (WARN_ON(!pdata))
		return -EINVAL;

	ev3_uart = kzalloc(sizeof(struct ev3_uart_sensor_data), GFP_KERNEL);
	if (!ev3_uart)
		return -ENOMEM;

	ev3_uart->in_port = pdata->in_port;

	err = ev3_input_port_enable_uart(ev3_uart->in_port);
	if (err)
		goto register_i2c_sensor_fail;

	err = dev_set_drvdata(&sensor->dev, ev3_uart);
	if (err)
		goto dev_set_drvdata_fail;

	dev_info(&sensor->dev, "EV3 UART sensor connected.\n");

	return 0;

dev_set_drvdata_fail:
	ev3_input_port_disable_uart(ev3_uart->in_port);
register_i2c_sensor_fail:
	kfree(ev3_uart);

	return err;
}

static int __devexit ev3_uart_sensor_remove(struct legoev3_port_device *sensor)
{
	struct ev3_uart_sensor_data *ev3_uart = dev_get_drvdata(&sensor->dev);

	dev_info(&sensor->dev, "EV3 UART sensor removed.\n");
	ev3_input_port_disable_uart(ev3_uart->in_port);
	dev_set_drvdata(&sensor->dev, NULL);
	kfree(ev3_uart);
	return 0;
}

struct legoev3_port_driver ev3_uart_sensor_driver = {
	.probe	= ev3_uart_sensor_probe,
	.remove	= __devexit_p(ev3_uart_sensor_remove),
	.driver = {
		.name	= "ev3-uart-sensor",
		.owner	= THIS_MODULE,
	},
};
legoev3_port_driver(ev3_uart_sensor_driver);

MODULE_DESCRIPTION("EV3 UART sensor device driver for LEGO Mindstorms EV3");
MODULE_AUTHOR("David Lechner <david@lechnology.com>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("legoev3:ev3-uart-sensor");
