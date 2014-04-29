/*
 * EV3 UART host device driver for LEGO Mindstorms EV3
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

struct ev3_uart_host_data {
	struct legoev3_port_device *in_port;
};

static int ev3_uart_host_probe(struct legoev3_port_device *host)
{
	struct ev3_uart_host_data *ev3_uart;
	struct ev3_sensor_platform_data *pdata = host->dev.platform_data;
	int err;

	if (WARN_ON(!pdata))
		return -EINVAL;

	ev3_uart = kzalloc(sizeof(struct ev3_uart_host_data), GFP_KERNEL);
	if (!ev3_uart)
		return -ENOMEM;

	ev3_uart->in_port = pdata->in_port;

	err = ev3_input_port_enable_uart(ev3_uart->in_port);
	if (err)
		goto err_ev3_input_port_enable_uart;

	err = dev_set_drvdata(&host->dev, ev3_uart);
	if (err)
		goto err_dev_set_drvdata;

	dev_info(&host->dev, "Started.\n");

	return 0;

err_dev_set_drvdata:
	ev3_input_port_disable_uart(ev3_uart->in_port);
err_ev3_input_port_enable_uart:
	kfree(ev3_uart);

	return err;
}

static int ev3_uart_host_remove(struct legoev3_port_device *host)
{
	struct ev3_uart_host_data *ev3_uart = dev_get_drvdata(&host->dev);

	dev_info(&host->dev, "Stopped.\n");
	ev3_input_port_disable_uart(ev3_uart->in_port);
	dev_set_drvdata(&host->dev, NULL);
	kfree(ev3_uart);
	return 0;
}

struct legoev3_port_driver ev3_uart_host_driver = {
	.probe	= ev3_uart_host_probe,
	.remove	= ev3_uart_host_remove,
	.driver = {
		.name	= "ev3-uart-host",
		.owner	= THIS_MODULE,
	},
};
legoev3_port_driver(ev3_uart_host_driver);

MODULE_DESCRIPTION("EV3 UART host device driver for LEGO Mindstorms EV3");
MODULE_AUTHOR("David Lechner <david@lechnology.com>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("legoev3:ev3-uart-host");
