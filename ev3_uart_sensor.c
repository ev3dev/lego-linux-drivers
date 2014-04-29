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

#include <linux/device.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/tty.h>
#include <linux/legoev3/legoev3_ports.h>
#include <linux/legoev3/legoev3_uart.h>

struct ev3_uart_sensor_data {
	struct legoev3_port_device *sensor;
	struct msensor_device *pdata;
};

static int ev3_uart_sensor_probe(struct legoev3_port_device *pdev)
{
	struct ev3_uart_sensor_data *sensor;
	struct msensor_device *pdata = pdev->dev.platform_data;
	int err, i;

	if (WARN_ON(!pdata))
		return -EINVAL;

	sensor = kzalloc(sizeof(struct ev3_uart_sensor_data), GFP_KERNEL);
	if (!sensor)
		return -ENOMEM;

	sensor->sensor = sensor;
	sensor->pdata = pdata;

	err = dev_set_drvdata(&pdev->dev, sensor);
	if (err)
		goto dev_set_drvdata_fail;

	dev_info(&pdev->dev, "Connected to port %s\n", "?");
//		 dev_name(&sensor->in_port->dev));

	return 0;

dev_set_drvdata_fail:
	kfree(sensor);

	return err;
}

static int ev3_uart_sensor_remove(struct legoev3_port_device *pdev)
{
	struct ev3_uart_sensor_data *sensor = dev_get_drvdata(&pdev->dev);

	dev_info(&pdev->dev, "Removed from port %s\n", "?");
//		 dev_name(&sensor->in_port->dev));

	dev_set_drvdata(&pdev->dev, NULL);
	kfree(sensor);
	return 0;
}

static struct legoev3_port_device_id ev3_uart_ids[LEGOEV3_UART_TYPE_MAX + 1];

struct legoev3_port_driver ev3_uart_sensor_driver = {
	.probe		= ev3_uart_sensor_probe,
	.remove		= ev3_uart_sensor_remove,
	.id_table	= ev3_uart_ids,
	.driver = {
		.name	= "ev3-uart-sensor",
		.owner	= THIS_MODULE,
	},
};
legoev3_port_driver(ev3_uart_sensor_driver);

MODULE_DESCRIPTION("EV3 uart sensor device driver for LEGO Mindstorms EV3");
MODULE_AUTHOR("David Lechner <david@lechnology.com>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("legoev3:ev3-uart-sensor");
