/*
 * Dexter Industries BrickPi data driver
 *
 * Copyright (C) 2015 David Lechner <david@lechnology.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.

 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>

#include <lego.h>
#include <tacho_motor_class.h>

#include "brickpi.h"

struct brickpi_motor_data {
	struct tacho_motor_device tm;
	struct lego_device *data;
};

int brickpi_motor_probe(struct lego_device *ldev)
{
	struct brickpi_motor_data *data;
	int err;

	if (!ldev->port->tacho_motor_ops) {
		dev_err(&ldev->dev, "Port '%s' does not support tacho motor.",
			ldev->port->port_name);
		return -EINVAL;
	}

	data = kzalloc(sizeof(struct brickpi_motor_data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->tm.driver_name = ldev->name;
	data->tm.port_name = ldev->port->port_name;
	data->tm.ops = ldev->port->tacho_motor_ops;
	data->tm.context = ldev->port->context;

	dev_set_drvdata(&ldev->dev, data);

	err = register_tacho_motor(&data->tm, &ldev->dev);
	if (err)
		goto err_register_tacho_motor;

	return 0;

err_register_tacho_motor:
	dev_set_drvdata(&ldev->dev, NULL);
	kfree(data);

	return err;
}

int brickpi_motor_remove(struct lego_device *ldev)
{
	struct brickpi_motor_data *data = dev_get_drvdata(&ldev->dev);

	unregister_tacho_motor(&data->tm);
	dev_set_drvdata(&ldev->dev, NULL);
	kfree(data);

	return 0;
}

struct lego_device_driver brickpi_motor_driver = {
	.probe	= brickpi_motor_probe,
	.remove	= brickpi_motor_remove,
	.driver = {
		.name	= "brickpi-motor",
		.owner	= THIS_MODULE,
	},
};
lego_device_driver(brickpi_motor_driver);

MODULE_DESCRIPTION("Dexter Industries BrickPi motor driver");
MODULE_AUTHOR("David Lechner <david@lechnology.com>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("lego:brickpi-motor");
