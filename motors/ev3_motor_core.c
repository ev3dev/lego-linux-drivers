/*
 * NXT/EV3 Motor driver
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

#include "ev3_motor.h"

struct ev3_motor_data {
	struct tacho_motor_device tm;
	struct lego_device *data;
};

static int ev3_motor_probe(struct lego_device *ldev)
{
	struct ev3_motor_data *data;
	int err;

	if (!ldev->port->tacho_motor_ops) {
		dev_err(&ldev->dev, "Port '%s' does not support tacho motor.",
			ldev->port->address);
		return -EINVAL;
	}

	data = kzalloc(sizeof(struct ev3_motor_data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->tm.driver_name = ldev->name;
	data->tm.address = ldev->port->address;
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

static int ev3_motor_remove(struct lego_device *ldev)
{
	struct ev3_motor_data *data = dev_get_drvdata(&ldev->dev);

	unregister_tacho_motor(&data->tm);
	dev_set_drvdata(&ldev->dev, NULL);
	kfree(data);

	return 0;
}

static const struct lego_device_id ev3_motor_driver_id_table[] = {
	LEGO_DEVICE_ID(LEGO_NXT_MOTOR),
	LEGO_DEVICE_ID(LEGO_EV3_LARGE_MOTOR),
	LEGO_DEVICE_ID(LEGO_EV3_MEDIUM_MOTOR),
	LEGO_DEVICE_ID(FIRGELLI_L12_EV3_50),
	LEGO_DEVICE_ID(FIRGELLI_L12_EV3_100),
};

static ssize_t driver_names_show(struct device_driver *drv, char *buf)
{
	int i;
	int size = 0;

	for (i = 0; i < NUM_EV3_MOTOR_ID; i++) {
		size += sprintf(buf + size, "%s ",
				ev3_motor_driver_id_table[i].name);
	}

	buf[size - 1] = '\n';

	return size;
}

static DRIVER_ATTR_RO(driver_names);

static struct attribute *ev3_motor_attrs[] = {
	&driver_attr_driver_names.attr,
	NULL
};

ATTRIBUTE_GROUPS(ev3_motor);

struct lego_device_driver ev3_motor_driver = {
	.probe	= ev3_motor_probe,
	.remove	= ev3_motor_remove,
	.driver = {
		.name	= "ev3-motor",
		.owner	= THIS_MODULE,
		.groups	= ev3_motor_groups,
	},
	.id_table = ev3_motor_driver_id_table,
};
lego_device_driver(ev3_motor_driver);

MODULE_DESCRIPTION("LEGO MINDSTORMS NXT/EV3 motor driver");
MODULE_AUTHOR("David Lechner <david@lechnology.com>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("lego:ev3-motor");
