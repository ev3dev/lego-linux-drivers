/*
 * LEGO MINDSTORMS EV3 Analog Sensor device driver
 *
 * Copyright (C) 2013-2014 David Lechner <david@lechnology.com>
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
 * Use kramdown (markdown) syntax. Use a '.' as a placeholder when blank lines
 * or leading whitespace is important for the markdown syntax.
 */

/**
 * DOC: website
 *
 * EV3 Analog Sensor Driver
 *
 * The `ev3-analog-sensor` module provides all of the drivers for Analog/EV3
 * sensors. You can find the complete list [here][supported sensors].
 * .
 * ### sysfs
 * .
 * You can find all of the the devices bound to this driver in the directory
 * `/sys/bus/lego/drivers/ev3-analog-sensor/`. However, these drivers provide a
 * [lego-sensor class] device, which is where all the really useful attributes
 * are.
 * .
 * [lego-sensor class]: ../lego-sensor-class
 * [supported sensors]: ../#supported-sensors
 */

#include <linux/device.h>
#include <linux/module.h>
#include <linux/slab.h>

#include <lego.h>

#include "ev3_analog_sensor.h"
#include "ms_ev3_smux.h"

static int ev3_analog_sensor_set_mode(void *context, u8 mode)
{
	struct ev3_analog_sensor_data *data = context;
	struct lego_sensor_mode_info *mode_info = &data->info.mode_info[mode];

	lego_port_set_raw_data_ptr_and_func(data->ldev->port, mode_info->raw_data,
		lego_sensor_get_raw_data_size(mode_info), NULL, NULL);

	return -EINVAL;
}

static int ev3_analog_sensor_probe(struct lego_device *ldev)
{
	struct ev3_analog_sensor_data *data;
	int err;

	if (WARN_ON(!ldev->entry_id))
		return -EINVAL;

	data = kzalloc(sizeof(struct ev3_analog_sensor_data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->ldev = ldev;

	memcpy(&data->info, &ev3_analog_sensor_defs[ldev->entry_id->driver_data],
	       sizeof(struct ev3_analog_sensor_info));
	data->sensor.name = ldev->entry_id->name;
	data->sensor.port_name = ldev->port->port_name;
	data->sensor.num_modes	= data->info.num_modes;
	data->sensor.mode_info	= data->info.mode_info;
	data->sensor.set_mode	= ev3_analog_sensor_set_mode;
	data->sensor.context	= data;

	err = register_lego_sensor(&data->sensor, &ldev->dev);
	if (err)
		goto err_register_lego_sensor;

	dev_set_drvdata(&ldev->dev, data);
	ev3_analog_sensor_set_mode(data, 0);

	return 0;

err_register_lego_sensor:
	kfree(data);

	return err;
}

static int ev3_analog_sensor_remove(struct lego_device *ldev)
{
	struct ev3_analog_sensor_data *data = dev_get_drvdata(&ldev->dev);

	lego_port_set_raw_data_ptr_and_func(ldev->port, NULL, 0, NULL, NULL);
	unregister_lego_sensor(&data->sensor);
	dev_set_drvdata(&ldev->dev, NULL);
	kfree(data);
	return 0;
}

static struct lego_device_id ev3_analog_sensor_device_ids [] = {
	{
		.name = "ev3-analog-01",
		.driver_data = GENERIC_EV3_ANALOG_SENSOR,
	},
	{
		.name = "lego-ev3-touch",
		.driver_data = LEGO_EV3_TOUCH_SENSOR,
	},
	{
		.name = "ev3-analog-03",
		.driver_data = GENERIC_EV3_ANALOG_SENSOR,
	},
	{
		.name = "ev3-analog-04",
		.driver_data = GENERIC_EV3_ANALOG_SENSOR,
	},
	{
		.name = "ev3-analog-05",
		.driver_data = GENERIC_EV3_ANALOG_SENSOR,
	},
	{
		.name = "ev3-analog-06",
		.driver_data = GENERIC_EV3_ANALOG_SENSOR,
	},
	{
		.name = "ev3-analog-07",
		.driver_data = GENERIC_EV3_ANALOG_SENSOR,
	},
	{
		.name = "ev3-analog-08",
		.driver_data = GENERIC_EV3_ANALOG_SENSOR,
	},
	{
		.name = "ev3-analog-09",
		.driver_data = GENERIC_EV3_ANALOG_SENSOR,
	},
	{
		.name = "ev3-analog-10",
		.driver_data = GENERIC_EV3_ANALOG_SENSOR,
	},
	{
		.name = "ev3-analog-11",
		.driver_data = GENERIC_EV3_ANALOG_SENSOR,
	},
	{
		.name = "ev3-analog-12",
		.driver_data = GENERIC_EV3_ANALOG_SENSOR,
	},
	{
		.name = "ev3-analog-13",
		.driver_data = GENERIC_EV3_ANALOG_SENSOR,
	},
	{
		.name = "ev3-analog-14",
		.driver_data = GENERIC_EV3_ANALOG_SENSOR,
	},
	{  }
};

struct lego_device_driver ev3_analog_sensor_driver = {
	.probe	= ev3_analog_sensor_probe,
	.remove	= ev3_analog_sensor_remove,
	.driver = {
		.name	= "ev3-analog-sensor",
		.owner	= THIS_MODULE,
	},
	.id_table = ev3_analog_sensor_device_ids,
};
lego_device_driver(ev3_analog_sensor_driver);

MODULE_DESCRIPTION("LEGO MINDSTORMS EV3 Analog sensor device driver");
MODULE_AUTHOR("David Lechner <david@lechnology.com>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("lego:ev3-analog-sensor");
