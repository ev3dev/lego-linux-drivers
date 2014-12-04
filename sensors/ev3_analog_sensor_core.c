/*
 * EV3 Analog Sensor device driver
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
 * These drivers provide a [lego-sensor class] device, which is where all the
 * really useful attributes are.
 * .
 * You can find this device at `/sys/bus/lego/devices/port<N>:<device-name>`
 * where `<N>` is the number of a [lego-port class] device and `<device-name>`
 * is the name of one of the drivers in the `ev3-analog-sensor` module (e.g.
 * `lego-ev3-touch`).
 * .
 * [lego-sensor class]: ../lego-sensor-class
 * [lego-port class]: ../lego-port-class
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
	const struct ms_ev3_smux_port_type_ops *ms_ev3_smux_ops;

	if (data->ldev->port->type == &ms_ev3_smux_port_type) {
		ms_ev3_smux_ops = data->ldev->port->type->ops;
		ms_ev3_smux_ops->set_raw_data_ptr(data->ldev->port,
			data->sensor.mode_info[mode].raw_data,
			lego_sensor_data_size[data->sensor.mode_info[mode].data_type]);
		return 0;
	}

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
	strncpy(data->sensor.name, ldev->entry_id->name, LEGO_SENSOR_NAME_SIZE);
	strncpy(data->sensor.port_name, ldev->port->port_name,
		LEGO_SENSOR_NAME_SIZE);
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
	const struct ms_ev3_smux_port_type_ops *ms_ev3_smux_ops;

	if (ldev->port->type == &ms_ev3_smux_port_type) {
		ms_ev3_smux_ops = ldev->port->type->ops;
		ms_ev3_smux_ops->set_raw_data_ptr(ldev->port, NULL, 0);
	}
	unregister_lego_sensor(&data->sensor);
	dev_set_drvdata(&ldev->dev, NULL);
	kfree(data);
	return 0;
}

static struct legoev3_port_device_id ev3_analog_sensor_device_ids [] = {
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

MODULE_DESCRIPTION("EV3 Analog sensor device driver");
MODULE_AUTHOR("David Lechner <david@lechnology.com>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("lego:ev3-analog-sensor");
