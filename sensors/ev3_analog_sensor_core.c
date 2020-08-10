/*
 * LEGO MINDSTORMS EV3 Analog Sensor device driver
 *
 * Copyright (C) 2013-2015,2017 David Lechner <david@lechnology.com>
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
 * The ``ev3-analog-sensor`` module provides all of the drivers for EV3/Analog
 * sensors.
 *
 * You can a list of the the devices implemented by this module by reading the
 * ``driver_names`` attribute in the ``/sys/bus/lego/drivers/ev3-analog-sensor/``.
 */

#include <linux/device.h>
#include <linux/module.h>
#include <linux/slab.h>

#include <lego.h>

#include "ev3_analog_sensor.h"
#include "ms_ev3_smux.h"

static void ev3_touch_notify_raw_data_func(void *context)
{
	struct ev3_analog_sensor_data *data = context;
	struct lego_sensor_mode_info *mode_info;
	long int new_value;

	mode_info = &data->info.mode_info[0];
	mode_info->scale(context, mode_info, 0, &new_value);
	if (new_value != data->last_value) {
		sysfs_notify(&data->sensor.dev.kobj, NULL, "value0");
		data->last_value = new_value;
	}
}

static int ev3_analog_sensor_set_mode(void *context, u8 mode)
{
	struct ev3_analog_sensor_data *data = context;
	struct lego_sensor_mode_info *mode_info;
	lego_port_notify_raw_data_func_t func = NULL;

	if (mode >= data->info.num_modes)
		return -EINVAL;

	mode_info = &data->info.mode_info[mode];
	if (strcmp(data->info.name, LEGO_EV3_TOUCH_SENSOR_NAME) == 0)
		func = ev3_touch_notify_raw_data_func;
	else
		context = NULL;
	lego_port_set_raw_data_ptr_and_func(data->ldev->port, mode_info->raw_data,
		lego_sensor_get_raw_data_size(mode_info),
		&mode_info->last_changed_time, func, context);

	return 0;
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
	data->sensor.address = ldev->port->address;
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

	lego_port_set_raw_data_ptr_and_func(ldev->port, NULL, 0, NULL,
					    NULL, NULL);
	unregister_lego_sensor(&data->sensor);
	dev_set_drvdata(&ldev->dev, NULL);
	kfree(data);
	return 0;
}

#define EV3_ANALOG_SENSOR_GENERIC(type) {		\
	.name		= type##_NAME,			\
	.driver_data	= GENERIC_EV3_ANALOG_SENSOR,	\
}

static struct lego_device_id ev3_analog_sensor_device_ids [] = {
	EV3_ANALOG_SENSOR_GENERIC(EV3_ANALOG_SENSOR_ID_01),
	LEGO_DEVICE_ID(LEGO_EV3_TOUCH_SENSOR),
	EV3_ANALOG_SENSOR_GENERIC(EV3_ANALOG_SENSOR_ID_03),
	EV3_ANALOG_SENSOR_GENERIC(EV3_ANALOG_SENSOR_ID_04),
	EV3_ANALOG_SENSOR_GENERIC(EV3_ANALOG_SENSOR_ID_05),
	EV3_ANALOG_SENSOR_GENERIC(EV3_ANALOG_SENSOR_ID_06),
	EV3_ANALOG_SENSOR_GENERIC(EV3_ANALOG_SENSOR_ID_07),
	EV3_ANALOG_SENSOR_GENERIC(EV3_ANALOG_SENSOR_ID_08),
	EV3_ANALOG_SENSOR_GENERIC(EV3_ANALOG_SENSOR_ID_09),
	EV3_ANALOG_SENSOR_GENERIC(EV3_ANALOG_SENSOR_ID_10),
	EV3_ANALOG_SENSOR_GENERIC(EV3_ANALOG_SENSOR_ID_11),
	EV3_ANALOG_SENSOR_GENERIC(EV3_ANALOG_SENSOR_ID_12),
	EV3_ANALOG_SENSOR_GENERIC(EV3_ANALOG_SENSOR_ID_13),
	EV3_ANALOG_SENSOR_GENERIC(EV3_ANALOG_SENSOR_ID_14),
	{ }
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
