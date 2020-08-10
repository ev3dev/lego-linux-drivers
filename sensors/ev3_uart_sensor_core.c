/*
 * LEGO MINDSTORMS EV3 UART Sensor driver
 *
 * Copyright (C) 2014-2015 David Lechner <david@lechnology.com>
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
 * The ``ev3-uart-sensor`` module provides all of the drivers for EV3/UART
 * sensors. This module is only used on devices that can't use the line
 * discipline.
 *
 * You can a list of the the devices implemented by this module by reading the
 * ``driver_names`` attribute in the ``/sys/bus/lego/drivers/ev3-uart-sensor/``.
 */

#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/slab.h>

#include <lego.h>
#include <lego_sensor_class.h>

#include "ev3_uart_sensor.h"
#include "ms_ev3_smux.h"

struct ev3_uart_sensor_data {
	struct lego_device *ldev;
	struct lego_sensor_device sensor;
	struct ev3_uart_sensor_info info;
	u8 mode;
};

static int ev3_uart_sensor_set_mode(void *context, u8 mode)
{
	struct ev3_uart_sensor_data *data = context;
	struct lego_port_device *port = data->ldev->port;
	struct lego_sensor_mode_info *mode_info = &data->info.mode_info[mode];

	if (port->ev3_uart_ops && port->ev3_uart_ops->set_mode) {
		int ret;

		ret = port->ev3_uart_ops->set_mode(port->context,
						   data->info.type_id, mode);
		if (ret < 0)
			return ret;
	} else
		return -EOPNOTSUPP;

	lego_port_set_raw_data_ptr_and_func(data->ldev->port, mode_info->raw_data,
		lego_sensor_get_raw_data_size(mode_info),
		&mode_info->last_changed_time, NULL, NULL);

	return 0;
}

static int ev3_uart_sensor_probe(struct lego_device *ldev)
{
	struct ev3_uart_sensor_data *data;
	int err;

	if (WARN_ON(!ldev->entry_id))
		return -EINVAL;

	data = kzalloc(sizeof(struct ev3_uart_sensor_data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->ldev = ldev;
	memcpy(&data->info,
	       &ev3_uart_sensor_defs[ldev->entry_id->driver_data],
	       sizeof(struct ev3_uart_sensor_info));
	data->sensor.name = ldev->entry_id->name;
	data->sensor.address = ldev->port->address;
#if defined(CONFIG_NXT_I2C_SENSORS) || defined(CONFIG_NXT_I2C_SENSORS_MODULE)
	/* mindsensors EV3 sensor mux only supports modes that return one value */
	if (ldev->port->dev.type == &ms_ev3_smux_port_type)
		data->sensor.num_modes = data->info.num_view_modes;
	else
#endif
		data->sensor.num_modes = data->info.num_modes;
	data->sensor.mode_info	= data->info.mode_info;
	data->sensor.set_mode	= ev3_uart_sensor_set_mode;
	data->sensor.context	= data;

	err = register_lego_sensor(&data->sensor, &ldev->dev);
	if (err)
		goto err_register_lego_sensor;

	dev_set_drvdata(&ldev->dev, data);
	ev3_uart_sensor_set_mode(data, 0);

	return 0;

err_register_lego_sensor:
	kfree(data);

	return err;
}

static int ev3_uart_sensor_remove(struct lego_device *ldev)
{
	struct ev3_uart_sensor_data *data = dev_get_drvdata(&ldev->dev);

	lego_port_set_raw_data_ptr_and_func(ldev->port, NULL, 0, NULL,
					    NULL, NULL);
	unregister_lego_sensor(&data->sensor);
	dev_set_drvdata(&ldev->dev, NULL);
	kfree(data);
	return 0;
}

static const struct lego_device_id ev3_uart_sensor_device_ids[] = {
	{ LEGO_EV3_COLOR_NAME,		LEGO_EV3_COLOR		},
	{ LEGO_EV3_ULTRASONIC_NAME,	LEGO_EV3_ULTRASONIC	},
	{ LEGO_EV3_GYRO_NAME,		LEGO_EV3_GYRO		},
	{ LEGO_EV3_INFRARED_NAME,	LEGO_EV3_INFRARED	},
};

struct lego_device_driver ev3_uart_sensor_driver = {
	.probe	= ev3_uart_sensor_probe,
	.remove	= ev3_uart_sensor_remove,
	.driver = {
		.name	= "ev3-uart-sensor",
		.owner	= THIS_MODULE,
	},
	.id_table = ev3_uart_sensor_device_ids,
};
lego_device_driver(ev3_uart_sensor_driver);

MODULE_DESCRIPTION("LEGO EV3 UART Sensor driver");
MODULE_AUTHOR("David Lechner <david@lechnology.com>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("lego:ev3-uart-sensor");
