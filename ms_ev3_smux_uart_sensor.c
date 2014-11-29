/*
 * mindsensors.com EV3 Sensor Mux UART Sensor device driver for LEGO MINDSTORMS EV3
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
 * mindsensors.com EV3 Sensor Mux UART Sensor Driver
 *
 * The `ms-ev3-smux-uart-sensor` module provides all of the drivers for
 * UART/EV3 sensors. You can find the complete list [here][supported sensors].
 * .
 * ### sysfs Attributes
 * .
 * These drivers provide a [msensor device], which is where all the really
 * useful attributes are.
 * .
 * You can find this device at `/sys/bus/legoev3/devices/in<N>:mux<M>:<device-name>`
 * where `<N>` is the number of an input port (1 to 4), `<M> is the number of
 * the port on the sensor mux and `<device-name>` is the name of one of the
 * drivers in the `ev3-uart-sensor` module (e.g. `lego-ev3-uart-29`).
 * .
 * `device_type` (read-only)
 * : Returns `ms-ev3-smux-uart-sensor`
 * .
 * `port_name` (read-only)
 * : Returns the name of the port this host is connected to (e.g. `in1:mux2`).
 * .
 * [msensor device]: ../msensor-class
 * [supported sensors]: ../#supported-sensors
 */

#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/legoev3/msensor_class.h>

#include <asm/bug.h>

#include "ev3_uart_sensor.h"
#include "ms_ev3_smux.h"

struct ms_ev3_smux_uart_sensor_data {
	struct legoev3_port *in_port;
	struct i2c_client * client;
	struct msensor_device ms;
	struct ev3_uart_sensor_info info;
	u8 mode;
};

static int ms_ev3_smux_uart_sensor_set_mode(void *context, u8 mode)
{
	struct ms_ev3_smux_uart_sensor_data *sensor_data = context;

	return i2c_smbus_write_byte_data(sensor_data->client,
					 MS_EV3_SMUX_MODE_REG, mode);
}

static int ms_ev3_smux_uart_sensor_probe(struct legoev3_port_device *sensor)
{
	struct ms_ev3_smux_uart_sensor_data *sensor_data;
	struct ms_ev3_smux_sensor_platform_data *pdata =
						sensor->dev.platform_data;
	int err;

	if (WARN_ON(!sensor->entry_id))
		return -EINVAL;
	if (WARN_ON(!pdata))
		return -EINVAL;

	sensor_data = kzalloc(sizeof(struct ms_ev3_smux_uart_sensor_data),
								GFP_KERNEL);
	if (!sensor_data)
		return -ENOMEM;

	sensor_data->in_port = sensor->port;
	sensor_data->client = pdata->client;

	memcpy(&sensor_data->info,
	       &ev3_uart_sensor_defs[sensor->entry_id->driver_data],
	       sizeof(struct ev3_uart_sensor_info));
	strncpy(sensor_data->ms.name, sensor->entry_id->name, MSENSOR_NAME_SIZE);
	strncpy(sensor_data->ms.port_name, dev_name(&sensor_data->in_port->dev),
		MSENSOR_NAME_SIZE);
	/*
	 * The sensor mux only supports one data value, so we use the number of
	 * view modes to limit the list of modes to the modes that only return
	 * one data value.
	 */
	sensor_data->ms.num_modes	= sensor_data->info.num_view_modes;
	sensor_data->ms.mode_info	= sensor_data->info.ms_mode_info;
	sensor_data->ms.set_mode	= ms_ev3_smux_uart_sensor_set_mode;
	sensor_data->ms.context		= sensor_data;

	err = register_msensor(&sensor_data->ms, &sensor->dev);
	if (err)
		goto err_register_msensor;

	dev_set_drvdata(&sensor->dev, sensor_data);
	ms_ev3_smux_uart_sensor_set_mode(sensor_data, 0);

	dev_info(&sensor->dev, "UART sensor connected to port %s\n",
		 dev_name(&sensor_data->in_port->dev));

	return 0;

err_register_msensor:
	kfree(sensor_data);

	return err;
}

static int ms_ev3_smux_uart_sensor_remove(struct legoev3_port_device *sensor)
{
	struct ms_ev3_smux_uart_sensor_data *as = dev_get_drvdata(&sensor->dev);

	dev_info(&sensor->dev, "Analog sensor removed from port %s\n",
		 dev_name(&as->in_port->dev));
	as->in_port->in_ops.register_analog_cb(as->in_port, NULL, NULL);
	unregister_msensor(&as->ms);
	dev_set_drvdata(&sensor->dev, NULL);
	kfree(as);
	return 0;
}

/*
 * currently only supports these four sensors.
 */
static const struct legoev3_port_device_id ms_ev3_smux_uart_sensor_device_ids[] = {
	{ "lego-ev3-uart-29",	LEGO_EV3_COLOR		},
	{ "lego-ev3-uart-30",	LEGO_EV3_ULTRASONIC	},
	{ "lego-ev3-uart-32",	LEGO_EV3_GYRO		},
	{ "lego-ev3-uart-33",	LEGO_EV3_INFRARED	},
};

struct legoev3_port_device_driver ms_ev3_smux_uart_sensor_driver = {
	.probe	= ms_ev3_smux_uart_sensor_probe,
	.remove	= ms_ev3_smux_uart_sensor_remove,
	.driver = {
		.name	= "ms-ev3-smux-uart-sensor",
		.owner	= THIS_MODULE,
	},
	.id_table = ms_ev3_smux_uart_sensor_device_ids,
};
legoev3_port_device_driver(ms_ev3_smux_uart_sensor_driver);

MODULE_DESCRIPTION("mindsensors.com EV3 Sensor Mux UART sensor device driver for LEGO MINDSTORMS EV3");
MODULE_AUTHOR("David Lechner <david@lechnology.com>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("legoev3:ms-ev3-smux-uart-sensor");
