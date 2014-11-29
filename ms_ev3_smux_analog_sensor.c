/*
 * mindsensors.com EV3 Sensor Mux Analog Sensor device driver for LEGO MINDSTORMS EV3
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
 * mindsensors.com EV3 Sensor Mux Analog Sensor Driver
 *
 * The `ms-ev3-smux-analog-sensor` module provides all of the drivers for
 * Analog/EV3 sensors. You can find the complete list [here][supported sensors].
 * .
 * ### sysfs Attributes
 * .
 * These drivers provide a [msensor device], which is where all the really
 * useful attributes are.
 * .
 * You can find this device at `/sys/bus/legoev3/devices/in<N>:mux<M>:<device-name>`
 * where `<N>` is the number of an input port (1 to 4), `<M> is the number of
 * the port on the sensor mux and `<device-name>` is the name of one of the
 * drivers in the `ev3-analog-sensor` module (e.g. `lego-ev3-touch`).
 * .
 * `device_type` (read-only)
 * : Returns `ms-ev3-smux-analog-sensor`
 * .
 * `port_name` (read-only)
 * : Returns the name of the port this host is connected to (e.g. `in1:mux2`).
 * .
 * [msensor device]: ../msensor-class
 * [supported sensors]: ../#supported-sensors
 */

#include <linux/device.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/legoev3/legoev3_ports.h>
#include <linux/legoev3/msensor_class.h>

#include <asm/bug.h>

#include "ev3_analog_sensor.h"

static int ms_ev3_smux_analog_sensor_set_mode(void *context, u8 mode)

	/* currently does not support modes other than 0*/{
	if (mode)
		return -EINVAL;

	return 0;
}
extern struct ev3_analog_sensor_data ev3_analog_sensor_defs[];

static int ms_ev3_smux_analog_sensor_probe(struct legoev3_port_device *sensor)
{
	struct ev3_analog_sensor_data *as;
	int err;

	if (WARN_ON(!sensor->entry_id))
		return -EINVAL;

	as = kzalloc(sizeof(struct ev3_analog_sensor_data), GFP_KERNEL);
	if (!as)
		return -ENOMEM;

	as->in_port = sensor->port;

	memcpy(&as->info, &ev3_analog_sensor_defs[sensor->entry_id->driver_data],
	       sizeof(struct ms_ev3_smux_analog_sensor_info));
	strncpy(as->ms.name, sensor->entry_id->name, MSENSOR_NAME_SIZE);
	strncpy(as->ms.port_name, dev_name(&as->in_port->dev),
		MSENSOR_NAME_SIZE);
	as->ms.num_modes	= as->info.num_modes;
	as->ms.mode_info	= as->info.ms_mode_info;
	as->ms.set_mode		= ms_ev3_smux_analog_sensor_set_mode;
	as->ms.context		= as;

	err = register_msensor(&as->ms, &sensor->dev);
	if (err)
		goto err_register_msensor;

	dev_set_drvdata(&sensor->dev, as);
	ms_ev3_smux_analog_sensor_set_mode(as, 0);

	dev_info(&sensor->dev, "Analog sensor connected to port %s\n",
		 dev_name(&as->in_port->dev));

	return 0;

err_register_msensor:
	kfree(as);

	return err;
}

static int ms_ev3_smux_analog_sensor_remove(struct legoev3_port_device *sensor)
{
	struct ev3_analog_sensor_data *as = dev_get_drvdata(&sensor->dev);

	dev_info(&sensor->dev, "Analog sensor removed from port %s\n",
		 dev_name(&as->in_port->dev));
	as->in_port->in_ops.register_analog_cb(as->in_port, NULL, NULL);
	unregister_msensor(&as->ms);
	dev_set_drvdata(&sensor->dev, NULL);
	kfree(as);
	return 0;
}

extern struct legoev3_port_device_id ev3_analog_sensor_device_ids;

struct legoev3_port_device_driver ms_ev3_smux_analog_sensor_driver = {
	.probe	= ms_ev3_smux_analog_sensor_probe,
	.remove	= ms_ev3_smux_analog_sensor_remove,
	.driver = {
		.name	= "ms-ev3-smux-analog-sensor",
		.owner	= THIS_MODULE,
	},
	.id_table = ev3_analog_sensor_device_ids,
};
legoev3_port_device_driver(ms_ev3_smux_analog_sensor_driver);

MODULE_DESCRIPTION("mindsensors.com EV3 Sensor Mux analog sensor device driver for LEGO MINDSTORMS EV3");
MODULE_AUTHOR("David Lechner <david@lechnology.com>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("legoev3:ms-ev3-smux-analog-sensor");
