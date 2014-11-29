/*
 * EV3 analog host driver for LEGO MINDSTORMS EV3
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
 * EV3 Analog Host Driver
 *
 * This driver tells an [EV3 input port] to configure itself for Analog/EV3
 * communications and loads a sensor device based on the id resistor of the
 * sensor that is connected to the port.
 * .
 * ### sysfs Attributes
 * .
 *  * You can find this device at `/sys/bus/legoev3/devices/in<N>:ev3-analog-host`
 * where `<N>` is the number of an input port (1 to 4).
 * .
 * `device_type` (read-only)
 * : Returns `ev3-analog-host`
 * .
 * `port_name` (read-only)
 * : Returns the name of the port this host is connected to (e.g. `in1`).
 * .
 * [EV3 input port]: ../ev3-input-port
 */

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/legoev3/legoev3_ports.h>
#include <linux/legoev3/ev3_input_port.h>

struct ev3_analog_host_data {
	struct legoev3_port *in_port;
	struct legoev3_port_device *sensor;
};

const struct attribute_group *ev3_analog_sensor_device_type_attr_groups[] = {
	&legoev3_port_device_type_attr_grp,
	NULL
};

struct device_type ev3_analog_sensor_device_type = {
	.name	= "ev3-analog-sensor",
	.groups	= ev3_analog_sensor_device_type_attr_groups,
	.uevent = legoev3_port_device_uevent,
};

static int ev3_analog_host_probe(struct legoev3_port_device *host)
{
	struct ev3_analog_host_data *data;
	struct ev3_analog_host_platform_data *pdata = host->dev.platform_data;
	struct legoev3_port_device *sensor;

	data = kzalloc(sizeof(struct ev3_analog_host_data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->in_port = host->port;
	dev_set_drvdata(&host->dev, data);

	if (pdata && pdata->inital_sensor)
	{
		sensor = legoev3_port_device_register(pdata->inital_sensor,
			&ev3_analog_sensor_device_type, &host->dev, NULL, 0,
			data->in_port);
		if (IS_ERR(sensor))
			dev_warn(&host->dev, "Failed to register sensor %s. %ld",
				pdata->inital_sensor, PTR_ERR(sensor));
		else
			data->sensor = sensor;
	}

	return 0;
}

static int ev3_analog_host_remove(struct legoev3_port_device *host)
{
	struct ev3_analog_host_data *data = dev_get_drvdata(&host->dev);

	if (data->sensor)
		legoev3_port_device_unregister(data->sensor);
	dev_set_drvdata(&host->dev, NULL);
	kfree(data);

	return 0;
}

struct legoev3_port_device_driver ev3_analog_host_driver = {
	.probe	= ev3_analog_host_probe,
	.remove	= ev3_analog_host_remove,
	.driver = {
		.name	= "ev3-analog-host",
		.owner	= THIS_MODULE,
	},
};
legoev3_port_device_driver(ev3_analog_host_driver);

MODULE_DESCRIPTION("EV3 analog host driver for LEGO MINDSTORMS EV3");
MODULE_AUTHOR("David Lechner <david@lechnology.com>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("legoev3:ev3-analog-host");
