/*
 * mindsensors.com EV3 Sensor Mux analog host driver for LEGO MINDSTORMS EV3
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
 * This driver is loaded when a [mindsensors.com EV3 Sensor Mux input port] is
 * set to the the `analog` mode.
 * .
 * ### sysfs Attributes
 * .
 *  * You can find this device at `/sys/bus/legoev3/devices/in<N>:ms-ev3-smux-analog-host`
 * where `<N>` is the number of an input port (1 to 4).
 * .
 * `device_type` (read-only)
 * : Returns `ms-ev3-smux-analog-host`
 * .
 * `port_name` (read-only)
 * : Returns the name of the port this host is connected to (e.g. `in1:mux2`).
 * .
 * [EV3 input port]: ../ev3-input-port
 */

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/legoev3/legoev3_ports.h>
#include <linux/legoev3/ev3_input_port.h>

struct ms_ev3_smux_analog_host_data {
	struct legoev3_port *in_port;
	struct legoev3_port_device *sensor;
};

const struct
attribute_group *ms_ev3_smux_analog_sensor_device_type_attr_groups[] = {
	&legoev3_port_device_type_attr_grp,
	NULL
};

struct device_type ms_ev3_smux_analog_sensor_device_type = {
	.name	= "ms-ev3-smux-analog-sensor",
	.groups	= ms_ev3_smux_analog_sensor_device_type_attr_groups,
	.uevent	= legoev3_port_device_uevent,
};

static int ms_ev3_smux_analog_host_probe(struct legoev3_port_device *host)
{
	struct ms_ev3_smux_analog_host_data *data;
	struct ms_ev3_smux_analog_host_platform_data *pdata =
							host->dev.platform_data;
	struct legoev3_port_device *sensor;

	data = kzalloc(sizeof(struct ms_ev3_smux_analog_host_data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->in_port = host->port;
	dev_set_drvdata(&host->dev, data);

	if (pdata && pdata->inital_sensor)
	{
		sensor = legoev3_port_device_register(pdata->inital_sensor,
			&ms_ev3_smux_analog_sensor_device_type, &host->dev,
			NULL, 0, data->in_port);
		if (IS_ERR(sensor))
			dev_warn(&host->dev, "Failed to register sensor %s. %ld",
				pdata->inital_sensor, PTR_ERR(sensor));
		else
			data->sensor = sensor;
	}

	return 0;
}

static int ms_ev3_smux_analog_host_remove(struct legoev3_port_device *host)
{
	struct ms_ev3_smux_analog_host_data *data = dev_get_drvdata(&host->dev);

	if (data->sensor)
		legoev3_port_device_unregister(data->sensor);
	dev_set_drvdata(&host->dev, NULL);
	kfree(data);

	return 0;
}

struct legoev3_port_device_driver ms_ev3_smux_analog_host_driver = {
	.probe	= ms_ev3_smux_analog_host_probe,
	.remove	= ms_ev3_smux_analog_host_remove,
	.driver = {
		.name	= "ms-ev3-smux-analog-host",
		.owner	= THIS_MODULE,
	},
};
legoev3_port_device_driver(ms_ev3_smux_analog_host_driver);

MODULE_DESCRIPTION("minsensors.com EV3 Sensor Mux analog host driver for LEGO MINDSTORMS EV3");
MODULE_AUTHOR("David Lechner <david@lechnology.com>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("legoev3:ms-ev3-smux-analog-host");
