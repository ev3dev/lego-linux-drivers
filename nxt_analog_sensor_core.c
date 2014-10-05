/*
 * NXT Analog Sensor device driver for LEGO Mindstorms EV3
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
 * Use kramdown (markdown) format. Use a '.' as a placeholder when blank lines
 * or leading whitespace is important for the markdown syntax.
 */

/**
 * DOC: website
 *
 * NXT Analog Sensor Driver
 *
 * The `nxt-analog-sensor` module provides all of the drivers for Analog/NXT
 * sensors. You can find the complete list [here][supported sensors].
 * .
 * ### sysfs Attributes
 * .
 * These drivers provide a [msensor device], which is where all the really
 * useful attributes are.
 * .
 * You can find this device at `/sys/bus/legoev3/devices/in<N>:<device-name>`
 * where `<N>` is the number of an input port (1 to 4) and `<device-name>` is
 * the name of one of the drivers in the `nxt-analog-sensor` module (e.g.
 * `lego-nxt-sound`). NOTE: These drivers are also used by the [HiTechnic NXT
 * Sensor Multiplexer], in which case the device name will be
 * `in<N>:mux<M>:nxt-analog-host`.
 * .
 * `device_type` (read-only)
 * : Returns `nxt-analog-sensor`
 * .
 * `port_name` (read-only)
 * : Returns the name of the port this host is connected to (e.g. `in1`).
 * .
 * [msensor device]: ../msensor-class
 * [supported sensors]: ../#supported-sensors
 */

#include <linux/device.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/legoev3/legoev3_ports.h>
#include <linux/legoev3/ev3_input_port.h>
#include <linux/legoev3/msensor_class.h>

#include <asm/bug.h>

#include "nxt_analog_sensor.h"

static void nxt_analog_sensor_cb(void *context)
{
	struct nxt_analog_sensor_data *as = context;

	*(int*)as->info.ms_mode_info[as->mode].raw_data =
				as->in_port->in_ops.get_pin1_mv(as->in_port);
}

static u8 nxt_analog_sensor_get_mode(void *context)
{
	struct nxt_analog_sensor_data *as = context;

	return as->mode;
}

static int nxt_analog_sensor_set_mode(void *context, u8 mode)
{
	struct nxt_analog_sensor_data *as = context;

	if (mode >= as->info.num_modes)
		return -EINVAL;

	as->in_port->in_ops.set_pin5_gpio(as->in_port,
				as->info.analog_mode_info[mode].pin5_state);
	if (as->info.analog_mode_info[mode].analog_cb)
		as->in_port->in_ops.register_analog_cb(as->in_port,
				as->info.analog_mode_info[mode].analog_cb, as);
	else
		as->in_port->in_ops.register_analog_cb(as->in_port,
						nxt_analog_sensor_cb, as);
	as->mode = mode;

	return 0;
}

static int nxt_analog_sensor_probe(struct legoev3_port_device *sensor)
{
	struct nxt_analog_sensor_data *as;
	int err;

	if (WARN_ON(!sensor->entry_id))
		return -EINVAL;

	as = kzalloc(sizeof(struct nxt_analog_sensor_data), GFP_KERNEL);
	if (!as)
		return -ENOMEM;

	as->in_port = sensor->port;

	memcpy(&as->info, &nxt_analog_sensor_defs[sensor->entry_id->driver_data],
	       sizeof(struct nxt_analog_sensor_info));
	strncpy(as->ms.name, sensor->entry_id->name, MSENSOR_NAME_SIZE);
	strncpy(as->ms.port_name, dev_name(&as->in_port->dev),
		MSENSOR_NAME_SIZE);
	as->ms.num_modes	= as->info.num_modes;
	as->ms.mode_info	= as->info.ms_mode_info;
	as->ms.get_mode		= nxt_analog_sensor_get_mode;
	as->ms.set_mode		= nxt_analog_sensor_set_mode;
	as->ms.context		= as;

	err = register_msensor(&as->ms, &sensor->dev);
	if (err)
		goto err_register_msensor;

	dev_set_drvdata(&sensor->dev, as);
	nxt_analog_sensor_set_mode(as, 0);

	return 0;

err_register_msensor:
	kfree(as);

	return err;
}

static int nxt_analog_sensor_remove(struct legoev3_port_device *sensor)
{
	struct nxt_analog_sensor_data *as = dev_get_drvdata(&sensor->dev);

	as->in_port->in_ops.set_pin5_gpio(as->in_port, EV3_INPUT_PORT_GPIO_FLOAT);
	as->in_port->in_ops.register_analog_cb(as->in_port, NULL, NULL);
	unregister_msensor(&as->ms);
	dev_set_drvdata(&sensor->dev, NULL);
	kfree(as);
	return 0;
}

static struct legoev3_port_device_id nxt_analog_sensor_device_ids [] = {
	{
		.name = "nxt-analog",
		.driver_data = GENERIC_NXT_ANALOG_SENSOR,
	},
	{
		.name = "lego-nxt-touch",
		.driver_data = LEGO_NXT_TOUCH_SENSOR,
	},
	{
		.name = "lego-nxt-light",
		.driver_data = LEGO_NXT_LIGHT_SENSOR,
	},
	{
		.name = "lego-nxt-sound",
		.driver_data = LEGO_NXT_SOUND_SENSOR,
	},
	{
		.name = "ht-nxt-eopd",
		.driver_data = HT_EOPD_SENSOR,
	},
	{
		.name = "ht-nxt-force",
		.driver_data = HT_FORCE_SENSOR,
	},
	{
		.name = "ht-nxt-gyro",
		.driver_data = HT_GYRO_SENSOR,
	},
	{
		.name = "ht-nxt-mag",
		.driver_data = HT_MAGNETIC_SENSOR,
	},
	{
		.name = "ms-nxt-touch-mux",
		.driver_data = MS_TOUCH_SENSOR_MUX,
	},
	{  }
};

static ssize_t sensor_names_show(struct device_driver *driver, char *buf)
{
	struct legoev3_port_device_id *id = nxt_analog_sensor_device_ids;
	ssize_t total = 0;
	int size;

	while (id->name[0]) {
		size = sprintf(buf, "%s ", id->name);
		total += size;
		buf += size;
		id++;
	}
	buf--;
	buf[0] = '\n';
	buf[1] = 0;
	total++;

	return total;
}

DRIVER_ATTR_RO(sensor_names);

static struct attribute *nxt_analog_sensor_names_attrs[] = {
	&driver_attr_sensor_names.attr,
	NULL,
};

ATTRIBUTE_GROUPS(nxt_analog_sensor_names);

struct legoev3_port_device_driver nxt_analog_sensor_driver = {
	.probe	= nxt_analog_sensor_probe,
	.remove	= nxt_analog_sensor_remove,
	.driver = {
		.name	= "nxt-analog-sensor",
		.owner	= THIS_MODULE,
		.groups	= nxt_analog_sensor_names_groups,
	},
	.id_table = nxt_analog_sensor_device_ids,
};
legoev3_port_device_driver(nxt_analog_sensor_driver);

MODULE_DESCRIPTION("NXT Analog sensor device driver for LEGO Mindstorms EV3");
MODULE_AUTHOR("David Lechner <david@lechnology.com>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("legoev3:nxt-analog-sensor");
