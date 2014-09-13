/*
 * EV3 Analog Sensor device driver for LEGO Mindstorms EV3
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

#include <linux/device.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/legoev3/legoev3_ports.h>
#include <linux/legoev3/ev3_input_port.h>
#include <linux/legoev3/msensor_class.h>

#include <asm/bug.h>

#include "ev3_analog_sensor.h"

static void ev3_analog_sensor_cb(void *context)
{
	struct ev3_analog_sensor_data *as = context;

	*(int*)as->info.ms_mode_info[as->mode].raw_data =
					ev3_input_port_get_pin6_mv(as->in_port);
}

static u8 ev3_analog_sensor_get_mode(void *context)
{
	struct ev3_analog_sensor_data *as = context;

	return as->mode;
}

static int ev3_analog_sensor_set_mode(void *context, u8 mode)
{
	struct ev3_analog_sensor_data *as = context;

	if (mode >= as->info.num_modes)
		return -EINVAL;

	ev3_input_port_set_pin5_gpio(as->in_port,
				as->info.analog_mode_info[mode].pin5_state);
	if (as->info.analog_mode_info[mode].analog_cb)
		ev3_input_port_register_analog_cb(as->in_port,
				as->info.analog_mode_info[mode].analog_cb, as);
	else
		ev3_input_port_register_analog_cb(as->in_port,
						  ev3_analog_sensor_cb, as);
	as->mode = mode;

	return 0;
}

static int ev3_analog_sensor_probe(struct legoev3_port_device *sensor)
{
	struct ev3_analog_sensor_data *as;
	int err;

	if (WARN_ON(!sensor->entry_id))
		return -EINVAL;

	as = kzalloc(sizeof(struct ev3_analog_sensor_data), GFP_KERNEL);
	if (!as)
		return -ENOMEM;

	as->in_port = sensor->port;

	strncpy(as->ms.name, dev_name(&sensor->dev), MSENSOR_NAME_SIZE);
	strncpy(as->ms.port_name, dev_name(&as->in_port->dev),
		MSENSOR_NAME_SIZE);
	as->ms.mode_info	= as->info.ms_mode_info;
	as->ms.get_mode		= ev3_analog_sensor_get_mode;
	as->ms.set_mode		= ev3_analog_sensor_set_mode;
	as->ms.context		= as;

	memcpy(&as->info, &ev3_analog_sensor_defs[sensor->entry_id->driver_data],
	       sizeof(struct ev3_analog_sensor_info));

	err = register_msensor(&as->ms, &sensor->dev);
	if (err)
		goto err_register_msensor;

	dev_set_drvdata(&sensor->dev, as);
	ev3_analog_sensor_set_mode(as, 0);

	dev_info(&sensor->dev, "Analog sensor connected to port %s\n",
		 dev_name(&as->in_port->dev));

	return 0;

err_register_msensor:
	kfree(as);

	return err;
}

static int ev3_analog_sensor_remove(struct legoev3_port_device *sensor)
{
	struct ev3_analog_sensor_data *as = dev_get_drvdata(&sensor->dev);

	dev_info(&sensor->dev, "Analog sensor removed from port %s\n",
		 dev_name(&as->in_port->dev));
	ev3_input_port_set_pin5_gpio(as->in_port, EV3_INPUT_PORT_GPIO_FLOAT);
	ev3_input_port_register_analog_cb(as->in_port, NULL, NULL);
	unregister_msensor(&as->ms);
	dev_set_drvdata(&sensor->dev, NULL);
	kfree(as);
	return 0;
}

static struct legoev3_port_device_id ev3_analog_sensor_device_ids [] = {
	{
		.name = "ev3-analog-01",
		.driver_data = GENERIC_EV3_ANALOG_SENSOR,
	},
	{
		.name = "ev3-touch",
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

/**
 * Returns 0 if the name is valid or -EINVAL if not.
 */
int ev3_analog_sensor_assert_valid_name(const char* name)
{
	struct legoev3_port_device_id *id = ev3_analog_sensor_device_ids;

	while (id->name[0]) {
		if (!strcmp(name, id->name))
			return 0;
		id++;
	}

	return -EINVAL;
}
EXPORT_SYMBOL_GPL(ev3_analog_sensor_assert_valid_name);

struct legoev3_port_device_driver ev3_analog_sensor_driver = {
	.probe	= ev3_analog_sensor_probe,
	.remove	= ev3_analog_sensor_remove,
	.driver = {
		.name	= "ev3-analog-sensor",
		.owner	= THIS_MODULE,
	},
	.id_table = ev3_analog_sensor_device_ids,
};
EXPORT_SYMBOL_GPL(ev3_analog_sensor_driver);
legoev3_port_device_driver(ev3_analog_sensor_driver);

MODULE_DESCRIPTION("EV3 Analog sensor device driver for LEGO Mindstorms EV3");
MODULE_AUTHOR("David Lechner <david@lechnology.com>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("legoev3:ev3-analog-sensor");
