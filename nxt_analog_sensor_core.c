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
					ev3_input_port_get_pin1_mv(as->in_port);
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

	ev3_input_port_set_pin5_gpio(as->in_port,
				as->info.analog_mode_info[mode].pin5_state);
	if (as->info.analog_mode_info[mode].analog_cb)
		ev3_input_port_register_analog_cb(as->in_port,
				as->info.analog_mode_info[mode].analog_cb, as);
	else
		ev3_input_port_register_analog_cb(as->in_port,
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

	strncpy(as->ms.name, dev_name(&sensor->dev), MSENSOR_NAME_SIZE);
	strncpy(as->ms.port_name, dev_name(&as->in_port->dev),
		MSENSOR_NAME_SIZE);
	as->ms.mode_info	= as->info.ms_mode_info;
	as->ms.get_mode		= nxt_analog_sensor_get_mode;
	as->ms.set_mode		= nxt_analog_sensor_set_mode;
	as->ms.context		= as;

	memcpy(&as->info, &nxt_analog_sensor_defs[sensor->entry_id->driver_data],
	       sizeof(struct nxt_analog_sensor_info));

	err = register_msensor(&as->ms, &sensor->dev);
	if (err)
		goto err_register_msensor;

	dev_set_drvdata(&sensor->dev, as);
	nxt_analog_sensor_set_mode(as, 0);

	dev_info(&sensor->dev, "Analog sensor connected to port %s\n",
		 dev_name(&as->in_port->dev));

	return 0;

err_register_msensor:
	kfree(as);

	return err;
}

static int nxt_analog_sensor_remove(struct legoev3_port_device *sensor)
{
	struct nxt_analog_sensor_data *as = dev_get_drvdata(&sensor->dev);

	dev_info(&sensor->dev, "Analog sensor removed from port %s\n",
		 dev_name(&as->in_port->dev));
	ev3_input_port_set_pin5_gpio(as->in_port, EV3_INPUT_PORT_GPIO_FLOAT);
	ev3_input_port_register_analog_cb(as->in_port, NULL, NULL);
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
		.name = "nxt-touch",
		.driver_data = LEGO_NXT_TOUCH_SENSOR,
	},
	{
		.name = "nxt-light",
		.driver_data = LEGO_NXT_LIGHT_SENSOR,
	},
	{
		.name = "nxt-sound",
		.driver_data = LEGO_NXT_SOUND_SENSOR,
	},
	{
		.name = "ht-neo1048",
		.driver_data = HT_EOPD_SENSOR,
	},
	{
		.name = "ht-nfs1074",
		.driver_data = HT_FORCE_SENSOR,
	},
	{
		.name = "ht-nyg1044",
		.driver_data = HT_GYRO_SENSOR,
	},
	{
		.name = "ht-nms1035",
		.driver_data = HT_MAGNETIC_SENSOR,
	},
	{
		.name = "ms-touch-mux",
		.driver_data = MS_TOUCH_SENSOR_MUX,
	},
	{  }
};

/**
 * Returns 0 if the name is valid or -EINVAL if not.
 */
int nxt_analog_sensor_assert_valid_name(const char* name)
{
	struct legoev3_port_device_id *id = nxt_analog_sensor_device_ids;

	while (id->name[0]) {
		if (!strcmp(name, id->name))
			return 0;
		id++;
	}

	return -EINVAL;
}
EXPORT_SYMBOL_GPL(nxt_analog_sensor_assert_valid_name);

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
EXPORT_SYMBOL_GPL(nxt_analog_sensor_driver);
legoev3_port_device_driver(nxt_analog_sensor_driver);

MODULE_DESCRIPTION("NXT Analog sensor device driver for LEGO Mindstorms EV3");
MODULE_AUTHOR("David Lechner <david@lechnology.com>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("legoev3:nxt-analog-sensor");
