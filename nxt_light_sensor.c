/*
 * NXT Light Sensor device driver for LEGO Mindstorms EV3
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

#define NUM_NXT_LIGHT_SENSOR_MODES	2

struct nxt_light_sensor_data {
	struct legoev3_port_device *in_port;
	struct msensor_device ms;
	struct msensor_mode_info mode_info[NUM_NXT_LIGHT_SENSOR_MODES];
	u8 mode;
};

static struct msensor_mode_info
nxt_light_sensor_mode_info[NUM_NXT_LIGHT_SENSOR_MODES] = {
	{
		.name = "NXT-REFLECT",
		.units = "pct",
		.raw_min = 3372,
		.raw_max = 445,
		.pct_max = 100,
		.si_max = 100,
		.data_sets = 1,
		.data_type = MSENSOR_DATA_U16,
	},
	{
		.name = "NXT-AMBIENT",
		.units = "pct",
		.raw_min = 3372,
		.raw_max = 445,
		.pct_max = 100,
		.si_max = 100,
		.data_sets = 1,
		.data_type = MSENSOR_DATA_U16,
	},
};

static u8 nxt_light_sensor_get_mode(void *context)
{
	struct nxt_light_sensor_data *ls = context;

	return ls->mode;
}

static int nxt_light_sensor_set_mode(void *context, u8 mode)
{
	struct nxt_light_sensor_data *ls = context;

	if (mode >= NUM_NXT_LIGHT_SENSOR_MODES)
		return -EINVAL;

	if (mode == 0)
		ev3_input_port_set_pin5_gpio(ls->in_port,
					     EV3_INPUT_PORT_GPIO_HIGH);
	else
		ev3_input_port_set_pin5_gpio(ls->in_port,
					     EV3_INPUT_PORT_GPIO_LOW);
	ls->mode = mode;

	return 0;
}

static void nxt_light_sensor_cb(void *context)
{
	struct nxt_light_sensor_data *ts = context;

	*(int*)ts->mode_info[0].raw_data =
			ev3_input_port_get_pin1_mv(ts->in_port);
}

static int nxt_light_sensor_probe(struct legoev3_port_device *sensor)
{
	struct nxt_light_sensor_data *ls;
	struct ev3_sensor_platform_data *pdata = sensor->dev.platform_data;
	int err;

	if (WARN_ON(!pdata))
		return -EINVAL;

	ls = kzalloc(sizeof(struct nxt_light_sensor_data), GFP_KERNEL);
	if (!ls)
		return -ENOMEM;

	ls->in_port = pdata->in_port;

	ls->ms.type_id		= sensor->type_id;
	strncpy(ls->ms.port_name, dev_name(&pdata->in_port->dev),
	        MSENSOR_PORT_NAME_SIZE);
	ls->ms.num_modes	= NUM_NXT_LIGHT_SENSOR_MODES;
	ls->ms.num_view_modes	= 1;
	ls->ms.mode_info	= ls->mode_info;
	ls->ms.get_mode		= nxt_light_sensor_get_mode;
	ls->ms.set_mode		= nxt_light_sensor_set_mode;
	ls->ms.context		= ls;

	memcpy(ls->mode_info, nxt_light_sensor_mode_info,
	       sizeof(struct msensor_mode_info) * NUM_NXT_LIGHT_SENSOR_MODES);

	err = register_msensor(&ls->ms, &sensor->dev);
	if (err)
		goto err_register_msensor;

	ev3_input_port_register_analog_cb(ls->in_port, nxt_light_sensor_cb, ls);

	err = dev_set_drvdata(&sensor->dev, ls);
	if (err)
		goto err_dev_set_drvdata;

	nxt_light_sensor_set_mode(ls, 0);

	dev_info(&sensor->dev, "Light sensor connected to port %s\n",
		 dev_name(&ls->in_port->dev));

	return 0;

err_dev_set_drvdata:
	unregister_msensor(&ls->ms);
err_register_msensor:
	kfree(ls);

	return err;
}

static int nxt_light_sensor_remove(struct legoev3_port_device *sensor)
{
	struct nxt_light_sensor_data *ls = dev_get_drvdata(&sensor->dev);

	dev_info(&sensor->dev, "Light sensor removed from port %s\n",
		 dev_name(&ls->in_port->dev));
	ev3_input_port_set_pin5_gpio(ls->in_port, EV3_INPUT_PORT_GPIO_FLOAT);
	ev3_input_port_register_analog_cb(ls->in_port, NULL, NULL);
	unregister_msensor(&ls->ms);
	dev_set_drvdata(&sensor->dev, NULL);
	kfree(ls);
	return 0;
}

static struct legoev3_port_device_id nxt_light_sensor_device_ids [] = {
	{
		.name = "nxt-analog-sensor",
		.type_id = NXT_LIGHT_SENSOR_TYPE_ID,
	},
	{  }
};

struct legoev3_port_driver nxt_light_sensor_driver = {
	.probe	= nxt_light_sensor_probe,
	.remove	= nxt_light_sensor_remove,
	.driver = {
		.name	= "nxt-light-sensor",
		.owner	= THIS_MODULE,
	},
	.id_table = nxt_light_sensor_device_ids,
};
EXPORT_SYMBOL_GPL(nxt_light_sensor_driver);
legoev3_port_driver(nxt_light_sensor_driver);

MODULE_DESCRIPTION("NXT Light sensor device driver for LEGO Mindstorms EV3");
MODULE_AUTHOR("David Lechner <david@lechnology.com>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("legoev3:nxt-analog-sensor");
