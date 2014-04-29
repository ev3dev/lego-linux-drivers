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

#define NUM_NXT_ANALOG_SENSOR_MODES	2

struct nxt_analog_sensor_data {
	struct legoev3_port_device *in_port;
	struct msensor_device ms;
	struct msensor_mode_info mode_info[NUM_NXT_ANALOG_SENSOR_MODES];
	u8 mode;
};

static struct msensor_mode_info
nxt_analog_sensor_mode_info[NUM_NXT_ANALOG_SENSOR_MODES] = {
	{
		.name = "NXT-ANALOG-0",
		.units = "V",
		.raw_max = 5000,
		.pct_max = 100,
		.si_max = 5000,
		.decimals = 3,
		.data_sets = 1,
		.data_type = MSENSOR_DATA_S32,
	},
	{
		.name = "NXT-ANALOG-1",
		.units = "V",
		.raw_max = 5000,
		.pct_max = 100,
		.si_max = 5000,
		.decimals = 3,
		.data_sets = 1,
		.data_type = MSENSOR_DATA_S32,
	},
};

static u8 nxt_analog_sensor_get_mode(void *context)
{
	struct nxt_analog_sensor_data *as = context;

	return as->mode;
}

static int nxt_analog_sensor_set_mode(void *context, u8 mode)
{
	struct nxt_analog_sensor_data *as = context;

	if (mode >= NUM_NXT_ANALOG_SENSOR_MODES)
		return -EINVAL;

	if (mode == 0)
		ev3_input_port_set_pin5_gpio(as->in_port,
					     EV3_INPUT_PORT_GPIO_FLOAT);
	else
		ev3_input_port_set_pin5_gpio(as->in_port,
					     EV3_INPUT_PORT_GPIO_HIGH);
	as->mode = mode;

	return 0;
}

static void nxt_analog_sensor_cb(void *context)
{
	struct nxt_analog_sensor_data *as = context;

	*(int*)as->mode_info[0].raw_data =
		ev3_input_port_get_pin1_mv(as->in_port);
}

static int nxt_analog_sensor_probe(struct legoev3_port_device *sensor)
{
	struct nxt_analog_sensor_data *as;
	struct ev3_sensor_platform_data *pdata = sensor->dev.platform_data;
	int err;

	if (WARN_ON(!pdata))
		return -EINVAL;

	as = kzalloc(sizeof(struct nxt_analog_sensor_data), GFP_KERNEL);
	if (!as)
		return -ENOMEM;

	as->in_port = pdata->in_port;

	as->ms.type_id		= sensor->type_id;
	strncpy(as->ms.port_name, dev_name(&pdata->in_port->dev),
	        MSENSOR_PORT_NAME_SIZE);
	as->ms.num_modes	= NUM_NXT_ANALOG_SENSOR_MODES;
	as->ms.num_view_modes	= 1;
	as->ms.mode_info	= as->mode_info;
	as->ms.get_mode		= nxt_analog_sensor_get_mode;
	as->ms.set_mode		= nxt_analog_sensor_set_mode;
	as->ms.context		= as;

	memcpy(as->mode_info, nxt_analog_sensor_mode_info,
	       sizeof(struct msensor_mode_info) * NUM_NXT_ANALOG_SENSOR_MODES);

	err = register_msensor(&as->ms, &sensor->dev);
	if (err)
		goto err_register_msensor;

	ev3_input_port_register_analog_cb(as->in_port, nxt_analog_sensor_cb, as);

	err = dev_set_drvdata(&sensor->dev, as);
	if (err)
		goto err_dev_set_drvdata;

	nxt_analog_sensor_set_mode(as, 0);

	dev_info(&sensor->dev, "Analog sensor connected to port %s\n",
		 dev_name(&as->in_port->dev));

	return 0;

err_dev_set_drvdata:
	unregister_msensor(&as->ms);
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

static struct legoev3_port_device_id nxt_light_sensor_device_ids [] = {
	{
		.name = "nxt-analog-sensor",
		.type_id = NXT_ANALOG_SENSOR_TYPE_ID,
	},
	{  }
};

struct legoev3_port_driver nxt_analog_sensor_driver = {
	.probe	= nxt_analog_sensor_probe,
	.remove	= nxt_analog_sensor_remove,
	.driver = {
		.name	= "nxt-analog-sensor",
		.owner	= THIS_MODULE,
	},
	.id_table = nxt_light_sensor_device_ids,
};
EXPORT_SYMBOL_GPL(nxt_analog_sensor_driver);
legoev3_port_driver(nxt_analog_sensor_driver);

MODULE_DESCRIPTION("NXT Analog sensor device driver for LEGO Mindstorms EV3");
MODULE_AUTHOR("David Lechner <david@lechnology.com>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("legoev3:nxt-analog-sensor");
