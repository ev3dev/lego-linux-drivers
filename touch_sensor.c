/*
 * Touch sensor device driver for LEGO Mindstorms EV3
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

#define NUM_TOUCH_SENSOR_MODES	1
#define PIN1_NEAR_5V		4800		/* 4.80V */
#define PIN6_NEAR_GND		250		/* 0.25V */

struct touch_sensor_data {
	struct legoev3_port_device *in_port;
	struct msensor_device ms;
	struct msensor_mode_info mode_info[NUM_TOUCH_SENSOR_MODES];
	u8 mode;
};

static struct msensor_mode_info touch_sensor_mode_info[NUM_TOUCH_SENSOR_MODES] = {
	{
		.name = "TOUCH",
		.raw_max = 1,
		.pct_max = 100,
		.si_max = 1,
		.data_sets = 1,
	}
};

static u8 touch_sensor_get_mode(void *context)
{
	struct touch_sensor_data *ts = context;

	return ts->mode;
}

static int touch_sensor_set_mode(void *context, u8 mode)
{
	struct touch_sensor_data *ts = context;

	if (mode >= NUM_TOUCH_SENSOR_MODES)
		return -EINVAL;

	ts->mode = mode;

	return 0;
}

static void touch_sensor_nxt_cb(void *context)
{
	struct touch_sensor_data *ts = context;

	ts->mode_info[0].raw_data[0] =
		ev3_input_port_get_pin1_mv(ts->in_port) < PIN1_NEAR_5V ? 1 : 0;
}

static void touch_sensor_ev3_cb(void *context)
{
	struct touch_sensor_data *ts = context;

	ts->mode_info[0].raw_data[0] =
		ev3_input_port_get_pin6_mv(ts->in_port) > PIN6_NEAR_GND ? 1 : 0;
}

static int touch_sensor_probe(struct legoev3_port_device *sensor)
{
	struct touch_sensor_data *ts;
	struct ev3_sensor_platform_data *pdata = sensor->dev.platform_data;
	int err;

	if (WARN_ON(!pdata))
		return -EINVAL;

	ts = kzalloc(sizeof(struct touch_sensor_data), GFP_KERNEL);
	if (!ts)
		return -ENOMEM;

	ts->in_port = pdata->in_port;

	ts->ms.type_id		= sensor->type_id;
	strncpy(ts->ms.port_name, dev_name(&pdata->in_port->dev),
	        MSENSOR_PORT_NAME_SIZE);
	ts->ms.num_modes	= NUM_TOUCH_SENSOR_MODES;
	ts->ms.num_view_modes	= 1;
	ts->ms.mode_info	= ts->mode_info;
	ts->ms.get_mode		= touch_sensor_get_mode;
	ts->ms.set_mode		= touch_sensor_set_mode;
	ts->ms.context		= ts;

	memcpy(ts->mode_info, touch_sensor_mode_info,
	       sizeof(struct msensor_mode_info) * NUM_TOUCH_SENSOR_MODES);

	err = register_msensor(&ts->ms, &sensor->dev);
	if (err)
		goto err_register_msensor;

	if (sensor->type_id == NXT_TOUCH_SENSOR_TYPE_ID)
		ev3_input_port_register_analog_cb(ts->in_port,
						  touch_sensor_nxt_cb, ts);
	else if (sensor->type_id == EV3_TOUCH_SENSOR_TYPE_ID)
		ev3_input_port_register_analog_cb(ts->in_port,
						  touch_sensor_ev3_cb, ts);
	else {
		dev_err(&sensor->dev, "Could not register unknown type id %d.\n",
			sensor->type_id);
		err = -EINVAL;
		goto err_unknown_type_id;
	}

	err = dev_set_drvdata(&sensor->dev, ts);
	if (err)
		goto err_dev_set_drvdata;

	dev_info(&sensor->dev, "Touch sensor connected to port %s\n",
		 dev_name(&ts->in_port->dev));

	return 0;

err_dev_set_drvdata:
err_unknown_type_id:
	unregister_msensor(&ts->ms);
err_register_msensor:
	kfree(ts);

	return err;
}

static int touch_sensor_remove(struct legoev3_port_device *sensor)
{
	struct touch_sensor_data *ts = dev_get_drvdata(&sensor->dev);

	dev_info(&sensor->dev, "Touch sensor removed from port %s\n",
		 dev_name(&ts->in_port->dev));
	ev3_input_port_register_analog_cb(ts->in_port, NULL, NULL);
	unregister_msensor(&ts->ms);
	dev_set_drvdata(&sensor->dev, NULL);
	kfree(ts);
	return 0;
}

static struct legoev3_port_device_id touch_sensor_device_ids [] = {
	{
		.name = "nxt-analog-sensor",
		.type_id = NXT_TOUCH_SENSOR_TYPE_ID,
	},
	{
		.name = "ev3-analog-sensor",
		.type_id = EV3_TOUCH_SENSOR_TYPE_ID,
	},
	{  }
};

struct legoev3_port_driver touch_sensor_driver = {
	.probe	= touch_sensor_probe,
	.remove	= touch_sensor_remove,
	.driver = {
		.name	= "touch-sensor",
		.owner	= THIS_MODULE,
	},
	.id_table = touch_sensor_device_ids,
};
EXPORT_SYMBOL_GPL(touch_sensor_driver);
legoev3_port_driver(touch_sensor_driver);

MODULE_DESCRIPTION("Touch sensor device driver for LEGO Mindstorms EV3");
MODULE_AUTHOR("David Lechner <david@lechnology.com>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("legoev3:nxt-analog-sensor");
MODULE_ALIAS("legoev3:ev3-analog-sensor");
