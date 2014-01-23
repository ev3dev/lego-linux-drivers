/*
 * EV3 Touch sensor device driver for LEGO Mindstorms EV3
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
#include <linux/legoev3/switch_sensor_class.h>

#include <asm/bug.h>

#define PIN6_NEAR_GND		250		/* 0.25V */

struct ev3_touch_sensor_data {
	struct switch_sensor_device ts;
	struct legoev3_port_device *in_port;
};

static bool ev3_touch_sensor_pressed(struct switch_sensor_device *ts)
{
	struct ev3_touch_sensor_data *ev3_ts =
			container_of(ts, struct ev3_touch_sensor_data, ts);

	return ev3_input_port_get_pin6_mv(ev3_ts->in_port) > PIN6_NEAR_GND;
}

static int __devinit ev3_touch_sensor_probe(struct legoev3_port_device *sensor)
{
	struct ev3_touch_sensor_data *ev3_ts;
	struct ev3_sensor_platform_data *pdata = sensor->dev.platform_data;
	int err;

	if (WARN_ON(!pdata))
		return -EINVAL;

	ev3_ts = kzalloc(sizeof(struct ev3_touch_sensor_data), GFP_KERNEL);
	if (!ev3_ts)
		return -ENOMEM;

	ev3_ts->ts.get_value = ev3_touch_sensor_pressed;
	ev3_ts->in_port = pdata->in_port;

	err = register_switch_sensor(&ev3_ts->ts, &sensor->dev);
	if (err)
		goto register_touch_sensor_fail;

	err = dev_set_drvdata(&sensor->dev, ev3_ts);
	if (err)
		goto dev_set_drvdata_fail;

	dev_info(&sensor->dev, "EV3 Touch sensor connected to port %s\n",
		 dev_name(&ev3_ts->in_port->dev));

	return 0;

dev_set_drvdata_fail:
	unregister_switch_sensor(&ev3_ts->ts);
register_touch_sensor_fail:
	kfree(ev3_ts);

	return err;
}

static int __devexit ev3_touch_sensor_remove(struct legoev3_port_device *sensor)
{
	struct ev3_touch_sensor_data *ev3_ts = dev_get_drvdata(&sensor->dev);

	dev_info(&sensor->dev, "EV3 Touch sensor removed from port %s\n",
		 dev_name(&ev3_ts->in_port->dev));
	unregister_switch_sensor(&ev3_ts->ts);
	dev_set_drvdata(&sensor->dev, NULL);
	kfree(ev3_ts);
	return 0;
}

struct legoev3_port_driver ev3_touch_sensor_driver = {
	.probe	= ev3_touch_sensor_probe,
	.remove	= __devexit_p(ev3_touch_sensor_remove),
	.driver = {
		.name	= "ev3-touch-sensor",
		.owner	= THIS_MODULE,
	},
};
EXPORT_SYMBOL_GPL(ev3_touch_sensor_driver);
legoev3_port_driver(ev3_touch_sensor_driver);

MODULE_DESCRIPTION("EV3 touch sensor device driver for LEGO Mindstorms EV3");
MODULE_AUTHOR("David Lechner <david@lechnology.com>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("legoev3:ev3-touch-sensor");
