/*
 * EV3 Touch sensor device driver for LEGO Mindstorms EV3
 *
 * Copyright (C) 2013 David Lechner <david@lechnology.com>
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
#include <linux/legoev3/legoev3_touch_sensor_class.h>
#include <linux/legoev3/legoev3_input_port.h>
#include <linux/legoev3/legoev3_ports.h>

#define PIN6_NEAR_GND		250		/* 0.25V */

struct legoev3_ev3_touch_sensor {
	struct legoev3_touch_sensor_device ts;
};

static bool legoev3_ev3_touch_sensor_pressed(struct legoev3_touch_sensor_device *ts)
{
	struct legoev3_input_port_device *ipd =
		container_of(ts->dev.parent, struct legoev3_input_port_device, dev);

	return (ipd->pin6_mv(ipd) > PIN6_NEAR_GND);
}

static int __devinit legoev3_ev3_touch_sensor_probe(struct device *dev)
{
	struct legoev3_ev3_touch_sensor *ev3_ts;
	int err;

	ev3_ts = kzalloc(sizeof(struct legoev3_ev3_touch_sensor), GFP_KERNEL);
	if (!ev3_ts)
		return -ENOMEM;

	ev3_ts->ts.pressed = legoev3_ev3_touch_sensor_pressed;

	err = register_legoev3_touch_sensor(&ev3_ts->ts, dev);
	if (err)
		goto register_legoev3_touch_sensor_fail;

	err = dev_set_drvdata(dev, ev3_ts);
	if (err)
		goto dev_set_drvdata_fail;

	return 0;

dev_set_drvdata_fail:
	unregister_legoev3_touch_sensor(&ev3_ts->ts);
register_legoev3_touch_sensor_fail:
	kfree(ev3_ts);

	return err;
}

static int __devexit legoev3_ev3_touch_sensor_remove(struct device *dev)
{
	struct legoev3_ev3_touch_sensor *ev3_ts = dev_get_drvdata(dev);

	unregister_legoev3_touch_sensor(&ev3_ts->ts);
	dev_set_drvdata(dev, NULL);
	kfree(ev3_ts);
	return 0;
}

struct legoev3_port_driver legoev3_ev3_touch_sensor_driver = {
	.driver = {
		.name	= "legoev3-ev3-sensor-02",
		.owner	= THIS_MODULE,
		.probe	= legoev3_ev3_touch_sensor_probe,
		.remove	= __devexit_p(legoev3_ev3_touch_sensor_remove),
	},
};
EXPORT_SYMBOL_GPL(legoev3_ev3_touch_sensor_driver);
legoev3_port_driver(legoev3_ev3_touch_sensor_driver);

MODULE_DESCRIPTION("EV3 touch sensor device driver for LEGO Mindstorms EV3");
MODULE_AUTHOR("David Lechner <david@lechnology.com>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("legoev3:legoev3-ev3-sensor-02");
