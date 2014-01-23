/*
 * NXT Touch sensor device driver for LEGO Mindstorms EV3
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

#define PIN1_NEAR_5V		4800		/* 4.80V */

struct nxt_touch_sensor_data {
	struct switch_sensor_device ts;
	struct legoev3_port_device *in_port;
};

static bool nxt_touch_sensor_pressed(struct switch_sensor_device *ts)
{
	struct nxt_touch_sensor_data *nxt_ts =
			container_of(ts, struct nxt_touch_sensor_data, ts);

	return ev3_input_port_get_pin1_mv(nxt_ts->in_port) < PIN1_NEAR_5V;
}

static int __devinit nxt_touch_sensor_probe(struct legoev3_port_device *sensor)
{
	struct nxt_touch_sensor_data *nxt_ts;
	struct ev3_sensor_platform_data *pdata = sensor->dev.platform_data;
	int err;

	if (WARN_ON(!pdata))
		return -EINVAL;

	nxt_ts = kzalloc(sizeof(struct nxt_touch_sensor_data), GFP_KERNEL);
	if (!nxt_ts)
		return -ENOMEM;

	nxt_ts->ts.get_value = nxt_touch_sensor_pressed;
	nxt_ts->in_port = pdata->in_port;

	err = register_switch_sensor(&nxt_ts->ts, &sensor->dev);
	if (err)
		goto register_touch_sensor_fail;

	err = dev_set_drvdata(&sensor->dev, nxt_ts);
	if (err)
		goto dev_set_drvdata_fail;

	dev_info(&sensor->dev, "NXT Touch sensor connected to port %s\n",
		 dev_name(&nxt_ts->in_port->dev));

	return 0;

dev_set_drvdata_fail:
	unregister_switch_sensor(&nxt_ts->ts);
register_touch_sensor_fail:
	kfree(nxt_ts);

	return err;
}

static int __devexit nxt_touch_sensor_remove(struct legoev3_port_device *sensor)
{
	struct nxt_touch_sensor_data *nxt_ts = dev_get_drvdata(&sensor->dev);

	dev_info(&sensor->dev, "NXT Touch sensor removed from port %s\n",
		 dev_name(&nxt_ts->in_port->dev));
	unregister_switch_sensor(&nxt_ts->ts);
	dev_set_drvdata(&sensor->dev, NULL);
	kfree(nxt_ts);
	return 0;
}

struct legoev3_port_driver nxt_touch_sensor_driver = {
	.probe	= nxt_touch_sensor_probe,
	.remove	= __devexit_p(nxt_touch_sensor_remove),
	.driver = {
		.name	= "nxt-touch-sensor",
		.owner	= THIS_MODULE,
	},
};
EXPORT_SYMBOL_GPL(nxt_touch_sensor_driver);
legoev3_port_driver(nxt_touch_sensor_driver);

MODULE_DESCRIPTION("NXT touch sensor device driver for LEGO Mindstorms EV3");
MODULE_AUTHOR("David Lechner <david@lechnology.com>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("legoev3:nxt-touch-sensor");
