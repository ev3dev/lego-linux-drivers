/*
 * EV3 Color sensor device driver for LEGO Mindstorms EV3
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

#include <linux/device.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/tty.h>
#include <linux/legoev3/legoev3_uart.h>
#include <linux/legoev3/sensor_controls_class.h>

struct ev3_color_sensor_data {
	struct sensor_controls_device ctrl;
	struct tty_struct *tty;
};

static struct sensor_controls_mode_info ev3_color_sensor_modes[] = {
	{
		.name	= "color",
		.id	= 0,
	},
	{
		.name	= "ambient-light",
		.id	= 0,
	},
	{
		.name	= "reflected-light",
		.id	= 0,
	},
	END_MODE_INFO
};

static int __devinit ev3_color_sensor_probe(struct legoev3_port_device *sensor)
{
	struct ev3_color_sensor_data *ev3_cs;
	struct msensor_device *pdata = sensor->dev.platform_data;
	int err;

	if (WARN_ON(!pdata))
		return -EINVAL;

	ev3_cs = kzalloc(sizeof(struct ev3_color_sensor_data), GFP_KERNEL);
	if (!ev3_cs)
		return -ENOMEM;

	ev3_cs->tty = pdata->context;
	ev3_cs->ctrl.name	= "color";
	ev3_cs->ctrl.id		= -1; /* TODO do we need to make this unique or get rid of id? */
	ev3_cs->ctrl.get_mode	= ev3_color_sensor_get_mode;
	ev3_cs->ctrl.set_mode	= ev3_color_sensor_set_mode;
	ev3_cs->ctrl.mode_info	= ev3_color_sensor_modes;
	ev3_cs->ctrl.mode_idx	= 0;

	err = register_switch_sensor(&ev3_cs->ts, &sensor->dev);
	if (err)
		goto register_color_sensor_fail;

	err = dev_set_drvdata(&sensor->dev, ev3_ts);
	if (err)
		goto dev_set_drvdata_fail;

	dev_info(&sensor->dev, "EV3 Touch sensor connected to port %s\n",
		 dev_name(&ev3_cs->in_port->dev));

	return 0;

dev_set_drvdata_fail:
	unregister_switch_sensor(&ev3_cs->ts);
register_color_sensor_fail:
	kfree(ev3_cs);

	return err;
}

static int __devexit ev3_color_sensor_remove(struct legoev3_port_device *sensor)
{
	struct ev3_color_sensor_data *ev3_ts = dev_get_drvdata(&sensor->dev);

	dev_info(&sensor->dev, "EV3 Touch sensor removed from port %s\n",
		 dev_name(&ev3_cs->in_port->dev));
	unregister_switch_sensor(&ev3_cs->ts);
	dev_set_drvdata(&sensor->dev, NULL);
	kfree(ev3_cs);
	return 0;
}

struct legoev3_port_driver ev3_color_sensor_driver = {
	.probe	= ev3_color_sensor_probe,
	.remove	= __devexit_p(ev3_color_sensor_remove),
	.driver = {
		.name	= "ev3-color-sensor",
		.owner	= THIS_MODULE,
	},
};
EXPORT_SYMBOL_GPL(ev3_color_sensor_driver);
legoev3_port_driver(ev3_color_sensor_driver);

MODULE_DESCRIPTION("EV3 color sensor device driver for LEGO Mindstorms EV3");
MODULE_AUTHOR("David Lechner <david@lechnology.com>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("legoev3:ev3-uart-sensor-type-29");
