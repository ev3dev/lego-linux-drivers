/*
 * Touch sensor device class for LEGO Mindstorms EV3
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
#include <linux/legoev3/legoev3_touch_sensor_class.h>

static void legoev3_touch_sensor_release(struct device *dev)
{
}

int register_legoev3_touch_sensor(struct legoev3_touch_sensor_device *ts,
				  struct device *parent)
{
	if (!ts)
		return -EINVAL;

	ts->dev.release = legoev3_touch_sensor_release;
	ts->dev.parent = parent;
	ts->dev.class = &legoev3_touch_sensor_class;
	dev_set_name(&ts->dev, "%s:touch", dev_name(parent));

	return device_register(&ts->dev);
}
EXPORT_SYMBOL_GPL(register_legoev3_touch_sensor);

void unregister_legoev3_touch_sensor(struct legoev3_touch_sensor_device *ts)
{
	device_unregister(&ts->dev);
}
EXPORT_SYMBOL_GPL(unregister_legoev3_touch_sensor);

static ssize_t legoev3_touch_sensor_show_pressed(struct device *dev,
						 struct device_attribute *attr,
						 char *buf)
{
	struct legoev3_touch_sensor_device *ts =
		container_of(dev, struct legoev3_touch_sensor_device, dev);

	return sprintf(buf, "%d\n", ts->pressed(ts));
}

static struct device_attribute legoev3_touch_sensor_class_dev_attrs[] = {
	__ATTR(pressed, S_IRUGO, legoev3_touch_sensor_show_pressed, NULL),
	__ATTR_NULL
};

static char *legoev3_touch_sensor_devnode(struct device *dev, umode_t *mode)
{
	return kasprintf(GFP_KERNEL, "legoev3/sensor/touch/%s", dev_name(dev));
}

struct class legoev3_touch_sensor_class = {
	.name		= "legoev3-touch-sensor",
	.owner		= THIS_MODULE,
	.dev_attrs	= legoev3_touch_sensor_class_dev_attrs,
	.devnode	= legoev3_touch_sensor_devnode,
};
EXPORT_SYMBOL_GPL(legoev3_touch_sensor_class);

static int __init legoev3_touch_sensor_class_init(void)
{
	int err;

	err = class_register(&legoev3_touch_sensor_class);
	if (err) {
		pr_err("unable to register legoev3_touch_sensor_class\n");
		return err;
	}

	return 0;
}
module_init(legoev3_touch_sensor_class_init);

static void __exit legoev3_touch_sensor_class_exit(void)
{
	class_unregister(&legoev3_touch_sensor_class);
}
module_exit(legoev3_touch_sensor_class_exit);

MODULE_DESCRIPTION("Touch sensor device class for LEGO Mindstorms EV3");
MODULE_AUTHOR("David Lechner <david@lechnology.com>");
MODULE_LICENSE("GPL");
