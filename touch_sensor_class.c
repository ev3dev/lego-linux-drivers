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
#include <linux/legoev3/touch_sensor_class.h>

static ssize_t touch_sensor_show_pressed(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
	struct touch_sensor_device *ts =
		container_of(dev, struct touch_sensor_device, dev);

	return sprintf(buf, "%d\n", ts->pressed(ts));
}

static struct device_attribute touch_sensor_class_dev_attrs[] = {
	__ATTR(pressed, S_IRUGO, touch_sensor_show_pressed, NULL),
	__ATTR_NULL
};

static void touch_sensor_release(struct device *dev)
{
}

int register_touch_sensor(struct touch_sensor_device *ts, struct device *parent)
{
	if (!ts)
		return -EINVAL;

	ts->dev.release = touch_sensor_release;
	ts->dev.parent = parent;
	ts->dev.class = &touch_sensor_class;
	dev_set_name(&ts->dev, "%s:touch", dev_name(parent));

	return device_register(&ts->dev);
}
EXPORT_SYMBOL_GPL(register_touch_sensor);

void unregister_touch_sensor(struct touch_sensor_device *ts)
{
	device_unregister(&ts->dev);
}
EXPORT_SYMBOL_GPL(unregister_touch_sensor);

static char *touch_sensor_devnode(struct device *dev, umode_t *mode)
{
	return kasprintf(GFP_KERNEL, "touch-sensor/%s", dev_name(dev));
}

struct class touch_sensor_class = {
	.name		= "touch-sensor",
	.owner		= THIS_MODULE,
	.dev_attrs	= touch_sensor_class_dev_attrs,
	.devnode	= touch_sensor_devnode,
};
EXPORT_SYMBOL_GPL(touch_sensor_class);

static int __init touch_sensor_class_init(void)
{
	int err;

	err = class_register(&touch_sensor_class);
	if (err) {
		pr_err("unable to register touch_sensor_class\n");
		return err;
	}

	return 0;
}
module_init(touch_sensor_class_init);

static void __exit touch_sensor_class_exit(void)
{
	class_unregister(&touch_sensor_class);
}
module_exit(touch_sensor_class_exit);

MODULE_DESCRIPTION("Touch sensor device class for LEGO Mindstorms EV3");
MODULE_AUTHOR("David Lechner <david@lechnology.com>");
MODULE_LICENSE("GPL");
