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
#include <linux/legoev3/switch_sensor_class.h>

static ssize_t switch_sensor_show_value(struct device *dev,
                                        struct device_attribute *attr,
                                        char *buf)
{
	struct switch_sensor_device *ts =
		container_of(dev, struct switch_sensor_device, dev);

	return sprintf(buf, "%d\n", ts->get_value(ts));
}

static struct device_attribute touch_sensor_class_dev_attrs[] = {
	__ATTR(pressed, S_IRUGO, switch_sensor_show_value, NULL),
	__ATTR_NULL
};

static void touch_sensor_release(struct device *dev)
{
}

int register_switch_sensor(struct switch_sensor_device *ts, struct device *parent)
{
	if (!ts)
		return -EINVAL;

	ts->dev.release = touch_sensor_release;
	ts->dev.parent = parent;
	ts->dev.class = &switch_sensor_class;
	dev_set_name(&ts->dev, "%s:switch", dev_name(parent));

	return device_register(&ts->dev);
}
EXPORT_SYMBOL_GPL(register_switch_sensor);

void unregister_switch_sensor(struct switch_sensor_device *ts)
{
	device_unregister(&ts->dev);
}
EXPORT_SYMBOL_GPL(unregister_switch_sensor);

static char *touch_sensor_devnode(struct device *dev, umode_t *mode)
{
	return kasprintf(GFP_KERNEL, "switch-sensor/%s", dev_name(dev));
}

struct class switch_sensor_class = {
	.name		= "switch-sensor",
	.owner		= THIS_MODULE,
	.dev_attrs	= touch_sensor_class_dev_attrs,
	.devnode	= touch_sensor_devnode,
};
EXPORT_SYMBOL_GPL(switch_sensor_class);

static int __init switch_sensor_class_init(void)
{
	int err;

	err = class_register(&switch_sensor_class);
	if (err) {
		pr_err("unable to register switch_sensor_class\n");
		return err;
	}

	return 0;
}
module_init(switch_sensor_class_init);

static void __exit switch_sensor_class_exit(void)
{
	class_unregister(&switch_sensor_class);
}
module_exit(switch_sensor_class_exit);

MODULE_DESCRIPTION("Switch sensor device class for LEGO Mindstorms EV3");
MODULE_AUTHOR("David Lechner <david@lechnology.com>");
MODULE_LICENSE("GPL");
