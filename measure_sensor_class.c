/*
 * Measurement sensor device class for LEGO Mindstorms EV3
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
#include <linux/legoev3/measure_sensor_class.h>

static void measure_sensor_release(struct device *dev)
{
}

int register_measure_sensor(struct measure_sensor_device *ms,
			    struct device *parent)
{
	if (!ms)
		return -EINVAL;

	ms->dev.release = measure_sensor_release;
	ms->dev.parent = parent;
	ms->dev.class = &measure_sensor_class;
	dev_set_name(&ms->dev, "%s:measure", dev_name(parent));

	return device_register(&ms->dev);
}
EXPORT_SYMBOL_GPL(register_measure_sensor);

void unregister_measure_sensor(struct measure_sensor_device *ms)
{
	device_unregister(&ms->dev);
}
EXPORT_SYMBOL_GPL(unregister_measure_sensor);

static ssize_t measure_sensor_show_raw_value(struct device *dev,
					     struct device_attribute *attr,
					     char *buf)
{
	struct measure_sensor_device *ms =
		container_of(dev, struct measure_sensor_device, dev);

	return sprintf(buf, "%d\n", ms->raw_value(ms));
}

static struct device_attribute measure_sensor_class_dev_attrs[] = {
	__ATTR(raw_value, S_IRUGO, measure_sensor_show_raw_value, NULL),
	__ATTR_NULL
};

static char *measure_sensor_devnode(struct device *dev, umode_t *mode)
{
	return kasprintf(GFP_KERNEL, "measurement-sensor/%s", dev_name(dev->parent));
}

struct class measure_sensor_class = {
	.name		= "measure-sensor",
	.owner		= THIS_MODULE,
	.dev_attrs	= measure_sensor_class_dev_attrs,
	.devnode	= measure_sensor_devnode,
};
EXPORT_SYMBOL_GPL(measure_sensor_class);

static int __init measure_sensor_class_init(void)
{
	int err;

	err = class_register(&measure_sensor_class);
	if (err) {
		pr_err("unable to register measure_sensor_class\n");
		return err;
	}

	return 0;
}
module_init(measure_sensor_class_init);

static void __exit measure_sensor_class_exit(void)
{
	class_unregister(&measure_sensor_class);
}
module_exit(measure_sensor_class_exit);

MODULE_DESCRIPTION("Measurement sensor device class for LEGO Mindstorms EV3");
MODULE_AUTHOR("David Lechner <david@lechnology.com>");
MODULE_LICENSE("GPL");
