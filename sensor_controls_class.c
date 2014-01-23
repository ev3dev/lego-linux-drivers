/*
 * Sensor controls device class for LEGO Mindstorms EV3
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
#include <linux/legoev3/sensor_controls_class.h>

static ssize_t sensor_controls_show_mode(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
	struct sensor_controls_device *ctrl =
		container_of(dev, struct sensor_controls_device, dev);
	int i = 0;
	ssize_t count = 0;
	int mode;

	if (!ctrl->get_mode)
		return -ENXIO;
	mode = ctrl->get_mode(ctrl);
	if (mode < 0)
		return mode;
	while (ctrl->mode_info[i].name) {
		if (ctrl->mode_info[i].id == mode)
			count += sprintf(buf + count, "[");
		count += sprintf(buf + count, "%s", ctrl->mode_info[i].name);
		if (ctrl->mode_info[i].id == mode)
			count += sprintf(buf + count, "]");
		i++;
		count += sprintf(buf + count, "%c",
				 ctrl->mode_info[i].name ? ' ' : '\n');
	}

	return count;
}

static ssize_t sensor_controls_store_mode(struct device *dev,
					  struct device_attribute *attr,
					  const char *buf, size_t count)
{
	struct sensor_controls_device *ctrl =
			container_of(dev, struct sensor_controls_device, dev);
	int i = 0;
	int err;

	if (!ctrl->set_mode)
		return -ENXIO;
	while (ctrl->mode_info[i].name) {
		if (sysfs_streq(buf, ctrl->mode_info[i].name)) {
			err = ctrl->set_mode(ctrl, ctrl->mode_info[i].id);
			if (err)
				return err;
			return count;
		}
		i++;
	}
	return -EINVAL;
}

static struct device_attribute sensor_controls_class_dev_attrs[] = {
	__ATTR(mode, S_IWUGO | S_IRUGO, sensor_controls_show_mode, sensor_controls_store_mode),
	__ATTR_NULL
};

static void sensor_controls_release(struct device *dev)
{
}

int register_sensor_controls(struct sensor_controls_device *ms,
			    struct device *parent)
{
	if (!ms)
		return -EINVAL;

	ms->dev.release = sensor_controls_release;
	ms->dev.parent = parent;
	ms->dev.class = &sensor_controls_class;
	if (ms->id < 0)
		dev_set_name(&ms->dev, "%s:%s", dev_name(parent), ms->name);
	else
		dev_set_name(&ms->dev, "%s:%s%d", dev_name(parent), ms->name, ms->id);

	return device_register(&ms->dev);
}
EXPORT_SYMBOL_GPL(register_sensor_controls);

void unregister_sensor_controls(struct sensor_controls_device *ms)
{
	device_unregister(&ms->dev);
}
EXPORT_SYMBOL_GPL(unregister_sensor_controls);

static char *sensor_controls_devnode(struct device *dev, umode_t *mode)
{
	return kasprintf(GFP_KERNEL, "sensor-controls/%s", dev_name(dev->parent));
}

struct class sensor_controls_class = {
	.name		= "sensor-controls",
	.owner		= THIS_MODULE,
	.dev_attrs	= sensor_controls_class_dev_attrs,
	.devnode	= sensor_controls_devnode,
};
EXPORT_SYMBOL_GPL(sensor_controls_class);

static int __init sensor_controls_class_init(void)
{
	int err;

	err = class_register(&sensor_controls_class);
	if (err) {
		pr_err("unable to register sensor_controls_class\n");
		return err;
	}

	return 0;
}
module_init(sensor_controls_class_init);

static void __exit sensor_controls_class_exit(void)
{
	class_unregister(&sensor_controls_class);
}
module_exit(sensor_controls_class_exit);

MODULE_DESCRIPTION("Sensor controls device class for LEGO Mindstorms EV3");
MODULE_AUTHOR("David Lechner <david@lechnology.com>");
MODULE_LICENSE("GPL");

