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

static ssize_t measure_sensor_show_raw_value(struct device *dev,
					     struct device_attribute *attr,
					     char *buf)
{
	struct measure_sensor_device *ms =
		container_of(dev, struct measure_sensor_device, dev);

	return sprintf(buf, "%d\n", ms->raw_value(ms));
}

static ssize_t measure_sensor_show_raw_min(struct device *dev,
					   struct device_attribute *attr,
					   char *buf)
{
	struct measure_sensor_device *ms =
		container_of(dev, struct measure_sensor_device, dev);

	return sprintf(buf, "%d\n", ms->raw_min);
}

static ssize_t measure_sensor_show_raw_max(struct device *dev,
					   struct device_attribute *attr,
					   char *buf)
{
	struct measure_sensor_device *ms =
		container_of(dev, struct measure_sensor_device, dev);

	return sprintf(buf, "%d\n", ms->raw_max);
}

static ssize_t measure_sensor_show_cal_value(struct device *dev,
					     struct device_attribute *attr,
					     char *buf)
{
	struct measure_sensor_device *ms =
		container_of(dev, struct measure_sensor_device, dev);

	return sprintf(buf, "%d\n", ms->cal_value(ms));
}

static ssize_t measure_sensor_show_cal_min(struct device *dev,
					   struct device_attribute *attr,
					   char *buf)
{
	struct measure_sensor_device *ms =
		container_of(dev, struct measure_sensor_device, dev);

	return sprintf(buf, "%d\n", ms->cal_min);
}

static ssize_t measure_sensor_show_cal_max(struct device *dev,
					   struct device_attribute *attr,
					   char *buf)
{
	struct measure_sensor_device *ms =
		container_of(dev, struct measure_sensor_device, dev);

	return sprintf(buf, "%d\n", ms->cal_max);
}

static ssize_t measure_sensor_show_scaled_value(struct device *dev,
						struct device_attribute *attr,
						char *buf)
{
	struct measure_sensor_device *ms =
		container_of(dev, struct measure_sensor_device, dev);
	int value;

	value = ms->cal_value(ms) * (ms->scale_info[ms->scale_idx].max
		- ms->scale_info[ms->scale_idx].min)
		/ (ms->cal_max - ms->cal_min);
	return sprintf(buf, "%d\n",value);
}

static ssize_t measure_sensor_show_scaled_min(struct device *dev,
					      struct device_attribute *attr,
					      char *buf)
{
	struct measure_sensor_device *ms =
		container_of(dev, struct measure_sensor_device, dev);

	return sprintf(buf, "%d\n", ms->scale_info[ms->scale_idx].min);
}

static ssize_t measure_sensor_show_scaled_max(struct device *dev,
					      struct device_attribute *attr,
					      char *buf)
{
	struct measure_sensor_device *ms =
		container_of(dev, struct measure_sensor_device, dev);

	return sprintf(buf, "%d\n", ms->scale_info[ms->scale_idx].max);
}

static ssize_t measure_sensor_show_scaled_dp(struct device *dev,
					     struct device_attribute *attr,
					     char *buf)
{
	struct measure_sensor_device *ms =
		container_of(dev, struct measure_sensor_device, dev);

	return sprintf(buf, "%d\n", ms->scale_info[ms->scale_idx].dp);
}

static ssize_t measure_sensor_show_scaled_units(struct device *dev,
						struct device_attribute *attr,
						char *buf)
{
	struct measure_sensor_device *ms =
		container_of(dev, struct measure_sensor_device, dev);
	int idx = 0;
	ssize_t count = 0;

	while (ms->scale_info[idx].units) {
		if (idx == ms->scale_idx)
			count += sprintf(buf + count, "[");
		count += sprintf(buf + count, "%s", ms->scale_info[idx].units);
		if (idx == ms->scale_idx)
			count += sprintf(buf + count, "]");
		idx++;
		count += sprintf(buf + count, "%c",
				 ms->scale_info[idx].units ? ' ' : '\n');
	}

	return count;
}

static ssize_t measure_sensor_store_scaled_units(struct device *dev,
						 struct device_attribute *attr,
						 const char *buf, size_t count)
{
	struct measure_sensor_device *ms =
		container_of(dev, struct measure_sensor_device, dev);
	int idx = 0;

	while (ms->scale_info[idx].units) {
		if (sysfs_streq(buf, ms->scale_info[idx].units)) {
			ms->scale_idx = idx;
			return count;
		}
		idx++;
	}
	return -EINVAL;
}

static struct device_attribute measure_sensor_class_dev_attrs[] = {
	__ATTR(raw_value, S_IRUGO, measure_sensor_show_raw_value, NULL),
	__ATTR(raw_min, S_IRUGO, measure_sensor_show_raw_min, NULL),
	__ATTR(raw_max, S_IRUGO, measure_sensor_show_raw_max, NULL),
	__ATTR(cal_value, S_IRUGO, measure_sensor_show_cal_value, NULL),
	__ATTR(cal_min, S_IRUGO, measure_sensor_show_cal_min, NULL),
	__ATTR(cal_max, S_IRUGO, measure_sensor_show_cal_max, NULL),
	__ATTR(scaled_value, S_IRUGO, measure_sensor_show_scaled_value, NULL),
	__ATTR(scaled_min, S_IRUGO, measure_sensor_show_scaled_min, NULL),
	__ATTR(scaled_max, S_IRUGO, measure_sensor_show_scaled_max, NULL),
	__ATTR(scaled_dp, S_IRUGO, measure_sensor_show_scaled_dp, NULL),
	__ATTR(scaled_units, S_IWUGO | S_IRUGO, measure_sensor_show_scaled_units,
	       measure_sensor_store_scaled_units),
	__ATTR_NULL
};

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
	if (ms->id < 0)
		dev_set_name(&ms->dev, "%s:%s", dev_name(parent), ms->name);
	else
		dev_set_name(&ms->dev, "%s:%s%d", dev_name(parent), ms->name, ms->id);

	return device_register(&ms->dev);
}
EXPORT_SYMBOL_GPL(register_measure_sensor);

void unregister_measure_sensor(struct measure_sensor_device *ms)
{
	device_unregister(&ms->dev);
}
EXPORT_SYMBOL_GPL(unregister_measure_sensor);

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
