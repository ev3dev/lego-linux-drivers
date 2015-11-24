/*
 * User-defined LEGO devices - Sensor driver
 *
 * Copyright (C) 2015 David Lechner <david@lechnology.com>
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

#include "user_lego_sensor.h"

#define USER_LEGO_SENSOR_NAME "user-lego-sensor"

#define to_user_lego_sensor_device(_dev) \
	container_of(_dev, struct user_lego_sensor_device, dev)

static ssize_t port_name_show(struct device *dev, struct device_attribute *attr,
			      char *buf)
{
	struct user_lego_sensor_device *sensor = to_user_lego_sensor_device(dev);

	return snprintf(buf, LEGO_PORT_NAME_SIZE, "%s\n", sensor->sensor.port_name);
}

static DEVICE_ATTR_RO(port_name);

static struct attribute *user_lego_sensor_class_attrs[] = {
	&dev_attr_port_name.attr,
	NULL
};

static ssize_t bin_data_read(struct file *file, struct kobject *kobj,
			     struct bin_attribute *attr,
			     char *buf, loff_t off, size_t count)
{
	struct device *dev = container_of(kobj, struct device, kobj);
	struct user_lego_sensor_device *sensor = to_user_lego_sensor_device(dev);
	size_t size = attr->size;

	if (off >= size || !count)
		return 0;
	size -= off;
	if (count < size)
		size = count;
	memcpy(buf + off, sensor->sensor.mode_info[sensor->sensor.mode].raw_data, size);

	return size;
}

static BIN_ATTR_RO(bin_data, LEGO_SENSOR_RAW_DATA_SIZE);

static struct bin_attribute *user_lego_sensor_class_bin_attrs[] = {
	&bin_attr_bin_data,
	NULL
};

static const struct attribute_group user_lego_sensor_class_group = {
	.attrs		= user_lego_sensor_class_attrs,
	.bin_attrs	= user_lego_sensor_class_bin_attrs,
};

static const struct attribute_group *user_lego_sensor_class_groups[] = {
	&user_lego_sensor_class_group,
	NULL
};

static void user_lego_sensor_release(struct device *dev)
{
}

struct class user_lego_sensor_class;
static unsigned user_lego_sensor_class_id = 0;

int user_lego_sensor_register(struct user_lego_sensor_device *sensor,
			      struct device *parent)
{
	int err;

	if (!sensor || !sensor->sensor.port_name || !parent)
		return -EINVAL;

	sensor->dev.release = user_lego_sensor_release;
	sensor->dev.parent = parent;
	sensor->dev.class = &user_lego_sensor_class;
	dev_set_name(&sensor->dev, "sensor%d", user_lego_sensor_class_id++);

	err = device_register(&sensor->dev);
	if (err)
		return err;

	dev_info(&sensor->dev, "Registered '%s' on '%s'.\n", sensor->sensor.name,
		 sensor->sensor.port_name);

	return 0;
}
EXPORT_SYMBOL_GPL(user_lego_sensor_register);

void user_lego_sensor_unregister(struct user_lego_sensor_device *sensor)
{
	dev_info(&sensor->dev, "Unregistered '%s' on '%s'.\n", sensor->sensor.name,
		 sensor->sensor.port_name);
	device_unregister(&sensor->dev);
}
EXPORT_SYMBOL_GPL(user_lego_sensor_unregister);

static int user_lego_sensor_dev_uevent(struct device *dev,
				       struct kobj_uevent_env *env)
{
	struct user_lego_sensor_device *sensor = to_user_lego_sensor_device(dev);
	int ret;

	ret = add_uevent_var(env, "LEGO_DRIVER_NAME=%s", sensor->sensor.name);
	if (ret) {
		dev_err(dev, "failed to add uevent LEGO_DRIVER_NAME\n");
		return ret;
	}

	add_uevent_var(env, "LEGO_PORT_NAME=%s", sensor->sensor.port_name);
	if (ret) {
		dev_err(dev, "failed to add uevent LEGO_PORT_NAME\n");
		return ret;
	}

	return 0;
}

static char *user_lego_sensor_devnode(struct device *dev, umode_t *mode)
{
	return kasprintf(GFP_KERNEL, USER_LEGO_SENSOR_NAME "/%s", dev_name(dev));
}

struct class user_lego_sensor_class = {
	.name		= USER_LEGO_SENSOR_NAME,
	.owner		= THIS_MODULE,
	.dev_groups	= user_lego_sensor_class_groups,
	.dev_uevent	= user_lego_sensor_dev_uevent,
	.devnode	= user_lego_sensor_devnode,
};
EXPORT_SYMBOL_GPL(user_lego_sensor_class);

static int __init user_lego_sensor_class_init(void)
{
	int err;

	err = class_register(&user_lego_sensor_class);
	if (err) {
		pr_err("unable to register " USER_LEGO_SENSOR_NAME " device class\n");
		return err;
	}

	return 0;
}
module_init(user_lego_sensor_class_init);

static void __exit user_lego_sensor_class_exit(void)
{
	class_unregister(&user_lego_sensor_class);
}
module_exit(user_lego_sensor_class_exit);

MODULE_DESCRIPTION("User-defined LEGO sensor device class");
MODULE_AUTHOR("David Lechner <david@lechnology.com>");
MODULE_LICENSE("GPL");
