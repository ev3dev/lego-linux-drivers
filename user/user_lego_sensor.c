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

/*
 * Note: The comment block below is used to generate docs on the ev3dev website.
 * Use kramdown (markdown) syntax. Use a '.' as a placeholder when blank lines
 * or leading whitespace is important for the markdown syntax.
 */

/**
 * DOC: website
 *
 * User-defined LEGO sensor devices
 *
 * The `user-lego-sensor` class provides an interface for implementing user-defined
 * sensors in userspace.
 * .
 * ### Identifying sensors
 * .
 * Since the name of the `sensor<N>` device node does not correspond to the port
 * that a sensor is plugged in to, you must look at the `port_name` attribute if
 * you need to know which port a sensor is plugged in to. This will match the
 * `port_name` of the corresponding sensor in the `lego-sensor` class.
 * .
 * ### sysfs Attributes
 * .
 * Sensors can be found at `/sys/class/user-lego-sensor/sensor<N>`, where `<N>`
 * is incremented each time a sensor is loaded (it is not related to which port
 * the sensor is plugged in to).
 * .
 * `bin_data` (write-only)
 * : The written data will be stored and can be read using the corresponding
 *   `bin_data` attribute in the `lego-sensor` class device.
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

static ssize_t text_value_store(struct device *dev, struct device_attribute *attr,
			        const char *buf, size_t count)
{
 	struct user_lego_sensor_device *sensor = to_user_lego_sensor_device(dev);
 
	return snprintf( sensor->text_value, USER_LEGO_SENSOR_TEXT_VALUE_SIZE, buf );
}

static DEVICE_ATTR_RO(port_name);
static DEVICE_ATTR_WO(text_value);

static struct attribute *user_lego_sensor_class_attrs[] = {
	&dev_attr_port_name.attr,
	&dev_attr_text_value.attr,
	NULL
};

static ssize_t bin_data_write(struct file *file, struct kobject *kobj,
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
	memcpy(sensor->sensor.mode_info[sensor->sensor.mode].raw_data + off,
	       buf, size);

	return size;
}

static BIN_ATTR(bin_data, S_IWUSR | S_IWGRP, NULL, bin_data_write,
		LEGO_SENSOR_RAW_DATA_SIZE);

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

const char *user_lego_sensor_get_text_value(void *context) {
	struct user_lego_sensor_device *sensor = context;

        return sensor->text_value;
}
 
static void user_lego_sensor_release(struct device *dev)
{
}

struct class user_lego_sensor_class;
static unsigned user_lego_sensor_class_id = 0;

int user_lego_sensor_register(struct user_lego_sensor_device *sensor,
			      struct device *parent)
{
	int err;

	if (WARN_ON(!sensor))
		return -EINVAL;
	if (WARN_ON(!sensor->sensor.port_name))
		return -EINVAL;
	if (WARN_ON(!parent))
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

	sensor->sensor.context = sensor; 
        sensor->sensor.get_text_value = user_lego_sensor_get_text_value;

	err = register_lego_sensor(&sensor->sensor, &sensor->dev);
	if (err) {
		dev_err(&sensor->dev,
			"Failed to register lego-sensor class device. %d\n",
			err);
		device_unregister(&sensor->dev);
		return err;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(user_lego_sensor_register);

void user_lego_sensor_unregister(struct user_lego_sensor_device *sensor)
{
	unregister_lego_sensor(&sensor->sensor);
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
