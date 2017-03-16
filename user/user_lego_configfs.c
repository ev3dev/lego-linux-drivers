/*
 * User-defined LEGO devices - configfs driver
 *
 * Copyright (C) 2015-2016 David Lechner <david@lechnology.com>
 *
 * Based on configfs.c from usb/gadget
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.

 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

/**
 * DOC: userspace
 *
 * This driver provides a `configfs`_ interface for creating user-defined devices
 * that use the various ev3dev drivers. Currently, only ports, sensors and LEDs
 * are implemented. Motors could be added in the future.
 *
 * .. _configfs: https://www.kernel.org/doc/Documentation/filesystems/configfs/configfs.txt
 *
 * Usage
 * -----
 *
 * .. note:: All commands assume root privileges.
 *
 * Here is an example of how to create a port and a sensor:
 *
 * - Make sure module is loaded::
 *
 *       modprobe user-lego-configfs
 *
 * - Go to the configfs directory for this driver::
 *
 *       cd /sys/kernel/config/lego_user_device
 *
 * - Create a new port called `p1`::
 *
 *       mkdir p1
 *
 * - Check out the new port - there should be ``live`` and ``sensors`` directories::
 *
 *       cd p1
 *       ls
 *
 * - This also creates a new port that is linked to ``sys/class/lego-port``::
 *
 *       ls /sys/devices/lego_user_device/lego-port/
 *
 * - Now create a sensor named ``s1`` attached to this port::
 *
 *       mkdir sensors/s1
 *
 * - Check out the sensor::
 *
 *       ls sensors/s1
 *
 * - These attributes correspond to attributes in the ``lego-sensor`` class.
 *   Set them as appropriate. For example::
 *
 *       echo "my-driver" > sensors/s1/driver_name
 *
 * - Once the attributes have been set, export the sensor by linking it to ``live``::
 *
 *       ln -s sensors/s1 live
 *
 *   There will be two new devices created, one in ``/sys/class/user-lego-sensor/``
 *   and one in ``/sys/class/lego-sensor``. The ``lego-sensor`` class device is used
 *   just as any other sensor. The ``user-lego-sensor`` device is used to feed
 *   data into the sensor. See the `user-lego-sensor-class`_ docs for more info.
 *
 * - To remove the sensor and port, perform the operations in reverse::
 *
 *       rm link/s1
 *       rmdir sensors/s1
 *       cd ..
 *       rmdir p1
 */

#include <linux/configfs.h>
#include <linux/module.h>
#include <linux/slab.h>

#include <lego.h>
#include <lego_port_class.h>
#include <lego_sensor_class.h>

#include "user_lego_sensor.h"
#include "user_led.h"

#define LEGO_USER_DEVICE_NAME "lego_user_device"

enum group_index {
	GROUP_INDEX_SENSORS,
	GROUP_INDEX_LEDS,
	GROUP_INDEX_LIVE,
	NUM_GROUP_INDEX
};

struct port_info {
	struct config_group group;
	/* default groups */
	struct config_group sensors_group;
	struct config_group uleds_group;
	struct config_group live_group;
	/* future default groups may include modes, motors and leds */
	struct config_group *default_groups[NUM_GROUP_INDEX];
	struct lego_port_device port;
	struct lego_port_mode_info mode0;
	struct mutex lock;
};

struct sensor_info {
	char driver_name[LEGO_NAME_SIZE];
	char address[LEGO_NAME_SIZE];
	struct port_info *port_info;
	struct config_group group;
	struct user_lego_sensor_device sensor;
	struct lego_sensor_mode_info mode0;
	struct mutex lock;
	bool live;
};

struct uled_info {
	char name[LEGO_NAME_SIZE];
	char color[LEGO_NAME_SIZE];
	struct port_info *port_info;
	struct config_group group;
	struct user_led led;
	struct mutex lock;
	bool live;
};

struct device *lego_user_cfs_parent;

static inline struct sensor_info *to_sensor_info(struct config_item *item)
{
	return container_of(to_config_group(item),
			    struct sensor_info, group);
}

static void sensor_info_release(struct config_item *item)
{
	struct sensor_info *info = to_sensor_info(item);

	kfree(info);
}

static struct configfs_item_operations sensor_info_ops = {
	.release		= sensor_info_release,
};

static ssize_t
sensor_info_bin_data_format_show(struct config_item *item, char *page)
{
	struct sensor_info *info = to_sensor_info(item);
	const char *value = lego_sensor_bin_data_format_to_str(
							info->mode0.data_type);

	if (!value)
		return -ENXIO;

	return sprintf(page, "%s\n", value);
}

static ssize_t
sensor_info_bin_data_format_store(struct config_item *item, const char *page,
				  size_t len)
{
	struct sensor_info *info = to_sensor_info(item);
	int ret;

	if (info->live)
		return -EBUSY;

	ret = lego_sensor_str_to_bin_data_format(page);
	if (ret < 0)
		return ret;

	info->mode0.data_type = ret;

	return len;
}

static ssize_t
sensor_info_decimals_show(struct config_item *item, char *page)
{
	struct sensor_info *info = to_sensor_info(item);

	return sprintf(page, "%d\n", info->mode0.decimals);
}

static ssize_t
sensor_info_decimals_store(struct config_item *item, const char *page,
			   size_t len)
{
	struct sensor_info *info = to_sensor_info(item);
	unsigned int value;

	if (info->live)
		return -EBUSY;

	if (kstrtouint(page, 10, &value) || value > 5)
		return -EINVAL;

	info->mode0.decimals = value;

	return len;
}

static ssize_t
sensor_info_num_values_show(struct config_item *item, char *page)
{
	struct sensor_info *info = to_sensor_info(item);

	return sprintf(page, "%d\n", info->mode0.num_values);
}

static ssize_t
sensor_info_num_values_store(struct config_item *item, const char *page,
			     size_t len)
{
	struct sensor_info *info = to_sensor_info(item);
	unsigned int value;

	if (info->live)
		return -EBUSY;

	if (kstrtouint(page, 10, &value) || value == 0 || value > 32)
		return -EINVAL;

	info->mode0.data_sets = value;
	info->mode0.num_values = value;

	return len;
}

static ssize_t
sensor_info_units_show(struct config_item *item, char *page)
{
	struct sensor_info *info = to_sensor_info(item);

	return sprintf(page, "%s\n", info->mode0.units);
}

static ssize_t
sensor_info_units_store(struct config_item *item, const char *page, size_t len)
{
	struct sensor_info *info = to_sensor_info(item);
	char *value;

	if (info->live)
		return -EBUSY;

	if (len > LEGO_SENSOR_UNITS_SIZE)
		return -EINVAL;

	value = kstrndup(page, len, GFP_KERNEL);
	if (!value)
		return -ENOMEM;

	snprintf(info->mode0.units, len, "%s", strim(value));
	kfree(value);

	return len;
}

static ssize_t
sensor_info_driver_name_show(struct config_item *item, char *page)
{
	struct sensor_info *info = to_sensor_info(item);

	return sprintf(page, "%s\n", info->driver_name);
}

static ssize_t
sensor_info_driver_name_store(struct config_item *item, const char *page,
			      size_t len)
{
	struct sensor_info *info = to_sensor_info(item);
	char *value;

	if (info->live)
		return -EBUSY;

	if (len > LEGO_NAME_SIZE)
		return -EINVAL;

	value = kstrndup(page, len, GFP_KERNEL);
	if (!value)
		return -ENOMEM;

	snprintf(info->driver_name, len, "%s", strim(value));
	kfree(value);

	return len;
}

static ssize_t
sensor_info_fw_version_show(struct config_item *item, char *page)
{
	struct sensor_info *info = to_sensor_info(item);

	return sprintf(page, "%s\n", info->sensor.sensor.fw_version);
}

static ssize_t
sensor_info_fw_version_store(struct config_item *item, const char *page,
			     size_t len)
{
	struct sensor_info *info = to_sensor_info(item);
	char *value;

	if (info->live)
		return -EBUSY;

	if (len > LEGO_SENSOR_FW_VERSION_SIZE)
		return -EINVAL;

	value = kstrndup(page, len, GFP_KERNEL);
	if (!value)
		return -ENOMEM;

	snprintf(info->sensor.sensor.fw_version, len, "%s", strim(value));
	kfree(value);

	return len;
}

/*
 * TODO: if we ever support more than one mode, these attributes need to be
 * defined per mode.
 */
CONFIGFS_ATTR(sensor_info_, bin_data_format);
CONFIGFS_ATTR(sensor_info_, decimals);
CONFIGFS_ATTR(sensor_info_, num_values);
CONFIGFS_ATTR(sensor_info_, units);

/* These attributes are common for all modes */
CONFIGFS_ATTR(sensor_info_, driver_name);
CONFIGFS_ATTR(sensor_info_, fw_version);

static struct configfs_attribute *sensor_info_attrs[] = {
	&sensor_info_attr_bin_data_format,
	&sensor_info_attr_decimals,
	&sensor_info_attr_num_values,
	&sensor_info_attr_units,
	&sensor_info_attr_driver_name,
	&sensor_info_attr_fw_version,
	NULL
};

static int sensor_info_set_mode(void *context, u8 mode)
{
	/* dummy implementation since we only have one mode */
	return 0;
}

static struct config_item_type sensor_info_type = {
	.ct_item_ops	= &sensor_info_ops,
	.ct_attrs	= sensor_info_attrs,
	.ct_owner	= THIS_MODULE,
};

static struct config_group
*sensor_make(struct config_group *group, const char *name)
{
	struct port_info *port_info =
			container_of(group, struct port_info, sensors_group);
	struct sensor_info *info;

	info = kzalloc(sizeof(*info), GFP_KERNEL);
	if (!info)
		return ERR_PTR(-ENOMEM);

	snprintf(info->driver_name, LEGO_NAME_SIZE, LEGO_USER_DEVICE_NAME);
	snprintf(info->address, LEGO_NAME_SIZE, "%s:%s",
		 port_info->port.address, name);
	info->port_info = port_info;
	info->sensor.sensor.name = info->driver_name;
	info->sensor.sensor.address = info->address;
	/* only supporting a single mode for now */
	info->sensor.sensor.num_modes = 1;
	info->sensor.sensor.mode_info = &info->mode0;
	info->sensor.sensor.set_mode = sensor_info_set_mode;
	info->sensor.sensor.context = info;

	snprintf(info->mode0.name, LEGO_NAME_SIZE, "USER");
	info->mode0.data_sets = 1;
	info->mode0.num_values = 1;
	mutex_init(&info->lock);

	config_group_init_type_name(&info->group, name, &sensor_info_type);
	return &info->group;
}

static void sensor_drop(struct config_group *group, struct config_item *item)
{
	struct sensor_info *info = to_sensor_info(item);

	mutex_lock(&info->lock);
	if (info->live)
		user_lego_sensor_unregister(&info->sensor);
	info->live = 0;
	mutex_unlock(&info->lock);

	config_item_put(item);
}

static struct configfs_group_operations sensors_group_ops = {
	.make_group	= &sensor_make,
	.drop_item	= &sensor_drop,
};

static struct config_item_type sensors_group_type = {
	.ct_group_ops	= &sensors_group_ops,
	.ct_owner	= THIS_MODULE,
};

/* User LEDs */

static inline struct uled_info *to_uled_info(struct config_item *item)
{
	return container_of(to_config_group(item), struct uled_info, group);
}

static void uled_info_release(struct config_item *item)
{
	struct uled_info *info = to_uled_info(item);

	kfree(info);
}

static struct configfs_item_operations uled_info_ops = {
	.release		= uled_info_release,
};

static ssize_t uled_info_name_show(struct config_item *item, char *page)
{
	struct uled_info *info = to_uled_info(item);

	return sprintf(page, "%s\n", info->name);
}

static ssize_t
uled_info_name_store(struct config_item *item, const char *page, size_t len)
{
	struct uled_info *info = to_uled_info(item);
	char *value;

	if (info->live)
		return -EBUSY;

	if (len > LEGO_NAME_SIZE)
		return -EINVAL;

	value = kstrndup(page, len, GFP_KERNEL);
	if (!value)
		return -ENOMEM;

	snprintf(info->name, len, "%s", strim(value));
	kfree(value);

	return len;
}

static ssize_t uled_info_color_show(struct config_item *item, char *page)
{
	struct uled_info *info = to_uled_info(item);

	return sprintf(page, "%s\n", info->color);
}

static ssize_t
uled_info_color_store(struct config_item *item, const char *page, size_t len)
{
	struct uled_info *info = to_uled_info(item);
	char *value;

	if (info->live)
		return -EBUSY;

	if (len > LEGO_NAME_SIZE)
		return -EINVAL;

	value = kstrndup(page, len, GFP_KERNEL);
	if (!value)
		return -ENOMEM;

	snprintf(info->color, len, "%s", strim(value));
	kfree(value);

	return len;
}

CONFIGFS_ATTR(uled_info_, name);
CONFIGFS_ATTR(uled_info_, color);

static struct configfs_attribute *uled_info_attrs[] = {
	&uled_info_attr_name,
	&uled_info_attr_color,
	NULL
};

static struct config_item_type uled_info_type = {
	.ct_item_ops	= &uled_info_ops,
	.ct_attrs	= uled_info_attrs,
	.ct_owner	= THIS_MODULE,
};

static struct config_group
*uled_make(struct config_group *group, const char *name)
{
	struct port_info *port_info =
			container_of(group, struct port_info, uleds_group);
	struct uled_info *info;

	info = kzalloc(sizeof(*info), GFP_KERNEL);
	if (!info)
		return ERR_PTR(-ENOMEM);

	info->port_info = port_info;

	mutex_init(&info->lock);

	config_group_init_type_name(&info->group, name, &uled_info_type);
	return &info->group;
}

static void uled_drop(struct config_group *group, struct config_item *item)
{
	struct uled_info *info = to_uled_info(item);

	mutex_lock(&info->lock);
	if (info->live)
		user_led_unregister(&info->led);
	info->live = false;
	mutex_unlock(&info->lock);

	config_item_put(item);
}

static struct configfs_group_operations uleds_group_ops = {
	.make_group	= &uled_make,
	.drop_item	= &uled_drop,
};

static struct config_item_type uleds_group_type = {
	.ct_group_ops	= &uleds_group_ops,
	.ct_owner	= THIS_MODULE,
};

/* Ports */

static int live_allow_link(struct config_item *src, struct config_item *target)
{
	int ret = -EPERM;

	if (target->ci_type == &sensor_info_type) {
		struct sensor_info *info = to_sensor_info(target);

		mutex_lock(&info->lock);
		if (info->live) {
			ret = -EBUSY;
		} else {
			/* zero out the device since the sensor struct can be reused */
			memset(&info->sensor.dev, 0, sizeof(struct device));
			/* same goes for the lego-sensor */
			memset(&info->sensor.sensor.dev, 0, sizeof(struct device));
			ret = user_lego_sensor_register(&info->sensor,
						&info->port_info->port.dev);
			if (ret == 0)
				info->live = true;
		}
		mutex_unlock(&info->lock);
	} else if (target->ci_type == &uled_info_type) {
		struct uled_info *info = to_uled_info(target);

		mutex_lock(&info->lock);
		if (info->live) {
			ret = -EBUSY;
		} else {
			snprintf(info->led.name, USER_LED_NAME_SIZE, "%s:%s:%s",
				 info->name, info->color, "ev3dev");
			ret = user_led_register(&info->led,
						&info->port_info->port.dev);
			if (ret == 0)
				info->live = true;
		}
		mutex_unlock(&info->lock);
	}

	return ret;
}

static int live_drop_link(struct config_item *src, struct config_item *target)
{
	if (target->ci_type == &sensor_info_type) {
		struct sensor_info *info = to_sensor_info(target);
		mutex_lock(&info->lock);
		user_lego_sensor_unregister(&info->sensor);
		info->live = false;
		mutex_unlock(&info->lock);
	}

	return 0;
}

static struct configfs_item_operations live_item_ops = {
	.allow_link	= &live_allow_link,
	.drop_link	= &live_drop_link,
};

static struct config_item_type live_group_type = {
	.ct_item_ops	= &live_item_ops,
	.ct_owner	= THIS_MODULE,
};

static inline struct port_info *to_port_info(struct config_item *item)
{
	return container_of(to_config_group(item), struct port_info, group);
}

static void port_info_release(struct config_item *item)
{
	struct port_info *info = to_port_info(item);

	kfree(info);
}

static struct configfs_item_operations port_info_ops = {
	.release		= port_info_release,
};

static struct config_item_type port_info_type = {
	.ct_item_ops	= &port_info_ops,
	.ct_owner	= THIS_MODULE,
};

static int port_set_mode(void *context, u8 mode)
{
	/* Ports only have one mode */
	return 0;
}

static const struct device_type lego_user_port_type = {
	.name = 	"user-lego-port",
};

static struct config_group
*port_make(struct config_group *group, const char *name)
{
	struct port_info *info;
	int err;

	info = kzalloc(sizeof(*info), GFP_KERNEL);
	if (!info)
		return ERR_PTR(-ENOMEM);

	info->port.name = LEGO_USER_DEVICE_NAME;
	snprintf(info->port.address, LEGO_NAME_SIZE, "user:%s", name);
	info->port.num_modes = 1;
	info->port.supported_modes = LEGO_PORT_ALL_MODES;
	info->port.mode_info = &info->mode0;
	info->port.set_mode = port_set_mode;
	info->port.context = info;

	snprintf(info->mode0.name, LEGO_NAME_SIZE, "USER");

	err = lego_port_register(&info->port, &lego_user_port_type,
				 lego_user_cfs_parent);
	if (err < 0) {
		kfree(info);
		return ERR_PTR(err);
	}

	mutex_init(&info->lock);

	config_group_init_type_name(&info->group, name, &port_info_type);
	config_group_init_type_name(&info->sensors_group, "sensors",
				    &sensors_group_type);
	configfs_add_default_group(&info->sensors_group, &info->group);
	config_group_init_type_name(&info->uleds_group, "leds",
				    &uleds_group_type);
	configfs_add_default_group(&info->uleds_group, &info->group);
	config_group_init_type_name(&info->live_group, "live", &live_group_type);
	configfs_add_default_group(&info->live_group, &info->group);

	return &info->group;
}

static void port_drop(struct config_group *group, struct config_item *item)
{
	struct port_info *info = to_port_info(item);

	lego_port_unregister(&info->port);
	config_item_put(item);
}

static struct configfs_group_operations lego_user_cfs_ops = {
	.make_group	= &port_make,
	.drop_item	= &port_drop,
};

static struct config_item_type lego_user_cfs_type = {
	.ct_group_ops	= &lego_user_cfs_ops,
	.ct_owner	= THIS_MODULE,
};

static struct configfs_subsystem lego_user_cfs_subsys = {
	.su_group = {
		.cg_item = {
			.ci_namebuf = LEGO_USER_DEVICE_NAME,
			.ci_type = &lego_user_cfs_type,
		},
	},
	.su_mutex = __MUTEX_INITIALIZER(lego_user_cfs_subsys.su_mutex),
};

static int __init lego_user_cfs_init(void)
{
	lego_user_cfs_parent = root_device_register(LEGO_USER_DEVICE_NAME);
	if (IS_ERR(lego_user_cfs_parent))
		return PTR_ERR(lego_user_cfs_parent);

	config_group_init(&lego_user_cfs_subsys.su_group);

	return configfs_register_subsystem(&lego_user_cfs_subsys);
}
module_init(lego_user_cfs_init);

static void __exit lego_user_cfs_exit(void)
{
	configfs_unregister_subsystem(&lego_user_cfs_subsys);
	root_device_unregister(lego_user_cfs_parent);
}
module_exit(lego_user_cfs_exit);

MODULE_DESCRIPTION("User-defined LEGO devices using configfs");
MODULE_AUTHOR("David Lechner <david@lechnology.com>");
MODULE_LICENSE("GPL");
