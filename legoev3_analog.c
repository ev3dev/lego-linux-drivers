/*
 * Analog framework for LEGO Mindstorms EV3
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

#include <linux/module.h>
#include <linux/types.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/legoev3/legoev3_analog.h>

struct class *legoev3_analog_class;
EXPORT_SYMBOL_GPL(legoev3_analog_class);

static struct device_type legoev3_analog_device_type;

static void legoev3_analog_device_release(struct device *dev)
{
	kfree(dev);
}

int legoev3_analog_device_register(struct device *parent,
				   struct legoev3_analog_device *alg)
{
	struct device *dev;
	int err;

	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;

	device_initialize(dev);
	dev->class = legoev3_analog_class;
	dev->type = &legoev3_analog_device_type;
	dev->parent = parent;
	dev->release = legoev3_analog_device_release;
	dev_set_drvdata(dev, alg);
	alg->dev = dev;

	err = kobject_set_name(&dev->kobj, "%s", alg->name);
	if (err)
		goto kobject_set_name_failed;

	err = device_add(dev);
	if (err)
		goto add_device_failed;

	return 0;

kobject_set_name_failed:
add_device_failed:
	put_device(dev);

	return err;
}
EXPORT_SYMBOL_GPL(legoev3_analog_device_register);

static ssize_t legoev3_analog_show_name(struct device *dev,
					struct device_attribute *devattr,
					char *buf)
{
	return sprintf(buf, "%s\n", dev->kobj.name);
}

static DEVICE_ATTR(name, S_IRUGO, legoev3_analog_show_name, NULL);

static struct attribute *legoev3_analog_attrs[] = {
	&dev_attr_name.attr,
	NULL
};

static struct attribute_group legoev3_analog_attr_grp = {
	.attrs = legoev3_analog_attrs,
};

static const struct attribute_group *legoev3_analog_attr_groups[] = {
	&legoev3_analog_attr_grp,
	NULL
};

void legoev3_analog_device_unregister(struct legoev3_analog_device *alg)
{
	device_unregister(alg->dev);
}
EXPORT_SYMBOL_GPL(legoev3_analog_device_unregister);

static int __init legoev3_analog_class_init(void)
{
	legoev3_analog_class = class_create(THIS_MODULE, "legoev3-analog");
	if (IS_ERR(legoev3_analog_class))
		return PTR_ERR(legoev3_analog_class);

	legoev3_analog_device_type.groups = legoev3_analog_attr_groups;

	return 0;
}
subsys_initcall(legoev3_analog_class_init);

static void __exit legoev3_analog_class_exit(void)
{
	class_destroy(legoev3_analog_class);
}
module_exit(legoev3_analog_class_exit);

MODULE_DESCRIPTION("Analog driver for LEGO Mindstorms EV3");
MODULE_AUTHOR("David Lechner <david@lechnology.com>");
MODULE_LICENSE("GPL");
