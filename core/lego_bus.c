/*
 * LEGO device bus driver
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
#include <linux/string.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/ioport.h>

#include <lego.h>
#include <lego_port_class.h>

static void lego_device_release (struct device *dev)
{
	struct lego_device *ldev = to_lego_device(dev);

	kfree(ldev->dev.platform_data);
	kfree(ldev);
}

/**
 * lego_device_register - Register a new device on the lego bus.
 * @name: The name of the device.
 * @type: The type of device.
 * @port: The port the device is attached to.
 * @platform_data: Device specific data that depends on the device type (optional).
 * @platform_data_size: Size of platform data.
 */
struct lego_device *lego_device_register(const char *name,
					 const struct device_type *type,
					 struct lego_port_device *port,
					 void *platform_data,
					 size_t platform_data_size)
{
	struct lego_device *ldev;
	void *pdata = NULL;
	char init_name[LEGO_NAME_SIZE + 1];
	int err;

	if (!name || !type || !port)
		return ERR_PTR(-EINVAL);

	ldev = kzalloc(sizeof(struct lego_device), GFP_KERNEL);
	if (!ldev)
		return ERR_PTR(-ENOMEM);

	strncpy(ldev->name, name, LEGO_NAME_SIZE);
	ldev->port = port;
	snprintf(init_name, LEGO_NAME_SIZE, "%s:%s", ldev->port->address,
		 ldev->name);
	ldev->dev.init_name = init_name;
	ldev->dev.id = -1;
	ldev->dev.parent = &port->dev;
	ldev->dev.type = type;
	ldev->dev.bus = &lego_bus_type;
	ldev->dev.release = lego_device_release;
	if (platform_data) {
		pdata = kmalloc(platform_data_size, GFP_KERNEL);
		if (!pdata) {
			err = -ENOMEM;
			goto err_kalloc_pdata;
		}
		memcpy(pdata, platform_data, platform_data_size);
		ldev->dev.platform_data = pdata;
	}

	err = device_register(&ldev->dev);
	if (err) {
		dev_err(&ldev->dev, "Failed to add device.\n");
		put_device(&ldev->dev);
		return ERR_PTR(err);
	}

	dev_info(ldev->dev.parent, "Added new device '%s'\n", dev_name(&ldev->dev));

	return ldev;

err_kalloc_pdata:
	kfree(ldev);

	return ERR_PTR(err);
}
EXPORT_SYMBOL_GPL(lego_device_register);

void lego_device_unregister(struct lego_device *ldev)
{
	if (!ldev)
		return;

	dev_info(ldev->dev.parent, "Removed device '%s'\n", dev_name(&ldev->dev));

	device_unregister(&ldev->dev);
}
EXPORT_SYMBOL_GPL(lego_device_unregister);

static int lego_device_driver_probe(struct device *dev)
{
	struct lego_device_driver *ldrv = to_lego_device_driver(dev->driver);
	struct lego_device *ldev = to_lego_device(dev);

	return ldrv->probe(ldev);
}

static int lego_device_driver_remove(struct device *dev)
{
	struct lego_device_driver *ldrv = to_lego_device_driver(dev->driver);
	struct lego_device *ldev = to_lego_device(dev);

	return ldrv->remove(ldev);
}

static void lego_device_driver_shutdown(struct device *dev)
{
	struct lego_device_driver *ldrv = to_lego_device_driver(dev->driver);
	struct lego_device *ldev = to_lego_device(dev);

	ldrv->shutdown(ldev);
}

int lego_device_driver_register(struct lego_device_driver *drv)
{
	drv->driver.bus = &lego_bus_type;
	if (drv->probe)
		drv->driver.probe = lego_device_driver_probe;
	if (drv->remove)
		drv->driver.remove = lego_device_driver_remove;
	if (drv->shutdown)
		drv->driver.shutdown = lego_device_driver_shutdown;

	return driver_register(&drv->driver);
}
EXPORT_SYMBOL_GPL(lego_device_driver_register);

void lego_device_driver_unregister(struct lego_device_driver *drv)
{
	driver_unregister(&drv->driver);
}
EXPORT_SYMBOL_GPL(lego_device_driver_unregister);

static ssize_t modalias_show(struct device *dev, struct device_attribute *attr,
			     char *buf)
{
	return sprintf(buf, "lego:%s\n", dev->type->name);
}

static DEVICE_ATTR_RO(modalias);

static struct attribute *lego_bus_dev_attrs[] = {
	&dev_attr_modalias.attr,
	NULL
};
ATTRIBUTE_GROUPS(lego_bus_dev);

static int lego_bus_match(struct device *dev, struct device_driver *drv)
{
	struct lego_device *ldev = to_lego_device(dev);
	struct lego_device_driver *ldrv = to_lego_device_driver(drv);
	const struct lego_device_id *id = ldrv->id_table;

	/* device type name must match driver name */
	if (strcmp(ldev->dev.type->name, drv->name))
		return 0;

	/* match entry from id table if there is one */
	if (id) {
		while (id->name[0]) {
			if (!strcmp(ldev->name, id->name)) {
				ldev->entry_id = id;
				return 1;
			}
			id++;
		}
		return 0;
	}

	return !strcmp(ldev->name, drv->name);
}

static int lego_bus_uevent(struct device *dev, struct kobj_uevent_env *env)
{
	struct lego_device *ldev = to_lego_device(dev);
	int err;

	err = add_uevent_var(env, "LEGO_DRIVER_NAME=%s", ldev->name);
	if (err)
		return err;
	err = add_uevent_var(env, "LEGO_ADDRESS=%s", ldev->port->address);
	if (err)
		return err;
	err = add_uevent_var(env, "MODALIAS=lego:%s", dev->type->name);
	if (err)
		return err;

	return 0;
}

struct bus_type lego_bus_type = {
	.name		= "lego",
	.dev_groups	= lego_bus_dev_groups,
	.match		= lego_bus_match,
	.uevent		= lego_bus_uevent,
};
EXPORT_SYMBOL_GPL(lego_bus_type);

static int __init lego_bus_init(void)
{
	return bus_register(&lego_bus_type);
}
module_init(lego_bus_init);

static void __exit lego_bus_exit(void)
{
	bus_unregister(&lego_bus_type);
}
module_exit(lego_bus_exit);

MODULE_DESCRIPTION("LEGO device bus");
MODULE_AUTHOR("David Lechner <david@lechnology.com>");
MODULE_LICENSE("GPL");
