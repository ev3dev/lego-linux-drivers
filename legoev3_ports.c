/*
 * Support for the input and output ports on the LEGO Mindstorms EV3
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

/*
 * -----------------------------------------------------------------------------
 * This module provides components for interfacing with the input and output
 * ports on the EV3 brick.
 *
 * There is a platform driver called legoev3-ports. The matching platform device
 * is declared in board-legoev3.c, which defineds the gpios and othere devices
 * used by the ports. When this driver is loaded, it also creates a bus called
 * "legoev3". This bus is used to match the sensor and motor devices plugged
 * into the ports to drivers for those devices.
 *
 * Each input and output port has its own device node. They preform device
 * discovery similar to Device3 in d_analog.c in the LMS2012 code and notify
 * the legoev3 bus when a device has been connected or disconnected.
 * -----------------------------------------------------------------------------
 */

#include <linux/module.h>
#include <linux/string.h>
#include <linux/slab.h>
#include <linux/legoev3/legoev3_ports.h>

static ssize_t legoev3_show_device_type(struct device *dev, struct device_attribute *attr,
				char *buf)
{
	return sprintf(buf, "%s\n", dev->type->name);
}

DEVICE_ATTR(device_type, S_IRUGO, legoev3_show_device_type, NULL);

static struct attribute *legoev3_device_type_attrs[] = {
	&dev_attr_device_type.attr,
	NULL
};

struct attribute_group legoev3_device_type_attr_grp = {
	.attrs	= legoev3_device_type_attrs,
};
EXPORT_SYMBOL_GPL(legoev3_device_type_attr_grp);

const struct attribute_group *legoev3_input_port_device_type_attr_groups[] = {
	&legoev3_device_type_attr_grp,
	NULL
};

struct device_type legoev3_input_port_device_type = {
	.name	= "legoev3-input-port",
	.groups	= legoev3_input_port_device_type_attr_groups,
};
EXPORT_SYMBOL_GPL(legoev3_input_port_device_type);

static void legoev3_device_release (struct device *dev)
{
	kfree(dev);
}

int legoev3_register_input_port(struct legoev3_input_port_platform_data *data,
				struct device *parent)
{
	struct device *dev;
	int err;

	if (!data)
		return -EINVAL;
	if (data->id >= LEGOEV3_NUM_PORT_IN)
		return -EINVAL;

	dev = kzalloc(sizeof(struct device), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;

	dev->parent = parent;
	err = dev_set_name(dev, "in%c", '1' + data->id);
	if (err < 0)
		goto dev_set_name_fail;
	dev->type = &legoev3_input_port_device_type;
	dev->bus = &legoev3_bus_type;
	dev->platform_data = data;
	dev->release = legoev3_device_release;

	err = device_register(dev);
	if (err)
		goto device_register_fail;

	return 0;

device_register_fail:
	put_device(dev);
	return err;
dev_set_name_fail:
	kfree(dev);
	return err;
}

static int legoev3_match_input_port_id(struct device *dev, void *data)
{
	enum legoev3_input_port_id *id = data;
	struct legoev3_input_port_platform_data *pdata;

	if (dev->type == &legoev3_input_port_device_type) {
		pdata = dev-> platform_data;
		return pdata->id == *id;
	}

	return 0;
}

void legoev3_unregister_input_port(enum legoev3_input_port_id id)
{
	struct device *dev = bus_find_device(&legoev3_bus_type, NULL, &id,
					     legoev3_match_input_port_id);

	if (dev)
		device_unregister(dev);
}

int legoev3_register_input_ports(struct legoev3_input_port_platform_data *data,
				 unsigned len, struct device *parent)
{
	int err, i;

	i = 0;
	do {
		err = legoev3_register_input_port(&data[i], parent);
		if (err)
			goto legoev3_register_input_port_fail;
	} while (++i < len);

	return 0;

legoev3_register_input_port_fail:
	while (i--)
		legoev3_unregister_input_port(data[i].id);

	return err;
}

int legoev3_register_port_driver(struct legoev3_port_driver *drv)
{
	drv->driver.bus = &legoev3_bus_type;
	return driver_register(&drv->driver);
}
EXPORT_SYMBOL_GPL(legoev3_register_port_driver);

void legoev3_unregister_port_driver(struct legoev3_port_driver *drv)
{
	driver_unregister(&drv->driver);
}
EXPORT_SYMBOL_GPL(legoev3_unregister_port_driver);

static int legoev3_bus_match(struct device *dev, struct device_driver *drv)
{
	return !strcmp(dev->type->name, drv->name);
}

static int legoev3_bus_uevent(struct device *dev, struct kobj_uevent_env *env)
{
	int err;

	err = add_uevent_var(env, "MODALIAS=legoev3:%s", dev->type->name);
	if (err)
		return err;

	return 0;
}

static ssize_t modalias_show(struct device *dev,
			     struct device_attribute *attr,
			     char *buf)
{
	return sprintf(buf, "legoev3:%s\n", dev->type->name);
}

static struct device_attribute legoev3_dev_attrs[] = {
	__ATTR_RO(modalias),
	__ATTR_NULL
};

struct bus_type legoev3_bus_type = {
	.name		= "legoev3",
	.dev_attrs	= legoev3_dev_attrs,
	.match		= legoev3_bus_match,
	.uevent		= legoev3_bus_uevent,
};
EXPORT_SYMBOL_GPL(legoev3_bus_type);

static int __devinit legoev3_ports_probe(struct platform_device *pdev)
{
	struct legoev3_ports_platform_data *pdata;
	int err;

	if (!pdev || !pdev->dev.platform_data)
		return -EINVAL;

	err = bus_register(&legoev3_bus_type);
	if (err)
		return err;

	pdata = pdev->dev.platform_data;

	err = legoev3_register_input_ports(pdata->input_port_data,
					   ARRAY_SIZE(pdata->input_port_data),
					   &pdev->dev);
	if (err) {
		dev_err(&pdev->dev, "Could not register input ports!\n");
		goto legoev3_register_input_ports_fail;
	}

	return 0;

legoev3_register_input_ports_fail:
	bus_unregister(&legoev3_bus_type);

	return err;
}

static int legoev3_bus_match_all (struct device *dev, void *data)
{
	return 1;
}

static int __devexit legoev3_ports_remove(struct platform_device *pdev)
{
	struct device *dev;

	while ((dev = bus_find_device(&legoev3_bus_type, NULL, NULL,
				      legoev3_bus_match_all)))
	{
		device_unregister(dev);
		put_device(dev);
	}
	bus_unregister(&legoev3_bus_type);
printk("%s\n", __func__);
	return 0;
}

static struct platform_driver legoev3_ports_driver = {
	.driver	= {
		.name	= "legoev3-ports",
		.owner	= THIS_MODULE,
	},
	.probe	= legoev3_ports_probe,
	.remove	= __devexit_p(legoev3_ports_remove),
};

module_platform_driver(legoev3_ports_driver);

MODULE_DESCRIPTION("Support for LEGO Mindstorms EV3 input and output ports.");
MODULE_AUTHOR("David Lechner <david@lechnology.com>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:legoev3-ports");
