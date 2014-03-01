/*
 * Support for the input and output ports on the LEGO Mindstorms EV3
 *
 * Copyright (C) 2013-2014 David Lechner <david@lechnology.com>
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
 * There is a platform driver called legoev3-ports. The matching platform
 * device is declared in board-legoev3.c, which defines the gpios and
 * other devices used by the input and output ports on the EV3. When this
 * driver is loaded, it also creates a bus called "legoev3". This bus is
 * used to match the sensor and motor devices plugged into the ports to
 * drivers for those devices.
 *
 * Each input and output port has its own device node. They perform device
 * discovery similar to Device3 in d_analog.c in the lms2012 code and notify
 * the legoev3 bus when a device has been connected or disconnected.
 * -----------------------------------------------------------------------------
 */

#include <linux/device.h>
#include <linux/module.h>
#include <linux/string.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/ioport.h>
#include <linux/legoev3/legoev3_ports.h>

struct legoev3_ports_data {
	struct platform_device *pdev;
	struct legoev3_ports_platform_data *pdata;
	struct legoev3_port_device *in_ports[NUM_EV3_PORT_IN];
	struct legoev3_port_device *out_ports[NUM_EV3_PORT_OUT];
};

static struct legoev3_ports_data *legoev3_ports;

static ssize_t legoev3_show_device_type(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	return sprintf(buf, "%s\n", dev->type->name);
}

DEVICE_ATTR(device_type, S_IRUGO, legoev3_show_device_type, NULL);

static struct attribute *legoev3_port_device_type_attrs[] = {
	&dev_attr_device_type.attr,
	NULL
};

struct attribute_group legoev3_port_device_type_attr_grp = {
	.attrs	= legoev3_port_device_type_attrs,
};

EXPORT_SYMBOL_GPL(legoev3_port_device_type_attr_grp);

const struct attribute_group *ev3_input_port_device_type_attr_groups[] = {
	&legoev3_port_device_type_attr_grp,
	NULL
};

struct device_type ev3_input_port_device_type = {
	.name	= "ev3-input-port",
	.groups	= ev3_input_port_device_type_attr_groups,
};

static struct attribute *legoev3_output_port_device_type_attrs[] = {
	&dev_attr_device_type.attr,
	NULL
};

struct attribute_group legoev3_output_port_device_type_attr_grp = {
	.attrs	= legoev3_output_port_device_type_attrs,
};
EXPORT_SYMBOL_GPL(legoev3_output_port_device_type_attr_grp);

const struct attribute_group *ev3_output_port_device_type_attr_groups[] = {
	&legoev3_output_port_device_type_attr_grp,
	NULL
};

struct device_type ev3_output_port_device_type = {
	.name	= "ev3-output-port",
	.groups	= ev3_output_port_device_type_attr_groups,
};

static void legoev3_port_device_release (struct device *dev)
{
	struct legoev3_port_device *pdev = to_legoev3_port_device(dev);

	kfree(pdev->dev.platform_data);
	kfree(pdev);
}

/**
 * legoev3_port_device_register - Register a new device on the legoev3 port bus.
 * @name: The name of the device.
 * @id: The id of the device or -1 if there is only one device with this name.
 * @type: The type of device (required).
 * @platform_data: Device specific data that depends on the device type.
 * @parent: The parent device.
 */
struct legoev3_port_device
*legoev3_port_device_register(const char *name, int id, struct device_type *type,
			      void *platform_data, size_t platform_data_size,
			      struct device *parent)
{
	struct legoev3_port_device *pdev;
	void *pdata = NULL;
	int err;

	if (!legoev3_ports)
		return ERR_PTR(-EINVAL);
	if(!type)
		return ERR_PTR(-EINVAL);

	pdev = kzalloc(sizeof(struct legoev3_port_device), GFP_KERNEL);
	if (!pdev)
		return ERR_PTR(-ENOMEM);

	strncpy(pdev->name, name, LEGOEV3_PORT_NAME_SIZE);
	pdev->id = id;
	device_initialize(&pdev->dev);
	pdev->dev.type = type;
	pdev->dev.bus = &legoev3_bus_type;
	pdev->dev.release = legoev3_port_device_release;
	if (platform_data) {
		pdata = kmalloc(platform_data_size, GFP_KERNEL);
		if (!pdata) {
			err = -ENOMEM;
			goto err_kalloc_pdata;
		}
		memcpy(pdata, platform_data, platform_data_size);
		pdev->dev.platform_data = pdata;
	}
	if(parent) {
		pdev->dev.parent = parent;
		if (pdev->id < 0)
			err = dev_set_name(&pdev->dev, "%s:%s",
					   dev_name(parent), pdev->name);
		else
			err = dev_set_name(&pdev->dev, "%s:%s%c",
					   dev_name(parent), pdev->name,
					   '0' + pdev->id);
	} else {
		pdev->dev.parent = &legoev3_ports->pdev->dev;
		if (pdev->id < 0)
			err = dev_set_name(&pdev->dev, "%s", pdev->name);
		else {
			if( 0 == strncmp( pdev->name, "out", LEGOEV3_PORT_NAME_SIZE ) ) {
				err = dev_set_name(&pdev->dev, "%s%c", pdev->name,
						   'A' - 1 + pdev->id);
			} else {
				err = dev_set_name(&pdev->dev, "%s%c", pdev->name,
						   '0' + pdev->id);
                        }
		}
	}
	if (err < 0) {
		dev_err(&pdev->dev, "Could not set name.\n");
		goto err_dev_set_name;
	}

	err = device_add(&pdev->dev);
	if (err) {
		dev_err(&pdev->dev, "Failed to add device.\n");
		goto err_device_add;
	}

	return pdev;

err_device_add:
err_dev_set_name:
	kfree(pdata);
err_kalloc_pdata:
	kfree(pdev);

	return ERR_PTR(err);
}
EXPORT_SYMBOL_GPL(legoev3_port_device_register);

void legoev3_port_device_unregister(struct legoev3_port_device *pdev)
{
	if (!pdev)
		return;

	device_del(&pdev->dev);
	put_device(&pdev->dev);
}
EXPORT_SYMBOL_GPL(legoev3_port_device_unregister);

static int legoev3_port_driver_probe(struct device *dev)
{
	struct legoev3_port_driver *pdrv = to_legoev3_port_driver(dev->driver);
	struct legoev3_port_device *pdev = to_legoev3_port_device(dev);

	return pdrv->probe(pdev);
}

static int legoev3_port_driver_remove(struct device *dev)
{
	struct legoev3_port_driver *pdrv = to_legoev3_port_driver(dev->driver);
	struct legoev3_port_device *pdev = to_legoev3_port_device(dev);

	return pdrv->remove(pdev);
}

static void legoev3_port_driver_shutdown(struct device *dev)
{
	struct legoev3_port_driver *pdrv = to_legoev3_port_driver(dev->driver);
	struct legoev3_port_device *pdev = to_legoev3_port_device(dev);

	pdrv->shutdown(pdev);
}

int legoev3_register_port_driver(struct legoev3_port_driver *drv)
{
	drv->driver.bus = &legoev3_bus_type;
	if (drv->probe)
		drv->driver.probe = legoev3_port_driver_probe;
	if (drv->remove)
		drv->driver.remove = legoev3_port_driver_remove;
	if (drv->shutdown)
		drv->driver.shutdown = legoev3_port_driver_shutdown;

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

int legoev3_register_input_ports(struct legoev3_port_device *ports[],
				 struct ev3_input_port_platform_data data[],
				 unsigned len)
{
	int err;
	int i = 0;

	do {
		ports[i] = legoev3_port_device_register("in", data[i].id + 1,
			&ev3_input_port_device_type, &data[i],
			sizeof(struct ev3_input_port_platform_data), NULL);
		if (IS_ERR(ports[i])) {
			err = PTR_ERR(ports[i]);
			goto err_legoev3_port_device_register;
		}
	} while (++i < len);

	return 0;

err_legoev3_port_device_register:
	while (i--)
		legoev3_port_device_unregister(ports[i]);

	return err;
}

int legoev3_register_output_ports(struct legoev3_port_device *ports[],
				 struct ev3_output_port_platform_data data[],
				 unsigned len)
{
	int err;
	int i = 0;

	do {
		ports[i] = legoev3_port_device_register("out", data[i].id + 1,
			&ev3_output_port_device_type, &data[i],
			sizeof(struct ev3_output_port_platform_data), NULL);
		if (IS_ERR(ports[i])) {
			err = PTR_ERR(ports[i]);
			goto err_legoev3_port_device_register;
		}
	} while (++i < len);

	return 0;

err_legoev3_port_device_register:
	while (i--)
		legoev3_port_device_unregister(ports[i]);

	return err;
}

static int __devinit legoev3_ports_probe(struct platform_device *pdev)
{
	struct legoev3_ports_data *ports;
	int i = 0;
	int err;

	if (!pdev || !pdev->dev.platform_data)
		return -EINVAL;

	err = bus_register(&legoev3_bus_type);
	if (err)
		return err;

	ports = kzalloc(sizeof(struct legoev3_ports_data), GFP_KERNEL);
	if (!ports) {
		err = -ENOMEM;
		goto err_ports_kzalloc;
	}

	ports->pdev = pdev;
	ports->pdata = pdev->dev.platform_data;
	legoev3_ports = ports;

	err = legoev3_register_input_ports(ports->in_ports,
				   ports->pdata->input_port_data,
				   NUM_EV3_PORT_IN);
	if (err) {
		dev_err(&pdev->dev, "Could not register input ports!\n");
		goto err_legoev3_register_input_ports;
	}

	err = legoev3_register_output_ports(ports->out_ports,
					    ports->pdata->output_port_data,
					    NUM_EV3_PORT_OUT);

	if (err) {
		dev_err(&pdev->dev, "Could not register output ports!\n");
		goto err_legoev3_register_output_ports;
	}

	return 0;

err_pwm_request_byname:

err_legoev3_register_output_ports:
	for(i = 0; i < NUM_EV3_PORT_IN; i++)
		legoev3_port_device_unregister(ports->in_ports[i]);

err_legoev3_register_input_ports:
	legoev3_ports = NULL;
	kfree(ports);
err_ports_kzalloc:
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

	legoev3_ports = NULL;
	while ((dev = bus_find_device(&legoev3_bus_type, NULL, NULL,
				      legoev3_bus_match_all)))
	{
		legoev3_port_device_unregister(to_legoev3_port_device(dev));
	}
	bus_unregister(&legoev3_bus_type);

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
