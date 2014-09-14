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
	struct legoev3_port *in_ports[NUM_EV3_PORT_IN];
	struct legoev3_port *out_ports[NUM_EV3_PORT_OUT];
};

static struct legoev3_ports_data *legoev3_ports;

static uint disable_in_port[NUM_EV3_PORT_IN];
static int num_disabled_in_port;
module_param_array(disable_in_port, uint, &num_disabled_in_port, 0);
MODULE_PARM_DESC(disable_in_port, "Disables specified input ports. (1,2,3,4)");
static uint disable_out_port[NUM_EV3_PORT_OUT];
static int num_disabled_out_port;
module_param_array(disable_out_port, uint, &num_disabled_out_port, 0);
MODULE_PARM_DESC(disable_out_port, "Disables specified output ports. (1,2,3,4)");

static ssize_t legoev3_port_device_show_port_name(struct device *dev,
						  struct device_attribute *attr,
						  char *buf)
{
	struct legoev3_port_device *pdev = to_legoev3_port_device(dev);

	return snprintf(buf, LEGOEV3_PORT_NAME_SIZE, "%s\n", dev_name(&pdev->port->dev));
}

static ssize_t legoev3_port_device_show_device_type(struct device *dev,
						    struct device_attribute *attr,
						    char *buf)
{
	return sprintf(buf, "%s\n", dev->type->name);
}

DEVICE_ATTR(port_name, S_IRUGO, legoev3_port_device_show_port_name, NULL);
DEVICE_ATTR(device_type, S_IRUGO, legoev3_port_device_show_device_type, NULL);

static struct attribute *legoev3_port_device_type_attrs[] = {
	&dev_attr_port_name.attr,
	&dev_attr_device_type.attr,
	NULL
};

struct attribute_group legoev3_port_device_type_attr_grp = {
	.attrs	= legoev3_port_device_type_attrs,
};

EXPORT_SYMBOL_GPL(legoev3_port_device_type_attr_grp);

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

static void legoev3_port_release (struct device *dev)
{
	struct legoev3_port *port = to_legoev3_port(dev);

	kfree(port->dev.platform_data);
	kfree(port);
}

/**
 * legoev3_port_register - Register a new port on the legoev3 port bus.
 * @name: The name of the port.
 * @id: The sysfs node id of the port - should match port number.
 * @type: The type of device.
 * @platform_data: Device specific data that depends on the device type.
 * @platform_data_size: Size of platform_data.
 */
struct legoev3_port
*legoev3_port_register(const char *name, int id, struct device_type *type,
		       void *platform_data, size_t platform_data_size)
{
	struct legoev3_port *port;
	void *pdata = NULL;
	int err;

	if (!legoev3_ports || !name || !type)
		return ERR_PTR(-EINVAL);

	port = kzalloc(sizeof(struct legoev3_port), GFP_KERNEL);
	if (!port)
		return ERR_PTR(-ENOMEM);

	strncpy(port->name, name, LEGOEV3_PORT_NAME_SIZE);
	port->id = id;
	device_initialize(&port->dev);
	port->dev.type = type;
	port->dev.bus = &legoev3_bus_type;
	port->dev.release = legoev3_port_release;
	if (platform_data) {
		pdata = kmalloc(platform_data_size, GFP_KERNEL);
		if (!pdata) {
			err = -ENOMEM;
			goto err_kalloc_pdata;
		}
		memcpy(pdata, platform_data, platform_data_size);
		port->dev.platform_data = pdata;
	}
	port->dev.parent = &legoev3_ports->pdev->dev;
	/* special case for output ports since they are labeled A-D on the EV3 */
	if (0 == strncmp(port->name, "out", LEGOEV3_PORT_NAME_SIZE))
		err = dev_set_name(&port->dev, "%s%c", port->name,
				   'A' + port->id - 1);
	else
		err = dev_set_name(&port->dev, "%s%c", port->name,
				   '0' + port->id);
	if (err < 0) {
		dev_err(&port->dev, "Could not set name.\n");
		goto err_dev_set_name;
	}

	err = device_add(&port->dev);
	if (err) {
		dev_err(&port->dev, "Failed to add device.\n");
		goto err_device_add;
	}

	return port;

err_device_add:
err_dev_set_name:
	kfree(pdata);
err_kalloc_pdata:
	kfree(port);

	return ERR_PTR(err);
}

void legoev3_port_unregister(struct legoev3_port *pdev)
{
	if (!pdev)
		return;

	device_del(&pdev->dev);
	put_device(&pdev->dev);
}

int legoev3_port_device_uevent(struct device *dev,
			       struct kobj_uevent_env *env)
{
	struct legoev3_port_device *pdev = to_legoev3_port_device(dev);
	int err;

	err = add_uevent_var(env, "PORT=%s", dev_name(&pdev->port->dev));
	if (err)
		return err;

	return 0;
}
EXPORT_SYMBOL_GPL(legoev3_port_device_uevent);

static void legoev3_port_device_release (struct device *dev)
{
	struct legoev3_port_device *pdev = to_legoev3_port_device(dev);

	kfree(pdev->dev.platform_data);
	kfree(pdev);
}

/**
 * legoev3_port_device_register - Register a new device on the legoev3 port bus.
 * @name: The name of the device.
 * @type: The type of device (required).
 * @platform_data: Device specific data that depends on the device type.
 * @port: The port the device is attached to.
 */
struct legoev3_port_device
*legoev3_port_device_register(const char *name, struct device_type *type,
			      void *platform_data,
			      size_t platform_data_size,
			      struct legoev3_port *port)
{
	struct legoev3_port_device *pdev;
	void *pdata = NULL;
	int err;

	if (!legoev3_ports || !name || !type || !port)
		return ERR_PTR(-EINVAL);

	pdev = kzalloc(sizeof(struct legoev3_port_device), GFP_KERNEL);
	if (!pdev)
		return ERR_PTR(-ENOMEM);

	strncpy(pdev->name, name, LEGOEV3_PORT_NAME_SIZE);
	pdev->port = port;
	pdev->id = -1;
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
	pdev->dev.parent = &port->dev;
	err = dev_set_name(&pdev->dev, "%s:%s", dev_name(&pdev->port->dev), pdev->name);
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
	struct legoev3_port *port = to_legoev3_port(dev);

	return pdrv->probe(port);
}

static int legoev3_port_driver_remove(struct device *dev)
{
	struct legoev3_port_driver *pdrv = to_legoev3_port_driver(dev->driver);
	struct legoev3_port *port = to_legoev3_port(dev);

	return pdrv->remove(port);
}

static void legoev3_port_driver_shutdown(struct device *dev)
{
	struct legoev3_port_driver *pdrv = to_legoev3_port_driver(dev->driver);
	struct legoev3_port *port = to_legoev3_port(dev);

	pdrv->shutdown(port);
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

static int legoev3_port_device_driver_probe(struct device *dev)
{
	struct legoev3_port_device_driver *pdrv =
				to_legoev3_port_device_driver(dev->driver);
	struct legoev3_port_device *pdev = to_legoev3_port_device(dev);

	return pdrv->probe(pdev);
}

static int legoev3_port_device_driver_remove(struct device *dev)
{
	struct legoev3_port_device_driver *pdrv =
				to_legoev3_port_device_driver(dev->driver);
	struct legoev3_port_device *pdev = to_legoev3_port_device(dev);

	return pdrv->remove(pdev);
}

static void legoev3_port_device_driver_shutdown(struct device *dev)
{
	struct legoev3_port_device_driver *pdrv =
				to_legoev3_port_device_driver(dev->driver);
	struct legoev3_port_device *pdev = to_legoev3_port_device(dev);

	pdrv->shutdown(pdev);
}

int legoev3_register_port_device_driver(struct legoev3_port_device_driver *drv)
{
	drv->driver.bus = &legoev3_bus_type;
	if (drv->probe)
		drv->driver.probe = legoev3_port_device_driver_probe;
	if (drv->remove)
		drv->driver.remove = legoev3_port_device_driver_remove;
	if (drv->shutdown)
		drv->driver.shutdown = legoev3_port_device_driver_shutdown;

	return driver_register(&drv->driver);
}
EXPORT_SYMBOL_GPL(legoev3_register_port_device_driver);

void legoev3_unregister_port_device_driver(struct legoev3_port_device_driver *drv)
{
	driver_unregister(&drv->driver);
}
EXPORT_SYMBOL_GPL(legoev3_unregister_port_device_driver);

static int legoev3_bus_match(struct device *dev, struct device_driver *drv)
{
	struct legoev3_port_device *pdev;
	struct legoev3_port_device_driver *pdevdrv;
	const struct legoev3_port_device_id *pdevid;

	/* special matching for port devices since they can have a table */
	if (drv->probe == legoev3_port_device_driver_probe) {
		pdev = to_legoev3_port_device(dev);
		pdevdrv = to_legoev3_port_device_driver(drv);
		pdevid = pdevdrv->id_table;

		if (pdevid) {
			while (pdevid->name[0]) {
				if (!strcmp(pdev->name, pdevid->name)) {
					pdev->entry_id = pdevid;
					return 1;
				}
				pdevid++;
			}
		}
		return !strcmp(pdev->name, drv->name);
	}
	/* otherwise regular name matching using type name */
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

DEVICE_ATTR_RO(modalias);

static struct attribute *legoev3_bus_dev_attrs[] = {
	&dev_attr_modalias.attr,
	NULL
};
ATTRIBUTE_GROUPS(legoev3_bus_dev);

struct bus_type legoev3_bus_type = {
	.name		= "legoev3",
	.dev_groups	= legoev3_bus_dev_groups,
	.match		= legoev3_bus_match,
	.uevent		= legoev3_bus_uevent,
};
EXPORT_SYMBOL_GPL(legoev3_bus_type);

struct device_type ev3_input_port_device_type = {
	.name	= "ev3-input-port",
};

int legoev3_register_input_ports(struct legoev3_port *ports[],
				 struct ev3_input_port_platform_data data[],
				 unsigned len)
{
	int err, j, id;
	int i = 0;
	bool skip;

	do {
		skip = false;
		id = data[i].id + 1;
		for (j = 0; j < num_disabled_in_port; j++) {
			if (disable_in_port[j] == id) {
				skip = true;
				break;
			}
		}
		if (skip) {
			dev_info(&legoev3_ports->pdev->dev,
				"Input port in%d is disabled.\n", id);
			continue;
		}
		ports[i] = legoev3_port_register("in", id,
			&ev3_input_port_device_type, &data[i],
			sizeof(struct ev3_input_port_platform_data));
		if (IS_ERR(ports[i])) {
			err = PTR_ERR(ports[i]);
			goto err_legoev3_port_register;
		}
	} while (++i < len);

	return 0;

err_legoev3_port_register:
	while (i--)
		legoev3_port_unregister(ports[i]);

	return err;
}

struct device_type ev3_output_port_device_type = {
	.name	= "ev3-output-port",
};

int legoev3_register_output_ports(struct legoev3_port *ports[],
				 struct ev3_output_port_platform_data data[],
				 unsigned len)
{
	int err, id, j;
	int i = 0;
	bool skip;

	do {
		skip = false;
		id = data[i].id + 1;
		for (j = 0; j < num_disabled_out_port; j++) {
			if (disable_out_port[j] == id) {
				skip = true;
				break;
			}
		}
		if (skip) {
			dev_info(&legoev3_ports->pdev->dev,
				"Output port out%c is disabled.\n", 'A' + id - 1);
			continue;
		}
		ports[i] = legoev3_port_register("out", id,
			&ev3_output_port_device_type, &data[i],
			sizeof(struct ev3_output_port_platform_data));
		if (IS_ERR(ports[i])) {
			err = PTR_ERR(ports[i]);
			goto err_legoev3_port_register;
		}
	} while (++i < len);

	return 0;

err_legoev3_port_register:
	while (i--)
		legoev3_port_unregister(ports[i]);

	return err;
}

static int legoev3_ports_probe(struct platform_device *pdev)
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

err_legoev3_register_output_ports:
	for(i = 0; i < NUM_EV3_PORT_IN; i++)
		legoev3_port_unregister(ports->in_ports[i]);
err_legoev3_register_input_ports:
	legoev3_ports = NULL;
	kfree(ports);
err_ports_kzalloc:
	bus_unregister(&legoev3_bus_type);

	return err;
}

static int legoev3_bus_match_port_device (struct device *dev, void *data)
{
	const struct device_type *type = dev->type;

	if (!type || type == &ev3_input_port_device_type
			|| type == &ev3_output_port_device_type)
		return 0;

	return 1;
}

static int legoev3_bus_match_port (struct device *dev, void *data)
{
	const struct device_type *type = dev->type;

	if (type == &ev3_input_port_device_type
			|| type == &ev3_output_port_device_type)
		return 1;

	return 0;
}

static int legoev3_ports_remove(struct platform_device *pdev)
{
	struct device *dev;

	legoev3_ports = NULL;
	while ((dev = bus_find_device(&legoev3_bus_type, NULL, NULL,
				      legoev3_bus_match_port_device)))
		legoev3_port_device_unregister(to_legoev3_port_device(dev));
	while ((dev = bus_find_device(&legoev3_bus_type, NULL, NULL,
				      legoev3_bus_match_port)))
		legoev3_port_unregister(to_legoev3_port(dev));
	bus_unregister(&legoev3_bus_type);

	return 0;
}

static struct platform_driver legoev3_ports_driver = {
	.driver	= {
		.name	= "legoev3-ports",
		.owner	= THIS_MODULE,
	},
	.probe	= legoev3_ports_probe,
	.remove	= legoev3_ports_remove,
};

module_platform_driver(legoev3_ports_driver);

MODULE_DESCRIPTION("Support for LEGO Mindstorms EV3 input and output ports.");
MODULE_AUTHOR("David Lechner <david@lechnology.com>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:legoev3-ports");
