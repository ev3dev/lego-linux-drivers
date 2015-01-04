/*
 * LEGO port class driver
 *
 * Copyright (C) 2014-2015 David Lechner <david@lechnology.com>
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
 * Use kramdown (markdown) syntas. Use a '.' as a placeholder when blank lines
 * or leading whitespace is important for the markdown syntax.
 */

/**
 * DOC: website
 *
 * LEGO Port Class Driver
 *
 * The `lego-port` class provides an interface for working with input and
 * output ports that are compatible with LEGO MINDSTORMS RCX/NXT/EV3, LEGO
 * WeDo and LEGO Power Functions sensors and motors. Supported devices include
 * the LEGO MINDSTORMS EV3 Intelligent Brick, the LEGO WeDo USB hub and
 * various sensor multiplexers from 3rd party manufacturers.
 * .
 * Some types of ports may have multiple modes of operation. For example, the
 * input ports on the EV3 brick can communicate with sensors using UART, I2C
 * or analog validate signals - but not all at the same time. Therefore there
 * are multiple modes available to connect to the different types of sensors.
 * .
 * In most cases, ports are able to automatically detect what type of sensor
 * or motor is connected. In some cases though, this must be manually specified
 * using the `mode` and `set_device` attributes. The `mode` attribute affects
 * how the port communicates with the connected device. For example the input
 * ports on the EV3 brick can communicate using UART, I2C or analog voltages,
 * but not all at the same time, so the mode must be set to the one that is
 * appropriate for the connected sensor. The `set_device` attribute is used to
 * specify the exact type of sensor that is connected. Note: the mode must be
 * correctly set before setting the sensor type.
 * .
 * ### sysfs attributes
 * .
 * Ports can be found at `/sys/class/lego-port/port<N>` where `<N>` is
 * incremented each time a new port is registered. Note: The number is not
 * related to the actual port at all - use the `port_name` attribute to find
 * a specific port.
 * .
 * `driver_name` (read-only)
 * : Returns the name of the driver that loaded this device. You can find the
 *   complete list of drivers in the [list of port drivers].
 * .
 * `modes` (read-only)
 * : Returns a space separated list of the available modes of the port.
 * .
 * `mode` (read/write)
 * : Reading returns the currently selected mode. Writing sets the mode.
 *   Generally speaking when the mode changes any sensor or motor devices
 *   associated with the port will be removed new ones loaded, however this
 *   this will depend on the individual driver implementing this class.
 * .
 * `port_name` (read-only)
 * : Returns the name of the port. See individual driver documentation for
 *   the name that will be returned.
 * .
 * `set_device`: (write-only)
 * : For modes that support it, writing the name of a driver will cause a new
 *   device to be registered for that driver and attached to this port. For
 *   example, since Analog/NXT sensors cannot be auto-detected, you must use
 *   this attribute to load the correct driver. Returns -EOPNOTSUPP if setting a
 *   device is not supported.
 * .
 * `status`: (read-only)
 * : In most cases, reading status will return the same value as `mode`. In
 *   cases where there is an `auto` mode additional values may be returned,
 *   such as `no-device` or `error`. See individual port driver documentation
 *   for the full list of possible values.
 * .
 * ### Events
 * .
 * In addition to the usual "add" and "remove" events, the kernel "change"
 * event is emitted when `mode` or `status` changes.
 * .
 * [list of port drivers]: /docs/ports/#list-of-port-drivers
 */

#include <linux/err.h>
#include <linux/delay.h>
#include <linux/module.h>

#include <lego_port_class.h>

static ssize_t mode_show(struct device *dev, struct device_attribute *attr,
			 char *buf)
{
	struct lego_port_device *port = to_lego_port_device(dev);

	return sprintf(buf, "%s\n", port->mode_info[port->mode].name);
}

static ssize_t mode_store(struct device *dev, struct device_attribute *attr,
			  const char *buf, size_t count)
{
	struct lego_port_device *port = to_lego_port_device(dev);
	int new_mode = -1;
	int i, ret;

	for (i = 0; i < port->num_modes; i++) {
		if (sysfs_streq(buf, port->mode_info[i].name)) {
			new_mode = i;
			break;
		}
	}
	if (new_mode == -1)
		return -EINVAL;

	ret = port->set_mode(port->context, i);
	if (ret < 0)
		return ret;

	port->mode = new_mode;
	kobject_uevent(&dev->kobj, KOBJ_CHANGE);

	return count;
}

static ssize_t modes_show(struct device *dev, struct device_attribute *attr,
			  char *buf)
{
	struct lego_port_device *port = to_lego_port_device(dev);
	size_t count = 0;
	int i;

	for (i = 0; i < port->num_modes; i++)
		count += sprintf(buf + count, "%s ", port->mode_info[i].name);
	buf[count - 1] = '\n';

	return count;
}

static ssize_t driver_name_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct lego_port_device *lego_port = to_lego_port_device(dev);

	return sprintf(buf, "%s\n", lego_port->name);
}

static ssize_t port_name_show(struct device *dev, struct device_attribute *attr,
			      char *buf)
{
	struct lego_port_device *lego_port = to_lego_port_device(dev);

	return sprintf(buf, "%s\n", lego_port->port_name);
}

static ssize_t set_device_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct lego_port_device *port = to_lego_port_device(dev);
	char name[LEGO_PORT_NAME_SIZE + 1];
	int ret;

	if (!port->set_device)
		return -EOPNOTSUPP;

	strncpy(name, buf, LEGO_PORT_NAME_SIZE);
	ret = port->set_device(port->context, strstrip(name));
	if (ret < 0)
		return ret;

	return count;
}

static ssize_t status_show(struct device *dev, struct device_attribute *attr,
			  char *buf)
{
	struct lego_port_device *port = to_lego_port_device(dev);

	if (port->get_status)
		return sprintf(buf, "%s\n", port->get_status(port->context));

	return mode_show(dev, attr, buf);
}

static DEVICE_ATTR_RW(mode);
static DEVICE_ATTR_RO(modes);
static DEVICE_ATTR_RO(driver_name);
static DEVICE_ATTR_RO(port_name);
static DEVICE_ATTR_WO(set_device);
static DEVICE_ATTR_RO(status);

static struct attribute *lego_port_class_attrs[] = {
	&dev_attr_modes.attr,
	&dev_attr_mode.attr,
	&dev_attr_driver_name.attr,
	&dev_attr_port_name.attr,
	&dev_attr_set_device.attr,
	&dev_attr_status.attr,
	NULL
};

ATTRIBUTE_GROUPS(lego_port_class);

static void lego_port_release(struct device *dev)
{
}

static unsigned lego_port_class_id = 0;

int lego_port_register(struct lego_port_device *port,
		       const struct device_type *type,
		       struct device *parent)
{
	int err;

	if (!port || !port->name || !port->port_name || !type || !parent)
		return -EINVAL;

	port->dev.release = lego_port_release;
	port->dev.parent = parent;
	port->dev.class = &lego_port_class;
	port->dev.type = type;
	dev_set_name(&port->dev, "port%d", lego_port_class_id++);

	err = device_register(&port->dev);
	if (err)
		return err;

	dev_info(&port->dev, "Bound to device '%s'\n", dev_name(parent));

	return 0;
}
EXPORT_SYMBOL_GPL(lego_port_register);

void lego_port_unregister(struct lego_port_device *port)
{
	dev_info(&port->dev, "Unregistered\n");
	device_unregister(&port->dev);
}
EXPORT_SYMBOL_GPL(lego_port_unregister);

static int lego_port_dev_uevent(struct device *dev, struct kobj_uevent_env *env)
{
	struct lego_port_device *lego_port = to_lego_port_device(dev);
	int ret;

	ret = add_uevent_var(env, "LEGO_DRIVER_NAME=%s", lego_port->name);
	if (ret) {
		dev_err(dev, "failed to add uevent LEGO_DRIVER_NAME\n");
		return ret;
	}

	ret = add_uevent_var(env, "LEGO_PORT_NAME=%s", lego_port->port_name);
	if (ret) {
		dev_err(dev, "failed to add uevent LEGO_PORT_NAME\n");
		return ret;
	}

	return 0;
}

static char *lego_port_devnode(struct device *dev, umode_t *mode)
{
	return kasprintf(GFP_KERNEL, "lego-port/%s", dev_name(dev));
}

struct class lego_port_class = {
	.name		= "lego-port",
	.owner		= THIS_MODULE,
	.dev_groups	= lego_port_class_groups,
	.dev_uevent	= lego_port_dev_uevent,
	.devnode	= lego_port_devnode,
};
EXPORT_SYMBOL_GPL(lego_port_class);

static int __init lego_port_class_init(void)
{
	return class_register(&lego_port_class);
}
module_init(lego_port_class_init);

static void __exit lego_port_class_exit(void)
{
	class_unregister(&lego_port_class);
}
module_exit(lego_port_class_exit);

MODULE_DESCRIPTION("LEGO port class");
MODULE_AUTHOR("David Lechner <david@lechnology.com>");
MODULE_LICENSE("GPL");
