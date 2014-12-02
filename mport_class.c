/*
 * mindsensors.com EV3 Sensor Multiplexer device driver for LEGO MINDSTORMS EV3
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

/*
 * Note: The comment block below is used to generate docs on the ev3dev website.
 * Use kramdown (markdown) format. Use a '.' as a placeholder when blank lines
 * or leading whitespace is important for the markdown syntax.
 */

/**
 * DOC: website
 *
 * Mindstorms Port Driver
 *
 * The `mport` class provides an interface for working with input and output
 * ports that are compatible with LEGO MINDSTORMS sensors and motors. If a port
 * supports more than one type of communication, there will be multiple modes
 * available that can be manually set. Some drivers may also support automatic
 * selection of the mode.
 * .
 * ### sysfs attributes
 * .
 * Ports can be found at `/sys/class/mport/port<N>` where `<N>` is incremented
 * each time a new port is registered. Note: The number is not related to
 * the actual port at all - use the `name` attribute to find a specific port.
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
 *   this attribute to load the correct driver. Returns -ENOSYS if setting a
 *   device is not supported.
 */

#include <linux/err.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/legoev3/mport_class.h>

static ssize_t mode_show(struct device *dev, struct device_attribute *attr,
			 char *buf)
{
	struct mport_device *mport = to_mport_device(dev);

	return sprintf(buf, "%s\n", mport->mode_info[mport->mode].name);
}

static ssize_t mode_store(struct device *dev, struct device_attribute *attr,
			  const char *buf, size_t count)
{
	struct mport_device *mport = to_mport_device(dev);
	int new_mode = -1;
	int i, ret;

	for (i = 0; i < mport->num_modes; i++) {
		if (sysfs_streq(buf, mport->mode_info[i].name)) {
			new_mode = i;
			break;
		}
	}
	if (new_mode == -1)
		return -EINVAL;

	ret = mport->set_mode(mport->context, i);
	if (ret < 0)
		return ret;

	mport->mode = new_mode;

	return count;
}

static ssize_t modes_show(struct device *dev, struct device_attribute *attr,
			  char *buf)
{
	struct mport_device *mport = to_mport_device(dev);
	size_t count = 0;
	int i;

	for (i = 0; i < mport->num_modes; i++)
		count += sprintf(buf + count, "%s ", mport->mode_info[i].name);
	buf[count - 1] = '\n';

	return count;
}

static ssize_t port_name_show(struct device *dev, struct device_attribute *attr,
			      char *buf)
{
	struct mport_device *mport = to_mport_device(dev);

	return sprintf(buf, "%s\n", mport->port_name);
}

static ssize_t set_device_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct mport_device *mport = to_mport_device(dev);
	char name[MPORT_NAME_SIZE + 1];
	int ret;

	if (!mport->set_device)
		return -ENOSYS;

	strncpy(name, buf, MPORT_NAME_SIZE);
	ret = mport->set_device(mport->context, strstrip(name));
	if (ret < 0)
		return ret;

	return count;
}

static ssize_t status_show(struct device *dev, struct device_attribute *attr,
			  char *buf)
{
	struct mport_device *mport = to_mport_device(dev);

	if (mport->get_status)
		return sprintf(buf, "%s\n", mport->get_status(mport->context));

	return mode_show(dev, attr, buf);
}

static DEVICE_ATTR_RW(mode);
static DEVICE_ATTR_RO(modes);
static DEVICE_ATTR_RO(port_name);
static DEVICE_ATTR_WO(set_device);
static DEVICE_ATTR_RO(status);

static struct attribute *mport_class_attrs[] = {
	&dev_attr_modes.attr,
	&dev_attr_mode.attr,
	&dev_attr_port_name.attr,
	&dev_attr_set_device.attr,
	&dev_attr_status.attr,
	NULL
};

ATTRIBUTE_GROUPS(mport_class);

static void mport_release(struct device *dev)
{
}

static unsigned mport_class_id = 0;

int register_mport(struct mport_device *mport, struct device *parent)
{
	int err;

	if (!mport || !mport->port_name || !parent)
		return -EINVAL;

	mport->dev.release = mport_release;
	mport->dev.parent = parent;
	mport->dev.class = &mport_class;
	dev_set_name(&mport->dev, "port%d", mport_class_id++);

	err = device_register(&mport->dev);
	if (err)
		return err;

	dev_info(&mport->dev, "Bound to device '%s'\n", dev_name(parent));

	return 0;
}
EXPORT_SYMBOL_GPL(register_mport);

void unregister_mport(struct mport_device *ms)
{
	dev_info(&ms->dev, "Unregistered\n");
	device_unregister(&ms->dev);
}
EXPORT_SYMBOL_GPL(unregister_mport);

static int mport_dev_uevent(struct device *dev, struct kobj_uevent_env *env)
{
	struct mport_device *mport = to_mport_device(dev);
	int ret;

	ret = add_uevent_var(env, "NAME=%s", mport->port_name);
	if (ret) {
		dev_err(dev, "failed to add uevent NAME\n");
		return ret;
	}

	return 0;
}

static char *mport_devnode(struct device *dev, umode_t *mode)
{
	return kasprintf(GFP_KERNEL, "mport/%s", dev_name(dev));
}

struct class mport_class = {
	.name		= "mport",
	.owner		= THIS_MODULE,
	.dev_groups	= mport_class_groups,
	.dev_uevent	= mport_dev_uevent,
	.devnode	= mport_devnode,
};
EXPORT_SYMBOL_GPL(mport_class);

static int __init mport_class_init(void)
{
	int err;

	err = class_register(&mport_class);
	if (err) {
		pr_err("unable to register mport device class\n");
		return err;
	}

	return 0;
}
module_init(mport_class_init);

static void __exit mport_class_exit(void)
{
	class_unregister(&mport_class);
}
module_exit(mport_class_exit);

MODULE_DESCRIPTION("Mindstorms port device class for LEGO MINDSTORMS EV3");
MODULE_AUTHOR("David Lechner <david@lechnology.com>");
MODULE_LICENSE("GPL");
