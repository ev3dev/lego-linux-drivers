/*
 * Hub functions for LEGO WeDo
 *
 * Copyright (C) 2014 Ralph Hempel <rhemple@hempeldesigngroup.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
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
 * WeDo Hub
 *
 * The LEGO WeDo hub device provide a high level interface to the WeDo.
 * Each hub has two ports - `port0` and `port1` - that are capable of
 * autodetecting WeDo compatible devices that are connected to the WeDo hub.
 * .
 * The WeDo hub was designed to be a HID device, and the `hidraw` driver
 * will automatically bind to the WeDo hub. This behaviour can be
 * overridden with the following udev rule:
 * .
 * .    SUBSYSTEM=="usb", ATTRS{idVendor}=="0694", ATTRS{idProduct}=="0003" ACTION=="add", \$
 * .        RUN+="/bin/sh -c 'echo $kernel > /sys/bus/usb/drivers/usbhid/unbind;           \$
 * .                          echo $kernel > /sys/bus/usb/drivers/legowedo/bind'"
 * .
 * ### sysfs Attributes
 * .
 * WeDo `hub` devices can be found at `/sys/bus/wedo/devices/hub<N>`, where `<N>`
 * is incremented each time a WeDo hub is plugged in to a USB port (it is
 * not related to which USB port the hub is plugged in to).
 *
 * Use the `dmesg` log to determine which USB port particular hub device is
 * plugged into.
 * .
 * `error` (read-only)
 * : Returns a 1 if the output current source has detected an error - most
 *   likely due to a stalled motor. If everything is OK, 0 is returned.
 * .
 * `clear_error` (write-only)
 * : Clears an error condition. Write a 1 to begin the clear operation. It
 *   is not necessary to write a 0 to stop clearing the error.
 * .
 * `high_power` (read/write)
 * : Sets or clears the high power (500 mA) capability of the output port. A
 *   value of 1 will request 500 mA from the USB port that the WeDo is
 *   plugged into. Returns 1 of the USB port is able to supply 500 mA to the
 *   WeDo hub, 0 if only 100 mA is available.
 * .
 * `shut_down` (write-only)
 * : Sets or clears the shut_down state of the WeDo hub. Writing 1 to this
 *   attribute turns off the output driver for both ports, writing a 0
 *   turns the output port back on.
 * .
 * `reset` (write-only)
 * : Sets or clears the reset state of the WeDo hub. Writing 1 to this
 *   attribute sets the output state to off and clears any errors.
 * .
 * `voltage` (read-only)
 * : Returns the voltage being supplied to the WeDo hub by the USB port, in
 *   millivolts. The nominal value is 5000.
 */

#include <linux/device.h>

#include "wedo_hub.h"
#include "wedo_port.h"

static ssize_t clear_error_store(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t count)
{
	struct wedo_hub_device *whd = to_wedo_hub_device(dev);
	int value;

	if (sscanf(buf, "%d", &value) != 1 || value < 0 || value > 1)
		return -EINVAL;
	whd->to_hub.status.clear_error = value;
	whd->event_callback (whd);

	return count;
}

static ssize_t high_power_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct wedo_hub_device *whd = to_wedo_hub_device(dev);
	int value;

	if (sscanf(buf, "%d", &value) != 1 || value < 0 || value > 1)
		return -EINVAL;
	whd->to_hub.status.high_power = value;
	whd->event_callback (whd);

	return count;
}

static ssize_t high_power_show(struct device *dev,
			 struct device_attribute *attr, char *buf)
{
	struct wedo_hub_device *whd = to_wedo_hub_device(dev);

	return sprintf(buf, "%d\n", whd->from_hub.status.high_power);
}

static ssize_t shut_down_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct wedo_hub_device *whd = to_wedo_hub_device(dev);
	int value;

	if (sscanf(buf, "%d", &value) != 1 || value < 0 || value > 1)
		return -EINVAL;
	whd->to_hub.status.shut_down = value;
	whd->event_callback (whd);

	return count;
}

static ssize_t reset_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct wedo_hub_device *whd = to_wedo_hub_device(dev);
	int value;

	if (sscanf(buf, "%d", &value) != 1 || value < 0 || value > 1)
		return -EINVAL;
	whd->to_hub.status.reset = value;
	whd->event_callback (whd);

	return count;
}

static ssize_t error_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct wedo_hub_device *whd = to_wedo_hub_device(dev);

	return sprintf(buf, "%d\n", whd->from_hub.status.error);
}

static ssize_t voltage_show(struct device *dev,
			 struct device_attribute *attr, char *buf)
{
	struct wedo_hub_device *whd = to_wedo_hub_device(dev);

	return sprintf(buf, "%d\n", whd->from_hub.voltage * 49);
}

static DEVICE_ATTR_WO(clear_error);
static DEVICE_ATTR_RW(high_power);
static DEVICE_ATTR_WO(shut_down);
static DEVICE_ATTR_WO(reset);
static DEVICE_ATTR_RO(error);
static DEVICE_ATTR_RO(voltage);

static struct attribute *wedo_hub_device_attrs[] = {
	&dev_attr_clear_error.attr,
	&dev_attr_high_power.attr,
	&dev_attr_shut_down.attr,
	&dev_attr_reset.attr,
	&dev_attr_error.attr,
	&dev_attr_voltage.attr,
	NULL
};

static struct attribute_group wedo_hub_attribute_group = {
	.attrs = wedo_hub_device_attrs
};

void wedo_hub_update_status(struct wedo_hub_device *whd)
{
}

struct bus_type wedo_bus_type = {
	.name		= "wedo",
};

static void wedo_hub_release(struct device *dev)
{
}

static unsigned wedo_hub_id = 0;

int register_wedo_hub(struct wedo_hub_device *whd, struct device *parent)
{
	int err;
	int i = 0;

	if (!whd || !whd->port_name || !parent)
		return -EINVAL;

	whd->dev.release = wedo_hub_release;
	whd->dev.parent = parent;
	whd->dev.bus = &wedo_bus_type;
	dev_set_name(&whd->dev, "hub%d", wedo_hub_id++);

	err = device_register(&whd->dev);
	if (err)
		return err;

	dev_info(&whd->dev, "Bound '%s' to '%s'\n", dev_name(&whd->dev), whd->port_name);

	err = sysfs_create_group(&whd->dev.kobj, &wedo_hub_attribute_group);
	if (err)
		goto err_sysfs_create_group;

	do {
		whd->wpd[i] = register_wedo_port( i, whd );
		if (IS_ERR(whd->wpd[i])) {
			err = PTR_ERR(whd->wpd[i]);
			goto err_register_wedo_ports;
		}

	} while (++i < WEDO_PORT_MAX);

	return 0;

err_register_wedo_ports:
	while (i--)
		unregister_wedo_port(whd->wpd[i]);

	sysfs_remove_group(&whd->dev.kobj, &wedo_hub_attribute_group);
err_sysfs_create_group:
	device_unregister(&whd->dev);
	return err;
}

void unregister_wedo_hub(struct wedo_hub_device *whd)
{
	int i;

	for (i=0; i<WEDO_PORT_MAX; ++i) {
		unregister_wedo_port(whd->wpd[i]);
	};

	sysfs_remove_group(&whd->dev.kobj, &wedo_hub_attribute_group);

	dev_info(&whd->dev, "Unregistered\n");
	device_unregister(&whd->dev);
}
