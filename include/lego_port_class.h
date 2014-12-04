/*
 * LEGO port class driver
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

#ifndef _LEGO_PORT_CLASS_H_
#define _LEGO_PORT_CLASS_H_

#include <linux/device.h>
#include <linux/types.h>

#define LEGO_PORT_NAME_SIZE	30

/**
 * struct lego_port_type
 *
 * A static lego_port_type should be declared by each driver that registers
 * a port. This info is used by other driver to get information from the port.
 *
 * @name: The name of the type.
 * @ops: Pointer to type-defined callback operations.
 */
struct lego_port_type {
	const char *name;
	const void *ops;
};

/**
 * struct lego_port_mode_info
 * @name: The name of this mode.
 */
struct lego_port_mode_info {
	char name[LEGO_PORT_NAME_SIZE + 1];
};

/**
 * struct lego_port_device
 * @port_name: Name of the port.
 * @port_type: The type of the port.
 * @num_modes: The number of valid modes.
 * @mode: The current mode.
 * @mode_info: Array of mode information.
 * @set_mode: Callback to set the sensor mode.
 * @set_device: Callback to load a device attached to this port.
 * @get_status: Callback to get the status string. (optional)
 * @context: Pointer to pass back to callback functions.
 * @dev: The device data structure.
 */
struct lego_port_device {
	char port_name[LEGO_PORT_NAME_SIZE + 1];
	const struct lego_port_type *type;
	u8 num_modes;
	u8 mode;
	const struct lego_port_mode_info *mode_info;
	int (*set_mode)(void *context, u8 mode);
	int (*set_device)(void *context, const char *device_name);
	const char *(*get_status)(void *context);
	void *context;
	/* private */
	struct device dev;
};

#define to_lego_port_device(_dev) container_of(_dev, struct lego_port_device, dev)

extern int lego_port_register(struct lego_port_device *lego_port,
			      struct device *parent);
extern void lego_port_unregister(struct lego_port_device *lego_port);

extern struct class lego_port_class;

#endif /* _LEGO_PORT_CLASS_H_ */
