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

#ifndef __LEGO_H
#define __LEGO_H

#include <linux/mod_devicetable.h>
#include <linux/device.h>
#include <lego_port_class.h>

#define LEGO_NAME_SIZE LEGO_PORT_NAME_SIZE

struct lego_device_id {
	char name[LEGO_NAME_SIZE + 1];
	kernel_ulong_t driver_data;
};

struct lego_device {
	struct device dev;
	char name[LEGO_NAME_SIZE + 1];
	struct lego_port_device *port;
	const struct lego_device_id *entry_id;
};

static inline struct lego_device *to_lego_device(struct device *dev)
{
	return dev ? container_of(dev, struct lego_device, dev) : NULL;
}

extern struct lego_device *lego_device_register(const char *name,
						const struct device_type *type,
						struct lego_port_device *port,
						void *platform_data,
						size_t platform_data_size);
extern void lego_device_unregister(struct lego_device *ldev);

#define LEGO_DEVICE_ID(_name, _driver_data)	\
	{					\
		.name = _name,			\
		.type_id = _driver_data,	\
	}

struct lego_device_driver {
	int (*probe)(struct lego_device *ldev);
	int (*remove)(struct lego_device *ldev);
	void (*shutdown)(struct lego_device *ldev);
	struct device_driver driver;
	const struct lego_device_id *id_table;
};

static inline struct lego_device_driver
*to_lego_device_driver(struct device_driver *drv)
{
	return drv ? container_of(drv, struct lego_device_driver, driver) : NULL;
}

extern int lego_device_driver_register(struct lego_device_driver *ldrv);
extern void lego_device_driver_unregister(struct lego_device_driver *ldrv);
#define lego_device_driver(driver) \
module_driver(driver, lego_device_driver_register, lego_device_driver_unregister);

extern struct bus_type lego_bus_type;

#endif /* __LEGO_H */
