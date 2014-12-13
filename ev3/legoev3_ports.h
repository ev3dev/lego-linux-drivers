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

#ifndef __LINUX_LEGOEV3_PORTS_H
#define __LINUX_LEGOEV3_PORTS_H

#include <linux/device.h>
#include <linux/hrtimer.h>
#include <linux/platform_device.h>

#include "ev3_input_port.h"
#include "ev3_output_port.h"

#define LEGOEV3_PORT_NAME_SIZE 30

struct legoev3_port_in_ops {
	int (*get_pin1_mv)(struct legoev3_port *);
	int (*get_pin6_mv)(struct legoev3_port *);
	void (*set_pin1_gpio)(struct legoev3_port *, enum ev3_input_port_gpio_state);
	void (*set_pin5_gpio)(struct legoev3_port *, enum ev3_input_port_gpio_state);
	void (*register_analog_cb)(struct legoev3_port *, legoev3_analog_cb_func_t, void *);
};

/**
 * struct legoev3_port
 * @name: The sysfs name of the port (e.g. "in").
 * @id: The sysfs id of the port.
 * @dev: The sysfs device structure for the port.
 * @in_ops: Operations for input ports. (To be assigned by the driver).
 */
struct legoev3_port {
	char name[LEGOEV3_PORT_NAME_SIZE + 1];
	int id;
	struct device dev;
	struct legoev3_port_in_ops in_ops;
};

extern struct legoev3_port
*legoev3_port_register(const char *name, int id, const struct device_type *type,
		       struct device *parent, void *platform_data,
		       size_t platform_data_size);
extern void legoev3_port_unregister(struct legoev3_port *pdev);

struct legoev3_port_device_id;

struct legoev3_port_device {
	struct device dev;
	char name[LEGOEV3_PORT_NAME_SIZE + 1];
	int id;
	struct legoev3_port *port;
	const struct legoev3_port_device_id *entry_id;
};

static inline struct legoev3_port *to_legoev3_port(struct device *dev)
{
	return dev ? container_of(dev, struct legoev3_port, dev) : NULL;
}

static inline struct legoev3_port_device
*to_legoev3_port_device(struct device *dev)
{
	return dev ? container_of(dev, struct legoev3_port_device, dev) : NULL;
}

extern int legoev3_port_device_uevent(struct device *dev,
				      struct kobj_uevent_env *env);
extern struct legoev3_port_device
*legoev3_port_device_register(const char *name, struct device_type *type,
			      struct device *parent, void *platform_data,
			      size_t platform_data_size,
			      struct legoev3_port *port);
extern void legoev3_port_device_unregister(struct legoev3_port_device *pdev);

struct legoev3_port_device_id {
	char name[LEGOEV3_PORT_NAME_SIZE];
	kernel_ulong_t driver_data;
};

#define LEGOEV3_PORT_DEVICE_ID(_type_name, _driver_data)	\
	{							\
		.name = _type_name,				\
		.type_id = _driver_data,			\
	}

struct legoev3_port_driver {
	int (*probe)(struct legoev3_port *);
	int (*remove)(struct legoev3_port *);
	void (*shutdown)(struct legoev3_port *);
	struct device_driver driver;
};

static inline struct legoev3_port_driver
*to_legoev3_port_driver(struct device_driver *drv)
{
	return drv ? container_of(drv, struct legoev3_port_driver, driver) : NULL;
}

extern int legoev3_register_port_driver(struct legoev3_port_driver *);
extern void legoev3_unregister_port_driver(struct legoev3_port_driver *);
#define legoev3_port_driver(driver) \
module_driver(driver, legoev3_register_port_driver, legoev3_unregister_port_driver);

struct legoev3_port_device_driver {
	int (*probe)(struct legoev3_port_device *);
	int (*remove)(struct legoev3_port_device *);
	void (*shutdown)(struct legoev3_port_device *);
	struct device_driver driver;
	struct legoev3_port_device_id *id_table;
};

static inline struct legoev3_port_device_driver
*to_legoev3_port_device_driver(struct device_driver *drv)
{
	return drv ? container_of(drv, struct legoev3_port_device_driver, driver) : NULL;
}

extern int legoev3_register_port_device_driver(struct legoev3_port_device_driver *);
extern void legoev3_unregister_port_device_driver(struct legoev3_port_device_driver *);
#define legoev3_port_device_driver(driver) \
module_driver(driver, legoev3_register_port_device_driver, legoev3_unregister_port_device_driver);

extern struct attribute_group legoev3_port_device_type_attr_grp;
extern struct bus_type legoev3_bus_type;

#endif /* __LINUX_LEGOEV3_PORTS_H */
