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
 * Used by sensor drivers to get notified when a port has new raw data available.
 */
typedef void (*lego_port_notify_raw_data_func_t)(void *context);

/**
 * struct lego_port_mode_info
 * @name: The name of this mode.
 */
struct lego_port_mode_info {
	char name[LEGO_PORT_NAME_SIZE + 1];
};

enum lego_port_gpio_state {
	LEGO_PORT_GPIO_FLOAT,
	LEGO_PORT_GPIO_LOW,
	LEGO_PORT_GPIO_HIGH,
};

struct lego_port_device;

struct lego_port_nxt_analog_ops {
	int (*set_pin5_gpio)(void *context, enum lego_port_gpio_state state);
};

struct lego_port_nxt_i2c_ops {
	int (*set_pin1_gpio)(void *context, enum lego_port_gpio_state state);
};

/**
 * struct lego_port_device
 * @port_name: Name of the port.
 * @port_alias: Alternate name of port for mapping other devices such as a tty
 * 	to this port device.
 * @num_modes: The number of valid modes.
 * @mode: The current mode.
 * @mode_info: Array of mode information.
 * @set_mode: Callback to set the sensor mode.
 * @set_device: Callback to load a device attached to this port.
 * @get_status: Callback to get the status string. (optional)
 * @nxt_analog_ops: Functions used by Analog/NXT ports (optional).
 * @nxt_i2c_ops: Functions used by I2C/NXT ports (optional).
 * @motor_ops: Functions used by motor ports (optional);
 * @context: Pointer to pass back to callback functions.
 * @dev: The device data structure.
 * @raw_data: Pointer to raw data storage.
 * @raw_data_size: Size of raw_data in bytes.
 * @notify_raw_data_func: Registered by sensor drivers to be notified of new
 * 	raw data.
 * @notify_raw_data_context: Send to notify_raw_data_func as parameter.
 */
struct lego_port_device {
	char port_name[LEGO_PORT_NAME_SIZE + 1];
	const char *port_alias;
	u8 num_modes;
	u8 mode;
	const struct lego_port_mode_info *mode_info;
	int (*set_mode)(void *context, u8 mode);
	int (*set_device)(void *context, const char *device_name);
	const char *(*get_status)(void *context);
	struct lego_port_nxt_analog_ops *nxt_analog_ops;
	struct lego_port_nxt_i2c_ops *nxt_i2c_ops;
	struct dc_motor_ops *motor_ops;
	void *context;
	/* private */
	struct device dev;
	u8 *raw_data;
	unsigned raw_data_size;
	lego_port_notify_raw_data_func_t notify_raw_data_func;
	void *notify_raw_data_context;
};

#define to_lego_port_device(_dev) container_of(_dev, struct lego_port_device, dev)

extern int lego_port_register(struct lego_port_device *lego_port,
			      const struct device_type *type,
			      struct device *parent);
extern void lego_port_unregister(struct lego_port_device *lego_port);

static inline void
lego_port_set_raw_data_ptr_and_func(struct lego_port_device *port,
				    u8 *raw_data, unsigned raw_data_size,
				    lego_port_notify_raw_data_func_t func,
				    void *context)
{
	port->raw_data = raw_data;
	port->raw_data_size = raw_data_size;
	port->notify_raw_data_func = func;
	port->notify_raw_data_context = context;
}

static inline void
lego_port_call_raw_data_func(struct lego_port_device *port)
{
	if (port->notify_raw_data_func)
		port->notify_raw_data_func(port->notify_raw_data_context);
}

extern struct class lego_port_class;

#endif /* _LEGO_PORT_CLASS_H_ */
