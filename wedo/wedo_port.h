/*
 * Port definitions for LEGO WeDo
 *
 * Copyright (C) 2014 Ralph Hempel <rhemple@hempeldesigngroup.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.

 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef _LINUX_WEDO_PORT_H
#define _LINUX_WEDO_PORT_H

#include <linux/device.h>

#define WEDO_PORT_NAME_SIZE	30

enum wedo_type_id {
	WEDO_TYPE_SHORTLO,
	WEDO_TYPE_BEND,
	WEDO_TYPE_TILT,
	WEDO_TYPE_FUTURE,
	WEDO_TYPE_RAW,
	WEDO_TYPE_TOUCH,
	WEDO_TYPE_SOUND,
	WEDO_TYPE_TEMP,
	WEDO_TYPE_LIGHT,
	WEDO_TYPE_MOTION,
	WEDO_TYPE_LIGHTBRICK,
	WEDO_TYPE_22,
	WEDO_TYPE_OPEN,
	WEDO_TYPE_MOTOR,
	WEDO_TYPE_SHORTHI,
	WEDO_TYPE_MAX,
};

enum wedo_ports {
	WEDO_PORT_1,
	WEDO_PORT_2,
	WEDO_PORT_MAX,
};

enum wedo_motor_command {
	WEDO_MOTOR_COMMAND_RUN,
	WEDO_MOTOR_COMMAND_COAST,
	WEDO_MOTOR_COMMAND_BRAKE,
	NUM_WEDO_MOTOR_COMMANDS
};

enum wedo_motor_direction {
	WEDO_MOTOR_DIRECTION_FORWARD,
	WEDO_MOTOR_DIRECTION_REVERSE,
	NUM_WEDO_MOTOR_DIRECTION
};

struct wedo_port_device {
	struct device dev;
	char port_name[WEDO_PORT_NAME_SIZE + 1];

	int type_debounce;
	enum wedo_type_id temp_type_id;

	enum wedo_type_id type_id;

	unsigned char id;
	unsigned char input;

	enum wedo_motor_command command;
	enum wedo_motor_direction direction;
	unsigned duty_cycle;
};
#define to_wedo_port_device(_dev) container_of(_dev, struct wedo_port_device, dev)

struct wedo_hub_device;

extern struct wedo_port_device *register_wedo_port(unsigned port_num, struct wedo_hub_device *);
extern void unregister_wedo_port(struct wedo_port_device *);

extern void wedo_port_update_status(struct wedo_port_device *);

#endif /* _LINUX_WEDO_PORT_H */
