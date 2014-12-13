/*
 * DC motor device class for LEGO MINDSTORMS EV3
 *
 * Copyright (C) 2014 David Lechner <david@lechnology.com>
 * Copyright (C) 2014 Ralph Hempel <rhempel@hempeldesigngroup.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.

 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef _LINUX_LEGOEV3_DC_MOTOR_CLASS_H
#define _LINUX_LEGOEV3_DC_MOTOR_CLASS_H

#include <linux/device.h>
#include <linux/hrtimer.h>
#include <linux/types.h>

#define DC_MOTOR_NAME_SIZE	30

enum dc_motor_command {
	DC_MOTOR_COMMAND_RUN,
	DC_MOTOR_COMMAND_COAST,
	DC_MOTOR_COMMAND_BRAKE,
	NUM_DC_MOTOR_COMMANDS
};

extern const char* dc_motor_command_names[];

enum dc_motor_polarity {
	DC_MOTOR_POLARITY_NORMAL,
	DC_MOTOR_POLARITY_INVERTED,
	NUM_DC_MOTOR_POLARITY
};

extern const char* dc_motor_polarity_values[];

enum dc_motor_direction {
	DC_MOTOR_DIRECTION_FORWARD,
	DC_MOTOR_DIRECTION_REVERSE,
	NUM_DC_MOTOR_DIRECTION
};


/**
 * @get_supported_commands: Return the supported commands as bit flags.
 * @get_command: Return the current command or negative error.
 * @set_command: Set the command for the motor. Returns 0 on success or
 * 	negative error;
 * @get_direction: Return the current direction.
 * @set_direction: Set the direction for the motor. Returns 0 on success or
 * 	negative error;
 * @get_duty_cycle: Returns the current duty cycle in percent (0 to 100).
 * @set_duty_cycle: Sets the duty cycle. Returns 0 on success or negative error.
 * @context: Pointer to data structure passed back to the functions.
 */
struct dc_motor_ops {
	unsigned (*get_supported_commands)(void* context);
	enum dc_motor_command (*get_command)(void* context);
	int (*set_command)(void* context, enum dc_motor_command);
	enum dc_motor_direction (*get_direction)(void *context);
	int (*set_direction)(void *context, enum dc_motor_direction direction);
	unsigned (*get_duty_cycle)(void *context);
	int (*set_duty_cycle)(void *context, unsigned duty_cycle);
	void *context;
};

/**
 * struct dc_motor_device
 * @name: The name of dc controller.
 * @port_name: The name of the port that this motor is connected to.
 * @ops: Function pointers to the controller that registered this dc.
 * @dev: The device struct used by the class.
 * @ramp_up_ms: The time to ramp up from 0 to 100% in milliseconds.
 * @ramp_up_ms: The time to ramp down from 100 to 0% in milliseconds.
 * @ramp_timer: Timer used for ramping.
 * @current_duty_cycle: The current duty cycle.
 * @target_duty_cycle: The requested duty cycle.
 */
struct dc_motor_device {
	char name[DC_MOTOR_NAME_SIZE + 1];
	char port_name[DC_MOTOR_NAME_SIZE + 1];
	struct dc_motor_ops ops;
	struct device dev;
	enum dc_motor_polarity polarity;
	unsigned ramp_up_ms;
	unsigned ramp_down_ms;
	struct hrtimer ramp_timer;
	int current_duty_cycle;
	int target_duty_cycle;
};

#define to_dc_motor_device(_dev) container_of(_dev, struct dc_motor_device, dev)

extern int register_dc_motor(struct dc_motor_device *, struct device *);
extern void unregister_dc_motor(struct dc_motor_device *);

#endif /* _LINUX_LEGOEV3_DC_MOTOR_CLASS_H */
