/*
 * DC motor device class for LEGO MINDSTORMS EV3
 *
 * Copyright (C) 2014-2015 David Lechner <david@lechnology.com>
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
	DC_MOTOR_COMMAND_RUN_FOREVER,
	DC_MOTOR_COMMAND_RUN_TIMED,
	DC_MOTOR_COMMAND_STOP,
	NUM_DC_MOTOR_COMMANDS
};

#define IS_DC_MOTOR_RUN_COMMAND(cmd) \
	(cmd == DC_MOTOR_COMMAND_RUN_FOREVER || cmd == DC_MOTOR_COMMAND_RUN_TIMED)

/*
 * These commands are used internally by drivers that implement the dc-motor class.
 */
enum dc_motor_internal_command {
	DC_MOTOR_INTERNAL_COMMAND_RUN_FORWARD,
	DC_MOTOR_INTERNAL_COMMAND_RUN_REVERSE,
	DC_MOTOR_INTERNAL_COMMAND_COAST,
	DC_MOTOR_INTERNAL_COMMAND_BRAKE,
	NUM_DC_MOTOR_INTERNAL_COMMAND
};

#define IS_DC_MOTOR_INTERNAL_RUN_COMMAND(cmd) \
	(cmd == DC_MOTOR_INTERNAL_COMMAND_RUN_FORWARD || cmd == DC_MOTOR_INTERNAL_COMMAND_RUN_REVERSE)

enum dc_motor_stop_command {
	DC_MOTOR_STOP_COMMAND_COAST,
	DC_MOTOR_STOP_COMMAND_BRAKE,
	NUM_DC_MOTOR_STOP_COMMANDS
};

enum dc_motor_polarity {
	DC_MOTOR_POLARITY_NORMAL,
	DC_MOTOR_POLARITY_INVERTED,
	NUM_DC_MOTOR_POLARITY
};

extern const char *dc_motor_polarity_values[];

enum dc_motor_state {
	DC_MOTOR_STATE_RUNNING,
	DC_MOTOR_STATE_RAMPING,
	NUM_DC_MOTOR_STATE
};

/**
 * @duty_cycle_sp: The requested duty cycle.
 * @polarity: Indicates which direction is the positive direction of rotation.
 * @ramp_up_sp: The time to ramp up from 0 to 100% in milliseconds.
 * @ramp_down_sp: The time to ramp down from 100 to 0% in milliseconds.
 * @stop_command: Indicates the behavior when the stop command is sent.
 * @time_sp: The time in milliseconds used by the run-timed command.
 */
struct dc_motor_params {
	int duty_cycle_sp;
	enum dc_motor_polarity polarity;
	int ramp_up_sp;
	int ramp_down_sp;
	enum dc_motor_stop_command stop_command;
	int time_sp;
};

/**
 * @get_supported_commands: Return the supported commands as bit flags.
 * @get_supported_stop_commands: Return the supported stop commands as bit flags.
 * @get_command: Get the active command for the motor.
 * @set_command: Set the command for the motor. Returns 0 on success or negative error.
 * @get_duty_cycle: Gets the current duty cycle in percent.
 * @set_duty_cycle: Sets the duty cycle. Returns 0 on success or negative error.
 */
struct dc_motor_ops {
	unsigned (*get_supported_commands)(void* context);
	unsigned (*get_supported_stop_commands)(void* context);
	enum dc_motor_internal_command (*get_command)(void* context);
	int (*set_command)(void* context, enum dc_motor_internal_command);
	unsigned (*get_duty_cycle)(void *context);
	int (*set_duty_cycle)(void *context, unsigned duty_cycle);
};

/**
 * struct dc_motor_device
 * @name: The name of the driver that loaded this device.
 * @port_name: The name of the port that this motor is connected to.
 * @ops: Function pointers to the controller that registered this dc.
 * @context: Pointer to data structure passed back to the ops functions.
 * @dev: The device struct used by the class.
 * @parms: The parameters set via sysfs attributes.
 * @active_params: Copy of params updated when command is sent.
 * @ramp_work: For ramp callbacks.
 * @run_timed_work: For run-timed command callback;
 * @duty_cycle: The current requested duty cycle.
 */
struct dc_motor_device {
	const char *name;
	const char *port_name;
	const struct dc_motor_ops *ops;
	void *context;
	/* private */
	struct device dev;
	struct dc_motor_params params;
	struct dc_motor_params active_params;
	enum dc_motor_command command;
	int ramp_delta_duty_cycle;
	int ramp_delta_time;
	unsigned long ramp_end_time;
	unsigned long last_ramp_work_time;
	struct delayed_work ramp_work;
	struct delayed_work run_timed_work;
	int duty_cycle;
};

#define to_dc_motor_device(_dev) container_of(_dev, struct dc_motor_device, dev)

extern int register_dc_motor(struct dc_motor_device *, struct device *);
extern void unregister_dc_motor(struct dc_motor_device *);

#endif /* _LINUX_LEGOEV3_DC_MOTOR_CLASS_H */
