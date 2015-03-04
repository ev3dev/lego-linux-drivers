/*
 * Tacho motor device class
 *
 * Copyright (C) 2013-2014 Ralph Hempel <rhempel@hempeldesigngroup.com>
 * Copyright (C) 2015 David Lechner <david@lechnology.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.

 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef __LINUX_LEGOEV3_TACHO_MOTOR_CLASS_H
#define __LINUX_LEGOEV3_TACHO_MOTOR_CLASS_H

#include <linux/device.h>

#include <dc_motor_class.h>
#include <lego_port_class.h>

enum tacho_motor_speed_regulation {
	TM_SPEED_REGULATION_OFF,
	TM_SPEED_REGULATION_ON,
	TM_NUM_SPEED_REGULATION_MODES,
};

enum tacho_motor_command {
	TM_COMMAND_RUN_FOREVER,
	TM_COMMAND_RUN_TO_ABS_POS,
	TM_COMMAND_RUN_TO_REL_POS,
	TM_COMMAND_RUN_TIMED,
	TM_COMMAND_STOP,
	TM_COMMAND_RESET,
	NUM_TM_COMMANDS
};

#define IS_POS_CMD(command) ((command == TM_COMMAND_RUN_TO_ABS_POS) || (command == TM_COMMAND_RUN_TO_ABS_POS))
#define IS_RUN_CMD(command) ((command != TM_COMMAND_STOP) && (command != TM_COMMAND_RESET))

enum tacho_motor_stop_command {
	TM_STOP_COMMAND_COAST,
	TM_STOP_COMMAND_BRAKE,
	TM_STOP_COMMAND_HOLD,
	TM_NUM_STOP_COMMANDS,
};

enum tacho_motor_type {
	TM_TYPE_TACHO,
	TM_TYPE_MINITACHO,
	TM_NUM_TYPES,
};

enum tacho_motor_state
{
  TM_STATE_RUN_FOREVER,
  TM_STATE_SETUP_RAMP_TIME,
  TM_STATE_SETUP_RAMP_POSITION,
  TM_STATE_SETUP_RAMP_REGULATION,
  TM_STATE_RAMP_UP,
  TM_STATE_RAMP_CONST,
  TM_STATE_POSITION_RAMP_DOWN,
  TM_STATE_RAMP_DOWN,
  TM_STATE_STOP,
  TM_STATE_IDLE,
  TM_NUM_STATES,
};


struct tacho_motor_ops;

struct tacho_motor_device {
	const char *driver_name;
	const char *port_name;
	const struct tacho_motor_ops const *ops;
	/* private */
	struct device dev;
};

struct tacho_motor_ops {
	int (*get_position)(struct tacho_motor_device *tm, long *position);
	int (*set_position)(struct tacho_motor_device *tm, long position);

	int  (*get_state)(struct tacho_motor_device *tm);

	int (*get_count_per_rot)(struct tacho_motor_device *tm, int *count_per_rot);
	int (*get_duty_cycle)(struct tacho_motor_device *tm, int *duty_cycle);
	int (*get_speed)(struct tacho_motor_device *tm, int *speed);

	int (*get_duty_cycle_sp)(struct tacho_motor_device *tm, int *duty_cycle);
	int (*set_duty_cycle_sp)(struct tacho_motor_device *tm, int duty_cycle);

	int (*get_speed_sp)(struct tacho_motor_device *tm, int *speed);
	int (*set_speed_sp)(struct tacho_motor_device *tm, int speed);

	int  (*get_time_sp)(struct tacho_motor_device *tm);
	void (*set_time_sp)(struct tacho_motor_device *tm, long time_sp);

	int  (*get_position_sp)(struct tacho_motor_device *tm);
	void (*set_position_sp)(struct tacho_motor_device *tm, long position_sp);

	unsigned (*get_commands)(struct tacho_motor_device *tm);
	int (*send_command)(struct tacho_motor_device *tm, enum tacho_motor_command command);

	int (*get_speed_regulation)(struct tacho_motor_device *tm);
	int (*set_regulation_mode)(struct tacho_motor_device *tm, enum tacho_motor_speed_regulation speed_regulation);

	unsigned (*get_stop_commands)(struct tacho_motor_device *tm);
	int (*get_stop_command)(struct tacho_motor_device *tm);
	int (*set_stop_command)(struct tacho_motor_device *tm, enum tacho_motor_stop_command stop_command);

	int (*get_polarity)(struct tacho_motor_device *tm);
	int (*set_polarity)(struct tacho_motor_device *tm, enum dc_motor_polarity polarity);

	int (*get_encoder_polarity)(struct tacho_motor_device *tm);
	int (*set_encoder_polarity)(struct tacho_motor_device *tm, enum dc_motor_polarity polarity);

 	int  (*get_speed_regulation_P)(struct tacho_motor_device *tm);
 	void (*set_speed_regulation_P)(struct tacho_motor_device *tm, long speed_regulation_P);

 	int  (*get_speed_regulation_I)(struct tacho_motor_device *tm);
 	void (*set_speed_regulation_I)(struct tacho_motor_device *tm, long speed_regulation_I);

 	int  (*get_speed_regulation_D)(struct tacho_motor_device *tm);
 	void (*set_speed_regulation_D)(struct tacho_motor_device *tm, long speed_regulation_D);

 	int  (*get_ramp_up_sp)(struct tacho_motor_device *tm);
 	void (*set_ramp_up_sp)(struct tacho_motor_device *tm, long ramp_up_sp);

 	int  (*get_ramp_down_sp)(struct tacho_motor_device *tm);
 	void (*set_ramp_down_sp)(struct tacho_motor_device *tm, long ramp_down_sp);
};

extern void tacho_motor_notify_state_change(struct tacho_motor_device *);

extern int register_tacho_motor(struct tacho_motor_device *, struct device *);

extern void unregister_tacho_motor(struct tacho_motor_device *);

extern struct class tacho_motor_class;

inline struct tacho_motor_device *to_tacho_motor(struct device *dev)
{
	return container_of(dev, struct tacho_motor_device, dev);
}

#endif /* __LINUX_LEGOEV3_TACHO_MOTOR_CLASS_H */
