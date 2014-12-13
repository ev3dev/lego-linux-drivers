/*
 * Tacho motor device class for LEGO Mindstorms EV3
 *
 * Copyright (C) 2013-2014 Ralph Hempel ,rhempel@hempeldesigngroup.com>
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

#define TACHO_MOTOR_PORT_NAME_SIZE 30

enum tacho_motor_regulation_mode {
	TM_REGULATION_OFF,
	TM_REGULATION_ON,
	TM_NUM_REGULATION_MODES,
};

enum tacho_motor_stop_mode {
	TM_STOP_COAST,
	TM_STOP_BRAKE,
	TM_STOP_HOLD,
	TM_NUM_STOP_MODES,
};

enum tacho_motor_position_mode {
	TM_POSITION_ABSOLUTE,
	TM_POSITION_RELATIVE,
	TM_NUM_POSITION_MODES,
};

enum tacho_motor_run_mode {
	TM_RUN_FOREVER,
	TM_RUN_TIME,
	TM_RUN_POSITION,
	TM_NUM_RUN_MODES,
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


struct function_pointers;

struct tm_log {
		unsigned int index;
		unsigned long timestamp[128];
		unsigned int  event[128];
		unsigned int  data[128];
	};

struct tacho_motor_device {
	char port_name[TACHO_MOTOR_PORT_NAME_SIZE + 1];
	const struct function_pointers const *fp;
	/* private */
	struct device dev;

	struct tm_log log;
};

struct function_pointers {
	int  (*get_type)(struct tacho_motor_device *tm);
	void (*set_type)(struct tacho_motor_device *tm, long type);

	int  (*get_position)(struct tacho_motor_device *tm);
	void (*set_position)(struct tacho_motor_device *tm, long position);

	int  (*get_state)(struct tacho_motor_device *tm);

	int  (*get_duty_cycle)(struct tacho_motor_device *tm);

	int  (*get_pulses_per_second)(struct tacho_motor_device *tm);

	int  (*get_duty_cycle_sp)(struct tacho_motor_device *tm);
	void (*set_duty_cycle_sp)(struct tacho_motor_device *tm, long duty_cycle_sp);

	int  (*get_pulses_per_second_sp)(struct tacho_motor_device *tm);
	void (*set_pulses_per_second_sp)(struct tacho_motor_device *tm, long pulses_per_second_sp);

	int  (*get_time_sp)(struct tacho_motor_device *tm);
	void (*set_time_sp)(struct tacho_motor_device *tm, long time_sp);

	int  (*get_position_sp)(struct tacho_motor_device *tm);
	void (*set_position_sp)(struct tacho_motor_device *tm, long position_sp);

	int  (*get_run_mode)(struct tacho_motor_device *tm);
	void (*set_run_mode)(struct tacho_motor_device *tm, long run_mode);

 	int  (*get_regulation_mode)(struct tacho_motor_device *tm);
 	void (*set_regulation_mode)(struct tacho_motor_device *tm, long regulation_mode);

 	int  (*get_stop_mode)(struct tacho_motor_device *tm);
 	void (*set_stop_mode)(struct tacho_motor_device *tm, long stop_mode);

 	int  (*get_position_mode)(struct tacho_motor_device *tm);
 	void (*set_position_mode)(struct tacho_motor_device *tm, long position_mode);

 	int  (*get_polarity_mode)(struct tacho_motor_device *tm);
 	void (*set_polarity_mode)(struct tacho_motor_device *tm, long polarity_mode);

 	int  (*get_encoder_mode)(struct tacho_motor_device *tm);
 	void (*set_encoder_mode)(struct tacho_motor_device *tm, long encoder_mode);

 	int  (*get_speed_regulation_P)(struct tacho_motor_device *tm);
 	void (*set_speed_regulation_P)(struct tacho_motor_device *tm, long speed_regulation_P);

 	int  (*get_speed_regulation_I)(struct tacho_motor_device *tm);
 	void (*set_speed_regulation_I)(struct tacho_motor_device *tm, long speed_regulation_I);

 	int  (*get_speed_regulation_D)(struct tacho_motor_device *tm);
 	void (*set_speed_regulation_D)(struct tacho_motor_device *tm, long speed_regulation_D);

 	int  (*get_speed_regulation_K)(struct tacho_motor_device *tm);
 	void (*set_speed_regulation_K)(struct tacho_motor_device *tm, long speed_regulation_K);

 	int  (*get_ramp_up_sp)(struct tacho_motor_device *tm);
 	void (*set_ramp_up_sp)(struct tacho_motor_device *tm, long ramp_up_sp);

 	int  (*get_ramp_down_sp)(struct tacho_motor_device *tm);
 	void (*set_ramp_down_sp)(struct tacho_motor_device *tm, long ramp_down_sp);
 
	int  (*get_run)(struct tacho_motor_device *tm);
	void (*set_run)(struct tacho_motor_device *tm, long run);

	int  (*get_estop)(struct tacho_motor_device *tm);
	void (*set_estop)(struct tacho_motor_device *tm, long estop);

	void (*set_reset)(struct tacho_motor_device *tm, long reset);
};

extern void tacho_motor_notify_state_change(struct tacho_motor_device *);

extern int register_tacho_motor(struct tacho_motor_device *, struct device *);

extern void unregister_tacho_motor(struct tacho_motor_device *);

extern struct class tacho_motor_class;

#endif /* __LINUX_LEGOEV3_TACHO_MOTOR_CLASS_H */
