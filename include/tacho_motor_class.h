/*
 * Tacho motor device class
 *
 * Copyright (C) 2013-2014,2016 Ralph Hempel <rhempel@hempeldesigngroup.com>
 * Copyright (C) 2015-2016 David Lechner <david@lechnology.com>
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

/*
 * Note: run-timed is handled completely in the tacho-motor class, so
 * TM_COMMAND_RUN_TIMED is never passed to implementing drivers.
 */
enum tacho_motor_command {
	TM_COMMAND_RUN_FOREVER,
	TM_COMMAND_RUN_TO_ABS_POS,
	TM_COMMAND_RUN_TO_REL_POS,
	TM_COMMAND_RUN_TIMED,
	TM_COMMAND_RUN_DIRECT,
	TM_COMMAND_STOP,
	TM_COMMAND_RESET,
	NUM_TM_COMMAND
};

#define IS_POS_CMD(c) \
(((c) == TM_COMMAND_RUN_TO_ABS_POS) || ((c) == TM_COMMAND_RUN_TO_REL_POS))
#define IS_RUN_CMD(c) \
(((c) != TM_COMMAND_STOP) && ((c) != TM_COMMAND_RESET))
#define SUPPORTS_RAMPING(tm) ((tm)->ops->run_regulated || (tm)->ops->run_to_pos)

enum tacho_motor_stop_command {
	TM_STOP_COMMAND_COAST,
	TM_STOP_COMMAND_BRAKE,
	TM_STOP_COMMAND_HOLD,
	NUM_TM_STOP_COMMAND,
};

enum tacho_motor_state
{
	TM_STATE_RUNNING,
	TM_STATE_RAMPING,
	TM_STATE_HOLDING,
	TM_STATE_OVERLOADED,
	TM_STATE_STALLED,
	NUM_TM_STATE,
};

enum tacho_motor_motion {
	TM_MOTION_ROTATION,
	TM_MOTION_LINEAR,
	NUM_TM_MOTION,
};

struct tacho_motor_ops;

/**
 * struct tacho_motor_params - user specified parameters
 *
 * These parameters do not take effect until a command is issued, so they are
 * maintained by the tacho-motor class so that each implementing driver does
 * not have to keep track of them twice.
 *
 * @ polarity: Indicates the positive direction of the motor.
 * @ encoder_polarity: Indicates the positive direction of the shaft encoder.
 * @ duty_cycle_sp: Used only for the run-direct command.
 * @ speed_sp: Used for all run commands except run-direct.
 * @ position_sp: Used by run-to-*-pos commands.
 * @ time_sp: The time in milliseconds used by the run-timed command.
 * @ ramp_up_sp: In milliseconds.
 * @ ramp_down_sp: In milliseconds.
 * @stop_command: What to do for stop command or when run command ends.
 */
struct tacho_motor_params {
	enum dc_motor_polarity polarity;
	enum dc_motor_polarity encoder_polarity;
	int duty_cycle_sp;
	int speed_sp;
	int position_sp;
	int time_sp;
	int ramp_up_sp;
	int ramp_down_sp;
	enum tacho_motor_stop_command stop_command;
};

/**
 * struct tacho_motor_device
 *
 * Implementing drivers need to set driver_name, address, ops and supports_*.
 * The tacho motor class will set the default values for params, so if a driver
 * needs to change them, it should be done after the tacho_motor_device is
 * registered.
 */
struct tacho_motor_device {
	const char *driver_name;
	const char *address;
	const struct tacho_motor_ops const *ops;
	const struct ev3_motor_info const *info;
	void *context;
	bool supports_encoder_polarity;
	enum tacho_motor_command command;
	struct tacho_motor_params params;
	struct tacho_motor_params active_params;
	int ramp_start_speed;
	int ramp_delta_speed;
	int ramp_end_speed;
	int ramp_max_speed;
	int ramp_last_speed;
	int ramp_delta_time;
	unsigned long ramp_end_time;
	unsigned long last_ramp_work_time;
	bool ramping;
	/* private */
	struct device dev;
	struct delayed_work run_timed_work;
	struct delayed_work ramp_work;
};

/**
 * struct tacho_motor_ops - Operations that must be implemented by tacho-motor
 * 	class drivers.
 *
 * @get_position: Returns the current encoder position in tacho counts.
 * @set_position: Sets the current encoder position to the specified value.
 * 	Returns an error instead of setting the value if the motor is running.
 * @get_state: Gets the state flags for the motor.
 * @get_duty_cycle: Gets the current PWM duty cycle being sent to the motor.
 * @get_speed: Gets the current speed of the motor in tacho counts per second.
 * @run_unregulated: Sends message to the motor controller to run in unregulated
 *	mode with the specified duty cycle.
 * @run_regulated: Sends message to the motor controller to run in regulated
 *	mode with the specified speed setpoint.
 * @run_to_pos: Sends message to the motor controller to run to the specified
 *	position using speed regulation and do the specified stop action when
 *	the target position is reached.
 * @stop: Sends message to the motor controller to stop using the specified
 *	action.
 * @reset: Sends message to the motor controller to reset. This will stop the
 *	motor and reset any motor controller parameters.
 * @get_commands: Gets flags representing the commands that the driver supports.
 * @get_command: Get the active command for the motor.
 * @send_command: Sends an command to the motor controller (makes the motor do
 * 	something).
 * @get_stop_commands: Gets flags representing the valid stop commands supported
 * 	by the driver.
 * @get_speed_Kp: Gets the current proportional PID constant for the speed PID.
 * @set_speed_Kp: Sets the current proportional PID constant for the speed PID.
 * @get_speed_Ki: Gets the current integral PID constant for the speed PID.
 * @set_speed_Ki: Sets the current integral PID constant for the speed PID.
 * @get_speed_Kd: Gets the current derivative PID constant for the speed PID.
 * @set_speed_Kd: Sets the current derivative PID constant for the speed PID.
 * @get_position_Kp: Gets the current proportional PID constant for the position PID.
 * @set_position_Kp: Sets the current proportional PID constant for the position PID.
 * @get_position_Ki: Gets the current integral PID constant for the position PID.
 * @set_position_Ki: Sets the current integral PID constant for the position PID.
 * @get_position_Kd: Gets the current derivative PID constant for the position PID.
 * @set_position_Kd: Sets the current derivative PID constant for the position PID.
 */
struct tacho_motor_ops {
	int (*get_position)(void *context, long *position);
	int (*set_position)(void *context, long position);

	int (*get_state)(void *context);

	int (*get_duty_cycle)(void *context, int *duty_cycle);
	int (*get_speed)(void *context, int *speed);

	int (*run_unregulated)(void *context, int duty_cycle);
	int (*run_regulated)(void *context, int speed);
	int (*run_to_pos)(void *context, int pos, int speed,
			  enum tacho_motor_stop_command action);
	int (*stop)(void *context, enum tacho_motor_stop_command action);
	int (*reset)(void *context);

	unsigned (*get_commands)(void *context);
	int (*send_command)(void *context, struct tacho_motor_params *params,
			    enum tacho_motor_command command);
	unsigned (*get_stop_commands)(void *context);

	int (*get_speed_Kp)(void *context);
	int (*set_speed_Kp)(void *context, int k);
	int (*get_speed_Ki)(void *context);
	int (*set_speed_Ki)(void *context, int k);
	int (*get_speed_Kd)(void *context);
	int (*set_speed_Kd)(void *context, int k);

	int (*get_hold_Kp)(void *context);
	int (*set_hold_Kp)(void *context, int k);
	int (*get_hold_Ki)(void *context);
	int (*set_hold_Ki)(void *context, int k);
	int (*get_hold_Kd)(void *context);
	int (*set_hold_Kd)(void *context, int k);
};

extern void tacho_motor_notify_state_change(struct tacho_motor_device *);
extern void tacho_motor_notify_position_ramp_down(struct tacho_motor_device *);

extern int register_tacho_motor(struct tacho_motor_device *, struct device *);

extern void unregister_tacho_motor(struct tacho_motor_device *);

extern struct class tacho_motor_class;

#endif /* __LINUX_LEGOEV3_TACHO_MOTOR_CLASS_H */
