/*
 * Servo motor device class for LEGO MINDSTORMS EV3
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

#ifndef _LINUX_LEGOEV3_SERVO_MOTOR_CLASS_H
#define _LINUX_LEGOEV3_SERVO_MOTOR_CLASS_H

#include <linux/device.h>
#include <linux/types.h>

#define SERVO_MOTOR_NAME_SIZE	30

enum servo_motor_command {
	SERVO_MOTOR_COMMAND_RUN,
	SERVO_MOTOR_COMMAND_FLOAT,
	NUM_SERVO_MOTOR_COMMAND
};

enum servo_motor_polarity {
	SERVO_MOTOR_POLARITY_NORMAL,
	SERVO_MOTOR_POLARITY_INVERTED,
	NUM_SERVO_MOTOR_POLARITY
};

/**
 * @get_position: Return position in milliseconds or 0 to indicate the motor
 * 	is floating or negative error.
 * @set_position: Sets the motor to position. Values is the pulse width in
 * 	milliseconds or 0 to float the output. Returns 0 on success or
 * 	negative error.
 * @get_rate: Returns the rate in msec or negative error.
 * @set_rate: Sets the rate to the specified value. Returns 0 on success or
 * 	negative error.
 */
struct servo_motor_ops {
	int (*get_position)(void* context);
	int (*set_position)(void* context, int position);
	int (*get_rate)(void* context);
	int (*set_rate)(void* context, unsigned rate);
};

/**
 * struct servo_motor_device
 * @name: The name of servo controller.
 * @port_name: The name of the port that this motor is connected to.
 * @ops: Function pointers to the controller that registered this servo.
 * @context: Data struct passed back to the ops.
 * @min_pulse_ms: The size of the pulse to drive the motor to 0 degrees.
 * @mid_pulse_ms: The size of the pulse to drive the motor to 90 degrees.
 * @max_pulse_ms: The size of the pulse to drive the motor to 180 degrees.
 * @command: The current command for the motor.
 * @polarity: The polarity of the motor.
 * @position: The current position of the motor.
 */
struct servo_motor_device {
	const char *name;
	const char *port_name;
	struct servo_motor_ops ops;
	void *context;
	/* private */
	struct device dev;
	unsigned min_pulse_ms;
	unsigned mid_pulse_ms;
	unsigned max_pulse_ms;
	enum servo_motor_command command;
	enum servo_motor_polarity polarity;
	int position;
};

#define to_servo_motor_device(_dev) container_of(_dev, struct servo_motor_device, dev)

extern int register_servo_motor(struct servo_motor_device *, struct device *);
extern void unregister_servo_motor(struct servo_motor_device *);

#endif /* _LINUX_LEGOEV3_SERVO_MOTOR_CLASS_H */
