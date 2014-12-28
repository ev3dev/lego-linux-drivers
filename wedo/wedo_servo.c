/*
 * Servo Definitions for LEGO WeDo
 *
 * Copyright (C) 2014 Ralph Hempel <rhemple@hempeldesigngroup.com>
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

#include "wedo.h"

/*
 * The WeDo motor only has the 5V drive voltage of a USB port
 * and no ability to regulate the speed. We can modulate the
 * speed, but regulation requires velocity or position feedback
 * which the PF motor does not have
 *
 * The port accepts values from 0 to 127 for output level, but
 * the motor won't actually start turning until the value is
 * around 30.
 *
 * The duty_cycle value of 1-100 will be scaled from 1-127.
 *
 * The DIRECTION and duty cycle determine how the motor turns
 *
 * A duty cycle of 0 when the motor is executing the "run"
 * command sets the output equivalent to "coast" or 0x00.
 *
 * If the motor is set to "brake" then the output is set to 0x80.
 *
 * Note well that the power level to the motor is contained in
 * the lower 7 bits of the value that is sent to the hub. The
 * high bit is the direction when the motor is running and the
 * brake bit when the duty cycle is 0.
 */

static void wedo_servo_update_output(struct wedo_servo_data *wsd)
{
	int output = 0;

	switch (wsd->command) {
	case SERVO_MOTOR_COMMAND_RUN:
		output = (wsd->raw_position * WEDO_OUTPUT_MAX_DUTY_CYCLE) / 100;
                break;
	default:
		break;
	}

	wedo_port_update_output(wsd->wpd, output);
}

/*
static unsigned wedo_servo_get_supported_commands(void* context)
{
	return BIT(SERVO_MOTOR_COMMAND_RUN);
}

static unsigned wedo_servo_get_command(void* context)
{
	struct wedo_servo_data *wsd = context;

	return wsd->command;
}

static int wedo_servo_set_command(void* context, unsigned command)
{
	struct wedo_servo_data *wsd = context;

	if (wsd->command == command)
		return 0;

	wsd->command = command;

	wedo_servo_update_output(wsd);

	return 0;
}
*/

static int wedo_servo_get_position(void *context)
{
	struct wedo_servo_data *wsd = context;

	return wsd->scaled_position;
}

static int wedo_servo_set_position(void *context,
				    int scaled_position,
				    int raw_position)
{
	struct wedo_servo_data *wsd = context;

	if (wsd->scaled_position == scaled_position)
		return 0;

	wsd->scaled_position = scaled_position;
	wsd->raw_position = raw_position;

	wedo_servo_update_output(wsd);

	return 0;
}

static int wedo_servo_get_rate(void *context)
{
	return -ENOSYS;
}

static int wedo_servo_set_rate(void *context, unsigned duty_cycle)
{
	return -ENOSYS;
}

struct servo_motor_ops wedo_servo_ops = {
	.get_position		= wedo_servo_get_position,
	.set_position		= wedo_servo_set_position,
//	.get_rate		= wedo_servo_get_rate,
//	.set_rate		= wedo_servo_set_rate,
};

