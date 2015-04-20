/*
 * Motor Definitions for LEGO WeDo
 *
 * Copyright (C) 2014 Ralph Hempel <rhemple@hempeldesigngroup.com>
 * Copyright (C) 2014-2015 David Lechner <david@lechnology.com>
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

static void wedo_motor_update_output(struct wedo_motor_data *wmd)
{
	int output = 0;

	switch (wmd->command) {
	case DC_MOTOR_INTERNAL_COMMAND_RUN_FORWARD:
		output = wmd->duty_cycle * WEDO_OUTPUT_MAX_DUTY_CYCLE / 100;
		break;
	case DC_MOTOR_INTERNAL_COMMAND_RUN_REVERSE:
		output = -wmd->duty_cycle * WEDO_OUTPUT_MAX_DUTY_CYCLE / 100;
		break;
	case DC_MOTOR_INTERNAL_COMMAND_BRAKE:
		output = WEDO_OUTPUT_BRAKE;
		break;
	default:
		break;
	}

	wedo_port_update_output(wmd->wpd, output);
}

static unsigned wedo_motor_get_supported_commands(void* context)
{
	return BIT(DC_MOTOR_COMMAND_RUN_FOREVER) | BIT(DC_MOTOR_COMMAND_STOP);
}

static unsigned wedo_motor_get_supported_stop_commands(void* context)
{
	return BIT(DC_MOTOR_STOP_COMMAND_COAST) | BIT(DC_MOTOR_STOP_COMMAND_BRAKE);
}

static enum dc_motor_internal_command wedo_motor_get_command(void* context)
{
	struct wedo_motor_data *wmd = context;

	return wmd->command;
}

static int wedo_motor_set_command(void* context,
				  enum dc_motor_internal_command command)
{
	struct wedo_motor_data *wmd = context;

	if (wmd->command == command)
		return 0;

	wmd->command = command;
	wedo_motor_update_output(wmd);

	return 0;
}

static unsigned wedo_motor_get_duty_cycle(void *context)
{
	struct wedo_motor_data *wmd = context;

	return wmd->duty_cycle;
}

static int wedo_motor_set_duty_cycle(void *context, unsigned duty_cycle)
{
	struct wedo_motor_data *wmd = context;

	if (duty_cycle > 100)
		return -EINVAL;

	if (wmd->duty_cycle == duty_cycle)
		return 0;

	wmd->duty_cycle = duty_cycle;
	wedo_motor_update_output(wmd);

	return 0;
}

const struct dc_motor_ops wedo_motor_ops = {
	.get_supported_commands	= wedo_motor_get_supported_commands,
	.get_supported_stop_commands = wedo_motor_get_supported_stop_commands,
	.get_command		= wedo_motor_get_command,
	.set_command		= wedo_motor_set_command,
	.get_duty_cycle		= wedo_motor_get_duty_cycle,
	.set_duty_cycle		= wedo_motor_set_duty_cycle,
};
