/*
 * Motor Definitions for LEGO WeDo
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

#include "wedo_hub.h"
#include "wedo_port.h"
#include "wedo_motor.h"

unsigned wedo_get_supported_commands (void* context)
{
	return BIT(DC_MOTOR_COMMAND_RUN) | BIT(DC_MOTOR_COMMAND_COAST)
		| BIT(DC_MOTOR_COMMAND_BRAKE);
}

void wedo_update_output(struct wedo_port_device *wpd)
{
	struct wedo_hub_device *whd = to_wedo_hub_device (wpd->dev.parent);

	whd->event_callback (whd);
}

static enum wedo_motor_command to_wedo_motor_command[NUM_DC_MOTOR_COMMANDS] = {
	[DC_MOTOR_COMMAND_RUN]	= WEDO_MOTOR_COMMAND_RUN,
	[DC_MOTOR_COMMAND_COAST]= WEDO_MOTOR_COMMAND_COAST,
	[DC_MOTOR_COMMAND_BRAKE]= WEDO_MOTOR_COMMAND_BRAKE,
};

unsigned wedo_get_command (void* context)
{
	struct wedo_motor_data *wmd = context;

	return wmd->command;
}

int wedo_set_command (void* context, unsigned command)
{
	struct wedo_motor_data *wmd = context;

	if (wmd->command == command)
		return 0;

	wmd->command = command;

	if (NUM_DC_MOTOR_COMMANDS > command)
		wmd->wpd->command = to_wedo_motor_command[command];

	wedo_update_output( wmd->wpd );

	return 0;
}

static enum wedo_motor_direction to_wedo_motor_direction[NUM_DC_MOTOR_POLARITY] = {
	[DC_MOTOR_DIRECTION_FORWARD]	= WEDO_MOTOR_DIRECTION_FORWARD,
	[DC_MOTOR_DIRECTION_REVERSE]	= WEDO_MOTOR_DIRECTION_REVERSE,
};

unsigned wedo_get_direction (void *context)
{
	struct wedo_motor_data *wmd = context;

	return wmd->direction;
}

int wedo_set_direction (void *context, enum dc_motor_direction direction)
{
	struct wedo_motor_data *wmd = context;

	if (wmd->direction == direction)
		return 0;

	wmd->direction = direction;

	if (NUM_DC_MOTOR_DIRECTION > direction)
		wmd->wpd->direction = to_wedo_motor_direction[direction];

	wedo_update_output( wmd->wpd );

	return 0;
}

unsigned wedo_get_duty_cycle (void *context)
{
	struct wedo_motor_data *wmd = context;

	return wmd->wpd->duty_cycle;
}

int wedo_set_duty_cycle (void *context, unsigned duty_cycle)
{
	struct wedo_motor_data *wmd = context;

	if (duty_cycle > 100)
		return -EINVAL;

	if (wmd->wpd->duty_cycle == duty_cycle)
		return 0;

	wmd->wpd->duty_cycle = duty_cycle;

	wedo_update_output( wmd->wpd );

	return 0;
}

const struct dc_motor_ops wedo_motor_ops = {
	.get_supported_commands	= wedo_get_supported_commands,
	.get_command		= wedo_get_command,
	.set_command		= wedo_set_command,
	.get_direction		= wedo_get_direction,
	.set_direction		= wedo_set_direction,
	.get_duty_cycle		= wedo_get_duty_cycle,
	.set_duty_cycle		= wedo_set_duty_cycle,
};
