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
 * The Power Functions Servo Motor (#88004) is normally intended
 * to be used with the IR Remote Control Receiver (#8884) or the
 * Rechargeable Battery Box (8878).
 *
 * This driver allows the motor to be used with the WeDo as well.
 *
 * The following descriptive text is taken from:
 *
 * <http://powerfunctions.lego.com/en-us/ElementSpecs/88004.aspx#88004>
 *
 * The output can, from its center position (vertical), turn up to 90 degrees
 * clockwise and counter-clockwise with 7 steps in each direction. In total
 * you have 15 positions:
 *
 * - 1 center position
 * - 7 positions clockwise
 * - 7 positions counter-clockwise
 * 
 * When the Servo Motor is controlled with full power in either direction
 * it will turn to the full 90 degrees position (for example with the IR
 * Remote Control).
 * 
 * When it is controlled with power steps in either direction it will turn
 * through the 7 positions corresponding to the 7 power levels (for example
 * with the IR Speed Remote Control or the Rechargeable Battery Box).
 * 
 * The Servo Motor delivers a maximum torque of 250 mNm (300 mA). Without
 * load it rotates with 360 degrees per second. This corresponds to the
 * output turning from center to horizontal position in 0,25 seconds.
 * 
 * The current consumption will depend heavily on the load it is driving.
 * Under normal conditions it can be around 150 mA and it should never
 * exceed 300 mA.
 */

static void wedo_servo_update_output(struct wedo_servo_data *wsd)
{
	int output = 0;

	switch (wsd->command) {
	case SERVO_MOTOR_COMMAND_RUN:
		output = wsd->scaled_position;
                break;
	case SERVO_MOTOR_COMMAND_FLOAT:
		output = 0x80;
                break;
	default:
		break;
	}

	wedo_port_update_output(wsd->wpd, output);
}

static int wedo_servo_get_position(void *context)
{
	struct wedo_servo_data *wsd = context;

	return wsd->scaled_position;
}

static int wedo_servo_set_position(void *context, int scaled_position )
{
	struct wedo_servo_data *wsd = context;

	if (wsd->scaled_position == scaled_position)
		return 0;

	wsd->scaled_position = scaled_position;

	wedo_servo_update_output(wsd);

	return 0;
}

const struct servo_motor_ops wedo_servo_ops = {
	.get_position		= wedo_servo_get_position,
	.set_position		= wedo_servo_set_position,
};

