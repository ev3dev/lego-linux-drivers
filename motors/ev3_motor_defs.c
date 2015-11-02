/*
 * Motor driver for LEGO MINDSTORMS EV3
 *
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

#include <linux/module.h>

#include "ev3_motor.h"

const struct ev3_motor_info ev3_motor_defs[] = {
	[LEGO_NXT_MOTOR] = {
		.name			= LEGO_NXT_MOTOR_NAME,
		.max_speed		= 1200,
		.count_per_rot		= 360,
		.legoev3_info		= {
			.samples_for_speed	= { 4, 16, 32, 64 },
			.speed_pid_k		= { .p = 1000, .i = 60, .d = 0 },
			.max_us_per_sample	= 100000,
		},
	},
	[LEGO_EV3_LARGE_MOTOR] = {
		.name			= LEGO_EV3_LARGE_MOTOR_NAME,
		.max_speed		= 1200,
		.count_per_rot		= 360,
		.legoev3_info		= {
			.samples_for_speed	= { 4, 16, 32, 64 },
			.speed_pid_k		= { .p = 1000, .i = 60, .d = 0 },
			.max_us_per_sample	= 100000,
		},
	},
	[LEGO_EV3_MEDIUM_MOTOR] = {
		.name			= LEGO_EV3_MEDIUM_MOTOR_NAME,
		.max_speed		= 900,
		.count_per_rot		= 360,
		.legoev3_info		= {
			.samples_for_speed	= { 2, 4, 8, 16 },
			.speed_pid_k		= { .p = 1000, .i = 60, .d = 0 },
			.max_us_per_sample	= 75000,
		},
	},
	[FIRGELLI_L12_EV3] = {
		.name			= FIRGELLI_L12_EV3_NAME,
		.max_speed		= 1200,
		.count_per_rot		= 360, /* TODO: need to get value in count per cm */
		.encoder_polarity	= DC_MOTOR_POLARITY_INVERSED,
		.legoev3_info		= {
			.samples_for_speed	= { 4, 16, 32, 64 },
			.speed_pid_k		= { .p = 1000, .i = 60, .d = 0 },
			/* TODO: need to put a scope on this and get correct values */
			.max_us_per_sample	= 100000,
		},
	},
};

EXPORT_SYMBOL_GPL(ev3_motor_defs);
