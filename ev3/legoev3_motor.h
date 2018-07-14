/*
 * Motor driver for LEGO MINDSTORMS LEGOEV3
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

#ifndef __LEGOEV3_MOTOR_H
#define __LEGOEV3_MOTOR_H

#include <dc_motor_class.h>

enum legoev3_motor_speed_range {
	SPEED_BELOW_40,
	SPEED_ABOVE_40,
	SPEED_ABOVE_60,
	SPEED_ABOVE_80,
	NUM_LEGOEV3_MOTOR_SPEED_RANGE
};

/**
 * struct legoev3_motor_pid_k - PID constants
 */
struct legoev3_motor_pid_k {
	int p;
	int i;
	int d;
};

/**
 * struct legoev3_motor_info - motor parameter data specific to the legoev3
 *	driver implementation
 *
 * @speed_pid_k: PID constants for speed regulation.
 */
struct legoev3_motor_info {
	struct legoev3_motor_pid_k position_pid_k;
	struct legoev3_motor_pid_k speed_pid_k;
};

#endif /* __LEGOEV3_MOTOR_H */
