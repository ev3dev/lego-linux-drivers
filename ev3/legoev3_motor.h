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
 * @samples_for_speed: Array of how many samples to use in the speed calculation
 *	for a given speed range. Must be a power of two???
 * @speed_pid_k: PID constants for speed regulation.
 * @max_us_per_sample: Maximum duration of a single sample in microseconds.
 *	This number should be 50 * the tacho period at full speed.
 *	Used for motor stall and direction calculations.
 */
struct legoev3_motor_info {
	int samples_for_speed[NUM_LEGOEV3_MOTOR_SPEED_RANGE];
	struct legoev3_motor_pid_k position_pid_k;
	struct legoev3_motor_pid_k speed_pid_k;
	/* TODO: clock_ticks_per_sample needs to be converted to usec */
	int max_us_per_sample;
};

#endif /* __LEGOEV3_MOTOR_H */
