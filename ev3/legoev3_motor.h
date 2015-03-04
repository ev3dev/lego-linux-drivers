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

#include <dc_motor_class.h>

enum legoev3_motor_id {
	LEGO_EV3_LARGE_MOTOR,
	LEGO_EV3_MEDIUM_MOTOR,
	FIRGELLI_L12_EV3,
	NUM_LEGOEV3_MOTOR_ID
};

#define LEGO_EV3_LARGE_MOTOR_NAME	"lego-ev3-l-motor"
#define LEGO_EV3_MEDIUM_MOTOR_NAME	"lego-ev3-m-motor"
#define FIRGELLI_L12_EV3_NAME		"fi-l12-ev3"

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
 * @name: The driver name.
 * @samples_for_speed: Array of how many samples to use in the speed calculation
 * 	for a given speed range. Must be a power of two???
 * @speed_pid_k: PID constants for speed regulation.
 * @clock_ticks_per_sample: Number of clock ticks to use in sample calculations.
 * @max_tacho_count_per_sec: Maximum number of tacho counts possible in one
 * 	second (at max speed).
 * @count_per_rot: The number of tacho counts in one rotation of the motor.
 * @encoder_polarity: Set to DC_MOTOR_POLARITY_INVERTED for motors with inverted
 * 	tacho outputs.
 */
struct legoev3_motor_info {
	const char *name;
	int samples_for_speed[NUM_LEGOEV3_MOTOR_SPEED_RANGE];
	struct legoev3_motor_pid_k speed_pid_k;
	/* TODO: clock_ticks_per_sample needs to be converted to usec */
	int clock_ticks_per_sample;
	int max_tacho_count_per_sec;
	int count_per_rot;
	enum dc_motor_polarity encoder_polarity;
};

extern const struct legoev3_motor_info legoev3_motor_defs[];
