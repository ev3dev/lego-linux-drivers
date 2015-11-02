/*
 * LEGO MINDSTORMS NXT/EV3 Motor driver
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

#ifndef __EV3_MOTOR_H
#define __EV3_MOTOR_H

#include <dc_motor_class.h>

#include "../ev3/legoev3_motor.h"

/*
 * Be sure to add devices to ev3_motor_driver_id_table in ev3_motor_core.c
 * when adding to the list here!
 */

enum ev3_motor_id {
	LEGO_NXT_MOTOR,
	LEGO_EV3_LARGE_MOTOR,
	LEGO_EV3_MEDIUM_MOTOR,
	FIRGELLI_L12_EV3_50,
	FIRGELLI_L12_EV3_100,
	NUM_EV3_MOTOR_ID
};

#define LEGO_NXT_MOTOR_NAME		"lego-nxt-motor"
#define LEGO_EV3_LARGE_MOTOR_NAME	"lego-ev3-l-motor"
#define LEGO_EV3_MEDIUM_MOTOR_NAME	"lego-ev3-m-motor"
#define FIRGELLI_L12_EV3_50_NAME	"fi-l12-ev3-50"
#define FIRGELLI_L12_EV3_100_NAME	"fi-l12-ev3-100"

/**
 * @name: The driver name.
 * @max_speed: Maximum speed of the motor in tacho counts per second.
 * @count_per_rot: The number of tacho counts in one rotation of the motor.
 * @encoder_polarity: Set to DC_MOTOR_POLARITY_INVERTED for motors with inverted
 * 	tacho outputs.
 */
struct ev3_motor_info {
	const char *name;
	int max_speed;
	int count_per_rot;
	enum dc_motor_polarity encoder_polarity;
	struct legoev3_motor_info legoev3_info;
};

extern const struct ev3_motor_info ev3_motor_defs[];

#endif /* __EV3_MOTOR_H */
