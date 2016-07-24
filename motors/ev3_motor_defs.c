/*
 * Motor driver for LEGO MINDSTORMS EV3
 *
 * Copyright (C) 2015 David Lechner <david@lechnology.com>
 * Copyright (C) 2016 Ralph Hempel <rhempel@hempeldesigngroup.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/module.h>
#include <tacho_motor_class.h>

#include "ev3_motor.h"

/*
 * Documentation is automatically generated from this struct, so formatting is
 * very important. Make sure any new motors have the same syntax. The comments
 * are also parsed to provide more information for the documentation. The
 * parser can be found in the ev3dev-kpkg repository.
 *
 * The LEGO Motor Comparison Page by Philippe Hurbain provided valuable data:
 *
 * http://www.philohome.com/motors/motorcomp.htm
 *
 * For ROTATION motors, the max_speed value is calculated as:
 *
 *     max_speed = (RPM * counts_per_rot) / seconds_per_minute
 *
 * to give a value with units of counts per second.
 */

const struct ev3_motor_info ev3_motor_defs[] = {
	[LEGO_NXT_MOTOR] = {
		/**
		 * @vendor_name: LEGO
		 * @vendor_part_number: 9842
		 * @vendor_part_name: Interactive Servo Motor (NXT)
		 * @vendor_website: http://shop.lego.com/en-US/Interactive-Servo-Motor-9842
		 */
		.name			= LEGO_NXT_MOTOR_NAME,
		.max_speed		= 1020,
		.count_per_rot		= 360,
		.motion_type		= TM_MOTION_ROTATION,
		.legoev3_info		= {
			.samples_for_speed	= { 4, 16, 32, 64 },
			.speed_pid_k		= { .p = 1000, .i = 60, .d = 0 },
			.position_pid_k		= { .p = 80000, .i = 0, .d = 0 },
			.max_us_per_sample	= 100000,
		},
	},
	[LEGO_EV3_LARGE_MOTOR] = {
		/**
		 * @vendor_name: LEGO
		 * @vendor_part_number: 45502
		 * @vendor_part_name: EV3 Large Servo Motor
		 * @vendor_website: http://shop.lego.com/en-US/EV3-Large-Servo-Motor-45502
		 */
		.name			= LEGO_EV3_LARGE_MOTOR_NAME,
		.max_speed		= 1050,
		.count_per_rot		= 360,
		.motion_type		= TM_MOTION_ROTATION,
		.legoev3_info		= {
			.samples_for_speed	= { 4, 16, 32, 64 },
			.speed_pid_k		= { .p = 1000, .i = 60, .d = 0 },
			.position_pid_k		= { .p = 80000, .i = 0, .d = 0 },
			.max_us_per_sample	= 100000,
		},
	},
	[LEGO_EV3_MEDIUM_MOTOR] = {
		/**
		 * @vendor_name: LEGO
		 * @vendor_part_number: 45503
		 * @vendor_part_name: EV3 Medium Servo Motor
		 * @vendor_website: http://shop.lego.com/en-US/EV3-Medium-Servo-Motor-45503
		 */
		.name			= LEGO_EV3_MEDIUM_MOTOR_NAME,
		.max_speed		= 1560,
		.count_per_rot		= 360,
		.motion_type		= TM_MOTION_ROTATION,
		.legoev3_info		= {
			.samples_for_speed	= { 2, 4, 8, 16 },
			.speed_pid_k		= { .p = 1000, .i = 60, .d = 0 },
			.position_pid_k		= { .p = 160000, .i = 0, .d = 0 },
			.max_us_per_sample	= 75000,
		},
	},
	[ACT_L12_EV3_50] = {
		/**
		 * @vendor_name: Actuonix
		 * @vendor_part_number: L12-EV3-50
		 * @vendor_part_name: L12 EV3 50mm
		 * @vendor_website: http://www.actuonix.com/product_p/l12-ev3-50.htm
		 */
		.name			= ACT_L12_EV3_50_NAME,
		.max_speed		= 24,
		.count_per_m		= 2000,
		.full_travel_count	= 100,
		.motion_type		= TM_MOTION_LINEAR,
		.encoder_polarity	= DC_MOTOR_POLARITY_INVERSED,
		.legoev3_info		= {
			.samples_for_speed	= { 4, 16, 32, 64 },
			.speed_pid_k		= { .p = 1000, .i = 60, .d = 0 },
			.position_pid_k		= { .p = 40000, .i = 0, .d = 0 },
			/* TODO: need to put a scope on this and get correct values */
			.max_us_per_sample	= 100000,
		},
	},
	[ACT_L12_EV3_100] = {
		/**
		 * @vendor_name: Actuonix
		 * @vendor_part_number: L12-EV3-100
		 * @vendor_part_name: L12 EV3 100mm
		 * @vendor_website: http://www.actuonix.com/product_p/l12-ev3-100.htm
		 */
		.name			= ACT_L12_EV3_100_NAME,
		.max_speed		= 24,
		.count_per_m		= 2000,
		.full_travel_count	= 200,
		.motion_type		= TM_MOTION_LINEAR,
		.encoder_polarity	= DC_MOTOR_POLARITY_INVERSED,
		.legoev3_info		= {
			.samples_for_speed	= { 4, 16, 32, 64 },
			.speed_pid_k		= { .p = 1000, .i = 60, .d = 0 },
			.position_pid_k		= { .p = 40000, .i = 0, .d = 0 },
			/* TODO: need to put a scope on this and get correct values */
			.max_us_per_sample	= 100000,
		},
	},
};
EXPORT_SYMBOL_GPL(ev3_motor_defs);
