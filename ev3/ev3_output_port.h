/*
 * Output Port driver for LEGO MINDSTORMS EV3
 *
 * Copyright (C) 2013-2014 David Lechner <david@lechnology.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.

 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef __LINUX_LEGOEV3_EV3_OUTPUT_PORT_H
#define __LINUX_LEGOEV3_EV3_OUTPUT_PORT_H

#include <linux/interrupt.h>

#include <mach/legoev3.h>

#include <dc_motor_class.h>
/*
 * TODO: This should be moved to ev3_output_port.c and ev3_tacho_motor.c should
 * use an id lookup table like sensors do for the various motor types.
 */
enum motor_type {
	MOTOR_NONE,
	MOTOR_NEWTACHO,
	MOTOR_MINITACHO,
	MOTOR_TACHO,
	MOTOR_ERR,
	NUM_MOTOR
};

struct ev3_motor_platform_data {
	struct legoev3_port *out_port;
	struct dc_motor_ops motor_ops;
	unsigned tacho_int_gpio;
	unsigned tacho_dir_gpio;
	enum motor_type motor_type;
};

/* resistor ids for EV3 output devices */
enum ev3_out_dev_id {
	EV3_OUT_DEV_ID_01,
	EV3_OUT_DEV_ID_02,
	EV3_OUT_DEV_ID_03,
	EV3_OUT_DEV_ID_04,
	EV3_OUT_DEV_ID_05,
	EV3_OUT_DEV_ID_06,
	EV3_OUT_DEV_ID_07,
	EV3_OUT_DEV_ID_08,
	EV3_OUT_DEV_ID_09,
	EV3_OUT_DEV_ID_10,
	EV3_OUT_DEV_ID_11,
	EV3_OUT_DEV_ID_12,
	EV3_OUT_DEV_ID_13,
	EV3_OUT_DEV_ID_14,
	NUM_EV3_OUT_DEV_ID,
	EV3_OUT_DEV_ID_ERR = -1
};

extern int ev3_output_port_float_pin56(struct legoev3_port *out_port);

extern unsigned long davinci_read_clocksource_cycles(void);

#endif /* __LINUX_LEGOEV3_EV3_OUTPUT_PORT_H */
