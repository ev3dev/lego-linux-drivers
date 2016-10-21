/*
 * LEGO MINDSTORMS EV3 UART sensor device driver
 *
 * Copyright (C) 2014-2016 David Lechner <david@lechnology.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.

 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef _EV3_UART_SENSOR_H_
#define _EV3_UART_SENSOR_H_

#include <lego_sensor_class.h>

/* Name template for unknown sensors */
#define EV3_UART_SENSOR_NAME(type_id)	"ev3-uart-" type_id

/* well-known sensor names and type ids */
#define LEGO_EV3_COLOR_NAME		"lego-ev3-color"
#define LEGO_EV3_COLOR_TYPE_ID		29
#define LEGO_EV3_ULTRASONIC_NAME	"lego-ev3-us"
#define LEGO_EV3_ULTRASONIC_TYPE_ID	30
#define LEGO_EV3_GYRO_NAME		"lego-ev3-gyro"
#define LEGO_EV3_GYRO_TYPE_ID		32
#define LEGO_EV3_INFRARED_NAME		"lego-ev3-ir"
#define LEGO_EV3_INFRARED_TYPE_ID	33
#define FATCATLAB_ADC_NAME		"fcl-adc"
#define FATCATLAB_ADC_TYPE_ID		71
#define FATCATLAB_GESTURE_NAME		"fcl-gesture"
#define FATCATLAB_GESTURE_TYPE_ID	72
#define FATCATLAB_LIGHT_NAME		"fcl-light"
#define FATCATLAB_LIGHT_TYPE_ID		73
#define FATCATLAB_ALTITUDE_NAME		"fcl-altitude"
#define FATCATLAB_ALTITUDE_TYPE_ID	74
#define FATCATLAB_IR_NAME		"fcl-ir"
#define FATCATLAB_IR_TYPE_ID		75
#define FATCATLAB_9DOF_NAME		"fcl-9dof"
#define FATCATLAB_9DOF_TYPE_ID		76
#define FATCATLAB_HUMIDITY_NAME		"fcl-humidity"
#define FATCATLAB_HUMIDITY_TYPE_ID	77


/* There must be one of these for each well-known name above */
enum ev3_uart_sensor_types {
	LEGO_EV3_COLOR,
	LEGO_EV3_ULTRASONIC,
	LEGO_EV3_GYRO,
	LEGO_EV3_INFRARED,
	FATCATLAB_ADC,
	FATCATLAB_GESTURE,
	FATCATLAB_LIGHT,
	FATCATLAB_ALTITUDE,
	FATCATLAB_IR,
	FATCATLAB_9DOF,
	FATCATLAB_HUMIDITY,
	NUM_LEGO_EV3_SENSOR_TYPES
};

/**
 * struct ev3_uart_sensor_info
 * @name: The driver name. Must match name in id_table.
 * @type_id: The type identifier sent by the sensor.
 * @ms_mode_info: Array of lego-sensor mode information for each sensor mode.
 * @num_modes: Number of valid elements in the mode_info array.
 * @num_view_modes: Number of modes that return a single value.
 */
struct ev3_uart_sensor_info {
	const char *name;
	unsigned type_id;
	struct lego_sensor_mode_info mode_info[LEGO_SENSOR_MODE_MAX + 1];
	int num_modes;
	int num_view_modes;
};

extern const struct ev3_uart_sensor_info ev3_uart_sensor_defs[];

#endif /* _EV3_UART_SENSOR_H_ */
