/*
 * LEGO MINDSTORMS EV3 analog sensor device driver
 *
 * Copyright (C) 2014-2015 David Lechner <david@lechnology.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.

 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef _EV3_ANALOG_SENSOR_H_
#define _EV3_ANALOG_SENSOR_H_

#include <lego_port_class.h>
#include <lego_sensor_class.h>

/**
 * struct ev3_analog_sensor_info
 * @name: The driver name. Must match name in id_table.
 * @mode_info: Array of lego-sensor mode information for each sensor mode.
 * @num_modes: Number of valid elements in the mode_info array.
 */
struct ev3_analog_sensor_info {
	const char *name;
	struct lego_sensor_mode_info mode_info[LEGO_SENSOR_MODE_MAX + 1];
	int num_modes;
};

enum ev3_analog_sensor_types {
	GENERIC_EV3_ANALOG_SENSOR,
	LEGO_EV3_TOUCH_SENSOR,
};

#define GENERIC_EV3_ANALOG_SENSOR_NAME	"ev3-analog-XX"
#define EV3_ANALOG_SENSOR_ID_01_NAME	"ev3-analog-01"
#define LEGO_EV3_TOUCH_SENSOR_NAME	"lego-ev3-touch"
#define EV3_ANALOG_SENSOR_ID_03_NAME	"ev3-analog-03"
#define EV3_ANALOG_SENSOR_ID_04_NAME	"ev3-analog-04"
#define EV3_ANALOG_SENSOR_ID_05_NAME	"ev3-analog-05"
#define EV3_ANALOG_SENSOR_ID_06_NAME	"ev3-analog-06"
#define EV3_ANALOG_SENSOR_ID_07_NAME	"ev3-analog-07"
#define EV3_ANALOG_SENSOR_ID_08_NAME	"ev3-analog-08"
#define EV3_ANALOG_SENSOR_ID_09_NAME	"ev3-analog-09"
#define EV3_ANALOG_SENSOR_ID_10_NAME	"ev3-analog-10"
#define EV3_ANALOG_SENSOR_ID_11_NAME	"ev3-analog-11"
#define EV3_ANALOG_SENSOR_ID_12_NAME	"ev3-analog-12"
#define EV3_ANALOG_SENSOR_ID_13_NAME	"ev3-analog-13"
#define EV3_ANALOG_SENSOR_ID_14_NAME	"ev3-analog-14"

extern const struct ev3_analog_sensor_info ev3_analog_sensor_defs[];

struct ev3_analog_sensor_data {
	struct lego_device *ldev;
	struct lego_sensor_device sensor;
	struct ev3_analog_sensor_info info;
	s32 last_value;
};

#endif /* _EV3_ANALOG_SENSOR_H_ */
