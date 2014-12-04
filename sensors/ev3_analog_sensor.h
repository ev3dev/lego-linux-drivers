/*
 * EV3 analog sensor device driver
 *
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
	const char* name;
	struct lego_sensor_mode_info mode_info[LEGO_SENSOR_MODE_MAX + 1];
	int num_modes;
};

enum ev3_analog_sensor_types {
	GENERIC_EV3_ANALOG_SENSOR,
	LEGO_EV3_TOUCH_SENSOR,
};

extern const struct ev3_analog_sensor_info ev3_analog_sensor_defs[];

struct ev3_analog_sensor_data {
	struct lego_device *ldev;
	struct lego_sensor_device sensor;
	struct ev3_analog_sensor_info info;
};

#endif /* _EV3_ANALOG_SENSOR_H_ */
