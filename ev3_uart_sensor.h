/*
 * EV3 UART sensor device driver for LEGO MINDSTORMS EV3
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

#ifndef EV3_UART_SENSOR_H_
#define EV3_UART_SENSOR_H_

#include <linux/legoev3/msensor_class.h>

/**
 * struct ev3_uart_sensor_info
 * @name: The driver name. Must match name in id_table.
 * @ms_mode_info: Array of msensor mode information for each sensor mode.
 * @num_modes: Number of valid elements in the mode_info array.
 * @num_view_modes: Number of modes that return a single value.
 */
struct ev3_uart_sensor_info {
	const char* name;
	struct msensor_mode_info ms_mode_info[MSENSOR_MODE_MAX + 1];
	int num_modes;
	int num_view_modes;
};

enum ev3_uart_sensor_types {
	LEGO_EV3_COLOR,
	LEGO_EV3_ULTRASONIC,
	LEGO_EV3_GYRO,
	LEGO_EV3_INFRARED,
	NUM_LEGO_EV3_SENSOR_TYPES
};

extern const struct ev3_uart_sensor_info ev3_uart_sensor_defs[];

#endif /* EV3_UART_SENSOR_H_ */
