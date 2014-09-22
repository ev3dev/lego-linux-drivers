/*
 * EV3 analog sensor device driver for LEGO Mindstorms EV3
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

#ifndef EV3_ANALOG_SENSOR_H_
#define EV3_ANALOG_SENSOR_H_

#include <linux/legoev3/ev3_input_port.h>
#include <linux/legoev3/msensor_class.h>

/**
 * struct ev3_analog_sensor_mode_info
 * @pin5_state: State of input port pin 5 for this mode.
 * @analog_cb: Analog callback function. Only needed if sensor requires special scaling.
 */
struct ev3_analog_sensor_mode_info {
	enum ev3_input_port_gpio_state pin5_state;
	void (*analog_cb)(void *context);
};

/**
 * struct ev3_analog_sensor_info
 * @name: The driver name. Must match name in id_table.
 * @ms_mode_info: Array of msensor mode information for each sensor mode.
 * @ms_mode_info: Array of analog sensor specific mode information for each
 * 	sensor mode.
 * @num_modes: Number of valid elements in the mode_info array.
 */
struct ev3_analog_sensor_info {
	const char* name;
	struct msensor_mode_info ms_mode_info[MSENSOR_MODE_MAX + 1];
	struct ev3_analog_sensor_mode_info analog_mode_info[MSENSOR_MODE_MAX + 1];
	int num_modes;
};

enum ev3_analog_sensor_types {
	GENERIC_EV3_ANALOG_SENSOR,
	LEGO_EV3_TOUCH_SENSOR,
};

extern const struct ev3_analog_sensor_info ev3_analog_sensor_defs[];

struct ev3_analog_sensor_data {
	struct legoev3_port *in_port;
	struct msensor_device ms;
	struct ev3_analog_sensor_info info;
	u8 mode;
};

#endif /* EV3_ANALOG_SENSOR_H_ */
