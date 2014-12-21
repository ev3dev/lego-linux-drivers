/*
 * LEGO MINSTORMS NXT analog sensor device driver
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

#ifndef NXT_ANALOG_SENSOR_H_
#define NXT_ANALOG_SENSOR_H_

#include <lego_port_class.h>
#include <lego_sensor_class.h>

/**
 * struct nxt_analog_sensor_mode_info
 * @pin5_state: State of input port pin 5 for this mode.
 */
struct nxt_analog_sensor_mode_info {
	enum lego_port_gpio_state pin5_state;
};

/**
 * struct nxt_analog_sensor_info
 * @name: The driver name. Must match name in id_table.
 * @ms_mode_info: Array of lego-sensor mode information for each sensor mode.
 * @ms_mode_info: Array of analog sensor specific mode information for each
 * 	sensor mode.
 * @num_modes: Number of valid elements in the mode_info array.
 */
struct nxt_analog_sensor_info {
	const char* name;
	struct lego_sensor_mode_info mode_info[LEGO_SENSOR_MODE_MAX + 1];
	struct nxt_analog_sensor_mode_info analog_mode_info[LEGO_SENSOR_MODE_MAX + 1];
	int num_modes;
};

enum nxt_analog_sensor_types {
	GENERIC_NXT_ANALOG_SENSOR,
	LEGO_NXT_TOUCH_SENSOR,
	LEGO_NXT_LIGHT_SENSOR,
	LEGO_NXT_SOUND_SENSOR,
	HT_EOPD_SENSOR,
	HT_FORCE_SENSOR,
	HT_GYRO_SENSOR,
	HT_MAGNETIC_SENSOR,
	MS_TOUCH_SENSOR_MUX,
};

extern const struct nxt_analog_sensor_info nxt_analog_sensor_defs[];

struct nxt_analog_sensor_data {
	struct lego_device *ldev;
	struct lego_sensor_device sensor;
	struct nxt_analog_sensor_info info;
};

#endif /* NXT_ANALOG_SENSOR_H_ */
