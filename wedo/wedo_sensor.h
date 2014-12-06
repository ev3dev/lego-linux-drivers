/*
 * Sensor Definitions for LEGO WeDo
 *
 * Copyright (C) 2014 Ralph Hempel <rhemple@hempeldesigngroup.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.

 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <lego_sensor_class.h>

/**
 * struct wedo_sensor_mode_info
 * @analog_cb: Analog callback function. Only needed if sensor requires special scaling.
 */
struct wedo_sensor_mode_info {
	void (*analog_cb)(void *context);
};

enum wedo_sensor_types {
	WEDO_TILT_SENSOR,
	WEDO_MOTION_SENSOR,
};

/**
 * struct wedo_sensor_info
 * @name: The driver name. Must match name in id_table.
 * @ms_mode_info: Array of lego-sensor mode information for each sensor mode.
 * @wedo_mode_info: Array of wedo sensor specific mode information for each
 * 	sensor mode.
 * @num_modes: Number of valid elements in the mode_info array.
 */
struct wedo_sensor_info {
	const char* name;
	struct lego_sensor_mode_info mode_info[LEGO_SENSOR_MODE_MAX + 1];
	struct wedo_sensor_mode_info wedo_mode_info[LEGO_SENSOR_MODE_MAX + 1];
	int num_modes;
};

extern const struct wedo_sensor_info wedo_sensor_defs[];

struct wedo_sensor_data {
	struct wedo_port_device *wpd;
	struct lego_sensor_device sensor;
	struct wedo_sensor_info info;

	int debounce_count;
	int debounce_status;
	int status;
};
