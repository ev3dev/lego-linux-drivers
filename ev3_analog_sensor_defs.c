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

#include "ev3_analog_sensor.h"

static void ev3_touch_sensor_cb(void *context)
{
	struct ev3_analog_sensor_data *as = context;

	as->info.ms_mode_info[0].raw_data[0] =
		ev3_input_port_get_pin6_mv(as->in_port) > 250 ? 1 : 0;
}

struct ev3_analog_sensor_info ev3_analog_sensor_defs[] = {
	[GENERIC_EV3_ANALOG_SENSOR] = {
		.num_modes = 1,
		.ms_mode_info = {
			[0] = {
				.name = "EV3-ANALOG",
				.units = "V",
				.raw_max = 5000,
				.pct_max = 100,
				.si_max = 5000,
				.decimals = 3,
				.data_sets = 1,
				.data_type = MSENSOR_DATA_S32,
			},
		},
	},
	[LEGO_EV3_TOUCH_SENSOR] = {
		.num_modes = 1,
		.ms_mode_info = {
			[0] = {
				.name = "TOUCH",
				.raw_max = 1,
				.pct_max = 100,
				.si_max = 1,
				.data_sets = 1,
			},
		},
		.analog_mode_info = {
			[0] = {
				.analog_cb = ev3_touch_sensor_cb,
			},
		},
	},
};
