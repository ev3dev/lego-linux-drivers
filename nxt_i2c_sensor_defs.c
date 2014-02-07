/*
 * NXT Ultrasonic sensor device driver for LEGO Mindstorms EV3
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

#include <linux/export.h>

#include "nxt_i2c_sensor.h"

/*
 * Definitions for all known sensors
 *
 * Required values:
 * - vendor_id
 * - product_id
 * - ms.type_id
 * - ms.num_modes
 * - mode_info.ms_mode_info.name
 * - i2c_mode_info.read_data_reg
 * - num_modes
 *
 * Optional values:
 * - ms.num_view_modes (default 1)
 * - ms_mode_info.raw_min
 * - ms_mode_info.raw_max (default 255)
 * - ms_mode_info.pct_min
 * - ms_mode_info.pct_max (default 100)
 * - ms_mode_info.si_min
 * - ms_mode_info.si_max (default 255)
 * - ms_mode_info.units
 * - ms_mode_info.data_sets (default 1)
 * - ms_mode_info.data_type (default MSENSOR_DATA_U8)
 * - ms_mode_info.figures (default 5)
 * - ms_mode_info.decimals
 * - i2c_mode_info.set_mode_reg and mode_info.set_mode_data
 * - i2c_mode_info.pin1_state
 *
 * All other values will be overwritten during device initialization.
 *
 * Each sensor should have at least one mode. Mode [0] will be the default mode.
 *
 * Type ids come from sys/settings/typedata.rcf in LMS2012
 * This link will probably break eventually, but for easy access, try:
 * <http://python-ev3.org/types.html>
 */
struct nxt_i2c_sensor_info nxt_i2c_sensor_defs[] = {
	{
		.vendor_id	= "LEGO",
		.product_id	= "Sonar",
		.ms.type_id	= 5,
		.ms.num_modes	= 5,
		.ms_mode_info	= {
			[0] = {
				.name	= "NXT-US-CM",
				.units	= "cm",
			},
			[1] = {
				.name	= "NXT-US-IN",
				.units	= "in",
				.si_max = 1000,
				.decimals = 1,
			},
			[2] = {
				.name	= "NXT-US-SI-CM",
				.units	= "cm",
			},
			[3] = {
				.name	= "NXT-US-SI-IN",
				.units	= "in",
				.si_max = 1000,
				.decimals = 1,
			},
			[4] = {
				.name	= "NXT-US-LIST",
				.raw_max = 1,
				.si_max  = 1,
			},
		},
		.i2c_mode_info	= {
			[0] = {
				.set_mode_reg	= 0x41,
				.set_mode_data	= 0x02,
				.read_data_reg	= 0x42,
				.pin1_state	= 1,
			},
			[1] = {
				.set_mode_reg	= 0x41,
				.set_mode_data	= 0x02,
				.read_data_reg	= 0x42,
				.pin1_state	= 1,
			},
			[2] = {
				.set_mode_reg	= 0x41,
				.set_mode_data	= 0x01,
				.read_data_reg	= 0x42,
				.pin1_state	= 1,
			},
			[3] = {
				.set_mode_reg	= 0x41,
				.set_mode_data	= 0x01,
				.read_data_reg	= 0x42,
				.pin1_state	= 1,
			},
			[4] = {
				.set_mode_reg	= 0x41,
				.set_mode_data	= 0x03,
				.read_data_reg	= 0x42,
				.pin1_state	= 1,
			},
		},
	},
	{
		.vendor_id	= "mndsnsrs",
		.product_id	= "LSArray",
		.ms.type_id	= 70,
		.ms.num_modes	= 2,
		.ms_mode_info	= {
			[0] = {
				.name	= "MS-LSA-CAL",
				.raw_max = 100,
				.si_max = 100,
				.data_sets = 8,
				.units	= "pct",
			},
			[1] = {
				.name	= "MS-LSA-RAW",
				.raw_max = 65535,
				.si_max = 65535,
				.data_sets = 8,
				.data_type = MSENSOR_DATA_S16,
			},
		},
		.i2c_mode_info	= {
			[0] = {
				.set_mode_reg	= 0x41,
				.set_mode_data	= 'w',
				.read_data_reg	= 0x42,
			},
			[1] = {
				.set_mode_reg	= 0x41,
				.set_mode_data	= 'w',
				.read_data_reg	= 0x6A,
			},
		},
	},
};
EXPORT_SYMBOL_GPL(nxt_i2c_sensor_defs);

const int num_nxt_i2c_sensor_defs = ARRAY_SIZE(nxt_i2c_sensor_defs);
