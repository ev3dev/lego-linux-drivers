/*
 * NXT I2C sensor device driver for LEGO Mindstorms EV3
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

/**
 * nxt_i2c_sensor_defs - Sensor definitions
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
 * When adding sensors, also add a driver name with the proper type id to
 * nxt_i2c_sensor_idtable in nxt_i2c_sensor_core.c so that the sensor can
 * be manually initialized.
 *
 * Type ids come from sys/settings/typedata.rcf in LMS2012
 * This link will probably break eventually, but for easy access, try:
 * <http://python-ev3.org/types.html>
 */
struct nxt_i2c_sensor_info nxt_i2c_sensor_defs[] = {
	{
		.vendor_id	= "UNKNOWN",
		.product_id	= "unknown",
		.ms.type_id	= 100,
		.ms.num_modes	= 3,
		.ms_mode_info	= {
			[0] = {
				.name = "I2C-U8",
			},
			[1] = {
				.name = "I2C-S8",
				.data_type = MSENSOR_DATA_S8,
			},
			[2] = {
				.name = "I2C-S16",
				.data_type = MSENSOR_DATA_S16,
			},
		},
		.i2c_mode_info	= {
			[0] = {
				.read_data_reg = 0x42,
			},
			[1] = {
				.read_data_reg = 0x42,
			},
			[2] = {
				.read_data_reg = 0x42,
			},
		},
	},
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
		.vendor_id	= "LEGO",
		.product_id	= "", /* LMS2012 fakes this with "Store." */
		.ms.type_id	= 99,
		.ms.num_modes	= 8,
		.ms_mode_info	= {
			[0] = {
				.name = "ES-IN-VOLT",
				.units = "V",
				.raw_max = 10000,
				.si_max = 10000,
				.decimals = 3,
				.data_type = MSENSOR_DATA_S16_BE,
			},
			[1] = {
				.name = "ES-IN-AMP",
				.units = "A",
				.raw_max = 10000,
				.si_max = 10000,
				.decimals = 3,
				.data_type = MSENSOR_DATA_S16_BE,
			},
			[2] = {
				.name = "ES-OUT-VOLT",
				.units = "V",
				.raw_max = 10000,
				.si_max = 10000,
				.decimals = 3,
				.data_type = MSENSOR_DATA_S16_BE,
			},
			[3] = {
				.name = "ES-OUT-AMP",
				.units = "A",
				.raw_max = 10000,
				.si_max = 10000,
				.decimals = 3,
				.data_type = MSENSOR_DATA_S16_BE,
			},
			[4] = {
				.name = "ES-JOULE",
				.units = "J",
				.raw_max = 100,
				.si_max = 100,
				.data_type = MSENSOR_DATA_S16_BE,
			},
			[5] = {
				.name = "ES-IN-WATT",
				.units = "V",
				.raw_max = 10000,
				.si_max = 10000,
				.decimals = 3,
				.data_type = MSENSOR_DATA_S16_BE,
			},
			[6] = {
				.name = "ES-OUT-WATT",
				.units = "A",
				.raw_max = 10000,
				.si_max = 10000,
				.decimals = 3,
				.data_type = MSENSOR_DATA_S16_BE,
			},
			[7] = {
				.name = "ES-ALL",
				.raw_max = 10000,
				.si_max = 10000,
				.decimals = 3,
				.data_sets = 7,
				.data_type = MSENSOR_DATA_S16_BE,
			},
		},
		.i2c_mode_info	= {
			[0] = {
				.read_data_reg	= 0x0A,
			},
			[1] = {
				.read_data_reg	= 0x0C,
			},
			[2] = {
				.read_data_reg	= 0x0E,
			},
			[3] = {
				.read_data_reg	= 0x10,
			},
			[4] = {
				.read_data_reg	= 0x12,
			},
			[5] = {
				.read_data_reg	= 0x14,
			},
			[6] = {
				.read_data_reg	= 0x16,
			},
			[7] = {
				.read_data_reg	= 0x0A,
			},
		},
	},
	{
		.vendor_id	= "HITECHNC",
		.product_id	= "PIR",
		.ms.type_id	= 50,
		.ms.num_modes	= 1,
		.ms_mode_info	= {
			[0] = {
				.name = "HT-PIR",
				.pct_min = -100,
				.si_min = -100,
				.si_max = 100,
				.units = "pct",
			},
		},
		.i2c_mode_info	= {
			[0] = {
				.read_data_reg	= 0x42,
			},
		},
	},
	{
		.vendor_id	= "HiTechnc",
		.product_id	= "Barometr",
		.ms.type_id	= 51,
		.ms.num_modes	= 2,
		.ms_mode_info	= {
			[0] = {
				.name = "HT-BAR-PRES",
				.raw_min = 30400,
				.raw_max = 29400,
				.si_max = 3000,
				.decimals = 1,
				.units = "m",
			},
			[1] = {
				.name = "HT-BAR-TEMP",
				.raw_max = 1000,
				.si_max = 1000,
				.decimals = 1,
				.units = "C",
			},
		},
		.i2c_mode_info	= {
			[0] = {
				.read_data_reg	= 0x42,
			},
			[1] = {
				.read_data_reg	= 0x42,
			},
		},
	},
	{
		.vendor_id	= "HiTechnc",
		.product_id	= "NewIRDir",
		.ms.type_id	= 52,
		.ms.num_modes	= 4,
		.ms_mode_info	= {
			[0] = {
				.name = "HT-DIR-DC",
				.raw_max = 9,
				.si_max = 9,
			},
			[1] = {
				.name = "HT-DIR-AC",
				.raw_max = 9,
				.si_max = 9,
			},
			[2] = {
				.name = "HT-DIR-DALL",
				.data_sets = 7,
			},
			[3] = {
				.name = "HT-DIR-AALL",
				.data_sets = 6,
			},
		},
		.i2c_mode_info	= {
			[0] = {
				.read_data_reg	= 0x42,
			},
			[1] = {
				.read_data_reg	= 0x49,
			},
			[2] = {
				.read_data_reg	= 0x42,
			},
			[3] = {
				.read_data_reg	= 0x49,
			},
		},
	},
	{
		.vendor_id	= "HiTechnc",
		.product_id	= "Color",
		.ms.type_id	= 53,
		.ms.num_modes	= 7,
		.ms_mode_info	= {
			[0] = {
				.name	= "HT-COL1-COL",
				.raw_max = 17,
				.si_max = 17,
			},
			[1] = {
				.name = "HT-COL1-RED",
			},
			[2] = {
				.name = "HT-COL1-GRN",
			},
			[3] = {
				.name = "HT-COL1-BLU",
			},
			[4] = {
				.name = "HT-COL1-RAW",
				.raw_max = USHRT_MAX,
				.si_max = USHRT_MAX,
				.data_sets = 3,
				.data_type = MSENSOR_DATA_U16,
			},
			[5] = {
				.name = "HT-COL1-NRM",
				.data_sets = 4,
			},
			[6] = {
				.name = "HT-COL1-ALL",
				.data_sets = 4,
			},
		},
		.i2c_mode_info	= {
			[0] = {
				.set_mode_reg	= 0x41,
				.read_data_reg	= 0x42,
			},
			[1] = {
				.set_mode_reg	= 0x41,
				.read_data_reg	= 0x43,
			},
			[2] = {
				.set_mode_reg	= 0x41,
				.read_data_reg	= 0x44,
			},
			[3] = {
				.set_mode_reg	= 0x41,
				.read_data_reg	= 0x45,
			},
			[4] = {
				.set_mode_reg	= 0x41,
				.read_data_reg	= 0x46,
			},
			[5] = {
				.set_mode_reg	= 0x41,
				.read_data_reg	= 0x4C,
			},
			[6] = {
				.set_mode_reg	= 0x41,
				.read_data_reg	= 0x42,
			},
		},
	},
	{
		.vendor_id	= "HiTechnc",
		.product_id	= "ColorPD",
		.ms.type_id	= 54,
		.ms.num_modes	= 8,
		.ms_mode_info	= {
			[0] = {
				.name	= "HT-COL2-COL",
				.raw_max = 17,
				.si_max = 17,
			},
			[1] = {
				.name = "HT-COL2-RED",
			},
			[2] = {
				.name = "HT-COL2-GRN",
			},
			[3] = {
				.name = "HT-COL2-BLU",
			},
			[4] = {
				.name = "HT-COL2-WHT",
			},
			[5] = {
				.name = "HT-COL2-RAW",
				.raw_max = USHRT_MAX,
				.si_max = USHRT_MAX,
				.data_sets = 4,
				.data_type = MSENSOR_DATA_U16,
			},
			[6] = {
				.name = "HT-COL2-NRM",
				.data_sets = 4,
			},
			[7] = {
				.name = "HT-COL2-ALL",
				.data_sets = 5,
			},
		},
		.i2c_mode_info	= {
			[0] = {
				.set_mode_reg	= 0x41,
				.read_data_reg	= 0x42,
			},
			[1] = {
				.set_mode_reg	= 0x41,
				.read_data_reg	= 0x43,
			},
			[2] = {
				.set_mode_reg	= 0x41,
				.read_data_reg	= 0x44,
			},
			[3] = {
				.set_mode_reg	= 0x41,
				.read_data_reg	= 0x45,
			},
			[4] = {
				.set_mode_reg	= 0x41,
				.read_data_reg	= 0x46,
			},
			[5] = {
				.set_mode_reg	= 0x41,
				.set_mode_data	= 0x03,
				.read_data_reg	= 0x42,
			},
			[6] = {
				.set_mode_reg	= 0x41,
				.read_data_reg	= 0x47,
			},
			[7] = {
				.set_mode_reg	= 0x41,
				.read_data_reg	= 0x42,
			},
		},
	},
	{
		.vendor_id	= "HITECHNC",
		.product_id	= "AnglSnsr",
		.ms.type_id	= 55,
		.ms.num_modes	= 4,
		.ms_mode_info	= {
			[0] = {
				.name = "HT-ANG-DEG2",
				.raw_max = 180,
				.si_max = 180,
				.units = "deg",
			},
			[1] = {
				.name = "HT-ANG-ACC",
				.raw_min = INT_MIN,
				.raw_max = INT_MAX,
				.si_min = INT_MIN,
				.si_max = INT_MAX,
				.data_type = MSENSOR_DATA_S32,
				.figures = 9,
				.units = "deg",
			},
			[2] = {
				.name = "HT-ANG-RPM",
				.raw_min = SHRT_MIN,
				.raw_max = SHRT_MAX,
				.si_min = SHRT_MIN,
				.si_max = SHRT_MAX,
				.data_type = MSENSOR_DATA_S16,
				.units = "RPM",
			},
			[3] = {
				.name = "HT-ANG-RSET",
				.raw_max = 180,
				.si_max = 180,
				.units = "deg",
			},
		},
		.i2c_mode_info	= {
			[0] = {
				.read_data_reg	= 0x42,
			},
			[1] = {
				.read_data_reg	= 0x44,
			},
			[2] = {
				.read_data_reg	= 0x46,
			},
			[3] = {
				.set_mode_reg	= 0x41,
				.set_mode_data	= 0x52,
				.read_data_reg	= 0x42,
			},
		},
	},
	{
		.vendor_id	= "HiTechnc",
		.product_id	= "Compass",
		.ms.type_id	= 56,
		.ms.num_modes	= 1,
		.ms_mode_info	= {
			[0] = {
				.name = "HT-CMP-DEG2",
				.raw_max = 180,
				.si_max = 180,
				.units = "deg",
				.data_type = MSENSOR_DATA_S8,
			},
		},
		.i2c_mode_info	= {
			[0] = {
				.read_data_reg	= 0x42,
			},
		},
	},
	{
		.vendor_id	= "HiTechnc",
		.product_id	= "IRRecv",
		.ms.type_id	= 57,
		.ms.num_modes	= 2,
		.ms_mode_info	= {
			[0] = {
				.name = "HT-IRRECV",
			},
			[1] = {
				.name = "HT-IRRECV-8",
				.data_sets = 8,
			},
		},
		.i2c_mode_info	= {
			[0] = {
				.read_data_reg	= 0x42,
			},
			[1] = {
				.read_data_reg	= 0x42,
			},
		},
	},
	{
		.vendor_id	= "HITECHNC",
		.product_id	= "Accel.",
		.ms.type_id	= 58,
		.ms.num_modes	= 2,
		.ms_mode_info	= {
			[0] = {
				.name = "HT-ACCL",
			},
			[1] = {
				.name = "HT-ACCL-ALL",
				.data_sets = 6,
			},
		},
		.i2c_mode_info	= {
			[0] = {
				.read_data_reg	= 0x42,
			},
			[1] = {
				.read_data_reg	= 0x42,
			},
		},
	},
	{
		.vendor_id	= "HiTechnc",
		.product_id	= "IRLink",
		.ms.type_id	= 59,
		.ms.num_modes	= 1,
		.ms_mode_info	= {
			[0] = {
				.name = "HT-IRLINK",
			},
		},
		.i2c_mode_info	= {
			[0] = {
				.read_data_reg	= 0x42,
			},
		},
	},
	{
		.vendor_id	= "HiTechnc",
		.product_id	= "SuperPro",
		.ms.type_id	= 60,
		.ms.num_modes	= 5,
		.ms_mode_info	= {
			[0] = {
				.name = "HT-SPRO-DIN",
			},
			[1] = {
				.name = "HT-SPRO-DOT",
			},
			[2] = {
				.name = "HT-SPRO-DCT",
			},
			[3] = {
				.name = "HT-SPRO-STB",
			},
			[4] = {
				.name = "HT-SPRO-LED",
			},
		},
		.i2c_mode_info	= {
			[0] = {
				.read_data_reg	= 0x4C,
			},
			[1] = {
				.read_data_reg	= 0x4D,
			},
			[2] = {
				.read_data_reg	= 0x4E,
			},
			[3] = {
				.read_data_reg	= 0x50,
			},
			[4] = {
				.read_data_reg	= 0x51,
			},
		},
	},
	{
		.vendor_id	= "mndsnsrs",
		.product_id	= "LSArray",
		.ms.type_id	= 157,
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
				.raw_max = USHRT_MAX,
				.si_max = USHRT_MAX,
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
