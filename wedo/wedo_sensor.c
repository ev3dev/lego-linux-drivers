/*
 * Sensor Definitions for LEGO WeDo
 *
 * Copyright (C) 2014 Ralph Hempel <rhemple@hempeldesigngroup.com>
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

#include "wedo.h"

enum wedo_tilt_status_id {
	WEDO_TILT_STATUS_UNKNOWN,
	WEDO_TILT_STATUS_BACK,
	WEDO_TILT_STATUS_RIGHT,
	WEDO_TILT_STATUS_LEVEL,
	WEDO_TILT_STATUS_FRONT,
	WEDO_TILT_STATUS_LEFT,
	WEDO_TILT_STATUS_MAX,
};

enum wedo_tilt_axis {
	WEDO_TILT_AXIS_LEFT_RIGHT,
	WEDO_TILT_AXIS_FRONT_BACK,
	WEDO_TILT_AXIS_OK,
};

struct wedo_tilt_status_info {
	unsigned char max;
	unsigned char *name;
};

/* The max fields in this table must be in ascending order for the
 * state calculation to work
 */

static const struct wedo_tilt_status_info wedo_tilt_status_infos[] = {
	[WEDO_TILT_STATUS_UNKNOWN]	= {   0, "unknown"	},
	[WEDO_TILT_STATUS_BACK]		= {  48, "back"		},
	[WEDO_TILT_STATUS_RIGHT]	= {  99, "right"	},
	[WEDO_TILT_STATUS_LEVEL]	= { 153, "level"	},
	[WEDO_TILT_STATUS_FRONT]	= { 204, "front"	},
	[WEDO_TILT_STATUS_LEFT]		= { 255, "left"		},
};

#define WEDO_TILT_STATUS_DEBOUNCE 4

static enum wedo_tilt_status_id
wedo_get_tilt_status(struct lego_sensor_mode_info *mode_info)
{
	enum wedo_tilt_status_id id;
	int rawval = mode_info->raw_data[0];

	for (id = 0; id < WEDO_TILT_STATUS_MAX; ++id)
		if (rawval <= wedo_tilt_status_infos[id].max)
			break;

	return id;
}

static int wedo_tilt_axis_scale(void *context,
				struct lego_sensor_mode_info *mode_info,
				u8 index, long int *value)
{
	enum wedo_tilt_status_id id;

	id = wedo_get_tilt_status(mode_info);

	switch (index) {
	case WEDO_TILT_AXIS_FRONT_BACK:
		if (id == WEDO_TILT_STATUS_BACK)
			*value = -1;
		else if (id == WEDO_TILT_STATUS_FRONT)
			*value = 1;
		else
			*value = 0;
		break;
	case WEDO_TILT_AXIS_LEFT_RIGHT:
		if (id == WEDO_TILT_STATUS_LEFT)
			*value = -1;
		else if (id == WEDO_TILT_STATUS_RIGHT)
			*value = 1;
		else
			*value = 0;
		break;
	case WEDO_TILT_AXIS_OK:
		*value = (id == WEDO_TILT_STATUS_UNKNOWN) ? 0 : 1;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

/* These values must match the values in the mode info docs */
static const u8 wedo_tilt_user_values[] = {
	[WEDO_TILT_STATUS_LEVEL]	= 0,
	[WEDO_TILT_STATUS_FRONT]	= 1,
	[WEDO_TILT_STATUS_BACK]		= 2,
	[WEDO_TILT_STATUS_LEFT]		= 3,
	[WEDO_TILT_STATUS_RIGHT]	= 4,
	[WEDO_TILT_STATUS_UNKNOWN]	= 5,
};

static int wedo_tilt_scale(void *context,
			   struct lego_sensor_mode_info *mode_info,
			   u8 index, long int *value)
{
	*value = wedo_tilt_user_values[wedo_get_tilt_status(mode_info)];

	return 0;
}

const struct wedo_sensor_info wedo_sensor_defs[] = {
	[WEDO_TILT_SENSOR] = {
		/**
		 * @vendor_name: LEGO
		 * @vendor_part_number: 9584
		 * @vendor_part_name: WeDo Tilt Sensor
		 * @vendor_website: https://education.lego.com/en-us/products/wedo-tilt-sensor/9584
		 * @module: wedo
		 */
		.name = "wedo-tilt",
		.num_modes = 3,
		.mode_info = {
			[0] = {
				/**
				 * .. [#wedo-tilt-mode0-value0] Tilt values:
				 *
				 *    =======  =============
				 *     Value    Description
				 *    =======  =============
				 *     0        Level
				 *     1        Front
				 *     2        Back
				 *     3        Left
				 *     4        Right
				 *     5        Unknown
				 *    =======  =============
				 *
				 * @description: Tilt status
				 * @value0: Tilt (0 to 5)
				 * @value0_footnote: [#wedo-tilt-mode0-value0]_
				 */
				.name = "TILT",
				.scale = wedo_tilt_scale,
				.data_sets = 1,
				.data_type = LEGO_SENSOR_DATA_U8,
			},
			[1] = {
				/**
				 * .. [#wedo-tilt-mode1-value0] Axis values:
				 *
				 *    ========  ========  ========  =============
				 *     Value0    Value1    Value2    Description
				 *    ========  ========  ========  =============
				 *     0         0         1         Level
				 *     0         1         1         Front
				 *     0         -1        1         Back
				 *     -1        0         1         Left
				 *     1         0         1         Right
				 *     0         0         0         Unknown
				 *    ========  ========  ========  =============
				 *
				 * @description: Tilt around 2 separate axes
				 * @value0: Tilt Left/Right (-1/0/1)
				 * @value0_footnote: [#wedo-tilt-mode1-value0]_
				 * @value1: Tilt Back/Front (-1/0/1)
				 * @value1_footnote: [#wedo-tilt-mode1-value0]_
				 * @value2: Tilt value valid (0/1)
				 * @value2_footnote: [#wedo-tilt-mode1-value0]_
				 */
				.name = "TILT-AXIS",
				.scale = wedo_tilt_axis_scale,
				.data_sets = 3,
				.data_type = LEGO_SENSOR_DATA_S8,
			},
			[2] = {
				/**
				 * .. [#wedo-tilt-mode2-value0] Raw values:
				 *
				 *    =======  =============
				 *     Value    Description
				 *    =======  =============
				 *     0        Unknown
				 *     < 48     Back
				 *     < 99     Right
				 *     < 153    Level
				 *     < 204    Front
				 *     < 255    Left
				 *    =======  =============
				 *
				 * @description: Raw analog value
				 * @value0: Tilt (0 - 255)
				 * @value0_footnote: [#wedo-tilt-mode2-value0]_
				 */
				.name = "RAW",
				.data_sets = 1,
				.data_type = LEGO_SENSOR_DATA_U8,
			},
		},
	},
	[WEDO_MOTION_SENSOR] = {
		/**
		 * @vendor_name: LEGO
		 * @vendor_part_number: 9583
		 * @vendor_part_name: WeDo Motion Sensor
		 * @vendor_website: https://education.lego.com/en-us/products/wedo-motion-sensor/9583
		 * @module: wedo
		 */
		.name = "wedo-motion",
		.num_modes = 2,
		.mode_info = {
			[0] = {
				/**
				 * @description: Proximity
				 * @value0: Proximity (0 - 100)
				 * @units_description: percent
				 */
				.name = "PROX",
				.raw_min = 71,
				.raw_max = 219,
				.si_max = 100,
				.data_sets = 1,
				.data_type = LEGO_SENSOR_DATA_U8,
				.units	= "pct",
			},
			[1] = {
				/**
				 * @description: Raw analog value
				 * @value0: Proximity (0 - 255)
				 */
				.name = "RAW",
				.data_sets = 1,
				.data_type = LEGO_SENSOR_DATA_U8,
			},
		},
	},
};
