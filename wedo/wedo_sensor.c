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

#include "wedo_sensor.h"
#include "wedo_hub.h"

static void wedo_raw_cb(void *context)
{
	struct wedo_sensor_data *wsd = context;

	wsd->sensor.mode_info[wsd->sensor.mode].raw_data[0] = wsd->wpd->input;
}

enum wedo_tilt_status_id {
	WEDO_TILT_STATUS_UNKNOWN,
	WEDO_TILT_STATUS_BACK,
	WEDO_TILT_STATUS_RIGHT,
	WEDO_TILT_STATUS_LEVEL,
	WEDO_TILT_STATUS_FRONT,
	WEDO_TILT_STATUS_LEFT,
	WEDO_TILT_STATUS_MAX,
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

static enum wedo_tilt_status_id wedo_update_tilt_status( struct wedo_sensor_data *wsd )
{
	enum wedo_tilt_status_id id;
	int rawval = wsd->wpd->input;

	for (id=0; id<WEDO_TILT_STATUS_MAX; ++id )
		if ( rawval <= wedo_tilt_status_infos[id].max )
			break;

	if (id != wsd->debounce_status) {
		wsd->debounce_count = 0;
		wsd->debounce_status = id;
	}
	else if (WEDO_TILT_STATUS_DEBOUNCE > wsd->debounce_count ) {
		wsd->debounce_count++;
	}
	else if (WEDO_TILT_STATUS_DEBOUNCE == wsd->debounce_count ) {
		/* Here's where we'd schedule a notification task */
		wsd->debounce_count++;
		wsd->status = id;
	}

	return wsd->status;
}

static void wedo_tilt_axis_cb(void *context)
{
	struct wedo_sensor_data *wsd = context;

	switch (wedo_update_tilt_status (wsd))
	{
	case WEDO_TILT_STATUS_BACK:
		wsd->sensor.mode_info[wsd->sensor.mode].raw_data[0] = 0;
		wsd->sensor.mode_info[wsd->sensor.mode].raw_data[1] = -1;
		wsd->sensor.mode_info[wsd->sensor.mode].raw_data[2] = 1;
		break;

	case WEDO_TILT_STATUS_RIGHT:
		wsd->sensor.mode_info[wsd->sensor.mode].raw_data[0] = 1;
		wsd->sensor.mode_info[wsd->sensor.mode].raw_data[1] = 0;
		wsd->sensor.mode_info[wsd->sensor.mode].raw_data[2] = 1;
		break;

	case WEDO_TILT_STATUS_FRONT:
		wsd->sensor.mode_info[wsd->sensor.mode].raw_data[0] = 0;
		wsd->sensor.mode_info[wsd->sensor.mode].raw_data[1] = 1;
		wsd->sensor.mode_info[wsd->sensor.mode].raw_data[2] = 1;
		break;

	case WEDO_TILT_STATUS_LEFT:
		wsd->sensor.mode_info[wsd->sensor.mode].raw_data[0] = -1;
		wsd->sensor.mode_info[wsd->sensor.mode].raw_data[1] = 0;
		wsd->sensor.mode_info[wsd->sensor.mode].raw_data[2] = 1;
		break;

	case WEDO_TILT_STATUS_LEVEL:
		wsd->sensor.mode_info[wsd->sensor.mode].raw_data[0] = 0;
		wsd->sensor.mode_info[wsd->sensor.mode].raw_data[1] = 0;
		wsd->sensor.mode_info[wsd->sensor.mode].raw_data[2] = 1;
		break;

	case WEDO_TILT_STATUS_UNKNOWN:
	default:
		wsd->sensor.mode_info[wsd->sensor.mode].raw_data[0] = 0;
		wsd->sensor.mode_info[wsd->sensor.mode].raw_data[1] = 0;
		wsd->sensor.mode_info[wsd->sensor.mode].raw_data[2] = 0;
		break;
	}
}

static void wedo_tilt_status_cb(void *context)
{
	struct wedo_sensor_data *wsd = context;

	switch (wedo_update_tilt_status (wsd))
	{
	case WEDO_TILT_STATUS_BACK:
		wsd->sensor.mode_info[wsd->sensor.mode].raw_data[0] = 2;
		break;

	case WEDO_TILT_STATUS_RIGHT:
		wsd->sensor.mode_info[wsd->sensor.mode].raw_data[0] = 4;
		break;

	case WEDO_TILT_STATUS_FRONT:
		wsd->sensor.mode_info[wsd->sensor.mode].raw_data[0] = 1;
		break;

	case WEDO_TILT_STATUS_LEFT:
		wsd->sensor.mode_info[wsd->sensor.mode].raw_data[0] = 3;
		break;

	case WEDO_TILT_STATUS_LEVEL:
		wsd->sensor.mode_info[wsd->sensor.mode].raw_data[0] = 0;
		break;

	case WEDO_TILT_STATUS_UNKNOWN:
	default:
		wsd->sensor.mode_info[wsd->sensor.mode].raw_data[0] = 5;
		break;
	}
}

const struct wedo_sensor_info wedo_sensor_defs[] = {
	[WEDO_TILT_SENSOR] = {
		/**
		 * @vendor_name: LEGO
		 * @vendor_part_number: 9584
		 * @vendor_part_name: WeDo Tilt Sensor
		 * @vendor_website: http://education.lego.com/en-us/lego-education-product-database/wedo/9584-tilt-sensor
		 */
		.name = "wedo-tilt",
		.num_modes = 3,
		.mode_info = {
			[0] = {
				/**
				 * [^tilt-values]: Tilt values:
				 *
				 * | Value | Description |
				 * |-------|-------------|
				 * | 0     | Level       |
				 * | 1     | Front       |
				 * | 2     | Back        |
				 * | 3     | Left        |
				 * | 4     | Right       |
				 * | 5     | Unknown     |
				 *
				 * @description: Tilt status
				 * @value0: Tilt (0 to 5)
				 * @value0_footnote: [^tilt-values]
				 */
				.name = "TILT",
				.raw_max = 5,
				.si_max = 5,
				.data_sets = 1,
				.data_type = LEGO_SENSOR_DATA_U8,
			},
			[1] = {
				/**
				 * [^axis-values]: Axis values:
				 *
				 * | Value0 | Value1 | Value2 | Description |
				 * |--------|--------|--------|-------------|
				 * | 0      | 0      | 1      | Level       |
				 * | 0      | 1      | 1      | Front       |
				 * | 0      | -1     | 1      | Back        |
				 * | -1     | 0      | 1      | Left        |
				 * | 1      | 0      | 1      | Right       |
				 * | 0      | 0      | 0      | Unknown     |
				 *
				 * @description: Tilt around 2 separate axes
				 * @value0: Tilt Left/Right (-1/0/1)
				 * @value0_footnote: [^axis-values]
				 * @value1: Tilt Back/Front (-1/0/1)
				 * @value1_footnote: [^axis-values]
				 * @value2: Tilt value valid (0/1)
				 * @value2_footnote: [^axis-values]
				 */
				.name = "TILT-AXIS",
				.raw_min = -1,
				.raw_max = 1,
				.si_min = -1,
				.si_max = 1,
				.data_sets = 3,
				.data_type = LEGO_SENSOR_DATA_S8,
			},
			[2] = {
				/**
				 * [^raw-values]: Raw values:
				 *
				 * | Value | Description |
				 * |-------|-------------|
				 * | 0     | Unknown     |
				 * | < 48  | Back        |
				 * | < 99  | Right       |
				 * | < 153 | Level       |
				 * | < 204 | Front       |
				 * | < 255 | Left        |
				 *
				 * @description: Raw analog value
				 * @value0: Tilt (0 - 255)
				 * @value0_footnote: [^raw-values]
				 */
				.name = "RAW",
				.raw_max = 255,
				.si_max = 255,
				.data_sets = 1,
				.data_type = LEGO_SENSOR_DATA_U8,
			},
		},
		.wedo_mode_info = {
			[0] = {
				.analog_cb = wedo_tilt_status_cb,
			},
			[1] = {
				.analog_cb = wedo_tilt_axis_cb,
			},
			[2] = {
				.analog_cb = wedo_raw_cb,
			},
		}
	},
	[WEDO_MOTION_SENSOR] = {
		/**
		 * @vendor_name: LEGO
		 * @vendor_part_number: 9583
		 * @vendor_part_name: WeDo Motion Sensor
		 * @vendor_website: http://education.lego.com/en-us/lego-education-product-database/wedo/9583-motion-sensor
		 */
		.name = "wedo-motion",
		.num_modes = 1,
		.mode_info = {
			[0] = {
				/**
				 * @description: Raw analog value
				 * @value0: Motion (0 - 255)
				 */
				.name = "RAW",
				.raw_max = 255,
				.si_max = 255,
				.data_sets = 1,
				.data_type = LEGO_SENSOR_DATA_U8,
			},
		},
		.wedo_mode_info = {
			[0] = {
				.analog_cb = wedo_raw_cb,
			},
		}
	},
};

