/*
 * LEGO MINDSTORMS EV3 analog sensor device driver
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

#include <lego.h>
#include <lego_port_class.h>

#include "ev3_analog_sensor.h"
#include "ms_ev3_smux.h"

static int lego_ev3_touch_sensor_scale(void *context,
				       struct lego_sensor_mode_info *mode_info,
				       u8 index, long int *value)
{
	s32 raw_value = *(s32 *)mode_info->raw_data;
#if defined(CONFIG_LEGOEV3_I2C_SENSORS) || defined(CONFIG_LEGOEV3_I2C_SENSORS_MODULE)
	struct ev3_analog_sensor_data *data = context;

	/* mindsensors.com EV3 Sensor Multiplexer returns scaled value */
	if (data->ldev->port->dev.type == &ms_ev3_smux_port_type) {
		return lego_sensor_default_scale(mode_info, index, value);
	}
#endif

	*value = (raw_value > 250) ? 1 : 0;

	return 0;
}

/*
 * Documentation is automatically generated from this struct, so formatting is
 * very important. Make sure any new sensors have the same layout. The comments
 * are also parsed to provide more information for the documentation. The
 * parser can be found in the ev3dev-kpkg repository.
 */

const struct ev3_analog_sensor_info ev3_analog_sensor_defs[] = {
	[GENERIC_EV3_ANALOG_SENSOR] = {
		/**
		 * @vendor_part_name: Generic EV3 Analog Sensor
		 */
		.name = "ev3-analog-XX",
		.num_modes = 1,
		.mode_info = {
			[0] = {
				/**
				 * @description: Raw analog value
				 * @value0: Voltage (0 - 5000)
				 * @units_description: volts
				 */
				.name = "ANALOG",
				.units = "V",
				.raw_max = 5000,
				.si_max = 5000,
				.decimals = 3,
				.data_sets = 1,
				.data_type = LEGO_SENSOR_DATA_S32,
			},
		},
	},
	[LEGO_EV3_TOUCH_SENSOR] = {
		/**
		 * @vendor_name: LEGO
		 * @vendor_part_number: 45507
		 * @vendor_part_name: EV3 Touch Sensor
		 */
		.name = "lego-ev3-touch",
		.num_modes = 1,
		.mode_info = {
			[0] = {
				/**
				 * [^mode0-value]: Values:
				 *
				 * | Value | Description |
				 * |:-----:|:-----------:|
				 * | `0`   | Released    |
				 * | `1`   | Pressed     |
				 *
				 * @description: Button state
				 * @value0: State (0 or 1)
				 * @value0_footnote: [^mode0-value]
				 */
				.name = "TOUCH",
				.data_sets = 1,
				.scale = lego_ev3_touch_sensor_scale,
			},
		},
	},
};
EXPORT_SYMBOL_GPL(ev3_analog_sensor_defs);
