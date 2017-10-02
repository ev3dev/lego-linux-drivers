/*
 * LEGO MINSTORMS NXT analog sensor device driver
 *
 * Copyright (C) 2014-2015 David Lechner <david@lechnology.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.

 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include "nxt_analog_sensor.h"

static int nxt_touch_sensor_scale(void *context,
				  struct lego_sensor_mode_info *mode_info,
				  u8 index, long int *value)
{
	s32 pin1_mv = *(s32 *)mode_info->raw_data;

	/*
	 * pin 1 is pulled up to 5V in the EV3, so anything less than close to
	 * 5V (5000) is pressed.
	 */
	*value = (pin1_mv < 4800) ? 1 : 0;

	return 0;
}

static int ht_eopd_sensor_scale(void *context,
				struct lego_sensor_mode_info *mode_info,
				u8 index, long int *value)
{
	s32 pin1_mv = *(s32 *)mode_info->raw_data;

	/*
	 * To make the sensor value linear, we have to take the square root.
	 * raw_volt max is 5000, so multiply by 2 to get max return value of 100
	 */
	*value = (u8)int_sqrt(pin1_mv * 2);

	return 0;
}

#define MS_TOUCH_MUX_H1    4194
#define MS_TOUCH_MUX_L1    3859
#define MS_TOUCH_MUX_H2    3692
#define MS_TOUCH_MUX_L2    3425
#define MS_TOUCH_MUX_H12   3158
#define MS_TOUCH_MUX_L12   2857
#define MS_TOUCH_MUX_H3    2723
#define MS_TOUCH_MUX_L3    2393
#define MS_TOUCH_MUX_H13   2389
#define MS_TOUCH_MUX_L13   2196
#define MS_TOUCH_MUX_H23   2192
#define MS_TOUCH_MUX_L23   1978
#define MS_TOUCH_MUX_H123  1975
#define MS_TOUCH_MUX_L123  1721

enum ms_touch_mux_port {
	MS_TOUCH_MUX_PORT_1,
	MS_TOUCH_MUX_PORT_2,
	MS_TOUCH_MUX_PORT_3,
	NUM_MS_TOUCH_MUX_PORT
};

static int ms_touch_mux_scale(void *context,
			      struct lego_sensor_mode_info *mode_info,
			      u8 index, long int *value)
{
	s32 pin1_mv = *(s32 *)mode_info->raw_data;
	u8 values[NUM_MS_TOUCH_MUX_PORT] = { 0 };

	if (index >= NUM_MS_TOUCH_MUX_PORT)
		return -EINVAL;

	/*
	 * TODO: This could be more efficient if we only figure out the value
	 * for the given index
	 */
	if (pin1_mv >= MS_TOUCH_MUX_L1 && pin1_mv < MS_TOUCH_MUX_H1) {
		values[MS_TOUCH_MUX_PORT_1] = 1;
	} else if (pin1_mv >= MS_TOUCH_MUX_L2 && pin1_mv < MS_TOUCH_MUX_H2) {
		values[MS_TOUCH_MUX_PORT_2] = 1;
	} else if (pin1_mv >= MS_TOUCH_MUX_L12 && pin1_mv < MS_TOUCH_MUX_H12) {
		values[MS_TOUCH_MUX_PORT_1] = 1;
		values[MS_TOUCH_MUX_PORT_2] = 1;
	} else if (pin1_mv >= MS_TOUCH_MUX_L3 && pin1_mv < MS_TOUCH_MUX_H3) {
		values[MS_TOUCH_MUX_PORT_3] = 1;
	} else if (pin1_mv >= MS_TOUCH_MUX_L13 && pin1_mv < MS_TOUCH_MUX_H13) {
		values[MS_TOUCH_MUX_PORT_1] = 1;
		values[MS_TOUCH_MUX_PORT_3] = 1;
	} else if (pin1_mv >= MS_TOUCH_MUX_L23 && pin1_mv < MS_TOUCH_MUX_H23) {
		values[MS_TOUCH_MUX_PORT_2] = 1;
		values[MS_TOUCH_MUX_PORT_3] = 1;
	} else if (pin1_mv >= MS_TOUCH_MUX_L123 && pin1_mv < MS_TOUCH_MUX_H123) {
		values[MS_TOUCH_MUX_PORT_1] = 1;
		values[MS_TOUCH_MUX_PORT_2] = 1;
		values[MS_TOUCH_MUX_PORT_3] = 1;
	}

	*value = values[index];

	return 0;
}

/*
 * Documentation is automatically generated from this struct, so formatting is
 * very important. Make sure any new sensors have the same layout. The comments
 * are also parsed to provide more information for the documentation. The
 * parser can be found in the Documentation/json/ directory.
 */

const struct nxt_analog_sensor_info nxt_analog_sensor_defs[] = {
	[GENERIC_NXT_ANALOG_SENSOR] = {
		/**
		 * @vendor_part_name: Generic NXT Analog Sensor
		 */
		.name = GENERIC_NXT_ANALOG_SENSOR_NAME,
		.num_modes = 2,
		/* TODO: do we want more modes to set pin 6 gpio */
		.mode_info = {
			[0] = {
				/**
				 * @description: Raw analog value
				 * @value0: Voltage (0 - 5000)
				 * @units_description: volts
				 */
				.name = "ANALOG-0",
				.units = "V",
				.raw_max = 5000,
				.pct_max = 100,
				.si_max = 5000,
				.decimals = 3,
				.data_sets = 1,
				.data_type = LEGO_SENSOR_DATA_S32,
			},
			[1] = {
				/**
				 * @description: Raw analog value - pin 5 high
				 * @value0: Voltage (0 - 5000)
				 * @units_description: volts
				 */
				.name = "ANALOG-1",
				.units = "V",
				.raw_max = 5000,
				.pct_max = 100,
				.si_max = 5000,
				.decimals = 3,
				.data_sets = 1,
				.data_type = LEGO_SENSOR_DATA_S32,
			},
		},
		.analog_mode_info = {
			[0] = {
				.pin5_state = LEGO_PORT_GPIO_LOW,
			},
			[1] = {
				.pin5_state = LEGO_PORT_GPIO_HIGH,
			},
		},
	},
	[LEGO_NXT_TOUCH_SENSOR] = {
		/**
		 * @vendor_name: LEGO
		 * @vendor_part_number: 9843
		 * @vendor_part_name: NXT Touch Sensor
		 * @vendor_website: http://shop.lego.com/en-US/Touch-Sensor-9843
		 */
		.name = LEGO_NXT_TOUCH_SENSOR_NAME,
		.num_modes = 1,
		.mode_info = {
			[0] = {
				/**
				 * .. [#lego-nxt-touch-mode0-value0] Values:
				 *
				 *    ======= =============
				 *     Value   Description
				 *    ======= =============
				 *     0       Released
				 *     1       Pressed
				 *    ======= =============
				 *
				 *    This value supports the ``poll` syscall
				 *    using ``POLLPRI``.
				 *
				 * @description: Button state
				 * @value0: State (0 or 1)
				 * @value0_footnote: [#lego-nxt-touch-mode0-value0]_
				 */
				.name = "TOUCH",
				.scale = nxt_touch_sensor_scale,
				.data_sets = 1,
			},
		},
	},
	[LEGO_NXT_LIGHT_SENSOR] = {
		/**
		 * @vendor_name: LEGO
		 * @vendor_part_number: 9844
		 * @vendor_part_name: NXT Light Sensor
		 * @vendor_website: http://shop.lego.com/en-US/Light-Sensor-9844
		 */
		.name = LEGO_NXT_LIGHT_SENSOR_NAME,
		.num_modes = 2,
		.mode_info = {
			[0] = {
				/**
				 * @description: Reflected light - LED on
				 * @value0: Reflected light intensity (0 to 1000)
				 * @units_description: percent
				 */
				.name = "REFLECT",
				.units = "pct",
				.raw_min = 4116,
				.raw_max = 543,
				.pct_max = 100,
				.si_max = 1000,
				.decimals = 1,
				.data_sets = 1,
				.data_type = LEGO_SENSOR_DATA_S32,
			},
			[1] = {
				/**
				 * @description: Ambient light - LED off
				 * @value0: Ambient light intensity (0 to 1000)
				 * @units_description: percent
				 */
				.name = "AMBIENT",
				.units = "pct",
				.raw_min = 4164,
				.raw_max = 773,
				.pct_max = 100,
				.si_max = 1000,
				.decimals = 1,
				.data_sets = 1,
				.data_type = LEGO_SENSOR_DATA_S32,
			},
		},
		.analog_mode_info = {
			[0] = {
				.pin5_state = LEGO_PORT_GPIO_HIGH,
			},
			[1] = {
				.pin5_state = LEGO_PORT_GPIO_LOW,
			},
		},
	},
	[LEGO_NXT_SOUND_SENSOR] = {
		/**
		 * @vendor_name: LEGO
		 * @vendor_part_number: 9845
		 * @vendor_part_name: NXT Sound Sensor
		 * @vendor_website: http://shop.lego.com/en-US/Sound-Sensor-9845
		 */
		.name = LEGO_NXT_SOUND_SENSOR_NAME,
		.num_modes = 2,
		.mode_info = {
			[0] = {
				/**
				 * @description: Sound pressure level - Flat weighting
				 * @value0: Sound pressure level (0 to 1000)
				 * @units_description: percent
				 */
				.name = "DB",
				.units = "pct",
				.raw_max = 5000,
				.pct_max = 100,
				.si_max = 1000,
				.decimals = 1,
				.data_sets = 1,
				.data_type = LEGO_SENSOR_DATA_S32,
			},
			[1] = {
				/**
				 * @description: Sound pressure level - A weighting
				 * @value0: Sound pressure level (0 to 1000)
				 * @units_description: percent
				 */
				.name = "DBA",
				.units = "pct",
				.raw_max = 5000,
				.pct_max = 100,
				.si_max = 1000,
				.decimals = 1,
				.data_sets = 1,
				.data_type = LEGO_SENSOR_DATA_S32,
			},
		},
		.analog_mode_info = {
			[0] = {
				.pin5_state = LEGO_PORT_GPIO_LOW,
			},
			[1] = {
				.pin5_state = LEGO_PORT_GPIO_HIGH,
			},
		},
	},
	[DI_DFLEX_SENSOR] = {
		/**
		 * @vendor_name: Dexter Industries
		 * @vendor_part_number: dFlex
		 * @vendor_part_name: Flexible Sensor for Mindstorms NXT
		 * @vendor_website: https://www.dexterindustries.com/manual/dflex/
		 */
		.name = DI_DFLEX_SENSOR_NAME,
		.num_modes = 1,
		.mode_info = {
			[0] = {
				/**
				 * @description: Flex
				 * @value0: Flex (0-100)
				 */
				.name = "FLEX",
				.raw_min = 4150,
				.raw_max = 4510,
				.si_min = 0,
				.si_max = 100,
				.data_sets = 1,
				.data_type = LEGO_SENSOR_DATA_S32,
			},
		}
	},
	[HT_EOPD_SENSOR] = {
		/**
		 * @vendor_name: HiTechnic
		 * @vendor_part_number: NEO1048
		 * @vendor_part_name: NXT EOPD
		 * @vendor_website: http://www.hitechnic.com/cgi-bin/commerce.cgi?preadd=action&key=NEO1048
		 */
		.name = HT_EOPD_SENSOR_NAME,
		.num_modes = 2,
		.mode_info = {
			[0] = {
				/**
				 * .. [#ht-nxt-epod-mode0-value0] This value is
				 *    the square root of the raw value. You can
				 *    derive a value proportional (linear) to
				 *    distance by dividing a constant by this
				 *    value, e.g. ``35 / value0``.
				 *
				 * @description: Proximity (long range)
				 * @value0: Proximity (0-100)
				 * @value0_footnote: [#ht-nxt-epod-mode0-value0]_
				 */
				.name = "LONG",
				.scale = ht_eopd_sensor_scale,
				.data_sets = 1,
				.data_type = LEGO_SENSOR_DATA_S32,
			},
			[1] = {
				/**
				 * @description: Proximity (short range)
				 * @value0: Proximity (0-100)
				 * @value0_footnote: [#ht-nxt-epod-mode0-value0]_
				 */
				.name = "SHORT",
				.scale = ht_eopd_sensor_scale,
				.data_sets = 1,
				.data_type = LEGO_SENSOR_DATA_S32,
			},
		},
		.analog_mode_info = {
			[0] = {
				.pin5_state = LEGO_PORT_GPIO_HIGH,
			},
			[1] = {
				.pin5_state = LEGO_PORT_GPIO_LOW,
			},
		},
	},
	[HT_FORCE_SENSOR] = {
		/**
		 * @vendor_name: HiTechnic
		 * @vendor_part_number: NFS1074
		 * @vendor_part_name: NXT Force Sensor
		 * @vendor_website: http://www.hitechnic.com/cgi-bin/commerce.cgi?preadd=action&key=NFS1074
		 */
		.name = HT_FORCE_SENSOR_NAME,
		.num_modes = 1,
		.mode_info = {
			[0] = {
				/**
				 * @description: Raw value (non-linear)
				 * @value0: (0-1023)
				 */
				.name = "FORCE",
				.raw_max = 5000,
				.pct_max = 100,
				.si_max = 1023,
				.data_sets = 1,
				.data_type = LEGO_SENSOR_DATA_S32,
			},
		},
		/*
		 * TODO: This sensor could use an analog_cb function that
		 * converts the value to grams. It is not linear, so it needs
		 * a fancy equation or lookup table.
		 */
	},
	[HT_GYRO_SENSOR] = {
		/**
		 * @vendor_name: HiTechnic
		 * @vendor_part_number: NGY1044
		 * @vendor_part_name: NXT Gyro Sensor
		 * @vendor_website: http://www.hitechnic.com/cgi-bin/commerce.cgi?preadd=action&key=NGY1044
		 */
		.name = HT_GYRO_SENSOR_NAME,
		.num_modes = 1,
		.mode_info = {
			[0] = {
				/**
				 * @description: Angular speed
				 * @value0: Angular speed (-540 to 400)
				 * @units_description: degrees per second
				 */
				.name = "GYRO",
				.units = "d/s",
				.raw_max = 4880,
				.pct_max = 100,
				.si_min = -540,
				.si_max = 400,
				.data_sets = 1,
				.data_type = LEGO_SENSOR_DATA_S32,
			},
		},
	},
	[HT_MAGNETIC_SENSOR] = {
		/**
		 * @vendor_name: HiTechnic
		 * @vendor_part_number: NMS1035
		 * @vendor_part_name: NXT Magnetic Sensor
		 * @vendor_website: http://www.hitechnic.com/cgi-bin/commerce.cgi?preadd=action&key=NMS1035
		 */
		.name = HT_MAGNETIC_SENSOR_NAME,
		.num_modes = 1,
		.mode_info = {
			[0] = {
				/**
				 * @description: Magnetic field???
				 * @value0: ???
				 */
				.name = "MAG",
				.raw_max = 5000,
				.pct_max = 100,
				.si_max = 1023,
				.data_sets = 1,
				.data_type = LEGO_SENSOR_DATA_S32,
			},
		},
	},
	[MS_TOUCH_SENSOR_MUX] = {
		/**
		 * @vendor_name: mindsensors.com
		 * @vendor_part_number: TouchMux
		 * @vendor_part_name: Touch Sensor Multiplexer for NXT & EV3
		 * @vendor_website: http://mindsensors.com/index.php?module=pagemaster&PAGE_user_op=view_page&PAGE_id=135
		 */
		.name = MS_TOUCH_SENSOR_MUX_NAME,
		.num_modes = 1,
		.mode_info = {
			[0] = {
				/**
				 * .. [#ms-nxt-touch-mux-mode0-value0] Values:
				 *
				 *    ======= =============
				 *     Value   Description
				 *    ======= =============
				 *     0       Released
				 *     1       Pressed
				 *    ======= =============
				 *
				 * @description: Touch sensors
				 * @value0: Sensor T1 state
				 * @value0_footnote: [#ms-nxt-touch-mux-mode0-value0]_
				 * @value1: Sensor T2 state
				 * @value1_footnote: [#ms-nxt-touch-mux-mode0-value0]_
				 * @value2: Sensor T3 state
				 * @value2_footnote: [#ms-nxt-touch-mux-mode0-value0]_
				 */
				.name = "TOUCH-MUX",
				.scale = ms_touch_mux_scale,
				.data_sets = 3,
			},
		},
	},
};
