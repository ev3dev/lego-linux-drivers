/*
 * NXT analog sensor device driver for LEGO Mindstorms EV3
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

#include "nxt_analog_sensor.h"

static void nxt_touch_sensor_cb(void *context)
{
	struct nxt_analog_sensor_data *as = context;

	/*
	 * pin 1 is pulled up to 5V in the EV3, so anything less than close to
	 * 5V (5000) is pressed.
	 */
	as->ms.mode_info[as->mode].raw_data[0] =
		as->in_port->in_ops.get_pin1_mv(as->in_port) < 4800 ? 1 : 0;
}

static void ht_eopd_sensor_cb(void *context)
{
	struct nxt_analog_sensor_data *as = context;
	unsigned long pin1_mv;

	/*
	 * To make the sensor value linear, we have to take the square root
	 */
	pin1_mv = as->in_port->in_ops.get_pin1_mv(as->in_port) * 2;
	as->ms.mode_info[as->mode].raw_data[0] = (u8)int_sqrt(pin1_mv);
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

static void ms_touch_mux_cb(void *context)
{
	struct nxt_analog_sensor_data *as = context;
	int pin1_mv;
	u8 sensor1 = 0;
	u8 sensor2 = 0;
	u8 sensor3 = 0;

	pin1_mv = as->in_port->in_ops.get_pin1_mv(as->in_port);
	if (pin1_mv >= MS_TOUCH_MUX_L1 && pin1_mv < MS_TOUCH_MUX_H1) {
		sensor1 = 1;
	} else if (pin1_mv >= MS_TOUCH_MUX_L2 && pin1_mv < MS_TOUCH_MUX_H2) {
		sensor2 = 1;
	} else if (pin1_mv >= MS_TOUCH_MUX_L12 && pin1_mv < MS_TOUCH_MUX_H12) {
		sensor1 = 1;
		sensor2 = 1;
	} else if (pin1_mv >= MS_TOUCH_MUX_L3 && pin1_mv < MS_TOUCH_MUX_H3) {
		sensor3 = 1;
	} else if (pin1_mv >= MS_TOUCH_MUX_L13 && pin1_mv < MS_TOUCH_MUX_H13) {
		sensor1 = 1;
		sensor3 = 1;
	} else if (pin1_mv >= MS_TOUCH_MUX_L23 && pin1_mv < MS_TOUCH_MUX_H23) {
		sensor2 = 1;
		sensor3 = 1;
	} else if (pin1_mv >= MS_TOUCH_MUX_L123 && pin1_mv < MS_TOUCH_MUX_H123) {
		sensor1 = 1;
		sensor2 = 1;
		sensor3 = 1;
	}
	as->ms.mode_info[as->mode].raw_data[0] = sensor1;
	as->ms.mode_info[as->mode].raw_data[1] = sensor2;
	as->ms.mode_info[as->mode].raw_data[2] = sensor3;
}

struct nxt_analog_sensor_info nxt_analog_sensor_defs[] = {
	[GENERIC_NXT_ANALOG_SENSOR] = {
		.num_modes = 2,
		/* TODO: do we want more modes to set pin 6 gpio */
		.ms_mode_info = {
			[0] = {
				.name = "NXT-ANALOG-0",
				.units = "V",
				.raw_max = 5000,
				.pct_max = 100,
				.si_max = 5000,
				.decimals = 3,
				.data_sets = 1,
				.data_type = MSENSOR_DATA_S32,
			},
			[1] = {
				.name = "NXT-ANALOG-1",
				.units = "V",
				.raw_max = 5000,
				.pct_max = 100,
				.si_max = 5000,
				.decimals = 3,
				.data_sets = 1,
				.data_type = MSENSOR_DATA_S32,
			},
		},
		.analog_mode_info = {
			[1] = {
				.pin5_state = EV3_INPUT_PORT_GPIO_HIGH,
			},
		},
	},
	[LEGO_NXT_TOUCH_SENSOR] = {
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
				.analog_cb = nxt_touch_sensor_cb,
			},
		},
	},
	[LEGO_NXT_LIGHT_SENSOR] = {
		.num_modes = 2,
		.ms_mode_info = {
			[0] = {
				.name = "NXT-REFLECT",
				.units = "pct",
				.raw_min = 4116,
				.raw_max = 543,
				.pct_max = 100,
				.si_max = 1000,
				.decimals = 1,
				.data_sets = 1,
				.data_type = MSENSOR_DATA_S32,
			},
			[1] = {
				.name = "NXT-AMBIENT",
				.units = "pct",
				.raw_min = 4164,
				.raw_max = 773,
				.pct_max = 100,
				.si_max = 1000,
				.decimals = 1,
				.data_sets = 1,
				.data_type = MSENSOR_DATA_S32,
			},
		},
		.analog_mode_info = {
			[0] = {
				.pin5_state = EV3_INPUT_PORT_GPIO_HIGH,
			},
			[1] = {
				.pin5_state = EV3_INPUT_PORT_GPIO_LOW,
			},
		},
	},
	[LEGO_NXT_SOUND_SENSOR] = {
		.num_modes = 2,
		.ms_mode_info = {
			[0] = {
				.name = "NXT-SND-DB",
				.units = "pct",
				.raw_max = 5000,
				.pct_max = 100,
				.si_max = 1000,
				.decimals = 1,
				.data_sets = 1,
				.data_type = MSENSOR_DATA_S32,
			},
			[1] = {
				.name = "NXT-SND_DBA",
				.units = "pct",
				.raw_max = 5000,
				.pct_max = 100,
				.si_max = 1000,
				.decimals = 1,
				.data_sets = 1,
				.data_type = MSENSOR_DATA_S32,
			},
		},
		.analog_mode_info = {
			[0] = {
				.pin5_state = EV3_INPUT_PORT_GPIO_LOW,
			},
			[1] = {
				.pin5_state = EV3_INPUT_PORT_GPIO_HIGH,
			},
		},
	},
	[HT_EOPD_SENSOR] = {
		.num_modes = 2,
		.ms_mode_info = {
			[0] = {
				.name = "HT-EOPD-L",
				.raw_max = 100,
				.pct_max = 100,
				.si_max = 100,
				.data_sets = 1,
			},
			[1] = {
				.name = "HT-EOPD-S",
				.raw_max = 100,
				.pct_max = 100,
				.si_max = 100,
				.data_sets = 1,
			},
		},
		.analog_mode_info = {
			[0] = {
				.pin5_state = EV3_INPUT_PORT_GPIO_HIGH,
				.analog_cb = ht_eopd_sensor_cb,
			},
			[1] = {
				.pin5_state = EV3_INPUT_PORT_GPIO_LOW,
				.analog_cb = ht_eopd_sensor_cb,
			},
		},
	},
	[HT_FORCE_SENSOR] = {
		.num_modes = 1,
		.ms_mode_info = {
			[0] = {
				.name = "HT-FORCE",
				.raw_max = 5000,
				.pct_max = 100,
				.si_max = 1023,
				.data_sets = 1,
				.data_type = MSENSOR_DATA_S32,
			},
		},
		/*
		 * TODO: This sensor could use an analog_cb function that
		 * converts the value to grams. It is not linear, so it needs
		 * a fancy equation or lookup table.
		 */
	},
	[HT_GYRO_SENSOR] = {
		.num_modes = 1,
		.ms_mode_info = {
			[0] = {
				.name = "HT-GYRO",
				.units = "d/s",
				.raw_max = 4880,
				.pct_max = 100,
				.si_min = -540,
				.si_max = 400,
				.data_sets = 1,
				.data_type = MSENSOR_DATA_S32,
			},
		},
	},
	[HT_MAGNETIC_SENSOR] = {
		.num_modes = 1,
		.ms_mode_info = {
			[0] = {
				.name = "HT-MAG",
				.raw_max = 5000,
				.pct_max = 100,
				.si_max = 1023,
				.data_sets = 1,
				.data_type = MSENSOR_DATA_S32,
			},
		},
	},
	[MS_TOUCH_SENSOR_MUX] = {
		.num_modes = 1,
		.ms_mode_info = {
			[0] = {
				.name = "TOUCH-MUX",
				.raw_max = 1,
				.pct_max = 100,
				.si_max = 1,
				.data_sets = 3,
			},
		},
		.analog_mode_info = {
			[0] = {
				.analog_cb = ms_touch_mux_cb,
			},
		},
	},
};
