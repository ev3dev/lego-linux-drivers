/*
 * HiTechnic NXT sensor multiplexer device driver for LEGO MINSTORMS EV3
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

#ifndef MS_EV3_SMUX_H_
#define MS_EV3_SMUX_H_

#define MS_EV3_SMUX_MODE_REG		0x52
#define MS_EV3_SMUX_DATA_REG		0x54
#define MS_EV3_SMUX_ANALOG_MODE_DATA	0x0F
#define MS_EV3_SMUX_UPDATE_REG		0x41
#define MS_EV3_SMUX_UPDATE_DATA		0xCC
#define MS_EV3_SMUX_MODE_NAME_REG	0x42
#define MS_EV3_SMUX_MODE_NAME_SIZE	11

struct ms_ev3_smux_input_port_platform_data {
	struct i2c_client *client;
};

struct ms_ev3_smux_host_platform_data {
	const char *inital_sensor;
};

struct ms_ev3_smux_sensor_platform_data {
	struct i2c_client *client;
};

#endif /* MS_EV3_SMUX_H_ */
