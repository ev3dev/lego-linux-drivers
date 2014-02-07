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

#ifndef NXT_I2C_SENSOR_H_
#define NXT_I2C_SENSOR_H_

#include <linux/legoev3/msensor_class.h>

#define NXT_I2C_ID_STR_LEN 8

/**
 * struct nxt_i2c_sensor_mode_info
 * @ms_mode_info: Mode info used by the msensor device class.
 * @set_mode_reg: The register address used to set the mode.
 * @set_mode_data: The data to write to the command register.
 * @read_data_reg: The starting register address of the data to be read.
 * @pin1_state: Sets input port pin 1 high (battery voltage) when 1.
 */
struct nxt_i2c_sensor_mode_info {
	u8 set_mode_reg;
	u8 set_mode_data;
	u8 read_data_reg;
	unsigned pin1_state:1;
};

/**
 * struct nxt_i2c_sensor_info
 * @vendor_id: The vendor ID string to match to the sensor.
 * @product_id: The product ID string to match to the sensor.
 * @fw_version: The firmware version read from the sensor.
 * @ms: The msensor class device for this sensor.
 * @mode_info: Array of mode information for each sensor mode.
 * @num_modes: Number of valid elements in the mode_info array.
 */
struct nxt_i2c_sensor_info {
	const char *vendor_id;
	const char *product_id;
	char fw_version[NXT_I2C_ID_STR_LEN + 1];
	struct msensor_device ms;
	struct msensor_mode_info ms_mode_info[MSENSOR_MODE_MAX + 1];
	struct nxt_i2c_sensor_mode_info i2c_mode_info[MSENSOR_MODE_MAX + 1];
};

extern struct nxt_i2c_sensor_info nxt_i2c_sensor_defs[];
extern const int num_nxt_i2c_sensor_defs;

#endif /* NXT_I2C_SENSOR_H_ */
