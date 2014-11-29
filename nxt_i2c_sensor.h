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

#ifndef NXT_I2C_SENSOR_H_
#define NXT_I2C_SENSOR_H_

#include <linux/legoev3/ev3_input_port.h>
#include <linux/legoev3/msensor_class.h>

#define NXT_I2C_ID_STR_LEN 8

#define NXT_I2C_FW_VER_REG	0x00
#define NXT_I2C_VEND_ID_REG	0x08
#define NXT_I2C_PROD_ID_REG	0x10

struct nxt_i2c_sensor_data;

/**
 * struct nxt_i2c_sensor_ops
 *
 * If these functions need private data, they should use callback_data
 *
 * @set_mode_pre_cb: Called before the mode is set. Returning a negative error
 * 	value will prevent the mode from being changed.
 * @set_mode_post_cb: Called after the mode has been changed.
 * @send_cmd_pre_cb: Called before the command is sent. Returning a negative
 * 	error value will prevent the command from being sent.
 * @send_cmd_post_cb: Called after the command has been sent
 * @poll_cb: Called after the sensor has been polled.
 * @probe_cb: Called at the end of the driver probe function.
 * @remove_cb: Called at the beginning of the driver remove function.
 */
struct nxt_i2c_sensor_ops {
	int (*set_mode_pre_cb)(struct nxt_i2c_sensor_data *data, u8 mode);
	void (*set_mode_post_cb)(struct nxt_i2c_sensor_data *data, u8 mode);
	int (*send_cmd_pre_cb)(struct nxt_i2c_sensor_data *data, u8 command);
	void (*send_cmd_post_cb)(struct nxt_i2c_sensor_data *data, u8 command);
	void (*poll_cb)(struct nxt_i2c_sensor_data *data);
	void (*probe_cb)(struct nxt_i2c_sensor_data *data);
	void (*remove_cb)(struct nxt_i2c_sensor_data *data);
};

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
	enum ev3_input_port_gpio_state pin1_state;
};

/**
 * struct nxt_i2c_sensor_mode_info
 * @cmd_reg: The register address used to set the command.
 * @cmd_data: The data to write to the command register.
 */
struct nxt_i2c_sensor_cmd_info {
	u8 cmd_reg;
	u8 cmd_data;
};

/**
 * struct nxt_i2c_sensor_info
 * @name: The driver name. Must match name in id_table.
 * @vendor_id: The vendor ID string to match to the sensor.
 * @product_id: The product ID string to match to the sensor.
 * @callback_data: Pointer for private data used by ops.
 * @ops: Optional hooks for special-case drivers.
 * @ms_mode_info: Array of mode information for each sensor mode. Used by the
 * 	msensor class.
 * @i2c_mode_info: Array of mode information each sensor mode. Used by this
 * 	driver.
 * @ms_cmd_info: Array of command information for each sensor command. Used by
 * 	the msensor class.
 * @i2c_mode_info: Array of command information for each sensor command. Used by
 * 	this driver.
 * @num_modes: Number of valid elements in the mode_info array.
 * @num_read_only_modes: Number of modes that are usable without having to
 * 	write data to the sensor.
 * @num_commands: The number of commands supported by the sensor.
 * @slow: The sensor cannot operate at 100kHz.
 */
struct nxt_i2c_sensor_info {
	const char *name;
	const char *vendor_id;
	const char *product_id;
	void *callback_data;
	struct nxt_i2c_sensor_ops ops;
	struct msensor_mode_info ms_mode_info[MSENSOR_MODE_MAX + 1];
	struct nxt_i2c_sensor_mode_info i2c_mode_info[MSENSOR_MODE_MAX + 1];
	struct msensor_cmd_info ms_cmd_info[MSENSOR_MODE_MAX + 1];
	struct nxt_i2c_sensor_cmd_info i2c_cmd_info[MSENSOR_MODE_MAX + 1];
	int num_modes;
	int num_read_only_modes;
	int num_commands;
	unsigned slow:1;
};

enum nxt_i2c_sensor_type {
	LEGO_NXT_ULTRASONIC_SENSOR,
	LEGO_POWER_STORAGE_SENSOR,
	HT_NXT_PIR_SENSOR,
	HT_NXT_BAROMETRIC_SENSOR,
	HT_NXT_IR_SEEKER_SENSOR_V2,
	HT_NXT_COLOR_SENSOR,
	HT_NXT_COLOR_SENSOR_V2,
	HT_NXT_ANGLE_SENSOR,
	HT_NXT_COMPASS_SENSOR,
	HT_NXT_IR_RECEIVER_SENSOR,
	HT_NXT_ACCELERATION_TILT_SENSOR,
	HT_NXT_IR_LINK_SENSOR,
	HT_NXT_SUPER_PRO_SENSOR,
	HT_NXT_SENSOR_MUX,
	MS_8CH_SERVO,
	MS_ABSOLUTE_IMU,
	MS_ANGLE_SENSOR,
	MS_LIGHT_SENSOR_ARRAY,
	MI_CRUIZCORE_XG1300L,
	NUM_NXT_I2C_SENSORS
};

/*
 * This table is shared by the nxt-i2c-sensor and ht-smux-i2c-sensor modules.
 */
#define NXT_I2C_SENSOR_ID_TABLE_DATA \
{ "lego-nxt-us",	LEGO_NXT_ULTRASONIC_SENSOR	}, \
{ "lego-power-storage",	LEGO_POWER_STORAGE_SENSOR	}, \
{ "ht-nxt-pir",		HT_NXT_PIR_SENSOR		}, \
{ "ht-nxt-barometric",	HT_NXT_BAROMETRIC_SENSOR	}, \
{ "ht-ir-seeker-v2",	HT_NXT_IR_SEEKER_SENSOR_V2	}, \
{ "ht-nxt-color",	HT_NXT_COLOR_SENSOR		}, \
{ "ht-nxt-color-v2",	HT_NXT_COLOR_SENSOR_V2		}, \
{ "ht-nxt-angle",	HT_NXT_ANGLE_SENSOR		}, \
{ "ht-nxt-compass",	HT_NXT_COMPASS_SENSOR		}, \
{ "ht-nxt-ir-receiver",	HT_NXT_IR_RECEIVER_SENSOR	}, \
{ "ht-nxt-accel",	HT_NXT_ACCELERATION_TILT_SENSOR	}, \
{ "ht-nxt-ir-link",	HT_NXT_IR_LINK_SENSOR		}, \
{ "ht-super-pro",	HT_NXT_SUPER_PRO_SENSOR		}, \
{ "ht-nxt-smux",	HT_NXT_SENSOR_MUX		}, \
{ "ms-8ch-servo", 	MS_8CH_SERVO			}, \
{ "ms-absolute-imu",	MS_ABSOLUTE_IMU			}, \
{ "ms-angle",		MS_ANGLE_SENSOR			}, \
{ "ms-light-array",	MS_LIGHT_SENSOR_ARRAY		}, \
{ "mi-xg1300l",		MI_CRUIZCORE_XG1300L		}, \
{ }

extern const struct nxt_i2c_sensor_info nxt_i2c_sensor_defs[];

struct nxt_i2c_sensor_data {
	struct i2c_client *client;
	struct legoev3_port *in_port;
	struct nxt_i2c_sensor_info info;
	struct msensor_device ms;
	struct delayed_work poll_work;
	enum nxt_i2c_sensor_type type;
	unsigned poll_ms;
};

#endif /* NXT_I2C_SENSOR_H_ */
