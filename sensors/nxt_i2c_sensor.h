/*
 * LEGO MINDSTORMS NXT I2C sensor device driver
 *
 * Copyright (C) 2013-2015 David Lechner <david@lechnology.com>
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

#include <linux/hrtimer.h>

#include <lego.h>
#include <lego_port_class.h>
#include <lego_sensor_class.h>

#define NXT_I2C_ID_STR_LEN 8

#define NXT_I2C_FW_VER_REG	0x00
#define NXT_I2C_VEND_ID_REG	0x08
#define NXT_I2C_PROD_ID_REG	0x10

struct nxt_i2c_sensor_data;

/**
 * struct nxt_i2c_sensor_platform_data
 *
 * @in_port: Used to pass an input port to be associated with the sensor.
 */
struct nxt_i2c_sensor_platform_data {
	struct lego_port_device *in_port;
};

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
	int (*probe_cb)(struct nxt_i2c_sensor_data *data);
	void (*remove_cb)(struct nxt_i2c_sensor_data *data);
};

/**
 * struct nxt_i2c_sensor_mode_info
 * @set_mode_reg: The register address used to set the mode.
 * @set_mode_data: The data to write to the command register.
 * @read_data_reg: The starting register address of the data to be read.
 */
struct nxt_i2c_sensor_mode_info {
	u8 set_mode_reg;
	u8 set_mode_data;
	u8 read_data_reg;
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
 * @alt_vendor_id: An alternate vendor ID, e.g. for working around case
 * 	sensitivity issues.
 * @product_id: The product ID string to match to the sensor.
 * @ops: Optional hooks for special-case drivers.
 * @mode_info: Array of mode information for each sensor mode. Used by the
 * 	lego-sensor class.
 * @i2c_mode_info: Array of mode information each sensor mode. Used by this
 * 	driver.
 * @cmd_info: Array of command information for each sensor command. Used by
 * 	the lego-sensor class.
 * @i2c_mode_info: Array of command information for each sensor command. Used by
 * 	this driver.
 * @num_modes: Number of valid elements in the mode_info array.
 * @num_read_only_modes: Number of modes that are usable without having to
 * 	write data to the sensor.
 * @num_commands: The number of commands supported by the sensor.
 * @pin1_state: Sets input port pin 1 high (battery voltage) when 1.
 * @slow: The sensor cannot operate at 100kHz.
 */
struct nxt_i2c_sensor_info {
	const char *name;
	const char *vendor_id;
	const char *alt_vendor_id;
	const char *product_id;
	const struct nxt_i2c_sensor_ops *ops;
	const struct lego_sensor_mode_info *mode_info;
	const struct nxt_i2c_sensor_mode_info *i2c_mode_info;
	const struct lego_sensor_cmd_info *cmd_info;
	const struct nxt_i2c_sensor_cmd_info *i2c_cmd_info;
	int num_modes;
	int num_read_only_modes;
	int num_commands;
	enum lego_port_gpio_state pin1_state;
	unsigned slow:1;
};

enum nxt_i2c_sensor_type {
	LEGO_NXT_ULTRASONIC_SENSOR,
	LEGO_NXT_TEMPERATURE_SENSOR,
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
	MS_EV3_SENSOR_MUX,
	MS_LIGHT_SENSOR_ARRAY,
	MS_LINE_LEADER,
	MS_NXTMMX,
	MS_NXTCAM,
	MS_NXTCAM5,
	MS_PIXYADAPTER,
	MS_PPS58_NX,
	MI_CRUIZCORE_XG1300L,
	PX_PIXY,
	NUM_NXT_I2C_SENSORS
};

/*
 * I2C driver names have a fixed size of 19 chars. That includes the null
 * termination, so no more than 18 chars here!
 */
#define LEGO_NXT_ULTRASONIC_SENSOR_NAME		"lego-nxt-us"
#define LEGO_NXT_TEMPERATURE_SENSOR_NAME	"lego-nxt-temp"
#define LEGO_POWER_STORAGE_SENSOR_NAME		"lego-power-storage"
#define HT_NXT_PIR_SENSOR_NAME			"ht-nxt-pir"
#define HT_NXT_BAROMETRIC_SENSOR_NAME		"ht-nxt-barometric"
/* TODO: need to find info on ht-nxt-ir-seek and add a sensor def */
#define HT_NXT_IR_SEEKER_SENSOR_NAME		"ht-nxt-ir-seek"
#define HT_NXT_IR_SEEKER_SENSOR_V2_NAME		"ht-nxt-ir-seek-v2"
#define HT_NXT_COLOR_SENSOR_NAME		"ht-nxt-color"
#define HT_NXT_COLOR_SENSOR_V2_NAME		"ht-nxt-color-v2"
#define HT_NXT_ANGLE_SENSOR_NAME		"ht-nxt-angle"
#define HT_NXT_COMPASS_SENSOR_NAME		"ht-nxt-compass"
#define HT_NXT_IR_RECEIVER_SENSOR_NAME		"ht-nxt-ir-receiver"
#define HT_NXT_ACCELERATION_TILT_SENSOR_NAME	"ht-nxt-accel"
#define HT_NXT_IR_LINK_SENSOR_NAME		"ht-nxt-ir-link"
#define HT_NXT_SUPER_PRO_SENSOR_NAME		"ht-super-pro"
#define HT_NXT_SENSOR_MUX_NAME			"ht-nxt-smux"
#define MS_8CH_SERVO_NAME			"ms-8ch-servo"
#define MS_ABSOLUTE_IMU_NAME			"ms-absolute-imu"
#define MS_ANGLE_SENSOR_NAME			"ms-angle"
#define MS_EV3_SENSOR_MUX_NAME			"ms-ev3-smux"
#define MS_LIGHT_SENSOR_ARRAY_NAME		"ms-light-array"
#define MS_LINE_LEADER_NAME			"ms-line-leader"
#define MS_NXTMMX_NAME				"ms-nxtmmx"
#define MS_NXTCAM_NAME				"ms-nxtcam"
#define MS_NXTCAM5_NAME				"ms-nxtcam5"
#define MS_PIXYADAPTER_NAME			"ms-pixy-adapter"
#define MS_PPS58_NX_NAME			"ms-pps58-nx"
#define MI_CRUIZCORE_XG1300L_NAME		"mi-xg1300l"
#define PX_PIXY_NAME				"pixy-lego"

/*
 * This table is shared by the nxt-i2c-sensor and ht-nxt-smux-i2c-sensor modules.
 */
#define NXT_I2C_SENSOR_ID_TABLE_DATA				\
	LEGO_DEVICE_ID(LEGO_NXT_ULTRASONIC_SENSOR),		\
	LEGO_DEVICE_ID(LEGO_NXT_TEMPERATURE_SENSOR),		\
	LEGO_DEVICE_ID(LEGO_POWER_STORAGE_SENSOR),		\
	LEGO_DEVICE_ID(HT_NXT_PIR_SENSOR),			\
	LEGO_DEVICE_ID(HT_NXT_BAROMETRIC_SENSOR),		\
	LEGO_DEVICE_ID(HT_NXT_IR_SEEKER_SENSOR_V2),		\
	LEGO_DEVICE_ID(HT_NXT_COLOR_SENSOR),			\
	LEGO_DEVICE_ID(HT_NXT_COLOR_SENSOR_V2),			\
	LEGO_DEVICE_ID(HT_NXT_ANGLE_SENSOR),			\
	LEGO_DEVICE_ID(HT_NXT_COMPASS_SENSOR),			\
	LEGO_DEVICE_ID(HT_NXT_IR_RECEIVER_SENSOR),		\
	LEGO_DEVICE_ID(HT_NXT_ACCELERATION_TILT_SENSOR),	\
	LEGO_DEVICE_ID(HT_NXT_IR_LINK_SENSOR),			\
	LEGO_DEVICE_ID(HT_NXT_SUPER_PRO_SENSOR),		\
	LEGO_DEVICE_ID(HT_NXT_SENSOR_MUX),			\
	LEGO_DEVICE_ID(MS_8CH_SERVO),				\
	LEGO_DEVICE_ID(MS_ABSOLUTE_IMU),			\
	LEGO_DEVICE_ID(MS_ANGLE_SENSOR),			\
	LEGO_DEVICE_ID(MS_EV3_SENSOR_MUX),			\
	LEGO_DEVICE_ID(MS_LIGHT_SENSOR_ARRAY),			\
	LEGO_DEVICE_ID(MS_LINE_LEADER),				\
	LEGO_DEVICE_ID(MS_NXTMMX),				\
	LEGO_DEVICE_ID(MS_NXTCAM),				\
	LEGO_DEVICE_ID(MS_NXTCAM5),				\
	LEGO_DEVICE_ID(MS_PIXYADAPTER),				\
	LEGO_DEVICE_ID(MS_PPS58_NX),				\
	LEGO_DEVICE_ID(MI_CRUIZCORE_XG1300L),			\
	LEGO_DEVICE_ID(PX_PIXY),				\
	{ }

extern struct i2c_driver nxt_i2c_sensor_driver;
extern const struct nxt_i2c_sensor_info nxt_i2c_sensor_defs[];

struct nxt_i2c_sensor_data {
	struct i2c_client *client;
	struct lego_port_device *in_port;
	char address[LEGO_NAME_SIZE + 1];
	const struct nxt_i2c_sensor_info *info;
	void *callback_data;
	struct lego_sensor_device sensor;
	struct hrtimer poll_timer;
	struct work_struct poll_work;
	enum nxt_i2c_sensor_type type;
	unsigned poll_ms;
};

#endif /* NXT_I2C_SENSOR_H_ */
