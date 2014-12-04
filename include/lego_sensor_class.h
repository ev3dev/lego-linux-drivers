/*
 * LEGO sensor device class
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

#ifndef _LEGO_SENSOR_CLASS_H_
#define _LEGO_SENSOR_CLASS_H_

#include <linux/device.h>
#include <linux/types.h>

#define LEGO_SENSOR_NAME_SIZE		30
#define LEGO_SENSOR_FW_VERSION_SIZE	8
#define LEGO_SENSOR_MODE_NAME_SIZE	15
#define LEGO_SENSOR_UNITS_SIZE		4
#define LEGO_SENSOR_MODE_MAX		8
#define LEGO_SENSOR_RAW_DATA_SIZE	128

/*
 * Be sure to add the size to lego_sensor_data_size[] when adding values
 * to lego_sensor_data_type.
 */
enum lego_sensor_data_type {
	LEGO_SENSOR_DATA_U8,
	LEGO_SENSOR_DATA_S8,
	LEGO_SENSOR_DATA_U16,
	LEGO_SENSOR_DATA_S16,
	LEGO_SENSOR_DATA_S16_BE,
	LEGO_SENSOR_DATA_U32,
	LEGO_SENSOR_DATA_S32,
	LEGO_SENSOR_DATA_FLOAT,
	NUM_LEGO_SENSOR_DATA_TYPE
};

extern size_t lego_sensor_data_size[];

/**
 * struct lego_sensor_mode_info
 * @name: The name of this mode
 * @raw_min: The minimum raw value of the data read.
 * @raw_min: The maximum raw value of the data read.
 * @pct_min: The minimum percentage value of the data read.
 * @pct_min: The maximum percentage value of the data read.
 * @si_min: The minimum scaled value of the data read.
 * @si_min: The maximum scaled value of the data read.
 * @units: Units of the scaled value.
 * @data_sets: Number of data points in raw data.
 * @data_type: Data type of raw data.
 * @figures: Number of digits that should be displayed, including decimal point.
 * @decimals: Decimal point position.
 * @raw_data: Raw data read from the sensor.
 */
struct lego_sensor_mode_info {
	char name[LEGO_SENSOR_MODE_NAME_SIZE + 1];
	int raw_min;
	int raw_max;
	int pct_min;
	int pct_max;
	int si_min;
	int si_max;
	char units[LEGO_SENSOR_UNITS_SIZE + 1];
	u8 data_sets;
	enum lego_sensor_data_type data_type;
	u8 figures;
	u8 decimals;
	u8 raw_data[LEGO_SENSOR_RAW_DATA_SIZE];
};

/**
 * struct lego_sensor_cmd_info
 * @name: The name of the command
 */
struct lego_sensor_cmd_info {
	char name[LEGO_SENSOR_MODE_NAME_SIZE + 1];
};

/**
 * struct lego_sensor_device
 * @name: Name of the sensor (same as device/driver name, e.g. nxt-touch)
 * @port_name: The name of the port that this sensor is connected to.
 * @num_modes: The number of valid modes.
 * @num_view_modes: The number of valid modes for data logging.
 * @mode: The current mode of the sensor.
 * @mode_info: Array of mode information for the sensor.
 * @num_commands: The number of commands.
 * @cmd_info: Array of command information for the sensor.
 * @set_mode: Callback to set the sensor mode.
 * @send_command: Callback to send a command to the sensor.
 * @write_data: Write data to sensor (optional).
 * @get_poll_ms: Get the polling period in milliseconds (optional).
 * @set_poll_ms: Set the polling period in milliseconds (optional).
 * @context: Pointer to data structure used by callbacks.
 * @fw_version: Firmware version of sensor (optional).
 * @i2c_addr: I2C address if this is an I2C sensor (optional).
 * @dev: The device data structure.
 */
struct lego_sensor_device {
	char name[LEGO_SENSOR_NAME_SIZE + 1];
	char port_name[LEGO_SENSOR_NAME_SIZE + 1];
	u8 num_modes;
	u8 num_view_modes;
	u8 mode;
	struct lego_sensor_mode_info *mode_info;
	u8 num_commands;
	struct lego_sensor_cmd_info *cmd_info;
	int (* set_mode)(void *context, u8 mode);
	int (* send_command)(void *context, u8 command);
	ssize_t (* write_data)(void *context, char *data, loff_t off, size_t count);
	int (* get_poll_ms)(void *context);
	int (* set_poll_ms)(void *context, unsigned value);
	void *context;
	char fw_version[LEGO_SENSOR_FW_VERSION_SIZE + 1];
	unsigned address;
	/* private */
	struct device dev;
};

#define to_lego_sensor_device(_dev) container_of(_dev, struct lego_sensor_device, dev)

extern int lego_sensor_ftoi(u32 f, unsigned dp);
extern u32 lego_sensor_itof(int i, unsigned dp);

extern int register_lego_sensor(struct lego_sensor_device *, struct device *);
extern void unregister_lego_sensor(struct lego_sensor_device *);

extern struct class lego_sensor_class;

#endif /* _LEGO_SENSOR_CLASS_H_ */
