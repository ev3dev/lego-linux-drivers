/*
 * LEGO sensor device class
 *
 * Copyright (C) 2014-2015 David Lechner <david@lechnology.com>
 * Copyright (C) 2015-     Ralph Hempel <rhempel@hempeldesigngroup.com>
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
#define LEGO_SENSOR_MODE_MAX		10
#define LEGO_SENSOR_RAW_DATA_SIZE	32

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
 * @scale: Optional scaling function to use instead of raw/si min/max.
 * @units: Units of the scaled value.
 * @data_sets: Number of data points in raw data.
 * @data_type: Data type of raw data.
 * @num_values: Number of value attributes to show. If 0, data_sets will be used.
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
	int (*scale)(void *context, struct lego_sensor_mode_info *mode_info,
		     u8 index, long int *value);
	char units[LEGO_SENSOR_UNITS_SIZE + 1];
	u8 data_sets;
	enum lego_sensor_data_type data_type;
	u8 num_values;
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
 * @name: Name of the driver that loaded this device, e.g. nxt-touch
 * @port_name: The name of the port that this sensor is connected to.
 * @num_modes: The number of valid modes.
 * @num_view_modes: The number of valid modes for data logging.
 * @mode: The current mode of the sensor.
 * @mode_info: Array of mode information for the sensor.
 * @num_commands: The number of commands.
 * @cmd_info: Array of command information for the sensor.
 * @set_mode: Callback to set the sensor mode.
 * @send_command: Callback to send a command to the sensor.
 * @direct_read: Write arbitrary data from sensor (optional).
 * @direct_write: Write arbitrary data to sensor (optional).
 * @get_poll_ms: Get the polling period in milliseconds (optional).
 * @set_poll_ms: Set the polling period in milliseconds (optional).
 * @context: Pointer to data structure used by callbacks.
 * @fw_version: Firmware version of sensor (optional).
 * @dev: The device data structure.
 */
struct lego_sensor_device {
	const char *name;
	const char *port_name;
	u8 num_modes;
	u8 num_view_modes;
	u8 mode;
	struct lego_sensor_mode_info *mode_info;
	u8 num_commands;
	const struct lego_sensor_cmd_info *cmd_info;
	int (* set_mode)(void *context, u8 mode);
	int (* send_command)(void *context, u8 command);
	ssize_t (*direct_read)(void *context, char *data, loff_t off, size_t count);
	ssize_t (*direct_write)(void *context, char *data, loff_t off, size_t count);
	int (* get_poll_ms)(void *context);
	int (* set_poll_ms)(void *context, unsigned value);
	ssize_t (*text_value_read)(char *data, size_t count);
	void *context;
	char fw_version[LEGO_SENSOR_FW_VERSION_SIZE + 1];
	/* private */
	struct device dev;
};

#define to_lego_sensor_device(_dev) container_of(_dev, struct lego_sensor_device, dev)

extern int lego_sensor_ftoi(u32 f, unsigned dp);
extern u32 lego_sensor_itof(int i, unsigned dp);

extern int register_lego_sensor(struct lego_sensor_device *, struct device *);
extern void unregister_lego_sensor(struct lego_sensor_device *);

extern struct class lego_sensor_class;

extern int lego_sensor_default_scale(struct lego_sensor_mode_info *mode_info,
				     u8 index, long int *value);
extern const char *lego_sensor_bin_data_format_to_str(enum lego_sensor_data_type value);
extern int lego_sensor_str_to_bin_data_format(const char *value);

static inline int lego_sensor_get_raw_data_size(struct lego_sensor_mode_info *mode_info)
{
	return mode_info->data_sets * lego_sensor_data_size[mode_info->data_type];
}

static inline int lego_sensor_get_num_values(struct lego_sensor_mode_info *mode_info)
{
	return mode_info->num_values ? mode_info->num_values : mode_info->data_sets;
}

#endif /* _LEGO_SENSOR_CLASS_H_ */
