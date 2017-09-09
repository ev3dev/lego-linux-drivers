/*
 * Dexter Industries BrickPi3 board info driver
 *
 * Copyright (C) 2017 David Lechner <david@lechnology.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.

 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

/**
 * DOC: userspace
 *
 * The BrickPi3 board info driver provides the following properties:
 *
 * ``BOARD_INFO_FW_VER``
 *
 *     The BrickPi3 firmware version.
 *
 * ``BOARD_INFO_HW_REV``
 *
 *     The BrickPi3 hardware revision.
 *
 * ``BOARD_INFO_MODEL``
 *
 *     Will be "Dexter Industries BrickPi3".
 *
 * ``BOARD_INFO_SERIAL_NUM``
 *
 *     The BrickPi3 "ID". Use this number in config.txt when stacking BrickPi3s.
 */

#include <linux/device.h>
#include <linux/slab.h>

#include "brickpi3.h"

#include "../linux/board_info/board_info.h"

#define BRICKPI3_BOARD_HW_REV_SIZE	10
#define BRICKPI3_BOARD_FW_VER_SIZE	10
#define BRICKPI3_BOARD_SERIAL_NUM_SIZE	33
#define BRICKPI3_BOARD_MODEL_SIZE	50

struct brickpi3_board_info {
	struct board_info_desc desc;
	char fw_ver[BRICKPI3_BOARD_FW_VER_SIZE];
	char hw_rev[BRICKPI3_BOARD_HW_REV_SIZE];
	char model[BRICKPI3_BOARD_MODEL_SIZE];
	char serial_num[BRICKPI3_BOARD_SERIAL_NUM_SIZE];
};

static const enum board_info_property brickpi3_board_properties[] = {
	BOARD_INFO_FW_VER,
	BOARD_INFO_HW_REV,
	BOARD_INFO_MODEL,
	BOARD_INFO_SERIAL_NUM,
};

static int brickpi3_board_get_property(struct board_info *info,
				       enum board_info_property prop,
				       const char **val)
{
	struct brickpi3_board_info *data = board_info_get_drvdata(info);

	switch (prop) {
	case BOARD_INFO_FW_VER:
		*val = data->fw_ver;
		break;
	case BOARD_INFO_HW_REV:
		*val = data->hw_rev;
		break;
	case BOARD_INFO_MODEL:
		*val = data->model;
		break;
	case BOARD_INFO_SERIAL_NUM:
		*val = data->serial_num;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}


int devm_brickpi3_register_board(struct device *dev, struct brickpi3 *bp,
				 u8 address)
{
	struct brickpi3_board_info *data;
	struct board_info *board;
	char string[BRICKPI3_STRING_MSG_SIZE + 1];
	char string2[BRICKPI3_STRING_MSG_SIZE + 1];
	u32 value;
	int ret;

	data = devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	/* ensure null terminator */
	string[BRICKPI3_STRING_MSG_SIZE] = 0;
	string2[BRICKPI3_STRING_MSG_SIZE] = 0;

	ret = brickpi3_read_string(bp, address, BRICKPI3_MSG_GET_MANUFACTURER,
				   string, BRICKPI3_STRING_MSG_SIZE);
	if (ret < 0)
		return ret;

	ret = brickpi3_read_string(bp, address, BRICKPI3_MSG_GET_NAME,
				   string2, BRICKPI3_STRING_MSG_SIZE);
	if (ret < 0)
		return ret;

	snprintf(data->model, BRICKPI3_BOARD_MODEL_SIZE, "%s %s", string,
		 string2);

	ret = brickpi3_read_u32(bp, address, BRICKPI3_MSG_GET_HARDWARE_VERSION,
				&value);
	if (ret < 0)
		return ret;

	snprintf(data->hw_rev, BRICKPI3_BOARD_HW_REV_SIZE, "%u.%u.%u",
		 value / 1000000 % 1000000, value / 1000 % 1000, value % 1000);

	ret = brickpi3_read_u32(bp, address, BRICKPI3_MSG_GET_FIRMWARE_VERSION,
				&value);
	if (ret < 0)
		return ret;

	snprintf(data->fw_ver, BRICKPI3_BOARD_FW_VER_SIZE, "%u.%u.%u",
		 value / 1000000 % 1000000, value / 1000 % 1000, value % 1000);

	ret = brickpi3_read_string(bp, address, BRICKPI3_MSG_GET_ID, string,
				   BRICKPI3_ID_MSG_SIZE);
	if (ret < 0)
		return ret;

	snprintf(data->serial_num, BRICKPI3_BOARD_SERIAL_NUM_SIZE,
		 "%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X",
		 string[0], string[1], string[2], string[3],
		 string[4], string[5], string[6], string[7],
		 string[8], string[9], string[10], string[11],
		 string[12], string[13], string[14], string[15]);

	data->desc.properties = brickpi3_board_properties;
	data->desc.num_properties = ARRAY_SIZE(brickpi3_board_properties);
	data->desc.get_property = brickpi3_board_get_property;

	board = devm_board_info_register(dev, &data->desc, data);
	ret = PTR_ERR_OR_ZERO(board);
	if (ret) {
		dev_err(dev, "Failed to register board info\n");
		return ret;
	}

	return 0;
 }
