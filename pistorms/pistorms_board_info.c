/*
 * Mindsensors PiStorms board info driver
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
 * The PiSensors board info driver provides the following properties:
 *
 * ``BOARD_INFO_FW_VER``
 *
 *     The PiStorms firmware version.
 *
 * ``BOARD_INFO_MODEL``
 *
 *     Will be "PiStorms".
 *
 * ``BOARD_INFO_TYPE``
 *
 *     Always returns ``aux``.
 */

#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/slab.h>

#include "pistorms.h"

#include "../linux/board_info/board_info.h"
#include "../sensors/nxt_i2c_sensor.h"

#define PISTORMS_BOARD_FW_VER_SIZE	9
#define PISTORMS_BOARD_MODEL_SIZE	9

struct pistorms_board_info {
	struct board_info_desc desc;
	char fw_ver[PISTORMS_BOARD_FW_VER_SIZE];
	char model[PISTORMS_BOARD_MODEL_SIZE];
};

static const enum board_info_property pistorms_board_properties[] = {
	BOARD_INFO_FW_VER,
	BOARD_INFO_MODEL,
	BOARD_INFO_TYPE,
};

static int pistorms_board_get_property(struct board_info *info,
				       enum board_info_property prop,
				       const char **val)
{
	struct pistorms_board_info *data = board_info_get_drvdata(info);

	switch (prop) {
	case BOARD_INFO_FW_VER:
		*val = data->fw_ver;
		break;
	case BOARD_INFO_MODEL:
		*val = data->model;
		break;
	case BOARD_INFO_TYPE:
		*val = BOARD_INFO_TYPE_NAME_AUX;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

int devm_pistorms_register_board(struct device *dev, struct pistorms_data *data)
{
	struct pistorms_board_info *info;
	struct board_info *board;
	int ret;

	info = devm_kzalloc(dev, sizeof(*info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;

	ret = i2c_smbus_read_i2c_block_data(data->client, NXT_I2C_FW_VER_REG,
		NXT_I2C_ID_STR_LEN, info->fw_ver);
	if (ret < 0)
		return ret;

	ret = i2c_smbus_read_i2c_block_data(data->client, NXT_I2C_PROD_ID_REG,
		NXT_I2C_ID_STR_LEN, info->model);
	if (ret < 0)
		return ret;

	info->desc.properties = pistorms_board_properties;
	info->desc.num_properties = ARRAY_SIZE(pistorms_board_properties);
	info->desc.get_property = pistorms_board_get_property;

	board = devm_board_info_register(dev, &info->desc, info);
	ret = PTR_ERR_OR_ZERO(board);
	if (ret) {
		dev_err(dev, "Failed to register board info\n");
		return ret;
	}

	return 0;
}
