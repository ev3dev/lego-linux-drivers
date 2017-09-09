/*
 * Support for Raspberry Pi board info
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
 * The Raspberry Pi board info driver provides the following properties:
 *
 * ``BOARD_INFO_HW_REV``
 *
 *     This is the same as "Revision" in ``/proc/cpuinfo``.
 *
 * ``BOARD_INFO_MODEL``
 *
 *     Should be something similar to "Raspberry Pi ...". This is the same
 *     as ``/proc/device-tree/model``.
 *
 * ``BOARD_INFO_SERIAL_NUM``
 *
 *     This is the same as "Serial" in ``/proc/cpuinfo``.
 */

#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

#include <asm/system_info.h>

#include "board_info.h"

static const enum board_info_property rpi_board_properties[] = {
	BOARD_INFO_HW_REV,
	BOARD_INFO_MODEL,
	BOARD_INFO_SERIAL_NUM,
};

#define RPI_BOARD_HW_REV_SIZE 9

struct rpi_board_data {
	struct board_info_desc board;
	char hw_rev[RPI_BOARD_HW_REV_SIZE];
	const char *serial_num;
	const char *model;
};

static int rpi_board_get_property(struct board_info *info,
				  enum board_info_property prop,
				  const char **val)
{
	struct rpi_board_data *data = board_info_get_drvdata(info);

	switch (prop) {
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

static int rpi_board_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *root;
	struct rpi_board_data *data;
	struct board_info *board;
	int err;

	data = devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->board.properties = rpi_board_properties;
	data->board.num_properties = ARRAY_SIZE(rpi_board_properties);
	data->board.get_property = rpi_board_get_property;

	snprintf(data->hw_rev, RPI_BOARD_HW_REV_SIZE, "%04X", system_rev);
	data->serial_num = system_serial;

	root = of_find_node_by_path("/");
	of_property_read_string(root, "model", &data->model);

	board = devm_board_info_register(dev, &data->board, data);
	err = PTR_ERR_OR_ZERO(board);
	if (err) {
		dev_err(dev, "Failed to register board info\n");
		return err;
	}

	return 0;
}

static const struct of_device_id rpi_board_of_device_ids[] = {
	{ .compatible = "ev3dev,rpi-board" },
	{ }
};
MODULE_DEVICE_TABLE(of, rpi_board_of_device_ids);

struct platform_driver rpi_board_driver = {
	.probe  = rpi_board_probe,
	.driver = {
		.name = "rpi-board",
		.of_match_table = rpi_board_of_device_ids,
	},
};
module_platform_driver(rpi_board_driver);

MODULE_DESCRIPTION("Board info driver for Raspberry Pi");
MODULE_AUTHOR("David Lechner <david@lechnology.com>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:rpi-board");
