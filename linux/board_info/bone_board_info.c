/*
 * Board info driver for BeagleBone.
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
 * The BeagleBone board info driver provides the following properties:
 *
 * ``BOARD_INFO_HW_REV``
 *
 *     Hardware version code. Refer to `eeprom database <https://github.com/RobertCNelson/omap-image-builder/blob/master/readme.md>`_.
 *
 * ``BOARD_INFO_MODEL``
 *
 *     Board Name.
 *
 * ``BOARD_INFO_SERIAL_NUM``
 *
 *     Serial Number. See BeagleBone SRM for details.
 *
 * ``BOARD_INFO_TYPE``
 *
 *     Always returns ``main``.
 */

#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/nvmem-consumer.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

#include "board_info.h"

static const enum board_info_property bone_board_properties[] = {
	BOARD_INFO_HW_REV,
	BOARD_INFO_MODEL,
	BOARD_INFO_SERIAL_NUM,
	BOARD_INFO_TYPE,
};

#define BONE_BOARD_HW_REV_SIZE		5
#define BONE_BOARD_SERIAL_NUM_SIZE	13
#define BONE_BOARD_MODEL_SIZE		9

struct bone_board_data {
	struct board_info_desc desc;
	char hw_rev[BONE_BOARD_HW_REV_SIZE];
	char serial_num[BONE_BOARD_SERIAL_NUM_SIZE];
	char model[BONE_BOARD_MODEL_SIZE];
};

static int bone_board_get_property(struct board_info *info,
				  enum board_info_property prop,
				  const char **val)
{
	struct bone_board_data *data = board_info_get_drvdata(info);

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
	case BOARD_INFO_TYPE:
		*val = BOARD_INFO_TYPE_NAME_MAIN;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int bone_board_probe(struct platform_device *pdev)
{
	struct bone_board_data *data;
	struct board_info *board;
	struct nvmem_cell *nvm;
	const u8 *p;
	size_t len;
	int ret, i;

	data = devm_kzalloc(&pdev->dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	nvm = devm_nvmem_cell_get(&pdev->dev, "eeprom");
	if (IS_ERR(nvm)) {
		dev_err(&pdev->dev, "Failed to get eeprom\n");
		return PTR_ERR(nvm);
	}

	p = nvmem_cell_read(nvm, &len);
	ret = PTR_ERR_OR_ZERO(p);
	if (ret) {
		dev_err(&pdev->dev, "Cannot read cell\n");
		return ret;
	}
	if (len < 28) {
		dev_err(&pdev->dev, "Data too small\n");
		kfree(p);
		return -EINVAL;
	}
	if (p[0] != 0xAA || p[1] != 0x55 || p[2] != 0x33 || p[3] != 0xEE) {
		dev_err(&pdev->dev, "Missing BeagleBone magic\n");
		kfree(p);
		return -EINVAL;
	}

	data->desc.properties = bone_board_properties;
	data->desc.num_properties = ARRAY_SIZE(bone_board_properties);
	data->desc.get_property = bone_board_get_property;

	for (i = 4; i < 12; i++)
		data->model[i - 4] = p[i];
	data->model[i - 4] = '\0';

	for (i = 12; i < 16; i++)
		data->hw_rev[i - 12] = p[i];
	data->hw_rev[i - 12] = '\0';

	for (i = 16; i < 28; i++)
		data->serial_num[i - 16] = p[i];
	data->serial_num[i - 16] = '\0';

	kfree(p);

	board = devm_board_info_register(&pdev->dev, &data->desc, data);

	return PTR_ERR_OR_ZERO(board);
}

static const struct of_device_id of_bone_board_match[] = {
	{ .compatible = "ev3dev,bone-board", },
	{ }
};
MODULE_DEVICE_TABLE(of, of_bone_board_match);

static struct platform_driver bone_board_driver = {
	.driver = {
		.name		= "bone-board",
		.of_match_table = of_bone_board_match,
	},
	.probe  = bone_board_probe,
};
module_platform_driver(bone_board_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("David Lechner <david@lechnology.com>");
MODULE_DESCRIPTION("BeagleBone board info");
MODULE_ALIAS("platform:bone-board");
