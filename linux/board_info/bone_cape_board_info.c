/*
 * Board info driver for BeagleBone capes without EEPROM.
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
 * The BeagleBone **cape** board info driver provides the following properties
 * for capes without an EEPROM:
 *
 * ``BOARD_INFO_MODEL``
 *
 *     The name of the cape. Known names are "FatcatLab EVB" and "QuestCape".
 */

#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

#include "board_info.h"

static const enum board_info_property bone_cape_properties[] = {
	BOARD_INFO_MODEL,
};


struct bone_cape_data {
	struct board_info_desc desc;
	const char *model;
};

static int bone_cape_get_property(struct board_info *info,
				  enum board_info_property prop,
				  const char **val)
{
	struct bone_cape_data *data = board_info_get_drvdata(info);

	switch (prop) {
	case BOARD_INFO_MODEL:
		*val = data->model;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int bone_cape_probe(struct platform_device *pdev)
{
	struct bone_cape_data *data;
	struct board_info *board;

	data = devm_kzalloc(&pdev->dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->desc.properties = bone_cape_properties;
	data->desc.num_properties = ARRAY_SIZE(bone_cape_properties);
	data->desc.get_property = bone_cape_get_property;

	of_property_read_string(pdev->dev.of_node, "model", &data->model);

	board = devm_board_info_register(&pdev->dev, &data->desc, data);

	return PTR_ERR_OR_ZERO(board);
}

static const struct of_device_id of_bone_cape_match[] = {
	{ .compatible = "ev3dev,bone-cape", },
	{ }
};
MODULE_DEVICE_TABLE(of, of_bone_cape_match);

static struct platform_driver bone_cape_driver = {
	.driver = {
		.name		= "bone-cape",
		.of_match_table = of_bone_cape_match,
	},
	.probe  = bone_cape_probe,
};
module_platform_driver(bone_cape_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("David Lechner <david@lechnology.com>");
MODULE_DESCRIPTION("BeagleBone cape board info");
MODULE_ALIAS("platform:bone-cape");
