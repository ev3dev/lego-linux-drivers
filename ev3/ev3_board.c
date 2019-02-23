/*
 * Support for LEGO MINDSTORMS EV3 board info
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
 * The EV3 board info driver provides the following properties:
 *
 * ``BOARD_INFO_HW_REV``
 *
 *     The hardware revision. This comes from the HWIDn resistors on the circuit
 *     board. (This value is not available in the official LEGO firmware.)
 *
 * ``BOARD_INFO_MODEL``
 *
 *     Should be "LEGO MINDSTORMS EV3".
 *
 * ``BOARD_INFO_ROM_REV``
 *
 *     The boot ROM version. This is the same as "Brick HW" in the official
 *     LEGO firmware minus the formatting. e.g. "V0.60" in the official firmware
 *     becomes "6" here.
 *
 * ``BOARD_INFO_SERIAL_NUM``
 *
 *     The brick "ID". This is also used as the Bluetooth MAC address in the
 *     official LEGO firmware.
 *
 * ``BOARD_INFO_TYPE``
 *
 *     Always returns ``main``.
 */

#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/nvmem-consumer.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

#include "../linux/board_info/board_info.h"

static const enum board_info_property ev3_board_properties[] = {
	BOARD_INFO_HW_REV,
	BOARD_INFO_MODEL,
	BOARD_INFO_ROM_REV,
	BOARD_INFO_SERIAL_NUM,
	BOARD_INFO_TYPE,
};

#define EV3_BOARD_HW_REV_SIZE		3	/* max is "15" */
#define EV3_BOARD_ROM_REV_SIZE		4	/* max is "255" */
#define EV3_BOARD_SERIAL_NUM_SIZE	13	/* MAC address */

struct ev3_board_data {
	struct board_info_desc board;
	char hw_rev[EV3_BOARD_HW_REV_SIZE];
	char rom_rev[EV3_BOARD_ROM_REV_SIZE];
	char serial_num[EV3_BOARD_SERIAL_NUM_SIZE];
	const char *model;
};

static int ev3_board_get_property(struct board_info *info,
				  enum board_info_property prop,
				  const char **val)
{
	struct ev3_board_data *data = board_info_get_drvdata(info);

	switch (prop) {
	case BOARD_INFO_HW_REV:
		*val = data->hw_rev;
		break;
	case BOARD_INFO_MODEL:
		*val = data->model;
		break;
	case BOARD_INFO_ROM_REV:
		*val = data->rom_rev;
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

static int ev3_board_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *root;
	struct ev3_board_data *data;
	struct board_info *board;
	struct gpio_descs *hw_id_gpios;
	struct nvmem_cell *hw_id_cell;
	char *buf;
	size_t size;
	int err, i, id;

	data = devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->board.properties = ev3_board_properties;
	data->board.num_properties = ARRAY_SIZE(ev3_board_properties);
	data->board.get_property = ev3_board_get_property;

	hw_id_gpios = devm_gpiod_get_array(dev, "hw-id", GPIOD_IN);
	err = PTR_ERR_OR_ZERO(hw_id_gpios);
	if (err) {
		if (err != -EPROBE_DEFER)
			dev_err(dev, "Failed to get hw-id gpios\n");
		return err;
	}
	if (hw_id_gpios->ndescs != 4) {
		dev_err(dev, "Need 4 hw-id gpios but got %d\n",
			hw_id_gpios->ndescs);
		return -EINVAL;
	}

	id = 0;
	for (i = 0; i < hw_id_gpios->ndescs; i++) {
		struct gpio_desc *gpiod = hw_id_gpios->desc[i];
		int value = gpiod_get_value_cansleep(gpiod) ? 1 : 0;

		id |= value << i;
	}
	snprintf(data->hw_rev, EV3_BOARD_HW_REV_SIZE, "%d", id);

	hw_id_cell = devm_nvmem_cell_get(dev, "hw-id");
	err = PTR_ERR_OR_ZERO(hw_id_cell);
	if (err) {
		if (err != -EPROBE_DEFER)
			dev_err(dev, "Failed to get hw-id nvmem %d\n", err);
		return err;
	}

	buf = nvmem_cell_read(hw_id_cell, &size);
	err = PTR_ERR_OR_ZERO(buf);
	if (err) {
		dev_err(dev, "Failed to read hw-id nvmem\n");
		return err;
	}
	if (size != 12) {
		dev_err(dev, "Bad hw-id nvmem size, expecting 12 but was %u\n",
			size);
		kfree(buf);
		return err;
	}
	id = buf[0];
	snprintf(data->rom_rev, EV3_BOARD_ROM_REV_SIZE, "%d", id);
	snprintf(data->serial_num, EV3_BOARD_SERIAL_NUM_SIZE,
		 "%02X%02X%02X%02X%02X%02X",
		 buf[6], buf[7], buf[8], buf[9], buf[10], buf[11]);
	kfree(buf);

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

static const struct of_device_id ev3_board_of_device_ids[] = {
	{ .compatible = "lego,ev3-board" },
	{ }
};
MODULE_DEVICE_TABLE(of, ev3_board_of_device_ids);

struct platform_driver ev3_board_driver = {
	.probe  = ev3_board_probe,
	.driver = {
		.name = "ev3-board",
		.of_match_table = ev3_board_of_device_ids,
	},
};
module_platform_driver(ev3_board_driver);

MODULE_DESCRIPTION("Board info driver for LEGO MINDSTORMS EV3");
MODULE_AUTHOR("David Lechner <david@lechnology.com>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:ev3-board");
