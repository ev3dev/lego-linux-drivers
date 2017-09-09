/*
 * Device class for board info
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

#ifndef __BOARD_INFO_H__
#define __BOARD_INFO_H__

enum board_info_property {
	BOARD_INFO_FW_VER,
	BOARD_INFO_HW_REV,
	BOARD_INFO_MODEL,
	BOARD_INFO_ROM_REV,
	BOARD_INFO_SERIAL_NUM,
	BOARD_INFO_TYPE,
	NUM_BOARD_INFO_PROPERTIES
};

#define BOARD_INFO_TYPE_NAME_MAIN	"main"
#define BOARD_INFO_TYPE_NAME_AUX	"aux"

struct board_info;

struct board_info_desc {
	const enum board_info_property *properties;
	size_t num_properties;
	int (*get_property)(struct board_info *info,
			    enum board_info_property prop,
			    const char **val);
};

extern struct class *board_info_class;
struct board_info *board_info_register(struct device *parent,
				       struct board_info_desc *desc,
				       void *drv_data);
void board_info_unregister(struct board_info *info);
struct board_info *devm_board_info_register(struct device *parent,
					    struct board_info_desc *desc,
					    void *drv_data);

void *board_info_get_drvdata(struct board_info *info);

#endif /* __BOARD_INFO_H__ */
