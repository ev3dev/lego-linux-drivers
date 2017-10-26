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

/**
 * DOC: userspace
 *
 * The board info subsystems provides information about hardware "boards", such
 * as embedded devices. Board device nodes can be found at ``/sys/class/board-info/board<N>``
 * where ``<N>`` is an automatically assigned number.
 *
 * The information is read via udev properties (i.e. the ``uevent`` sysfs
 * attribute). Possible properties are:
 *
 * ``BOARD_INFO_FW_VER``
 *
 *     The firmware version.
 *
 * ``BOARD_INFO_HW_REV``
 *
 *     The hardware revision.
 *
 * ``BOARD_INFO_MODEL``
 *
 *     The model name (usually the same as /proc/device-tree/model).
 *
 * ``BOARD_INFO_ROM_REV``
 *
 *     The ROM revision (e.g. for the boot ROM).
 *
 * ``BOARD_INFO_SERIAL_NUM``
 *
 *     The serial number.
 *
 * ``BOARD_INFO_TYPE``
 *
 *     The type of board. This can be ``main`` for a mainboard (where the CPU
 *     lives) or ``aux`` for other boards attached to a mainboard.
 *
 * .. note:: Not all hardware will have all properties.
 */


#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>

#include "board_info.h"

struct board_info {
	struct device dev;
	struct board_info_desc *desc;
	void *drv_data;
};

struct class *board_info_class;
EXPORT_SYMBOL_GPL(board_info_class);

unsigned board_info_class_id = 0;

static const char *board_info_property_name[NUM_BOARD_INFO_PROPERTIES] = {
	[BOARD_INFO_FW_VER]	= __stringify(BOARD_INFO_FW_VER),
	[BOARD_INFO_HW_REV]	= __stringify(BOARD_INFO_HW_REV),
	[BOARD_INFO_MODEL]	= __stringify(BOARD_INFO_MODEL),
	[BOARD_INFO_ROM_REV]	= __stringify(BOARD_INFO_ROM_REV),
	[BOARD_INFO_SERIAL_NUM]	= __stringify(BOARD_INFO_SERIAL_NUM),
	[BOARD_INFO_TYPE]	= __stringify(BOARD_INFO_TYPE),
};

void *board_info_get_drvdata(struct board_info *info)
{
	return info->drv_data;
}
EXPORT_SYMBOL_GPL(board_info_get_drvdata);

static void board_info_release(struct device *dev)
{
	struct board_info *info = dev_get_drvdata(dev);

	kfree(info);
}

struct board_info *__must_check
board_info_register(struct device *parent, struct board_info_desc *desc,
		    void *drv_data)
{
	struct board_info *info;
	int err;

	if (!desc || !desc->properties || !desc->get_property || !parent)
		return ERR_PTR(-EINVAL);

	info = kzalloc(sizeof(*info), GFP_KERNEL);
	if (!info)
		return ERR_PTR(-ENOMEM);

	device_initialize(&info->dev);

	info->dev.release = board_info_release;
	info->dev.parent = parent;
	info->dev.class = board_info_class;
	dev_set_name(&info->dev, "board%d", board_info_class_id++);
	dev_set_drvdata(&info->dev, info);

	info->drv_data = drv_data;
	info->desc = desc;

	err = device_add(&info->dev);
	if (err)
		return ERR_PTR(err);

	return info;
}
EXPORT_SYMBOL_GPL(board_info_register);

void board_info_unregister(struct board_info *info)
{
	device_unregister(&info->dev);
}
EXPORT_SYMBOL_GPL(board_info_unregister);

static void devm_board_info_release(struct device *dev, void *res)
{
	struct board_info **info = res;

	board_info_unregister(*info);
}

struct board_info *__must_check
devm_board_info_register(struct device *parent, struct board_info_desc *desc,
			 void *drv_data)
{
	struct board_info **ptr, *info;

	ptr = devres_alloc(devm_board_info_release, sizeof(*ptr), GFP_KERNEL);

	if (!ptr)
		return ERR_PTR(-ENOMEM);

	info = board_info_register(parent, desc, drv_data);
	if (IS_ERR(info)) {
		devres_free(ptr);
	} else {
		*ptr = info;
		devres_add(parent, ptr);
	}

	return info;
}
EXPORT_SYMBOL_GPL(devm_board_info_register);

static int board_info_uevent(struct device *dev, struct kobj_uevent_env *env)
{
	struct board_info *info = dev_get_drvdata(dev);
	const char *prop_name, *prop_value;
	enum board_info_property prop;
	int i, ret;

	for (i = 0; i < info->desc->num_properties; i++) {
		prop = info->desc->properties[i];
		prop_name = board_info_property_name[prop];
		info->desc->get_property(info, prop, &prop_value);
		ret = add_uevent_var(env, "%s=%s", prop_name, prop_value);
		if (ret) {
			dev_err(dev, "Failed to add uevent %s\n", prop_name);
			return ret;
		}
	}

	return 0;
}

static char *board_info_devnode(struct device *dev, umode_t *mode)
{
	return kasprintf(GFP_KERNEL, "board-info/%s", dev_name(dev));
}

static int __init board_info_class_init(void)
{
	board_info_class = class_create(THIS_MODULE, "board-info");
	if (IS_ERR(board_info_class)) {
		pr_err("unable to register board-info device class\n");
		return PTR_ERR(board_info_class);
	}

	board_info_class->dev_uevent = board_info_uevent;
	board_info_class->devnode = board_info_devnode;

	return 0;
}
subsys_initcall(board_info_class_init);

static void __exit board_info_class_exit(void)
{
	class_destroy(board_info_class);
}
module_exit(board_info_class_exit);

MODULE_DESCRIPTION("Board info subsystem");
MODULE_AUTHOR("David Lechner <david@lechnology.com>");
MODULE_LICENSE("GPL");
