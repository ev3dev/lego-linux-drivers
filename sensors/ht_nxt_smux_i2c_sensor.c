/*
 * HiTechnic NXT Sensor Multiplexer device driver
 *
 * Copyright (C) 2013-2015 David Lechner <david@lechnology.com>
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
 * A ``ht-smux-i2c-sensor`` device is loaded by the :ref:`ht-nxt-smux` driver
 * when it is detected by the sensor mux (automatic detection only works with
 * the sensors listed in the linked page) or when manually specified by setting
 * the port to ``i2c`` mode and writing the device name to ``set_device``.
 * You can use any one of the sensors that has the ``nxt-i2c-sensor`` module
 * from the :ref:`supported-sensors`. Keep in mind though that the sensor mux
 * operates in a read-only mode with I2C sensors. You will not be able to use
 * commands with these sensors. Additionally, some modes of I2C sensors require
 * writing data to the sensor and as a result, these modes will not be usable
 * either.
 */

#include <linux/device.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/delay.h>

#include <lego.h>
#include <lego_sensor_class.h>

#include "nxt_i2c_sensor.h"
#include "ht_nxt_smux.h"

struct ht_nxt_smux_i2c_sensor_data {
	struct lego_device *ldev;
	const struct nxt_i2c_sensor_info *info;
	struct lego_sensor_device sensor;
	enum nxt_i2c_sensor_type type;
};

static int ht_nxt_smux_i2c_sensor_set_mode(void *context, u8 mode)
{
	struct ht_nxt_smux_i2c_sensor_data *data = context;
	struct lego_port_device *port = data->ldev->port;
	struct lego_sensor_mode_info *mode_info = &data->sensor.mode_info[mode];
	const struct nxt_i2c_sensor_mode_info *i2c_mode_info = data->info->i2c_mode_info;
	int size = lego_sensor_get_raw_data_size(mode_info);

	ht_nxt_smux_port_set_i2c_data_reg(port, i2c_mode_info[mode].read_data_reg,
					  size);
	lego_port_set_raw_data_ptr_and_func(port, mode_info->raw_data, size,
					    &mode_info->last_changed_time,
					    NULL, NULL);

	return 0;
}

static int ht_nxt_smux_i2c_sensor_probe(struct lego_device *ldev)
{
	struct ht_nxt_smux_i2c_sensor_data *data;
	const struct nxt_i2c_sensor_info *sensor_info;
	struct ht_nxt_smux_i2c_sensor_platform_data *pdata =
		ldev->dev.platform_data;
	size_t mode_info_size;
	int err, i;

	if (WARN_ON(!ldev->entry_id))
		return -EINVAL;

	if (WARN_ON(!pdata))
		return -EINVAL;

	sensor_info = &nxt_i2c_sensor_defs[ldev->entry_id->driver_data];

	if (sensor_info->ops) {
		dev_err(&ldev->dev, "The '%s' driver requires special operations"
			" that are not supported in the '%s' module.",
			ldev->entry_id->name, "ht-nxt-smux-i2c-sensor");
		return -EINVAL;
	}

	data = kzalloc(sizeof(struct ht_nxt_smux_i2c_sensor_data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	mode_info_size = sizeof(struct nxt_i2c_sensor_info) * sensor_info->num_modes;
	data->sensor.mode_info = kmalloc(mode_info_size, GFP_KERNEL);
	if (!data->sensor.mode_info) {
		err = -ENOMEM;
		goto err_kalloc_mode_info;
	}

	data->ldev = ldev;
	data->type = ldev->entry_id->driver_data;
	data->info = sensor_info;

	data->sensor.name = ldev->entry_id->name;
	data->sensor.address = data->ldev->port->address;
	if (data->info->num_read_only_modes)
		data->sensor.num_modes = data->info->num_read_only_modes;
	else
		data->sensor.num_modes = data->info->num_modes;
	data->sensor.num_view_modes = 1;
	memcpy(data->sensor.mode_info, data->info->mode_info, mode_info_size);
	data->sensor.set_mode = ht_nxt_smux_i2c_sensor_set_mode;
	data->sensor.context = data;

	for (i = 0; i < data->sensor.num_modes; i++) {
		struct lego_sensor_mode_info *minfo = &data->sensor.mode_info[i];

		if (!minfo->raw_min && !minfo->raw_max)
			minfo->raw_max = 255;
		if (!minfo->pct_min && !minfo->pct_max)
			minfo->pct_max = 100;
		if (!minfo->si_min && !minfo->si_max)
			minfo->si_max = 255;
		if (!minfo->data_sets)
			minfo->data_sets = 1;
		if (!minfo->figures)
			minfo->figures = 5;
	}

	dev_set_drvdata(&ldev->dev, data);

	err = register_lego_sensor(&data->sensor, &ldev->dev);
	if (err) {
		dev_err(&ldev->dev, "could not register sensor!\n");
		goto err_register_lego_sensor;
	}

	ldev->port->nxt_i2c_ops->set_pin1_gpio(ldev->port->context,
					       data->info->pin1_state);
	ht_nxt_smux_port_set_i2c_addr(data->ldev->port, pdata->address,
				      data->info->slow);
	ht_nxt_smux_i2c_sensor_set_mode(data, 0);

	return 0;

err_register_lego_sensor:
	kfree(data->sensor.mode_info);
err_kalloc_mode_info:
	kfree(data);

	return err;
}

static int ht_nxt_smux_i2c_sensor_remove(struct lego_device *ldev)
{
	struct ht_nxt_smux_i2c_sensor_data *data = dev_get_drvdata(&ldev->dev);

	lego_port_set_raw_data_ptr_and_func(ldev->port, NULL, 0, NULL,
					    NULL, NULL);
	ldev->port->nxt_i2c_ops->set_pin1_gpio(ldev->port->context,
					       LEGO_PORT_GPIO_FLOAT);
	unregister_lego_sensor(&data->sensor);
	dev_set_drvdata(&ldev->dev, NULL);
	kfree(data->sensor.mode_info);
	kfree(data);

	return 0;
}

static struct lego_device_id ht_nxt_smux_i2c_sensor_id_table[] = {
	NXT_I2C_SENSOR_ID_TABLE_DATA
};
MODULE_DEVICE_TABLE(legoev3, ht_nxt_smux_i2c_sensor_id_table);

static struct lego_device_driver ht_nxt_smux_i2c_sensor_driver = {
	.driver = {
		.name	= "ht-nxt-smux-i2c-sensor",
	},
	.id_table	= ht_nxt_smux_i2c_sensor_id_table,
	.probe		= ht_nxt_smux_i2c_sensor_probe,
	.remove		= ht_nxt_smux_i2c_sensor_remove,
};
lego_device_driver(ht_nxt_smux_i2c_sensor_driver);

MODULE_DESCRIPTION("HiTechnic NXT Sensor Multiplexer I2C sensor device driver");
MODULE_AUTHOR("David Lechner <david@lechnology.com>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("lego:ht-nxt-smux-i2c-sensor");
