/*
 * Dexter Industries BrickPi driver
 *
 * Copyright (C) 2015 David Lechner <david@lechnology.com>
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
 * There is currently not a standard I2C adapter driver for BrickPi/BrickPi+
 * (i.e. You will not find ``/dev/i2c-X`` for the BrickPi input ports. Instead,
 * this driver is used for working with :ref:`nxt-i2c-sensors`.
 *
 * To load this driver, set the BrickPi input port to ``nxt-i2c`` mode and then
 * write the driver name and I2C address of the sensor to the ``set_device``
 * attribute. You can find the driver name and default I2C address on the
 * sensor's information page.
 *
 * Example::
 *
 *    echo "nxt-i2c" > /sys/class/lego-port/port0/mode
 *    echo "ht-angle 0x01" > /sys/class/lego-port/port0/set_device
 *
 * Once the driver has been loaded, there will be a new sensor device node
 * added in the :ref:`lego-sensor-class`.
 *
 * Unfortunately, some sensors require special operations that are not currently
 * supported in this driver, so attempting to use one of those sensors will
 * fail (check ``dmesg``). If you have a BrickPi, you can use `Input
 * Port 5`_ instead.
 */

#include <linux/device.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/delay.h>

#include <lego.h>
#include <lego_sensor_class.h>

#include "../sensors/nxt_i2c_sensor.h"
#include "brickpi.h"

struct brickpi_i2c_sensor_data {
	struct lego_device *ldev;
	char address[LEGO_NAME_SIZE + 1];
	const struct nxt_i2c_sensor_info *info;
	struct lego_sensor_device sensor;
	enum nxt_i2c_sensor_type type;
};

static int brickpi_i2c_sensor_set_mode(void *context, u8 mode)
{
	struct brickpi_i2c_sensor_data *data = context;
	struct lego_port_device *port = data->ldev->port;
	struct lego_sensor_mode_info *mode_info = &data->sensor.mode_info[mode];
	const struct nxt_i2c_sensor_mode_info *i2c_mode_info = data->info->i2c_mode_info;
	int size = lego_sensor_get_raw_data_size(mode_info);
	int err;

	err = brickpi_in_port_set_i2c_mode(data->ldev,
					   i2c_mode_info[mode].set_mode_reg,
					   i2c_mode_info[mode].set_mode_data,
					   i2c_mode_info[mode].read_data_reg,
					   size);
	if (err < 0)
		return err;

	lego_port_set_raw_data_ptr_and_func(port, mode_info->raw_data, size,
					    &mode_info->last_changed_time,
					    NULL, NULL);

	return 0;
}

static int brickpi_i2c_sensor_send_command(void *context, u8 mode)
{
	struct brickpi_i2c_sensor_data *data = context;
	struct lego_sensor_mode_info *mode_info = &data->sensor.mode_info[mode];
	const struct nxt_i2c_sensor_cmd_info *i2c_cmd_info = data->info->i2c_cmd_info;
	const struct nxt_i2c_sensor_mode_info *i2c_mode_info = data->info->i2c_mode_info;
	int size = lego_sensor_get_raw_data_size(mode_info);
	int err; /* temporary workaround for ms-absolute-imu (2 of 4) */

	/* set mode function also works for sending command */
	err = brickpi_in_port_set_i2c_mode(data->ldev,
					    i2c_cmd_info[mode].cmd_reg,
					    i2c_cmd_info[mode].cmd_data,
					    i2c_mode_info[mode].read_data_reg,
					    size);

	/* temporary workaround for ms-absolute-imu (3 of 4) */
	if (err)
		return err;

	/* temporary workaround for ms-absolute-imu (4 of 4) */
	if (data->ldev->entry_id->driver_data == MS_ABSOLUTE_IMU &&
	data->info->ops && data->info->ops->send_cmd_post_cb) {
		struct nxt_i2c_sensor_data workaround_sensor;

		workaround_sensor.sensor.mode_info = data->sensor.mode_info;
		data->info->ops->send_cmd_post_cb(&workaround_sensor, mode);
	}

	return err;
}

static int brickpi_i2c_sensor_probe(struct lego_device *ldev)
{
	struct brickpi_i2c_sensor_data *data;
	const struct nxt_i2c_sensor_info *sensor_info;
	struct brickpi_i2c_sensor_platform_data *pdata =
		ldev->dev.platform_data;
	size_t mode_info_size;
	int err, i;

	if (WARN_ON(!ldev->entry_id))
		return -EINVAL;

	if (WARN_ON(!pdata))
		return -EINVAL;

	sensor_info = &nxt_i2c_sensor_defs[ldev->entry_id->driver_data];

	/* below temporary workaround for ms_absolute_imu (1 of 4) */
	if (sensor_info->ops &&
		ldev->entry_id->driver_data != MS_ABSOLUTE_IMU) {
		dev_err(&ldev->dev, "The '%s' driver requires special operations"
			" that are not supported in the '%s' module.",
			ldev->entry_id->name, "brickpi-i2c-sensor");
		return -EINVAL;
	}

	data = kzalloc(sizeof(struct brickpi_i2c_sensor_data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	mode_info_size = sizeof(struct lego_sensor_mode_info) * sensor_info->num_modes;
	data->sensor.mode_info = kmalloc(mode_info_size, GFP_KERNEL);
	if (!data->sensor.mode_info) {
		err = -ENOMEM;
		goto err_kalloc_mode_info;
	}

	data->ldev = ldev;
	data->type = ldev->entry_id->driver_data;
	data->info = sensor_info;

	data->sensor.name = ldev->entry_id->name;
	snprintf(data->address, LEGO_NAME_SIZE, "%s:i2c%d",
		 data->ldev->port->address, pdata->address);
	data->sensor.address = data->address;
	data->sensor.num_modes = data->info->num_modes;
	data->sensor.num_view_modes = 1;
	memcpy(data->sensor.mode_info, data->info->mode_info, mode_info_size);
	data->sensor.set_mode = brickpi_i2c_sensor_set_mode;
	data->sensor.send_command = brickpi_i2c_sensor_send_command;
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

	brickpi_in_port_set_i2c_data(ldev, data->info->slow, data->info->pin1_state);
	brickpi_i2c_sensor_set_mode(data, 0);

	return 0;

err_register_lego_sensor:
	kfree(data->sensor.mode_info);
err_kalloc_mode_info:
	kfree(data);

	return err;
}

static int brickpi_i2c_sensor_remove(struct lego_device *ldev)
{
	struct brickpi_i2c_sensor_data *data = dev_get_drvdata(&ldev->dev);

	lego_port_set_raw_data_ptr_and_func(ldev->port, NULL, 0, NULL,
					    NULL, NULL);
	unregister_lego_sensor(&data->sensor);
	dev_set_drvdata(&ldev->dev, NULL);
	kfree(data->sensor.mode_info);
	kfree(data);

	return 0;
}

static struct lego_device_id brickpi_i2c_sensor_id_table[] = {
	NXT_I2C_SENSOR_ID_TABLE_DATA
};
MODULE_DEVICE_TABLE(legoev3, brickpi_i2c_sensor_id_table);

static struct lego_device_driver brickpi_i2c_sensor_driver = {
	.driver = {
		.name	= "brickpi-i2c-sensor",
	},
	.id_table	= brickpi_i2c_sensor_id_table,
	.probe		= brickpi_i2c_sensor_probe,
	.remove		= brickpi_i2c_sensor_remove,
};
lego_device_driver(brickpi_i2c_sensor_driver);

MODULE_DESCRIPTION("Dexter Industries BrickPi I2C sensor device driver");
MODULE_AUTHOR("David Lechner <david@lechnology.com>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("lego:brickpi-i2c-sensor");
