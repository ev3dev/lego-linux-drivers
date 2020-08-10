/*
 * LEGO MINDSTORMS NXT I2C sensor device driver
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
 * The ``nxt-i2c-sensor`` module provides all of the drivers for NXT/I2C sensors.
 *
 * .. flat-table:: Module Parameters
 *    :widths: 1 5
 *
 *    * - ``allow_autodetect``
 *      - Setting to ``N`` disables probing of sensors. Default is ``Y``.
 *
 *    * - ``default_poll_ms``
 *      - This provides the default value for the ``poll_ms`` attribute. A value
 *        of 0 will disable polling by default. Changes only affect sensors
 *        plugged in after the change was made. Default is 100 msec. Values must
 *        not be negative.
 *
 * .. note:: These parameters can be changed at runtime by writing to
 *    ``/sys/module/nxt_i2c_sensor/parameters/<parameter>``.
 *
 * You can a list of the the devices implemented by this module by reading the
 * ``driver_names`` attribute in the ``/sys/bus/lego/drivers/nxt-i2c-sensor/``.
 */

#include <linux/device.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/bug.h>
#include <linux/i2c.h>
#include <linux/workqueue.h>

#include <lego_sensor_class.h>

#include "nxt_i2c_sensor.h"

#if defined(CONFIG_LEGOEV3_I2C) || defined(CONFIG_LEGOEV3_I2C_MODULE)
#include "../ev3/legoev3_i2c.h"
#endif

#ifndef I2C_CLASS_LEGO
#define I2C_CLASS_LEGO (1<<31)
#endif

static u16 default_poll_ms = 100;
module_param(default_poll_ms, ushort, 0644);
MODULE_PARM_DESC(default_poll_ms, "Polling period in milliseconds.Set to 0 to disable polling.");
static bool allow_autodetect = 1;
module_param(allow_autodetect, bool, 0644);
MODULE_PARM_DESC(allow_autodetect, "Allow NXT I2C sensors to be automatically detected.");

void nxt_i2c_sensor_poll_work(struct work_struct *work);

static int nxt_i2c_sensor_set_mode(void *context, u8 mode)
{
	struct nxt_i2c_sensor_data *sensor = context;
	int err;

	if (sensor->info->ops && sensor->info->ops->set_mode_pre_cb) {
		err = sensor->info->ops->set_mode_pre_cb(sensor, mode);
		if (err < 0)
			return err;
	}

	hrtimer_cancel(&sensor->poll_timer);

	if (sensor->info->i2c_mode_info[mode].set_mode_reg) {
		err = i2c_smbus_write_byte_data(sensor->client,
			sensor->info->i2c_mode_info[mode].set_mode_reg,
			sensor->info->i2c_mode_info[mode].set_mode_data);
		if (err < 0)
			return err;
	}

	/* If we are not polling, we still call the poll function once */
	if (sensor->poll_ms)
		hrtimer_start(&sensor->poll_timer, ktime_set(0, 0), HRTIMER_MODE_REL);
	else
		nxt_i2c_sensor_poll_work(&sensor->poll_work);

	if (sensor->info->ops && sensor->info->ops->set_mode_post_cb)
		sensor->info->ops->set_mode_post_cb(sensor, mode);

	return 0;
}

static int nxt_i2c_sensor_send_command(void *context, u8 command)
{
	struct nxt_i2c_sensor_data *sensor = context;
	int err;

	if (sensor->info->ops && sensor->info->ops->send_cmd_pre_cb) {
		err = sensor->info->ops->send_cmd_pre_cb(sensor, command);
		if (err)
			return err;
	}

	err = i2c_smbus_write_byte_data(sensor->client,
		sensor->info->i2c_cmd_info[command].cmd_reg,
		sensor->info->i2c_cmd_info[command].cmd_data);
	if (err)
		return err;

	if (sensor->info->ops && sensor->info->ops->send_cmd_post_cb)
		sensor->info->ops->send_cmd_post_cb(sensor, command);

	return 0;
}

static ssize_t nxt_i2c_sensor_direct_read(void *context, char *data, loff_t off,
					  size_t count)
{
	struct nxt_i2c_sensor_data *sensor = context;
	int err;

	if (off > 255)
		return -EINVAL;

	err = i2c_smbus_read_i2c_block_data(sensor->client, off, count, data);
	if (err < 0)
		return err;

	return count;
}

static ssize_t nxt_i2c_sensor_direct_write(void *context, char *data, loff_t off,
					   size_t count)
{
	struct nxt_i2c_sensor_data *sensor = context;
	int err;

	if (off > 255)
		return -EINVAL;

	err = i2c_smbus_write_i2c_block_data(sensor->client, off, count, data);
	if (err < 0)
		return err;

	return count;
}

static int nxt_i2c_sensor_get_poll_ms(void *context)
{
	struct nxt_i2c_sensor_data *sensor = context;

	return sensor->poll_ms;
}

static int nxt_i2c_sensor_set_poll_ms(void *context, unsigned value)
{
	struct nxt_i2c_sensor_data *sensor = context;

	if (sensor->poll_ms == value)
		return 0;
	if (!value)
		hrtimer_cancel(&sensor->poll_timer);
	else if (!sensor->poll_ms)
		hrtimer_start(&sensor->poll_timer, ktime_set(0, 0),
			HRTIMER_MODE_REL);

	sensor->poll_ms = value;
	return 0;
}

enum hrtimer_restart nxt_i2c_sensor_poll_timer(struct hrtimer *timer)
{
	struct nxt_i2c_sensor_data *data =
		container_of(timer, struct nxt_i2c_sensor_data, poll_timer);

	hrtimer_forward_now(timer, ms_to_ktime(data->poll_ms));
	schedule_work(&data->poll_work);

	return HRTIMER_RESTART;
}

void nxt_i2c_sensor_poll_work(struct work_struct *work)
{
	struct nxt_i2c_sensor_data *data =
		container_of(work, struct nxt_i2c_sensor_data, poll_work);
	const struct nxt_i2c_sensor_mode_info *i2c_mode_info =
		&data->info->i2c_mode_info[data->sensor.mode];
	struct lego_sensor_mode_info *mode_info =
			&data->sensor.mode_info[data->sensor.mode];

	u8 old_data[32];
	int raw_data_size = lego_sensor_get_raw_data_size(mode_info);
	int check_size = raw_data_size <= 32 ? raw_data_size : 32;


	if (data->info->ops && data->info->ops->poll_cb) {
		data->info->ops->poll_cb(data);
	} else {

		memcpy(old_data, mode_info->raw_data, check_size);

		i2c_smbus_read_i2c_block_data(data->client,
			i2c_mode_info->read_data_reg,
			raw_data_size,
			mode_info->raw_data);

		if (memcmp(old_data, mode_info->raw_data, check_size) != 0) {
			mode_info->last_changed_time = ktime_get();
		}
	}
}

static int nxt_i2c_sensor_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{
	struct nxt_i2c_sensor_platform_data *pdata = client->dev.platform_data;
	struct nxt_i2c_sensor_data *data;
	struct lego_port_device *in_port = NULL;
	const struct nxt_i2c_sensor_info *sensor_info;
	const struct i2c_device_id *i2c_dev_id = id;
	char version[NXT_I2C_ID_STR_LEN + 1] = { 0 };
	size_t mode_info_size;
	int err, i;

	sensor_info = &nxt_i2c_sensor_defs[i2c_dev_id->driver_data];

	if (pdata)
		in_port = pdata->in_port;

	data = kzalloc(sizeof(struct nxt_i2c_sensor_data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	mode_info_size = sizeof(struct lego_sensor_mode_info) * sensor_info->num_modes;
	data->sensor.mode_info = kmalloc(mode_info_size, GFP_KERNEL);
	if (!data->sensor.mode_info) {
		err = -ENOMEM;
		goto err_kalloc_mode_info;
	}

	data->client = client;
	data->in_port = in_port;
	data->type = i2c_dev_id->driver_data;
	data->info = sensor_info;

	data->sensor.name = i2c_dev_id->name;
	snprintf(data->address, LEGO_NAME_SIZE, "%s:i2c%d",
		 in_port ? in_port->address : client->adapter->name,
		 client->addr);

	data->sensor.address = data->address;
	data->sensor.num_modes = data->info->num_modes;
	data->sensor.num_view_modes = 1;
	memcpy(data->sensor.mode_info, data->info->mode_info, mode_info_size);
	data->sensor.num_commands = data->info->num_commands;
	data->sensor.cmd_info = data->info->cmd_info;
	data->sensor.set_mode = nxt_i2c_sensor_set_mode;
	data->sensor.send_command = nxt_i2c_sensor_send_command;
	data->sensor.direct_read = nxt_i2c_sensor_direct_read;
	data->sensor.direct_write = nxt_i2c_sensor_direct_write;
	data->sensor.get_poll_ms = nxt_i2c_sensor_get_poll_ms;
	data->sensor.set_poll_ms = nxt_i2c_sensor_set_poll_ms;
	data->sensor.context = data;
	i2c_smbus_read_i2c_block_data(client, NXT_I2C_FW_VER_REG,
				      NXT_I2C_ID_STR_LEN, version);
	/*
	 * HiTechnic sensors have 0xfd before the version because of an early
	 * bug in the NXT I2C code where register 0x00 could not be read
	 * reliably. So, we just ignore it along with the whitespace.
	 */
	strncpy(data->sensor.fw_version, strim(version[0] == 0xfd ?
		(version + 1) : version), NXT_I2C_ID_STR_LEN + 1);

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

	INIT_WORK(&data->poll_work, nxt_i2c_sensor_poll_work);
	hrtimer_init(&data->poll_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	data->poll_timer.function = nxt_i2c_sensor_poll_timer;
	data->poll_ms = default_poll_ms;
	i2c_set_clientdata(client, data);

	if (data->info->ops && data->info->ops->probe_cb) {
		err = data->info->ops->probe_cb(data);
		if (err < 0)
			goto err_probe_cb;
	}

	err = register_lego_sensor(&data->sensor, &client->dev);
	if (err) {
		dev_err(&client->dev, "could not register sensor!\n");
		goto err_register_lego_sensor;
	}

	if (data->in_port && data->in_port->nxt_i2c_ops)
		data->in_port->nxt_i2c_ops->set_pin1_gpio(data->in_port->context,
							  data->info->pin1_state);
	if (data->type == LEGO_NXT_ULTRASONIC_SENSOR)
		msleep (1);
	nxt_i2c_sensor_set_mode(data, data->sensor.mode);

	return 0;

err_probe_cb:
err_register_lego_sensor:
	i2c_set_clientdata(client, NULL);
	kfree(data->sensor.mode_info);
err_kalloc_mode_info:
	kfree(data);

	return err;
}

static int nxt_i2c_sensor_remove(struct i2c_client *client)
{
	struct nxt_i2c_sensor_data *data = i2c_get_clientdata(client);

	if (data->info->ops && data->info->ops->remove_cb)
		data->info->ops->remove_cb(data);
	data->poll_ms = 0;
	hrtimer_cancel(&data->poll_timer);
	cancel_work_sync(&data->poll_work);
	if (data->in_port && data->in_port->nxt_i2c_ops)
		data->in_port->nxt_i2c_ops->set_pin1_gpio(data->in_port->context,
							  LEGO_PORT_GPIO_FLOAT);
	unregister_lego_sensor(&data->sensor);
	kfree(data->sensor.mode_info);
	kfree(data);

	return 0;
}

static struct i2c_device_id nxt_i2c_sensor_id_table[];

static int nxt_i2c_sensor_detect_lego_power(struct i2c_client *client)
{
	int ret;

	/*
	 * Confirm there is a device that satisfies the LMS2012 checks.
	 */
	ret = i2c_smbus_read_byte_data(client, 0x08);
	if (ret != 0)
		return -ENODEV;
	ret = i2c_smbus_read_byte_data(client, 0x10);
	if (ret != 0)
		return -ENODEV;

	return 0;
}

static int nxt_i2c_sensor_detect_lego_temp(struct i2c_client *client)
{
	int ret;

	/*
	 * Try to set the sensor to 12-bit mode.
	 */
	ret = i2c_smbus_write_byte_data(client, 0x01, 0x60);
	if (ret)
		return ret;

	/*
	 * Confirm this is a NXT Temperature Sensor.
	 */
	ret = i2c_smbus_read_byte_data(client, 0x01);
	if (ret < 0 || ret != 0x60)
		return -ENODEV;

	return 0;
}

static int nxt_i2c_sensor_detect(struct i2c_client *client,
				 struct i2c_board_info *info)
{
	char vendor_id[NXT_I2C_ID_STR_LEN + 1] = { 0 };
	char product_id[NXT_I2C_ID_STR_LEN + 1] = { 0 };
	int ret, i;
	int tries = 2;

	if (!allow_autodetect)
		return -ENODEV;

#if defined(CONFIG_LEGOEV3_I2C) || defined(CONFIG_LEGOEV3_I2C_MODULE)
	if (!info->platform_data && client->adapter->algo == &i2c_legoev3_algo) {
		struct i2c_legoev3_platform_data *apdata =
					client->adapter->dev.platform_data;

		info->platform_data = &apdata->sensor_platform_data;
	}
#endif

	/*
	 * NXT Power Storage and Temperature sensor detection are handled here.
	 *
	 * These sensors do not conform to the LEGO I2C specification and so
	 * cannot be handled by the normal auto-detection code. However, they
	 * are LEGO-brand sensors that are supported by the Mindstorms software,
	 * and so are treated here as special cases.
	 *
	 * We try to detect the sensors at addresses 0x02 and 0x4c, but fall
	 * through to the standard path when there is a device there that
	 * does not match expectation in case the address is being used by
	 * another device.
	 */
	switch (client->addr) {
	case 0x02:
		ret = nxt_i2c_sensor_detect_lego_power(client);
		if (ret == 0) {
			snprintf(info->type, I2C_NAME_SIZE, "%s",
				LEGO_POWER_STORAGE_SENSOR_NAME);
			return 0;
		}
		break;
	case 0x4c:
		ret = nxt_i2c_sensor_detect_lego_temp(client);
		if (ret == 0) {
			snprintf(info->type, I2C_NAME_SIZE, "%s",
				LEGO_NXT_TEMPERATURE_SENSOR_NAME);
			return 0;
		}
	}

	/*
	 * Some sensors can fall asleep during boot, so we try reading twice
	 * to make sure we wake them up.
	 */
	while (tries--) {
		ret = i2c_smbus_read_i2c_block_data(client, NXT_I2C_VEND_ID_REG,
						    NXT_I2C_ID_STR_LEN, vendor_id);
		if (ret > 0 || !tries)
			break;
		msleep(1);
	}
	if (ret < 0)
		return -ENODEV;

	/*
	 * NXT Ultrasonic sensor requires a long delay between reads or else
	 * we will get NAKed. msleep(1) tends to vary between 10 and 20msec.
	 */
	msleep(1);
	ret = i2c_smbus_read_i2c_block_data(client, NXT_I2C_PROD_ID_REG,
					    NXT_I2C_ID_STR_LEN, product_id);
	if (ret < 0)
		return -ENODEV;

	for (i = 0; i < NUM_NXT_I2C_SENSORS; i++) {
		if (strcmp(nxt_i2c_sensor_defs[i].vendor_id, strim(vendor_id))) {
			if (!nxt_i2c_sensor_defs[i].alt_vendor_id)
				continue; /* vendor_id does not match and no alt */
			if (strcmp(nxt_i2c_sensor_defs[i].alt_vendor_id, strim(vendor_id)))
				continue; /* alt_vendor_id does not match */
		}
		if (strcmp(nxt_i2c_sensor_defs[i].product_id, strim(product_id)))
			continue; /* product_id does not match */

		snprintf(info->type, I2C_NAME_SIZE, nxt_i2c_sensor_defs[i].name);
		return 0;
	}

	return -ENODEV;
}

static struct i2c_device_id nxt_i2c_sensor_id_table[] = {
	NXT_I2C_SENSOR_ID_TABLE_DATA
};
MODULE_DEVICE_TABLE(i2c, nxt_i2c_sensor_id_table);

struct i2c_driver nxt_i2c_sensor_driver = {
	.driver = {
		.name	= "nxt-i2c-sensor",
	},
	.id_table	= nxt_i2c_sensor_id_table,
	.probe		= nxt_i2c_sensor_probe,
	.remove		= nxt_i2c_sensor_remove,
	.class		= I2C_CLASS_LEGO,
	.detect		= nxt_i2c_sensor_detect,
	.address_list	= I2C_ADDRS(0x01, 0x02, 0x03, 0x08, 0x0a, 0x0c, 0x11, 0x15, 
				    0x18, 0x4c, 0x50, 0x51, 0x52, 0x58),
};
module_i2c_driver(nxt_i2c_sensor_driver);
EXPORT_SYMBOL_GPL(nxt_i2c_sensor_driver);

MODULE_DESCRIPTION("LEGO MINDSTORMS NXT I2C sensor device driver");
MODULE_AUTHOR("David Lechner <david@lechnology.com>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("lego:nxt-i2c-sensor");
MODULE_ALIAS("lego:nxt-i2c-host");
