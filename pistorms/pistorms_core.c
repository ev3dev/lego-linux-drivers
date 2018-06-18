/*
 * mindsensors.com PiStorms device driver
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
 * The ``pistorms`` module provides drivers for most of the functionality of the
 * `mindsensors.com PiStorms <pistorms>`_ including lego-port class instances
 * for the input and output ports, an evdev driver (evdev is not to be confused
 * with ev3dev) for the touchscreen and GO button, a power_supply driver for
 * monitoring the battery and an leds driver for the multi-color LEDs.
 */

#include <linux/bug.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/notifier.h>
#include <linux/reboot.h>
#include <linux/slab.h>
#include <linux/workqueue.h>

#include "pistorms.h"
#include "../sensors/nxt_i2c_sensor.h"

#define PISTORMS_BANK_A_ADDR	0x1a
#define PISTORMS_BANK_B_ADDR	0x1b

static int _pistorms_detect(struct i2c_client *client)
{
	char vendor_id[NXT_I2C_ID_STR_LEN + 1] = { 0 };
	char product_id[NXT_I2C_ID_STR_LEN + 1] = { 0 };
	int ret;

	ret = i2c_smbus_read_i2c_block_data(client, NXT_I2C_VEND_ID_REG,
					    NXT_I2C_ID_STR_LEN, vendor_id);
	if (ret < 0 || strcmp(vendor_id, "mndsnsrs"))
		return -ENODEV;

	ret = i2c_smbus_read_i2c_block_data(client, NXT_I2C_PROD_ID_REG,
					    NXT_I2C_ID_STR_LEN, product_id);
	if (ret < 0 || strcmp(product_id, "PiStorms"))
		return -ENODEV;

	return 0;
}

static struct i2c_client *pistorms_reboot_client;

static int pistorms_reboot_notifier_call(struct notifier_block *nb,
					 unsigned long action, void *data)
{
	if (pistorms_reboot_client && action == SYS_POWER_OFF) {
		i2c_smbus_write_byte_data(pistorms_reboot_client, 0x41, 'H');
		return NOTIFY_OK;
	}

	return NOTIFY_DONE;
}

static struct notifier_block pistorms_reboot_notifier = {
	.notifier_call = pistorms_reboot_notifier_call,
};

static int pistorms_probe(struct i2c_client *client,
			  const struct i2c_device_id *id)
{
	struct pistorms_data *data;
	int ret;

	ret = _pistorms_detect(client);
	if (ret) {
		dev_err(&client->dev, "PiStorms not found at 0x%02x\n",
			client->addr);
		return ret;
	}

	data = kzalloc(sizeof(struct pistorms_data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	snprintf(data->name, PISTORMS_NAME_SIZE, "pistorms:B%c",
		'A' + client->addr - PISTORMS_BANK_A_ADDR);
	data->client = client;

	i2c_set_clientdata(client, data);

	/* some devices are duplicated on both banks */
	if (client->addr == PISTORMS_BANK_A_ADDR) {
		ret = devm_pistorms_register_board(&client->dev, data);
		if (ret < 0)
			dev_err(&client->dev, "Failed to register board info: %d",
				ret);
		ret = pistorms_battery_register(data);
		if (ret < 0)
			goto err_pistorms_battery_register;
		ret = pistorms_input_register(data);
		if (ret < 0)
			goto err_pistorms_input_register;

		if (!pistorms_reboot_client) {
			pistorms_reboot_client = client;
			ret = register_reboot_notifier(&pistorms_reboot_notifier);
			if (ret < 0) {
				dev_warn(&client->dev,
					 "Failed to register reboot notifier (%d)\n",
					 ret);
				pistorms_reboot_client = NULL;
			}
		}
	}
	ret = pistorms_leds_register(data);
	if (ret < 0)
		goto err_pistorms_leds_register;
	ret = pistorms_in_ports_register(data);
	if (ret < 0)
		goto err_pistorms_in_ports_register;
	ret = pistorms_out_ports_register(data);
	if (ret < 0)
		goto err_pistorms_out_ports_register;

	dev_info(&client->dev, "Registered pistorms.");
	return 0;

	pistorms_out_ports_unregister(data);
err_pistorms_out_ports_register:
	pistorms_in_ports_unregister(data);
err_pistorms_in_ports_register:
	pistorms_leds_unregister(data);
err_pistorms_leds_register:
	unregister_reboot_notifier(&pistorms_reboot_notifier);
	pistorms_reboot_client = NULL;
	pistorms_input_unregister(data);
err_pistorms_input_register:
	pistorms_battery_unregister(data);
err_pistorms_battery_register:
	i2c_set_clientdata(client, NULL);
	kfree(data);

	return ret;
}

static int pistorms_remove(struct i2c_client *client)
{
	struct pistorms_data *data = i2c_get_clientdata(client);
	pistorms_out_ports_unregister(data);
	pistorms_in_ports_unregister(data);
	pistorms_leds_unregister(data);
	unregister_reboot_notifier(&pistorms_reboot_notifier);
	pistorms_reboot_client = NULL;
	pistorms_input_unregister(data);
	pistorms_battery_unregister(data);
	i2c_set_clientdata(client, NULL);
	dev_info(&client->dev, "Unregistered pistorms.");
	return 0;
}

/*
 * Note: This function is probably not needed. Since we don't declare a .class
 * in the i2c_driver struct, it won't be called.
 */
static int pistorms_detect(struct i2c_client *client,
			   struct i2c_board_info *info)
{
	int ret;

	ret = _pistorms_detect(client);
	if (ret)
		return ret;

	strcpy(info->type, "pistorms");

	return 0;
}

static const struct i2c_device_id pistorms_id_table[] = {
	{ "pistorms" },
	{ }
};
MODULE_DEVICE_TABLE(i2c, pistorms_id_table);

#ifdef CONFIG_OF
static const struct of_device_id pistorms_of_match[] = {
	{ .compatible = "mindsensors,pistorms" },
	{ }
};
MODULE_DEVICE_TABLE(of, pistorms_of_match);
#endif

static struct i2c_driver pistorms_driver = {
	.driver = {
		.name	= "pistorms",
	},
	.id_table	= pistorms_id_table,
	.probe		= pistorms_probe,
	.remove		= pistorms_remove,
	.detect		= pistorms_detect,
	.address_list	= I2C_ADDRS(PISTORMS_BANK_A_ADDR, PISTORMS_BANK_B_ADDR),
};
module_i2c_driver(pistorms_driver);

MODULE_DESCRIPTION("mindsensors.com PiStorms driver");
MODULE_AUTHOR("David Lechner <david@lechnology.com>");
MODULE_LICENSE("GPL");
