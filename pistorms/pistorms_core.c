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

/*
 * Note: The comment block below is used to generate docs on the ev3dev website.
 * Use kramdown (markdown) syntax. Use a '.' as a placeholder when blank lines
 * or leading whitespace is important for the markdown syntax.
 */

/**
 * DOC: website
 *
 * mindsensors.com PiStorms
 *
 * TODO: Add some docs.
 */

#include <linux/device.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/bug.h>
#include <linux/i2c.h>
#include <linux/workqueue.h>

#include "pistorms.h"
#include "../sensors/nxt_i2c_sensor.h"

#define PISTORMS_BANK_A_ADDR	0x1a
#define PISTORMS_BANK_B_ADDR	0x1b

static int pistorms_probe(struct i2c_client *client,
			  const struct i2c_device_id *id)
{
	struct pistorms_data *data;
	int ret;

	data = kzalloc(sizeof(struct pistorms_data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	snprintf(data->name, PISTORMS_NAME_SIZE, "pistorms:B%c",
		'A' + client->addr - PISTORMS_BANK_A_ADDR);
	data->client = client;

	i2c_set_clientdata(client, data);

	/* some devices are duplicated on both banks */
	if (client->addr == PISTORMS_BANK_A_ADDR) {
		ret = pistorms_battery_register(data);
		if (ret < 0)
			goto err_pistorms_battery_register;
		ret = pistorms_input_register(data);
		if (ret < 0)
			goto err_pistorms_input_register;
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
	char vendor_id[NXT_I2C_ID_STR_LEN + 1] = { 0 };
	char product_id[NXT_I2C_ID_STR_LEN + 1] = { 0 };
	int ret;

	ret = i2c_smbus_read_i2c_block_data(client, NXT_I2C_VEND_ID_REG,
					    NXT_I2C_ID_STR_LEN, vendor_id);
	if (ret < 0 || strcmp(vendor_id, "mndsnsrs"))
		return -ENODEV;

	ret = i2c_smbus_read_i2c_block_data(client, NXT_I2C_PROD_ID_REG,
					    NXT_I2C_ID_STR_LEN, product_id);
	if (ret < 0 || strcmp(vendor_id, "PiStorms"))
		return -ENODEV;

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
