/*
 * NXT Ultrasonic sensor device driver for LEGO Mindstorms EV3
 *
 * Copyright (C) 2013 David Lechner <david@lechnology.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.

 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/device.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/i2c.h>
//#include <linux/legoev3/classes/dist_sensor.h>
#include <linux/legoev3/legoev3_input_port.h>
#include <linux/legoev3/legoev3_ports.h>

#define FIRMWARE_REG	0x00	/* Firmware version (8 registers) */
#define	VENDOR_ID_REG	0x08	/* Vendor ID (8 registers) */
#define DEVICE_ID_REG	0x10	/* Device ID (8 registers) */

#define STR_LEN	8	/* length of string registers */

static int __devinit nxt_us_sensor_probe(struct i2c_client *client,
					 const struct i2c_device_id *id)
{printk("%s\n", __func__);
	return 0;
}

static int __devexit nxt_us_sensor_remove(struct i2c_client *client)
{printk("%s\n", __func__);
	return 0;
}

static int nxt_us_sensor_detect(struct i2c_client *client,
				struct i2c_board_info *info)
{
	char fw_ver[STR_LEN + 1] = { 0 };
	char vend_id[STR_LEN + 1] = { 0 };
	char dev_id[STR_LEN + 1] = { 0 };
	int ret;

	ret = i2c_smbus_read_i2c_block_data(client, FIRMWARE_REG, STR_LEN, fw_ver);
	if (ret < 0)
		return -ENODEV;
	ret = i2c_smbus_read_i2c_block_data(client, VENDOR_ID_REG, STR_LEN, vend_id);
	if (ret < 0)
		return -ENODEV;
	ret = i2c_smbus_read_i2c_block_data(client, DEVICE_ID_REG, STR_LEN, dev_id);
	if (ret < 0)
		return -ENODEV;
printk("%s: fw:%s, vend:%s, dev:%s\n", __func__, fw_ver, vend_id, dev_id);
	if (strcmp(vend_id, "LEGO"))
		return -ENODEV;
	if (strcmp(dev_id, "Sonar"))
		return -ENODEV;

	sprintf(info->type, "nxt-us-sensor");
	return 0;
}

static struct i2c_device_id nxt_us_sensor_idtable[] = {
	{ "nxt-us-sensor", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, nxt_us_sensor_idtable);

static struct i2c_driver nxt_us_sensor_driver = {
	.driver = {
		.name	= "nxt-us-sensor",
	},
	.id_table	= nxt_us_sensor_idtable,
	.probe		= nxt_us_sensor_probe,
	.remove		= __devexit_p(nxt_us_sensor_remove),
	.class		= I2C_CLASS_LEGOEV3,
	.detect		= nxt_us_sensor_detect,
	.address_list	= I2C_ADDRS(0x01),
};
module_i2c_driver(nxt_us_sensor_driver);

MODULE_DESCRIPTION("NXT ultrasonic sensor device driver for LEGO Mindstorms EV3");
MODULE_AUTHOR("David Lechner <david@lechnology.com>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("legoev3:legoev3-nxt-i2c-sensor");
