/*
 * NXT Ultrasonic sensor device driver for LEGO Mindstorms EV3
 *
 * Copyright (C) 2013-2014 David Lechner <david@lechnology.com>
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
#include <linux/delay.h>
#include <linux/bug.h>
#include <linux/i2c-legoev3.h>
#include <linux/legoev3/legoev3_ports.h>
#include <linux/legoev3/ev3_input_port.h>
#include <linux/legoev3/nxt_i2c_sensor.h>
#include <linux/legoev3/measure_sensor_class.h>

#define VENDOR_ID	"LEGO"
#define PRODUCT_ID	"Sonar"

#define CONT_MEASURE_REG	0x40
#define CMD_STATE_REG		0x41
#define MEASURE_0_REG		0x42
#define ACTUAL_ZERO_REG		0x50
#define ACTUAL_SCALE_REG	0x51
#define ACTUAL_DIV_REG		0x52


struct nxt_us_data {
	struct i2c_client *client;
	struct legoev3_port_device *in_port;
	struct measure_sensor_device ms;
	char fw_ver[ID_STR_LEN + 1];
};

static struct measure_sensor_scale_info nxt_us_scale_info[] = {
	{
		.units	= "cm",
		.min	= 0,
		.max	= 255,
		.dp	= 0,
	},
	{
		.units	= "in",
		.min	= 0,
		.max	= 1000,
		.dp	= 1,
	},
	END_SCALE_INFO
};

static int nxt_us_cal_value(struct measure_sensor_device *ms)
{
	struct nxt_us_data *nxt_us = container_of(ms, struct nxt_us_data, ms);

	return nxt_i2c_read_byte(nxt_us->client, MEASURE_0_REG);
}

static int __devinit nxt_us_probe(struct i2c_client *client,
				  const struct i2c_device_id *id)
{
	struct nxt_us_data *nxt_us;
	struct i2c_legoev3_platform_data *pdata =
					client->adapter->dev.platform_data;
	int err;

	if (WARN_ON(!pdata))
		return -EINVAL;
	if (WARN_ON(!pdata->in_port))
		return -EINVAL;
	nxt_us = kzalloc(sizeof(struct nxt_us_data), GFP_KERNEL);
	if (!nxt_us)
		return -ENOMEM;

	nxt_us->client = client;
	nxt_us->in_port = pdata->in_port;
	nxt_us->ms.name = "distance";
	nxt_us->ms.id = -1;
	/*
	 * NXT Ultrasonic does not provide uncalibrated raw value, so we
	 * just use the calibrated value.
	 */
	nxt_us->ms.raw_value = nxt_us_cal_value;
	nxt_us->ms.raw_min = 0;
	nxt_us->ms.raw_max = 255;
	nxt_us->ms.cal_value = nxt_us_cal_value;
	nxt_us->ms.cal_min = 0;
	nxt_us->ms.cal_max = 255;
	nxt_us->ms.scale_info = nxt_us_scale_info;
	nxt_i2c_read_string(client, FIRMWARE_REG, nxt_us->fw_ver, ID_STR_LEN);

	err = register_measure_sensor(&nxt_us->ms, &client->dev);
	if (err) {
		dev_err(&client->dev, "could not register measurement sensor!\n");
		goto err_register_measure_sensor;
	}

	ev3_input_port_set_pin1_out(nxt_us->in_port, 1);
	i2c_set_clientdata(client, nxt_us);
	dev_info(&client->dev, "NXT Ultrasonic sensor registered as '%s'\n",
		 dev_name(&client->dev));

	return 0;

err_register_measure_sensor:
	kfree(nxt_us);

	return err;
}

static int __devexit nxt_us_remove(struct i2c_client *client)
{
	struct nxt_us_data *nxt_us = i2c_get_clientdata(client);

	dev_info(&client->dev, "NXT Ultrasonic sensor '%s' removed.\n",
		 dev_name(&client->dev));
	ev3_input_port_set_pin1_out(nxt_us->in_port, 0);
	unregister_measure_sensor(&nxt_us->ms);
	kfree(nxt_us);

	return 0;
}

static int nxt_us_detect(struct i2c_client *client, struct i2c_board_info *info)
{
	int ret;

	ret = nxt_i2c_test_id_string(client, VENDOR_ID_REG, VENDOR_ID);
	if (ret)
		return -ENODEV;
	/*
	 * NXT Ultrasonic sensor requires a long delay between reads or else
	 * we will get NAKed. msleep(1) tends to vary between 10 and 20msec.
	 */
	msleep(1);
	ret = nxt_i2c_test_id_string(client, DEVICE_ID_REG, PRODUCT_ID);
	if (ret)
		return -ENODEV;

	sprintf(info->type, "nxt-ultrasonic");
	return 0;
}

static struct i2c_device_id nxt_us_idtable[] = {
	{ "nxt-ultrasonic", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, nxt_us_idtable);

static struct i2c_driver nxt_us_driver = {
	.driver = {
		.name	= "nxt-ultrasonic",
	},
	.id_table	= nxt_us_idtable,
	.probe		= nxt_us_probe,
	.remove		= __devexit_p(nxt_us_remove),
	.class		= I2C_CLASS_LEGOEV3,
	.detect		= nxt_us_detect,
	.address_list	= I2C_ADDRS(0x01),
};
module_i2c_driver(nxt_us_driver);

MODULE_DESCRIPTION("NXT ultrasonic sensor device driver for LEGO Mindstorms EV3");
MODULE_AUTHOR("David Lechner <david@lechnology.com>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("legoev3:nxt-i2c-sensor");
