/*
 * Mindsensors Light Array sensor device driver for LEGO Mindstorms EV3
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
#include <linux/bug.h>
#include <linux/i2c-legoev3.h>
#include <linux/legoev3/legoev3_ports.h>
#include <linux/legoev3/ev3_input_port.h>
#include <linux/legoev3/nxt_i2c_sensor.h>
#include <linux/legoev3/measure_sensor_class.h>

#define COMMAND_REG	0x41	/* Command */
#define CAL_READ_REG	0x42	/* Calibrated sensor reading (8 registers) */
#define WHITE_LIM_REG	0x4A	/* White reading limit (8 registers) */
#define BLACK_LIM_REG	0x52	/* Black reading limit (8 registers) */
#define WHITE_CAL_REG	0x5A	/* White calibration data (8 registers) */
#define BLACK_CAL_REG	0x62	/* Black calibration data (8 registers) */
#define UNCAL_REG	0x6A	/* Uncalibrated sensor voltage (8 registers) */

#define STR_LEN	8	/* length of string registers */

#define VENDOR_ID	"mndsnsrs"
#define PRODUCT_ID	"LSArray"

enum sensors {
	SENSOR_0,
	SENSOR_1,
	SENSOR_2,
	SENSOR_3,
	SENSOR_4,
	SENSOR_5,
	SENSOR_6,
	SENSOR_7,
	NUM_SENSOR
};

struct ms_light_array_data {
	struct i2c_client *client;
	struct legoev3_port_device *in_port;
	struct measure_sensor_device ms[NUM_SENSOR];
	char fw_ver[ID_STR_LEN + 1];
};

static struct measure_sensor_scale_info ms_light_array_scale_info[] = {
	{
		.units	= "%",
		.min	= 0,
		.max	= 100,
		.dp	= 0,
	},
	END_SCALE_INFO
};

static int ms_light_array_raw_value(struct measure_sensor_device *ms)
{
	struct ms_light_array_data *la =
		container_of(ms, struct ms_light_array_data, ms[ms->id]);

	return nxt_i2c_read_word(la->client, CAL_READ_REG + ms->id);
}

static int ms_light_array_cal_value(struct measure_sensor_device *ms)
{
	struct ms_light_array_data *la =
		container_of(ms, struct ms_light_array_data, ms[ms->id]);

	return nxt_i2c_read_byte(la->client, UNCAL_REG + ms->id * 2);
}

int ms_light_array_register_measure_sensors(struct ms_light_array_data *la,
					    struct device *parent)
{
	int err;
	int i = 0;

	do {
		err = register_msensor(&la->ms[i], parent);
		if (err)
			goto err_register_measure_sensor;
	} while (++i < NUM_SENSOR);

	return 0;

err_register_measure_sensor:
	while (i--)
		unregister_msensor(&la->ms[i]);

	return err;
}

static int __devinit ms_light_array_sensor_probe(struct i2c_client *client,
						 const struct i2c_device_id *id)
{
	struct ms_light_array_data *la;
	struct i2c_legoev3_platform_data *pdata =
					client->adapter->dev.platform_data;
	int err, i;

	if (WARN_ON(!pdata))
		return -EINVAL;
	if (WARN_ON(!pdata->in_port))
		return -EINVAL;
	la = kzalloc(sizeof(struct ms_light_array_data), GFP_KERNEL);
	if (!la)
		return -ENOMEM;

	la->client = client;
	la->in_port = pdata->in_port;
	for (i = 0; i < NUM_SENSOR; i++) {
		la->ms[i].name = "reflected-light";
		la->ms[i].id = i;
		la->ms[i].raw_value = ms_light_array_raw_value;
		la->ms[i].raw_min = 0;
		la->ms[i].raw_max = 0xffff;
		la->ms[i].cal_value = ms_light_array_cal_value;
		la->ms[i].cal_min = 0;
		la->ms[i].cal_max = 100;
		la->ms[i].scale_info = ms_light_array_scale_info;
	}
	nxt_i2c_read_string(client, FIRMWARE_REG, la->fw_ver, ID_STR_LEN);

	err = ms_light_array_register_measure_sensors(la, &client->dev);
	if (err) {
		dev_err(&client->dev, "could not register measurement sensor!\n");
		goto err_register_measure_sensor;
	}

	i2c_set_clientdata(client, la);
	dev_info(&client->dev, "Mindsensors Light Sensor Array registered as '%s'\n",
		 dev_name(&client->dev));

	return 0;

err_register_measure_sensor:
	kfree(la);

	return err;
}

static int __devexit ms_light_array_sensor_remove(struct i2c_client *client)
{
	struct ms_light_array_data *la = i2c_get_clientdata(client);
	int i;

	dev_info(&client->dev, "Mindsensors Light Sensor Array '%s' removed.\n",
		 dev_name(&client->dev));
	for (i = 0; i < NUM_SENSOR; i++)
		unregister_msensor(&la->ms[i]);
	kfree(la);

	return 0;
}

static int ms_light_array_sensor_detect(struct i2c_client *client,
					struct i2c_board_info *info)
{
	int ret;

	ret = nxt_i2c_test_id_string(client, VENDOR_ID_REG, VENDOR_ID);
	if (ret)
		return -ENODEV;
	ret = nxt_i2c_test_id_string(client, DEVICE_ID_REG, PRODUCT_ID);
	if (ret)
		return -ENODEV;

	sprintf(info->type, "ms-light-array");
	return 0;
}

static struct i2c_device_id ms_light_array_sensor_idtable[] = {
	{ "ms-light-array", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, ms_light_array_sensor_idtable);

static struct i2c_driver ms_light_array_sensor_driver = {
	.driver = {
		.name	= "ms-light-array",
	},
	.id_table	= ms_light_array_sensor_idtable,
	.probe		= ms_light_array_sensor_probe,
	.remove		= __devexit_p(ms_light_array_sensor_remove),
	.class		= I2C_CLASS_LEGOEV3,
	.detect		= ms_light_array_sensor_detect,
	.address_list	= I2C_ADDRS(0x0A),
};
module_i2c_driver(ms_light_array_sensor_driver);

MODULE_DESCRIPTION("Mindsensors light array sensor device driver for LEGO Mindstorms EV3");
MODULE_AUTHOR("David Lechner <david@lechnology.com>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("legoev3:nxt-i2c-sensor");
