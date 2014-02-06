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
#include <linux/i2c.h>
#include <linux/i2c-legoev3.h>
#include <linux/workqueue.h>
#include <linux/legoev3/legoev3_ports.h>
#include <linux/legoev3/ev3_input_port.h>
#include <linux/legoev3/msensor_class.h>

#define NXT_I2C_FW_VER_REG	0x00
#define	NXT_I2C_VEND_ID_REG	0x08
#define NXT_I2C_PROD_ID_REG	0x10

#define NXT_I2C_ID_STR_LEN	8

#define NXT_I2C_POLL_MS		100
#define NXT_I2C_POLL_DELAY	msecs_to_jiffies(NXT_I2C_POLL_MS)

/**
 * struct nxt_i2c_sensor_mode_info
 * @ms_mode_info: Mode info used by the msensor device class.
 * @set_mode_reg: The register address used to set the mode.
 * @set_mode_data: The data to write to the command register.
 * @read_data_reg: The starting register address of the data to be read.
 * @pin1_state: Sets input port pin 1 high (battery voltage) when 1.
 */
struct nxt_i2c_sensor_mode_info {
	u8 set_mode_reg;
	u8 set_mode_data;
	u8 read_data_reg;
	unsigned pin1_state:1;
};

/**
 * struct nxt_i2c_sensor_info
 * @vendor_id: The vendor ID string to match to the sensor.
 * @product_id: The product ID string to match to the sensor.
 * @fw_version: The firmware version read from the sensor.
 * @ms: The msensor class device for this sensor.
 * @mode_info: Array of mode information for each sensor mode.
 * @num_modes: Number of valid elements in the mode_info array.
 */
struct nxt_i2c_sensor_info {
	const char *vendor_id;
	const char *product_id;
	char fw_version[NXT_I2C_ID_STR_LEN + 1];
	struct msensor_device ms;
	struct msensor_mode_info ms_mode_info[MSENSOR_MODE_MAX + 1];
	struct nxt_i2c_sensor_mode_info i2c_mode_info[MSENSOR_MODE_MAX + 1];
};

struct nxt_i2c_sensor_data {
	struct i2c_client *client;
	struct legoev3_port_device *in_port;
	struct nxt_i2c_sensor_info info;
	struct delayed_work poll_work;
	u8 mode;
};

/*
 * Definitions for all known sensors
 *
 * Required values:
 * - vendor_id
 * - product_id
 * - ms.type_id
 * - ms.num_modes
 * - mode_info.ms_mode_info.name
 * - num_modes
 *
 * Optional values:
 * - ms.num_view_modes (default 1)
 * - ms_mode_info.raw_min
 * - ms_mode_info.raw_max (default 255)
 * - ms_mode_info.pct_min
 * - ms_mode_info.pct_max (default 100)
 * - ms_mode_info.si_min
 * - ms_mode_info.si_max (default 255)
 * - ms_mode_info.units
 * - ms_mode_info.data_sets (default 1)
 * - ms_mode_info.format (default MSENSOR_DATA_8)
 * - ms_mode_info.figures (default 5)
 * - ms_mode_info.decimals
 * - i2c_mode_info.set_mode_reg and mode_info.set_mode_data
 * - i2c_mode_info.read_data_reg
 * - i2c_mode_info.pin1_state
 *
 * All other values will be overwritten during device initialization.
 *
 * Each sensor should have at least one mode. Mode [0] should be the default
 * mode that the sensor is in when it first starts.
 *
 * Type ids come from sys/settings/typedata.rcf in LMS2012
 * This link will probably break eventually, but for easy access, try:
 * <http://python-ev3.org/types.html>
 */
struct nxt_i2c_sensor_info nxt_i2c_sensor_defs[] = {
	{
		.vendor_id	= "LEGO",
		.product_id	= "Sonar",
		.ms.type_id	= 5,
		.ms.num_modes	= 3,
		.ms_mode_info	= {
			[0] = {
				.name	= "NXT-US-CM",
				.units	= "cm",
			},
			[1] = {
				.name	= "NXT-US-SI-CM",
				.units	= "cm",
				.data_sets = 8,
			},
			[2] = {
				.name	= "NXT-US-LIST",
				.raw_max = 1,
				.si_max  = 1,
			},
		},
		.i2c_mode_info	= {
			[0] = {
				.set_mode_reg	= 0x41,
				.set_mode_data	= 0x02,
				.read_data_reg	= 0x42,
				.pin1_state	= 1,
			},
			[1] = {
				.set_mode_reg	= 0x41,
				.set_mode_data	= 0x01,
				.read_data_reg	= 0x42,
				.pin1_state	= 1,
			},
			[2] = {
				.set_mode_reg	= 0x41,
				.set_mode_data	= 0x03,
				.read_data_reg	= 0x42,
				.pin1_state	= 1,
			},
		},
	},
	{
		.vendor_id	= "mndsnsrs",
		.product_id	= "LSArray",
		.ms.type_id	= 70,
		.ms.num_modes	= 2,
		.ms_mode_info	= {
			[0] = {
				.name	= "MS-LSA-CAL",
				.raw_max = 100,
				.si_max = 100,
				.data_sets = 8,
				.units	= "pct",
			},
			[1] = {
				.name	= "MS-LSA-RAW",
				.raw_max = 65535,
				.si_max = 65535,
				.data_sets = 8,
				.format = MSENSOR_DATA_16,
			},
		},
		.i2c_mode_info	= {
			[0] = {
				.set_mode_reg	= 0x41,
				.set_mode_data	= 'w',
				.read_data_reg	= 0x42,
			},
			[1] = {
				.set_mode_reg	= 0x41,
				.set_mode_data	= 'w',
				.read_data_reg	= 0x6A,
			},
		},
	},
};

static u8 nxt_i2c_sensor_get_mode(void *context)
{
	struct nxt_i2c_sensor_data *sensor = context;

	return sensor->mode;
}

static int nxt_i2c_sensor_set_mode(void *context, u8 mode)
{
	struct nxt_i2c_sensor_data *sensor = context;
	int err;

	err = i2c_smbus_write_byte_data(sensor->client,
		sensor->info.i2c_mode_info[mode].set_mode_reg,
		sensor->info.i2c_mode_info[mode].set_mode_data);
	if (err < 0)
		return err;

	ev3_input_port_set_pin1_out(sensor->in_port,
	                            sensor->info.i2c_mode_info[mode].pin1_state);
	sensor->mode = mode;

	return 0;
}

static ssize_t nxt_i2c_sensor_write_data(void *context, char *data, loff_t off,
                                         size_t count)
{
	struct nxt_i2c_sensor_data *sensor = context;
	int err;

	err = i2c_smbus_write_i2c_block_data(sensor->client, off, count, data);
	if (err < 0)
		return err;

	return count;
}

void nxt_i2c_sensor_poll_work(struct work_struct *work)
{
	struct delayed_work *dwork =
		container_of(work, struct delayed_work, work);
	struct nxt_i2c_sensor_data *sensor =
		container_of(dwork, struct nxt_i2c_sensor_data, poll_work);
	struct nxt_i2c_sensor_mode_info *i2c_mode_info =
		&sensor->info.i2c_mode_info[sensor->mode];
	struct msensor_mode_info *ms_mode_info =
			&sensor->info.ms_mode_info[sensor->mode];

	i2c_smbus_read_i2c_block_data(sensor->client, i2c_mode_info->read_data_reg,
	                              ms_mode_info->data_sets
	                              * legoev3_msensor_data_size[ms_mode_info->format],
	                              ms_mode_info->raw_data);

	schedule_delayed_work(&sensor->poll_work, NXT_I2C_POLL_DELAY);
}

static int __devinit nxt_i2c_sensor_probe(struct i2c_client *client,
				  const struct i2c_device_id *id)
{
	struct nxt_i2c_sensor_data *sensor;
	struct i2c_legoev3_platform_data *apdata =
					client->adapter->dev.platform_data;
	struct nxt_i2c_sensor_info *cpdata = client->dev.platform_data;
	int err, i;

	if (WARN(!apdata, "Adapter platform data is required."))
		return -EINVAL;
	if (WARN(!apdata->in_port, "Adapter platform data missing input port."))
		return -EINVAL;
	if (WARN(!cpdata, "Client platform data is required."))
		return -EINVAL;
	sensor = kzalloc(sizeof(struct nxt_i2c_sensor_data), GFP_KERNEL);
	if (!sensor)
		return -ENOMEM;

	sensor->client = client;
	sensor->in_port = apdata->in_port;
	memcpy(&sensor->info, cpdata, sizeof(struct nxt_i2c_sensor_info));

	i2c_smbus_read_i2c_block_data(client, NXT_I2C_FW_VER_REG,
	                              NXT_I2C_ID_STR_LEN,
	                              sensor->info.fw_version);
	if (!sensor->info.ms.num_view_modes)
		sensor->info.ms.num_view_modes = 1;
	sensor->info.ms.mode_info = sensor->info.ms_mode_info;
	sensor->info.ms.get_mode = nxt_i2c_sensor_get_mode;
	sensor->info.ms.set_mode = nxt_i2c_sensor_set_mode;
	sensor->info.ms.write_data = nxt_i2c_sensor_write_data;
	sensor->info.ms.context = sensor;

	for (i = 0; i < sensor->info.ms.num_modes; i++) {
		struct msensor_mode_info *minfo = &sensor->info.ms_mode_info[i];

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

	err = register_msensor(&sensor->info.ms, &client->dev);
	if (err) {
		dev_err(&client->dev, "could not register sensor!\n");
		goto err_register_msensor;
	}

	if (sensor->info.i2c_mode_info[0].pin1_state)
		ev3_input_port_set_pin1_out(sensor->in_port, 1);

	INIT_DELAYED_WORK(&sensor->poll_work, nxt_i2c_sensor_poll_work);
	i2c_set_clientdata(client, sensor);
	dev_info(&client->dev, "NXT I2C sensor registered as '%s'\n",
		 dev_name(&client->dev));

	schedule_delayed_work(&sensor->poll_work, NXT_I2C_POLL_DELAY);

	return 0;

err_register_msensor:
	kfree(sensor);

	return err;
}

static int __devexit nxt_i2c_sensor_remove(struct i2c_client *client)
{
	struct nxt_i2c_sensor_data *sensor = i2c_get_clientdata(client);

	dev_info(&client->dev, "NXT I2C sensor '%s' removed.\n",
		 dev_name(&client->dev));
	cancel_delayed_work_sync(&sensor->poll_work);
	ev3_input_port_set_pin1_out(sensor->in_port, 0);
	unregister_msensor(&sensor->info.ms);
	kfree(sensor);

	return 0;
}

static int nxt_i2c_sensor_detect(struct i2c_client *client,
                                 struct i2c_board_info *info)
{
	char vendor_id[NXT_I2C_ID_STR_LEN + 1] = { 0 };
	char product_id[NXT_I2C_ID_STR_LEN + 1] = { 0 };
	int ret, i;

	ret = i2c_smbus_read_i2c_block_data(client, NXT_I2C_VEND_ID_REG,
					    NXT_I2C_ID_STR_LEN, vendor_id);
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

	for (i = 0; i < ARRAY_SIZE(nxt_i2c_sensor_defs); i++)
	{
		if (!strcmp(nxt_i2c_sensor_defs[i].vendor_id, vendor_id)
			&& !strcmp(nxt_i2c_sensor_defs[i].product_id, product_id))
		{
			sprintf(info->type, "nxt-i2c-sensor");
			info->platform_data = &nxt_i2c_sensor_defs[i];
			return 0;
		}
	}
	return -ENODEV;
}

static struct i2c_device_id nxt_i2c_sensor_idtable[] = {
	{ "nxt-i2c-sensor", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, nxt_i2c_sensor_idtable);

static struct i2c_driver nxt_i2c_sensor_driver = {
	.driver = {
		.name	= "nxt-i2c-sensor",
	},
	.id_table	= nxt_i2c_sensor_idtable,
	.probe		= nxt_i2c_sensor_probe,
	.remove		= __devexit_p(nxt_i2c_sensor_remove),
	.class		= I2C_CLASS_LEGOEV3,
	.detect		= nxt_i2c_sensor_detect,
	.address_list	= I2C_ADDRS(0x01, 0x02, 0x08, 0x0A),
};
module_i2c_driver(nxt_i2c_sensor_driver);

MODULE_DESCRIPTION("NXT I2C sensor device driver for LEGO Mindstorms EV3");
MODULE_AUTHOR("David Lechner <david@lechnology.com>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("legoev3:nxt-i2c-host");
