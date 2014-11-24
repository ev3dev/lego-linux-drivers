/*
 * NXT I2C sensor device driver for LEGO MINDSTORMS EV3
 *
 * Copyright (C) 2013-2014 David Lechner <david@lechnology.com>
 * Copyright (C) 2014 Bartosz Meglicki <meglickib@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.

 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/i2c.h>
#include <linux/legoev3/legoev3_ports.h>
#include <linux/legoev3/servo_motor_class.h>

#include "nxt_i2c_sensor.h"
#include "ht_smux.h"

/* HiTechnic NXT Sensor Multiplexer implementation */

static const struct device_type ht_smux_input_port_device_type = {
	.name	= "ht-smux-input-port",
};

struct ht_smux_input_port_data {
	struct legoev3_port *port;
	legoev3_analog_cb_func_t cb;
};

static int ht_sensor_mux_set_mode_pre_cb(struct nxt_i2c_sensor_data * sensor,
					 u8 mode)
{
	int err;

	err = i2c_smbus_read_byte_data(sensor->client, HT_SMUX_STATUS_REG);
	if (err < 0)
		return err;

	/* can't switch to detect mode from run mode */
	if (mode == HT_SMUX_COMMAND_DETECT && !(err & HT_SMUX_STATUS_HALT))
		return -EPERM;

	/* can't change modes while detect is in progress */
	err = i2c_smbus_read_byte_data(sensor->client, HT_SMUX_COMMAND_REG);
	if (err < 0)
		return err;
	if (err == HT_SMUX_COMMAND_DETECT)
		return -EBUSY;

	return 0;
}

static void ht_sensor_mux_set_mode_post_cb(struct nxt_i2c_sensor_data *data,
					   u8 mode)
{
	struct ht_smux_input_port_data *ports = data->info.callback_data;
	struct ht_smux_input_port_platform_data pdata;
	char name[LEGOEV3_PORT_NAME_SIZE];
	int i;

	if (mode == 0 /* run mode */ && !ports) {
		ports = kzalloc(sizeof(struct ht_smux_input_port_data)
						* NUM_HT_SMUX_CH, GFP_KERNEL);
		for (i = 0; i < NUM_HT_SMUX_CH; i++) {
			pdata.client = data->client;
			pdata.channel = i;
			pdata.sensor_data = data->ms.mode_info[0].raw_data;
			sprintf(name, "%s:mux", dev_name(&data->in_port->dev));

			data->info.callback_data = ports;
			ports[i].port = legoev3_port_register(name, i + 1,
				&ht_smux_input_port_device_type, &data->client->dev,
				&pdata, sizeof(struct ht_smux_input_port_platform_data));

			if (IS_ERR(ports[i].port)) {
				dev_err(&data->client->dev,
					"Failed to register HiTechnic Sensor Multiplexer input port. %ld\n",
					PTR_ERR(ports[i].port));
				for (i--; i >= 0; i--)
					legoev3_port_unregister(ports[i].port);
				kfree(ports);
				return;
			}
		}
	} else if (ports) {
		for (i = 0; i < NUM_HT_SMUX_CH; i++)
			legoev3_port_unregister(ports[i].port);
		data->info.callback_data = NULL;
		kfree(ports);
	}
}

static void ht_sensor_mux_poll_cb(struct nxt_i2c_sensor_data *data)
{
	struct ht_smux_input_port_data *ports = data->info.callback_data;
	int mode = data->ms.get_mode(data->ms.context);
	u8 *raw_data = data->ms.mode_info[mode].raw_data;
	int i;

	/* i2c can only transfer up to 32 bytes at a time */
	i2c_smbus_read_i2c_block_data(data->client, HT_SMUX_COMMAND_REG,
		32, raw_data);
	/* only read ch1 and ch2 i2c data if an i2c sensor is connected */
	if ((raw_data[HT_SMUX_CH1_CONFIG_REG - HT_SMUX_COMMAND_REG]
		& HT_SMUX_CONFIG_I2C) || (raw_data[HT_SMUX_CH2_CONFIG_REG
		- HT_SMUX_COMMAND_REG] & HT_SMUX_CONFIG_I2C))
	{
		i2c_smbus_read_i2c_block_data(data->client,
			HT_SMUX_CH1_I2C_DATA_REG, 32, raw_data + 32);
	}
	/* only read ch3 and ch4 i2c data if an i2c sensor is connected */
	if ((raw_data[HT_SMUX_CH3_CONFIG_REG - HT_SMUX_COMMAND_REG]
		& HT_SMUX_CONFIG_I2C) || (raw_data[HT_SMUX_CH4_CONFIG_REG
		- HT_SMUX_COMMAND_REG] & HT_SMUX_CONFIG_I2C))
	{
		i2c_smbus_read_i2c_block_data(data->client,
			HT_SMUX_CH3_I2C_DATA_REG, 32, raw_data + 64);
	}
	if (ports) {
		for (i = 0; i < NUM_HT_SMUX_CH; i++) {
			if (ports[i].cb && ports[i].port)
				ports[i].cb(ports[i].port);
		}
	}
}

static void ht_sensor_mux_remove_cb(struct nxt_i2c_sensor_data *data)
{
	struct ht_smux_input_port_data *ports = data->info.callback_data;
	int i;

	if (ports) {
		for (i = 0; i < NUM_HT_SMUX_CH; i++)
			legoev3_port_unregister(ports[i].port);
		data->info.callback_data = NULL;
		kfree(ports);
	}
}

void ht_sensor_mux_register_poll_cb(struct i2c_client *client,
				    enum ht_smux_channel channel,
				    legoev3_analog_cb_func_t cb)
{
	struct nxt_i2c_sensor_data *data = i2c_get_clientdata(client);
	struct ht_smux_input_port_data *ports = data->info.callback_data;

	ports[channel].cb = cb;
}
EXPORT_SYMBOL_GPL(ht_sensor_mux_register_poll_cb);

/* mindsensors.com 8-channel servo motor controller implementation */

struct ms_8ch_servo_data {
	int id;
	struct nxt_i2c_sensor_data *sensor;
	struct servo_motor_device servo;
};

static int ms_8ch_servo_get_position(void* context)
{
	struct ms_8ch_servo_data *servo = context;
	struct i2c_client *client = servo->sensor->client;

	return i2c_smbus_read_word_data(client, 0x42 + servo->id * 2);
}

static int ms_8ch_servo_set_position(void* context, int value)
{
	struct ms_8ch_servo_data *servo = context;
	struct i2c_client *client = servo->sensor->client;

	return i2c_smbus_write_word_data(client, 0x42 + servo->id * 2, value);
}

static int ms_8ch_servo_get_rate(void* context)
{
	struct ms_8ch_servo_data *servo = context;
	struct i2c_client *client = servo->sensor->client;
	int ret;

	ret = i2c_smbus_read_word_data(client, 0x52 + servo->id);
	if (ret < 0)
		return ret;

	if (ret == 0)
		return 0;
	return 24000 / ret;
}

static int ms_8ch_servo_set_rate(void* context, unsigned value)
{
	struct ms_8ch_servo_data *servo = context;
	struct i2c_client *client = servo->sensor->client;
	int scaled;

	if (value >= 24000)
		scaled = 1;
	else if (value < 94)
		scaled = 0;
	else
		scaled = 24000 / value;

	return i2c_smbus_write_word_data(client, 0x52 + servo->id * 2, scaled);
}

static void ms_8ch_servo_probe_cb(struct nxt_i2c_sensor_data *data)
{
	struct ms_8ch_servo_data *servos;
	int i, err;

	servos = kzalloc(sizeof(struct ms_8ch_servo_data) * 8, GFP_KERNEL);
	if (IS_ERR(servos)) {
		dev_err(&data->client->dev, "Error allocating servos. %ld",
			PTR_ERR(servos));
		return;
	}
	for (i = 0; i < 8; i++) {
		servos[i].id = i;
		servos[i].sensor = data;
		strncpy(servos[i].servo.name, data->ms.name, SERVO_MOTOR_NAME_SIZE);
		snprintf(servos[i].servo.port_name, SERVO_MOTOR_NAME_SIZE,
			 "%s:sv%d", data->ms.port_name, i + 1);
		servos[i].servo.ops.get_position = ms_8ch_servo_get_position;
		servos[i].servo.ops.set_position = ms_8ch_servo_set_position;
		servos[i].servo.ops.get_rate = ms_8ch_servo_get_rate;
		servos[i].servo.ops.set_rate = ms_8ch_servo_set_rate;
		servos[i].servo.context = &servos[i];
		err = register_servo_motor(&servos[i].servo, &data->client->dev);
		if (err)
			break;
	}
	if (err < 0) {
		for (i--; i >= 0; i--)
			unregister_servo_motor(&servos[i].servo);
		kfree(servos);
		dev_err(&data->client->dev, "Error registering servos. %d", err);
		return;
	}
	data->info.callback_data = servos;
	data->poll_ms = 1000;
}

static void ms_8ch_servo_remove_cb(struct nxt_i2c_sensor_data *data)
{
	struct ms_8ch_servo_data *servos = data->info.callback_data;
	int i;

	if (servos) {
		for (i = 0; i < 8; i++)
			unregister_servo_motor(&servos[i].servo);
		kfree(servos);
	}
}

/*
 * Lookup table for rad2deg(asin(x / 128)). Used to convert raw value to degrees.
 */
static const u8 ms_imu_tilt2deg[] = {
	0, 0, 1, 1, 2, 2, 3, 3, 4, 4, 4, 5, 5, 6, 6, 7, 7, 8, 8, 9, 9, 9, 10,
	10, 11, 11, 12, 12, 13, 13, 14, 14, 14, 15, 15, 16, 16, 17, 17, 18, 18,
	19, 19, 20, 20, 21, 21, 22, 22, 23, 23, 23, 24, 24, 25, 25, 26, 26, 27,
	27, 28, 28, 29, 29, 30, 31, 31, 32, 32, 33, 33, 34, 34, 35, 35, 36, 36,
	37, 38, 38, 39, 39, 40, 40, 41, 42, 42, 43, 43, 44, 45, 45, 46, 47, 47,
	48, 49, 49, 50, 51, 51, 52, 53, 54, 54, 55, 56, 57, 58, 58, 59, 60, 61,
	62, 63, 64, 65, 66, 67, 68, 70, 71, 72, 74, 76, 78, 80, 83, 90, 97, 100,
	102, 104, 106, 108, 109, 110, 112, 113, 114, 115, 116, 117, 118, 119,
	120, 121, 122, 122, 123, 124, 125, 126, 126, 127, 128, 129, 129, 130,
	131, 131, 132, 133, 133, 134, 135, 135, 136, 137, 137, 138, 138, 139,
	140, 140, 141, 141, 142, 142, 143, 144, 144, 145, 145, 146, 146, 147,
	147, 148, 148, 149, 149, 150, 151, 151, 152, 152, 153, 153, 154, 154,
	155, 155, 156, 156, 157, 157, 157, 158, 158, 159, 159, 160, 160, 161,
	161, 162, 162, 163, 163, 164, 164, 165, 165, 166, 166, 166, 167, 167,
	168, 168, 169, 169, 170, 170, 171, 171, 171, 172, 172, 173, 173, 174,
	174, 175, 175, 176, 176, 176, 177, 177, 178, 178, 179, 179, 180
};

static void ms_imu_poll_cb(struct nxt_i2c_sensor_data *sensor)
{
	struct nxt_i2c_sensor_mode_info *i2c_mode_info =
		&sensor->info.i2c_mode_info[sensor->mode];
	struct msensor_mode_info *ms_mode_info =
			&sensor->info.ms_mode_info[sensor->mode];

	/*
	 * Perform normal i2c read (just like nxt_i2c_sensor_poll_work).
	 */
	i2c_smbus_read_i2c_block_data(sensor->client,
		i2c_mode_info->read_data_reg, ms_mode_info->data_sets
		* msensor_data_size[ms_mode_info->data_type],
		ms_mode_info->raw_data);

	/* scale values for tilt mode */
	if (sensor->mode == 0) {
		ms_mode_info->raw_data[0] = ms_imu_tilt2deg[ms_mode_info->raw_data[0]];
		ms_mode_info->raw_data[1] = ms_imu_tilt2deg[ms_mode_info->raw_data[1]];
		ms_mode_info->raw_data[2] = ms_imu_tilt2deg[ms_mode_info->raw_data[2]];
	}
}

/*
 * 	Microinfinity CruizCore XG1300L gyroscope and accelerometer related functions
 */
static void mi_cruizcore_xg1300l_poll_cb(struct nxt_i2c_sensor_data *sensor)
{
	u8 *scaling_factor = sensor->info.callback_data;
	
	struct nxt_i2c_sensor_mode_info *i2c_mode_info =
		&sensor->info.i2c_mode_info[sensor->mode];
	struct msensor_mode_info *ms_mode_info =
			&sensor->info.ms_mode_info[sensor->mode];

	s16 *raw_as_s16 = (s16*) ms_mode_info->raw_data;
	
	/*
	 * Perform normal i2c read (just like nxt_i2c_sensor_poll_work).
	 */
	i2c_smbus_read_i2c_block_data(sensor->client,
		i2c_mode_info->read_data_reg, ms_mode_info->data_sets
		* msensor_data_size[ms_mode_info->data_type],
		ms_mode_info->raw_data);

	/* scale values for acceleration */

	if(sensor->mode < 2)  /* "ANG-ACC", "ANG-SPEED" - no acceleration info */
		return;
	
	if(sensor->mode == 3) /* "ALL", accelerometer data starting from fourth byte */
		raw_as_s16 += 2;
	
	raw_as_s16[0] *= *scaling_factor;
	raw_as_s16[1] *= *scaling_factor;
	raw_as_s16[2] *= *scaling_factor;
}

static void mi_cruizcore_xg1300l_send_command_post_cb(struct nxt_i2c_sensor_data *data, u8 command)
{
	u8 *scaling_factor = data->info.callback_data;

	if (command==0 || command==1) *scaling_factor=1;	/* "RESET", "ACCEL-2G"	*/
	else if (command==2) *scaling_factor=2;  			/* "ACCEL-4G" 			*/
	else if (command==3) *scaling_factor=4; 			/* "ACCEL-8G"				*/
}

static void mi_cruizcore_xg1300l_probe_cb(struct nxt_i2c_sensor_data *data)
{		
	u8 *scaling_factor = kzalloc(sizeof(u8), GFP_KERNEL);
	*scaling_factor = 1;
	data->info.callback_data = scaling_factor;
}

static void mi_cruizcore_xg1300l_remove_cb(struct nxt_i2c_sensor_data *data)
{
	u8 *scaling_factor = data->info.callback_data;

	if(scaling_factor)
	{
		data->info.callback_data = NULL;
		kfree(scaling_factor);
	}
}

/**
 * nxt_i2c_sensor_defs - Sensor definitions
 *
 * Required values:
 * - name
 * - vendor_id
 * - product_id
 * - num_modes
 * - mode_info.ms_mode_info.name
 * - i2c_mode_info.read_data_reg
 *
 * Optional values:
 * - num_read_only_modes (default num_modes)
 * - ops.set_mode_pre_cb
 * - ops.set_mode_post_cb
 * - ops.send_command_post_cb
 * - ops.poll_cb
 * - ops.probe_cb
 * - ops.remove_cb
 * - ms_mode_info.raw_min
 * - ms_mode_info.raw_max (default 255)
 * - ms_mode_info.pct_min
 * - ms_mode_info.pct_max (default 100)
 * - ms_mode_info.si_min
 * - ms_mode_info.si_max (default 255)
 * - ms_mode_info.units
 * - ms_mode_info.data_sets (default 1)
 * - ms_mode_info.data_type (default MSENSOR_DATA_U8)
 * - ms_mode_info.figures (default 5)
 * - ms_mode_info.decimals
 * - i2c_mode_info.set_mode_reg and mode_info.set_mode_data
 * - i2c_mode_info.pin1_state
 *
 * All other values will be overwritten during device initialization.
 *
 * Each sensor should have at least one mode. Mode [0] will be the default mode.
 *
 * When adding sensors, also add a driver name with the proper type id to
 * nxt_i2c_sensor_id_table in nxt_i2c_sensor_core.c so that the sensor can
 * be manually initialized.
 *
 * Documentation is automatically generated from this struct, so formatting is
 * very important. Make sure any new sensors have the same layout. The comments
 * are also parsed to provide more information for the documentation. The
 * parser can be found in the ev3dev-kpkg repository.
 */
const struct nxt_i2c_sensor_info nxt_i2c_sensor_defs[] = {
	[UNKNOWN_I2C_SENSOR] = {
		/**
		 * @vendor_part_name: Unknown NXT I2C Sensor
		 */
		.name		= "nxt-i2c",
		.vendor_id	= "UNKNOWN",
		.product_id	= "unknown",
		.num_modes	= 3,
		.ms_mode_info	= {
			[0] = {
				/**
				 * @description: Unsigned 8-bit data
				 */
				.name = "I2C-U8",
			},
			[1] = {
				/**
				 * @description: Signed 8-bit data
				 */
				.name = "I2C-S8",
				.data_type = MSENSOR_DATA_S8,
			},
			[2] = {
				/**
				 * @description: Signed 16-bit data
				 */
				.name = "I2C-S16",
				.data_type = MSENSOR_DATA_S16,
			},
		},
		.i2c_mode_info	= {
			[0] = {
				.read_data_reg = 0x42,
			},
			[1] = {
				.read_data_reg = 0x42,
			},
			[2] = {
				.read_data_reg = 0x42,
			},
		},
	},
	[LEGO_NXT_ULTRASONIC_SENSOR] = {
		/**
		 * @vendor_name: LEGO
		 * @vendor_part_number: 9846
		 * @vendor_part_name: NXT Ultrasonic Sensor
		 * @vendor_website: http://www.lego.com/en-us/mindstorms/downloads/software/nxt-hdk/
		 * @default_address: 0x01
		 */
		.name		= "lego-nxt-ultrasonic",
		.vendor_id	= "LEGO",
		.product_id	= "Sonar",
		.num_modes	= 5,
		.num_read_only_modes = 2,
		.slow		= true,
		.ms_mode_info	= {
			[0] = {
				/**
				 * @description: Continuous measurement
				 * @value0: Distance (0 to 255)
				 * @units_description: centimeters
				 */
				.name	= "NXT-US-CM",
				.units	= "cm",
			},
			[1] = {
				/**
				 * @description: Continuous measurement
				 * @value0: Distance (0 to 1000)
				 * @units_description: inches
				 */
				.name	= "NXT-US-IN",
				.units	= "in",
				.si_max = 1000,
				.decimals = 1,
			},
			[2] = {
				/**
				 * [^single-measurement]: The value is read when the mode is set
				 * and does not change - even when polling is enabled. To read a
				 * new value, set the mode again (e.g. `echo NXT-US-SI-CM > mode`).
				 *
				 * @description: Single measurement
				 * @value0: Distance (0 to 255)
				 * @value0_footnote: [^single-measurement]
				 * @units_description: centimeters
				 */
				.name	= "NXT-US-SI-CM",
				.units	= "cm",
			},
			[3] = {
				/**
				 * @description: Single measurement
				 * @value0: Distance (0 to 1000)
				 * @value0_footnote: [^single-measurement]
				 * @units_description: inches
				 */
				.name	= "NXT-US-SI-IN",
				.units	= "in",
				.si_max = 1000,
				.decimals = 1,
			},
			[4] = {
				/**
				 * [^listen-value]: A value of `1` indicates that another ultrasonic
				 * sensor has been detected. A `1` can also be triggered by a loud
				 * noise such as clapping.
				 *
				 * @description: Listen
				 * @value0: Presence (0 or 1)
				 * @value0_footnote: [^listen-value]
				 */
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
				.pin1_state	= EV3_INPUT_PORT_GPIO_HIGH,
			},
			[1] = {
				.set_mode_reg	= 0x41,
				.set_mode_data	= 0x02,
				.read_data_reg	= 0x42,
				.pin1_state	= EV3_INPUT_PORT_GPIO_HIGH,
			},
			[2] = {
				.set_mode_reg	= 0x41,
				.set_mode_data	= 0x01,
				.read_data_reg	= 0x42,
				.pin1_state	= EV3_INPUT_PORT_GPIO_HIGH,
			},
			[3] = {
				.set_mode_reg	= 0x41,
				.set_mode_data	= 0x01,
				.read_data_reg	= 0x42,
				.pin1_state	= EV3_INPUT_PORT_GPIO_HIGH,
			},
			[4] = {
				.set_mode_reg	= 0x41,
				.set_mode_data	= 0x03,
				.read_data_reg	= 0x42,
				.pin1_state	= EV3_INPUT_PORT_GPIO_HIGH,
			},
		},
	},
	[LEGO_POWER_STORAGE_SENSOR] = {
		/**
		 * @vendor_name: LEGO
		 * @vendor_part_number: 9668
		 * @vendor_part_name: Energy Display
		 * @vendor_website: http://education.lego.com/en-us/lego-education-product-database/machines-and-mechanisms/9668-energy-display
		 * @default_address: 0x02
		 */
		.name		= "lego-power-storage",
		.vendor_id	= "LEGO",
		.product_id	= "", /* LMS2012 fakes this with "Store." */
		.num_modes	= 8,
		.ms_mode_info	= {
			[0] = {
				/**
				 * @description: Input Voltage
				 * @value0: Voltage (0 to 10000)
				 * @units_description: volts
				 */
				.name = "ES-IN-VOLT",
				.units = "V",
				.raw_max = 10000,
				.si_max = 10000,
				.decimals = 3,
				.data_type = MSENSOR_DATA_S16_BE,
			},
			[1] = {
				/**
				 * @description: Input Current
				 * @value0: Current (0 to 10000)
				 * @units_description: amps
				 */
				.name = "ES-IN-AMP",
				.units = "A",
				.raw_max = 10000,
				.si_max = 10000,
				.decimals = 3,
				.data_type = MSENSOR_DATA_S16_BE,
			},
			[2] = {
				/**
				 * @description: Output Voltage
				 * @value0: Voltage (0 to 10000)
				 * @units_description: volts
				 */
				.name = "ES-OUT-VOLT",
				.units = "V",
				.raw_max = 10000,
				.si_max = 10000,
				.decimals = 3,
				.data_type = MSENSOR_DATA_S16_BE,
			},
			[3] = {
				/**
				 * @description: Output Current
				 * @value0: Current (0 to 10000)
				 * @units_description: amps
				 */
				.name = "ES-OUT-AMP",
				.units = "A",
				.raw_max = 10000,
				.si_max = 10000,
				.decimals = 3,
				.data_type = MSENSOR_DATA_S16_BE,
			},
			[4] = {
				/**
				 * @description: Energy
				 * @value0: Energy (0 to 100)
				 * @units_description: Joules
				 */
				.name = "ES-JOULE",
				.units = "J",
				.raw_max = 100,
				.si_max = 100,
				.data_type = MSENSOR_DATA_S16_BE,
			},
			[5] = {
				/**
				 * @description: Input Power
				 * @value0: Power (0 to 10000)
				 * @units_description: Watts
				 */
				.name = "ES-IN-WATT",
				.units = "W",
				.raw_max = 10000,
				.si_max = 10000,
				.decimals = 3,
				.data_type = MSENSOR_DATA_S16_BE,
			},
			[6] = {
				/**
				 * @description: Output Power
				 * @value0: Power (0 to 10000)
				 * @units_description: Watts
				 */
				.name = "ES-OUT-WATT",
				.units = "W",
				.raw_max = 10000,
				.si_max = 10000,
				.decimals = 3,
				.data_type = MSENSOR_DATA_S16_BE,
			},
			[7] = {
				/**
				 * @description: All
				 * @value0: Input Voltage (0 to 10000)
				 * @value1: Input Current (0 to 10000)
				 * @value2: Output Voltage (0 to 10000)
				 * @value3: Output Current (0 to 10000)
				 * @value4: Energy (0 to 100)
				 * @value5: Input Power (0 to 10000)
				 * @value6: Output Power (0 to 10000)
				 */
				.name = "ES-ALL",
				.raw_max = 10000,
				.si_max = 10000,
				.decimals = 3,
				.data_sets = 7,
				.data_type = MSENSOR_DATA_S16_BE,
			},
		},
		.i2c_mode_info	= {
			[0] = {
				.read_data_reg	= 0x0A,
			},
			[1] = {
				.read_data_reg	= 0x0C,
			},
			[2] = {
				.read_data_reg	= 0x0E,
			},
			[3] = {
				.read_data_reg	= 0x10,
			},
			[4] = {
				.read_data_reg	= 0x12,
			},
			[5] = {
				.read_data_reg	= 0x14,
			},
			[6] = {
				.read_data_reg	= 0x16,
			},
			[7] = {
				.read_data_reg	= 0x0A,
			},
		},
	},
	[HT_NXT_PIR_SENSOR] = {
		/**
		 * @vendor_name: HiTechnic
		 * @vendor_part_number: NIS1070
		 * @vendor_part_name: NXT PIR Sensor
		 * @vendor_website: http://www.hitechnic.com/cgi-bin/commerce.cgi?preadd=action&key=NIS1070
		 * @default_address: 0x01
		 */
		.name		= "ht-nxt-pir",
		.vendor_id	= "HITECHNC",
		.product_id	= "PIR",
		.num_modes	= 1,
		.ms_mode_info	= {
			[0] = {
				/**
				 * @description: IR Proximity
				 * @value0: Proximity (-100 to 100)
				 * @units_description: percent
				 */
				.name = "HT-PIR",
				.pct_min = -100,
				.si_min = -100,
				.si_max = 100,
				.units = "pct",
			},
		},
		.i2c_mode_info	= {
			[0] = {
				.read_data_reg	= 0x42,
			},
		},
	},
	[HT_NXT_BAROMETRIC_SENSOR] = {
		/**
		 * @vendor_name: HiTechnic
		 * @vendor_part_number: NBR1036
		 * @vendor_part_name: NXT Barometric Sensor
		 * @vendor_website: http://www.hitechnic.com/cgi-bin/commerce.cgi?preadd=action&key=NBR1036
		 * @default_address: 0x01
		 */
		.name		= "ht-nxt-barometric",
		.vendor_id	= "HiTechnc",
		.product_id	= "Barometr",
		.num_modes	= 2,
		.ms_mode_info	= {
			[0] = {
				/**
				 * @description: Barometric Pressure
				 * @value0: Absolute Pressure (0 to 3000)
				 * @units_description: ???
				 */
				.name = "HT-BAR-PRES",
				.raw_min = 30400,
				.raw_max = 29400,
				.si_max = 3000,
				.decimals = 1,
				.units = "m",
			},
			[1] = {
				/**
				 * @description: Temperature
				 * @value0: Absolute Pressure (0 to 1000)
				 * @units_description: degrees Celsius
				 */
				.name = "HT-BAR-TEMP",
				.raw_max = 1000,
				.si_max = 1000,
				.decimals = 1,
				.units = "C",
			},
		},
		.i2c_mode_info	= {
			[0] = {
				.read_data_reg	= 0x42,
			},
			[1] = {
				.read_data_reg	= 0x42,
			},
		},
	},
	[HT_NXT_IR_SEEKER_SENSOR_V2] = {
		/**
		 * @vendor_name: HiTechnic
		 * @vendor_part_number: NSK1042
		 * @vendor_part_name: NXT IRSeeker V2
		 * @vendor_website: http://www.hitechnic.com/cgi-bin/commerce.cgi?preadd=action&key=NSK1042
		 * @default_address: 0x08
		 */
		.name		= "ht-nxt-ir-seeker-v2",
		.vendor_id	= "HiTechnc",
		.product_id	= "NewIRDir",
		.num_modes	= 4,
		.ms_mode_info	= {
			[0] = {
				/**
				 * [^values]: Direction values:
				 *
				 * | Value | Description |
				 * |-------|-------------|
				 * | 0     | No signal   |
				 * | 1     | Far left    |
				 * | ...   |             |
				 * | 5     | Center      |
				 * | ...   |             |
				 * | 9     | Far right   |
				 *
				 * @description: Direction (unmodulated)
				 * @value0: Direction (0 to 9)
				 * @value0_footnote: [^values]
				 */
				.name = "HT-DIR-DC",
				.raw_max = 9,
				.si_max = 9,
			},
			[1] = {
				/**
				 * @description: Direction (modulated)
				 * @value0: Direction (0 to 9)
				 * @value0_footnote: [^values]
				 */
				.name = "HT-DIR-AC",
				.raw_max = 9,
				.si_max = 9,
			},
			[2] = {
				/**
				 * @description: All values (unmodulated)
				 * @value0: Direction (0 to 9)
				 * @value0_footnote: [^values]
				 * @value1: Sensor 1 signal strength (0 to 9)
				 * @value1_footnote: [^values]
				 * @value2: Sensor 2 signal strength (0 to 9)
				 * @value2_footnote: [^values]
				 * @value3: Sensor 3 signal strength (0 to 9)
				 * @value3_footnote: [^values]
				 * @value4: Sensor 4 signal strength (0 to 9)
				 * @value4_footnote: [^values]
				 * @value5: Sensor 5 signal strength (0 to 9)
				 * @value5_footnote: [^values]
				 * @value6: Sensor mean (0 to 9)
				 * @value6_footnote: [^values]
				 */
				.name = "HT-DIR-DALL",
				.data_sets = 7,
			},
			[3] = {
				/**
				 * @description: All values (modulated)
				 * @value0: Direction (0 to 9)
				 * @value0_footnote: [^values]
				 * @value1: Sensor 1 signal strength (0 to 9)
				 * @value1_footnote: [^values]
				 * @value2: Sensor 2 signal strength (0 to 9)
				 * @value2_footnote: [^values]
				 * @value3: Sensor 3 signal strength (0 to 9)
				 * @value3_footnote: [^values]
				 * @value4: Sensor 4 signal strength (0 to 9)
				 * @value4_footnote: [^values]
				 * @value5: Sensor 5 signal strength (0 to 9)
				 * @value5_footnote: [^values]
				 */
				.name = "HT-DIR-AALL",
				.data_sets = 6,
			},
		},
		.i2c_mode_info	= {
			[0] = {
				.read_data_reg	= 0x42,
			},
			[1] = {
				.read_data_reg	= 0x49,
			},
			[2] = {
				.read_data_reg	= 0x42,
			},
			[3] = {
				.read_data_reg	= 0x49,
			},
		},
	},
	[HT_NXT_COLOR_SENSOR] = {
		/**
		 * @vendor_name: HiTechnic
		 * @vendor_part_name: NXT Color Sensor
		 * @default_address: 0x01
		 */
		.name		= "ht-nxt-color",
		.vendor_id	= "HiTechnc",
		.product_id	= "Color",
		.num_modes	= 7,
		.ms_mode_info	= {
			[0] = {
				/**
				 * [^color-value]: Color Values:<br />
				 * ![Color chart](http://www.hitechnic.com/contents/media/Color%20Number.jpg)
				 *
				 * @description: Color
				 * @value0: Color (0 to 17)
				 * @value0_footnote: [^color-value]
				 */
				.name	= "HT-COL1-COL",
				.raw_max = 17,
				.si_max = 17,
			},
			[1] = {
				/**
				 * @description: Red component
				 * @value0: Reflected light intensity (0 to 255)
				 */
				.name = "HT-COL1-RED",
			},
			[2] = {
				/**
				 * @description: Green component
				 * @value0: Reflected light intensity (0 to 255)
				 */
				.name = "HT-COL1-GRN",
			},
			[3] = {
				/**
				 * @description: Blue component
				 * @value0: Reflected light intensity (0 to 255)
				 */
				.name = "HT-COL1-BLU",
			},
			[4] = {
				/**
				 * @description: Raw values
				 * @value0: Red Component (0 to 255)
				 * @value1: Green Component (0 to 255)
				 * @value2: Blue Component (0 to 255)
				 */
				.name = "HT-COL1-RAW",
				.raw_max = USHRT_MAX,
				.si_max = USHRT_MAX,
				.data_sets = 3,
				.data_type = MSENSOR_DATA_U16,
			},
			[5] = {
				/**
				 * @description: Normalized values
				 * @value0: Red Component (0 to 255)
				 * @value1: Green Component (0 to 255)
				 * @value2: Blue Component (0 to 255)
				 * @value3: ??? Component (0 to 255)
				 */
				.name = "HT-COL1-NRM",
				.data_sets = 4,
			},
			[6] = {
				/**
				 * @description: All values
				 * @value0: Color (0 to 17)
				 * @value0_footnote: [^color-value]
				 * @value1: Red Component (0 to 255)
				 * @value2: Green Component (0 to 255)
				 * @value3: Blue Component (0 to 255)
				 */
				.name = "HT-COL1-ALL",
				.data_sets = 4,
			},
		},
		.i2c_mode_info	= {
			[0] = {
				.read_data_reg	= 0x42,
			},
			[1] = {
				.read_data_reg	= 0x43,
			},
			[2] = {
				.read_data_reg	= 0x44,
			},
			[3] = {
				.read_data_reg	= 0x45,
			},
			[4] = {
				.read_data_reg	= 0x46,
			},
			[5] = {
				.read_data_reg	= 0x4C,
			},
			[6] = {
				.read_data_reg	= 0x42,
			},
		},
	},
	[HT_NXT_COLOR_SENSOR_V2] = {
		/**
		 * @vendor_name: HiTechnic
		 * @vendor_part_number: NCO1038
		 * @vendor_part_name: NXT Color Sensor V2
		 * @vendor_website: http://www.hitechnic.com/cgi-bin/commerce.cgi?preadd=action&key=NCO1038
		 * @default_address: 0x01
		 */
		.name		= "ht-nxt-color-v2",
		.vendor_id	= "HiTechnc",
		.product_id	= "ColorPD",
		.num_modes	= 8,
		.num_read_only_modes = 7,
		.ms_mode_info	= {
			[0] = {
				/**
				 * [^color-value]: Color Values:<br />
				 * ![Color chart](http://www.hitechnic.com/contents/media/Color%20Number.jpg)
				 *
				 * @description: Color
				 * @value0: Color (0 to 17)
				 * @value0_footnote: [^color-value]
				 */
				.name	= "HT-COL2-COL",
				.raw_max = 17,
				.si_max = 17,
			},
			[1] = {
				/**
				 * @description: Red component
				 * @value0: Reflected light intensity (0 to 255)
				 */
				.name = "HT-COL2-RED",
			},
			[2] = {
				/**
				 * @description: Green component
				 * @value0: Reflected light intensity (0 to 255)
				 */
				.name = "HT-COL2-GRN",
			},
			[3] = {
				/**
				 * @description: Blue component
				 * @value0: Reflected light intensity (0 to 255)
				 */
				.name = "HT-COL2-BLU",
			},
			[4] = {
				/**
				 * @description: White component
				 * @value0: Reflected light intensity (0 to 255)
				 */
				.name = "HT-COL2-WHT",
			},
			[5] = {
				/**
				 * @description: Normalized values
				 * @value0: Red Component (0 to 255)
				 * @value1: Green Component (0 to 255)
				 * @value2: Blue Component (0 to 255)
				 * @value3: White Component (0 to 255)
				 */
				.name = "HT-COL2-NRM",
				.data_sets = 4,
			},
			[6] = {
				/**
				 * @description: All values
				 * @value0: Red Component (0 to 255)
				 * @value1: Green Component (0 to 255)
				 * @value2: Blue Component (0 to 255)
				 * @value3: White Component (0 to 255)
				 * @value4: ??? (0 to 255)
				 */
				.name = "HT-COL2-ALL",
				.data_sets = 5,
			},
			[7] = {
				/**
				 * @description: Raw values
				 * @value0: Red Component (0 to 255)
				 * @value1: Green Component (0 to 255)
				 * @value2: Blue Component (0 to 255)
				 * @value3: White Component (0 to 255)
				 */
				.name = "HT-COL2-RAW",
				.raw_max = USHRT_MAX,
				.si_max = USHRT_MAX,
				.data_sets = 4,
				.data_type = MSENSOR_DATA_U16,
			},
		},
		.i2c_mode_info	= {
			[0] = {
				.set_mode_reg	= 0x41,
				.read_data_reg	= 0x42,
			},
			[1] = {
				.set_mode_reg	= 0x41,
				.read_data_reg	= 0x43,
			},
			[2] = {
				.set_mode_reg	= 0x41,
				.read_data_reg	= 0x44,
			},
			[3] = {
				.set_mode_reg	= 0x41,
				.read_data_reg	= 0x45,
			},
			[4] = {
				.set_mode_reg	= 0x41,
				.read_data_reg	= 0x46,
			},
			[5] = {
				.set_mode_reg	= 0x41,
				.read_data_reg	= 0x47,
			},
			[6] = {
				.set_mode_reg	= 0x41,
				.read_data_reg	= 0x42,
			},
			[7] = {
				.set_mode_reg	= 0x41,
				.set_mode_data	= 0x03,
				.read_data_reg	= 0x42,
			},
		},
	},
	[HT_NXT_ANGLE_SENSOR] = {
		/**
		 * @vendor_name: HiTechnic
		 * @vendor_part_number: NAA1030
		 * @vendor_part_name: NXT Angle Sensor
		 * @vendor_website: http://www.hitechnic.com/cgi-bin/commerce.cgi?preadd=action&key=NAA1030
		 * @default_address: 0x01
		 */
		.name		= "ht-nxt-angle",
		.vendor_id	= "HITECHNC",
		.product_id	= "AnglSnsr",
		.num_modes	= 3,
		.ms_mode_info	= {
			[0] = {
				/**
				 * @description: Angle
				 * @units_description: degrees
				 * @value0: Angle (0 to 180)
				 */
				.name = "HT-ANG-DEG2",
				.raw_max = 180,
				.si_max = 180,
				.units = "deg",
			},
			[1] = {
				/**
				 * @description: Accumulated angle
				 * @units_description: degrees
				 * @value0: Angle (-2147483648 to 2147483647)
				 */
				.name = "HT-ANG-ACC",
				.raw_min = INT_MIN,
				.raw_max = INT_MAX,
				.si_min = INT_MIN,
				.si_max = INT_MAX,
				.data_type = MSENSOR_DATA_S32,
				.figures = 9,
				.units = "deg",
			},
			[2] = {
				/**
				 * @description: Rotational speed
				 * @units_description: revolutions per minute
				 * @value0: Angle (-32768 to 32768)
				 */
				.name = "HT-ANG-RPM",
				.raw_min = SHRT_MIN,
				.raw_max = SHRT_MAX,
				.si_min = SHRT_MIN,
				.si_max = SHRT_MAX,
				.data_type = MSENSOR_DATA_S16,
				.units = "RPM",
			},
		},
		.i2c_mode_info	= {
			[0] = {
				.read_data_reg	= 0x42,
			},
			[1] = {
				.read_data_reg	= 0x44,
			},
			[2] = {
				.read_data_reg	= 0x46,
			},
		},
		.num_commands	= 1,
		.ms_cmd_info	= {
			[0] = {
				/**
				 * @description: Reset angle values
				 */
				.name = "RESET",
			},
		},
		.i2c_cmd_info	= {
			[0] = {
				.cmd_reg	= 0x41,
				.cmd_data	= 0x52,
			}
		},
	},
	[HT_NXT_COMPASS_SENSOR] = {
		/**
		 * @vendor_name: HiTechnic
		 * @vendor_part_number: NMC1034
		 * @vendor_part_name: NXT Compass Sensor
		 * @vendor_website: http://www.hitechnic.com/cgi-bin/commerce.cgi?preadd=action&key=NMC1034
		 * @default_address: 0x01
		 */
		.name		= "ht-nxt-compass",
		.vendor_id	= "HiTechnc",
		.product_id	= "Compass",
		.num_modes	= 1,
		.ms_mode_info	= {
			[0] = {
				/**
				 * @description: Compass Direction
				 * @value0: Direction (-180 to 180)
				 * @units_description: degrees
				 */
				.name = "HT-CMP-DEG2",
				.raw_max = 180,
				.si_max = 180,
				.units = "deg",
				.data_type = MSENSOR_DATA_S8,
			},
		},
		.i2c_mode_info	= {
			[0] = {
				.read_data_reg	= 0x42,
			},
		},
	},
	[HT_NXT_IR_RECEIVER_SENSOR] = {
		/**
		 * @vendor_name: HiTechnic
		 * @vendor_part_number: NIR1032
		 * @vendor_part_name: NXT IRReceiver Sensor
		 * @vendor_website: http://www.hitechnic.com/cgi-bin/commerce.cgi?preadd=action&key=NIR1032
		 * @default_address: 0x01
		 */
		.name		= "ht-nxt-ir-receiver",
		.vendor_id	= "HiTechnc",
		.product_id	= "IRRecv",
		.num_modes	= 2,
		.ms_mode_info	= {
			[0] = {
				/**
				 * [^values]: Value of -128 is brake. Speed values only occur in
				 * discrete steps (-100, -86, -72, -58, -44, -30, -16, 0, 16, 30,
				 * 44, 58, 72, 86 and 100).
				 *
				 * @description: Single Motor Control
				 * @value0: Motor 1A Speed (-128 and -100 to 100)
				 * @value0_footnote: [^values]<sup>,</sup>[^value-map]
				 * @units_description: percent
				 */
				.name = "HT-IRRECV",
				.units = "pct",
				.data_type = MSENSOR_DATA_S8,
			},
			[1] = {
				/**
				 * [^value-map]: In "Motor NX", the number is the channel,
				 * A is the red/left control and B is the blue/right control.
				 *
				 * @description: Eight Motor Controls
				 * @value0: Motor 1A Speed (-128 and -100 to 100)
				 * @value0_footnote: [^values]<sup>,</sup>[^value-map]
				 * @value1: Motor 1B Speed (-128 and -100 to 100)
				 * @value1_footnote: [^values]<sup>,</sup>[^value-map]
				 * @value2: Motor 2A Speed (-128 and -100 to 100)
				 * @value2_footnote: [^values]<sup>,</sup>[^value-map]
				 * @value3: Motor 2B Speed (-128 and -100 to 100)
				 * @value3_footnote: [^values]<sup>,</sup>[^value-map]
				 * @value4: Motor 3A Speed (-128 and -100 to 100)
				 * @value4_footnote: [^values]<sup>,</sup>[^value-map]
				 * @value5: Motor 3B Speed (-128 and -100 to 100)
				 * @value5_footnote: [^values]<sup>,</sup>[^value-map]
				 * @value6: Motor 4A Speed (-128 and -100 to 100)
				 * @value6_footnote: [^values]<sup>,</sup>[^value-map]
				 * @value7: Motor 4B Speed (-128 and -100 to 100)
				 * @value7_footnote: [^values]<sup>,</sup>[^value-map]
				 * @units_description: percent
				 */
				.name = "HT-IRRECV-8",
				.data_sets = 8,
				.units = "pct",
				.data_type = MSENSOR_DATA_S8,
			},
		},
		.i2c_mode_info	= {
			[0] = {
				.read_data_reg	= 0x42,
			},
			[1] = {
				.read_data_reg	= 0x42,
			},
		},
	},
	[HT_NXT_ACCELERATION_TILT_SENSOR] = {
		/**
		 * @vendor_name: HiTechnic
		 * @vendor_part_number: NAC1040
		 * @vendor_part_name: NXT Acceleration / Tilt Sensor
		 * @vendor_website: http://www.hitechnic.com/cgi-bin/commerce.cgi?preadd=action&key=NAC1040
		 * @default_address: 0x01
		 */
		.name		= "ht-nxt-accel",
		.vendor_id	= "HITECHNC",
		.product_id	= "Accel.",
		.num_modes	= 2,
		.ms_mode_info	= {
			[0] = {
				/**
				 * [^mode-0-value]: Value is 8 most significant bits out of 10-bit total resolution.
				 *
				 * @description: Single-axis acceleration
				 * @value0: Acceleration (coarse value)
				 * @value0_footnote: [^mode-0-value]
				 */
				.name = "HT-ACCL",
			},
			[1] = {
				/**
				 * [^mode-1-value]: Only the 2 most significant bits are used.
				 * Actual value is `MSB << 2 + LSB >> 6` or `MSB << 2 + LSB & 0x03`
				 * (can someone confirm which one?).
				 *
				 * @description: Three-axis acceleration
				 * @value0: X-axis acceleration (most significant byte)
				 * @value1: Y-axis acceleration (most significant byte)
				 * @value2: Z-axis acceleration (most significant byte)
				 * @value3: X-axis acceleration (least significant byte)
				 * @value3_footnote: [^mode-1-value]
				 * @value4: Y-axis acceleration (least significant byte)
				 * @value4_footnote: [^mode-1-value]
				 * @value5: Z-axis acceleration (least significant byte)
				 * @value5_footnote: [^mode-1-value]
				 */
				.name = "HT-ACCL-ALL",
				.data_sets = 6,
			},
		},
		.i2c_mode_info	= {
			[0] = {
				.read_data_reg	= 0x42,
			},
			[1] = {
				.read_data_reg	= 0x42,
			},
		},
	},
	[HT_NXT_IR_LINK_SENSOR] = {
		/**
		 * @vendor_name: HiTechnic
		 * @vendor_part_number: NIL1046
		 * @vendor_part_name: NXT IRLink Sensor
		 * @vendor_website: http://www.hitechnic.com/cgi-bin/commerce.cgi?preadd=action&key=NIL1046
		 * @default_address: 0x01
		 */
		.name		= "ht-nxt-ir-link",
		.vendor_id	= "HiTechnc",
		.product_id	= "IRLink",
		.num_modes	= 1,
		.ms_mode_info	= {
			[0] = {
				/**
				 * @description: ???
				 * @value0: ???
				 */
				.name = "HT-IRLINK",
			},
		},
		.i2c_mode_info	= {
			[0] = {
				.read_data_reg	= 0x42,
			},
		},
	},
	[HT_NXT_SUPER_PRO_SENSOR] = {
		/**
		 * @vendor_name: HiTechnic
		 * @vendor_part_number: SPR2010
		 * @vendor_part_name: NXT SuperPro Prototype Board
		 * @vendor_website: http://www.hitechnic.com/cgi-bin/commerce.cgi?preadd=action&key=SPR2010
		 * @default_address: 0x08
		 */
		.name		= "ht-super-pro",
		.vendor_id	= "HiTechnc",
		.product_id	= "SuperPro",
		.num_modes	= 5,
		.ms_mode_info	= {
			[0] = {
				/**
				 * @description: Analog inputs
				 * @value0: Analog input A0 (0 to 1023)
				 * @value1: Analog input A1 (0 to 1023)
				 * @value2: Analog input A2 (0 to 1023)
				 * @value3: Analog input A3 (0 to 1023)
				 */
				.name = "HT-SPRO-AIN",
				.data_sets = 4,
				.data_type = MSENSOR_DATA_U16,
			},
			[1] = {
				/**
				 * @description: Digital inputs
				 * @value0: Bits B0-B7 (0 to 255)
				 */
				.name = "HT-SPRO-DIN",
			},
			[2] = {
				/**
				 * @description: Digital outputs
				 * @value0: Bits B0-B7 (0 to 255)
				 */
				.name = "HT-SPRO-DOT",
			},
			[3] = {
				/**
				 * @description: Digital input/output controls
				 * @value0: Bits B0-B7 (0 to 255)
				 */
				.name = "HT-SPRO-DCT",
			},
			[4] = {
				/**
				 * @description: Strobe output
				 * @value0: Bits S0-S3 (0 to 15)
				 */
				.name = "HT-SPRO-STB",
			},
			[5] = {
				/**
				 * [^led-states]: LED states:
				 *
				 * | Value | Description  |
				 * |-------|--------------|
				 * | 0     | None         |
				 * | 1     | Red          |
				 * | 2     | Blue         |
				 * | 3     | Red and blue |
				 *
				 * @description: LED control
				 * @value0: LED state
				 * @value0_footnote: [^led-states]
				 */
				.name = "HT-SPRO-LED",
			},
			[6] = {
				/**
				 * @description: Analog output O0
				 * @value0: Mode
				 * @value1: Frequency, most significant byte
				 * @value2: Frequency, least significant byte
				 * @value3: Voltage, most significant byte
				 * @value4: Voltage, least significant byte
				 */
				.name = "HT-SPRO-AO0",
				.data_sets = 5,
			},
			[7] = {
				/**
				 * @description: Analog output O1
				 * @value0: Mode
				 * @value1: Frequency, most significant byte
				 * @value2: Frequency, least significant byte
				 * @value3: Voltage, most significant byte
				 * @value4: Voltage, least significant byte
				 */
				.name = "HT-SPRO-AO1",
				.data_sets = 5,
			},
		},
		.i2c_mode_info	= {
			[0] = {
				.read_data_reg = 0x42,
			},
			[1] = {
				.read_data_reg	= 0x4C,
			},
			[2] = {
				.read_data_reg	= 0x4D,
			},
			[3] = {
				.read_data_reg	= 0x4E,
			},
			[4] = {
				.read_data_reg	= 0x50,
			},
			[5] = {
				.read_data_reg	= 0x51,
			},
			[6] = {
				.read_data_reg	= 0x52,
			},
			[7] = {
				.read_data_reg	= 0x57,
			},
		},
	},
	[HT_NXT_SENSOR_MUX] = {
		/**
		 * [^more-devices]: The `ht-nxt-smux` driver loads more devices in addition to
		 * the [msensor] device. See [ht-smux-input-port](../ht-smux-input-port) for
		 * more information.
		 *
		 * @vendor_name: HiTechnic
		 * @vendor_part_number: NSX2020
		 * @vendor_part_name: NXT Sensor Multiplexer
		 * @vendor_website: http://www.hitechnic.com/cgi-bin/commerce.cgi?preadd=action&key=NSX2020
		 * @default_address: 0x08
		 * @device_class_footnote: [^more-devices]
		 */
		.name			= "ht-nxt-smux",
		.vendor_id		= "HiTechnc",
		.product_id		= "SensrMUX",
		.num_modes		= 3,
		.num_read_only_modes	= 1,
		.ops.set_mode_pre_cb	= ht_sensor_mux_set_mode_pre_cb,
		.ops.set_mode_post_cb	= ht_sensor_mux_set_mode_post_cb,
		.ops.poll_cb		= ht_sensor_mux_poll_cb,
		.ops.remove_cb		= ht_sensor_mux_remove_cb,
		.ms_mode_info = {
			[0] = {
				/**
				 * [^mode]: Actual mode:
				 *
				 * | Value | Description |
				 * |-------|-------------|
				 * | 0     | Halt        |
				 * | 1     | Detect      |
				 * | 2     | Run         |
				 *
				 * [^status-bits]: Status bits:
				 *
				 * | Bit | Description    |
				 * |-----|----------------|
				 * | 0   | Low/no battery |
				 * | 1   | Running        |
				 * | 2   | Halted         |
				 * | 3   | Error          |
				 *
				 * @description: Run mode (polling attached sensors)
				 * @value0: Mode
				 * @value0_footnote: [^mode]
				 * @value1: Status
				 * @value1_footnote: [^status-bits]
				 */
				.name = "HT-SMUX-RUN",
				.data_sets = 2,
			},
			[1]= {
				/**
				 * [^auto-detect-mode]: The sensor must be in `HT-SMUX-HALT` mode before
				 * entering `HT-SMUX-DETECT` mode. Attempting to set `HT-SMUX-DETECT`
				 * mode from `HT-SMUX-RUN` mode will result in an error (-EPERM).
				 *
				 * [^auto-detect-sensors]: Only these sensors can be auto-detected:
				 *
				 * - LEGO NXT Ultrasonic
				 * - HiTechnic NXT Compass
				 * - HiTechnic NXT Color
				 * - HiTechnic NXT Acceleration / Tilt
				 * - HiTechnic NXT IR Seeker
				 * - HiTechnic Super Pro
				 * - HiTechnic NXT Color V2
				 * - HiTechnic NXT IR Seeker V2
				 * @description: Start auto-detection
				 * @name_footnote: [^auto-detect-mode]<sup>,</sup>[^auto-detect-sensors]
				 * @value0: Mode
				 * @value0_footnote: [^mode]
				 * @value1: Status
				 * @value1_footnote: [^status-bits]
				 */
				.name = "HT-SMUX-DETECT",
				.data_sets = 2,
			},
			[2]= {
				/**
				 * @description: Halt mode (not polling)
				 * @value0: Mode
				 * @value0_footnote: [^mode]
				 * @value1: Status
				 * @value1_footnote: [^status-bits]
				 */
				.name = "HT-SMUX-HALT",
				.data_sets = 2,
			},
		},
		.i2c_mode_info = {
			[0] = {
				.read_data_reg	= 0x20,
				.set_mode_reg = 0x20,
				.set_mode_data = 2,
			},
			[1] = {
				.read_data_reg	= 0x20,
				.set_mode_reg = 0x20,
				.set_mode_data = 1,
			},
			[2] = {
				.read_data_reg	= 0x20,
				.set_mode_reg = 0x20,
				.set_mode_data = 0,
			},
		},
	},
	[MS_8CH_SERVO] = {
		/**
		 * [^address]: The address is programmable. See manufacturer
		 * documentation for more information.
		 * [^servo-motor-devices]: The `ms-8ch-servo` driver loads separate
		 * servo motor devices (one for each of the 8 channels) in addition
		 * to the [msensor] device. See the [Servo Motor Class](../servo-motor-class)
		 * for more information. The `servo_motor` class `port_name` attribute
		 * will return `in<N>:sv<M>` where `<N>` is the input port the servo
		 * controller is connected to and `<M>` is the channel as indicated
		 * on the servo controller itself.
		 *
		 * @vendor_name: mindsensors.com
		 * @vendor_part_number: NxtServo
		 * @vendor_part_name: 8-channel Servo Controller
		 * @vendor_website: http://mindsensors.com/index.php?module=pagemaster&PAGE_user_op=view_page&PAGE_id=93
		 * @default_address: 0x58
		 * @default_address_footnote: [^address]
		 * @device_class_footnote: [^servo-motor-devices]
		 */
		.name			= "ms-8ch-servo",
		.vendor_id		= "mndsnsrs",
		.product_id		= "NXTServo",
		.num_modes		= 2,
		.ops.probe_cb		= ms_8ch_servo_probe_cb,
		.ops.remove_cb		= ms_8ch_servo_remove_cb,
		.ms_mode_info		= {
			[0] = {
				/**
				 *
				 * [^battery-voltage]: The current voltage scaling is based on
				 * the manufacturers documentation, however it seems to be low.
				 * If you are seeing this too, please open an issue on GitHub
				 * and we will change the scaling.
				 *
				 * @description: EV3 Compatible
				 * @value0: Battery voltage (0 to 9400)
				 * @value0_footnote: [^battery-voltage]
				 * @units_description: volts
				 */
				.name = "MS-8CH-SERVO-V3",
				.raw_min = 127,
				.raw_max = 255,
				.si_min = 4700,
				.si_max = 9400,
				.decimals = 3,
				.units = "V",
			},
			[1] = {
				/**
				 * [^old-mode]: Older versions of this sensor have the battery
				 * voltage at a different address. If the default mode does not
				 * return a value, try this mode.
				 *
				 * @name_footnote: [^old-mode]
				 * @description: Older versions
				 * @value0: Battery voltage (0 to 9400)
				 * @value0_footnote: [^battery-voltage]
				 * @units_description: volts
				 */
				.name = "MS-8CH-SERVO",
				.raw_min = 127,
				.raw_max = 255,
				.si_min = 4700,
				.si_max = 9400,
				.decimals = 3,
				.units = "V",
			},
		},
		.i2c_mode_info	= {
			[0] = {
				.read_data_reg	= 0x62,
			},
			[1] = {
				.read_data_reg	= 0x41,
			},
		},
	},
	[MS_ABSOLUTE_IMU] = {
		/**
		 * [^address]: The address is programmable. See manufacturer
		 * documentation for more information.
		 *
		 * @vendor_name: mindsensors.com
		 * @vendor_part_number: AbsoluteIMU(-A/C/G)
		 * @vendor_part_name: Gyro, MultiSensitivity Accelerometer and/or Compass
		 * @vendor_website: http://www.mindsensors.com/index.php?module=pagemaster&PAGE_user_op=view_page&PAGE_id=169&MMN_position=30:30
		 * @default_address: 0x11
		 * @default_address_footnote: [^address]
		 */
		.name			= "ms-absolute-imu",
		.vendor_id		= "mndsnsrs",
		.product_id		= "AbsIMU",
		.num_modes		= 8,
		.num_read_only_modes	= 6,
		.ops.poll_cb		= ms_imu_poll_cb,
		.ms_mode_info	= {
			[0] = {
				/**
				 * @description: Tilt
				 * @value0: X-axis angle (0 to 180)
				 * @value1: Y-axis angle (0 to 180)
				 * @value2: Y-axis angle (0 to 180)
				 * @units_description: degrees
				 */
				.name		= "MS-IMU-TILT",
				.data_sets	= 3,
				.data_type	= MSENSOR_DATA_U8,
				.units		= "deg",
			},
			[1] = {
				/**
				 * [^accel]: Only returns data from models with an accelerometer
				 * (AbsoluteIMU-AC/AbsoluteIMU-A).
				 *
				 * @name_footnote: [^accel]
				 * @description: Acceleration
				 * @value0: X-axis acceleration
				 * @value1: Y-axis acceleration
				 * @value2: Z-axis acceleration
				 * @units_description: milli-G (G = 9.81m/s<sup>2</sup>)
				 */
				.name		= "MS-IMU-ACCEL",
				.data_sets	= 3,
				.data_type	= MSENSOR_DATA_S16,
				.units		= "mG",
			},
			[2] = {
				/**
				 * [^compass]: Only returns data from models with a compass
				 * (AbsoluteIMU-C/AbsoluteIMU-AC/AbsoluteIMU-ACG).
				 *
				 * @name_footnote: [^compass]
				 * @description: Compass
				 * @value0: Heading (0 to 360)
				 * @units_description: degrees
				 */
				.name		= "MS-IMU-COMP",
				.data_sets	= 1,
				.decimals	= 2,
				.units		= "deg",
				.data_type	= MSENSOR_DATA_U16,
			},
			[3] = {
				/**
				 * @name_footnote: [^compass]
				 * @description: Magnetic field
				 * @value0: X-axis magnetic field
				 * @value1: Y-axis magnetic field
				 * @value2: Z-axis magnetic field
				 */
				.name		= "MS-IMU-MAG",
				.data_sets	= 3,
				.data_type	= MSENSOR_DATA_S16,
			},
			[4] = {
				/**
				 * [^gyro]: Only returns data from models with a gyro
				 * (AbsoluteIMU-ACG).
				 *
				 * @name_footnote: [^gyro]
				 * @description: Gryo (250 deg/sec sensitivity)
				 * @value0: X-axis rotational speed
				 * @value1: Y-axis rotational speed
				 * @value2: Z-axis rotational speed
				 * @units_description: degrees per second
				 */
				.name		= "MS-IMU-GYRO-250",
				.raw_max	= 1000,
				.si_max		= 875,
				.decimals	= 2,
				.data_sets	= 3,
				.data_type	= MSENSOR_DATA_S16,
				.units		= "d/s",
			},
			[5] = {
				/**
				 * [^all]: Reads all data from the sensor. Use `bin_data`
				 * attribute to read values. Some values will not be scaled.
				 * See manufacturer docs for more info.
				 *
				 * @name_footnote: [^all]
				 * @description: All data
				 */
				.name		= "MS-IMU-ALL",
				.data_sets	= 23,
			},
			[6] = {
				/**
				 * @name_footnote: [^gyro]
				 * @description: Gryo (500 deg/sec sensitivity)
				 * @value0: X-axis rotational speed
				 * @value1: Y-axis rotational speed
				 * @value2: Z-axis rotational speed
				 * @units_description: degrees per second
				 */
				.name		= "MS-IMU-GYRO-500",
				.raw_max	= 1000,
				.si_max		= 175,
				.decimals	= 1,
				.data_sets	= 3,
				.data_type	= MSENSOR_DATA_S16,
				.units		= "d/s",
			},
			[7] = {
				/**
				 * @name_footnote: [^gyro]
				 * @description: Gryo (2000 deg/sec sensitivity)
				 * @value0: X-axis rotational speed
				 * @value1: Y-axis rotational speed
				 * @value2: Z-axis rotational speed
				 * @units_description: degrees per second
				 */
				.name		= "MS-IMU-GYRO-2000",
				.raw_max	= 1000,
				.si_max		= 700,
				.decimals	= 1,
				.data_sets	= 3,
				.data_type	= MSENSOR_DATA_S16,
				.units		= "d/s",
			},
		},
		.i2c_mode_info	= {
			[0] = {
				.read_data_reg	= 0x42,
			},
			[1] = {
				.read_data_reg	= 0x45,
			},
			[2] = {
				.read_data_reg	= 0x4B,
			},
			[3] = {
				.read_data_reg	= 0x4D,
			},
			[4] = {
				.read_data_reg	= 0x53,
				.set_mode_reg	= 0x41,
				.set_mode_data	= '1',
			},
			[5] = {
				.read_data_reg	= 0x42,
			},
			[6] = {
				.read_data_reg	= 0x53,
				.set_mode_reg	= 0x41,
				.set_mode_data	= '2',
			},
			[7] = {
				.read_data_reg	= 0x53,
				.set_mode_reg	= 0x41,
				.set_mode_data	= '3',
			},
		},
		.num_commands	= 6,
		.ms_cmd_info	= {
			[0] = {
				/**
				 * @description: Begin compass calibration
				 */
				.name = "BEGIN-COMP-CAL",
			},
			[1] = {
				/**
				 * @description: End compass calibration
				 */
				.name = "END-COMP-CAL",
			},
			[2] = {
				/**
				 * [^accel-commands]: The acceleration commands are only
				 * intended for use with the `MS-IMU-ACCEL` and `MS-IMU-ALL`
				 * modes. Sending these commands in any of the `MS-IMU-GRYO-*`
				 * modes will cause incorrect readings.
				 *
				 * @description: Change Accelerometer Sensitivity to 2G
				 * @name_footnote: [^accel-commands]
				 */
				.name = "ACCEL-2G",
			},
			[3] = {
				/**
				 * @description: Change Accelerometer Sensitivity to 4G
				 * @name_footnote: [^accel-commands]
				 */
				.name = "ACCEL-4G",
			},
			[4] = {
				/**
				 * @description: Change Accelerometer Sensitivity to 8G
				 * @name_footnote: [^accel-commands]
				 */
				.name = "ACCEL-8G",
			},
			[5] = {
				/**
				 * @description: Change Accelerometer Sensitivity to 16G
				 * @name_footnote: [^accel-commands]
				 */
				.name = "ACCEL-16G",
			},
		},
		.i2c_cmd_info	= {
			[0] = {
				.cmd_reg	= 0x41,
				.cmd_data	= 'C',
			},
			[1] = {
				.cmd_reg	= 0x41,
				.cmd_data	= 'c',
			},
			[2] = {
				.cmd_reg	= 0x41,
				.cmd_data	= '1',
			},
			[3] = {
				.cmd_reg	= 0x41,
				.cmd_data	= '2',
			},
			[4] = {
				.cmd_reg	= 0x41,
				.cmd_data	= '3',
			},
			[5] = {
				.cmd_reg	= 0x41,
				.cmd_data	= '4',
			},
		},
	},
	[MS_ANGLE_SENSOR] = {
		/**
		 * @vendor_name: mindsensors.com
		 * @vendor_part_number: AngleSensor
		 * @vendor_part_name: GlideWheel-AS
		 * @vendor_website: http://www.mindsensors.com/index.php?module=pagemaster&PAGE_user_op=view_page&PAGE_id=173
		 * @default_address: 0x18
		 */
		.name			= "ms-angle-sensor",
		.vendor_id		= "mndsnsrs",
		.product_id		= "AngSens",
		.num_modes		= 4,
		.ms_mode_info	= {
			[0] = {
				/**
				 * @description: Angle
				 * @value0: Angle
				 * @units_description: degrees
				 */
				.name		= "MS-AS-ANGLE",
				.data_sets	= 1,
				.data_type	= MSENSOR_DATA_S32,
				.units		= "deg",
			},
			[1] = {
				/**
				 * @description: High-precision angle
				 * @value0: Angle
				 * @units_description: degrees
				 */
				.name		= "MS-AS-ANGLE2",
				.raw_max	= 360,
				.si_max		= 1800,
				.data_sets	= 1,
				.decimals	= 1,
				.data_type	= MSENSOR_DATA_S32,
				.units		= "deg",
			},
			[2] = {
				/**
				 * @description: Rotational Speed
				 * @value0: Rotational Speed (-4000 to 4000)
				 * @units_description: revolutions per minute
				 */
				.name		= "MS-AS-RPM",
				.raw_max	= 100,
				.si_max		= 100,
				.data_sets	= 1,
				.units		= "rpm",
				.data_type	= MSENSOR_DATA_S16,
			},
			[3] = {
				/**
				 * [^mode3-value1]: Angle value times 2
				 * (i.e. value of 10 = angle of 5 degrees).
				 * Allows for 0.5 degree precision.
				 * [^mode3-value2]: Value needs to be converted to
				 * 16-bit signed integer. Example:
				 * `if (value2 > 32767) value2 = value2 - 65536`
				 *
				 * @description: All values
				 * @value0: Angle
				 * @value1: Angle x2
				 * @value1_footnote: [^mode3-value1]
				 * @value2: Rotational Speed
				 * @value2_footnote: [^mode3-value2]
				 */
				.name		= "MS-AS-ALL",
				.raw_max	= 100,
				.si_max		= 100,
				.data_sets	= 3,
				.data_type	= MSENSOR_DATA_S32,
			},
		},
		.i2c_mode_info	= {
			[0] = {
				.read_data_reg	= 0x42,
			},
			[1] = {
				.read_data_reg	= 0x46,
			},
			[2] = {
				.read_data_reg	= 0x4A,
			},
			[3] = {
				.read_data_reg	= 0x42,
			},
		},
		.num_commands	= 1,
		.ms_cmd_info	= {
			[0] = {
				/**
				 * @description: Reset angle values
				 */
				.name = "RESET",
			}
		},
		.i2c_cmd_info	= {
			[0] = {
				.cmd_reg	= 0x41,
				.cmd_data	= 'r',
			},
		}
	},
	[MS_LIGHT_SENSOR_ARRAY] = {
		/**
		 * [^address]: The address is programmable. See manufacturer
		 * documentation for more information.
		 *
		 * @vendor_name: mindsensors.com
		 * @vendor_part_number: LightSensorArray
		 * @vendor_part_name: Light Sensor Array
		 * @vendor_website: http://mindsensors.com/index.php?module=pagemaster&PAGE_user_op=view_page&PAGE_id=168
		 * @default_address: 0x0A
		 * @default_address_footnote: [^address]
		 */
		.name			= "ms-light-array",
		.vendor_id		= "mndsnsrs",
		.product_id		= "LSArray",
		.num_modes		= 2,
		.ms_mode_info	= {
			[0] = {
				/**
				 * @description: Calibrated values
				 * @value0: LED 0 (0 to 100)
				 * @value1: LED 1 (0 to 100)
				 * @value2: LED 2 (0 to 100)
				 * @value3: LED 3 (0 to 100)
				 * @value4: LED 4 (0 to 100)
				 * @value5: LED 5 (0 to 100)
				 * @value6: LED 6 (0 to 100)
				 * @value7: LED 7 (0 to 100)
				 * @units_description: percent
				 */
				.name	= "MS-LSA-CAL",
				.raw_max = 100,
				.si_max = 100,
				.data_sets = 8,
				.units	= "pct",
			},
			[1] = {
				/**
				 * @description: Raw values
				 * @value0: LED 0 (0 to ???)
				 * @value1: LED 1 (0 to ???)
				 * @value2: LED 2 (0 to ???)
				 * @value3: LED 3 (0 to ???)
				 * @value4: LED 4 (0 to ???)
				 * @value5: LED 5 (0 to ???)
				 * @value6: LED 6 (0 to ???)
				 * @value7: LED 7 (0 to ???)
				 */
				.name	= "MS-LSA-RAW",
				.raw_max = USHRT_MAX,
				.si_max = USHRT_MAX,
				.data_sets = 8,
				.data_type = MSENSOR_DATA_S16,
			},
		},
		.i2c_mode_info	= {
			[0] = {
				.read_data_reg	= 0x42,
			},
			[1] = {
				.read_data_reg	= 0x6A,
			},
		},
		.num_commands	= 7,
		.ms_cmd_info	= {
			[0] = {
				/**
				 * @description: Calibrate white
				 */
				.name = "CAL-WHITE",
			},
			[1] = {
				/**
				 * @description: Calibrate black
				 */
				.name = "CAL-BLACK",
			},
			[2] = {
				/**
				 * [^sleep]: `poll_ms` must be set to `0` in order for sensor to sleep.
				 *
				 * @name_footnote: [^sleep]
				 * @description: Put sensor to sleep
				 */
				.name = "SLEEP",
			},
			[3] = {
				/**
				 * [^wake]: Will return an error (-ENXIO) if sensor is actually asleep.
				 * Completes successfully if sensor is already awake.
				 *
				 * @name_footnote: [^wake]
				 * @description: Wake up the sensor
				 */
				.name = "WAKE",
			},
			[4] = {
				/**
				 * @description: Configures sensor for 60Hz electrical mains
				 */
				.name = "60HZ",
			},
			[5] = {
				/**
				 * @description: Configures sensor for 50Hz electrical mains
				 */
				.name = "50HZ",
			},
			[6] = {
				/**
				 * @description: Configures sensor for any (50/60Hz) electrical mains
				 */
				.name = "UNIVERSAL",
			},
		},
		.i2c_cmd_info	= {
			[0] = {
				.cmd_reg	= 0x41,
				.cmd_data	= 'W',
			},
			[1] = {
				.cmd_reg	= 0x41,
				.cmd_data	= 'B',
			},
			[2] = {
				.cmd_reg	= 0x41,
				.cmd_data	= 'D',
			},
			[3] = {
				.cmd_reg	= 0x41,
				.cmd_data	= 'P',
			},
			[4] = {
				.cmd_reg	= 0x41,
				.cmd_data	= 'A',
			},
			[5] = {
				.cmd_reg	= 0x41,
				.cmd_data	= 'E',
			},
			[6] = {
				.cmd_reg	= 0x41,
				.cmd_data	= 'U',
			},
		},
	},
	[MI_CRUIZCORE_XG1300L] = {
		/**	
		 * [^usage]: Sample usage:
		 *
		 * CruizCore XG1300L doesn't follow LEGO guidelines and it can't be autodected.
		 * Until we find other way to identify the sensor the driver has to be loaded manually.
		 * 
		 * Register I2C device:
		 *
		 * <pre><code>echo mi-cruizcore-xg1300 0x01 > /sys/bus/i2c/devices/i2c-<port+2>/new_device
		 * </code></pre>
		 *
		 * @vendor_name: Microinfinity
		 * @vendor_part_number: CruizCore XG 1300L
		 * @vendor_part_name: Digital Gyroscope And Accelerometer
		 * @vendor_website: http://www.minfinity.com/eng/page.php?Main=1&sub=1&tab=5
		 * @default_address: 0x01
		 * @default_address_footnote: [^usage]
		 */
		.name							= "mi-xg1300l",
		.vendor_id						= "mnfinity", /* The sensor doesn't return vendor_id, it can't be autodetected this way */
		.product_id					= "XG1300L",  /* The sensor doesn't return product_id, it can't be autodetected this way */
		.num_modes						= 4,
		.num_read_only_modes			= 4,
		.ops.poll_cb					= mi_cruizcore_xg1300l_poll_cb,
		.ops.send_command_post_cb 	= mi_cruizcore_xg1300l_send_command_post_cb,
		.ops.probe_cb					= mi_cruizcore_xg1300l_probe_cb,
		.ops.remove_cb				= mi_cruizcore_xg1300l_remove_cb,
		.ms_mode_info	= {
			[0] = {
				/**
				 * @description: Accumulated angle
				 * @value0: Z-axis accumulated angle (-180 to 180)
				 * @units_description: degrees
				 */
				.name		= "ANG-ACC",
				.data_sets	= 1,
				.data_type	= MSENSOR_DATA_S16,
				.units		= "deg",
				.decimals	= 2,
			},
			[1] = {
				/**
				 *
				 * @description: Rotational speed
				 * @value0: Z-axis rotational speed
				 * @units_description: degrees per second
				 */
				.name		= "ANG-SPEED",
				.data_sets	= 1,
				.data_type	= MSENSOR_DATA_S16,
				.decimals	= 2,
				.units		= "d/s",
			},
			[2] = {
				/**
				 *
				 * @description: Acceleration in X, Y, Z axis
				 * @value0: Acceleration in X axis
				 * @value1: Acceleration in Y axis
				 * @value2: Acceleration in Z axis
				 * @units_description: g (1g ~ 9.81 m/s2)
				 */
				.name		= "ACCEL-XYZ",
				.data_sets	= 3,
				.units		= "g",
				.data_type	= MSENSOR_DATA_S16,
				.decimals	= 3,
			},
			[3] = {
				/**
				 * [^acceleration-description]: three decimal places, range as was set by last command
				 * [^accumulated-angle-description]: two decimal places, (-180 to 180)
				 * [^rotational-speed-description]: two decimal places
				 *
				 * @description: All values
				 * @value0: Z-axis accumulated angle
				 * @value0_footnote: [^accumulated-angle-description]
				 * @value1: Z-axis rotational speed
				 * @value1_footnote: [^rotational-speed-description]
				 * @value2: X-axis acceleration
				 * @value2_footnote: [^acceleration-description]
				 * @value3: Y-axis acceleration
				 * @value3_footnote: [^acceleration-description]
				 * @value4: Z-axis acceleration
				 * @value4_footnote: [^acceleration-description]
				 *
				 */
				.name		= "ALL",
				.data_sets	= 5,
				.data_type	= MSENSOR_DATA_S16,
			},
		},
		.i2c_mode_info	= {
			[0] = {
				.read_data_reg	= 0x42,
			},
			[1] = {
				.read_data_reg	= 0x44,
			},
			[2] = {
				.read_data_reg	= 0x46,
			},
			[3] = {
				.read_data_reg	= 0x42,
			},
		},
		.num_commands	= 4,
		.ms_cmd_info	= {
			[0] = {
				/**
				 * [^reset-description]: recalculate bias drift, reset accumulated angle,
				 * set accelerometer scaling factor to 2G,
				 * this has to be done with sensor not moving
				 * and is strongly recommended to be called manually before work
				 *
				 * @description: Reset device
				 * @description_footnote: [^reset-description]
				 */
				.name		= "RESET",
			},
			[1] = {
				/**
				 * @description: Acceleration scaling 2G
				 */
				.name		= "ACCEL-2G",
			},
			[2] = {
				/**
				 * @description: Acceleration scaling 4G
				 */
				.name		= "ACCEL-4G",
			},
			[3] = {
				/**
				 * @description: Acceleration scaling 8G
				 */
				.name		= "ACCEL-8G",
			},
		},
		.i2c_cmd_info	= {
			[0] = {
				.cmd_reg	= 0x60,
			},
			[1] = {
				.cmd_reg	= 0x61,
			},
			[2] = {
				.cmd_reg	= 0x62,
			},
			[3] = {
				.cmd_reg	= 0x63,
			},
		},
	},
};

EXPORT_SYMBOL_GPL(nxt_i2c_sensor_defs);
