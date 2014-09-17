/*
 * NXT I2C sensor device driver for LEGO Mindstorms EV3
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

#include <linux/i2c.h>
#include <linux/legoev3/legoev3_ports.h>

#include "nxt_i2c_sensor.h"
#include "ht_smux.h"

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
			pdata.sensor_i2c_data = data->ms.mode_info[MSENSOR_MODE_MAX].raw_data;
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
	int i;

	if (ports) {
		/* Use unused mode's raw_data to store I2C data bytes */
		i2c_smbus_read_i2c_block_data(data->client, HT_SMUX_DATA_GROUP2_BASE,
			32, data->ms.mode_info[MSENSOR_MODE_MAX].raw_data);
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

/**
 * nxt_i2c_sensor_defs - Sensor definitions
 *
 * Required values:
 * - vendor_id
 * - product_id
 * - num_modes
 * - mode_info.ms_mode_info.name
 * - i2c_mode_info.read_data_reg
 *
 * Optional values:
 * - num_read_only_modes (default num_modes)
 * - ops.set_mode_cb
 * - ops.poll_cb
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
 */
const struct nxt_i2c_sensor_info nxt_i2c_sensor_defs[] = {
	[UNKNOWN_I2C_SENSOR] = {
		.vendor_id	= "UNKNOWN",
		.product_id	= "unknown",
		.num_modes	= 3,
		.ms_mode_info	= {
			[0] = {
				.name = "I2C-U8",
			},
			[1] = {
				.name = "I2C-S8",
				.data_type = MSENSOR_DATA_S8,
			},
			[2] = {
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
		.vendor_id	= "LEGO",
		.product_id	= "Sonar",
		.num_modes	= 5,
		.num_read_only_modes = 2,
		.slow		= true,
		.ms_mode_info	= {
			[0] = {
				.name	= "NXT-US-CM",
				.units	= "cm",
			},
			[1] = {
				.name	= "NXT-US-IN",
				.units	= "in",
				.si_max = 1000,
				.decimals = 1,
			},
			[2] = {
				.name	= "NXT-US-SI-CM",
				.units	= "cm",
			},
			[3] = {
				.name	= "NXT-US-SI-IN",
				.units	= "in",
				.si_max = 1000,
				.decimals = 1,
			},
			[4] = {
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
		.vendor_id	= "LEGO",
		.product_id	= "", /* LMS2012 fakes this with "Store." */
		.num_modes	= 8,
		.ms_mode_info	= {
			[0] = {
				.name = "ES-IN-VOLT",
				.units = "V",
				.raw_max = 10000,
				.si_max = 10000,
				.decimals = 3,
				.data_type = MSENSOR_DATA_S16_BE,
			},
			[1] = {
				.name = "ES-IN-AMP",
				.units = "A",
				.raw_max = 10000,
				.si_max = 10000,
				.decimals = 3,
				.data_type = MSENSOR_DATA_S16_BE,
			},
			[2] = {
				.name = "ES-OUT-VOLT",
				.units = "V",
				.raw_max = 10000,
				.si_max = 10000,
				.decimals = 3,
				.data_type = MSENSOR_DATA_S16_BE,
			},
			[3] = {
				.name = "ES-OUT-AMP",
				.units = "A",
				.raw_max = 10000,
				.si_max = 10000,
				.decimals = 3,
				.data_type = MSENSOR_DATA_S16_BE,
			},
			[4] = {
				.name = "ES-JOULE",
				.units = "J",
				.raw_max = 100,
				.si_max = 100,
				.data_type = MSENSOR_DATA_S16_BE,
			},
			[5] = {
				.name = "ES-IN-WATT",
				.units = "W",
				.raw_max = 10000,
				.si_max = 10000,
				.decimals = 3,
				.data_type = MSENSOR_DATA_S16_BE,
			},
			[6] = {
				.name = "ES-OUT-WATT",
				.units = "W",
				.raw_max = 10000,
				.si_max = 10000,
				.decimals = 3,
				.data_type = MSENSOR_DATA_S16_BE,
			},
			[7] = {
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
		.vendor_id	= "HITECHNC",
		.product_id	= "PIR",
		.num_modes	= 1,
		.ms_mode_info	= {
			[0] = {
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
		.vendor_id	= "HiTechnc",
		.product_id	= "Barometr",
		.num_modes	= 2,
		.ms_mode_info	= {
			[0] = {
				.name = "HT-BAR-PRES",
				.raw_min = 30400,
				.raw_max = 29400,
				.si_max = 3000,
				.decimals = 1,
				.units = "m",
			},
			[1] = {
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
		.vendor_id	= "HiTechnc",
		.product_id	= "NewIRDir",
		.num_modes	= 4,
		.ms_mode_info	= {
			[0] = {
				.name = "HT-DIR-DC",
				.raw_max = 9,
				.si_max = 9,
			},
			[1] = {
				.name = "HT-DIR-AC",
				.raw_max = 9,
				.si_max = 9,
			},
			[2] = {
				.name = "HT-DIR-DALL",
				.data_sets = 7,
			},
			[3] = {
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
		.vendor_id	= "HiTechnc",
		.product_id	= "Color",
		.num_modes	= 7,
		.ms_mode_info	= {
			[0] = {
				.name	= "HT-COL1-COL",
				.raw_max = 17,
				.si_max = 17,
			},
			[1] = {
				.name = "HT-COL1-RED",
			},
			[2] = {
				.name = "HT-COL1-GRN",
			},
			[3] = {
				.name = "HT-COL1-BLU",
			},
			[4] = {
				.name = "HT-COL1-RAW",
				.raw_max = USHRT_MAX,
				.si_max = USHRT_MAX,
				.data_sets = 3,
				.data_type = MSENSOR_DATA_U16,
			},
			[5] = {
				.name = "HT-COL1-NRM",
				.data_sets = 4,
			},
			[6] = {
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
		.vendor_id	= "HiTechnc",
		.product_id	= "ColorPD",
		.num_modes	= 8,
		.num_read_only_modes = 7,
		.ms_mode_info	= {
			[0] = {
				.name	= "HT-COL2-COL",
				.raw_max = 17,
				.si_max = 17,
			},
			[1] = {
				.name = "HT-COL2-RED",
			},
			[2] = {
				.name = "HT-COL2-GRN",
			},
			[3] = {
				.name = "HT-COL2-BLU",
			},
			[4] = {
				.name = "HT-COL2-WHT",
			},
			[5] = {
				.name = "HT-COL2-NRM",
				.data_sets = 4,
			},
			[6] = {
				.name = "HT-COL2-ALL",
				.data_sets = 5,
			},
			[7] = {
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
		.vendor_id	= "HITECHNC",
		.product_id	= "AnglSnsr",
		.num_modes	= 4,
		.num_read_only_modes = 3,
		.ms_mode_info	= {
			[0] = {
				.name = "HT-ANG-DEG2",
				.raw_max = 180,
				.si_max = 180,
				.units = "deg",
			},
			[1] = {
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
				.name = "HT-ANG-RPM",
				.raw_min = SHRT_MIN,
				.raw_max = SHRT_MAX,
				.si_min = SHRT_MIN,
				.si_max = SHRT_MAX,
				.data_type = MSENSOR_DATA_S16,
				.units = "RPM",
			},
			[3] = {
				.name = "HT-ANG-RSET",
				.raw_max = 180,
				.si_max = 180,
				.units = "deg",
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
				.set_mode_reg	= 0x41,
				.set_mode_data	= 0x52,
				.read_data_reg	= 0x42,
			},
		},
	},
	[HT_NXT_COMPASS_SENSOR] = {
		.vendor_id	= "HiTechnc",
		.product_id	= "Compass",
		.num_modes	= 1,
		.ms_mode_info	= {
			[0] = {
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
	[HT_NXT_IR_RECIEIVER_SENSOR] = {
		.vendor_id	= "HiTechnc",
		.product_id	= "IRRecv",
		.num_modes	= 2,
		.ms_mode_info	= {
			[0] = {
				.name = "HT-IRRECV",
				.units = "pct",
				.data_type = MSENSOR_DATA_S8,
			},
			[1] = {
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
		.vendor_id	= "HITECHNC",
		.product_id	= "Accel.",
		.num_modes	= 2,
		.ms_mode_info	= {
			[0] = {
				.name = "HT-ACCL",
			},
			[1] = {
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
		.vendor_id	= "HiTechnc",
		.product_id	= "IRLink",
		.num_modes	= 1,
		.ms_mode_info	= {
			[0] = {
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
		.vendor_id	= "HiTechnc",
		.product_id	= "SuperPro",
		.num_modes	= 5,
		.ms_mode_info	= {
			[0] = {
				.name = "HT-SPRO-AIN",
				.data_sets = 4,
				.data_type = MSENSOR_DATA_U16,
			},
			[1] = {
				.name = "HT-SPRO-DIN",
			},
			[2] = {
				.name = "HT-SPRO-DOT",
			},
			[3] = {
				.name = "HT-SPRO-DCT",
			},
			[4] = {
				.name = "HT-SPRO-STB",
			},
			[5] = {
				.name = "HT-SPRO-LED",
			},
			[6] = {
				.name = "HT-SPRO-AO0",
				.data_sets = 5,
			},
			[7] = {
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
				.name = "HT-SMUX-RUN",
				.data_sets = 30,
			},
			[1]= {
				.name = "HT-SMUX-DETECT",
				.data_sets = 30,
			},
			[2]= {
				.name = "HT-SMUX-HALT",
				.data_sets = 30,
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
	[MS_LIGHT_SENSOR_ARRAY] = {
		.vendor_id	= "mndsnsrs",
		.product_id	= "LSArray",
		.num_modes	= 2,
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
	},
};
EXPORT_SYMBOL_GPL(nxt_i2c_sensor_defs);
