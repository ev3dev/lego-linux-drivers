/*
 * mindsensors.com EV3 Sensor Multiplexer driver for LEGO MINDSTORMS EV3
 *
 * Copyright (C) 2014 David Lechner <david@lechnology.com>
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
 * Use kramdown (markdown) format. Use a '.' as a placeholder when blank lines
 * or leading whitespace is important for the markdown syntax.
 */

#include <linux/err.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/legoev3/mport_class.h>

#include "ms_ev3_smux.h"

enum ms_ev3_smux_mode {
	MS_EV3_SMUX_MODE_UART,
	MS_EV3_SMUX_MODE_ANALOG,
	NUM_MS_EV3_SMUX_MODES
};

static const struct mport_mode_info ms_ev3_smux_mode_defs[] = {
	[MS_EV3_SMUX_MODE_UART] = {
		.name	= "uart",
	},
	[MS_EV3_SMUX_MODE_ANALOG] = {
		.name	= "analog",
	},
};

/**
 * struct ms_ev3_smux_data - Driver data for an input port on the mux
 * @mport: The mport class device for this mux port.
 * @analog_cb: The callback to sensor driver to notify it that new data is available.
 * @analog_cb_context: Argument for analog_cb.
 * @mode: The current mode of the port.
 */
struct ms_ev3_smux_data {
	struct mport_device mport;
	legoev3_analog_cb_func_t analog_cb;
	void *analog_cb_context;
};

static int ms_ev3_smux_set_mode(void *context, u8 mode)
{
	struct nxt_i2c_sensor_data *data = context;

	return i2c_smbus_write_byte_data(data->client, MS_EV3_SMUX_MODE_REG,
					 (mode == MS_EV3_SMUX_MODE_UART) ?
					 0 : MS_EV3_SMUX_ANALOG_MODE_DATA);
}

void ms_ev3_smux_poll_cb(struct nxt_i2c_sensor_data *data)
{
	struct ms_ev3_smux_data *smux = data->info.callback_data;
	u8 *raw_data = data->ms.mode_info[data->ms.mode].raw_data;

	/* TODO: get data size from somewhere */
	i2c_smbus_read_i2c_block_data(data->client, MS_EV3_SMUX_DATA_REG,
		4, raw_data);
}

int ms_ev3_smux_probe_cb(struct nxt_i2c_sensor_data *data)
{
	struct ms_ev3_smux_data *smux;
	int ret;

	smux = kzalloc(sizeof(struct ms_ev3_smux_data), GFP_KERNEL);
	if (!smux)
		return -ENOMEM;

	/*
	 * Expects sensor to return the string "CH1" (or 2/3) at 0x18, so we
	 * read the 3rd byte and convert ascii char to an integer.
	*/
	ret = i2c_smbus_read_byte_data(data->client, 0x1A);
	if (ret < 0) {
		dev_err(&data->client->dev,
			"Could not read channel from mux (%d).\n", ret);
		goto err_get_channel;
	}
	ret -= '0';
	snprintf(smux->mport.port_name, MPORT_NAME_SIZE, "%s:mux%d",
		 dev_name(&data->in_port->dev), ret);

	ret = i2c_smbus_read_byte_data(data->client, MS_EV3_SMUX_MODE_REG);
	if (ret < 0) {
		dev_err(&data->client->dev,
			"Could not read mode from mux (%d).\n", ret);
		goto err_get_mode;
	}
	smux->mport.mode = (ret == MS_EV3_SMUX_ANALOG_MODE_DATA) ?
		MS_EV3_SMUX_MODE_ANALOG : MS_EV3_SMUX_MODE_UART;

	smux->mport.num_modes = NUM_MS_EV3_SMUX_MODES;
	smux->mport.mode_info = ms_ev3_smux_mode_defs;
	smux->mport.set_mode = ms_ev3_smux_set_mode;
	smux->mport.context = data;

	ret = register_mport(&smux->mport, &data->client->dev);
	if (ret < 0) {
		dev_err(&data->client->dev, "Failed to register mport. (%d)",
			ret);
		goto err_register_mport;
	}

	data->info.callback_data = smux;

	return 0;

err_register_mport:
err_get_mode:
err_get_channel:
	kfree(smux);
	return ret;
}

void ms_ev3_smux_remove_cb(struct nxt_i2c_sensor_data *data)
{
	struct ms_ev3_smux_data *smux = data->info.callback_data;

	if (smux) {
		unregister_mport(&smux->mport);
		data->info.callback_data = NULL;
		kfree(smux);
	}
}
