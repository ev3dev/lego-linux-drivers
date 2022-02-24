/*
 * HiTechnic NXT sensor multiplexer device driver
 *
 * Copyright (C) 2014-2015 David Lechner <david@lechnology.com>
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
 * The sensor multiplexer provides 4 sensor ports via one input port. A port
 * device is registered for each port. These can be found in ``/sys/class/lego-port/``.
 *
 * Any type of NXT/Analog sensor can be used with this multiplexer. Most NXT/I2C
 * sensors can be uses as well, but they can only be operated as read-only.
 *
 * This device cannot detect when motors are attached or removed. However, there
 * is a command that can be used to attempt to detect sensors after they have
 * been attached. This only works for certain LEGO and HiTechnic NXT sensors.
 */

#include<linux/i2c.h>
#include <linux/slab.h>

#include <lego.h>
#include <lego_port_class.h>
#include <lego_sensor_class.h>

#include "ht_nxt_smux.h"
#include "nxt_analog_sensor.h"

enum ht_nxt_smux_port_mode {
	HT_NXT_SMUX_PORT_MODE_ANALOG,
	HT_NXT_SMUX_PORT_MODE_I2C,
	NUM_HT_NXT_SMUX_PORT_MODES
};

/*
 * Documentation is automatically generated from this struct, so formatting is
 * very important. Make sure any new modes have the same syntax. The comments
 * are also parsed to provide more information for the documentation. The
 * parser can be found in the Documentation/json/ directory.
 */

static const struct lego_port_mode_info ht_nxt_smux_port_mode_info[] = {
	/**
	 * .. [#ht-nxt-smux-prefix] The full ``address`` is in the format:
	 *    ``<parent-address>:]mux<n>``.
	 *
	 *    For example, if we are looking at port 1 of this mux plugged into
	 *    input port 2 on the EV3, the address will be ``in2:i2c08:mux1``.
	 *
	 * @description: HiTechnic NXT Sensor Multiplexer Input Port
	 * @connection_types: NXT/I2C, NXT/Analog
	 * @prefix: mux
	 * @prefix_footnote: [#ht-nxt-smux-prefix]_
	 */
	[HT_NXT_SMUX_PORT_MODE_ANALOG] = {
		/**
		 * .. [#ht-nxt-smux-analog-mode] The generic ``nxt-analog``
		 *    driver will be loaded when this mode is set. You must
		 *    manually specify the correct driver for your sensor using
		 *    ``set_device`` if you want to use another driver. Any
		 *    driver with a connection type of NXT/Analog is allowed.
		 *
		 * @description: NXT/Analog sensor
		 * @name_footnote: [#ht-nxt-smux-analog-mode]_
		 */
		.name	= "analog",
	},
	[HT_NXT_SMUX_PORT_MODE_I2C] = {
		/**
		 * .. [#ht-nxt-smux-i2c-mode] If one of the supported sensors
		 *    was detected by invoking the ``DETECT`` command on the
		 *    :ref:`ht-nxt-smux` associated with this port, then the
		 *    appropriate driver will be automatically loaded. Otherwise,
		 *    you can use ``set_device`` to load the correct driver for
		 *    your sensor. Most drivers with a connection type of NXT/I2C
		 *    are allowed.
		 *
		 * @description: NXT/I2C sensor
		 * @name_footnote: [#ht-nxt-smux-i2c-mode]_
		 */
		.name	= "i2c",
	},
};

static const u8 ht_nxt_smux_config_reg[] = {
	[HT_NXT_SMUX_CH1] = HT_NXT_SMUX_CH1_CONFIG_REG,
	[HT_NXT_SMUX_CH2] = HT_NXT_SMUX_CH2_CONFIG_REG,
	[HT_NXT_SMUX_CH3] = HT_NXT_SMUX_CH3_CONFIG_REG,
	[HT_NXT_SMUX_CH4] = HT_NXT_SMUX_CH4_CONFIG_REG,
};

static const u8 ht_nxt_smux_analog_data_reg[] = {
	[HT_NXT_SMUX_CH1] = HT_NXT_SMUX_CH1_ANALOG_DATA_REG,
	[HT_NXT_SMUX_CH2] = HT_NXT_SMUX_CH2_ANALOG_DATA_REG,
	[HT_NXT_SMUX_CH3] = HT_NXT_SMUX_CH3_ANALOG_DATA_REG,
	[HT_NXT_SMUX_CH4] = HT_NXT_SMUX_CH4_ANALOG_DATA_REG,
};

static u8 ht_nxt_smux_i2c_data_reg[] = {
	[HT_NXT_SMUX_CH1] = HT_NXT_SMUX_CH1_I2C_DATA_REG,
	[HT_NXT_SMUX_CH2] = HT_NXT_SMUX_CH2_I2C_DATA_REG,
	[HT_NXT_SMUX_CH3] = HT_NXT_SMUX_CH3_I2C_DATA_REG,
	[HT_NXT_SMUX_CH4] = HT_NXT_SMUX_CH4_I2C_DATA_REG,
};

static const char * const ht_nxt_smux_supported_i2c_sensor_names[] = {
	[HT_NXT_SMUX_SENSOR_ANALOG]		= NULL,
	[HT_NXT_SMUX_SENSOR_LEGO_ULTRASONIC]	= LEGO_NXT_ULTRASONIC_SENSOR_NAME,
	[HT_NXT_SMUX_SENSOR_HT_COMPASS]		= HT_NXT_COMPASS_SENSOR_NAME,
	[HT_NXT_SMUX_SENSOR_HT_COLOR]		= HT_NXT_COLOR_SENSOR_NAME,
	[HT_NXT_SMUX_SENSOR_HT_ACCEL]		= HT_NXT_ACCELERATION_TILT_SENSOR_NAME,
	[HT_NXT_SMUX_SENSOR_HT_IR_SEEKER]	= HT_NXT_IR_SEEKER_SENSOR_NAME,
	[HT_NXT_SMUX_SENSOR_HT_PROTO_BOARD]	= HT_NXT_SUPER_PRO_SENSOR_NAME,
	[HT_NXT_SMUX_SENSOR_HT_COLOR_V2]	= HT_NXT_COLOR_SENSOR_V2_NAME,
	[HT_NXT_SMUX_SENSOR_RESERVED]		= NULL,
	[HT_NXT_SMUX_SENSOR_IR_SEEKER_V2]	= HT_NXT_IR_SEEKER_SENSOR_V2_NAME,
};

struct device_type ht_nxt_smux_port_type = {
	.name	= "ht-nxt-smux-port",
};
EXPORT_SYMBOL_GPL(ht_nxt_smux_port_type);

static const struct device_type ht_nxt_smux_analog_sensor_device_type = {
	.name	= "nxt-analog-sensor",
};

static const struct device_type ht_nxt_smux_i2c_sensor_device_type = {
	.name	= "ht-nxt-smux-i2c-sensor",
};

struct ht_nxt_smux_port_data {
	struct lego_port_device port;
	unsigned channel;
	struct nxt_i2c_sensor_data *i2c;
	struct lego_device *sensor;
};

static inline bool ht_nxt_smux_is_running(struct nxt_i2c_sensor_data *data)
{
	return data->sensor.mode_info[data->sensor.mode].raw_data[1]
		& HT_NXT_SMUX_STATUS_BUSY;
}

static int ht_nxt_smux_port_set_config_bit(struct ht_nxt_smux_port_data *data,
					   u8 bit, bool value)
{
	int offset = ht_nxt_smux_config_reg[data->channel];
	int old, new;

	old = i2c_smbus_read_byte_data(data->i2c->client, offset);
	if (old < 0)
		return old;

	if (value)
		new = old | bit;
	else
		new = old & ~bit;

	if (new == old)
		return 0;

	return i2c_smbus_write_byte_data(data->i2c->client, offset, new);
}

static int ht_nxt_smux_port_set_pin1_gpio(void *context,
					  enum lego_port_gpio_state state)
{
	struct ht_nxt_smux_port_data * data = context;

	return ht_nxt_smux_port_set_config_bit(data, HT_NXT_SMUX_CONFIG_9V_EN,
					state == LEGO_PORT_GPIO_HIGH);
}

static struct lego_port_nxt_i2c_ops ht_nxt_smux_port_nxt_i2c_ops = {
	.set_pin1_gpio = ht_nxt_smux_port_set_pin1_gpio,
};

static int ht_nxt_smux_port_set_pin5_gpio(void *context,
					  enum lego_port_gpio_state state)
{
	struct ht_nxt_smux_port_data * data = context;

	return ht_nxt_smux_port_set_config_bit(data, HT_NXT_SMUX_CONFIG_DIG0,
					state == LEGO_PORT_GPIO_HIGH);
}

static struct lego_port_nxt_analog_ops ht_nxt_smux_port_nxt_analog_ops = {
	.set_pin5_gpio = ht_nxt_smux_port_set_pin5_gpio,
};

void ht_nxt_smux_port_set_i2c_addr(struct lego_port_device *port, u8 addr,
				   bool slow)
{
	struct ht_nxt_smux_port_data *data =
		container_of(port, struct ht_nxt_smux_port_data, port);
	int offset = ht_nxt_smux_config_reg[data->channel];

	i2c_smbus_write_byte_data(data->i2c->client,
		offset + HT_NXT_SMUX_CFG_I2C_ADDR,
		addr << 1);
	ht_nxt_smux_port_set_config_bit(data, HT_NXT_SMUX_CONFIG_SLOW, slow);
}
EXPORT_SYMBOL_GPL(ht_nxt_smux_port_set_i2c_addr);

void ht_nxt_smux_port_set_i2c_data_reg(struct lego_port_device *port, u8 reg,
				       u8 size)
{
	struct ht_nxt_smux_port_data *data =
		container_of(port, struct ht_nxt_smux_port_data, port);
	int offset = ht_nxt_smux_config_reg[data->channel];

	i2c_smbus_write_byte_data(data->i2c->client,
		offset + HT_NXT_SMUX_CFG_I2C_REG,
		reg);
	/* The HT sensor mux can only read up to 8 bytes at a time */
	if (size > 8)
		size = 8;
	i2c_smbus_write_byte_data(data->i2c->client,
		offset + HT_NXT_SMUX_CFG_I2C_CNT, size);
}
EXPORT_SYMBOL_GPL(ht_nxt_smux_port_set_i2c_data_reg);

static int ht_nxt_smux_register_analog_sensor(struct ht_nxt_smux_port_data *data,
					      const char *name)
{
	struct lego_device *new_sensor;

	new_sensor = lego_device_register(name,
		&ht_nxt_smux_analog_sensor_device_type, &data->port, NULL, 0);
	if (IS_ERR(new_sensor))
		return PTR_ERR(new_sensor);

	data->sensor = new_sensor;

	return 0;
}

static int ht_nxt_smux_register_i2c_sensor(struct ht_nxt_smux_port_data *data,
					   const char *name, u8 address)
{
	struct ht_nxt_smux_i2c_sensor_platform_data pdata;
	struct lego_device *new_sensor;

	pdata.address = address;
	new_sensor = lego_device_register(name,
		&ht_nxt_smux_i2c_sensor_device_type, &data->port,
		&pdata, sizeof(pdata));
	if (IS_ERR(new_sensor))
		return PTR_ERR(new_sensor);

	data->sensor = new_sensor;

	return 0;
}

static void ht_nxt_smux_unregister_sensor(struct ht_nxt_smux_port_data *data)
{
	if (data->sensor) {
		lego_device_unregister(data->sensor);
		data->sensor = NULL;
	}
}

static void ht_nxt_smux_port_detect_sensor(struct ht_nxt_smux_port_data *data)
{
	int config_reg = ht_nxt_smux_config_reg[data->channel];
	int ret;
	const char *name = NULL;

	ht_nxt_smux_unregister_sensor(data);

	if (data->port.mode == HT_NXT_SMUX_PORT_MODE_ANALOG) {
		ret = ht_nxt_smux_register_analog_sensor(data, GENERIC_NXT_ANALOG_SENSOR_NAME);
		if (ret < 0) {
			dev_err(&data->port.dev,
				"Failed to register analog sensor (%d)\n", ret);
			return;
		}
	} else {
		ret = i2c_smbus_read_byte_data(data->i2c->client,
			config_reg + HT_NXT_SMUX_CFG_TYPE);
		if (ret < 0) {
			dev_err(&data->port.dev,
				"Failed to read I2C sensor type (%d)\n", ret);
			return;
		}

		if (ret < NUM_HT_NXT_SMUX_SENSOR_TYPE)
			name = ht_nxt_smux_supported_i2c_sensor_names[ret];

		if (!name) {
			dev_err(&data->port.dev, "Unknown sensor type\n");
			return;
		}

		if (ret == HT_NXT_SMUX_SENSOR_LEGO_ULTRASONIC) {
			/*
			 * The automatic detection seems to give this sensor the
			 * wrong address (8). It has only one possible address,
			 * so assign it manually.
			 */
			ret = 1;
		} else {
			ret = i2c_smbus_read_byte_data(data->i2c->client,
				config_reg + HT_NXT_SMUX_CFG_I2C_ADDR);
			if (ret < 0) {
				dev_err(&data->port.dev,
					"Failed to read I2C address (%d)\n", ret);
				return;
			}
			/* convert to Linux-style 7-bit I2C address */
			ret >>= 1;
		}

		ret = ht_nxt_smux_register_i2c_sensor(data, name, ret);
		if (ret < 0) {
			dev_err(&data->port.dev,
				"Failed to register I2C sensor (%d)\n", ret);
			return;
		}
	}
}

static int ht_nxt_smux_port_set_mode(void *context, u8 mode)
{
	struct ht_nxt_smux_port_data *data = context;
	int ret;

	ret = ht_nxt_smux_port_set_config_bit(data, HT_NXT_SMUX_CONFIG_I2C,
		mode == HT_NXT_SMUX_PORT_MODE_I2C);
	if (ret < 0)
		return ret;

	data->port.mode = mode;

	if (ht_nxt_smux_is_running(data->i2c))
		ht_nxt_smux_port_detect_sensor(data);

	return 0;
}

static int ht_nxt_smux_port_set_device(void *context, const char *name)
{
	struct ht_nxt_smux_port_data *data = context;
	struct lego_device *new_sensor = ERR_PTR(-EOPNOTSUPP);
	char *blank, end;
	char i2c_name[I2C_NAME_SIZE] = { 0 };
	struct ht_nxt_smux_i2c_sensor_platform_data pdata;
	int ret, hex;

	if (!ht_nxt_smux_is_running(data->i2c))
		return -EPERM;

	ht_nxt_smux_unregister_sensor(data);

	if (data->port.mode == HT_NXT_SMUX_PORT_MODE_ANALOG)
		new_sensor = lego_device_register(name,
			&ht_nxt_smux_analog_sensor_device_type, &data->port,
			NULL, 0);
	else if (data->port.mode == HT_NXT_SMUX_PORT_MODE_I2C) {
		/* credit: parameter parsing code copied from i2c_core.c */
		blank = strchr(name, ' ');
		if (!blank) {
			dev_err(&data->port.dev, "%s: Missing parameters\n",
				"set_device");
			return -EINVAL;
		}
		if (blank - name >= I2C_NAME_SIZE) {
			dev_err(&data->port.dev, "%s: Invalid device name\n",
				"set_device");
			return -EINVAL;
		}
		memcpy(i2c_name, name, blank - name);

		/* Parse remaining parameters, reject extra parameters */
		ret = sscanf(++blank, "%hhu%c%x", &pdata.address, &end, &hex);
		if (ret < 1) {
			dev_err(&data->port.dev, "%s: Can't parse I2C address\n",
				"set_device");
			return -EINVAL;
		}
		if (pdata.address == 0 && end == 'x')
			pdata.address = hex;

		new_sensor = lego_device_register(i2c_name,
			&ht_nxt_smux_i2c_sensor_device_type, &data->port,
			&pdata, sizeof(pdata));
	}
	if (IS_ERR(new_sensor)) {
		dev_err(&data->port.dev, "Failed to set sensor %s. %ld", name,
			PTR_ERR(new_sensor));
		return PTR_ERR(new_sensor);
	}
	data->sensor = new_sensor;

	return 0;
}

int ht_nxt_smux_send_cmd_pre_cb(struct nxt_i2c_sensor_data *data, u8 command)
{
	int ret;

	ret = i2c_smbus_read_byte_data(data->client, HT_NXT_SMUX_STATUS_REG);
	if (ret < 0)
		return ret;

	/* can't switch to detect mode from run mode */
	if (command == HT_NXT_SMUX_COMMAND_DETECT
					&& !(ret & HT_NXT_SMUX_STATUS_HALT))
		return -EPERM;

	/* can't change modes while detect is in progress */
	ret = i2c_smbus_read_byte_data(data->client, HT_NXT_SMUX_COMMAND_REG);
	if (ret < 0)
		return ret;
	if (ret == HT_NXT_SMUX_COMMAND_DETECT)
		return -EBUSY;

	return 0;
}

void ht_nxt_smux_send_cmd_post_cb(struct nxt_i2c_sensor_data *data, u8 command)
{
	struct ht_nxt_smux_port_data *ports = data->callback_data;
	int i, status;

	if (command == HT_NXT_SMUX_COMMAND_RUN) {
		for (i = 0; i < NUM_HT_NXT_SMUX_CH; i++) {
			/* Ignore run command if we already have sensor */
			if (ports[i].sensor)
				continue;
			status = i2c_smbus_read_byte_data(data->client,
				ht_nxt_smux_config_reg[i]);
			if (status < 0) {
				dev_err(&data->client->dev,
					"Failed to read port status (%d)\n",
					status);
				continue;
			}
			ports[i].port.mode = (status & HT_NXT_SMUX_CONFIG_I2C)
				? HT_NXT_SMUX_PORT_MODE_I2C
				: HT_NXT_SMUX_PORT_MODE_ANALOG;
			ht_nxt_smux_port_detect_sensor(&ports[i]);
		}
	} else if (command == HT_NXT_SMUX_COMMAND_HALT) {
		for (i = 0; i < NUM_HT_NXT_SMUX_CH; i++)
			ht_nxt_smux_unregister_sensor(&ports[i]);
	}
}

void ht_nxt_smux_poll_cb(struct nxt_i2c_sensor_data *data)
{
	struct lego_sensor_mode_info *mode_info =
		&data->sensor.mode_info[data->sensor.mode];
	const struct nxt_i2c_sensor_mode_info *i2c_info =
		&data->info->i2c_mode_info[data->sensor.mode];
	struct ht_nxt_smux_port_data *ports = data->callback_data;
	int i;
	u8 raw_analog[2];
	u8 old_data[32];
	int raw_data_size;
	int check_size;

	raw_data_size = lego_sensor_get_raw_data_size(mode_info);
	check_size = raw_data_size <= 32 ? raw_data_size : 32;
	memcpy(old_data, mode_info->raw_data, raw_data_size);

	i2c_smbus_read_i2c_block_data(data->client, i2c_info->read_data_reg,
		raw_data_size, mode_info->raw_data);

	if (memcmp(old_data, mode_info->raw_data, check_size) != 0)
		mode_info->last_changed_time = ktime_get();

	for (i = 0; i < NUM_HT_NXT_SMUX_CH; i++) {
		ktime_t *last_change_ptr = ports[i].port.last_changed_time;
		u8 *raw_data = ports[i].port.raw_data;
		raw_data_size = ports[i].port.raw_data_size;

		if (!raw_data)
			continue;

		if (last_change_ptr)
			check_size = raw_data_size <= 32 ? raw_data_size : 32;
		else
			check_size = 0;

		if (check_size)
			memcpy(old_data, raw_data, check_size);

		if (ports[i].port.mode == HT_NXT_SMUX_PORT_MODE_ANALOG) {
			i2c_smbus_read_i2c_block_data(data->client,
				ht_nxt_smux_analog_data_reg[i], 2, raw_analog);
			/* values are 0-1023, so this scales to 0-5000 mV */
			*(s32 *)raw_data = ((raw_analog[0] << 2)
				+ (raw_analog[1] & 3)) * 5005 >> 10;
		} else if (ports[i].port.mode == HT_NXT_SMUX_PORT_MODE_I2C) {
			i2c_smbus_read_i2c_block_data(data->client,
				ht_nxt_smux_i2c_data_reg[i],
				raw_data_size, raw_data);
		}

		if (check_size) {
			if (memcmp(old_data, raw_data, check_size) != 0)
				*last_change_ptr = ktime_get();
		}

		lego_port_call_raw_data_func(&ports[i].port);
	}
}

int ht_nxt_smux_probe_cb(struct nxt_i2c_sensor_data *data)
{
	struct ht_nxt_smux_port_data *ports;
	int i, ret;

	ports = kzalloc(sizeof(*ports) * NUM_HT_NXT_SMUX_CH, GFP_KERNEL);
	if (!ports)
		return -ENOMEM;

	data->callback_data = ports;
	for (i = 0; i < NUM_HT_NXT_SMUX_CH; i++) {
		ports[i].port.name = ht_nxt_smux_port_type.name;
		snprintf(ports[i].port.address, LEGO_NAME_SIZE,
			 "%s:mux%d", data->address, i + 1);
		ports[i].port.num_modes = NUM_HT_NXT_SMUX_PORT_MODES;
		ports[i].port.supported_modes = LEGO_PORT_ALL_MODES;
		ports[i].port.mode_info = ht_nxt_smux_port_mode_info;
		ports[i].port.set_mode = ht_nxt_smux_port_set_mode;
		ports[i].port.set_device = ht_nxt_smux_port_set_device;
		ports[i].port.context = &ports[i];
		ports[i].port.nxt_analog_ops = &ht_nxt_smux_port_nxt_analog_ops;
		ports[i].port.nxt_i2c_ops = &ht_nxt_smux_port_nxt_i2c_ops;
		ports[i].i2c = data;
		ports[i].channel = i;

		ret = lego_port_register(&ports[i].port, &ht_nxt_smux_port_type,
					 &data->client->dev);
		if (ret) {
			dev_err(&data->client->dev,
				"Failed to register ht-nxt-smux input port. (%d)\n",
				ret);
			for (i--; i >= 0; i--)
				lego_port_unregister(&ports[i].port);
			data->callback_data = NULL;
			kfree(ports);
			return ret;
		}
		ret = i2c_smbus_read_byte_data(ports[i].i2c->client,
						ht_nxt_smux_config_reg[i]);
		if (ret < 0)
			dev_err(&ports[i].port.dev,
				"Failed to read initial mode. (%d)", ret);
		else
			ports[i].port.mode = (ret & HT_NXT_SMUX_CONFIG_I2C)
					? HT_NXT_SMUX_PORT_MODE_I2C
					: HT_NXT_SMUX_PORT_MODE_ANALOG;
	}

	ht_nxt_smux_poll_cb(data);
	if (ht_nxt_smux_is_running(data)) {
		for (i = 0; i < NUM_HT_NXT_SMUX_CH; i++) {
			ht_nxt_smux_port_detect_sensor(&ports[i]);
		}
	}

	return 0;
}

void ht_nxt_smux_remove_cb(struct nxt_i2c_sensor_data *data)
{
	struct ht_nxt_smux_port_data *ports = data->callback_data;
	int i;

	if (ports) {
		for (i = 0; i < NUM_HT_NXT_SMUX_CH; i++) {
			ht_nxt_smux_unregister_sensor(&ports[i]);
			lego_port_unregister(&ports[i].port);
		}
		data->callback_data = NULL;
		kfree(ports);
	}
}
