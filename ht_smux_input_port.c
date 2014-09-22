/*
 * HiTechnic NXT sensor multiplexer device driver for LEGO Mindstorms EV3
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

/**
 * DOC: website
 *
 * HiTechnic NXT Sensor Multiplexer input port driver
 *
 * A `ht-smux-input-port` device is loaded for each port of the HiTechnic NXT
 * Sensor Multiplexer by the [ht-nxt-smux] driver. The devices are similar to
 * the [EV3 input port] devices except that they only support two host types,
 * [nxt-analog-host] and [ht-smux-i2c-host]. This means only NXT sensors
 * (Analog/NXT and I2C/NXT connection types) will work with the sensor mux.
 * The new EV3 sensors (Analog/EV3 and UART/EV3 connection types) won't work.
 * Additionally, I2C sensors connected to the sensor mux cannot be written to.
 * They operate in a read-only mode, so some features of certain I2c sensors
 * may not be usable via the sensor mux.
 * .
 * ### sysfs attributes
 * .
 * The HiTechnic NXT Sensor Multiplexer I2C input port devices can be found at
 * `/sys/bus/legoev3/in<N>:mux<M>` where `<N>` is the number of the port on the
 * EV3 (1 to 4) and `<M>` is the number of the port on the sensor mux (1 to 4).
 * .
 * `modes` (read-only)
 * : Returns a space separated list of the possible modes, namely `analog` and
 *   `i2c`.
 * .
 * `mode` (read/write)
 * : Reading returns the currently selected mode. Writing sets the mode. When
 *   the mode is changed, the host ([nxt-analog-host] or [ht-smux-i2c-host])
 *   and any devices form the previous mode are removed and a new host is
 *   loaded. The [nxt-analog-host] will load the generic [nxt-analog] driver.
 *   The [ht-smux-i2c-host] does not automatically load any sensor drivers.
 * .
 * [ht-nxt-smux]: ../hitechnic-nxt-sensor-multiplexer
 * [EV3 input port]: ../ev3-input-port
 * [nxt-analog-host]: ../nxt-analog-host
 * [ht-smux-i2c-host]: ../ht-smux-i2c-host
 * [nxt-analog]: ../generic-nxt-analog-sensor
 */

#include <linux/err.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/legoev3/legoev3_ports.h>

#include <mach/mux.h>

#include "ht_smux.h"

static const u8 ht_smux_config_offset[] = {
	[HT_SMUX_CH1] = HT_SMUX_CH1_CONFIG_REG - HT_SMUX_COMMAND_REG,
	[HT_SMUX_CH2] = HT_SMUX_CH2_CONFIG_REG - HT_SMUX_COMMAND_REG,
	[HT_SMUX_CH3] = HT_SMUX_CH3_CONFIG_REG - HT_SMUX_COMMAND_REG,
	[HT_SMUX_CH4] = HT_SMUX_CH4_CONFIG_REG - HT_SMUX_COMMAND_REG,
};

static const u8 ht_smux_analog_offset[] = {
	[HT_SMUX_CH1] = HT_SMUX_CH1_ANALOG_REG - HT_SMUX_COMMAND_REG,
	[HT_SMUX_CH2] = HT_SMUX_CH2_ANALOG_REG - HT_SMUX_COMMAND_REG,
	[HT_SMUX_CH3] = HT_SMUX_CH3_ANALOG_REG - HT_SMUX_COMMAND_REG,
	[HT_SMUX_CH4] = HT_SMUX_CH4_ANALOG_REG - HT_SMUX_COMMAND_REG,
};

static u8 ht_smux_i2c_data_offset[] = {
	[HT_SMUX_CH1] = HT_SMUX_CH1_I2C_DATA_REG - HT_SMUX_COMMAND_REG,
	[HT_SMUX_CH2] = HT_SMUX_CH2_I2C_DATA_REG - HT_SMUX_COMMAND_REG,
	[HT_SMUX_CH3] = HT_SMUX_CH3_I2C_DATA_REG - HT_SMUX_COMMAND_REG,
	[HT_SMUX_CH4] = HT_SMUX_CH4_I2C_DATA_REG - HT_SMUX_COMMAND_REG,
};

enum ht_smux_input_port_mode {
	HT_SMUX_MODE_ANALOG,
	HT_SMUX_MODE_I2C,
	NUM_HT_SMUX_MODES
};

static const char *ht_smux_input_port_mode_names[] = {
	[HT_SMUX_MODE_ANALOG] = "analog",
	[HT_SMUX_MODE_I2C] = "i2c",
};

static const char* ht_smux_input_port_sensor_names[] = {
	[HT_SMUX_SENSOR_NXT_ANALOG]	= "nxt-analog",
	[HT_SMUX_SENSOR_LEGO_ULTRASONIC]= "lego-nxt-ultrasonic",
	[HT_SMUX_SENSOR_HT_COMPASS]	= "ht-nxt-compass",
	[HT_SMUX_SENSOR_HT_COLOR]	= "ht-nxt-color",
	[HT_SMUX_SENSOR_HT_ACCEL]	= "ht-nxt-accel",
	[HT_SMUX_SENSOR_HT_IR_SEEKER]	= "ht-nxt-ir-seeker",
	[HT_SMUX_SENSOR_HT_PROTO_BOARD]	= "ht-super-pro",
	[HT_SMUX_SENSOR_HT_COLOR_V2]	= "ht-nxt-color-v2",
	[HT_SMUX_SENSOR_RESERVED]	= NULL,
	[HT_SMUX_SENSOR_IR_SEEKER_V2]	= "ht-ir-seeker-v2",
};

const struct attribute_group *ht_smux_port_device_type_attr_groups[] = {
	&legoev3_port_device_type_attr_grp,
	NULL
};

struct device_type ht_smux_input_port_host_device_types [] = {
	[HT_SMUX_MODE_ANALOG] = {
		.name	= "nxt-analog-host",
		.groups	= ht_smux_port_device_type_attr_groups,
		.uevent = legoev3_port_device_uevent,
	},
	[HT_SMUX_MODE_I2C] = {
		.name	= "ht-smux-i2c-host",
		.groups	= ht_smux_port_device_type_attr_groups,
		.uevent = legoev3_port_device_uevent,
	},
};

/**
 * struct ht_smux_input_port_data - Driver data for an input port on the EV3 brick
 * @pdata: Platform data.
 * @in_port: The device that is bound to this driver.
 * @host: Either a nxt_analog_host or ht_smux_i2c_host.
 * @analog_cb: The callback to sensor driver to notify it that new data is available.
 * @analog_cb_context: Argument for analog_cb.
 */
struct ht_smux_input_port_data {
	struct ht_smux_input_port_platform_data pdata;
	struct legoev3_port *in_port;
	struct legoev3_port_device *host;
	legoev3_analog_cb_func_t analog_cb;
	void *analog_cb_context;
};

static int ht_smux_input_port_get_pin1_mv(struct legoev3_port *in_port)
{
	struct ht_smux_input_port_data *data = dev_get_drvdata(&in_port->dev);
	int offset = ht_smux_analog_offset[data->pdata.channel];

	/* values are 0-1023, so this scales to 0-5000 mV */
	return ((data->pdata.sensor_data[offset] << 2)
		+ (data->pdata.sensor_data[offset + 1] & 3)) * 5005 >> 10;
}

static int ht_smux_input_port_get_pin6_mv(struct legoev3_port *in_port)
{
	/* does not have analog on pin 6 */
	return 0;
}

static void ht_smux_input_port_set_config_bit(struct legoev3_port *in_port,
					      u8 bit, bool value)
{
	struct ht_smux_input_port_data *data = dev_get_drvdata(&in_port->dev);
	int offset = ht_smux_config_offset[data->pdata.channel];
	u8 old, new;

	old = data->pdata.sensor_data[offset];
	if (value)
		new = old | bit;
	else
		new = old & ~bit;

	if (new == old)
		return;

	i2c_smbus_write_byte_data(data->pdata.client,
				  HT_SMUX_COMMAND_REG + offset, new);
}

static void ht_smux_input_port_set_pin1_gpio(struct legoev3_port *in_port,
					     enum ev3_input_port_gpio_state state)
{
	ht_smux_input_port_set_config_bit(in_port, HT_SMUX_CONFIG_9V_EN,
					  state == EV3_INPUT_PORT_GPIO_HIGH);
}

static void ht_smux_input_port_set_pin5_gpio(struct legoev3_port *in_port,
					     enum ev3_input_port_gpio_state state)
{
	ht_smux_input_port_set_config_bit(in_port, HT_SMUX_CONFIG_DIG0,
					  state == EV3_INPUT_PORT_GPIO_HIGH);
}

static void ht_smux_input_port_register_analog_cb(struct legoev3_port *in_port,
						  legoev3_analog_cb_func_t function,
						  void *context)
{
	struct ht_smux_input_port_data *data = dev_get_drvdata(&in_port->dev);

	data->analog_cb = function;
	data->analog_cb_context = context;
}

static void ht_smux_input_port_poll_cb(void *context)
{
	struct legoev3_port *in_port = context;
	struct ht_smux_input_port_data *data = dev_get_drvdata(&in_port->dev);

	if (data->analog_cb)
		data->analog_cb(data->analog_cb_context);
}

void ht_smux_input_port_copy_i2c_data(struct legoev3_port *in_port, u8 *dest)
{
	struct ht_smux_input_port_data *data = dev_get_drvdata(&in_port->dev);
	int offset = ht_smux_i2c_data_offset[data->pdata.channel];

	memcpy(dest, data->pdata.sensor_data + offset, 8);
}
EXPORT_SYMBOL_GPL(ht_smux_input_port_copy_i2c_data);

void ht_smux_input_port_set_i2c_addr(struct legoev3_port *in_port, u8 addr,
				     bool slow)
{
	struct ht_smux_input_port_data *data = dev_get_drvdata(&in_port->dev);
	int offset = ht_smux_config_offset[data->pdata.channel];
	u8 old, new;

	if (data->pdata.sensor_data[offset + HT_SMUX_CFG_I2C_ADDR] != addr)
		i2c_smbus_write_byte_data(data->pdata.client,
			HT_SMUX_COMMAND_REG + offset + HT_SMUX_CFG_I2C_ADDR,
			addr);
	old = data->pdata.sensor_data[offset + HT_SMUX_CFG_MODE];
	if (slow)
		new = old | HT_SMUX_CONFIG_SLOW;
	else
		new = old & ~HT_SMUX_CONFIG_SLOW;
	if (old != new)
		i2c_smbus_write_byte_data(data->pdata.client,
			HT_SMUX_COMMAND_REG + offset + HT_SMUX_CFG_MODE,
			new);
}
EXPORT_SYMBOL_GPL(ht_smux_input_port_set_i2c_addr);

void ht_smux_input_port_set_i2c_data_reg(struct legoev3_port *in_port, u8 reg,
					 u8 count)
{
	struct ht_smux_input_port_data *data = dev_get_drvdata(&in_port->dev);
	int offset = ht_smux_config_offset[data->pdata.channel];

	if (data->pdata.sensor_data[offset + HT_SMUX_CFG_I2C_REG] != reg)
		i2c_smbus_write_byte_data(data->pdata.client,
			HT_SMUX_COMMAND_REG + offset + HT_SMUX_CFG_I2C_REG,
			reg);
	if (count > 8)
		count = 8;
	if (data->pdata.sensor_data[offset + HT_SMUX_CFG_I2C_CNT] != count)
		i2c_smbus_write_byte_data(data->pdata.client,
			HT_SMUX_COMMAND_REG + offset + HT_SMUX_CFG_I2C_CNT,
			count);
}
EXPORT_SYMBOL_GPL(ht_smux_input_port_set_i2c_data_reg);

void ht_smux_input_port_register_host(struct ht_smux_input_port_data *data,
				      enum ht_smux_input_port_mode host_type,
				      enum ht_smux_detected_sensor initial_sensor_type,
				      u8 initial_sensor_address)
{
	struct legoev3_port_device *host;
	struct ht_smux_i2c_host_platform_data pdata;

	if (host_type == HT_SMUX_MODE_I2C
			&& initial_sensor_type == HT_SMUX_SENSOR_NXT_ANALOG)
		pdata.inital_sensor = NULL;
	else
		pdata.inital_sensor =
			ht_smux_input_port_sensor_names[initial_sensor_type];

	pdata.address = initial_sensor_address;
	host = legoev3_port_device_register(
		ht_smux_input_port_host_device_types[host_type].name,
		&ht_smux_input_port_host_device_types[host_type],
		&data->in_port->dev, &pdata, sizeof(pdata), data->in_port);
	if (IS_ERR(host)) {
		dev_err(&data->in_port->dev, "Could not register host on port %s.\n",
			dev_name(&data->in_port->dev));
		return;
	}

	data->host = host;

	return;
}

void ht_smux_input_port_unregister_host(struct ht_smux_input_port_data *data)
{
	legoev3_port_device_unregister(data->host);
	data->host = NULL;
}

static ssize_t modes_show(struct device *dev, struct device_attribute *attr,
			  char *buf)
{
	size_t total = 0;
	int i, size;

	for (i = 0; i < NUM_HT_SMUX_MODES; i++) {
		size = sprintf(buf, "%s ", ht_smux_input_port_mode_names[i]);
		total += size;
		buf += size;
	}
	buf--;
	buf[0] = '\n';
	buf[1] = 0;
	total++;

	return total;
}

static ssize_t mode_show(struct device *dev, struct device_attribute *attr,
			 char *buf)
{
	struct ht_smux_input_port_data *data = dev_get_drvdata(dev);
	int ret;

	ret = i2c_smbus_read_byte_data(data->pdata.client,
		HT_SMUX_COMMAND_REG + ht_smux_config_offset[data->pdata.channel]);
	if (ret < 0)
		return ret;

	return sprintf(buf, "%s\n",
		ht_smux_input_port_mode_names[ret & HT_SMUX_CONFIG_I2C]);
}

static ssize_t mode_store(struct device *dev, struct device_attribute *attr,
			  const char *buf, size_t count)
{
	struct ht_smux_input_port_data *data = dev_get_drvdata(dev);
	enum ht_smux_input_port_mode new_mode = -1;
	enum ht_smux_detected_sensor sensor_type;
	int i, config_offset;
	u8 sensor_address;

	for (i = 0; i < NUM_HT_SMUX_MODES; i++) {
		if (sysfs_streq(buf, ht_smux_input_port_mode_names[i])) {
			new_mode = i;
		}
	}
	if (new_mode == -1)
		return -EINVAL;
	config_offset = ht_smux_config_offset[data->pdata.channel];
	if (new_mode == data->pdata.sensor_data[config_offset])
		return count;

	ht_smux_input_port_set_config_bit(data->in_port, HT_SMUX_CONFIG_I2C,
					  new_mode == HT_SMUX_MODE_I2C);

	if (data->host)
		ht_smux_input_port_unregister_host(data);

	sensor_type = data->pdata.sensor_data[config_offset + HT_SMUX_CFG_TYPE];
	sensor_address = data->pdata.sensor_data[config_offset + HT_SMUX_CFG_I2C_ADDR];
	ht_smux_input_port_register_host(data, new_mode, sensor_type, sensor_address);

	return count;
}

static DEVICE_ATTR_RO(modes);
static DEVICE_ATTR_RW(mode);

static struct attribute *ht_smux_input_port_attrs[] = {
	&dev_attr_modes.attr,
	&dev_attr_mode.attr,
	NULL
};

ATTRIBUTE_GROUPS(ht_smux_input_port);

extern void ht_sensor_mux_register_poll_cb(struct i2c_client *client,
					   enum ht_smux_channel channel,
					   legoev3_analog_cb_func_t cb);

static int ht_smux_input_port_probe(struct legoev3_port *port)
{
	struct ht_smux_input_port_data *data;
	struct ht_smux_input_port_platform_data *pdata = port->dev.platform_data;
	enum ht_smux_input_port_mode mode;
	enum ht_smux_detected_sensor sensor_type;
	int err, config_offset;
	u8 sensor_address;

	if (WARN(!pdata, "Platform data is required."))
		return -EINVAL;

	data = kzalloc(sizeof(struct ht_smux_input_port_data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	err = sysfs_create_groups(&port->dev.kobj, ht_smux_input_port_groups);
	if (err)
		goto err_sysfs_create_groups;

	data->in_port = port;
	memcpy(&data->pdata, pdata, sizeof(struct ht_smux_input_port_platform_data));
	port->in_ops.get_pin1_mv = ht_smux_input_port_get_pin1_mv;
	port->in_ops.get_pin6_mv = ht_smux_input_port_get_pin6_mv;
	port->in_ops.set_pin1_gpio = ht_smux_input_port_set_pin1_gpio;
	port->in_ops.set_pin5_gpio = ht_smux_input_port_set_pin5_gpio;
	port->in_ops.register_analog_cb = ht_smux_input_port_register_analog_cb;
	dev_set_drvdata(&port->dev, data);
	ht_sensor_mux_register_poll_cb(data->pdata.client, data->pdata.channel,
				       ht_smux_input_port_poll_cb);

	if (data->pdata.sensor_data[0] == HT_SMUX_COMMAND_RUN) {
		config_offset = ht_smux_config_offset[data->pdata.channel];
		mode = data->pdata.sensor_data[config_offset] & HT_SMUX_CONFIG_I2C;
		sensor_type = data->pdata.sensor_data[config_offset + HT_SMUX_CFG_TYPE];
		sensor_address = data->pdata.sensor_data[config_offset + HT_SMUX_CFG_I2C_ADDR];
		ht_smux_input_port_register_host(data, mode, sensor_type, sensor_address);
	}

	return 0;

err_sysfs_create_groups:
	kfree(data);

	return err;
}

static int ht_smux_input_port_remove(struct legoev3_port *port)
{
	struct ht_smux_input_port_data *data = dev_get_drvdata(&port->dev);


	if (data->host)
		ht_smux_input_port_unregister_host(data);
	ht_sensor_mux_register_poll_cb(data->pdata.client, data->pdata.channel,
				       NULL);
	dev_set_drvdata(&port->dev, NULL);
	port->in_ops.get_pin1_mv = NULL;
	port->in_ops.get_pin6_mv = NULL;
	port->in_ops.set_pin1_gpio = NULL;
	port->in_ops.set_pin5_gpio = NULL;
	port->in_ops.register_analog_cb = NULL;
	sysfs_remove_groups(&port->dev.kobj, ht_smux_input_port_groups);
	kfree(data);

	return 0;
}

struct legoev3_port_driver ht_smux_input_port_driver = {
	.probe		= ht_smux_input_port_probe,
	.remove		= ht_smux_input_port_remove,
	.driver = {
		.name	= "ht-smux-input-port",
		.owner	= THIS_MODULE,
	},
};
legoev3_port_driver(ht_smux_input_port_driver);

MODULE_DESCRIPTION("Input port driver for HiTechnic NXT Sensor Multiplexer");
MODULE_AUTHOR("David Lechner <david@lechnology.com>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("legoev3:ht-smux-input-port");
