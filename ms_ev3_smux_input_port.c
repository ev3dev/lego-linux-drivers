/*
 * mindsensors.com EV3 Sensor Multiplexer device driver for LEGO MINDSTORMS EV3
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
 * mindsensors.com EV3 Sensor Multiplexer Input Port Driver
 *
 * A `ms-ev3-smux-input-port` device is loaded for each port of the
 * mindsensors.com EV3 Sensor Multiplexer by the [ms-ev3-smux] driver. The
 * devices are similar to the [EV3 input port] devices except that they only
 * support two host types, [ms-ev3-smux-analog-host] and [ms-ev3-smux-uart-host].
 * This means only EV3 sensors (Analog/EV3 and UART/EV3 connection types) will
 * work with this sensor mux. NXT compatible sensors (Analog/NXT and I2C/NXT
 * connection types and any others not listed) won't work.
 * .
 * ### sysfs attributes
 * .
 * The mindsensors.com EV3 Sensor Multiplexer input port devices can be found at
 * `/sys/bus/legoev3/in<N>:mux<M>` where `<N>` is the number of the port on the
 * EV3 (1 to 4) and `<M>` is the number of the port on the sensor mux (1 to 3).
 * .
 * `modes` (read-only)
 * : Returns a space separated list of the possible modes, namely `uart` and
 *   `analog`.
 * .
 * `mode` (read/write)
 * : Reading returns the currently selected mode. Writing sets the mode. When
 *   the mode is changed, the host ([ms-ev3-smux-analog-host] or
 *   [ms-ev3-smux-uart-host]) and any devices form the previous mode are
 *   removed and a new host is loaded. Additionally, the host driver will
 *   load the driver for the sensor that is connected if one is detected.
 * .
 * [ms-ev3-smux]: /docs/sensors/mindsensors.com-ev3-sensor-multiplexer
 * [EV3 input port]: ../ev3-input-port
 * [ms-ev3-smux-analog-host]: ../ms-ev3-smux-analog-host
 * [ms-ev3-smux-uart-host]: ../ms-ev3-smux-uart-host
 */

#include <linux/err.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/legoev3/legoev3_ports.h>

#include "ms_ev3_smux.h"

enum ms_ev3_smux_input_port_mode {
	MS_EV3_SMUX_MODE_UART,
	MS_EV3_SMUX_MODE_ANALOG,
	NUM_MS_EV3_SMUX_MODES
};

static const char *ms_ev3_smux_input_port_mode_names[] = {
	[MS_EV3_SMUX_MODE_UART]   = "uart",
	[MS_EV3_SMUX_MODE_ANALOG] = "analog",
};

struct ms_ev3_mode_sensor_pair {
	const char *mode_name;
	const char *sensor_name;
};

/*
 * These are the only sensors supported by this mux. Do not add sensors to
 * the following lists unless there is a firmware update for the mux that
 * warrants it.
 */

static const struct ms_ev3_mode_sensor_pair ms_ev3_smux_uart_sensor_map[] = {
	{ "COL-REFLECT",	"lego-ev3-uart-29"	},
	{ "US-DIST-CM",		"lego-ev3-uart-30"	},
	{ "GYRO-ANG",		"lego-ev3-uart-32"	},
	{ "IR-PROX",		"lego-ev3-uart-33"	},
	{ },
};

static const struct ms_ev3_mode_sensor_pair ms_ev3_smux_analog_sensor_map[] = {
	{ "Touch",		"lego-ev3-touch"	},
	{ },
};

static const struct ms_ev3_mode_sensor_pair *ms_ev3_smux_mode_sensor_map[] = {
	[MS_EV3_SMUX_MODE_UART]   = ms_ev3_smux_uart_sensor_map,
	[MS_EV3_SMUX_MODE_ANALOG] = ms_ev3_smux_analog_sensor_map,
};

const struct attribute_group *ms_ev3_smux_port_device_type_attr_groups[] = {
	&legoev3_port_device_type_attr_grp,
	NULL
};

struct device_type ms_ev3_smux_input_port_host_device_types [] = {
	[MS_EV3_SMUX_MODE_UART] = {
		.name	= "ms-ev3-smux-uart-host",
		.groups	= ms_ev3_smux_port_device_type_attr_groups,
		.uevent = legoev3_port_device_uevent,
	},
	[MS_EV3_SMUX_MODE_ANALOG] = {
		.name	= "ms-ev3-smux-analog-host",
		.groups	= ms_ev3_smux_port_device_type_attr_groups,
		.uevent = legoev3_port_device_uevent,
	},
};

/**
 * struct ms_ev3_smux_input_port_data - Driver data for an input port on the mux
 * @pdata: Platform data.
 * @detect_sensor_work: Used to schedule sensor detection callback.
 * @in_port: The device that is bound to this driver.
 * @host: Either a ms_ev3_smux_analog_host or ms_ev3_smux_uart_host.
 * @analog_cb: The callback to sensor driver to notify it that new data is available.
 * @analog_cb_context: Argument for analog_cb.
 * @mode: The current mode of the port.
 * @removing_driver: Indicates that driver removal is in progress.
 */
struct ms_ev3_smux_input_port_data {
	struct ms_ev3_smux_input_port_platform_data pdata;
	struct delayed_work detect_sensor_work;
	struct legoev3_port *in_port;
	struct legoev3_port_device *host;
	legoev3_analog_cb_func_t analog_cb;
	void *analog_cb_context;
	enum ms_ev3_smux_input_port_mode mode;
	bool removing_driver;
};

static void
ms_ev3_smux_input_port_register_analog_cb(struct legoev3_port *in_port,
					  legoev3_analog_cb_func_t function,
					  void *context)
{
	struct ms_ev3_smux_input_port_data *data = dev_get_drvdata(&in_port->dev);

	data->analog_cb = function;
	data->analog_cb_context = context;
}

static void ms_ev3_smux_input_port_poll_cb(void *context)
{
	struct legoev3_port *in_port = context;
	struct ms_ev3_smux_input_port_data *data = dev_get_drvdata(&in_port->dev);

	if (data->analog_cb)
		data->analog_cb(data->analog_cb_context);
}

void
ms_ev3_smux_input_port_register_host(struct ms_ev3_smux_input_port_data *data,
				     const char *inital_sensor_name)
{
	struct legoev3_port_device *host;
	struct ms_ev3_smux_host_platform_data pdata;

	pdata.inital_sensor = inital_sensor_name;

	host = legoev3_port_device_register(
		ms_ev3_smux_input_port_host_device_types[data->mode].name,
		&ms_ev3_smux_input_port_host_device_types[data->mode],
		&data->in_port->dev, &pdata, sizeof(pdata), data->in_port);
	if (IS_ERR(host)) {
		dev_err(&data->in_port->dev, "Could not register host on port %s.\n",
			dev_name(&data->in_port->dev));
		return;
	}

	data->host = host;

	return;
}

void
ms_ev3_smux_input_port_unregister_host(struct ms_ev3_smux_input_port_data *data)
{
	legoev3_port_device_unregister(data->host);
	data->host = NULL;
}

void
ms_ev3_smux_input_port_detect_sensor(struct work_struct *work)
{
	struct delayed_work *delayed_work = to_delayed_work(work);
	struct ms_ev3_smux_input_port_data *data = container_of(delayed_work,
		struct ms_ev3_smux_input_port_data, detect_sensor_work);
	char mode_name[MS_EV3_SMUX_MODE_NAME_SIZE + 1] = { 0 };
	const char *sensor_name = NULL;
	const struct ms_ev3_mode_sensor_pair *pair;
	int ret;

	if (data->host)
		ms_ev3_smux_input_port_unregister_host(data);

	ret = i2c_smbus_read_i2c_block_data(data->pdata.client,
		MS_EV3_SMUX_MODE_NAME_REG, MS_EV3_SMUX_MODE_NAME_SIZE, mode_name);
	if (ret < 0) {
		dev_err(&data->pdata.client->dev,
			"Failed to read mode_name (%d)\n", ret);
		return;
	}
	/* TODO: add workaround for IR sensor. */
	for (pair = ms_ev3_smux_mode_sensor_map[data->mode]; pair->mode_name; pair++) {
		if (strcmp(mode_name, pair->mode_name))
			continue;
		else {
			sensor_name = pair->sensor_name;
			break;
		}
	}
	if (!sensor_name) {
		if (data->removing_driver)
			return;
		dev_err(&data->pdata.client->dev,
			"Failed to lookup sensor_name for mode_name '%s' "
			"in port mode '%s'\n", mode_name,
			ms_ev3_smux_input_port_mode_names[data->mode]);
		schedule_delayed_work(&data->detect_sensor_work,
				      msecs_to_jiffies(1000));
		return;
	}

	ms_ev3_smux_input_port_register_host(data, sensor_name);
}

static ssize_t modes_show(struct device *dev, struct device_attribute *attr,
			  char *buf)
{
	size_t count = 0;
	int i;

	for (i = 0; i < NUM_MS_EV3_SMUX_MODES; i++) {
		count += sprintf(buf + count, "%s ",
				 ms_ev3_smux_input_port_mode_names[i]);
	}
	buf[count - 1] = '\n';

	return count;
}

static ssize_t mode_show(struct device *dev, struct device_attribute *attr,
			 char *buf)
{
	struct ms_ev3_smux_input_port_data *data = dev_get_drvdata(dev);

	return sprintf(buf, "%s\n",
		ms_ev3_smux_input_port_mode_names[data->mode]);
}

static ssize_t mode_store(struct device *dev, struct device_attribute *attr,
			  const char *buf, size_t count)
{
	struct ms_ev3_smux_input_port_data *data = dev_get_drvdata(dev);
	enum ms_ev3_smux_input_port_mode new_mode = -1;
	int i, ret;

	for (i = 0; i < NUM_MS_EV3_SMUX_MODES; i++) {
		if (sysfs_streq(buf, ms_ev3_smux_input_port_mode_names[i])) {
			new_mode = i;
		}
	}
	if (new_mode == -1)
		return -EINVAL;

	ret = i2c_smbus_write_byte_data(data->pdata.client, MS_EV3_SMUX_MODE_REG,
		new_mode == MS_EV3_SMUX_MODE_UART ? 0 : MS_EV3_SMUX_ANALOG_MODE_DATA);
	if (ret < 0)
		return ret;
	data->mode = new_mode;
	schedule_delayed_work(&data->detect_sensor_work, 0);

	return count;
}

static DEVICE_ATTR_RO(modes);
static DEVICE_ATTR_RW(mode);

static struct attribute *ms_ev3_smux_input_port_attrs[] = {
	&dev_attr_modes.attr,
	&dev_attr_mode.attr,
	NULL
};

ATTRIBUTE_GROUPS(ms_ev3_smux_input_port);

extern void ms_ev3_smux_register_poll_cb(struct i2c_client *client,
					 legoev3_analog_cb_func_t cb);

static int ms_ev3_smux_input_port_probe(struct legoev3_port *port)
{
	struct ms_ev3_smux_input_port_data *data;
	struct ms_ev3_smux_input_port_platform_data *pdata =
							port->dev.platform_data;
	int ret;

	if (WARN(!pdata, "Platform data is required."))
		return -EINVAL;

	data = kzalloc(sizeof(struct ms_ev3_smux_input_port_data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	ret = sysfs_create_groups(&port->dev.kobj, ms_ev3_smux_input_port_groups);
	if (ret)
		goto err_sysfs_create_groups;

	data->in_port = port;
	memcpy(&data->pdata, pdata,
	       sizeof(struct ms_ev3_smux_input_port_platform_data));
	port->in_ops.register_analog_cb = ms_ev3_smux_input_port_register_analog_cb;
	dev_set_drvdata(&port->dev, data);
	ms_ev3_smux_register_poll_cb(data->pdata.client,
				     ms_ev3_smux_input_port_poll_cb);

	ret = i2c_smbus_read_byte_data(data->pdata.client, MS_EV3_SMUX_MODE_REG);
	data->mode = ret == MS_EV3_SMUX_ANALOG_MODE_DATA ?
		MS_EV3_SMUX_MODE_ANALOG : MS_EV3_SMUX_MODE_UART;

	INIT_DELAYED_WORK(&data->detect_sensor_work,
			  ms_ev3_smux_input_port_detect_sensor);
	schedule_delayed_work(&data->detect_sensor_work, 0);

	return 0;

err_sysfs_create_groups:
	kfree(data);

	return ret;
}

static int ms_ev3_smux_input_port_remove(struct legoev3_port *port)
{
	struct ms_ev3_smux_input_port_data *data = dev_get_drvdata(&port->dev);

	data->removing_driver = true;
	cancel_delayed_work_sync(&data->detect_sensor_work);
	if (data->host)
		ms_ev3_smux_input_port_unregister_host(data);
	ms_ev3_smux_register_poll_cb(data->pdata.client, NULL);
	dev_set_drvdata(&port->dev, NULL);
	port->in_ops.register_analog_cb = NULL;
	sysfs_remove_groups(&port->dev.kobj, ms_ev3_smux_input_port_groups);
	kfree(data);

	return 0;
}

struct legoev3_port_driver ms_ev3_smux_input_port_driver = {
	.probe		= ms_ev3_smux_input_port_probe,
	.remove		= ms_ev3_smux_input_port_remove,
	.driver = {
		.name	= "ms-ev3-smux-input-port",
		.owner	= THIS_MODULE,
	},
};
legoev3_port_driver(ms_ev3_smux_input_port_driver);

MODULE_DESCRIPTION("Input port driver for mindsensors.com EV3 Sensor Multiplexer");
MODULE_AUTHOR("David Lechner <david@lechnology.com>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("legoev3:ms-ev3-smux-input-port");
