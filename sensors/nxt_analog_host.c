/*
 * NXT analog host driver for LEGO Mindstorms EV3
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
 * NXT Analog Host Driver
 *
 * This driver tells an [EV3 input port] to configure itself for Analog/NXT
 * communications. If the auto-detection algorithm was able to determine the
 * exact type of sensor, that sensor driver is loaded, otherwise the generic
 * [nxt-analog] sensor driver is loaded. You can set the correct driver for
 * the attached sensor using the `set_sensor` attribute (see below).
 * .
 * ### sysfs Attributes
 * .
 * You can find this device at `/sys/bus/legoev3/devices/in<N>:nxt-analog-host`
 * where `<N>` is the number of an input port (1 to 4). NOTE: This host is also
 * used by the [HiTechnic NXT Sensor Multiplexer], in which case the device
 * name will be `in<N>:mux<M>:nxt-analog-host`.
 * .
 * `device_type` (read-only)
 * : Returns `nxt-analog-host`
 * .
 * `port_name` (read-only)
 * : Returns the name of the port this host is connected to (e.g. `in1`).
 * .
 * `set_sensor` (write-only)
 * : Writing the name of the sensor driver will remove the existing device from
 *   this host and load a new sysfs device for the sensor specified. The name
 *   of the sensor is one of the drivers in the [nxt-analog-sensor] module from
 *   the [list of supported sensors] (e.g. `lego-nxt-sound`). In other words,
 *   only sensors with connection type of Analog/NXT work with the
 *   `nxt-analog-host`. A space separated list of valid sensor names can be
 *   obtained by reading `/sys/bus/legoev3/drivers/nxt-analog-sensor/sensor_names`.
 * .
 * [EV3 input port]: ../ev3-input-port
 * [nxt-analog]: ../generic-nxt-analog-sensor
 * [HiTechnic NXT Sensor Multiplexer]: ../hitechnic-nxt-sensor-multiplexer
 * [nxt-analog-sensor]: ../nxt-analog-sensor
 * [list of supported sensors]: ../#supported-sensors
 */

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/legoev3/legoev3_ports.h>
#include <linux/legoev3/ev3_input_port.h>

struct nxt_analog_host_data {
	struct legoev3_port *in_port;
	struct legoev3_port_device *sensor;
	struct mutex sensor_mutex;
};

const struct attribute_group *nxt_analog_sensor_device_type_attr_groups[] = {
	&legoev3_port_device_type_attr_grp,
	NULL
};

struct device_type nxt_analog_sensor_device_type = {
	.name	= "nxt-analog-sensor",
	.groups	= nxt_analog_sensor_device_type_attr_groups,
	.uevent = legoev3_port_device_uevent,
};

static ssize_t store_set_sensor(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct nxt_analog_host_data *data = dev_get_drvdata(dev);
	struct legoev3_port_device *sensor;
	char buf_copy[count + 1];
	char* name;

	if (mutex_is_locked(&data->sensor_mutex))
		return -EBUSY;

	strncpy(buf_copy, buf, count);
	buf_copy[count] = 0;
	name = strim(buf_copy);

	if (data->sensor && !strcmp(name, data->sensor->name))
		return count;

	mutex_lock(&data->sensor_mutex);
	if (data->sensor) {
		legoev3_port_device_unregister(data->sensor);
		data->sensor = NULL;
	}
	sensor = legoev3_port_device_register(name,
		&nxt_analog_sensor_device_type, dev, NULL, 0, data->in_port);
	if (IS_ERR(sensor))
		dev_warn(dev, "Failed to sensor %s. %ld", name, PTR_ERR(sensor));
	else
		data->sensor = sensor;
	mutex_unlock(&data->sensor_mutex);

	return count;
}

DEVICE_ATTR(set_sensor, S_IWUSR, NULL, store_set_sensor);

static struct attribute *nxt_analog_host_attrs[] = {
	&dev_attr_set_sensor.attr,
	NULL
};

const struct attribute_group nxt_analog_host_attr_grp = {
	.attrs = nxt_analog_host_attrs,
};

const struct attribute_group *nxt_analog_host_driver_attr_groups[] = {
	&nxt_analog_host_attr_grp,
	NULL
};

static int nxt_analog_host_probe(struct legoev3_port_device *host)
{
	struct nxt_analog_host_data *data;
	struct ev3_analog_host_platform_data *pdata = host->dev.platform_data;
	struct legoev3_port_device *sensor;
	int err;

	data = kzalloc(sizeof(struct nxt_analog_host_data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->in_port = host->port;
	mutex_init(&data->sensor_mutex);
	dev_set_drvdata(&host->dev, data);

	err = sysfs_create_groups(&host->dev.kobj,
					nxt_analog_host_driver_attr_groups);
	if (err) {
		dev_err(&host->dev, "failed to add attribute groups");
		goto err_sysfs_create_groups;
	}

	if (pdata && pdata->inital_sensor) {
		sensor = legoev3_port_device_register(pdata->inital_sensor,
			&nxt_analog_sensor_device_type, &host->dev, NULL, 0,
			data->in_port);
		if (IS_ERR(sensor))
			dev_warn(&host->dev, "Failed to sensor %s. %ld",
				pdata->inital_sensor, PTR_ERR(sensor));
		else
			data->sensor = sensor;
	}

	return 0;

err_sysfs_create_groups:
	kfree(data);

	return err;
}

static int nxt_analog_host_remove(struct legoev3_port_device *host)
{
	struct nxt_analog_host_data *data = dev_get_drvdata(&host->dev);

	mutex_destroy(&data->sensor_mutex);
	if (data->sensor)
		legoev3_port_device_unregister(data->sensor);
	sysfs_remove_groups(&host->dev.kobj, nxt_analog_host_driver_attr_groups);
	dev_set_drvdata(&host->dev, NULL);
	kfree(data);

	return 0;
}

struct legoev3_port_device_driver nxt_analog_host_driver = {
	.probe	= nxt_analog_host_probe,
	.remove	= nxt_analog_host_remove,
	.driver = {
		.name	= "nxt-analog-host",
		.owner	= THIS_MODULE,
	},
};
legoev3_port_device_driver(nxt_analog_host_driver);

MODULE_DESCRIPTION("NXT analog host driver for LEGO Mindstorms EV3");
MODULE_AUTHOR("David Lechner <david@lechnology.com>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("legoev3:nxt-analog-host");
