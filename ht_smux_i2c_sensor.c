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

/*
 * Note: The comment block below is used to generate docs on the ev3dev website.
 * Use kramdown (markdown) format. Use a '.' as a placeholder when blank lines
 * or leading whitespace is important for the markdown syntax.
 */

/**
 * DOC: website
 *
 * HiTechnic NXT Sensor Multiplexer I2C sensor driver
 *
 * A `ht-smux-i2c-sensor` device is loaded by the [ht-smux-input-port] driver
 * when it is detected by the [sensor mux] (automatic detection only works with
 * the sensors listed in the linked page) or when the sensor is set via the
 * [ht-smux-i2c-host] device. You can use any one of the sensors that has the
 * `nxt-i2c-sensor` module from the [list of supported sensors]. Keep in mind
 * though that the [sensor mux] operates in a read-only mode with I2C sensors.
 * Some modes of I2C sensors require writing data to the sensor and as a result,
 * these modes will not be usable via the [sensor mux].
 * .
 * ### sysfs attributes
 * .
 * These sensors use the [msensor class]. Follow the link for more information.
 * .
 * This device can be found at `/sys/bus/legoev3/devices/in<N>:mux<M>:<device-name>`
 * where `<N>` is the input port on the EV3 (1 to 4), `<M>` is the input port
 * on the sensor mux (1 to 4) and `<device-name`> is the name of the sensor
 * (e.g. `lego-nxt-ultrasonic`).
 * .
 * `device_type` (read-only)
 * : Returns `ht-smux-i2c-sensor`
 * .
 * `port_name` (read-only)
 * : Returns the name of the port this host is connected to (e.g. `in1:mux1`).
 * .
 * [sensor mux]: ../hitechnic-nxt-sensor-multiplexer
 * [list of supported sensors]: ../#supported-sensors
 * [msensor class]: ../msensor-class
 */

#include <linux/device.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/bug.h>
#include <linux/legoev3/legoev3_ports.h>
#include <linux/legoev3/msensor_class.h>

#include "nxt_i2c_sensor.h"
#include "ht_smux.h"

struct ht_smux_i2c_sensor_data {
	struct legoev3_port *in_port;
	struct nxt_i2c_sensor_info info;
	struct msensor_device ms;
	enum nxt_i2c_sensor_type type;
};

extern void ht_smux_input_port_set_i2c_data_reg(struct legoev3_port *in_port,
						u8 reg, u8 count);

static int ht_smux_i2c_sensor_set_mode(void *context, u8 mode)
{
	struct ht_smux_i2c_sensor_data *sensor = context;

	/*
	 * Only allow modes that don't require writing to the sensor.
	 * If we get an error here, it means we did not set num_read_only_modes
	 * correctly in nxt_i2c_sensor_defs.c
	 * */
	if (sensor->info.i2c_mode_info[mode].set_mode_reg) {
		if (sensor->info.i2c_mode_info[mode].set_mode_data
			!= sensor->info.i2c_mode_info[sensor->ms.mode].set_mode_data);
		return -EPERM;
	}

	ht_smux_input_port_set_i2c_data_reg(sensor->in_port,
				sensor->info.i2c_mode_info[mode].read_data_reg,
				sensor->info.ms_mode_info[mode].data_sets);
	sensor->in_port->in_ops.set_pin1_gpio(sensor->in_port,
				sensor->info.i2c_mode_info[mode].pin1_state);

	return 0;
}

extern void ht_smux_input_port_copy_i2c_data(struct legoev3_port *in_port,
					     u8 *dest);

static void ht_smux_i2c_sensor_analog_cb(void *context) {
	struct ht_smux_i2c_sensor_data *sensor = context;
	u8 *raw_data = sensor->ms.mode_info[sensor->ms.mode].raw_data;

	ht_smux_input_port_copy_i2c_data(sensor->in_port, raw_data);
}

extern void ht_smux_input_port_set_i2c_addr(struct legoev3_port *in_port,
					    u8 addr, bool slow);

static int ht_smux_i2c_sensor_probe(struct legoev3_port_device *pdev)
{
	struct ht_smux_i2c_sensor_data *sensor;
	const struct nxt_i2c_sensor_info *sensor_info;
	struct ht_smux_i2c_sensor_platform_data *pdata = pdev->dev.platform_data;
	int err, i;

	if (WARN_ON(!pdev->entry_id))
		return -EINVAL;

	if (WARN_ON(!pdata))
		return -EINVAL;

	sensor_info = &nxt_i2c_sensor_defs[pdev->entry_id->driver_data];

	sensor = kzalloc(sizeof(struct ht_smux_i2c_sensor_data), GFP_KERNEL);
	if (!sensor)
		return -ENOMEM;

	sensor->in_port = pdev->port;
	sensor->type = pdev->entry_id->driver_data;
	memcpy(&sensor->info, sensor_info, sizeof(struct nxt_i2c_sensor_info));

	strncpy(sensor->ms.name, pdev->entry_id->name, MSENSOR_NAME_SIZE);
	strncpy(sensor->ms.port_name, dev_name(&sensor->in_port->dev),
		MSENSOR_NAME_SIZE);
	if (sensor->info.num_read_only_modes)
		sensor->ms.num_modes = sensor->info.num_read_only_modes;
	else
		sensor->ms.num_modes = sensor->info.num_modes;
	sensor->ms.num_view_modes = 1;
	sensor->ms.mode_info = sensor->info.ms_mode_info;
	sensor->ms.set_mode = ht_smux_i2c_sensor_set_mode;
	sensor->ms.context = sensor;
	sensor->ms.address = pdata->address >> 1;

	for (i = 0; i < sensor->ms.num_modes; i++) {
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

	dev_set_drvdata(&pdev->dev, sensor);

	err = register_msensor(&sensor->ms, &pdev->dev);
	if (err) {
		dev_err(&pdev->dev, "could not register sensor!\n");
		goto err_register_msensor;
	}

	sensor->in_port->in_ops.register_analog_cb(sensor->in_port,
		ht_smux_i2c_sensor_analog_cb, sensor);
	ht_smux_input_port_set_i2c_addr(sensor->in_port, pdata->address,
					sensor->info.slow);
	ht_smux_i2c_sensor_set_mode(sensor, 0);

	return 0;

err_register_msensor:
	kfree(sensor);

	return err;
}

static int ht_smux_i2c_sensor_remove(struct legoev3_port_device *pdev)
{
	struct ht_smux_i2c_sensor_data *sensor = dev_get_drvdata(&pdev->dev);

	sensor->in_port->in_ops.register_analog_cb(sensor->in_port, NULL, NULL);
	sensor->in_port->in_ops.set_pin1_gpio(sensor->in_port, 0);
	unregister_msensor(&sensor->ms);
	dev_set_drvdata(&pdev->dev, NULL);
	kfree(sensor);

	return 0;
}

static struct legoev3_port_device_id ht_smux_i2c_sensor_id_table[] = {
	NXT_I2C_SENSOR_ID_TABLE_DATA
};
MODULE_DEVICE_TABLE(legoev3, ht_smux_i2c_sensor_id_table);

static struct legoev3_port_device_driver ht_smux_i2c_sensor_driver = {
	.driver = {
		.name	= "ht-smux-i2c-sensor",
	},
	.id_table	= ht_smux_i2c_sensor_id_table,
	.probe		= ht_smux_i2c_sensor_probe,
	.remove		= ht_smux_i2c_sensor_remove,
};
legoev3_port_device_driver(ht_smux_i2c_sensor_driver);

MODULE_DESCRIPTION("HiTechnic NXT Sensor Multiplexer I2C sensor device driver for LEGO Mindstorms EV3");
MODULE_AUTHOR("David Lechner <david@lechnology.com>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("legoev3:ht-smux-i2c-sensor");
