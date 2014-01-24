/*
 * EV3 generic UART sensor device driver for LEGO Mindstorms EV3
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

#include <linux/device.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/tty.h>
#include <linux/legoev3/legoev3_ports.h>
#include <linux/legoev3/legoev3_uart.h>
#include <linux/legoev3/sensor_controls_class.h>
#include <linux/legoev3/measure_sensor_class.h>

struct ev3_generic_uart_sensor_data {
	struct sensor_controls_device ctrl;
	struct sensor_controls_mode_info modes[LEGOEV3_UART_MODE_MAX + 2];
	struct measure_sensor_device measure;
	struct measure_sensor_scale_info scale_info[2];
	struct legoev3_port_device *sensor;
	struct legoev3_uart_sensor_platform_data *pdata;
};

static inline void print_raw_data(u8 *data)
{
	int i;

	for (i = 0; i < 32; i++) {
		printk("0x%02X", data[i]);
		printk("%c", i % 15 ? '\n': ' ');
	}
}

int ev3_generic_uart_sensor_raw_u8_value(struct measure_sensor_device *measure)
{
	struct ev3_generic_uart_sensor_data *ev3_gen =
		container_of(measure, struct ev3_generic_uart_sensor_data, measure);
	int mode = legoev3_uart_get_mode(ev3_gen->pdata->tty);
print_raw_data(ev3_gen->pdata->mode_info[mode].raw_data);
	return ev3_gen->pdata->mode_info[mode].raw_data[0];
}

int ev3_generic_uart_sensor_raw_u16_value(struct measure_sensor_device *measure)
{
	struct ev3_generic_uart_sensor_data *ev3_gen =
		container_of(measure, struct ev3_generic_uart_sensor_data, measure);
	int mode = legoev3_uart_get_mode(ev3_gen->pdata->tty);
print_raw_data(ev3_gen->pdata->mode_info[mode].raw_data);
	return *(u16 *)ev3_gen->pdata->mode_info[mode].raw_data;
}

int ev3_generic_uart_sensor_raw_u32_value(struct measure_sensor_device *measure)
{
	struct ev3_generic_uart_sensor_data *ev3_gen =
		container_of(measure, struct ev3_generic_uart_sensor_data, measure);
	int mode = legoev3_uart_get_mode(ev3_gen->pdata->tty);
print_raw_data(ev3_gen->pdata->mode_info[mode].raw_data);
	return *(u32 *)ev3_gen->pdata->mode_info[mode].raw_data;
}

int ev3_generic_uart_sensor_raw_float_value(struct measure_sensor_device *measure)
{
	struct ev3_generic_uart_sensor_data *ev3_gen =
		container_of(measure, struct ev3_generic_uart_sensor_data, measure);
	int mode = legoev3_uart_get_mode(ev3_gen->pdata->tty);
print_raw_data(ev3_gen->pdata->mode_info[mode].raw_data);
	/* TODO: need a float to int conversion function */
	return *(u32 *)ev3_gen->pdata->mode_info[mode].raw_data;
}

int ev3_generic_uart_sensor_register_measure(
	struct ev3_generic_uart_sensor_data *ev3_gen, int mode)
{
	ev3_gen->measure.name = ev3_gen->pdata->mode_info[mode].name;
	ev3_gen->measure.raw_min = ev3_gen->pdata->mode_info[mode].raw_min;
	ev3_gen->measure.raw_max = ev3_gen->pdata->mode_info[mode].raw_max;
	ev3_gen->measure.cal_min = ev3_gen->pdata->mode_info[mode].raw_min;
	ev3_gen->measure.cal_max = ev3_gen->pdata->mode_info[mode].raw_max;
	ev3_gen->scale_info[0].units = ev3_gen->pdata->mode_info[mode].units;
	ev3_gen->scale_info[0].min = ev3_gen->pdata->mode_info[mode].si_min;
	ev3_gen->scale_info[0].max = ev3_gen->pdata->mode_info[mode].si_max;
	ev3_gen->scale_info[0].dp = ev3_gen->pdata->mode_info[mode].decimals;
	switch (ev3_gen->pdata->mode_info[mode].format) {
	case LEGOEV3_UART_DATA_8:
		ev3_gen->measure.raw_value = ev3_generic_uart_sensor_raw_u8_value;
		break;
	case LEGOEV3_UART_DATA_16:
		ev3_gen->measure.raw_value = ev3_generic_uart_sensor_raw_u16_value;
		break;
	case LEGOEV3_UART_DATA_32:
		ev3_gen->measure.raw_value = ev3_generic_uart_sensor_raw_u32_value;
		break;
	case LEGOEV3_UART_DATA_FLOAT:
		ev3_gen->measure.raw_value = ev3_generic_uart_sensor_raw_float_value;
		break;
	}
	ev3_gen->measure.cal_value = ev3_gen->measure.raw_value;

	return register_measure_sensor(&ev3_gen->measure, &ev3_gen->sensor->dev);
}

static int ev3_generic_uart_sensor_get_mode(struct sensor_controls_device *ctrl)
{
	struct ev3_generic_uart_sensor_data *ev3_gen =
		container_of(ctrl, struct ev3_generic_uart_sensor_data, ctrl);

	return legoev3_uart_get_mode(ev3_gen->pdata->tty);
}

static int ev3_generic_uart_sensor_set_mode(struct sensor_controls_device *ctrl,
                                            int mode)
{
	struct ev3_generic_uart_sensor_data *ev3_gen =
		container_of(ctrl, struct ev3_generic_uart_sensor_data, ctrl);
	int ret;

	if (mode == legoev3_uart_get_mode(ev3_gen->pdata->tty))
		return 0;

	ret = legoev3_uart_set_mode(ev3_gen->pdata->tty, mode);
	if (ret)
		return ret;

	unregister_measure_sensor(&ev3_gen->measure);

	return ev3_generic_uart_sensor_register_measure(ev3_gen, mode);
}

static int __devinit ev3_generic_uart_sensor_probe(struct legoev3_port_device *sensor)
{
	struct ev3_generic_uart_sensor_data *ev3_gen;
	struct legoev3_uart_sensor_platform_data *pdata = sensor->dev.platform_data;
	int err, i;

	if (WARN_ON(!pdata))
		return -EINVAL;

	ev3_gen = kzalloc(sizeof(struct ev3_generic_uart_sensor_data), GFP_KERNEL);
	if (!ev3_gen)
		return -ENOMEM;

	ev3_gen->ctrl.name	= "generic";
	ev3_gen->ctrl.id	= -1; /* TODO do we need to make this unique or get rid of id? */
	ev3_gen->ctrl.get_mode	= ev3_generic_uart_sensor_get_mode;
	ev3_gen->ctrl.set_mode	= ev3_generic_uart_sensor_set_mode;
	for (i = 0; i < pdata->num_modes; i++) {
		ev3_gen->modes[i].name = pdata->mode_info[i].name;
		ev3_gen->modes[i].id = i;
	}
	ev3_gen->ctrl.mode_info	= ev3_gen->modes;
	ev3_gen->measure.scale_info = ev3_gen->scale_info;
	ev3_gen->sensor = sensor;
	ev3_gen->pdata = pdata;

	err = register_sensor_controls(&ev3_gen->ctrl, &sensor->dev);
	if (err)
		goto register_generic_uart_sensor_fail;

	err = ev3_generic_uart_sensor_register_measure(ev3_gen,
				legoev3_uart_get_mode(ev3_gen->pdata->tty));
	if (err)
		goto err_ev3_generic_uart_sensor_register_measure;

	err = dev_set_drvdata(&sensor->dev, ev3_gen);
	if (err)
		goto dev_set_drvdata_fail;

	dev_info(&sensor->dev, "EV3 generic sensor connected to port %s\n", "?");
//		 dev_name(&ev3_gen->in_port->dev));

	return 0;

dev_set_drvdata_fail:
	unregister_measure_sensor(&ev3_gen->measure);
err_ev3_generic_uart_sensor_register_measure:
	unregister_sensor_controls(&ev3_gen->ctrl);
register_generic_uart_sensor_fail:
	kfree(ev3_gen);

	return err;
}

static int __devexit ev3_generic_uart_sensor_remove(struct legoev3_port_device *sensor)
{
	struct ev3_generic_uart_sensor_data *ev3_gen = dev_get_drvdata(&sensor->dev);

	dev_info(&sensor->dev, "EV3 Touch sensor removed from port %s\n", "?");
//		 dev_name(&ev3_gen->in_port->dev));

	unregister_sensor_controls(&ev3_gen->ctrl);
	unregister_measure_sensor(&ev3_gen->measure);
	dev_set_drvdata(&sensor->dev, NULL);
	kfree(ev3_gen);
	return 0;
}

static struct legoev3_port_device_id ev3_uart_ids[LEGOEV3_UART_TYPE_MAX + 1];

struct legoev3_port_driver ev3_generic_uart_sensor_driver = {
	.probe		= ev3_generic_uart_sensor_probe,
	.remove		= __devexit_p(ev3_generic_uart_sensor_remove),
	.id_table	= ev3_uart_ids,
	.driver = {
		.name	= "ev3-generic-uart-sensor",
		.owner	= THIS_MODULE,
	},
};

int ev3_generic_uart_sensor_driver_register(struct legoev3_port_driver *drv)
{
	int i;

	for (i = 0; i <= LEGOEV3_UART_TYPE_MAX; i++) {
		snprintf(ev3_uart_ids[i].name, LEGOEV3_PORT_NAME_SIZE, "%s","ev3-uart-sensor");
		ev3_uart_ids[i].type_id = i;
	}
	return legoev3_register_port_driver(drv);
}

void ev3_generic_uart_sensor_driver_unregister(struct legoev3_port_driver *drv)
{
	legoev3_unregister_port_driver(drv);
}

module_driver(ev3_generic_uart_sensor_driver,
	      ev3_generic_uart_sensor_driver_register,
	      ev3_generic_uart_sensor_driver_unregister);

MODULE_DESCRIPTION("EV3 generic uart sensor device driver for LEGO Mindstorms EV3");
MODULE_AUTHOR("David Lechner <david@lechnology.com>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("legoev3:ev3-uart-sensor");
