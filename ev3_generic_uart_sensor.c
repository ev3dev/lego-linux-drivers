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

struct ev3_generic_uart_sensor_data {
	struct sensor_controls_device ctrl;
	struct sensor_controls_mode_info modes[LEGOEV3_UART_MODE_MAX + 2];
	struct tty_struct *tty;
};

static int ev3_generic_uart_sensor_get_mode(struct sensor_controls_device *ctrl)
{
	struct ev3_generic_uart_sensor_data *ev3_unk =
		container_of(ctrl, struct ev3_generic_uart_sensor_data, ctrl);

	return legoev3_uart_get_mode(ev3_unk->tty);
}

static int ev3_generic_uart_sensor_set_mode(struct sensor_controls_device *ctrl,
                                            int mode)
{
	struct ev3_generic_uart_sensor_data *ev3_unk =
		container_of(ctrl, struct ev3_generic_uart_sensor_data, ctrl);

	return legoev3_uart_set_mode(ev3_unk->tty, mode);
}

static int __devinit ev3_generic_uart_sensor_probe(struct legoev3_port_device *sensor)
{
	struct ev3_generic_uart_sensor_data *ev3_unk;
	struct legoev3_uart_sensor_platform_data *pdata = sensor->dev.platform_data;
	int err, i;

	if (WARN_ON(!pdata))
		return -EINVAL;

	ev3_unk = kzalloc(sizeof(struct ev3_generic_uart_sensor_data), GFP_KERNEL);
	if (!ev3_unk)
		return -ENOMEM;

	ev3_unk->tty = pdata->tty;
	ev3_unk->ctrl.name	= "generic";
	ev3_unk->ctrl.id	= -1; /* TODO do we need to make this unique or get rid of id? */
	ev3_unk->ctrl.get_mode	= ev3_generic_uart_sensor_get_mode;
	ev3_unk->ctrl.set_mode	= ev3_generic_uart_sensor_set_mode;
	for (i = 0; i < pdata->num_modes; i++) {
		ev3_unk->modes[i].name = pdata->mode_info[i].name;
		ev3_unk->modes[i].id = i;
	}
	ev3_unk->ctrl.mode_info	= ev3_unk->modes;

	err = register_sensor_controls(&ev3_unk->ctrl, &sensor->dev);
	if (err)
		goto register_generic_uart_sensor_fail;

	err = dev_set_drvdata(&sensor->dev, ev3_unk);
	if (err)
		goto dev_set_drvdata_fail;

	dev_info(&sensor->dev, "EV3 generic sensor connected to port %s\n", "?");
//		 dev_name(&ev3_unk->in_port->dev));

	return 0;

dev_set_drvdata_fail:
	unregister_sensor_controls(&ev3_unk->ctrl);
register_generic_uart_sensor_fail:
	kfree(ev3_unk);

	return err;
}

static int __devexit ev3_generic_uart_sensor_remove(struct legoev3_port_device *sensor)
{
	struct ev3_generic_uart_sensor_data *ev3_unk = dev_get_drvdata(&sensor->dev);

	dev_info(&sensor->dev, "EV3 Touch sensor removed from port %s\n", "?");
//		 dev_name(&ev3_unk->in_port->dev));

	unregister_sensor_controls(&ev3_unk->ctrl);
	dev_set_drvdata(&sensor->dev, NULL);
	kfree(ev3_unk);
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
