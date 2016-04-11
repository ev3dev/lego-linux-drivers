/*
 * Support for the input and output ports on the LEGO MINDSTORMS EV3
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
 * -----------------------------------------------------------------------------
 * This module provides components for interfacing with the input and output
 * ports on the EV3 brick.
 *
 * There is a platform driver called legoev3-ports. The matching platform
 * device is declared in board-legoev3.c, which defines the gpios and
 * other devices used by the input and output ports on the EV3. When this
 * driver is loaded, it also creates a bus called "legoev3". This bus is
 * used to match the sensor and motor devices plugged into the ports to
 * drivers for those devices.
 *
 * Each input and output port has its own device node. They perform device
 * discovery similar to Device3 in d_analog.c in the lms2012 code and notify
 * the legoev3 bus when a device has been connected or disconnected.
 * -----------------------------------------------------------------------------
 */

/*
 * Note: The comment block below is used to generate docs on the ev3dev website.
 * Use kramdown (markdown) format. Use a '.' as a placeholder when blank lines
 * or leading whitespace is important for the markdown syntax.
 */

/**
 * DOC: website
 *
 * EV3 Input and Output Ports
 *
 * By default, a sysfs device is created for each input and output port on the
 * EV3. See the [legoev3-input-port] and [legoev3-output-port] drivers for more
 * information on how these work.
 * .
 * ### Module parameters
 * .
 * `disable_in_port`
 * : Used to prevent the input port device from being loaded. This is useful
 *   if you want to use input port 1 for printing kernel messages while you
 *   are debugging the Linux kernel. You may also want to do this if you want
 *   to control the input port gpios directly.
 * .
 * `disable_out_port`
 * : Used to prevent the output port from being loaded. This leaves the pwm
 *   device and gpios used by the port free to be controlled directly or used
 *   by other drivers.
 * .
 * [legoev3-input-port]: /docs/ports/legoev3-input-port
 * [legoev3-output-port]: /docs/ports/legoev3-output-port
 */

#include <linux/device.h>
#include <linux/module.h>
#include <linux/string.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/ioport.h>
#include <linux/platform_device.h>
#include <linux/platform_data/legoev3.h>

#include <lego_port_class.h>

#include "legoev3_ports.h"

struct legoev3_ports_data {
	struct platform_device *pdev;
	struct legoev3_ports_platform_data *pdata;
	struct lego_port_device *in_ports[NUM_EV3_PORT_IN];
	struct lego_port_device *out_ports[NUM_EV3_PORT_OUT];
};

static uint disable_in_port[NUM_EV3_PORT_IN];
static int num_disabled_in_port;
module_param_array(disable_in_port, uint, &num_disabled_in_port, 0);
MODULE_PARM_DESC(disable_in_port, "Disables specified input ports. (1,2,3,4)");
static uint disable_out_port[NUM_EV3_PORT_OUT];
static int num_disabled_out_port;
module_param_array(disable_out_port, uint, &num_disabled_out_port, 0);
MODULE_PARM_DESC(disable_out_port, "Disables specified output ports. (1,2,3,4)");

int legoev3_register_input_ports(struct legoev3_ports_data *ports,
				 struct ev3_input_port_platform_data data[],
				 unsigned len)
{
	int err, j, id;
	int i = 0;
	bool skip;

	do {
		skip = false;
		id = data[i].id + 1;
		for (j = 0; j < num_disabled_in_port; j++) {
			if (disable_in_port[j] == id) {
				skip = true;
				break;
			}
		}
		if (skip) {
			ports->in_ports[i] = NULL;
			dev_info(&ports->pdev->dev,
				"Input port in%d is disabled.\n", id);
			continue;
		}
		ports->in_ports[i] =
			ev3_input_port_register(&data[i], &ports->pdev->dev);
		if (IS_ERR(ports->in_ports[i])) {
			err = PTR_ERR(ports->in_ports[i]);
			goto err_legoev3_port_register;
		}
	} while (++i < len);

	return 0;

err_legoev3_port_register:
	while (i--)
		ev3_input_port_unregister(ports->in_ports[i]);

	return err;
}

int legoev3_register_output_ports(struct legoev3_ports_data *ports,
				  struct ev3_output_port_platform_data data[],
				  unsigned len)
{
	int err, id, j;
	int i = 0;
	bool skip;

	do {
		skip = false;
		id = data[i].id + 1;
		for (j = 0; j < num_disabled_out_port; j++) {
			if (disable_out_port[j] == id) {
				skip = true;
				break;
			}
		}
		if (skip) {
			ports->out_ports[i] = NULL;
			dev_info(&ports->pdev->dev,
				"Output port out%c is disabled.\n", 'A' + id - 1);
			continue;
		}

		ports->out_ports[i] =
			ev3_output_port_register(&data[i], &ports->pdev->dev);
		if (IS_ERR(ports->out_ports[i])) {
			err = PTR_ERR(ports->out_ports[i]);
			goto err_legoev3_port_register;
		}
	} while (++i < len);

	return 0;

err_legoev3_port_register:
	while (i--)
		ev3_output_port_unregister(ports->out_ports[i]);

	return err;
}

static int legoev3_ports_probe(struct platform_device *pdev)
{
	struct legoev3_ports_data *ports;
	int i = 0;
	int err;

	if (!pdev || !pdev->dev.platform_data)
		return -EINVAL;

	ports = kzalloc(sizeof(struct legoev3_ports_data), GFP_KERNEL);
	if (!ports)
		return -ENOMEM;

	ports->pdev = pdev;
	ports->pdata = pdev->dev.platform_data;
	dev_set_drvdata(&pdev->dev, ports);

	err = legoev3_register_input_ports(ports,
					   ports->pdata->input_port_data,
					   NUM_EV3_PORT_IN);
	if (err) {
		dev_err(&pdev->dev, "Could not register input ports!\n");
		goto err_legoev3_register_input_ports;
	}

	err = legoev3_register_output_ports(ports,
					    ports->pdata->output_port_data,
					    NUM_EV3_PORT_OUT);

	if (err) {
		dev_err(&pdev->dev, "Could not register output ports!\n");
		goto err_legoev3_register_output_ports;
	}

	return 0;

err_legoev3_register_output_ports:
	for(i = 0; i < NUM_EV3_PORT_IN; i++)
		ev3_input_port_unregister(ports->in_ports[i]);
err_legoev3_register_input_ports:
	dev_set_drvdata(&pdev->dev, NULL);
	kfree(ports);

	return err;
}

static int legoev3_ports_remove(struct platform_device *pdev)
{
	struct legoev3_ports_data *ports = dev_get_drvdata(&pdev->dev);
	int i;

	for(i = 0; i < NUM_EV3_PORT_IN; i++)
		ev3_input_port_unregister(ports->in_ports[i]);
	for(i = 0; i < NUM_EV3_PORT_OUT; i++)
		ev3_output_port_unregister(ports->out_ports[i]);
	dev_set_drvdata(&pdev->dev, NULL);
	kfree(ports);

	return 0;
}

static struct platform_driver legoev3_ports_driver = {
	.driver	= {
		.name	= "legoev3-ports",
		.owner	= THIS_MODULE,
	},
	.probe	= legoev3_ports_probe,
	.remove	= legoev3_ports_remove,
};
module_platform_driver(legoev3_ports_driver);

MODULE_DESCRIPTION("Support for LEGO MINDSTORMS EV3 input and output ports.");
MODULE_AUTHOR("David Lechner <david@lechnology.com>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:legoev3-ports");
