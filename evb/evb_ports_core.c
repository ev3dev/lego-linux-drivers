/*
 * Support for the input and output ports on the FatcatLab EVB
 *
 * Copyright (C) 2013-2014,2016 David Lechner <david@lechnology.com>
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
 * FatcatLab EVB Input and Output Ports
 *
 * By default, a sysfs device is created for each input and output port on the
 * EVB. See the [evb-input-port] and [evb-output-port] drivers for more
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
 * [evb-input-port]: /docs/ports/evb-input-port
 * [evb-output-port]: /docs/ports/evb-output-port
 */

#include <linux/device.h>
#include <linux/module.h>
#include <linux/string.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/ioport.h>
#include <linux/platform_device.h>
#include <linux/of_platform.h>

#include <lego_port_class.h>

#include "evb_ports.h"

struct evb_ports_data {
	struct platform_device *pdev;
	struct evb_ports_platform_data *pdata;
	struct lego_port_device *in_ports;
	struct lego_port_device *out_ports;
};
#if 0
static uint disable_in_port[NUM_EVB_PORT_IN];
static int num_disabled_in_port;
module_param_array(disable_in_port, uint, &num_disabled_in_port, 0);
MODULE_PARM_DESC(disable_in_port, "Disables specified input ports. (1,2,3,4)");
static uint disable_out_port[NUM_EVB_PORT_OUT];
static int num_disabled_out_port;
module_param_array(disable_out_port, uint, &num_disabled_out_port, 0);
MODULE_PARM_DESC(disable_out_port, "Disables specified output ports. (1,2,3,4)");

int evb_register_input_ports(struct evb_ports_data *ports,
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
			goto err_evb_port_register;
		}
	} while (++i < len);

	return 0;

err_evb_port_register:
	while (i--)
		ev3_input_port_unregister(ports->in_ports[i]);

	return err;
}

int evb_register_output_ports(struct evb_ports_data *ports,
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
			goto err_evb_port_register;
		}
	} while (++i < len);

	return 0;

err_evb_port_register:
	while (i--)
		ev3_output_port_unregister(ports->out_ports[i]);

	return err;
}
#endif

static int evb_ports_probe(struct platform_device *pdev)
{
	int err;

	err = of_platform_populate(pdev->dev.of_node, NULL, NULL, &pdev->dev);
	if (err < 0) {
		dev_err(&pdev->dev, "Error populating children.\n");
		return -err;
	}

	return 0;
}

static int evb_ports_remove(struct platform_device *pdev)
{
	of_platform_depopulate(&pdev->dev);

	return 0;
}

static const struct of_device_id evb_ports_dt_ids[] = {
	{ .compatible = "ev3dev,evb-ports", },
	{ }
};
MODULE_DEVICE_TABLE(of, evb_ports_dt_ids);

static struct platform_driver evb_ports_driver = {
	.driver	= {
		.name	= "evb-ports",
		.of_match_table = evb_ports_dt_ids,
	},
	.probe	= evb_ports_probe,
	.remove	= evb_ports_remove,
};
module_platform_driver(evb_ports_driver);

MODULE_DESCRIPTION("Support for FatcatLab EVB input and output ports.");
MODULE_AUTHOR("David Lechner <david@lechnology.com>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:evb-ports");
