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
 * DOC: userspace
 *
 * By default, a sysfs device is created for each input and output port on the
 * EVB. See the `input ports`_ and `output ports`_ driver descriptions for more
 * information on how these work. There is a single module for input and output
 * ports named ``evb_ports``. This module has some module parameters.
 *
 * .. flat-table:: Module Parameters
 *    :widths: 1 5
 *    :header-rows: 1
 *
 *    * - Name
 *      - Description
 *
 *    * - ``disable_in_port``
 *      - Used to prevent the input port device from being loaded. This is
 *        useful if you want to use input port 1 for printing kernel messages
 *        this if you want while you are debugging the Linux kernel. You may
 *        also want to do to control the input port gpios directly.
 *
 *    * - ``disable_out_port``
 *      - Used to prevent the output port from being loaded. This leaves the
 *        pwm device and gpios used by the port free to be controlled directly
 *        or used by other drivers.
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

struct dentry *evb_ports_debug;
EXPORT_SYMBOL_GPL(evb_ports_debug);

static int __init evb_ports_driver_init(void)
{
	int err;

	err = platform_driver_register(&evb_ports_driver);
	if (err)
		return err;

	evb_ports_debug = debugfs_create_dir("evb-ports", NULL);

	return 0;
}
module_init(evb_ports_driver_init);

static void __exit evb_ports_driver_exit(void)
{
	debugfs_remove(evb_ports_debug);
	platform_driver_unregister(&evb_ports_driver);
}
module_exit(evb_ports_driver_exit);

MODULE_DESCRIPTION("Support for FatcatLab EVB input and output ports.");
MODULE_AUTHOR("David Lechner <david@lechnology.com>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:evb-ports");
