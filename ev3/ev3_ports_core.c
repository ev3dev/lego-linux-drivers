/*
 * Support for the input and output ports on LEGO MINDSTORMS EV3
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

/**
 * DOC: userspace
 *
 * By default, a sysfs device is created for each input and output port on the
 * EV3. See the `input ports`_ and `output ports`_ driver descriptions for more
 * information on how these work. There is a single module for input and output
 * ports named ``ev3_ports``. This module has some module parameters.
 */

#include <linux/console.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/string.h>

#include <lego_port_class.h>

#include "ev3_ports.h"

static void ev3_ports_disable_node(struct device_node *np)
{
	struct property *newprop;

	newprop = kzalloc(sizeof(*newprop), GFP_KERNEL);
	if (!newprop)
		return;

	newprop->name = kstrdup("status", GFP_KERNEL);
	newprop->value = kstrdup("disabled", GFP_KERNEL);
	newprop->length = sizeof("disabled");
	of_update_property(np, newprop);
}

static int ev3_ports_probe(struct platform_device *pdev)
{
	struct device_node *child;
	struct console *con;
	const char *tty_name;
	char con_name[20];
	int err;

	/* FIXME: this is bad if there is no console= kernel parameter */
	if (!console_drivers)
		return -EPROBE_DEFER;

	for_each_available_child_of_node(pdev->dev.of_node, child) {
		/* ev3dev,tty-name property is optional */
		err = of_property_read_string(child, "ev3dev,tty-name",
					      &tty_name);
		if (err)
			continue;

		/*
		 * if the child node has a tty associated with it and the tty
		 * is being used as a console, then disable that node so we
		 * don't interfere with the console
		 */
		for_each_console(con) {
			snprintf(con_name, 20, "%s%d", con->name, con->index);
			if (strcmp(tty_name, con_name) == 0) {
				dev_info(&pdev->dev,
					"Skipping %s due to UART in use\n",
					 of_node_full_name(child));
				ev3_ports_disable_node(child);
			}
		}
	}

	err = of_platform_populate(pdev->dev.of_node, NULL, NULL, &pdev->dev);
	if (err < 0) {
		dev_err(&pdev->dev, "Error populating children.\n");
		return -err;
	}

	return 0;
}

static int ev3_ports_remove(struct platform_device *pdev)
{
	of_platform_depopulate(&pdev->dev);

	return 0;
}

static const struct of_device_id ev3_ports_dt_ids[] = {
	{ .compatible = "ev3dev,ev3-ports", },
	{ }
};
MODULE_DEVICE_TABLE(of, ev3_ports_dt_ids);

static struct platform_driver ev3_ports_driver = {
	.driver	= {
		.name	= "ev3-ports",
		.of_match_table = ev3_ports_dt_ids,
	},
	.probe	= ev3_ports_probe,
	.remove	= ev3_ports_remove,
};

struct dentry *ev3_ports_debug;
EXPORT_SYMBOL_GPL(ev3_ports_debug);

static int __init ev3_ports_driver_init(void)
{
	int err;

	err = platform_driver_register(&ev3_ports_driver);
	if (err)
		return err;

	ev3_ports_debug = debugfs_create_dir("ev3-ports", NULL);

	return 0;
}
module_init(ev3_ports_driver_init);

static void __exit ev3_ports_driver_exit(void)
{
	debugfs_remove(ev3_ports_debug);
	platform_driver_unregister(&ev3_ports_driver);
}
module_exit(ev3_ports_driver_exit);

MODULE_DESCRIPTION("Support for LEGO MINDSTORMS EV3 input and output ports.");
MODULE_AUTHOR("David Lechner <david@lechnology.com>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:ev3-ports");
