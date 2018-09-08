// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2018 David Lechner <david@lechnology.com>
 *
 * Bus driver for TI Programmable Realtime Unit
 *
 * This driver just makes sure the the PRUSS is powered up and then registers
 * the various PRU subcomponents as platform devices.
 */

#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>

static int ti_pruss_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	int err;

	pm_runtime_enable(dev);

	err = devm_of_platform_populate(dev);
	if (err) {
		pm_runtime_disable(dev);
		return err;
	}

	return 0;
}

static int ti_pruss_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;

	pm_runtime_disable(dev);

	return 0;
}

static const struct of_device_id ti_pruss_of_match[] = {
	{ .compatible = "ev3dev,da850-pruss" },
	{ },
};
MODULE_DEVICE_TABLE(of, ti_pruss_of_match);

static struct platform_driver ti_pruss_driver = {
	.probe	= ti_pruss_probe,
	.remove	= ti_pruss_remove,
	.driver	= {
		.name = "ti-pruss",
		.of_match_table = ti_pruss_of_match,
	},
};
module_platform_driver(ti_pruss_driver);

MODULE_AUTHOR("David Lechner <david@lechnology.com>");
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Bus driver for TI PRU");
