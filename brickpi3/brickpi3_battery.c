/*
 * Dexter Industries BrickPi3 battery power supply driver
 *
 * Copyright (C) 2017 David Lechner <david@lechnology.com>
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
 * - This driver is used to get information about the BrickPi battery.
 * - It uses the `power_supply`_ subsytem.
 * - It registers a sysfs device node at ``/sys/class/power_supply/brickpi3-battery/``.
 *
 * .. flat-table:: Sysfs Attributes
 *    :widths: 1 2
 *
 *    * - ``scope``
 *      - Always returns ``System``.
 *
 *    * - ``voltage_now``
 *      - Returns the battery voltage in microvolts.
 *
 * .. _power_supply: http://lxr.free-electrons.com/source/Documentation/power/power_supply_class.txt?v=4.4
 */

#include <linux/device.h>
#include <linux/iio/consumer.h>
#include <linux/iio/types.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>

#include "brickpi3.h"


struct brickpi3_battery {
	struct iio_channel *iio;
	struct power_supply *psy;
};

static int brickpi3_battery_get_property(struct power_supply *psy,
					 enum power_supply_property psp,
					 union power_supply_propval *val)
{
	struct brickpi3_battery *batt = power_supply_get_drvdata(psy);
	int ret = 0;

	switch (psp) {
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		ret = iio_read_channel_processed(batt->iio, &val->intval);
		/* it will cause log flooding if we return error, so just warn */
		if (WARN_ONCE(ret < 0, "iio_read_channel_processed error (%d)",
				ret))
			break;
		/* iio processed value is in mV, but power supply wants uV */
		val->intval *= 1000;
		break;
	case POWER_SUPPLY_PROP_SCOPE:
		val->intval = POWER_SUPPLY_SCOPE_SYSTEM;
		break;
		return 0;
	default:
		return -EINVAL;
	}

	return 0;
}

static enum power_supply_property brickpi3_battery_props[] = {
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_SCOPE,
};

static const struct power_supply_desc brickpi3_battery_desc = {
	.name			= "brickpi3-battery",
	.type			= POWER_SUPPLY_TYPE_BATTERY,
	.get_property		= brickpi3_battery_get_property,
	.properties		= brickpi3_battery_props,
	.num_properties		= ARRAY_SIZE(brickpi3_battery_props),
};

static void brickpi3_battery_release(struct device *dev, void *res)
{
	struct brickpi3_battery *batt = res;

	power_supply_unregister(batt->psy);
	iio_channel_release(batt->iio);
}

static int brickpi3_battery_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct brickpi3_battery *batt;
	struct power_supply_config psy_cfg = {};

	batt = devres_alloc(brickpi3_battery_release, sizeof(*batt), GFP_KERNEL);
	if (!batt)
		return -ENOMEM;

	batt->iio = iio_channel_get(dev, "voltage");
	if (IS_ERR(batt->iio)) {
		if (PTR_ERR(batt->iio) != -EPROBE_DEFER)
			dev_err(dev, "Failed to get voltage iio channel\n");
		devres_free(batt);
		return PTR_ERR(batt->iio);
	}

	psy_cfg.of_node = dev->of_node;
	psy_cfg.drv_data = batt;

	batt->psy = power_supply_register(dev, &brickpi3_battery_desc, &psy_cfg);
	if (IS_ERR(batt->psy)) {
		dev_err(dev, "Failed to register power supply\n");
		iio_channel_release(batt->iio);
		devres_free(batt);
		return PTR_ERR(batt->psy);
	}

	devres_add(dev, batt);

	return 0;
}

static const struct of_device_id of_brickpi3_battery_match[] = {
	{ .compatible = "ev3dev,brickpi3-battery", },
	{ }
};
MODULE_DEVICE_TABLE(of, of_brickpi3_battery_match);

static struct platform_driver brickpi3_battery_driver = {
	.driver	= {
		.name		= "brickpi3-battery",
		.of_match_table = of_brickpi3_battery_match,
	},
	.probe	= brickpi3_battery_probe,
};
module_platform_driver(brickpi3_battery_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("David Lechner <david@lechnology.com>");
MODULE_DESCRIPTION("Generic voltage-only battery");
MODULE_ALIAS("platform:brickpi3-battery");
