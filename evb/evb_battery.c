/*
 * Battery driver for FatcatLab's EVB
 *
 * Copyright (C) 2016 David Lechner <david@lechnology.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.

 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/err.h>
#include <linux/iio/consumer.h>
#include <linux/iio/types.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/slab.h>

struct evb_battery {
	struct iio_channel *iio;
	struct power_supply *psy;
};

static int evb_battery_get_property(struct power_supply *psy,
				    enum power_supply_property psp,
				    union power_supply_propval *val)
{
	struct evb_battery *batt = power_supply_get_drvdata(psy);
	int ret = 0;

	switch (psp) {
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		/*
		 * This is a 12-bit analog input with 1.8V reference and a
		 * 201k/33k voltage divider. So reading in 2677uV increments.
		 */
		ret = iio_read_channel_raw(batt->iio, &val->intval);
		/* causes log flooding if we return error */
		if (WARN_ONCE(ret < 0, "iio_read_channel_raw error %d", ret))
			ret = 0;
		val->intval *= 2677;
		break;
	case POWER_SUPPLY_PROP_SCOPE:
		val->intval = POWER_SUPPLY_SCOPE_SYSTEM;
		break;
		return 0;
	default:
		return -EINVAL;
	}

	return ret;
}

static enum power_supply_property evb_battery_props[] = {
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_SCOPE,
};

static const struct power_supply_desc evb_battery_desc = {
	.name			= "evb-battery",
	.type			= POWER_SUPPLY_TYPE_BATTERY,
	.get_property		= evb_battery_get_property,
	.properties		= evb_battery_props,
	.num_properties		= ARRAY_SIZE(evb_battery_props),
};

static int evb_battery_probe(struct platform_device *pdev)
{
	struct evb_battery *batt;
	struct power_supply_config psy_cfg = {};
	int err;

	batt = devm_kzalloc(&pdev->dev, sizeof(*batt), GFP_KERNEL);
	if (!batt)
		return -ENOMEM;

	platform_set_drvdata(pdev, batt);

	batt->iio = iio_channel_get(&pdev->dev, "voltage");
	if (IS_ERR(batt->iio)) {
		err = PTR_ERR(batt->iio);
		if (err != -EPROBE_DEFER)
			dev_err(&pdev->dev, "Failed to get iio channel.");
		return err;
	}

	psy_cfg.of_node		= pdev->dev.of_node;
	psy_cfg.drv_data	= batt;

	batt->psy = power_supply_register(&pdev->dev, &evb_battery_desc,
					  &psy_cfg);
	if (IS_ERR(batt->psy)) {
		dev_err(&pdev->dev, "failed to register power supply\n");
		iio_channel_release(batt->iio);
		return PTR_ERR(batt->psy);
	}

	return 0;
}

static int evb_battery_remove(struct platform_device *pdev)
{
	struct evb_battery *batt = platform_get_drvdata(pdev);

	power_supply_unregister(batt->psy);
	iio_channel_release(batt->iio);

	return 0;
}

static const struct of_device_id of_evb_battery_match[] = {
	{ .compatible = "ev3dev,evb-battery", },
	{ }
};
MODULE_DEVICE_TABLE(of, of_evb_battery_match);

static struct platform_driver evb_battery_driver = {
	.driver	= {
		.name		= "evb-battery",
		.of_match_table = of_evb_battery_match,
	},
	.probe	= evb_battery_probe,
	.remove	= evb_battery_remove,
};
module_platform_driver(evb_battery_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("David Lechner <david@lechnology.com>");
MODULE_DESCRIPTION("FatcatLab EVB Battery");
MODULE_ALIAS("platform:evb-battery");
