/*
 * Battery driver for the LEGO MINDSTORMS EV3
 *
 * Copyright (c) 2013 David Lechner <david@lechnology.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

/**
 * DOC: userspace
 *
 * - This driver is used to get information about the EV3 battery.
 * - It uses the `power_supply`_ subsytem.
 * - It registers a sysfs device node at ``/sys/class/power_supply/legoev3-battery/``.
 *
 * .. flat-table:: Sysfs Attributes
 *    :widths: 1 3
 *
 *    * - ``current_now``
 *      - Returns the battery current in microamps.
 *
 *    * - ``scope``
 *      - Always returns ``System``.
 *
 *    * - ``technology``
 *      - Returns ``Unknown`` or ``Li-on`` depending on if the rechargeable 
 *        battery is present.
 *
 *    * - ``type``
 *      - Always returns ``Battery``.
 *
 *    * - ``voltage_max_design``
 *      - Returns the nominal "full" battery voltage. The value returned
 *        depends on ``technology``.
 *
 *    * - ``voltage_min_design``
 *      - Returns the nominal "empty" battery voltage. The value returned
 *        depends on ``technology``.
 *
 *    * - ``voltage_now``
 *      - Returns the battery voltage in microvolts.
 *
 * .. _power_supply: http://lxr.free-electrons.com/source/Documentation/power/power_supply_class.txt?v=4.4
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/platform_data/legoev3.h>
#include <linux/delay.h>
#include <linux/spinlock.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/hrtimer.h>

#include <mach/legoev3.h>

#include "legoev3_analog.h"

enum legoev3_battery_gpio {
	LEGOEV3_BATTERY_GPIO_ADC,
	LEGOEV3_BATTERY_GPIO_TYPE,
	NUM_LEGOEV3_BATTERY_GPIO
};

/*
 * struct legoev3_battery
 * @psy: power supply class data structure
 * @alg: pointer to A/DC device
 * @technology: li-ion or unknown (alkaline/NiMH/etc.)
 * @gpio: gpios for battery type switch and A/DC input
 * @max_V: max design voltage for given battery technology
 * @min_V: min design voltage for given battery technology
 */
struct legoev3_battery {
	struct power_supply_desc desc;
	struct power_supply *psy;
	struct legoev3_analog_device *alg;
	int technology;
	struct gpio gpio[NUM_LEGOEV3_BATTERY_GPIO];
	int max_V;
	int min_V;
};

int legoev3_battery_get_voltage(struct legoev3_battery *bat)
{
	/* formula from official LEGO firmware */
	return legoev3_analog_batt_volt_value(bat->alg) * 2000
		 + legoev3_analog_batt_curr_value(bat->alg) * 1000 / 15 + 50000;
}

int legoev3_battery_get_current(struct legoev3_battery *bat)
{
	/* formula from official LEGO firmware */
	return legoev3_analog_batt_curr_value(bat->alg) * 20000 / 15;
}

static int legoev3_battery_get_property(struct power_supply *psy,
					enum power_supply_property prop,
					union power_supply_propval *val)
{
	int ret = 0;
	struct legoev3_battery *bat =
		container_of(psy->desc, struct legoev3_battery, desc);

	switch (prop) {
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = bat->technology;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN:
		val->intval = bat->max_V;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN:
		val->intval = bat->min_V;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		ret = legoev3_battery_get_voltage(bat);
		if (WARN_ONCE(ret < 0, "Failed to get voltage (%d)\n", ret))
			break;
		val->intval = ret;
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		ret = legoev3_battery_get_current(bat);
		if (WARN_ONCE(ret < 0, "Failed to get current (%d)\n", ret))
			break;
		val->intval = ret;
		break;
	case POWER_SUPPLY_PROP_SCOPE:
		val->intval = POWER_SUPPLY_SCOPE_SYSTEM;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static enum power_supply_property legoev3_battery_props[] = {
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_SCOPE,
};

static int legoev3_battery_probe(struct platform_device *pdev)
{
	struct legoev3_battery *bat;
	struct legoev3_battery_platform_data *pdata;
	int ret;

	bat = devm_kzalloc(&pdev->dev, sizeof(struct legoev3_battery),
								GFP_KERNEL);
	if (!bat)
		return -ENOMEM;

	pdata = pdev->dev.platform_data;
	if (!pdata) {
		dev_err(&pdev->dev, "Platform data is required!\n");
		return -EINVAL;
	}

	bat->alg = get_legoev3_analog();
	if (IS_ERR(bat->alg)) {
		dev_err(&pdev->dev, "could not get analog device!\n");
		return PTR_ERR(bat->alg);
	}

	bat->desc.name = "legoev3-battery";
	bat->desc.type = POWER_SUPPLY_TYPE_BATTERY;
	bat->desc.properties = legoev3_battery_props;
	bat->desc.num_properties = ARRAY_SIZE(legoev3_battery_props);
	bat->desc.get_property = legoev3_battery_get_property;
	bat->desc.use_for_apm = 1;

	bat->gpio[LEGOEV3_BATTERY_GPIO_ADC].gpio = pdata->batt_adc_gpio;
	bat->gpio[LEGOEV3_BATTERY_GPIO_ADC].flags = GPIOF_OUT_INIT_HIGH;
	bat->gpio[LEGOEV3_BATTERY_GPIO_ADC].label = "EV3 battery to ADC";
	bat->gpio[LEGOEV3_BATTERY_GPIO_TYPE].gpio = pdata->batt_type_gpio;
	bat->gpio[LEGOEV3_BATTERY_GPIO_TYPE].flags = GPIOF_IN;
	bat->gpio[LEGOEV3_BATTERY_GPIO_TYPE].label = "EV3 battery type indicator";

	ret = gpio_request_array(bat->gpio, NUM_LEGOEV3_BATTERY_GPIO);
	if (ret < 0)
		goto gpio_request_gpio_request_array;

	ret = gpio_get_value(bat->gpio[LEGOEV3_BATTERY_GPIO_TYPE].gpio);
	if (ret < 0)
		goto gpio_get_value_fail;
	else if (ret) {
		/* average AA alkaline battery */
		bat->max_V = 7500000;
		bat->min_V = 6200000;
	} else {
		/* the official LEGO rechargable battery */
		bat->technology = POWER_SUPPLY_TECHNOLOGY_LION;
		bat->max_V = 7500000;
		bat->min_V = 7100000;
	}

	platform_set_drvdata(pdev, bat);

	bat->psy = devm_power_supply_register(&pdev->dev, &bat->desc, NULL);
	if (IS_ERR(bat->psy)) {
		ret = PTR_ERR(bat->psy);
		goto power_supply_register_fail;
	}

	return 0;

power_supply_register_fail:
	platform_set_drvdata(pdev, NULL);
gpio_get_value_fail:
	gpio_set_value(bat->gpio[LEGOEV3_BATTERY_GPIO_ADC].gpio, 0);
	gpio_free_array(bat->gpio, NUM_LEGOEV3_BATTERY_GPIO);
gpio_request_gpio_request_array:
	put_legoev3_analog(bat->alg);

	return ret;
}

static int legoev3_battery_remove(struct platform_device *pdev)
{
	struct legoev3_battery *bat= platform_get_drvdata(pdev);

	platform_set_drvdata(pdev, NULL);
	gpio_set_value(bat->gpio[LEGOEV3_BATTERY_GPIO_ADC].gpio, 0);
	gpio_free_array(bat->gpio, NUM_LEGOEV3_BATTERY_GPIO);
	put_legoev3_analog(bat->alg);

	return 0;
}

static struct platform_driver legoev3_battery_driver = {
	.driver.name	= "legoev3-battery",
	.driver.owner	= THIS_MODULE,
	.probe		= legoev3_battery_probe,
	.remove		= legoev3_battery_remove,
};
module_platform_driver(legoev3_battery_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("David Lechner");
MODULE_DESCRIPTION("LEGO Mindstorms EV3 battery driver");
MODULE_ALIAS("platform:legoev3-battery");
