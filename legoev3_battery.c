/*
 * Battery driver for the LEGO Mindstorms EV3
 *
 * Copyright (c) 2013 David Lechner <david@lechnology.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.

 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/power/legoev3_battery.h>
#include <linux/legoev3/legoev3_analog.h>
#include <linux/delay.h>
#include <linux/spinlock.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>

#include <mach/legoev3.h>

enum legoev3_battery_gpio {
	LEGOEV3_BATTERY_GPIO_ADC,
	LEGOEV3_BATTERY_GPIO_TYPE,
	NUM_LEGOEV3_BATTERY_GPIO
};

struct legoev3_battery {
	struct power_supply psy;
	struct legoev3_analog_device *alg;
	int status;
	int technology;
	struct gpio gpio[NUM_LEGOEV3_BATTERY_GPIO];
	int v_max;
	int v_min;
};

static int legoev3_battery_get_property(struct power_supply *psy,
					enum power_supply_property prop,
					union power_supply_propval *val)
{
	int ret = 0;
	struct legoev3_battery *bat = container_of(psy, struct legoev3_battery,
						   psy);

	switch (prop) {
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = bat->status;
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = 1;
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = bat->technology;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN:
		val->intval = bat->v_max;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN:
		val->intval = bat->v_min;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		ret = legoev3_analog_batt_volt_value(bat->alg) * 2000
		      + legoev3_analog_batt_curr_value(bat->alg) * 1000 / 15
		      + 50000;
		if (ret < 0)
			break;
		val->intval = ret;
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		ret = legoev3_analog_batt_curr_value(bat->alg) * 20000 / 15;
		if (ret < 0)
			break;
		val->intval = ret;
		break;
	case POWER_SUPPLY_PROP_SCOPE:
		val->intval = POWER_SUPPLY_SCOPE_SYSTEM;
		break;
	default:
		ret = -EINVAL;
		break;
	}
	return ret;
}

static enum power_supply_property legoev3_battery_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_SCOPE,
};

static int __devinit legoev3_battery_probe(struct platform_device *pdev)
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
		ret = -EINVAL;
		goto no_platform_data;
	}

	bat->alg = get_legoev3_analog();
	if (IS_ERR(bat->alg)) {
		dev_err(&pdev->dev, "could not get analog device!\n");
		ret = PTR_ERR(bat->alg);
		goto no_analog_device;
	}

	bat->status = POWER_SUPPLY_STATUS_DISCHARGING;
	bat->psy.name = "legoev3-battery";
	bat->psy.type = POWER_SUPPLY_TYPE_BATTERY;
	bat->psy.properties = legoev3_battery_props;
	bat->psy.num_properties = ARRAY_SIZE(legoev3_battery_props);
	bat->psy.get_property = legoev3_battery_get_property;
	bat->psy.use_for_apm = 1;

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
		bat->v_max = 7500000;
		bat->v_min = 6200000;
	} else {
		bat->technology = POWER_SUPPLY_TECHNOLOGY_LION;
		bat->v_max = 7500000;
		bat->v_min = 7100000;
	}

	ret = power_supply_register(&pdev->dev, &bat->psy);
	if (ret)
		goto power_supply_register_fail;

	platform_set_drvdata(pdev, bat);

	return 0;

power_supply_register_fail:
gpio_get_value_fail:
	gpio_set_value(bat->gpio[LEGOEV3_BATTERY_GPIO_ADC].gpio, 0);
	gpio_free_array(bat->gpio, NUM_LEGOEV3_BATTERY_GPIO);
gpio_request_gpio_request_array:
	put_legoev3_analog(bat->alg);
no_analog_device:
no_platform_data:
	devm_kfree(&pdev->dev, bat);
	return ret;
}

static int __devexit legoev3_battery_remove(struct platform_device *pdev)
{
	struct legoev3_battery *bat= platform_get_drvdata(pdev);

	platform_set_drvdata(pdev, NULL);
	power_supply_unregister(&bat->psy);
	gpio_set_value(bat->gpio[LEGOEV3_BATTERY_GPIO_ADC].gpio, 0);
	gpio_free_array(bat->gpio, NUM_LEGOEV3_BATTERY_GPIO);
	put_legoev3_analog(bat->alg);
	devm_kfree(&pdev->dev, bat);
	return 0;
}

static struct platform_driver legoev3_battery_driver = {
	.driver.name	= "legoev3-battery",
	.driver.owner	= THIS_MODULE,
	.probe		= legoev3_battery_probe,
	.remove		= __devexit_p(legoev3_battery_remove),
};
module_platform_driver(legoev3_battery_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("David Lechner");
MODULE_DESCRIPTION("LEGO Mindstorms EV3 battery driver");
MODULE_ALIAS("platform:legoev3-battery");
