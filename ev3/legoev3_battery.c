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
#include <linux/hrtimer.h>

#include <mach/legoev3.h>

#define LEGOEV3_BATTERY_CHARGE_TIME (NSEC_PER_SEC / 10)

enum legoev3_battery_gpio {
	LEGOEV3_BATTERY_GPIO_ADC,
	LEGOEV3_BATTERY_GPIO_TYPE,
	NUM_LEGOEV3_BATTERY_GPIO
};

/*
 * struct legoev3_battery
 * @psy: power supply class data structure
 * @alg: pointer to A/DC device
 * @status: charging/discharging
 * @technology: li-ion or unknown (alkaline/NiMH/etc.)
 * @gpio: gpios for battery type switch and A/DC input
 * @max_V: max design voltage for given battery technology
 * @min_V: min design voltage for given battery technology
 * @capacity_mAh: rated capacity of battery in mAh
 * @capacity_full_V: voltage level that indicates the bater is 100% full
 * @current_charge_mAh: the estimated charge remaining in mAh
 * @charge_timer: timer used to integrate current over time to estimate charge
 */
struct legoev3_battery {
	struct power_supply psy;
	struct legoev3_analog_device *alg;
	int status;
	int technology;
	struct gpio gpio[NUM_LEGOEV3_BATTERY_GPIO];
	int max_V;
	int min_V;
	int capacity_mAh;
	int capacity_full_V;
	int current_charge_mAh;
	struct hrtimer charge_timer;
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

static enum hrtimer_restart
legoev3_battery_charge_timer_handler(struct hrtimer *timer)
{
	struct legoev3_battery *bat =
		container_of(timer, struct legoev3_battery, charge_timer);
	long change;
	int ret;

	change = hrtimer_forward_now(&bat->charge_timer,
				ktime_set(0, LEGOEV3_BATTERY_CHARGE_TIME));
return HRTIMER_RESTART;
	ret = legoev3_battery_get_voltage(bat);
	if (ret > bat->capacity_full_V)
		bat->current_charge_mAh = bat->capacity_mAh;
	else {
		ret = legoev3_battery_get_current(bat) / 1000;
		if (ret > 0) {
			change *= ret;
			change *= LEGOEV3_BATTERY_CHARGE_TIME;
			change *= 3600;
			change /= 1000000;
			bat->current_charge_mAh -= change;
		}
	}
	return HRTIMER_RESTART;
}

static int legoev3_battery_get_property(struct power_supply *psy,
					enum power_supply_property prop,
					union power_supply_propval *val)
{
	int ret = 0;
	struct legoev3_battery *bat =
		container_of(psy, struct legoev3_battery, psy);

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
		val->intval = bat->max_V;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN:
		val->intval = bat->min_V;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		ret = legoev3_battery_get_voltage(bat);
		if (ret < 0)
			break;
		val->intval = ret;
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		ret = legoev3_battery_get_current(bat);
		if (ret < 0)
			break;
		val->intval = ret;
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
		val->intval = bat->capacity_mAh;
		break;
	case POWER_SUPPLY_PROP_CHARGE_NOW:
		val->intval = bat->current_charge_mAh;
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
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_CHARGE_NOW,
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
		/* average AA alkaline battery */
		bat->max_V = 7500000;
		bat->min_V = 6200000;
		bat->capacity_mAh = 2000000; /* just guessing on capacity */
	} else {
		/* the official LEGO rechargable battery */
		bat->technology = POWER_SUPPLY_TECHNOLOGY_LION;
		bat->max_V = 7500000;
		bat->min_V = 7100000;
		/* LEGO education website says 2050mAh, schematic says 2200mAh */
		bat->capacity_mAh = 2050000;
	}
	/*
	 * according to the graphs in the hardware development kit, both types
	 * of batteries spike at around 8V.
	 */
	bat->capacity_full_V = 8000000;
	/* try to estimate inital battery capacity remaining */
	bat->current_charge_mAh = bat->capacity_mAh;
	// ret = legoev3_battery_get_voltage(bat);
	// if (ret < bat->min_V)
	// 	bat->current_charge_mAh = 0;
	// else if (ret < bat->max_V)
	// 	bat->current_charge_mAh *= (ret - bat->min_V)
	// 		/ (bat->max_V - bat->min_V);

	ret = power_supply_register(&pdev->dev, &bat->psy);
	if (ret)
		goto power_supply_register_fail;

	platform_set_drvdata(pdev, bat);
	hrtimer_init(&bat->charge_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	bat->charge_timer.function = legoev3_battery_charge_timer_handler;
	hrtimer_start(&bat->charge_timer,
		 ktime_set(0, LEGOEV3_BATTERY_CHARGE_TIME), HRTIMER_MODE_REL);
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

	hrtimer_cancel(&bat->charge_timer);
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
