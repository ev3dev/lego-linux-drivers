/*
 * Battery driver for the LEGO MINDSTORMS EV3
 *
 * Copyright (c) 2013,2017 David Lechner <david@lechnology.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.

 */

#include <linux/err.h>
#include <linux/gpio/consumer.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/power_supply.h>
#include <linux/slab.h>

struct lms2012_battery {
	struct power_supply_desc desc;
	struct power_supply *psy;
	struct gpio_desc *adc_gpio;
	int (*get_volts)(void *);
	int (*get_amps)(void *);
	void *context;
	int technology;
	int max_V;
	int min_V;
};

int lms2012_battery_get_voltage(struct lms2012_battery *bat)
{
	/* formula from official LEGO firmware */
	return bat->get_volts(bat->context) * 2000
		 + bat->get_amps(bat->context) * 1000 / 15 + 50000;
}

int lms2012_battery_get_current(struct lms2012_battery *bat)
{
	/* formula from official LEGO firmware */
	return bat->get_amps(bat->context) * 20000 / 15;
}

static int lms2012_battery_get_property(struct power_supply *psy,
					enum power_supply_property prop,
					union power_supply_propval *val)
{
	int ret = 0;
	struct lms2012_battery *bat =
		container_of(psy->desc, struct lms2012_battery, desc);

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
		ret = lms2012_battery_get_voltage(bat);
		if (WARN_ONCE(ret < 0, "Failed to get voltage (%d)\n", ret))
			break;
		val->intval = ret;
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		ret = lms2012_battery_get_current(bat);
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

static enum power_supply_property lms2012_battery_props[] = {
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_SCOPE,
};

struct lms2012_battery *lms2012_battery_probe(struct device *parent,
					      struct gpio_desc *adc_gpio,
					      struct gpio_desc *type_gpio,
					      int (*get_volts)(void *),
					      int (*get_amps)(void *),
					      void *context)
{
	struct lms2012_battery *bat;
	int ret;

	if (!parent || !adc_gpio || !type_gpio || !get_volts || !get_amps)
		return ERR_PTR(-EINVAL);

	bat = devm_kzalloc(parent, sizeof(*bat), GFP_KERNEL);
	if (!bat)
		return ERR_PTR(-ENOMEM);

	bat->adc_gpio = adc_gpio;
	bat->get_volts = get_volts;
	bat->get_amps = get_amps;
	bat->context = context;

	bat->desc.name = "lms2012-battery";
	bat->desc.type = POWER_SUPPLY_TYPE_BATTERY;
	bat->desc.properties = lms2012_battery_props;
	bat->desc.num_properties = ARRAY_SIZE(lms2012_battery_props);
	bat->desc.get_property = lms2012_battery_get_property;
	bat->desc.use_for_apm = 1;

	gpiod_direction_output(adc_gpio, 1);
	gpiod_direction_input(type_gpio);

	ret = gpiod_get_value(type_gpio);
	if (ret < 0) {
		goto gpio_get_value_fail;
	} else if (ret) {
		/* average AA alkaline battery */
		bat->max_V = 7500000;
		bat->min_V = 6200000;
	} else {
		/* the official LEGO rechargable battery */
		bat->technology = POWER_SUPPLY_TECHNOLOGY_LION;
		bat->max_V = 7500000;
		bat->min_V = 7100000;
	}

	bat->psy = devm_power_supply_register(parent, &bat->desc, NULL);
	ret = PTR_ERR_OR_ZERO(bat->psy);
	if (ret)
		goto power_supply_register_fail;

	return bat;

power_supply_register_fail:
gpio_get_value_fail:
	gpiod_set_value(bat->adc_gpio, 0);

	return ERR_PTR(ret);
}
EXPORT_SYMBOL_GPL(lms2012_battery_probe);

void lms2012_battery_remove(struct lms2012_battery *bat)
{
	gpiod_set_value(bat->adc_gpio, 0);
}
EXPORT_SYMBOL_GPL(lms2012_battery_remove);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("David Lechner");
MODULE_DESCRIPTION("lsm2012-compat battery driver");
