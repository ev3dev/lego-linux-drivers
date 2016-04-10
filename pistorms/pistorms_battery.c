/*
 * Battery driver for mindsensors.com PiStorms
 *
 * Copyright (c) 2015-2016 David Lechner <david@lechnology.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.

 */

#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/power_supply.h>

#include "pistorms.h"

#define PISTORMS_BATTERY_REG 0x6E

/*
 * struct pistorms_battery
 * @psy: power supply class data structure
 */
struct pistorms_battery {
	struct i2c_client *client;
	struct power_supply *psy;
	struct power_supply_desc desc;
};

static int pistorms_battery_get_property(struct power_supply *psy,
					 enum power_supply_property prop,
					 union power_supply_propval *val)
{
	struct pistorms_battery *bat =
		container_of(psy->desc, struct pistorms_battery, desc);
	int ret = 0;

	switch (prop) {
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		ret = i2c_smbus_read_byte_data(bat->client, PISTORMS_BATTERY_REG);
		if (WARN_ONCE(ret < 0, "Failed to read voltage"))
			break;
		val->intval = ret * 40000; /* convert to microvolts */
		break;
	case POWER_SUPPLY_PROP_SCOPE:
		val->intval = POWER_SUPPLY_SCOPE_SYSTEM;
		break;
	default:
		ret = -EINVAL;
		break;
	}

	return 0;
}

static enum power_supply_property pistorms_battery_props[] = {
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_SCOPE,
};

int pistorms_battery_register(struct pistorms_data *data)
{
	struct pistorms_battery *bat;

	bat = devm_kzalloc(&data->client->dev, sizeof(struct pistorms_battery),
			   GFP_KERNEL);
	if (!bat)
		return -ENOMEM;

	bat->client = data->client;

	bat->desc.name = "pistorms-battery";
	bat->desc.type = POWER_SUPPLY_TYPE_BATTERY;
	bat->desc.properties = pistorms_battery_props;
	bat->desc.num_properties = ARRAY_SIZE(pistorms_battery_props);
	bat->desc.get_property = pistorms_battery_get_property;

	data->battery_data = bat;

	bat->psy = power_supply_register(&data->client->dev, &bat->desc, NULL);
	if (IS_ERR(bat->psy)) {
		data->battery_data = NULL;
		devm_kfree(&data->client->dev, bat);
		return PTR_ERR(bat->psy);
	}

	return 0;
}

void pistorms_battery_unregister(struct pistorms_data *data)
{
	struct pistorms_battery *bat = data->battery_data;

	if (!bat)
		return;

	power_supply_unregister(bat->psy);
	data->battery_data = NULL;
}
