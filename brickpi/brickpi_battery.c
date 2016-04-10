/*
 * Battery driver for Dexter Industries BrickPi+
 *
 * Copyright (c) 2015-2016 David Lechner <david@lechnology.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

/*
 * Note: The comment block below is used to generate docs on the ev3dev website.
 * Use kramdown (markdown) syntax. Use a '.' as a placeholder when blank lines
 * or leading whitespace is important for the markdown syntax.
 */

/**
 * DOC: website
 *
 * Dexter Industries BrickPi+ Battery Driver
 *
 * A `brickpi-battery` is available to monitor battery voltage on the BrickPi+.
 * (Not available on the original BrickPi.) Use the `voltage_now` attribute to
 * read the instantaneous battery voltage.
 */

#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/power_supply.h>

struct brickpi_battery {
	struct i2c_client *client;
	struct power_supply *psy;
	struct power_supply_desc desc;
};

/* credit: this function copied from mcp3021_read16 in mcp3021.c */
static int brickpi_battery_read16(struct i2c_client *client)
{
	int ret;
	u16 reg;
	__be16 buf;

	ret = i2c_master_recv(client, (char *)&buf, 2);
	if (ret < 0)
		return ret;
	if (ret != 2)
		return -EIO;

	/* The output code of the MCP3021 is transmitted with MSB first. */
	reg = be16_to_cpu(buf);

	/*
	 * The ten-bit output code is composed of the lower 4-bit of the
	 * first byte and the upper 6-bit of the second byte.
	 */
	reg = (reg >> 2) & 0x3ff;

	return reg;
}

static int brickpi_battery_get_property(struct power_supply *psy,
					enum power_supply_property prop,
					union power_supply_propval *val)
{
	struct brickpi_battery *bat =
		container_of(psy->desc, struct brickpi_battery, desc);
	int ret = 0;

	switch (prop) {
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		ret = brickpi_battery_read16(bat->client);
		if (WARN_ONCE(ret < 0, "Failed to read voltage"))
			break;
		/*
		 * Converting to microvolts. The chip has a 3.3V reference and
		 * there is a 45k/10k voltage divider, so ret == 1024 => 18.15V.
		 */
		val->intval = ret * 17725;
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

static enum power_supply_property brickpi_battery_props[] = {
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_SCOPE,
};

static int brickpi_battery_probe(struct i2c_client *client,
				 const struct i2c_device_id *id)
{
	struct brickpi_battery *bat;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
		return -ENODEV;

	bat = devm_kzalloc(&client->dev, sizeof(struct brickpi_battery),
			   GFP_KERNEL);
	if (!bat)
		return -ENOMEM;

	i2c_set_clientdata(client, bat);
	bat->client = client;

	bat->desc.name = "brickpi-battery";
	bat->desc.type = POWER_SUPPLY_TYPE_BATTERY;
	bat->desc.properties = brickpi_battery_props;
	bat->desc.num_properties = ARRAY_SIZE(brickpi_battery_props);
	bat->desc.get_property = brickpi_battery_get_property;

	bat->psy = power_supply_register(&client->dev, &bat->desc, NULL);
	if (IS_ERR(bat->psy)) {
		i2c_set_clientdata(client, NULL);
		devm_kfree(&client->dev, bat);
		return PTR_ERR(bat->psy);
	}

	return 0;
}

static int brickpi_battery_remove(struct i2c_client *client)
{
	struct brickpi_battery *bat = i2c_get_clientdata(client);

	power_supply_unregister(bat->psy);
	i2c_set_clientdata(client, NULL);

	return 0;
}

static const struct i2c_device_id brickpi_battery_id_table[] = {
	{ "brickpi-battery" },
	{ }
};
MODULE_DEVICE_TABLE(i2c, brickpi_battery_id_table);

#ifdef CONFIG_OF
static const struct of_device_id brickpi_battery_of_match[] = {
	{ .compatible = "dexterind,brickpi-battery" },
	{ }
};
MODULE_DEVICE_TABLE(of, brickpi_battery_of_match);
#endif

static struct i2c_driver brickpi_battery_driver = {
	.driver = {
		.name	= "brickpi-battery",
	},
	.id_table	= brickpi_battery_id_table,
	.probe		= brickpi_battery_probe,
	.remove		= brickpi_battery_remove,
};
module_i2c_driver(brickpi_battery_driver);

MODULE_DESCRIPTION("Dexter Industries BrickPi+ battery driver");
MODULE_AUTHOR("David Lechner <david@lechnology.com>");
MODULE_LICENSE("GPL");
