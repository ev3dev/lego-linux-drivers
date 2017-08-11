/*
 * pistorms_leds.c
 *
 * LED driver for mindsensors.com PiStorms
 *
 * Copyright (C) 2015 David Lechner <david@lechnology.com>
 *
 * based on leds-pwm.c by Luotao Fu @ Pengutronix (l.fu@pengutronix.de)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

/**
 * DOC: userspace
 *
 * The ``pistorms`` module provides six LED devices for controlling the LEDs
 * on the Pistorms. There are two tri-color LEDs. Each component (red, green,
 * blue) is controlled separately.
 *
 * The device nodes can be found at ``/sys/class/leds/pistorms:B<bank>:<color>:brick-status``
 * where ``<bank>>`` is ``A`` or ``B`` and ``<color>`` is ``red``, ``green`` or
 * ``blue``.
 *
 * .. note:: Some models of PiStorms only have one LED. The driver will still
 *    provide controls for two, but only one will actually work.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/leds.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/workqueue.h>

#include "pistorms.h"

#define PISTORMS_LED_REG	0xD7

enum pistorms_leds {
	PISTORMS_RED_LED,
	PISTORMS_GREEN_LED,
	PISTORMS_BLUE_LED,
	PISTORMS_NUM_LEDS
};

static const char * const pistorms_led_color_names[PISTORMS_NUM_LEDS] = {
	[PISTORMS_RED_LED]	= "red",
	[PISTORMS_GREEN_LED]	= "green",
	[PISTORMS_BLUE_LED]	= "blue",
};

struct pistorms_led_group;

struct pistorms_led_data {
	char				name[PISTORMS_NAME_SIZE];
	struct led_classdev		cdev;
	struct delayed_work		work;
	struct pistorms_led_group	*group;
	enum pistorms_leds		index;
};

struct pistorms_led_group {
	struct i2c_client		*client;
	int				num_leds;
	struct pistorms_led_data	leds[PISTORMS_NUM_LEDS];
	u8				brightness[PISTORMS_NUM_LEDS];
};

/*
 * The PiStorms LEDs only change when the last (blue) LED is written to. So, it
 * is easiest to just write all 3 values at once. We used delayed work so that
 * if all 3 LEDs are changed consecutively, hopefully it will only result in
 * one I2C message.
 */
static void pistorms_led_work(struct work_struct *work)
{
	struct delayed_work *delayed_work =
		container_of(work, struct delayed_work, work);
	struct pistorms_led_data *led_data =
		container_of(delayed_work, struct pistorms_led_data, work);
	struct pistorms_led_group *group = led_data->group;

	i2c_smbus_write_i2c_block_data(group->client, PISTORMS_LED_REG,
				       group->num_leds, group->brightness);
}

static void pistorms_led_set(struct led_classdev *led_cdev,
	enum led_brightness brightness)
{
	struct pistorms_led_data *led_data =
		container_of(led_cdev, struct pistorms_led_data, cdev);
	struct pistorms_led_group *group = led_data->group;

	group->brightness[led_data->index] = brightness;

	schedule_delayed_work(&led_data->work, msecs_to_jiffies(1));
}

static enum led_brightness pistorms_led_get(struct led_classdev *led_cdev)
{
	struct pistorms_led_data *led_data =
		container_of(led_cdev, struct pistorms_led_data, cdev);
	struct pistorms_led_group *group = led_data->group;
	int ret;

	ret = i2c_smbus_read_byte_data(group->client,
				       PISTORMS_LED_REG + led_data->index);
	if (ret >= 0)
		group->brightness[led_data->index] = ret;

	return ret;
}

static void pistorms_led_cleanup(struct pistorms_led_group *group)
{
	while (group->num_leds--) {
		led_classdev_unregister(&group->leds[group->num_leds].cdev);
		cancel_delayed_work_sync(&group->leds[group->num_leds].work);
	}
}

static int pistorms_led_add(struct device *dev, struct pistorms_led_group *group,
			    const char *parent_name)
{
	int index = group->num_leds;
	struct pistorms_led_data *led_data = &group->leds[index];
	int ret;

	snprintf(led_data->name, PISTORMS_NAME_SIZE, "%s:%s:brick-status",
	         parent_name, pistorms_led_color_names[index]);
	led_data->cdev.name = led_data->name;
	led_data->cdev.brightness_set = pistorms_led_set;
	led_data->cdev.brightness_get = pistorms_led_get;
	led_data->cdev.max_brightness = 255;

	INIT_DELAYED_WORK(&led_data->work, pistorms_led_work);
	led_data->group = group;
	led_data->index = index;

	ret = led_classdev_register(dev, &led_data->cdev);
	if (ret == 0)
		group->num_leds++;
	else
		dev_err(dev, "Failed to register PiStorms LED for %s: %d\n",
			led_data->name, ret);

	return ret;
}

int pistorms_leds_register(struct pistorms_data *data)
{
	struct pistorms_led_group *group;
	int i;
	int ret = 0;

	group = devm_kzalloc(&data->client->dev, sizeof(struct pistorms_led_group),
			     GFP_KERNEL);
	if (!group)
		return -ENOMEM;

	group->client = data->client;
	for (i = 0; i < PISTORMS_NUM_LEDS; i++) {
		ret = pistorms_led_add(&data->client->dev, group, data->name);
		if (ret)
			break;
	}

	if (ret) {
		pistorms_led_cleanup(group);
		return ret;
	}

	data->leds_data = group;

	return 0;
}

void pistorms_leds_unregister(struct pistorms_data *data)
{
	struct pistorms_led_group *group = data->leds_data;
	pistorms_led_cleanup(group);
	data->leds_data = NULL;
}
