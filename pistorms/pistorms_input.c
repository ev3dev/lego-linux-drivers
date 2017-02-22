/*
 * mindsensors.com PiStorms input device driver
 *
 * Copyright (C) 2015 David Lechner <david@lechnology.com>
 *
 * based on /drivers/input/keyboard/gpio_keys_polled.c
 *
 *  Copyright (C) 2007-2010 Gabor Juhos <juhosg@openwrt.org>
 *  Copyright (C) 2010 Nuno Goncalves <nunojpg@gmail.com>
 *
 *  This file was based on: /drivers/input/misc/cobalt_btns.c
 *  Copyright (C) 2007 Yoichi Yuasa <yoichi_yuasa@tripeaks.co.jp>
 *
 *  also was based on: /drivers/input/keyboard/gpio_keys.c
 *  Copyright 2005 Phil Blundell
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */

/**
 * DOC: userspace
 *
 * The ``pistorms`` module registers an input device node for the touchscreen
 * an the GO button on the PiStorms.
 *
 * The touchscreen provides ``ABS_X`` and ``ABS_Y`` events to indicate where the
 * screen was touched and ``BTN_TOUCH`` to indicate when a touch event starts
 * and stops.
 *
 * The GO button is mapped to ``KEY_ENTER``.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/input-polldev.h>
#include <linux/ioport.h>
#include <linux/property.h>

#include "pistorms.h"

/*
 * When the text on the PiStorms is right side up, the screen is technically
 * rotated 90 degrees, which is why the X and Y values look backwards here.
 */
#define PISTORMS_INPUT_TOUCH_MAX_X	240
#define PISTORMS_INPUT_TOUCH_MAX_Y	320
#define PISTORMS_INPUT_TOUCH_FUZZ	5

#define PISTORMS_INPUT_BUTTON_REG	0xDA
#define PISTORMS_INPUT_BUTTON_LEN	2
#define PISTORMS_INPUT_TOUCH_REG	0xE3
#define PISTORMS_INPUT_TOUCH_LEN	4
#define PISTORMS_INPUT_POLL_MS		10
#define PISTORMS_INPUT_DEBOUNCE_COUNT	3

enum pistorms_keys {
	PISTORMS_KEY_GO,
	PISTORMS_KEY_POWER,
/* keys are divided into two groups since it takes two I2C messages to read them. */
#define PISTORMS_KEYS_FIRST_TOUCH PISTORMS_BTN_TOUCH
	PISTORMS_BTN_TOUCH,
	PISTORMS_ABS_X,
	PISTORMS_ABS_Y,
	NUM_PISTORMS_KEYS
};

/**
 * struct pistorms_input_button_data
 *
 * @type: The input type, e.g. EV_KEY or EV_ABS
 * @code: They key code
 * @value: The current value of the button
 * @last_value: The previous value of the button
 * @count: How long the button has been in a new value (only used for touch)
 */
struct pistorms_input_button_data {
	unsigned int type;
	unsigned int code;
	int value;
	int last_value;
	int count;
};

struct pistorms_input_dev {
	struct i2c_client *client;
	struct input_polled_dev *poll_dev;
	struct pistorms_input_button_data data[NUM_PISTORMS_KEYS];
};

static void pistorms_input_check_value(struct input_dev *input,
				       struct pistorms_input_button_data *bdata)
{
	if (bdata->value != bdata->last_value) {
		input_event(input, bdata->type, bdata->code, bdata->value);
		input_sync(input);
		bdata->count = 0;
		bdata->last_value = bdata->value;
	}
}

static void pistorms_input_abs_values(struct input_dev *input,
				      struct pistorms_input_button_data *x,
				      struct pistorms_input_button_data *y)
{
	input_event(input, x->type, x->code, x->value);
	input_event(input, y->type, y->code, y->value);
}

static void pistorms_input_poll(struct input_polled_dev *dev)
{
	struct pistorms_input_dev *bdev = dev->private;
	struct input_dev *input = dev->input;
	struct pistorms_input_button_data *touch_button;
	int err, i;
	u8 button_data[PISTORMS_INPUT_BUTTON_LEN];
	u8 touch_data[PISTORMS_INPUT_TOUCH_LEN];
	u16 x, y;

	err = i2c_smbus_read_i2c_block_data(bdev->client, PISTORMS_INPUT_BUTTON_REG,
					    PISTORMS_INPUT_BUTTON_LEN, button_data);
	if (err < 0)
		return;

	bdev->data[PISTORMS_KEY_GO].value = button_data[0] & 0x01;
	bdev->data[PISTORMS_KEY_POWER].value = button_data[1] == 253;

	for (i = 0; i < PISTORMS_KEYS_FIRST_TOUCH; i++)
		pistorms_input_check_value(input, &bdev->data[i]);

	err = i2c_smbus_read_i2c_block_data(bdev->client, PISTORMS_INPUT_TOUCH_REG,
					    PISTORMS_INPUT_TOUCH_LEN, touch_data);
	if (err < 0)
		return;

	/* Using standard BrickPi orientation, *_X and *_Y are swapped. */
	x = PISTORMS_INPUT_TOUCH_MAX_Y - le16_to_cpu(*(u16 *)touch_data);
	y = le16_to_cpu(*(u16 *)(touch_data + 2));
	touch_button = &bdev->data[PISTORMS_BTN_TOUCH];
	touch_button->value = x > 1 && y > 1;

	/* Filter out false touches */
	if (touch_button->value ^ touch_button->last_value)
		touch_button->count++;
	else
		touch_button->count = 0;

	if (!touch_button->value
	    || touch_button->count > PISTORMS_INPUT_DEBOUNCE_COUNT)
	{
		if (touch_button->value) {
			/* on initial touch, use actual values */
			bdev->data[PISTORMS_ABS_X].value = x;
			bdev->data[PISTORMS_ABS_Y].value = y;
			pistorms_input_abs_values(input,
						  &bdev->data[PISTORMS_ABS_X],
						  &bdev->data[PISTORMS_ABS_Y]);
		}
		pistorms_input_check_value(input, touch_button);
	} else if (touch_button->last_value) {
		/* after first initial touch, use low-pass filter */
		bdev->data[PISTORMS_ABS_X].value =
			((bdev->data[PISTORMS_ABS_X].value * 70)
			 + (x * 30)) / 100;
		bdev->data[PISTORMS_ABS_Y].value =
			((bdev->data[PISTORMS_ABS_Y].value * 70)
			 + (y * 30)) / 100;
		pistorms_input_abs_values(input, &bdev->data[PISTORMS_ABS_X],
					  &bdev->data[PISTORMS_ABS_Y]);
		input_sync(input);
	}
}

int pistorms_input_register(struct pistorms_data *data)
{
	struct pistorms_input_dev *bdev;
	struct input_polled_dev *poll_dev;
	struct input_dev *input;
	int error;
	int i;

	bdev = devm_kzalloc(&data->client->dev, sizeof(struct pistorms_input_dev),
			    GFP_KERNEL);
	if (!bdev) {
		dev_err(&data->client->dev, "no memory for private data\n");
		return -ENOMEM;
	}

	poll_dev = devm_input_allocate_polled_device(&data->client->dev);
	if (!poll_dev) {
		dev_err(&data->client->dev, "no memory for polled device\n");
		return -ENOMEM;
	}

	poll_dev->private = bdev;
	poll_dev->poll = pistorms_input_poll;
	poll_dev->poll_interval = PISTORMS_INPUT_POLL_MS;

	input = poll_dev->input;

	input->name = "PiStorms";
	input->phys = "pistorms/input0";

	input->id.bustype = BUS_I2C;

	__set_bit(INPUT_PROP_DIRECT, input->propbit);
	__set_bit(EV_KEY, input->evbit);
	__set_bit(EV_ABS, input->evbit);

	bdev->data[PISTORMS_KEY_GO].type = EV_KEY;
	bdev->data[PISTORMS_KEY_GO].code = KEY_ENTER;
	bdev->data[PISTORMS_KEY_POWER].type = EV_KEY;
	bdev->data[PISTORMS_KEY_POWER].code = KEY_POWER;
	bdev->data[PISTORMS_BTN_TOUCH].type = EV_KEY;
	bdev->data[PISTORMS_BTN_TOUCH].code = BTN_TOUCH;
	bdev->data[PISTORMS_ABS_X].type = EV_ABS;
	bdev->data[PISTORMS_ABS_X].code = ABS_X;
	bdev->data[PISTORMS_ABS_Y].type = EV_ABS;
	bdev->data[PISTORMS_ABS_Y].code = ABS_Y;

	for (i = 0; i < NUM_PISTORMS_KEYS; i++) {
		struct pistorms_input_button_data *bdata = &bdev->data[i];
		input_set_capability(input, bdata->type, bdata->code);
	}

	/*
	 * Starting with standard orientation on PiStorms. Swapped *_Y and *_X
	 * is not a mistake.
	 */
	input_set_abs_params(input, ABS_X, 0, PISTORMS_INPUT_TOUCH_MAX_Y,
			     PISTORMS_INPUT_TOUCH_FUZZ, 0);
	input_set_abs_params(input, ABS_Y, 0, PISTORMS_INPUT_TOUCH_MAX_X,
			     PISTORMS_INPUT_TOUCH_FUZZ, 0);

	bdev->poll_dev = poll_dev;
	bdev->client = data->client;
	data->input_data = bdev;

	error = input_register_polled_device(poll_dev);
	if (error) {
		data->input_data = NULL;
		dev_err(&data->client->dev,
			"unable to register polled device, err=%d\n", error);
		return error;
	}

	return 0;
}

void pistorms_input_unregister(struct pistorms_data *data)
{
	struct pistorms_input_dev *bdev = data->input_data;

	if (!bdev)
		return;
	input_unregister_polled_device(bdev->poll_dev);
	data->input_data = NULL;
}
