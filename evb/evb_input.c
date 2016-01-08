/*
 * Input driver for FatcatLab's EVB
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
#include <linux/input.h>
#include <linux/input-polldev.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

/* Raw values when key is pressed (12-bit, 1.8V analog input) */
#define RAW_UP		 511 /* 33kohm */
#define RAW_ENTER	 848 /* 18kohm */
#define RAW_DOWN	1310 /* 10kohm */
#define RAW_RIGHT	2048 /* 4.7kohm */
#define RAW_LEFT	2912 /* 1.91kohm */
#define RAW_BACK	3377 /* 1kohm */

#define RAW_PLUS_MINUS	  10 /* Allowable range +/- actual value */

#define MIN(x) ((x)-RAW_PLUS_MINUS)
#define MAX(x) ((x)+RAW_PLUS_MINUS)

#define NO_KEY (-1)
#define DEBOUNCE 2

struct evb_input_data {
	struct iio_channel *iio;
	struct input_polled_dev *ipd;
	int last_value;
	int debounce;
};

static void evb_input_poll(struct input_polled_dev *ipd)
{
	struct evb_input_data *data = ipd->private;
	int ret, raw_value;
	int new_value = NO_KEY;

	ret = iio_read_channel_raw(data->iio, &raw_value);
	if (ret < 0)
		return;

	/* For now, we aren't handling simultaneous presses */
	if (raw_value > MIN(RAW_UP) && raw_value < MAX(RAW_UP))
		new_value = KEY_UP;
	else if (raw_value > MIN(RAW_ENTER) && raw_value < MAX(RAW_ENTER))
		new_value = KEY_ENTER;
	else if (raw_value > MIN(RAW_DOWN) && raw_value < MAX(RAW_DOWN))
		new_value = KEY_DOWN;
	else if (raw_value > MIN(RAW_RIGHT) && raw_value < MAX(RAW_RIGHT))
		new_value = KEY_RIGHT;
	else if (raw_value > MIN(RAW_LEFT-100) && raw_value < MAX(RAW_LEFT))
		/* Left seems a bit touchy, so giving it extra wiggle room */
		new_value = KEY_LEFT;
	else if (raw_value > MIN(RAW_BACK) && raw_value < MAX(RAW_BACK))
		new_value = KEY_BACKSPACE;
	else
		new_value = NO_KEY;

	if (new_value == data->last_value) {
		data->debounce = 0;
		return;
	}

	/* TODO: make debounce function of poll period */
	if (data->debounce++ < DEBOUNCE)
		return;

	if (new_value == NO_KEY)
		input_report_key(data->ipd->input, data->last_value, 0);
	else
		input_report_key(data->ipd->input, new_value, 1);
	input_sync(data->ipd->input);

	data->last_value = new_value;
}

static int evb_input_probe(struct platform_device *pdev)
{
	struct evb_input_data *data;
	struct input_dev *idev;
	int ret;

	data = devm_kzalloc(&pdev->dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->ipd = devm_input_allocate_polled_device(&pdev->dev);
	if (!data->ipd)
		return -ENOMEM;

	platform_set_drvdata(pdev, data);

	data->iio = iio_channel_get(&pdev->dev, "voltage");
	if (IS_ERR(data->iio)) {
		dev_err(&pdev->dev, "Failed to get iio channel");
		return PTR_ERR(data->iio);
	}

	data->last_value = NO_KEY;

	data->ipd->private = data;
	data->ipd->poll = evb_input_poll;
	data->ipd->poll_interval = 20; /* msec */

	idev = data->ipd->input;
	idev->name = "evb-input";
	idev->phys = "evb";
	set_bit(EV_KEY, idev->evbit);
	set_bit(KEY_BACKSPACE, idev->keybit);
	set_bit(KEY_ENTER, idev->keybit);
	set_bit(KEY_UP, idev->keybit);
	set_bit(KEY_LEFT, idev->keybit);
	set_bit(KEY_RIGHT, idev->keybit);
	set_bit(KEY_DOWN, idev->keybit);

	ret = input_register_polled_device(data->ipd);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to register input device\n");
		iio_channel_release(data->iio);
		return ret;
	}

	return 0;
}

static int evb_input_remove(struct platform_device *pdev)
{
	struct evb_input_data *data = platform_get_drvdata(pdev);

	input_unregister_polled_device(data->ipd);
	iio_channel_release(data->iio);

	return 0;
}

static const struct of_device_id of_evb_input_match[] = {
	{ .compatible = "ev3dev,evb-buttons", },
	{ .compatible = "ev3dev,evb-input", },
	{ }
};
MODULE_DEVICE_TABLE(of, of_evb_input_match);

static struct platform_driver evb_input_driver = {
	.driver = {
		.name       = "evb-input",
		.of_match_table = of_evb_input_match,
	},
	.probe  = evb_input_probe,
	.remove = evb_input_remove,
};
module_platform_driver(evb_input_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("David Lechner <david@lechnology.com>");
MODULE_DESCRIPTION("FatcatLab EVB Input Driver");
MODULE_ALIAS("platform:evb-input");
