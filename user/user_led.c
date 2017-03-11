/*
 * User-defined LEDs
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

/**
 * DOC: userspace
 *
 * The ``user-led`` module provides a user-defined LED interface. This module
 * is deprecated.
 */

#include <linux/device.h>
#include <linux/fs.h>
#include <linux/module.h>
#include <linux/uaccess.h>

#include "user_led.h"

#define DEVICE_NAME "user_led"

static int major_dev_num;

#define to_user_led(_dev) container_of(_dev, struct user_led, led_cdev)

static void user_led_brightness_set(struct led_classdev *led_cdev,
				    enum led_brightness brightness)
{
	struct user_led *led = to_user_led(led_cdev);

	led->led_cdev.brightness = brightness;
	/* TODO: add support for poll() and notify here */
}

int user_led_register(struct user_led *led, struct device *parent)
{
	int err;

	if (WARN_ON(!parent))
		return -EINVAL;

	memset(&led->led_cdev, 0, sizeof(led->led_cdev));
	led->led_cdev.name = led->name;
	led->led_cdev.max_brightness = LED_FULL;
	led->led_cdev.brightness_set = user_led_brightness_set;

	err = led_classdev_register(parent, &led->led_cdev);
	if (err < 0)
		return err;

	memset(&led->cdev, 0, sizeof(led->cdev));
	/* TODO: register character device here */

	dev_info(parent, "Registered '%s'\n", led->name);

	return 0;
}
EXPORT_SYMBOL_GPL(user_led_register);

void user_led_unregister(struct user_led *led)
{
	struct device *parent = led->led_cdev.dev->parent;

	led_classdev_unregister(&led->led_cdev);
	dev_info(parent, "Unregistered '%s'\n", led->name);
}
EXPORT_SYMBOL_GPL(user_led_unregister);

static ssize_t user_led_read(struct file *file, char __user *buffer,
			     size_t count, loff_t *ptr)
{
	struct user_led *led = file->private_data;

	if (!count)
		return 0;

	count = min_t(size_t, count, sizeof(enum led_brightness));
	count -= copy_to_user(buffer, &led->led_cdev.brightness, count);

	return count;
}

static int user_led_open(struct inode *inode, struct file *file)
{
	struct user_led *led =
		container_of(inode->i_cdev, struct user_led, cdev);

	file->private_data = led;

	return 0;
}

static int user_led_release(struct inode *inode, struct file *file)
{
	file->private_data = NULL;

	return 0;
}

static const struct file_operations user_led_fops = {
	.read		= user_led_read,
	.open		= user_led_open,
	.release	= user_led_release,
};

static int __init user_led_init(void)
{
	major_dev_num = register_chrdev(0, DEVICE_NAME, &user_led_fops);
	if (major_dev_num < 0) {
		pr_err("Failed to register user_led chrdev\n");
		return major_dev_num;
	}

	return 0;
}
module_init(user_led_init);

static void __exit user_led_exit(void)
{
	unregister_chrdev(major_dev_num, DEVICE_NAME);
}
module_exit(user_led_exit);

MODULE_DESCRIPTION("User-defined LEGO led device class");
MODULE_AUTHOR("David Lechner <david@lechnology.com>");
MODULE_LICENSE("GPL");
