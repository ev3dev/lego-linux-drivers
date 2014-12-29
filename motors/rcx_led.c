/*
 * RCX/Power Functions LED device driver for LEGO MINDSTORMS EV3
 *
 * Copyright (C) 2014 David Lechner <david@lechnology.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.

 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

/*
 * Note: The comment block below is used to generate docs on the ev3dev website.
 * Use kramdown (markdown) format. Use a '.' as a placeholder when blank lines
 * or leading whitespace is important for the markdown syntax.
 */

/**
 * DOC: website
 *
 * RCX/Power Functions LED Driver
 *
 * This driver provides a [leds] interface for RCX motors, Power Functions
 * motors or any other 9V rated LED connected to an output port. You can find
 * the sysfs device at `/sys/bus/lego/devices/<port>:rcx-led` where `<port>`
 * is the the name of the output port the motor is connected to (e.g. `outA`).
 * There is not much of interest there though - all the useful stuff is in the
 * [leds] class.
 * .
 * This device is loaded when an [ev3-output-port] is set to `rcx-led` mode.
 * It is not automatically detected.
 * .
 * [leds]: https://github.com/ev3dev/ev3dev/wiki/Using-the-LEDs
 * [ev3-output-port]: /docs/ports/legoev3-output-port
 */

#include <linux/device.h>
#include <linux/module.h>
#include <linux/leds.h>
#include <linux/slab.h>

#include <lego.h>
#include <dc_motor_class.h>

struct rcx_led_data {
	char name[LEGO_NAME_SIZE];
	struct led_classdev cdev;
	struct lego_device *motor;
};

static void rcx_led_brightness_set(struct led_classdev *led_cdev,
				   enum led_brightness brightness)
{
	struct rcx_led_data *data =
			container_of(led_cdev, struct rcx_led_data, cdev);
	struct lego_port_device *port = data->motor->port;

	port->motor_ops->set_duty_cycle(port->context, brightness);
}

enum led_brightness rcx_led_brightness_get(struct led_classdev *led_cdev)
{
	struct rcx_led_data *data =
			container_of(led_cdev, struct rcx_led_data, cdev);
	struct lego_port_device *port = data->motor->port;

	return port->motor_ops->get_duty_cycle(port->context);
}

/*
 * Note: we use the term "motor" here because we call all output port devices
 * motors event though they may not really be a motor.
 */

static int rcx_led_probe(struct lego_device *motor)
{
	struct rcx_led_data *data;
	struct ev3_motor_platform_data *pdata = motor->dev.platform_data;
	int err;

	if (WARN_ON(!pdata))
		return -EINVAL;
	if (WARN_ON(!motor->port->motor_ops))
		return -EINVAL;

	data = kzalloc(sizeof(struct rcx_led_data), GFP_KERNEL);
	if (IS_ERR(data))
		return -ENOMEM;

	snprintf(data->name, LEGO_NAME_SIZE, "ev3::%s", motor->port->port_name);
	data->cdev.name = data->name;
	data->cdev.default_trigger = "none";
	data->cdev.brightness_set = rcx_led_brightness_set;
	data->cdev.brightness_get = rcx_led_brightness_get;
	data->cdev.brightness = LED_OFF;
	data->cdev.max_brightness = 100;
	data->motor = motor;

	err = led_classdev_register(&motor->dev, &data->cdev);
	if (err)
		goto err_led_classdev_register;

	dev_set_drvdata(&motor->dev, data);

	motor->port->motor_ops->set_command(motor->port->context,
					    DC_MOTOR_COMMAND_RUN);
	dev_info(data->cdev.dev, "Bound to device '%s'\n",
		 dev_name(&motor->dev));

	return 0;

err_led_classdev_register:
	kfree(data);

	return err;
}

static int rcx_led_remove(struct lego_device *motor)
{
	struct rcx_led_data *data = dev_get_drvdata(&motor->dev);

	dev_info(data->cdev.dev, "Unregistered.\n");
	led_classdev_unregister(&data->cdev);
	motor->port->motor_ops->set_command(motor->port->context,
					    DC_MOTOR_COMMAND_COAST);
	dev_set_drvdata(&motor->dev, NULL);
	kfree(data);

	return 0;
}

struct lego_device_driver rcx_led_driver = {
	.probe	= rcx_led_probe,
	.remove	= rcx_led_remove,
	.driver = {
		.name	= "rcx-led",
		.owner	= THIS_MODULE,
	},
};
lego_device_driver(rcx_led_driver);

MODULE_DESCRIPTION("RCX/Power Functions LED driver for LEGO MINDSTORMS EV3");
MODULE_AUTHOR("David Lechner <david@lechnology.com>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("lego:rcx-led");

