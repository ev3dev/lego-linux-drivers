/*
 * Input port driver for LEGO Mindstorms EV3
 *
 * Copyright (C) 2013 David Lechner <david@lechnology.com>
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
#include <linux/module.h>
#include <linux/legoev3/legoev3_analog.h>
#include <linux/legoev3/legoev3_ports.h>

#define INPUT_PORT_POLL_NS	10000000	/* 10 msec */

/*
 * Inital state for gpios: Pin 1 current generator is off, pins 2, 5 and 6 are
 * set to input and buffer is disabled. Not sure how the i2c clock pin works
 * yet, so setting it as input to be safe.
 */
#define pin1_INIT	GPIOF_OUT_INIT_LOW
#define pin2_INIT	GPIOF_IN
#define pin5_INIT	GPIOF_IN
#define pin6_INIT	GPIOF_IN
#define buf_ena_INIT	GPIOF_OUT_INIT_HIGH

#define GPIO_INIT(gpio_name)			\
	{					\
		.gpio	= ip->gpio_name##_gpio,	\
		.flags	= gpio_name##_INIT,	\
		.label	= "gpio_name",		\
	},

static enum hrtimer_restart legoev3_input_port_timer_callback(struct hrtimer *timer)
{
	struct legoev3_input_port_device *ip =
		container_of(timer, struct legoev3_input_port_device, timer);

	hrtimer_forward_now(timer, ktime_set(1, INPUT_PORT_POLL_NS));
printk("scanning port %s\n", dev_name(&ip->dev));
printk("pin1 analog: %d\n", legoev3_analog_in_pin1_value(ip->analog, ip->id));
printk("pin1 gpio: %d\n", gpio_get_value(ip->pin1_gpio));
printk("pin2 gpio: %d\n", gpio_get_value(ip->pin2_gpio));
printk("pin5 gpio: %d\n", gpio_get_value(ip->pin5_gpio));
printk("pin6 gpio: %d\n", gpio_get_value(ip->pin6_gpio));
printk("pin6 analog: %d\n", legoev3_analog_in_pin6_value(ip->analog, ip->id));
printk("\n");
	return HRTIMER_RESTART;
}

static int __devinit
legoev3_input_port_probe(struct device *dev)
{
	struct legoev3_input_port_device *ip = to_legoev3_input_port_device(dev);
	struct gpio gpio_data[] = {
		GPIO_INIT(pin1)
		GPIO_INIT(pin2)
		GPIO_INIT(pin5)
		GPIO_INIT(pin6)
		GPIO_INIT(buf_ena)
	};
	int err;
printk("probing input port %s\n", dev_name(dev));
	/* TODO: make a kernel option to disable port 1 when using serial port */
	/* or find a way to auto-detect */
	if (ip->id == LEGOEV3_PORT_IN1)
		return -EINVAL;
	if (!ip)
		return -EINVAL;
printk("have a pointer\n");
	ip->analog = request_legoev3_analog();
	if (IS_ERR(ip->analog)) {
		dev_err(&ip->dev, "Could not get legoev3-analog device.\n");
		return PTR_ERR(ip->analog);
	}
printk("got analog, requesting gpios %d, %d, %d, %d, %d, %d (%d)",
	ip->pin1_gpio, ip->pin2_gpio, ip->pin5_gpio, ip->pin6_gpio,
	ip->buf_ena_gpio, ip->i2c_clk_gpio, ARRAY_SIZE(gpio_data));
	err = gpio_request_array(gpio_data, ARRAY_SIZE(gpio_data));
	if (err) {
		dev_err(&ip->dev, "Requesting GPIOs failed.\n");
		return err;
	}
printk("got gpios\n");
	hrtimer_init(&ip->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	ip->timer.function = legoev3_input_port_timer_callback;
	hrtimer_start(&ip->timer, ktime_set(1, INPUT_PORT_POLL_NS),
		      HRTIMER_MODE_REL);
printk("done\n");
	return 0;
}

static int __devexit
legoev3_input_port_remove(struct device *dev)
{
	struct legoev3_input_port_device *ip = to_legoev3_input_port_device(dev);
	struct gpio gpio_data[] = {
		GPIO_INIT(pin1)
		GPIO_INIT(pin2)
		GPIO_INIT(pin5)
		GPIO_INIT(pin6)
		GPIO_INIT(buf_ena)
	};
printk("removing device %s\n", dev_name(dev));
	hrtimer_cancel(&ip->timer);
	gpio_free_array(gpio_data, ARRAY_SIZE(gpio_data));
	dev_set_drvdata(&ip->dev, NULL);

	return 0;
}

struct legoev3_port_driver legoev3_input_port_driver = {
	.driver = {
		.name	= "legoev3-input-port",
		.owner	= THIS_MODULE,
		.probe	= legoev3_input_port_probe,
		.remove	= __devexit_p(legoev3_input_port_remove),
	},
};
EXPORT_SYMBOL_GPL(legoev3_input_port_driver);
legoev3_port_driver(legoev3_input_port_driver);

MODULE_DESCRIPTION("Input port driver for LEGO Mindstorms EV3");
MODULE_AUTHOR("David Lechner <david@lechnology.com>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("legoev3:legoev3-input-port");
