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

enum gpio_index {
	GPIO_PIN1,
	GPIO_PIN2,
	GPIO_PIN5,
	GPIO_PIN6,
	GPIO_BUF_ENA,
	GPIO_I2C_CLK,
	NUM_GPIO
};

enum pin5_mux_mode {
	PIN5_MUX_MODE_I2C,
	PIN5_MUX_MODE_UART,
	NUM_PIN5_MUX_MODE
};

struct legoev3_input_port_device {
	enum legoev3_input_port_id id;
	struct device *dev;
	struct legoev3_analog_device *analog;
	struct gpio gpio[NUM_GPIO];
	unsigned pin5_mux[NUM_PIN5_MUX_MODE];
	struct i2c_gpio_platform_data i2c_data;
	struct platform_device i2c_device;
	struct hrtimer timer;
};

static enum hrtimer_restart legoev3_input_port_timer_callback(struct hrtimer *timer)
{
	struct legoev3_input_port_device *ip =
		container_of(timer, struct legoev3_input_port_device, timer);

	hrtimer_forward_now(timer, ktime_set(1, INPUT_PORT_POLL_NS));
printk("scanning port %s\n", dev_name(ip->dev));
printk("pin1 analog: %d\n", legoev3_analog_in_pin1_value(ip->analog, ip->id));
printk("pin1 gpio: %d\n", !!gpio_get_value(ip->gpio[GPIO_PIN1].gpio));
printk("pin2 gpio: %d\n", !!gpio_get_value(ip->gpio[GPIO_PIN2].gpio));
printk("pin5 gpio: %d\n", !!gpio_get_value(ip->gpio[GPIO_PIN5].gpio));
printk("pin6 gpio: %d\n", !!gpio_get_value(ip->gpio[GPIO_PIN6].gpio));
printk("pin6 analog: %d\n", legoev3_analog_in_pin6_value(ip->analog, ip->id));
printk("\n");
	return HRTIMER_RESTART;
}

static int __devinit
legoev3_input_port_probe(struct device *dev)
{
	struct legoev3_input_port_device *ip;
	struct legoev3_input_port_platform_data *pdata = dev->platform_data;
	int err;

	/* TODO: make a kernel option to disable port 1 when using serial port */
	/* or find a way to auto-detect */
	if (pdata->id == LEGOEV3_PORT_IN1)
		return -EINVAL;

printk("probing input port %s\n", dev_name(dev));
	ip = kzalloc(sizeof(struct legoev3_input_port_device), GFP_KERNEL);
	if (!ip)
		return -ENOMEM;

	ip->id = pdata->id;
	ip->dev = dev;
	ip->analog = request_legoev3_analog();
	if (IS_ERR(ip->analog)) {
		dev_err(dev, "Could not get legoev3-analog device.\n");
		err = PTR_ERR(ip->analog);
		goto request_legoev3_analog_fail;
	}

	ip->gpio[GPIO_PIN1].gpio	= pdata->pin1_gpio;
	ip->gpio[GPIO_PIN1].flags	= GPIOF_OUT_INIT_LOW;
	ip->gpio[GPIO_PIN2].gpio	= pdata->pin2_gpio;
	ip->gpio[GPIO_PIN2].flags	= GPIOF_IN;
	ip->gpio[GPIO_PIN5].gpio	= pdata->pin5_gpio;
	ip->gpio[GPIO_PIN5].flags	= GPIOF_IN;
	ip->gpio[GPIO_PIN6].gpio	= pdata->pin6_gpio;
	ip->gpio[GPIO_PIN6].flags	= GPIOF_IN;
	ip->gpio[GPIO_BUF_ENA].gpio	= pdata->buf_ena_gpio;
	ip->gpio[GPIO_BUF_ENA].flags	= GPIOF_OUT_INIT_HIGH;
	ip->gpio[GPIO_I2C_CLK].gpio	= pdata->i2c_clk_gpio;
	ip->gpio[GPIO_I2C_CLK].flags	= GPIOF_IN;

	err = gpio_request_array(ip->gpio, ARRAY_SIZE(ip->gpio));
	if (err) {
		dev_err(dev, "Requesting GPIOs failed.\n");
		goto gpio_request_array_fail;
	}

	ip->pin5_mux[PIN5_MUX_MODE_I2C] = pdata->i2c_pin_mux;
	ip->pin5_mux[PIN5_MUX_MODE_UART] = pdata->uart_pin_mux;

	ip->i2c_data.sda_pin	= pdata->pin6_gpio;
	ip->i2c_data.scl_pin	= pdata->i2c_clk_gpio;
	ip->i2c_data.udelay	= 52;	/* ~9.6kHz */
	ip->i2c_device.name			= "legoev3-input-port-i2c";
	ip->i2c_device.id			= pdata->id;
	ip->i2c_device.dev.parent		= dev;
	ip->i2c_device.dev.platform_data	= &ip->i2c_data;

	hrtimer_init(&ip->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	ip->timer.function = legoev3_input_port_timer_callback;
	hrtimer_start(&ip->timer, ktime_set(1, INPUT_PORT_POLL_NS),
		      HRTIMER_MODE_REL);

	dev_set_drvdata(dev, ip);

	return 0;

gpio_request_array_fail:
request_legoev3_analog_fail:
	kfree(ip);

	return err;
}

static int __devexit
legoev3_input_port_remove(struct device *dev)
{
	struct legoev3_input_port_device *ip = dev_get_drvdata(dev);
printk("removing device %s\n", dev_name(dev));
	hrtimer_cancel(&ip->timer);
	/* TODO: check if i2c or uart device is registered and unregister it too */
	gpio_free_array(ip->gpio, ARRAY_SIZE(ip->gpio));
	dev_set_drvdata(dev, NULL);
	kfree(ip);

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
