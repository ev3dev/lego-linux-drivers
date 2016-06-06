/*
 * On-board bluetooth support for LEGO MINDSTORMS EV3
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
 * -----------------------------------------------------------------------------
 * The bluetooth module in the LEGO Mindstorms EV3 is a Panasonic
 * PAN1325A-HCI-85, which has a TI CC2560A chip.
 *
 * The chip requires an external clock to function. This clock is generated
 * by one of the PWM devices on the AM1808 SoC.
 * -----------------------------------------------------------------------------
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/pwm.h>
#include <linux/platform_data/legoev3.h>

#define SLOW_CLOCK_PERIOD_NS (NSEC_PER_SEC / 32768)

enum legoev3_bluetooth_gpios {
	LEGOEV3_BT_GPIO_BT_ENA,
	LEGOEV3_BT_GPIO_BT_CLK_ENA,
	NUM_LEGOEV3_BT_GPIO
};

struct legoev3_bluetooth_device {
	struct gpio gpios[NUM_LEGOEV3_BT_GPIO];
	struct pwm_device *pwm;
};

static ssize_t legoev3_bluetooth_enable_show(struct device *dev,
					     struct device_attribute *attr,
					     char *buf)
{
	struct legoev3_bluetooth_device *btdev = dev_get_drvdata(dev);

	return sprintf(buf, "%d\n",
		!!gpio_get_value(btdev->gpios[LEGOEV3_BT_GPIO_BT_ENA].gpio));
}

static ssize_t legoev3_bluetooth_enable_store(struct device *dev,
					      struct device_attribute *attr,
					      const char *buf, size_t count)
{
	struct legoev3_bluetooth_device *btdev = dev_get_drvdata(dev);
	bool enable, current_state;

	if (strtobool(buf, &enable))
		return -EINVAL;

	current_state = gpio_get_value(btdev->gpios[LEGOEV3_BT_GPIO_BT_ENA].gpio);

	if (enable == current_state)
		return count;

	if (enable)
		pwm_enable(btdev->pwm);
	else
		pwm_disable(btdev->pwm);
	gpio_set_value(btdev->gpios[LEGOEV3_BT_GPIO_BT_ENA].gpio, enable);

	return count;
}

static DEVICE_ATTR(enable, S_IWUSR | S_IRUGO, legoev3_bluetooth_enable_show,
		   legoev3_bluetooth_enable_store);

static struct attribute *legoev3_bluetooth_attrs[] = {
	&dev_attr_enable.attr,
	NULL
};

static struct attribute_group legoev3_bluetooth_attr_grp = {
	.attrs = legoev3_bluetooth_attrs,
};

static int legoev3_bluetooth_probe(struct platform_device *pdev)
{
	struct legoev3_bluetooth_device *btdev;
	struct legoev3_bluetooth_platform_data *pdata;
	struct pwm_device *pwm;
	int err;

	if (!pdev->dev.platform_data) {
		dev_err(&pdev->dev, "%s: Platform data is required!\n", __func__);
		return -EINVAL;
	}
	pdata = pdev->dev.platform_data;

	btdev = devm_kzalloc(&pdev->dev, sizeof(struct legoev3_bluetooth_device),
			     GFP_KERNEL);
	if (!btdev) {
		dev_err(&pdev->dev, "%s: Could not allocate memory!\n",
			__func__);
		return -ENOMEM;
	}

	btdev->gpios[LEGOEV3_BT_GPIO_BT_ENA].gpio	= pdata->bt_ena_gpio;
	btdev->gpios[LEGOEV3_BT_GPIO_BT_ENA].flags	= GPIOF_OUT_INIT_LOW;
	btdev->gpios[LEGOEV3_BT_GPIO_BT_ENA].label	= "bluetooth disable";
	btdev->gpios[LEGOEV3_BT_GPIO_BT_CLK_ENA].gpio	= pdata->bt_clk_ena_gpio;
	btdev->gpios[LEGOEV3_BT_GPIO_BT_CLK_ENA].flags	= GPIOF_IN;
	btdev->gpios[LEGOEV3_BT_GPIO_BT_CLK_ENA].label	= "bluetooth slow clock enable";
	err = gpio_request_array(btdev->gpios, NUM_LEGOEV3_BT_GPIO);
	if (err) {
		dev_err(&pdev->dev, "%s: Failed to request gpios! (%d)\n",
			__func__, err);
		return err;
	}

	pwm = pwm_get(&pdev->dev, NULL);
	if (IS_ERR(pwm)) {
		err = PTR_ERR(pwm);
		if (err != -EPROBE_DEFER) {
			dev_err(&pdev->dev, "%s: Could not get pwm! (%ld)\n",
				__func__, PTR_ERR(pwm));
		}
		goto err_pwm_get;
	}
	err = pwm_config(pwm, SLOW_CLOCK_PERIOD_NS / 2, SLOW_CLOCK_PERIOD_NS);
	if (err) {
		dev_err(&pdev->dev, "%s: Failed to set pwm duty cycle and frequency! (%d)\n",
			__func__, err);
		goto err_pwm_config;
	}
	err = pwm_enable(pwm);
	if (err) {
		dev_err(&pdev->dev, "%s: Failed to start pwm! (%d)\n",
			__func__, err);
		goto err_pwm_start;
	}
	gpio_set_value(btdev->gpios[LEGOEV3_BT_GPIO_BT_ENA].gpio, 1);

	err = sysfs_create_group(&pdev->dev.kobj, &legoev3_bluetooth_attr_grp);
	if (err)
		goto err_sysfs_create_group;

	btdev->pwm = pwm;
	platform_set_drvdata(pdev, btdev);

	return 0;

err_sysfs_create_group:
	gpio_set_value(btdev->gpios[LEGOEV3_BT_GPIO_BT_ENA].gpio, 0);
	pwm_disable(pwm);
err_pwm_start:
err_pwm_config:
	pwm_put(pwm);
err_pwm_get:
	gpio_free_array(btdev->gpios, NUM_LEGOEV3_BT_GPIO);

	return err;
}

static int legoev3_bluetooth_remove(struct platform_device *pdev)
{
	struct legoev3_bluetooth_device *btdev = platform_get_drvdata(pdev);

	sysfs_remove_group(&pdev->dev.kobj, &legoev3_bluetooth_attr_grp);
	if (gpio_get_value(btdev->gpios[LEGOEV3_BT_GPIO_BT_ENA].gpio)) {
		gpio_set_value(btdev->gpios[LEGOEV3_BT_GPIO_BT_ENA].gpio, 0);
		pwm_disable(btdev->pwm);
	}
	pwm_put(btdev->pwm);
	gpio_free_array(btdev->gpios, NUM_LEGOEV3_BT_GPIO);
	platform_set_drvdata(pdev, NULL);

	return 0;
}

struct platform_driver legoev3_bluetooth_driver = {
	.probe	= legoev3_bluetooth_probe,
	.remove	= legoev3_bluetooth_remove,
	.driver	= {
		.name	= "legoev3-bluetooth",
		.owner	= THIS_MODULE,
	},
};
module_platform_driver(legoev3_bluetooth_driver);

MODULE_DESCRIPTION("Bluetooth driver for LEGO MINDSTORMS EV3");
MODULE_AUTHOR("David Lechner <david@lechnology.com>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:legoev3-bluetooth");
