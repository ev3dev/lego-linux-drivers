/*
 * On-board bluetooth support for LEGO Mindstorms EV3
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
 *
 * This also provides an interface similar to Device1 in d_bt.c from lms2012.
 * -----------------------------------------------------------------------------
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/pwm.h>
#include <linux/legoev3/legoev3_bluetooth.h>

#define SLOW_CLOCK_PERIOD_NS (NSEC_PER_SEC / 32768)

enum legoev3_bluetooth_gpios {
	LEGOEV3_BT_GPIO_BT_ENA,
	LEGOEV3_BT_GPIO_BT_CLK_ENA,
	LEGOEV3_BT_GPIO_PIC_ENA,
	LEGOEV3_BT_GPIO_PIC_RST,
	LEGOEV3_BT_GPIO_PIC_CTS,
	NUM_LEGOEV3_BT_GPIO
};

struct legoev3_bluetooth_device {
	struct gpio gpios[NUM_LEGOEV3_BT_GPIO];
	struct pwm_device *pwm;
};

static ssize_t legoev3_bluetooth_attr_show(struct device *dev,
					   struct device_attribute *attr,
					   char *buf)
{
	struct legoev3_bluetooth_device *btdev = dev_get_drvdata(dev);
	int gpio = -1;

	if (!strcmp(attr->attr.name, "enabled"))
		gpio = btdev->gpios[LEGOEV3_BT_GPIO_PIC_ENA].gpio;
	else if (!strcmp(attr->attr.name, "reset"))
		gpio = btdev->gpios[LEGOEV3_BT_GPIO_PIC_RST].gpio;
	else if (!strcmp(attr->attr.name, "cts"))
		gpio = btdev->gpios[LEGOEV3_BT_GPIO_PIC_CTS].gpio;
	else
		return -EINVAL;

	return sprintf(buf, "%d\n", !!gpio_get_value(gpio));
}

static ssize_t legoev3_bluetooth_attr_store(struct device *dev,
					    struct device_attribute *attr,
					    const char *buf, size_t count)
{
	struct legoev3_bluetooth_device *btdev = dev_get_drvdata(dev);
	int gpio = -1;
	bool value;

	if (!strcmp(attr->attr.name, "enabled"))
		gpio = btdev->gpios[LEGOEV3_BT_GPIO_PIC_ENA].gpio;
	else if (!strcmp(attr->attr.name,"reset"))
		gpio = btdev->gpios[LEGOEV3_BT_GPIO_PIC_RST].gpio;
	else
		return -EINVAL;

	if (strtobool(buf, &value))
		return -EINVAL;

	gpio_set_value(gpio, value);

	return count;
}

static DEVICE_ATTR(enabled, S_IWUSR | S_IRUGO, legoev3_bluetooth_attr_show,
		   legoev3_bluetooth_attr_store);
static DEVICE_ATTR(reset, S_IWUSR | S_IRUGO, legoev3_bluetooth_attr_show,
		   legoev3_bluetooth_attr_store);
static DEVICE_ATTR(cts, S_IRUGO, legoev3_bluetooth_attr_show, NULL);

static struct attribute *legoev3_bluetooth_attrs[] = {
	&dev_attr_enabled.attr,
	&dev_attr_reset.attr,
	&dev_attr_cts.attr,
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

	btdev = kzalloc(sizeof(struct legoev3_bluetooth_device), GFP_KERNEL);
	if (!btdev) {
		dev_err(&pdev->dev, "%s: Could not allocate memory!\n",
			__func__);
		return -ENOMEM;
	}

	btdev->gpios[LEGOEV3_BT_GPIO_BT_ENA].gpio	= pdata->bt_ena_gpio;
	btdev->gpios[LEGOEV3_BT_GPIO_BT_ENA].flags	= GPIOF_OUT_INIT_LOW;
	btdev->gpios[LEGOEV3_BT_GPIO_BT_ENA].label	= "bluetooth enable";
	btdev->gpios[LEGOEV3_BT_GPIO_BT_CLK_ENA].gpio	= pdata->bt_clk_ena_gpio;
	btdev->gpios[LEGOEV3_BT_GPIO_BT_CLK_ENA].flags	= GPIOF_IN;
	btdev->gpios[LEGOEV3_BT_GPIO_BT_CLK_ENA].label	= "bluetooth slow clock enable";
	btdev->gpios[LEGOEV3_BT_GPIO_PIC_ENA].gpio	= pdata->pic_ena_gpio;
	btdev->gpios[LEGOEV3_BT_GPIO_PIC_ENA].flags	= GPIOF_OUT_INIT_LOW;
	btdev->gpios[LEGOEV3_BT_GPIO_PIC_ENA].label	= "bt pic enable";
	btdev->gpios[LEGOEV3_BT_GPIO_PIC_RST].gpio	= pdata->pic_rst_gpio;
	btdev->gpios[LEGOEV3_BT_GPIO_PIC_RST].flags	= GPIOF_OUT_INIT_HIGH;
	btdev->gpios[LEGOEV3_BT_GPIO_PIC_RST].label	= "bt pic reset";
	btdev->gpios[LEGOEV3_BT_GPIO_PIC_CTS].gpio	= pdata->pic_cts_gpio;
	btdev->gpios[LEGOEV3_BT_GPIO_PIC_CTS].flags	= GPIOF_IN;
	btdev->gpios[LEGOEV3_BT_GPIO_PIC_CTS].label	= "bt pic cts";
	err = gpio_request_array(btdev->gpios, NUM_LEGOEV3_BT_GPIO);
	if (err) {
		dev_err(&pdev->dev, "%s: Failed to request gpios! (%d)\n",
			__func__, err);
		goto err_gpio_request_array;
	}

	pwm = pwm_get(&pdev->dev, "legoev3 bluetooth clock");
	if (IS_ERR(pwm)) {
		dev_err(&pdev->dev, "%s: Could not request pwm device '%s'! (%ld)\n",
			__func__, pdata->clk_pwm_dev, PTR_ERR(pwm));
		err = PTR_ERR(pwm);
		goto err_pwm_request_byname;
	}
	err = pwm_set_polarity(pwm, 0);
	if (err) {
		dev_err(&pdev->dev, "%s: Failed to set pwm polarity! (%d)\n",
			__func__, err);
		goto err_pwm_set_polarity;
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
	pwm_disable(pwm);
err_pwm_start:
err_pwm_config:
err_pwm_set_polarity:
	pwm_put(pwm);
err_pwm_request_byname:
	gpio_free_array(btdev->gpios, NUM_LEGOEV3_BT_GPIO);
err_gpio_request_array:
	kfree(btdev);

	return err;
}

static int legoev3_bluetooth_remove(struct platform_device *pdev)
{
	struct legoev3_bluetooth_device *btdev = platform_get_drvdata(pdev);

	/* TODO: set gpios to turn off device */
	sysfs_remove_group(&pdev->dev.kobj, &legoev3_bluetooth_attr_grp);
	gpio_set_value(btdev->gpios[LEGOEV3_BT_GPIO_BT_ENA].gpio, 0);
	pwm_disable(btdev->pwm);
	pwm_put(btdev->pwm);
	gpio_free_array(btdev->gpios, NUM_LEGOEV3_BT_GPIO);
	platform_set_drvdata(pdev, NULL);
	kfree(btdev);

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

MODULE_DESCRIPTION("Bluetooth driver for LEGO Mindstorms EV3");
MODULE_AUTHOR("David Lechner <david@lechnology.com>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:legoev3-bluetooth");
