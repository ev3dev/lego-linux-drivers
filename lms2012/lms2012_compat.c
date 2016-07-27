/*
 * lms2012 compatibility driver
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
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/slab.h>

#include "lms2012.h"

static int lms2012_compat_match(struct device *dev, void *data)
{
    return strcmp(dev_name(dev), "lms2012-compat") == 0;
}

/**
 * lms2012_compat_get - get the global lms2012-compat instance
 *
 * Returns the instance or -EPROBE_DEFER if it is not found.
 *
 * Release the device with put_device() when finished.
 */
struct device *lms2012_compat_get(void)
{
    struct device *dev;

    dev = bus_find_device(&platform_bus_type, NULL, NULL, lms2012_compat_match);
    if (!dev)
        return ERR_PTR(-EPROBE_DEFER);

    return dev;
}
EXPORT_SYMBOL_GPL(lms2012_compat_get);

static int lms2012_compat_probe(struct platform_device *pdev)
{
    struct lms2012_compat *lms;
    int ret, i;
    char name[5];

    lms = devm_kzalloc(&pdev->dev, sizeof(*lms), GFP_KERNEL);
    if (!lms)
        return -ENOMEM;

    ret = of_property_read_u32_array(pdev->dev.of_node, "adc-channels",
                                     lms->adc_map, INPUTADC);
    if (ret < 0)
        return ret;

    lms->spi_pins = devm_gpiod_get_array(&pdev->dev, "spi", GPIOD_ASIS);
    if (IS_ERR(lms->spi_pins)) {
        dev_err(&pdev->dev, "Failed to get spi pins\n");
        return PTR_ERR(lms->spi_pins);
    }
    if (lms->spi_pins->ndescs != ADC_SPI_PINS) {
        dev_err(&pdev->dev, "Incorrect number of spi pins\n");
        return -EINVAL;
    }

    for (i = 0; i < INPUTS; i++) {
        snprintf(name, 5, "in%d", i + 1);
        lms->in_pins[i] = devm_gpiod_get_array(&pdev->dev, name, GPIOD_ASIS);
        if (IS_ERR(lms->in_pins[i])) {
            dev_err(&pdev->dev, "Failed to get %s pins\n", name);
            return PTR_ERR(lms->in_pins[i]);
        }
        if (lms->in_pins[i]->ndescs != INPUT_PORT_PINS) {
            dev_err(&pdev->dev, "Incorrect number of %s pins\n", name);
            return -EINVAL;
        }
    }

    for (i = 0; i < OUTPUTS; i++) {
        snprintf(name, 5, "out%c", i + 'A');
        lms->out_pins[i] = devm_gpiod_get_array(&pdev->dev, name, GPIOD_ASIS);
        if (IS_ERR(lms->out_pins[i])) {
            dev_err(&pdev->dev, "Failed to get %s pins\n", name);
            return PTR_ERR(lms->out_pins[i]);
        }
        if (lms->out_pins[i]->ndescs != OUTPUT_PORT_PINS) {
            dev_err(&pdev->dev, "Incorrect number of %s pins\n", name);
            return -EINVAL;
        }
    }

    platform_set_drvdata(pdev, lms);

    dev_info(&pdev->dev, "Registered lms2012-compat\n");

    return 0;
}

static int lms2012_compat_remove(struct platform_device *pdev)
{
    // struct lms2012_compat *lms = platform_get_drvdata(pdev);

    dev_info(&pdev->dev, "Unregistered lms2012-compat\n");

    return 0;
}

static const struct of_device_id of_lms2012_compat_match[] = {
    { .compatible = "ev3dev,lms2012-compat", },
    { }
};
MODULE_DEVICE_TABLE(of, of_lms2012_compat_match);

static struct platform_driver lms2012_compat_driver = {
    .driver = {
        .name       = "lms2012-compat",
        .of_match_table = of_lms2012_compat_match,
    },
    .probe  = lms2012_compat_probe,
    .remove = lms2012_compat_remove,
};
module_platform_driver(lms2012_compat_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("David Lechner <david@lechnology.com>");
MODULE_DESCRIPTION("lms2012 compatibility driver");
MODULE_ALIAS("platform:lms2012-compat");
