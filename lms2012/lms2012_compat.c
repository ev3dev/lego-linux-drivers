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
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

#include "lms2012.h"

static struct device *global_dev = NULL;

/**
 * lms2012_compat_get - get the global lms2012-compat instance
 *
 * Returns the instance or NULL if no device has been probed yet.
 *
 * Release the device with put_device() when finished.
 */
struct device *lms2012_compat_get(void)
{
	return get_device(global_dev);
}
EXPORT_SYMBOL_GPL(lms2012_compat_get);

struct lms2012_compat_clk {
	struct clk *clk;
};

static void lms2012_compat_release_clk(struct device *dev, void *res)
{
	struct lms2012_compat_clk *clk = res;

	clk_disable_unprepare(clk->clk);
	clk_put(clk->clk);
}

static int lms2012_compat_probe(struct platform_device *pdev)
{
	struct lms2012_compat *lms;
	struct lms2012_compat_clk *clk;
	int ret, i;
	char name[5];

	if (global_dev)
		return -EBUSY;

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
		lms->in_pins[i] = devm_gpiod_get_array(&pdev->dev, name,
						       GPIOD_ASIS);
		if (IS_ERR(lms->in_pins[i])) {
			dev_err(&pdev->dev, "Failed to get %s pins\n", name);
			return PTR_ERR(lms->in_pins[i]);
		}
		if (lms->in_pins[i]->ndescs != INPUT_PORT_PINS) {
			dev_err(&pdev->dev, "Incorrect number of %s pins\n",
				name);
			return -EINVAL;
		}
	}

	ret = of_count_phandle_with_args(pdev->dev.of_node, "in-uarts",
					 "#uart-cells");
	if (ret < 0) {
		dev_err(&pdev->dev, "Could not get uarts\n");
		return ret;
	} else if (ret != INPUTS) {
		dev_err(&pdev->dev, "Incorrect number of uarts (%d)\n", ret);
		return -EINVAL;
	}
	for (i = 0; i < INPUTS; i++) {
		struct of_phandle_args args;
		struct resource res;
		char name[10];

		snprintf(name, 10, "in%d_uart", i + 1);

		ret = of_parse_phandle_with_args(pdev->dev.of_node, "in-uarts",
						 "#uart-cells", i, &args);
		if (ret < 0) {
			dev_err(&pdev->dev, "Could not get uart %d\n", i + 1);
			return ret;
		}

		ret = of_address_to_resource(args.np, 0, &res);
		if (ret < 0) {
			dev_err(&pdev->dev, "Could not get uart %d resource\n",
				i + 1);
			of_node_put(args.np);

			return ret;
		}

		lms->uart_mem[i] = devm_ioremap_resource(&pdev->dev, &res);
		if (IS_ERR(lms->uart_mem[i])) {
			dev_err(&pdev->dev, "Could not map uart %d memory\n",
				i + 1);
			of_node_put(args.np);

			return PTR_ERR(lms->uart_mem[i]);
		}

		lms->uart_irq[i] = of_irq_get(args.np, 0);
		if (lms->uart_irq[i] < 0) {
			dev_err(&pdev->dev, "Could not get uart %d irq\n",
				i + 1);
			of_node_put(args.np);

			return lms->uart_irq[i];
		}

		ret = of_property_read_u32(args.np, "clock-frequency",
					   &lms->uart_clock_freq[i]);
		if (ret < 0) {
			dev_err(&pdev->dev,
				"Could not get uart %d clock frequency\n",
				i + 1);
			of_node_put(args.np);

			return ret;
		}

		clk = devres_alloc(lms2012_compat_release_clk,
				   sizeof(struct lms2012_compat_clk),
				   GFP_KERNEL);

		clk->clk = of_clk_get_by_name(args.np, "fck");
		if (IS_ERR(clk->clk)) {
			dev_err(&pdev->dev, "Could not get uart %d clock\n",
				i + 1);
			devres_free(clk);
			of_node_put(args.np);

			return PTR_ERR(clk->clk);
		}

		ret = clk_prepare_enable(clk->clk);
		if (ret < 0) {
			dev_err(&pdev->dev,
				"Failed to prepare/enable uart %d clock\n",
				i + 1);
			clk_put(clk->clk);
			devres_free(clk);
			of_node_put(args.np);

			return ret;
		}

		devres_add(&pdev->dev, clk);

		of_node_put(args.np);
	}

	for (i = 0; i < OUTPUTS; i++) {
		snprintf(name, 5, "out%c", i + 'A');
		lms->out_pins[i] = devm_gpiod_get_array(&pdev->dev, name,
							GPIOD_ASIS);
		if (IS_ERR(lms->out_pins[i])) {
			dev_err(&pdev->dev, "Failed to get %s pins\n", name);
			return PTR_ERR(lms->out_pins[i]);
		}
		if (lms->out_pins[i]->ndescs != OUTPUT_PORT_PINS) {
			dev_err(&pdev->dev, "Incorrect number of %s pins\n",
				name);
			return -EINVAL;
		}
	}

	platform_set_drvdata(pdev, lms);
	global_dev = &pdev->dev;

	dev_info(&pdev->dev, "Registered lms2012-compat\n");

	return 0;
}

static int lms2012_compat_remove(struct platform_device *pdev)
{
	// struct lms2012_compat *lms = platform_get_drvdata(pdev);

	global_dev = NULL;

	dev_info(&pdev->dev, "Unregistered lms2012-compat\n");

	return 0;
}

static const struct of_device_id of_lms2012_compat_match[] = {
	{ .compatible = "ev3dev,lms2012-compat", },
	{ }
};
MODULE_DEVICE_TABLE(of, of_lms2012_compat_match);

static struct platform_driver lms2012_compat_driver = {
	.driver	= {
		.name		= "lms2012-compat",
		.of_match_table	= of_lms2012_compat_match,
	},
	.probe	= lms2012_compat_probe,
	.remove	= lms2012_compat_remove,
};
module_platform_driver(lms2012_compat_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("David Lechner <david@lechnology.com>");
MODULE_DESCRIPTION("lms2012 compatibility driver");
MODULE_ALIAS("platform:lms2012-compat");
