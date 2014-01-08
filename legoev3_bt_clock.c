/*
 * Bluetooth clock for LEGO Mindstorms EV3
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

/*
 * -----------------------------------------------------------------------------
 * The bluetooth chip on the LEGO Mindstorms EV3 is driven by one of the PWM
 * devices on the AM1808 SoC. This driver sets up the specified PWM device
 * to generate the required clock signal.
 * -----------------------------------------------------------------------------
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/pwm/pwm.h>
#include <linux/legoev3/legoev3_bt_clock.h>

static int __devinit legoev3_bt_clock_probe(struct platform_device *pdev)
{
	struct legoev3_bt_clock_platform_data *pdata;
	struct pwm_device *pwm;
	int err;

	if (!pdev->dev.platform_data) {
		dev_err(&pdev->dev, "%s: Platform data is required!\n", __func__);
		return -EINVAL;
	}

	pdata = pdev->dev.platform_data;

	pwm = pwm_request_byname(pdata->clk_pwm_dev, "legoev3 bluetooth clock");
	if (IS_ERR(pwm)) {
		dev_err(&pdev->dev, "%s: Could not request pwm device '%s'! (%ld)\n",
			__func__, pdata->clk_pwm_dev, PTR_ERR(pwm));
		return (PTR_ERR(pwm));
	}	
	err = pwm_set_frequency(pwm, 32768);
	if (err) {
		dev_err(&pdev->dev, "%s: Failed to set pwm frequency! (%d)\n",
			__func__, err);
		goto err_pwm_set_period_ticks;
	}
	err = pwm_set_duty_percent(pwm, 50);
	if (err) {
		dev_err(&pdev->dev, "%s: Failed to set pwm duty percent! (%d)\n",
			__func__, err);
		goto err_pwm_set_duty_ticks;
	}
	err = pwm_start(pwm);
	if (err) {
		dev_err(&pdev->dev, "%s: Failed to start pwm! (%d)\n",
			__func__, err);
		goto err_pwm_start;
	}

	platform_set_drvdata(pdev, pwm);
	
	return 0;

err_pwm_set_duty_ticks:
err_pwm_set_period_ticks:
err_pwm_start:
	pwm_release(pwm);
	return err;
}

static int __devexit legoev3_bt_clock_remove(struct platform_device *pdev)
{
	struct pwm_device *pwm = platform_get_drvdata(pdev);

	pwm_release(pwm);

	return 0;
}

struct platform_driver legoev3_bt_clock_driver = {
	.probe = legoev3_bt_clock_probe,
	.remove = __devexit_p(legoev3_bt_clock_remove),
	.driver = {
		.name = "legoev3-bt-clock",
		.owner = THIS_MODULE,
	},
};
module_platform_driver(legoev3_bt_clock_driver);

MODULE_DESCRIPTION("Bluetooth clock driver for LEGO Mindstorms EV3");
MODULE_AUTHOR("David Lechner <david@lechnology.com>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:legoev3-bt-clock");
