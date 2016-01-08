/*
 * System Info driver for BeagleBone.
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

/*
 * The Revision and Serial in /proc/cpuinfo are not populated by the bb.org
 * kernel. This is most likely because BeagleBone uses ascii values for these
 * and the kernel uses integers (displayed as hexadecimal). This driver does
 * it's best to populate these anyway. Most of the characters are valid hex
 * so it works OK.
 */

#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/nvmem-consumer.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <asm/system_info.h>

static u8 to_hex(u8 c)
{
	if (c >= '0' && c <= '9')
		return c - '0';
	if (c >= 'A' && c <= 'F')
		return c - 'A' + 0xA;
	if (c >= 'a' && c <= 'f')
		return c - 'a' + 0xA;
	return 0;
}

static int evb_system_info_probe(struct platform_device *pdev)
{
	struct nvmem_cell *nvm;
	const u8 *p;
	size_t len;
	int ret = 0;

	nvm = nvmem_cell_get(&pdev->dev, "baseboard");
	if (IS_ERR(nvm)) {
		dev_err(&pdev->dev, "Failed to get baseboard eeprom.\n");
		return PTR_ERR(nvm);
	}

	p = nvmem_cell_read(nvm, &len);
	if (IS_ERR(p)) {
		ret = PTR_ERR(p);
		dev_err(&pdev->dev, "Cannot read cell.\n");
		goto err;
	}
	if (len < 28) {
		ret = -EINVAL;
		dev_err(&pdev->dev, "Data too small.\n");
		goto err;
	}
	if (p[0] != 0xAA || p[1] != 0x55 || p[2] != 0x33 || p[3] != 0xEE) {
		ret = -EINVAL;
		dev_err(&pdev->dev, "Missing BeagleBone magic.\n");
		goto err;
	}

	system_rev	   = (to_hex(p[12]) << 12) + (to_hex(p[13]) <<  8)
			   + (to_hex(p[14]) <<  4) + (to_hex(p[15]) <<  0);
	system_serial_high = (to_hex(p[16]) << 12) + (to_hex(p[17]) <<  8)
			   + (to_hex(p[18]) <<  4) + (to_hex(p[19]) <<  0);
	system_serial_low  = (to_hex(p[20]) << 28) + (to_hex(p[21]) << 24)
			   + (to_hex(p[22]) << 20) + (to_hex(p[23]) << 16)
			   + (to_hex(p[24]) << 12) + (to_hex(p[25]) <<  8)
			   + (to_hex(p[26]) <<  4) + (to_hex(p[27]) <<  0);

err:
	nvmem_cell_put(nvm);
	return ret;
}

static int evb_system_info_remove(struct platform_device *pdev)
{
	return 0;
}

static const struct of_device_id of_evb_system_info_match[] = {
	{ .compatible = "ev3dev,evb-system-info", },
	{ }
};
MODULE_DEVICE_TABLE(of, of_evb_system_info_match);

static struct platform_driver evb_system_info_driver = {
	.driver = {
		.name		= "evb-system-info",
		.of_match_table = of_evb_system_info_match,
	},
	.probe  = evb_system_info_probe,
	.remove = evb_system_info_remove,
};
module_platform_driver(evb_system_info_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("David Lechner <david@lechnology.com>");
MODULE_DESCRIPTION("FatcatLab EVB System Information");
MODULE_ALIAS("platform:evb-system-info");
