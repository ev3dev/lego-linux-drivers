// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2018 David Lechner <david@lechnology.com>
 *
 * Remoteproc driver for TI AM18xx/OMAPL138 PRU
 */

#include <linux/bitops.h>
#include <linux/debugfs.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/remoteproc.h>
#include <linux/regmap.h>

/* memory map */
#define DA8XX_PRU_DATA_RAM_0	0x0000
#define DA8XX_PRU_DATA_RAM_1	0x0200
#define DA8XX_PRU_INTC		0x4000
#define DA8XX_PRU_PRU0_CS	0x7000
#define DA8XX_PRU_PRU1_CS	0x7800
#define DA8XX_PRU_PRU0_INST	0x8000
#define DA8XX_PRU_PRU1_INST	0xc000

#define DA8XX_PRU_DATA_RAM_SIZE	0x200
#define DA8XX_PRU_INTC_SIZE	0x3000
#define DA8XX_PRU_CS_SIZE	0x800
#define DA8XX_PRU_INST_SIZE	0x1000

/* control/status registers */
#define DA8XX_PRU_CS_CONTROL	0x0
#define DA8XX_PRU_CS_STATUS	0x4
#define DA8XX_PRU_CS_WAKEUP	0x8
#define DA8XX_PRU_CS_CYCLECNT	0xc

/* control register bits */
#define DA8XX_PRU_CONTROL_PCRESETVAL	GENMASK(31, 16)
#define DA8XX_PRU_CONTROL_RUNSTATE	BIT(15)
#define DA8XX_PRU_CONTROL_SINGLESTEP	BIT(8)
#define DA8XX_PRU_CONTROL_COUNTENABLE	BIT(3)
#define DA8XX_PRU_CONTROL_SLEEPING	BIT(2)
#define DA8XX_PRU_CONTROL_ENABLE	BIT(1)
#define DA8XX_PRU_CONTROL_SOFTRESET	BIT(0)

/* status bits */
#define DA8XX_PRU_STATUS_PCOUNTER	GENMASK(15, 0)

enum da8xx_pru_evtout {
	DA8XX_PRU_EVTOUT0,
	DA8XX_PRU_EVTOUT1,
	DA8XX_PRU_EVTOUT2,
	DA8XX_PRU_EVTOUT3,
	DA8XX_PRU_EVTOUT4,
	DA8XX_PRU_EVTOUT5,
	DA8XX_PRU_EVTOUT6,
	DA8XX_PRU_EVTOUT7,
	NUM_DA8XX_PRU_EVTOUT
};

struct da8xx_pru_data {
	struct device *dev;
	void __iomem *base;
	struct rproc *rproc;
	struct regmap *regmap;
	int evtout_irq[NUM_DA8XX_PRU_EVTOUT];
};

static int da8xx_pru_rproc_start(struct rproc *rproc)
{
	struct da8xx_pru_data *data = rproc->priv;
	u32 val;

	val = (rproc->bootaddr >> 2) << ffs(DA8XX_PRU_CONTROL_PCRESETVAL);
	val |= DA8XX_PRU_CONTROL_ENABLE;

	return regmap_write(data->regmap, DA8XX_PRU_CS_CONTROL, val);
}

static int da8xx_pru_rproc_stop(struct rproc *rproc)
{
	struct da8xx_pru_data *data = rproc->priv;
	u32 mask;

	mask = DA8XX_PRU_CONTROL_ENABLE;

	return regmap_write_bits(data->regmap, DA8XX_PRU_CS_CONTROL, mask, 0);
}

static void *da8xx_pru_rproc_da_to_va(struct rproc *rproc, u64 da, int len,
				      int page)
{
	struct da8xx_pru_data *data = rproc->priv;

	printk("%s: %llu %d %d\n", __func__, da, len, page);

	if (page == 0) {
		if (da + len > DA8XX_PRU_INST_SIZE)
			return ERR_PTR(-EINVAL);

		return data->base + DA8XX_PRU_PRU0_INST + da;
	}

	if (page == 1) {
		if (da + len > DA8XX_PRU_DATA_RAM_SIZE)
			return ERR_PTR(-EINVAL);

		return data->base + DA8XX_PRU_DATA_RAM_0 + da;
	}

	return ERR_PTR(-EINVAL);
}

static const struct rproc_ops da8xx_pru_rproc_ops = {
	.start = da8xx_pru_rproc_start,
	.stop = da8xx_pru_rproc_stop,
	.da_to_va = da8xx_pru_rproc_da_to_va,
};

static struct regmap_config da8xx_pru_regmap_config = {
	.reg_bits = 32,
	.val_bits = 32,
	.reg_stride = 4,
	.max_register = 0x2c,
};

static int da8xx_pru_rproc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct rproc *rproc;
	struct da8xx_pru_data *data;
	struct resource *res;
	int i, ret;

	rproc = rproc_alloc(dev, pdev->name, &da8xx_pru_rproc_ops, "ev3-pru-test.out",
			    sizeof(*data));
	if (!rproc)
		return -ENOMEM;

	data = rproc->priv;
	data->dev = dev;
	data->rproc = rproc;

	platform_set_drvdata(pdev, data);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	data->base = devm_ioremap_resource(dev, res);
	if (IS_ERR(data->base)) {
		dev_err(dev, "failed to ioremap resource\n");
		return PTR_ERR(data->base);
	}

	data->regmap = devm_regmap_init_mmio(dev, data->base + DA8XX_PRU_PRU0_CS,
					     &da8xx_pru_regmap_config);
	if (IS_ERR(data->regmap)) {
		dev_err(dev, "failed to initialize regmap\n");
		return PTR_ERR(data->regmap);
	}

	for (i = 0; i < NUM_DA8XX_PRU_EVTOUT; i++) {
		data->evtout_irq[i] = platform_get_irq(pdev, i);
		if (data->evtout_irq[i] < 0) {
			dev_err(dev, "failed to get IRQ for EVTOUT%d\n", i);
			return data->evtout_irq[i];
		}
	}

	pm_runtime_enable(dev);

	ret = pm_runtime_get_sync(dev);
	if (ret < 0)
		goto err_pm_runtime_disable;

	ret = rproc_add(rproc);
	if (ret < 0)
		goto err_pm_runtime_put;

	return 0;

err_pm_runtime_put:
	pm_runtime_put(dev);
err_pm_runtime_disable:
	pm_runtime_disable(dev);

	return ret;
}

static int da8xx_pru_rproc_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct da8xx_pru_data *data = platform_get_drvdata(pdev);

	rproc_del(data->rproc);
	pm_runtime_put(dev);
	pm_runtime_disable(dev);
	rproc_free(data->rproc);

	return 0;
}

static const struct of_device_id da8xx_pru_rproc_of_match[] = {
	{ .compatible = "ev3dev,da850-pru-rproc", },
	{ },
};
MODULE_DEVICE_TABLE(of, da8xx_pru_rproc_of_match);

static struct platform_driver da8xx_pru_rproc_driver = {
	.probe	= da8xx_pru_rproc_probe,
	.remove	= da8xx_pru_rproc_remove,
	.driver	= {
		.name = "da8xx-pru-rproc",
		.of_match_table = da8xx_pru_rproc_of_match,
	},
};

module_platform_driver(da8xx_pru_rproc_driver);

MODULE_AUTHOR("David Lechner <david@lechnology.com>");
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Remoteproc driver for TI AM18xx/OMAPL138 PRU");
