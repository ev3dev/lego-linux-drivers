// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2018 David Lechner <david@lechnology.com>
 *
 * Remoteproc driver for TI Programmable Realtime Unit
 */

#include <linux/bitops.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/remoteproc.h>
#include <linux/regmap.h>
#include <linux/sizes.h>
#include <linux/types.h>

#define SZ_12K 0x3000

/* control/status registers */
#define TI_PRU_CS_CONTROL	0x0
#define TI_PRU_CS_STATUS	0x4
#define TI_PRU_CS_WAKEUP	0x8
#define TI_PRU_CS_CYCLECNT	0xc

/* control register bits */
#define TI_PRU_CONTROL_PCRESETVAL	GENMASK(31, 16)
#define TI_PRU_CONTROL_RUNSTATE		BIT(15)
#define TI_PRU_CONTROL_SINGLESTEP	BIT(8)
#define TI_PRU_CONTROL_COUNTENABLE	BIT(3)
#define TI_PRU_CONTROL_SLEEPING		BIT(2)
#define TI_PRU_CONTROL_ENABLE		BIT(1)
#define TI_PRU_CONTROL_SOFTRESET	BIT(0)

/* status bits */
#define TI_PRU_STATUS_PCOUNTER		GENMASK(15, 0)

enum ti_pru {
	TI_PRU0,
	TI_PRU1,
	NUM_TI_PRU
};

enum ti_pru_type {
	TI_PRU_TYPE_AM18XX,
	TI_PRU_TYPE_AM335X,
	NUM_TI_PRU_TYPE
};

enum ti_pru_evtout {
	TI_PRU_EVTOUT0,
	TI_PRU_EVTOUT1,
	TI_PRU_EVTOUT2,
	TI_PRU_EVTOUT3,
	TI_PRU_EVTOUT4,
	TI_PRU_EVTOUT5,
	TI_PRU_EVTOUT6,
	TI_PRU_EVTOUT7,
	NUM_TI_PRU_EVTOUT
};

struct ti_pru_mem_region {
	off_t offset;
	size_t size;
};

/**
 * ti_pru_shared_info - common init info for the PRUSS
 * @ram: shared RAM, if present
 * @intc: interrupt controller
 */
struct ti_pru_shared_info {
	struct ti_pru_mem_region ram;
	struct ti_pru_mem_region intc;
};

/**
 * ti_pru_info - init info each individual PRU
 * @ram: PRU RAM
 * @ctrl: PRU control/status registers
 * @dbg: PRU dbg registers
 * @inst: instruction RAM
 */
struct ti_pru_info {
	struct ti_pru_mem_region ram;
	struct ti_pru_mem_region ctrl;
	struct ti_pru_mem_region dbg;
	struct ti_pru_mem_region inst;
};

struct ti_pru_device_info {
	struct ti_pru_shared_info shared;
	struct ti_pru_info pru[NUM_TI_PRU];
};

static const struct ti_pru_device_info ti_pru_devices[NUM_TI_PRU_TYPE] = {
	[TI_PRU_TYPE_AM18XX] = {
		.shared = {
			.intc =	{ .offset = 0x4000,	.size = SZ_12K,	},
		},
		.pru[TI_PRU0] = {
			.ram =	{ .offset = 0x0000,	.size = SZ_512,	},
			.ctrl =	{ .offset = 0x7000,	.size = SZ_1K,	},
			.dbg =	{ .offset = 0x7400,	.size = SZ_1K,	},
			.inst =	{ .offset = 0x8000,	.size = SZ_4K,	},
		},
		.pru[TI_PRU1] = {
			.ram =	{ .offset = 0x2000,	.size = SZ_512,	},
			.ctrl =	{ .offset = 0x7800,	.size = SZ_1K,	},
			.dbg =	{ .offset = 0x7c00,	.size = SZ_1K,	},
			.inst =	{ .offset = 0xc000,	.size = SZ_4K,	},
		},
	},
	[TI_PRU_TYPE_AM335X] = {
		.shared = {
			.ram =	{ .offset = 0x10000,	.size = SZ_12K,	},
			.intc =	{ .offset = 0x20000,	.size = SZ_8K,	},
		},
		.pru[TI_PRU0] = {
			.ram =	{ .offset = 0x00000,	.size = SZ_8K,	},
			.ctrl =	{ .offset = 0x22000,	.size = SZ_1K,	},
			.dbg =	{ .offset = 0x22400,	.size = SZ_1K,	},
			.inst =	{ .offset = 0x34000,	.size = SZ_8K,	},
		},
		.pru[TI_PRU1] = {
			.ram =	{ .offset = 0x02000,	.size = SZ_8K,	},
			.ctrl =	{ .offset = 0x24000,	.size = SZ_1K,	},
			.dbg =	{ .offset = 0x24400,	.size = SZ_1K,	},
			.inst =	{ .offset = 0x38000,	.size = SZ_8K,	},
		},
	},
};

struct ti_pru_shared_data {
	const struct ti_pru_shared_info *info;
	struct device *dev;
	void __iomem *base;
	int evtout_irq[NUM_TI_PRU_EVTOUT];
	struct rproc *pru[NUM_TI_PRU];
};

struct ti_pru_data {
	const struct ti_pru_info *info;
	struct ti_pru_shared_data *shared;
	struct regmap *ctrl;
};

static int ti_pru_rproc_start(struct rproc *rproc)
{
	struct ti_pru_data *pru = rproc->priv;
	u32 val;

	val = (rproc->bootaddr >> 2) << (ffs(TI_PRU_CONTROL_PCRESETVAL) - 1);
	val |= TI_PRU_CONTROL_ENABLE;

	return regmap_write(pru->ctrl, TI_PRU_CS_CONTROL, val);
}

static int ti_pru_rproc_stop(struct rproc *rproc)
{
	struct ti_pru_data *pru = rproc->priv;
	u32 mask;

	mask = TI_PRU_CONTROL_ENABLE;

	return regmap_write_bits(pru->ctrl, TI_PRU_CS_CONTROL, mask, 0);
}

static void *ti_pru_rproc_da_to_va(struct rproc *rproc, u64 da, int len,
				   int page)
{
	struct ti_pru_data *pru = rproc->priv;
	struct ti_pru_shared_data *shared = pru->shared;

	if (page == 0) {
		if (da + len > pru->info->inst.size)
			return ERR_PTR(-EINVAL);

		return shared->base + pru->info->inst.offset + da;
	}

	if (page == 1) {
		if (da + len > pru->info->ram.size)
			return ERR_PTR(-EINVAL);

		return shared->base + pru->info->ram.offset + da;
	}

	return ERR_PTR(-EINVAL);
}

static const struct rproc_ops ti_pru_rproc_ops = {
	.start = ti_pru_rproc_start,
	.stop = ti_pru_rproc_stop,
	.da_to_va = ti_pru_rproc_da_to_va,
};

static struct regmap_config ti_pru_ctrl_regmap_config = {
	.reg_bits = 32,
	.val_bits = 32,
	.reg_stride = 4,
	.max_register = 0x2c,
};

static const struct of_device_id ti_pru_rproc_of_match[] = {
	{
		.compatible = "ev3dev,da850-pru-rproc",
		.data = &ti_pru_devices[TI_PRU_TYPE_AM18XX]
	},
	{
		.compatible = "ev3dev,am3352-pru-rproc",
		.data = &ti_pru_devices[TI_PRU_TYPE_AM335X]
	},
	{ },
};
MODULE_DEVICE_TABLE(of, ti_pru_rproc_of_match);

static struct rproc *ti_pru_init_one_rproc(struct ti_pru_shared_data *shared,
					   const struct ti_pru_info *info,
					   enum ti_pru id)
{
	struct device *dev = shared->dev;
	const char *name;
	struct rproc *rproc;
	struct ti_pru_data *pru;

	name = devm_kasprintf(dev, GFP_KERNEL, "pru%u", id);
	if (!name)
		return ERR_PTR(-ENOMEM);

	rproc = rproc_alloc(dev, name, &ti_pru_rproc_ops, NULL, sizeof(*pru));
	if (!rproc)
		return ERR_PTR(-ENOMEM);

	/* don't auto-boot for now - bad firmware can lock up the system */
	rproc->auto_boot = false;

	pru = rproc->priv;
	pru->info = info;
	pru->shared = shared;

	pru->ctrl = devm_regmap_init_mmio(&rproc->dev,
					  shared->base + info->ctrl.offset,
					  &ti_pru_ctrl_regmap_config);
	if (IS_ERR(pru->ctrl)) {
		dev_err(&rproc->dev, "failed to init regmap\n");
		rproc_free(rproc);
		return ERR_CAST(pru->ctrl);
	}

	return rproc;
}

static int ti_pru_rproc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	const struct of_device_id *of_id;
	const struct ti_pru_device_info *info;
	struct ti_pru_shared_data *shared;
	struct resource *res;
	int i, err;

	of_id = of_match_device(ti_pru_rproc_of_match, dev);
	if (!of_id || !of_id->data)
		return -EINVAL;

	info = of_id->data;

	shared = devm_kzalloc(dev, sizeof(*shared), GFP_KERNEL);

	platform_set_drvdata(pdev, shared);

	shared->info = &info->shared;
	shared->dev = dev;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	shared->base = devm_ioremap_resource(dev, res);
	if (IS_ERR(shared->base)) {
		dev_err(dev, "failed to ioremap resource\n");
		return PTR_ERR(shared->base);
	}

	for (i = 0; i < NUM_TI_PRU_EVTOUT; i++) {
		shared->evtout_irq[i] = platform_get_irq(pdev, i);
		if (shared->evtout_irq[i] < 0) {
			dev_err(dev, "failed to get IRQ for EVTOUT%d\n", i);
			return shared->evtout_irq[i];
		}
	}

	shared->pru[TI_PRU0] = ti_pru_init_one_rproc(shared, &info->pru[TI_PRU0],
						     TI_PRU0);
	if (IS_ERR(shared->pru[TI_PRU0]))
		return PTR_ERR(shared->pru[TI_PRU0]);

	shared->pru[TI_PRU1] = ti_pru_init_one_rproc(shared, &info->pru[TI_PRU1],
						     TI_PRU1);
	if (IS_ERR(shared->pru[TI_PRU1])) {
		err = PTR_ERR(shared->pru[TI_PRU1]);
		goto err_free_pru0;
	}

	pm_runtime_enable(dev);

	err = pm_runtime_get_sync(dev);
	if (err < 0)
		goto err_pm_runtime_disable;

	err = rproc_add(shared->pru[TI_PRU0]);
	if (err < 0)
		goto err_pm_runtime_put;

	err = rproc_add(shared->pru[TI_PRU1]);
	if (err < 0)
		goto err_del_pru0;

	return 0;

err_del_pru0:
	rproc_del(shared->pru[TI_PRU0]);
err_pm_runtime_put:
	pm_runtime_put(dev);
err_pm_runtime_disable:
	pm_runtime_disable(dev);
	rproc_free(shared->pru[TI_PRU1]);
err_free_pru0:
	rproc_free(shared->pru[TI_PRU0]);

	return err;
}

static int ti_pru_rproc_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct ti_pru_shared_data *shared = platform_get_drvdata(pdev);

	rproc_del(shared->pru[TI_PRU1]);
	rproc_del(shared->pru[TI_PRU0]);
	pm_runtime_put(dev);
	pm_runtime_disable(dev);
	rproc_free(shared->pru[TI_PRU1]);
	rproc_free(shared->pru[TI_PRU0]);

	return 0;
}

static struct platform_driver ti_pru_rproc_driver = {
	.probe	= ti_pru_rproc_probe,
	.remove	= ti_pru_rproc_remove,
	.driver	= {
		.name = "ti-pru-rproc",
		.of_match_table = ti_pru_rproc_of_match,
	},
};

module_platform_driver(ti_pru_rproc_driver);

MODULE_AUTHOR("David Lechner <david@lechnology.com>");
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Remoteproc driver for TI PRU");
