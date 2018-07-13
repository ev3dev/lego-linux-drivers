// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2018 David Lechner <david@lechnology.com>
 *
 * Remoteproc driver for TI Programmable Realtime Unit
 */

#include <linux/bitops.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/remoteproc.h>
#include <linux/regmap.h>
#include <linux/types.h>

#include "../../../remoteproc/remoteproc_internal.h"

/* control/status registers */
#define TI_PRU_CS_CONTROL	0x0
#define TI_PRU_CS_STATUS	0x4
#define TI_PRU_CS_WAKEUP	0x8
#define TI_PRU_CS_CYCLECNT	0xc

/* control register bits (for TI_PRU_CS_CONTROL) */
#define TI_PRU_CONTROL_PCRESETVAL	GENMASK(31, 16)
#define TI_PRU_CONTROL_RUNSTATE		BIT(15)
#define TI_PRU_CONTROL_SINGLESTEP	BIT(8)
#define TI_PRU_CONTROL_COUNTENABLE	BIT(3)
#define TI_PRU_CONTROL_SLEEPING		BIT(2)
#define TI_PRU_CONTROL_ENABLE		BIT(1)
#define TI_PRU_CONTROL_SOFTRESET	BIT(0)

/* status bits (for TI_PRU_CS_STATUS) */
#define TI_PRU_STATUS_PCOUNTER		GENMASK(15, 0)

/**
 * ti_pru_data - private data for each PRU core
 * @ctrl: regmap of the PRU control/status register
 * @dram: the local PRU data RAM
 * @iram: the local PRU instruction RAM
 * @vq_to_pru_irq: virtualqueue ARM to PRU system event
 * @vq_from_pru_irq: virtualqueue PRU to ARM system event
 */
struct ti_pru_data {
	struct regmap *ctrl;
	void __iomem *dram;
	void __iomem *iram;
	int vq_to_pru_irq;
	int vq_from_pru_irq;
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

static void ti_pru_rproc_kick(struct rproc *rproc, int vqid)
{
	struct ti_pru_data *pru = rproc->priv;

	irq_set_irqchip_state(pru->vq_to_pru_irq, IRQCHIP_STATE_PENDING, true);
}

static void *ti_pru_rproc_da_to_va(struct rproc *rproc, u64 da, int len,
				   int page)
{
	struct ti_pru_data *pru = rproc->priv;

	// TODO: to be safe, should check that len is within memory region

	if (page == 0)
		return pru->iram + da;

	if (page == 1)
		return pru->dram + da;

	return NULL;
}

static const struct rproc_ops ti_pru_rproc_ops = {
	.start = ti_pru_rproc_start,
	.stop = ti_pru_rproc_stop,
	.kick = ti_pru_rproc_kick,
	.da_to_va = ti_pru_rproc_da_to_va,
};

static struct regmap_config ti_pru_ctrl_regmap_config = {
	.name = "ctrl",
	.reg_bits = 32,
	.val_bits = 32,
	.reg_stride = 4,
	.max_register = 0x2c,
};

static const struct of_device_id ti_pru_rproc_of_match[] = {
	{ .compatible = "ev3dev,da850-pru-rproc", },
	{ },
};
MODULE_DEVICE_TABLE(of, ti_pru_rproc_of_match);

static irqreturn_t ti_pru_vq_irq_thread(int irq, void *p)
{
	struct rproc *rproc = p;

	rproc_vq_interrupt(rproc, 0);
	rproc_vq_interrupt(rproc, 1);

	return IRQ_HANDLED;
}

static void ti_pru_free_rproc(void *data)
{
	struct rproc *rproc = data;

	rproc_free(rproc);
}

static int ti_pru_rproc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	const char *name, *firmware = NULL;
	struct rproc *rproc;
	struct ti_pru_data *pru;
	struct resource *res;
	void __iomem *ctrl;
	int err;

	name = dev->of_node ? dev->of_node->name : dev_name(dev);
	device_property_read_string(dev, "firmware", &firmware);

	rproc = rproc_alloc(dev, name, &ti_pru_rproc_ops, firmware, sizeof(*pru));
	if (!rproc)
		return -ENOMEM;

	devm_add_action(dev, ti_pru_free_rproc, rproc);
	platform_set_drvdata(pdev, rproc);

	/* auto boot only if device tree specified firmware file */
	rproc->auto_boot = firmware != NULL;

	pru = rproc->priv;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "dram");
	pru->dram = devm_ioremap_resource(dev, res);
	if (!pru->dram) {
		dev_err(dev, "%s: could not get dram\n", name);
		return -ENOMEM;
	}

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "ctrl");
	ctrl = devm_ioremap_resource(dev, res);
	if (!ctrl) {
		dev_err(dev, "%s: could not get ctrl\n", name);
		return -ENOMEM;
	}

	pru->ctrl = devm_regmap_init_mmio(dev, ctrl,
					  &ti_pru_ctrl_regmap_config);
	if (IS_ERR(pru->ctrl))
		return PTR_ERR(pru->ctrl);

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "iram");
	pru->iram = devm_ioremap_resource(dev, res);
	if (!pru->iram) {
		dev_err(dev, "%s: could not get iram\n", name);
		return -ENOMEM;
	}

	pru->vq_to_pru_irq = platform_get_irq_byname(pdev, "vq-to-pru");
	if (pru->vq_to_pru_irq < 0) {
		dev_err(dev, "failed to get vq-to-pru IRQ\n");
		return pru->vq_to_pru_irq;
	}

	/* no_action should never be called since this IRQ is routed to PRU */
	err = devm_request_irq(dev, pru->vq_to_pru_irq, no_action, 0,
			       name, rproc);
	if (err < 0) {
		dev_err(dev, "failed to request vq-to-pru IRQ\n");
		return err;
	}

	pru->vq_from_pru_irq = platform_get_irq_byname(pdev, "vq-from-pru");
	if (pru->vq_from_pru_irq < 0) {
		dev_err(dev, "failed to get vq-from-pru IRQ\n");
		return pru->vq_from_pru_irq;
	}

	err = devm_request_threaded_irq(dev, pru->vq_from_pru_irq, NULL,
				 	ti_pru_vq_irq_thread, IRQF_ONESHOT,
					name, rproc);
	if (err < 0) {
		dev_err(dev, "failed to request vq-from-pru IRQ\n");
		return err;
	}

	return rproc_add(rproc);;
}

static int ti_pru_rproc_remove(struct platform_device *pdev)
{
	struct rproc *rproc = platform_get_drvdata(pdev);

	rproc_del(rproc);

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
