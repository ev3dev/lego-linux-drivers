// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2018 David Lechner <david@lechnology.com>
 *
 * Interrupt Controller driver for TI Programmable Realtime Unit
 */

#include <linux/bitops.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/irqchip/chained_irq.h>
#include <linux/irqdomain.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/types.h>

#define TI_PRU_INTC_NUM_PRU_EVENT	64
#define TI_PRU_INTC_NUM_HOST_EVENT	8
#define TI_PRU_INTC_HOST_OFFSET		2
#define TI_PRU_INTC_HOST_NONE		(-1)

/* interrupt contoller registers */
#define TI_PRU_INTC_GLBLEN		0x10
#define TI_PRU_INTC_STATIDXSET		0x20
#define TI_PRU_INTC_STATIDXCLR		0x24
#define TI_PRU_INTC_ENIDXSET		0x28
#define TI_PRU_INTC_ENIDXCLR		0x2c
#define TI_PRU_INTC_HSTINTENIDXSET	0x34
#define TI_PRU_INTC_STATESETINT(n)	(0x200 + (n) * 4)
#define TI_PRU_INTC_CHANMAP(n)		(0x400 + (n) * 4)
#define TI_PRU_INTC_HOSTMAP(n)		(0x800 + (n) * 4)
#define TI_PRU_INTC_HOSTINTPRIIDX(n)	(0x900 + (n) * 4)
#define TI_PRU_INTC_POLARITY(n)		(0xd00 + (n) * 4)
#define TI_PRU_INTC_TYPE(n)		(0xd80 + (n) * 4)

/* HOSTINTPRIIDX bits */
#define TI_PRU_INTC_HOSTINTPRIIDX_NONE		BIT(31)
#define TI_PRU_INTC_HOSTINTPRIIDX_PRI_INDEX	GENMASK(9, 0)

enum ti_pru_intc_event_select {
	TI_PRU_EVTSEL_0,
	TI_PRU_EVTSEL_1,
	TI_PRU_EVTSEL_ANY = -1,
};

/**
 * ti_pru_intc_system_event - private data for individual system event
 * @host: the host (channel) this event is mapped to
 */
struct ti_pru_intc_system_event {
	int host;
};

/**
 * ti_pru_intc_data - private platform driver data
 * @domain: the interrupt domain
 * irqchip: the irqchip instance
 * @dev: the platform device
 * @regmap: regmap of the interrupt controller
 * @events: array of system event data
 * @num_events: the number of system events;
 * @num_hosts: the number host interrupts
 * @host_irqs: the ARM INTC IRQs for PRU events
 * @has_event_select: this interrupt controller has an event select switch
 */
struct ti_pru_intc_data {
	struct irq_domain *domain;
	struct irq_chip irqchip;
	struct device *dev;
	struct regmap *regmap;
	struct ti_pru_intc_system_event events[TI_PRU_INTC_NUM_PRU_EVENT];
	int num_events;
	int num_hosts;
	int host_irqs[TI_PRU_INTC_NUM_HOST_EVENT];
	bool has_event_select;
};

/**
 * ti_pru_intc_set_channel_map - configure interrupt event to channel mapping
 * @intc: the interrtup controller regmap
 * @event: the source event
 * @ch: the channel to be assigned to the event
 */
static void ti_pru_intc_set_channel_map(struct regmap *regmap, int event, int ch)
{
	u32 shift, mask, val;

	/* 4 channels per 32-bit register */
	shift = event % 4 * 8;
	mask = 0xff << shift;
	val = ch << shift;

	regmap_write_bits(regmap, TI_PRU_INTC_CHANMAP(event / 4), mask, val);
}

/**
 * ti_pru_intc_set_host_map - configure interrupt channel to host mapping
 * @intc: the interrtup controller regmap
 * @ch: the source channel
 * @host: the host interrupt to be assigned to the channel
 */
static void ti_pru_intc_set_host_map(struct regmap *regmap, int ch, int host)
{
	u32 shift, mask, val;

	/* 4 hosts per 32-bit register */
	shift = ch % 4 * 8;
	mask = 0xff << shift;
	val = host << shift;

	regmap_write_bits(regmap, TI_PRU_INTC_HOSTMAP(ch / 4), mask, val);
}

static void ti_pru_intc_irq_ack(struct irq_data *data)
{
	struct ti_pru_intc_data *intc = irq_data_get_irq_chip_data(data);

	regmap_write(intc->regmap, TI_PRU_INTC_STATIDXCLR, data->hwirq);
}

static void ti_pru_intc_irq_mask(struct irq_data *data)
{
	struct ti_pru_intc_data *intc = irq_data_get_irq_chip_data(data);

	regmap_write(intc->regmap, TI_PRU_INTC_ENIDXCLR, data->hwirq);
}

static void ti_pru_intc_irq_unmask(struct irq_data *data)
{
	struct ti_pru_intc_data *intc = irq_data_get_irq_chip_data(data);

	regmap_write(intc->regmap, TI_PRU_INTC_ENIDXSET, data->hwirq);
}

static int ti_pru_intc_irq_get_irqchip_state(struct irq_data *data,
					     enum irqchip_irq_state which,
					     bool *state)
{
	struct ti_pru_intc_data *intc = irq_data_get_irq_chip_data(data);
	u32 reg, mask, val;

	if (which != IRQCHIP_STATE_PENDING)
		return -EINVAL;

	reg = TI_PRU_INTC_STATESETINT(data->hwirq / 32);
	mask = BIT(data->hwirq % 32);

	regmap_read(intc->regmap, reg, &val);

	*state = val & mask;

	return 0;
}

static int ti_pru_intc_irq_set_irqchip_state(struct irq_data *data,
					     enum irqchip_irq_state which,
					     bool state)
{
	struct ti_pru_intc_data *intc = irq_data_get_irq_chip_data(data);
	u32 reg, mask, val;

	if (which != IRQCHIP_STATE_PENDING)
		return -EINVAL;
	
	reg = TI_PRU_INTC_STATESETINT(data->hwirq / 32);
	mask = BIT(data->hwirq % 32);
	val = state ? ~0 : 0;

	regmap_write_bits(intc->regmap, reg, mask, val);

	return 0;
}

static int ti_pru_intc_map(struct irq_domain *d, unsigned int virq,
			   irq_hw_number_t hw)
{
	struct ti_pru_intc_data *intc = d->host_data;
	int host = intc->events[hw].host;
	int err;

	if (host == TI_PRU_INTC_HOST_NONE)
		return -EINVAL;

	ti_pru_intc_set_channel_map(intc->regmap, hw, host);
	ti_pru_intc_set_host_map(intc->regmap, host, host);
	regmap_write(intc->regmap, TI_PRU_INTC_HSTINTENIDXSET, host);

	err = irq_set_chip_data(virq, intc);
	if (err)
		return err;

	irq_set_chip_and_handler(virq, &intc->irqchip, handle_level_irq);

	return 0;
}

static void ti_pru_intc_unmap(struct irq_domain *d, unsigned int virq)
{
	irq_set_chip_and_handler(virq, NULL, NULL);
	irq_set_chip_data(virq, NULL);
}

static int ti_pru_intc_xlate(struct irq_domain *d, struct device_node *node,
			     const u32 *intspec, unsigned int intsize,
			     unsigned long *out_hwirq, unsigned int *out_type)
{
	struct ti_pru_intc_data *intc = d->host_data;
	int num_cells = intc->has_event_select ? 3 : 2;
	int sys_event, host;

	if (WARN_ON(intsize != num_cells))
		return -EINVAL;

	sys_event = intspec[0];
	if (sys_event >= intc->num_events)
		return -EINVAL;

	if (intc->has_event_select) {
		enum ti_pru_intc_event_select evtsel = intspec[1];

		// TODO: need to implement CFGCHIP3[3].PRUSSEVTSEL
		// the first xlate that is != TI_PRU_EVTSEL_ANY should set
		// CFGCHIP3[3].PRUSSEVTSEL, then any subsequent requests that
		// would change CFGCHIP3[3].PRUSSEVTSEL should return -EBUSY
		if (evtsel == TI_PRU_EVTSEL_1)
			return -EBUSY;
	}

	host = intspec[intsize - 1];
	if (host >= intc->num_hosts)
		return -EINVAL;

	intc->events[sys_event].host = host;

	*out_hwirq = sys_event;
	*out_type = IRQ_TYPE_NONE;

	return 0;
}

static const struct irq_domain_ops ti_pru_intc_irq_domain_ops = {
	.map = ti_pru_intc_map,
	.unmap = ti_pru_intc_unmap,
	.xlate = ti_pru_intc_xlate,
};

static void ti_pru_intc_irq_handler(struct irq_desc *desc)
{
	unsigned int irq = irq_desc_get_irq(desc);
	struct irq_chip *chip = irq_desc_get_chip(desc);
	struct ti_pru_intc_data *intc = irq_get_handler_data(irq);
	u32 host, val;
	unsigned int hwirq, virq;
	int i;

	chained_irq_enter(chip, desc);

	for (i = 0; i < TI_PRU_INTC_NUM_HOST_EVENT; i++) {
		if (intc->host_irqs[i] == irq)
			break;
	}

	if (i == TI_PRU_INTC_NUM_HOST_EVENT) {
		chained_irq_exit(chip, desc);
		return;
	}

	host = i + TI_PRU_INTC_HOST_OFFSET;

	/* get highest priority pending PRUSS system event */
	regmap_read(intc->regmap, TI_PRU_INTC_HOSTINTPRIIDX(host), &val);

	while (!(val & TI_PRU_INTC_HOSTINTPRIIDX_NONE)) {
		hwirq = val & TI_PRU_INTC_HOSTINTPRIIDX_PRI_INDEX;
		virq = irq_linear_revmap(intc->domain, hwirq);

		if (unlikely(!virq))
			regmap_write(intc->regmap, TI_PRU_INTC_STATIDXCLR, hwirq);
		else
			generic_handle_irq(virq);

		/* get next system event */
		regmap_read(intc->regmap, TI_PRU_INTC_HOSTINTPRIIDX(host), &val);
	}

	chained_irq_exit(chip, desc);
}

static void ti_pru_intc_init_events(struct ti_pru_intc_system_event *events)
{
	int i;

	for (i = 0; i < TI_PRU_INTC_NUM_PRU_EVENT; i++)
		events[i].host = TI_PRU_INTC_HOST_NONE;
}

static struct regmap_config ti_pru_intc_regmap_config = {
	.name = "intc",
	.reg_bits = 32,
	.val_bits = 32,
	.reg_stride = 4,
	.max_register = 0x1500,
};

static int ti_pru_intc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct ti_pru_intc_data *intc;
	struct resource *res;
	void __iomem *base;
	int count, i, err;

	intc = devm_kzalloc(dev, sizeof(*intc), GFP_KERNEL);

	platform_set_drvdata(pdev, intc);

	intc->dev = dev;
	ti_pru_intc_init_events(intc->events);
	/* TODO: this info should come from device id lookup */
	intc->num_events = TI_PRU_INTC_NUM_PRU_EVENT;
	intc->num_hosts = 10;
	intc->has_event_select = true;

	intc->irqchip.parent_device = dev;
	intc->irqchip.name = dev_name(dev);
	intc->irqchip.irq_ack = ti_pru_intc_irq_ack;
	intc->irqchip.irq_mask = ti_pru_intc_irq_mask;
	intc->irqchip.irq_unmask = ti_pru_intc_irq_unmask;
	intc->irqchip.irq_get_irqchip_state = ti_pru_intc_irq_get_irqchip_state;
	intc->irqchip.irq_set_irqchip_state = ti_pru_intc_irq_set_irqchip_state;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	base = devm_ioremap_resource(dev, res);
	if (!base)
		return -ENOMEM;

	intc->regmap = devm_regmap_init_mmio(dev, base,
					     &ti_pru_intc_regmap_config);
	if (IS_ERR(intc->regmap))
		return PTR_ERR(intc->regmap);

	count = platform_irq_count(pdev);
	if (count != intc->num_hosts - TI_PRU_INTC_HOST_OFFSET) {
		dev_err(dev, "incorrect number of event interrupts\n");
		return -EINVAL;
	}

	/* shouldn't happen, but just in case to prevent crash */
	if (count > TI_PRU_INTC_NUM_HOST_EVENT)
		return -EINVAL;

	for (i = 0; i < count; i++) {
		int irq = platform_get_irq(pdev, i);

		if (irq < 0) {
			dev_err(dev, "failed to get IRQ %d\n", i);
			return irq;
		}

		intc->host_irqs[i] = irq;

		irq_set_chained_handler_and_data(irq, ti_pru_intc_irq_handler,
						 intc);
	}

	pm_runtime_enable(dev);

	err = pm_runtime_get_sync(dev);
	if (err < 0)
		goto err_pm_runtime_disable;

	intc->domain = irq_domain_add_linear(dev->of_node, intc->num_events,
					     &ti_pru_intc_irq_domain_ops, intc);
	if (!intc->domain) {
		err = -ENOMEM;
		goto err_pm_runtime_disable;
	}

	/* system events require polarity = 1 and type = 0 */
	regmap_write(intc->regmap, TI_PRU_INTC_POLARITY(0), ~0);
	regmap_write(intc->regmap, TI_PRU_INTC_POLARITY(1), ~0);
	regmap_write(intc->regmap, TI_PRU_INTC_TYPE(0), 0);
	regmap_write(intc->regmap, TI_PRU_INTC_TYPE(1), 0);

	regmap_write_bits(intc->regmap, TI_PRU_INTC_GLBLEN, 1, 1);

	return 0;

err_pm_runtime_disable:
	pm_runtime_disable(dev);

	return err;
}

static int ti_pru_intc_remove(struct platform_device *pdev)
{
	struct ti_pru_intc_data *intc = platform_get_drvdata(pdev);

	regmap_write_bits(intc->regmap, TI_PRU_INTC_GLBLEN, 1, 0);
	irq_domain_remove(intc->domain);
	pm_runtime_disable(&pdev->dev);

	return 0;
}

static const struct of_device_id ti_pru_intc_of_match[] = {
	{ .compatible = "ev3dev,da850-pru-intc" },
	{ },
};
MODULE_DEVICE_TABLE(of, ti_pru_intc_of_match);

static struct platform_driver ti_pru_intc_driver = {
	.probe	= ti_pru_intc_probe,
	.remove	= ti_pru_intc_remove,
	.driver	= {
		.name = "ti-pru-intc",
		.of_match_table = ti_pru_intc_of_match,
	},
};
module_platform_driver(ti_pru_intc_driver);

MODULE_AUTHOR("David Lechner <david@lechnology.com>");
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Interrupt controller driver for TI PRU");
