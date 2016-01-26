/**
 * The industrial I/O periodic hrtimer trigger driver
 *
 * Copyright (C) Intuitive Aerial AB
 * Written by Marten Svanfeldt, marten@intuitiveaerial.com
 * Copyright (C) 2012, Analog Device Inc.
 *  Author: Lars-Peter Clausen <lars@metafoo.de>
 * Copyright (C) 2015, Intel Corporation
 *
 * Slightly modified for kernels < 4.5
 * Copyright (C) 2016 David Lechner <david@lechnology.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 */
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/hrtimer.h>

#include <linux/iio/iio.h>
#include <linux/iio/trigger.h>

/* default sampling frequency - 100Hz */
#define HRTIMER_DEFAULT_SAMPLING_FREQUENCY 100

struct iio_hrtimer_info {
	struct iio_trigger *trigger;
	struct hrtimer timer;
	unsigned long sampling_frequency;
	ktime_t period;
};

static
ssize_t iio_hrtimer_show_sampling_frequency(struct device *dev,
					    struct device_attribute *attr,
					    char *buf)
{
	struct iio_trigger *trig = to_iio_trigger(dev);
	struct iio_hrtimer_info *info = iio_trigger_get_drvdata(trig);

	return snprintf(buf, PAGE_SIZE, "%lu\n", info->sampling_frequency);
}

static
ssize_t iio_hrtimer_store_sampling_frequency(struct device *dev,
					     struct device_attribute *attr,
					     const char *buf, size_t len)
{
	struct iio_trigger *trig = to_iio_trigger(dev);
	struct iio_hrtimer_info *info = iio_trigger_get_drvdata(trig);
	unsigned long val;
	int ret;

	ret = kstrtoul(buf, 10, &val);
	if (ret)
		return ret;

	if (!val || val > NSEC_PER_SEC)
		return -EINVAL;

	info->sampling_frequency = val;
	info->period = ktime_set(0, NSEC_PER_SEC / val);

	return len;
}

static DEVICE_ATTR(sampling_frequency, S_IRUGO | S_IWUSR,
		   iio_hrtimer_show_sampling_frequency,
		   iio_hrtimer_store_sampling_frequency);

static struct attribute *iio_hrtimer_attrs[] = {
	&dev_attr_sampling_frequency.attr,
	NULL
};

static const struct attribute_group iio_hrtimer_attr_group = {
	.attrs = iio_hrtimer_attrs,
};

static const struct attribute_group *iio_hrtimer_attr_groups[] = {
	&iio_hrtimer_attr_group,
	NULL
};

static enum hrtimer_restart iio_hrtimer_trig_handler(struct hrtimer *timer)
{
	struct iio_hrtimer_info *info;

	info = container_of(timer, struct iio_hrtimer_info, timer);

	hrtimer_forward_now(timer, info->period);
	iio_trigger_poll(info->trigger);

	return HRTIMER_RESTART;
}

static int iio_trig_hrtimer_set_state(struct iio_trigger *trig, bool state)
{
	struct iio_hrtimer_info *trig_info;

	trig_info = iio_trigger_get_drvdata(trig);

	if (state)
		hrtimer_start(&trig_info->timer, trig_info->period,
			      HRTIMER_MODE_REL);
	else
		hrtimer_cancel(&trig_info->timer);

	return 0;
}

static const struct iio_trigger_ops iio_hrtimer_trigger_ops = {
	.owner = THIS_MODULE,
	.set_trigger_state = iio_trig_hrtimer_set_state,
};

__must_check struct iio_trigger *iio_trig_hrtimer_probe(const char *name)
{
	struct iio_hrtimer_info *trig_info;
	int ret;

	trig_info = kzalloc(sizeof(*trig_info), GFP_KERNEL);
	if (!trig_info)
		return ERR_PTR(-ENOMEM);

	trig_info->trigger = iio_trigger_alloc("%s", name);
	if (!trig_info->trigger) {
		ret = -ENOMEM;
		goto err_free_trig_info;
	}

	iio_trigger_set_drvdata(trig_info->trigger, trig_info);
	trig_info->trigger->ops = &iio_hrtimer_trigger_ops;
	trig_info->trigger->dev.groups = iio_hrtimer_attr_groups;

	hrtimer_init(&trig_info->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	trig_info->timer.function = iio_hrtimer_trig_handler;

	trig_info->sampling_frequency = HRTIMER_DEFAULT_SAMPLING_FREQUENCY;
	trig_info->period = ktime_set(0, NSEC_PER_SEC /
				      trig_info->sampling_frequency);

	ret = iio_trigger_register(trig_info->trigger);
	if (ret)
		goto err_free_trigger;

	return trig_info->trigger;

err_free_trigger:
	iio_trigger_free(trig_info->trigger);
err_free_trig_info:
	kfree(trig_info);

	return ERR_PTR(ret);
}
EXPORT_SYMBOL_GPL(iio_trig_hrtimer_probe);

int iio_trig_hrtimer_remove(struct iio_trigger *trigger)
{
	struct iio_hrtimer_info *trig_info;

	trig_info = iio_trigger_get_drvdata(trigger);

	iio_trigger_unregister(trigger);

	/* cancel the timer after unreg to make sure no one rearms it */
	hrtimer_cancel(&trig_info->timer);
	iio_trigger_free(trigger);
	kfree(trig_info);

	return 0;
}
EXPORT_SYMBOL_GPL(iio_trig_hrtimer_remove);

MODULE_AUTHOR("Marten Svanfeldt <marten@intuitiveaerial.com>");
MODULE_AUTHOR("Daniel Baluta <daniel.baluta@intel.com>");
MODULE_DESCRIPTION("Periodic hrtimer trigger for the IIO subsystem");
MODULE_LICENSE("GPL v2");
