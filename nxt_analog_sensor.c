/*
 * NXT Analog Sensor device driver for LEGO Mindstorms EV3
 *
 * Copyright (C) 2013-2014 David Lechner <david@lechnology.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.

 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/device.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/legoev3/legoev3_ports.h>
#include <linux/legoev3/ev3_input_port.h>
#include <linux/legoev3/msensor_class.h>

#include <asm/bug.h>

#define NUM_NXT_ANALOG_SENSOR_MODES	2

struct nxt_analog_sensor_data {
	struct legoev3_port_device *in_port;
	struct msensor_device ms;
	struct msensor_mode_info mode_info[NUM_NXT_ANALOG_SENSOR_MODES];
	u8 mode;
};

static struct msensor_mode_info
nxt_analog_sensor_mode_info[NUM_NXT_ANALOG_SENSOR_MODES] = {
	{
		.name = "NXT-ANALOG-0",
		.raw_max = 5000,
		.pct_max = 100,
		.si_max = 5000,
		.data_sets = 1,
		.data_type = MSENSOR_DATA_S32,
	},
	{
		.name = "NXT-ANALOG-1",
		.raw_max = 5000,
		.pct_max = 100,
		.si_max = 5000,
		.data_sets = 1,
		.data_type = MSENSOR_DATA_S32,
	},
};

static u8 nxt_analog_sensor_get_mode(void *context)
{
	struct nxt_analog_sensor_data *as = context;

	return as->mode;
}

static int nxt_analog_sensor_set_mode(void *context, u8 mode)
{
	struct nxt_analog_sensor_data *as = context;

	if (mode >= NUM_NXT_ANALOG_SENSOR_MODES)
		return -EINVAL;

	if (mode == 0)
		ev3_input_port_set_pin5_gpio(as->in_port,
					     EV3_INPUT_PORT_GPIO_FLOAT);
	else
		ev3_input_port_set_pin5_gpio(as->in_port,
					     EV3_INPUT_PORT_GPIO_HIGH);
	as->mode = mode;

	return 0;
}

static void nxt_analog_sensor_cb(void *context)
{
	struct nxt_analog_sensor_data *as = context;

	*(int*)as->mode_info[0].raw_data =
		ev3_input_port_get_pin1_mv(as->in_port);
}

#define nxt_analog_sensor_attr_funcs(_name)					\
static ssize_t nxt_analog_sensor_show_##_name(struct device * dev,		\
					      struct device_attribute *attr,	\
					      char *buf)			\
{										\
	struct msensor_device *ms = to_msensor_device(dev);			\
	struct nxt_analog_sensor_data *as =					\
		container_of(ms, struct nxt_analog_sensor_data, ms);		\
										\
	return sprintf(buf, "%d\n", ms->mode_info[as->mode]._name);		\
}										\
										\
static ssize_t nxt_analog_sensor_store_##_name(struct device * dev,		\
					       struct device_attribute *attr,	\
					       const char *buf, size_t count)	\
{										\
	struct msensor_device *ms = to_msensor_device(dev);			\
	struct nxt_analog_sensor_data *as =					\
		container_of(ms, struct nxt_analog_sensor_data, ms);		\
	char *end;								\
	long value = simple_strtol(buf, &end, 0);				\
										\
	if (end == buf || value > INT_MAX || value < INT_MIN)			\
		return -EINVAL;							\
	ms->mode_info[as->mode]._name = value;					\
										\
	return count;								\
}

nxt_analog_sensor_attr_funcs(raw_min)
nxt_analog_sensor_attr_funcs(raw_max)
nxt_analog_sensor_attr_funcs(si_min)
nxt_analog_sensor_attr_funcs(si_max)

DEVICE_ATTR(raw_min, S_IRUGO | S_IWUGO, nxt_analog_sensor_show_raw_min,
	    nxt_analog_sensor_store_raw_min);
DEVICE_ATTR(raw_max, S_IRUGO | S_IWUGO, nxt_analog_sensor_show_raw_max,
	    nxt_analog_sensor_store_raw_max);
DEVICE_ATTR(scaled_min, S_IRUGO | S_IWUGO, nxt_analog_sensor_show_si_min,
	    nxt_analog_sensor_store_si_min);
DEVICE_ATTR(scaled_max, S_IRUGO | S_IWUGO, nxt_analog_sensor_show_si_max,
	    nxt_analog_sensor_store_si_max);

struct attribute *nxt_analog_sensor_attrs[] = {
	&dev_attr_raw_min.attr,
	&dev_attr_raw_max.attr,
	&dev_attr_scaled_min.attr,
	&dev_attr_scaled_max.attr,
	NULL
};

struct attribute_group nxt_analog_sensor_attr_grp = {
	.attrs = nxt_analog_sensor_attrs,
};

static int __devinit nxt_analog_sensor_probe(struct legoev3_port_device *sensor)
{
	struct nxt_analog_sensor_data *as;
	struct ev3_sensor_platform_data *pdata = sensor->dev.platform_data;
	int err;

	if (WARN_ON(!pdata))
		return -EINVAL;

	as = kzalloc(sizeof(struct nxt_analog_sensor_data), GFP_KERNEL);
	if (!as)
		return -ENOMEM;

	as->in_port = pdata->in_port;

	as->ms.type_id		= sensor->type_id;
	strncpy(as->ms.port_name, dev_name(&pdata->in_port->dev),
	        MSENSOR_PORT_NAME_SIZE);
	as->ms.num_modes	= NUM_NXT_ANALOG_SENSOR_MODES;
	as->ms.num_view_modes	= 1;
	as->ms.mode_info	= as->mode_info;
	as->ms.get_mode		= nxt_analog_sensor_get_mode;
	as->ms.set_mode		= nxt_analog_sensor_set_mode;
	as->ms.context		= as;

	memcpy(as->mode_info, nxt_analog_sensor_mode_info,
	       sizeof(struct msensor_mode_info) * NUM_NXT_ANALOG_SENSOR_MODES);

	err = register_msensor(&as->ms, &sensor->dev);
	if (err)
		goto err_register_msensor;

	err = sysfs_create_group(&as->ms.dev.kobj, &nxt_analog_sensor_attr_grp);
	if (err)
		goto err_sysfs_create_group;

	ev3_input_port_register_analog_cb(as->in_port, nxt_analog_sensor_cb, as);

	err = dev_set_drvdata(&sensor->dev, as);
	if (err)
		goto err_dev_set_drvdata;

	nxt_analog_sensor_set_mode(as, 0);

	dev_info(&sensor->dev, "Analog sensor connected to port %s\n",
		 dev_name(&as->in_port->dev));

	return 0;

err_dev_set_drvdata:
	sysfs_remove_group(&as->ms.dev.kobj, &nxt_analog_sensor_attr_grp);
err_sysfs_create_group:
	unregister_msensor(&as->ms);
err_register_msensor:
	kfree(as);

	return err;
}

static int __devexit nxt_analog_sensor_remove(struct legoev3_port_device *sensor)
{
	struct nxt_analog_sensor_data *as = dev_get_drvdata(&sensor->dev);

	dev_info(&sensor->dev, "Analog sensor removed from port %s\n",
		 dev_name(&as->in_port->dev));
	ev3_input_port_set_pin5_gpio(as->in_port, EV3_INPUT_PORT_GPIO_FLOAT);
	ev3_input_port_register_analog_cb(as->in_port, NULL, NULL);
	sysfs_remove_group(&as->ms.dev.kobj, &nxt_analog_sensor_attr_grp);
	unregister_msensor(&as->ms);
	dev_set_drvdata(&sensor->dev, NULL);
	kfree(as);
	return 0;
}

struct legoev3_port_driver nxt_analog_sensor_driver = {
	.probe	= nxt_analog_sensor_probe,
	.remove	= __devexit_p(nxt_analog_sensor_remove),
	.driver = {
		.name	= "nxt-analog-sensor",
		.owner	= THIS_MODULE,
	},
};
EXPORT_SYMBOL_GPL(nxt_analog_sensor_driver);
legoev3_port_driver(nxt_analog_sensor_driver);

MODULE_DESCRIPTION("NXT Analog sensor device driver for LEGO Mindstorms EV3");
MODULE_AUTHOR("David Lechner <david@lechnology.com>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("legoev3:nxt-analog-sensor");
