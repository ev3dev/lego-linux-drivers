/*
 * Tacho motor device class for LEGO Mindstorms EV3
 *
 * Copyright (C) 2013-2014 Ralph Hempel <rhempel@hempeldesigngroup.com>
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
#include <linux/legoev3/tacho_motor_class.h>

struct tacho_motor_mode_item {
	const char *name;
};

static struct tacho_motor_mode_item tacho_motor_mode_items[NUM_TACHO_MOTOR_MODES] = {
	[UNLIMITED_UNREG]         =  { "unlimited_unreg"         },
	[UNLIMITED_REG]           =  { "unlimited_reg"           },
	[LIMITED_REG_STEPUP]      =  { "limited_reg_stepup"      },
	[LIMITED_REG_STEPCONST]   =  { "limited_reg_stepconst"   },
	[LIMITED_REG_STEPDOWN]    =  { "limited_reg_stepdown"    },
	[LIMITED_UNREG_STEPUP]    =  { "limited_unreg_stepup"    },
	[LIMITED_UNREG_STEPCONST] =  { "limited_unreg_stepconst" },
	[LIMITED_UNREG_STEPDOWN]  =  { "limited_unreg_stepdown"  },
	[LIMITED_REG_TIMEUP]      =  { "limited_reg_timeup"      },
	[LIMITED_REG_TIMECONST]   =  { "limited_reg_timeconst"   },
	[LIMITED_REG_TIMEDOWN]    =  { "limited_reg_timedown"    },
	[LIMITED_UNREG_TIMEUP]    =  { "limited_unreg_timeup"    },
	[LIMITED_UNREG_TIMECONST] =  { "limited_unreg_timeconst" },
	[LIMITED_UNREG_TIMEDOWN]  =  { "limited_unreg_timedown"  },
	[LIMITED_STEP_SYNC]       =  { "limited_step_sync"       },
	[LIMITED_TURN_SYNC]       =  { "limited_turn_sync"       },
	[LIMITED_DIFF_TURN_SYNC]  =  { "limited_diff_turn_sync"  },
	[SYNCED_SLAVE]            =  { "synced_slave"            },
	[RAMP_DOWN_SYNC]          =  { "ramp_down_sync"          },
	[HOLD]                    =  { "hold"                    },
	[BRAKED]                  =  { "braked"                  },
	[STOP_MOTOR]              =  { "stop_motor"              },
	[IDLE]                    =  { "idle"                    },
};

static ssize_t tacho_motor_show_tacho(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct tacho_motor_device *tm = container_of(dev, struct tacho_motor_device, dev);

	return sprintf(buf, "%d\n", tm->get_tacho(tm));
}

static ssize_t tacho_motor_show_step(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct tacho_motor_device *tm = container_of(dev, struct tacho_motor_device, dev);

	return sprintf(buf, "%d\n", tm->get_step(tm));
}

static ssize_t tacho_motor_show_direction(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct tacho_motor_device *tm = container_of(dev, struct tacho_motor_device, dev);

	return sprintf(buf, "%d\n", tm->get_direction(tm));
}

static ssize_t tacho_motor_show_speed(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct tacho_motor_device *tm = container_of(dev, struct tacho_motor_device, dev);

	return sprintf(buf, "%d\n", tm->get_speed(tm));
}

static ssize_t tacho_motor_show_power(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct tacho_motor_device *tm = container_of(dev, struct tacho_motor_device, dev);

	return sprintf(buf, "%d\n", tm->get_power(tm));
}

static ssize_t tacho_motor_show_time(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct tacho_motor_device *tm = container_of(dev, struct tacho_motor_device, dev);

	return sprintf(buf, "%d\n", tm->get_time(tm));
}

static ssize_t tacho_motor_show_state(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct tacho_motor_device *tm = container_of(dev, struct tacho_motor_device, dev);

	return sprintf(buf, "%s\n", tacho_motor_mode_items[tm->get_state(tm)].name);
}


static ssize_t tacho_motor_show_target_power(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct tacho_motor_device *tm = container_of(dev, struct tacho_motor_device, dev);

	return sprintf(buf, "%d\n", tm->get_target_power(tm));
}

static ssize_t tacho_motor_store_target_power(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct tacho_motor_device *tm = container_of(dev, struct tacho_motor_device, dev);

        char *end;
        long target_power = simple_strtol(buf, &end, 0);

	/* FIXME: Make these hardcoded values #defines */
        if (end == buf || target_power > 100 || target_power < -100 )
                return -EINVAL;

        tm->set_target_power(tm, target_power);

        /* Always return full write size even if we didn't consume all */
        return size;
}

static ssize_t tacho_motor_show_target_tacho(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct tacho_motor_device *tm = container_of(dev, struct tacho_motor_device, dev);

	return sprintf(buf, "%d\n", tm->get_target_tacho(tm));
}

static ssize_t tacho_motor_store_target_tacho(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct tacho_motor_device *tm = container_of(dev, struct tacho_motor_device, dev);

        char *end;
        long target_tacho = simple_strtol(buf, &end, 0);

        /* Normally we'd add range checking here - but it's not needed when setting 
         * target positions because we can't really know what the user wants 
         */
        if (end == buf)
                return -EINVAL;

        tm->set_target_tacho(tm, target_tacho);

        /* Always return full write size even if we didn't consume all */
        return size;
}

static ssize_t tacho_motor_show_target_step(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct tacho_motor_device *tm = container_of(dev, struct tacho_motor_device, dev);

	return sprintf(buf, "%d\n", tm->get_target_step(tm));
}

static ssize_t tacho_motor_store_target_step(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct tacho_motor_device *tm = container_of(dev, struct tacho_motor_device, dev);

        char *end;
        long target_step = simple_strtol(buf, &end, 0);

        /* Normally we'd add range checking here - but it's not needed when setting 
         * target positions because we can't really know what the user wants 
         */
        if (end == buf)
                return -EINVAL;

        tm->set_target_step(tm, target_step);

        /* Always return full write size even if we didn't consume all */
        return size;
}

static ssize_t tacho_motor_show_target_speed(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct tacho_motor_device *tm = container_of(dev, struct tacho_motor_device, dev);

	return sprintf(buf, "%d\n", tm->get_target_speed(tm));
}

static ssize_t tacho_motor_store_target_speed(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct tacho_motor_device *tm = container_of(dev, struct tacho_motor_device, dev);

        char *end;
        long target_speed = simple_strtol(buf, &end, 0);

	/* FIXME: Make these hardcoded values #defines */
        if (end == buf || target_speed > 100 || target_speed < -100 )
                return -EINVAL;

        tm->set_target_speed(tm, target_speed);

        /* Always return full write size even if we didn't consume all */
        return size;
}

static ssize_t tacho_motor_show_target_time(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct tacho_motor_device *tm = container_of(dev, struct tacho_motor_device, dev);

	return sprintf(buf, "%d\n", tm->get_target_time(tm));
}

static ssize_t tacho_motor_store_target_time(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct tacho_motor_device *tm = container_of(dev, struct tacho_motor_device, dev);

        char *end;
        long target_time = simple_strtol(buf, &end, 0);

	/* FIXME: Make these hardcoded values #defines */
        if (end == buf) 
                return -EINVAL;

        tm->set_target_time(tm, target_time);

        /* Always return full write size even if we didn't consume all */
        return size;
}


static struct device_attribute tacho_motor_class_dev_attrs[] = {
	__ATTR(tacho,     S_IRUGO, tacho_motor_show_tacho,     NULL),
	__ATTR(step,      S_IRUGO, tacho_motor_show_step,      NULL),
	__ATTR(direction, S_IRUGO, tacho_motor_show_direction, NULL),
	__ATTR(speed,     S_IRUGO, tacho_motor_show_speed,     NULL),
	__ATTR(power,     S_IRUGO, tacho_motor_show_power,     NULL),
	__ATTR(time,      S_IRUGO, tacho_motor_show_time,      NULL),
	__ATTR(state,     S_IRUGO, tacho_motor_show_state,     NULL),

	__ATTR(target_power,     S_IRUGO | S_IWUGO, tacho_motor_show_target_power, tacho_motor_store_target_power),
	__ATTR(target_tacho,     S_IRUGO | S_IWUGO, tacho_motor_show_target_tacho, tacho_motor_store_target_tacho),
	__ATTR(target_speed,     S_IRUGO | S_IWUGO, tacho_motor_show_target_speed, tacho_motor_store_target_speed),
	__ATTR(target_step,      S_IRUGO | S_IWUGO, tacho_motor_show_target_step,  tacho_motor_store_target_step),
	__ATTR(target_time,      S_IRUGO | S_IWUGO, tacho_motor_show_target_time, tacho_motor_store_target_time),

	__ATTR_NULL
};

static void tacho_motor_release(struct device *dev)
{
}

int register_tacho_motor(struct tacho_motor_device *tm, struct device *parent)
{
	if (!tm)
		return -EINVAL;

	tm->dev.release = tacho_motor_release;
	tm->dev.parent = parent;
	tm->dev.class = &tacho_motor_class;
	dev_set_name(&tm->dev, "%s:tacho", dev_name(parent));

	return device_register(&tm->dev);
}
EXPORT_SYMBOL_GPL(register_tacho_motor);

void unregister_tacho_motor(struct tacho_motor_device *tm)
{
	device_unregister(&tm->dev);
}
EXPORT_SYMBOL_GPL(unregister_tacho_motor);

static char *tacho_motor_devnode(struct device *dev, umode_t *mode)
{
	return kasprintf(GFP_KERNEL, "tacho_motor/%s", dev_name(dev));
}

struct class tacho_motor_class = {
	.name		= "tacho-motor",
	.owner		= THIS_MODULE,
	.dev_attrs	= tacho_motor_class_dev_attrs,
	.devnode	= tacho_motor_devnode,
};
EXPORT_SYMBOL_GPL(tacho_motor_class);

static int __init tacho_motor_class_init(void)
{
	int err;

	err = class_register(&tacho_motor_class);
	if (err) {
		pr_err("unable to register tacho_motor_class\n");
		return err;
	}

	return 0;
}
module_init(tacho_motor_class_init);

static void __exit tacho_motor_class_exit(void)
{
	class_unregister(&tacho_motor_class);
}
module_exit(tacho_motor_class_exit);

MODULE_DESCRIPTION("Tacho Motor device class for LEGO Mindstorms EV3");
MODULE_AUTHOR("Ralph Hempel <rhempel@hempeldesigngroup.com>");
MODULE_LICENSE("GPL");
