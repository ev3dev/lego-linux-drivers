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

/* Note that for the string comparison to work for kernel module attribute
 * values that are echo'ed to the device, the len value must be exactly the
 * number of characters in the corresponding tacho_motor_mode_item.
 *
 * Why? Because echo tacks a helpful carriage return on the end of the text
 *      that is being echo'ed - and that becomes part of the comparison string
 *
 * Of course, you can ask users to use "echo -n" or use strlen() on the name
 * at run time, but it's probably easier and clearer to just specify the
 * actual length in this structure.
 *
 * This has the added benefit of being able to specify a maximum number of
 * characters to be checked.
 */

struct tacho_motor_mode_item {
	const char *name;
	const int  len;
};

static struct tacho_motor_mode_item tacho_motor_stop_modes[NUM_STOP_MODES] = {
	[STOP_COAST]     =  { "coast", 5 },
	[STOP_BRAKE]     =  { "brake", 5 },
};

static struct tacho_motor_mode_item tacho_motor_regulation_modes[NUM_REGULATION_MODES] = {
	[REGULATION_OFF] =  { "off", 3 },
	[REGULATION_ON]  =  { "on",  2 },
};

static struct tacho_motor_mode_item tacho_motor_tacho_modes[NUM_TACHO_MODES] = {
	[TACHO_ABSOLUTE] =  { "absolute", 8 },
	[TACHO_RELATIVE] =  { "relative", 8 },
};

static struct tacho_motor_mode_item tacho_motor_ramp_modes[NUM_RAMP_MODES] = {
	[RAMP_OFF]       =  { "off",   3 },
	[RAMP_TIME]      =  { "time",  4 },
	[RAMP_TACHO]     =  { "tacho", 5 },
};

struct tacho_motor_type_item {
	const char *name;
	const int  len;
};

static struct tacho_motor_type_item tacho_motor_tacho_types[NUM_TACHO_TYPES] = {
	[TACHO_TYPE_TACHO]     =  { "tacho",      5 },
	[TACHO_TYPE_MINITACHO] =  { "minitacho",  9 },
};

struct tacho_motor_state_item {
	const char *name;
};

#warning "These states have to line up with the enum's"

static struct tacho_motor_mode_item tacho_motor_state_items[NUM_TACHO_MOTOR_STATES] = {
	[UNLIMITED_UNREG]		= { "unlimited_unreg"		},
	[UNLIMITED_REG]			= { "unlimited_reg"		},
	[SETUP_RAMP_TIME]		= { "setup_ramp_time"		},
	[SETUP_RAMP_ABSOLUTE_TACHO]	= { "setup_ramp_absolute_tacho" },
	[SETUP_RAMP_RELATIVE_TACHO]	= { "setup_ramp_relative_tacho"	},
	[SETUP_RAMP_REGULATION]		= { "setup_ramp_regualtion"	},
	[RAMP_UP]			= { "ramp_up"			},
	[RAMP_CONST]			= { "ramp_const"		},
	[RAMP_DOWN]			= { "ramp_down"			},
	[STOP_MOTOR]			= { "stop_motor"		},
	[IDLE]				= { "idle"			},
};

static ssize_t tacho_motor_show_tacho(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct tacho_motor_device *tm = container_of(dev, struct tacho_motor_device, dev);

	return sprintf(buf, "%d\n", tm->get_tacho(tm));
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

	return sprintf(buf, "%s\n", tacho_motor_state_items[tm->get_state(tm)].name);
}

/* -------------------------------------------------------------------------- */

static ssize_t tacho_motor_show_stop_mode(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct tacho_motor_device *tm = container_of(dev, struct tacho_motor_device, dev);

	return sprintf(buf, "%s\n", tacho_motor_stop_modes[tm->get_stop_mode(tm)].name);
}

static ssize_t tacho_motor_store_stop_mode(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct tacho_motor_device *tm = container_of(dev, struct tacho_motor_device, dev);

        unsigned int i;

	for (i=0; i<NUM_STOP_MODES; ++i)
		if (strnicmp( buf, tacho_motor_stop_modes[i].name, tacho_motor_stop_modes[i].len) == 0) break;

	if (i >= NUM_STOP_MODES)
                return -EINVAL;

        tm->set_stop_mode(tm, i);

        /* Always return full write size even if we didn't consume all */
        return size;
}

static ssize_t tacho_motor_show_regulation_mode(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct tacho_motor_device *tm = container_of(dev, struct tacho_motor_device, dev);

	return sprintf(buf, "%s\n", tacho_motor_regulation_modes[tm->get_regulation_mode(tm)].name);
}

static ssize_t tacho_motor_store_regulation_mode(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct tacho_motor_device *tm = container_of(dev, struct tacho_motor_device, dev);

        unsigned int i;

	for (i=0; i<NUM_REGULATION_MODES; ++i)
		if (strnicmp( buf, tacho_motor_regulation_modes[i].name, tacho_motor_regulation_modes[i].len) == 0) break;

	if (i >= NUM_REGULATION_MODES)
                return -EINVAL;

        tm->set_regulation_mode(tm, i);

        /* Always return full write size even if we didn't consume all */
        return size;
}

static ssize_t tacho_motor_show_tacho_mode(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct tacho_motor_device *tm = container_of(dev, struct tacho_motor_device, dev);

	return sprintf(buf, "%s\n", tacho_motor_tacho_modes[tm->get_tacho_mode(tm)].name);
}

static ssize_t tacho_motor_store_tacho_mode(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct tacho_motor_device *tm = container_of(dev, struct tacho_motor_device, dev);

        unsigned int i;

	for (i=0; i<NUM_TACHO_MODES; ++i)
		if (strnicmp( buf, tacho_motor_tacho_modes[i].name, tacho_motor_tacho_modes[i].len) == 0) break;

	if (i >= NUM_TACHO_MODES)
                return -EINVAL;

        tm->set_tacho_mode(tm, i);

        /* Always return full write size even if we didn't consume all */
        return size;
}




static ssize_t tacho_motor_show_ramp_mode(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct tacho_motor_device *tm = container_of(dev, struct tacho_motor_device, dev);

	return sprintf(buf, "%s\n", tacho_motor_ramp_modes[tm->get_ramp_mode(tm)].name);
}

static ssize_t tacho_motor_store_ramp_mode(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct tacho_motor_device *tm = container_of(dev, struct tacho_motor_device, dev);

        unsigned int i;

	for (i=0; i<NUM_RAMP_MODES; ++i)
		if (strnicmp( buf, tacho_motor_ramp_modes[i].name, tacho_motor_ramp_modes[i].len) == 0) break;

	if (i >= NUM_RAMP_MODES)
                return -EINVAL;

        tm->set_ramp_mode(tm, i);

        /* Always return full write size even if we didn't consume all */
        return size;
}

static ssize_t tacho_motor_show_tacho_type(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct tacho_motor_device *tm = container_of(dev, struct tacho_motor_device, dev);

	return sprintf(buf, "%s\n", tacho_motor_tacho_types[tm->get_tacho_type(tm)].name);
}

static ssize_t tacho_motor_store_tacho_type(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct tacho_motor_device *tm = container_of(dev, struct tacho_motor_device, dev);

        unsigned int i;

	for (i=0; i<NUM_TACHO_TYPES; ++i)
		if (strnicmp( buf, tacho_motor_tacho_types[i].name, tacho_motor_tacho_types[i].len) == 0) break;

	if (i >= NUM_TACHO_TYPES)
                return -EINVAL;

        tm->set_tacho_type(tm, i);

        /* Always return full write size even if we didn't consume all */
        return size;
}



/* -------------------------------------------------------------------------- */

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

static ssize_t tacho_motor_show_target_steer(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct tacho_motor_device *tm = container_of(dev, struct tacho_motor_device, dev);

	return sprintf(buf, "%d\n", tm->get_target_steer(tm));
}

static ssize_t tacho_motor_store_target_steer(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct tacho_motor_device *tm = container_of(dev, struct tacho_motor_device, dev);

        char *end;
        long target_steer = simple_strtol(buf, &end, 0);

	/* FIXME: Make these hardcoded values #defines */
        if (end == buf || target_steer > 100 || target_steer < -100 )
                return -EINVAL;

        tm->set_target_steer(tm, target_steer);

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
        if ((end == buf) || (target_time < 0))
                return -EINVAL;

        tm->set_target_time(tm, target_time);

        /* Always return full write size even if we didn't consume all */
        return size;
}

static ssize_t tacho_motor_show_target_ramp_up_count(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct tacho_motor_device *tm = container_of(dev, struct tacho_motor_device, dev);

	return sprintf(buf, "%d\n", tm->get_target_ramp_up_count(tm));
}

static ssize_t tacho_motor_store_target_ramp_up_count(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct tacho_motor_device *tm = container_of(dev, struct tacho_motor_device, dev);

        char *end;
        long target_ramp_up_count = simple_strtol(buf, &end, 0);

	/* FIXME: Make these hardcoded values #defines */
        if ((end == buf) || (target_ramp_up_count < 0))
                return -EINVAL;

        tm->set_target_ramp_up_count(tm, target_ramp_up_count);

        /* Always return full write size even if we didn't consume all */
        return size;
}

static ssize_t tacho_motor_show_target_total_count(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct tacho_motor_device *tm = container_of(dev, struct tacho_motor_device, dev);

	return sprintf(buf, "%d\n", tm->get_target_total_count(tm));
}

static ssize_t tacho_motor_store_target_total_count(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct tacho_motor_device *tm = container_of(dev, struct tacho_motor_device, dev);

        char *end;
        long target_total_count = simple_strtol(buf, &end, 0);

        /* Normally we'd add range checking here - but it's not needed when setting 
         * target positions because we can't really know what the user wants 
         */
        if (end == buf)
                return -EINVAL;

        tm->set_target_total_count(tm, target_total_count);

        /* Always return full write size even if we didn't consume all */
        return size;
}


static ssize_t tacho_motor_show_target_ramp_down_count(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct tacho_motor_device *tm = container_of(dev, struct tacho_motor_device, dev);

	return sprintf(buf, "%d\n", tm->get_target_ramp_down_count(tm));
}

static ssize_t tacho_motor_store_target_ramp_down_count(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct tacho_motor_device *tm = container_of(dev, struct tacho_motor_device, dev);

        char *end;
        long target_ramp_down_count = simple_strtol(buf, &end, 0);

	/* FIXME: Make these hardcoded values #defines */
        if ((end == buf) || (target_ramp_down_count < 0))
                return -EINVAL;

        tm->set_target_ramp_down_count(tm, target_ramp_down_count);

        /* Always return full write size even if we didn't consume all */
        return size;
}

static ssize_t tacho_motor_show_mode(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct tacho_motor_device *tm = container_of(dev, struct tacho_motor_device, dev);

	return sprintf(buf, "%d\n", tm->get_run(tm));
}

static ssize_t tacho_motor_store_mode(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct tacho_motor_device *tm = container_of(dev, struct tacho_motor_device, dev);

        char *end;
        long mode = simple_strtol(buf, &end, 0);

	/* FIXME: Make these hardcoded values #defines */
        if (end == buf) 
                return -EINVAL;

        tm->set_mode(tm, mode);

        /* Always return full write size even if we didn't consume all */
        return size;
}

static ssize_t tacho_motor_show_run(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct tacho_motor_device *tm = container_of(dev, struct tacho_motor_device, dev);

	return sprintf(buf, "%d\n", tm->get_run(tm));
}

static ssize_t tacho_motor_store_run(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct tacho_motor_device *tm = container_of(dev, struct tacho_motor_device, dev);

        char *end;
        long run = simple_strtol(buf, &end, 0);

	/* FIXME: Make these hardcoded values #defines */
        if (end == buf) 
                return -EINVAL;

        tm->set_run(tm, run);

        /* Always return full write size even if we didn't consume all */
        return size;
}

static struct device_attribute tacho_motor_class_dev_attrs[] = {
	__ATTR(tacho,     S_IRUGO, tacho_motor_show_tacho,     NULL),
	__ATTR(direction, S_IRUGO, tacho_motor_show_direction, NULL),
	__ATTR(speed,     S_IRUGO, tacho_motor_show_speed,     NULL),
	__ATTR(power,     S_IRUGO, tacho_motor_show_power,     NULL),
	__ATTR(time,      S_IRUGO, tacho_motor_show_time,      NULL),
	__ATTR(state,     S_IRUGO, tacho_motor_show_state,     NULL),

	__ATTR(stop_mode,        S_IRUGO | S_IWUGO, tacho_motor_show_stop_mode,       tacho_motor_store_stop_mode),
	__ATTR(regulation_mode,  S_IRUGO | S_IWUGO, tacho_motor_show_regulation_mode, tacho_motor_store_regulation_mode),
	__ATTR(ramp_mode,        S_IRUGO | S_IWUGO, tacho_motor_show_ramp_mode,       tacho_motor_store_ramp_mode),
	__ATTR(tacho_mode,       S_IRUGO | S_IWUGO, tacho_motor_show_tacho_mode,      tacho_motor_store_tacho_mode),

	__ATTR(target_power,     S_IRUGO | S_IWUGO, tacho_motor_show_target_power, tacho_motor_store_target_power),
	__ATTR(target_tacho,     S_IRUGO | S_IWUGO, tacho_motor_show_target_tacho, tacho_motor_store_target_tacho),
	__ATTR(target_speed,     S_IRUGO | S_IWUGO, tacho_motor_show_target_speed, tacho_motor_store_target_speed),
	__ATTR(target_steer,     S_IRUGO | S_IWUGO, tacho_motor_show_target_steer, tacho_motor_store_target_steer),
	__ATTR(target_time,      S_IRUGO | S_IWUGO, tacho_motor_show_target_time,  tacho_motor_store_target_time),
	__ATTR(target_ramp_up_count,   S_IRUGO | S_IWUGO, tacho_motor_show_target_ramp_up_count,   tacho_motor_store_target_ramp_up_count),
	__ATTR(target_total_count,     S_IRUGO | S_IWUGO, tacho_motor_show_target_total_count,     tacho_motor_store_target_total_count),
	__ATTR(target_ramp_down_count, S_IRUGO | S_IWUGO, tacho_motor_show_target_ramp_down_count, tacho_motor_store_target_ramp_down_count),

	__ATTR(type,      S_IRUGO | S_IWUGO, tacho_motor_show_tacho_type, tacho_motor_store_tacho_type),

	__ATTR(mode,      S_IRUGO | S_IWUGO, tacho_motor_show_mode, tacho_motor_store_mode),
	__ATTR(run,       S_IRUGO | S_IWUGO, tacho_motor_show_run,  tacho_motor_store_run),

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
