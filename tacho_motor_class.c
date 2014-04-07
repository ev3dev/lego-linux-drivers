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

static struct tacho_motor_mode_item tacho_motor_regulation_modes[NUM_REGULATION_MODES] = {
	[REGULATION_OFF] =  { "off" },
	[REGULATION_ON]  =  { "on"  },
};

static struct tacho_motor_mode_item tacho_motor_brake_modes[NUM_BRAKE_MODES] = {
	[BRAKE_OFF]     =  { "off" },
	[BRAKE_ON]      =  { "on" },
};

static struct tacho_motor_mode_item tacho_motor_hold_modes[NUM_HOLD_MODES] = {
	[HOLD_OFF]     =  { "off" },
	[HOLD_ON]      =  { "on" },
};

static struct tacho_motor_mode_item tacho_motor_position_modes[NUM_POSITION_MODES] = {
	[POSITION_ABSOLUTE] =  { "absolute" },
	[POSITION_RELATIVE] =  { "relative" },
};

static struct tacho_motor_mode_item tacho_motor_run_modes[NUM_RUN_MODES] = {
	[RUN_FOREVER]   =  { "forever"  },
	[RUN_TIME]      =  { "time"     },
	[RUN_POSITION]  =  { "position" },
};

static struct tacho_motor_mode_item tacho_motor_polarity_modes[NUM_POLARITY_MODES] = {
	[POLARITY_POSITIVE]	=  { "positive"  },
	[POLARITY_NEGATIVE]	=  { "negative"  },
};

struct tacho_motor_type_item {
	const char *name;
};

static struct tacho_motor_type_item tacho_motor_types[NUM_TACHO_TYPES] = {
	[TACHO_TYPE_TACHO]     =  { "tacho"     },
	[TACHO_TYPE_MINITACHO] =  { "minitacho" },
};

struct tacho_motor_state_item {
	const char *name;
};

static struct tacho_motor_mode_item tacho_motor_state_items[NUM_TACHO_MOTOR_STATES] = {
	[STATE_RUN_FOREVER]			= { "run_forever"		},
	[STATE_SETUP_RAMP_TIME]			= { "setup_ramp_time"		},
	[STATE_SETUP_RAMP_POSITION]		= { "setup_ramp_position"	},
	[STATE_SETUP_RAMP_REGULATION]		= { "setup_ramp_regulation"	},
	[STATE_RAMP_UP]				= { "ramp_up"			},
	[STATE_RAMP_CONST]			= { "ramp_const"		},
	[STATE_POSITION_RAMP_DOWN]		= { "position_ramp_down"	},
	[STATE_RAMP_DOWN]			= { "ramp_down"			},
	[STATE_STOP]				= { "stop"			},
	[STATE_IDLE]				= { "idle"			},
};

static ssize_t tacho_motor_show_type(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct tacho_motor_device *tm = container_of(dev, struct tacho_motor_device, dev);

	return sprintf(buf, "%s\n", tacho_motor_types[tm->fp->get_type(tm)].name);
}

static ssize_t tacho_motor_store_type(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct tacho_motor_device *tm = container_of(dev, struct tacho_motor_device, dev);

        unsigned int i;

	for (i=0; i<NUM_TACHO_TYPES; ++i)
		if (sysfs_streq(buf, tacho_motor_types[i].name)) break;

	if (i >= NUM_TACHO_TYPES)
                return -EINVAL;

        tm->fp->set_type(tm, i);

        return size;
}

static ssize_t tacho_motor_show_position(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct tacho_motor_device *tm = container_of(dev, struct tacho_motor_device, dev);

	return sprintf(buf, "%d\n", tm->fp->get_position(tm));
}

ssize_t tacho_motor_store_position(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct tacho_motor_device *tm = container_of(dev, struct tacho_motor_device, dev);

        char *end;
        long position = simple_strtol(buf, &end, 0);

        if (*end == buf)
                return -EINVAL;

        tm->fp->set_position(tm, position);

        return size;
}

static ssize_t tacho_motor_show_speed(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct tacho_motor_device *tm = container_of(dev, struct tacho_motor_device, dev);

	return sprintf(buf, "%d\n", tm->fp->get_speed(tm));
}

static ssize_t tacho_motor_show_power(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct tacho_motor_device *tm = container_of(dev, struct tacho_motor_device, dev);

	return sprintf(buf, "%d\n", tm->fp->get_power(tm));
}

static ssize_t tacho_motor_show_state(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct tacho_motor_device *tm = container_of(dev, struct tacho_motor_device, dev);

	return sprintf(buf, "%s\n", tacho_motor_state_items[tm->fp->get_state(tm)].name);
}

static ssize_t tacho_motor_show_pulses_per_second(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct tacho_motor_device *tm = container_of(dev, struct tacho_motor_device, dev);

	return sprintf(buf, "%d\n", tm->fp->get_pulses_per_second(tm));
}

static ssize_t tacho_motor_show_run_mode(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct tacho_motor_device *tm = container_of(dev, struct tacho_motor_device, dev);

	return sprintf(buf, "%s\n", tacho_motor_run_modes[tm->fp->get_run_mode(tm)].name);
}

static ssize_t tacho_motor_store_run_mode(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct tacho_motor_device *tm = container_of(dev, struct tacho_motor_device, dev);

        unsigned int i;

	for (i=0; i<NUM_RUN_MODES; ++i)
		if (sysfs_streq(buf, tacho_motor_run_modes[i].name)) break;

	if (i >= NUM_RUN_MODES)
                return -EINVAL;

        tm->fp->set_run_mode(tm, i);

        return size;
}

static ssize_t tacho_motor_show_regulation_mode(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct tacho_motor_device *tm = container_of(dev, struct tacho_motor_device, dev);

	return sprintf(buf, "%s\n", tacho_motor_regulation_modes[tm->fp->get_regulation_mode(tm)].name);
}

static ssize_t tacho_motor_store_regulation_mode(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct tacho_motor_device *tm = container_of(dev, struct tacho_motor_device, dev);

        unsigned int i;

	for (i=0; i<NUM_REGULATION_MODES; ++i)
		if (sysfs_streq( buf, tacho_motor_regulation_modes[i].name)) break;

	if (i >= NUM_REGULATION_MODES)
                return -EINVAL;

        tm->fp->set_regulation_mode(tm, i);

        return size;
}

static ssize_t tacho_motor_show_brake_mode(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct tacho_motor_device *tm = container_of(dev, struct tacho_motor_device, dev);

	return sprintf(buf, "%s\n", tacho_motor_brake_modes[tm->fp->get_brake_mode(tm)].name);
}

static ssize_t tacho_motor_store_brake_mode(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct tacho_motor_device *tm = container_of(dev, struct tacho_motor_device, dev);

        unsigned int i;

	for (i=0; i<NUM_BRAKE_MODES; ++i)
		if (sysfs_streq( buf, tacho_motor_brake_modes[i].name)) break;

	if (i >= NUM_BRAKE_MODES)
                return -EINVAL;

        tm->fp->set_brake_mode(tm, i);

        return size;
}

static ssize_t tacho_motor_show_hold_mode(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct tacho_motor_device *tm = container_of(dev, struct tacho_motor_device, dev);

	return sprintf(buf, "%s\n", tacho_motor_hold_modes[tm->fp->get_hold_mode(tm)].name);
}

static ssize_t tacho_motor_store_hold_mode(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct tacho_motor_device *tm = container_of(dev, struct tacho_motor_device, dev);

        unsigned int i;

	for (i=0; i<NUM_HOLD_MODES; ++i)
		if (sysfs_streq(buf, tacho_motor_hold_modes[i].name)) break;

	if (i >= NUM_HOLD_MODES)
                return -EINVAL;

        tm->fp->set_hold_mode(tm, i);

        return size;
}

static ssize_t tacho_motor_show_position_mode(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct tacho_motor_device *tm = container_of(dev, struct tacho_motor_device, dev);

	return sprintf(buf, "%s\n", tacho_motor_position_modes[tm->fp->get_position_mode(tm)].name);
}

static ssize_t tacho_motor_store_position_mode(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct tacho_motor_device *tm = container_of(dev, struct tacho_motor_device, dev);

        unsigned int i;

	for (i=0; i<NUM_POSITION_MODES; ++i)
		if (sysfs_streq( buf, tacho_motor_position_modes[i].name)) break;

	if (i >= NUM_POSITION_MODES)
                return -EINVAL;

        tm->fp->set_position_mode(tm, i);

        return size;
}

static ssize_t tacho_motor_show_polarity_mode(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct tacho_motor_device *tm = container_of(dev, struct tacho_motor_device, dev);

	return sprintf(buf, "%s\n", tacho_motor_polarity_modes[tm->fp->get_polarity_mode(tm)].name);
}

static ssize_t tacho_motor_store_polarity_mode(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct tacho_motor_device *tm = container_of(dev, struct tacho_motor_device, dev);

        unsigned int i;

	for (i=0; i<NUM_POLARITY_MODES; ++i)
		if (sysfs_streq(buf, tacho_motor_polarity_modes[i].name)) break;

	if (i >= NUM_RUN_MODES)
                return -EINVAL;

        tm->fp->set_polarity_mode(tm, i);

        return size;
}

static ssize_t tacho_motor_show_ramp_up(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct tacho_motor_device *tm = container_of(dev, struct tacho_motor_device, dev);

	return sprintf(buf, "%d\n", tm->fp->get_ramp_up(tm));
}

static ssize_t tacho_motor_store_ramp_up(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct tacho_motor_device *tm = container_of(dev, struct tacho_motor_device, dev);

        char *end;
        long ramp_up = simple_strtol(buf, &end, 0);

        if ((end == buf) || (ramp_up < 0) || (ramp_up > 10000))
                return -EINVAL;

        tm->fp->set_ramp_up(tm, ramp_up);

        return size;
}

static ssize_t tacho_motor_show_ramp_down(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct tacho_motor_device *tm = container_of(dev, struct tacho_motor_device, dev);

	return sprintf(buf, "%d\n", tm->fp->get_ramp_down(tm));
}

static ssize_t tacho_motor_store_ramp_down(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct tacho_motor_device *tm = container_of(dev, struct tacho_motor_device, dev);

        char *end;
        long ramp_down = simple_strtol(buf, &end, 0);

        if ((end == buf) || (ramp_down < 0) || (ramp_down > 10000))
                return -EINVAL;

        tm->fp->set_ramp_down(tm, ramp_down);

        return size;
}

static ssize_t tacho_motor_show_speed_setpoint(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct tacho_motor_device *tm = container_of(dev, struct tacho_motor_device, dev);

	return sprintf(buf, "%d\n", tm->fp->get_speed_setpoint(tm));
}

static ssize_t tacho_motor_store_speed_setpoint(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct tacho_motor_device *tm = container_of(dev, struct tacho_motor_device, dev);

        char *end;
        long speed_setpoint = simple_strtol(buf, &end, 0);

        if ((end == buf) || (speed_setpoint > 100) || (speed_setpoint < -100))
                return -EINVAL;

        tm->fp->set_speed_setpoint(tm, speed_setpoint);

        return size;
}

static ssize_t tacho_motor_show_time_setpoint(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct tacho_motor_device *tm = container_of(dev, struct tacho_motor_device, dev);

	return sprintf(buf, "%d\n", tm->fp->get_time_setpoint(tm));
}

static ssize_t tacho_motor_store_time_setpoint(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct tacho_motor_device *tm = container_of(dev, struct tacho_motor_device, dev);

        char *end;
        long time_setpoint = simple_strtol(buf, &end, 0);

        if ((end == buf) || (time_setpoint < 0))
                return -EINVAL;

        tm->fp->set_time_setpoint(tm, time_setpoint);

        return size;
}

static ssize_t tacho_motor_show_position_setpoint(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct tacho_motor_device *tm = container_of(dev, struct tacho_motor_device, dev);

	return sprintf(buf, "%d\n", tm->fp->get_position_setpoint(tm));
}

static ssize_t tacho_motor_store_position_setpoint(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct tacho_motor_device *tm = container_of(dev, struct tacho_motor_device, dev);

        char *end;
        long position_setpoint = simple_strtol(buf, &end, 0);

        if (end == buf)
                return -EINVAL;

        tm->fp->set_position_setpoint(tm, position_setpoint);

        return size;
}

static ssize_t tacho_motor_show_run(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct tacho_motor_device *tm = container_of(dev, struct tacho_motor_device, dev);

	return sprintf(buf, "%d\n", tm->fp->get_run(tm));
}

static ssize_t tacho_motor_store_run(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct tacho_motor_device *tm = container_of(dev, struct tacho_motor_device, dev);

        char *end;
        long run = simple_strtol(buf, &end, 0);

        if ((end == buf) || (run > 1) || (run < 0))
                return -EINVAL;

        tm->fp->set_run(tm, run);

        return size;
}

static ssize_t tacho_motor_store_reset(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct tacho_motor_device *tm = container_of(dev, struct tacho_motor_device, dev);

        char *end;
        long reset = simple_strtol(buf, &end, 0);

        if ((end == buf) || (reset != 1))
                return -EINVAL;

        tm->fp->set_reset(tm, reset);

        return size;
}

static struct device_attribute tacho_motor_class_dev_attrs[] = {
	__ATTR(type,      		S_IRUGO | S_IWUGO, tacho_motor_show_type,		tacho_motor_store_type),
	__ATTR(position,		S_IRUGO | S_IWUGO, tacho_motor_show_position,		tacho_motor_store_position),

	__ATTR(speed,			S_IRUGO, 	   tacho_motor_show_speed,		NULL),
	__ATTR(power,			S_IRUGO,	   tacho_motor_show_power,		NULL),
	__ATTR(state,			S_IRUGO,	   tacho_motor_show_state,		NULL),
	__ATTR(pulses_per_second,	S_IRUGO,	   tacho_motor_show_pulses_per_second,	NULL),

	__ATTR(speed_setpoint,		S_IRUGO | S_IWUGO, tacho_motor_show_speed_setpoint,	tacho_motor_store_speed_setpoint),
	__ATTR(time_setpoint,		S_IRUGO | S_IWUGO, tacho_motor_show_time_setpoint,	tacho_motor_store_time_setpoint),
	__ATTR(position_setpoint,	S_IRUGO | S_IWUGO, tacho_motor_show_position_setpoint,	tacho_motor_store_position_setpoint),

	__ATTR(run_mode,		S_IRUGO | S_IWUGO, tacho_motor_show_run_mode,		tacho_motor_store_run_mode),
	__ATTR(regulation_mode,		S_IRUGO | S_IWUGO, tacho_motor_show_regulation_mode,	tacho_motor_store_regulation_mode),
	__ATTR(brake_mode,		S_IRUGO | S_IWUGO, tacho_motor_show_brake_mode,		tacho_motor_store_brake_mode),
	__ATTR(hold_mode,		S_IRUGO | S_IWUGO, tacho_motor_show_hold_mode,		tacho_motor_store_hold_mode),
	__ATTR(position_mode,		S_IRUGO | S_IWUGO, tacho_motor_show_position_mode,	tacho_motor_store_position_mode),
	__ATTR(polarity_mode,		S_IRUGO | S_IWUGO, tacho_motor_show_polarity_mode,	tacho_motor_store_polarity_mode),

	__ATTR(ramp_up,			S_IRUGO | S_IWUGO, tacho_motor_show_ramp_up,		tacho_motor_store_ramp_up),
	__ATTR(ramp_down,		S_IRUGO | S_IWUGO, tacho_motor_show_ramp_down,		tacho_motor_store_ramp_down),

	__ATTR(run,			S_IRUGO | S_IWUGO, tacho_motor_show_run,		tacho_motor_store_run),

	__ATTR(reset,				  S_IWUGO, NULL,				tacho_motor_store_reset),

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
