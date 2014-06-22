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

static struct tacho_motor_mode_item tacho_motor_stop_modes[NUM_STOP_MODES] = {
	[STOP_COAST]     =  { "coast" },
	[STOP_BRAKE]     =  { "brake" },
	[STOP_HOLD]      =  { "hold" },
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

static ssize_t tacho_motor_show_port_name(struct device *dev,
				      struct device_attribute *attr,
				      char *buf)
{
	struct tacho_motor_device *tm = container_of(dev, struct tacho_motor_device, dev);

	return snprintf(buf, TACHO_MOTOR_PORT_NAME_SIZE, "%s\n", tm->port_name);
}

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

        if (end == buf)
                return -EINVAL;

        tm->fp->set_position(tm, position);

        return size;
}

static ssize_t tacho_motor_show_state(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct tacho_motor_device *tm = container_of(dev, struct tacho_motor_device, dev);

	return sprintf(buf, "%s\n", tacho_motor_state_items[tm->fp->get_state(tm)].name);
}

static ssize_t tacho_motor_show_duty_cycle(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct tacho_motor_device *tm = container_of(dev, struct tacho_motor_device, dev);

	return sprintf(buf, "%d\n", tm->fp->get_duty_cycle(tm));
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

static ssize_t tacho_motor_show_stop_modes(struct device *dev, struct device_attribute *attr, char *buf)
{
        unsigned int i;

	int size = 0;

// struct tacho_motor_device *tm = container_of(dev, struct tacho_motor_device, dev);

	for (i=0; i<NUM_STOP_MODES; ++i)
		size += sprintf(buf+size, "%s ", tacho_motor_stop_modes[i].name);

	size += sprintf(buf+size, "\n");

        return size;
}

static ssize_t tacho_motor_show_stop_mode(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct tacho_motor_device *tm = container_of(dev, struct tacho_motor_device, dev);

	return sprintf(buf, "%s\n", tacho_motor_stop_modes[tm->fp->get_stop_mode(tm)].name);
}

static ssize_t tacho_motor_store_stop_mode(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct tacho_motor_device *tm = container_of(dev, struct tacho_motor_device, dev);

        unsigned int i;

	for (i=0; i<NUM_STOP_MODES; ++i)
		if (sysfs_streq( buf, tacho_motor_stop_modes[i].name)) break;

	if (i >= NUM_STOP_MODES)
                return -EINVAL;

        tm->fp->set_stop_mode(tm, i);

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

static ssize_t tacho_motor_show_ramp_up_sp(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct tacho_motor_device *tm = container_of(dev, struct tacho_motor_device, dev);

	return sprintf(buf, "%d\n", tm->fp->get_ramp_up_sp(tm));
}

static ssize_t tacho_motor_store_ramp_up_sp(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct tacho_motor_device *tm = container_of(dev, struct tacho_motor_device, dev);

        char *end;
        long ramp_up_sp = simple_strtol(buf, &end, 0);

        if ((end == buf) || (ramp_up_sp < 0) || (ramp_up_sp > 10000))
                return -EINVAL;

        tm->fp->set_ramp_up_sp(tm, ramp_up_sp);

        return size;
}

static ssize_t tacho_motor_show_ramp_down_sp(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct tacho_motor_device *tm = container_of(dev, struct tacho_motor_device, dev);

	return sprintf(buf, "%d\n", tm->fp->get_ramp_down_sp(tm));
}

static ssize_t tacho_motor_store_ramp_down_sp(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct tacho_motor_device *tm = container_of(dev, struct tacho_motor_device, dev);

        char *end;
        long ramp_down_sp = simple_strtol(buf, &end, 0);

        if ((end == buf) || (ramp_down_sp < 0) || (ramp_down_sp > 10000))
                return -EINVAL;

        tm->fp->set_ramp_down_sp(tm, ramp_down_sp);

        return size;
}

static ssize_t tacho_motor_show_speed_regulation_P(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct tacho_motor_device *tm = container_of(dev, struct tacho_motor_device, dev);

	return sprintf(buf, "%d\n", tm->fp->get_speed_regulation_P(tm));
}

static ssize_t tacho_motor_store_speed_regulation_P(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct tacho_motor_device *tm = container_of(dev, struct tacho_motor_device, dev);

        char *end;
        long speed_regulation_P = simple_strtol(buf, &end, 0);

        if ((end == buf) || (speed_regulation_P < 0))
                return -EINVAL;

        tm->fp->set_speed_regulation_P(tm, speed_regulation_P);

        return size;
}

static ssize_t tacho_motor_show_speed_regulation_I(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct tacho_motor_device *tm = container_of(dev, struct tacho_motor_device, dev);

	return sprintf(buf, "%d\n", tm->fp->get_speed_regulation_I(tm));
}

static ssize_t tacho_motor_store_speed_regulation_I(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct tacho_motor_device *tm = container_of(dev, struct tacho_motor_device, dev);

        char *end;
        long speed_regulation_I = simple_strtol(buf, &end, 0);

        if ((end == buf) || (speed_regulation_I < 0))
                return -EINVAL;

        tm->fp->set_speed_regulation_I(tm, speed_regulation_I);

        return size;
}
static ssize_t tacho_motor_show_speed_regulation_D(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct tacho_motor_device *tm = container_of(dev, struct tacho_motor_device, dev);

	return sprintf(buf, "%d\n", tm->fp->get_speed_regulation_D(tm));
}

static ssize_t tacho_motor_store_speed_regulation_D(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct tacho_motor_device *tm = container_of(dev, struct tacho_motor_device, dev);

        char *end;
        long speed_regulation_D = simple_strtol(buf, &end, 0);

        if ((end == buf) || (speed_regulation_D < 0))
                return -EINVAL;

        tm->fp->set_speed_regulation_D(tm, speed_regulation_D);

        return size;
}

static ssize_t tacho_motor_show_speed_regulation_K(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct tacho_motor_device *tm = container_of(dev, struct tacho_motor_device, dev);

	return sprintf(buf, "%d\n", tm->fp->get_speed_regulation_K(tm));
}

static ssize_t tacho_motor_store_speed_regulation_K(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct tacho_motor_device *tm = container_of(dev, struct tacho_motor_device, dev);

        char *end;
        long speed_regulation_K = simple_strtol(buf, &end, 0);

        if ((end == buf) || (speed_regulation_K < 0))
                return -EINVAL;

        tm->fp->set_speed_regulation_K(tm, speed_regulation_K);

        return size;
}

static ssize_t tacho_motor_show_duty_cycle_sp(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct tacho_motor_device *tm = container_of(dev, struct tacho_motor_device, dev);

	return sprintf(buf, "%d\n", tm->fp->get_duty_cycle_sp(tm));
}

static ssize_t tacho_motor_store_duty_cycle_sp(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct tacho_motor_device *tm = container_of(dev, struct tacho_motor_device, dev);

        char *end;
        long duty_cycle_sp = simple_strtol(buf, &end, 0);

        if ((end == buf) || (duty_cycle_sp > 100) || (duty_cycle_sp < -100))
                return -EINVAL;

        tm->fp->set_duty_cycle_sp(tm, duty_cycle_sp);

        return size;
}

static ssize_t tacho_motor_show_pulses_per_second_sp(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct tacho_motor_device *tm = container_of(dev, struct tacho_motor_device, dev);

	return sprintf(buf, "%d\n", tm->fp->get_pulses_per_second_sp(tm));
}

static ssize_t tacho_motor_store_pulses_per_second_sp(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct tacho_motor_device *tm = container_of(dev, struct tacho_motor_device, dev);

        char *end;
        long pulses_per_second_sp = simple_strtol(buf, &end, 0);

        if ((end == buf) || (pulses_per_second_sp > 2000) || (pulses_per_second_sp < -2000))
                return -EINVAL;

        tm->fp->set_pulses_per_second_sp(tm, pulses_per_second_sp);

        return size;
}

static ssize_t tacho_motor_show_time_sp(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct tacho_motor_device *tm = container_of(dev, struct tacho_motor_device, dev);

	return sprintf(buf, "%d\n", tm->fp->get_time_sp(tm));
}

static ssize_t tacho_motor_store_time_sp(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct tacho_motor_device *tm = container_of(dev, struct tacho_motor_device, dev);

        char *end;
        long time_sp = simple_strtol(buf, &end, 0);

        if ((end == buf) || (time_sp < 0))
                return -EINVAL;

        tm->fp->set_time_sp(tm, time_sp);

        return size;
}

static ssize_t tacho_motor_show_position_sp(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct tacho_motor_device *tm = container_of(dev, struct tacho_motor_device, dev);

	return sprintf(buf, "%d\n", tm->fp->get_position_sp(tm));
}

static ssize_t tacho_motor_store_position_sp(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct tacho_motor_device *tm = container_of(dev, struct tacho_motor_device, dev);

        char *end;
        long position_sp = simple_strtol(buf, &end, 0);

        if (end == buf)
                return -EINVAL;

        tm->fp->set_position_sp(tm, position_sp);

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

static ssize_t tacho_motor_show_estop(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct tacho_motor_device *tm = container_of(dev, struct tacho_motor_device, dev);

	return sprintf(buf, "%d\n", tm->fp->get_estop(tm));
}

static ssize_t tacho_motor_store_estop(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct tacho_motor_device *tm = container_of(dev, struct tacho_motor_device, dev);

        char *end;
        long estop = simple_strtol(buf, &end, 0);

        if (end == buf)
                return -EINVAL;

        tm->fp->set_estop(tm, estop);

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
	__ATTR(port_name,		S_IRUGO,	   tacho_motor_show_port_name,		  NULL),
	__ATTR(type,      		S_IRUGO | S_IWUGO, tacho_motor_show_type,		  tacho_motor_store_type),
	__ATTR(position,		S_IRUGO | S_IWUGO, tacho_motor_show_position,		  tacho_motor_store_position),

	__ATTR(state,			S_IRUGO,	   tacho_motor_show_state,		  NULL),
	__ATTR(duty_cycle,		S_IRUGO,	   tacho_motor_show_duty_cycle,		  NULL),
	__ATTR(pulses_per_second,	S_IRUGO,	   tacho_motor_show_pulses_per_second,	  NULL),

	__ATTR(duty_cycle_sp,		S_IRUGO | S_IWUGO, tacho_motor_show_duty_cycle_sp,	  tacho_motor_store_duty_cycle_sp),
	__ATTR(pulses_per_second_sp,	S_IRUGO | S_IWUGO, tacho_motor_show_pulses_per_second_sp, tacho_motor_store_pulses_per_second_sp),
	__ATTR(time_sp,			S_IRUGO | S_IWUGO, tacho_motor_show_time_sp,		  tacho_motor_store_time_sp),
	__ATTR(position_sp,		S_IRUGO | S_IWUGO, tacho_motor_show_position_sp,	  tacho_motor_store_position_sp),

	__ATTR(run_mode,		S_IRUGO | S_IWUGO, tacho_motor_show_run_mode,		  tacho_motor_store_run_mode),
	__ATTR(regulation_mode,		S_IRUGO | S_IWUGO, tacho_motor_show_regulation_mode,	  tacho_motor_store_regulation_mode),
	__ATTR(stop_modes,		S_IRUGO | S_IWUGO, tacho_motor_show_stop_modes,	NULL),
	__ATTR(stop_mode,		S_IRUGO | S_IWUGO, tacho_motor_show_stop_mode,		  tacho_motor_store_stop_mode),
	__ATTR(position_mode,		S_IRUGO | S_IWUGO, tacho_motor_show_position_mode,	  tacho_motor_store_position_mode),
	__ATTR(polarity_mode,		S_IRUGO | S_IWUGO, tacho_motor_show_polarity_mode,	  tacho_motor_store_polarity_mode),

	__ATTR(ramp_up_sp,		S_IRUGO | S_IWUGO, tacho_motor_show_ramp_up_sp,		  tacho_motor_store_ramp_up_sp),
	__ATTR(ramp_down_sp,		S_IRUGO | S_IWUGO, tacho_motor_show_ramp_down_sp,	  tacho_motor_store_ramp_down_sp),

	__ATTR(speed_regulation_P,	S_IRUGO | S_IWUGO, tacho_motor_show_speed_regulation_P,	  tacho_motor_store_speed_regulation_P),
	__ATTR(speed_regulation_I,	S_IRUGO | S_IWUGO, tacho_motor_show_speed_regulation_I,	  tacho_motor_store_speed_regulation_I),
	__ATTR(speed_regulation_D,	S_IRUGO | S_IWUGO, tacho_motor_show_speed_regulation_D,	  tacho_motor_store_speed_regulation_D),
	__ATTR(speed_regulation_K,	S_IRUGO | S_IWUGO, tacho_motor_show_speed_regulation_K,	  tacho_motor_store_speed_regulation_K),

	__ATTR(run,			S_IRUGO | S_IWUGO, tacho_motor_show_run,		  tacho_motor_store_run),
	__ATTR(estop,			S_IRUGO | S_IWUGO, tacho_motor_show_estop,		  tacho_motor_store_estop),

	__ATTR(reset,				  S_IWUGO, NULL,				  tacho_motor_store_reset),

	__ATTR_NULL
};

static void tacho_motor_release(struct device *dev)
{
}

static unsigned tacho_motor_class_id = 0;

int register_tacho_motor(struct tacho_motor_device *tm, struct device *parent)
{
	int err;

	if (!tm || !tm->port_name || !parent)
		return -EINVAL;
	
	tm->dev.release = tacho_motor_release;
	tm->dev.parent = parent;
	tm->dev.class = &tacho_motor_class;
	dev_set_name(&tm->dev, "tacho_motor%d", tacho_motor_class_id++);

	err = device_register(&tm->dev);
	if (err)
		return err;

	dev_info(&tm->dev, "Tacho motor registered.\n");

	return 0;
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
