/*
 * Tacho motor device class
 *
 * Copyright (C) 2013-2014 Ralph Hempel <rhempel@hempeldesigngroup.com>
 * Copyright (C) 2015 David Lechner <david@lechnology.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.

 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

/*
 * Note: The comment block below is used to generate docs on the ev3dev website.
 * Use kramdown (markdown) syntax. Use a '.' as a placeholder when blank lines
 * or leading whitespace is important for the markdown syntax.
 */

/**
 * DOC: website
 *
 * Tacho Motor Class
 *
* The `tacho-motor` class provides a uniform interface for using motors with
* positional and directional feedback such as the EV3 and NXT motors. This
* feedback allows for precise control of the motors. This documentation is not
* complete. For now, see the [old wiki] for more information.
* .
* ### sysfs Attributes
* .
* Tacho motors can be found at `/sys/class/tacho-motor/motor<N>`, where `<N>`
* is incremented each time a motor is loaded (it is not related to which port
* the motor is plugged in to).
* .
* `driver_name` (read-only)
* : Returns the name of the driver that provides this tacho motor device.
* .
* `duty_cycle` (read-only)
* : Returns the current duty cycle of the motor. Units are percent. Values
*   are -100 to 100.
* .
* `duty_cycle_sp` (read/write)
* : TODO
* .
* `encoder_polarity` (read/write)
* : Sets the polarity of the rotary encoder. This is an advanced feature to all
*   use of motors that send inverted encoder signals to the EV3. This should
*   be set correctly by the driver of a device. It You only need to change this
*   value if you are using a unsupported device. Valid values are `normal` and
*   `inverted`.
* .
* `polarity` (read/write)
* : Sets the polarity of the motor. With `normal` polarity, a positive duty
*   cycle will cause the motor to rotate clockwise. With `inverted` polarity,
*   a positive duty cycle will cause the motor to rotate counter-clockwise.
*   Valid values are `normal` and `inverted`.
* .
* `port_name` (read-only)
* : Returns the name of the port that the motor is connected to.
* .
* `position` (read/write)
* : Returns the current position of the motor in pulses of the rotary
*   encoder. When the motor rotates clockwise, the position will increase.
*   Likewise, rotating counter-clockwise causes the position to decrease.
*   Writing will set the position to that value.
* `position_mode` (read/write)
* : TODO
* .
* `position_modes` (read-only)
* : Returns a space-separated list of valid position modes.
* .
* `position_sp` (read/write)
* : TODO
* .
* `pulses_per_second` (read-only)
* : TODO
* .
* `pulses_per_second_sp` (read/write)
* : TODO
* .
* `ramp_up_sp` (read/write)
* : TODO
* .
* `ramp_down_sp` (read/write)
* : TODO
* .
* `reset` (write-only)
* : Writing `1` will reset all attributes to the default values.
* .
* `run` (read/write)
* : Commands the motor to run or stop. Writing a `1` will cause the motor to
*   run. Writing `0` will cause the motor to stop.
* .
* `run_mode` (read/write)
* : TODO
* .
* `run_modes` (read-only)
* : Returns a space-separated list of valid run modes.
* .
* `speed_regulation` (read/write)
* : Turns speed regulation on or off. If speed regulation is on, the motor
*   controller will vary the power supplied to the motor to try to maintain the
*   speed specified in `speed_sp`. If speed regulation is off, the controller
*   will use the power specified in `duty_cycle_sp`. Valid values are `on` and
*   `off`.
* .
* `speed_regulation_D`: (read/write)
* : TODO
* .
* `speed_regulation_I`: (read/write)
* : TODO
* .
* `speed_regulation_K`: (read/write)
* : TODO
* .
* `speed_regulation_P`: (read/write)
* : TODO
* .
* `state` (read-only)
* : TODO
* .
* `stop_command` (read/write)
* : Reading returns the current stop command. Writing sets the stop command.
*   The value determines the motors behavior when `command` is set to `stop`.
*   Also, it determines the motors behavior when a run command completes. See
*   `stop_commands` for a list of possible values.
* .
* `stop_commands` (read-only)
* : Returns a space-separated list of stop modes supported by the motor controller.
*   Possible values are `coast`, `brake` and `hold`. `coast` means that power will
*   be removed from the motor and it will freely coast to a stop. `brake` means
*   that power will be removed from the motor and a passive electrical load will
*   be placed on the motor. This is usually done by shorting the motor terminals
*   together. This load will absorb the energy from the rotation of the motors and
*   cause the motor to stop more quickly than coasting. `hold` does not remove
*   power from the motor. Instead it actively try to hold the motor at the current
*   position. If an external force tries to turn the motor, the motor will "push
*   back" to maintain its position.
* .
* `time_sp` (read/write)
* : TODO
* .
* [old wiki]: https://github.com/ev3dev/ev3dev/wiki/Using-Motors
*/

#include <linux/device.h>
#include <linux/module.h>

#include <dc_motor_class.h>
#include <tacho_motor_class.h>

struct tacho_motor_value_names {
	const char *name;
};

static struct tacho_motor_value_names
tacho_motor_speed_regulation_names[TM_NUM_SPEED_REGULATION_MODES] = {
	[TM_SPEED_REGULATION_OFF] =  { "off" },
	[TM_SPEED_REGULATION_ON]  =  { "on"  },
};

static struct tacho_motor_value_names
tacho_motor_stop_command_names[TM_NUM_STOP_COMMANDS] = {
	[TM_STOP_COMMAND_COAST]     =  { "coast" },
	[TM_STOP_COMMAND_BRAKE]     =  { "brake" },
	[TM_STOP_COMMAND_HOLD]      =  { "hold" },
};

static struct tacho_motor_value_names tacho_motor_position_modes[TM_NUM_POSITION_MODES] = {
	[TM_POSITION_ABSOLUTE] =  { "absolute" },
	[TM_POSITION_RELATIVE] =  { "relative" },
};

static struct tacho_motor_value_names tacho_motor_run_modes[TM_NUM_RUN_MODES] = {
	[TM_RUN_FOREVER]   =  { "forever"  },
	[TM_RUN_TIME]      =  { "time"     },
	[TM_RUN_POSITION]  =  { "position" },
};

struct tacho_motor_type_item {
	const char *name;
};

struct tacho_motor_state_item {
	const char *name;
};

static struct tacho_motor_value_names tacho_motor_states[TM_NUM_STATES] = {
	[TM_STATE_RUN_FOREVER]			= { "run_forever"		},
	[TM_STATE_SETUP_RAMP_TIME]		= { "setup_ramp_time"		},
	[TM_STATE_SETUP_RAMP_POSITION]		= { "setup_ramp_position"	},
	[TM_STATE_SETUP_RAMP_REGULATION]	= { "setup_ramp_regulation"	},
	[TM_STATE_RAMP_UP]			= { "ramp_up"			},
	[TM_STATE_RAMP_CONST]			= { "ramp_const"		},
	[TM_STATE_POSITION_RAMP_DOWN]		= { "position_ramp_down"	},
	[TM_STATE_RAMP_DOWN]			= { "ramp_down"			},
	[TM_STATE_STOP]				= { "stop"			},
	[TM_STATE_IDLE]				= { "idle"			},
};

static ssize_t port_name_show(struct device *dev, struct device_attribute *attr,
			      char *buf)
{
	struct tacho_motor_device *tm = to_tacho_motor(dev);

	return snprintf(buf, LEGO_PORT_NAME_SIZE, "%s\n", tm->port_name);
}

static ssize_t driver_name_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct tacho_motor_device *tm = to_tacho_motor(dev);

	return sprintf(buf, "%s\n", tm->driver_name);
}

static ssize_t position_show(struct device *dev, struct device_attribute *attr,
			     char *buf)
{
	struct tacho_motor_device *tm = to_tacho_motor(dev);
	long position;
	int err;

	err = tm->ops->get_position(tm, &position);
	if (err < 0)
		return err;

	return sprintf(buf, "%ld\n", position);
}

ssize_t position_store(struct device *dev, struct device_attribute *attr,
		       const char *buf, size_t size)
{
	struct tacho_motor_device *tm = to_tacho_motor(dev);
	char *end;
	long position = simple_strtol(buf, &end, 0);
	int err;

	if (end == buf)
		return -EINVAL;

	err = tm->ops->set_position(tm, position);
	if (err < 0)
		return err;

	return size;
}
#if 0
static ssize_t tacho_motor_show_states(struct device *dev, struct device_attribute *attr, char *buf)
{
        unsigned int i;

	int size = 0;

	for (i=0; i<TM_NUM_STATES; ++i)
		size += sprintf(buf+size, "%s ", tacho_motor_states[i].name);

	size += sprintf(buf+size, "\n");

        return size;
}
#endif
static ssize_t tacho_motor_show_state(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct tacho_motor_device *tm = to_tacho_motor(dev);

	return sprintf(buf, "%s\n", tacho_motor_states[tm->ops->get_state(tm)].name);
}

static ssize_t tacho_motor_show_duty_cycle(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct tacho_motor_device *tm = to_tacho_motor(dev);

	return sprintf(buf, "%d\n", tm->ops->get_duty_cycle(tm));
}

static ssize_t tacho_motor_show_pulses_per_second(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct tacho_motor_device *tm = to_tacho_motor(dev);

	return sprintf(buf, "%d\n", tm->ops->get_pulses_per_second(tm));
}

static ssize_t tacho_motor_show_run_modes(struct device *dev, struct device_attribute *attr, char *buf)
{
        unsigned int i;

	int size = 0;

	for (i=0; i<TM_NUM_RUN_MODES; ++i)
		size += sprintf(buf+size, "%s ", tacho_motor_run_modes[i].name);

	size += sprintf(buf+size, "\n");

        return size;
}

static ssize_t tacho_motor_show_run_mode(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct tacho_motor_device *tm = to_tacho_motor(dev);

	return sprintf(buf, "%s\n", tacho_motor_run_modes[tm->ops->get_run_mode(tm)].name);
}

static ssize_t tacho_motor_store_run_mode(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct tacho_motor_device *tm = to_tacho_motor(dev);

        unsigned int i;

	for (i=0; i<TM_NUM_RUN_MODES; ++i)
		if (sysfs_streq(buf, tacho_motor_run_modes[i].name)) break;

	if (i >= TM_NUM_RUN_MODES)
                return -EINVAL;

        tm->ops->set_run_mode(tm, i);

        return size;
}

static ssize_t speed_regulation_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	struct tacho_motor_device *tm = to_tacho_motor(dev);
	int ret;

	ret = tm->ops->get_speed_regulation(tm);
	if (ret < 0)
		return ret;

	return sprintf(buf, "%s\n", tacho_motor_speed_regulation_names[ret].name);
}

static ssize_t speed_regulation_store(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t size)
{
	struct tacho_motor_device *tm = to_tacho_motor(dev);
	unsigned int i;
	int err;

	for (i=0; i<TM_NUM_SPEED_REGULATION_MODES; ++i) {
		if (sysfs_streq(buf, tacho_motor_speed_regulation_names[i].name))
			break;
	}

	if (i >= TM_NUM_SPEED_REGULATION_MODES)
		return -EINVAL;

	err = tm->ops->set_regulation_mode(tm, i);
	if (err < 0)
		return err;

	return size;
}

static ssize_t stop_commands_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct tacho_motor_device *tm = to_tacho_motor(dev);
	unsigned commands;
	int i;
	int size = 0;

	commands = tm->ops->get_stop_commands(tm);

	for (i = 0; i < TM_NUM_STOP_COMMANDS; i++) {
		if (commands & BIT(i)) {
			size += sprintf(buf + size, "%s ",
				tacho_motor_stop_command_names[i].name);
		}
	}

	buf[size - 1] = '\n';

	return size;
}

static ssize_t stop_command_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct tacho_motor_device *tm = to_tacho_motor(dev);
	int ret;

	ret = tm->ops->get_stop_command(tm);
	if (ret < 0)
		return ret;

	return sprintf(buf, "%s\n", tacho_motor_stop_command_names[ret].name);
}

static ssize_t stop_command_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t size)
{
	struct tacho_motor_device *tm = to_tacho_motor(dev);
	int i, err;

	for (i = 0; i < TM_NUM_STOP_COMMANDS; i++) {
		if (sysfs_streq(buf, tacho_motor_stop_command_names[i].name))
			break;
	}

	if (i >= TM_NUM_STOP_COMMANDS)
		return -EINVAL;

	err = tm->ops->set_stop_command(tm, i);
	if (err < 0)
		return err;

	return size;
}

static ssize_t tacho_motor_show_position_modes(struct device *dev, struct device_attribute *attr, char *buf)
{
        unsigned int i;

	int size = 0;

	for (i=0; i<TM_NUM_POSITION_MODES; ++i)
		size += sprintf(buf+size, "%s ", tacho_motor_position_modes[i].name);

	size += sprintf(buf+size, "\n");

        return size;
}

static ssize_t tacho_motor_show_position_mode(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct tacho_motor_device *tm = to_tacho_motor(dev);

	return sprintf(buf, "%s\n", tacho_motor_position_modes[tm->ops->get_position_mode(tm)].name);
}

static ssize_t tacho_motor_store_position_mode(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct tacho_motor_device *tm = to_tacho_motor(dev);

        unsigned int i;

	for (i=0; i<TM_NUM_POSITION_MODES; ++i)
		if (sysfs_streq( buf, tacho_motor_position_modes[i].name)) break;

	if (i >= TM_NUM_POSITION_MODES)
                return -EINVAL;

        tm->ops->set_position_mode(tm, i);

        return size;
}

static ssize_t polarity_show(struct device *dev, struct device_attribute *attr,
			     char *buf)
{
	struct tacho_motor_device *tm = to_tacho_motor(dev);
	int ret;

	ret = tm->ops->get_polarity(tm);
	if (ret < 0)
		return ret;

	return sprintf(buf, "%s\n", dc_motor_polarity_values[ret]);
}

static ssize_t polarity_store(struct device *dev, struct device_attribute *attr,
			      const char *buf, size_t size)
{
	struct tacho_motor_device *tm = to_tacho_motor(dev);
	int i, err;

	for (i = 0; i < NUM_DC_MOTOR_POLARITY; i++) {
		if (sysfs_streq(buf, dc_motor_polarity_values[i])) {
			err = tm->ops->set_polarity(tm, i);
			if (err < 0)
				return err;
			return size;
		}
	}

	return -EINVAL;
}

static ssize_t encoder_polarity_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	struct tacho_motor_device *tm = to_tacho_motor(dev);
	int ret;

	ret = tm->ops->get_encoder_polarity(tm);
	if (ret < 0)
		return ret;

	return sprintf(buf, "%s\n", dc_motor_polarity_values[ret]);
}

static ssize_t encoder_polarity_store(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t size)
{
	struct tacho_motor_device *tm = to_tacho_motor(dev);
	int i, err;

	for (i = 0; i < NUM_DC_MOTOR_POLARITY; i++) {
		if (sysfs_streq(buf, dc_motor_polarity_values[i])) {
			err = tm->ops->set_encoder_polarity(tm, i);
			if (err < 0)
				return err;
			return size;
		}
	}

	return -EINVAL;
}

static ssize_t tacho_motor_show_ramp_up_sp(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct tacho_motor_device *tm = to_tacho_motor(dev);

	return sprintf(buf, "%d\n", tm->ops->get_ramp_up_sp(tm));
}

static ssize_t tacho_motor_store_ramp_up_sp(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct tacho_motor_device *tm = to_tacho_motor(dev);

        char *end;
        long ramp_up_sp = simple_strtol(buf, &end, 0);

        if ((end == buf) || (ramp_up_sp < 0) || (ramp_up_sp > 10000))
                return -EINVAL;

        tm->ops->set_ramp_up_sp(tm, ramp_up_sp);

        return size;
}

static ssize_t tacho_motor_show_ramp_down_sp(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct tacho_motor_device *tm = to_tacho_motor(dev);

	return sprintf(buf, "%d\n", tm->ops->get_ramp_down_sp(tm));
}

static ssize_t tacho_motor_store_ramp_down_sp(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct tacho_motor_device *tm = to_tacho_motor(dev);

        char *end;
        long ramp_down_sp = simple_strtol(buf, &end, 0);

        if ((end == buf) || (ramp_down_sp < 0) || (ramp_down_sp > 10000))
                return -EINVAL;

        tm->ops->set_ramp_down_sp(tm, ramp_down_sp);

        return size;
}

static ssize_t tacho_motor_show_speed_regulation_P(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct tacho_motor_device *tm = to_tacho_motor(dev);

	return sprintf(buf, "%d\n", tm->ops->get_speed_regulation_P(tm));
}

static ssize_t tacho_motor_store_speed_regulation_P(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct tacho_motor_device *tm = to_tacho_motor(dev);

        char *end;
        long speed_regulation_P = simple_strtol(buf, &end, 0);

        if ((end == buf) || (speed_regulation_P < 0))
                return -EINVAL;

        tm->ops->set_speed_regulation_P(tm, speed_regulation_P);

        return size;
}

static ssize_t tacho_motor_show_speed_regulation_I(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct tacho_motor_device *tm = to_tacho_motor(dev);

	return sprintf(buf, "%d\n", tm->ops->get_speed_regulation_I(tm));
}

static ssize_t tacho_motor_store_speed_regulation_I(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct tacho_motor_device *tm = to_tacho_motor(dev);

        char *end;
        long speed_regulation_I = simple_strtol(buf, &end, 0);

        if ((end == buf) || (speed_regulation_I < 0))
                return -EINVAL;

        tm->ops->set_speed_regulation_I(tm, speed_regulation_I);

        return size;
}
static ssize_t tacho_motor_show_speed_regulation_D(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct tacho_motor_device *tm = to_tacho_motor(dev);

	return sprintf(buf, "%d\n", tm->ops->get_speed_regulation_D(tm));
}

static ssize_t tacho_motor_store_speed_regulation_D(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct tacho_motor_device *tm = to_tacho_motor(dev);

        char *end;
        long speed_regulation_D = simple_strtol(buf, &end, 0);

        if ((end == buf) || (speed_regulation_D < 0))
                return -EINVAL;

        tm->ops->set_speed_regulation_D(tm, speed_regulation_D);

        return size;
}

static ssize_t tacho_motor_show_duty_cycle_sp(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct tacho_motor_device *tm = to_tacho_motor(dev);

	return sprintf(buf, "%d\n", tm->ops->get_duty_cycle_sp(tm));
}

static ssize_t tacho_motor_store_duty_cycle_sp(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct tacho_motor_device *tm = to_tacho_motor(dev);

        char *end;
        long duty_cycle_sp = simple_strtol(buf, &end, 0);

        if ((end == buf) || (duty_cycle_sp > 100) || (duty_cycle_sp < -100))
                return -EINVAL;

        tm->ops->set_duty_cycle_sp(tm, duty_cycle_sp);

        return size;
}

static ssize_t tacho_motor_show_pulses_per_second_sp(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct tacho_motor_device *tm = to_tacho_motor(dev);

	return sprintf(buf, "%d\n", tm->ops->get_pulses_per_second_sp(tm));
}

static ssize_t tacho_motor_store_pulses_per_second_sp(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct tacho_motor_device *tm = to_tacho_motor(dev);

        char *end;
        long pulses_per_second_sp = simple_strtol(buf, &end, 0);

        if ((end == buf) || (pulses_per_second_sp > 2000) || (pulses_per_second_sp < -2000))
                return -EINVAL;

        tm->ops->set_pulses_per_second_sp(tm, pulses_per_second_sp);

        return size;
}

static ssize_t tacho_motor_show_time_sp(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct tacho_motor_device *tm = to_tacho_motor(dev);

	return sprintf(buf, "%d\n", tm->ops->get_time_sp(tm));
}

static ssize_t tacho_motor_store_time_sp(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct tacho_motor_device *tm = to_tacho_motor(dev);

        char *end;
        long time_sp = simple_strtol(buf, &end, 0);

        if ((end == buf) || (time_sp < 0))
                return -EINVAL;

        tm->ops->set_time_sp(tm, time_sp);

        return size;
}

static ssize_t tacho_motor_show_position_sp(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct tacho_motor_device *tm = to_tacho_motor(dev);

	return sprintf(buf, "%d\n", tm->ops->get_position_sp(tm));
}

static ssize_t tacho_motor_store_position_sp(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct tacho_motor_device *tm = to_tacho_motor(dev);

        char *end;
        long position_sp = simple_strtol(buf, &end, 0);

        if (end == buf)
                return -EINVAL;

        tm->ops->set_position_sp(tm, position_sp);

        return size;
}

static ssize_t tacho_motor_show_run(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct tacho_motor_device *tm = to_tacho_motor(dev);

	return sprintf(buf, "%d\n", tm->ops->get_run(tm));
}

static ssize_t tacho_motor_store_run(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct tacho_motor_device *tm = to_tacho_motor(dev);

        char *end;
        long run = simple_strtol(buf, &end, 0);

        if ((end == buf) || (run > 1) || (run < 0))
                return -EINVAL;

        tm->ops->set_run(tm, run);

        return size;
}

static ssize_t tacho_motor_store_reset(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct tacho_motor_device *tm = to_tacho_motor(dev);

        char *end;
        long reset = simple_strtol(buf, &end, 0);

        if ((end == buf) || (reset != 1))
                return -EINVAL;

        tm->ops->set_reset(tm, reset);

        return size;
}

DEVICE_ATTR_RO(driver_name);
DEVICE_ATTR_RO(port_name);
DEVICE_ATTR_RW(position);

DEVICE_ATTR(state, S_IRUGO, tacho_motor_show_state, NULL);
DEVICE_ATTR(duty_cycle, S_IRUGO, tacho_motor_show_duty_cycle, NULL);
DEVICE_ATTR(pulses_per_second, S_IRUGO, tacho_motor_show_pulses_per_second, NULL);

DEVICE_ATTR(duty_cycle_sp, S_IRUGO | S_IWUSR, tacho_motor_show_duty_cycle_sp, tacho_motor_store_duty_cycle_sp);
DEVICE_ATTR(pulses_per_second_sp, S_IRUGO | S_IWUSR, tacho_motor_show_pulses_per_second_sp, tacho_motor_store_pulses_per_second_sp);
DEVICE_ATTR(time_sp, S_IRUGO | S_IWUSR, tacho_motor_show_time_sp, tacho_motor_store_time_sp);
DEVICE_ATTR(position_sp, S_IRUGO | S_IWUSR, tacho_motor_show_position_sp, tacho_motor_store_position_sp);

DEVICE_ATTR(run_modes, S_IRUGO, tacho_motor_show_run_modes, NULL);
DEVICE_ATTR(run_mode, S_IRUGO | S_IWUSR, tacho_motor_show_run_mode, tacho_motor_store_run_mode);
DEVICE_ATTR_RW(speed_regulation);
DEVICE_ATTR_RO(stop_commands);
DEVICE_ATTR_RW(stop_command);
DEVICE_ATTR(position_modes, S_IRUGO, tacho_motor_show_position_modes, NULL);
DEVICE_ATTR(position_mode, S_IRUGO | S_IWUSR, tacho_motor_show_position_mode, tacho_motor_store_position_mode);
DEVICE_ATTR_RW(polarity);
DEVICE_ATTR_RW(encoder_polarity);

DEVICE_ATTR(ramp_up_sp, S_IRUGO | S_IWUSR, tacho_motor_show_ramp_up_sp, tacho_motor_store_ramp_up_sp);
DEVICE_ATTR(ramp_down_sp, S_IRUGO | S_IWUSR, tacho_motor_show_ramp_down_sp, tacho_motor_store_ramp_down_sp);

DEVICE_ATTR(speed_regulation_P, S_IRUGO | S_IWUSR, tacho_motor_show_speed_regulation_P, tacho_motor_store_speed_regulation_P);
DEVICE_ATTR(speed_regulation_I, S_IRUGO | S_IWUSR, tacho_motor_show_speed_regulation_I, tacho_motor_store_speed_regulation_I);
DEVICE_ATTR(speed_regulation_D, S_IRUGO | S_IWUSR, tacho_motor_show_speed_regulation_D, tacho_motor_store_speed_regulation_D);

DEVICE_ATTR(run, S_IRUGO | S_IWUSR, tacho_motor_show_run, tacho_motor_store_run);

DEVICE_ATTR(reset, S_IWUSR, NULL, tacho_motor_store_reset);

static struct attribute *tacho_motor_class_attrs[] = {
	&dev_attr_driver_name.attr,
	&dev_attr_port_name.attr,
	&dev_attr_position.attr,
	&dev_attr_state.attr,
	&dev_attr_duty_cycle.attr,
	&dev_attr_pulses_per_second.attr,
	&dev_attr_duty_cycle_sp.attr,
	&dev_attr_pulses_per_second_sp.attr,
	&dev_attr_time_sp.attr,
	&dev_attr_position_sp.attr,
	&dev_attr_run_modes.attr,
	&dev_attr_run_mode.attr,
	&dev_attr_speed_regulation.attr,
	&dev_attr_stop_commands.attr,
	&dev_attr_stop_command.attr,
	&dev_attr_position_modes.attr,
	&dev_attr_position_mode.attr,
	&dev_attr_polarity.attr,
	&dev_attr_encoder_polarity.attr,
	&dev_attr_ramp_up_sp.attr,
	&dev_attr_ramp_down_sp.attr,
	&dev_attr_speed_regulation_P.attr,
	&dev_attr_speed_regulation_I.attr,
	&dev_attr_speed_regulation_D.attr,
	&dev_attr_run.attr,
	&dev_attr_reset.attr,
	NULL
};
ATTRIBUTE_GROUPS(tacho_motor_class);

static void tacho_motor_release(struct device *dev)
{
}

void tacho_motor_notify_state_change(struct tacho_motor_device *tm)
{
	sysfs_notify(&tm->dev.kobj, NULL, "state");
}
EXPORT_SYMBOL_GPL(tacho_motor_notify_state_change);

static unsigned tacho_motor_class_id = 0;

int register_tacho_motor(struct tacho_motor_device *tm, struct device *parent)
{
	int err;

	if (!tm || !tm->port_name || !parent)
		return -EINVAL;
	
	tm->dev.release = tacho_motor_release;
	tm->dev.parent = parent;
	tm->dev.class = &tacho_motor_class;
	dev_set_name(&tm->dev, "motor%d", tacho_motor_class_id++);

	err = device_register(&tm->dev);
	if (err)
		return err;

	dev_info(&tm->dev, "Bound to '%s'.\n", dev_name(parent));

	return 0;
}
EXPORT_SYMBOL_GPL(register_tacho_motor);

void unregister_tacho_motor(struct tacho_motor_device *tm)
{
	dev_info(&tm->dev, "Unregistered.\n");
	device_unregister(&tm->dev);
}
EXPORT_SYMBOL_GPL(unregister_tacho_motor);

static int tacho_motor_dev_uevent(struct device *dev, struct kobj_uevent_env *env)
{
	struct tacho_motor_device *tm = to_tacho_motor(dev);
	int ret;

	ret = add_uevent_var(env, "LEGO_DRIVER_NAME=%s", tm->driver_name);
		if (ret) {
			dev_err(dev, "failed to add uevent LEGO_DRIVER_NAME\n");
			return ret;
		}

	ret = add_uevent_var(env, "LEGO_PORT_NAME=%s", tm->port_name);
	if (ret) {
		dev_err(dev, "failed to add uevent LEGO_PORT_NAME\n");
		return ret;
	}

	return 0;
}

static char *tacho_motor_devnode(struct device *dev, umode_t *mode)
{
	return kasprintf(GFP_KERNEL, "tacho-motor/%s", dev_name(dev));
}

struct class tacho_motor_class = {
	.name		= "tacho-motor",
	.owner		= THIS_MODULE,
	.dev_groups	= tacho_motor_class_groups,
	.dev_uevent	= tacho_motor_dev_uevent,
	.devnode	= tacho_motor_devnode,
};

static int tacho_motor_class_init(void)
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

static void tacho_motor_class_exit(void)
{
	class_unregister(&tacho_motor_class);
}
module_exit(tacho_motor_class_exit);

MODULE_DESCRIPTION("Tacho Motor device class");
MODULE_AUTHOR("Ralph Hempel <rhempel@hempeldesigngroup.com>");
MODULE_LICENSE("GPL");
