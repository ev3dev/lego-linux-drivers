/*
 * DC motor device class for LEGO MINDSTORMS EV3
 *
 * Copyright (C) 2014 David Lechner <david@lechnology.com>
 * Copyright (C) 2014 Ralph Hempel <rhempel@hempeldesigngroup.com>
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
 * Use kramdown (markdown) format. Use a '.' as a placeholder when blank lines
 * or leading whitespace is important for the markdown syntax.
 */

/**
 * DOC: website
 *
 * DC Motor Class
 *
 * The `dc-motor` class provides a uniform interface for using regular DC motors
 * with no fancy controls or feedback. This includes LEGO MINDSTORMS RCX motors
 * and LEGO Power Functions motors.
 * .
 * ### sysfs Attributes
 * .
 * DC motors can be found at `/sys/class/dc-motor/motor<N>`, where `<N>`
 * is incremented each time a motor is loaded (it is not related to which port
 * the motor is plugged in to).
 * .
 * `command` (read/write)
 * : Sets the command for the motor. Possible values are `run`, `brake` and
 *  `coast`. Not all commands may be supported, so be sure to check the contents
 *   of the `commands` attribute.
 * .
 * `commands` (read-only)
 * : Returns a space separated list of commands supported by the motor
 *   controller.
 * .
 * `driver_name` (read-only)
 * : Returns the name of the motor driver that loaded this device. See the list
 *   of [supported devices] for a list of drivers.
 * .
 * `duty_cycle` (read)
 * : Shows the current duty cycle of the PWM signal sent to the motor. Values
 *   are -100 to 100 (-100% to 100%).
 * .
 * `duty_cycle_sp` (read/write)
 * : Writing sets the duty cycle setpoint of the PWM signal sent to the motor.
 *   Valid values are -100 to 100 (-100% to 100%). Reading returns the current
 *   setpoint.
 * .
 * `polarity`: (read/write)
 * : Sets the polarity of the motor. Valid values are `normal` and `inverted`.
 * .
 * `port_name` (read-only)
 * : Returns the name of the port that the motor is connected to.
 * .
 * `ramp_down_sp` (read/write)
 * : Sets the time in milliseconds that it take the motor to ramp down from 100%
 *   to 0%. Valid values are 0 to 10000 (10 seconds). Default is 0. If the
 *   controller does not support ramping, then reading and writing will fail
 *   with -EOPNOTSUPP.
 * .
 * `ramp_up_sp` (read/write)
 * : Sets the time in milliseconds that it take the motor to up ramp from 0% to
 *   100%. Valid values are 0 to 10000 (10 seconds). Default is 0. If the
 *   controller does not support ramping, then reading and writing will fail
 *   with -EOPNOTSUPP.
 * .
 * [supported devices]: /docs/motors/#supported-devices
 */

#include <linux/device.h>
#include <linux/module.h>

#include <dc_motor_class.h>

const char* dc_motor_command_names[] = {
	[DC_MOTOR_COMMAND_RUN]		= "run",
	[DC_MOTOR_COMMAND_COAST]	= "coast",
	[DC_MOTOR_COMMAND_BRAKE]	= "brake",
};
EXPORT_SYMBOL_GPL(dc_motor_command_names);

const char* dc_motor_polarity_values[] = {
	[DC_MOTOR_POLARITY_NORMAL]	= "normal",
	[DC_MOTOR_POLARITY_INVERTED]	= "inverted",
};
EXPORT_SYMBOL_GPL(dc_motor_polarity_values);

enum hrtimer_restart dc_motor_class_ramp_timer_handler(struct hrtimer *timer)
{
	struct dc_motor_device *motor =
		container_of(timer, struct dc_motor_device, ramp_timer);
	int ramp_ns, err, direction;

	if (motor->current_duty_cycle == motor->target_duty_cycle)
		return HRTIMER_NORESTART;

	if (motor->current_duty_cycle < motor->target_duty_cycle) {
		ramp_ns = motor->ramp_up_sp * 10000;
		motor->current_duty_cycle++;
	} else {
		ramp_ns = motor->ramp_down_sp * 10000;
		motor->current_duty_cycle--;
	}

	if (0 == ramp_ns)
		motor->current_duty_cycle = motor->target_duty_cycle;

	hrtimer_forward_now(&motor->ramp_timer, ktime_set(0, ramp_ns));

	if (motor->polarity == DC_MOTOR_POLARITY_NORMAL )
		direction = (motor->current_duty_cycle >= 0)
				? DC_MOTOR_DIRECTION_FORWARD
				: DC_MOTOR_DIRECTION_REVERSE;
	else
		direction = (motor->current_duty_cycle <= 0)
				? DC_MOTOR_DIRECTION_FORWARD
				: DC_MOTOR_DIRECTION_REVERSE;

	err = motor->ops->set_direction(motor->context, direction );
	WARN_ONCE(err, "Failed to set direction.");

	err = motor->ops->set_duty_cycle(motor->context,
					abs(motor->current_duty_cycle));
	WARN_ONCE(err, "Failed to set duty cycle.");

	return HRTIMER_RESTART;
}

static ssize_t driver_name_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct dc_motor_device *motor = to_dc_motor_device(dev);

	return snprintf(buf, DC_MOTOR_NAME_SIZE, "%s\n", motor->name);
}

static ssize_t port_name_show(struct device *dev, struct device_attribute *attr,
			      char *buf)
{
	struct dc_motor_device *motor = to_dc_motor_device(dev);

	return snprintf(buf, DC_MOTOR_NAME_SIZE, "%s\n", motor->port_name);
}

static ssize_t ramp_up_sp_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct dc_motor_device *motor = to_dc_motor_device(dev);

	return sprintf(buf, "%u\n", motor->ramp_up_sp);
}

static ssize_t ramp_up_sp_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct dc_motor_device *motor = to_dc_motor_device(dev);
	unsigned value;

	if (sscanf(buf, "%ud", &value) != 1 || value > 10000)
		return -EINVAL;
	motor->ramp_up_sp = value;

	return count;
}

static ssize_t ramp_down_sp_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct dc_motor_device *motor = to_dc_motor_device(dev);

	return sprintf(buf, "%u\n", motor->ramp_down_sp);
}

static ssize_t ramp_down_sp_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	struct dc_motor_device *motor = to_dc_motor_device(dev);
	unsigned value;

	if (sscanf(buf, "%ud", &value) != 1 || value > 10000)
		return -EINVAL;
	motor->ramp_down_sp = value;

	return count;
}

static ssize_t polarity_show(struct device *dev,
			     struct device_attribute *attr, char *buf)
{
	struct dc_motor_device *motor = to_dc_motor_device(dev);

	return sprintf(buf, "%s\n", dc_motor_polarity_values[motor->polarity]);
}

static ssize_t polarity_store(struct device *dev, struct device_attribute *attr,
			      const char *buf, size_t count)
{
	struct dc_motor_device *motor = to_dc_motor_device(dev);
	int i;

	for (i = 0; i < NUM_DC_MOTOR_POLARITY; i++) {
		if (sysfs_streq(buf, dc_motor_polarity_values[i])) {
			if (motor->polarity != i) {
				/*
				 * Force the motor state to get re-evaluated when
				 * timer expires, makes the motor use ramping even
				 * if we change direction mid-flight
				 */
				motor->current_duty_cycle = -motor->current_duty_cycle;

				motor->polarity = i;
				hrtimer_start(&motor->ramp_timer,
					ktime_set(0, 0), HRTIMER_MODE_REL);
			}
			return count;
		}

	}
	return -EINVAL;
}

static ssize_t duty_cycle_sp_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct dc_motor_device *motor = to_dc_motor_device(dev);
	int duty_cycle_sp;

	duty_cycle_sp = motor->target_duty_cycle;
	return sprintf(buf, "%d\n", duty_cycle_sp);
}

static ssize_t duty_cycle_sp_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	struct dc_motor_device *motor = to_dc_motor_device(dev);
	int value;

	if (sscanf(buf, "%d", &value) != 1 || value < -100 || value > 100)
		return -EINVAL;
	motor->target_duty_cycle = value;

	if (motor->ops->get_command(motor->context) == DC_MOTOR_COMMAND_RUN)
		hrtimer_start(&motor->ramp_timer, ktime_set(0, 0), HRTIMER_MODE_REL);

	return count;
}

static ssize_t duty_cycle_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct dc_motor_device *motor = to_dc_motor_device(dev);
	int duty_cycle;
	unsigned direction;
	unsigned polarity;

	duty_cycle = motor->ops->get_duty_cycle(motor->context);
	direction = motor->ops->get_direction(motor->context);
	polarity = motor->polarity;

	duty_cycle *= ((direction == DC_MOTOR_DIRECTION_FORWARD) ? 1 : -1);
	duty_cycle *= ((polarity == DC_MOTOR_POLARITY_NORMAL) ? 1 : -1);

	return sprintf(buf, "%d\n", duty_cycle);
}

static ssize_t commands_show(struct device *dev, struct device_attribute *attr,
			     char *buf)
{
	struct dc_motor_device *motor = to_dc_motor_device(dev);
	unsigned supported_commands;
	int i;
	int count = 0;

	supported_commands = motor->ops->get_supported_commands(motor->context);
	for (i = 0; i < NUM_DC_MOTOR_COMMANDS; i++) {
		if (supported_commands & BIT(i))
			count += sprintf(buf + count, "%s ",
				dc_motor_command_names[i]);
	}
	buf[count - 1] = '\n';
	return count;
}

static ssize_t command_show(struct device *dev, struct device_attribute *attr,
			    char *buf)
{
	struct dc_motor_device *motor = to_dc_motor_device(dev);
	int ret;

	ret = motor->ops->get_command(motor->context);
	if (ret < 0)
		return ret;
	return sprintf(buf, "%s\n", dc_motor_command_names[ret]);
}

static ssize_t command_store(struct device *dev, struct device_attribute *attr,
			     const char *buf, size_t count)
{
	struct dc_motor_device *motor = to_dc_motor_device(dev);
	unsigned supported_commands;
	int i, err;

	supported_commands = motor->ops->get_supported_commands(motor->context);
	for (i = 0; i < NUM_DC_MOTOR_COMMANDS; i++) {
		if (!sysfs_streq(buf, dc_motor_command_names[i]))
			continue;
		if (supported_commands & BIT(i)) {
			err = motor->ops->set_command(motor->context,
				i);
			if (err)
				return err;
			if (i == DC_MOTOR_COMMAND_RUN) {
				hrtimer_start(&motor->ramp_timer,
					ktime_set(0, 0), HRTIMER_MODE_REL);
			} else {
				hrtimer_cancel(&motor->ramp_timer);
				motor->current_duty_cycle = 0;
			}
			return count;
		}

	}
	return -EINVAL;
}

static DEVICE_ATTR_RO(driver_name);
static DEVICE_ATTR_RO(port_name);
static DEVICE_ATTR_RW(ramp_up_sp);
static DEVICE_ATTR_RW(ramp_down_sp);
static DEVICE_ATTR_RW(polarity);
static DEVICE_ATTR_RW(duty_cycle_sp);
static DEVICE_ATTR_RO(duty_cycle);
static DEVICE_ATTR_RO(commands);
static DEVICE_ATTR_RW(command);

static struct attribute *dc_motor_class_attrs[] = {
	&dev_attr_driver_name.attr,
	&dev_attr_port_name.attr,
	&dev_attr_ramp_up_sp.attr,
	&dev_attr_ramp_down_sp.attr,
	&dev_attr_polarity.attr,
	&dev_attr_duty_cycle_sp.attr,
	&dev_attr_duty_cycle.attr,
	&dev_attr_commands.attr,
	&dev_attr_command.attr,
	NULL
};

static const struct attribute_group dc_motor_class_group = {
	.attrs		= dc_motor_class_attrs,
};

static const struct attribute_group *dc_motor_class_groups[] = {
	&dc_motor_class_group,
	NULL
};

static void dc_motor_release(struct device *dev)
{
}

static unsigned dc_motor_class_id = 0;
struct class dc_motor_class;

int register_dc_motor(struct dc_motor_device *dc, struct device *parent)
{
	int err;

	if (!dc || !dc->port_name || !parent)
		return -EINVAL;

	dc->dev.release = dc_motor_release;
	dc->dev.parent = parent;
	dc->dev.class = &dc_motor_class;
	dev_set_name(&dc->dev, "motor%d", dc_motor_class_id++);

	hrtimer_init(&dc->ramp_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	dc->ramp_timer.function = dc_motor_class_ramp_timer_handler;

	err = device_register(&dc->dev);
	if (err)
		return err;

	dev_info(&dc->dev, "Bound to device '%s'\n", dev_name(parent));

	return 0;
}
EXPORT_SYMBOL_GPL(register_dc_motor);

void unregister_dc_motor(struct dc_motor_device *dc)
{
	dev_info(&dc->dev, "Unregistered.\n");
	hrtimer_cancel(&dc->ramp_timer);
	device_unregister(&dc->dev);
}
EXPORT_SYMBOL_GPL(unregister_dc_motor);

static int dc_motor_dev_uevent(struct device *dev, struct kobj_uevent_env *env)
{
	struct dc_motor_device *motor = to_dc_motor_device(dev);
	int ret;

	ret = add_uevent_var(env, "LEGO_DRIVER_NAME=%s", motor->name);
	if (ret) {
		dev_err(dev, "failed to add uevent LEGO_DRIVER_NAME\n");
		return ret;
	}
	add_uevent_var(env, "LEGO_PORT_NAME=%s", motor->port_name);
	if (ret) {
		dev_err(dev, "failed to add uevent LEGO_PORT_NAME\n");
		return ret;
	}

	return 0;
}

static char *dc_motor_devnode(struct device *dev, umode_t *mode)
{
	return kasprintf(GFP_KERNEL, "dc-motor/%s", dev_name(dev));
}

struct class dc_motor_class = {
	.name		= "dc-motor",
	.owner		= THIS_MODULE,
	.dev_groups	= dc_motor_class_groups,
	.dev_uevent	= dc_motor_dev_uevent,
	.devnode	= dc_motor_devnode,
};
EXPORT_SYMBOL_GPL(dc_motor_class);

static int __init dc_motor_class_init(void)
{
	int err;

	err = class_register(&dc_motor_class);
	if (err) {
		pr_err("unable to register DC motor device class\n");
		return err;
	}

	return 0;
}
module_init(dc_motor_class_init);

static void __exit dc_motor_class_exit(void)
{
	class_unregister(&dc_motor_class);
}
module_exit(dc_motor_class_exit);

MODULE_DESCRIPTION("DC motor device class for LEGO MINDSTORMS EV3");
MODULE_AUTHOR("David Lechner <david@lechnology.com>");
MODULE_LICENSE("GPL");
