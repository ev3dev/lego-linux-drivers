/*
 * DC motor device class for LEGO MINDSTORMS EV3
 *
 * Copyright (C) 2014-2015 David Lechner <david@lechnology.com>
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
 * Use kramdown (markdown) syntax. Use a '.' as a placeholder when blank lines
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
 * `address`
 * : (read-only) Returns the name of the port that the motor is connected to.
 * .
 * `command`
 * : (write-only) Sets the command for the motor. Possible values are:
 * .
 * .    - `run-forever`: Causes the motor to run until another command is sent.
 * .    - `run-timed`: Runs the motor for the amount of time specified in
 * .      `time_sp` and then stops the motor using the command specified by
 * .      `stop_action`.
 * .    - `run-direct`: Runs the motor at the duty cycle specified by
 * .      `duty_cycle_sp`. Unlike other run commands, changing `duty_cycle_sp`
 * .      while running *will* take effect immediately.
 * .    - `stop`: Stops any of the run commands before they are complete using
 * .      the command specified by `stop_action`.
 * .
 * .    Not all commands may be supported. Read `commands` to find out which
 * .    commands are supported for a particular driver.
 * .
 * `commands`
 * : (read-only) Returns a space separated list of commands supported by the
 *   motor controller.
 * .
 * `driver_name`
 * : (read-only) Returns the name of the motor driver that loaded this device.
 *   See the list of [supported devices] for a list of drivers.
 * .
 * `duty_cycle`
 * : (read-only) Shows the current duty cycle of the PWM signal sent to the
 *   motor. Values are -100 to 100 (-100% to 100%).
 * .
 * `duty_cycle_sp`
 * : (read/write) Writing sets the duty cycle setpoint of the PWM signal sent to
 *   the motor. Valid values are -100 to 100 (-100% to 100%). Reading returns
 *   the current setpoint.
 * .
 * `polarity`
 * : (read/write) Sets the polarity of the motor. Valid values are:
 * .
 * .    - `normal`: Causes the motor to turn in the direction indicated by the
 * .      sign (+/-) of the `duty_cycle_sp`.
 * .    - `inversed`: Causes the motor to turn in the opposite direction of the
 * .      sign (+/-) of the `duty_cycle_sp`.
 * .
 * `state`
 * : (read-only) Gets a space separated list of flags indicating the motor
 *   status. Possible flags are:
 * .
 * .    - `running`: Indicates that the motor is powered.
 * .    - `ramping`: Indicates that the motor has not yet reached the
 * .      `duty_cycle_sp`.
 * .
 * `stop_action`
 * : (write-only) Sets the stop action that will be used when the motor stops.
 *   Possible values are:
 * .
 * .    - `coast`: Causes the motor to coast to a stop by floating the outputs.
 * .    - `brake`: Causes the motor to stop more quickly by shorting the outputs.
 * .
 * .    Not all values may be supported. Read `stop_actions` to find out which
 * .    actions are supported for a particular driver.
 * .
 * `stop_actions`
 * : (read-only) Gets a space separated list of supported stop actions.
 * .
 * `ramp_down_sp`
 * : (read/write) Sets the time in milliseconds that it take the motor to ramp
 *   down from 100% to 0%. Valid values are 0 to 10000 (10 seconds). Default is
 *   0.
 * .
 * `ramp_up_sp`
 * : (read/write) Sets the time in milliseconds that it take the motor to up
 *   ramp from 0% to 100%. Valid values are 0 to 10000 (10 seconds). Default is
 *   0.
 * .
 * `time_sp`
 * : (read/write) Sets the time setpoint used with the `run-timed` command.
 *   Units are in milliseconds.
 * .
 * [supported devices]: /docs/motors/#supported-devices
 */

#include <linux/device.h>
#include <linux/module.h>

#include <dc_motor_class.h>

#define RAMP_PERIOD	msecs_to_jiffies(100)

const char* dc_motor_command_names[] = {
	[DC_MOTOR_COMMAND_RUN_FOREVER]	= "run-forever",
	[DC_MOTOR_COMMAND_RUN_TIMED]	= "run-timed",
	[DC_MOTOR_COMMAND_RUN_DIRECT]	= "run-direct",
	[DC_MOTOR_COMMAND_STOP]		= "stop",
};

const char* dc_motor_stop_action_names[] = {
	[DC_MOTOR_STOP_ACTION_COAST]	= "coast",
	[DC_MOTOR_STOP_ACTION_BRAKE]	= "brake",
};

const char* dc_motor_polarity_values[] = {
	[DC_MOTOR_POLARITY_NORMAL]	= "normal",
	[DC_MOTOR_POLARITY_INVERSED]	= "inversed",
};
EXPORT_SYMBOL_GPL(dc_motor_polarity_values);

const char* dc_motor_state_names[] = {
	[DC_MOTOR_STATE_RUNNING]	= "running",
	[DC_MOTOR_STATE_RAMPING]	= "ramping",
};

void dc_motor_class_start_motor_ramp(struct dc_motor_device *motor)
{
	unsigned long ramp_sp, now;

	motor->ramp_delta_duty_cycle =
		motor->active_params.duty_cycle_sp - motor->duty_cycle;
	if (motor->ramp_delta_duty_cycle > 0)
		ramp_sp = msecs_to_jiffies(motor->active_params.ramp_up_sp);
	else
		ramp_sp = msecs_to_jiffies(motor->active_params.ramp_down_sp);
	motor->ramp_delta_time = ramp_sp * abs(motor->ramp_delta_duty_cycle) / 100;
	now = jiffies;
	motor->ramp_end_time = now + motor->ramp_delta_time;
	motor->last_ramp_work_time = now;
	schedule_delayed_work(&motor->ramp_work, 0);
}

static void dc_motor_class_ramp_work(struct work_struct *work)
{
	struct dc_motor_device *motor =
		container_of(to_delayed_work(work), struct dc_motor_device, ramp_work);
	enum dc_motor_internal_command internal_command;
	unsigned long remaining_ramp_time = 0;
	unsigned long last_ramp_time;
	int  err;

	/* check to see if we are running and done ramping */
	if (IS_DC_MOTOR_RUN_COMMAND(motor->command)
		&& motor->active_params.duty_cycle_sp == motor->duty_cycle)
	{
		return;
	}

	/*
	 * If we haven't reached the end of the ramp yet, set the duty cycle
	 * to the appropriate point along the ramp, otherwise set the duty
	 * directly to the setpoint.
	 */
	if (time_is_after_jiffies(motor->ramp_end_time))
		remaining_ramp_time = motor->ramp_end_time - jiffies;
	if (motor->ramp_delta_time != 0 && remaining_ramp_time != 0) {
		motor->duty_cycle = motor->active_params.duty_cycle_sp
			- motor->ramp_delta_duty_cycle * (int)remaining_ramp_time
				/ motor->ramp_delta_time;
		if (motor->duty_cycle > DC_MOTOR_MAX_DUTY_CYCLE)
			motor->duty_cycle = DC_MOTOR_MAX_DUTY_CYCLE;
		else if (motor->duty_cycle < -DC_MOTOR_MAX_DUTY_CYCLE)
			motor->duty_cycle = -DC_MOTOR_MAX_DUTY_CYCLE;
	} else {
		motor->duty_cycle = motor->active_params.duty_cycle_sp;
	}

	/*
	 * Translate the current state into an internal command.
	 */
	if (IS_DC_MOTOR_RUN_COMMAND(motor->command) || motor->duty_cycle != 0) {
		internal_command = (motor->duty_cycle >= 0)
				? DC_MOTOR_INTERNAL_COMMAND_RUN_FORWARD
				: DC_MOTOR_INTERNAL_COMMAND_RUN_REVERSE;
	} else {
		internal_command =
			(motor->active_params.stop_action == DC_MOTOR_STOP_ACTION_BRAKE)
			? DC_MOTOR_INTERNAL_COMMAND_BRAKE
			: DC_MOTOR_INTERNAL_COMMAND_COAST;
	}

	err = motor->ops->set_command(motor->context, internal_command);
	WARN_ONCE(err, "Failed to set command");

	err = motor->ops->set_duty_cycle(motor->context,
					 abs(motor->duty_cycle));
	WARN_ONCE(err, "Failed to set duty cycle.");

	/* don't reschedule ramp_work if we are stopped */
	if (!IS_DC_MOTOR_RUN_COMMAND(motor->command)
		&& !IS_DC_MOTOR_INTERNAL_RUN_COMMAND(internal_command))
	{
		return;
	}

	/*
	 * Measure how long it took since the last call to ramp_work and
	 * schedule the next ramp as close to RAMP_PERIOD as we can get
	 */
	last_ramp_time = jiffies - motor->last_ramp_work_time;
	motor->last_ramp_work_time = jiffies;
	schedule_delayed_work(&motor->ramp_work,
		last_ramp_time >= RAMP_PERIOD ? 0 : RAMP_PERIOD - last_ramp_time);
}

static void dc_motor_class_run_timed_work(struct work_struct *work)
{
	struct dc_motor_device *motor =
		container_of(to_delayed_work(work), struct dc_motor_device, run_timed_work);

	motor->command = DC_MOTOR_COMMAND_STOP;
	motor->active_params.duty_cycle_sp = 0;
	dc_motor_class_start_motor_ramp(motor);
}

static ssize_t driver_name_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct dc_motor_device *motor = to_dc_motor_device(dev);

	return snprintf(buf, DC_MOTOR_NAME_SIZE, "%s\n", motor->name);
}

static ssize_t address_show(struct device *dev, struct device_attribute *attr,
			      char *buf)
{
	struct dc_motor_device *motor = to_dc_motor_device(dev);

	return snprintf(buf, DC_MOTOR_NAME_SIZE, "%s\n", motor->address);
}

static ssize_t ramp_up_sp_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct dc_motor_device *motor = to_dc_motor_device(dev);

	return sprintf(buf, "%u\n", motor->params.ramp_up_sp);
}

static ssize_t ramp_up_sp_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct dc_motor_device *motor = to_dc_motor_device(dev);
	unsigned value;

	if (sscanf(buf, "%ud", &value) != 1 || value > 10000)
		return -EINVAL;
	motor->params.ramp_up_sp = value;

	return count;
}

static ssize_t ramp_down_sp_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct dc_motor_device *motor = to_dc_motor_device(dev);

	return sprintf(buf, "%u\n", motor->params.ramp_down_sp);
}

static ssize_t ramp_down_sp_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	struct dc_motor_device *motor = to_dc_motor_device(dev);
	unsigned value;

	if (sscanf(buf, "%ud", &value) != 1 || value > 10000)
		return -EINVAL;
	motor->params.ramp_down_sp = value;

	return count;
}

static ssize_t polarity_show(struct device *dev,
			     struct device_attribute *attr, char *buf)
{
	struct dc_motor_device *motor = to_dc_motor_device(dev);

	return sprintf(buf, "%s\n", dc_motor_polarity_values[motor->params.polarity]);
}

static ssize_t polarity_store(struct device *dev, struct device_attribute *attr,
			      const char *buf, size_t count)
{
	struct dc_motor_device *motor = to_dc_motor_device(dev);
	int i;

	for (i = 0; i < NUM_DC_MOTOR_POLARITY; i++) {
		if (sysfs_streq(buf, dc_motor_polarity_values[i])) {
			motor->params.polarity = i;
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

	duty_cycle_sp = motor->params.duty_cycle_sp;
	return sprintf(buf, "%d\n", duty_cycle_sp);
}

static ssize_t duty_cycle_sp_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	struct dc_motor_device *motor = to_dc_motor_device(dev);
	int err, duty_cycle;

	err = kstrtoint(buf, 10, &duty_cycle);
	if (err < 0)
		return err;

	if (abs(duty_cycle) > DC_MOTOR_MAX_DUTY_CYCLE)
		return -EINVAL;

	motor->params.duty_cycle_sp = duty_cycle;

	/* If we're in run-direct mode, allow the duty_cycle_sp to change */

	if( motor->command == DC_MOTOR_COMMAND_RUN_DIRECT ) {
		if (motor->active_params.polarity == DC_MOTOR_POLARITY_INVERSED)
			motor->active_params.duty_cycle_sp = motor->params.duty_cycle_sp * -1;
		else
			motor->active_params.duty_cycle_sp = motor->params.duty_cycle_sp;
		dc_motor_class_start_motor_ramp(motor);
	}

	return count;
}

static ssize_t duty_cycle_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct dc_motor_device *motor = to_dc_motor_device(dev);
	int duty_cycle;

	duty_cycle = motor->ops->get_duty_cycle(motor->context);

	if (motor->ops->get_command(motor->context) == DC_MOTOR_INTERNAL_COMMAND_RUN_REVERSE)
		duty_cycle *= -1;
	if (motor->active_params.polarity == DC_MOTOR_POLARITY_INVERSED)
		duty_cycle *= -1;

	return sprintf(buf, "%d\n", duty_cycle);
}

static unsigned get_supported_commands(struct dc_motor_device *motor)
{
	unsigned supported_commands;

	supported_commands = motor->ops->get_supported_commands(motor->context);

	if ((supported_commands & BIT(DC_MOTOR_COMMAND_RUN_FOREVER))
		&& (supported_commands & BIT(DC_MOTOR_COMMAND_STOP)))
	{
		supported_commands |= BIT(DC_MOTOR_COMMAND_RUN_TIMED);
	}

	return supported_commands;
}

static ssize_t commands_show(struct device *dev, struct device_attribute *attr,
			     char *buf)
{
	struct dc_motor_device *motor = to_dc_motor_device(dev);
	unsigned supported_commands;
	int i;
	int count = 0;

	supported_commands = get_supported_commands(motor);
	for (i = 0; i < NUM_DC_MOTOR_COMMANDS; i++) {
		if (supported_commands & BIT(i))
			count += sprintf(buf + count, "%s ",
				dc_motor_command_names[i]);
	}

	if (count == 0)
		count = 1;
	buf[count - 1] = '\n';

	return count;
}

static ssize_t command_store(struct device *dev, struct device_attribute *attr,
			     const char *buf, size_t count)
{
	struct dc_motor_device *motor = to_dc_motor_device(dev);
	unsigned supported_commands;
	int i;

	supported_commands = get_supported_commands(motor);
	for (i = 0; i < NUM_DC_MOTOR_COMMANDS; i++) {
		if (!sysfs_streq(buf, dc_motor_command_names[i]))
			continue;
		if (supported_commands & BIT(i)) {
			motor->active_params = motor->params;
			if (!IS_DC_MOTOR_RUN_COMMAND(i))
				motor->active_params.duty_cycle_sp = 0;
			else if (motor->active_params.polarity == DC_MOTOR_POLARITY_INVERSED)
				motor->active_params.duty_cycle_sp *= -1;
			motor->command = i;
			dc_motor_class_start_motor_ramp(motor);
			cancel_delayed_work_sync(&motor->run_timed_work);
			if (motor->command == DC_MOTOR_COMMAND_RUN_TIMED)
				schedule_delayed_work(&motor->run_timed_work,
					msecs_to_jiffies(motor->active_params.time_sp));
			return count;
		}
	}
	return -EINVAL;
}

static ssize_t stop_actions_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct dc_motor_device *motor = to_dc_motor_device(dev);
	unsigned supported_commands;
	int i;
	int count = 0;

	supported_commands = motor->ops->get_supported_stop_actions(motor->context);
	for (i = 0; i < NUM_DC_MOTOR_STOP_ACTIONS; i++) {
		if (supported_commands & BIT(i))
			count += sprintf(buf + count, "%s ",
				dc_motor_stop_action_names[i]);
	}

	if (count == 0)
		count = 1;
	buf[count - 1] = '\n';

	return count;
}

static ssize_t stop_action_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	struct dc_motor_device *motor = to_dc_motor_device(dev);
	unsigned supported_commands;
	int i;

	supported_commands = motor->ops->get_supported_stop_actions(motor->context);
	for (i = 0; i < NUM_DC_MOTOR_STOP_ACTIONS; i++) {
		if (!sysfs_streq(buf, dc_motor_stop_action_names[i]))
			continue;
		if (supported_commands & BIT(i)) {
			motor->params.stop_action = i;
			return count;
		}

	}
	return -EINVAL;
}

static ssize_t state_show(struct device *dev, struct device_attribute *attr,
			  char *buf)
{
	struct dc_motor_device *motor = to_dc_motor_device(dev);
	enum dc_motor_internal_command command;
	unsigned flags = 0;
	int i;
	size_t size = 0;

	command = motor->ops->get_command(motor->context);
	if (IS_DC_MOTOR_INTERNAL_RUN_COMMAND(command))
		flags |= BIT(DC_MOTOR_STATE_RUNNING);

	if (delayed_work_pending(&motor->ramp_work))
		flags |= BIT(DC_MOTOR_STATE_RAMPING);

	for (i = 0; i < NUM_DC_MOTOR_STATE; i++) {
		if (flags & BIT(i))
			size += sprintf(buf + size, "%s ", dc_motor_state_names[i]);
	}

	if (!size)
		size = 1;

	buf[size - 1] = '\n';

	return size;
}

static ssize_t time_sp_show(struct device *dev, struct device_attribute *attr,
			    char *buf)
{
	struct dc_motor_device *motor = to_dc_motor_device(dev);

	return sprintf(buf, "%d\n", motor->params.time_sp);
}

static ssize_t time_sp_store(struct device *dev, struct device_attribute *attr,
			     const char *buf, size_t count)
{
	struct dc_motor_device *motor = to_dc_motor_device(dev);
	int err, time;

	err = kstrtoint(buf, 10, &time);
	if (err < 0)
		return err;

	if (time < 0 || time > 60000)
		return -EINVAL;

	motor->params.time_sp = time;
	return count;
}


static DEVICE_ATTR_RO(driver_name);
static DEVICE_ATTR_RO(address);
static DEVICE_ATTR_RW(ramp_up_sp);
static DEVICE_ATTR_RW(ramp_down_sp);
static DEVICE_ATTR_RW(polarity);
static DEVICE_ATTR_RW(duty_cycle_sp);
static DEVICE_ATTR_RO(duty_cycle);
static DEVICE_ATTR_RO(commands);
static DEVICE_ATTR_WO(command);
static DEVICE_ATTR_RO(stop_actions);
static DEVICE_ATTR_WO(stop_action);
static DEVICE_ATTR_RO(state);
static DEVICE_ATTR_RW(time_sp);

static struct attribute *dc_motor_class_attrs[] = {
	&dev_attr_driver_name.attr,
	&dev_attr_address.attr,
	&dev_attr_ramp_up_sp.attr,
	&dev_attr_ramp_down_sp.attr,
	&dev_attr_polarity.attr,
	&dev_attr_duty_cycle_sp.attr,
	&dev_attr_duty_cycle.attr,
	&dev_attr_commands.attr,
	&dev_attr_command.attr,
	&dev_attr_stop_actions.attr,
	&dev_attr_stop_action.attr,
	&dev_attr_state.attr,
	&dev_attr_time_sp.attr,
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

	if (!dc || !dc->address || !parent)
		return -EINVAL;

	dc->dev.release = dc_motor_release;
	dc->dev.parent = parent;
	dc->dev.class = &dc_motor_class;
	dev_set_name(&dc->dev, "motor%d", dc_motor_class_id++);

	INIT_DELAYED_WORK(&dc->ramp_work, dc_motor_class_ramp_work);
	INIT_DELAYED_WORK(&dc->run_timed_work, dc_motor_class_run_timed_work);

	err = device_register(&dc->dev);
	if (err)
		return err;

	dev_info(&dc->dev, "Registered '%s' on '%s'.\n", dc->name, dc->address);

	return 0;
}
EXPORT_SYMBOL_GPL(register_dc_motor);

void unregister_dc_motor(struct dc_motor_device *dc)
{
	dev_info(&dc->dev, "Unregistered '%s' on '%s'.\n", dc->name, dc->address);
	cancel_delayed_work_sync(&dc->run_timed_work);
	cancel_delayed_work_sync(&dc->ramp_work);
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
	add_uevent_var(env, "LEGO_ADDRESS=%s", motor->address);
	if (ret) {
		dev_err(dev, "failed to add uevent LEGO_ADDRESS\n");
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
