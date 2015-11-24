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
* feedback allows for precise control of the motors.
* .
* ### sysfs Attributes
* .
* Tacho motors can be found at `/sys/class/tacho-motor/motor<N>`, where `<N>`
* is incremented each time a motor is loaded (it is not related to which port
* the motor is plugged in to).
* .
* `command` (write-only)
* : Sends a command to the motor controller. See `commands` for a list of
*   possible values.
* .
* `commands` (read-only)
* : Returns a space separated list of commands that are supported by the motor
*   controller. Possible values are `run-forever`, `run-to-abs-pos`, `run-to-rel-pos`,
*   `run-timed`, `run-direct`, `stop` and `reset`. Not all commands may be supported.
*   `run-forever` will cause the motor to run until another command is sent.
*   `run-to-abs-pos` will run to an absolute position specified by `position_sp`
*   and then stop using the command specified in `stop_command`.
*   `run-to-rel-pos` will run to a position relative to the current `position` value.
*   The new position will be current `position` + `position_sp`. When the new
*   position is reached, the motor will stop using the command specified by `stop_command`.
*   `run-timed` will run the motor for the amount of time specified in `time_sp`
*   and then stop the motor using the command specified by `stop_command`.
*   `run-direct` will run the motor at the duty cycle specified by `duty_cycle_sp`.
*   Unlike other run commands, changing `duty_cycle_sp` while running *will*
*   take effect immediately.
*   `stop` will stop any of the run commands before they are complete using the
*   command specified by `stop_command`.
*   `reset` will reset all of the motor parameter attributes to their default value.
*   This will also have the effect of stopping the motor.
* .
* `count_per_rot` (read-only)
* : Returns the number of tacho counts in one rotation of the motor. Tacho counts
*   are used by the position and speed attributes, so you can use this value
*   to convert rotations or degrees to tacho counts. In the case of linear
*   actuators, the units here will be counts per centimeter.
* .
* `driver_name` (read-only)
* : Returns the name of the driver that provides this tacho motor device.
* .
* `duty_cycle` (read-only)
* : Returns the current duty cycle of the motor. Units are percent. Values
*   are -100 to 100.
* .
* `duty_cycle_sp` (read/write)
* : Writing sets the duty cycle setpoint. Reading returns the current value.
*   Units are in percent. Valid values are -100 to 100. A negative value causes
*   the motor to rotate in reverse. This value is only used when `speed_regulation`
*   is off.
* .
* `encoder_polarity` (read/write)
* : Sets the polarity of the rotary encoder. This is an advanced feature to all
*   use of motors that send inversed encoder signals to the EV3. This should
*   be set correctly by the driver of a device. It You only need to change this
*   value if you are using a unsupported device. Valid values are `normal` and
*   `inversed`.
* .
* `polarity` (read/write)
* : Sets the polarity of the motor. With `normal` polarity, a positive duty
*   cycle will cause the motor to rotate clockwise. With `inversed` polarity,
*   a positive duty cycle will cause the motor to rotate counter-clockwise.
*   Valid values are `normal` and `inversed`.
* .
* `address` (read-only)
* : Returns the name of the port that the motor is connected to.
* .
* `position` (read/write)
* : Returns the current position of the motor in pulses of the rotary
*   encoder. When the motor rotates clockwise, the position will increase.
*   Likewise, rotating counter-clockwise causes the position to decrease.
*   Writing will set the position to that value.
* .
* `hold_pid/Kd`: (read/write)
* : The derivative constant for the position PID.
* .
* `hold_pid/Ki`: (read/write)
* : The integral constant for the position PID.
* .
* `hold_pid/Kp`: (read/write)
* : The proportional constant for the position PID.
* .
* `position_sp` (read/write)
* : Writing specifies the target position for the `run-to-abs-pos` and `run-to-rel-pos`
*   commands. Reading returns the current value. Units are in tacho counts. You
*   can use the value returned by `counts_per_rot` to convert tacho counts to/from
*   rotations or degrees.
* .
* `speed` (read-only)
* : Returns the current motor speed in tacho counts per second. Not, this is
*   not necessarily degrees (although it is for LEGO motors). Use the `count_per_rot`
*   attribute to convert this value to RPM or deg/sec.
* .
* `speed_sp` (read/write)
* : Writing sets the target speed in tacho counts per second used when `speed_regulation`
*   is on. Reading returns the current value.  Use the `count_per_rot` attribute
*   to convert RPM or deg/sec to tacho counts per second.
* .
* `ramp_up_sp` (read/write)
* : Writing sets the ramp up setpoint. Reading returns the current value. Units
*   are in milliseconds. When set to a value > 0, the motor will ramp the power
*   sent to the motor from 0 to 100% duty cycle over the span of this setpoint
*   when starting the motor. If the maximum duty cycle is limited by `duty_cycle_sp`
*   or speed regulation, the actual ramp time duration will be less than the setpoint.
* .
* `ramp_down_sp` (read/write)
* : Writing sets the ramp down setpoint. Reading returns the current value. Units
*   are in milliseconds. When set to a value > 0, the motor will ramp the power
*   sent to the motor from 100% duty cycle down to 0 over the span of this setpoint
*   when stopping the motor. If the starting duty cycle is less than 100%, the
*   ramp time duration will be less than the full span of the setpoint.
* .
* `speed_regulation` (read/write)
* : Turns speed regulation on or off. If speed regulation is on, the motor
*   controller will vary the power supplied to the motor to try to maintain the
*   speed specified in `speed_sp`. If speed regulation is off, the controller
*   will use the power specified in `duty_cycle_sp`. Valid values are `on` and
*   `off`.
* .
* `speed_pid/Kd`: (read/write)
* : The derivative constant for the speed regulation PID.
* .
* `speed_pid/Ki`: (read/write)
* : The integral constant for the speed regulation PID.
* .
* `speed_pid/Kp`: (read/write)
* : The proportional constant for the speed regulation PID.
* .
* `state` (read-only)
* : Reading returns a space separated list of state flags. Possible flags are
*   `running`, `ramping` `holding` and `stalled`.
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
* : Writing specifies the amount of time the motor will run when using the
*   `run-timed` command. Reading returns the current value. Units are in
*   milliseconds.
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

static struct tacho_motor_value_names tacho_motor_command_names[] = {
	[TM_COMMAND_RUN_FOREVER]	= { "run-forever" },
	[TM_COMMAND_RUN_TO_ABS_POS]	= { "run-to-abs-pos" },
	[TM_COMMAND_RUN_TO_REL_POS]	= { "run-to-rel-pos" },
	[TM_COMMAND_RUN_TIMED]		= { "run-timed" },
	[TM_COMMAND_RUN_DIRECT]		= { "run-direct" },
	[TM_COMMAND_STOP] 		= { "stop" },
	[TM_COMMAND_RESET]		= { "reset" },
};

struct tacho_motor_type_item {
	const char *name;
};

struct tacho_motor_state_item {
	const char *name;
};

static struct tacho_motor_value_names tacho_motor_states[NUM_TM_STATES] = {
	[TM_STATE_RUNNING]	= { "running" },
	[TM_STATE_RAMPING]	= { "ramping" },
	[TM_STATE_HOLDING]	= { "holding" },
	[TM_STATE_STALLED]	= { "stalled" },
};

inline struct tacho_motor_device *to_tacho_motor(struct device *dev)
{
	return container_of(dev, struct tacho_motor_device, dev);
}

/*
 * Set all parameters to the default values.
 */
void tacho_motor_class_reset(struct tacho_motor_device *tm)
{
	tm->params.polarity		= DC_MOTOR_POLARITY_NORMAL;
	tm->params.encoder_polarity	= DC_MOTOR_POLARITY_NORMAL;
	tm->params.duty_cycle_sp	= 0;
	tm->params.speed_sp		= 0;
	tm->params.position_sp		= 0;
	tm->params.ramp_up_sp		= 0;
	tm->params.ramp_down_sp		= 0;
	tm->params.speed_regulation	= TM_SPEED_REGULATION_OFF;
	tm->params.stop_command		= TM_STOP_COMMAND_COAST;
}

static ssize_t address_show(struct device *dev, struct device_attribute *attr,
			      char *buf)
{
	struct tacho_motor_device *tm = to_tacho_motor(dev);

	return snprintf(buf, LEGO_NAME_SIZE, "%s\n", tm->address);
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

	err = tm->ops->get_position(tm->context, &position);
	if (err < 0)
		return err;

	return sprintf(buf, "%ld\n", position);
}

ssize_t position_store(struct device *dev, struct device_attribute *attr,
		       const char *buf, size_t size)
{
	struct tacho_motor_device *tm = to_tacho_motor(dev);
	int err, position;

	err = kstrtoint(buf, 10, &position);
	if (err < 0)
		return err;

	err = tm->ops->set_position(tm->context, position);
	if (err < 0)
		return err;

	return size;
}

static ssize_t state_show(struct device *dev, struct device_attribute *attr,
			  char *buf)
{
	struct tacho_motor_device *tm = to_tacho_motor(dev);
	int ret, i;
	size_t size = 0;

	ret = tm->ops->get_state(tm->context);
	if (ret < 0)
		return ret;

	for (i = 0; i < NUM_TM_STATES; i++) {
		if (ret & BIT(i))
			size += sprintf(buf + size, "%s ", tacho_motor_states[i].name);
	}
	if (size == 0)
		size = sprintf(buf, "\n");
	else
		buf[size - 1] = '\n';

	return size;
}

static ssize_t count_per_rot_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct tacho_motor_device *tm = to_tacho_motor(dev);
	int ret;

	ret = tm->ops->get_count_per_rot(tm->context);
	if (ret < 0)
		return ret;

	return sprintf(buf, "%d\n", ret);
}

static ssize_t duty_cycle_show(struct device *dev, struct device_attribute *attr,
			       char *buf)
{
	struct tacho_motor_device *tm = to_tacho_motor(dev);
	int err, duty_cycle;

	if (!tm->ops->get_duty_cycle)
		return -EOPNOTSUPP;

	err = tm->ops->get_duty_cycle(tm->context, &duty_cycle);
	if (err < 0)
		return err;

	return sprintf(buf, "%d\n", duty_cycle);
}

static ssize_t speed_show(struct device *dev, struct device_attribute *attr,
			  char *buf)
{
	struct tacho_motor_device *tm = to_tacho_motor(dev);
	int err, speed;

	if (!tm->ops->get_speed)
		return -EOPNOTSUPP;

	err = tm->ops->get_speed(tm->context, &speed);
	if (err < 0)
		return err;

	return sprintf(buf, "%d\n", speed);
}
/*
 * Run timed command is implemented in the tacho motor class, so if an implementing
 * driver can run forever and stop, then it can run timed.
 */
static inline unsigned get_supported_commands(struct tacho_motor_device *tm)
{
	unsigned supported_commands = tm->ops->get_commands(tm->context);

	if ((supported_commands & BIT(TM_COMMAND_RUN_FOREVER))
				&& (supported_commands & BIT(TM_COMMAND_STOP)))
	{
		supported_commands |= BIT(TM_COMMAND_RUN_TIMED);
	}

	return supported_commands;
}

static ssize_t commands_show(struct device *dev, struct device_attribute *attr,
			     char *buf)
{
	struct tacho_motor_device *tm = to_tacho_motor(dev);
	unsigned supported_commands;
	int i;
	int size = 0;

	supported_commands = get_supported_commands(tm);
	for (i = 0; i < NUM_TM_COMMANDS; i++) {
		if (supported_commands & BIT(i))
			size += sprintf(buf+size, "%s ",
				tacho_motor_command_names[i].name);
	}

	buf[size - 1] = '\n';

	return size;
}

static ssize_t command_store(struct device *dev, struct device_attribute *attr,
			     const char *buf, size_t size)
{
	struct tacho_motor_device *tm = to_tacho_motor(dev);
	int i, err;
	bool start_timer = false;

	for (i = 0; i < NUM_TM_COMMANDS; i++) {
		if (sysfs_streq(buf, tacho_motor_command_names[i].name))
			break;
	}

	if (i >= NUM_TM_COMMANDS)
		return -EINVAL;

	if (!(BIT(i) & get_supported_commands(tm)))
		return -EINVAL;

	cancel_delayed_work_sync(&tm->run_timed_work);
	if (i == TM_COMMAND_RUN_TIMED) {
		i = TM_COMMAND_RUN_FOREVER;
		start_timer = true;
	}
	if (i == TM_COMMAND_RESET)
		tacho_motor_class_reset(tm);

	err = tm->ops->send_command(tm->context, &tm->params, i);
	if (err < 0)
		return err;

	if (start_timer)
		schedule_delayed_work(&tm->run_timed_work, msecs_to_jiffies(tm->time_sp));

	return size;
}

static void tacho_motor_class_run_timed_work(struct work_struct *work)
{
	struct tacho_motor_device *tm = container_of(to_delayed_work(work),
				struct tacho_motor_device, run_timed_work);

	tm->ops->send_command(tm->context, &tm->params, TM_COMMAND_STOP);
}

static ssize_t speed_regulation_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	struct tacho_motor_device *tm = to_tacho_motor(dev);

	return sprintf(buf, "%s\n",
		tacho_motor_speed_regulation_names[tm->params.speed_regulation].name);
}

static ssize_t speed_regulation_store(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t size)
{
	struct tacho_motor_device *tm = to_tacho_motor(dev);
	unsigned i;

	for (i = 0; i < TM_NUM_SPEED_REGULATION_MODES; i++) {
		if (sysfs_streq(buf, tacho_motor_speed_regulation_names[i].name))
			break;
	}

	if (i >= TM_NUM_SPEED_REGULATION_MODES)
		return -EINVAL;

	if (!(BIT(i) & tm->ops->get_speed_regulations(tm->context)))
		return -EINVAL;

	tm->params.speed_regulation = i;

	return size;
}

static ssize_t stop_commands_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct tacho_motor_device *tm = to_tacho_motor(dev);
	unsigned commands;
	int i;
	int size = 0;

	commands = tm->ops->get_stop_commands(tm->context);

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

	return sprintf(buf, "%s\n",
		tacho_motor_stop_command_names[tm->params.stop_command].name);
}

static ssize_t stop_command_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t size)
{
	struct tacho_motor_device *tm = to_tacho_motor(dev);
	unsigned i;

	for (i = 0; i < TM_NUM_STOP_COMMANDS; i++) {
		if (sysfs_streq(buf, tacho_motor_stop_command_names[i].name))
			break;
	}

	if (i >= TM_NUM_STOP_COMMANDS)
		return -EINVAL;

	if (!(BIT(i) & tm->ops->get_stop_commands(tm->context)))
		return -EINVAL;

	tm->params.stop_command = i;

	return size;
}

static ssize_t polarity_show(struct device *dev, struct device_attribute *attr,
			     char *buf)
{
	struct tacho_motor_device *tm = to_tacho_motor(dev);

	return sprintf(buf, "%s\n", dc_motor_polarity_values[tm->params.polarity]);
}

static ssize_t polarity_store(struct device *dev, struct device_attribute *attr,
			      const char *buf, size_t size)
{
	struct tacho_motor_device *tm = to_tacho_motor(dev);
	unsigned i;

	for (i = 0; i < NUM_DC_MOTOR_POLARITY; i++) {
		if (sysfs_streq(buf, dc_motor_polarity_values[i])) {
			tm->params.polarity = i;
			return size;
		}
	}

	return -EINVAL;
}

static ssize_t encoder_polarity_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	struct tacho_motor_device *tm = to_tacho_motor(dev);

	return sprintf(buf, "%s\n", dc_motor_polarity_values[tm->params.encoder_polarity]);
}

static ssize_t encoder_polarity_store(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t size)
{
	struct tacho_motor_device *tm = to_tacho_motor(dev);
	unsigned i;

	if (!tm->supports_encoder_polarity)
		return -EOPNOTSUPP;

	for (i = 0; i < NUM_DC_MOTOR_POLARITY; i++) {
		if (sysfs_streq(buf, dc_motor_polarity_values[i])) {
			tm->params.encoder_polarity = i;
			return size;
		}
	}

	return -EINVAL;
}

static ssize_t ramp_up_sp_show(struct device *dev, struct device_attribute *attr,
			       char *buf)
{
	struct tacho_motor_device *tm = to_tacho_motor(dev);

	if (!tm->supports_ramping)
		return -EOPNOTSUPP;

	return sprintf(buf, "%d\n", tm->params.ramp_up_sp);
}

static ssize_t ramp_up_sp_store(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct tacho_motor_device *tm = to_tacho_motor(dev);
	int err, ms;

	if (!tm->supports_ramping)
		return -EOPNOTSUPP;

	err = kstrtoint(buf, 10, &ms);
	if (err < 0)
		return err;

	if (ms < 0 || ms > 60000)
		return -EINVAL;

	tm->params.ramp_up_sp = ms;

	return size;
}

static ssize_t ramp_down_sp_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct tacho_motor_device *tm = to_tacho_motor(dev);

	if (!tm->supports_ramping)
		return -EOPNOTSUPP;

	return sprintf(buf, "%d\n", tm->params.ramp_down_sp);
}

static ssize_t ramp_down_sp_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t size)
{
	struct tacho_motor_device *tm = to_tacho_motor(dev);
	int err, ms;

	if (!tm->supports_ramping)
		return -EOPNOTSUPP;

	err = kstrtoint(buf, 10, &ms);
	if (err < 0)
		return err;

	if (ms < 0 || ms > 60000)
		return -EINVAL;

	tm->params.ramp_down_sp = ms;

	return size;
}

static ssize_t duty_cycle_sp_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct tacho_motor_device *tm = to_tacho_motor(dev);

	return sprintf(buf, "%d\n", tm->params.duty_cycle_sp);
}

static ssize_t duty_cycle_sp_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t size)
{
	struct tacho_motor_device *tm = to_tacho_motor(dev);
	int duty_cycle_sp, err;

	if (!(BIT(TM_SPEED_REGULATION_OFF) & tm->ops->get_speed_regulations(tm->context)))
		return -EOPNOTSUPP;

	err = kstrtoint(buf, 10, &duty_cycle_sp);
	if (err < 0)
		return err;

	if (duty_cycle_sp > 100 || duty_cycle_sp < -100)
		return -EINVAL;

	tm->params.duty_cycle_sp = duty_cycle_sp;

	return size;
}

static ssize_t speed_sp_show(struct device *dev, struct device_attribute *attr,
			     char *buf)
{
	struct tacho_motor_device *tm = to_tacho_motor(dev);

	return sprintf(buf, "%d\n", tm->params.speed_sp);
}

static ssize_t speed_sp_store(struct device *dev, struct device_attribute *attr,
			      const char *buf, size_t size)
{
	struct tacho_motor_device *tm = to_tacho_motor(dev);
	int err, speed;

	if (!(BIT(TM_SPEED_REGULATION_ON) & tm->ops->get_speed_regulations(tm->context)))
		return -EOPNOTSUPP;

	err = kstrtoint(buf, 10, &speed);
	if (err < 0)
		return err;

	if (speed > 2000 || speed < -2000)
		return -EINVAL;

	tm->params.speed_sp = speed;

	return size;
}

static ssize_t time_sp_show(struct device *dev, struct device_attribute *attr,
			    char *buf)
{
	struct tacho_motor_device *tm = to_tacho_motor(dev);

	return sprintf(buf, "%d\n", tm->time_sp);
}

static ssize_t time_sp_store(struct device *dev, struct device_attribute *attr,
			     const char *buf, size_t size)
{
	struct tacho_motor_device *tm = to_tacho_motor(dev);
	int err, time;

	if (!(BIT(TM_COMMAND_RUN_TIMED) & get_supported_commands(tm)))
		return -EOPNOTSUPP;

	err = kstrtoint(buf, 10, &time);
	if (err < 0)
		return err;

	if (time < 0)
		return -EINVAL;

	tm->time_sp = time;

	return size;
}

static ssize_t position_sp_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct tacho_motor_device *tm = to_tacho_motor(dev);

	return sprintf(buf, "%d\n", tm->params.position_sp);
}

static ssize_t position_sp_store(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t size)
{
	struct tacho_motor_device *tm = to_tacho_motor(dev);
	int err;
	long position;

	if (!((BIT(TM_COMMAND_RUN_TO_ABS_POS) | BIT(TM_COMMAND_RUN_TO_REL_POS))
						& get_supported_commands(tm)))
	{
		return -EOPNOTSUPP;
	}

	err = kstrtol(buf, 10, &position);
	if (err < 0)
		return err;

	tm->params.position_sp = position;

	return size;
}

static DEVICE_ATTR_RO(driver_name);
static DEVICE_ATTR_RO(address);
static DEVICE_ATTR_RW(position);
static DEVICE_ATTR_RO(state);
static DEVICE_ATTR_RO(count_per_rot);
static DEVICE_ATTR_RO(duty_cycle);
static DEVICE_ATTR_RO(speed);
static DEVICE_ATTR_RW(duty_cycle_sp);
static DEVICE_ATTR_RW(speed_sp);
static DEVICE_ATTR_RW(time_sp);
static DEVICE_ATTR_RW(position_sp);
static DEVICE_ATTR_RO(commands);
static DEVICE_ATTR_WO(command);
static DEVICE_ATTR_RW(speed_regulation);
static DEVICE_ATTR_RO(stop_commands);
static DEVICE_ATTR_RW(stop_command);
static DEVICE_ATTR_RW(polarity);
static DEVICE_ATTR_RW(encoder_polarity);
static DEVICE_ATTR_RW(ramp_up_sp);
static DEVICE_ATTR_RW(ramp_down_sp);

static struct attribute *tacho_motor_class_attrs[] = {
	&dev_attr_driver_name.attr,
	&dev_attr_address.attr,
	&dev_attr_position.attr,
	&dev_attr_state.attr,
	&dev_attr_count_per_rot.attr,
	&dev_attr_duty_cycle.attr,
	&dev_attr_speed.attr,
	&dev_attr_duty_cycle_sp.attr,
	&dev_attr_speed_sp.attr,
	&dev_attr_time_sp.attr,
	&dev_attr_position_sp.attr,
	&dev_attr_commands.attr,
	&dev_attr_command.attr,
	&dev_attr_speed_regulation.attr,
	&dev_attr_stop_commands.attr,
	&dev_attr_stop_command.attr,
	&dev_attr_polarity.attr,
	&dev_attr_encoder_polarity.attr,
	&dev_attr_ramp_up_sp.attr,
	&dev_attr_ramp_down_sp.attr,
	NULL
};

static const struct attribute_group tacho_motor_class_group = {
	.attrs = tacho_motor_class_attrs,
};

#define PID_ATTR_FUNCS(name)							\
static ssize_t name##_show(struct device *dev, struct device_attribute *attr,	\
			   char *buf)						\
{										\
	struct tacho_motor_device *tm = to_tacho_motor(dev);			\
	int ret;								\
										\
	if (!tm->ops->get_##name)						\
		return -EOPNOTSUPP;						\
										\
	ret = tm->ops->get_##name(tm->context);					\
	if (ret < 0)								\
		return ret;							\
										\
	return sprintf(buf, "%d\n", ret);					\
}										\
										\
static ssize_t name##_store(struct device *dev, struct device_attribute *attr,	\
			    const char *buf, size_t size)			\
{										\
	struct tacho_motor_device *tm = to_tacho_motor(dev);			\
	int err, k;								\
										\
	if (!tm->ops->set_##name)						\
		return -EOPNOTSUPP;						\
										\
	err = kstrtoint(buf, 10, &k);						\
	if (err < 0)								\
		return err;							\
										\
	if (k < 0)								\
		return -EINVAL;							\
										\
	err = tm->ops->set_##name(tm->context, k);				\
	if (err < 0)								\
		return err;							\
										\
	return size;								\
}

#define PID_ATTR(pid, k) struct device_attribute dev_attr_##pid##_##k = \
	__ATTR(k, S_IWUSR | S_IRUGO, pid##_##k##_show, pid##_##k##_store)

#define PID_ATTR_GROUP(pid)							\
PID_ATTR_FUNCS(pid##_Kp)							\
PID_ATTR_FUNCS(pid##_Ki)							\
PID_ATTR_FUNCS(pid##_Kd)							\
										\
PID_ATTR(pid, Kp);								\
PID_ATTR(pid, Ki);								\
PID_ATTR(pid, Kd);								\
										\
static struct attribute *tacho_motor_##pid##_pid_attrs[] = {			\
	&dev_attr_##pid##_Kp.attr,						\
	&dev_attr_##pid##_Ki.attr,						\
	&dev_attr_##pid##_Kd.attr,						\
	NULL									\
};										\
										\
static const struct attribute_group tacho_motor_##pid##_pid_group = {		\
	.name = __stringify(pid) "_pid",					\
	.attrs = tacho_motor_##pid##_pid_attrs,					\
}

PID_ATTR_GROUP(speed);
PID_ATTR_GROUP(hold);

static const struct attribute_group *tacho_motor_class_groups[] = {
	&tacho_motor_class_group,
	&tacho_motor_speed_pid_group,
	&tacho_motor_hold_pid_group,
	NULL
};

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

	if (!tm || !tm->address || !parent)
		return -EINVAL;
	
	tm->dev.release = tacho_motor_release;
	tm->dev.parent = parent;
	tm->dev.class = &tacho_motor_class;
	dev_set_name(&tm->dev, "motor%d", tacho_motor_class_id++);
	tacho_motor_class_reset(tm);
	INIT_DELAYED_WORK(&tm->run_timed_work, tacho_motor_class_run_timed_work);

	err = device_register(&tm->dev);
	if (err)
		return err;

	dev_info(&tm->dev, "Registered '%s' on '%s'.\n", tm->driver_name,
		 tm->address);

	return 0;
}
EXPORT_SYMBOL_GPL(register_tacho_motor);

void unregister_tacho_motor(struct tacho_motor_device *tm)
{
	cancel_delayed_work_sync(&tm->run_timed_work);
	dev_info(&tm->dev, "Unregistered '%s' on '%s'.\n", tm->driver_name,
		 tm->address);
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

	ret = add_uevent_var(env, "LEGO_ADDRESS=%s", tm->address);
	if (ret) {
		dev_err(dev, "failed to add uevent LEGO_ADDRESS\n");
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
