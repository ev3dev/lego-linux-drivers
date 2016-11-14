/*
 * Tacho motor helpers
 *
 * Copyright (C) 2016 David Lechner <david@lechnology.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.

 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/export.h>

#include <dc_motor_class.h>
#include <tacho_motor_helper.h>

/*
 * Speed helper:
 *
 * Performs averaging of speed over a time period. This is not the most
 * accurate - especially at low speeds, but is the best we can do with motor
 * controllers that just report position.
 */

/**
 * tm_speed_update - update the speed based on new position and time
 *
 * @spd: Pointer to the speed helper.
 * @pos: The new position.
 * @t: The timestamp of the new position.
 */
void tm_speed_update(struct tm_speed *spd, int pos, ktime_t t)
{
	s64 ds = USEC_PER_SEC * (pos - spd->pos[spd->tail]);
	ktime_t dt = ktime_sub(t, spd->time[spd->tail]);

	spd->tail++;
	spd->tail &= BUFFER_SIZE - 1;

	spd->speed = div64_s64(ds, ktime_to_us(dt));

	spd->head++;
	spd->head &= BUFFER_SIZE - 1;
	spd->pos[spd->head] = pos;
	spd->time[spd->head] = t;
}
EXPORT_SYMBOL_GPL(tm_speed_update);

/**
 * tm_speed_init - initialize the speed helper
 *
 * @spd: Pointer to the speed helper.
 * @pos: The current position.
 * @t: The timestamp of the current position.
 * @count: The calculation period in terms of # calls to tm_speed_update.
 */
void tm_speed_init(struct tm_speed *spd, int pos, ktime_t t, int count)
{
	int i;

	BUG_ON(count >= BUFFER_SIZE);
	for (i = 0; i < BUFFER_SIZE; i++) {
		spd->pos[i] = pos;
		spd->time[i] = t;
	}
	spd->speed = 0;
	spd->head = 0;
	spd->tail = BUFFER_SIZE - count;
}
EXPORT_SYMBOL_GPL(tm_speed_init);


/*
 * PID helper:
 *
 * This is a generic PID controller for use with motor controllers that do not
 * have speed or position regulation implemented in hardware.
 */

/**
 * tm_pid_update - run one iteration of a PID
 *
 * @pid: Pointer to the private data.
 * @value: The current input (process) value.
 */
int tm_pid_update(struct tm_pid *pid, int value)
{
	int duty_cycle, error, delta_error;

	/* Discrete PID calculations */

	error = pid->setpoint - value;
	pid->integral += error;
	delta_error = pid->prev_error - error;
	pid->prev_error = error;

	duty_cycle = ((error * pid->Kp) + (pid->integral * pid->Ki)
		      + (delta_error * pid->Kd)) / 10000;

	/*
	 * Subtract the value error to avoid integral windup if the resulting
	 * duty_cycle is more than 100%
	 */
	pid->overloaded = abs(duty_cycle) > DC_MOTOR_MAX_DUTY_CYCLE;
	if (pid->overloaded)
		pid->integral -= error;

	duty_cycle = min(duty_cycle, DC_MOTOR_MAX_DUTY_CYCLE);
	duty_cycle = max(duty_cycle, -DC_MOTOR_MAX_DUTY_CYCLE);

	return duty_cycle;
}
EXPORT_SYMBOL_GPL(tm_pid_update);

/**
 * tm_pid_reinit - reset everything except for the PID constants
 *
 * @pid: Pointer to the private data struct.
 */
void tm_pid_reinit(struct tm_pid *pid)
{
	pid->setpoint = 0;
	pid->integral = 0;
	pid->prev_error = 0;
	pid->overloaded = false;
}
EXPORT_SYMBOL_GPL(tm_pid_reinit);

/**
 * tm_pid_init - initialize/reset tm_pid struct.
 *
 * @pid: Pointer to the private data struct.
 * @Kp: PID proportional gain constant.
 * @Ki: PID proportional gain constant.
 * @Kd: PID proportional gain constant.
 */
void tm_pid_init(struct tm_pid *pid, int Kp, int Ki, int Kd)
{
	tm_pid_reinit(pid);
	pid->Kp = Kp;
	pid->Ki = Ki;
	pid->Kd = Kd;
}
EXPORT_SYMBOL_GPL(tm_pid_init);
