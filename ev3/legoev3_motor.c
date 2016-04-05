/*
 * Motor driver for LEGO MINDSTORMS EV3
 *
 * Copyright (C) 2016 Ralph Hempel <rhempel@hempeldesigngroup.com>
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
 * EV3/NXT Tacho Motor Driver
 *
 * This driver provides a [tacho-motor] interface for EV3 and NXT motors or any
 * other compatible motor with an [incremental rotary encoder] for position
 * and direction feedback that is connected to an output port. We call them
 * "tacho" motors because that is what the LMS2012 source code calls them. You
 * can find the devices bound to this driver in the directory
 * `/sys/bus/lego/drivers/legoev3-motor /`. There is not much of interest
 * there though - all of the useful stuff is in the [tacho-motor] class.
 * .
 * [tacho-motor]: ../tacho-motor-class
 * [incremental rotary encoder]: https://en.wikipedia.org/wiki/Rotary_encoder#Incremental_rotary_encoder
 */

#include <linux/device.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/math64.h>
#include <linux/random.h>

#include <lego.h>
#include <lego_port_class.h>
#include <dc_motor_class.h>
#include <tacho_motor_class.h>
#include <tacho_motor_helper.h>

#include "legoev3_motor.h"
#include "../motors/ev3_motor.h"

#define TACHO_MOTOR_POLL_MS	2	/* 2 msec */

#define TACHO_SAMPLES		128

#define MAX_SYNC_MOTORS		2

enum legoev3_motor_command {
	UNKNOWN,
	FORWARD,
	REVERSE,
	BRAKE,
	COAST,
};

enum legoev3_motor_state {
	STATE_RUNNING,
	STATE_STOPPED,
	NUM_STATES,
};

struct legoev3_motor_data {
	struct tacho_motor_device tm;
	struct lego_device *ldev;

	struct hrtimer timer;
	struct work_struct notify_state_change_work;
	struct work_struct notify_position_ramp_down_work;
	struct tm_pid speed_pid;
	struct tm_pid hold_pid;

	ktime_t tacho_samples[TACHO_SAMPLES];
	unsigned tacho_samples_head;

	bool got_new_sample;

	int num_samples;
	int dir_chg_samples;

	int max_us_per_sample;

	int position_sp;

	enum legoev3_motor_command run_direction;
	bool overloaded;
	bool stalled;
	bool ramping;

	int position;
	int speed;
	int duty_cycle;
	enum legoev3_motor_state state;

	enum tacho_motor_command run_command;
	bool speed_pid_ena;
	bool hold_pid_ena;
};

static void set_num_samples_for_speed(struct legoev3_motor_data *ev3_tm,
				      int speed)
{
	const int *samples_for_speed =
			ev3_tm->tm.info->legoev3_info.samples_for_speed;

	if (speed > 80)
		ev3_tm->num_samples = samples_for_speed[SPEED_ABOVE_80];
	else if (speed > 60)
		ev3_tm->num_samples = samples_for_speed[SPEED_ABOVE_60];
	else if (speed > 40)
		ev3_tm->num_samples = samples_for_speed[SPEED_ABOVE_40];
	else
		ev3_tm->num_samples = samples_for_speed[SPEED_BELOW_40];
}

/*
 * Handling the Tachometer Inputs
 *
 * The tacho motor driver uses two pins on each port to determine the direction
 * of rotation of the motor.
 *
 * `pdata->tacho_int_gpio` is the pin that is set up to trigger an interrupt
 * any edge change
 *
 * `pdata->tacho_dir_gpio` is the pin that helps to determine the direction
 * of rotation
 *
 * When int == dir then the encoder is turning in the forward direction
 * When int != dir then the encoder is turning in the reverse direction
 *
 * -----     --------           --------      -----
 *     |     |      |           |      |      |
 *     |     |      |           |      |      |
 *     -------      -------------       -------          DIRx signal
 *
 *  -------     --------     --------     --------       INTx signal
 *        |     |      |     |      |     |      |
 *        |     |      |     |      |     |      |
 *        -------      -------      -------      -----
 *        \     \      \     \      \     \      \
 *         ^     ^      ^     ^      ^     ^      ^      ISR handler
 *         +1    +1     +1    -1     -1    -1     -1     TachoCount
 *
 * All this works perfectly well when there are no missed interrupts, and when
 * the transitions on these pins are clean (no bounce or noise). It is possible
 * to get noisy operation when the transitions are very slow, and we have
 * observed signals similar to this:
 *
 * -------------                       -------------
 *             |                       |
 *             |                       |
 *             -------------------------                 DIRx signal
 *
 *    ---------------   ----                             INTx signal
 *    |             |   |  |
 *    |             |   |  |
 * ----             -----  -------------------------
 *    \              \   \  \
 *     ^              ^   ^  ^                           ISR Handler
 *     +1             +1  -1 +1                          TachoCount
 *                    A   B  C
 *
 * The example above has three transitions that we are interested in
 * labeled A, B, and C - they represent a noisy signal. As long as
 * all three transitions are caught by the ISR, then the count is
 * incremented by 2 as expected. But there are other outcomes possible.
 *
 * For example, if the A transition is handled, but the INT signal
 * is not measured until after B, then the final count value is 1.
 *
 * On the other hand, if the B transition is missed, and only A and
 * C are handled, then the final count value is 3.
 *
 * Either way, we need to figure out a way to clean things up, and as
 * long as at least two of the interrupts are caught, we can "undo"
 * a reading quite easily.
 *
 * The medium motor turns at a maximum of 1200 pulses per second, the
 * large tacho motor has a maximum speed of 900 pulses per second. Taking
 * the highest value, this means that about 800 usec is the fastest time
 * between interrupts. If we see two interrupts with a delta of much less
 * than, say 400 usec, then we're probably looking at a noisy transition.
 *
 * In most cases that have been captured, the shortest delta is the A-B
 * transition, anywhere from 10 to 20 usec, which is faster than the ISR
 * response time. The B-C transition has been measured up to 150 usec.
 *
 * It is clear that the correct transition to use for changing the
 * value of `TachoCount` is C - so if the delta from A-C is less than
 * the threshold, we should "undo" whatever the A transition told us.
 */

static irqreturn_t tacho_motor_isr(int irq, void *id)
{
	struct legoev3_motor_data *ev3_tm = id;
	struct ev3_motor_platform_data *pdata = ev3_tm->ldev->dev.platform_data;
	bool int_state = gpio_get_value(pdata->tacho_int_gpio);
	bool dir_state = !gpio_get_value(pdata->tacho_dir_gpio);
	ktime_t timer = ktime_get();
	ktime_t prev_timer = ev3_tm->tacho_samples[ev3_tm->tacho_samples_head];
	unsigned next_sample = (ev3_tm->tacho_samples_head + 1) % TACHO_SAMPLES;
	enum legoev3_motor_command next_direction = ev3_tm->run_direction;

	/*
	 * If the difference in timestamps is too small, then undo the
	 * previous increment - it's OK for a count to waver once in
	 * a while - better than being wrong!
	 *
	 * Here's what we'll do when the transition is too small:
	 *
	 * 1) UNDO the increment to the next timer sample update
	 *    dir_chg_samples count!
	 * 2) UNDO the previous run_direction count update
	 */
	if (ktime_to_us(ktime_sub(timer, prev_timer)) < 400) {
		ev3_tm->tacho_samples[ev3_tm->tacho_samples_head] = timer;

		if (FORWARD == ev3_tm->run_direction)
			ev3_tm->position--;
		else
			ev3_tm->position++;

		next_sample = ev3_tm->tacho_samples_head;
	} else {
		/*
		 * Update the tacho count and motor direction for low
		 * speed, taking advantage of the fact that if state and
		 * dir match, then the motor is turning FORWARD!
		 *
		 * We also look after the polarity_mode and encoder_mode
		 * here as follows:
		 *
		 * polarity_mode | encoder_mode | next_direction
		 * --------------+--------------+---------------
		 * normal        | normal       | normal
		 * normal        | inverted     | inverted
		 * inverted      | normal       | inverted
		 * inverted      | inverted     | normal
		 *
		 * Yes, this could be compressed into a clever set of
		 * conditionals that results in only two assignments, or
		 * a lookup table, but it's clearer to write nested if
		 * statements in this case - it looks a lot more like the
		 * truth table
		 */
		if (ev3_tm->tm.active_params.polarity == DC_MOTOR_POLARITY_NORMAL) {
			if (ev3_tm->tm.active_params.encoder_polarity == DC_MOTOR_POLARITY_NORMAL)
				next_direction = (int_state == dir_state) ? FORWARD : REVERSE;
			else
				next_direction = (int_state == dir_state) ? REVERSE : FORWARD;
		} else {
			if (ev3_tm->tm.active_params.encoder_polarity == DC_MOTOR_POLARITY_NORMAL)
				next_direction = (int_state == dir_state) ? REVERSE : FORWARD;
			else
				next_direction = (int_state == dir_state) ? FORWARD : REVERSE;
		}

		/*
		 * If the saved and next direction states
		 * match, then update the dir_chg_sample count
		 */
		if (ev3_tm->run_direction == next_direction) {
			if (ev3_tm->dir_chg_samples < (TACHO_SAMPLES-1))
				ev3_tm->dir_chg_samples++;
		} else {
			ev3_tm->dir_chg_samples = 0;
		}
	}

	ev3_tm->run_direction = next_direction;

	/* Grab the next incremental sample timestamp */

	ev3_tm->tacho_samples[next_sample] = timer;
	ev3_tm->tacho_samples_head = next_sample;

	if (FORWARD == ev3_tm->run_direction)
		ev3_tm->position++;
	else
		ev3_tm->position--;

	ev3_tm->got_new_sample = true;

	return IRQ_HANDLED;
}

static void set_duty_cycle(struct legoev3_motor_data *ev3_tm, int duty_cycle)
{
	const struct dc_motor_ops *motor_ops = ev3_tm->ldev->port->dc_motor_ops;
	void *context = ev3_tm->ldev->port->context;
	int err;

	if (duty_cycle > DC_MOTOR_MAX_DUTY_CYCLE)
		ev3_tm->duty_cycle = DC_MOTOR_MAX_DUTY_CYCLE;
	else if (duty_cycle < -DC_MOTOR_MAX_DUTY_CYCLE)
		ev3_tm->duty_cycle = -DC_MOTOR_MAX_DUTY_CYCLE;
	else
		ev3_tm->duty_cycle = duty_cycle;

	if (ev3_tm->duty_cycle > 0)
			motor_ops->set_command(context, DC_MOTOR_INTERNAL_COMMAND_RUN_FORWARD);
	else
			motor_ops->set_command(context, DC_MOTOR_INTERNAL_COMMAND_RUN_REVERSE);

	err = motor_ops->set_duty_cycle(context, abs(ev3_tm->duty_cycle));

	WARN_ONCE(err, "Failed to set pwm duty cycle! (%d)\n", err);
}

static int legoev3_motor_stop(void *context,
			      enum tacho_motor_stop_command action)
{
	struct legoev3_motor_data *ev3_tm = context;
	const struct dc_motor_ops *motor_ops = ev3_tm->ldev->port->dc_motor_ops;
	void *dc_ctx = ev3_tm->ldev->port->context;

	ev3_tm->speed_pid_ena = false;
	ev3_tm->hold_pid_ena = false;
	/*
	 * Reset the PID terms here to avoid having these terms influence the
	 * motor operation at the beginning of the next sequence. The most
	 * common issue is having some residual integral value briefly turn
	 * the motor on hard if we're ramping up slowly.
	 */
	tm_pid_reinit(&ev3_tm->speed_pid);
	tm_pid_reinit(&ev3_tm->hold_pid);

	switch (action) {
	case TM_STOP_COMMAND_COAST:
		motor_ops->set_command(dc_ctx, DC_MOTOR_INTERNAL_COMMAND_COAST);
		break;
	case TM_STOP_COMMAND_BRAKE:
		motor_ops->set_command(dc_ctx, DC_MOTOR_INTERNAL_COMMAND_BRAKE);
		break;
	case TM_STOP_COMMAND_HOLD:
		if (IS_POS_CMD(ev3_tm->run_command))
			ev3_tm->hold_pid.setpoint = ev3_tm->position_sp;
		else
			ev3_tm->hold_pid.setpoint = ev3_tm->position;
		ev3_tm->hold_pid_ena = true;
		break;
	default:
		BUG();
	}

	ev3_tm->state = STATE_STOPPED;

	return 0;
}

/**
 * legoev3_motor_reset - reinitialize driver to default values.
 *
 * @context: The motor data structure.
 */
static int legoev3_motor_reset(void *context)
{
	struct legoev3_motor_data *ev3_tm = context;
	const struct legoev3_motor_info *info = &ev3_tm->tm.info->legoev3_info;

	legoev3_motor_stop(ev3_tm, TM_STOP_COMMAND_COAST);

	memset(ev3_tm->tacho_samples, 0, sizeof(unsigned) * TACHO_SAMPLES);

	ev3_tm->tacho_samples_head	= 0;
	ev3_tm->got_new_sample		= false;
	ev3_tm->num_samples		= info->samples_for_speed[SPEED_BELOW_40];
	ev3_tm->dir_chg_samples		= 0;
	ev3_tm->max_us_per_sample	= info->max_us_per_sample;

	ev3_tm->run_direction		= UNKNOWN;

	ev3_tm->position_sp		= 0;
	ev3_tm->position		= 0;
	ev3_tm->speed			= 0;
	ev3_tm->duty_cycle		= 0;
	ev3_tm->overloaded		= 0;
	ev3_tm->stalled			= 0;

	ev3_tm->run_command		= TM_COMMAND_STOP;
	tm_pid_init(&ev3_tm->speed_pid, info->speed_pid_k.p,
		    info->speed_pid_k.i, info->speed_pid_k.d);
	tm_pid_init(&ev3_tm->hold_pid, info->position_pid_k.p,
		    info->position_pid_k.i, info->position_pid_k.d);

	return 0;
};

/*
 * NOTE: The comments are from the original LEGO source code, but the code
 *       has been changed to reflect the per-motor data structures.
 *
 * NOTE: The original LEGO code used the 24 most significant bits of the
 *       free-running P3 timer by dividing the values captured at every
 *       interrupt edge by 256. Unfortunately, this results in having to
 *       mask off the 24 least significant bits in all subsequent calculations
 *       that involve the captured values.
 *
 *       The reason is probably to keep most of the calculations within the
 *       16 bit value size. This driver avoids that problem by simply scaling
 *       the results when needed. The code and comments are updated to
 *       reflect the change.
 *
 *  - Calculates the actual speed for a motor
 *
 *  - Returns true when a new speed has been calculated, false if otherwise
 *
 *  - Time is sampled every edge on the tacho
 *
 *  - Tacho counter is updated on every edge of the tacho INTx pin signal
 *  - Time capture is updated on every edge of the tacho INTx pin signal
 *
 *  - Speed is calculated from the following parameters
 *
 *  - Time is measured edge to edge of the tacho interrupt pin. Average of
 *    time is always minimum 2 pulses:
 *
 *      (1 high + 1 low period or 1 low + 1 high period)
 *
 *    because the duty of the high and low period of the tacho pulses is
 *    are not always 50%.
 *
 *        - Average of the large motor
 *          - Above speed 80 it is:          64 samples
 *          - Between speed 60 - 80 it is:   32 samples
 *          - Between speed 40 - 60 it is:   16 samples
 *          - below speed 40 it is:           4 samples
 *
 *        - Average of the medium motor
 *          - Above speed 80 it is:          16 samples
 *          - Between speed 60 - 80 it is:    8 samples
 *          - Between speed 40 - 60 it is:    4 samples
 *          - below speed 40 it is:           2 sample
 *
 *      - Number of samples is always determined based on 1 sample meaning
 *        1 low period or 1 high period, this is to enable fast adoption to
 *        changes in speed. Medium motor has the critical timing because
 *        it can change speed and direction very fast.
 *
 *      - Large Motor
 *        - Maximum speed of the Large motor is approximately 2ms per tacho
 *          pulse (low + high period)
 *        - Minimum speed of the large motor is a factor of 100 less than
 *          max. speed
 *          Because 1 sample is based on only half a period minimum speed
 *          is 2ms / 2 => 1000000us
 *
 *      - Medium Motor
 *        - Maximum speed of the medium motor is approximately 1.25ms per
 *          tacho pulse (low + high period)
 *        - Minimum speed of the medium motor is a factor of 100 less than
 *          max. speed
 *          Because 1 sample is based on only half a period minimum speed
 *          is 1.25ms / 2 => 750000us
 *
 *  - Tacho pulse examples:
 *
 *
 *    - Normal
 *
 *      ----       ------       ------       ------
 *          |     |      |     |      |     |      |
 *          |     |      |     |      |     |      |
 *          -------      -------      -------      --- DIRx signal
 *
 *
 *         ----       ------       ------       ------ INTx signal
 *             |     |      |     |      |     |
 *             |     |      |     |      |     |
 *             -------      -------      -------
 *
 *             ^     ^      ^     ^      ^     ^
 *             |     |      |     |      |     |
 *             |   Timer    |   Timer    |   Timer
 *             |     +      |     +      |     +
 *             |  Counter   |  Counter   |  Counter
 *             |            |            |
 *           Timer        Timer        Timer
 *             +            +            +
 *          Counter      Counter      Counter
 *
 *
 *
 *    - Direction change
 *
 *      DirChgPtr variable is used to indicate how many timer samples have
 *      been sampled since direction has been changed. DirChgPtr is set
 *      to 0 when tacho interrupt detects direction change and then it is
 *      counted up for every timer sample. So when DirChgPtr has the value
 *      of 2 then there must be 2 timer samples in the the same direction
 *      available.
 *
 *      ----       ------       ------       ------       ---
 *          |     |      |     |      |     |      |     |
 *          |     |      |     |      |     |      |     |
 *          -------      -------      -------      -------   DIRx signal
 *
 *
 *       ------       -------------       ------       ------INTx signal
 *             |     |             |     |      |     |
 *             |     |             |     |      |     |
 *             -------             -------      -------
 *
 *             ^     ^             ^     ^      ^     ^
 *             |     |             |     |      |     |
 *           Timer   |           Timer   |    Timer   |
 *             +     |             +     |      +     |
 *          Counter  |          Counter  |   Counter  |
 *             +     |             +     |      +     |
 *       DirChgPtr++ |       DirChgPtr=0 |DirChgPtr++ |
 *                 Timer               Timer        Timer
 *                   +                   +            +
 *                Counter             Counter      Counter
 *                   +                   +            +
 *               DirChgPtr++         DirChgPtr++  DirChgPtr++
 *
 *
 *
 *
 *      ----       ------             ------        ----
 *          |     |      |           |      |      |
 *          |     |      |           |      |      |
 *          -------      -------------       -------          DIRx signal
 *
 *
 *       ------       ------       ------       ------        INTx signal
 *             |     |      |     |      |     |      |
 *             |     |      |     |      |     |      |
 *             -------      -------      -------       ----
 *
 *             ^     ^      ^     ^      ^     ^      ^
 *             |     |      |     |      |     |      |
 *           Timer   |    Timer   |    Timer   |    Timer
 *             +     |      +     |      +     |      +
 *          Counter  |   Counter  |   Counter  |   Counter
 *             +     |      +     |      +     |      +
 *        DirChgPtr++| DirChgPtr++| DirChgPtr++| DirChgPtr++
 *                 Timer        Timer        Timer
 *                   +            +            +
 *                Counter      Counter      Counter
 *                   +            +            +
 *               DirChgPtr++  DirChgPtr=0  DirChgPtr++
 *
 */

static void calculate_speed(struct legoev3_motor_data *ev3_tm)
{
	unsigned diff_idx;
	ktime_t diff;

	/* TODO - Don't run this if we're updating the ev3_tm in the isr! */

	/*
	 * Determine the approximate speed of the motor using the difference
	 * in time between this tacho pulse and the previous pulse.
	 *
	 * The old code had a conditional that forced the difference to be at
	 * least 1 by checking for the special case of a zero difference, which
	 * would be almost impossible to achieve in practice.
	 *
	 * This version simply ORs a 1 into the LSB of the difference - now
	 * that we're using the full 32 bit free running counter, the impact
	 * on an actual speed calculation is insignificant, and it avoids the
	 * issue with simply adding 1 in the obscure case that the difference
	 * is 0xFFFFFFFF!
	 *
	 * Only do this estimated speed calculation if we've accumulated at
	 * least two tacho pulses where the motor is turning in the same
	 * direction!
	 */

	diff_idx = ev3_tm->tacho_samples_head;

	/* TODO - This should really be a boolean value that gets set at the ISR level */
	/* TODO - Can/Should we change this to not set_samples_per_speed every time we're called? */

	if (ev3_tm->dir_chg_samples >= 1) {
		diff = ktime_sub(ev3_tm->tacho_samples[diff_idx],
			ev3_tm->tacho_samples[(diff_idx + TACHO_SAMPLES - 1) % TACHO_SAMPLES]);

		if (unlikely(ktime_equal(diff, ktime_set(0, 0))))
			diff = ktime_set(0, 1);

		set_num_samples_for_speed(ev3_tm, ev3_tm->max_us_per_sample / (int)ktime_to_us(diff));
	}

	/*
	 * Now get a better estimate of the motor speed by using the total
	 * time used to accumulate the last n samples, where n is determined
	 * by the first approximation to the speed.
	 *
	 * The new speed can only be updated if we have accumulated at least
	 * as many samples as are required depending on the estimated speed
	 * of the motor.
	 *
	 * If the speed cannot be updated, then we need to check if the speed
	 * is 0!
	 *
	 * The last case is for updating the speed when the last time we got
	 * a new sample was longer than the tacho update cycle. Instead of
	 * using the last captured sample as the start point for the differnce
	 * we use the current time.
	 */
	if (ev3_tm->got_new_sample && (ev3_tm->dir_chg_samples >= ev3_tm->num_samples)) {
		long new_speed;

		diff = ktime_sub(ev3_tm->tacho_samples[diff_idx],
			ev3_tm->tacho_samples[(diff_idx + TACHO_SAMPLES - ev3_tm->num_samples) % TACHO_SAMPLES]);

		if (unlikely(ktime_equal(diff, ktime_set(0, 0))))
			diff = ktime_set(0, 1);

		new_speed = USEC_PER_SEC * ev3_tm->num_samples;
		new_speed = div64_s64(new_speed, ktime_to_us(diff));

		if (ev3_tm->run_direction == FORWARD)
			ev3_tm->speed  = new_speed;
		else
			ev3_tm->speed  = -new_speed;

		ev3_tm->stalled = 0;
		ev3_tm->got_new_sample = false;

	} else if (ev3_tm->max_us_per_sample < ktime_to_us(ktime_sub(ktime_get(), ev3_tm->tacho_samples[diff_idx]))) {

		ev3_tm->dir_chg_samples = 0;
		ev3_tm->speed = 0;
		ev3_tm->stalled = 1;
	  }

	else if ((TACHO_MOTOR_POLL_MS * USEC_PER_MSEC) < ktime_to_us(ktime_sub(ktime_get(), ev3_tm->tacho_samples[diff_idx]))) {
		long new_speed;

		diff = ktime_sub(ktime_get(),
			ev3_tm->tacho_samples[(diff_idx + TACHO_SAMPLES - ev3_tm->num_samples) % TACHO_SAMPLES]);

		new_speed = USEC_PER_SEC * ev3_tm->num_samples;
		new_speed = div64_s64(new_speed, ktime_to_us(diff));

		if (ev3_tm->run_direction == FORWARD)
			ev3_tm->speed  = new_speed;
		else
			ev3_tm->speed  = -new_speed;

		ev3_tm->stalled = 0;
	}
}

static void update_position(struct legoev3_motor_data *ev3_tm)
{
	int rampdown_endpoint;
	int rampdown_time;
	int new_speed_sp;

	/* Calculate where we would end up if we started a
	 * linear ramp down right now, the 1 prevents the
	 * value from being 0 and causing a division problem
	 */

	rampdown_time = (abs(ev3_tm->speed) * ev3_tm->tm.active_params.ramp_down_sp)
				/ (1 + ev3_tm->tm.info->max_speed);

	rampdown_endpoint = ev3_tm->position
			  + ((ev3_tm->speed * rampdown_time) / (2*MSEC_PER_SEC));

	new_speed_sp = (2*MSEC_PER_SEC * (ev3_tm->position_sp - ev3_tm->position)
				/ (1 + rampdown_time));

	if (ev3_tm->speed_pid.setpoint > 0) {
		if (rampdown_endpoint > ev3_tm->position_sp) {
			ev3_tm->speed_pid.setpoint = new_speed_sp;
			ev3_tm->ramping = 1;
		}

		if (ev3_tm->position >= ev3_tm->position_sp) {
			schedule_work(&ev3_tm->notify_position_ramp_down_work);
			ev3_tm->run_command = TM_COMMAND_STOP;
			legoev3_motor_stop(ev3_tm,
					ev3_tm->tm.active_params.stop_command);
			ev3_tm->ramping = 0;
		}
	} else if (ev3_tm->speed_pid.setpoint < 0) {
		if (rampdown_endpoint < ev3_tm->position_sp) {
			ev3_tm->speed_pid.setpoint = new_speed_sp;
			ev3_tm->ramping = 1;
		}

		if (ev3_tm->position <= ev3_tm->position_sp) {
			schedule_work(&ev3_tm->notify_position_ramp_down_work);
			ev3_tm->run_command = TM_COMMAND_STOP;
			legoev3_motor_stop(ev3_tm,
					ev3_tm->tm.active_params.stop_command);
			ev3_tm->ramping = 0;
		}
	}
}

static enum hrtimer_restart legoev3_motor_timer_callback(struct hrtimer *timer)
{
	struct legoev3_motor_data *ev3_tm =
			container_of(timer, struct legoev3_motor_data, timer);
	int duty_cycle = 0;

	hrtimer_forward_now(timer, ktime_set(0, TACHO_MOTOR_POLL_MS * NSEC_PER_MSEC));

	/* Continue with the actual calculations */

	calculate_speed(ev3_tm);

	if (ev3_tm->speed_pid_ena) {
		if (ev3_tm->speed_pid.setpoint == 0) {
			duty_cycle = 0;
			tm_pid_reinit(&ev3_tm->speed_pid);
		} else
			duty_cycle = tm_pid_update(&ev3_tm->speed_pid,
						   ev3_tm->speed);
	} else if (ev3_tm->hold_pid_ena)
		duty_cycle = tm_pid_update(&ev3_tm->hold_pid, ev3_tm->position);

	if (IS_POS_CMD(ev3_tm->run_command))
		update_position(ev3_tm);

	set_duty_cycle(ev3_tm, duty_cycle);

	return HRTIMER_RESTART;
}

static void legoev3_motor_notify_position_ramp_down_work(struct work_struct *work)
{
	struct legoev3_motor_data *ev3_tm =
		container_of(work, struct legoev3_motor_data, notify_position_ramp_down_work);

	tacho_motor_notify_position_ramp_down(&ev3_tm->tm);
}

static void legoev3_motor_notify_state_change_work(struct work_struct *work)
{
	struct legoev3_motor_data *ev3_tm =
		container_of(work, struct legoev3_motor_data, notify_state_change_work);

	tacho_motor_notify_state_change(&ev3_tm->tm);
}

static int legoev3_motor_get_position(void *context, long *position)
{
	struct legoev3_motor_data *ev3_tm = context;

	*position = ev3_tm->position;

	return 0;
}

static int legoev3_motor_get_state(void *context);

static int legoev3_motor_set_position(void *context, long position)
{
	struct legoev3_motor_data *ev3_tm = context;

	if (legoev3_motor_get_state(ev3_tm) & TM_STATE_RUNNING)
		return -EBUSY;

	ev3_tm->position    = position;
	ev3_tm->position_sp = position;

	return 0;
}

static int legoev3_motor_get_duty_cycle(void *context,
					int *duty_cycle)
{
	struct legoev3_motor_data *ev3_tm = context;

	*duty_cycle = ev3_tm->duty_cycle;

	return 0;
}

static int legoev3_motor_run_unregulated(void *context, int duty_cycle)
{
	struct legoev3_motor_data *ev3_tm = context;

	ev3_tm->duty_cycle = duty_cycle;

	if (ev3_tm->run_command == TM_COMMAND_RUN_DIRECT)
		set_duty_cycle(ev3_tm, ev3_tm->duty_cycle);

	return 0;
}

static int legoev3_motor_get_state(void *context)
{
	struct legoev3_motor_data *ev3_tm = context;
	unsigned state = 0;

	if (ev3_tm->state == STATE_RUNNING) {
		state |= BIT(TM_STATE_RUNNING);
		if (ev3_tm->speed_pid_ena
		    && tm_pid_is_overloaded(&ev3_tm->speed_pid))
			state |= BIT(TM_STATE_OVERLOADED);
		if (ev3_tm->overloaded)
			state |= BIT(TM_STATE_OVERLOADED);
		if (ev3_tm->stalled)
			state |= BIT(TM_STATE_STALLED);
		if (ev3_tm->ramping)
			state |= BIT(TM_STATE_RAMPING);
	}
	if (ev3_tm->hold_pid_ena) {
		state |= BIT(TM_STATE_HOLDING);
		if (tm_pid_is_overloaded(&ev3_tm->hold_pid))
			state |= BIT(TM_STATE_OVERLOADED);
	}

	return state;
}

static int legoev3_motor_get_speed(void *context, int *speed)
{
	struct legoev3_motor_data *ev3_tm = context;

	*speed = ev3_tm->speed;

	return 0;
}

static int legoev3_motor_run_regulated(void *context, int speed)
{
	struct legoev3_motor_data *ev3_tm = context;

	if (IS_RUN_CMD(ev3_tm->run_command)) {
		if (IS_POS_CMD(ev3_tm->run_command)) {
			if (ev3_tm->position < ev3_tm->position_sp)
				ev3_tm->speed_pid.setpoint = abs(speed);
			else
				ev3_tm->speed_pid.setpoint = abs(speed) * -1;
		} else {
			ev3_tm->speed_pid.setpoint = speed;
		}
	}

	return 0;
}

static unsigned legoev3_motor_get_stop_commands(void *context)
{
	return BIT(TM_STOP_COMMAND_COAST) | BIT(TM_STOP_COMMAND_BRAKE) |
		BIT(TM_STOP_COMMAND_HOLD);
}

TM_PID_GET_FUNC(legoev3_motor, speed_Kp, legoev3_motor_data, speed_pid.Kp);
TM_PID_SET_FUNC(legoev3_motor, speed_Kp, legoev3_motor_data, speed_pid.Kp);
TM_PID_GET_FUNC(legoev3_motor, speed_Ki, legoev3_motor_data, speed_pid.Ki);
TM_PID_SET_FUNC(legoev3_motor, speed_Ki, legoev3_motor_data, speed_pid.Ki);
TM_PID_GET_FUNC(legoev3_motor, speed_Kd, legoev3_motor_data, speed_pid.Kd);
TM_PID_SET_FUNC(legoev3_motor, speed_Kd, legoev3_motor_data, speed_pid.Kd);
TM_PID_GET_FUNC(legoev3_motor, position_Kp, legoev3_motor_data, hold_pid.Kp);
TM_PID_SET_FUNC(legoev3_motor, position_Kp, legoev3_motor_data, hold_pid.Kp);
TM_PID_GET_FUNC(legoev3_motor, position_Ki, legoev3_motor_data, hold_pid.Ki);
TM_PID_SET_FUNC(legoev3_motor, position_Ki, legoev3_motor_data, hold_pid.Ki);
TM_PID_GET_FUNC(legoev3_motor, position_Kd, legoev3_motor_data, hold_pid.Kd);
TM_PID_SET_FUNC(legoev3_motor, position_Kd, legoev3_motor_data, hold_pid.Kd);

static unsigned legoev3_motor_get_commands(void *context)
{
	return BIT(TM_COMMAND_RUN_FOREVER) | BIT(TM_COMMAND_RUN_TO_ABS_POS)
		| BIT(TM_COMMAND_RUN_TO_REL_POS) | BIT(TM_COMMAND_RUN_DIRECT)
		| BIT(TM_COMMAND_STOP) | BIT(TM_COMMAND_RESET);
}

static int legoev3_motor_send_command(void *context,
					struct tacho_motor_params *params,
					enum tacho_motor_command command)
{
	struct legoev3_motor_data *ev3_tm = context;

	if (command == TM_COMMAND_RESET) {
		legoev3_motor_reset(ev3_tm);
		return 0;
	}

	switch (command) {
	case TM_COMMAND_RUN_TO_ABS_POS:
			ev3_tm->speed_pid_ena = true;
			ev3_tm->hold_pid_ena = false;
			ev3_tm->position_sp = params->position_sp;
			ev3_tm->state = STATE_RUNNING;
			break;
	case TM_COMMAND_RUN_TO_REL_POS:
			ev3_tm->speed_pid_ena = true;
			ev3_tm->hold_pid_ena = false;
			/* To check the previous command, we MUST be looking
			 * at ev3_tm->tm.command because that's the previous
			 * command that the user sent! The ev3_tm->run_command
			 * is likely to be different since we use
			 * TM_COMMAND_STOP to stop the motor when running to
			 * position.
			 */

			if (ev3_tm->tm.command != TM_COMMAND_RUN_TO_REL_POS)
				ev3_tm->position_sp = ev3_tm->position + params->position_sp;
			else
				ev3_tm->position_sp += params->position_sp;
			ev3_tm->state = STATE_RUNNING;
			break;
	case TM_COMMAND_RUN_DIRECT:
			ev3_tm->speed_pid_ena = false;
			ev3_tm->hold_pid_ena = false;
			set_duty_cycle(ev3_tm, ev3_tm->duty_cycle);
			ev3_tm->state = STATE_RUNNING;
			break;
	case TM_COMMAND_RUN_FOREVER:
	case TM_COMMAND_RUN_TIMED:
			ev3_tm->speed_pid_ena = true;
			ev3_tm->hold_pid_ena = false;
			ev3_tm->state = STATE_RUNNING;
			break;
	case TM_COMMAND_STOP:
			legoev3_motor_stop(ev3_tm, params->stop_command);
			break;
	default:
			return -EOPNOTSUPP;
	}

	ev3_tm->run_command = command;

	return 0;
}

static const struct tacho_motor_ops legoev3_motor_ops = {
	.get_position		= legoev3_motor_get_position,
	.set_position		= legoev3_motor_set_position,

	.get_state		= legoev3_motor_get_state,
	.get_duty_cycle		= legoev3_motor_get_duty_cycle,
	.get_speed		= legoev3_motor_get_speed,

	.run_unregulated	= legoev3_motor_run_unregulated,
	.run_regulated		= legoev3_motor_run_regulated,
	.stop			= legoev3_motor_stop,
	.reset			= legoev3_motor_reset,

	.get_commands		= legoev3_motor_get_commands,
	.send_command		= legoev3_motor_send_command,

	.get_stop_commands	= legoev3_motor_get_stop_commands,

	.get_speed_Kp		= legoev3_motor_get_speed_Kp,
	.set_speed_Kp		= legoev3_motor_set_speed_Kp,
	.get_speed_Ki		= legoev3_motor_get_speed_Ki,
	.set_speed_Ki		= legoev3_motor_set_speed_Ki,
	.get_speed_Kd		= legoev3_motor_get_speed_Kd,
	.set_speed_Kd		= legoev3_motor_set_speed_Kd,

	.get_hold_Kp		= legoev3_motor_get_position_Kp,
	.set_hold_Kp		= legoev3_motor_set_position_Kp,
	.get_hold_Ki		= legoev3_motor_get_position_Ki,
	.set_hold_Ki		= legoev3_motor_set_position_Ki,
	.get_hold_Kd		= legoev3_motor_get_position_Kd,
	.set_hold_Kd		= legoev3_motor_set_position_Kd,
};


static int legoev3_motor_probe(struct lego_device *ldev)
{
	struct legoev3_motor_data *ev3_tm;
	struct ev3_motor_platform_data *pdata = ldev->dev.platform_data;
	int err;

	if (WARN_ON(!pdata))
		return -EINVAL;
	if (WARN_ON(!ldev->port->dc_motor_ops))
		return -EINVAL;
	if (WARN_ON(!ldev->entry_id))
		return -EINVAL;

	ev3_tm = kzalloc(sizeof(struct legoev3_motor_data), GFP_KERNEL);
	if (!ev3_tm)
		return -ENOMEM;

	ev3_tm->ldev = ldev;

	ev3_tm->tm.driver_name = ldev->entry_id->name;
	ev3_tm->tm.address = ldev->port->address;
	ev3_tm->tm.ops = &legoev3_motor_ops;
	ev3_tm->tm.info = &ev3_motor_defs[ldev->entry_id->driver_data];
	ev3_tm->tm.context = ev3_tm;
	ev3_tm->tm.supports_encoder_polarity = true;

	dev_set_drvdata(&ldev->dev, ev3_tm);

	hrtimer_init(&ev3_tm->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	ev3_tm->timer.function = legoev3_motor_timer_callback;

	INIT_WORK(&ev3_tm->notify_state_change_work,
		  legoev3_motor_notify_state_change_work);
	INIT_WORK(&ev3_tm->notify_position_ramp_down_work,
		  legoev3_motor_notify_position_ramp_down_work);

	err = register_tacho_motor(&ev3_tm->tm, &ldev->dev);

	if (err)
		goto err_register_tacho_motor;

	legoev3_motor_reset(ev3_tm);

	err = request_irq(gpio_to_irq(pdata->tacho_int_gpio), tacho_motor_isr,
				IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
				dev_name(&ldev->dev), ev3_tm);

	if (err)
		goto err_request_irq;

	hrtimer_start(&ev3_tm->timer, ktime_set(0, TACHO_MOTOR_POLL_MS * NSEC_PER_MSEC),
		      HRTIMER_MODE_REL);

	return 0;

err_request_irq:
	dev_set_drvdata(&ldev->dev, NULL);
	unregister_tacho_motor(&ev3_tm->tm);
err_register_tacho_motor:
	kfree(ev3_tm);

	return err;
}

static int legoev3_motor_remove(struct lego_device *ldev)
{
	struct ev3_motor_platform_data *pdata = ldev->dev.platform_data;
	struct legoev3_motor_data *ev3_tm = dev_get_drvdata(&ldev->dev);

	hrtimer_cancel(&ev3_tm->timer);
	cancel_work_sync(&ev3_tm->notify_state_change_work);
	cancel_work_sync(&ev3_tm->notify_position_ramp_down_work);
	free_irq(gpio_to_irq(pdata->tacho_int_gpio), ev3_tm);
	dev_set_drvdata(&ldev->dev, NULL);
	unregister_tacho_motor(&ev3_tm->tm);
	kfree(ev3_tm);
	return 0;
}

struct lego_device_id legoev3_motor_driver_id_table[NUM_EV3_MOTOR_ID] = {
	LEGO_DEVICE_ID(LEGO_NXT_MOTOR),
	LEGO_DEVICE_ID(LEGO_EV3_LARGE_MOTOR),
	LEGO_DEVICE_ID(LEGO_EV3_MEDIUM_MOTOR),
	LEGO_DEVICE_ID(FIRGELLI_L12_EV3_50),
	LEGO_DEVICE_ID(FIRGELLI_L12_EV3_100),
};

/* TODO: Make this a bus driver attribute instead so it is present for all drivers on the lego bus */
static ssize_t driver_names_show(struct device_driver *drv, char *buf)
{
	int i;
	int size = 0;

	for (i = 0; i < NUM_EV3_MOTOR_ID; i++) {
		size += sprintf(buf + size, "%s ",
			legoev3_motor_driver_id_table[i].name);
	}

	buf[size - 1] = '\n';

	return size;
}

static DRIVER_ATTR_RO(driver_names);

static struct attribute *legoev3_motor_attrs[] = {
	&driver_attr_driver_names.attr,
	NULL
};

ATTRIBUTE_GROUPS(legoev3_motor);

struct lego_device_driver legoev3_motor_driver = {
	.probe	= legoev3_motor_probe,
	.remove	= legoev3_motor_remove,
	.driver = {
		.name	= "legoev3-motor",
		.owner	= THIS_MODULE,
		.groups	= legoev3_motor_groups,
	},
	.id_table = legoev3_motor_driver_id_table,
};
lego_device_driver(legoev3_motor_driver);

MODULE_DESCRIPTION("Motor driver for LEGO MINDSTORMS EV3");
MODULE_AUTHOR("Ralph Hempel <rhempel@hempeldesigngroup.com>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("lego:legoev3-motor");

