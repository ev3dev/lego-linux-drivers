/*
 * EV3 Tacho Motor device driver for LEGO Mindstorms EV3
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
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/pwm/pwm.h>
#include <linux/legoev3/legoev3_ports.h>
#include <linux/legoev3/ev3_output_port.h>
#include <linux/legoev3/tacho_motor_class.h>

#include <mach/time.h>

#include <asm/bug.h>

#define TACHO_MOTOR_POLL_NS	(  2000000)	                /* 2 msec */

#define   TACHO_SAMPLES           128

#define   MAX_PWM_CNT                   (10000)
#define   MAX_SPEED                     (100)
#define   SPEED_PWMCNT_REL              (100) //(MAX_PWM_CNT/MAX_SPEED)
#define   RAMP_FACTOR                   (1000)
#define   MAX_SYNC_MOTORS               (2)

// These are wrong, recalcualte for full 32 bit free running counter
//
// #define   COUNTS_PER_PULSE_LM           12800L
// #define   COUNTS_PER_PULSE_MM           8100L

/*
 *
 */

enum
{
  MOTOR_TYPE_0  = 0,
  MOTOR_TYPE_1,
  MOTOR_TYPE_2,
  MOTOR_TYPE_3,
  MOTOR_TYPE_4,
  MOTOR_TYPE_5,
  MOTOR_TYPE_6,
  MOTOR_TYPE_TACHO,
  MOTOR_TYPE_MINITACHO,
  MOTOR_TYPE_NEWTACHO,
  MOTOR_TYPE_10,
  MOTOR_TYPE_11,
  MOTOR_TYPE_12,
  MOTOR_TYPE_13,
  MOTOR_TYPE_14,
  MOTOR_TYPE_15,
  NO_OF_MOTOR_TYPES,
};

enum
{
  SAMPLES_PER_SPEED_BELOW_40 =  0,
  SAMPLES_PER_SPEED_ABOVE_40 =  1,
  SAMPLES_PER_SPEED_ABOVE_60 =  2,
  SAMPLES_PER_SPEED_ABOVE_80 =  3,
  NO_OF_SAMPLE_STEPS     =  4
};

enum {
	UNKNOWN,
	FORWARD,
	REVERSE,
	BRAKE,
	COAST,
};

/*! \brief    Number of average samples for each motor
 *
 *  - Medium motor = 1, 2,  4,  8
 *  - Large motor  = 2, 8, 16, 32
 *
 *  Medium motor has not the jitter on the tacho when as large motor because
 *  it has one gear wheel less that the large motor.
 *
 *  Medium motor reaction time is much faster than large motor due to smaller motor
 *  and smaller gearbox
 */

struct ev3_tacho_motor_data {
	struct tacho_motor_device tm;

	struct legoev3_port_device *out_port;
	struct legoev3_port_device *motor_port;

 	struct hrtimer timer;

        unsigned tacho_samples[TACHO_SAMPLES];
        unsigned tacho_samples_head;

	bool got_new_sample;
	
	unsigned samples_per_speed;
//	int	 *ramp_power_steps;
	
	unsigned counts_per_pulse;
	unsigned dir_chg_samples;

	/* FIXME - this should really just be tacho! And moved below target_xxx */

	int irq_tacho;

	/* FIXME - these need better names */

	bool mutex;
	int timer_foo;

//	/* FIXME - these relate to the time setpoints, probably need renaming */
//
//	int time_cnt;
//	int time_inc;
//
//	/* FIXME - these relate to the ramp times, probbaly need renaming */
//
//	int ramp_up_factor;
//	int ramp_up_offset;
//
//	int tacho_cnt_up;
//
//	int prg_stop_timer;
//
//	bool lock_ramp_down;
//	bool brake_after;

        struct {
		int start;
		int end;
		int slope;
	} ramp_up;

        struct {
		int start;
		int end;
		int slope;
	} ramp_down;

	struct {
		int setpoint;
		int setpoint_sign;
		int offset;
		int count;	/* This must be set to either tacho or time increment! */
	} ramp;

	struct {
		int P;
		int I;
		int D;
		int oldError;
	} pid;

	int state;

	unsigned int stop_mode;
	unsigned int regulation_mode;
	unsigned int ramp_mode;
	unsigned int tacho_mode;

	int target_state;
	int target_speed;
	int target_steer;
	int target_power;
        int target_direction;
        int target_tacho;
        int target_step;
        int target_time;

        int target_ramp_up_count;
        int target_total_count;
        int target_ramp_down_count;

	int tacho_cnt;
	int time_cnt;

	int tacho_sensor;
	int old_tacho_cnt;

	int motor_type;

	int run;
	int mode;

	int power_setpoint;
        int speed_setpoint;
        int speed;
        int run_direction;
        int set_direction;
        int tacho;
        int time;
};
// static    unsigned    SamplesMediumMotor[NO_OF_SAMPLE_STEPS] = {2, 4,  8,  16};
// static    unsigned    SamplesLargeMotor[NO_OF_SAMPLE_STEPS]  = {4, 16, 32, 64};
//
//  TYPE_TACHO                    =   7,  //!< Device is a tacho motor
//  TYPE_MINITACHO                =   8,  //!< Device is a mini tacho motor
//  TYPE_NEWTACHO                 =   9,  //!< Device is a new tacho motor


static unsigned SamplesPerSpeed[NO_OF_MOTOR_TYPES][NO_OF_SAMPLE_STEPS] = {
	{  2,  2,  2,  2 } , /* Motor Type  0             */
	{  2,  2,  2,  2 } , /* Motor Type  1             */
	{  2,  2,  2,  2 } , /* Motor Type  2             */
	{  2,  2,  2,  2 } , /* Motor Type  3             */
	{  2,  2,  2,  2 } , /* Motor Type  4             */
	{  2,  2,  2,  2 } , /* Motor Type  5             */
	{  2,  2,  2,  2 } , /* Motor Type  6             */
	{  4, 16, 32, 64 } , /* Motor Type  7 - TACHO     */
	{  2,  4,  8, 16 } , /* Motor Type  8 - MINITACHO */
	{  2,  2,  2,  2 } , /* Motor Type  9 - NEWTACHO  */
	{  2,  2,  2,  2 } , /* Motor Type 10             */
	{  2,  2,  2,  2 } , /* Motor Type 11             */
	{  2,  2,  2,  2 } , /* Motor Type 12             */
	{  2,  2,  2,  2 } , /* Motor Type 13             */
	{  2,  2,  2,  2 } , /* Motor Type 14             */
	{  2,  2,  2,  2 } , /* Motor Type 15             */
};

static unsigned CountsPerPulse[NO_OF_MOTOR_TYPES] = {
	      0, /* Motor Type  0             */
	      0, /* Motor Type  1             */
	      0, /* Motor Type  2             */
	      0, /* Motor Type  3             */
	      0, /* Motor Type  4             */
	      0, /* Motor Type  5             */
	      0, /* Motor Type  6             */
	3300000, /* Motor Type  7 - TACHO     */
	2062500, /* Motor Type  8 - MINITACHO */
	      0, /* Motor Type  9 - NEWTACHO  */
	      0, /* Motor Type 10             */
	      0, /* Motor Type 11             */
	      0, /* Motor Type 12             */
	      0, /* Motor Type 13             */
	      0, /* Motor Type 14             */
	      0, /* Motor Type 15             */
};

static void set_samples_per_speed( struct ev3_tacho_motor_data *ev3_tm, int speed ) {
	if (speed > 80) {
		ev3_tm->samples_per_speed = SamplesPerSpeed[ev3_tm->motor_type][SAMPLES_PER_SPEED_ABOVE_80];
	} else if (speed > 60) {
		ev3_tm->samples_per_speed = SamplesPerSpeed[ev3_tm->motor_type][SAMPLES_PER_SPEED_ABOVE_60];
	} else if (speed > 40) {
		ev3_tm->samples_per_speed = SamplesPerSpeed[ev3_tm->motor_type][SAMPLES_PER_SPEED_ABOVE_40];
	} else {
		ev3_tm->samples_per_speed = SamplesPerSpeed[ev3_tm->motor_type][SAMPLES_PER_SPEED_BELOW_40];
	}
}

// static    unsigned    AVG_TACHO_COUNTS[NO_OF_OUTPUT_PORTS]   = {2,2,2,2};
// static    unsigned    AVG_COUNTS[NO_OF_OUTPUT_PORTS]         = {(2 * COUNTS_PER_PULSE_LM),(2 * COUNTS_PER_PULSE_LM),(2 * COUNTS_PER_PULSE_LM),(2 * COUNTS_PER_PULSE_LM)};

#define  NON_INV   1
#define  INV      -1

static irqreturn_t tacho_motor_isr(int irq, void *id)
{
	struct ev3_tacho_motor_data *ev3_tm = id;
	struct ev3_motor_platform_data *pdata = ev3_tm->motor_port->dev.platform_data; 

	bool int_state =  gpio_get_value(pdata->tacho_int_gpio);
	bool dir_state = !gpio_get_value(pdata->tacho_dir_gpio);
	unsigned long timer = legoev3_hires_timer_read();

	unsigned next_sample;

	int  next_direction;

	/* Grab the next incremental sample timestamp */

	next_sample = (ev3_tm->tacho_samples_head + 1) % TACHO_SAMPLES;

	ev3_tm->tacho_samples[next_sample] = timer;
	ev3_tm->tacho_samples_head = next_sample;

	/* If the speed is high enough, just update the tacho counter based on direction */

	if ((35 < ev3_tm->speed) || (-35 > ev3_tm->speed)) {

		if (ev3_tm->dir_chg_samples < (TACHO_SAMPLES-1))
			ev3_tm->dir_chg_samples++;

	} else {

	/* Update the tacho count and motor direction for low speed, taking
	/  advantage of the fact that if state and dir match, then the motor
	/  is turning FORWARD!
	*/

		if (int_state == dir_state)
			next_direction = FORWARD;
		else
			next_direction = REVERSE;

		/* If the saved and next direction states match, then update the dir_chg_sample count */

		if (ev3_tm->run_direction == next_direction) {
			if (ev3_tm->dir_chg_samples < (TACHO_SAMPLES-1))
				ev3_tm->dir_chg_samples++;
		} else {
//		        printk( "DirChg! %d %d \n", int_state, dir_state );
			ev3_tm->dir_chg_samples = 0;
		}

		ev3_tm->run_direction = next_direction;
	}

	if (FORWARD == ev3_tm->run_direction)
		ev3_tm->irq_tacho++;
	else			
		ev3_tm->irq_tacho--;

	ev3_tm->got_new_sample = true;

	return IRQ_HANDLED;
}

#warning "Consider inlining these functions with tacho_motor_set_power"
 
static void ev3_tacho_motor_forward(struct ev3_tacho_motor_data *ev3_tm, struct ev3_motor_platform_data *pdata)
{
//	if (FORWARD != ev3_tm->set_direction) {
		gpio_direction_output(pdata->motor_dir0_gpio, 1);
		gpio_direction_input( pdata->motor_dir1_gpio   );
		ev3_tm->set_direction = FORWARD;
//	}
}
static void ev3_tacho_motor_reverse(struct ev3_tacho_motor_data *ev3_tm, struct ev3_motor_platform_data *pdata)
{
//	if (REVERSE != ev3_tm->set_direction) {
		gpio_direction_input( pdata->motor_dir0_gpio   );
		gpio_direction_output(pdata->motor_dir1_gpio, 1);
		ev3_tm->set_direction = REVERSE;
//	}
}
static void ev3_tacho_motor_brake(struct ev3_tacho_motor_data *ev3_tm, struct ev3_motor_platform_data *pdata)
{
	#warning "The LEGO code sets the regulation power to an opposite level to stop the motor hard"

//	if (BRAKE != ev3_tm->set_direction) {
		gpio_direction_output(pdata->motor_dir0_gpio, 1);
		gpio_direction_output(pdata->motor_dir1_gpio, 1);
		ev3_tm->set_direction = BRAKE;
//	}
}
static void ev3_tacho_motor_coast(struct ev3_tacho_motor_data *ev3_tm, struct ev3_motor_platform_data *pdata)
{
//	if (COAST != ev3_tm->set_direction) {
		gpio_direction_output(pdata->motor_dir0_gpio, 0);
		gpio_direction_output(pdata->motor_dir1_gpio, 0);
		ev3_tm->set_direction = COAST;
//	}
}

static int ev3_tacho_motor_set_power(struct ev3_tacho_motor_data *ev3_tm, int power)
{
	int err;
	struct ev3_motor_platform_data *pdata = ev3_tm->motor_port->dev.platform_data; 

	ev3_tm->power_setpoint = power;

        if (0 < power) {
		ev3_tacho_motor_forward(ev3_tm, pdata);
	} else if (0 > power) {
		ev3_tacho_motor_reverse(ev3_tm, pdata);
	} else {
		if (STOP_COAST == ev3_tm->stop_mode)
			ev3_tacho_motor_coast(ev3_tm, pdata);
		else
			ev3_tacho_motor_brake(ev3_tm, pdata);
	}

	err = pwm_set_duty_percent(pdata->pwm, abs( power ));

 	if (err) {
 		dev_err(&ev3_tm->motor_port->dev, "%s: Failed to set pwm duty percent! (%d)\n",
 			__func__, err);
 	}

	return 0;
}

/*
 *! \brief    calculate_speed
 *
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
 *      - Timer used is 64bit timer plus (P3) module (dual 32bit un-chained mode)
 *      - 64bit timer is running 33Mhz (24Mhz (Osc) * 22 (Multiplier) / 2 (Post divider) / 2 (DIV2)) / 4 (T64 prescaler)
 *
 *  - Tacho counter is updated on every edge of the tacho INTx pin signal
 *  - Time capture is updated on every edge of the tacho INTx pin signal
 *
 *  - Speed is calculated from the following parameters
 *
 *      - Time is measured edge to edge of the tacho interrupt pin. Average of time is always minimum 2 pulses
 *        (1 high + 1 low period or 1 low + 1 high period) because the duty of the high and low period of the
 *        tacho pulses are not always 50%.
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
 *      - Number of samples is always determined based on 1 sample meaning 1 low period or 1 high period,
 *        this is to enable fast adoption to changes in speed. Medium motor has the critical timing because
 *        it can change speed and direction very fast.
 *
 *      - Large Motor
 *        - Maximum speed of the Large motor is approximately 2mS per tacho pulse (low + high period)
 *          resulting in minimum timer value of: 2mS / (1/(33MHz)) = 66000 T64 timer ticks.
 *          Because 1 sample is based on only half a period minimum speed is 66000/2 = 33000
 *        - Minimum speed of the large motor is a factor of 100 less than max. speed
 *          max. speed timer ticks * 100 => 66000 * 100 = 6,600,000 T64 timer ticks
 *          Because 1 sample is based on only half a period minimum speed is 6,600,000/2 = 3,300,000.
 *
 *      - Medium Motor
 *        - Maximum speed of the medium motor is approximately 1,25mS per tacho pulse (low + high period)
 *          resulting in minimum timer value og: 1,25mS / (1/(33MHz)) = 41250 (approximately)
 *          Because 1 sample is based on only half a period minimum speed is 41250/2 = 20625.
 *        - Minimum speed of the medium motor is a factor of 100 less than max. speed
 *          max. speed timer ticks * 100 => 41250 * 100 = 4,125,000 T64 timer ticks
 *          Because 1 sample is based on only half a period minimum speed is 4,125,000/2 = 2,062,500.
 *
 *      - Actual speed is then calculated as:
 *        - Large motor:
 *          3,300,000 * number of samples / actual time elapsed for number of samples
 *        - Medium motor:
 *          2,062,500 * number of samples / actual time elapsed for number of samples
 *
 *  - Parameters:
 *    - Input:
 *      - No        : Motor output number
 *      - *pSpeed   : Pointer to the speed value
 *
 *    - Output:
 *      - Status    : Indication of new speed available or not
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
 *      DirChgPtr variable is used to indicate how many timer samples have been sampled
 *      since direction has been changed. DirChgPtr is set to 0 when tacho interrupt detects
 *      direction change and then it is counted up for every timer sample. So when DirChgPtr
 *      has the value of 2 then there must be 2 timer samples in the the same direction
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

static bool calculate_speed(struct ev3_tacho_motor_data *ev3_tm)
{
	unsigned DiffIdx;
	unsigned Diff;

	int  speed;

	bool speed_updated = false;

	#warning "Don't run this if we're updating the ev3_tm in the isr!"

	/* Determine the approximate speed of the motor using the difference
	 * in time between this tacho pulse and the previous pulse.
	 *
	 * The old code had a conditional that forced the difference to be at
	 * least 1 by checking for the special case of a zero difference, which
	 * would be almost impossible to acheive in practice.
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

	DiffIdx = ev3_tm->tacho_samples_head;

	#warning "This should really be a boolean value that gets set at the ISR level"
	#warning "Can/Should we change this to not set_samples_per_speed evry time we're called?"

	if (ev3_tm->dir_chg_samples >= 1) {

		Diff = ev3_tm->tacho_samples[DiffIdx] 
				- ev3_tm->tacho_samples[(DiffIdx + TACHO_SAMPLES - 1) % TACHO_SAMPLES];

		Diff |= 1;

		set_samples_per_speed( ev3_tm, ev3_tm->counts_per_pulse / Diff );
	}

	/* Now get a better estimate of the motor speed by using the total
	 * time used to accumulate the last n samples, where n is determined
	 * by the first approximation to the speed.
	 *
	 * The new speed can only be updated if we have accumulated at least
	 * as many samples as are required depending on the estimated speed
	 * of the motor.
	 *
	 * If the speed cannot be updated, then we need to check if the speed
	 * is 0!
	 */

	if (ev3_tm->got_new_sample && (ev3_tm->dir_chg_samples >= ev3_tm->samples_per_speed) ) {

		Diff = ev3_tm->tacho_samples[DiffIdx] 
				- ev3_tm->tacho_samples[(DiffIdx + TACHO_SAMPLES - ev3_tm->samples_per_speed) % TACHO_SAMPLES];

		Diff |= 1;

		speed = (ev3_tm->counts_per_pulse * ev3_tm->samples_per_speed) / Diff;

		/* And do some cleanup to limit the max and get direction right */

		if (speed > MAX_SPEED)
			speed = MAX_SPEED;

		if (ev3_tm->run_direction == REVERSE)
			speed = -speed;

		speed_updated = true;

		ev3_tm->got_new_sample = false;

	} else if ( ev3_tm->counts_per_pulse < (legoev3_hires_timer_read() - ev3_tm->tacho_samples[DiffIdx] ) ) {

		ev3_tm->dir_chg_samples = 0;

		speed = 0;

		speed_updated = true;
	}

	if( speed_updated )
		ev3_tm->speed = speed;

	return(speed_updated);
}

static void regulate_speed(struct ev3_tacho_motor_data *ev3_tm)
{
	int power;
	int speed_error;

	speed_error = ev3_tm->speed_setpoint - ev3_tm->speed;

	#warning "Implement an attribute set for PID constants that adjusts based on speed"

	if (MOTOR_TYPE_MINITACHO == ev3_tm->motor_type) {
		ev3_tm->pid.P = speed_error * 4;
		ev3_tm->pid.I = ((ev3_tm->pid.I * 9)/10) + (speed_error / 3);
		ev3_tm->pid.D = (((speed_error - ev3_tm->pid.oldError) * 4)/2) * 40;

	} else if (MOTOR_TYPE_TACHO == ev3_tm->motor_type) {
		ev3_tm->pid.P = speed_error * 2;
//		ev3_tm->pid.P = speed_error * 8;
		ev3_tm->pid.I = ((ev3_tm->pid.I * 9)/10) + (speed_error / 4);
//		ev3_tm->pid.I = ((ev3_tm->pid.I *98)/100) + (speed_error / 4);
//  		ev3_tm->pid.I = ((ev3_tm->pid.I *95)/100) + (speed_error / 2);
		ev3_tm->pid.D = (((speed_error - ev3_tm->pid.oldError) * 4)/2) * 40;
//		ev3_tm->pid.D = (((speed_error - ev3_tm->pid.oldError) * 4)/2) * 20;

	} else {
		/* This space intentionally left blank! */
	}

	ev3_tm->pid.oldError = speed_error;

	#warning "Maybe we can avoid the scale and rescale of power here and just scale the PID result?"
	power = (ev3_tm->power_setpoint * (MAX_PWM_CNT/100)) + ((ev3_tm->pid.P + ev3_tm->pid.I + ev3_tm->pid.D));

	/* Make sure power is within a reasonable range */

	if (power > MAX_PWM_CNT)
		power = MAX_PWM_CNT;

	if (power < -MAX_PWM_CNT)
		power = -MAX_PWM_CNT;

	#warning "Is this necessary? It messes up changes in direction!"
	/* Make sure power is not fighting direction */
	
//	if (0 > (ev3_tm->speed_setpoint * power) )
//		power = 0;

	if (0 == ev3_tm->timer_foo) {
		printk( "regulate: power %d speed %d speed-setpoint %d error %d p %d i %d d %d pid %d\n", power, ev3_tm->speed, ev3_tm->speed_setpoint, speed_error, ev3_tm->pid.P, ev3_tm->pid.I, ev3_tm->pid.D, ((ev3_tm->pid.P + ev3_tm->pid.I + ev3_tm->pid.D)) );
        }
	
	ev3_tacho_motor_set_power(ev3_tm, (power * 100) / MAX_PWM_CNT);

//printk( "New Regulation Power is %d - %d\n", ev3_tm->power, (ev3_tm->power * 100) / MAX_PWM_CNT);
}



#define RAMP_SLOPE_FACTOR 10000

#define RAMP_MODE_TIME  1
#define RAMP_MODE_TACHO 2

static int calculate_ramp_slope(int start, int end, int diff)
{
	/* Return a scaled ramp factor that represents the slope. Always use the
	 * corresponding function to figure out the new value on the slope line!
	 */

	return( (diff * RAMP_SLOPE_FACTOR) / (end - start ) );
}

// static void ev3_tacho_motor_get_compare_counts( struct ev3_tacho_motor_data *ev3_tm, int *ramp_count, int *ramp_count_test )
// {
// #warning "ev3_tacho_motor_get_compare_counts is not yet implemented!"
// 	*ramp_count      = 0;
// 	*ramp_count_test = 0;
// }
// static bool ev3_tacho_motor_check_less_than_special( int v1, int v2, int v3 )
// {
// #warning "ev3_tacho_motor_check_less_than_special is not yet implemented!"
// 	return 0;
// }
// static bool ev3_tacho_motor_step_speed_check_tacho_count_const( struct ev3_tacho_motor_data *ev3_tm, int tacho_cnt )
// {
// #warning "ev3_tacho_motor_step_speed_check_tacho_count_const is not yet implemented!"
// 	return 0;
// }
// static bool ev3_tacho_motor_step_speed_check_tacho_count_down( struct ev3_tacho_motor_data *ev3_tm, int tacho_cnt )
// {
// #warning "ev3_tacho_motor_step_speed_check_tacho_count_down is not yet implemented!"
// 	return 0;
// }

/* Return a value from 0 to 100 representing the percentage of the ramp that is complete */

static int calculate_ramp_up_progress( int start, int end, int now )
{
	int progress = (((now - start) * 100) / (end - start));
	
	if (progress < 0)
		progress = 0;

	if (progress > 100)
		progress = 100;

	return progress;
}

static int calculate_ramp_down_progress( int start, int end, int now )
{
	int progress = (((end - now) * 100) / (end - start));

	if (progress < 0)
		progress = 0;

	if (progress > 100)
		progress = 100;

	return progress;
}

static enum hrtimer_restart ev3_tacho_motor_timer_callback(struct hrtimer *timer)
{
 	struct ev3_tacho_motor_data *ev3_tm =
 			container_of(timer, struct ev3_tacho_motor_data, timer);

// 	/* FIXME - these need better names */
// 
// 	int tmp_tacho;
// 	int tmp;
// 
 	int speed;
// 
// 	/* These get use in the ramp modes */
// 
// 	bool ramp_complete;
// 	int  ramp_count;
// 	int  ramp_count_test;

	int ramp_progress;
	int setpoint;

	bool reprocess = true;

 	hrtimer_forward_now(timer, ktime_set(0, TACHO_MOTOR_POLL_NS));

        /* Here's where the business end of things starts - update the tacho data that's
         *   shared with the world
         */

// 	tmp_tacho = ev3_tm->irq_tacho;
// 	tmp       = tmp_tacho - ev3_tm->old_tacho_cnt;
// 
// 	ev3_tm->tacho_cnt    += tmp;
// 	ev3_tm->tacho_sensor += tmp;
// 	ev3_tm->old_tacho_cnt = tmp_tacho;
// 
// 	ev3_tm->time_cnt += ev3_tm->time_inc;
// 
// 	/* Early exit from function if someone is reading the struct! */
// 
// 	if (ev3_tm->mutex )
// 		return HRTIMER_RESTART;

	/* Continue with the actual calculations */

	speed = calculate_speed( ev3_tm );

	if ( !ev3_tm->run )
		goto no_run;

	// regulate_speed( ev3_tm );

	/* Update the ramp counter if we're in any of the ramp modes.
	 *
	 * This has to be done outside of the main state processing 
	 * loop, otherwise we can end up updating hte counter multiple
	 * times.
	 */

	switch (ev3_tm->state) {

	case RAMP_UP:
	case RAMP_CONST:
//	case RAMP_DOWN:	 ev3_tm->ramp.count++;
	case RAMP_DOWN:	 if (ev3_tm->ramp_mode == RAMP_TIME)
				#warning "Make the tick factor of 2 a compile-time #define to make changes easier"
				ev3_tm->ramp.count = ev3_tm->ramp.count + 2;
			 else if (ev3_tm->ramp_mode == RAMP_TACHO)
				ev3_tm->ramp.count = ev3_tm->ramp.setpoint_sign * ev3_tm->irq_tacho;

			ev3_tm->timer_foo = (++ev3_tm->timer_foo % 50);
			
			#warning "This is a temporary fudge to nudge a stalled motor"
			if ((0 == ev3_tm->timer_foo) && (8 > abs(ev3_tm->speed)))
				ev3_tm->ramp.offset = ev3_tm->ramp.offset + ev3_tm->ramp.setpoint_sign;

	default: break;
	}

	while ( reprocess ) {

		/* Some cases (such as RAMP_XXX) may change the state of the
		 * handler and require reprocessing. If so, they must set the
		 * reprocess flag to force an extra evaluation
		 */

		reprocess = 0;	

		switch (ev3_tm->state) {
	
	        case UNLIMITED_UNREG:
			#warning "Consider folding this code into ev3_tacho_motor_set_power"
			if (ev3_tm->target_power != ev3_tm->power_setpoint ) {
				ev3_tacho_motor_set_power( ev3_tm, ev3_tm->target_power );
			}
			break;
	
		case UNLIMITED_REG:
			#warning "Consider folding this code into ev3_tacho_motor_regulate_speed"
			if (ev3_tm->target_speed != ev3_tm->speed_setpoint ) {
				ev3_tm->speed_setpoint = ev3_tm->target_speed;
			}
			break;
	
		case SETUP_RAMP_TIME:
			ev3_tm->ramp_up.start   = 0;
			ev3_tm->ramp_up.end     = ev3_tm->target_ramp_up_count;

			ev3_tm->ramp_down.start = (ev3_tm->target_time - ev3_tm->target_ramp_down_count);
			ev3_tm->ramp_down.end   = ev3_tm->target_time;

			ev3_tm->ramp.setpoint_sign = 0;

			ev3_tm->state = SETUP_RAMP_REGULATION;
			reprocess = true;
			break;

		case SETUP_RAMP_ABSOLUTE_TACHO:
			ev3_tm->ramp_up.start   = 0;
			ev3_tm->ramp_up.end     = ev3_tm->target_ramp_up_count;

			ev3_tm->ramp_down.start = abs(ev3_tm->target_tacho - ev3_tm->tacho) - ev3_tm->target_ramp_down_count;
			ev3_tm->ramp_down.end   = abs(ev3_tm->target_tacho - ev3_tm->tacho);

			/* Now adjust the ramp points based on direction */

			if (ev3_tm->ramp_up.end > abs(ev3_tm->target_tacho - ev3_tm->tacho))
				ev3_tm->ramp_up.end = ((abs(ev3_tm->target_tacho - ev3_tm->tacho) * 1) / 4);

			if (ev3_tm->ramp_down.start < ev3_tm->ramp_up.end)
				ev3_tm->ramp_down.start = ev3_tm->ramp_up.end;

			if ((ev3_tm->target_tacho - ev3_tm->tacho) >= 0 )
				ev3_tm->ramp.setpoint_sign =  1;
			else
				ev3_tm->ramp.setpoint_sign = -1;

			ev3_tm->state = SETUP_RAMP_REGULATION;
			reprocess = true;
			break;

				
		case SETUP_RAMP_RELATIVE_TACHO:
			ev3_tm->ramp_up.start   = 0;
			ev3_tm->ramp_up.end     = ev3_tm->target_ramp_up_count;

			ev3_tm->ramp_down.start = abs(ev3_tm->target_tacho) - ev3_tm->target_ramp_down_count;
			ev3_tm->ramp_down.end   = abs(ev3_tm->target_tacho);
			/* Now adjust the ramp points based on direction */

			if (ev3_tm->ramp_up.end > abs(ev3_tm->target_tacho))
				ev3_tm->ramp_up.end = ((abs(ev3_tm->target_tacho) * 1) / 4);

			if (ev3_tm->ramp_down.start < ev3_tm->ramp_up.end)
				ev3_tm->ramp_down.start = ev3_tm->ramp_up.end;

			if ((ev3_tm->target_tacho) >= 0 )
				ev3_tm->ramp.setpoint_sign =  1;
			else
				ev3_tm->ramp.setpoint_sign = -1;

			ev3_tm->state = SETUP_RAMP_REGULATION;
			reprocess = true;
			break;

		case SETUP_RAMP_REGULATION:
			if (REGULATION_OFF == ev3_tm->regulation_mode) {
				ev3_tm->ramp.offset   = 0;
				ev3_tm->ramp.setpoint = ev3_tm->target_power;
			} else {
				ev3_tm->ramp.offset   = 8;
				ev3_tm->ramp.setpoint = ev3_tm->target_speed;
			}

			if (ev3_tm->ramp_mode == RAMP_TACHO) {
				ev3_tm->ramp.offset   = ev3_tm->ramp.setpoint_sign *     ev3_tm->ramp.offset;
				ev3_tm->ramp.setpoint = ev3_tm->ramp.setpoint_sign * abs(ev3_tm->ramp.setpoint);
				ev3_tm->ramp.setpoint = ev3_tm->ramp.setpoint - ev3_tm->ramp.offset;
			}

			ev3_tm->ramp.count    = 0;

			ev3_tm->state = RAMP_UP;
			reprocess = true;
			break;

			/* The LIMITED_XXX functions have to handle the three phases (any of
		 * which are optional) of a motor move operation. It is assumed that
		 * when the run mode was set, the ramp factors were calculated.
		 *
		 * The LIMITED_XXX functions need to handle the following combinations:
		 *
		 * REGULATED_TIME    - Speed is regulated, ramping is time based
		 * REGULATED_TACHO   - Speed is regulated, ramping is tacho based
		 * UNREGULATED_TIME  - Speed is not regulated, ramping is time based
		 * UNREGULATED_TACHO - Speed is not regulated, ramping is tacho based
		 *
		 * When ramping, the code needs to figure out which combination is in
		 * use, and that's handled by a couple of booleans in the motor struct.
		 *
		 * Regardless of the direction of the ramp (up or down), the first part
		 * of the sequence is ramping up, the tail end of the sequence is
		 * ramping down.
		 */
	
		case RAMP_UP:
			/* Figure out if we're done ramping up - if yes set state to RAMP_CONST
			 * and allow states to get reprocessed
			 */
	
			if ( ev3_tm->ramp_up.end <= ev3_tm->ramp.count ) {
				ev3_tm->state = RAMP_CONST;
				reprocess = true;
				printk( "Switching to RAMP_CONST @ %d\n", ev3_tm->ramp.count );
			} else {
		
				/* Figure out how far along we are in the ramp operation */
				
				ramp_progress = calculate_ramp_up_progress( ev3_tm->ramp_up.start, ev3_tm->ramp_up.end, ev3_tm->ramp.count );
		
				/* Now determine the new setpoint for the ramp */

				setpoint = ev3_tm->ramp.offset + (ramp_progress * (ev3_tm->ramp.setpoint - ev3_tm->ramp.offset)) / 100;		
	
				if (REGULATION_OFF == ev3_tm->regulation_mode)
					ev3_tacho_motor_set_power( ev3_tm, setpoint );
				else
					ev3_tm->speed_setpoint = setpoint;

//				#warning( "Combine this check into the progress calc, rename to calculate_ramp_setpoint" );
//				setpoint = ev3_tm->ramp.offset + ev3_tm->ramp.setpoint;
//				/* Check the setpoint for minimum and maximum range, and adjust */
//				setpoint = setpoint + 0;
//		
//				/* Set either power or speed based on whether or not we're regulating */
//		
//				if (ev3_tm->regulated) {
//					ev3_tm->speed = setpoint;
//					regulate_speed( ev3_tm );
//				} else {
//					ev3_tm->power = setpoint;
//					ev3_tacho_motor_set_power( ev3_tm, ev3_tm->power );
//				}
			}
			break;
	
		case RAMP_CONST:
			/* Figure out if we're done with the const section - if yes set state to RAMP_DOWN
			 * and allow states to get reprocessed
			 */
	
			if ( ev3_tm->ramp_down.start <= ev3_tm->ramp.count ) {
				ev3_tm->state = RAMP_DOWN;
				reprocess = true;
				printk( "Switching to RAMP_DOWN @ %d\n", ev3_tm->ramp.count );
			} else {
				setpoint = ev3_tm->ramp.offset + ev3_tm->ramp.setpoint;		

				if (REGULATION_OFF == ev3_tm->regulation_mode)
					ev3_tacho_motor_set_power( ev3_tm, setpoint );
				else
					ev3_tm->speed_setpoint = setpoint;

				/* Figure out how far along we are in the ramp operation */
//
//				switch (ev3_tm->ramp_mode) {
//	
//				case RAMP_MODE_TIME:
//			
//				case RAMP_MODE_TACHO:
//
//				default: break;
//				}
			}
			break;
	
		case RAMP_DOWN:
			/* Figure out if we're done ramping up - if yes then 
			 * decide whether to brake, coast, or leave the motor
			 * unchanged, and allow states to get reprocessed
			 */
	
			if ( ev3_tm->ramp_down.end <= ev3_tm->ramp.count ) {
				ev3_tm->state = STOP_MOTOR;
				reprocess = true;
				printk( "Switching to STOP @ %d\n", ev3_tm->ramp.count );
			} else {
				/* Figure out how far along we are in the ramp operation */
				
				ramp_progress = calculate_ramp_down_progress( ev3_tm->ramp_down.start, ev3_tm->ramp_down.end, ev3_tm->ramp.count );
		
				setpoint = ev3_tm->ramp.offset + (ramp_progress * (ev3_tm->ramp.setpoint - ev3_tm->ramp.offset)) / 100;		

				if (REGULATION_OFF == ev3_tm->regulation_mode)
					ev3_tacho_motor_set_power( ev3_tm, setpoint );
				else
					ev3_tm->speed_setpoint = setpoint;

//				/* Now determine the new setpoint for the ramp */
//		
//				#warning( "Combine this check into the progress calc, rename to calculate_ramp_setpoint" );
//				setpoint = ev3_tm->ramp.offset + ((ramp_progress * ev3_tm->ramp_down.slope)/RAMP_SLOPE_FACTOR);
//				/* Check the setpoint for minimum and maximum range, and adjust */
//				setpoint = setpoint + 0;
//		
//				/* Set either power or speed based on whether or not we're regulating */
//		
//				if (ev3_tm->regulated) {
//					ev3_tm->speed = setpoint;
//					regulate_speed( ev3_tm );
////				} else {
//					ev3_tm->power = setpoint;
//					ev3_tacho_motor_set_power( ev3_tm, ev3_tm->power );
//				}
			}
			break;
	
	          case STOP_MOTOR:
	          {	
			ev3_tacho_motor_set_power( ev3_tm, 0 );

			reprocess     = true;
			ev3_tm->state = IDLE;
	          }
	          break;

	          case IDLE:
	          {
			ev3_tm->run        = 0;
	          }
	          break;
	  	default:
	          { /* Intentionally left empty */
				printk( "UNHANDLED MOTOR STATE %d\n", ev3_tm->state );
	          }
	 	break;
	 	}
	}

	if (ev3_tm->run && (REGULATION_ON == ev3_tm->regulation_mode))
		regulate_speed( ev3_tm );

no_run:
	return HRTIMER_RESTART;
}


	
/* FIXME: All these getters and setters can be macrofied!! */

static int ev3_tacho_motor_get_tacho(struct tacho_motor_device *tm)
{
	struct ev3_tacho_motor_data *ev3_tm =
			container_of(tm, struct ev3_tacho_motor_data, tm);

	return ev3_tm->tacho + ev3_tm->irq_tacho;
}

static int ev3_tacho_motor_get_direction(struct tacho_motor_device *tm)
{
	struct ev3_tacho_motor_data *ev3_tm =
			container_of(tm, struct ev3_tacho_motor_data, tm);

	return ev3_tm->run_direction;
}

static int ev3_tacho_motor_get_speed(struct tacho_motor_device *tm)
{
	struct ev3_tacho_motor_data *ev3_tm =
			container_of(tm, struct ev3_tacho_motor_data, tm);

	// return ev3_tm->speed;
	return ev3_tm->speed;
}

static int ev3_tacho_motor_get_power(struct tacho_motor_device *tm)
{
	struct ev3_tacho_motor_data *ev3_tm =
			container_of(tm, struct ev3_tacho_motor_data, tm);

	return ev3_tm->power_setpoint;
}

static int ev3_tacho_motor_get_time(struct tacho_motor_device *tm)
{
	struct ev3_tacho_motor_data *ev3_tm =
			container_of(tm, struct ev3_tacho_motor_data, tm);

	return ev3_tm->time;
}

static int ev3_tacho_motor_get_state(struct tacho_motor_device *tm)
{
	struct ev3_tacho_motor_data *ev3_tm =
			container_of(tm, struct ev3_tacho_motor_data, tm);

	return ev3_tm->state;
}


static int ev3_tacho_motor_get_stop_mode(struct tacho_motor_device *tm)
{
	struct ev3_tacho_motor_data *ev3_tm =
			container_of(tm, struct ev3_tacho_motor_data, tm);

	return ev3_tm->stop_mode;
}

static void ev3_tacho_motor_set_stop_mode(struct tacho_motor_device *tm, unsigned int stop_mode)
{
	struct ev3_tacho_motor_data *ev3_tm =
			container_of(tm, struct ev3_tacho_motor_data, tm);

	ev3_tm->stop_mode = stop_mode;
}

static int ev3_tacho_motor_get_regulation_mode(struct tacho_motor_device *tm)
{
	struct ev3_tacho_motor_data *ev3_tm =
			container_of(tm, struct ev3_tacho_motor_data, tm);

	return ev3_tm->regulation_mode;
}

static void ev3_tacho_motor_set_regulation_mode(struct tacho_motor_device *tm, unsigned int regulation_mode)
{
	struct ev3_tacho_motor_data *ev3_tm =
			container_of(tm, struct ev3_tacho_motor_data, tm);

	ev3_tm->regulation_mode = regulation_mode;
}

static int ev3_tacho_motor_get_tacho_mode(struct tacho_motor_device *tm)
{
	struct ev3_tacho_motor_data *ev3_tm =
			container_of(tm, struct ev3_tacho_motor_data, tm);

	return ev3_tm->tacho_mode;
}

static void ev3_tacho_motor_set_tacho_mode(struct tacho_motor_device *tm, unsigned int tacho_mode)
{
	struct ev3_tacho_motor_data *ev3_tm =
			container_of(tm, struct ev3_tacho_motor_data, tm);

	ev3_tm->tacho_mode = tacho_mode;
}

static int ev3_tacho_motor_get_ramp_mode(struct tacho_motor_device *tm)
{
	struct ev3_tacho_motor_data *ev3_tm =
			container_of(tm, struct ev3_tacho_motor_data, tm);

	return ev3_tm->ramp_mode;
}

static void ev3_tacho_motor_set_ramp_mode(struct tacho_motor_device *tm, unsigned int ramp_mode)
{
	struct ev3_tacho_motor_data *ev3_tm =
			container_of(tm, struct ev3_tacho_motor_data, tm);

	ev3_tm->ramp_mode = ramp_mode;
}

static int ev3_tacho_motor_get_tacho_type(struct tacho_motor_device *tm)
{
	struct ev3_tacho_motor_data *ev3_tm =
			container_of(tm, struct ev3_tacho_motor_data, tm);

	if (ev3_tm->motor_type == MOTOR_MINITACHO)
		return TACHO_TYPE_MINITACHO;
	else if (ev3_tm->motor_type == MOTOR_TACHO)
		return TACHO_TYPE_TACHO;
	else
		return TACHO_TYPE_TACHO;
}

static void ev3_tacho_motor_set_tacho_type(struct tacho_motor_device *tm, unsigned int tacho_type)
{
	struct ev3_tacho_motor_data *ev3_tm =
			container_of(tm, struct ev3_tacho_motor_data, tm);

	struct ev3_motor_platform_data *pdata = ev3_tm->motor_port->dev.platform_data; 
	
	if (tacho_type == TACHO_TYPE_TACHO)
		ev3_tm->motor_type = MOTOR_TACHO;
	else if (tacho_type == TACHO_TYPE_MINITACHO)
		ev3_tm->motor_type = MOTOR_MINITACHO;
}

static int ev3_tacho_motor_get_target_power(struct tacho_motor_device *tm)
{
	struct ev3_tacho_motor_data *ev3_tm =
			container_of(tm, struct ev3_tacho_motor_data, tm);

	return ev3_tm->target_power;
}

static void ev3_tacho_motor_set_target_power(struct tacho_motor_device *tm, long target_power)
{
	struct ev3_tacho_motor_data *ev3_tm =
			container_of(tm, struct ev3_tacho_motor_data, tm);

	struct ev3_motor_platform_data *pdata = ev3_tm->motor_port->dev.platform_data; 

	ev3_tm->target_power = target_power;
}

static int ev3_tacho_motor_get_target_tacho(struct tacho_motor_device *tm)
{
	struct ev3_tacho_motor_data *ev3_tm =
			container_of(tm, struct ev3_tacho_motor_data, tm);

	return ev3_tm->target_tacho;
}

static void ev3_tacho_motor_set_target_tacho(struct tacho_motor_device *tm, long target_tacho)
{
	struct ev3_tacho_motor_data *ev3_tm =
			container_of(tm, struct ev3_tacho_motor_data, tm);

	struct ev3_motor_platform_data *pdata = ev3_tm->motor_port->dev.platform_data; 

	int err;

	ev3_tm->target_tacho = target_tacho;
}

static int ev3_tacho_motor_get_target_speed(struct tacho_motor_device *tm)
{
	struct ev3_tacho_motor_data *ev3_tm =
			container_of(tm, struct ev3_tacho_motor_data, tm);

	return ev3_tm->target_speed;
}

static void ev3_tacho_motor_set_target_speed(struct tacho_motor_device *tm, long target_speed)
{
	struct ev3_tacho_motor_data *ev3_tm =
			container_of(tm, struct ev3_tacho_motor_data, tm);

	struct ev3_motor_platform_data *pdata = ev3_tm->motor_port->dev.platform_data; 

	int err;

	ev3_tm->target_speed = target_speed;
}

static int ev3_tacho_motor_get_target_steer(struct tacho_motor_device *tm)
{
	struct ev3_tacho_motor_data *ev3_tm =
			container_of(tm, struct ev3_tacho_motor_data, tm);

	return ev3_tm->target_steer;
}

static void ev3_tacho_motor_set_target_steer(struct tacho_motor_device *tm, long target_steer)
{
	struct ev3_tacho_motor_data *ev3_tm =
			container_of(tm, struct ev3_tacho_motor_data, tm);

	struct ev3_motor_platform_data *pdata = ev3_tm->motor_port->dev.platform_data; 

	int err;

	ev3_tm->target_steer = target_steer;
}

static int ev3_tacho_motor_get_target_time(struct tacho_motor_device *tm)
{
	struct ev3_tacho_motor_data *ev3_tm =
			container_of(tm, struct ev3_tacho_motor_data, tm);

	return ev3_tm->target_time;
}

static void ev3_tacho_motor_set_target_time(struct tacho_motor_device *tm, long target_time)
{
	struct ev3_tacho_motor_data *ev3_tm =
			container_of(tm, struct ev3_tacho_motor_data, tm);

	struct ev3_motor_platform_data *pdata = ev3_tm->motor_port->dev.platform_data; 

	int err;

	ev3_tm->target_time = target_time;
}

static int ev3_tacho_motor_get_target_ramp_up_count(struct tacho_motor_device *tm)
{
	struct ev3_tacho_motor_data *ev3_tm =
			container_of(tm, struct ev3_tacho_motor_data, tm);

	return ev3_tm->target_ramp_up_count;
}

static void ev3_tacho_motor_set_target_ramp_up_count(struct tacho_motor_device *tm, long target_ramp_up_count)
{
	struct ev3_tacho_motor_data *ev3_tm =
			container_of(tm, struct ev3_tacho_motor_data, tm);

	struct ev3_motor_platform_data *pdata = ev3_tm->motor_port->dev.platform_data; 

	int err;

	ev3_tm->target_ramp_up_count = target_ramp_up_count;
}

static int ev3_tacho_motor_get_target_total_count(struct tacho_motor_device *tm)
{
	struct ev3_tacho_motor_data *ev3_tm =
			container_of(tm, struct ev3_tacho_motor_data, tm);

	return ev3_tm->target_total_count;
}

static void ev3_tacho_motor_set_target_total_count(struct tacho_motor_device *tm, long target_total_count)
{
	struct ev3_tacho_motor_data *ev3_tm =
			container_of(tm, struct ev3_tacho_motor_data, tm);

	struct ev3_motor_platform_data *pdata = ev3_tm->motor_port->dev.platform_data; 

	int err;

	ev3_tm->target_total_count = target_total_count;
}

static int ev3_tacho_motor_get_target_ramp_down_count(struct tacho_motor_device *tm)
{
	struct ev3_tacho_motor_data *ev3_tm =
			container_of(tm, struct ev3_tacho_motor_data, tm);

	return ev3_tm->target_ramp_down_count;
}

static void ev3_tacho_motor_set_target_ramp_down_count(struct tacho_motor_device *tm, long target_ramp_down_count)
{
	struct ev3_tacho_motor_data *ev3_tm =
			container_of(tm, struct ev3_tacho_motor_data, tm);

	struct ev3_motor_platform_data *pdata = ev3_tm->motor_port->dev.platform_data; 

	int err;

	ev3_tm->target_ramp_down_count = target_ramp_down_count;
}

static int ev3_tacho_motor_get_mode(struct tacho_motor_device *tm)
{
	struct ev3_tacho_motor_data *ev3_tm =
			container_of(tm, struct ev3_tacho_motor_data, tm);

	return ev3_tm->mode;
}

static void ev3_tacho_motor_set_mode(struct tacho_motor_device *tm, long mode)
{
	struct ev3_tacho_motor_data *ev3_tm =
			container_of(tm, struct ev3_tacho_motor_data, tm);

	struct ev3_motor_platform_data *pdata = ev3_tm->motor_port->dev.platform_data; 

	int err;

	ev3_tm->mode = mode;
}

static int ev3_tacho_motor_get_run(struct tacho_motor_device *tm)
{
	struct ev3_tacho_motor_data *ev3_tm =
			container_of(tm, struct ev3_tacho_motor_data, tm);

	return ev3_tm->run;
}

static void ev3_tacho_motor_set_run(struct tacho_motor_device *tm, long run)
{
	struct ev3_tacho_motor_data *ev3_tm =
			container_of(tm, struct ev3_tacho_motor_data, tm);

	if ( 0 == run )
		ev3_tm->state = STOP_MOTOR;

	else if ((RAMP_OFF == ev3_tm->ramp_mode) && (REGULATION_OFF == ev3_tm->regulation_mode))
		ev3_tm->state = UNLIMITED_UNREG;

	else if ((RAMP_OFF == ev3_tm->ramp_mode) && (REGULATION_ON  == ev3_tm->regulation_mode))
		ev3_tm->state = UNLIMITED_REG;

	else if ((RAMP_TIME == ev3_tm->ramp_mode))
		ev3_tm->state = SETUP_RAMP_TIME;

	else if ((RAMP_TACHO == ev3_tm->ramp_mode) && (TACHO_ABSOLUTE == ev3_tm->tacho_mode))
		ev3_tm->state = SETUP_RAMP_ABSOLUTE_TACHO;

	else if ((RAMP_TACHO == ev3_tm->ramp_mode) && (TACHO_RELATIVE == ev3_tm->tacho_mode))
		ev3_tm->state = SETUP_RAMP_RELATIVE_TACHO;

	else
		ev3_tm->state = IDLE;

//	int direction = 0;

	ev3_tm->tacho     += ev3_tm->irq_tacho;
	ev3_tm->irq_tacho  = 0;
//
//	#warning "FIXME: this needs some tuning to make sure start/end/setpoint don't fight!"
//
//	/* Sort out which direction our target is taking us in, then prepare to ramp! */
//
//	if (ev3_tm->target_tacho > ev3_tm->tacho)
//		direction = 1;
//	else if (ev3_tm->target_tacho > ev3_tm->tacho)
//		direction = -1;
//	
//	if (ev3_tm->target_tacho > ev3_tm->tacho) {
//		ev3_tm->ramp.setpoint= 75;
//		ev3_tm->ramp.offset  = 6;
//		ev3_tm->ramp.count   = 0;
//
//		ev3_tm->ramp_up.start   = 0;
//		ev3_tm->ramp_up.end     = ev3_tm->target_ramp_up_count;
//
//		ev3_tm->ramp_down.start = (1)*(ev3_tm->target_tacho - ev3_tm->tacho) - ev3_tm->target_ramp_down_count;
//		ev3_tm->ramp_down.end   = (1)*(ev3_tm->target_tacho - ev3_tm->tacho);
//
//		if ( (ev3_tm->tacho + ev3_tm->ramp_up.end) > ev3_tm->target_tacho )
//			ev3_tm->ramp_up.end = (((ev3_tm->target_tacho - ev3_tm->tacho) * 1) / 4 );
//		
//		if ( (ev3_tm->target_tacho - ev3_tm->ramp_down.start) < ev3_tm->tacho )
//			ev3_tm->ramp_down.start = 0 ;			
//	} else {
//		ev3_tm->ramp.setpoint= (-1)*75;
//		ev3_tm->ramp.offset  = (-1)*6;
//		ev3_tm->ramp.count   =      0;
//
//		ev3_tm->ramp_up.start   =     0;
//		ev3_tm->ramp_up.end     =     ev3_tm->target_ramp_up_count;
//
//		ev3_tm->ramp_down.start = (-1)*(ev3_tm->target_tacho - ev3_tm->tacho) - ev3_tm->target_ramp_down_count;
//		ev3_tm->ramp_down.end   = (-1)*(ev3_tm->target_tacho - ev3_tm->tacho);
//
//		if ( (ev3_tm->tacho - ev3_tm->ramp_up.end) < ev3_tm->target_tacho )
//			ev3_tm->ramp_up.end = (((ev3_tm->target_tacho - ev3_tm->tacho) * 1) / 4 );
//		
//		if ( (ev3_tm->target_tacho + ev3_tm->ramp_down.start) > ev3_tm->tacho )
//			ev3_tm->ramp_down.start = 0;
//	}
//	
//	ev3_tm->state  = RAMP_UP;
//
	printk( "Set new state to %d\n", ev3_tm->state );

	/* What's going on here - why is run always set to 1? 
	 *
	 * The answer is that we check for run == 0 as the first condition
	 * at the top of this function. If it's set, then the next motor state
	 * is STOP_MOTOR, but it won't be evaluated if run == 0
	 *
	 * So we always force the state machine to run once, and count on the
	 * state machine to DTRT (Do The Right Thing) This avoids setting motor
	 * power in wierd places
	 */

	ev3_tm->run = 1;


//	ev3_tm->ramp_up.start   = 0;
//	ev3_tm->ramp_up.end     = 90;
//	ev3_tm->ramp_down.start = ev3_tm->target_tacho - ev3_tm->tacho - 90;
//	ev3_tm->ramp_down.end   = ev3_tm->target_tacho - ev3_tm->tacho;


}

static int __devinit ev3_tacho_motor_probe(struct legoev3_port_device *motor)
{
	struct ev3_tacho_motor_data *ev3_tm;
	struct ev3_motor_platform_data *pdata = motor->dev.platform_data;
	int err;

	if (WARN_ON(!pdata))
		return -EINVAL;

	ev3_tm = kzalloc(sizeof(struct ev3_tacho_motor_data), GFP_KERNEL);

	if (!ev3_tm)
		return -ENOMEM;

	ev3_tm->out_port   = pdata->out_port;
	ev3_tm->motor_port = motor;

	if (pdata->motor_type == MOTOR_MINITACHO)
		ev3_tm->motor_type = MOTOR_MINITACHO;
	else if (pdata->motor_type == MOTOR_TACHO)
		ev3_tm->motor_type = MOTOR_TACHO;
	else
		ev3_tm->motor_type = MOTOR_TACHO;

        /* Set up motor specific initialization constants */
        /* FIXME: These need to be motor specific later!  */
        /* Maybe a better way is to have this set up as a constant array that's memcpy'ed */

	ev3_tm->tm.get_tacho     = ev3_tacho_motor_get_tacho;
	ev3_tm->tm.get_direction = ev3_tacho_motor_get_direction;
	ev3_tm->tm.get_speed     = ev3_tacho_motor_get_speed;
	ev3_tm->tm.get_power     = ev3_tacho_motor_get_power;
	ev3_tm->tm.get_time      = ev3_tacho_motor_get_time;
	ev3_tm->tm.get_state     = ev3_tacho_motor_get_state;

	ev3_tm->tm.get_stop_mode       = ev3_tacho_motor_get_stop_mode;
	ev3_tm->tm.set_stop_mode       = ev3_tacho_motor_set_stop_mode;

	ev3_tm->tm.get_regulation_mode = ev3_tacho_motor_get_regulation_mode;
	ev3_tm->tm.set_regulation_mode = ev3_tacho_motor_set_regulation_mode;

	ev3_tm->tm.get_tacho_mode      = ev3_tacho_motor_get_tacho_mode;
	ev3_tm->tm.set_tacho_mode      = ev3_tacho_motor_set_tacho_mode;

	ev3_tm->tm.get_ramp_mode       = ev3_tacho_motor_get_ramp_mode;
	ev3_tm->tm.set_ramp_mode       = ev3_tacho_motor_set_ramp_mode;

	ev3_tm->tm.get_tacho_type      = ev3_tacho_motor_get_tacho_type;
	ev3_tm->tm.set_tacho_type      = ev3_tacho_motor_set_tacho_type;


	ev3_tm->tm.get_target_power    = ev3_tacho_motor_get_target_power;
	ev3_tm->tm.set_target_power    = ev3_tacho_motor_set_target_power;

	ev3_tm->tm.get_target_tacho    = ev3_tacho_motor_get_target_tacho;
	ev3_tm->tm.set_target_tacho    = ev3_tacho_motor_set_target_tacho;

	ev3_tm->tm.get_target_speed    = ev3_tacho_motor_get_target_speed;
	ev3_tm->tm.set_target_speed    = ev3_tacho_motor_set_target_speed;

	ev3_tm->tm.get_target_steer    = ev3_tacho_motor_get_target_steer;
	ev3_tm->tm.set_target_steer    = ev3_tacho_motor_set_target_steer;

	ev3_tm->tm.get_target_time     = ev3_tacho_motor_get_target_time;
	ev3_tm->tm.set_target_time     = ev3_tacho_motor_set_target_time;

	ev3_tm->tm.get_target_ramp_up_count   = ev3_tacho_motor_get_target_ramp_up_count;
	ev3_tm->tm.set_target_ramp_up_count   = ev3_tacho_motor_set_target_ramp_up_count;

	ev3_tm->tm.get_target_total_count     = ev3_tacho_motor_get_target_total_count;
	ev3_tm->tm.set_target_total_count     = ev3_tacho_motor_set_target_total_count;

	ev3_tm->tm.get_target_ramp_down_count = ev3_tacho_motor_get_target_ramp_down_count;
	ev3_tm->tm.set_target_ramp_down_count = ev3_tacho_motor_set_target_ramp_down_count;

	ev3_tm->tm.get_run     = ev3_tacho_motor_get_run;
	ev3_tm->tm.set_run     = ev3_tacho_motor_set_run;

	ev3_tm->state             = IDLE;
	ev3_tm->samples_per_speed = SamplesPerSpeed[MOTOR_TYPE_TACHO][SAMPLES_PER_SPEED_BELOW_40];
	ev3_tm->counts_per_pulse  = CountsPerPulse[MOTOR_TYPE_TACHO];

        /* FIXME: These need to be motor specific later! */

	err = register_tacho_motor(&ev3_tm->tm, &motor->dev);
	if (err)
		goto register_tacho_motor_fail;

	err = dev_set_drvdata(&motor->dev, ev3_tm);
	if (err)
		goto dev_set_drvdata_fail;

	dev_info(&motor->dev, "Tacho Motor connected to port %s gpio %d irq %d\n",
		dev_name(&ev3_tm->out_port->dev),
                pdata->tacho_int_gpio,
                gpio_to_irq(pdata->tacho_int_gpio));

        // Here's where we set up the port pins on a per-port basis
        
        if(request_irq(gpio_to_irq(pdata->tacho_int_gpio), tacho_motor_isr, 0, dev_name(&ev3_tm->out_port->dev), ev3_tm ))
		goto dev_request_irq_fail;

        irq_set_irq_type(gpio_to_irq(pdata->tacho_int_gpio), IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING);

	/* Set up the output port status processing timer */

	hrtimer_init(&ev3_tm->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	ev3_tm->timer.function = ev3_tacho_motor_timer_callback;
	hrtimer_start(&ev3_tm->timer, ktime_set(0, TACHO_MOTOR_POLL_NS),
		      HRTIMER_MODE_REL);

	return 0;

dev_request_irq_fail:
	dev_set_drvdata(&motor->dev, NULL);

dev_set_drvdata_fail:
	unregister_tacho_motor(&ev3_tm->tm);

register_tacho_motor_fail:
	kfree(ev3_tm);

	return err;
}
// int ev3_output_port_request_irq(struct legoev3_port_device *motor, unsigned int type, irq_handler_t handler, void *id)
// {
// 	struct ev3_output_port_data *port =
// 			container_of(motor, struct ev3_output_port_data, motor);
// 
//         pr_warning("GPIO_PIN1       for port %d is: %d\n", port->id, port->gpio[GPIO_PIN1].gpio);
//         pr_warning("GPIO_PIN2       for port %d is: %d\n", port->id, port->gpio[GPIO_PIN2].gpio);
//         pr_warning("GPIO_PIN5       for port %d is: %d\n", port->id, port->gpio[GPIO_PIN5].gpio);
//         pr_warning("GPIO_PIN5_TACHO for port %d is: %d\n", port->id, port->gpio[GPIO_PIN5_TACHO].gpio);
//         pr_warning("GPIO_PIN6       for port %d is: %d\n", port->id, port->gpio[GPIO_PIN6].gpio);
// 
// 
// //        request_irq(port->gpio[GPIO_PIN5_TACHO].gpio, handler, 0, dev_name(&port->pdev->dev), id );
// //        irq_set_irq_type(port->gpio[GPIO_PIN5_TACHO].gpio, type);
// 
//         return 0;
// }
// EXPORT_SYMBOL_GPL(ev3_output_port_request_irq);
// 
// void ev3_output_port_free_irq(struct legoev3_port_device *out_port)
// {
// //	struct ev3_output_port_data *port = dev_get_drvdata(&out_port->dev);
// 
// //        free_irq(port->gpio[GPIO_PIN5_TACHO].gpio, NULL );
// }
// EXPORT_SYMBOL_GPL(ev3_output_port_free_irq);
// 
// 
// 
// int ev3_output_port_get_pin56_levels(struct legoev3_port_device *out_port, unsigned *pin5, unsigned *pin6)
// {
// 	struct ev3_output_port_data *port = dev_get_drvdata(&out_port->dev);
// 
//  	*pin5 = gpio_get_value(port->gpio[GPIO_PIN5_TACHO].gpio);
//  	*pin6 = gpio_get_value(port->gpio[GPIO_PIN6].gpio);
// 
// 	return 0;
// }
// EXPORT_SYMBOL_GPL(ev3_output_port_get_pin56_levels);
// 
// 
static int __devexit ev3_tacho_motor_remove(struct legoev3_port_device *motor)
{
	struct ev3_motor_platform_data *pdata = motor->dev.platform_data;
	struct ev3_tacho_motor_data *ev3_tm = dev_get_drvdata(&motor->dev);

 	hrtimer_cancel(&ev3_tm->timer);

	dev_info(&motor->dev, "Unregistering interrupt from gpio %d irq %d on port %s\n",
		pdata->tacho_int_gpio,
		gpio_to_irq(pdata->tacho_int_gpio),
		dev_name(&ev3_tm->out_port->dev));

        free_irq(gpio_to_irq(pdata->tacho_int_gpio), ev3_tm);

	dev_info(&motor->dev, "Tacho motor removed from port %s\n",
		 dev_name(&ev3_tm->out_port->dev));
	dev_set_drvdata(&motor->dev, NULL);
	unregister_tacho_motor(&ev3_tm->tm);
	kfree(ev3_tm);
	return 0;
}

struct legoev3_port_driver ev3_tacho_motor_driver = {
	.probe	= ev3_tacho_motor_probe,
	.remove	= __devexit_p(ev3_tacho_motor_remove),
	.driver = {
		.name	= "ev3-tacho-motor",
		.owner	= THIS_MODULE,
	},
};
EXPORT_SYMBOL_GPL(ev3_tacho_motor_driver);
legoev3_port_driver(ev3_tacho_motor_driver);

MODULE_DESCRIPTION("EV3 tacho motor driver for LEGO Mindstorms EV3");
MODULE_AUTHOR("Ralph Hempel <rhempel@hempeldesigngroup.com>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("legoev3:ev3-tacho-motor");

