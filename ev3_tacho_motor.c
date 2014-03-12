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

// #define PIN6_NEAR_GND		250		/* 0.25V */

#define TACHO_MOTOR_POLL_NS	(  2000000)	                /* 2 msec */

#define   TACHO_SAMPLES           128

#define   MAX_PWM_CNT                   (10000)
#define   MAX_SPEED                     (100)
#define   SPEED_PWMCNT_REL              (100) //(MAX_PWM_CNT/MAX_SPEED)
#define   RAMP_FACTOR                   (1000)
#define   MAX_SYNC_MOTORS               (2)

#define   COUNTS_PER_PULSE_LM           12800L
#define   COUNTS_PER_PULSE_MM           8100L

enum
{
  SAMPLES_BELOW_SPEED_25  =  0,
  SAMPLES_SPEED_25_50     =  1,
  SAMPLES_SPEED_50_75     =  2,
  SAMPLES_ABOVE_SPEED_75  =  3,
  NO_OF_SAMPLE_STEPS      =  4
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
static    unsigned    SamplesMediumMotor[NO_OF_SAMPLE_STEPS] = {2, 4,  8,  16};
static    unsigned    SamplesLargeMotor[NO_OF_SAMPLE_STEPS]  = {4, 16, 32, 64};

struct ev3_tacho_motor_data {
	struct tacho_motor_device tm;

	struct legoev3_port_device *out_port;
	struct legoev3_port_device *motor_port;

 	struct hrtimer timer;

        unsigned tacho_samples[TACHO_SAMPLES];
        unsigned tacho_samples_head;
        unsigned tacho_samples_tail;

	unsigned *samples_per_speed;
	int	 *ramp_power_steps;
	
	unsigned counts_per_pulse;
	unsigned dir_chg_samples;

	/* FIXME - this should really just be tacho! And moved below target_xxx */

	int irq_tacho;

	/* FIXME - these need better names */

	bool mutex;

	/* FIXME - these relate to the time setpoints, probably need renaming */

	int time_cnt;
	int time_inc;

	/* FIXME - these relate to the ramp times, probbaly need renaming */

	int ramp_up_factor;
	int ramp_up_offset;

	int tacho_cnt_up;

	int prg_stop_timer;

	bool lock_ramp_down;
	bool brake_after;

	int state;

	int target_state;
	int target_speed;
	int target_steer;
	int target_power;
        int target_direction;
        int target_tacho;
        int target_step;
        int target_time;
        int target_ramp_up_time;
        int target_ramp_down_time;

	int tacho_cnt;
	int tacho_sensor;
	int old_tacho_cnt;
      
	int power;
        int speed;
        int direction;
        int step;
        int time;
};

// static    unsigned    AVG_TACHO_COUNTS[NO_OF_OUTPUT_PORTS]   = {2,2,2,2};
// static    unsigned    AVG_COUNTS[NO_OF_OUTPUT_PORTS]         = {(2 * COUNTS_PER_PULSE_LM),(2 * COUNTS_PER_PULSE_LM),(2 * COUNTS_PER_PULSE_LM),(2 * COUNTS_PER_PULSE_LM)};

#define  NON_INV   1
#define  INV      -1


static irqreturn_t tacho_motor_isr(int irq, void *id)
{
	struct ev3_tacho_motor_data *ev3_tm = id;
	struct ev3_motor_platform_data *pdata = ev3_tm->motor_port->dev.platform_data; 

	bool int_state = gpio_get_value(pdata->tacho_int_gpio);
	bool dir_state = gpio_get_value(pdata->tacho_dir_gpio);
	unsigned long timer = legoev3_hires_timer_read() >> 8;

	unsigned next_sample;

	int  next_direction;

	/* Grab the next incremental sample timestamp */

	next_sample = (ev3_tm->tacho_samples_head + 1) % TACHO_SAMPLES;

	ev3_tm->tacho_samples[next_sample] = timer;
	ev3_tm->tacho_samples_head = next_sample;

	/* If the speed is high enough, just update the tacho counter based on direction */

	if ((35 < ev3_tm->speed) || (-35 > ev3_tm->speed)) {

		if (ev3_tm->dir_chg_samples < ev3_tm->samples_per_speed[SAMPLES_ABOVE_SPEED_75])
			ev3_tm->dir_chg_samples++;

	} else {

	/* Update the tacho count and motor direction for low speed, taking
	/  advantage of the fact that if state and dir match, then the motor
	/  is turning FORWARD!
	*/
		if (int_state == dir_state)
			next_direction = FORWARD;
		else
			next_direction = BACKWARD;

		/* If the saved and next direction states match, then update the dir_chg_sample count */

		if (ev3_tm->direction == next_direction) {
			if (ev3_tm->dir_chg_samples < ev3_tm->samples_per_speed[SAMPLES_ABOVE_SPEED_75])
				ev3_tm->dir_chg_samples++;
		} else {
			ev3_tm->dir_chg_samples = 0;
		}

		ev3_tm->direction = next_direction;
	}

	if (FORWARD == ev3_tm->direction)
		ev3_tm->irq_tacho++;
	else			
		ev3_tm->irq_tacho--;

	return IRQ_HANDLED;
}
static int process_count = 0;

static void ev3_tacho_motor_forward(struct ev3_motor_platform_data *pdata)
{
	gpio_direction_output(pdata->motor_dir0_gpio, 0);
	gpio_direction_input( pdata->motor_dir1_gpio   );
}
static void ev3_tacho_motor_reverse(struct ev3_motor_platform_data *pdata)
{
	gpio_direction_input( pdata->motor_dir0_gpio   );
	gpio_direction_output(pdata->motor_dir1_gpio, 1);
}
static void ev3_tacho_motor_brake(struct ev3_motor_platform_data *pdata)
{
	gpio_direction_output(pdata->motor_dir0_gpio, 1);
	gpio_direction_output(pdata->motor_dir1_gpio, 1);
}
static void ev3_tacho_motor_coast(struct ev3_motor_platform_data *pdata)
{
	gpio_direction_output(pdata->motor_dir0_gpio, 0);
	gpio_direction_output(pdata->motor_dir1_gpio, 0);
}

static int ev3_tacho_motor_calculate_speed(struct ev3_tacho_motor_data *ev3_tm)
{
#warning "ev3_tacho_motor_calculate_speed is not yet implemented!"
	return 0;
}
static int ev3_tacho_motor_regulate_speed(struct ev3_tacho_motor_data *ev3_tm)
{
#warning "ev3_tacho_motor_regulate_speed is not yet implemented!"
	return 0;
}
static int ev3_tacho_motor_set_power(struct ev3_tacho_motor_data *ev3_tm)
{
#warning "ev3_tacho_motor_set_power is not yet implemented!"
	return 0;
}
static int ev3_tacho_motor_brake_motor(struct ev3_tacho_motor_data *ev3_tm)
{
#warning "ev3_tacho_motor_brake is not yet implemented!"
	return 0;
}
static void ev3_tacho_motor_get_compare_counts( struct ev3_tacho_motor_data *ev3_tm, int *ramp_count, int *ramp_count_test )
{
#warning "ev3_tacho_motor_get_compare_counts is not yet implemented!"
	*ramp_count      = 0;
	*ramp_count_test = 0;
}
static bool ev3_tacho_motor_check_less_than_special( int v1, int v2, int v3 )
{
#warning "ev3_tacho_motor_check_less_than_special is not yet implemented!"
	return 0;
}
static bool ev3_tacho_motor_step_speed_check_tacho_count_const( struct ev3_tacho_motor_data *ev3_tm, int tacho_cnt )
{
#warning "ev3_tacho_motor_step_speed_check_tacho_count_const is not yet implemented!"
	return 0;
}
static bool ev3_tacho_motor_step_speed_check_tacho_count_down( struct ev3_tacho_motor_data *ev3_tm, int tacho_cnt )
{
#warning "ev3_tacho_motor_step_speed_check_tacho_count_down is not yet implemented!"
	return 0;
}
static enum hrtimer_restart ev3_tacho_motor_timer_callback(struct hrtimer *timer)
{
 	struct ev3_tacho_motor_data *ev3_tm =
 			container_of(timer, struct ev3_tacho_motor_data, timer);

	struct ev3_motor_platform_data *pdata = ev3_tm->motor_port->dev.platform_data; 

	/* FIXME - these need better names */

	int tmp_tacho;
	int tmp;

	int speed;

	/* These get use in the ramp modes */

	bool ramp_complete;
	int  ramp_count;
	int  ramp_count_test;
 
 	hrtimer_forward_now(timer, ktime_set(0, TACHO_MOTOR_POLL_NS));

        /* Here's where the business end of things starts - update the tacho data that's
        /   shared with the world
        */

	tmp_tacho = ev3_tm->irq_tacho;
	tmp       = tmp_tacho - ev3_tm->old_tacho_cnt;

	ev3_tm->tacho_cnt    += tmp;
	ev3_tm->tacho_sensor += tmp;
	ev3_tm->old_tacho_cnt = tmp_tacho;

	ev3_tm->time_cnt += ev3_tm->time_inc;

	/* Early exit from function if someone is reading the struct! */

	if (ev3_tm->mutex )
		return HRTIMER_RESTART;

	/* Continue with the actual calculations */

	speed = ev3_tacho_motor_calculate_speed( ev3_tm );

	switch (ev3_tm->state) {

        case UNLIMITED_UNREG:
		if (ev3_tm->target_power != ev3_tm->power ) {
			ev3_tm->power = ev3_tm->target_power;
			ev3_tacho_motor_set_power( ev3_tm );
		}

		ev3_tm->tacho_cnt = 0;
		ev3_tm->time_cnt  = 0;
		break;

	case UNLIMITED_REG:
		ev3_tacho_motor_regulate_speed( ev3_tm );

		ev3_tm->tacho_cnt = 0;
		ev3_tm->time_cnt  = 0;
		break;

	case LIMITED_REG_STEPUP:
	          // Status used to check if ramp up has completed
		ramp_complete = 0;

		if (ev3_tm->ramp_power_steps != &ev3_tm->tacho_cnt)
			ev3_tm->tacho_cnt = 0;
		else	
			ev3_tm->time_cnt = 0;
 
		ev3_tacho_motor_get_compare_counts( ev3_tm, &ramp_count, &ramp_count_test );

		if (ev3_tacho_motor_check_less_than_special( ramp_count_test, ev3_tm->tacho_cnt_up, ev3_tm->target_direction ) && !ev3_tm->lock_ramp_down) {

			ev3_tm->target_speed = ((ramp_count * ev3_tm->ramp_up_factor)/RAMP_FACTOR) + ev3_tm->ramp_up_offset;

			if (ev3_tacho_motor_check_less_than_special( ev3_tm->target_speed, 6 * ev3_tm->target_direction, ev3_tm->target_direction ))
				ev3_tm->target_speed = 6 * ev3_tm->target_direction;

			ev3_tacho_motor_regulate_speed( ev3_tm );
		} else {
			if (ev3_tm->brake_after) {
				ev3_tm->lock_ramp_down = 1;
/* FIXME: Add ramp down to brake here */
// Status = RampDownToBrake(No, StepCnt, Motor[No].TachoCntUp, Motor[No].Dir);

			} else {
				ramp_complete = 1;
			}
		}

		if (ramp_complete && !ev3_tacho_motor_step_speed_check_tacho_count_const( ev3_tm, ev3_tm->tacho_cnt_up ))
			ev3_tacho_motor_step_speed_check_tacho_count_down( ev3_tm, ev3_tm->tacho_cnt_up );

		break;

	case LIMITED_REG_STEPCONST:

		if (ev3_tm->ramp_power_steps != &ev3_tm->tacho_cnt)
			ev3_tm->tacho_cnt = 0;
		else	
			ev3_tm->time_cnt = 0;

		ev3_tacho_motor_get_compare_counts(ev3_tm, &ramp_count, &ramp_count_test);

		if (ev3_tacho_motor_check_less_than_special( ramp_count_test, ev3_tm->tacho_cnt_up, ev3_tm->target_direction ) && !ev3_tm->lock_ramp_down) {
			ev3_tacho_motor_regulate_speed( ev3_tm );
		} else {
			if (ev3_tm->brake_after) {
				ev3_tm->lock_ramp_down = 1;
//               if (TRUE == RampDownToBrake(No, StepCnt, Motor[No].TachoCntConst, Motor[No].Dir))
//               {
//                 StepSpeedCheckTachoCntDown(No, Motor[No].TachoCntConst);
//               }

			} else {
//               StepSpeedCheckTachoCntDown(No, Motor[No].TachoCntConst);
			}
		}

		break;

	case LIMITED_REG_STEPDOWN:
		ramp_count = *ev3_tm->ramp_power_steps;

		if (ev3_tm->ramp_power_steps != &ev3_tm->tacho_cnt)
			ev3_tm->tacho_cnt = 0;
		else	
			ev3_tm->time_cnt = 0;

// 
//           if (TRUE == CheckLessThanSpecial(StepCnt, Motor[No].TachoCntDown, Motor[No].Dir))
//           {
//             NewSpeed = Motor[No].TargetPower - ((StepCnt * (Motor[No].RampDownFactor))/ RAMP_FACTOR);
//             if (TRUE == CheckLessThanSpecial((4 * Motor[No].Dir), NewSpeed, Motor[No].Dir))
//             {
//               Motor[No].TargetSpeed = NewSpeed;
//             }
//             dRegulateSpeed(No);
//           }
//           else
//           {
//             StepPowerStopMotor(No, Motor[No].TachoCntDown);
//           }
//         }
		break;
// 
//         case LIMITED_UNREG_STEPUP:
//         {
// 
//           UBYTE  Status;
//           SLONG  StepCnt;
//           SLONG  StepCntTst;
// 
//           // Status used to check if ramp up has completed
//           Status  = FALSE;
// 
//           if (StepPowerSteps[No] != &(Motor[No].TachoCnt))
//           {
//             Motor[No].TachoCnt  =  0;
//           }
//           else
//           {
//             Motor[No].TimeCnt =  0;
//           }
// 
//           GetCompareCounts(No, &StepCnt, &StepCntTst);
// 
//           if ((TRUE == CheckLessThanSpecial(StepCntTst, Motor[No].TachoCntUp, Motor[No].Dir)) && (FALSE == Motor[No].LockRampDown))
//           {
//             if (0 != Motor[No].Speed)
//             {
//               if (TRUE == CheckLessThanSpecial(Motor[No].Power, (StepCnt * (Motor[No].RampUpFactor))/*/RAMP_FACTOR*/, Motor[No].Dir))
//               {
//                 // if very slow ramp up then power could be calculated as 0 for a while
//                 // avoid starting and stopping
//                 Motor[No].Power = (StepCnt * (Motor[No].RampUpFactor))/RAMP_FACTOR;
// 
//               }
// 
//               if (TRUE == CheckLessThanSpecial(Motor[No].Power, 0, Motor[No].Dir))
//               {
//                 // Avoid motor turning the wrong way
//                 Motor[No].Power = Motor[No].Power * -1;
//               }
//             }
//             else
//             {
//               Motor[No].Power += (20 * Motor[No].Dir);
//             }
//             SetPower(No,Motor[No].Power);
//           }
//           else
//           {
//             // Done Stepping up
//             Motor[No].LockRampDown = TRUE;
//             if (TRUE == Motor[No].BrakeAfter)
//             {
//               Status = RampDownToBrake(No, StepCnt, Motor[No].TachoCntUp, Motor[No].Dir);
//             }
//             else
//             {
//               Status = TRUE;
//             }
//           }
// 
//           if (TRUE == Status)
//           {
//             // Ramp up completed check for next step
//             if (FALSE == StepPowerCheckTachoCntConst(No, Motor[No].TachoCntUp))
//             {
//               StepPowerCheckTachoCntDown(No, Motor[No].TachoCntUp);
//             }
//           }
//         }
//         break;
// 
//         case LIMITED_UNREG_STEPCONST:
//         {
//           SLONG   StepCnt, StepCntTst;
// 
//           if (StepPowerSteps[No] != &(Motor[No].TachoCnt))
//           {
//             Motor[No].TachoCnt =  0;
//           }
//           else
//           {
//             Motor[No].TimeCnt  =  0;
//           }
// 
//           GetCompareCounts(No, &StepCnt, &StepCntTst);
// 
//           if ((TRUE == CheckLessThanSpecial(StepCntTst, Motor[No].TachoCntConst, Motor[No].Dir)) && (FALSE == Motor[No].LockRampDown))
//           {
//           }
//           else
//           {
//             if (TRUE == Motor[No].BrakeAfter)
//             {
// 
//               Motor[No].LockRampDown = TRUE;
//               if (TRUE == RampDownToBrake(No, StepCnt, Motor[No].TachoCntConst, Motor[No].Dir))
//               {
//                 StepPowerCheckTachoCntDown(No, Motor[No].TachoCntConst);
//               }
//             }
//             else
//             {
//               StepPowerCheckTachoCntDown(No, Motor[No].TachoCntConst);
//             }
//           }
//         }
//         break;
// 
//         case LIMITED_UNREG_STEPDOWN:
//         {
//           SLONG   StepCnt;
// 
//           StepCnt = *StepPowerSteps[No];
// 
//           if (StepPowerSteps[No] != &(Motor[No].TachoCnt))
//           {
//             Motor[No].TachoCnt  =  0;
//           }
//           else
//           {
//             Motor[No].TimeCnt =  0;
//           }
// 
//           if (TRUE == CheckLessThanSpecial(StepCnt, Motor[No].TachoCntDown, Motor[No].Dir))
//           {
//             if ((TRUE == CheckLessThanSpecial((2 * Motor[No].Dir), Motor[No].Speed, Motor[No].Dir)) && (FALSE == MinRegEnabled[No]))
//             {
//               if (TRUE == CheckLessThanSpecial((4 * Motor[No].Dir), (Motor[No].TargetPower - ((StepCnt * (Motor[No].RampDownFactor))/ RAMP_FACTOR)), Motor[No].Dir))
//               {
//                 Motor[No].Power = Motor[No].TargetPower - ((StepCnt * (Motor[No].RampDownFactor))/ RAMP_FACTOR);
//               }
//               SetPower(No,Motor[No].Power);
//             }
//             else
//             {
// 
//               MinRegEnabled[No]     = TRUE;
//               Motor[No].TargetSpeed = (2 * Motor[No].Dir);
//               dRegulateSpeed(No);
//             }
//           }
//           else
//           {
//             StepPowerStopMotor(No, Motor[No].TachoCntDown);
//           }
//         }
//         break;
// 
//         case LIMITED_STEP_SYNC:
//         {
//           // Here motor are syncronized and supposed to drive straight
// 
//           UBYTE   Cnt;
//           UBYTE   Status;
//           SLONG   StepCnt;
// 
//           StepCnt = *StepPowerSteps[No];
// 
//           if (StepPowerSteps[No] != &(Motor[No].TachoCnt))
//           {
//             Motor[No].TachoCnt  =  0;
//           }
//           else
//           {
//             Motor[No].TimeCnt =  0;
//           }
// 
//           Status  = FALSE;
// 
//           if ((Motor[SyncMNos[0]].Power > (MAX_PWM_CNT - 100)) || (Motor[SyncMNos[0]].Power < (-MAX_PWM_CNT + 100)))
//           {
//             // Regulation is stretched to the limit....... Checked in both directions
//             Status = TRUE;
//             if (TRUE == CheckLessThanSpecial((SLONG)(Motor[SyncMNos[0]].Speed), (SLONG)(Motor[SyncMNos[0]].TargetSpeed), Motor[SyncMNos[0]].Dir))
//             {
//               if (TRUE == CheckLessThanSpecial((SLONG)(Motor[SyncMNos[0]].Speed), (SLONG)(MaxSyncSpeed), Motor[SyncMNos[0]].Dir))
//               {
//                 // Check for running the same direction as target speed
//                 if (((Motor[SyncMNos[0]].Speed <= 0) && (Motor[SyncMNos[0]].TargetSpeed <= 0)) ||
//                     ((Motor[SyncMNos[0]].Speed >= 0) && (Motor[SyncMNos[0]].TargetSpeed >= 0)))
//                 {
//                   MaxSyncSpeed = Motor[SyncMNos[0]].Speed;
//                 }
//               }
//             }
//           }
//           if ((Motor[SyncMNos[1]].Power > (MAX_PWM_CNT - 100)) || (Motor[SyncMNos[1]].Power < (-MAX_PWM_CNT + 100)))
//           {
//             // Regulation is stretched to the limit....... Checked in both directions
//             Status = TRUE;
//             if (TRUE == CheckLessThanSpecial((SLONG)(Motor[SyncMNos[1]].Speed), (SLONG)(Motor[SyncMNos[1]].TargetSpeed), (Motor[SyncMNos[0]].Dir * Motor[SyncMNos[1]].Dir)))
//             {
//               if (TRUE == CheckLessThanSpecial((SLONG)(Motor[SyncMNos[1]].Speed), (SLONG)(MaxSyncSpeed), (Motor[SyncMNos[0]].Dir * Motor[SyncMNos[1]].Dir)))
//               {
//                 // Check for running the same direction as target speed
//                 if (((Motor[SyncMNos[1]].Speed <= 0) && (Motor[SyncMNos[1]].TargetSpeed <= 0)) ||
//                     ((Motor[SyncMNos[1]].Speed >= 0) && (Motor[SyncMNos[1]].TargetSpeed >= 0)))
//                 {
//                   MaxSyncSpeed = Motor[SyncMNos[1]].Speed;
//                 }
//               }
//             }
//           }
// 
//           if (FALSE == Status)
//           {
//             if (TRUE == CheckLessThanSpecial((SLONG)(MaxSyncSpeed), Motor[SyncMNos[0]].TargetPower, Motor[SyncMNos[0]].Dir))
//             {
//               // TargetSpeed has been reduced but now there is room to increase
//               IncSpeed(Motor[SyncMNos[0]].Dir, &MaxSyncSpeed);
//             }
//           }
// 
//           for (Cnt = 0; Cnt < MAX_SYNC_MOTORS; Cnt++)
//           {
//             //Update all motors to the max sync speed
//             Motor[SyncMNos[Cnt]].TargetSpeed = MaxSyncSpeed;
//           }
// 
//           if (TRUE == CheckLessThanSpecial(Motor[SyncMNos[1]].TachoCnt, Motor[SyncMNos[0]].TachoCnt, Motor[SyncMNos[0]].Dir))
//           {
//             DecSpeed(Motor[SyncMNos[0]].Dir, &(Motor[SyncMNos[0]].TargetSpeed));
//           }
// 
//           if (TRUE == CheckLessThanSpecial(Motor[SyncMNos[0]].TachoCnt, Motor[SyncMNos[1]].TachoCnt, Motor[SyncMNos[0]].Dir))
//           {
//             DecSpeed(Motor[SyncMNos[0]].Dir, &(Motor[SyncMNos[1]].TargetSpeed));
//           }
// 
//           dRegulateSpeed(SyncMNos[0]);
//           dRegulateSpeed(SyncMNos[1]);
// 
//           CheckforEndOfSync();
// 
//         }
//         break;
//         case SYNCED_SLAVE:
//         {
//           if (StepPowerSteps[No] != &(Motor[No].TachoCnt))
//           {
//             Motor[No].TachoCnt  =  0;
//           }
//           else
//           {
//             Motor[No].TimeCnt =  0;
//           }
//         }
//         break;
//         case LIMITED_DIFF_TURN_SYNC:
//         {
//           UBYTE   Status;
//           SLONG   StepCnt;
// 
//           StepCnt = *StepPowerSteps[No];
// 
//           if (StepPowerSteps[No] != &(Motor[No].TachoCnt))
//           {
//             Motor[No].TachoCnt  =  0;
//           }
//           else
//           {
//             Motor[No].TimeCnt =  0;
//           }
// 
//           Status  = FALSE;
// 
//           if ((Motor[SyncMNos[0]].Power > (MAX_PWM_CNT - 100)) || (Motor[SyncMNos[0]].Power < (-MAX_PWM_CNT + 100)))
//           {
//             // Regulation is stretched to the limit....... in both directions
//             Status = TRUE;
//             if (TRUE == CheckLessThanSpecial((SLONG)(Motor[SyncMNos[0]].Speed), (SLONG)(Motor[SyncMNos[0]].TargetSpeed), Motor[SyncMNos[0]].Dir))
//             {
//               if (TRUE == CheckLessThanSpecial((SLONG)(Motor[SyncMNos[0]].Speed), (SLONG)(MaxSyncSpeed), Motor[SyncMNos[0]].Dir))
//               {
//                 // Check for running the same direction as target speed
//                 if (((Motor[SyncMNos[0]].Speed <= 0) && (Motor[SyncMNos[0]].TargetSpeed <= 0)) ||
//                     ((Motor[SyncMNos[0]].Speed >= 0) && (Motor[SyncMNos[0]].TargetSpeed >= 0)))
//                 {
//                   MaxSyncSpeed = Motor[SyncMNos[0]].Speed;
//                 }
//               }
//             }
//           }
// 
//           if ((Motor[SyncMNos[1]].Power > (MAX_PWM_CNT - 100)) || (Motor[SyncMNos[1]].Power < (-MAX_PWM_CNT + 100)))
//           {
//             // Regulation is stretched to the limit....... in both directions
//             Status = TRUE;
//             if (TRUE == CheckLessThanSpecial((SLONG)(Motor[SyncMNos[1]].Speed), (SLONG)(Motor[SyncMNos[1]].TargetSpeed), (Motor[SyncMNos[1]].Dir * Motor[SyncMNos[0]].Dir)))
//             {
//               if (TRUE == CheckLessThanSpecial((SLONG)((Motor[SyncMNos[1]].Speed * 100)/(Motor[SyncMNos[1]].TurnRatio * Motor[SyncMNos[1]].Dir)), (SLONG)(MaxSyncSpeed), Motor[SyncMNos[0]].Dir))
//               {
//                 if ((0 == Motor[SyncMNos[1]].TurnRatio) || (0 == Motor[SyncMNos[1]].Speed))
//                 {
//                   MaxSyncSpeed = 0;
//                 }
//                 else
//                 {
//                   // Check for running the same direction as target speed
//                   if (((Motor[SyncMNos[1]].Speed <= 0) && (Motor[SyncMNos[1]].TargetSpeed <= 0)) ||
//                       ((Motor[SyncMNos[1]].Speed >= 0) && (Motor[SyncMNos[1]].TargetSpeed >= 0)))
//                   {
//                     MaxSyncSpeed = ((Motor[SyncMNos[1]].Speed * 100)/Motor[SyncMNos[1]].TurnRatio) * Motor[SyncMNos[1]].Dir;
//                   }
//                 }
//               }
//             }
//           }
// 
//           if (FALSE == Status)
//           {
//             if (TRUE == CheckLessThanSpecial((SLONG)(MaxSyncSpeed), Motor[SyncMNos[0]].TargetPower, Motor[SyncMNos[0]].Dir))
//             {
//               // TargetSpeed has been reduced but now there is room to increase
//               IncSpeed(Motor[SyncMNos[0]].Dir, &MaxSyncSpeed);
//             }
//           }
// 
//           // Set the new
//           Motor[SyncMNos[0]].TargetSpeed = MaxSyncSpeed;
//           if ((0 == Motor[SyncMNos[1]].TurnRatio) || (0 == MaxSyncSpeed))
//           {
//             Motor[SyncMNos[1]].TargetSpeed = 0;
//           }
//           else
//           {
//             Motor[SyncMNos[1]].TargetSpeed = ((MaxSyncSpeed * Motor[SyncMNos[1]].TurnRatio)/100) * Motor[SyncMNos[1]].Dir;
//           }
// 
//           if(0 != Motor[SyncMNos[1]].TurnRatio)
//           {
//             if (TRUE == CheckLessThanSpecial((((Motor[SyncMNos[1]].TachoCnt * 100)/Motor[SyncMNos[1]].TurnRatio) * Motor[SyncMNos[1]].Dir), Motor[SyncMNos[0]].TachoCnt, Motor[SyncMNos[0]].Dir))
//             {
//               DecSpeed(Motor[SyncMNos[0]].Dir, &(Motor[SyncMNos[0]].TargetSpeed));
//             }
//           }
// 
//           if(0 != Motor[SyncMNos[1]].TurnRatio)
//           {
//             if (TRUE == CheckLessThanSpecial(Motor[SyncMNos[0]].TachoCnt, ((Motor[SyncMNos[1]].TachoCnt * 100)/(Motor[SyncMNos[1]].TurnRatio) * Motor[SyncMNos[1]].Dir), Motor[SyncMNos[0]].Dir))
//             {
//               DecSpeed(Motor[SyncMNos[1]].Dir * Motor[SyncMNos[0]].Dir, &(Motor[SyncMNos[1]].TargetSpeed));
//             }
//           }
// 
//           dRegulateSpeed(SyncMNos[0]);
//           dRegulateSpeed(SyncMNos[1]);
// 
//           CheckforEndOfSync();
// 
//         }
//         break;
// 
//         case RAMP_DOWN_SYNC:
//         {
// 
//           SLONG   Count0, Count1;
// 
//           if (StepPowerSteps[No] != &(Motor[No].TachoCnt))
//           {
//             Motor[No].TachoCnt = 0;
//           }
//           else
//           {
//             Motor[No].TimeCnt  = 0;
//           }
// 
//           // Duration is either dependent on timer ticks or tacho counts
//           GetSyncDurationCnt(&Count0, &Count1);
// 
//           if (TRUE == CheckLessThanSpecial(Count0, Motor[SyncMNos[0]].TachoCntConst, Motor[SyncMNos[0]].Dir))
//           {
// 
//             RampDownToBrake(SyncMNos[0], Count0, Motor[SyncMNos[0]].TachoCntConst, Motor[SyncMNos[0]].Dir);
// 
//             if (StepPowerSteps[SyncMNos[0]] == TimerSteps[SyncMNos[0]])
//             {
//               // Needs to adjust second synchronised in time mode - both motor needs to run for the same
//               // amount of time but not at the same speed
//               RampDownToBrake(SyncMNos[1], ((Count1 * Motor[SyncMNos[1]].TurnRatio)/100),(( Motor[SyncMNos[1]].TachoCntConst * Motor[SyncMNos[1]].TurnRatio)/100), (Motor[SyncMNos[0]].Dir * Motor[SyncMNos[1]].Dir));
//             }
//             else
//             {
//               RampDownToBrake(SyncMNos[1], Count1, Motor[SyncMNos[1]].TachoCntConst, (Motor[SyncMNos[0]].Dir * Motor[SyncMNos[1]].Dir));
//             }
//           }
//           else
//           {
//             MaxSyncSpeed                    = 0;
//             Motor[SyncMNos[0]].TargetSpeed  = 0;
//             Motor[SyncMNos[1]].TargetSpeed  = 0;
//             StepPowerStopMotor(SyncMNos[0], Motor[SyncMNos[0]].TachoCntConst);
//             StepPowerStopMotor(SyncMNos[1], Motor[SyncMNos[1]].TachoCntConst);
//           }
//         }
//         break;
 
         case STOP_MOTOR:
         {
           if (ev3_tm->prg_stop_timer)
           {
             ev3_tm->prg_stop_timer--;
           }
           else
           {
             ev3_tm->state        = IDLE;
             ev3_tm->target_state = UNLIMITED_UNREG;

             ev3_tacho_motor_coast(pdata);
           }
         }
         break;
 
         case BRAKED:
         {
           ev3_tacho_motor_brake_motor(ev3_tm);
         }
         break;
 
         case IDLE:
         { /* Intentionally left empty */
         }
         break;
 	default:
         { /* Intentionally left empty */
         }
	break;
	}

	return HRTIMER_RESTART;
}


	
/* FIXME: All these getters and setters can be macrofied!! */

static int ev3_tacho_motor_get_tacho(struct tacho_motor_device *tm)
{
	struct ev3_tacho_motor_data *ev3_tm =
			container_of(tm, struct ev3_tacho_motor_data, tm);

	return ev3_tm->irq_tacho;
}

static int ev3_tacho_motor_get_step(struct tacho_motor_device *tm)
{
	struct ev3_tacho_motor_data *ev3_tm =
			container_of(tm, struct ev3_tacho_motor_data, tm);

	return ev3_tm->step;
}

static int ev3_tacho_motor_get_direction(struct tacho_motor_device *tm)
{
	struct ev3_tacho_motor_data *ev3_tm =
			container_of(tm, struct ev3_tacho_motor_data, tm);

	return ev3_tm->direction;
}

static int ev3_tacho_motor_get_speed(struct tacho_motor_device *tm)
{
	struct ev3_tacho_motor_data *ev3_tm =
			container_of(tm, struct ev3_tacho_motor_data, tm);

	return ev3_tm->speed;
}

static int ev3_tacho_motor_get_power(struct tacho_motor_device *tm)
{
	struct ev3_tacho_motor_data *ev3_tm =
			container_of(tm, struct ev3_tacho_motor_data, tm);

	return ev3_tm->power;
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

	int err;

	ev3_tm->target_power = target_power;

	/* FIXME: Move all this to the actual state handler!!! */
	
        if (0 < target_power) {
		ev3_tacho_motor_forward(pdata);
	} else if (0 > target_power) {
		ev3_tacho_motor_reverse(pdata);
		target_power = -target_power;
	} else {
		/* FIXME: Add code to float or brake here depending on mode */
		ev3_tacho_motor_coast(pdata);
	}

	err = pwm_set_duty_percent(pdata->pwm, target_power);

 	if (err) {
 		dev_err(&ev3_tm->motor_port->dev, "%s: Failed to set pwm duty percent! (%d)\n",
 			__func__, err);
 	}
 
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

static int ev3_tacho_motor_get_target_step(struct tacho_motor_device *tm)
{
	struct ev3_tacho_motor_data *ev3_tm =
			container_of(tm, struct ev3_tacho_motor_data, tm);

	return ev3_tm->target_step;
}

static void ev3_tacho_motor_set_target_step(struct tacho_motor_device *tm, long target_step)
{
	struct ev3_tacho_motor_data *ev3_tm =
			container_of(tm, struct ev3_tacho_motor_data, tm);

	struct ev3_motor_platform_data *pdata = ev3_tm->motor_port->dev.platform_data; 

	int err;

	ev3_tm->target_step = target_step;
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

static int ev3_tacho_motor_get_target_ramp_up_time(struct tacho_motor_device *tm)
{
	struct ev3_tacho_motor_data *ev3_tm =
			container_of(tm, struct ev3_tacho_motor_data, tm);

	return ev3_tm->target_ramp_up_time;
}

static void ev3_tacho_motor_set_target_ramp_up_time(struct tacho_motor_device *tm, long target_ramp_up_time)
{
	struct ev3_tacho_motor_data *ev3_tm =
			container_of(tm, struct ev3_tacho_motor_data, tm);

	struct ev3_motor_platform_data *pdata = ev3_tm->motor_port->dev.platform_data; 

	int err;

	ev3_tm->target_ramp_up_time = target_ramp_up_time;
}

static int ev3_tacho_motor_get_target_ramp_down_time(struct tacho_motor_device *tm)
{
	struct ev3_tacho_motor_data *ev3_tm =
			container_of(tm, struct ev3_tacho_motor_data, tm);

	return ev3_tm->target_ramp_down_time;
}

static void ev3_tacho_motor_set_target_ramp_down_time(struct tacho_motor_device *tm, long target_ramp_down_time)
{
	struct ev3_tacho_motor_data *ev3_tm =
			container_of(tm, struct ev3_tacho_motor_data, tm);

	struct ev3_motor_platform_data *pdata = ev3_tm->motor_port->dev.platform_data; 

	int err;

	ev3_tm->target_ramp_down_time = target_ramp_down_time;
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

        /* Set up motor specific initialization constants */
        /* FIXME: These need to be motor specific later!  */
        /* Maybe a better way is to have this set up as a constant array that's memcpy'ed */

	ev3_tm->tm.get_tacho     = ev3_tacho_motor_get_tacho;
	ev3_tm->tm.get_direction = ev3_tacho_motor_get_direction;
	ev3_tm->tm.get_speed     = ev3_tacho_motor_get_speed;
	ev3_tm->tm.get_power     = ev3_tacho_motor_get_power;
	ev3_tm->tm.get_time      = ev3_tacho_motor_get_time;
	ev3_tm->tm.get_state     = ev3_tacho_motor_get_state;

	ev3_tm->tm.get_target_power    = ev3_tacho_motor_get_target_power;
	ev3_tm->tm.set_target_power    = ev3_tacho_motor_set_target_power;

	ev3_tm->tm.get_target_tacho    = ev3_tacho_motor_get_target_tacho;
	ev3_tm->tm.set_target_tacho    = ev3_tacho_motor_set_target_tacho;

	ev3_tm->tm.get_target_step     = ev3_tacho_motor_get_target_step;
	ev3_tm->tm.set_target_step     = ev3_tacho_motor_set_target_step;

	ev3_tm->tm.get_target_speed    = ev3_tacho_motor_get_target_speed;
	ev3_tm->tm.set_target_speed    = ev3_tacho_motor_set_target_speed;

	ev3_tm->tm.get_target_steer    = ev3_tacho_motor_get_target_steer;
	ev3_tm->tm.set_target_steer    = ev3_tacho_motor_set_target_steer;

	ev3_tm->tm.get_target_time     = ev3_tacho_motor_get_target_time;
	ev3_tm->tm.set_target_time     = ev3_tacho_motor_set_target_time;

	ev3_tm->tm.get_target_ramp_up_time   = ev3_tacho_motor_get_target_ramp_up_time;
	ev3_tm->tm.set_target_ramp_up_time   = ev3_tacho_motor_set_target_ramp_up_time;

	ev3_tm->tm.get_target_ramp_down_time = ev3_tacho_motor_get_target_ramp_down_time;
	ev3_tm->tm.set_target_ramp_down_time = ev3_tacho_motor_set_target_ramp_down_time;

	ev3_tm->state             = IDLE;
	ev3_tm->samples_per_speed = SamplesLargeMotor;
	ev3_tm->counts_per_pulse  = COUNTS_PER_PULSE_LM;

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

