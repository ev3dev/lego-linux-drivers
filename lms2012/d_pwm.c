/*
 * LEGO® MINDSTORMS EV3
 *
 * Copyright (C) 2010-2013 The LEGO Group
 * Copyright (C) 2016 David Lechner <david@lechnology.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <linux/debugfs.h>
#include <linux/fs.h>
#include <linux/hrtimer.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/ioport.h>
#include <linux/irq.h>
#include <linux/kernel.h>
#include <linux/miscdevice.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/rcupdate.h>
#include <linux/sched.h>
#include <linux/signal.h>
#include <linux/slab.h>
#include <linux/uaccess.h>

#include "lms2012.h"

#define MODULE_NAME		"pwm_module"
#define DEVICE1_NAME		PWM_DEVICE
#define DEVICE2_NAME		MOTOR_DEVICE

#define SOFT_TIMER_MS		2
#define SOFT_TIMER_SETUP	(SOFT_TIMER_MS * 1000000)

/*
 *  NO_OF_TACHO_SAMPLES holds the number of recent tacho samples
 */
#define NO_OF_TACHO_SAMPLES	128

#define MAX_PWM_CNT		(10000)
#define MAX_SPEED		(100)
#define SPEED_PWMCNT_REL	(MAX_PWM_CNT / MAX_SPEED)
#define RAMP_FACTOR		(1000)
#define MAX_SYNC_MOTORS		(2)

#define PWM_PERIOD		(NSEC_PER_SEC / MAX_PWM_CNT)

//#define COUNT_PER_PULSE_LM	12800L
//#define COUNT_PER_PULSE_MM	8100L
// COUNT_PER_PULSE_* was from 33MHz / 256 clock counts, so convert to usec...
#define USEC_PER_PULSE_LM	99297
#define USEC_PER_PULSE_MM	62836

enum {
	SAMPLES_BELOW_SPEED_25,
	SAMPLES_SPEED_25_50,
	SAMPLES_SPEED_50_75,
	SAMPLES_ABOVE_SPEED_75,
	NO_OF_SAMPLE_STEPS
};

#define NON_INV	1
#define INV	-1

enum {
	FORWARD,
	BACKWARD,
	BRAKE,
	COAST,
};

enum {
	UNLIMITED_UNREG,
	UNLIMITED_REG,
	LIMITED_REG_STEPUP,
	LIMITED_REG_STEPCONST,
	LIMITED_REG_STEPDOWN,
	LIMITED_UNREG_STEPUP,
	LIMITED_UNREG_STEPCONST,
	LIMITED_UNREG_STEPDOWN,
	LIMITED_REG_TIMEUP,
	LIMITED_REG_TIMECONST,
	LIMITED_REG_TIMEDOWN,
	LIMITED_UNREG_TIMEUP,
	LIMITED_UNREG_TIMECONST,
	LIMITED_UNREG_TIMEDOWN,
	LIMITED_STEP_SYNC,
	LIMITED_TURN_SYNC,
	LIMITED_DIFF_TURN_SYNC,
	SYNCED_SLAVE,
	RAMP_DOWN_SYNC,
	HOLD,
	BRAKED,
	STOP_MOTOR,
	IDLE,
};

enum {
	UNUSED_SYNC_MOTOR = 0xFF,
};

typedef struct {
	SLONG IrqTacho;
	SLONG TachoCnt;
	SLONG TachoSensor;
	SLONG TimeCnt;
	SLONG TimeInc;
	SLONG OldTachoCnt;
	SLONG TachoCntUp;
	SLONG TachoCntConst;
	SLONG TachoCntDown;
	SLONG RampUpFactor;
	SLONG RampUpOffset;
	SLONG RampDownFactor;
	SLONG PVal;
	SLONG IVal;
	SLONG DVal;
	SLONG OldSpeedErr;
	SLONG Power;
	SLONG TargetPower;
	SLONG Dir;
	SBYTE Speed;
	SBYTE TargetSpeed;
	SBYTE TargetBrake;
	UBYTE BrakeAfter;
	UBYTE LockRampDown;
	SBYTE Pol;
	UBYTE Direction;
	UBYTE Type;
	UBYTE State;
	UBYTE TargetState;
	UBYTE Owner;
	UBYTE Mutex;
	UBYTE DirChgPtr;
	SWORD TurnRatio;
} MOTOR;

#define TRUE	1
#define FALSE	0

/* MOTORDATA must match lms2012-compat! */
typedef struct {
	SLONG TachoCounts;
	SBYTE Speed;
	SLONG TachoSensor;
} MOTORDATA;

typedef struct {
	DATA8   Cmd;
	DATA8   Nos;
	DATA8   Power;
	DATA32  Step1;
	DATA32  Step2;
	DATA32  Step3;
	DATA8   Brake;
} STEPPOWER;

typedef struct {
	DATA8   Cmd;
	DATA8   Nos;
	DATA8   Power;
	DATA32  Time1;
	DATA32  Time2;
	DATA32  Time3;
	DATA8   Brake;
} TIMEPOWER;

typedef struct {
	DATA8   Cmd;
	DATA8   Nos;
	DATA8   Speed;
	DATA32  Step1;
	DATA32  Step2;
	DATA32  Step3;
	DATA8   Brake;
} STEPSPEED;

typedef struct {
	DATA8   Cmd;
	DATA8   Nos;
	DATA8   Speed;
	DATA32  Time1;
	DATA32  Time2;
	DATA32  Time3;
	DATA8   Brake;
} TIMESPEED;

typedef struct {
	DATA8   Cmd;
	DATA8   Nos;
	DATA8   Speed;
	DATA16  Turn;
	DATA32  Step;
	DATA8   Brake;
} STEPSYNC;

typedef struct {
	DATA8   Cmd;
	DATA8   Nos;
	DATA8   Speed;
	DATA16  Turn;
	DATA32  Time;
	DATA8   Brake;
} TIMESYNC;

enum {
	opPROGRAM_STOP		= 0x02,
	opPROGRAM_START		= 0x03,
	// opOUTPUT_GET_TYPE	= 0xA0,
	opOUTPUT_SET_TYPE	= 0xA1,
	opOUTPUT_RESET		= 0xA2,
	opOUTPUT_STOP		= 0xA3,
	opOUTPUT_POWER		= 0xA4,
	opOUTPUT_SPEED		= 0xA5,
	opOUTPUT_START		= 0xA6,
	opOUTPUT_POLARITY	= 0xA7,
	// opOUTPUT_READ	= 0xA8,
	// opOUTPUT_TEST	= 0xA9,
	// opOUTPUT_READY	= 0xAA,
	// opOUTPUT_POSITION	= 0xAB,
	opOUTPUT_STEP_POWER	= 0xAC,
	opOUTPUT_TIME_POWER	= 0xAD,
	opOUTPUT_STEP_SPEED	= 0xAE,
	opOUTPUT_TIME_SPEED	= 0xAF,
	opOUTPUT_STEP_SYNC	= 0xB0,
	opOUTPUT_TIME_SYNC	= 0xB1,
	opOUTPUT_CLR_COUNT	= 0xB2,
	// opOUTPUT_GET_COUNT	= 0xB3,
};

typedef struct {
	ktime_t TachoArray[NO_OF_TACHO_SAMPLES];
	UBYTE ArrayPtr;
	UBYTE ArrayPtrOld;
} TACHOSAMPLES;

static irqreturn_t IntA (int irq, void * dev);
static irqreturn_t IntB (int irq, void * dev);
static irqreturn_t IntC (int irq, void * dev);
static irqreturn_t IntD (int irq, void * dev);

static UBYTE dCalculateSpeed(UBYTE No, SBYTE *pSpeed);
static void SetGpioRisingIrq(UBYTE Port, UBYTE PinNo, irqreturn_t (*IntFuncPtr)(int, void *));
static void GetSyncDurationCnt(SLONG *pCount0, SLONG *pCount1);
static void CheckforEndOfSync(void);

#define OutputReadDir(port, pin) \
	gpiod_get_value(Device1Lms2012Compat->out_pins[port]->desc[pin])

#define READDirA OutputReadDir(0, OUTPUT_PORT_PIN5R)
#define READDirB OutputReadDir(1, OUTPUT_PORT_PIN5R)
#define READDirC OutputReadDir(2, OUTPUT_PORT_PIN5R)
#define READDirD OutputReadDir(3, OUTPUT_PORT_PIN5R)

static struct device *Device1Lms2012CompatDev;
static struct lms2012_compat *Device1Lms2012Compat;

static MOTOR Motor[OUTPUTS];
static SLONG *(StepPowerSteps[OUTPUTS]);
static SLONG *(TachoSteps[OUTPUTS]) = {
	&(Motor[0].TachoCnt),
	&(Motor[1].TachoCnt),
	&(Motor[2].TachoCnt),
	&(Motor[3].TachoCnt)
};
static SLONG *(TimerSteps[OUTPUTS]) = {
	&(Motor[0].TimeCnt),
	&(Motor[1].TimeCnt),
	&(Motor[2].TimeCnt),
	&(Motor[3].TimeCnt)
};

static TACHOSAMPLES    TachoSamples[OUTPUTS];
static UBYTE           ReadyStatus = 0;
static UBYTE           TestStatus  = 0;

static MOTORDATA       MotorData[OUTPUTS];
static MOTORDATA       *pMotor = MotorData;

static ktime_t         TimeOutSpeed0[OUTPUTS];
static UBYTE           MinRegEnabled[OUTPUTS];

static UBYTE           SyncMNos[MAX_SYNC_MOTORS];
static SBYTE           MaxSyncSpeed;

static struct hrtimer  Device1Timer;
static ktime_t         Device1Time;

static UBYTE           PrgStopTimer[OUTPUTS];

static ULONG USecPerPulse[4] = {
	USEC_PER_PULSE_LM,
	USEC_PER_PULSE_LM,
	USEC_PER_PULSE_LM,
	USEC_PER_PULSE_LM
};

/*
 * Number of average samples for each motor
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
static UBYTE	SamplesMediumMotor[NO_OF_SAMPLE_STEPS] = { 2, 4,  8,  16 };
static UBYTE	SamplesLargeMotor[NO_OF_SAMPLE_STEPS]  = { 4, 16, 32, 64 };
//static UBYTE	SamplesMediumMotor[NO_OF_SAMPLE_STEPS] = { 1, 2,  4,  8 };
//static UBYTE	SamplesLargeMotor[NO_OF_SAMPLE_STEPS]  = { 2, 8, 16, 32 };

static UBYTE *SamplesPerSpeed[OUTPUTS] = {
	SamplesLargeMotor,
	SamplesLargeMotor,
	SamplesLargeMotor,
	SamplesLargeMotor,
};

static UBYTE AVG_TACHO_COUNTS[OUTPUTS] = { 2, 2, 2, 2 };

static ULONG AVG_COUNTS[OUTPUTS] = {
	(2 * USEC_PER_PULSE_LM),
	(2 * USEC_PER_PULSE_LM),
	(2 * USEC_PER_PULSE_LM),
	(2 * USEC_PER_PULSE_LM)
};

static inline void SETMotorType(u8 Port, u8 NewType)
{
	Motor[Port].Type = NewType;
	if (TYPE_MINITACHO == NewType) {
		USecPerPulse[Port] = USEC_PER_PULSE_MM;
		SamplesPerSpeed[Port] = SamplesMediumMotor;
	} else {
		USecPerPulse[Port] = USEC_PER_PULSE_LM;
		SamplesPerSpeed[Port] = SamplesLargeMotor;
	}
}

static inline void SETAvgTachoCount(u8 Port, u8 Speed)
{
	if (Speed > 80) {
		AVG_TACHO_COUNTS[Port] = SamplesPerSpeed[Port][3];
		AVG_COUNTS[Port]       = SamplesPerSpeed[Port][3] * USecPerPulse[Port];
	} else {
		if (Speed > 60) {
			AVG_TACHO_COUNTS[Port] = SamplesPerSpeed[Port][2];
			AVG_COUNTS[Port]       = SamplesPerSpeed[Port][2] * USecPerPulse[Port];
		} else {
			if (Speed > 40) {
				AVG_TACHO_COUNTS[Port] = SamplesPerSpeed[Port][1];
				AVG_COUNTS[Port]       = SamplesPerSpeed[Port][1] * USecPerPulse[Port];
			} else {
				AVG_TACHO_COUNTS[Port] = SamplesPerSpeed[Port][0];
				AVG_COUNTS[Port]       = SamplesPerSpeed[Port][0] * USecPerPulse[Port];
			}
		}
	}
}

#define CLEARTachoArray(No) { \
	Motor[No].DirChgPtr = 0;\
	memset(&TachoSamples[No], 0, sizeof(TACHOSAMPLES));\
}

#define OutputFloat(port, pin) \
	gpiod_direction_input(Device1Lms2012Compat->out_pins[port]->desc[pin])

#define OutputRead(port, pin) \
	gpiod_get_value(Device1Lms2012Compat->out_pins[port]->desc[pin])

#define OutputHigh(port, pin) \
	gpiod_direction_output(Device1Lms2012Compat->out_pins[port]->desc[pin], 1)

#define OutputLow(port, pin) \
	gpiod_direction_output(Device1Lms2012Compat->out_pins[port]->desc[pin], 0)

#define READIntA OutputRead(0, OUTPUT_PORT_PIN6)
#define READIntB OutputRead(1, OUTPUT_PORT_PIN6)
#define READIntC OutputRead(2, OUTPUT_PORT_PIN6)
#define READIntD OutputRead(3, OUTPUT_PORT_PIN6)

static void CheckSpeedPowerLimits(SBYTE *pCheckVal)
{
	if (MAX_SPEED < *pCheckVal) {
		*pCheckVal = MAX_SPEED;
	}
	if (-MAX_SPEED > *pCheckVal) {
		*pCheckVal = -MAX_SPEED;
	}
}

static UBYTE CheckLessThanSpecial(SLONG Param1, SLONG Param2, SLONG Inv)
{
	UBYTE RtnVal = FALSE;

	if ((Param1 * Inv) < (Param2 * Inv)) {
		RtnVal = TRUE;
	}

	return RtnVal;
}

static void IncSpeed(SBYTE Dir, SBYTE *pSpeed)
{
	if ((NON_INV == Dir) && (MAX_SPEED > *pSpeed)) {
		(*pSpeed)++;
	} else {
		if ((INV == Dir) && (-MAX_SPEED < *pSpeed)) {
			(*pSpeed)--;
		}
	}
}

static void DecSpeed(SBYTE Dir, SBYTE *pSpeed)
{
	if ((NON_INV == Dir) && (0 < *pSpeed)) {
		(*pSpeed)--;
	} else {
		if ((INV == Dir) && (0 > *pSpeed)) {
			(*pSpeed)++;
		}
	}
}

static inline void SetDirRwd(UBYTE Port)
{
	OutputFloat(Port, OUTPUT_PORT_PIN1);
	OutputHigh(Port, OUTPUT_PORT_PIN2);
}

static inline void SetDirFwd(UBYTE Port)
{
	OutputHigh(Port, OUTPUT_PORT_PIN1);
	OutputFloat(Port, OUTPUT_PORT_PIN2);
}

static inline void SetBrake(UBYTE Port)
{
	OutputHigh(Port, OUTPUT_PORT_PIN1);
	OutputHigh(Port, OUTPUT_PORT_PIN2);
}

static inline void SetCoast(UBYTE Port)
{
	OutputLow(Port, OUTPUT_PORT_PIN1);
	OutputLow(Port, OUTPUT_PORT_PIN2);
}

static inline UWORD GetTachoDir(UBYTE Port)
{
	return OutputRead(Port, OUTPUT_PORT_PIN5R);
}

static inline UWORD GetTachoInt(UBYTE Port)
{
	return OutputRead(Port, OUTPUT_PORT_PIN6);
}

static inline void SetDuty(UBYTE Port, ULONG Duty)
{
	Duty = PWM_PERIOD * Duty / MAX_PWM_CNT;
	pwm_config(Device1Lms2012Compat->out_pwms[Port], Duty, PWM_PERIOD);
}

static void SetPower(UBYTE Port, SLONG Power)
{
	if (MAX_PWM_CNT < Power) {
		Power             = MAX_PWM_CNT;
		Motor[Port].Power = Power;
	}
	if (-MAX_PWM_CNT > Power) {
		Power             = -MAX_PWM_CNT;
		Motor[Port].Power = Power;
	}

	if (TYPE_MINITACHO == Motor[Port].Type) {
		// Medium Motor
		if (0 != Power) {
			if (0 < Power) {
				SetDirFwd(Port);
			} else {
				SetDirRwd(Port);
				Power = 0 - Power;
			}
			Power = ((Power * 8000)/10000) + 2000;
		}
	} else {
		// Large motor
		if (0 != Power) {
			if (0 < Power) {
				SetDirFwd(Port);
			} else {
				SetDirRwd(Port);
				Power = 0 - Power;
			}
			Power = ((Power * 9500)/10000) + 500;
		}
	}
	SetDuty(Port, Power);
}

static void SetRegulationPower(UBYTE Port, SLONG Power)
{
	if (MAX_PWM_CNT < Power) {
		Power             = MAX_PWM_CNT;
		Motor[Port].Power = Power;
	}
	if (-MAX_PWM_CNT > Power) {
		Power             = -MAX_PWM_CNT;
		Motor[Port].Power = Power;
	}

	if (TYPE_MINITACHO == Motor[Port].Type) {
		// Medium Motor
		if (0 != Power) {
			if (0 < Power) {
				SetDirFwd(Port);
			} else {
				SetDirRwd(Port);
				Power = 0 - Power;
			}
			Power = ((Power * 9000)/10000) + 1000;
		}
	} else {
		// Large motor
		if (0 != Power) {
			if (0 < Power) {
				SetDirFwd(Port);
			} else {
				SetDirRwd(Port);
				Power = 0 - Power;
			}
			Power = ((Power * 10000)/10000) + 0;
		}
	}
	SetDuty(Port, Power);
}

static void ClearPIDParameters(UBYTE No)
{
	Motor[No].PVal        = 0;
	Motor[No].IVal        = 0;
	Motor[No].DVal        = 0;
	Motor[No].OldSpeedErr = 0;
}

/*
 *  Helper function
 *  Checks how the motor should stop
 *
 *  Parameters:
 *  No: The motor number
 *  AdjustTachoValue: The value that the tacho should be decremented
 */
static void StepPowerStopMotor(UBYTE No, SLONG AdjustTachoValue)
{
	*StepPowerSteps[No] -= AdjustTachoValue;
	SetPower(No, 0);                // Don't destroy power level if next command needs to use it
	if (Motor[No].TargetBrake) {
		if (StepPowerSteps[No] != &(Motor[No].TachoCnt)) {
			// To be able to brake at tacho = 0 if this was timed command
			Motor[No].TachoCnt = 0;
		}
		Motor[No].State  = BRAKED;
	} else {
		Motor[No].State  = IDLE;
		SetCoast(No);
	}
	Motor[No].TargetState = UNLIMITED_UNREG;  // Default motor startup

	// If this was a synced motor then clear it
	if (SyncMNos[0] == No) {
		SyncMNos[0] = UNUSED_SYNC_MOTOR;
	}
	if (SyncMNos[1] == No) {
		SyncMNos[1] = UNUSED_SYNC_MOTOR;
	}
	ReadyStatus &= ~(0x01 << No);             // Clear ready flag
	TestStatus  &= ~(0x01 << No);             // Clear ready flag
}

/*
 *  Helper function
 *  Checks if step power should have a Count up sequence
 *
 *  Parameters:
 *  No: The motor number
 */
static UBYTE StepSpeedCheckTachoCntUp(UBYTE No)
{
	UBYTE  Return;
	SLONG  CorrPower;

	Return = FALSE;
	if (Motor[No].TachoCntUp) { // RampUp enabled

		Motor[No].State = LIMITED_REG_STEPUP;

		if (0 != Motor[No].Speed) {
			CorrPower = Motor[No].TargetPower - (SLONG)(Motor[No].Speed);
		} else {
			CorrPower = Motor[No].TargetPower;
		}

		// Number of powersteps per tacho count
		Motor[No].RampUpFactor = ((CorrPower * RAMP_FACTOR) + (Motor[No].TachoCntUp/2))/Motor[No].TachoCntUp;

		if (0 == Motor[No].RampUpFactor) {
			Motor[No].RampUpFactor = 1;
		}
		Motor[No].RampUpOffset = Motor[No].Speed;

		if (0 == Motor[No].Speed) {
			ClearPIDParameters(No);
		}

		if ((0 == Motor[No].TachoCntConst) && (0 == Motor[No].TachoCntDown) && (1 == Motor[No].TargetBrake)) {
			Motor[No].BrakeAfter = TRUE;
		} else {
			Motor[No].BrakeAfter = FALSE;
		}
		Motor[No].LockRampDown = FALSE;
		Return                 = TRUE;
	}

	return Return;
}

/*
 *  Helper function
 *  Checks if step power should have a constant power sequence
 *
 *  Parameters:
 *  No: The motor number
 *  AdjustTachoValue: The value that the tacho should be decremented
 */
static UBYTE StepSpeedCheckTachoCntConst(UBYTE No, SLONG AdjustTachoValue)
{
	UBYTE ReturnState;

	ReturnState = FALSE;

	if (Motor[No].TachoCntConst) {
		*StepPowerSteps[No]  -= AdjustTachoValue;
		Motor[No].TargetSpeed = Motor[No].TargetPower; //(Motor[No].TargetPower / SPEED_PWMCNT_REL);
		Motor[No].State       = LIMITED_REG_STEPCONST;
		ClearPIDParameters(No);

		if ((0 == Motor[No].TachoCntDown) && (1 == Motor[No].TargetBrake)) {
			Motor[No].BrakeAfter = TRUE;
		} else {
			Motor[No].BrakeAfter = FALSE;
		}
		Motor[No].LockRampDown = FALSE;
		ReturnState           = TRUE;
	}

	return ReturnState;
}

/*
 *  Helper function
 *  Checks if step power should ramp down or stop
 *
 *  Parameters:
 *  No: The motor number
 *  AdjustTachoValue: The value that the tacho should be decremented
 */
static void StepSpeedCheckTachoCntDown(UBYTE No, SLONG AdjustTachoValue)
{
	if (Motor[No].TachoCntDown) {
		Motor[No].RampDownFactor = ((Motor[No].TargetPower * RAMP_FACTOR) + (Motor[No].TachoCntDown/2))/Motor[No].TachoCntDown;

		*StepPowerSteps[No]     -= AdjustTachoValue;
		Motor[No].State          = LIMITED_REG_STEPDOWN;
		MinRegEnabled[No]        = FALSE;
		ClearPIDParameters(No);
	} else {
		StepPowerStopMotor(No, AdjustTachoValue);
	}
}

/*
 *  Helper function
 *  Checks if step power should have a Count up sequence
 *
 *  Parameters:
 *  No: The motor number
 */
static UBYTE StepPowerCheckTachoCntUp(UBYTE No)
{
	UBYTE Return;

	Return = FALSE;

	if (Motor[No].TachoCntUp) { // RampUp enabled
		Motor[No].State        = LIMITED_UNREG_STEPUP;
		Motor[No].RampUpFactor = ((Motor[No].TargetPower * RAMP_FACTOR) + (Motor[No].TachoCntUp/2))/Motor[No].TachoCntUp;
		ClearPIDParameters(No);

		if ((0 == Motor[No].TachoCntConst) && (0 == Motor[No].TachoCntDown) && (1 == Motor[No].TargetBrake)) {
			Motor[No].BrakeAfter = TRUE;
		} else {
			Motor[No].BrakeAfter = FALSE;
		}
		Motor[No].LockRampDown = FALSE;
		Return                 = TRUE;
	}

	return Return;
}

/*
 *  Helper function
 *  Checks if step power should have a constant power sequence
 *
 *  Parameters:
 *  No: The motor number
 *  AdjustTachoValue: The value that the tacho should be decremented
 */
static UBYTE StepPowerCheckTachoCntConst(UBYTE No, SLONG AdjustTachoValue)
{
	UBYTE ReturnState;

	ReturnState = FALSE;

	if (Motor[No].TachoCntConst) {
		*StepPowerSteps[No] -= AdjustTachoValue;
		Motor[No].Power      = Motor[No].TargetPower;
		Motor[No].State      = LIMITED_UNREG_STEPCONST;
		SetPower(No, Motor[No].Power);
		ClearPIDParameters(No);

		if ((0 == Motor[No].TachoCntDown) && (1 == Motor[No].TargetBrake)) {
			Motor[No].BrakeAfter = TRUE;
		} else {
			Motor[No].BrakeAfter = FALSE;
		}
		Motor[No].LockRampDown = FALSE;
		ReturnState            = TRUE;
	}

	return ReturnState;
}

/*
 *  Helper function
 *  Checks if step power should ramp down or stop
 *
 *  Parameters:
 *  No: The motor number
 *  AdjustTachoValue: The value that the tacho should be decremented
 */
static void StepPowerCheckTachoCntDown(UBYTE No, SLONG AdjustTachoValue)
{
	if (Motor[No].TachoCntDown) {
		Motor[No].RampDownFactor  = ((Motor[No].TargetPower * RAMP_FACTOR) + (Motor[No].TachoCntDown/2))/Motor[No].TachoCntDown;
		*StepPowerSteps[No]      -= AdjustTachoValue;
		Motor[No].State           = LIMITED_UNREG_STEPDOWN;
		MinRegEnabled[No]         = FALSE;
		ClearPIDParameters(No);
	} else {
		StepPowerStopMotor(No, AdjustTachoValue);
	}
}

/*
 * Calculates the new motor power value (duty cycle)
 *
 *  - Regulation method:
 *    To be defined
 *
 *  - Parameters:
 *    - Input:
 *      - No:       Motor output number
 *
 *    -Output:
 *      - None
 */
static void dRegulateSpeed(UBYTE No)
{
	SLONG SpeedErr;

	if (TYPE_MINITACHO == Motor[No].Type) {
		SpeedErr       = ((SLONG)(Motor[No].TargetSpeed) - (SLONG)(Motor[No].Speed));
		Motor[No].PVal = SpeedErr * 4;
		Motor[No].IVal = ((Motor[No].IVal * 9)/10) + (SpeedErr / 3);
		Motor[No].DVal = (((SpeedErr - (Motor[No].OldSpeedErr)) * 4)/2) * 40;
	} else {
		SpeedErr       = ((SLONG)(Motor[No].TargetSpeed) - (SLONG)(Motor[No].Speed));
		Motor[No].PVal = SpeedErr * 2;
		Motor[No].IVal = ((Motor[No].IVal * 9)/10) + (SpeedErr / 4);
		Motor[No].DVal = (((SpeedErr - (Motor[No].OldSpeedErr)) * 4)/2) * 40;
	}

	Motor[No].Power = Motor[No].Power + ((Motor[No].PVal + Motor[No].IVal + Motor[No].DVal));

	if (Motor[No].Power > MAX_PWM_CNT) {
		Motor[No].Power = MAX_PWM_CNT;
	}

	if (Motor[No].Power < -MAX_PWM_CNT) {
		Motor[No].Power = -MAX_PWM_CNT;
	}
	Motor[No].OldSpeedErr = SpeedErr;

	if (((Motor[No].TargetSpeed) > 0) && (Motor[No].Power < 0)) {
		Motor[No].Power = 0;
	}

	if (((Motor[No].TargetSpeed) < 0) && (Motor[No].Power > 0)) {
		Motor[No].Power = 0;
	}

	SetRegulationPower(No, Motor[No].Power);
}

static void BrakeMotor(UBYTE No, SLONG TachoCnt)
{
	SLONG TmpTacho;

	TmpTacho = TachoCnt * 100;

	TmpTacho <<= 2;

	if (TmpTacho > MAX_PWM_CNT) {
		TmpTacho = MAX_PWM_CNT;
	}
	if (TmpTacho < -MAX_PWM_CNT) {
		TmpTacho = -MAX_PWM_CNT;
	}

	Motor[No].Power = 0 - TmpTacho;
	SetRegulationPower(No, Motor[No].Power);
}

static UBYTE RampDownToBrake(UBYTE No, SLONG CurrentCnt, SLONG TargetCnt, SLONG Dir)
{
	UBYTE    Status;

	Status = FALSE;

	if (TRUE == CheckLessThanSpecial(CurrentCnt, TargetCnt, Dir)) {

//		Motor[No].TargetSpeed = (SBYTE)((Motor[No].TachoCntConst - CurrentCnt));
		Motor[No].TargetSpeed = (SBYTE)(TargetCnt - CurrentCnt);

		if ((Motor[No].TargetSpeed > 5) || (Motor[No].TargetSpeed < -5)) {
			if (TRUE == CheckLessThanSpecial(Motor[No].TargetSpeed, Motor[No].Speed, Dir)) {
				Motor[No].TargetSpeed = 1 * Dir;
			}
		}

		dRegulateSpeed(No);
	} else {
		// Done ramping down to brake
		Status = TRUE;
	}

	return Status;
}

static void GetCompareCounts(UBYTE No, SLONG *Counts, SLONG *CompareCounts)
{
	*Counts = *StepPowerSteps[No];

	if (TRUE == Motor[No].BrakeAfter) {
		if (StepPowerSteps[No] != &(Motor[No].TachoCnt)) {
			// Run for time
			*CompareCounts = *Counts + (Motor[No].Speed);
		} else {
			// Run for tacho
			if (TYPE_TACHO == Motor[No].Type) {
				*CompareCounts = *Counts + ((Motor[No].Speed)/2);
			} else {
				*CompareCounts = *Counts + ((Motor[No].Speed/3));
			}
		}
	} else {
		*CompareCounts = *Counts;
	}
}

static void StopAndBrakeMotor(UBYTE MotorNo)
{
	ReadyStatus            &= ~(0x01 << MotorNo);
	TestStatus             &= ~(0x01 << MotorNo);
	Motor[MotorNo].Power    = 0;
	Motor[MotorNo].Speed    = 0;
	Motor[MotorNo].TachoCnt = 0;
	Motor[MotorNo].State    = BRAKED;
	CLEARTachoArray(MotorNo);
	SetPower(MotorNo, Motor[MotorNo].Power);
	SetBrake(MotorNo);
}

static void StopAndFloatMotor(UBYTE MotorNo)
{
	ReadyStatus            &= ~(0x01 << MotorNo);
	TestStatus             &= ~(0x01 << MotorNo);
	Motor[MotorNo].Power    = 0;
	Motor[MotorNo].Speed    = 0;
	Motor[MotorNo].TachoCnt = 0;
	Motor[MotorNo].State    = IDLE;
	CLEARTachoArray(MotorNo);
	SetPower(MotorNo, Motor[MotorNo].Power);
	SetCoast(MotorNo);
}

static void FloatSyncedMotors(void)
{
	UBYTE TmpNo0, TmpNo1;

	TmpNo0 = SyncMNos[0];
	TmpNo1 = SyncMNos[1];

	Motor[TmpNo0].Mutex = TRUE;
	Motor[TmpNo1].Mutex = TRUE;

	StopAndFloatMotor(TmpNo0);
	StopAndFloatMotor(TmpNo1);
	SyncMNos[0] = UNUSED_SYNC_MOTOR;
	SyncMNos[1] = UNUSED_SYNC_MOTOR;

	MaxSyncSpeed              = 0;
	Motor[TmpNo0].TargetSpeed = 0;
	Motor[TmpNo1].TargetSpeed = 0;

	Motor[TmpNo0].Mutex = FALSE;
	Motor[TmpNo1].Mutex = FALSE;
}

static void TestAndFloatSyncedMotors(UBYTE MotorBitField, UBYTE SyncCmd)
{
	// Only if motors are already sync'ed
	if ((SyncMNos[0] != UNUSED_SYNC_MOTOR) && (SyncMNos[1] != UNUSED_SYNC_MOTOR)) {
		if (TRUE == SyncCmd) {
			//This is new sync'ed motor command
			if (((MotorBitField & (0x01 << SyncMNos[0])) && (MotorBitField & (0x01 << SyncMNos[1])))) {
				// Same motors used -> don't stop motors
			} else {
				// Only 2 motors can be sync'ed at a time -> stop sync'ed command
				FloatSyncedMotors();
			}
		} else {
			if ((MotorBitField & (0x01 << SyncMNos[0])) || (MotorBitField & (0x01 << SyncMNos[1]))) {
				// Same motors used -> stop sync'ed command
				FloatSyncedMotors();
			}
		}
#if 0
		// Check if new sync'ed motor command uses same motors as previous sync cmd
		if ((MotorBitField & (0x01 << SyncMNos[0])) && (MotorBitField & (0x01 << SyncMNos[1]))) {
			// Check if only one of the sync'ed motors are affected by the new motor cmd
			if (((MotorBitField & (0x01 << SyncMNos[0])) || (MotorBitField & (0x01 << SyncMNos[1]))) || (TRUE == SyncCmd)) {
				UBYTE TmpNo0, TmpNo1;

				// The motor is in sync'ed mode -> float both sync'ed motors
				TmpNo0 = SyncMNos[0];
				TmpNo1 = SyncMNos[1];

				Motor[TmpNo0].Mutex = TRUE;
				Motor[TmpNo1].Mutex = TRUE;

				StopAndFloatMotor(TmpNo0);
				StopAndFloatMotor(TmpNo1);
				SyncMNos[0] = UNUSED_SYNC_MOTOR;
				SyncMNos[1] = UNUSED_SYNC_MOTOR;

				MaxSyncSpeed              = 0;
				Motor[TmpNo0].TargetSpeed = 0;
				Motor[TmpNo1].TargetSpeed = 0;

				Motor[TmpNo0].Mutex = FALSE;
				Motor[TmpNo1].Mutex = FALSE;
			}
		}
#endif
	}
}

/*
 *  Motor timer interrupt function
 *
 *  Handles all motor regulation and timing related functionality
 */
static enum hrtimer_restart Device1TimerInterrupt1(struct hrtimer *pTimer)
{
	UBYTE No;
	UBYTE Test;

	static SLONG volatile TmpTacho;
	static SLONG volatile Tmp;

	hrtimer_forward_now(pTimer, Device1Time);
	for (No = 0; No < OUTPUTS; No++) {
		TmpTacho = Motor[No].IrqTacho;
		Tmp      = (TmpTacho - Motor[No].OldTachoCnt);

		Motor[No].TachoCnt     += Tmp;
		Motor[No].TachoSensor  += Tmp;
		Motor[No].OldTachoCnt   = TmpTacho;

		/* Update shared memory */
		pMotor[No].TachoCounts  = Motor[No].TachoCnt;
		pMotor[No].Speed        = Motor[No].Speed;
		pMotor[No].TachoSensor  = Motor[No].TachoSensor;

		Motor[No].TimeCnt      += Motor[No].TimeInc;  // Add or sub so that TimerCnt is 1 mS resolution

		if (FALSE == Motor[No].Mutex) {
			Test = dCalculateSpeed(No, &(Motor[No].Speed));
			switch (Motor[No].State) {
			case UNLIMITED_UNREG:
				if (Motor[No].TargetPower != Motor[No].Power) {
					Motor[No].Power  = Motor[No].TargetPower;
					SetPower(No, Motor[No].Power);
				}
				Motor[No].TachoCnt = 0;
				Motor[No].TimeCnt  = 0;
				break;

			case UNLIMITED_REG:
				dRegulateSpeed(No);
				Motor[No].TachoCnt = 0;
				Motor[No].TimeCnt  = 0;
				break;

			case LIMITED_REG_STEPUP:
			{
				UBYTE Status;
				SLONG StepCnt;
				SLONG StepCntTst;

				// Status used to check if ramp up has completed
				Status  = FALSE;

				if (StepPowerSteps[No] != &(Motor[No].TachoCnt)) {
					Motor[No].TachoCnt =  0;
				} else {
					Motor[No].TimeCnt = 0;
				}

				GetCompareCounts(No, &StepCnt, &StepCntTst);

				if ((TRUE == CheckLessThanSpecial(StepCntTst, Motor[No].TachoCntUp, Motor[No].Dir)) && (FALSE == Motor[No].LockRampDown)) {
					Motor[No].TargetSpeed = ((StepCnt * (Motor[No].RampUpFactor))/ RAMP_FACTOR) + Motor[No].RampUpOffset;
					if (TRUE == CheckLessThanSpecial((SLONG)Motor[No].TargetSpeed, ((SLONG)6 * Motor[No].Dir), Motor[No].Dir)) {
						//Ensure minimum speed
						Motor[No].TargetSpeed = 6 * Motor[No].Dir;
					}
					dRegulateSpeed(No);
				} else {
					if (TRUE == Motor[No].BrakeAfter) {

						Motor[No].LockRampDown = TRUE;
						Status = RampDownToBrake(No, StepCnt, Motor[No].TachoCntUp, Motor[No].Dir);
					} else {
						Status = TRUE;
					}
				}

				if (TRUE == Status) {
					// Ramp up completed check for next step
					if (FALSE == StepSpeedCheckTachoCntConst(No, Motor[No].TachoCntUp)) {
						StepSpeedCheckTachoCntDown(No, Motor[No].TachoCntUp);
					}
				}
			}
				break;

			case LIMITED_REG_STEPCONST:
			{
				SLONG   StepCnt, StepCntTst;

				if (StepPowerSteps[No] != &(Motor[No].TachoCnt)) {
					Motor[No].TachoCnt = 0;
				} else {
					Motor[No].TimeCnt  = 0;
				}

				GetCompareCounts(No, &StepCnt, &StepCntTst);

				if ((TRUE == CheckLessThanSpecial(StepCntTst, Motor[No].TachoCntConst, Motor[No].Dir)) && (FALSE == Motor[No].LockRampDown)) {
					dRegulateSpeed(No);
				} else {
					if (TRUE == Motor[No].BrakeAfter) {
						Motor[No].LockRampDown = TRUE;
						if (TRUE == RampDownToBrake(No, StepCnt, Motor[No].TachoCntConst, Motor[No].Dir)) {
							StepSpeedCheckTachoCntDown(No, Motor[No].TachoCntConst);
						}
					} else {
						StepSpeedCheckTachoCntDown(No, Motor[No].TachoCntConst);
					}
				}
			}
				break;

			case LIMITED_REG_STEPDOWN:
			{
				SLONG   StepCnt;
				SBYTE   NewSpeed;

				StepCnt = *StepPowerSteps[No];

				if (StepPowerSteps[No] != &(Motor[No].TachoCnt)) {
					Motor[No].TachoCnt = 0;
				} else {
					Motor[No].TimeCnt =  0;
				}

				if (TRUE == CheckLessThanSpecial(StepCnt, Motor[No].TachoCntDown, Motor[No].Dir)) {
					NewSpeed = Motor[No].TargetPower - ((StepCnt * (Motor[No].RampDownFactor))/ RAMP_FACTOR);
					if (TRUE == CheckLessThanSpecial((4 * Motor[No].Dir), NewSpeed, Motor[No].Dir)) {
						Motor[No].TargetSpeed = NewSpeed;
					}
					dRegulateSpeed(No);
				} else {
					StepPowerStopMotor(No, Motor[No].TachoCntDown);
				}
			}
				break;

			case LIMITED_UNREG_STEPUP:
			{
				UBYTE  Status;
				SLONG  StepCnt;
				SLONG  StepCntTst;

				// Status used to check if ramp up has completed
				Status  = FALSE;

				if (StepPowerSteps[No] != &(Motor[No].TachoCnt)) {
					Motor[No].TachoCnt = 0;
				} else {
					Motor[No].TimeCnt =  0;
				}

				GetCompareCounts(No, &StepCnt, &StepCntTst);

				if ((TRUE == CheckLessThanSpecial(StepCntTst, Motor[No].TachoCntUp, Motor[No].Dir)) && (FALSE == Motor[No].LockRampDown)) {
					if (0 != Motor[No].Speed) {
						if (TRUE == CheckLessThanSpecial(Motor[No].Power, (StepCnt * (Motor[No].RampUpFactor))/*/RAMP_FACTOR*/, Motor[No].Dir)) {
							// if very slow ramp up then power could be calculated as 0 for a while
							// avoid starting and stopping
							Motor[No].Power = (StepCnt * (Motor[No].RampUpFactor))/RAMP_FACTOR;

						}

						if (TRUE == CheckLessThanSpecial(Motor[No].Power, 0, Motor[No].Dir)) {
							// Avoid motor turning the wrong way
							Motor[No].Power = Motor[No].Power * -1;
						}
					} else {
						Motor[No].Power += (20 * Motor[No].Dir);
					}
					SetPower(No, Motor[No].Power);
				} else {
					// Done Stepping up
					Motor[No].LockRampDown = TRUE;
					if (TRUE == Motor[No].BrakeAfter) {
						Status = RampDownToBrake(No, StepCnt, Motor[No].TachoCntUp, Motor[No].Dir);
					} else {
						Status = TRUE;
					}
				}

				if (TRUE == Status) {
					// Ramp up completed check for next step
					if (FALSE == StepPowerCheckTachoCntConst(No, Motor[No].TachoCntUp)) {
						StepPowerCheckTachoCntDown(No, Motor[No].TachoCntUp);
					}
				}
			}
				break;

			case LIMITED_UNREG_STEPCONST:
			{
				SLONG StepCnt, StepCntTst;

				if (StepPowerSteps[No] != &(Motor[No].TachoCnt)) {
					Motor[No].TachoCnt =  0;
				} else {
					Motor[No].TimeCnt = 0;
				}

				GetCompareCounts(No, &StepCnt, &StepCntTst);

				if ((TRUE == CheckLessThanSpecial(StepCntTst, Motor[No].TachoCntConst, Motor[No].Dir)) && (FALSE == Motor[No].LockRampDown)) {
				} else {
					if (TRUE == Motor[No].BrakeAfter) {

						Motor[No].LockRampDown = TRUE;
						if (TRUE == RampDownToBrake(No, StepCnt, Motor[No].TachoCntConst, Motor[No].Dir)) {
							StepPowerCheckTachoCntDown(No, Motor[No].TachoCntConst);
						}
					} else {
						StepPowerCheckTachoCntDown(No, Motor[No].TachoCntConst);
					}
				}
			}
				break;

			case LIMITED_UNREG_STEPDOWN:
			{
				SLONG StepCnt;

				StepCnt = *StepPowerSteps[No];

				if (StepPowerSteps[No] != &(Motor[No].TachoCnt)) {
					Motor[No].TachoCnt = 0;
				} else {
					Motor[No].TimeCnt =  0;
				}

				if (TRUE == CheckLessThanSpecial(StepCnt, Motor[No].TachoCntDown, Motor[No].Dir)) {
					if ((TRUE == CheckLessThanSpecial((2 * Motor[No].Dir), Motor[No].Speed, Motor[No].Dir)) && (FALSE == MinRegEnabled[No])) {
						if (TRUE == CheckLessThanSpecial((4 * Motor[No].Dir), (Motor[No].TargetPower - ((StepCnt * (Motor[No].RampDownFactor))/ RAMP_FACTOR)), Motor[No].Dir)) {
							Motor[No].Power = Motor[No].TargetPower - ((StepCnt * (Motor[No].RampDownFactor))/ RAMP_FACTOR);
						}
						SetPower(No, Motor[No].Power);
					} else {

						MinRegEnabled[No]     = TRUE;
						Motor[No].TargetSpeed = (2 * Motor[No].Dir);
						dRegulateSpeed(No);
					}
				} else {
					StepPowerStopMotor(No, Motor[No].TachoCntDown);
				}
			}
				break;

			case LIMITED_STEP_SYNC:
			{
				// Here motor are syncronized and supposed to drive straight
				UBYTE   Cnt;
				UBYTE   Status;
				SLONG   StepCnt;

				StepCnt = *StepPowerSteps[No];

				if (StepPowerSteps[No] != &(Motor[No].TachoCnt)) {
					Motor[No].TachoCnt = 0;
				} else {
					Motor[No].TimeCnt =  0;
				}

				Status  = FALSE;

				if ((Motor[SyncMNos[0]].Power > (MAX_PWM_CNT - 100)) || (Motor[SyncMNos[0]].Power < (-MAX_PWM_CNT + 100))) {
					// Regulation is stretched to the limit....... Checked in both directions
					Status = TRUE;
					if (TRUE == CheckLessThanSpecial((SLONG)(Motor[SyncMNos[0]].Speed), (SLONG)(Motor[SyncMNos[0]].TargetSpeed), Motor[SyncMNos[0]].Dir)) {
						if (TRUE == CheckLessThanSpecial((SLONG)(Motor[SyncMNos[0]].Speed), (SLONG)(MaxSyncSpeed), Motor[SyncMNos[0]].Dir)) {
							// Check for running the same direction as target speed
							if (((Motor[SyncMNos[0]].Speed <= 0) && (Motor[SyncMNos[0]].TargetSpeed <= 0)) ||
									((Motor[SyncMNos[0]].Speed >= 0) && (Motor[SyncMNos[0]].TargetSpeed >= 0)))
							{
								MaxSyncSpeed = Motor[SyncMNos[0]].Speed;
							}
						}
					}
				}
				if ((Motor[SyncMNos[1]].Power > (MAX_PWM_CNT - 100)) || (Motor[SyncMNos[1]].Power < (-MAX_PWM_CNT + 100))) {
					// Regulation is stretched to the limit....... Checked in both directions
					Status = TRUE;
					if (TRUE == CheckLessThanSpecial((SLONG)(Motor[SyncMNos[1]].Speed), (SLONG)(Motor[SyncMNos[1]].TargetSpeed), (Motor[SyncMNos[0]].Dir * Motor[SyncMNos[1]].Dir))) {
						if (TRUE == CheckLessThanSpecial((SLONG)(Motor[SyncMNos[1]].Speed), (SLONG)(MaxSyncSpeed), (Motor[SyncMNos[0]].Dir * Motor[SyncMNos[1]].Dir))) {
							// Check for running the same direction as target speed
							if (((Motor[SyncMNos[1]].Speed <= 0) && (Motor[SyncMNos[1]].TargetSpeed <= 0)) ||
									((Motor[SyncMNos[1]].Speed >= 0) && (Motor[SyncMNos[1]].TargetSpeed >= 0)))
							{
								MaxSyncSpeed = Motor[SyncMNos[1]].Speed;
							}
						}
					}
				}

				if (FALSE == Status) {
					if (TRUE == CheckLessThanSpecial((SLONG)(MaxSyncSpeed), Motor[SyncMNos[0]].TargetPower, Motor[SyncMNos[0]].Dir)) {
						// TargetSpeed has been reduced but now there is room to increase
						IncSpeed(Motor[SyncMNos[0]].Dir, &MaxSyncSpeed);
					}
				}

				for (Cnt = 0; Cnt < MAX_SYNC_MOTORS; Cnt++) {
					//Update all motors to the max sync speed
					Motor[SyncMNos[Cnt]].TargetSpeed = MaxSyncSpeed;
				}

				if (TRUE == CheckLessThanSpecial(Motor[SyncMNos[1]].TachoCnt, Motor[SyncMNos[0]].TachoCnt, Motor[SyncMNos[0]].Dir)) {
					DecSpeed(Motor[SyncMNos[0]].Dir, &(Motor[SyncMNos[0]].TargetSpeed));
				}

				if (TRUE == CheckLessThanSpecial(Motor[SyncMNos[0]].TachoCnt, Motor[SyncMNos[1]].TachoCnt, Motor[SyncMNos[0]].Dir)) {
					DecSpeed(Motor[SyncMNos[0]].Dir, &(Motor[SyncMNos[1]].TargetSpeed));
				}

				dRegulateSpeed(SyncMNos[0]);
				dRegulateSpeed(SyncMNos[1]);

				CheckforEndOfSync();

			}
				break;

			case SYNCED_SLAVE:
				if (StepPowerSteps[No] != &(Motor[No].TachoCnt)) {
					Motor[No].TachoCnt = 0;
				} else {
					Motor[No].TimeCnt =  0;
				}
				break;

			case LIMITED_DIFF_TURN_SYNC:
			{
				UBYTE Status;
				SLONG StepCnt;

				StepCnt = *StepPowerSteps[No];

				if (StepPowerSteps[No] != &(Motor[No].TachoCnt)) {
					Motor[No].TachoCnt = 0;
				} else {
					Motor[No].TimeCnt =  0;
				}

				Status  = FALSE;

				if ((Motor[SyncMNos[0]].Power > (MAX_PWM_CNT - 100)) || (Motor[SyncMNos[0]].Power < (-MAX_PWM_CNT + 100))) {
					// Regulation is stretched to the limit....... in both directions
					Status = TRUE;
					if (TRUE == CheckLessThanSpecial((SLONG)(Motor[SyncMNos[0]].Speed), (SLONG)(Motor[SyncMNos[0]].TargetSpeed), Motor[SyncMNos[0]].Dir)) {
						if (TRUE == CheckLessThanSpecial((SLONG)(Motor[SyncMNos[0]].Speed), (SLONG)(MaxSyncSpeed), Motor[SyncMNos[0]].Dir)) {
							// Check for running the same direction as target speed
							if (((Motor[SyncMNos[0]].Speed <= 0) && (Motor[SyncMNos[0]].TargetSpeed <= 0)) ||
									((Motor[SyncMNos[0]].Speed >= 0) && (Motor[SyncMNos[0]].TargetSpeed >= 0)))
							{
								MaxSyncSpeed = Motor[SyncMNos[0]].Speed;
							}
						}
					}
				}

				if ((Motor[SyncMNos[1]].Power > (MAX_PWM_CNT - 100)) || (Motor[SyncMNos[1]].Power < (-MAX_PWM_CNT + 100))) {
					// Regulation is stretched to the limit....... in both directions
					Status = TRUE;
					if (TRUE == CheckLessThanSpecial((SLONG)(Motor[SyncMNos[1]].Speed), (SLONG)(Motor[SyncMNos[1]].TargetSpeed), (Motor[SyncMNos[1]].Dir * Motor[SyncMNos[0]].Dir))) {
						if (TRUE == CheckLessThanSpecial((SLONG)((Motor[SyncMNos[1]].Speed * 100)/(Motor[SyncMNos[1]].TurnRatio * Motor[SyncMNos[1]].Dir)), (SLONG)(MaxSyncSpeed), Motor[SyncMNos[0]].Dir)) {
							if ((0 == Motor[SyncMNos[1]].TurnRatio) || (0 == Motor[SyncMNos[1]].Speed)) {
								MaxSyncSpeed = 0;
							} else {
								// Check for running the same direction as target speed
								if (((Motor[SyncMNos[1]].Speed <= 0) && (Motor[SyncMNos[1]].TargetSpeed <= 0)) ||
										((Motor[SyncMNos[1]].Speed >= 0) && (Motor[SyncMNos[1]].TargetSpeed >= 0)))
								{
									MaxSyncSpeed = ((Motor[SyncMNos[1]].Speed * 100)/Motor[SyncMNos[1]].TurnRatio) * Motor[SyncMNos[1]].Dir;
								}
							}
						}
					}
				}

				if (FALSE == Status) {
					if (TRUE == CheckLessThanSpecial((SLONG)(MaxSyncSpeed), Motor[SyncMNos[0]].TargetPower, Motor[SyncMNos[0]].Dir)) {
						// TargetSpeed has been reduced but now there is room to increase
						IncSpeed(Motor[SyncMNos[0]].Dir, &MaxSyncSpeed);
					}
				}

				// Set the new
				Motor[SyncMNos[0]].TargetSpeed = MaxSyncSpeed;
				if ((0 == Motor[SyncMNos[1]].TurnRatio) || (0 == MaxSyncSpeed)) {
					Motor[SyncMNos[1]].TargetSpeed = 0;
				} else {
					Motor[SyncMNos[1]].TargetSpeed = ((MaxSyncSpeed * Motor[SyncMNos[1]].TurnRatio)/100) * Motor[SyncMNos[1]].Dir;
				}

				if (0 != Motor[SyncMNos[1]].TurnRatio) {
					if (TRUE == CheckLessThanSpecial((((Motor[SyncMNos[1]].TachoCnt * 100)/Motor[SyncMNos[1]].TurnRatio) * Motor[SyncMNos[1]].Dir), Motor[SyncMNos[0]].TachoCnt, Motor[SyncMNos[0]].Dir)) {
						DecSpeed(Motor[SyncMNos[0]].Dir, &(Motor[SyncMNos[0]].TargetSpeed));
					}
				}

				if (0 != Motor[SyncMNos[1]].TurnRatio) {
					if (TRUE == CheckLessThanSpecial(Motor[SyncMNos[0]].TachoCnt, ((Motor[SyncMNos[1]].TachoCnt * 100)/(Motor[SyncMNos[1]].TurnRatio) * Motor[SyncMNos[1]].Dir), Motor[SyncMNos[0]].Dir)) {
						DecSpeed(Motor[SyncMNos[1]].Dir * Motor[SyncMNos[0]].Dir, &(Motor[SyncMNos[1]].TargetSpeed));
					}
				}

				dRegulateSpeed(SyncMNos[0]);
				dRegulateSpeed(SyncMNos[1]);

				CheckforEndOfSync();

			}
				break;

			case RAMP_DOWN_SYNC:
			{
				SLONG Count0, Count1;

				if (StepPowerSteps[No] != &(Motor[No].TachoCnt)) {
					Motor[No].TachoCnt = 0;
				} else {
					Motor[No].TimeCnt  = 0;
				}

				// Duration is either dependent on timer ticks or tacho counts
				GetSyncDurationCnt(&Count0, &Count1);

				if (TRUE == CheckLessThanSpecial(Count0, Motor[SyncMNos[0]].TachoCntConst, Motor[SyncMNos[0]].Dir)) {

					RampDownToBrake(SyncMNos[0], Count0, Motor[SyncMNos[0]].TachoCntConst, Motor[SyncMNos[0]].Dir);

					if (StepPowerSteps[SyncMNos[0]] == TimerSteps[SyncMNos[0]]) {
						// Needs to adjust second synchronised in time mode - both motor needs to run for the same
						// amount of time but not at the same speed
						RampDownToBrake(SyncMNos[1], ((Count1 * Motor[SyncMNos[1]].TurnRatio)/100), (( Motor[SyncMNos[1]].TachoCntConst * Motor[SyncMNos[1]].TurnRatio)/100), (Motor[SyncMNos[0]].Dir * Motor[SyncMNos[1]].Dir));
					} else {
						RampDownToBrake(SyncMNos[1], Count1, Motor[SyncMNos[1]].TachoCntConst, (Motor[SyncMNos[0]].Dir * Motor[SyncMNos[1]].Dir));
					}
				} else {
					MaxSyncSpeed                    = 0;
					Motor[SyncMNos[0]].TargetSpeed  = 0;
					Motor[SyncMNos[1]].TargetSpeed  = 0;
					StepPowerStopMotor(SyncMNos[0], Motor[SyncMNos[0]].TachoCntConst);
					StepPowerStopMotor(SyncMNos[1], Motor[SyncMNos[1]].TachoCntConst);
				}
			}
				break;

			case STOP_MOTOR:
				if (PrgStopTimer[No]) {
					PrgStopTimer[No]--;
				} else {
					Motor[No].State        = IDLE;
					Motor[No].TargetState  = UNLIMITED_UNREG;
					SetCoast(No);
				}
				break;

			case BRAKED:
				BrakeMotor(No, Motor[No].TachoCnt);
				break;

			case IDLE:
				/* Intentionally left empty */
				break;
			default:
				/* Intentionally left empty */
				break;
			}
		}
	}

	return HRTIMER_RESTART;
}

static void GetSyncDurationCnt(SLONG *pCount0, SLONG *pCount1)
{
	if (StepPowerSteps[SyncMNos[0]] == TimerSteps[SyncMNos[0]]) {
		*pCount0 = *TimerSteps[SyncMNos[0]];
		*pCount1 = *TimerSteps[SyncMNos[1]];
	} else {
		*pCount0 = *TachoSteps[SyncMNos[0]];
		*pCount1 = *TachoSteps[SyncMNos[1]];
	}
}

static void CheckforEndOfSync(void)
{
	SLONG Count0, Count1;
	SBYTE Speed;

	// Duration is either dependent on timer ticks or tacho counts
	GetSyncDurationCnt(&Count0, &Count1);

	if (StepPowerSteps[SyncMNos[0]] == TimerSteps[SyncMNos[0]]) {
		Speed = (Motor[SyncMNos[0]].Speed);
	} else {
		Speed = (Motor[SyncMNos[0]].Speed/2);
	}

	if ((TRUE == CheckLessThanSpecial(Count0 + (SLONG)(Speed), Motor[SyncMNos[0]].TachoCntConst, Motor[SyncMNos[0]].Dir)) ||
			(0 == Motor[SyncMNos[0]].TachoCntConst))
	{
		/* do nothing */
	} else {
		Motor[SyncMNos[0]].State = RAMP_DOWN_SYNC;
	}
}

/*
 *  VALID COMMANDS:
 *
 *  opPROGRAM_STOP:       User program stopped -> either brake or float motors
 *  opPROGRAM_START:      User program started -> reset motor parameters
 *  opOUTPUT_SET_TYPE:    Set motor type, and reset all parameters upon motor type change
 *  opOUTPUT_RESET:       Resets the output tacho counters
 *  opOUTPUT_CLR_COUNT:   Resets the tacho count related to when motor is used as a sensor
 *  opOUTPUT_STOP:        Stops the motors - either Braked or coasted
 *  opOUTPUT_POWER:       Sets the power - this command so not start the motor
 *  opOUTPUT_SPEED:       Sets the Speed - setpoint for regulation - this command do not start the motor
 *  opOUTPUT_START:       Starts the motor if not started
 *  opOUTPUT_POLARITY:    Sets the polarity of the motor - this command do not start the motor
 *  opOUTPUT_STEP_POWER:  Runs the motor un-regulated with ramp up const and down according to the tacho
 *  opOUTPUT_TIME_POWER:  Runs the motor un-regulated with ramp up const and down according to time
 *  opOUTPUT_STEP_SPEED:  Runs the motor regulated with ramp up const and down according to the tacho
 *  opOUTPUT_TIME_SPEED:  Runs the motor regulated with ramp up const and down according to the time
 *  opOUTPUT_STEP_SYNC:   Runs two motors regulated and syncronized, duration as specified by tacho cnts
 *  opOUTPUT_TIME_SYNC:   Runs two motors regulated and syncronized, duration as specified by time
 *  Default state:        TBD
 */
static ssize_t Device1Write(struct file *File, const char *Buffer, size_t Count,
			    loff_t *Data)
{

	SBYTE Buf[20];
	int Lng = 0;
	UBYTE Tmp;
	int ret;

	ret = copy_from_user(Buf, Buffer, Count);
	if (ret < 0)
		return ret;

	switch ((UBYTE)(Buf[0])) {

	case opPROGRAM_STOP:
		for (Tmp = 0; Tmp < OUTPUTS; Tmp++) {

			Motor[Tmp].Mutex  = TRUE;
			ReadyStatus      &= ~(0x01 << Tmp);   // Clear Ready flag
			TestStatus       &= ~(0x01 << Tmp);   // Clear Test flag

			Motor[Tmp].Power       = 0;
			Motor[Tmp].Speed       = 0;
			MaxSyncSpeed           = 0;
			Motor[Tmp].TargetSpeed = 0;

			if (((IDLE == Motor[Tmp].State) && ((OutputRead(Tmp, OUTPUT_PORT_PIN1)) && (OutputRead(Tmp, OUTPUT_PORT_PIN2)))) || (BRAKED == Motor[Tmp].State)) {
				PrgStopTimer[Tmp]       = 100/SOFT_TIMER_MS;
				Motor[Tmp].State        = STOP_MOTOR;
				Motor[Tmp].TargetState  = UNLIMITED_UNREG;
				SetBrake(Tmp);
			} else {
				Motor[Tmp].State        = IDLE;
				Motor[Tmp].TargetState  = UNLIMITED_UNREG;
				SetCoast(Tmp);
			}
			Motor[Tmp].Mutex = FALSE;
		}
		/* fallthrough */

	case opPROGRAM_START:
		for (Tmp = 0; Tmp < OUTPUTS; Tmp++) {
			Motor[Tmp].Pol = 1;
		}
		SyncMNos[0] = UNUSED_SYNC_MOTOR;
		SyncMNos[1] = UNUSED_SYNC_MOTOR;
		break;

	case opOUTPUT_SET_TYPE:
		for (Tmp = 0; Tmp < OUTPUTS; Tmp++) {
			if (Buf[Tmp + 1] != Motor[Tmp].Type) {
				Motor[Tmp].Mutex = TRUE;
				if ((TYPE_TACHO == Buf[Tmp + 1]) || (TYPE_MINITACHO == Buf[Tmp + 1])) {
					// There is a motor attached the port
					CLEARTachoArray(Tmp);
					//  Motor types can be: TYPE_TACHO, TYPE_NONE, TYPE_MINITACHO
					SETMotorType(Tmp, Buf[Tmp + 1]);
					//  Switching motor => speed = 0
					SETAvgTachoCount(Tmp, 0);
				} else {
					// No motor is connected
					Motor[Tmp].State = IDLE;
					SetCoast(Tmp);
				}
				Motor[Tmp].Type         = Buf[Tmp + 1];

				// All counts are reset when motor type changes
				Motor[Tmp].TachoCnt     = 0;
				pMotor[Tmp].TachoCounts = 0;
				Motor[Tmp].TimeCnt      = 0;
				pMotor[Tmp].TachoSensor = 0;
				Motor[Tmp].TachoSensor  = 0;
				Motor[Tmp].Mutex        = FALSE;
			}
		}
		break;

	case opOUTPUT_RESET:
		TestAndFloatSyncedMotors(Buf[1], FALSE);

		for (Tmp = 0;Tmp < OUTPUTS;Tmp++) {
			if (Buf[1] & (1 << Tmp)) {
				Motor[Tmp].Mutex        = TRUE;
				Motor[Tmp].TachoCnt     = 0;
				pMotor[Tmp].TachoCounts = 0;
				Motor[Tmp].TimeCnt      = 0;
				Motor[Tmp].Mutex        = FALSE;
			}
		}
		break;

	case opOUTPUT_CLR_COUNT:
		for (Tmp = 0;Tmp < OUTPUTS;Tmp++) {
			if (Buf[1] & (1 << Tmp)) {
				Motor[Tmp].Mutex        = TRUE;
				pMotor[Tmp].TachoSensor = 0;
				Motor[Tmp].TachoSensor  = 0;
				Motor[Tmp].Mutex        = FALSE;
			}
		}
		break;

	case opOUTPUT_STOP:
		TestAndFloatSyncedMotors(Buf[1], FALSE);

		for (Tmp = 0;Tmp < OUTPUTS;Tmp++) {
			if (Buf[1] & (1 << Tmp)) {
				Motor[Tmp].Mutex     = TRUE;

				if (Buf[2]) {
					StopAndBrakeMotor(Tmp);
				} else {
					StopAndFloatMotor(Tmp);
				}

				Motor[Tmp].Mutex = FALSE;
			}
		}
		break;

	case opOUTPUT_POWER:
		TestAndFloatSyncedMotors(Buf[1], FALSE);

		CheckSpeedPowerLimits(&(Buf[2]));

		for (Tmp = 0;Tmp < OUTPUTS;Tmp++) {
			if (Buf[1] & (1 << Tmp)) {
				Motor[Tmp].Mutex       = TRUE;
				Motor[Tmp].TargetPower = (SLONG)(Buf[2]) * (SLONG)(Motor[Tmp].Pol) * (SLONG)SPEED_PWMCNT_REL;

				if ((IDLE == Motor[Tmp].State) || (BRAKED == Motor[Tmp].State)) {
					Motor[Tmp].TargetState = UNLIMITED_UNREG;
				}
				Motor[Tmp].Mutex = FALSE;
			}
		}
		break;

	case opOUTPUT_SPEED:
		TestAndFloatSyncedMotors(Buf[1], FALSE);

		CheckSpeedPowerLimits(&(Buf[2]));

		for (Tmp = 0;Tmp < OUTPUTS;Tmp++) {
			if (Buf[1] & (1 << Tmp)) {
				Motor[Tmp].Mutex        = TRUE;
				Motor[Tmp].TargetSpeed  = (Buf[2]) * (Motor[Tmp].Pol);
				if ((IDLE == Motor[Tmp].State) || (BRAKED == Motor[Tmp].State)) {
					Motor[Tmp].TargetState = UNLIMITED_REG;
				}
				Motor[Tmp].Mutex = FALSE;
			}
		}
		break;

	case opOUTPUT_START:
		for (Tmp = 0;Tmp < OUTPUTS;Tmp++) {
			if (Buf[1] & (1 << Tmp)) {
				Motor[Tmp].Mutex = TRUE;
				if ((IDLE == Motor[Tmp].State) || (BRAKED == Motor[Tmp].State)) {
					Motor[Tmp].State  = Motor[Tmp].TargetState;
				}
				Motor[Tmp].Mutex = FALSE;
			}
		}
		break;

	case opOUTPUT_POLARITY:
		TestAndFloatSyncedMotors(Buf[1], FALSE);

		for (Tmp = 0;Tmp < OUTPUTS;Tmp++) {
			if (Buf[1] & (1 << Tmp)) {
				Motor[Tmp].Mutex = TRUE;
				if (0 == (SBYTE)Buf[2]) {
					/* 0 == Toggle */
					Motor[Tmp].TargetPower = (Motor[Tmp].TargetPower) * -1;
					Motor[Tmp].TargetSpeed = (Motor[Tmp].TargetSpeed) * -1;

					if (1 == Motor[Tmp].Pol) {
						Motor[Tmp].Pol = -1;
					} else {
						Motor[Tmp].Pol =  1;
					}
				} else {

					if (Motor[Tmp].Pol != (SBYTE)Buf[2]) {
						Motor[Tmp].TargetPower  = (Motor[Tmp].TargetPower) * -1;
						Motor[Tmp].TargetSpeed  = (Motor[Tmp].TargetSpeed) * -1;
						Motor[Tmp].Pol          = Buf[2];
					}
				}
				SetPower(Tmp, Motor[Tmp].TargetPower);
				Motor[Tmp].Mutex = FALSE;
			}
		}
		break;

	case opOUTPUT_STEP_POWER:
	{
		STEPPOWER   StepPower;

		memcpy((UBYTE*)(&(StepPower.Cmd)), &Buf[0], sizeof(StepPower));

		TestAndFloatSyncedMotors(StepPower.Nos, FALSE);

		CheckSpeedPowerLimits(&(StepPower.Power));

		// Adjust if there is inconsistency between power and steps
		if (((StepPower.Power < 0) && ((StepPower.Step1 > 0) || (StepPower.Step2 > 0) || (StepPower.Step3 > 0))) ||
				((StepPower.Power > 0) && ((StepPower.Step1 < 0) || (StepPower.Step2 < 0) || (StepPower.Step3 < 0))))
		{
			StepPower.Step1 *= -1;
			StepPower.Step2 *= -1;
			StepPower.Step3 *= -1;
		}

		for (Tmp = 0;Tmp < OUTPUTS;Tmp++) {
			if (StepPower.Nos & (1 << Tmp)) {
				Motor[Tmp].Mutex = TRUE;

				StepPowerSteps[Tmp] = TachoSteps[Tmp];

				ReadyStatus |= (0x01 << Tmp);   // Set Ready flag
				TestStatus  |= (0x01 << Tmp);   // Set Test flag

				Motor[Tmp].TargetPower   = StepPower.Power * SPEED_PWMCNT_REL * (Motor[Tmp].Pol);
				Motor[Tmp].TachoCntUp    = StepPower.Step1 * (Motor[Tmp].Pol);
				Motor[Tmp].TachoCntConst = StepPower.Step2 * (Motor[Tmp].Pol);
				Motor[Tmp].TachoCntDown  = StepPower.Step3 * (Motor[Tmp].Pol);
				Motor[Tmp].TargetBrake   = StepPower.Brake;
				if (Motor[Tmp].TargetPower >= 0) {
					Motor[Tmp].Dir = 1;
				} else {
					Motor[Tmp].Dir = -1;
				}

				TimeOutSpeed0[Tmp] = ktime_get();

				if (FALSE == StepPowerCheckTachoCntUp(Tmp)) {
					SLONG AdjustTachoValue = 0;
					if (FALSE == StepPowerCheckTachoCntConst(Tmp, AdjustTachoValue)) {
						StepPowerCheckTachoCntDown(Tmp, AdjustTachoValue);
					}
				}
				Motor[Tmp].Mutex = FALSE;
			}
		}
	}
		break;

	case opOUTPUT_TIME_POWER:
	{
		TIMEPOWER   TimePower;
		SLONG       Inc;

		memcpy((UBYTE*)(&(TimePower.Cmd)), &Buf[0], sizeof(TimePower));

		TestAndFloatSyncedMotors(TimePower.Nos, FALSE);

		// Adjust if there is inconsistency between power and Time
		if (((TimePower.Power < 0) && ((TimePower.Time1 > 0) || (TimePower.Time2 > 0) || (TimePower.Time3 > 0))) ||
				((TimePower.Power > 0) && ((TimePower.Time1 < 0) || (TimePower.Time2 < 0) || (TimePower.Time3 < 0))))
		{
			TimePower.Time1 *= -1;
			TimePower.Time2 *= -1;
			TimePower.Time3 *= -1;
		}

		if ((TimePower.Time1 > 0) || (TimePower.Time2 > 0) || (TimePower.Time3 > 0)) {
			Inc =  SOFT_TIMER_MS;
		} else {
			Inc = -SOFT_TIMER_MS;
		}

		for (Tmp = 0;Tmp < OUTPUTS;Tmp++) {
			if (TimePower.Nos & (1 << Tmp)) {
				Motor[Tmp].Mutex = TRUE;

				StepPowerSteps[Tmp]  = TimerSteps[Tmp];
				Motor[Tmp].TimeInc   = Inc * (Motor[Tmp].Pol);
				*StepPowerSteps[Tmp] = 0;

				ReadyStatus |= (0x01 << Tmp);   // Set Ready flag
				TestStatus  |= (0x01 << Tmp);   // Set Test flag

				Motor[Tmp].TargetPower   = TimePower.Power * SPEED_PWMCNT_REL * (Motor[Tmp].Pol);
				Motor[Tmp].TachoCntUp    = TimePower.Time1 * (Motor[Tmp].Pol);
				Motor[Tmp].TachoCntConst = TimePower.Time2 * (Motor[Tmp].Pol);
				Motor[Tmp].TachoCntDown  = TimePower.Time3 * (Motor[Tmp].Pol);
				Motor[Tmp].TargetBrake   = TimePower.Brake;

				if (Motor[Tmp].TargetPower >= 0) {
					Motor[Tmp].Dir = 1;
				} else {
					Motor[Tmp].Dir = -1;
				}

				TimeOutSpeed0[Tmp] = ktime_get();

				if (FALSE == StepPowerCheckTachoCntUp(Tmp)) {
					SLONG AdjustTachoValue = 0;
					if (FALSE == StepPowerCheckTachoCntConst(Tmp, AdjustTachoValue)) {
						StepPowerCheckTachoCntDown(Tmp, AdjustTachoValue);
					}
				}
				Motor[Tmp].Mutex = FALSE;
			}
		}
	}
		break;

	case opOUTPUT_STEP_SPEED:
	{
		STEPSPEED   StepSpeed;

		memcpy((UBYTE*)(&(StepSpeed.Cmd)), &Buf[0], sizeof(StepSpeed));

		TestAndFloatSyncedMotors(StepSpeed.Nos, FALSE);

		CheckSpeedPowerLimits(&(StepSpeed.Speed));

		// Adjust if there is inconsistency between Speed and Steps
		if (((StepSpeed.Speed < 0) && ((StepSpeed.Step1 > 0) || (StepSpeed.Step2 > 0) || (StepSpeed.Step3 > 0))) ||
				((StepSpeed.Speed > 0) && ((StepSpeed.Step1 < 0) || (StepSpeed.Step2 < 0) || (StepSpeed.Step3 < 0))))
		{
			StepSpeed.Step1 *= -1;
			StepSpeed.Step2 *= -1;
			StepSpeed.Step3 *= -1;
		}

		for (Tmp = 0;Tmp < OUTPUTS;Tmp++) {
			if (StepSpeed.Nos & (1 << Tmp)) {
				Motor[Tmp].Mutex = TRUE;

				StepPowerSteps[Tmp] = TachoSteps[Tmp];

				ReadyStatus |= (0x01 << Tmp);   // Set Ready flag
				TestStatus  |= (0x01 << Tmp);   // Set Test flag

				Motor[Tmp].TargetPower   = StepSpeed.Speed * (Motor[Tmp].Pol);
				Motor[Tmp].TachoCntUp    = StepSpeed.Step1 * (Motor[Tmp].Pol);
				Motor[Tmp].TachoCntConst = StepSpeed.Step2 * (Motor[Tmp].Pol);
				Motor[Tmp].TachoCntDown  = StepSpeed.Step3 * (Motor[Tmp].Pol);
				Motor[Tmp].TargetBrake   = StepSpeed.Brake;

				if (Motor[Tmp].TargetPower >= 0) {
					Motor[Tmp].Dir = 1;
				} else {
					Motor[Tmp].Dir = -1;
				}

				TimeOutSpeed0[Tmp] = ktime_get();

				if (FALSE == StepSpeedCheckTachoCntUp(Tmp)) {
					SLONG AdjustTachoValue = 0;
					if (FALSE == StepSpeedCheckTachoCntConst(Tmp, AdjustTachoValue)) {
						StepSpeedCheckTachoCntDown(Tmp, AdjustTachoValue);
					}
				}
				Motor[Tmp].TargetSpeed = Motor[Tmp].TargetPower;
				Motor[Tmp].Mutex       = FALSE;
			}
		}
	}
		break;

	case opOUTPUT_TIME_SPEED:
	{
		SLONG       Inc;
		TIMESPEED   TimeSpeed;

		memcpy((UBYTE*)(&(TimeSpeed.Cmd)), &Buf[0], sizeof(TimeSpeed));

		TestAndFloatSyncedMotors(TimeSpeed.Nos, FALSE);

		CheckSpeedPowerLimits(&(TimeSpeed.Speed));

		// Adjust if there is inconsistency between Speed and Time
		if (((TimeSpeed.Speed < 0) && ((TimeSpeed.Time1 > 0) || (TimeSpeed.Time2 > 0) || (TimeSpeed.Time3 > 0))) ||
				((TimeSpeed.Speed > 0) && ((TimeSpeed.Time1 < 0) || (TimeSpeed.Time2 < 0) || (TimeSpeed.Time3 < 0))))
		{
			TimeSpeed.Time1 *= -1;
			TimeSpeed.Time2 *= -1;
			TimeSpeed.Time3 *= -1;
		}

		if ((TimeSpeed.Time1 > 0) || (TimeSpeed.Time2 > 0) || (TimeSpeed.Time3 > 0)) {
			Inc = SOFT_TIMER_MS;
		} else {
			Inc = -SOFT_TIMER_MS;
		}

		for (Tmp = 0;Tmp < OUTPUTS;Tmp++) {
			if (TimeSpeed.Nos & (1 << Tmp)) {
				Motor[Tmp].Mutex = TRUE;

				StepPowerSteps[Tmp]  = TimerSteps[Tmp];
				Motor[Tmp].TimeInc   = Inc * (Motor[Tmp].Pol);
				*StepPowerSteps[Tmp] = 0;

				ReadyStatus |= (0x01 << Tmp);   // Set Ready flag
				TestStatus  |= (0x01 << Tmp);   // Set Test flag

				Motor[Tmp].TargetPower   = TimeSpeed.Speed * (Motor[Tmp].Pol);
				Motor[Tmp].TachoCntUp    = TimeSpeed.Time1 * (Motor[Tmp].Pol);
				Motor[Tmp].TachoCntConst = TimeSpeed.Time2 * (Motor[Tmp].Pol);
				Motor[Tmp].TachoCntDown  = TimeSpeed.Time3 * (Motor[Tmp].Pol);
				Motor[Tmp].TargetBrake   = TimeSpeed.Brake;

				if (Motor[Tmp].TargetPower >= 0) {
					Motor[Tmp].Dir = 1;
				} else {
					Motor[Tmp].Dir = -1;
				}

				TimeOutSpeed0[Tmp] = ktime_get();

				if (FALSE == StepSpeedCheckTachoCntUp(Tmp)) {
					SLONG AdjustTachoValue = 0;
					if (FALSE == StepSpeedCheckTachoCntConst(Tmp, AdjustTachoValue)) {
						StepSpeedCheckTachoCntDown(Tmp, AdjustTachoValue);
					}
				}
				Motor[Tmp].TargetSpeed = Motor[Tmp].TargetPower;
				Motor[Tmp].Mutex       = FALSE;
			}
		}
	}
		break;

	case opOUTPUT_STEP_SYNC:
	{
		UBYTE       No  = 0;
		STEPSYNC    StepSync;

		memcpy((UBYTE*)(&(StepSync.Cmd)), &Buf[0], sizeof(StepSync));

		TestAndFloatSyncedMotors(StepSync.Nos, TRUE);

		//Check if exceeding speed limits
		CheckSpeedPowerLimits(&(StepSync.Speed));

		if (((StepSync.Speed < 0) && (StepSync.Step > 0)) ||
				((StepSync.Speed > 0) && (StepSync.Step < 0)))
		{
			StepSync.Step *= -1;
		}

		for (Tmp = 0;Tmp < OUTPUTS;Tmp++) {

			if (StepSync.Nos & (1 << Tmp)) {
				Motor[Tmp].Mutex = TRUE;

				ReadyStatus           |= (0x01 << Tmp);   // Set Ready flag
				TestStatus            |= (0x01 << Tmp);   // Set Test flag
				StepPowerSteps[Tmp]    = TachoSteps[Tmp];
				Motor[Tmp].TargetBrake =  StepSync.Brake;

				if (0 == StepSync.Step) {
					// If run forever
					*StepPowerSteps[Tmp] = 0;
				}

				if (0 == StepSync.Turn) {
					// Synced motors are going straight
					Motor[Tmp].TargetPower   = StepSync.Speed;
					Motor[Tmp].TurnRatio     = 100;
					Motor[Tmp].TachoCntConst = StepSync.Step;
					SyncMNos[No]             = Tmp;
					TimeOutSpeed0[Tmp]       = ktime_get();
					MaxSyncSpeed             = StepSync.Speed;

					// Find the direction the main motor will drive
					if (0 <= StepSync.Speed) {
						Motor[Tmp].Dir = NON_INV;
					} else {
						Motor[Tmp].Dir = INV;
					}

					if (0 == No) {
						Motor[Tmp].State = LIMITED_STEP_SYNC;
					} else {
						Motor[Tmp].State = SYNCED_SLAVE;
						Motor[Tmp].Dir   = NON_INV;
					}
				} else {
					// Turning
					UBYTE MotorIndex;

					if (0 < StepSync.Turn) {
						MotorIndex = No;
					} else {
						// Invert if turning left (right motor runs fastest)
						MotorIndex = 1 - No;
					}

					if (0 == MotorIndex) {
						if (StepSync.Speed > 0) {
							Motor[Tmp].Dir = NON_INV;
						} else {
							Motor[Tmp].Dir = INV;
						}
						Motor[Tmp].TurnRatio = StepSync.Turn;
					} else {
						if ((StepSync.Turn < 100) && (StepSync.Turn > -100)) {
							Motor[Tmp].Dir = NON_INV;

							if (StepSync.Turn > 0)       // Invert the ratio in the first quarter
							{
								Motor[Tmp].TurnRatio = 100 - StepSync.Turn;
							} else {
								Motor[Tmp].TurnRatio = -100 - StepSync.Turn;
							}
						} else {
							if ((StepSync.Turn >= 100) || (StepSync.Turn <= -100)) {
								// Both motors are turning
								Motor[Tmp].Dir = INV;
								if (StepSync.Turn <= -100) {
									Motor[Tmp].TurnRatio = (StepSync.Turn + 100);
								} else {
									Motor[Tmp].TurnRatio = (StepSync.Turn - 100);
								}
							}
						}
					}

					SyncMNos[MotorIndex] = Tmp;

					if (0 > StepSync.Turn) {
						Motor[Tmp].TurnRatio *= -1;
					}

					if (0 == MotorIndex) {
						Motor[Tmp].TargetPower   = StepSync.Speed;
						Motor[Tmp].TachoCntConst = StepSync.Step;
						MaxSyncSpeed             = StepSync.Speed;
						Motor[Tmp].State         = LIMITED_DIFF_TURN_SYNC;
					} else {
						Motor[Tmp].TargetPower   = (SLONG)(((SLONG)(StepSync.Speed) * (SLONG)(Motor[Tmp].TurnRatio))/((SLONG)100 * Motor[Tmp].Dir));
						Motor[Tmp].TachoCntConst = ((StepSync.Step  * (SLONG)(Motor[Tmp].TurnRatio))/100) * Motor[Tmp].Dir;
						Motor[Tmp].State         = SYNCED_SLAVE;
					}

					TimeOutSpeed0[Tmp] = ktime_get();
				}
				No++;
			}
		}
		Motor[SyncMNos[0]].Mutex     = FALSE;
		Motor[SyncMNos[1]].Mutex     = FALSE;
	}
		break;

	case opOUTPUT_TIME_SYNC:
	{
		SLONG     Inc;
		UBYTE     No  = 0;
		TIMESYNC  TimeSync;

		memcpy((UBYTE*)(&(TimeSync.Cmd)), &Buf[0], sizeof(TimeSync));

		TestAndFloatSyncedMotors(TimeSync.Nos, TRUE);

		//Check if exceeding speed limits
		CheckSpeedPowerLimits(&(TimeSync.Speed));

		if (((TimeSync.Speed < 0) && (TimeSync.Time > 0)) ||
				((TimeSync.Speed > 0) && (TimeSync.Time < 0)))
		{
			TimeSync.Time *= -1;
		}

		if (TimeSync.Time > 0) {
			Inc = SOFT_TIMER_MS;
		} else {
			Inc = -SOFT_TIMER_MS;
		}

		for (Tmp = 0;Tmp < OUTPUTS;Tmp++) {
			if (TimeSync.Nos & (1 << Tmp)) {
				Motor[Tmp].Mutex      = TRUE;

				ReadyStatus          |= (0x01 << Tmp);   // Set Ready flag
				TestStatus           |= (0x01 << Tmp);   // Set Test flag

				StepPowerSteps[Tmp]   = TimerSteps[Tmp];
				Motor[Tmp].TimeInc    = Inc;
				*TimerSteps[Tmp]      = 0;

				Motor[Tmp].TargetBrake =  TimeSync.Brake;

				if (0 == TimeSync.Turn) {
					// Synced motors are going straight
					Motor[Tmp].TargetPower   = TimeSync.Speed;
					Motor[Tmp].TurnRatio     = 100;
					Motor[Tmp].TachoCntConst = TimeSync.Time;
					SyncMNos[No]             = Tmp;
					TimeOutSpeed0[Tmp]       = ktime_get();
					MaxSyncSpeed             = TimeSync.Speed;

					// Find the direction the main motor will drive
					if (0 <= TimeSync.Speed) {
						Motor[Tmp].Dir = NON_INV;
					} else {
						Motor[Tmp].Dir = INV;
					}

					if (0 == No) {
						Motor[Tmp].State = LIMITED_STEP_SYNC;
					} else {
						Motor[Tmp].State = SYNCED_SLAVE;
						Motor[Tmp].Dir   = NON_INV;
					}
				} else {
					//   Turning
					UBYTE MotorIndex;

					if (0 < TimeSync.Turn) {
						MotorIndex = No;
					} else {
						// Invert if turning left (right motor runs fastest)
						MotorIndex = 1 - No;
					}

					if (0 == MotorIndex) {
						if (TimeSync.Speed > 0) {
							Motor[Tmp].Dir = NON_INV;
						} else {
							Motor[Tmp].Dir = INV;
						}
						Motor[Tmp].TurnRatio = TimeSync.Turn;
					} else {
						if ((TimeSync.Turn < 100) && (TimeSync.Turn > -100)) {
							Motor[Tmp].Dir = NON_INV;

							if (TimeSync.Turn > 0)       // Invert the ratio in the first quarter
							{
								Motor[Tmp].TurnRatio = 100 - TimeSync.Turn;
							} else {
								Motor[Tmp].TurnRatio = -100 - TimeSync.Turn;
							}
						} else {
							if ((TimeSync.Turn >= 100) || (TimeSync.Turn <= -100)) {
								// Both motors are turning
								Motor[Tmp].Dir = INV;
								if (TimeSync.Turn <= -100) {
									Motor[Tmp].TurnRatio = (TimeSync.Turn + 100);
								} else {
									Motor[Tmp].TurnRatio = (TimeSync.Turn - 100);
								}
							}
						}
					}

					SyncMNos[MotorIndex] = Tmp;

					if (0 > TimeSync.Turn) {
						Motor[Tmp].TurnRatio *= -1;
					}

					if (0 == MotorIndex) {
						Motor[Tmp].TargetPower   = TimeSync.Speed;
						Motor[Tmp].TachoCntConst = TimeSync.Time;
						MaxSyncSpeed             = TimeSync.Speed;
						Motor[Tmp].State         = LIMITED_DIFF_TURN_SYNC;
					} else {
						Motor[Tmp].TargetPower    = (SLONG)(((SLONG)(TimeSync.Speed) * (SLONG)(Motor[Tmp].TurnRatio))/((SLONG)100 * Motor[Tmp].Dir));
						Motor[Tmp].TachoCntConst  = TimeSync.Time  * Motor[Tmp].Dir;
						Motor[Tmp].TimeInc       *= Motor[Tmp].Dir;
						Motor[Tmp].State          = SYNCED_SLAVE;
					}
					TimeOutSpeed0[Tmp] = ktime_get();
				}
				No++;
			}
		}
		Motor[SyncMNos[0]].Mutex     = FALSE;
		Motor[SyncMNos[1]].Mutex     = FALSE;
	}
		break;

	default:
		break;
	}

	return Lng;
}

static ssize_t Device1Read(struct file *File, char *Buffer, size_t Count,
			   loff_t *Offset)
{
	char Buf[10];
	int Lng, ret;

	Lng = snprintf(&Buf[0], Count, "%01u ", ReadyStatus);
	Lng += snprintf(&Buf[Lng], Count - Lng, "%01u ", TestStatus);

	ret = copy_to_user(Buffer, Buf, Lng);
	if (ret < 0)
		return ret;

	return Lng;
}

static const struct file_operations Device1Entries = {
	.owner	= THIS_MODULE,
	.read	= Device1Read,
	.write	= Device1Write,
};

static struct miscdevice Device1 = {
	.minor	= MISC_DYNAMIC_MINOR,
	.name	= DEVICE1_NAME,
	.fops	= &Device1Entries,
};

static int Device1Init(void)
{
	int ret;
	UBYTE Tmp = 0;

	Device1Lms2012CompatDev = lms2012_compat_get();
	if (!Device1Lms2012CompatDev)
		return -EPROBE_DEFER;

	Device1Lms2012Compat = dev_get_drvdata(Device1Lms2012CompatDev);

	for (Tmp = 0; Tmp < OUTPUTS; Tmp++) {
		memset(&Motor[Tmp], 0, sizeof(MOTOR));

		Motor[Tmp].TargetBrake = COAST;
		Motor[Tmp].Pol         = 1;
		Motor[Tmp].Direction   = FORWARD;
		Motor[Tmp].Type        = TYPE_NONE;
		Motor[Tmp].State       = IDLE;
		Motor[Tmp].TargetState = UNLIMITED_UNREG; //default startup state
		Motor[Tmp].Mutex       = FALSE;
		Motor[Tmp].BrakeAfter  = FALSE;

		CLEARTachoArray(Tmp);
		SETMotorType(Tmp, TYPE_NONE); //  Motor types can be: TYPE_TACHO, TYPE_NONE, TYPE_MINITACHO
		SETAvgTachoCount(Tmp, 0);     //  At initialisation speed is assumed to be zero
	}

	/* Float the tacho inputs */
	for (Tmp = 0; Tmp < OUTPUTS; Tmp++) {
		pwm_config(Device1Lms2012Compat->out_pwms[Tmp], 0, PWM_PERIOD);
		pwm_enable(Device1Lms2012Compat->out_pwms[Tmp]);
		OutputFloat(Tmp, OUTPUT_PORT_PIN6);
		OutputFloat(Tmp, OUTPUT_PORT_PIN5R);
		SetCoast(Tmp);
	}

	SyncMNos[0] = UNUSED_SYNC_MOTOR;
	SyncMNos[1] = UNUSED_SYNC_MOTOR;

	// Setup interrupt for the tacho int pins
	SetGpioRisingIrq(0, OUTPUT_PORT_PIN6, IntA);
	SetGpioRisingIrq(1, OUTPUT_PORT_PIN6, IntB);
	SetGpioRisingIrq(2, OUTPUT_PORT_PIN6, IntC);
	SetGpioRisingIrq(3, OUTPUT_PORT_PIN6, IntD);

	ret = misc_register(&Device1);

	return ret;
}

static void Device1Exit(void)
{
	int i;

	misc_deregister(&Device1);
	hrtimer_cancel(&Device1Timer);
	for (i = 0; i < OUTPUTS; i++) {
		SetCoast(i);
		pwm_disable(Device1Lms2012Compat->out_pwms[i]);
	}
	put_device(Device1Lms2012CompatDev);
}

static void SetGpioRisingIrq(UBYTE Port, UBYTE PinNo, irqreturn_t (*IntFuncPtr)(int, void *))
{
	UWORD Status;
	int irq;

	irq = gpiod_to_irq(Device1Lms2012Compat->out_pins[Port]->desc[PinNo]);

	Status = request_irq(irq, IntFuncPtr, 0, "PWM_DEVICE", NULL);
	if (Status < 0) {
		printk("error %d requesting GPIO IRQ %d\n", Status, PinNo);
	}
	irq_set_irq_type(irq, IRQ_TYPE_EDGE_RISING | IRQ_TYPE_EDGE_FALLING);
}

/*
 *  Tacho A interrupt function
 *
 *  Tacho count is incremented or decremented on both positive
 *  and negative edges of the OUTPUT_PORT_PIN6 signal.
 *
 *  For each positive and negative edge of the OUTPUT_PORT_PIN6 tacho signal
 *  a timer is sampled. this is used to calculate the speed later on.
 *
 *  DirChgPtr is implemented for ensuring that there is enough
 *  samples in the same direction to calculate a speed.
 */
static irqreturn_t IntA (int irq, void * dev)
{
	UBYTE TmpPtr;
	ULONG IntAState;
	ULONG DirAState;
	ktime_t Timer;

	// Sample all necessary items as fast as possible
	IntAState = READIntA;
	DirAState = READDirA;
	Timer     = ktime_get();

	TmpPtr = (TachoSamples[0].ArrayPtr + 1) & (NO_OF_TACHO_SAMPLES-1);
	TachoSamples[0].TachoArray[TmpPtr] = Timer;
	TachoSamples[0].ArrayPtr           = TmpPtr;

	if ((35 < Motor[0].Speed) || (-35 > Motor[0].Speed)) {
		if (FORWARD == Motor[0].Direction) {
			(Motor[0].IrqTacho)++;
		} else {
			(Motor[0].IrqTacho)--;
		}
		if (Motor[0].DirChgPtr < SamplesPerSpeed[0][SAMPLES_ABOVE_SPEED_75]) {
			Motor[0].DirChgPtr++;
		}
	} else {
		if (IntAState) {
			if (DirAState) {
				if (FORWARD == Motor[0].Direction) {
					if (Motor[0].DirChgPtr < SamplesPerSpeed[0][SAMPLES_ABOVE_SPEED_75]) {
						Motor[0].DirChgPtr++;
					}
				} else {
					Motor[0].DirChgPtr = 0;
				}
				(Motor[0].IrqTacho)++;
				Motor[0].Direction = FORWARD;
			} else {
				if (BACKWARD == Motor[0].Direction) {
					TachoSamples[0].TachoArray[TmpPtr] = Timer;
					TachoSamples[0].ArrayPtr = TmpPtr;
					if (Motor[0].DirChgPtr < SamplesPerSpeed[0][SAMPLES_ABOVE_SPEED_75]) {
						Motor[0].DirChgPtr++;
					}
				} else {
					Motor[0].DirChgPtr = 0;
				}
				(Motor[0].IrqTacho)--;
				Motor[0].Direction = BACKWARD;
			}
		} else {
			if (DirAState) {
				if (BACKWARD == Motor[0].Direction) {
					if (Motor[0].DirChgPtr < SamplesPerSpeed[0][SAMPLES_ABOVE_SPEED_75]) {
						Motor[0].DirChgPtr++;
					}
				} else {
					Motor[0].DirChgPtr = 0;
				}
				(Motor[0].IrqTacho)--;
				Motor[0].Direction = BACKWARD;
			} else {
				if (FORWARD == Motor[0].Direction) {
					if (Motor[0].DirChgPtr < SamplesPerSpeed[0][SAMPLES_ABOVE_SPEED_75]) {
						Motor[0].DirChgPtr++;
					}
				} else {
					Motor[0].DirChgPtr = 0;
				}
				(Motor[0].IrqTacho)++;
				Motor[0].Direction = FORWARD;
			}
		}
	}

	return IRQ_HANDLED;
}

static irqreturn_t IntB (int irq, void * dev)
{
	UBYTE TmpPtr;
	ULONG IntBState;
	ULONG DirBState;
	ktime_t Timer;

	// Sample all necessary items as fast as possible
	IntBState = READIntB;
	DirBState = READDirB;
	Timer     = ktime_get();

	TmpPtr = (TachoSamples[1].ArrayPtr + 1) & (NO_OF_TACHO_SAMPLES-1);
	TachoSamples[1].TachoArray[TmpPtr] = Timer;
	TachoSamples[1].ArrayPtr           = TmpPtr;

	if ((35 < Motor[1].Speed) || (-35 > Motor[1].Speed)) {
		if (FORWARD == Motor[1].Direction) {
			(Motor[1].IrqTacho)++;
		} else {
			(Motor[1].IrqTacho)--;
		}

		if (Motor[1].DirChgPtr < SamplesPerSpeed[1][SAMPLES_ABOVE_SPEED_75]) {
			Motor[1].DirChgPtr++;
		}
	} else {
		if (IntBState) {
			if (DirBState) {
				if (FORWARD == Motor[1].Direction) {
					if (Motor[1].DirChgPtr < SamplesPerSpeed[1][SAMPLES_ABOVE_SPEED_75]) {
						Motor[1].DirChgPtr++;
					}
				} else {
					Motor[1].DirChgPtr = 0;
				}
				(Motor[1].IrqTacho)++;
				Motor[1].Direction = FORWARD;
			} else {
				if (BACKWARD == Motor[1].Direction) {
					if (Motor[1].DirChgPtr < SamplesPerSpeed[1][SAMPLES_ABOVE_SPEED_75]) {
						Motor[1].DirChgPtr++;
					}
				} else {
					Motor[1].DirChgPtr = 0;
				}
				(Motor[1].IrqTacho)--;
				Motor[1].Direction = BACKWARD;
			}
		} else {
			if (DirBState) {
				if (BACKWARD == Motor[1].Direction) {
					if (Motor[1].DirChgPtr < SamplesPerSpeed[1][SAMPLES_ABOVE_SPEED_75]) {
						Motor[1].DirChgPtr++;
					}
				} else {
					Motor[1].DirChgPtr = 0;
				}
				(Motor[1].IrqTacho)--;
				Motor[1].Direction = BACKWARD;
			} else {
				if (FORWARD == Motor[1].Direction) {
					if (Motor[1].DirChgPtr < SamplesPerSpeed[1][SAMPLES_ABOVE_SPEED_75]) {
						Motor[1].DirChgPtr++;
					}
				} else {
					Motor[1].DirChgPtr = 0;
				}
				(Motor[1].IrqTacho)++;
				Motor[1].Direction = FORWARD;
			}
		}
	}

	return IRQ_HANDLED;
}

static irqreturn_t IntC (int irq, void * dev)
{
	UBYTE TmpPtr;
	ULONG IntCState;
	ULONG DirCState;
	ktime_t Timer;

	// Sample all necessary items as fast as possible
	IntCState = READIntC;
	DirCState = READDirC;
	Timer     = ktime_get();

	TmpPtr = (TachoSamples[2].ArrayPtr + 1) & (NO_OF_TACHO_SAMPLES-1);
	TachoSamples[2].TachoArray[TmpPtr] = Timer;
	TachoSamples[2].ArrayPtr           = TmpPtr;

	if ((35 < Motor[2].Speed) || (-35 > Motor[2].Speed)) {
		if (FORWARD == Motor[2].Direction) {
			(Motor[2].IrqTacho)++;
		} else {
			(Motor[2].IrqTacho)--;
		}
		if (Motor[2].DirChgPtr < SamplesPerSpeed[2][SAMPLES_ABOVE_SPEED_75]) {
			Motor[2].DirChgPtr++;
		}
	} else {
		if (IntCState) {
			if (DirCState) {
				if (FORWARD == Motor[2].Direction) {
					if (Motor[2].DirChgPtr < SamplesPerSpeed[2][SAMPLES_ABOVE_SPEED_75]) {
						Motor[2].DirChgPtr++;
					}
				} else {
					Motor[2].DirChgPtr = 0;
				}
				(Motor[2].IrqTacho)++;
				Motor[2].Direction = FORWARD;
			} else {
				if (BACKWARD == Motor[2].Direction) {
					if (Motor[2].DirChgPtr < SamplesPerSpeed[2][SAMPLES_ABOVE_SPEED_75]) {
						Motor[2].DirChgPtr++;
					}
				} else {
					Motor[2].DirChgPtr = 0;
				}
				(Motor[2].IrqTacho)--;
				Motor[2].Direction = BACKWARD;
			}
		} else {
			if (DirCState) {
				if (BACKWARD == Motor[2].Direction) {
					if (Motor[2].DirChgPtr < SamplesPerSpeed[2][SAMPLES_ABOVE_SPEED_75]) {
						Motor[2].DirChgPtr++;
					}
				} else {
					Motor[2].DirChgPtr = 0;
				}
				(Motor[2].IrqTacho)--;
				Motor[2].Direction = BACKWARD;
			} else {
				if (FORWARD == Motor[2].Direction) {
					if (Motor[2].DirChgPtr < SamplesPerSpeed[2][SAMPLES_ABOVE_SPEED_75]) {
						Motor[2].DirChgPtr++;
					}
				} else {
					Motor[2].DirChgPtr = 0;
				}
				(Motor[2].IrqTacho)++;
				Motor[2].Direction = FORWARD;
			}
		}
	}

	return IRQ_HANDLED;
}

static irqreturn_t IntD (int irq, void * dev)
{
	UBYTE TmpPtr;
	ULONG IntDState;
	ULONG DirDState;
	ktime_t Timer;

	// Sample all necessary items as fast as possible
	IntDState = READIntD;
	DirDState = READDirD;
	Timer     = ktime_get();

	TmpPtr = (TachoSamples[3].ArrayPtr + 1) & (NO_OF_TACHO_SAMPLES-1);
	TachoSamples[3].TachoArray[TmpPtr] = Timer;
	TachoSamples[3].ArrayPtr           = TmpPtr;

	if ((35 < Motor[3].Speed) || (-35 > Motor[3].Speed)) {
		if (FORWARD == Motor[3].Direction) {
			(Motor[3].IrqTacho)++;
		} else {
			(Motor[3].IrqTacho)--;
		}
		if (Motor[3].DirChgPtr < SamplesPerSpeed[3][SAMPLES_ABOVE_SPEED_75]) {
			Motor[3].DirChgPtr++;
		}
	} else {
		if (IntDState) {
			if (DirDState) {
				if (FORWARD == Motor[3].Direction) {
					if (Motor[3].DirChgPtr < SamplesPerSpeed[3][SAMPLES_ABOVE_SPEED_75]) {
						Motor[3].DirChgPtr++;
					}
				} else {
					Motor[3].DirChgPtr = 0;
				}
				(Motor[3].IrqTacho)++;
				Motor[3].Direction = FORWARD;
			} else {
				if (BACKWARD == Motor[3].Direction) {
					if (Motor[3].DirChgPtr < SamplesPerSpeed[3][SAMPLES_ABOVE_SPEED_75]) {
						Motor[3].DirChgPtr++;
					}
				} else {
					Motor[3].DirChgPtr = 0;
				}
				(Motor[3].IrqTacho)--;
				Motor[3].Direction = BACKWARD;
			}
		} else {
			if (DirDState) {
				if (BACKWARD == Motor[3].Direction) {
					if (Motor[3].DirChgPtr < SamplesPerSpeed[3][SAMPLES_ABOVE_SPEED_75]) {
						Motor[3].DirChgPtr++;
					}
				} else {
					Motor[3].DirChgPtr = 0;
				}
				(Motor[3].IrqTacho)--;
				Motor[3].Direction = BACKWARD;
			} else {
				if (FORWARD == Motor[3].Direction) {
					if (Motor[3].DirChgPtr < SamplesPerSpeed[3][SAMPLES_ABOVE_SPEED_75]) {
						Motor[3].DirChgPtr++;
					}
				} else {
					Motor[3].DirChgPtr = 0;
				}
				(Motor[3].IrqTacho)++;
				Motor[3].Direction = FORWARD;
			}
		}
	}

	return IRQ_HANDLED;
}

static UBYTE dCalculateSpeed(UBYTE No, SBYTE *pSpeed)
{
	ktime_t Tmp1, Tmp2;
	ULONG Diff;
	UBYTE Ptr, Status;
	SWORD Speed;

	Status  = FALSE;
	Ptr     = TachoSamples[No].ArrayPtr;

	if (Motor[No].DirChgPtr >= 1) {
		Diff = ktime_to_us(ktime_sub(TachoSamples[No].TachoArray[Ptr], TachoSamples[No].TachoArray[((Ptr - 1) & (NO_OF_TACHO_SAMPLES - 1))]));
		if (Diff) {
			SETAvgTachoCount(No, (ULONG)((USecPerPulse[No] / Diff)));
		} else {
			SETAvgTachoCount(No, (ULONG)1);
		}
	}

	Speed   = *pSpeed;    // Maintain old speed if not changed
	Tmp1    = TachoSamples[No].TachoArray[Ptr];
	Tmp2    = TachoSamples[No].TachoArray[((Ptr - AVG_TACHO_COUNTS[No]) & (NO_OF_TACHO_SAMPLES - 1))];

	if ((Ptr != TachoSamples[No].ArrayPtrOld) && (Motor[No].DirChgPtr >= AVG_TACHO_COUNTS[No])) {

		Status                        = TRUE;
		TimeOutSpeed0[No]             = Tmp1;
		TachoSamples[No].ArrayPtrOld  = Ptr;

		Diff = ktime_to_us(ktime_sub(Tmp1, Tmp2));
		if (Diff) {
			Speed = (SWORD)((ULONG)(AVG_COUNTS[No])/(ULONG)Diff);
		} else {
			// Safety check
			Speed = 1;
		}

		if (Speed > 100) {
			Speed = 100;
		}

		if (BACKWARD == Motor[No].Direction) {
			Speed = 0 - Speed;
		}
	} else {
		// No new Values check for speed 0
		if ((USecPerPulse[No]) < ktime_to_us(ktime_sub(ktime_get(), TimeOutSpeed0[No]))) {
			TimeOutSpeed0[No]   = ktime_get();
			Speed               = 0;
			Motor[No].DirChgPtr = 0;
			Status              = TRUE;
		}
	}

	*pSpeed = (SBYTE)Speed;

	return Status;
}

/*
 * Only used for daisy chaining to ensure Busy flags being set from when message
 * has been received until being executed by the VM
 */
static ssize_t Device2Write(struct file *File, const char *Buffer, size_t Count,
			    loff_t *Data)
{
	int Lng = 0;
	SBYTE Buf[20];
	int ret;

	if (Count > 20)
		return -EINVAL;

	ret = copy_from_user(Buf, Buffer, Count);
	if (ret < 0)
		return ret;

	ReadyStatus |= Buf[0];   // Set Ready flag
	TestStatus  |= Buf[0];   // Set Test flag

	return Lng;
}

static ssize_t Device2Read(struct file *File, char *Buffer, size_t Count, loff_t *Offset)
{
	return 0;
}

#define SHM_LENGTH    (sizeof(MotorData))
#define NPAGES        ((SHM_LENGTH + PAGE_SIZE - 1) / PAGE_SIZE)

static void *kmalloc_ptr;

static int Device2Mmap(struct file *filp, struct vm_area_struct *vma)
{
	int ret;

	ret = remap_pfn_range(vma, vma->vm_start, virt_to_phys((void*)((unsigned long)pMotor)) >> PAGE_SHIFT, vma->vm_end-vma->vm_start, PAGE_SHARED);

	if (ret < 0)
		ret = -EAGAIN;

	return ret;
}

static const struct file_operations Device2Entries = {
	.owner	= THIS_MODULE,
	.read	= Device2Read,
	.write	= Device2Write,
	.mmap	= Device2Mmap,
};

static struct miscdevice Device2 = {
	.minor	= MISC_DYNAMIC_MINOR,
	.name	= DEVICE2_NAME,
	.fops	= &Device2Entries,
};

static int Device2Init(void)
{
	MOTORDATA *pTmp;
	int i, ret;

	kmalloc_ptr = kmalloc((NPAGES + 2) * PAGE_SIZE, GFP_KERNEL);
	if (!kmalloc_ptr)
		return -ENOMEM;

	pTmp = (MOTORDATA*)((((unsigned long)kmalloc_ptr) + PAGE_SIZE - 1) & PAGE_MASK);
	for (i = 0; i < NPAGES * PAGE_SIZE; i += PAGE_SIZE) {
		SetPageReserved(virt_to_page(((unsigned long)pTmp) + i));
	}
	pMotor = pTmp;
	memset(pMotor, 0, sizeof(MotorData));

	ret = misc_register(&Device2);
	if (ret < 0) {
		kfree(kmalloc_ptr);
		return ret;
	}

	return 0;
}

static void Device2Exit(void)
{
	MOTORDATA   *pTmp;
	int         i;

	misc_deregister(&Device2);

	pTmp   = pMotor;
	pMotor = MotorData;
	// free shared memory
	for (i = 0; i < NPAGES * PAGE_SIZE; i+= PAGE_SIZE) {
		ClearPageReserved(virt_to_page(((unsigned long)pTmp) + i));
	}

	kfree(kmalloc_ptr);
}

static int d_pwm_probe(struct platform_device *pdev)
{
	int ret;

	ret = Device1Init();
	if (ret < 0)
		return ret;

	ret = Device2Init();
	if (ret < 0) {
		Device1Exit();
		return ret;
	}

	// Setup timer irq
	Device1Time = ktime_set(0, SOFT_TIMER_SETUP);
	hrtimer_init(&Device1Timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	Device1Timer.function = Device1TimerInterrupt1;
	hrtimer_start(&Device1Timer, Device1Time, HRTIMER_MODE_REL);

	pr_info("d_pwm registered\n");

	return 0;
}

static int d_pwm_remove(struct platform_device *pdev)
{
	Device2Exit();
	Device1Exit();

	pr_info("d_pwm removed\n");

	return 0;
}

static struct platform_driver d_pwm_driver = {
	.driver	= {
		.name	= "d_pwm",
	},
	.probe	= d_pwm_probe,
	.remove	= d_pwm_remove,
};
module_platform_driver(d_pwm_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("LIU");
MODULE_DESCRIPTION(MODULE_NAME);
MODULE_ALIAS("platform:d_pwm");
