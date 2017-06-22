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
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 */

#include <linux/completion.h>
#include <linux/debugfs.h>
#include <linux/fs.h>
#include <linux/hrtimer.h>
#include <linux/init.h>
#include <linux/ioport.h>
#include <linux/kernel.h>
#include <linux/miscdevice.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/pinctrl/consumer.h>
#include <linux/platform_device.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/workqueue.h>

#include "lms2012.h"

#define MODULE_NAME "iic_module"
#define DEVICE1_NAME IIC_DEVICE

//*****************************************************************************

#define PINInput(port, pin) \
	gpiod_direction_input(Device1Lms2012Compat->in_pins[port]->desc[pin])
#define PINRead(port, pin) \
	gpiod_get_value(Device1Lms2012Compat->in_pins[port]->desc[pin])
#define PINHigh(port, pin) \
	gpiod_direction_output(Device1Lms2012Compat->in_pins[port]->desc[pin], 1)
#define PINLow(port, pin) \
	gpiod_direction_output(Device1Lms2012Compat->in_pins[port]->desc[pin], 0)

typedef struct {
	UWORD Timer;
	UWORD Time;
	UBYTE Initialised;
	UBYTE ChangeMode;
	UBYTE State;
	UBYTE OldState;
	UBYTE SubState;
	UBYTE Repeat;
	UBYTE Cmd;
	UBYTE Mode;
	UBYTE Addr;
	UBYTE Retries;
	SBYTE Name[TYPE_NAME_LENGTH + 1];
	UBYTE Reverse;
	UBYTE InLength;
	UBYTE InPointer;
	UBYTE OutLength;
	UBYTE OutPointer;
	UBYTE InBuffer[IIC_DATA_LENGTH];
	UBYTE OutBuffer[IIC_DATA_LENGTH];
} IICPORT;

typedef struct {
	UBYTE port;
	struct work_struct message_work;
	struct completion message_completion;
	struct i2c_msg i2c_msg[2];
	u8 msg0_buf[IIC_DATA_LENGTH];
	u8 msg1_buf[IIC_DATA_LENGTH];
	u8 num_msg;
	int xfer_result;
} IICCTRL;

static IICPORT IicPortDefault = {
	.Timer		= 0,
	.Time		= 0,
	.Initialised	= 0,
	.ChangeMode	= 0,
	.State		= 0,
	.OldState	= -1,
	.SubState	= 0,
	.Repeat		= 0,
	.Cmd		= 0,
	.Mode		= 0,
	.Addr		= 1,
	.Retries	= 3,
	.Name		= "",
	.Reverse	= 0,
	.InLength	= 0,
	.InPointer	= 0,
	.OutLength	= 0,
	.OutPointer	= 0,
};

static UBYTE IicConfigured[INPUTS];
static IICPORT IicPort[INPUTS];
static IICCTRL IicCtrl[INPUTS];

static IIC IicDefault;
static IIC *pIic = &IicDefault;

static struct device *Device1Lms2012CompatDev;
static struct lms2012_compat *Device1Lms2012Compat;

static void IicPortDisable(UBYTE Port)
{
	PINHigh(Port, INPUT_PORT_BUF);
	PINInput(Port, INPUT_PORT_PIN6);

	pinctrl_select_state(Device1Lms2012Compat->pinctrl[Port],
			     Device1Lms2012Compat->pinctrl_default[Port]);
}

static void IicPortEnable(UBYTE Port)
{
	pinctrl_select_state(Device1Lms2012Compat->pinctrl[Port],
			     Device1Lms2012Compat->pinctrl_i2c[Port]);

	PINLow(Port, INPUT_PORT_BUF);
	// PINHigh(Port, IIC_PORT_CLOCK);
	PINHigh(Port, INPUT_PORT_PIN6);
}

static void Device1MessageWork(struct work_struct *work)
{
	IICCTRL *ctrl = container_of(work, IICCTRL, message_work);

	ctrl->xfer_result =
		i2c_transfer(Device1Lms2012Compat->i2c_adapter[ctrl->port],
			     ctrl->i2c_msg, ctrl->num_msg);
	complete(&ctrl->message_completion);
}

static UBYTE IicPortBusy(UBYTE Port)
{
	UBYTE Result = 0;

	if (!completion_done(&IicCtrl[Port].message_completion))
		Result = 1;

	return Result;
}

static void IicPortSend(UBYTE Port)
{
	u16 addr = IicPort[Port].OutBuffer[0];
	u16 msg0_len = IicPort[Port].OutLength - 1;
	u16 msg1_len = IicPort[Port].InLength;

	IicCtrl[Port].i2c_msg[0].addr = addr;
	IicCtrl[Port].i2c_msg[0].flags = 0;
	IicCtrl[Port].i2c_msg[0].len = msg0_len;
	memcpy(IicCtrl[Port].msg0_buf, &IicPort[Port].OutBuffer[1], msg0_len);
	IicCtrl[Port].i2c_msg[0].buf = IicCtrl[Port].msg0_buf;

	IicCtrl[Port].i2c_msg[1].addr = addr;
	IicCtrl[Port].i2c_msg[1].flags = I2C_M_RD;
	IicCtrl[Port].i2c_msg[1].len = msg1_len;
	memcpy(IicCtrl[Port].msg1_buf, IicPort[Port].InBuffer, msg1_len);
	IicCtrl[Port].i2c_msg[1].buf = IicCtrl[Port].msg1_buf;

	IicCtrl[Port].num_msg = (msg1_len > 0) ? 2 : 1;

	reinit_completion(&IicCtrl[Port].message_completion);
	schedule_work(&IicCtrl[Port].message_work);
}

static RESULT IicPortReceive(UBYTE Port, UBYTE *pTmpBuffer)
{
	RESULT Result = BUSY;

	if (completion_done(&IicCtrl[Port].message_completion)) {
		memset(pTmpBuffer, 0, IIC_DATA_LENGTH);
		memcpy(pTmpBuffer, IicCtrl[Port].msg1_buf,
		       IicPort[Port].InLength);

		if (IicCtrl[Port].xfer_result == IicCtrl[Port].num_msg) {
			Result = OK;
		} else {
			Result = FAIL;
		}
	}

	return Result;
}

// DEVICE1 ********************************************************************

enum IIC_STATE {
	IIC_IDLE,
	IIC_INIT,
	IIC_RESTART,
	IIC_ENABLE,
	IIC_NXT_TEMP_START,
	IIC_NXT_TEMP_WRITE,
	IIC_NXT_TEMP_READ,
	IIC_MANUFACTURER_START,
	IIC_MANUFACTURER_WRITE,
	IIC_MANUFACTURER_READ,
	IIC_TYPE_START,
	IIC_TYPE_WRITE,
	IIC_TYPE_READ,
	IIC_SETUP_WAIT,
	IIC_SETUP_START,
	IIC_SETUP_WRITE,
	IIC_SETUP_READ,
	IIC_WAITING,
	IIC_WRITING,
	IIC_READING,
	IIC_REPEAT,
	IIC_ERROR,
	IIC_EXIT,
	IIC_STATES
};

static DATA8 IicPortType[INPUTS];
static IICSTR IicStrings[INPUTS];

#define IIC_TIMER_RESOLUTION 20 // [100u]
static struct hrtimer Device1Timer;
static ktime_t Device1Time;

static enum hrtimer_restart Device1TimerInterrupt1(struct hrtimer *pTimer)
{
	UBYTE Port;
	UBYTE Tmp;
	UBYTE TmpBuffer[IIC_DATA_LENGTH];

	hrtimer_forward_now(pTimer, Device1Time);

	for (Port = 0; Port < INPUTS; Port++) { // look at one port at a time

		switch (IicPort[Port].State) { // Main state machine

			case IIC_IDLE:
			{ // Port inactive

				pIic->Status[Port] = 0;
			}
			break;

			case IIC_INIT:
			{ // Initialize port hardware

				memset((void*)IicStrings[Port].Manufacturer, 0, IIC_NAME_LENGTH + 1);
				memset((void*)IicStrings[Port].SensorType, 0, IIC_NAME_LENGTH + 1);
				IicPort[Port].State = IIC_ENABLE;
			}
			break;

			case IIC_RESTART:
			{
				IicPortDisable(Port);
				IicPort[Port].State = IIC_ENABLE;
			}
			break;

			case IIC_ENABLE:
			{ // Initialize port variables

				IicPortEnable(Port);

				IicPort[Port] = IicPortDefault;

				IicPort[Port].Timer = 0;

				IicPort[Port].State = IIC_NXT_TEMP_START;
			}
			break;





			case IIC_NXT_TEMP_START:
			{
				if (++(IicPort[Port].Timer) >= (10 / IIC_TIMER_RESOLUTION)) {
					IicPort[Port].OutBuffer[0] = 0x4C;
					IicPort[Port].OutBuffer[1] = 0x01;
					IicPort[Port].OutBuffer[2] = 0x60;
					IicPort[Port].OutBuffer[3] = 0x00;
					IicPort[Port].OutLength = 3;
					IicPort[Port].InLength = 1;
					IicPort[Port].Repeat = 1;
					IicPort[Port].Time = 0;
					IicPort[Port].Timer = 0;
					IicPort[Port].State = IIC_NXT_TEMP_WRITE;
				}
			}
			break;

			case IIC_NXT_TEMP_WRITE:
			{
				IicPortSend(Port);
				IicPort[Port].Timer = 0;
				IicPort[Port].State = IIC_NXT_TEMP_READ;
			}
			break;

			case IIC_NXT_TEMP_READ:
			{
				if (IicPortReceive(Port, TmpBuffer) != BUSY) {
					if (TmpBuffer[0] == 0x60) {
						memcpy(IicStrings[Port].Manufacturer, "LEGO", IIC_NAME_LENGTH);
						IicStrings[Port].Manufacturer[IIC_NAME_LENGTH] = 0;
						memcpy(IicStrings[Port].SensorType, "Temp.", IIC_NAME_LENGTH);
						IicStrings[Port].SensorType[IIC_NAME_LENGTH] = 0;

						pIic->Changed[Port] = 1;

						IicPort[Port].Initialised = 1;
						IicPort[Port].Timer = 0;
						IicPort[Port].State = IIC_SETUP_WAIT;
					} else {
						IicPort[Port].Timer = 0;
						IicPort[Port].State = IIC_MANUFACTURER_START;
					}
				}
				if (++(IicPort[Port].Timer) >= (500 / IIC_TIMER_RESOLUTION)) {
					IicPort[Port].Timer = 0;
					IicPort[Port].State = IIC_MANUFACTURER_START;
				}
			}
			break;

			case IIC_MANUFACTURER_START:
			{
				if (++(IicPort[Port].Timer) >= (10 / IIC_TIMER_RESOLUTION)) {
					IicPort[Port].OutBuffer[0] = IicPort[Port].Addr;
					IicPort[Port].OutBuffer[1] = 0x08;
					IicPort[Port].OutBuffer[2] = 0x00;
					IicPort[Port].OutBuffer[3] = 0x00;
					IicPort[Port].OutLength = 2;
					IicPort[Port].InLength = 8;
					IicPort[Port].Repeat = 1;
					IicPort[Port].Time = 0;
					IicPort[Port].Timer = 0;
					IicPort[Port].State = IIC_MANUFACTURER_WRITE;
				}
			}
			break;

			case IIC_MANUFACTURER_WRITE:
			{
				IicPortSend(Port);
				IicPort[Port].Timer = 0;
				IicPort[Port].State = IIC_MANUFACTURER_READ;
			}
			break;

			case IIC_MANUFACTURER_READ:
			{
				if (IicPortReceive(Port, TmpBuffer) != BUSY) {
					memcpy(IicStrings[Port].Manufacturer, TmpBuffer, IIC_NAME_LENGTH);
					IicStrings[Port].Manufacturer[IIC_NAME_LENGTH] = 0;

					if (TmpBuffer[0] == 0x08) {
						if (IicPort[Port].Addr < 0x50) {
							IicPort[Port].Addr++;
							IicPort[Port].Timer = 0;
							IicPort[Port].State = IIC_MANUFACTURER_START;
						} else {
							if (--IicPort[Port].Retries > 0) {
								IicPort[Port].Addr = 0x01;
								IicPort[Port].Timer = 0;
								IicPort[Port].State = IIC_MANUFACTURER_START;
							} else {
								IicPort[Port].Initialised = 1;
								IicPort[Port].State = IIC_SETUP_START;
							}
						}
					} else {
						if (TmpBuffer[0] == 0) {
							memcpy(IicStrings[Port].Manufacturer, "LEGO", IIC_NAME_LENGTH);
							IicStrings[Port].Manufacturer[IIC_NAME_LENGTH] = 0;
						}
						IicPort[Port].Initialised = 1;
						IicPort[Port].State = IIC_TYPE_START;
					}

				}
				if (++(IicPort[Port].Timer) >= (500 / IIC_TIMER_RESOLUTION)) {
					IicPort[Port].Addr = 0x01;
					pIic->Status[Port] &= ~IIC_WRITE_REQUEST;
					pIic->Status[Port] &= ~IIC_DATA_READY;
					IicPort[Port].State = IIC_WAITING;
				}
			}
			break;

			case IIC_TYPE_START:
			{
				IicPort[Port].OutBuffer[0] = IicPort[Port].Addr;
				IicPort[Port].OutBuffer[1] = 0x10;
				IicPort[Port].OutBuffer[2] = 0x00;
				IicPort[Port].OutBuffer[3] = 0x00;
				IicPort[Port].OutLength = 2;
				IicPort[Port].InLength = 8;
				IicPort[Port].Repeat = 1;
				IicPort[Port].Time = 0;
				IicPort[Port].Timer = 0;
				IicPort[Port].State = IIC_TYPE_WRITE;
			}
			break;

			case IIC_TYPE_WRITE:
			{
				IicPortSend(Port);
				IicPort[Port].Timer = 0;
				IicPort[Port].State = IIC_TYPE_READ;
			}
			break;

			case IIC_TYPE_READ:
			{
				if (IicPortReceive(Port, TmpBuffer) != BUSY) {
					memcpy((void*)IicStrings[Port].SensorType, (void*)TmpBuffer, IIC_NAME_LENGTH);
					IicStrings[Port].SensorType[IIC_NAME_LENGTH] = 0;

					if (TmpBuffer[0] == 0) {
						memcpy((void*)IicStrings[Port].SensorType, (void*)"Store", IIC_NAME_LENGTH);
						IicStrings[Port].Manufacturer[IIC_NAME_LENGTH] = 0;
					}

					pIic->Changed[Port] = 1;

					IicPort[Port].Initialised = 1;
					IicPort[Port].Timer = 0;
					IicPort[Port].State = IIC_SETUP_WAIT;
				}
				if (++(IicPort[Port].Timer) >= (500 / IIC_TIMER_RESOLUTION)) {
					pIic->Status[Port] &= ~IIC_WRITE_REQUEST;
					pIic->Status[Port] &= ~IIC_DATA_READY;
					IicPort[Port].State = IIC_WAITING;
				}
			}
			break;

			case IIC_SETUP_WAIT:
			{
				if (++(IicPort[Port].Timer) >= (10000 / IIC_TIMER_RESOLUTION)) {
					IicPort[Port] = IicPortDefault;
					IicPort[Port].Timer = 0;
					IicPort[Port].State = IIC_NXT_TEMP_START;
				}
			}
			break;

			case IIC_SETUP_START:
			{
#ifndef DISABLE_FAST_DATALOG_BUFFER
				pIic->Actual[Port] = 0;
				pIic->LogIn[Port] = 0;
#endif

				if (IicStrings[Port].SetupLng) {
					IicPort[Port].OutBuffer[0] = (UBYTE)((IicStrings[Port].SetupString >> 24) & 0xFF);
					IicPort[Port].OutBuffer[1] = (UBYTE)((IicStrings[Port].SetupString >> 16) & 0xFF);
					IicPort[Port].OutBuffer[2] = (UBYTE)((IicStrings[Port].SetupString >> 8) & 0xFF);
					IicPort[Port].OutBuffer[3] = (UBYTE)((IicStrings[Port].SetupString) & 0xFF);
					IicPort[Port].OutLength = IicStrings[Port].SetupLng;
					IicPort[Port].InLength = 0;
					IicPort[Port].Repeat = 1;
					IicPort[Port].Time = 0;
					IicPort[Port].State = IIC_SETUP_WRITE;
				} else {
					IicPort[Port].State = IIC_SETUP_READ;
				}
				IicPort[Port].Timer = 0;
			}
			break;

			case IIC_SETUP_WRITE:
			{
				IicPortSend(Port);
				IicPort[Port].Timer = 0;
				IicPort[Port].State = IIC_SETUP_READ;
			}
			break;

			case IIC_SETUP_READ:
			{
				if (IicPortReceive(Port, TmpBuffer) != BUSY) {
					IicPort[Port].State = IIC_WAITING;
					if (IicStrings[Port].PollLng) {
						IicPort[Port].OutBuffer[0] = (UBYTE)((IicStrings[Port].PollString >> 24) & 0xFF);
						IicPort[Port].OutBuffer[1] = (UBYTE)((IicStrings[Port].PollString >> 16) & 0xFF);
						IicPort[Port].OutBuffer[2] = (UBYTE)((IicStrings[Port].PollString >> 8) & 0xFF);
						IicPort[Port].OutBuffer[3] = (UBYTE)((IicStrings[Port].PollString) & 0xFF);
						IicPort[Port].OutLength = IicStrings[Port].PollLng;
						if (IicStrings[Port].ReadLng < 0) {
							IicPort[Port].InLength = 0 - IicStrings[Port].ReadLng;
							IicPort[Port].Reverse = 1;
						} else {
							IicPort[Port].InLength = IicStrings[Port].ReadLng;
							IicPort[Port].Reverse = 0;
						}
						IicPort[Port].Repeat = 0;
						IicPort[Port].Time = IicStrings[Port].Time;
						IicPort[Port].Timer = 0;
						IicPort[Port].State = IIC_WRITING;
						pIic->Status[Port] = 0;
					}
					IicPort[Port].Initialised = 1;
				}
				if (++(IicPort[Port].Timer) >= (500 / IIC_TIMER_RESOLUTION)) {
					pIic->Status[Port] &= ~IIC_WRITE_REQUEST;
					pIic->Status[Port] &= ~IIC_DATA_READY;
					IicPort[Port].State = IIC_WAITING;
				}
			}
			break;

			case IIC_WAITING:
			{
				if (pIic->Status[Port] & IIC_WRITE_REQUEST) {
					if (IicPortBusy(Port) == 0) {
						IicPort[Port].Timer = 0;
						IicPort[Port].State = IIC_WRITING;
					}
				}
			}
			break;

			case IIC_WRITING:
			{
				IicPortSend(Port);
				IicPort[Port].State = IIC_READING;
			}
			break;

			case IIC_READING:
			{
				if (IicPortReceive(Port, TmpBuffer) != BUSY) {

#ifndef DISABLE_FAST_DATALOG_BUFFER
					if (IicPort[Port].InLength > 1) {
						if (IicPort[Port].Reverse) {
							for (Tmp = 0; Tmp < IicPort[Port].InLength; Tmp++) {
								IicPort[Port].InBuffer[Tmp] = TmpBuffer[Tmp];
								pIic->Raw[Port][pIic->LogIn[Port]][Tmp] = TmpBuffer[Tmp];
							}
						} else {
							for (Tmp = 0; Tmp < IicPort[Port].InLength; Tmp++) {
								IicPort[Port].InBuffer[Tmp] = TmpBuffer[(IicPort[Port].InLength - 1) - Tmp];
								pIic->Raw[Port][pIic->LogIn[Port]][Tmp] = TmpBuffer[(IicPort[Port].InLength - 1) - Tmp];
							}
						}
					} else {
						IicPort[Port].InBuffer[0] = TmpBuffer[0];
						IicPort[Port].InBuffer[1] = 0;
						pIic->Raw[Port][pIic->LogIn[Port]][0] = TmpBuffer[0];
						pIic->Raw[Port][pIic->LogIn[Port]][1] = 0;
					}

					pIic->Actual[Port] = pIic->LogIn[Port];
					pIic->Repeat[Port][pIic->Actual[Port]] = 0;

					if (++(pIic->LogIn[Port]) >= DEVICE_LOGBUF_SIZE) {
						pIic->LogIn[Port] = 0;
					}
#else
					if (IicPort[Port].InLength > 1) {
						if (IicPort[Port].Reverse) {
							for (Tmp = 0; Tmp < IicPort[Port].InLength; Tmp++) {
								IicPort[Port].InBuffer[Tmp] = TmpBuffer[Tmp];
								pIic->Raw[Port][Tmp] = TmpBuffer[Tmp];
							}
						} else {
							for (Tmp = 0; Tmp < IicPort[Port].InLength; Tmp++) {
								IicPort[Port].InBuffer[Tmp] = TmpBuffer[(IicPort[Port].InLength - 1) - Tmp];
								pIic->Raw[Port][Tmp] = TmpBuffer[(IicPort[Port].InLength - 1) - Tmp];
							}
						}
					} else {
						IicPort[Port].InBuffer[0] = TmpBuffer[0];
						IicPort[Port].InBuffer[1] = 0;
						pIic->Raw[Port][0] = TmpBuffer[0];
						pIic->Raw[Port][1] = 0;
					}
#endif

					pIic->Status[Port] |= IIC_DATA_READY;
					IicPort[Port].State = IIC_REPEAT;

					if (IicPort[Port].Repeat != 0) {
						IicPort[Port].Repeat--;
						if (IicPort[Port].Repeat == 0) {
							IicPort[Port].State = IIC_WAITING;
						}
					}
				}
				if (++(IicPort[Port].Timer) >= (5000 / IIC_TIMER_RESOLUTION)) {
					pIic->Status[Port] &= ~IIC_WRITE_REQUEST;
					pIic->Status[Port] &= ~IIC_DATA_READY;
					IicPort[Port].State = IIC_WAITING;
				}
			}
			break;

			case IIC_REPEAT:
			{
				if (pIic->Status[Port] & IIC_WRITE_REQUEST) {
					pIic->Status[Port] = IIC_WRITE_REQUEST;
					IicPort[Port].State = IIC_WAITING;
				} else {
					if (++(IicPort[Port].Timer) >= (UWORD)(IicPort[Port].Time / (IIC_TIMER_RESOLUTION / 10))) {
						IicPort[Port].Timer = 0;
						IicPort[Port].State = IIC_WRITING;
					}
				}
			}
			break;

			case IIC_EXIT:
			{
				IicPortDisable(Port);
				IicPort[Port] = IicPortDefault;

				pIic->Status[Port] = 0;

				IicPort[Port].State = IIC_IDLE;
			}
			break;
		}

#ifndef DISABLE_FAST_DATALOG_BUFFER
		(pIic->Repeat[Port][pIic->Actual[Port]]) += (IIC_TIMER_RESOLUTION / 10);
#endif
	}

	return HRTIMER_RESTART;
}

static long Device1Ioctl(struct file *File, unsigned int Request, unsigned long Pointer)
{
	DEVCON DevCon;
	IICSTR IicStr;
	IICDAT IicDat;
	DATA8 Port = 0;
	int ret;

	switch (Request) {
		case IIC_SET_CONN:
		{
			ret = copy_from_user((void*)&DevCon, (void*)Pointer, sizeof(DevCon));
			if (ret < 0)
				return ret;

			for (Port = 0; Port < INPUTS; Port++) {
				if (DevCon.Connection[Port] == CONN_NXT_IIC) {
					if (IicConfigured[Port] == 0) {
						IicConfigured[Port] = 1;
						IicPortType[Port] = DevCon.Type[Port];
						IicPort[Port].State = IIC_INIT;
					} else {
						if (IicPort[Port].Initialised) {
							if (IicPort[Port].Mode != DevCon.Mode[Port]) {
								IicPort[Port].Mode = DevCon.Mode[Port];
								IicPort[Port].ChangeMode = 1;
								pIic->Status[Port] &= ~IIC_DATA_READY;
								IicPort[Port].State = IIC_SETUP_START;
							}
						}
					}
				} else {
					pIic->Status[Port] &= ~IIC_DATA_READY;
					if (IicConfigured[Port]) {
						IicConfigured[Port] = 0;
						IicPort[Port].State = IIC_EXIT;
					}
				}
			}

		}
		break;

		case IIC_SET:
		{
			ret = copy_from_user((void*)&IicStr, (void*)Pointer, sizeof(IicStr));
			if (ret < 0)
				return ret;

			Port = IicStr.Port;

			memcpy(&IicStrings[Port], &IicStr, sizeof(IICSTR));
			IicPort[Port].State = IIC_SETUP_START;

		}
		break;

		case IIC_SETUP:
		{
			ret = copy_from_user((void*)&IicDat, (void*)Pointer, sizeof(IicDat));
			if (ret < 0)
				return ret;

			Port = IicDat.Port;
			IicDat.Result = BUSY;

			if (!(pIic->Status[Port] & IIC_WRITE_REQUEST)) {
				IicPort[Port].Repeat = IicDat.Repeat;
				IicPort[Port].Time = IicDat.Time;
				IicPort[Port].OutLength = IicDat.WrLng;
				if (IicDat.RdLng < 0) {
					IicPort[Port].InLength = 0 - IicDat.RdLng;
					IicPort[Port].Reverse = 1;
				} else {
					IicPort[Port].InLength = IicDat.RdLng;
					IicPort[Port].Reverse = 0;
				}

				if (IicPort[Port].OutLength > IIC_DATA_LENGTH) {
					IicPort[Port].OutLength = IIC_DATA_LENGTH;
				}
				if (IicPort[Port].InLength > IIC_DATA_LENGTH) {
					IicPort[Port].InLength = IIC_DATA_LENGTH;
				}

				memcpy(IicPort[Port].OutBuffer, IicDat.WrData,
				       IicPort[Port].OutLength);
				memset(IicPort[Port].InBuffer, 0, IIC_DATA_LENGTH);

				pIic->Status[Port] = IIC_WRITE_REQUEST;
			}

			if ((pIic->Status[Port] & IIC_DATA_READY)) {
				memcpy(IicDat.RdData, IicPort[Port].InBuffer,
				       IicPort[Port].InLength);

				IicDat.Result = OK;
				pIic->Status[Port] = 0;
			}

			ret = copy_to_user((void *)Pointer, (void *)&IicDat, sizeof(IicDat));
			if (ret < 0)
				return ret;
		}
		break;

		case IIC_READ_TYPE_INFO:
		{
			ret = copy_from_user((void*)&IicStr, (void*)Pointer, sizeof(IicStr));
			if (ret < 0)
				return ret;

			Port = IicStr.Port;
			memcpy(IicStr.Manufacturer, IicStrings[Port].Manufacturer, IIC_NAME_LENGTH + 1);
			memcpy(IicStr.SensorType, IicStrings[Port].SensorType, IIC_NAME_LENGTH + 1);

			pIic->Changed[Port] = 0;

			ret = copy_to_user((void *)Pointer, (void *)&IicStr, sizeof(IicStr));
			if (ret < 0)
				return ret;

		}
		break;
	}

	return 0;
}

#define SHM_LENGTH (sizeof(IicDefault))
#define NPAGES ((SHM_LENGTH + PAGE_SIZE - 1) / PAGE_SIZE)
static void *kmalloc_ptr;

static int Device1Mmap(struct file *filp, struct vm_area_struct *vma)
{
	int ret;

	ret = remap_pfn_range(vma, vma->vm_start, virt_to_phys((void*)((unsigned long)pIic)) >> PAGE_SHIFT, vma->vm_end-vma->vm_start, PAGE_SHARED);
	if (ret < 0)
		ret = -EAGAIN;

	return ret;
}

static const struct file_operations Device1Entries = {
	.owner		= THIS_MODULE,
	.mmap		= Device1Mmap,
	.unlocked_ioctl	= Device1Ioctl,
};

static struct miscdevice Device1 = {
	.minor	= MISC_DYNAMIC_MINOR,
	.name	= DEVICE1_NAME,
	.fops	= &Device1Entries,
};

static int Device1Init(struct device * parent)
{
	IIC *pTmp;
	UBYTE Port;
	int ret;
	int i;

	Device1Lms2012CompatDev = lms2012_compat_get();
	if (!Device1Lms2012CompatDev)
		return -EPROBE_DEFER;

	Device1Lms2012Compat = dev_get_drvdata(Device1Lms2012CompatDev);

	kmalloc_ptr = kmalloc((NPAGES + 2) * PAGE_SIZE, GFP_KERNEL);
	if (!kmalloc_ptr) {
		ret = -ENOMEM;
		goto err0;
	}

	pTmp = (IIC*)((((unsigned long)kmalloc_ptr) + PAGE_SIZE - 1) & PAGE_MASK);
	for (i = 0; i < NPAGES * PAGE_SIZE; i += PAGE_SIZE)
		SetPageReserved(virt_to_page(((unsigned long)pTmp) + i));
	pIic = pTmp;
	memset(pIic, 0, sizeof(IIC));

	for (Port = 0; Port < INPUTS; Port++) {
		IicPort[Port] = IicPortDefault;
		IicCtrl[Port].port = Port;
		INIT_WORK(&IicCtrl[Port].message_work, Device1MessageWork);
		init_completion(&IicCtrl[Port].message_completion);
		IicConfigured[Port] = 0;
		IicStrings[Port].Manufacturer[0] = 0;
		IicStrings[Port].SensorType[0] = 0;
	}

	Device1.parent = parent;
	ret = misc_register(&Device1);
	if (ret)
		goto err1;

	Device1Time = ktime_set(0, IIC_TIMER_RESOLUTION * 100000);
	hrtimer_init(&Device1Timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	Device1Timer.function = Device1TimerInterrupt1;
	hrtimer_start(&Device1Timer, Device1Time, HRTIMER_MODE_REL);

	return 0;

err1:
	for (i = 0; i < NPAGES * PAGE_SIZE; i+= PAGE_SIZE)
		ClearPageReserved(virt_to_page(((unsigned long)pTmp) + i));
	kfree(kmalloc_ptr);
err0:
	put_device(Device1Lms2012CompatDev);

	return ret;
}

static void Device1Exit(void)
{
	IIC *pTmp;
	int i;

	hrtimer_cancel(&Device1Timer);

	pTmp = pIic;
	pIic = &IicDefault;

	// free shared memory
	for (i = 0; i < NPAGES * PAGE_SIZE; i+= PAGE_SIZE)
		ClearPageReserved(virt_to_page(((unsigned long)pTmp) + i));
	kfree(kmalloc_ptr);

	misc_deregister(&Device1);
	put_device(Device1Lms2012CompatDev);
}

// MODULE *********************************************************************

static int d_iic_probe(struct platform_device *pdev)
{
	int ret;

	ret = Device1Init(&pdev->dev);
	if (ret < 0)
		return ret;

	pr_info("d_iic registered\n");

	return 0;
}

static int d_iic_remove(struct platform_device *pdev)
{
	Device1Exit();

	pr_info("d_iic removed\n");

	return 0;
}

static struct platform_driver d_iic_driver = {
	.driver	= {
		.name	= "d_iic",
	},
	.probe	= d_iic_probe,
	.remove	= d_iic_remove,
};
module_platform_driver(d_iic_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("The LEGO Group");
MODULE_DESCRIPTION(MODULE_NAME);
MODULE_ALIAS("platform:d_iic");
