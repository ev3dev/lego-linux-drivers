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
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/hrtimer.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/miscdevice.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/time.h>
#include <linux/uaccess.h>

#include "lms2012.h"

/* Input port stuff */

#define IN_CONNECT_STEADY_TIME		350   //  [mS]  time needed to be sure that the connection is steady
#define IN_DISCONNECT_STEADY_TIME	100   //  [mS]  time needed to be sure that the disconnection is steady

#define IN1_NEAR_5V			4800  //  [mV]  higher values mean that connection 1 is floating
#define IN1_NEAR_PIN2			3100  //  [mV]  higher values mean that connection 1 is shorted to connection 2 (5000 * 18K / (18K + 10k))
#define IN1_TOUCH_HIGH			950   //  [mV]  values in between these limits means that an old touch sensor is connected
#define IN1_TOUCH_LOW			850   //  [mV]
#define IN1_NEAR_GND			100   //  [mV]  lower  values mean that connection 1 is shorted to connection 3
#define IN6_NEAR_GND			150   //  [mV]  lower  values mean that connection 6 is floating

/* output port stuff */

#define OUT5_IIC_HIGH			3700  //  [mV]  values in between these limits means that an old IIC sensor or color sensor is connected
#define OUT5_IIC_LOW			2800  //  [mV]

#define OUT5_BALANCE_HIGH		2600  //  [mV]  values in between these limits means that connection 5 is floating
#define OUT5_BALANCE_LOW		2400  //  [mV]

#define OUT5_LIGHT_HIGH			850   //  [mV]  values in between these limits means that an old light sensor is connected
#define OUT5_LIGHT_LOW			650   //  [mV]

#define OUT5_NEAR_GND			100   //  [mV]  lower  values mean that connection 5 is shorted to ground

#define OUT5_DUMP_HIGH			2350
#define OUT5_DUMP_LOW			1150

#define OUT5_MINITACHO_HIGH1		2000  //  [mV]  values in between these limits means that a mini tacho motor is pulling high when pin5 is pulling low
#define OUT5_MINITACHO_LOW1		1600  //  [mV]

#define OUT5_NEWTACHO_HIGH1		1600  //  [mV]  values in between these limits means that a new tacho motor is pulling high when pin5 is pulling low
#define OUT5_NEWTACHO_LOW1		1200  //  [mV]

#define OUT5_INTELLIGENT_HIGH1		1150
#define OUT5_INTELLIGENT_LOW1		850

#define OUT5_INTELLIGENT_HIGH2		1150
#define OUT5_INTELLIGENT_LOW2		850

#define OUT5_NEWTACHO_HIGH2		650  //  [mV]  values in between these limits means that a new tacho motor is pulling low when pin5 floats
#define OUT5_NEWTACHO_LOW2		450  //  [mV]

#define OUT5_MINITACHO_HIGH2		450  //  [mV]  values in between these limits means that a mini tacho motor is pulling low when pin5 floats
#define OUT5_MINITACHO_LOW2		250  //  [mV]

#define INPUTADCPORTS		12
#define INPUTADCPOWERS		4
#define INPUTADC		(INPUTADCPORTS + INPUTADCPOWERS)

#define NO_OF_INPUT_PORTS	INPUTS
#define NO_OF_OUTPUT_PORTS	OUTPUTS

#define MODULE_NAME		"d_analog"
#define DEVICE1_NAME		ANALOG_DEVICE
#define DEVICE3_NAME		DCM_DEVICE

// DEVICE1 ********************************************************************

static struct device *Device1Lms2012CompatDev;
static struct lms2012_compat *Device1Lms2012Compat;
static struct hrtimer Device1Timer;
static ktime_t Device1Time;

static u8 TestMode = 0;

static ANALOG AnalogDefault;

static ANALOG *pAnalog = &AnalogDefault;
static u16 *pInputs = (u16*)&AnalogDefault;

static struct dentry *device1_debug;

typedef struct {
	u16  Value;
	u8   Connected;
	u8   Cmd;
	u8   State;
	u8   OldState;
	u8   Event;
	u8   Timer;
	u8   FSMEnabled;
} INPORT;

static INPORT InputPort[NO_OF_INPUT_PORTS];

typedef struct {
	u16  Value5Float;
	u16  Value5Low;
	u8   Connected;
	u8   Code;
	u8   Type;
	u8   State;
	u8   OldState;
	u8   Event;
	u8   Timer;
} OUTPORT;

static OUTPORT OutputPort[NO_OF_OUTPUT_PORTS];

#define SCHEMESIZE1    10

static const u8 MuxSetup1[SCHEMESIZE1] = {
	0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x20, 0x20
};
static const u8 Reading1[SCHEMESIZE1] = {
	0x80, 0x20, 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07
};
static const u8 NextTime1[SCHEMESIZE1] = {
	0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x02, 0x00, 0x01
};
static ktime_t Time1[2];

#define SCHEMESIZE2    15

static const u8 MuxSetup2[SCHEMESIZE2] = {
	0x13, 0x00, 0x01, 0x02, 0x10, 0x03, 0x04, 0x05,
	0x11, 0x06, 0x07, 0x20, 0x12, 0x20, 0x20
};
static const u8 Reading2[SCHEMESIZE2] = {
	0x80, 0x20, 0x13, 0x00, 0x01, 0x02, 0x10, 0x03,
	0x04, 0x05, 0x11, 0x06, 0x07, 0x80, 0x12
};
static const u8 ClockHigh2[SCHEMESIZE2] = {
	0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00
};
static const u8 ClockLow2[SCHEMESIZE2] = {
	0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00
};
static const u8 NextTime2[SCHEMESIZE2] = {
	0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x01,
	0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x01
};

static ktime_t Time2[2];
static ktime_t NextTime;

static u8 InputPoint1 = 8;

static u8 Color = 0;
static u8 NxtPointer = 0;
static u8 NxtColorActive[INPUTS];
static u8 Nxtcolor[INPUTS];
static u8 NxtcolorCmd[INPUTS];
static u8 NxtcolorLatchedCmd[INPUTS];
static u8 InputPoint2 = 0;

#define PINRead(p, i)	gpiod_get_value(Device1Lms2012Compat->in_pins[p]->desc[i])
#define PIN1Read(p)	gpiod_get_value(Device1Lms2012Compat->in_pin1[p])
#define PIN2Read(p)	(!Device1Lms2012Compat->in_pin2[p] || gpiod_get_value(Device1Lms2012Compat->in_pin2[p]))
#define PINLow(p, i)	gpiod_direction_output(Device1Lms2012Compat->in_pins[p]->desc[i], 0)
#define PIN1Low(p)	gpiod_direction_output(Device1Lms2012Compat->in_pin1[p], 0)
#define PINHigh(p, i)	gpiod_direction_output(Device1Lms2012Compat->in_pins[p]->desc[i], 1)
#define PIN1High(p)	gpiod_direction_output(Device1Lms2012Compat->in_pin1[p], 1)
#define PINFloat(p, i)	gpiod_direction_input(Device1Lms2012Compat->in_pins[p]->desc[i])
#define PIN2Float(p)	{ if (Device1Lms2012Compat->in_pin2[p]) gpiod_direction_input(Device1Lms2012Compat->in_pin2[p]); }

#define POUTRead(p, i)	gpiod_get_value(Device1Lms2012Compat->out_pins[p]->desc[i])
#define POUTLow(p, i)	gpiod_direction_output(Device1Lms2012Compat->out_pins[p]->desc[i], 0)
#define POUTHigh(p, i)	gpiod_direction_output(Device1Lms2012Compat->out_pins[p]->desc[i], 1)
#define POUTFloat(p, i)	gpiod_direction_input(Device1Lms2012Compat->out_pins[p]->desc[i])

#define SPIRead(i)	gpiod_get_value(Device1Lms2012Compat->spi_pins->desc[i])
#define SPILow(i)	gpiod_direction_output(Device1Lms2012Compat->spi_pins->desc[i], 0)
#define SPIHigh(i)	gpiod_direction_output(Device1Lms2012Compat->spi_pins->desc[i], 1)
#define SPIFloat(i)	gpiod_direction_input(Device1Lms2012Compat->spi_pins->desc[i])

static void SpiReset(void)
{
	SPIHigh(ADCCS);
	SPILow(ADCCLK);
	SPILow(ADCMOSI);
	SPIFloat(ADCMISO);
}

u16 SpiUpdate(u16 DataOut)
{
	u16 DataIn = 0;
	u8 Bit = 16;

	SPILow(ADCCS);

	while (Bit--) {
		if (DataOut & 0x8000)
			SPIHigh(ADCMOSI);
		else
			SPILow(ADCMOSI);
		SPIHigh(ADCCLK);
		DataOut <<= 1;
		DataIn  <<= 1;
		if (SPIRead(ADCMISO))
			DataIn |= 1;
		SPILow(ADCCLK);
	}

	SPIHigh(ADCCS);

	return DataIn;
}

static enum hrtimer_restart Device1TimerInterrupt1(struct hrtimer *pTimer)
{
	u8   Port;
	u16 *pData;
	u16  Data;
	u16  Input;

	// restart timer
	hrtimer_forward_now(pTimer, NextTime);

	if (NxtPointer == 0) {
		if (++InputPoint2 >= INPUTS)
			InputPoint2 = 0;
	}

	if (!Color) { // No NXT color sensor attached -> use scheme 1
		do {
			pData = &Data;
			if (MuxSetup1[NxtPointer] & 0x20) {
				Input = (u16)Device1Lms2012Compat->adc_map[InputPoint1];
			} else {
				Input =  (u16)Device1Lms2012Compat->adc_map[MuxSetup1[NxtPointer] & 0x0F];
			}

			if (Reading1[NxtPointer] & 0x20) {
				pData = &pInputs[InputPoint1];
				if (++InputPoint1 >= INPUTADC) {
					InputPoint1 = 8;
				}
			} else {
				if (!(Reading1[NxtPointer] & 0xF0)) {
					pData =  &pInputs[Reading1[NxtPointer]];
				}
			}

			*pData = (u16)SpiUpdate((0x1840 | ((Input & 0x000F) << 7)));
			*pData &=  0x0FFF;

			NxtPointer++;
		}
		while ((NextTime1[NxtPointer - 1] == 0));

		NextTime = Time1[NextTime1[NxtPointer - 1] - 1];

		if (NxtPointer >= SCHEMESIZE1) {
			NxtPointer = 0;
		}
	} else { // One or more NXT color sensors attached -> use scheme 2
		do {
			pData = &Data;
			if (MuxSetup2[NxtPointer] & 0x20) {
				Input = (u16)Device1Lms2012Compat->adc_map[InputPoint1];
			} else {
				if (MuxSetup2[NxtPointer] & 0x10) {
					Input = (u16)Device1Lms2012Compat->adc_map[InputPoint2 + INPUTS];
				} else {
					Input =  (u16)Device1Lms2012Compat->adc_map[MuxSetup2[NxtPointer] & 0x0F];
				}
			}

			if (Reading2[NxtPointer] & 0x20) {
				pData = &pInputs[InputPoint1];
				if (++InputPoint1 >= INPUTADC) {
					InputPoint1 = 8;
				}
			} else {
				if (Reading2[NxtPointer] & 0x10) {
					pData = (u16*)&pAnalog->NxtCol[InputPoint2].ADRaw[(Reading2[NxtPointer] & 0x03)];
				} else {
					if (!(Reading2[NxtPointer] & 0xF0)) {
						pData =  &pInputs[Reading2[NxtPointer]];
					}
				}
			}

			*pData = (u16)SpiUpdate((0x1840 | ((Input & 0x000F) << 7))) & 0xFFF;
			if (NxtcolorLatchedCmd[InputPoint2] == 0x0D) {
				if (ClockHigh2[NxtPointer]) {
					if (Nxtcolor[InputPoint2]) {
						if (ClockHigh2[NxtPointer] == 0x01) {
							PINHigh(InputPoint2, INPUT_PORT_PIN5);
						} else {
							PINFloat(InputPoint2, INPUT_PORT_PIN5);
						}
					}
				}
				if (ClockLow2[NxtPointer]) {
					if (Nxtcolor[InputPoint2]) {
						PINLow(InputPoint2, INPUT_PORT_PIN5);
					}
				}
			}

			NxtPointer++;
		}
		while ((NextTime2[NxtPointer - 1] == 0));

		NextTime = Time2[NextTime2[NxtPointer - 1] - 1];

		if (NxtPointer >= SCHEMESIZE2)
			NxtPointer = 0;
	}

	if (NxtPointer == 0) {
		Color = 0;
		for (Port = 0;Port < INPUTS;Port++) {
#ifndef DISABLE_FAST_DATALOG_BUFFER
			if (NxtColorActive[Port]) {
				Color = 1;
			} else { // Buffer for fast data logging

				pAnalog->Pin1[Port][pAnalog->LogIn[Port]] = pAnalog->InPin1[Port];
				pAnalog->Pin6[Port][pAnalog->LogIn[Port]] = pAnalog->InPin6[Port];

				pAnalog->Actual[Port] = pAnalog->LogIn[Port];

				if (++(pAnalog->LogIn[Port]) >= DEVICE_LOGBUF_SIZE) {
					pAnalog->LogIn[Port] = 0;
				}
				if (pAnalog->LogIn[Port] == pAnalog->LogOut[Port]) {
					if (++(pAnalog->LogOut[Port]) >= DEVICE_LOGBUF_SIZE) {
						pAnalog->LogOut[Port] = 0;
					}
				}
			}
			Nxtcolor[Port] = NxtColorActive[Port];
#else
			if (NxtColorActive[Port]) {
				Color = 1;
			}
			Nxtcolor[Port] = NxtColorActive[Port];
#endif

			pAnalog->Updated[Port] = 1;
		}
		NxtcolorLatchedCmd[InputPoint2] = NxtcolorCmd[InputPoint2];
	}

	return HRTIMER_RESTART;
}

enum DCM_STATE {
	DCM_INIT,
	DCM_FLOATING_DELAY,
	DCM_FLOATING,
	DCM_WAITING_FOR_PIN5_LOW,
	DCM_WAITING_FOR_PIN6_LOW,
	DCM_CONNECTION,
	DCM_PIN2_LOW,
	DCM_NXT_TOUCH_CHECK,
#ifndef DISABLE_OLD_COLOR
	DCM_NXT_COLOR_INIT,
	DCM_NXT_COLOR_WAIT,
	DCM_NXT_COLOR_START,
	DCM_NXT_COLOR_BUSY,
#endif
	DCM_CONNECTED_WAITING_FOR_PIN2_HIGH,
	DCM_PIN1_LOADED,
	DCM_CONNECTED_WAITING_FOR_PIN1_TO_FLOAT,
	DCM_PIN6_HIGH,
	DCM_CONNECTED_WAITING_FOR_PIN6_LOW,
	DCM_PIN5_LOW,
	DCM_CONNECTED_WAITING_FOR_PIN5_HIGH,
	DCM_CONNECTED_WAITING_FOR_PORT_OPEN,
	DCM_DISABLED,
	DCM_STATES
};

static const char * const DcmStateText[DCM_STATES] = {
	"DCM_INIT",
	"DCM_FLOATING_DELAY",
	"DCM_FLOATING",
	"DCM_WAITING_FOR_PIN5_LOW",
	"DCM_WAITING_FOR_PIN6_LOW",
	"DCM_CONNECTION",
	"DCM_PIN2_LOW",
	"DCM_NXT_TOUCH_CHECK",
#ifndef DISABLE_OLD_COLOR
	"DCM_NXT_COLOR_INIT",
	"DCM_NXT_COLOR_WAIT",
	"DCM_NXT_COLOR_START",
	"DCM_NXT_COLOR_BUSY",
#endif
	"DCM_CONNECTED_WAITING_FOR_PIN2_HIGH",
	"DCM_PIN1_LOADED",
	"DCM_CONNECTED_WAITING_FOR_PIN1_TO_FLOAT",
	"DCM_PIN6_HIGH",
	"DCM_PIN5_LOW",
	"DCM_CONNECTED_WAITING_FOR_PIN5_HIGH",
	"DCM_CONNECTED_WAITING_FOR_PIN6_LOW",
	"DCM_CONNECTED_WAITING_FOR_PORT_OPEN",
	"DCM_DISABLED"
};

static const INPORT InputPortDefault = {
	.Value		= 0,
	.Connected	= 0,
	.Cmd		= 0,
	.State		= DCM_INIT,
	.OldState	= -1,
	.Event		= 0,
	.Timer		= 0,
	.FSMEnabled	= 1
};

static const OUTPORT OutputPortDefault = {
	.Value5Float	= 0,
	.Value5Low	= 0,
	.Connected	= 0,
	.Code		= 0,
	.Type		= 0,
	.State		= DCM_INIT,
	.OldState	= -1,
	.Event		= 0,
	.Timer		= 0,
};

static void InputPortFloat(int Port)
{
	PIN1Low(Port);
	PIN2Float(Port);
	PINFloat(Port, INPUT_PORT_PIN5);
	PINFloat(Port, INPUT_PORT_PIN6);
	PINHigh(Port, INPUT_PORT_BUF);
}

static void OutputPortFloat(int Port)
{
	POUTLow(Port, OUTPUT_PORT_PIN1);
	POUTLow(Port, OUTPUT_PORT_PIN2);
	POUTLow(Port, OUTPUT_PORT_PIN5W);
	POUTFloat(Port, OUTPUT_PORT_PIN6);
}

u16 Device1GetInputPins(u8 Port)
{
	u16 Pins = 0x0000;
	u16 Mask = 0x0001;
	u8 Tmp;

	for (Tmp = 0;Tmp < INPUT_PORT_PINS;Tmp++) {
		if (PINRead(Port, Tmp)) {
			Pins |= Mask;
		}
		Mask <<= 1;
	}

	return Pins;
}

u16 Device1GetOutputPins(u8 Port)
{
	u16 Pins = 0x0000;
	u16 Mask = 0x0001;
	u8 Tmp;

	for (Tmp = 0;Tmp < OUTPUT_PORT_PINS;Tmp++) {
		if (POUTRead(Port, Tmp)) {
			Pins |= Mask;
		}
		Mask <<= 1;
	}

	return Pins;
}

static ssize_t Device1Write(struct file *File, const char *Buffer, size_t Count, loff_t *Data)
{
	char Buf[INPUTS + 2];
	u8   Port;
	u8   Char;
	int  ret;

	if (Count < INPUTS)
		return -EINVAL;

	ret = copy_from_user(Buf, Buffer, sizeof(Buf));
	if (ret < 0)
		return ret;

	// first character determines command type
	// e == enable auto-id, followed by either 1 or 0 to enable or disable
	// '-' indicates no change
	// Indicator on per port basis, so e--0-, means no change for ports 1, 2, 4 and disable for 3
	//
	// t == set the connection type
	// This should be preceded by disabling the auto-id.  Setting the type without disabling the
	// auto-id will do nothing.

	// Enabling/disabling the auto-id FSM for the port
	if (Buf[0] == 'e') {
		for (Port = 0;Port < NO_OF_INPUT_PORTS;Port++) {
			Char = Buf[Port + 1];

			switch (Char) {
			case '-' :
				// do nothing
				break;

			case '0':
				InputPort[Port].FSMEnabled = 0;
				break;

			case '1':
				InputPort[Port].FSMEnabled = 1;
				break;
			}
		}
	}
	// Set the type
	else if (Buf[0] == 't') {
		for (Port = 0;Port < NO_OF_INPUT_PORTS;Port++) {
			Char = Buf[Port + 1];

			// Don't bother if the port is not connected
			if (InputPort[Port].Connected == 0)
				continue;

			// Don't bother if the FSM is not disabled for this port
			if (InputPort[Port].FSMEnabled == 1)
				continue;

			switch (Char) {
			case '-' :
				// do nothing
				break;

			case CONN_NXT_IIC:
				pAnalog->InDcm[Port]	= TYPE_NXT_IIC;
				pAnalog->InConn[Port]	= CONN_NXT_IIC;
				break;

			// Pretend it's an old NXT light sensor
			// You can still read the raw value from pin 1
			case CONN_NXT_DUMB:
				pAnalog->InDcm[Port]	= TYPE_NXT_LIGHT;
				pAnalog->InConn[Port]	= CONN_NXT_DUMB;
				break;

			// Pretend it's an EV3 touch sensor
			case CONN_INPUT_DUMB:
				pAnalog->InDcm[Port]	= TYPE_TOUCH;
				pAnalog->InConn[Port]	= CONN_INPUT_DUMB;
				break;

			case CONN_NONE:
				pAnalog->InDcm[Port]	= TYPE_NONE;
				pAnalog->InConn[Port]	= CONN_NONE;
				break;
			}
		}
	}

	return Count;
}

#define SHM_LENGTH (sizeof(AnalogDefault))
#define NPAGES     ((SHM_LENGTH + PAGE_SIZE - 1) / PAGE_SIZE)

static void *kmalloc_ptr;

static int Device1Mmap(struct file *filp, struct vm_area_struct *vma)
{
	int ret;

	ret = remap_pfn_range(vma, vma->vm_start,
		virt_to_phys((void*)((unsigned long)pAnalog)) >> PAGE_SHIFT,
		vma->vm_end-vma->vm_start, PAGE_SHARED);

	if (ret != 0)
		ret = -EAGAIN;

	return ret;
}

static const struct file_operations Device1Entries = {
	.owner	= THIS_MODULE,
	.write	= Device1Write,
	.mmap	= Device1Mmap,
};

static struct miscdevice Device1 = {
	.minor	= MISC_DYNAMIC_MINOR,
	.name	= DEVICE1_NAME,
	.fops	= &Device1Entries,
};

#ifndef DISABLE_OLD_COLOR
static void NxtColorCommInit(void);
#endif

static int device1_debug_show(struct seq_file *m, void *data)
{
	seq_printf(m, "Port\t1\t2\t3\t4\n");
	seq_printf(m, "Pin 1\t%d\t%d\t%d\t%d\n",
		   PIN1Read(0), PIN1Read(1), PIN1Read(2), PIN1Read(3));
	seq_printf(m, "\t%04u\t%04u\t%04u\t%04u\n",
		   CtoV(pInputs[0]), CtoV(pInputs[1]),
		   CtoV(pInputs[2]), CtoV(pInputs[3]));
	seq_printf(m, "Pin 2\t%d\t%d\t%d\t%d\n",
		   PIN2Read(0), PIN2Read(1), PIN2Read(2), PIN2Read(3));
	seq_printf(m, "Pin 5\t%d\t%d\t%d\t%d\n",
		   PINRead(0, INPUT_PORT_PIN5), PINRead(1, INPUT_PORT_PIN5),
		   PINRead(2, INPUT_PORT_PIN5), PINRead(3, INPUT_PORT_PIN5));
	seq_printf(m, "Pin 6\t%d\t%d\t%d\t%d\n",
		   PINRead(0, INPUT_PORT_PIN6), PINRead(1, INPUT_PORT_PIN6),
		   PINRead(2, INPUT_PORT_PIN6), PINRead(3, INPUT_PORT_PIN6));
	seq_printf(m, "\t%04u\t%04u\t%04u\t%04u\n",
		   CtoV(pInputs[4]), CtoV(pInputs[5]),
		   CtoV(pInputs[6]), CtoV(pInputs[7]));
	seq_printf(m, "Buffer\t%d\t%d\t%d\t%d\n",
		   PINRead(0, INPUT_PORT_BUF), PINRead(1, INPUT_PORT_BUF),
		   PINRead(2, INPUT_PORT_BUF), PINRead(3, INPUT_PORT_BUF));
	seq_printf(m, "\n");
	seq_printf(m, "Port\tA\tB\tC\tD\n");
	seq_printf(m, "Pin 1\t%d\t%d\t%d\t%d\n",
		   POUTRead(0, OUTPUT_PORT_PIN1), POUTRead(1, OUTPUT_PORT_PIN1),
		   POUTRead(2, OUTPUT_PORT_PIN1), POUTRead(3, OUTPUT_PORT_PIN1));
	seq_printf(m, "Pin 2\t%d\t%d\t%d\t%d\n",
		   POUTRead(0, OUTPUT_PORT_PIN2), POUTRead(1, OUTPUT_PORT_PIN2),
		   POUTRead(2, OUTPUT_PORT_PIN2), POUTRead(3, OUTPUT_PORT_PIN2));
	seq_printf(m, "Pin 5W\t%d\t%d\t%d\t%d\n",
		   POUTRead(0, OUTPUT_PORT_PIN5W), POUTRead(1, OUTPUT_PORT_PIN5W),
		   POUTRead(2, OUTPUT_PORT_PIN5W), POUTRead(3, OUTPUT_PORT_PIN5W));
	seq_printf(m, "\t%04u\t%04u\t%04u\t%04u\n",
		   CtoV(pInputs[8]), CtoV(pInputs[9]),
		   CtoV(pInputs[10]), CtoV(pInputs[11]));
	seq_printf(m, "Pin 5R\t%d\t%d\t%d\t%d\n",
		   POUTRead(0, OUTPUT_PORT_PIN5R), POUTRead(1, OUTPUT_PORT_PIN5R),
		   POUTRead(2, OUTPUT_PORT_PIN5R), POUTRead(3, OUTPUT_PORT_PIN5R));
	seq_printf(m, "Pin 6\t%d\t%d\t%d\t%d\n",
		   POUTRead(0, OUTPUT_PORT_PIN6), POUTRead(1, OUTPUT_PORT_PIN6),
		   POUTRead(2, OUTPUT_PORT_PIN6), POUTRead(3, OUTPUT_PORT_PIN6));
	seq_printf(m, "\n");
	seq_printf(m, "Battery\n");
	seq_printf(m, "Temp\t%04u\n", CtoV(pInputs[12]));
	seq_printf(m, "Motor\t%04u\n", CtoV(pInputs[13]));
	seq_printf(m, "Amp\t%04u\n", CtoV(pInputs[14]));
	seq_printf(m, "Volt\t%04u\n", CtoV(pInputs[15]));

	return 0;
}

static int device1_debug_open(struct inode *inode, struct file *file)
{
	return single_open(file, device1_debug_show, NULL);
}

static const struct file_operations device1_debug_fops = {
	.open		= device1_debug_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int Device1Init(struct device * parent)
{
	u16 *pTmp;
	int  ret, i;
	s8   Port;

	Device1Lms2012CompatDev = lms2012_compat_get();
	if (!Device1Lms2012CompatDev)
		return -EPROBE_DEFER;

	Device1Lms2012Compat = dev_get_drvdata(Device1Lms2012CompatDev);

	kmalloc_ptr = kmalloc((NPAGES + 2) * PAGE_SIZE, GFP_KERNEL);
	if (!kmalloc_ptr) {
		put_device(Device1Lms2012CompatDev);
		return -ENOMEM;
	}

	pTmp = (u16*)((((unsigned long)kmalloc_ptr) + PAGE_SIZE - 1) & PAGE_MASK);
	for (i = 0; i < NPAGES * PAGE_SIZE; i += PAGE_SIZE) {
		SetPageReserved(virt_to_page(((unsigned long)pTmp) + i));
	}
	pAnalog = (ANALOG*)pTmp;
	memset(pAnalog, 0, sizeof(ANALOG));
	pInputs = pTmp;

	for (Port = 0;Port < INPUTS;Port++) {
		pAnalog->InDcm[Port]   = 0;
		pAnalog->InConn[Port]  = 0;
	}
	for (Port = 0;Port < OUTPUTS;Port++) {
		pAnalog->OutDcm[Port]  = 0;
		pAnalog->OutConn[Port] = 0;
	}

	SpiReset();
	SpiUpdate(0x400F);
	SpiUpdate(0x400F);
	SpiUpdate(0x400F);
	SpiUpdate(0x400F);
	SpiUpdate(0x400F);
	SpiUpdate(0x400F);

	Device1.parent = parent;
	ret = misc_register(&Device1);
	if (ret < 0) {
		for (i = 0; i < NPAGES * PAGE_SIZE; i+= PAGE_SIZE) {
			ClearPageReserved(virt_to_page(((unsigned long)pTmp) + i));
		}
		kfree(kmalloc_ptr);
		put_device(Device1Lms2012CompatDev);

		return ret;
	}

	// setup analog update timer interrupt

	Time1[0] = ktime_set(0, 2000000);
	Time1[1] = ktime_set(0, 6000000);
	Time2[0] = ktime_set(0, 2000000);
	Time2[1] = ktime_set(0, 4000000);

	NextTime = Time1[0];

#ifndef DISABLE_OLD_COLOR
	NxtColorCommInit();
#endif

	Device1Time = ktime_set(0, DEVICE_UPDATE_TIME);
	hrtimer_init(&Device1Timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	Device1Timer.function = Device1TimerInterrupt1;

	hrtimer_start(&Device1Timer, Device1Time, HRTIMER_MODE_REL);

	device1_debug = debugfs_create_file(DEVICE1_NAME, 0444, NULL, NULL,
					    &device1_debug_fops);

	return 0;
}

static void Device1Exit(void)
{
	u16   *pTmp;
	int     i;

	debugfs_remove(device1_debug);

	hrtimer_cancel(&Device1Timer);

	misc_deregister(&Device1);

	SpiReset();

	pTmp    = pInputs;
	pInputs = (u16*)&AnalogDefault;
	pAnalog = &AnalogDefault;

	for (i = 0; i < NPAGES * PAGE_SIZE; i+= PAGE_SIZE) {
		ClearPageReserved(virt_to_page(((unsigned long)pTmp) + i));
	}
	kfree(kmalloc_ptr);
	put_device(Device1Lms2012CompatDev);
}

// DEVICE3 ********************************************************************

#define DCM_TIMER_RESOLUTION		10                        // [mS]
#define DCM_DEVICE_RESET_TIME		2000                      // [mS]
#define DCM_FLOAT_DELAY			20                        // [mS]
#define DCM_LOW_DELAY			100                       // [mS]
#define DCM_TOUCH_DELAY			20                        // [mS]
#define DCM_CONNECT_STABLE_DELAY	IN_CONNECT_STEADY_TIME    // [mS]
#define DCM_EVENT_STABLE_DELAY		IN_DISCONNECT_STEADY_TIME // [mS]

#ifndef DISABLE_OLD_COLOR
#define DCM_NXT_COLOR_TIMEOUT		500                       // [mS]
#define DCM_NXT_COLOR_INIT_DELAY	100                       // [mS]
#define DCM_NXT_COLOR_HIGH_TIME		20                        // [mS]
#endif

static struct hrtimer	Device3Timer;
static ktime_t		Device3Time;

static u8 Device3State = 0;
static u16 Device3StateTimer;

#ifndef DISABLE_OLD_COLOR

static struct hrtimer NxtColorTimer;
static ktime_t        NxtColorTime;
#define NXTCOLOR_TIMER_RESOLUTION	200              // [uS]

#define NXTCOLOR_BYTES			(12 * 4 + 3 * 2)
#define NXTCOLOR_BITS			(NXTCOLOR_BYTES * 8)

static u8 NxtColorCmd[INPUTS];
static u8 NxtColorByte[INPUTS];
static u8 NxtColorTx[INPUTS];
static u8 NxtColorClkHigh[INPUTS];

static u8 NxtColorState[INPUTS] = { 0, 0, 0, 0 };
static u8 NxtColorBytePnt[INPUTS];
static u8 NxtColorByteCnt[INPUTS];
static u8 NxtColorBitCnt[INPUTS];
static u8 NxtColorBuffer[INPUTS][NXTCOLOR_BYTES];

static u16 NxtColorInitTimer[INPUTS];
static u8 NxtColorInitCnt[INPUTS];

static u8 NxtColorInitInUse;

static struct dentry *device3_debug;

static enum hrtimer_restart NxtColorCommIntr(struct hrtimer *pTimer)
{
	u8 Port;

	hrtimer_forward_now(pTimer, NxtColorTime);

	for (Port = 0;Port < NO_OF_INPUT_PORTS;Port++) { // look at one port at a time

		if (NxtColorState[Port]) {
			switch (NxtColorState[Port]) {
			case 1:
				PINFloat(Port, INPUT_PORT_PIN5);
				NxtColorState[Port]++;
				break;

			case 2:
				if (PINRead(Port, INPUT_PORT_PIN5)) {
					if (NxtColorInitCnt[Port] == 0) {
						PINHigh(Port, INPUT_PORT_PIN5);
						NxtColorState[Port]++;
					} else {
						NxtColorInitTimer[Port] = 0;
						NxtColorState[Port] += 2;
					}
				} else {
					PINHigh(Port, INPUT_PORT_PIN5);
					NxtColorState[Port]++;
				}
				break;

			case 3:
				PINLow(Port, INPUT_PORT_PIN5);
				if (++NxtColorInitCnt[Port] >= 2)
					NxtColorState[Port]++;
				else
					NxtColorState[Port] = 1;
				break;

			case 4:
				PINLow(Port, INPUT_PORT_PIN5);
				if (++NxtColorInitTimer[Port] >= ((DCM_NXT_COLOR_INIT_DELAY * 1000) / NXTCOLOR_TIMER_RESOLUTION))
					NxtColorState[Port]++;
				break;

			case 5:
				NxtColorBuffer[Port][0]  = NxtColorCmd[Port];
				NxtColorByteCnt[Port]    = 1;
				NxtColorBytePnt[Port]    = 0;
				NxtColorTx[Port]         = 1;
				NxtColorState[Port]++;
				break;

			case 6:
				if ((NxtColorBitCnt[Port] == 0)  && (NxtColorByteCnt[Port] == 0)) {
					NxtColorByteCnt[Port]    = NXTCOLOR_BYTES;
					NxtColorBytePnt[Port]    = 0;
					NxtColorTx[Port]         = 0;
					NxtColorState[Port]++;
				}
				break;

			case 7:
				if ((NxtColorBitCnt[Port] == 0)  && (NxtColorByteCnt[Port] == 0))
					NxtColorState[Port]++;
				break;

			default:
				NxtColorState[Port] = 0;
				break;

			}

			if (NxtColorBitCnt[Port]) {
				if (!NxtColorClkHigh[Port]) {
					if (NxtColorTx[Port]) {
						if (NxtColorByte[Port] & 1) {
							PINHigh(Port, INPUT_PORT_PIN6);
						} else {
							PINLow(Port, INPUT_PORT_PIN6);
						}
						NxtColorByte[Port] >>= 1;
					} else {
						PINFloat(Port, INPUT_PORT_PIN6);
					}
					PINHigh(Port, INPUT_PORT_PIN5);
					NxtColorClkHigh[Port] = 1;
				} else {

					NxtColorBitCnt[Port]--;
					if (!NxtColorTx[Port]) {
						NxtColorByte[Port] >>= 1;
						if (PINRead(Port, INPUT_PORT_PIN6)) {
							NxtColorByte[Port] |=  0x80;
						} else {
							NxtColorByte[Port] &= ~0x80;
						}
						if (!NxtColorBitCnt[Port]) {
							NxtColorBuffer[Port][NxtColorBytePnt[Port]] = NxtColorByte[Port];
							NxtColorBytePnt[Port]++;
						}
					}
					PINLow(Port, INPUT_PORT_PIN5);
					NxtColorClkHigh[Port] = 0;
				}
			} else {
				if (NxtColorByteCnt[Port]) {
					if (NxtColorTx[Port]) {
						NxtColorByte[Port] = NxtColorBuffer[Port][NxtColorBytePnt[Port]];
						NxtColorBytePnt[Port]++;
					}
					NxtColorBitCnt[Port] = 8;
					NxtColorByteCnt[Port]--;
				}
			}
		}
	}

	return HRTIMER_RESTART;
}

static void NxtColorCommStart(u8 Port, u8 Cmd)
{
	NxtColorState[Port]    = 1;
	NxtColorInitCnt[Port]  = 0;
	NxtColorBytePnt[Port]  = 0;
	NxtColorByteCnt[Port]  = 0;
	NxtColorBitCnt[Port]   = 0;
	NxtColorCmd[Port]      = Cmd;

	if (NxtColorInitInUse == 0)
		hrtimer_start(&NxtColorTimer, NxtColorTime, HRTIMER_MODE_REL);

	NxtColorInitInUse |=  (1 << Port);
}

static u8 NxtColorCommReady(u8 Port)
{
	u8   Result = 0;

	if (NxtColorState[Port] == 0) {
		Result = 1;
	}

	return Result;
}

static void NxtColorCommStop(u8 Port)
{

	NxtColorInitInUse &= ~(1 << Port);

	if (NxtColorInitInUse == 0) {
		hrtimer_try_to_cancel(&NxtColorTimer);
	}
	NxtColorState[Port] = 0;
}

static void NxtColorCommInit(void)
{
	NxtColorTime = ktime_set(0, NXTCOLOR_TIMER_RESOLUTION * 1000);
	hrtimer_init(&NxtColorTimer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	NxtColorTimer.function = NxtColorCommIntr;
}

#endif

static enum hrtimer_restart Device3TimerInterrupt1(struct hrtimer *pTimer)
{
	u8   Port;
	u8   Event;
	u16   Tmp;
#ifndef DISABLE_OLD_COLOR
	u8   *pData;
#endif

	hrtimer_forward_now(pTimer, Device3Time);

	switch (Device3State) {
	case 0:
		if (TestMode == 0)
			Device3State = 1;
		break;

	case 1:
//		IGENOff;
		Device3StateTimer = 0;
		Device3State++;
		break;

	case 2:
		if (++Device3StateTimer >= (DCM_DEVICE_RESET_TIME / DCM_TIMER_RESOLUTION)) {
//			IGENOn;
			Device3State++;
		}
		break;

	default:
		for (Port = 0;Port < NO_OF_INPUT_PORTS;Port++) { // look at one port at a time
			if (InputPort[Port].FSMEnabled == 0) {
				InputPort[Port].State =  DCM_DISABLED;
			}

			switch (InputPort[Port].State) {
			case DCM_INIT:
				 // set input port inactive
#ifndef DISABLE_OLD_COLOR
				NxtColorActive[Port] = 0;
				NxtColorCommStop(Port);
#endif
				InputPortFloat(Port);
				pAnalog->InDcm[Port]    = TYPE_NONE;
				pAnalog->InConn[Port]   = CONN_NONE;
				InputPort[Port].Timer     = 0;
				InputPort[Port].Event     = 0;
				InputPort[Port].State     = DCM_FLOATING_DELAY;
				break;

			case DCM_FLOATING_DELAY:
				// wait for port pins to float

				if (++(InputPort[Port].Timer) >= (DCM_FLOAT_DELAY / DCM_TIMER_RESOLUTION)) {
					InputPort[Port].Timer  = 0;
					InputPort[Port].State  = DCM_FLOATING;
				}
				break;

			case DCM_FLOATING:
				// pins floating - check and for connection event

				Event = 0;
				if (!PIN2Read(Port))
					Event |= 0x01 << INPUT_PORT_PIN2;
				if ((pAnalog->InPin1[Port]) < VtoC(IN1_NEAR_5V))	//<3931
				{ // pin 1 loaded

					Event |=  (0x01 << INPUT_PORT_VALUE);
				}
				if (!(PINRead(Port, INPUT_PORT_PIN5))) { // pin 5 low

					Event |=  (0x01 << INPUT_PORT_PIN5);
				}
				if ((PINRead(Port, INPUT_PORT_PIN6))) { // pin 6 high

					Event |=  (0x01 << INPUT_PORT_PIN6);
				}
				if (InputPort[Port].Event != Event) { // pins has changed - reset timer
#ifdef DEBUG
					printk("\nPort%d\n", Port);
					printk("i ! %d Event = %02X Old = %02X\n", Port, Event, InputPort[Port].Event);
#endif
					InputPort[Port].Event   = Event;
					InputPort[Port].Timer   = 0;
				}

				if (InputPort[Port].Event) { // some event

					if (++(InputPort[Port].Timer) >= (DCM_CONNECT_STABLE_DELAY / DCM_TIMER_RESOLUTION)) {
						// some event is stable

						InputPort[Port].State = DCM_CONNECTION;
					}
				}
				break;

			case DCM_CONNECTION:
				// something is connected - try to evaluate

				if (InputPort[Port].Event & (0x01 << INPUT_PORT_PIN2)) {
					InputPort[Port].State = DCM_PIN2_LOW;
				} else {
					if (InputPort[Port].Event & (0x01 << INPUT_PORT_VALUE))	//6, the right pos, & 0x40
					{ // pin 1 is loaded, 13

						InputPort[Port].State = DCM_PIN1_LOADED;
					} else {
						if (InputPort[Port].Event & (0x01 << INPUT_PORT_PIN6))	//3
						{ // pin 6 is high, 15

							InputPort[Port].State = DCM_PIN6_HIGH;
						} else {
							if (InputPort[Port].Event & (0x01 << INPUT_PORT_PIN5))	//3
							{ // pin 5 is low, 17

								InputPort[Port].State = DCM_PIN5_LOW;
							} else { // ?, 0

								InputPort[Port].State = DCM_INIT;
							}
						}
					}
				}
#ifndef DISABLE_FAST_DATALOG_BUFFER
				pAnalog->Actual[Port]    = 0;
				pAnalog->LogIn[Port]     = 0;
				pAnalog->LogOut[Port]    = 0;
#endif
				break;

			case DCM_PIN2_LOW:
				InputPort[Port].Connected  = 1;
				InputPortFloat(Port);
				InputPort[Port].Timer      = 0;
				InputPort[Port].State      = DCM_CONNECTED_WAITING_FOR_PIN2_HIGH;

				if ((!(InputPort[Port].Event & (0x01 << INPUT_PORT_PIN5))) && (InputPort[Port].Event & (0x01 << INPUT_PORT_PIN6))) { // pin 5 and 6 is high

					if (pAnalog->InPin1[Port] < VtoC(IN1_NEAR_GND)) { // nxt color sensor

						pAnalog->InDcm[Port]     = TYPE_NXT_COLOR;
#ifndef DISABLE_OLD_COLOR
						pAnalog->InConn[Port]    = CONN_NXT_COLOR;
						InputPort[Port].State      = DCM_NXT_COLOR_INIT;
#else
						pAnalog->InConn[Port]    = CONN_NXT_DUMB;
#endif
					} else { // nxt IIC sensor

						pAnalog->InDcm[Port]     = TYPE_NXT_IIC;
						pAnalog->InConn[Port]    = CONN_NXT_IIC;
					}
				} else {
					if (InputPort[Port].Event & (0x01 << INPUT_PORT_PIN5)) { // nxt light sensor

						if (InputPort[Port].Event & (0x01 << INPUT_PORT_PIN6)) { // nxt test sensor

							pAnalog->InDcm[Port]   = TYPE_NXT_TEST;
							pAnalog->InConn[Port]  = CONN_NXT_DUMB;
						} else {
							pAnalog->InDcm[Port]   = TYPE_NXT_LIGHT;
							pAnalog->InConn[Port]  = CONN_NXT_DUMB;
						}
					} else {
						if (pAnalog->InPin1[Port] < VtoC(IN1_NEAR_GND)) { // nxt color sensor

							pAnalog->InDcm[Port]     = TYPE_NXT_COLOR;
#ifndef DISABLE_OLD_COLOR
							pAnalog->InConn[Port]    = CONN_NXT_COLOR;
							InputPort[Port].State      = DCM_NXT_COLOR_INIT;
#else
							pAnalog->InConn[Port]    = CONN_NXT_DUMB;
#endif
						} else {
							if (pAnalog->InPin1[Port] > VtoC(IN1_NEAR_5V)) { // nxt touch sensor

								pAnalog->InDcm[Port]   = TYPE_NXT_TOUCH;
								pAnalog->InConn[Port]  = CONN_NXT_DUMB;
							} else {
								if ((pAnalog->InPin1[Port] > VtoC(IN1_TOUCH_LOW)) && (pAnalog->InPin1[Port] < VtoC(IN1_TOUCH_HIGH))) { // nxt touch sensor

									InputPort[Port].Timer    = 0;
									InputPort[Port].Value    = pAnalog->InPin1[Port];
									InputPort[Port].State    = DCM_NXT_TOUCH_CHECK;
								} else { // nxt sound sensor

									pAnalog->InDcm[Port]   = TYPE_NXT_SOUND;
									pAnalog->InConn[Port]  = CONN_NXT_DUMB;
								}
							}

						}
					}
				}
				break;

			case DCM_NXT_TOUCH_CHECK:
				if (++(InputPort[Port].Timer) >= (DCM_TOUCH_DELAY / DCM_TIMER_RESOLUTION)) {
					InputPort[Port].State = DCM_CONNECTED_WAITING_FOR_PIN2_HIGH;
					if ((pAnalog->InPin1[Port] > (InputPort[Port].Value - 10)) && (pAnalog->InPin1[Port] < (InputPort[Port].Value + 10))) { // nxt touch sensor

						pAnalog->InDcm[Port]   = TYPE_NXT_TOUCH;
						pAnalog->InConn[Port]  = CONN_NXT_DUMB;
					} else { // nxt sound sensor

						pAnalog->InDcm[Port]   = TYPE_NXT_SOUND;
						pAnalog->InConn[Port]  = CONN_NXT_DUMB;
					}
				}
				break;

#ifndef DISABLE_OLD_COLOR

			case DCM_NXT_COLOR_INIT:
				NxtcolorCmd[Port]          = 0;
				NxtColorCommStop(Port);

				InputPort[Port].Timer      = 0;
				InputPort[Port].State      = DCM_NXT_COLOR_WAIT;
				break;

			case DCM_NXT_COLOR_WAIT:
				if (NxtcolorCmd[Port] == NxtcolorLatchedCmd[Port]) {
					NxtColorCommStart(Port, InputPort[Port].Cmd);
					InputPort[Port].State = DCM_NXT_COLOR_BUSY;
				}
				break;

			case DCM_NXT_COLOR_BUSY:
				if (NxtColorCommReady(Port)) {
					NxtcolorCmd[Port]          = InputPort[Port].Cmd;
					InputPort[Port].Timer      = 0;
					InputPort[Port].State      = DCM_CONNECTED_WAITING_FOR_PIN2_HIGH;

					pData = (u8*)&(pAnalog->NxtCol[Port]);

					for (Tmp = 0;Tmp < NXTCOLOR_BYTES;Tmp++) {
						*pData = NxtColorBuffer[Port][Tmp];
						pData++;
					}

					NxtColorCommStop(Port);

					NxtColorActive[Port] = 1;
				}
				if (++(InputPort[Port].Timer) > (DCM_NXT_COLOR_TIMEOUT / DCM_TIMER_RESOLUTION)) {
#ifdef DEBUG
					printk("i ! %d NXT Color sensor timeout\n", Port);
#endif
					InputPort[Port].Timer      = 0;
					InputPort[Port].State      = DCM_CONNECTED_WAITING_FOR_PIN2_HIGH;
					NxtColorCommStop(Port);
				}
				break;
#endif
			case DCM_CONNECTED_WAITING_FOR_PIN2_HIGH:
				if (PIN2Read(Port)) {
					if (++(InputPort[Port].Timer) >= (DCM_EVENT_STABLE_DELAY / DCM_TIMER_RESOLUTION)) {
						InputPort[Port].Connected  = 0;
						InputPort[Port].State      = DCM_INIT;
					}
				} else {
					InputPort[Port].Timer = 0;
				}
				break;

			case DCM_PIN1_LOADED:
				if (pAnalog->InPin1[Port] > VtoC(IN1_NEAR_PIN2)) {
					pAnalog->InDcm[Port]     = TYPE_ERROR;
					pAnalog->InConn[Port]    = CONN_ERROR;
				} else {
					if (pAnalog->InPin1[Port] < VtoC(IN1_NEAR_GND)) {
						pAnalog->InDcm[Port]   = TYPE_UNKNOWN;
						pAnalog->InConn[Port]  = CONN_INPUT_UART;
					} else {
						pAnalog->InDcm[Port]   = TYPE_UNKNOWN;
						pAnalog->InConn[Port]  = CONN_INPUT_DUMB;
					}
				}
				InputPort[Port].Connected  = 1;
				InputPort[Port].Timer      = 0;
				InputPort[Port].State      = DCM_CONNECTED_WAITING_FOR_PIN1_TO_FLOAT;
				break;

			case DCM_CONNECTED_WAITING_FOR_PIN1_TO_FLOAT:
				if (pAnalog->InPin1[Port] > VtoC(IN1_NEAR_5V)) { // pin 1 floating

					if (++(InputPort[Port].Timer) >= (DCM_EVENT_STABLE_DELAY / DCM_TIMER_RESOLUTION)) {
						InputPort[Port].Connected  = 0;
						InputPort[Port].State      = DCM_INIT;
					}
				} else {
					InputPort[Port].Timer = 0;
				}
				break;

			case DCM_PIN6_HIGH:
				// nxt IIC sensor

				pAnalog->InDcm[Port]     = TYPE_NXT_IIC;
				pAnalog->InConn[Port]    = CONN_NXT_IIC;
				InputPort[Port].Connected  = 1;
				InputPort[Port].Timer      = 0;
				InputPort[Port].State      = DCM_CONNECTED_WAITING_FOR_PIN6_LOW;
				break;

			case DCM_CONNECTED_WAITING_FOR_PIN6_LOW:
				if (!(PINRead(Port, INPUT_PORT_PIN6))) {
					if (++(InputPort[Port].Timer) >= (DCM_EVENT_STABLE_DELAY / DCM_TIMER_RESOLUTION)) {
						InputPort[Port].Connected  = 0;
						InputPort[Port].State      = DCM_INIT;
					}
				} else {
					InputPort[Port].Timer = 0;
				}
				break;

			case DCM_PIN5_LOW:
				pAnalog->InDcm[Port]     = TYPE_ERROR;
				pAnalog->InConn[Port]    = CONN_ERROR;
				InputPort[Port].Connected  = 1;
				InputPort[Port].Timer      = 0;
				InputPort[Port].State      = DCM_CONNECTED_WAITING_FOR_PIN5_HIGH;
				break;

			case DCM_CONNECTED_WAITING_FOR_PIN5_HIGH:
				if ((PINRead(Port, INPUT_PORT_PIN5))) {
					if (++(InputPort[Port].Timer) >= (DCM_EVENT_STABLE_DELAY / DCM_TIMER_RESOLUTION)) {
						InputPort[Port].Connected  = 0;
						InputPort[Port].State      = DCM_INIT;
					}
				} else {
					InputPort[Port].Timer = 0;
				}
				break;

			case DCM_DISABLED:
				// If the FSM is disabled, set the timer to 0
				// and pretend we've got a connection, regardless of reality
				if (InputPort[Port].FSMEnabled == 0) {
					InputPort[Port].Timer     = 0;
					InputPort[Port].Connected = 1;
				} else {
					InputPort[Port].State = DCM_INIT;
				}
				break;

			default:
				InputPort[Port].State = DCM_INIT;
				break;

			}
#ifdef DEBUG
			if (InputPort[Port].OldState != InputPort[Port].State) {
				InputPort[Port].OldState = InputPort[Port].State;

				printk("i   %d %s\n", Port, DcmStateText[InputPort[Port].State]);
			}
#endif
		}

//*****************************************************************************

		for (Port = 0;Port < NO_OF_OUTPUT_PORTS;Port++) { // look at one port at a time

			switch (OutputPort[Port].State) {
			case DCM_INIT:
				// set output port inactive

				OutputPortFloat(Port);
				pAnalog->OutDcm[Port]   = TYPE_NONE;
				pAnalog->OutConn[Port]  = CONN_NONE;
				OutputPort[Port].Timer    = 0;
				OutputPort[Port].Event    = 0;
				OutputPort[Port].State    = DCM_FLOATING_DELAY;
				break;

			case DCM_FLOATING_DELAY:
				// wait for port pins to float

				if (++(OutputPort[Port].Timer) >= (DCM_FLOAT_DELAY / DCM_TIMER_RESOLUTION)) {
					OutputPort[Port].Timer  = 0;
					OutputPort[Port].State  = DCM_FLOATING;
				}
				break;

			case DCM_FLOATING:
				// pins floating - check and for connection event

				Event = 0;

				if (!(POUTRead(Port, OUTPUT_PORT_PIN6))) { // pin 6 low

					Event |=  (0x01 << OUTPUT_PORT_PIN6);
				}

				if ((pAnalog->OutPin5[Port] < VtoC(OUT5_BALANCE_LOW)) || (pAnalog->OutPin5[Port] > VtoC(OUT5_BALANCE_HIGH))) { // pin 5 out of balance

					Event |=  (0x01 << OUTPUT_PORT_VALUE);
				}

				if (OutputPort[Port].Event != Event) { // pins has changed - reset timer

					OutputPort[Port].Event  = Event;
					OutputPort[Port].Timer  = 0;
				}

				if (OutputPort[Port].Event) { // some event

					if (++(OutputPort[Port].Timer) >= (DCM_CONNECT_STABLE_DELAY / DCM_TIMER_RESOLUTION)) {
						// some event is stable - store value on connection 5

						OutputPort[Port].Value5Float = CtoV(pAnalog->OutPin5[Port]);
						OutputPort[Port].Timer       = 0;
						OutputPort[Port].State       = DCM_WAITING_FOR_PIN6_LOW;
						POUTFloat(Port, OUTPUT_PORT_PIN6);
					}
				}
				break;

			case DCM_WAITING_FOR_PIN6_LOW:
				if (++(OutputPort[Port].Timer) >= (DCM_LOW_DELAY / DCM_TIMER_RESOLUTION)) {
					OutputPort[Port].Value5Low   = CtoV(pAnalog->OutPin5[Port]);
					OutputPort[Port].State       = DCM_CONNECTION;
					POUTFloat(Port, OUTPUT_PORT_PIN6);
				}
				break;

			case DCM_CONNECTION:
				// something is connected - try to evaluate

				OutputPort[Port].State = DCM_CONNECTED_WAITING_FOR_PORT_OPEN;
				Tmp = ADC_REF;
				Tmp +=  OutputPort[Port].Value5Float;
				Tmp -=  OutputPort[Port].Value5Low;

				if ((Tmp > (ADC_REF - 50)) && (Tmp < (ADC_REF + 50))) { // Value5Float is equal to Value5Low

					if ((OutputPort[Port].Value5Float >= OUT5_BALANCE_LOW) && (OutputPort[Port].Value5Float <= OUT5_BALANCE_HIGH) && (OutputPort[Port].Event & (0x01 << OUTPUT_PORT_PIN6))) { // NXT TOUCH SENSOR, NXT SOUND SENSOR or NEW UART SENSOR

						pAnalog->OutDcm[Port]    = TYPE_ERROR;
						pAnalog->OutConn[Port]   = CONN_ERROR;
						OutputPort[Port].Connected = 1;
					} else {
						if (OutputPort[Port].Value5Float < OUT5_NEAR_GND) { // NEW DUMB SENSOR

							pAnalog->OutDcm[Port]    = TYPE_ERROR;
							pAnalog->OutConn[Port]   = CONN_ERROR;
							OutputPort[Port].Connected = 1;
						} else {
							if ((OutputPort[Port].Value5Float >= OUT5_LIGHT_LOW) && (OutputPort[Port].Value5Float <= OUT5_LIGHT_HIGH)) { // NXT LIGHT SENSOR

								pAnalog->OutDcm[Port]    = TYPE_ERROR;
								pAnalog->OutConn[Port]   = CONN_ERROR;
								OutputPort[Port].Connected = 1;
							} else {
								if ((OutputPort[Port].Value5Float >= OUT5_IIC_LOW) && (OutputPort[Port].Value5Float <= OUT5_IIC_HIGH)) { // NXT IIC SENSOR

									pAnalog->OutDcm[Port]    = TYPE_ERROR;
									pAnalog->OutConn[Port]   = CONN_ERROR;
									OutputPort[Port].Connected = 1;
								} else {
									if (OutputPort[Port].Value5Float < OUT5_BALANCE_LOW) {
										if ((OutputPort[Port].Value5Float >= OUT5_DUMP_LOW) && (OutputPort[Port].Value5Float < OUT5_DUMP_HIGH)) {
											pAnalog->OutPin5Low[Port] = OutputPort[Port].Value5Float;
											pAnalog->OutDcm[Port]     = TYPE_UNKNOWN;
											pAnalog->OutConn[Port]    = CONN_OUTPUT_DUMB;
										} else {
											if ((OutputPort[Port].Value5Float >= OUT5_INTELLIGENT_LOW2) && (OutputPort[Port].Value5Float < OUT5_INTELLIGENT_HIGH2)) {
												pAnalog->OutDcm[Port]     = TYPE_UNKNOWN;
												pAnalog->OutConn[Port]    = CONN_OUTPUT_INTELLIGENT;
											} else {
												if ((OutputPort[Port].Value5Float >= OUT5_NEWTACHO_LOW2) && (OutputPort[Port].Value5Float < OUT5_NEWTACHO_HIGH2)) {
													pAnalog->OutDcm[Port]    = TYPE_NEWTACHO;
													pAnalog->OutConn[Port]   = CONN_OUTPUT_TACHO;
												} else {
													if ((OutputPort[Port].Value5Float >= OUT5_MINITACHO_LOW2) && (OutputPort[Port].Value5Float < OUT5_MINITACHO_HIGH2)) {
														pAnalog->OutDcm[Port]  = TYPE_MINITACHO;
														pAnalog->OutConn[Port] = CONN_OUTPUT_TACHO;
													} else {
														pAnalog->OutDcm[Port]  = TYPE_TACHO;
														pAnalog->OutConn[Port] = CONN_OUTPUT_TACHO;
													}
												}
											}
										}
										OutputPort[Port].Connected   = 1;
									} else {
										POUTHigh(Port, OUTPUT_PORT_PIN5W);
										OutputPort[Port].State = DCM_WAITING_FOR_PIN5_LOW;
									}
								}
							}
						}
					}

				} else { // Value5Float is NOT equal to Value5Low

					if ((OutputPort[Port].Value5Low > OUT5_NEAR_GND) && (OutputPort[Port].Value5Low < OUT5_BALANCE_LOW)) { // NEW ACTUATOR

						pAnalog->OutPin5Low[Port]  = OutputPort[Port].Value5Low;
						pAnalog->OutDcm[Port]      = TYPE_UNKNOWN;
						pAnalog->OutConn[Port]     = CONN_OUTPUT_DUMB;
						OutputPort[Port].Connected   = 1;
					} else {
						pAnalog->OutDcm[Port]      = TYPE_ERROR;
						pAnalog->OutConn[Port]     = CONN_ERROR;
						OutputPort[Port].Connected   = 1;
					}

				}
				OutputPort[Port].Timer = 0;
#ifdef DEBUG
				if (OutputPort[Port].Connected) {
					printk("\no    %d Type = %c, Float = %u, Low = %u\n", Port, (char)pAnalog->OutDcm[Port], (unsigned int)OutputPort[Port].Value5Float, (unsigned int)OutputPort[Port].Value5Low);
				}
#endif
				break;

			case DCM_WAITING_FOR_PIN5_LOW:
				if (++(OutputPort[Port].Timer) >= (DCM_LOW_DELAY / DCM_TIMER_RESOLUTION)) {
					OutputPort[Port].Value5Low     = CtoV(pAnalog->OutPin5[Port]);
					OutputPort[Port].State         = DCM_CONNECTION;
					POUTLow(Port, OUTPUT_PORT_PIN5W);

					if ((OutputPort[Port].Value5Low >= OUT5_NEWTACHO_LOW1) && (OutputPort[Port].Value5Low < OUT5_NEWTACHO_HIGH1)) {
						pAnalog->OutDcm[Port]      = TYPE_NEWTACHO;
						pAnalog->OutConn[Port]     = CONN_OUTPUT_TACHO;
					} else {
						if ((OutputPort[Port].Value5Low >= OUT5_MINITACHO_LOW1) && (OutputPort[Port].Value5Low < OUT5_MINITACHO_HIGH1)) {
							pAnalog->OutDcm[Port]    = TYPE_MINITACHO;
							pAnalog->OutConn[Port]   = CONN_OUTPUT_TACHO;
						} else {
							pAnalog->OutDcm[Port]    = TYPE_TACHO;
							pAnalog->OutConn[Port]   = CONN_OUTPUT_TACHO;
						}
					}
					OutputPort[Port].Connected = 1;
#ifdef DEBUG
					printk("\no   %d Type = %03u, Float = %u, Low = %u\n", Port, (char)pAnalog->OutDcm[Port], (unsigned int)OutputPort[Port].Value5Float, (unsigned int)OutputPort[Port].Value5Low);
#endif
					OutputPort[Port].State = DCM_CONNECTED_WAITING_FOR_PORT_OPEN;
				}
				break;

			case DCM_CONNECTED_WAITING_FOR_PORT_OPEN:
				if ((pAnalog->OutPin5[Port] < VtoC(OUT5_BALANCE_LOW)) || (pAnalog->OutPin5[Port] > VtoC(OUT5_BALANCE_HIGH))) { // connection 5 out of balance

					OutputPort[Port].Timer = 0;
				}

				if (!(POUTRead(Port, OUTPUT_PORT_PIN6))) { // pin 6 low

					OutputPort[Port].Timer = 0;
				}

				if (++(OutputPort[Port].Timer) >= (DCM_EVENT_STABLE_DELAY / DCM_TIMER_RESOLUTION)) {
					OutputPort[Port].Connected = 0;
					OutputPort[Port].State     = DCM_INIT;
				}
				break;

			default:
				OutputPort[Port].State = DCM_INIT;
				break;

			}
#ifdef DEBUG
			if (OutputPort[Port].OldState != OutputPort[Port].State) {
				OutputPort[Port].OldState = OutputPort[Port].State;

				printk("o   %d %s\n", Port, DcmStateText[OutputPort[Port].State]);
			}
#endif
		}
		break;
	}

	return HRTIMER_RESTART;
}

static ssize_t Device3Write(struct file *File, const char *Buffer, size_t Count, loff_t *Data)
{
	char Buf[INPUTS + 1];
	u8   Port;
	u8   Char;
	int  ret;

	if (Count < INPUTS)
		return -EINVAL;

	ret = copy_from_user(Buf, Buffer, INPUTS);
	if (ret < 0)
		return ret;

	for (Port = 0;Port < NO_OF_INPUT_PORTS;Port++) {
		Char = Buf[Port];
		switch (Char) {
		case '-' :
			// do nothing
			break;

		case 'f' :
			// float
			InputPortFloat(Port);
			break;

		default:
			if (InputPort[Port].Connected) {
				if ((Char & 0xF8) == '0') { // 0, 1, 2, 3, 4, 5, 6, 7

					if (Char & 0x01) { // pin 1
						PIN1High(Port);
					} else {
						PIN1Low(Port);
					}
					if (Char & 0x02) { // pin 5

						PINHigh(Port, INPUT_PORT_PIN5);
					} else {
						PINLow(Port, INPUT_PORT_PIN5);
					}
				}
#ifndef DISABLE_OLD_COLOR
				else {
					if ((Char >= 0x0D) && (Char <= 0x11)) { // NXT color sensor setup

						InputPort[Port].Cmd   = Char;
						InputPort[Port].Timer = 0;
						InputPort[Port].State = DCM_NXT_COLOR_INIT;
					}
				}
#endif
			}
			break;
		}
	}

	return Count;
}

static const struct file_operations Device3Entries = {
	.owner	= THIS_MODULE,
	.write	= Device3Write,
};

static struct miscdevice Device3 = {
	.minor	= MISC_DYNAMIC_MINOR,
	.name	= DEVICE3_NAME,
	.fops	= &Device3Entries,
};

static int device3_debug_show(struct seq_file *m, void *data)
{
	seq_printf(m, "Port\t1\t2\t3\t4\n");
	seq_printf(m, "State\t%s\t%s\t%s\t%s\n",
		   DcmStateText[InputPort[0].State],
		   DcmStateText[InputPort[1].State],
		   DcmStateText[InputPort[2].State],
		   DcmStateText[InputPort[3].State]);
	seq_printf(m, "Type\t%d\t%d\t%d\t%d\n",
		   pAnalog->InDcm[0], pAnalog->InDcm[1],
		   pAnalog->InDcm[2], pAnalog->InDcm[3]);
	seq_printf(m, "Conn\t%d\t%d\t%d\t%d\n",
		   pAnalog->InConn[0], pAnalog->InConn[1],
		   pAnalog->InConn[2], pAnalog->InConn[3]);
	seq_printf(m, "\n");
	seq_printf(m, "Port\tA\tB\tC\tD\n");
	seq_printf(m, "State\t%s\t%s\t%s\t%s\n",
		   DcmStateText[OutputPort[0].State],
		   DcmStateText[OutputPort[1].State],
		   DcmStateText[OutputPort[2].State],
		   DcmStateText[OutputPort[3].State]);
	seq_printf(m, "Type\t%d\t%d\t%d\t%d\n",
		   pAnalog->OutDcm[0], pAnalog->OutDcm[1],
		   pAnalog->OutDcm[2], pAnalog->OutDcm[3]);
	seq_printf(m, "Conn\t%d\t%d\t%d\t%d\n",
		   pAnalog->OutConn[0], pAnalog->OutConn[1],
		   pAnalog->OutConn[2], pAnalog->OutConn[3]);

	return 0;
}

static int device3_debug_open(struct inode *inode, struct file *file)
{
	return single_open(file, device3_debug_show, NULL);
}

static const struct file_operations device3_debug_fops = {
	.open		= device3_debug_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int Device3Init(struct device * parent)
{
	int ret, Tmp;

	for (Tmp = 0;Tmp < NO_OF_INPUT_PORTS;Tmp++)
		InputPort[Tmp] = InputPortDefault;
	for (Tmp = 0;Tmp < NO_OF_OUTPUT_PORTS;Tmp++)
		OutputPort[Tmp] = OutputPortDefault;

	Device3State = 0;
	TestMode     = 0;

	Device3.parent = parent;
	ret = misc_register(&Device3);
	if (ret < 0)
		return ret;

	Device3Time = ktime_set(0, DCM_TIMER_RESOLUTION * 1000000);
	hrtimer_init(&Device3Timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	Device3Timer.function = Device3TimerInterrupt1;

	hrtimer_start(&Device3Timer, Device3Time, HRTIMER_MODE_REL);

	device3_debug = debugfs_create_file(DEVICE3_NAME, 0444, NULL, NULL,
					    &device3_debug_fops);

	return 0;
}

static void Device3Exit(void)
{
	int Tmp;

	debugfs_remove(device3_debug);

	hrtimer_cancel(&Device3Timer);
	misc_deregister(&Device3);

	for (Tmp = 0;Tmp < NO_OF_INPUT_PORTS;Tmp++)
		InputPortFloat(Tmp);
}

// MODULE *********************************************************************

static struct lms2012_battery *bat;

static int d_analog_get_volts(void *context)
{
	return CtoV(pInputs[15]);
}

static int d_analog_get_amps(void *context)
{
	return CtoV(pInputs[14]);
}

static int d_analog_probe(struct platform_device *pdev)
{
	int ret;

	ret = Device1Init(&pdev->dev);
	if (ret < 0)
		return ret;
	ret = Device3Init(&pdev->dev);
	if (ret < 0)
		goto err0;

	if (Device1Lms2012Compat->adc_gpios) {
		bat = lms2012_battery_probe(Device1Lms2012CompatDev,
			Device1Lms2012Compat->adc_gpios->desc[ADC_VOLT_ENA_GPIO],
			Device1Lms2012Compat->adc_gpios->desc[ADC_BATT_TYPE_GPIO],
			d_analog_get_volts, d_analog_get_amps, NULL);
		ret = PTR_ERR_OR_ZERO(bat);
		if (ret < 0)
			goto err1;
	}

	pr_info("d_analog registered\n");

	return 0;

err1:
	Device3Exit();
err0:
	Device1Exit();

	return ret;
}

static int d_analog_remove(struct platform_device *pdev)
{
	if (bat)
		lms2012_battery_remove(bat);
	Device3Exit();
	Device1Exit();

	pr_info("d_analog removed\n");

	return 0;
}

static struct platform_driver d_analog_driver = {
	.driver	= {
		.name	= "d_analog",
	},
	.probe	= d_analog_probe,
	.remove	= d_analog_remove,
};
module_platform_driver(d_analog_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("The LEGO Group");
MODULE_DESCRIPTION("d_analog from lms2012");
MODULE_ALIAS("platform:d_analog");
