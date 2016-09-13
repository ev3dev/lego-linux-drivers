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
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/hrtimer.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/ioport.h>
#include <linux/kernel.h>
#include <linux/miscdevice.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/uaccess.h>

#include <asm/io.h>

#include "lms2012.h"

#define DEBUG_UART		(-1)    // must match settings in lms2012

#define LOWEST_BITRATE		  2400  //  Lowest possible bit rate (always used for sync and info)  [b/S]
#define MIDLE_BITRATE		 57600  //  Highest bit rate allowed when adjusting clock             [b/S]
#define HIGHEST_BITRATE		460800  //  Highest possible bit rate                                 [b/S]

// FIRST BYTE

#define BYTE_SYNC		0x00                            // Synchronisation byte
#define BYTE_ACK		0x04                            // Acknowledge byte
#define BYTE_NACK		0x02                            // Not acknowledge byte

#define MESSAGE_SYS		0x00                            // System message
#define MESSAGE_CMD		0x40                            // Command message
#define MESSAGE_INFO		0x80                            // Info message
#define MESSAGE_DATA		0xC0                            // Data message
#define GET_MESSAGE_TYPE(B)	(B & 0xC0)                      // Get message type

#define CMD_TYPE		0x00                            // CMD command - TYPE     (device type for VM reference)
#define CMD_MODES		0x01                            // CMD command - MODES    (number of supported modes 0=1)
#define CMD_SPEED		0x02                            // CMD command - SPEED    (maximun communication speed)
#define CMD_SELECT		0x03                            // CMD command - SELECT   (select mode)
#define CMD_WRITE		0x04                            // CMD command - WRITE    (write to device)
#define GET_CMD_COMMAND(B)	(B & 0x07)                      // Get CMD command

#define GET_MODE(B)		(B & 0x07)                      // Get mode

#define CONVERT_LENGTH(C)	(1 << (C & 0x07))
#define GET_MESSAGE_LENGTH(B)	(CONVERT_LENGTH(B >> 3))        // Get message length exclusive check byte

#define MAKE_CMD_COMMAND(C, LC)	(MESSAGE_CMD + (C & 0x07) + ((LC & 0x07) << 3))

// SECOND INFO BYTE

#define INFO_NAME		0x00                            // INFO command - NAME    (device name)
#define INFO_RAW		0x01                            // INFO command - RAW     (device RAW value span)
#define INFO_PCT		0x02                            // INFO command - PCT     (device PCT value span)
#define INFO_SI			0x03                            // INFO command - SI      (device SI  value span)
#define INFO_SYMBOL		0x04                            // INFO command - SYMBOL  (device SI  unit symbol)
#define INFO_FORMAT		0x80                            // INFO command - FORMAT  (device data sets and format)
#define GET_INFO_COMMAND(B)	(B)                             // Get INFO command

static const TYPES TypeDefaultUart[] = {
	{
		.Name		= "",
		.Type		= TYPE_UNKNOWN,
		.Connection	= CONN_UNKNOWN,
		.Mode		= 0,
		.DataSets	= 1,
		.Format		= 1,
		.Figures	= 4,
		.Decimals	= 0,
		.Views		= 0,
		.RawMin		= 0.0,
		.RawMax		= 1023.0,
		.PctMin		= 0.0,
		.PctMax		= 100.0,
		.SiMin		= 0.0,
		.SiMax		= 1023.0,
		.InvalidTime	= 10,
		.IdValue	= 0,
		.Pins		= '-',
		.Symbol		= "",
	},
	{
		.Name		= "TERMINAL",
		.Type		= TYPE_TERMINAL,
		.Connection	= CONN_INPUT_UART,
		.Mode		= 0,
		.DataSets	= 0,
		.Format		= 0,
		.Figures	= 4,
		.Decimals	= 0,
		.Views		= 0,
		.RawMin		= 0.0,
		.RawMax		= 4095.0,
		.PctMin		= 0.0,
		.PctMax		= 100.0,
		.SiMin		= 0.0,
		.SiMax		= 1000.0,
		.InvalidTime	= 0,
		.IdValue	= 0,
		.Pins		= '-',
		.Symbol		= "",
	},
	{
		/* list terminator */
		.Name = "\0"
	}
};

#define MODULE_NAME	"uart_module"
#define DEVICE1_NAME	UART_DEVICE
#define DEVICE2_NAME	TEST_UART_DEVICE

static struct device *Device1Lms2012CompatDev;
static struct lms2012_compat *Device1Lms2012Compat;

enum UartPort {
	Uart1,
	Uart2,
	Uart3,
	Uart4,
};

#define PUARTFloat(port, pin) \
	gpiod_direction_input(Device1Lms2012Compat->in_pins[port]->desc[pin])

#define PUARTRead(port, pin) \
	gpiod_get_value(Device1Lms2012Compat->in_pins[port]->desc[pin])

#define PUARTHigh(port, pin) \
	gpiod_direction_output(Device1Lms2012Compat->in_pins[port]->desc[pin], 1)

#define PUARTLow(port, pin) \
	gpiod_direction_output(Device1Lms2012Compat->in_pins[port]->desc[pin], 0)

#define UART_RHR		(0x00>>2)
#define UART_THR		(0x00>>2)
#define UART_DLL		(0x00>>2)
#define UART_IER		(0x04>>2)
#define UART_DLH		(0x04>>2)
#define UART_EFR		(0x08>>2)
#define UART_IIR		(0x08>>2)
#define UART_FCR		(0x08>>2)
#define UART_LCR		(0x0C>>2)
#define UART_MCR		(0x10>>2)
#define UART_LSR		(0x14>>2)
#define UART_TCR		(0x18>>2)
#define UART_MSR		(0x18>>2)
#define UART_TLR		(0x1C>>2)
#define UART_MDR1		(0x20>>2)
#define UART_MDR2		(0x24>>2)
#define UART_SCR		(0x40>>2)

#define UART_SYSC		(0x54>>2)
#define UART_SYSS		(0x58>>2)

#define UART_RECBUF_SIZE	256

#ifdef DEBUG_D_UART_ERROR
#define UARTBUFFERSIZE		250
static char UartBuffer[UARTBUFFERSIZE];
#endif

static UBYTE UartPortSend(UBYTE Port, UBYTE Byte);

UWORD   ShowTimer[INPUTS];

#define LOGPOOLSIZE 100000
static ULONG LogPointer = 0;
static ULONG LogOutPointer = 0;
static char LogPool[LOGPOOLSIZE];

void UartWrite(char *pString)
{
	ULONG Tmp;

	while (*pString) {
		LogPool[LogPointer] = *pString;

		Tmp = LogPointer;
		Tmp++;
		if (Tmp >= LOGPOOLSIZE) {
			Tmp = 0;
		}
		if (Tmp != LogOutPointer) {
			LogPointer = Tmp;
			pString++;

		} else {
//			if (UartPortSend(DEBUG_UART, *pString)) {
				pString++;
			// }
		}
	}
}

// UART 1 *********************************************************************

static volatile u32 *Uart1Base;
static char Uart1Name[20];

static UBYTE Uart1RecBuf[UART_RECBUF_SIZE];
static UWORD Uart1RecBufIn;
static UWORD Uart1RecBufOut;

static UBYTE Uart1RecMesLng;
static UBYTE Uart1RecMes[UART_BUFFER_SIZE];
static UBYTE Uart1RecMesIn;

irqreturn_t Uart1Interrupt(int irq, void *dev_id)
{
	UBYTE IntrType;

	IntrType = (UBYTE)Uart1Base[UART_IIR] & 0x0F;

	while (!(IntrType & 1)) {
		if (IntrType == 2) {
		} else {
			if (IntrType & 2) {
				Uart1RecBuf[Uart1RecBufIn] = (UBYTE)Uart1Base[UART_LSR];
			}
			Uart1RecBuf[Uart1RecBufIn] = (UBYTE)Uart1Base[UART_RHR];

			if (++Uart1RecBufIn >= UART_RECBUF_SIZE) {
				Uart1RecBufIn = 0;
			}
		}
		IntrType = (UBYTE)Uart1Base[UART_IIR] & 0x0F;
	}

	return IRQ_HANDLED;
}

static UBYTE Uart1Read(UBYTE *pByte)
{
	UBYTE   Result = 0;

	if (Uart1RecBufIn != Uart1RecBufOut) {
		*pByte = Uart1RecBuf[Uart1RecBufOut];

		if (++Uart1RecBufOut >= UART_RECBUF_SIZE) {
			Uart1RecBufOut = 0;
		}
		Result   = 1;
	}

	return Result;
}

static void Uart1Flush(void)
{
	Uart1Base[UART_FCR]  = 0x07;
	Uart1RecBufIn        = 0;
	Uart1RecBufOut       = 0;
	Uart1RecMesIn        = 0;
}

static UBYTE Uart1ReadData(UBYTE *pCmd, UBYTE *pData, UBYTE *pCheck, UBYTE *pFail)
{
	UBYTE   Byte;
	UBYTE   Length;
	UBYTE   Collect;

	Length   = 0;
	*pFail   = 0xFF;
	Collect  = 1;

	while (Collect) {
		if (Uart1Read(&Byte)) {
			if (Uart1RecMesIn == 0) { // Wait for data message start

				if (GET_MESSAGE_TYPE(Byte) == MESSAGE_DATA) {

					Uart1RecMesLng = GET_MESSAGE_LENGTH(Byte) + 2;

					if (Uart1RecMesLng <= UART_BUFFER_SIZE) { // Valid length

						Uart1RecMes[Uart1RecMesIn] = Byte;
						Uart1RecMesIn++;
					}
				}
			} else {
				Uart1RecMes[Uart1RecMesIn] = Byte;

				if (++Uart1RecMesIn >= Uart1RecMesLng) { // Message ready

					*pCmd   = Uart1RecMes[0];
					*pFail  ^=  *pCmd;

					while (Length < (Uart1RecMesLng - 2)) {
						pData[Length]  = Uart1RecMes[Length + 1];
						*pFail       ^=  pData[Length];
						Length++;
					}
					*pCheck = Uart1RecMes[Length + 1];
					*pFail  ^=  *pCheck;

					Uart1RecMesIn = 0;
					Collect = 0;
				}
			}
		} else {
			Collect = 0;
		}
	}

	return Length;
}

static UBYTE Uart1Write(UBYTE Byte)
{
	UBYTE Result = 0;

	if (Uart1Base[UART_LSR] & 0x20) {
		Uart1Base[UART_THR]  = Byte;
		Result               = 1;
	}

	return Result;
}

static void Uart1Setup(ULONG BitRate)
{
	ULONG Divisor;

	Divisor = Device1Lms2012Compat->uart_clock_freq[Uart1] / (BitRate * (ULONG)16);

	Uart1Base[UART_LCR] = 0xBF;
	Uart1Base[UART_EFR] |= 0x10;    //Enables access to MCR

	Uart1Base[UART_LCR] = 0x80;
	Uart1Base[UART_MCR] |= 0x40;    //Enables access to TCR and TLR

	Uart1Base[UART_LCR] = 0xBF;
	Uart1Base[UART_SCR] |= 0x80;    //Enable granularity of 1 for trigger RX level
	Uart1Base[UART_SCR] &= ~(0x40); //Disable granularity of 1 for trigger TX level
	Uart1Base[UART_TLR] = 0x0;

	Uart1Base[UART_SCR] |= 0x1; //DMAMODE is set with SCR[2:1]
	Uart1Base[UART_SCR] &= ~(0x6);  //no DMA

	Uart1Base[UART_LCR] = 0x7F;
	Uart1Base[UART_IER] &= ~(0x10); //Disables sleep mode

	Uart1Base[UART_LCR] = 0xBF;

	Uart1Base[UART_MDR1] |= 0x7;    //Disable uart
	Uart1Base[UART_DLL] = 0x0;
	Uart1Base[UART_DLH] = 0x0;
	//Uart1Base[UART_MDR1] &= ~(0x7);   //uart 16x mode

	Uart1Base[UART_LCR] = 0x80;
	Uart1Base[UART_FCR] = (1 << 6) | 0x7;

	Uart1Base[UART_LCR] = 0xBF;

	//Uart1Base[UART_MDR1] |= 0x7;  //Disable uart
	Uart1Base[UART_DLL] = Divisor & 0xFF;
	Uart1Base[UART_DLH] = (Divisor & 0x3F00) >> 8;
	Uart1Base[UART_MDR1] &= ~(0x7); //uart 16x mode

	Uart1Base[UART_LCR] = 0x3;
	Uart1Base[UART_IER] = 0x1;
}

static int Uart1Init(void)
{
	int timeout = 10000;
	int ret;

	snprintf(Uart1Name, 20, "%s.port%d", DEVICE1_NAME, Uart1 + 1);
	Uart1Base = Device1Lms2012Compat->uart_mem[Uart1];

	Uart1Base[UART_SYSC] |= 0x2;
	while(!(Uart1Base[UART_SYSS] & 0x1)) {
		if (!timeout--)
			return -ETIMEDOUT;
		udelay(1);
	}

	ret = request_irq(Device1Lms2012Compat->uart_irq[Uart1], &Uart1Interrupt,
			  IRQF_SHARED, Uart1Name, &Uart1Name);

	return ret;
}

void Uart1Exit(void)
{
	Uart1Base[UART_IER] = 0x00;
	free_irq(Device1Lms2012Compat->uart_irq[Uart1], Uart1Name);
}

// UART 2 *********************************************************************

typedef struct {
	ULONG InfoData;
	ULONG BitRate;
	ULONG BitRateMax;
	UWORD Timer;
	UWORD WatchDog;
	UWORD BreakTimer;
	UBYTE Initialised;
	UBYTE ChangeMode;
	UBYTE State;
	UBYTE OldState;
	UBYTE SubState;
	UBYTE Cmd;
	UBYTE InfoCmd;
	UBYTE Check;
	UBYTE Types;
	UBYTE Views;
	UBYTE Mode;
	UBYTE Type;
	UBYTE DataOk;
	UBYTE DataErrors;
	SBYTE Name[TYPE_NAME_LENGTH + 1];
	UBYTE InLength;
	UBYTE InPointer;
	UBYTE OutLength;
	UBYTE OutPointer;
	UBYTE InBuffer[UART_BUFFER_SIZE];
	UBYTE OutBuffer[UART_BUFFER_SIZE];
} UARTPORT;

static UARTPORT UartPort[INPUTS];

static volatile u32 *Uart2Base;
static char Uart2Name[20];

static UBYTE Uart2RecBuf[UART_RECBUF_SIZE];
static UWORD Uart2RecBufIn;
static UWORD Uart2RecBufOut;

static UBYTE Uart2RecMesLng;
static UBYTE Uart2RecMes[UART_BUFFER_SIZE];
static UBYTE Uart2RecMesIn;

irqreturn_t Uart2Interrupt(int irq, void *dev_id)
{
	UBYTE IntrType;

	IntrType = (UBYTE)Uart2Base[UART_IIR] & 0x0F;

	while (!(IntrType & 1)) {
		if (IntrType == 2) {
		} else {
			if (IntrType & 2) {
				Uart2RecBuf[Uart2RecBufIn] = (UBYTE)Uart2Base[UART_LSR];
			}
			Uart2RecBuf[Uart2RecBufIn] = (UBYTE)Uart2Base[UART_RHR];

			if (++Uart2RecBufIn >= UART_RECBUF_SIZE) {
				Uart2RecBufIn = 0;
			}
		}
		IntrType = (UBYTE)Uart2Base[UART_IIR] & 0x0F;
	}

	return IRQ_HANDLED;
}

static UBYTE Uart2Read(UBYTE *pByte)
{
	UBYTE Result = 0;

	if (Uart2RecBufIn != Uart2RecBufOut) {
		*pByte = Uart2RecBuf[Uart2RecBufOut];

		if (++Uart2RecBufOut >= UART_RECBUF_SIZE) {
			Uart2RecBufOut = 0;
		}
		Result = 1;
	}

	return Result;
}

static void Uart2Flush(void)
{
	Uart2Base[UART_FCR] |= 0x06;
	Uart2RecBufIn        = 0;
	Uart2RecBufOut       = 0;
	Uart2RecMesIn        = 0;
}

static UBYTE Uart2ReadData(UBYTE *pCmd, UBYTE *pData, UBYTE *pCheck, UBYTE *pFail)
{
	UBYTE   Byte;
	UBYTE   Length;
	UBYTE   Collect;

	Length   = 0;
	*pFail   = 0xFF;
	Collect  = 1;

	while (Collect) {

		if (Uart2Read(&Byte)) {
			if (Uart2RecMesIn == 0) { // Wait for data message start

				if (GET_MESSAGE_TYPE(Byte) == MESSAGE_DATA) {

					Uart2RecMesLng = GET_MESSAGE_LENGTH(Byte) + 2;

					if (Uart2RecMesLng <= UART_BUFFER_SIZE) { // Valid length

						Uart2RecMes[Uart2RecMesIn] = Byte;
						Uart2RecMesIn++;
					}
				}
			} else {
				Uart2RecMes[Uart2RecMesIn] = Byte;

				if (++Uart2RecMesIn >= Uart2RecMesLng) { // Message ready

					*pCmd   = Uart2RecMes[0];
					*pFail  ^=  *pCmd;

					while (Length < (Uart2RecMesLng - 2)) {
						pData[Length]  = Uart2RecMes[Length + 1];
						*pFail       ^=  pData[Length];
						Length++;
					}
					*pCheck = Uart2RecMes[Length + 1];
					*pFail  ^=  *pCheck;

					Uart2RecMesIn = 0;
					Collect       = 0;
				}
			}
		} else {
			Collect = 0;
		}
	}

	return Length;
}

static UBYTE Uart2Write(UBYTE Byte)
{
	UBYTE Result = 0;

	if (Uart2Base[UART_LSR] & 0x20) {
		Uart2Base[UART_THR]  = Byte;
		Result               = 1;
	}

	return Result;
}

static void Uart2Setup(ULONG BitRate)
{
	ULONG Divisor;

	Divisor = Device1Lms2012Compat->uart_clock_freq[Uart2] / (BitRate * (ULONG)16);

	Uart2Base[UART_LCR] = 0xBF;
	Uart2Base[UART_EFR] |= 0x10;    //Enables access to MCR

	Uart2Base[UART_LCR] = 0x80;
	Uart2Base[UART_MCR] |= 0x40;    //Enables access to TCR and TLR

	Uart2Base[UART_LCR] = 0xBF;
	Uart2Base[UART_SCR] |= 0x80;    //Enable granularity of 1 for trigger RX level
	Uart2Base[UART_SCR] &= ~(0x40); //Disable granularity of 1 for trigger TX level
	Uart2Base[UART_TLR] = 0x0;

	Uart2Base[UART_SCR] |= 0x1; //DMAMODE is set with SCR[2:1]
	Uart2Base[UART_SCR] &= ~(0x6);  //no DMA

	Uart2Base[UART_LCR] = 0x7F;
	Uart2Base[UART_IER] &= ~(0x10); //Disables sleep mode

	Uart2Base[UART_LCR] = 0xBF;

	Uart2Base[UART_MDR1] |= 0x7;    //Disable uart
	Uart2Base[UART_DLL] = 0x0;
	Uart2Base[UART_DLH] = 0x0;
	//Uart2Base[UART_MDR1] &= ~(0x7);   //uart 16x mode

	Uart2Base[UART_LCR] = 0x80;
	Uart2Base[UART_FCR] = (1 << 6) | 0x7;

	Uart2Base[UART_LCR] = 0xBF;

	//Uart2Base[UART_MDR1] |= 0x7;  //Disable uart
	Uart2Base[UART_DLL] = Divisor & 0xFF;
	Uart2Base[UART_DLH] = (Divisor & 0x3F00) >> 8;
	Uart2Base[UART_MDR1] &= ~(0x7); //uart 16x mode

	Uart2Base[UART_LCR] = 0x3;
	Uart2Base[UART_IER] = 0x1;
}

static int Uart2Init(void)
{
	int timeout = 10000;
	int ret;

	snprintf(Uart2Name, 20, "%s.port%d", DEVICE1_NAME, Uart2 + 1);
	Uart2Base = Device1Lms2012Compat->uart_mem[Uart2];

	Uart2Base[UART_SYSC] |= 0x2;
	while(!(Uart2Base[UART_SYSS] & 0x1)) {
		if (!timeout--)
			return -ETIMEDOUT;
		udelay(1);
	}

	ret = request_irq(Device1Lms2012Compat->uart_irq[Uart2], &Uart2Interrupt,
			  IRQF_SHARED, Uart2Name, &Uart2Name);

	return ret;
}

void Uart2Exit(void)
{
	Uart2Base[UART_IER] = 0x00;

	free_irq(Device1Lms2012Compat->uart_irq[Uart2], Uart2Name);
}

//Uart 3 **************************************************************************************

static volatile u32 *Uart3Base;
static char Uart3Name[20];

static UBYTE Uart3RecBuf[UART_RECBUF_SIZE];
static UWORD Uart3RecBufIn;
static UWORD Uart3RecBufOut;

static UBYTE Uart3RecMesLng;
static UBYTE Uart3RecMes[UART_BUFFER_SIZE];
static UBYTE Uart3RecMesIn;

irqreturn_t Uart3Interrupt(int irq, void *dev_id)
{
	UBYTE IntrType;

	IntrType = (UBYTE)Uart3Base[UART_IIR] & 0x0F;

	while (!(IntrType & 1)) {
		if (IntrType == 2) {
		} else {
			if (IntrType & 2) {
				Uart3RecBuf[Uart3RecBufIn] = (UBYTE)Uart3Base[UART_LSR];
			}
			Uart3RecBuf[Uart3RecBufIn] = (UBYTE)Uart3Base[UART_RHR];

			if (++Uart3RecBufIn >= UART_RECBUF_SIZE) {
				Uart3RecBufIn = 0;
			}
		}
		IntrType = (UBYTE)Uart3Base[UART_IIR] & 0x0F;
	}

	return IRQ_HANDLED;
}

static UBYTE Uart3Read(UBYTE *pByte)
{
	UBYTE Result = 0;

	if (Uart3RecBufIn != Uart3RecBufOut) {
		*pByte = Uart3RecBuf[Uart3RecBufOut];

		if (++Uart3RecBufOut >= UART_RECBUF_SIZE) {
			Uart3RecBufOut = 0;
		}
		Result   = 1;
	}

	return Result;
}

static void Uart3Flush(void)
{
	Uart3Base[UART_FCR]  = 0x07;
	Uart3RecBufIn        = 0;
	Uart3RecBufOut       = 0;
	Uart3RecMesIn        = 0;
}

static UBYTE Uart3ReadData(UBYTE *pCmd, UBYTE *pData, UBYTE *pCheck, UBYTE *pFail)
{
	UBYTE   Byte;
	UBYTE   Length;
	UBYTE   Collect;

	Length   = 0;
	*pFail   = 0xFF;
	Collect  = 1;

	while (Collect) {
		if (Uart3Read(&Byte)) {
			if (Uart3RecMesIn == 0) { // Wait for data message start

				if (GET_MESSAGE_TYPE(Byte) == MESSAGE_DATA) {

					Uart3RecMesLng = GET_MESSAGE_LENGTH(Byte) + 2;

					if (Uart3RecMesLng <= UART_BUFFER_SIZE) { // Valid length

						Uart3RecMes[Uart3RecMesIn] = Byte;
						Uart3RecMesIn++;
					}
				}
			} else {
				Uart3RecMes[Uart3RecMesIn] = Byte;

				if (++Uart3RecMesIn >= Uart3RecMesLng) { // Message ready

					*pCmd   = Uart3RecMes[0];
					*pFail  ^=  *pCmd;

					while (Length < (Uart3RecMesLng - 2)) {
						pData[Length]  = Uart3RecMes[Length + 1];
						*pFail       ^=  pData[Length];
						Length++;
					}
					*pCheck = Uart3RecMes[Length + 1];
					*pFail  ^=  *pCheck;

					Uart3RecMesIn = 0;
					Collect = 0;
				}
			}
		} else {
			Collect = 0;
		}
	}

	return Length;
}

static UBYTE Uart3Write(UBYTE Byte)
{
	UBYTE Result = 0;

	if (Uart3Base[UART_LSR] & 0x20) {
		Uart3Base[UART_THR]  = Byte;
		Result               = 1;
	}

	return Result;
}

static void Uart3Setup(ULONG BitRate)
{
	ULONG Divisor;

	Divisor = Device1Lms2012Compat->uart_clock_freq[Uart3] / (BitRate * (ULONG)16);

	Uart3Base[UART_LCR] = 0xBF;
	Uart3Base[UART_EFR] |= 0x10;    //Enables access to MCR

	Uart3Base[UART_LCR] = 0x80;
	Uart3Base[UART_MCR] |= 0x40;    //Enables access to TCR and TLR

	Uart3Base[UART_LCR] = 0xBF;
	Uart3Base[UART_SCR] |= 0x80;    //Enable granularity of 1 for trigger RX level
	Uart3Base[UART_SCR] &= ~(0x40); //Disable granularity of 1 for trigger TX level
	Uart3Base[UART_TLR] = 0x0;

	Uart3Base[UART_SCR] |= 0x1; //DMAMODE is set with SCR[2:1]
	Uart3Base[UART_SCR] &= ~(0x6);  //no DMA

	Uart3Base[UART_LCR] = 0x7F;
	Uart3Base[UART_IER] &= ~(0x10); //Disables sleep mode

	Uart3Base[UART_LCR] = 0xBF;

	Uart3Base[UART_MDR1] |= 0x7;    //Disable uart
	Uart3Base[UART_DLL] = 0x0;
	Uart3Base[UART_DLH] = 0x0;
	//Uart3Base[UART_MDR1] &= ~(0x7);   //uart 16x mode

	Uart3Base[UART_LCR] = 0x80;
	Uart3Base[UART_FCR] = (1 << 6) | 0x7;

	Uart3Base[UART_LCR] = 0xBF;

	//Uart3Base[UART_MDR1] |= 0x7;  //Disable uart
	Uart3Base[UART_DLL] = Divisor & 0xFF;
	Uart3Base[UART_DLH] = (Divisor & 0x3F00) >> 8;
	Uart3Base[UART_MDR1] &= ~(0x7); //uart 16x mode

	Uart3Base[UART_LCR] = 0x3;
	Uart3Base[UART_IER] = 0x1;
}

static int Uart3Init(void)
{
	int timeout = 10000;
	int ret;

	snprintf(Uart3Name, 20, "%s.port%d", DEVICE1_NAME, Uart3 + 1);
	Uart3Base = Device1Lms2012Compat->uart_mem[Uart3];

	Uart3Base[UART_SYSC] |= 0x2;
	while(!(Uart3Base[UART_SYSS] & 0x1)) {
		if (!timeout--)
			return -ETIMEDOUT;
		udelay(1);
	}

	ret = request_irq(Device1Lms2012Compat->uart_irq[Uart3], &Uart3Interrupt,
			  IRQF_SHARED, Uart3Name, &Uart3Name);

	return ret;
}

void Uart3Exit(void)
{
	Uart3Base[UART_IER] = 0x00;

	free_irq(Device1Lms2012Compat->uart_irq[Uart3], Uart3Name);
}

// UART 4 *********************************************************************

static volatile u32 *Uart4Base;
static char Uart4Name[20];

static UBYTE Uart4RecBuf[UART_RECBUF_SIZE];
static UWORD Uart4RecBufIn;
static UWORD Uart4RecBufOut;

static UBYTE Uart4RecMesLng;
static UBYTE Uart4RecMes[UART_BUFFER_SIZE];
static UBYTE Uart4RecMesIn;

irqreturn_t Uart4Interrupt(int irq, void *dev_id)
{
	UBYTE IntrType;

	IntrType = (UBYTE)Uart4Base[UART_IIR] & 0x0F;

	while (!(IntrType & 1)) {
		if (IntrType == 2) {
		} else {
			if (IntrType & 2) {
				Uart4RecBuf[Uart4RecBufIn] = (UBYTE)Uart4Base[UART_LSR];
			}
			Uart4RecBuf[Uart4RecBufIn] = (UBYTE)Uart4Base[UART_RHR];

			if (++Uart4RecBufIn >= UART_RECBUF_SIZE) {
				Uart4RecBufIn = 0;
			}
		}
		IntrType = (UBYTE)Uart4Base[UART_IIR] & 0x0F;
	}

	return IRQ_HANDLED;

}

static UBYTE Uart4Read(UBYTE *pByte)
{
	UBYTE Result = 0;

	if (Uart4RecBufIn != Uart4RecBufOut) {
		*pByte = Uart4RecBuf[Uart4RecBufOut];

		if (++Uart4RecBufOut >= UART_RECBUF_SIZE) {
			Uart4RecBufOut = 0;
		}
		Result   = 1;
	}

	return Result;
}

static void Uart4Flush(void)
{
	Uart4Base[UART_FCR]  = 0x07;
	Uart4RecBufIn        = 0;
	Uart4RecBufOut       = 0;
	Uart4RecMesIn        = 0;
}

static UBYTE Uart4ReadData(UBYTE *pCmd, UBYTE *pData, UBYTE *pCheck, UBYTE *pFail)
{
	UBYTE   Byte;
	UBYTE   Length;
	UBYTE   Collect;

	Length   = 0;
	*pFail   = 0xFF;
	Collect  = 1;

	while (Collect) {

		if (Uart4Read(&Byte)) {
			if (Uart4RecMesIn == 0) { // Wait for data message start

				if (GET_MESSAGE_TYPE(Byte) == MESSAGE_DATA) {

					Uart4RecMesLng = GET_MESSAGE_LENGTH(Byte) + 2;

					if (Uart4RecMesLng <= UART_BUFFER_SIZE) { // Valid length

						Uart4RecMes[Uart4RecMesIn] = Byte;
						Uart4RecMesIn++;
					}
				}
			} else {
				Uart4RecMes[Uart4RecMesIn] = Byte;

				if (++Uart4RecMesIn >= Uart4RecMesLng) { // Message ready

					*pCmd   = Uart4RecMes[0];
					*pFail  ^=  *pCmd;

					while (Length < (Uart4RecMesLng - 2)) {
						pData[Length]  = Uart4RecMes[Length + 1];
						*pFail       ^=  pData[Length];
						Length++;
					}
					*pCheck = Uart4RecMes[Length + 1];
					*pFail  ^=  *pCheck;

					Uart4RecMesIn = 0;
					Collect       = 0;
				}
			}
		} else {
			Collect = 0;
		}
	}

	return Length;
}

static UBYTE Uart4Write(UBYTE Byte)
{
	UBYTE   Result = 0;

	if (Uart4Base[UART_LSR] & 0x20) {
		Uart4Base[UART_THR]  = Byte;
		Result               = 1;
	}

	return Result;
}

static void Uart4Setup(ULONG BitRate)
{
	ULONG Divisor;

	Divisor = Device1Lms2012Compat->uart_clock_freq[Uart4] / (BitRate * (ULONG)16);

	Uart4Base[UART_LCR] = 0xBF;
	Uart4Base[UART_EFR] |= 0x10;    //Enables access to MCR

	Uart4Base[UART_LCR] = 0x80;
	Uart4Base[UART_MCR] |= 0x40;    //Enables access to TCR and TLR

	Uart4Base[UART_LCR] = 0xBF;
	Uart4Base[UART_SCR] |= 0x80;    //Enable granularity of 1 for trigger RX level
	Uart4Base[UART_SCR] &= ~(0x40); //Disable granularity of 1 for trigger TX level
	Uart4Base[UART_TLR] = 0x0;

	Uart4Base[UART_SCR] |= 0x1; //DMAMODE is set with SCR[2:1]
	Uart4Base[UART_SCR] &= ~(0x6);  //no DMA

	Uart4Base[UART_LCR] = 0x7F;
	Uart4Base[UART_IER] &= ~(0x10); //Disables sleep mode

	Uart4Base[UART_LCR] = 0xBF;

	Uart4Base[UART_MDR1] |= 0x7;    //Disable uart
	Uart4Base[UART_DLL] = 0x0;
	Uart4Base[UART_DLH] = 0x0;
	//Uart4Base[UART_MDR1] &= ~(0x7);   //uart 16x mode

	Uart4Base[UART_LCR] = 0x80;
	Uart4Base[UART_FCR] = (1 << 6) | 0x7;

	Uart4Base[UART_LCR] = 0xBF;

	//Uart4Base[UART_MDR1] |= 0x7;  //Disable uart
	Uart4Base[UART_DLL] = Divisor & 0xFF;
	Uart4Base[UART_DLH] = (Divisor & 0x3F00) >> 8;
	Uart4Base[UART_MDR1] &= ~(0x7); //uart 16x mode

	Uart4Base[UART_LCR] = 0x3;
	Uart4Base[UART_IER] = 0x1;

}

static int Uart4Init(void)
{
	int timeout = 10000;
	int ret;

	snprintf(Uart4Name, 20, "%s.port%d", DEVICE1_NAME, Uart4 + 1);
	Uart4Base = Device1Lms2012Compat->uart_mem[Uart4];

	Uart4Base[UART_SYSC] |= 0x2;
	while(!(Uart4Base[UART_SYSS] & 0x1)) {
		if (!timeout--)
			return -ETIMEDOUT;
		udelay(1);
	}

	ret = request_irq(Device1Lms2012Compat->uart_irq[Uart4], &Uart4Interrupt,
			  IRQF_SHARED, Uart4Name, &Uart4Name);

	return ret;
}

void Uart4Exit(void)
{
	Uart4Base[UART_IER] = 0x00;

	free_irq(Device1Lms2012Compat->uart_irq[Uart4], Uart4Name);
}

// DEVICE1 ********************************************************************

#define INFODATA_INIT		0x00000000L
#define INFODATA_CMD_TYPE	0x00000001L
#define INFODATA_CMD_MODES	0x00000002L
#define INFODATA_CMD_SPEED	0x00000004L

#define INFODATA_INFO_NAME	0x00000100L
#define INFODATA_INFO_RAW	0x00000200L
#define INFODATA_INFO_PCT	0x00000400L
#define INFODATA_INFO_SI	0x00000800L
#define INFODATA_INFO_SYMBOL	0x00001000L
#define INFODATA_INFO_FORMAT	0x00002000L

#define INFODATA_CLEAR		(~(INFODATA_INFO_NAME | INFODATA_INFO_RAW | INFODATA_INFO_PCT | INFODATA_INFO_SI | INFODATA_INFO_SYMBOL | INFODATA_INFO_FORMAT))

#define INFODATA_NEEDED		(INFODATA_CMD_TYPE | INFODATA_CMD_MODES | INFODATA_INFO_NAME | INFODATA_INFO_FORMAT)

enum UART_STATE {
	UART_IDLE,
	UART_INIT,
	UART_RESTART,
	UART_ENABLE,
	UART_FLUSH,
	UART_SYNC,
	UART_MESSAGE_START,
	UART_ESCAPE,
	UART_CMD,
	UART_INFO,
	UART_DATA,
	UART_DATA_COPY,
	UART_ACK_WAIT,
	UART_ACK_INFO,
	UART_CMD_ERROR,
	UART_INFO_ERROR,
	UART_TERMINAL,
	UART_DATA_ERROR,
	UART_ERROR,
	UART_EXIT,
	UART_STATES
};

static const char UartStateText[UART_STATES][50] = {
	"IDLE",
	"INIT",
	"UART_RESTART",
	"ENABLE",
	"FLUSH",
	"SYNC",
	"MESSAGE_START",
	"ESCAPE",
	"CMD",
	"INFO",
	"DATA",
	"DATA_COPY",
	"ACK_WAIT",
	"ACK_INFO",
	"CMD_ERROR",
	"INFO_ERROR",
	"TERMINAL",
	"DATA_ERROR",
	"ERROR",
	"EXIT"
};

UARTPORT UartPortDefault = {
	.InfoData	= INFODATA_INIT,
	.BitRate	= (ULONG)LOWEST_BITRATE,
	.BitRateMax	= (ULONG)LOWEST_BITRATE,
	.Timer		= 0,
	.WatchDog	= 0,
	.BreakTimer	= 0,
	.Initialised	= 0,
	.ChangeMode	= 0,
	.State		= UART_IDLE,
	.OldState	= -1,
	.SubState	= 0,
	.Cmd		= 0,
	.InfoCmd	= 0,
	.Check		= 0,
	.Types		= 0,
	.Views		= 0,
	.Mode		= 0,
	.Type		= TYPE_UNKNOWN,
	.DataOk		= 0,
	.DataErrors	= 0,
	.Name		= "",
	.InLength	= 0,
	.InPointer	= 0,
	.OutLength	= 0,
	.OutPointer	= 0,
};

static TYPES TypeData[INPUTS][MAX_DEVICE_MODES]; //!< TypeData
static DATA8 Changed[INPUTS][MAX_DEVICE_MODES];

#define UART_TIMER_RESOLUTION		10                // [100uS]

#define UART_BREAK_TIME			1000              // [100uS]
#define UART_TERMINAL_DELAY		20000             // [100uS]
#define UART_CHANGE_BITRATE_DELAY	100               // [100uS]
#define UART_ACK_DELAY			100               // [100uS]
#define UART_SHOW_TIME			2500              // [100uS]

#define UART_WATCHDOG_TIME		1000              // [100uS]

#define UART_ALLOWABLE_DATA_ERRORS	6

static UBYTE UartConfigured[INPUTS];
//UARTPORT  UartPort[INPUTS];

static UART UartDefault;
static UART *pUart = &UartDefault;

static struct hrtimer Device1Timer;
static ktime_t        Device1Time;

static UBYTE TestMode = 0;

static void UartPortDisable(UBYTE Port)
{
	switch (Port) {
	case Uart1:
		break;

	case Uart2:
		break;

	case Uart3:
		break;

	case Uart4:
		break;
	}

	PUARTHigh(Port, INPUT_PORT_BUF);
}

static void UartPortFlush(UBYTE Port)
{
	switch (Port) {
	case Uart1:
		Uart1Flush();
		break;

	case Uart2:
		Uart2Flush();
		break;

	case Uart3:
		Uart3Flush();
		break;

	case Uart4:
		Uart4Flush();
		break;
	}
}

static void UartPortEnable(UBYTE Port)
{
	// TODO: well need to do pin mux when I2C is working
	// SetGpio(InputUartPin[Port][INPUT_UART_TXD].Pin);
	PUARTLow(Port, INPUT_PORT_BUF);

	switch (Port) {
	case Uart1:
		Uart1Flush();
		break;

	case Uart2:
		Uart2Flush();
		break;

	case Uart3:
		Uart3Flush();
		break;

	case Uart4:
		Uart4Flush();
		break;
	}
}

static UBYTE UartPortSend(UBYTE Port, UBYTE Byte)
{
	UBYTE Result = 1;

	switch (Port) {
	case Uart1:
		Result = Uart1Write(Byte);
		break;

	case Uart2:
		Result = Uart2Write(Byte);
		break;

	case Uart3:
		Result = Uart3Write(Byte);
		break;

	case Uart4:
		Result = Uart4Write(Byte);
		break;
	}

	return Result;
}

static UBYTE UartPortReceive(UBYTE Port, UBYTE *pByte)
{
	UBYTE Result = 0;

	switch (Port) {
	case Uart1:
		Result = Uart1Read(pByte);
		break;

	case Uart2:
		Result = Uart2Read(pByte);
		break;

	case Uart3:
		Result = Uart3Read(pByte);
		break;

	case Uart4:
		Result = Uart4Read(pByte);
		break;
	}

	return Result;
}

static UBYTE UartPortReadData(UBYTE Port, UBYTE *pCmd, UBYTE *pData,
			      UBYTE *pCheck, UBYTE *pFail)
{
	UBYTE Result = 0;

	switch (Port) {
	case Uart1:
		Result = Uart1ReadData(pCmd, pData, pCheck, pFail);
		break;

	case Uart2:
		Result = Uart2ReadData(pCmd, pData, pCheck, pFail);
		break;

	case Uart3:
		Result = Uart3ReadData(pCmd, pData, pCheck, pFail);
		break;

	case Uart4:
		Result = Uart4ReadData(pCmd, pData, pCheck, pFail);
		break;
	}

	return Result;
}

static void UartPortSetup(UBYTE Port, ULONG BitRate)
{
	switch (Port) {
	case Uart1:
		Uart1Setup(BitRate);
		break;

	case Uart2:
		Uart2Setup(BitRate);
		break;

	case Uart3:
		Uart3Setup(BitRate);
		break;

	case Uart4:
		Uart4Setup(BitRate);
		break;
	}
}

static UBYTE WriteRequest[INPUTS];

static enum hrtimer_restart Device1TimerInterrupt1(struct hrtimer *pTimer)
{
  UBYTE   Port;
  UBYTE   Byte;
  UBYTE   CrcError = 0;
  UBYTE   Tmp = 0;
  ULONG   TmpL;
  UBYTE   Chksum;
  UBYTE   Pointer;
  UBYTE   Mode;
  UBYTE   TmpBuffer[UART_DATA_LENGTH];

  hrtimer_forward_now(pTimer, Device1Time);

  for (Port = 0; Port < INPUTS; Port++) {
    // look at one port at a time

    if ((UartPort[Port].State > UART_ENABLE) && (!TestMode)) { // If port active

      if (++UartPort[Port].BreakTimer >= (UART_BREAK_TIME / UART_TIMER_RESOLUTION)) { // Reset state machine if break received

        if (PUARTRead(Port, INPUT_PORT_PIN6)) {

#ifdef DEBUG_D_UART_ERROR
          snprintf(UartBuffer, UARTBUFFERSIZE, "    %d BREAK\n", Port);
          UartWrite(UartBuffer);
#endif

          UartPortDisable(Port);
          UartPort[Port]                         = UartPortDefault;
          UartPortEnable(Port);
          UartPortSetup(Port, UartPort[Port].BitRate);
          for (Tmp = 0;Tmp < MAX_DEVICE_MODES;Tmp++) {
            TypeData[Port][Tmp]          = TypeDefaultUart[0];
            Changed[Port][Tmp]           = 0;
          }
#ifndef DISABLE_FAST_DATALOG_BUFFER
          pUart->Actual[Port]          = 0;
          pUart->LogIn[Port]           = 0;
#endif
          pUart->Status[Port]          = 0;
          UartPortFlush(Port);
          UartPort[Port].State         = UART_SYNC;
        }
      }
      if (PUARTRead(Port, INPUT_PORT_PIN6)) {
        UartPort[Port].BreakTimer = 0;
      }
    }

    if (Port != DEBUG_UART) { // If port not used for debug

#ifdef DEBUG
      ShowTimer[Port]++;
#endif

      if (!TestMode) {

        switch (UartPort[Port].State) { // Main state machine

        case UART_IDLE:
          // Port inactive

          pUart->Status[Port] &= ~UART_WRITE_REQUEST;
          pUart->Status[Port] &= ~UART_DATA_READY;
          WriteRequest[Port]    = 0;
          break;

        case UART_INIT:
          // Initialise port hardware

          PUARTFloat(Port, INPUT_PORT_PIN5);
          PUARTFloat(Port, INPUT_PORT_PIN6);
          UartPort[Port].State       = UART_ENABLE;
          break;

        case UART_RESTART:
          UartPortDisable(Port);
          UartPort[Port].State       = UART_ENABLE;
          break;

        case UART_ENABLE:
          // Initialise port variables

          UartPort[Port]                         = UartPortDefault;
          UartPortEnable(Port);
          UartPortSetup(Port, UartPort[Port].BitRate);
          for (Tmp = 0;Tmp < MAX_DEVICE_MODES;Tmp++) {
            TypeData[Port][Tmp]          = TypeDefaultUart[0];
            Changed[Port][Tmp]           = 0;
          }
#ifndef DISABLE_FAST_DATALOG_BUFFER
          pUart->Actual[Port]          = 0;
          pUart->LogIn[Port]           = 0;
#endif
          pUart->Status[Port]          = 0;
          UartPortFlush(Port);
          UartPort[Port].State           = UART_SYNC;
          break;

        case UART_SYNC:
          // Look for UART_CMD, TYPE in rolling buffer window

          if (UartPortReceive(Port, &Byte)) {
            if (UartPort[Port].InPointer < 3) { // 0, 1, 2
              UartPort[Port].InBuffer[UartPort[Port].InPointer] = Byte;
              UartPort[Port].InPointer++;
            }
            if (UartPort[Port].InPointer >= 3) {
              // Validate
              UartPort[Port].Check     = 0xFF;
              for (Tmp = 0;Tmp < 2;Tmp++) {
                UartPort[Port].Check   ^=  UartPort[Port].InBuffer[Tmp];
              }
              if ((UartPort[Port].Check == UartPort[Port].InBuffer[2]) && (UartPort[Port].InBuffer[0] == 0x40) && (UartPort[Port].InBuffer[1] > 0) && (UartPort[Port].InBuffer[1] <= MAX_VALID_TYPE)) {
                UartPort[Port].Type      = UartPort[Port].InBuffer[1];
                UartPort[Port].InfoData |= INFODATA_CMD_TYPE;
#ifdef HIGHDEBUG
                snprintf(UartBuffer, UARTBUFFERSIZE, "    %d TYPE   = %-3u\n", Port, (UWORD)UartPort[Port].Type & 0xFF);
                UartWrite(UartBuffer);
#endif
                UartPort[Port].State     = UART_MESSAGE_START;
              } else {
#ifdef DEBUG_D_UART_ERROR
//                snprintf(UartBuffer, UARTBUFFERSIZE, "[%02X]", Byte);
//                UartWrite(UartBuffer);
//                snprintf(UartBuffer, UARTBUFFERSIZE, "    %d No sync %02X %02X %02X\n", Port, UartPort[Port].InBuffer[0], UartPort[Port].InBuffer[1], UartPort[Port].InBuffer[2]);
//                UartWrite(UartBuffer);
#endif
                for (Tmp = 0;Tmp < 2;Tmp++) {
                  UartPort[Port].InBuffer[Tmp] = UartPort[Port].InBuffer[Tmp + 1];
                }

                UartPort[Port].InPointer--;
              }
            }
          }
          if ((++(UartPort[Port].Timer) >= (UART_TERMINAL_DELAY / UART_TIMER_RESOLUTION))) {
            UartPort[Port].BitRate     = 115200;
            UartPortSetup(Port, UartPort[Port].BitRate);
            TypeData[Port][0]          = TypeDefaultUart[1];
            UartPort[Port].State       = UART_TERMINAL;
            Changed[Port][0]           = 1;
            pUart->Status[Port]     |= UART_PORT_CHANGED;
          }
          break;

        default:
          // Get sensor informations

          if (UartPortReceive(Port, &Byte)) {

            switch (UartPort[Port].State) {

//** INTERPRETER **************************************************************

            case UART_MESSAGE_START:
              UartPort[Port].InPointer = 0;
              UartPort[Port].SubState  = 0;
              UartPort[Port].Check     = 0xFF;
              UartPort[Port].Cmd       = Byte;

              switch (GET_MESSAGE_TYPE(Byte)) {
              case MESSAGE_CMD:
                UartPort[Port].InLength  = GET_MESSAGE_LENGTH(Byte);
                UartPort[Port].State     = UART_CMD;
                break;

              case MESSAGE_INFO:
                UartPort[Port].InLength  = GET_MESSAGE_LENGTH(Byte);
                UartPort[Port].State     = UART_INFO;
                break;

              case MESSAGE_DATA:
                break;

              default:
                switch (Byte) {
                case BYTE_ACK:
#ifdef HIGHDEBUG
                  snprintf(UartBuffer, UARTBUFFERSIZE, "    %d ACK RECEIVED\n", Port);
                  UartWrite(UartBuffer);
#endif
                  if (UartPort[Port].Types == 0) {
                    if ((UartPort[Port].InfoData & INFODATA_NEEDED) == INFODATA_NEEDED) {
                      UartPort[Port].Timer   = 0;
                      UartPort[Port].State   = UART_ACK_WAIT;
                    } else {
                      UartPort[Port].State = UART_INFO_ERROR;
                    }
                  } else {
                    UartPort[Port].State = UART_INFO_ERROR;
                  }
                  break;

                case BYTE_NACK:
                  break;

                case BYTE_SYNC:
                  break;

                default:
                  UartPort[Port].InLength  = GET_MESSAGE_LENGTH(Byte);
                  UartPort[Port].State     = UART_ESCAPE;
                  break;
                }
                break;

              }
              break;

            case UART_ESCAPE:
              if (UartPort[Port].InPointer < UartPort[Port].InLength) {
                UartPort[Port].InBuffer[UartPort[Port].InPointer] = Byte;
                UartPort[Port].InPointer++;
              } else { // Message complete

                UartPort[Port].InBuffer[UartPort[Port].InPointer] = 0;
                UartPort[Port].State = UART_MESSAGE_START;
              }
              break;

//** CMD **********************************************************************

            case UART_CMD:
              // Command message in progress

              if (UartPort[Port].InPointer < UartPort[Port].InLength) {
                UartPort[Port].InBuffer[UartPort[Port].InPointer] = Byte;
                UartPort[Port].InPointer++;
              } else { // Message complete

                UartPort[Port].InBuffer[UartPort[Port].InPointer] = 0;
                UartPort[Port].State = UART_MESSAGE_START;

                if (UartPort[Port].Check !=  Byte) { // Check not correct
#ifdef DEBUG_D_UART_ERROR
                  snprintf(UartBuffer, UARTBUFFERSIZE, " c  %d %02X[", Port, UartPort[Port].Cmd & 0xFF);
                  UartWrite(UartBuffer);
                  for (Tmp = 0;Tmp < UartPort[Port].InLength;Tmp++) {
                    snprintf(UartBuffer, UARTBUFFERSIZE, "%02X", UartPort[Port].InBuffer[Tmp] & 0xFF);
                    UartWrite(UartBuffer);
                  }
                  snprintf(UartBuffer, UARTBUFFERSIZE, "]%02X\n", UartPort[Port].Check & 0xFF);
                  UartWrite(UartBuffer);
#endif
                  UartPort[Port].State = UART_CMD_ERROR;
                } else { // Command message valid
#ifdef HIGHDEBUG
                  snprintf(UartBuffer, UARTBUFFERSIZE, "    %d %02X[", Port, UartPort[Port].Cmd & 0xFF);
                  UartWrite(UartBuffer);
                  for (Tmp = 0;Tmp < UartPort[Port].InLength;Tmp++) {
                    snprintf(UartBuffer, UARTBUFFERSIZE, "%02X", UartPort[Port].InBuffer[Tmp] & 0xFF);
                    UartWrite(UartBuffer);
                  }
                  snprintf(UartBuffer, UARTBUFFERSIZE, "]%02X\n", UartPort[Port].Check & 0xFF);
                  UartWrite(UartBuffer);
#endif
                  switch (GET_CMD_COMMAND(UartPort[Port].Cmd)) { // Command message type

                    case CMD_MODES:
                    { // Number of modes

                      if ((UartPort[Port].InBuffer[0] >= 0) && (UartPort[Port].InBuffer[0] < MAX_DEVICE_MODES)) { // Number of modes valid

                        if ((UartPort[Port].InfoData & INFODATA_CMD_MODES)) { // Modes already given

#ifdef DEBUG_D_UART_ERROR
                          snprintf(UartBuffer, UARTBUFFERSIZE, " ## %d MODES ALREADY GIVEN\n", Port);
                          UartWrite(UartBuffer);
#endif
                          UartPort[Port].State = UART_CMD_ERROR;
                        } else {

                          UartPort[Port].Types = UartPort[Port].InBuffer[0] + 1;
                          if (UartPort[Port].InLength >= 2) { // Both modes and views present

                            UartPort[Port].Views = UartPort[Port].InBuffer[1] + 1;
#ifdef HIGHDEBUG
                            snprintf(UartBuffer, UARTBUFFERSIZE, "    %d MODES  = %u  VIEWS  = %u\n", Port, (UWORD)UartPort[Port].Types & 0xFF, (UWORD)UartPort[Port].Views & 0xFF);
                            UartWrite(UartBuffer);
#endif
                          } else { // Only modes present

                            UartPort[Port].Views = UartPort[Port].Types;
#ifdef HIGHDEBUG
                            snprintf(UartBuffer, UARTBUFFERSIZE, "    %d MODES  = %u = VIEWS\n", Port, (UWORD)UartPort[Port].Types & 0xFF);
                            UartWrite(UartBuffer);
#endif
                          }
                          UartPort[Port].InfoData |=  INFODATA_CMD_MODES;
                        }
                      } else { // Number of modes invalid
#ifdef DEBUG_D_UART_ERROR
                        snprintf(UartBuffer, UARTBUFFERSIZE, " ## %d MODES ERROR  %u\n", Port, (UWORD)UartPort[Port].Types & 0xFF);
                        UartWrite(UartBuffer);
#endif
                        UartPort[Port].State = UART_CMD_ERROR;
                      }
                    }
                    break;

                    case CMD_SPEED:
                    { // Highest communication speed

                      TmpL = 0;
                      for (Tmp = 0;Tmp < UartPort[Port].InLength;Tmp++) {
                        TmpL |=  (ULONG)UartPort[Port].InBuffer[Tmp] << (8 * Tmp);
                      }

                      if ((TmpL >= LOWEST_BITRATE) && (TmpL <= HIGHEST_BITRATE)) { // Speed valid

                        if ((UartPort[Port].InfoData & INFODATA_CMD_SPEED)) { // Speed already given

#ifdef DEBUG_D_UART_ERROR
                          snprintf(UartBuffer, UARTBUFFERSIZE, " ## %d SPEED ALREADY GIVEN\n", Port);
                          UartWrite(UartBuffer);
#endif
                          UartPort[Port].State = UART_CMD_ERROR;
                        } else {
                          if ((UartPort[Port].BitRate != LOWEST_BITRATE) && (TmpL <= MIDLE_BITRATE)) { // allow bit rate adjust
#ifdef HIGHDEBUG
                            snprintf(UartBuffer, UARTBUFFERSIZE, "    %d SPEED ADJUST\n", Port);
                            UartWrite(UartBuffer);
#endif
                            UartPort[Port].BitRateMax = (UartPort[Port].BitRate * TmpL) / LOWEST_BITRATE;
                          } else {

                            UartPort[Port].BitRateMax = TmpL;
                          }
#ifdef HIGHDEBUG
                          snprintf(UartBuffer, UARTBUFFERSIZE, "    %d SPEED  = %lu\n", Port, (unsigned long)UartPort[Port].BitRateMax);
                          UartWrite(UartBuffer);
#endif
                          UartPort[Port].InfoData |=  INFODATA_CMD_SPEED;
                        }
                      } else { // Speed invalid
#ifdef DEBUG_D_UART_ERROR
                        snprintf(UartBuffer, UARTBUFFERSIZE, " ## %d SPEED ERROR\n", Port);
                        UartWrite(UartBuffer);
#endif
                        UartPort[Port].State = UART_CMD_ERROR;
                      }
                    }
                    break;

                  }
                }
              }
              break;

//** INFO *********************************************************************

              case UART_INFO:
              { // Info messages in progress

                switch (UartPort[Port].SubState) {
                  case 0:
                  {
                    UartPort[Port].InfoCmd     = Byte;

                    // validate length

                    switch (GET_INFO_COMMAND(UartPort[Port].InfoCmd)) {

                      case INFO_FORMAT:
                      {
                        if (UartPort[Port].InLength < 2) {
#ifdef DEBUG_D_UART_ERROR
                          snprintf(UartBuffer, UARTBUFFERSIZE, " ## %d FORMAT ERROR\n", Port);
                          UartWrite(UartBuffer);
#endif
                          UartPort[Port].State = UART_INFO_ERROR;
                        }
                      }
                      break;

                    }
                    UartPort[Port].SubState++;
                  }
                  break;

                  default:
                  {
                    if (UartPort[Port].InPointer < UartPort[Port].InLength) { // Info message in progress

                      UartPort[Port].InBuffer[UartPort[Port].InPointer] = Byte;
                      UartPort[Port].InPointer++;
                    } else { // Message complete

                      UartPort[Port].InBuffer[UartPort[Port].InPointer] = 0;
                      UartPort[Port].State = UART_MESSAGE_START;

                      if (UartPort[Port].Check !=  Byte) {
#ifdef DEBUG_D_UART_ERROR
                        snprintf(UartBuffer, UARTBUFFERSIZE, " c  %d %02X ", Port, UartPort[Port].Cmd & 0xFF);
                        UartWrite(UartBuffer);
                        snprintf(UartBuffer, UARTBUFFERSIZE, "%02X[", UartPort[Port].InfoCmd & 0xFF);
                        UartWrite(UartBuffer);
                        for (Tmp = 0;Tmp < UartPort[Port].InLength;Tmp++) {
                          snprintf(UartBuffer, UARTBUFFERSIZE, "%02X", UartPort[Port].InBuffer[Tmp] & 0xFF);
                          UartWrite(UartBuffer);
                        }
                        snprintf(UartBuffer, UARTBUFFERSIZE, "]%02X\n", UartPort[Port].Check & 0xFF);
                        UartWrite(UartBuffer);
#endif
                        UartPort[Port].State = UART_INFO_ERROR;
                      } else {
#ifdef HIGHDEBUG
                        snprintf(UartBuffer, UARTBUFFERSIZE, "    %d %02X ", Port, UartPort[Port].Cmd & 0xFF);
                        UartWrite(UartBuffer);
                        snprintf(UartBuffer, UARTBUFFERSIZE, "%02X[", UartPort[Port].InfoCmd & 0xFF);
                        UartWrite(UartBuffer);
                        for (Tmp = 0;Tmp < UartPort[Port].InLength;Tmp++) {
                          snprintf(UartBuffer, UARTBUFFERSIZE, "%02X", UartPort[Port].InBuffer[Tmp] & 0xFF);
                          UartWrite(UartBuffer);
                        }
                        snprintf(UartBuffer, UARTBUFFERSIZE, "]%02X\n", UartPort[Port].Check & 0xFF);
                        UartWrite(UartBuffer);
#endif

                        Mode = GET_MODE(UartPort[Port].Cmd);

                        switch (GET_INFO_COMMAND(UartPort[Port].InfoCmd)) { // Info mesage type

                          case INFO_NAME:
                          { // Device name

                            UartPort[Port].InfoData &=  INFODATA_CLEAR;
                            if ((UartPort[Port].InBuffer[0] >= 'A') && (UartPort[Port].InBuffer[0] <= 'z') && (strlen(UartPort[Port].InBuffer) <= TYPE_NAME_LENGTH)) {
                              snprintf((char*)UartPort[Port].Name, TYPE_NAME_LENGTH + 1, "%s", (char*)UartPort[Port].InBuffer);
#ifdef HIGHDEBUG
                              snprintf(UartBuffer, UARTBUFFERSIZE, "    %d NAME   = %s\n", Port, UartPort[Port].Name);
                              UartWrite(UartBuffer);
#endif
                              TypeData[Port][Mode].Mode  = Mode;
                              UartPort[Port].InfoData   |= INFODATA_INFO_NAME;
                            } else {
#ifdef DEBUG_D_UART_ERROR
                              UartPort[Port].InBuffer[TYPE_NAME_LENGTH] = 0;
                              snprintf(UartBuffer, UARTBUFFERSIZE, " f  %d NAME = %s\n", Port, UartPort[Port].InBuffer);
                              UartWrite(UartBuffer);
#endif
                              UartPort[Port].State = UART_INFO_ERROR;
                            }
                          }
                          break;

                          case INFO_RAW:
                          { // Raw scaling values

                            TmpL = 0;
                            for (Tmp = 0;(Tmp < UartPort[Port].InLength) && (Tmp < 4);Tmp++) {
                              TmpL |=  (ULONG)UartPort[Port].InBuffer[Tmp] << (8 * Tmp);
                            }
                            *((ULONG*)&TypeData[Port][Mode].RawMin) = TmpL;
                            TmpL = 0;
                            for (Tmp = 0;(Tmp < (UartPort[Port].InLength - 4)) && (Tmp < 4);Tmp++) {
                              TmpL |=  (ULONG)UartPort[Port].InBuffer[Tmp + 4] << (8 * Tmp);
                            }
                            *((ULONG*)&TypeData[Port][Mode].RawMax) = TmpL;

                            if (TypeData[Port][Mode].Mode == GET_MODE(UartPort[Port].Cmd)) {
                              if ((UartPort[Port].InfoData & INFODATA_INFO_RAW)) { // Raw already given

#ifdef DEBUG_D_UART_ERROR
                                snprintf(UartBuffer, UARTBUFFERSIZE, " ## %d RAW ALREADY GIVEN\n", Port);
                                UartWrite(UartBuffer);
#endif
                                UartPort[Port].State = UART_INFO_ERROR;
                              } else {
#ifdef HIGHDEBUG
                                snprintf(UartBuffer, UARTBUFFERSIZE, "    %d RAW = Min..Max\n", Port);
                                UartWrite(UartBuffer);
#endif
                                UartPort[Port].InfoData |=  INFODATA_INFO_RAW;
                              }
                            } else {
#ifdef DEBUG_D_UART_ERROR
                              snprintf(UartBuffer, UARTBUFFERSIZE, " f  %d RAW = Min..Max\n", Port);
                              UartWrite(UartBuffer);
#endif
                              UartPort[Port].State = UART_INFO_ERROR;
                            }
                          }
                          break;

                          case INFO_PCT:
                          { // Pct scaling values

                            TmpL = 0;
                            for (Tmp = 0;(Tmp < UartPort[Port].InLength) && (Tmp < 4);Tmp++) {
                              TmpL |=  (ULONG)UartPort[Port].InBuffer[Tmp] << (8 * Tmp);
                            }
                            *((ULONG*)&TypeData[Port][Mode].PctMin) = TmpL;
                            TmpL = 0;
                            for (Tmp = 0;(Tmp < (UartPort[Port].InLength - 4)) && (Tmp < 4);Tmp++) {
                              TmpL |=  (ULONG)UartPort[Port].InBuffer[Tmp + 4] << (8 * Tmp);
                            }
                            *((ULONG*)&TypeData[Port][Mode].PctMax) = TmpL;

                            if (TypeData[Port][Mode].Mode == GET_MODE(UartPort[Port].Cmd)) { // Mode valid

                              if ((UartPort[Port].InfoData & INFODATA_INFO_PCT)) { // Pct already given

#ifdef DEBUG_D_UART_ERROR
                                snprintf(UartBuffer, UARTBUFFERSIZE, " ## %d PCT ALREADY GIVEN\n", Port);
                                UartWrite(UartBuffer);
#endif
                                UartPort[Port].State = UART_INFO_ERROR;
                              } else {
#ifdef HIGHDEBUG
                                snprintf(UartBuffer, UARTBUFFERSIZE, "    %d PCT = Min..Max\n", Port);
                                UartWrite(UartBuffer);
#endif
                                UartPort[Port].InfoData |=  INFODATA_INFO_PCT;
                              }
                            } else { // Mode invalid
#ifdef DEBUG_D_UART_ERROR
                              snprintf(UartBuffer, UARTBUFFERSIZE, " f  %d PCT = Min..Max\n", Port);
                              UartWrite(UartBuffer);
#endif
                              UartPort[Port].State = UART_INFO_ERROR;
                            }
                          }
                          break;

                          case INFO_SI:
                          { // SI unit scaling values

                            TmpL = 0;
                            for (Tmp = 0;(Tmp < UartPort[Port].InLength) && (Tmp < 4);Tmp++) {
                              TmpL |=  (ULONG)UartPort[Port].InBuffer[Tmp] << (8 * Tmp);
                            }
                            *((ULONG*)&TypeData[Port][Mode].SiMin) = TmpL;
                            TmpL = 0;
                            for (Tmp = 0;(Tmp < (UartPort[Port].InLength - 4)) && (Tmp < 4);Tmp++) {
                              TmpL |=  (ULONG)UartPort[Port].InBuffer[Tmp + 4] << (8 * Tmp);
                            }
                            *((ULONG*)&TypeData[Port][Mode].SiMax) = TmpL;

                            if (TypeData[Port][Mode].Mode == GET_MODE(UartPort[Port].Cmd)) { // Mode valid

                              if ((UartPort[Port].InfoData & INFODATA_INFO_SI)) { // Si already given

#ifdef DEBUG_D_UART_ERROR
                                snprintf(UartBuffer, UARTBUFFERSIZE, " ## %d SI ALREADY GIVEN\n", Port);
                                UartWrite(UartBuffer);
#endif
                                UartPort[Port].State = UART_INFO_ERROR;
                              } else {
#ifdef HIGHDEBUG
                                snprintf(UartBuffer, UARTBUFFERSIZE, "    %d SI  = Min..Max\n", Port);
                                UartWrite(UartBuffer);
#endif
                                UartPort[Port].InfoData |=  INFODATA_INFO_SI;
                              }
                            } else { // Mode invalid
#ifdef DEBUG_D_UART_ERROR
                              snprintf(UartBuffer, UARTBUFFERSIZE, " f  %d SI  = Min..Max\n", Port);
                              UartWrite(UartBuffer);
#endif
                              UartPort[Port].State = UART_INFO_ERROR;
                            }
                          }
                          break;

                          case INFO_SYMBOL:
                          { // Presentation format

                            if ((UartPort[Port].InfoData & INFODATA_INFO_SYMBOL)) { // Symbol already given

#ifdef DEBUG_D_UART_ERROR
                              snprintf(UartBuffer, UARTBUFFERSIZE, " ## %d SYMBOL ALREADY GIVEN\n", Port);
                              UartWrite(UartBuffer);
#endif
                              UartPort[Port].State = UART_INFO_ERROR;
                            } else {
                              snprintf((char*)TypeData[Port][Mode].Symbol, SYMBOL_LENGTH + 1, "%s", (char*)UartPort[Port].InBuffer);
#ifdef HIGHDEBUG
                              snprintf(UartBuffer, UARTBUFFERSIZE, "    %d SYMBOL = %s\n", Port, TypeData[Port][Mode].Symbol);
                              UartWrite(UartBuffer);
#endif
                              UartPort[Port].InfoData |=  INFODATA_INFO_SYMBOL;
                            }
                          }
                          break;

                          case INFO_FORMAT:
                          { // Data sets and format

                            if ((UartPort[Port].InfoData & INFODATA_INFO_FORMAT)) { // Format already given

#ifdef DEBUG_D_UART_ERROR
                              snprintf(UartBuffer, UARTBUFFERSIZE, " ## %d FORMAT ALREADY GIVEN\n", Port);
                              UartWrite(UartBuffer);
#endif
                              UartPort[Port].State = UART_INFO_ERROR;
                            } else {
                              TypeData[Port][Mode].DataSets = UartPort[Port].InBuffer[0];
                              TypeData[Port][Mode].Format   = UartPort[Port].InBuffer[1];

                              if (TypeData[Port][Mode].DataSets > 0) { // Data sets valid

                                if (UartPort[Port].Types) { // Modes left

                                  UartPort[Port].Types--;
                                  if (TypeData[Port][Mode].Mode == GET_MODE(UartPort[Port].Cmd)) { // Mode valid

                                    if (UartPort[Port].InLength >= 4) { // Figures and decimals present

                                      UartPort[Port].InfoData |=  INFODATA_INFO_FORMAT;

                                      if ((UartPort[Port].InfoData & INFODATA_NEEDED) == INFODATA_NEEDED) {
                                        snprintf((char*)TypeData[Port][Mode].Name, TYPE_NAME_LENGTH + 1, "%s", (char*)UartPort[Port].Name);

                                        TypeData[Port][Mode].Type       = UartPort[Port].Type;
                                        TypeData[Port][Mode].Connection = CONN_INPUT_UART;
                                        TypeData[Port][Mode].Views      = UartPort[Port].Views;

                                        TypeData[Port][Mode].Figures    = UartPort[Port].InBuffer[2];
                                        TypeData[Port][Mode].Decimals   = UartPort[Port].InBuffer[3];

//!<  \todo IR seeker hack
                                        if (TypeData[Port][Mode].Type == TYPE_IR) {
                                          TypeData[Port][Mode].InvalidTime = 1100;
                                        }

                                        Changed[Port][Mode]             = 1;
#ifdef HIGHDEBUG
                                        snprintf(UartBuffer, UARTBUFFERSIZE, "    %d FORMAT = %u * %u  %u.%u\n", Port, TypeData[Port][Mode].DataSets, TypeData[Port][Mode].Format, TypeData[Port][Mode].Figures, TypeData[Port][Mode].Decimals);
                                        UartWrite(UartBuffer);
#endif
                                      } else { // Not enough info data given
#ifdef DEBUG_D_UART_ERROR
                                        snprintf(UartBuffer, UARTBUFFERSIZE, " ## %d NOT ENOUGH INFO GIVEN\n", Port);
                                        UartWrite(UartBuffer);
#endif
                                        UartPort[Port].State = UART_INFO_ERROR;

                                      }
                                    } else { // Format invalid
#ifdef DEBUG_D_UART_ERROR
                                      snprintf(UartBuffer, UARTBUFFERSIZE, " ## %d FORMAT ERROR\n", Port);
                                      UartWrite(UartBuffer);
#endif
                                      UartPort[Port].State = UART_INFO_ERROR;

                                    }
                                  } else { // Mode invalid
#ifdef DEBUG_D_UART_ERROR
                                    snprintf(UartBuffer, UARTBUFFERSIZE, " f  %d FORMAT = %u * %u  %u.%u\n", Port, TypeData[Port][Mode].DataSets, TypeData[Port][Mode].Format, TypeData[Port][Mode].Figures, TypeData[Port][Mode].Decimals);
                                    UartWrite(UartBuffer);
#endif
                                    UartPort[Port].State = UART_INFO_ERROR;
                                  }
                                } else { // No more modes left
#ifdef DEBUG_D_UART_ERROR
                                  snprintf(UartBuffer, UARTBUFFERSIZE, " ## %d TYPES ERROR\n", Port);
                                  UartWrite(UartBuffer);
#endif
                                  UartPort[Port].State = UART_INFO_ERROR;

                                }
                              } else { // Data sets invalid
#ifdef DEBUG_D_UART_ERROR
                                snprintf(UartBuffer, UARTBUFFERSIZE, " f  %d FORMAT = %u * %u  %u.%u\n", Port, TypeData[Port][Mode].DataSets, TypeData[Port][Mode].Format, TypeData[Port][Mode].Figures, TypeData[Port][Mode].Decimals);
                                UartWrite(UartBuffer);
#endif
                                UartPort[Port].State = UART_INFO_ERROR;
                              }
                            }
                          }
                          break;

                        }
                      }
                      break;

                    }
                  }
                }

                if (UartPort[Port].Type == UartPortDefault.Type) {
#ifdef DEBUG_D_UART_ERROR
                  snprintf(UartBuffer, UARTBUFFERSIZE, " ## %d TYPE ERROR\n", Port);
                  UartWrite(UartBuffer);
#endif
                  UartPort[Port].State = UART_INFO_ERROR;
                }
              }
              break;

//** ERRORS *******************************************************************

            case UART_CMD_ERROR:
              UartPort[Port].State = UART_ERROR;
              break;

            case UART_INFO_ERROR:
              UartPort[Port].State = UART_ERROR;
              break;

            default:
              UartPort[Port].State = UART_MESSAGE_START;
              break;
            }

//** END OF INFORMATIONS ******************************************************

            UartPort[Port].Check ^=  Byte;
          }
          break;

  //** DATA *********************************************************************

        case UART_DATA:
          // Get device data

          UartPort[Port].InLength  = UartPortReadData(Port, &UartPort[Port].Cmd, TmpBuffer, &UartPort[Port].Check, &CrcError);

          if (UartPort[Port].InLength) {
//!<  \todo Color sensor hack (wrong checksum in mode 4 data)
            if ((UartPort[Port].Type == TYPE_COLOR) && (GET_MODE(UartPort[Port].Cmd) == 4)) {
              CrcError = 0;
            }

            if (!CrcError) {
              if (UartPort[Port].Initialised == 0) {
                UartPort[Port].Initialised = 1;
              }
              if (!(pUart->Status[Port] & UART_PORT_CHANGED)) {
                if (UartPort[Port].Mode == GET_MODE(UartPort[Port].Cmd)) {
                  if (!(pUart->Status[Port] & UART_DATA_READY)) {
                    pUart->Status[Port] |=  UART_DATA_READY;

#ifdef DEBUG_TRACE_MODE_CHANGE
                    snprintf(UartBuffer, UARTBUFFERSIZE, "d_uart %d   State machine: mode changed to  %d\n", Port, UartPort[Port].Mode);
                    UartWrite(UartBuffer);
#endif
                  }

#ifndef DISABLE_FAST_DATALOG_BUFFER
                  memcpy((void*)pUart->Raw[Port][pUart->LogIn[Port]], (void*)TmpBuffer, UART_DATA_LENGTH);

                  pUart->Actual[Port] = pUart->LogIn[Port];
                  pUart->Repeat[Port][pUart->Actual[Port]] = 0;

                  if (++(pUart->LogIn[Port]) >= DEVICE_LOGBUF_SIZE) {
                    pUart->LogIn[Port] = 0;
                  }
#else
                  memcpy((void*)pUart->Raw[Port], (void*)TmpBuffer, UART_DATA_LENGTH);
#endif
                  if (UartPort[Port].DataErrors) {
                    UartPort[Port].DataErrors--;
                  }
                  UartPort[Port].DataOk      = 1;
                } else {
                  UartPort[Port].ChangeMode  = 1;
                }
              }
            } else {
#ifdef DEBUG_D_UART_ERROR
              snprintf(UartBuffer, UARTBUFFERSIZE, " c  %d %02X[", Port, UartPort[Port].Cmd & 0xFF);
              UartWrite(UartBuffer);

              for (Tmp = 0;Tmp < UartPort[Port].InLength;Tmp++) {
                snprintf(UartBuffer, UARTBUFFERSIZE, "%02X", TmpBuffer[Tmp] & 0xFF);
                UartWrite(UartBuffer);
              }
              snprintf(UartBuffer, UARTBUFFERSIZE, "]%02X\n", UartPort[Port].Check & 0xFF);
              UartWrite(UartBuffer);
#endif
#ifndef DISABLE_UART_DATA_ERROR
              if (++UartPort[Port].DataErrors >= UART_ALLOWABLE_DATA_ERRORS) {
#ifdef DEBUG_D_UART_ERROR
                snprintf(UartBuffer, UARTBUFFERSIZE, " ## %d No valid data in %d messages\n", Port, UartPort[Port].DataErrors);
                UartWrite(UartBuffer);
#endif
                UartPort[Port].State = UART_DATA_ERROR;
              }
#endif
            }

#ifdef DEBUG
            if (ShowTimer[Port] >= (UART_SHOW_TIME / UART_TIMER_RESOLUTION)) {
              ShowTimer[Port] = 0;
              if (CrcError) {
                snprintf(UartBuffer, UARTBUFFERSIZE, " c  %d %02X[", Port, UartPort[Port].Cmd & 0xFF);
              } else {
                snprintf(UartBuffer, UARTBUFFERSIZE, "    %d %02X[", Port, UartPort[Port].Cmd & 0xFF);
              }
              UartWrite(UartBuffer);

              for (Tmp = 0;Tmp < UartPort[Port].InLength;Tmp++) {
                snprintf(UartBuffer, UARTBUFFERSIZE, "%02X", TmpBuffer[Tmp] & 0xFF);
                UartWrite(UartBuffer);
              }
              snprintf(UartBuffer, UARTBUFFERSIZE, "]%02X\n", UartPort[Port].Check & 0xFF);
              UartWrite(UartBuffer);
            }
#endif
          }

          if (UartPort[Port].ChangeMode) { // Try to change mode

            if (UartPort[Port].OutPointer >= UartPort[Port].OutLength) { // Transmitter ready

#ifdef DEBUG
              ShowTimer[Port] = 0;
              if (CrcError) {
                snprintf(UartBuffer, UARTBUFFERSIZE, " c  %d %02X[", Port, UartPort[Port].Cmd & 0xFF);
              } else {
                snprintf(UartBuffer, UARTBUFFERSIZE, "    %d %02X[", Port, UartPort[Port].Cmd & 0xFF);
              }
              UartWrite(UartBuffer);

              for (Tmp = 0;Tmp < UartPort[Port].InLength;Tmp++) {
                snprintf(UartBuffer, UARTBUFFERSIZE, "%02X", TmpBuffer[Tmp] & 0xFF);
                UartWrite(UartBuffer);
              }
              snprintf(UartBuffer, UARTBUFFERSIZE, "]%02X\n", UartPort[Port].Check & 0xFF);
              UartWrite(UartBuffer);
#endif
              UartPort[Port].Cmd           = UartPort[Port].Mode;
              UartPort[Port].OutBuffer[0]  = MAKE_CMD_COMMAND(CMD_SELECT, 0);
              UartPort[Port].OutBuffer[1]  = UartPort[Port].Mode;
              UartPort[Port].OutBuffer[2]  = 0xFF ^ UartPort[Port].OutBuffer[0] ^ UartPort[Port].OutBuffer[1];
              UartPort[Port].OutPointer    = 0;
              UartPort[Port].OutLength     = 3;

              UartPort[Port].ChangeMode = 0;

#ifdef DEBUG_TRACE_MODE_CHANGE
              snprintf(UartBuffer, UARTBUFFERSIZE, " WR %d %02X[", Port, UartPort[Port].OutBuffer[0]);
              UartWrite(UartBuffer);

              for (Tmp = 1;Tmp < (UartPort[Port].OutLength - 1);Tmp++) {
                snprintf(UartBuffer, UARTBUFFERSIZE, "%02X", UartPort[Port].OutBuffer[Tmp] & 0xFF);
                UartWrite(UartBuffer);
              }
              snprintf(UartBuffer, UARTBUFFERSIZE, "]%02X\n", UartPort[Port].OutBuffer[Tmp] & 0xFF);
              UartWrite(UartBuffer);
#endif
            }
            pUart->Status[Port] &= ~UART_DATA_READY;
          }
          if (++UartPort[Port].WatchDog >= (UART_WATCHDOG_TIME / UART_TIMER_RESOLUTION)) { // Try to service watch dog

            if (UartPort[Port].OutPointer >= UartPort[Port].OutLength) { // Transmitter ready

              UartPort[Port].WatchDog = 0;

              if (!UartPort[Port].DataOk) { // No ok data since last watch dog service

#ifndef DISABLE_UART_DATA_ERROR
                if (++UartPort[Port].DataErrors >= UART_ALLOWABLE_DATA_ERRORS) {
#ifdef DEBUG_D_UART_ERROR
                  snprintf(UartBuffer, UARTBUFFERSIZE, " ## %d No valid data in %d services\n", Port, UART_ALLOWABLE_DATA_ERRORS);
                  UartWrite(UartBuffer);
#endif
                  UartPort[Port].State       = UART_DATA_ERROR;
                } else {
                  UartPort[Port].DataOk      = 1;
                }
#else
                UartPort[Port].DataOk        = 1;
#endif
              }
              if (UartPort[Port].DataOk) {
                UartPort[Port].DataOk        = 0;

                UartPort[Port].OutBuffer[0]  = BYTE_NACK;
                UartPort[Port].OutPointer    = 0;
                UartPort[Port].OutLength     = 1;
#ifdef DEBUG
                snprintf(UartBuffer, UARTBUFFERSIZE, " WD %d %02X\n", Port, UartPort[Port].OutBuffer[0]);
                UartWrite(UartBuffer);
#endif
              }
            }
          }

          if (WriteRequest[Port]) { // Try to write message

            if (UartPort[Port].OutPointer >= UartPort[Port].OutLength) { // Transmitter ready

              // convert length to length code
              Byte = 0;
              Tmp = CONVERT_LENGTH(Byte);
              while ((Tmp < UART_DATA_LENGTH) && (Tmp < pUart->OutputLength[Port])) {
                Byte++;
                Tmp = CONVERT_LENGTH(Byte);
              }

              Chksum = 0xFF;

              UartPort[Port].OutBuffer[0] = MAKE_CMD_COMMAND(CMD_WRITE, Byte);
              Chksum                     ^=  UartPort[Port].OutBuffer[0];

              Pointer  = 0;
              while (Pointer < Tmp) {
                if (Pointer < pUart->OutputLength[Port]) {
                  UartPort[Port].OutBuffer[1 + Pointer] = pUart->Output[Port][Pointer];
                } else {
                  UartPort[Port].OutBuffer[1 + Pointer] = 0;
                }
                Chksum ^=  UartPort[Port].OutBuffer[1 + Pointer];
                Pointer++;
              }
              UartPort[Port].OutBuffer[1 + Pointer]  = Chksum;
              UartPort[Port].OutPointer              = 0;
              UartPort[Port].OutLength               = Tmp + 2;

              WriteRequest[Port]                     = 0;
              pUart->Status[Port]                   &= ~UART_WRITE_REQUEST;
#ifdef DEBUG
              snprintf(UartBuffer, UARTBUFFERSIZE, " WR %d %02X[", Port, UartPort[Port].OutBuffer[0]);
              UartWrite(UartBuffer);

              for (Tmp = 1;Tmp < (UartPort[Port].OutLength - 1);Tmp++) {
                snprintf(UartBuffer, UARTBUFFERSIZE, "%02X", UartPort[Port].OutBuffer[Tmp] & 0xFF);
                UartWrite(UartBuffer);
              }
              snprintf(UartBuffer, UARTBUFFERSIZE, "]%02X\n", UartPort[Port].OutBuffer[Tmp] & 0xFF);
              UartWrite(UartBuffer);
#endif
            }
          }

#ifndef DISABLE_FAST_DATALOG_BUFFER
          (pUart->Repeat[Port][pUart->Actual[Port]])++;
#endif
          break;

        case UART_ACK_WAIT:
          if (++(UartPort[Port].Timer) >= (UART_ACK_DELAY / UART_TIMER_RESOLUTION)) {
            pUart->Status[Port] |= UART_PORT_CHANGED;
            UartPortSend(Port, BYTE_ACK);
            UartPort[Port].Timer   = 0;
            UartPort[Port].State   = UART_ACK_INFO;
#ifdef DEBUG_D_UART_ERROR
            snprintf(UartBuffer, UARTBUFFERSIZE, "    %d Type %-3d has changed modes: ", Port, TypeData[Port][0].Type);
            UartWrite(UartBuffer);
            for (Mode = 0;Mode < MAX_DEVICE_MODES;Mode++) {
              UartBuffer[Mode] = Changed[Port][Mode] + '0';
            }
            UartBuffer[Mode++] = '\r';
            UartBuffer[Mode++] = '\n';
            UartBuffer[Mode]   = 0;
            UartWrite(UartBuffer);
#endif
          }
          break;

        case UART_ACK_INFO:
          if (++(UartPort[Port].Timer) >= (UART_CHANGE_BITRATE_DELAY / UART_TIMER_RESOLUTION)) {
            UartPort[Port].DataOk      = 1;
            UartPort[Port].DataErrors  = 0;
            UartPort[Port].Mode        = 0;
            UartPort[Port].BitRate     = UartPort[Port].BitRateMax;
            UartPortSetup(Port, UartPort[Port].BitRate);
            UartPort[Port].WatchDog    = (UART_WATCHDOG_TIME / UART_TIMER_RESOLUTION);
            UartPort[Port].State       = UART_DATA;
          }
          break;

        case UART_TERMINAL:
          break;

        case UART_DATA_ERROR:
          UartPort[Port].State = UART_ERROR;
          break;

        case UART_ERROR:
          break;

        case UART_EXIT:
          UartPortDisable(Port);
          UartPort[Port]                         = UartPortDefault;

          for (Tmp = 0;Tmp < MAX_DEVICE_MODES;Tmp++) {
            TypeData[Port][Tmp].Name[0]          = 0;
            TypeData[Port][Tmp].Type             = 0;
            Changed[Port][Tmp]                   = 0;
          }
          pUart->Status[Port]                  = 0;

          UartPort[Port].State                   = UART_IDLE;
          break;

        }
        if (UartPort[Port].OutPointer < UartPort[Port].OutLength) {
          if (UartPortSend(Port, UartPort[Port].OutBuffer[UartPort[Port].OutPointer])) {
            UartPort[Port].OutPointer++;
          }
        }

      } else {
        switch (UartPort[Port].State) { // Test state machine

        case UART_IDLE:
          // Port inactive
          break;

        case UART_INIT:
          // Initialise port hardware

          UartPortDisable(Port);
          PUARTFloat(Port, INPUT_PORT_PIN5);
          PUARTFloat(Port, INPUT_PORT_PIN6);
          UartPort[Port].State       = UART_ENABLE;
          break;

        case UART_ENABLE:
          // Initialise port variables

          UartPortEnable(Port);
          UartPortSetup(Port, UartPort[Port].BitRate);
          pUart->Status[Port]          = 0;
          UartPortFlush(Port);
          UartPort[Port].InPointer       = 0;
          UartPort[Port].State           = UART_MESSAGE_START;
          break;

        case UART_MESSAGE_START:
          if (UartPort[Port].OutPointer < UartPort[Port].OutLength) {
            if (UartPortSend(Port, UartPort[Port].OutBuffer[UartPort[Port].OutPointer])) {
              UartPort[Port].OutPointer++;
            }
          }
          if (UartPortReceive(Port, &Byte)) {
#ifdef HIGHDEBUG
            snprintf(UartBuffer, UARTBUFFERSIZE, "[0x%02X]\n", Byte);
            UartWrite(UartBuffer);
#endif
            if (UartPort[Port].InPointer < UART_BUFFER_SIZE) {
              UartPort[Port].InBuffer[UartPort[Port].InPointer] = Byte;
              UartPort[Port].InPointer++;
            }
          }
          break;
        }
      }
#ifdef HIGHDEBUG
      if (UartPort[Port].OldState != UartPort[Port].State) {
        UartPort[Port].OldState   = UartPort[Port].State;
        if (UartPort[Port].State  != UART_ENABLE) {
          snprintf(UartBuffer, UARTBUFFERSIZE, "    %d %s\n", Port, UartStateText[UartPort[Port].State]);
        } else {
          snprintf(UartBuffer, UARTBUFFERSIZE, "*** %d %s ***\n", Port, UartStateText[UartPort[Port].State]);
        }
        UartWrite(UartBuffer);
      }
#endif
    }
  }

  return HRTIMER_RESTART;
}

static long Device1Ioctl(struct file *File, unsigned int Request, unsigned long Pointer)
{
	UARTCTL  UartCtl;
	DEVCON   DevCon;
	DATA8    Port;
	DATA8    Mode;
	int      ret;

	switch (Request) {

	case UART_SET_CONN:
		ret = copy_from_user((void*)&DevCon, (void*)Pointer, sizeof(DEVCON));
		if (ret < 0)
			return ret;

		for (Port = 0; Port < INPUTS; Port++) {
			if (DevCon.Connection[Port] == CONN_INPUT_UART) {
				if (UartConfigured[Port] == 0) {
					UartConfigured[Port]       = 1;
					UartPort[Port].State       = UART_INIT;
				} else {
					if (UartPort[Port].Initialised) {
						if (UartPort[Port].Mode != DevCon.Mode[Port]) {
#ifdef DEBUG_TRACE_MODE_CHANGE
							snprintf(UartBuffer, UARTBUFFERSIZE, "d_uart %d   Device1Ioctl: Changing to    %c\n", Port, DevCon.Mode[Port] + '0');
							UartWrite(UartBuffer);
#endif
							UartPort[Port].Mode        = DevCon.Mode[Port];
							UartPort[Port].ChangeMode  = 1;
							pUart->Status[Port]      &= ~UART_DATA_READY;
						}
					}
				}
			} else {
				pUart->Status[Port] &= ~UART_DATA_READY;
				if (UartConfigured[Port]) {
					UartConfigured[Port]       = 0;
					UartPort[Port].State       = UART_EXIT;
				}
			}
		}
		break;

	case UART_READ_MODE_INFO:
		ret = copy_from_user((void*)&UartCtl, (void*)Pointer, sizeof(UARTCTL));
		if (ret < 0)
			return ret;
		Port = UartCtl.Port;
		Mode = UartCtl.Mode;

#ifdef DEBUG
		if (TypeData[Port][Mode].Name[0]) {
			snprintf(UartBuffer, UARTBUFFERSIZE, "d_uart %d   Device1Ioctl: READ    Type=%d Mode=%d\n", Port, TypeData[Port][Mode].Type, Mode);
			UartWrite(UartBuffer);
		}
#endif
		if ((Mode < MAX_DEVICE_MODES) && (Port < INPUTS)) {
			UartCtl.TypeData = TypeData[Port][Mode];
			if (Changed[Port][Mode] == 0) {
				UartCtl.TypeData.Name[0] = 0;
			}
			Changed[Port][Mode] = 0;
		}
		ret = copy_to_user((void*)Pointer, (void*)&UartCtl, sizeof(UARTCTL));
		if (ret < 0)
			return ret;
		break;

	case UART_NACK_MODE_INFO:
		ret = copy_from_user((void*)&UartCtl, (void*)Pointer, sizeof(UARTCTL));
		if (ret < 0)
			return ret;
		Port = UartCtl.Port;
		Mode = UartCtl.Mode;

#ifdef DEBUG
		snprintf(UartBuffer, UARTBUFFERSIZE, "d_uart %d   Device1Ioctl: NACK    Type=%d Mode=%d\n", Port, TypeData[Port][Mode].Type, Mode);
		UartWrite(UartBuffer);
#endif
		if ((Mode < MAX_DEVICE_MODES) && (Port < INPUTS)) {
			Changed[Port][Mode] = 1;
		}
		break;

	case UART_CLEAR_CHANGED:
		ret = copy_from_user((void*)&UartCtl, (void*)Pointer, sizeof(UARTCTL));
		if (ret < 0)
			return ret;
		Port = UartCtl.Port;

		pUart->Status[Port] &= ~UART_PORT_CHANGED;
		break;
	}

	return 0;

}

static ssize_t Device1Write(struct file *File, const char *Buffer, size_t Count, loff_t *Data)
{
	char    Buf[UART_DATA_LENGTH + 1] = { 0 };
	DATA8   Port;
	int     Lng = 0;
	int     ret;

	if (Count <= (UART_DATA_LENGTH + 1)) {

		ret = copy_from_user(Buf, Buffer, Count);
		if (ret < 0)
			return ret;

		Port = Buf[0];

		if (Port < INPUTS) {
			Lng = 1;

			while (Lng < Count) {
				pUart->Output[Port][Lng - 1] = Buf[Lng];
				Lng++;
			}
			pUart->OutputLength[Port]  = Lng - 1;
			pUart->Status[Port]       |= UART_WRITE_REQUEST;
			WriteRequest[Port]           = 1;
		}
	}

	return Lng;
}

static ssize_t Device1Read(struct file *File, char *Buffer, size_t Count, loff_t *Offset)
{
	char Buf[256];
	int Lng = 0;
	int ret;

#ifdef DEBUG_UART_WRITE
	if (LogOutPointer != LogPointer) {
		while ((Count--) && (LogOutPointer != LogPointer)) {
			Buf[Lng++] = LogPool[LogOutPointer];
			Buf[Lng]   = 0;

			LogOutPointer++;
			if (LogOutPointer >= LOGPOOLSIZE) {
				LogOutPointer = 0;
			}
		}
	}
	if (Lng == 0) {
		UartBuffer[0] = 0x1B;
		UartBuffer[1] = '[';
		UartBuffer[2] = '2';
		UartBuffer[3] = 'J';
		UartBuffer[4] = 0x1B;
		UartBuffer[5] = '[';
		UartBuffer[6] = 'H';
		UartBuffer[7] = 0;
		UartWrite(UartBuffer);
		UartWrite(UartBuffer);
		snprintf(UartBuffer, UARTBUFFERSIZE, "-----------------------------------------------------------------\n");
		UartWrite(UartBuffer);
		snprintf(UartBuffer, UARTBUFFERSIZE, "    UART DUMP\n");
		UartWrite(UartBuffer);
		snprintf(UartBuffer, UARTBUFFERSIZE, "-----------------------------------------------------------------\n");
		UartWrite(UartBuffer);
	}
#else
	int Tmp;
	int Port;

	Port  = 0;
	Tmp   = 5;
	while ((Count > Tmp) && (Port < INPUTS)) {
		if (Port != (INPUTS - 1)) {
			Tmp = snprintf(&Buf[Lng], 4, "%2u ", (UWORD)UartPort[Port].State);
		} else {
			Tmp = snprintf(&Buf[Lng], 5, "%2u\r", (UWORD)UartPort[Port].State);
		}
		Lng   +=  Tmp;
		Count -=  Tmp;
		Port++;
	}
#endif

	ret = copy_to_user(Buffer, Buf, Lng);
	if (ret < 0)
		return ret;

	return Lng;
}

#define SHM_LENGTH (sizeof(UartDefault))
#define NPAGES ((SHM_LENGTH + PAGE_SIZE - 1) / PAGE_SIZE)
static void *kmalloc_ptr;

static int Device1Mmap(struct file *filp, struct vm_area_struct *vma)
{
	int ret;

	ret = remap_pfn_range(vma, vma->vm_start, virt_to_phys((void*)((unsigned long)pUart)) >> PAGE_SHIFT, vma->vm_end-vma->vm_start, PAGE_SHARED);
	if (ret < 0)
		ret = -EAGAIN;

	return ret;
}

static const struct file_operations Device1Entries = {
	.owner		= THIS_MODULE,
	.read		= Device1Read,
	.write		= Device1Write,
	.mmap		= Device1Mmap,
	.unlocked_ioctl	= Device1Ioctl,
};

static struct miscdevice Device1 = {
	MISC_DYNAMIC_MINOR,
	DEVICE1_NAME,
	&Device1Entries
};

static int Device1Init(void)
{
	UWORD *pTmp;
	int ret, i, Tmp;

	Device1Lms2012CompatDev = lms2012_compat_get();
	if (!Device1Lms2012CompatDev)
		return -EPROBE_DEFER;

	Device1Lms2012Compat = dev_get_drvdata(Device1Lms2012CompatDev);

	// allocate kernel shared memory for uart values (pUart)
	kmalloc_ptr = kmalloc((NPAGES + 2) * PAGE_SIZE, GFP_KERNEL);
	if (!kmalloc_ptr) {
		ret = -ENOMEM;
		goto err0;
	}

	pTmp = (UWORD*)((((unsigned long)kmalloc_ptr) + PAGE_SIZE - 1) & PAGE_MASK);
	for (i = 0; i < NPAGES * PAGE_SIZE; i += PAGE_SIZE)
		SetPageReserved(virt_to_page(((unsigned long)pTmp) + i));
	pUart = (UART*)pTmp;
	memset(pUart, 0, sizeof(UART));

	ret = Uart1Init();
	if (ret < 0)
		goto err1;
	ret = Uart2Init();
	if (ret < 0)
		goto err2;
	ret = Uart3Init();
	if (ret < 0)
		goto err3;
	ret = Uart4Init();
	if (ret < 0)
		goto err4;

	for (Tmp = 0;Tmp < INPUTS;Tmp++) {
		UartPort[Tmp]       = UartPortDefault;
		UartConfigured[Tmp] = 0;

		if (Tmp == DEBUG_UART) {
			UartPort[Tmp].BitRate = 115200;
			UartPortSetup(Tmp, UartPort[Tmp].BitRate);
			TypeData[Tmp][0] = TypeDefaultUart[1];
			pUart->Status[Tmp] |= UART_PORT_CHANGED;
			UartPortEnable(Tmp);
#ifdef DEBUG
			snprintf(UartBuffer, UARTBUFFERSIZE, "  %s debug uart init test\n", DEVICE1_NAME);
			UartWrite(UartBuffer);
#endif
		} else {
			UartPortDisable(Tmp);
		}
	}

	ret = misc_register(&Device1);
	if (ret)
		goto err5;

	Device1Time = ktime_set(0, UART_TIMER_RESOLUTION * 100000);
	hrtimer_init(&Device1Timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	Device1Timer.function = Device1TimerInterrupt1;

	hrtimer_start(&Device1Timer, Device1Time, HRTIMER_MODE_REL);

	return 0;

err5:
	Uart4Exit();
err4:
	Uart3Exit();
err3:
	Uart2Exit();
err2:
	Uart1Exit();
err1:
	for (i = 0; i < NPAGES * PAGE_SIZE; i+= PAGE_SIZE) {
		ClearPageReserved(virt_to_page(((unsigned long)pTmp) + i));
	}
	kfree(kmalloc_ptr);
err0:
	put_device(Device1Lms2012CompatDev);

	return ret;
}

static void Device1Exit(void)
{
	int     Tmp;
	UWORD   *pTmp;
	int     i;

	hrtimer_cancel(&Device1Timer);

	misc_deregister(&Device1);

	Uart4Exit();
	Uart3Exit();
	Uart2Exit();
	Uart1Exit();

	for (Tmp = 0; Tmp < INPUTS; Tmp++) {
		UartPort[Tmp] = UartPortDefault;

		if (Tmp == DEBUG_UART)
			UartPortEnable(Tmp);
		else
			UartPortDisable(Tmp);
	}

	if (Tmp == DEBUG_UART)
		UartPortEnable(0);
	// free shared memory
	pTmp = (UWORD*)pUart;
	pUart = &UartDefault;

	for (i = 0; i < NPAGES * PAGE_SIZE; i+= PAGE_SIZE) {
		ClearPageReserved(virt_to_page(((unsigned long)pTmp) + i));
	}
	kfree(kmalloc_ptr);

	put_device(Device1Lms2012CompatDev);
}

// MODULE *********************************************************************

static int d_uart_probe(struct platform_device *pdev)
{
	int ret;

	ret = Device1Init();

	pr_info("d_uart registered\n");

	return ret;
}

static int d_uart_remove(struct platform_device *pdev)
{
	Device1Exit();

	pr_info("d_uart removed\n");

	return 0;
}

static struct platform_driver d_uart_driver = {
	.driver	= {
		.name	= "d_uart",
	},
	.probe	= d_uart_probe,
	.remove	= d_uart_remove,
};
module_platform_driver(d_uart_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("LIU");
MODULE_DESCRIPTION(MODULE_NAME);
MODULE_ALIAS("platform:d_uart");
