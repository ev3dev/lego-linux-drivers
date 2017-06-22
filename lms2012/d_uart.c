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
#include <linux/kernel.h>
#include <linux/miscdevice.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/tty.h>
#include <linux/uaccess.h>

#include <asm/io.h>

#include "lms2012.h"

#define N_D_UART		27 /* line dicipline no. */

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

#define PUARTFloat(port, pin) \
	gpiod_direction_input(Device1Lms2012Compat->in_pins[port]->desc[pin])

#define PUARTRead(port, pin) \
	gpiod_get_value(Device1Lms2012Compat->in_pins[port]->desc[pin])

#define PUARTHigh(port, pin) \
	gpiod_direction_output(Device1Lms2012Compat->in_pins[port]->desc[pin], 1)

#define PUARTLow(port, pin) \
	gpiod_direction_output(Device1Lms2012Compat->in_pins[port]->desc[pin], 0)

#define UART_RECBUF_SIZE	256

#ifdef DEBUG
static UWORD ShowTimer[INPUTS];
#endif

#if defined(HIGHDEBUG) || defined(DEBUG_D_UART_ERROR) || defined(DEBUG) || defined(DEBUG_TRACE_MODE_CHANGE) || defined(DEBUG_UART_WRITE)

#define UARTBUFFERSIZE 250
static char UartBuffer[UARTBUFFERSIZE];
#define LOGPOOLSIZE 100000
static ULONG LogPointer = 0;
static ULONG LogOutPointer = 0;
static char LogPool[LOGPOOLSIZE];

static UBYTE UartPortSend(UBYTE Port, UBYTE Byte);

static void UartDebug(char *pString)
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

#endif

// UART *********************************************************************

struct d_uart_ldisc_data {
	struct tty_struct *tty;
	UBYTE port;
	UBYTE UartRecBuf[UART_RECBUF_SIZE];
	UWORD UartRecBufIn;
	UWORD UartRecBufOut;
	UBYTE UartRecMesLng;
	UBYTE UartRecMes[UART_BUFFER_SIZE];
	UBYTE UartRecMesIn;
};

static struct d_uart_ldisc_data d_uart_ldisc_data[INPUTS];

static void d_uart_receive_buf(struct tty_struct *tty, const unsigned char *cp,
			       char *fp, int count)
{
	struct d_uart_ldisc_data *data = tty->disc_data;
	int i;

	for (i = 0; i < count; i++) {
		data->UartRecBuf[data->UartRecBufIn] = cp[i];

		if (++data->UartRecBufIn >= UART_RECBUF_SIZE)
			data->UartRecBufIn = 0;
	}
}

static UBYTE UartRead(UBYTE port, UBYTE *pByte)
{
	struct d_uart_ldisc_data *data = &d_uart_ldisc_data[port];
	UBYTE   Result = 0;

	if (data->UartRecBufIn != data->UartRecBufOut) {
		*pByte = data->UartRecBuf[data->UartRecBufOut];

		if (++data->UartRecBufOut >= UART_RECBUF_SIZE) {
			data->UartRecBufOut = 0;
		}
		Result   = 1;
	}

	return Result;
}

static void UartFlush(int port)
{
	struct d_uart_ldisc_data *data = &d_uart_ldisc_data[port];
	struct tty_struct *tty = data->tty;

	if (tty->ldisc->ops->flush_buffer)
		tty->ldisc->ops->flush_buffer(tty);
	tty_driver_flush_buffer(tty);
	data->UartRecBufIn = 0;
	data->UartRecBufOut = 0;
	data->UartRecMesIn = 0;
}

static UBYTE UartReadData(UBYTE port, UBYTE *pCmd, UBYTE *pData, UBYTE *pCheck, UBYTE *pFail)
{
	struct d_uart_ldisc_data *data = &d_uart_ldisc_data[port];
	UBYTE   Byte;
	UBYTE   Length;
	UBYTE   Collect;

	Length   = 0;
	*pFail   = 0xFF;
	Collect  = 1;

	while (Collect) {
		if (UartRead(port, &Byte)) {
			if (data->UartRecMesIn == 0) { // Wait for data message start

				if (GET_MESSAGE_TYPE(Byte) == MESSAGE_DATA) {

					data->UartRecMesLng = GET_MESSAGE_LENGTH(Byte) + 2;

					if (data->UartRecMesLng <= UART_BUFFER_SIZE) { // Valid length

						data->UartRecMes[data->UartRecMesIn] = Byte;
						data->UartRecMesIn++;
					}
				}
			} else {
				data->UartRecMes[data->UartRecMesIn] = Byte;

				if (++data->UartRecMesIn >= data->UartRecMesLng) { // Message ready

					*pCmd   = data->UartRecMes[0];
					*pFail  ^=  *pCmd;

					while (Length < (data->UartRecMesLng - 2)) {
						pData[Length]  = data->UartRecMes[Length + 1];
						*pFail       ^=  pData[Length];
						Length++;
					}
					*pCheck = data->UartRecMes[Length + 1];
					*pFail  ^=  *pCheck;

					data->UartRecMesIn = 0;
					Collect = 0;
				}
			}
		} else {
			Collect = 0;
		}
	}

	return Length;
}

static UBYTE UartWrite(UBYTE port, UBYTE Byte)
{
	struct d_uart_ldisc_data *data = &d_uart_ldisc_data[port];
	struct tty_struct *tty = data->tty;
	int ret;

	/*
	 * FIXME: there is a bunch of sleeping stuff here that is called in an
	 * atomic context.
	 */
	set_bit(TTY_DO_WRITE_WAKEUP, &tty->flags);
	ret = tty_put_char(tty, Byte);
	if (tty->ops->flush_chars)
		tty->ops->flush_chars(tty);

	return ret;
}

static void UartSetup(UBYTE port, ULONG BitRate)
{
	struct d_uart_ldisc_data *data = &d_uart_ldisc_data[port];
	struct tty_struct *tty = data->tty;
	struct ktermios old_termios = tty->termios;

	/*
	 * FIXME: there is a bunch of sleeping stuff here that is called in an
	 * atomic context.
	 */
	tty_wait_until_sent(tty, 0);
	down_write(&tty->termios_rwsem);
	tty_encode_baud_rate(tty, BitRate, BitRate);
	if (tty->ops->set_termios)
		tty->ops->set_termios(tty, &old_termios);
	up_write(&tty->termios_rwsem);
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

static const char * const UartStateText[UART_STATES] = {
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
	"EXIT",
};

static const UARTPORT UartPortDefault = {
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
static UARTPORT UartPort[INPUTS];

static UART UartDefault;
static UART *pUart = &UartDefault;

static struct tasklet_hrtimer Device1Timer;
static ktime_t        Device1Time;

static UBYTE TestMode = 0;

static void UartPortDisable(UBYTE Port)
{
	if (Port >= INPUTS || !d_uart_ldisc_data[Port].tty)
		return;

	PUARTHigh(Port, INPUT_PORT_BUF);
}

static void UartPortFlush(UBYTE Port)
{
	if (Port >= INPUTS || !d_uart_ldisc_data[Port].tty)
		return;

	UartFlush(Port);
}

static void UartPortEnable(UBYTE Port)
{
	if (Port >= INPUTS || !d_uart_ldisc_data[Port].tty)
		return;

	// TODO: well need to do pin mux when I2C is working
	// SetGpio(InputUartPin[Port][INPUT_UART_TXD].Pin);
	PUARTLow(Port, INPUT_PORT_BUF);
	UartFlush(Port);
}

static UBYTE UartPortSend(UBYTE Port, UBYTE Byte)
{
	if (Port >= INPUTS || !d_uart_ldisc_data[Port].tty)
		return 1;

	return UartWrite(Port, Byte);
}

static UBYTE UartPortReceive(UBYTE Port, UBYTE *pByte)
{
	if (Port >= INPUTS || !d_uart_ldisc_data[Port].tty)
		return 0;

	return UartRead(Port, pByte);
}

static UBYTE UartPortReadData(UBYTE Port, UBYTE *pCmd, UBYTE *pData,
			      UBYTE *pCheck, UBYTE *pFail)
{
	if (Port >= INPUTS || !d_uart_ldisc_data[Port].tty)
		return 0;

	return UartReadData(Port, pCmd, pData, pCheck, pFail);
}

static void UartPortSetup(UBYTE Port, ULONG BitRate)
{
	if (Port >= INPUTS || !d_uart_ldisc_data[Port].tty)
		return;

	UartSetup(Port, BitRate);
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
          UartDebug(UartBuffer);
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
                UartDebug(UartBuffer);
#endif
                UartPort[Port].State     = UART_MESSAGE_START;
              } else {
#ifdef DEBUG_D_UART_ERROR
//                snprintf(UartBuffer, UARTBUFFERSIZE, "[%02X]", Byte);
//                UartDebug(UartBuffer);
//                snprintf(UartBuffer, UARTBUFFERSIZE, "    %d No sync %02X %02X %02X\n", Port, UartPort[Port].InBuffer[0], UartPort[Port].InBuffer[1], UartPort[Port].InBuffer[2]);
//                UartDebug(UartBuffer);
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
                  UartDebug(UartBuffer);
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
                  UartDebug(UartBuffer);
                  for (Tmp = 0;Tmp < UartPort[Port].InLength;Tmp++) {
                    snprintf(UartBuffer, UARTBUFFERSIZE, "%02X", UartPort[Port].InBuffer[Tmp] & 0xFF);
                    UartDebug(UartBuffer);
                  }
                  snprintf(UartBuffer, UARTBUFFERSIZE, "]%02X\n", UartPort[Port].Check & 0xFF);
                  UartDebug(UartBuffer);
#endif
                  UartPort[Port].State = UART_CMD_ERROR;
                } else { // Command message valid
#ifdef HIGHDEBUG
                  snprintf(UartBuffer, UARTBUFFERSIZE, "    %d %02X[", Port, UartPort[Port].Cmd & 0xFF);
                  UartDebug(UartBuffer);
                  for (Tmp = 0;Tmp < UartPort[Port].InLength;Tmp++) {
                    snprintf(UartBuffer, UARTBUFFERSIZE, "%02X", UartPort[Port].InBuffer[Tmp] & 0xFF);
                    UartDebug(UartBuffer);
                  }
                  snprintf(UartBuffer, UARTBUFFERSIZE, "]%02X\n", UartPort[Port].Check & 0xFF);
                  UartDebug(UartBuffer);
#endif
                  switch (GET_CMD_COMMAND(UartPort[Port].Cmd)) { // Command message type

                    case CMD_MODES:
                    { // Number of modes

                      if ((UartPort[Port].InBuffer[0] >= 0) && (UartPort[Port].InBuffer[0] < MAX_DEVICE_MODES)) { // Number of modes valid

                        if ((UartPort[Port].InfoData & INFODATA_CMD_MODES)) { // Modes already given

#ifdef DEBUG_D_UART_ERROR
                          snprintf(UartBuffer, UARTBUFFERSIZE, " ## %d MODES ALREADY GIVEN\n", Port);
                          UartDebug(UartBuffer);
#endif
                          UartPort[Port].State = UART_CMD_ERROR;
                        } else {

                          UartPort[Port].Types = UartPort[Port].InBuffer[0] + 1;
                          if (UartPort[Port].InLength >= 2) { // Both modes and views present

                            UartPort[Port].Views = UartPort[Port].InBuffer[1] + 1;
#ifdef HIGHDEBUG
                            snprintf(UartBuffer, UARTBUFFERSIZE, "    %d MODES  = %u  VIEWS  = %u\n", Port, (UWORD)UartPort[Port].Types & 0xFF, (UWORD)UartPort[Port].Views & 0xFF);
                            UartDebug(UartBuffer);
#endif
                          } else { // Only modes present

                            UartPort[Port].Views = UartPort[Port].Types;
#ifdef HIGHDEBUG
                            snprintf(UartBuffer, UARTBUFFERSIZE, "    %d MODES  = %u = VIEWS\n", Port, (UWORD)UartPort[Port].Types & 0xFF);
                            UartDebug(UartBuffer);
#endif
                          }
                          UartPort[Port].InfoData |=  INFODATA_CMD_MODES;
                        }
                      } else { // Number of modes invalid
#ifdef DEBUG_D_UART_ERROR
                        snprintf(UartBuffer, UARTBUFFERSIZE, " ## %d MODES ERROR  %u\n", Port, (UWORD)UartPort[Port].Types & 0xFF);
                        UartDebug(UartBuffer);
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
                          UartDebug(UartBuffer);
#endif
                          UartPort[Port].State = UART_CMD_ERROR;
                        } else {
                          if ((UartPort[Port].BitRate != LOWEST_BITRATE) && (TmpL <= MIDLE_BITRATE)) { // allow bit rate adjust
#ifdef HIGHDEBUG
                            snprintf(UartBuffer, UARTBUFFERSIZE, "    %d SPEED ADJUST\n", Port);
                            UartDebug(UartBuffer);
#endif
                            UartPort[Port].BitRateMax = (UartPort[Port].BitRate * TmpL) / LOWEST_BITRATE;
                          } else {

                            UartPort[Port].BitRateMax = TmpL;
                          }
#ifdef HIGHDEBUG
                          snprintf(UartBuffer, UARTBUFFERSIZE, "    %d SPEED  = %lu\n", Port, (unsigned long)UartPort[Port].BitRateMax);
                          UartDebug(UartBuffer);
#endif
                          UartPort[Port].InfoData |=  INFODATA_CMD_SPEED;
                        }
                      } else { // Speed invalid
#ifdef DEBUG_D_UART_ERROR
                        snprintf(UartBuffer, UARTBUFFERSIZE, " ## %d SPEED ERROR\n", Port);
                        UartDebug(UartBuffer);
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
                          UartDebug(UartBuffer);
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
                        UartDebug(UartBuffer);
                        snprintf(UartBuffer, UARTBUFFERSIZE, "%02X[", UartPort[Port].InfoCmd & 0xFF);
                        UartDebug(UartBuffer);
                        for (Tmp = 0;Tmp < UartPort[Port].InLength;Tmp++) {
                          snprintf(UartBuffer, UARTBUFFERSIZE, "%02X", UartPort[Port].InBuffer[Tmp] & 0xFF);
                          UartDebug(UartBuffer);
                        }
                        snprintf(UartBuffer, UARTBUFFERSIZE, "]%02X\n", UartPort[Port].Check & 0xFF);
                        UartDebug(UartBuffer);
#endif
                        UartPort[Port].State = UART_INFO_ERROR;
                      } else {
#ifdef HIGHDEBUG
                        snprintf(UartBuffer, UARTBUFFERSIZE, "    %d %02X ", Port, UartPort[Port].Cmd & 0xFF);
                        UartDebug(UartBuffer);
                        snprintf(UartBuffer, UARTBUFFERSIZE, "%02X[", UartPort[Port].InfoCmd & 0xFF);
                        UartDebug(UartBuffer);
                        for (Tmp = 0;Tmp < UartPort[Port].InLength;Tmp++) {
                          snprintf(UartBuffer, UARTBUFFERSIZE, "%02X", UartPort[Port].InBuffer[Tmp] & 0xFF);
                          UartDebug(UartBuffer);
                        }
                        snprintf(UartBuffer, UARTBUFFERSIZE, "]%02X\n", UartPort[Port].Check & 0xFF);
                        UartDebug(UartBuffer);
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
                              UartDebug(UartBuffer);
#endif
                              TypeData[Port][Mode].Mode  = Mode;
                              UartPort[Port].InfoData   |= INFODATA_INFO_NAME;
                            } else {
#ifdef DEBUG_D_UART_ERROR
                              UartPort[Port].InBuffer[TYPE_NAME_LENGTH] = 0;
                              snprintf(UartBuffer, UARTBUFFERSIZE, " f  %d NAME = %s\n", Port, UartPort[Port].InBuffer);
                              UartDebug(UartBuffer);
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
                                UartDebug(UartBuffer);
#endif
                                UartPort[Port].State = UART_INFO_ERROR;
                              } else {
#ifdef HIGHDEBUG
                                snprintf(UartBuffer, UARTBUFFERSIZE, "    %d RAW = Min..Max\n", Port);
                                UartDebug(UartBuffer);
#endif
                                UartPort[Port].InfoData |=  INFODATA_INFO_RAW;
                              }
                            } else {
#ifdef DEBUG_D_UART_ERROR
                              snprintf(UartBuffer, UARTBUFFERSIZE, " f  %d RAW = Min..Max\n", Port);
                              UartDebug(UartBuffer);
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
                                UartDebug(UartBuffer);
#endif
                                UartPort[Port].State = UART_INFO_ERROR;
                              } else {
#ifdef HIGHDEBUG
                                snprintf(UartBuffer, UARTBUFFERSIZE, "    %d PCT = Min..Max\n", Port);
                                UartDebug(UartBuffer);
#endif
                                UartPort[Port].InfoData |=  INFODATA_INFO_PCT;
                              }
                            } else { // Mode invalid
#ifdef DEBUG_D_UART_ERROR
                              snprintf(UartBuffer, UARTBUFFERSIZE, " f  %d PCT = Min..Max\n", Port);
                              UartDebug(UartBuffer);
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
                                UartDebug(UartBuffer);
#endif
                                UartPort[Port].State = UART_INFO_ERROR;
                              } else {
#ifdef HIGHDEBUG
                                snprintf(UartBuffer, UARTBUFFERSIZE, "    %d SI  = Min..Max\n", Port);
                                UartDebug(UartBuffer);
#endif
                                UartPort[Port].InfoData |=  INFODATA_INFO_SI;
                              }
                            } else { // Mode invalid
#ifdef DEBUG_D_UART_ERROR
                              snprintf(UartBuffer, UARTBUFFERSIZE, " f  %d SI  = Min..Max\n", Port);
                              UartDebug(UartBuffer);
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
                              UartDebug(UartBuffer);
#endif
                              UartPort[Port].State = UART_INFO_ERROR;
                            } else {
                              snprintf((char*)TypeData[Port][Mode].Symbol, SYMBOL_LENGTH + 1, "%s", (char*)UartPort[Port].InBuffer);
#ifdef HIGHDEBUG
                              snprintf(UartBuffer, UARTBUFFERSIZE, "    %d SYMBOL = %s\n", Port, TypeData[Port][Mode].Symbol);
                              UartDebug(UartBuffer);
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
                              UartDebug(UartBuffer);
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
                                        UartDebug(UartBuffer);
#endif
                                      } else { // Not enough info data given
#ifdef DEBUG_D_UART_ERROR
                                        snprintf(UartBuffer, UARTBUFFERSIZE, " ## %d NOT ENOUGH INFO GIVEN\n", Port);
                                        UartDebug(UartBuffer);
#endif
                                        UartPort[Port].State = UART_INFO_ERROR;

                                      }
                                    } else { // Format invalid
#ifdef DEBUG_D_UART_ERROR
                                      snprintf(UartBuffer, UARTBUFFERSIZE, " ## %d FORMAT ERROR\n", Port);
                                      UartDebug(UartBuffer);
#endif
                                      UartPort[Port].State = UART_INFO_ERROR;

                                    }
                                  } else { // Mode invalid
#ifdef DEBUG_D_UART_ERROR
                                    snprintf(UartBuffer, UARTBUFFERSIZE, " f  %d FORMAT = %u * %u  %u.%u\n", Port, TypeData[Port][Mode].DataSets, TypeData[Port][Mode].Format, TypeData[Port][Mode].Figures, TypeData[Port][Mode].Decimals);
                                    UartDebug(UartBuffer);
#endif
                                    UartPort[Port].State = UART_INFO_ERROR;
                                  }
                                } else { // No more modes left
#ifdef DEBUG_D_UART_ERROR
                                  snprintf(UartBuffer, UARTBUFFERSIZE, " ## %d TYPES ERROR\n", Port);
                                  UartDebug(UartBuffer);
#endif
                                  UartPort[Port].State = UART_INFO_ERROR;

                                }
                              } else { // Data sets invalid
#ifdef DEBUG_D_UART_ERROR
                                snprintf(UartBuffer, UARTBUFFERSIZE, " f  %d FORMAT = %u * %u  %u.%u\n", Port, TypeData[Port][Mode].DataSets, TypeData[Port][Mode].Format, TypeData[Port][Mode].Figures, TypeData[Port][Mode].Decimals);
                                UartDebug(UartBuffer);
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
                  UartDebug(UartBuffer);
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
                    UartDebug(UartBuffer);
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
              UartDebug(UartBuffer);

              for (Tmp = 0;Tmp < UartPort[Port].InLength;Tmp++) {
                snprintf(UartBuffer, UARTBUFFERSIZE, "%02X", TmpBuffer[Tmp] & 0xFF);
                UartDebug(UartBuffer);
              }
              snprintf(UartBuffer, UARTBUFFERSIZE, "]%02X\n", UartPort[Port].Check & 0xFF);
              UartDebug(UartBuffer);
#endif
#ifndef DISABLE_UART_DATA_ERROR
              if (++UartPort[Port].DataErrors >= UART_ALLOWABLE_DATA_ERRORS) {
#ifdef DEBUG_D_UART_ERROR
                snprintf(UartBuffer, UARTBUFFERSIZE, " ## %d No valid data in %d messages\n", Port, UartPort[Port].DataErrors);
                UartDebug(UartBuffer);
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
              UartDebug(UartBuffer);

              for (Tmp = 0;Tmp < UartPort[Port].InLength;Tmp++) {
                snprintf(UartBuffer, UARTBUFFERSIZE, "%02X", TmpBuffer[Tmp] & 0xFF);
                UartDebug(UartBuffer);
              }
              snprintf(UartBuffer, UARTBUFFERSIZE, "]%02X\n", UartPort[Port].Check & 0xFF);
              UartDebug(UartBuffer);
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
              UartDebug(UartBuffer);

              for (Tmp = 0;Tmp < UartPort[Port].InLength;Tmp++) {
                snprintf(UartBuffer, UARTBUFFERSIZE, "%02X", TmpBuffer[Tmp] & 0xFF);
                UartDebug(UartBuffer);
              }
              snprintf(UartBuffer, UARTBUFFERSIZE, "]%02X\n", UartPort[Port].Check & 0xFF);
              UartDebug(UartBuffer);
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
              UartDebug(UartBuffer);

              for (Tmp = 1;Tmp < (UartPort[Port].OutLength - 1);Tmp++) {
                snprintf(UartBuffer, UARTBUFFERSIZE, "%02X", UartPort[Port].OutBuffer[Tmp] & 0xFF);
                UartDebug(UartBuffer);
              }
              snprintf(UartBuffer, UARTBUFFERSIZE, "]%02X\n", UartPort[Port].OutBuffer[Tmp] & 0xFF);
              UartDebug(UartBuffer);
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
                  UartDebug(UartBuffer);
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
                UartDebug(UartBuffer);
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
              UartDebug(UartBuffer);

              for (Tmp = 1;Tmp < (UartPort[Port].OutLength - 1);Tmp++) {
                snprintf(UartBuffer, UARTBUFFERSIZE, "%02X", UartPort[Port].OutBuffer[Tmp] & 0xFF);
                UartDebug(UartBuffer);
              }
              snprintf(UartBuffer, UARTBUFFERSIZE, "]%02X\n", UartPort[Port].OutBuffer[Tmp] & 0xFF);
              UartDebug(UartBuffer);
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
            UartDebug(UartBuffer);
            for (Mode = 0;Mode < MAX_DEVICE_MODES;Mode++) {
              UartBuffer[Mode] = Changed[Port][Mode] + '0';
            }
            UartBuffer[Mode++] = '\r';
            UartBuffer[Mode++] = '\n';
            UartBuffer[Mode]   = 0;
            UartDebug(UartBuffer);
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
            UartDebug(UartBuffer);
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
        UartDebug(UartBuffer);
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
							UartDebug(UartBuffer);
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
			UartDebug(UartBuffer);
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
		UartDebug(UartBuffer);
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
		UartDebug(UartBuffer);
		UartDebug(UartBuffer);
		snprintf(UartBuffer, UARTBUFFERSIZE, "-----------------------------------------------------------------\n");
		UartDebug(UartBuffer);
		snprintf(UartBuffer, UARTBUFFERSIZE, "    UART DUMP\n");
		UartDebug(UartBuffer);
		snprintf(UartBuffer, UARTBUFFERSIZE, "-----------------------------------------------------------------\n");
		UartDebug(UartBuffer);
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

static int Device1Init(struct device *parent)
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
			UartDebug(UartBuffer);
#endif
		} else {
			UartPortDisable(Tmp);
		}
	}

	Device1.parent = parent;
	ret = misc_register(&Device1);
	if (ret)
		goto err1;

	Device1Time = ktime_set(0, UART_TIMER_RESOLUTION * 100000);
	tasklet_hrtimer_init(&Device1Timer, Device1TimerInterrupt1,
			     CLOCK_MONOTONIC, HRTIMER_MODE_REL);

	tasklet_hrtimer_start(&Device1Timer, Device1Time, HRTIMER_MODE_REL);

	return 0;

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

	tasklet_hrtimer_cancel(&Device1Timer);

	misc_deregister(&Device1);

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

static int d_uart_open(struct tty_struct *tty)
{
	struct ktermios old_termios = tty->termios;
	struct d_uart_ldisc_data *data = NULL;
	int i;

	for (i = 0; i < INPUTS; i++) {
		const char *n1 = Device1Lms2012Compat->tty_names[i];
		const char *n2 = dev_name(tty->dev);

		if (!n1 || !n2)
			continue;
		if (!strcmp(n1, n2)) {
			data = &d_uart_ldisc_data[i];
			memset(data, 0, sizeof(*data));
			data->tty = tty;
			data->port = i;
			break;
		}
	}

	if (!data) {
		pr_err("%s: Could not find matching tty\n", __func__);
		return -ENXIO;
	}

	tty->disc_data = data;

	/* set baud rate and other port settings */
	down_write(&tty->termios_rwsem);
	tty->termios.c_iflag &=
		~(IGNBRK	/* disable ignore break */
		| BRKINT	/* disable break causes interrupt */
		| PARMRK	/* disable mark parity errors */
		| ISTRIP	/* disable clear high bit of input characters */
		| INLCR		/* disable translate NL to CR */
		| IGNCR		/* disable ignore CR */
		| ICRNL		/* disable translate CR to NL */
		| IXON);	/* disable enable XON/XOFF flow control */

	/* disable postprocess output characters */
	tty->termios.c_oflag &= ~OPOST;

	tty->termios.c_lflag &=
		~(ECHO		/* disable echo input characters */
		| ECHONL	/* disable echo new line */
		| ICANON	/* disable erase, kill, werase, and rprnt
				   special characters */
		| ISIG		/* disable interrupt, quit, and suspend special
				   characters */
		| IEXTEN);	/* disable non-POSIX special characters */

	/* 2400 baud, 8bits, no parity, 1 stop */
	tty->termios.c_cflag = B2400 | CS8 | CREAD | HUPCL | CLOCAL;
	tty->ops->set_termios(tty, &old_termios);
	up_write(&tty->termios_rwsem);
	tty->ops->tiocmset(tty, 0, ~0); /* clear all */

	tty->receive_room = 65536;

	/* flush any existing data in the buffer */
	if (tty->ldisc->ops->flush_buffer)
		tty->ldisc->ops->flush_buffer(tty);
	tty_driver_flush_buffer(tty);

	return 0;
}

static void d_uart_close(struct tty_struct *tty)
{
	struct d_uart_ldisc_data *data = tty->disc_data;

	data->tty = NULL;
	tty->disc_data = NULL;
}

static struct tty_ldisc_ops d_uart_ldisc = {
	.magic			= TTY_LDISC_MAGIC,
	.name			= "n_d_uart",
	.open			= d_uart_open,
	.close			= d_uart_close,
	.ioctl			= tty_mode_ioctl,
	.receive_buf		= d_uart_receive_buf,
	.owner			= THIS_MODULE,
};

static int d_uart_probe(struct platform_device *pdev)
{
	int ret;

	ret = Device1Init(&pdev->dev);

	/*
	 * Technically, this should be in module init, but since there is only
	 * on device, it should be safe to register the line discipline here.
	 */
	ret = tty_register_ldisc(N_D_UART, &d_uart_ldisc);
	if (ret) {
		pr_err("Could not register d_uart line discipline. (%d)\n", ret);
		Device1Exit();
		return ret;
	}

	pr_info("d_uart registered\n");

	return ret;
}

static int d_uart_remove(struct platform_device *pdev)
{
	tty_unregister_ldisc(N_D_UART);
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
