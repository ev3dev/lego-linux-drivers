/*
 * LEGO® MINDSTORMS EV3
 *
 * Copyright (C) 2010-2013 The LEGO Group
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
 *
 * As a special exception, if other files instantiate templates or use macros or
 * inline functions from this file, or you  compile this file and link it with
 * other works to produce a work based on this file, this file does not by itself
 * cause the resulting work to be covered by the GNU General Public License.
 * However the source code for this file must still be made available in accordance
 * with section (3) of the GNU General Public License.
 *
 */


/*
 * This file includes parts of lms2912.h, bytecodes.h and lmstypes.h from
 * lms2012-compat. DO NOT CHANGE THIS FILE unless lms2012-compat has been
 * changed as well.
 */


#ifndef   LMS2012_H_
#define   LMS2012_H_

#include <linux/clk.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/pwm.h>

//        BASIC DATA TYPES

typedef   unsigned char         UBYTE;  //!< Basic Type used to symbolise 8  bit unsigned values
typedef   unsigned short        UWORD;  //!< Basic Type used to symbolise 16 bit unsigned values
typedef   unsigned int          ULONG;  //!< Basic Type used to symbolise 32 bit unsigned values

typedef   signed char           SBYTE;  //!< Basic Type used to symbolise 8  bit signed values
typedef   signed short          SWORD;  //!< Basic Type used to symbolise 16 bit signed values
typedef   signed int            SLONG;  //!< Basic Type used to symbolise 32 bit signed values

typedef   float                 FLOAT;  //!< Basic Type used to symbolise 32 bit floating point values

//        VM DATA TYPES

typedef   SBYTE                 DATA8;  //!< VM Type for 1 byte signed value
typedef   SWORD                 DATA16; //!< VM Type for 2 byte signed value
typedef   SLONG                 DATA32; //!< VM Type for 4 byte signed value
typedef   FLOAT                 DATAF;  //!< VM Type for 4 byte floating point value

// Hardware

#define   OUTPUTS               4                     //!< Number of output ports in the system
#define   INPUTS                4                     //!< Number of input  ports in the system

#define   MAX_DEVICE_TYPE       127                   //!< Highest type number (positive)
#define   MAX_VALID_TYPE        101                   //!< Highest valid type
#define   MAX_DEVICE_MODES      8                     //!< Max number of modes in one device
#define   MAX_DEVICE_DATASETS   8                     //!< Max number of data sets in one device
#define   MAX_DEVICE_DATALENGTH 32                    //!< Max device data length

#define   LOGBUFFER_SIZE        1000                  //!< Min log buffer size
#define   DEVICE_LOGBUF_SIZE    300                   //!< Device log buffer size (black layer buffer)
#define   MIN_LIVE_UPDATE_TIME  10                    //!< [mS] Min sample time when live update

#define   ADC_REF               5000                  //!< [mV]  maximal value on ADC
#define   ADC_RES               4095                  //!< [CNT] maximal count on ADC

#define   DEVICE_UPDATE_TIME    1000000               //!< Min device (sensor) update time [nS]

#define   PWM_DEVICE            "lms_pwm"             //!< PWM device name
#define   MOTOR_DEVICE          "lms_motor"           //!< TACHO device name
#define   ANALOG_DEVICE         "lms_analog"          //!< ANALOG device name
#define   DCM_DEVICE            "lms_dcm"             //!< DCM device name
#define   UART_DEVICE           "lms_uart"            //!< UART device name
#define   IIC_DEVICE            "lms_iic"             //!< IIC device name

#define   VtoC(V)               ((UWORD)(((V) * ADC_RES) / ADC_REF))
#define   CtoV(C)               ((UWORD)(((C) * ADC_REF) / ADC_RES))
#define   MtoV(M)               ((UWORD)(((M) * ADC_REF * 100) / (ADC_RES * 52)))

typedef   enum
{
  OK            = 0,                    //!< No errors to report
  BUSY          = 1,                    //!< Busy - try again
  FAIL          = 2,                    //!< Something failed
  STOP          = 4,                    //!< Stopped
  START         = 8                     //!< Start
}
RESULT;

typedef   enum
{
  CONN_UNKNOWN                  = 111,  //!< Connection is fake (test)

  CONN_DAISYCHAIN               = 117,  //!< Connection is daisy chained
  CONN_NXT_COLOR                = 118,  //!< Connection type is NXT color sensor
  CONN_NXT_DUMB                 = 119,  //!< Connection type is NXT analog sensor
  CONN_NXT_IIC                  = 120,  //!< Connection type is NXT IIC sensor

  CONN_INPUT_DUMB               = 121,  //!< Connection type is LMS2012 input device with ID resistor
  CONN_INPUT_UART               = 122,  //!< Connection type is LMS2012 UART sensor

  CONN_OUTPUT_DUMB              = 123,  //!< Connection type is LMS2012 output device with ID resistor
  CONN_OUTPUT_INTELLIGENT       = 124,  //!< Connection type is LMS2012 output device with communication
  CONN_OUTPUT_TACHO             = 125,  //!< Connection type is LMS2012 tacho motor with series ID resistance

  CONN_NONE                     = 126,  //!< Port empty or not available
  CONN_ERROR                    = 127,  //!< Port not empty and type is invalid
}
CONN;

#define     TYPE_NAME_LENGTH    11
#define     SYMBOL_LENGTH       4       //!< Symbol leng th (not including zero)

typedef   enum
{
  MODE_KEEP                     =  -1,  //!< Mode value that won't change mode in byte codes (convenient place to define)
  TYPE_KEEP                     =   0,  //!< Type value that won't change type in byte codes

  // Types defined in "typedata.rcf"
  TYPE_NXT_TOUCH                =   1,  //!< Device is NXT touch sensor
  TYPE_NXT_LIGHT                =   2,  //!< Device is NXT light sensor
  TYPE_NXT_SOUND                =   3,  //!< Device is NXT sound sensor
  TYPE_NXT_COLOR                =   4,  //!< Device is NXT color sensor
  TYPE_NXT_ULTRASONIC           =   5,  //!< Device is NXT ultra sonic sensor
  TYPE_NXT_TEMPERATURE          =   6,  //!< Device is NXT temperature sensor
  TYPE_TACHO                    =   7,  //!< Device is EV3/NXT tacho motor
  TYPE_MINITACHO                =   8,  //!< Device is EV3 mini tacho motor
  TYPE_NEWTACHO                 =   9,  //!< Device is EV3 new tacho motor

  TYPE_TOUCH                    =  16,  //!< Device is EV3 touch sensor

  // Types defined in known EV3 digital devices
  TYPE_COLOR                    =  29,  //!< Device is EV3 color sensor
  TYPE_ULTRASONIC               =  30,  //!< Device is EV3 ultra sonic sensor
  TYPE_GYRO                     =  32,  //!< Device is EV3 gyro sensor
  TYPE_IR                       =  33,  //!< Device is EV3 IR sensor

  // Type range reserved for third party devices
  TYPE_THIRD_PARTY_START        =  50,
  TYPE_THIRD_PARTY_END          =  98,

  // Special types
  TYPE_ENERGYMETER              =  99,  //!< Device is energy meter
  TYPE_IIC_UNKNOWN              = 100,  //!< Device type is not known yet
  TYPE_NXT_TEST                 = 101,  //!< Device is a NXT ADC test sensor

  TYPE_NXT_IIC                  = 123,  //!< Device is NXT IIC sensor
  TYPE_TERMINAL                 = 124,  //!< Port is connected to a terminal
  TYPE_UNKNOWN                  = 125,  //!< Port not empty but type has not been determined
  TYPE_NONE                     = 126,  //!< Port empty or not available
  TYPE_ERROR                    = 127,  //!< Port not empty and type is invalid
}
TYPE;

typedef   struct // if data type changes - remember to change "cInputTypeDataInit" !
{
  SBYTE     Name[TYPE_NAME_LENGTH + 1]; //!< Device name
  DATA8     Type;                       //!< Device type
  DATA8     Connection;
  DATA8     Mode;                       //!< Device mode
  DATA8     DataSets;
  DATA8     Format;
  DATA8     Figures;
  DATA8     Decimals;
  DATA8     Views;
  DATAF     RawMin;                     //!< Raw minimum value      (e.c. 0.0)
  DATAF     RawMax;                     //!< Raw maximum value      (e.c. 1023.0)
  DATAF     PctMin;                     //!< Percent minimum value  (e.c. -100.0)
  DATAF     PctMax;                     //!< Percent maximum value  (e.c. 100.0)
  DATAF     SiMin;                      //!< SI unit minimum value  (e.c. -100.0)
  DATAF     SiMax;                      //!< SI unit maximum value  (e.c. 100.0)
  UWORD     InvalidTime;                //!< mS from type change to valid data
  UWORD     IdValue;                    //!< Device id value        (e.c. 0 ~ UART)
  DATA8     Pins;                       //!< Device pin setup
  SBYTE     Symbol[SYMBOL_LENGTH + 1];  //!< SI unit symbol
  UWORD     Align;
}
TYPES;

#define     TYPE_PARAMETERS         19  //!< Number of members in the structure above
#define     MAX_DEVICE_INFOLENGTH   54  //!< Number of bytes in the structure above (can not be changed)


#ifndef   DISABLE_OLD_COLOR

#define   COLORS                        4
#define   CALPOINTS                     3

typedef   struct
{
  ULONG   Calibration[CALPOINTS][COLORS];
  UWORD   CalLimits[CALPOINTS - 1];
  UWORD   Crc;
  UWORD   ADRaw[COLORS];
  UWORD   SensorRaw[COLORS];
}
COLORSTRUCT;

#endif /* DISABLE_OLD_COLOR */

typedef   struct
{
  DATA16  InPin1[INPUTS];         //!< Analog value at input port connection 1
  DATA16  InPin6[INPUTS];         //!< Analog value at input port connection 6
  DATA16  OutPin5[OUTPUTS];       //!< Analog value at output port connection 5
  DATA16  BatteryTemp;            //!< Battery temperature
  DATA16  MotorCurrent;           //!< Current flowing to motors
  DATA16  BatteryCurrent;         //!< Current flowing from the battery
  DATA16  Cell123456;             //!< Voltage at battery cell 1, 2, 3,4, 5, and 6
#ifndef DISABLE_FAST_DATALOG_BUFFER
  DATA16  Pin1[INPUTS][DEVICE_LOGBUF_SIZE];      //!< Raw value from analog device
  DATA16  Pin6[INPUTS][DEVICE_LOGBUF_SIZE];      //!< Raw value from analog device
  UWORD   Actual[INPUTS];
  UWORD   LogIn[INPUTS];
  UWORD   LogOut[INPUTS];
#endif
#ifndef   DISABLE_OLD_COLOR
  COLORSTRUCT  NxtCol[INPUTS];
#endif
  DATA16  OutPin5Low[OUTPUTS];    //!< Analog value at output port connection 5 when connection 6 is low

  DATA8   Updated[INPUTS];

  DATA8   InDcm[INPUTS];          //!< Input port device types
  DATA8   InConn[INPUTS];

  DATA8   OutDcm[OUTPUTS];        //!< Output port device types
  DATA8   OutConn[OUTPUTS];
}
ANALOG;

#define   UART_DATA_LENGTH        MAX_DEVICE_DATALENGTH
#define   UART_BUFFER_SIZE        64

typedef   struct
{
  TYPES   TypeData[INPUTS][MAX_DEVICE_MODES]; //!< TypeData

#ifndef DISABLE_FAST_DATALOG_BUFFER
  UWORD   Repeat[INPUTS][DEVICE_LOGBUF_SIZE];
  DATA8   Raw[INPUTS][DEVICE_LOGBUF_SIZE][UART_DATA_LENGTH];      //!< Raw value from UART device
  UWORD   Actual[INPUTS];
  UWORD   LogIn[INPUTS];
#else
  DATA8   Raw[INPUTS][UART_DATA_LENGTH];      //!< Raw value from UART device
#endif
  DATA8   Status[INPUTS];                     //!< Status
  DATA8   Output[INPUTS][UART_DATA_LENGTH];   //!< Bytes to UART device
  DATA8   OutputLength[INPUTS];
}
UART;


#define   UART_PORT_CHANGED       0x01        //!< Input port changed
#define   UART_DATA_READY         0x08        //!< Data is ready
#define   UART_WRITE_REQUEST      0x10        //!< Write request

typedef   struct
{
  DATA8     Connection[INPUTS];
  DATA8     Type[INPUTS];
  DATA8     Mode[INPUTS];
}
DEVCON;

typedef   struct
{
  TYPES   TypeData;
  DATA8   Port;
  DATA8   Mode;
}
UARTCTL;

#define   UART_SET_CONN           _IOWR('u',0,DEVCON)
#define   UART_READ_MODE_INFO     _IOWR('u',1,UARTCTL)
#define   UART_NACK_MODE_INFO     _IOWR('u',2,UARTCTL)
#define   UART_CLEAR_CHANGED      _IOWR('u',3,UARTCTL)

#define   IIC_DATA_LENGTH         MAX_DEVICE_DATALENGTH
#define   IIC_NAME_LENGTH         8

typedef   struct
{
  TYPES   TypeData[INPUTS][MAX_DEVICE_MODES]; //!< TypeData

#ifndef DISABLE_FAST_DATALOG_BUFFER
  UWORD   Repeat[INPUTS][DEVICE_LOGBUF_SIZE];
  DATA8   Raw[INPUTS][DEVICE_LOGBUF_SIZE][IIC_DATA_LENGTH];      //!< Raw value from IIC device
  UWORD   Actual[INPUTS];
  UWORD   LogIn[INPUTS];
#else
  DATA8   Raw[INPUTS][IIC_DATA_LENGTH];      //!< Raw value from IIC device
#endif
  DATA8   Status[INPUTS];                     //!< Status
  DATA8   Changed[INPUTS];
  DATA8   Output[INPUTS][IIC_DATA_LENGTH];    //!< Bytes to IIC device
  DATA8   OutputLength[INPUTS];
}
IIC;


#define   IIC_PORT_CHANGED       0x01         //!< Input port changed
#define   IIC_DATA_READY         0x08         //!< Data is ready
#define   IIC_WRITE_REQUEST      0x10         //!< Write request

typedef   struct
{
  TYPES   TypeData;
  DATA8   Port;
  DATA8   Mode;
}
IICCTL;

typedef   struct
{
  RESULT  Result;
  DATA8   Port;
  DATA8   Repeat;
  DATA16  Time;
  DATA8   WrLng;
  DATA8   WrData[IIC_DATA_LENGTH];
  DATA8   RdLng;
  DATA8   RdData[IIC_DATA_LENGTH];
}
IICDAT;

typedef   struct
{
  DATA8   Port;
  DATA16  Time;
  DATA8   Type;
  DATA8   Mode;
  DATA8   Manufacturer[IIC_NAME_LENGTH + 1];
  DATA8   SensorType[IIC_NAME_LENGTH + 1];
  DATA8   SetupLng;
  ULONG   SetupString;
  DATA8   PollLng;
  ULONG   PollString;
  DATA8   ReadLng;
}
IICSTR;

#define   IIC_SET_CONN            _IOWR('i',2,DEVCON)
#define   IIC_READ_TYPE_INFO      _IOWR('i',3,IICCTL)
#define   IIC_SETUP               _IOWR('i',5,IICDAT)
#define   IIC_SET                 _IOWR('i',6,IICSTR)
#define   IIC_READ_STATUS         _IOWR('i',7,IICDAT)
#define   IIC_READ_DATA           _IOWR('i',8,IICDAT)
#define   IIC_WRITE_DATA          _IOWR('i',9,IICDAT)

/* lms2012_compat stuff */

enum InputSpiPins {
  ADCMOSI,
  ADCMISO,
  ADCCLK,
  ADCCS,
  ADC_SPI_PINS
};

enum InputPortPins {
  INPUT_PORT_PIN5,
  INPUT_PORT_PIN6,
  INPUT_PORT_BUF,
  INPUT_PORT_PINS,
  /* the following values are not used as array indexes */
  INPUT_PORT_PIN2,
  INPUT_PORT_VALUE
};

enum OutputPortPins {
  OUTPUT_PORT_PIN1,
  OUTPUT_PORT_PIN2,
  OUTPUT_PORT_PIN5W,
  OUTPUT_PORT_PIN5R,
  OUTPUT_PORT_PIN6,
  OUTPUT_PORT_PINS,
  OUTPUT_PORT_VALUE
};

#define   INPUTADCPORTS   12
#define   INPUTADCPOWERS  4
#define   INPUTADC        (INPUTADCPORTS + INPUTADCPOWERS)

struct lms2012_compat {
    struct platform_device *d_analog;
    struct platform_device *d_iic;
    struct platform_device *d_uart;
    struct platform_device *d_pwm;
    u32 adc_map[INPUTADC];
    struct pinctrl *pinctrl[INPUTS];
    struct pinctrl_state *pinctrl_default[INPUTS];
    struct pinctrl_state *pinctrl_i2c[INPUTS];
    struct gpio_descs *spi_pins;
    struct gpio_desc *in_pin1[INPUTS];
    struct gpio_desc *in_pin2[INPUTS];
    struct gpio_descs *in_pins[INPUTS];
    struct i2c_adapter *i2c_adapter[INPUTS];
    const char *tty_names[INPUTS];
    struct gpio_descs *out_pins[OUTPUTS];
    struct pwm_device *out_pwms[OUTPUTS];
};

struct device *lms2012_compat_get(void);

#endif /* LMS2012_H_ */
