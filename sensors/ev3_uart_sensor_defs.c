/*
 * LEGO MINDSTORMS EV3 UART Sensor driver
 *
 * Copyright (C) 2013-2016 David Lechner <david@lechnology.com>
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
 * Documentation is automatically generated from this struct, so formatting is
 * very important. Make sure any new sensors have the same layout. The comments
 * are also parsed to provide more information for the documentation. The
 * parser can be found in the Documentation/json/ directory.
 */

#include "ev3_uart_sensor.h"

const struct ev3_uart_sensor_info ev3_uart_sensor_defs[] = {
	[LEGO_EV3_COLOR] = {
		/**
		 * @vendor_name: LEGO
		 * @vendor_part_number: 45506
		 * @vendor_part_name: EV3 Color Sensor
		 * @vendor_website: https://education.lego.com/en-us/products/ev3-color-sensor/45506
		 */
		.name		= LEGO_EV3_COLOR_NAME,
		.type_id	= LEGO_EV3_COLOR_TYPE_ID,
		.num_modes	= 6,
		.num_view_modes	= 3,
		.mode_info	= {
			[0] = {
				/**
				 * @description: Reflected light - sets LED color to red
				 * @value0: Reflected light intensity (0 to 100)
				 * @units_description: percent
				 */
				.name		= "COL-REFLECT",
				.data_sets	= 1,
				.data_type	= LEGO_SENSOR_DATA_S8,
				.units		= "pct",
			},
			[1] = {
				/**
				 * @description: Ambient light - sets LED color to blue (dimly lit)
				 * @value0: Ambient light intensity (0 to 100)
				 * @units_description: percent
				 */
				.name		= "COL-AMBIENT",
				.data_sets	= 1,
				.data_type	= LEGO_SENSOR_DATA_S8,
				.units		= "pct",
			},
			[2] = {
				/**
				 * .. [#lego-ev3-color-mode2-value0] Color values:
				 *
				 *    ======= =========
				 *     Value   Color
				 *    ======= =========
				 *     0       none
				 *     1       black
				 *     2       blue
				 *     3       green
				 *     4       yellow
				 *     5       red
				 *     6       white
				 *     7       brown
				 *    ======= =========
				 *
				 * @description: Color - sets LED color to white (all LEDs rapidly cycling)
				 * @value0: Detected color (0 to 7)
				 * @value0_footnote: [#lego-ev3-color-mode2-value0]_
				 * @units_description: color
				 */
				.name		= "COL-COLOR",
				.data_sets	= 1,
				.data_type	= LEGO_SENSOR_DATA_S8,
				.units		= "col",
			},
			[3] = {
				/**
				 * .. [#lego-ev3-color-mode3-values] This mode
				 *    provides direct access to the measurements
				 *    done by the ADC inside the sensor. It
				 *    works by continuously flashing the red LED
				 *    on and off and measuring the current
				 *    flowing through a bypass capacitor. It is
				 *    not possible to measure true ambient light
				 *    this way, but the LED off measurement
				 *    works as a reference for the LED on
				 *    measurement. Higher photodiode
				 *    illumination will cause lower ADC
				 *    measurements, so to obtain reflected light
				 *    intensity similar to ``COL-REFLECT``, it
				 *    is necessary to do
				 *    ``reflect = value1 - value0``.
				 *
				 * .. [#lego-ev3-color-mode3-value1] While
				 *    this value cannot be used to measure
				 *    still ambient light, it can be used to
				 *    measure *changes* in external
				 *    illumination. When it increases, the
				 *    this value goes below its baseline and
				 *    vice versa.
				 *
				 * @description: Raw Reflected Light - sets LED color to red
				 * @value0: ADC reading for LED on (reflection) (0 to 1023)
				 * @value0_footnote: [#lego-ev3-color-mode3-values]_
				 * @value1: ADC reading for LED off (reference) (0 to 1023)
				 * @value1_footnote: [#lego-ev3-color-mode3-values]_ [#lego-ev3-color-mode3-value1]_
				 */
				.name		= "REF-RAW",
				.data_sets	= 2,
				.data_type	= LEGO_SENSOR_DATA_S16,
			},
			[4] = {
				/**
				 * .. [#lego-ev3-color-mode4-value012] These
				 *    values represent the difference between
				 *    the raw sensor ADC measurements when the
				 *    LEDs are off and when they are on. They
				 *    work similar to the ``COL-REFLECT`` values
				 *    but they are not precalibrated.
				 *
				 * .. [#lego-ev3-color-mode4-value3] This value
				 *    is similar to the ``REF-RAW`` ambient
				 *    ``value1``. It is the value measured by
				 *    the ADC inside the sensor when all LEDs
				 *    are off, so it works as a reference point
				 *    for the first three values. However here
				 *    the ``value0``, ``value1`` and ``value2``
				 *    are sent as relative to this value by the
				 *    sensor automatically, so it is not
				 *    necessary to use this value at all.
				 *
				 * @description: Raw Color Components - sets LED color to white (all LEDs rapidly cycling)
				 * @value0: Reflected red light intensity (0 to 1023)
				 * @value0_footnote: [#lego-ev3-color-mode4-value012]_
				 * @value1: Reflected green light intensity (0 to 1023)
				 * @value1_footnote: [#lego-ev3-color-mode4-value012]_
				 * @value2: Reflected blue light intensity (0 to 1023)
				 * @value2_footnote: [#lego-ev3-color-mode4-value012]_
				 * @value3: ADC reading for LEDs off (reference) (0 to 1023)
				 * @value3_footnote: [#lego-ev3-color-mode4-value3]_
				 */
				.name		= "RGB-RAW",
				.data_sets	= 4,
				.data_type	= LEGO_SENSOR_DATA_S16,
			},
			[5] = {
				/**
				 * .. [#lego-ev3-color-mode5] This is a factory
				 *    calibration mode. Writing
				 *    ``LEGO-FAC-CAL-1`` directly to the sensor
				 *    shortly after switching to this mode will
				 *    trigger hardware recalibration. Then,
				 *    after circa 500 ms, the new calibration
				 *    parameters can be read from the sensor.
				 *    Proceed only at your own risk, this will
				 *    permanently overwrite the accurate
				 *    factory-provided calibration data!
				 *    If you do not write the unlock string
				 *    quickly enough, the sensor will time out
				 *    and reset. This happens because the
				 *    sensor does not respond to the keep-alive
				 *    packets sent from the EV3 brick while
				 *    in this mode.
				 *
				 * .. [#lego-ev3-color-mode5-values] The
				 *    physical meaning of these values is not
				 *    known. They are most probably used
				 *    internally by the sensor firmware to
				 *    correctly scale its measurements.
				 *
				 * @name_footnote: [#lego-ev3-color-mode5]_
				 * @description: Factory Calibration - turns off all LEDs, then flashes all LEDs briefly when recalibrating
				 * @value0: Red color calibration multiplier??? (0 to 65535)
				 * @value0_footnote: [#lego-ev3-color-mode5-values]_
				 * @value1: Green color calibration multiplier??? (0 to 65535)
				 * @value1_footnote: [#lego-ev3-color-mode5-values]_
				 * @value2: Blue color calibration multiplier??? (0 to 65535)
				 * @value2_footnote: [#lego-ev3-color-mode5-values]_
				 */
				.name		= "COL-CAL",
				.data_sets	= 4,
				.num_values	= 3,
				.data_type	= LEGO_SENSOR_DATA_U16,
			},
		},
	},
	[LEGO_EV3_ULTRASONIC] = {
		/**
		 * @vendor_name: LEGO
		 * @vendor_part_number: 45504
		 * @vendor_part_name: EV3 Ultrasonic Sensor
		 * @vendor_website: https://education.lego.com/en-us/products/ev3-ultrasonic-sensor/45504
		 */
		.name		= LEGO_EV3_ULTRASONIC_NAME,
		.type_id	= LEGO_EV3_ULTRASONIC_TYPE_ID,
		.num_modes	= 7,
		.num_view_modes	= 3,
		.mode_info	= {
			[0] = {
				/**
				 * @description: Continuous measurement - sets LEDs on, steady
				 * @value0: Distance (0-2550)
				 * @units_description: centimeters
				 */
				.name		= "US-DIST-CM",
				.data_sets	= 1,
				.data_type	= LEGO_SENSOR_DATA_S16,
				.units		= "cm",
				.decimals	= 1,
			},
			[1] = {
				/**
				 * @description: Continuous measurement - sets LEDs on, steady
				 * @value0: Distance (0-1003)
				 * @units_description: inches
				 */
				.name		= "US-DIST-IN",
				.data_sets	= 1,
				.data_type	= LEGO_SENSOR_DATA_S16,
				.units		= "in",
				.decimals	= 1,
			},
			[2] = {
				/**
				 * .. [#lego-ev3-us-mode2-value0] A value of 1
				 *    indicates that another ultrasonic sensor
				 *    has been detected. A 1 can also be
				 *    triggered by a loud noise such as clapping.
				 *
				 * @description: Listen - sets LEDs on, blinking
				 * @value0: Presence (0-1)
				 * @value0_footnote: [#lego-ev3-us-mode2-value0]_
				 */
				.name		= "US-LISTEN",
				.data_sets	= 1,
				.data_type	= LEGO_SENSOR_DATA_S8,
			},
			[3] = {
				/**
				 * .. [#lego-ev3-us-mode3-value0] A measurement
				 *    is taken when the mode is set and ``value0``
				 *    will not change after this. To take another
				 *    measurement set the mode again. **NOTE:**
				 *    If you write the mode too frequently (e.g.
				 *    every 100msec), the sensor will sometimes
				 *    lock up and writing to the ``mode`` attribute
				 *    will return an error. A delay of 250msec
				 *    between each write to the mode attribute
				 *    seems sufficient to keep the sensor from
				 *    locking up.
				 *
				 * @description: Single measurement - LEDs on momentarily when mode is set, then off
				 * @value0: Distance (0-2550)
				 * @value0_footnote: [#lego-ev3-us-mode3-value0]_
				 * @units_description: centimeters
				 */
				.name		= "US-SI-CM",
				.data_sets	= 1,
				.data_type	= LEGO_SENSOR_DATA_S16,
				.units		= "cm",
				.decimals	= 1,
			},
			[4] = {
				/**
				 * @description: Single measurement - sets LED on momentarily when mode is set, then off
				 * @value0: Distance (0-1003)
				 * @value0_footnote: [#lego-ev3-us-mode3-value0]_
				 * @units_description: inches
				 */
				.name		= "US-SI-IN",
				.data_sets	= 1,
				.data_type	= LEGO_SENSOR_DATA_S16,
				.units		= "in",
				.decimals	= 1,
			},
			[5] = {
				/**
				 * .. [#lego-ev3-us-mode5] Not sure what DC mode
				 *    stands for. Seems to work like the continuous
				 *    measurement modes.
				 *
				 * @name_footnote: [#lego-ev3-us-mode5]_
				 * @description: ??? - sets LEDs on, steady
				 * @value0: Distance (0-2550)
				 * @units_description: centimeters
				 */
				.name		= "US-DC-CM",
				.data_sets	= 1,
				.data_type	= LEGO_SENSOR_DATA_S16,
				.units		= "cm",
				.decimals	= 1,
			},
			[6] = {
				/**
				 * @name_footnote: [#lego-ev3-us-mode5]_
				 * @description: ??? sets LEDs on, steady
				 * @value0: Distance (0-1003)
				 * @units_description: inches
				 */
				.name		= "US-DC-IN",
				.data_sets	= 1,
				.data_type	= LEGO_SENSOR_DATA_S16,
				.units		= "in",
				.decimals	= 1,
			},
		},
	},
	[LEGO_EV3_GYRO] = {
		/**
		 * @vendor_name: LEGO
		 * @vendor_part_number: 45505
		 * @vendor_part_name: EV3 Gyro Sensor
		 * @vendor_website: https://education.lego.com/en-us/products/ev3-gyro-sensor-/45505
		 */
		.name		= LEGO_EV3_GYRO_NAME,
		.type_id	= LEGO_EV3_GYRO_TYPE_ID,
		.num_modes	= 7,
		.num_view_modes	= 3,
		.mode_info	= {
			[0] = {
				/**
				 * .. [#lego-ev3-gyro-mode0-value0-overflow] If
				 *    you spin around too many times in ``GYRO-ANG``
				 *    or ``GYRO-G&A`` mode, it will get stuck at
				 *    32767 or overflow through -32768 depending
				 *    on when the sensor was manufactured.
				 *
				 * .. [#lego-ev3-gyro-mode0-value0-direction]
				 *    Clockwise is positive when looking at the
				 *    side of the sensor with the arrows.
				 *
				 * @description: Angle
				 * @value0: Angle (-32768 to 32767)
				 * @value0_footnote: [#lego-ev3-gyro-mode0-value0-overflow]_ [#lego-ev3-gyro-mode0-value0-direction]_
				 * @units_description: degrees
				 */
				.name		= "GYRO-ANG",
				.data_sets	= 1,
				.data_type	= LEGO_SENSOR_DATA_S16,
				.units		= "deg",
			},
			[1] = {
				/**
				 * @description: Rotational Speed
				 * @value0: Rotational Speed (-440 to 440)
				 * @value0_footnote: [#lego-ev3-gyro-mode0-value0-direction]_
				 * @units_description: degrees per second
				 */
				.name		= "GYRO-RATE",
				.data_sets	= 1,
				.data_type	= LEGO_SENSOR_DATA_S16,
				.units		= "d/s",
			},
			[2] = {
				/**
				 * @description: Rotational Speed
				 * @value0: Rotational Speed (-1464 to 1535)
				 * @value0_footnote: [#lego-ev3-gyro-mode0-value0-direction]_
				 */
				.name		= "GYRO-FAS",
				.data_sets	= 1,
				.data_type	= LEGO_SENSOR_DATA_S16,
			},
			[3] = {
				/**
				 * .. [#lego-ev3-gyro-mode3] Older versions of
				 *    this sensor will get stuck in ``GYRO-G&A``
				 *    mode (date code ending with 2 or 3).
				 *    Attempting to set the mode again (including
				 *    setting ``GYRO-G&A`` mode) will result in
				 *    a timeout error. The sensor must be reset
				 *    by unplugging it or by changing the mode
				 *    of the `lego-port` device that the sensor
				 *    is connected to to a different mode.
				 *
				 * @name_footnote: [#lego-ev3-gyro-mode3]_
				 * @description: Angle and Rotational Speed
				 * @value0: Angle (-32768 to 32767)
				 * @value0_footnote: [#lego-ev3-gyro-mode0-value0-overflow]_ [#lego-ev3-gyro-mode0-value0-direction]_
				 * @value1: Rotational Speed (-440 to 440)
				 * @value1_footnote: [#lego-ev3-gyro-mode0-value0-direction]_
				 */
				.name		= "GYRO-G&A",
				.data_sets	= 2,
				.data_type	= LEGO_SENSOR_DATA_S16,
			},
			[4] = {
				/**
				 * @description: Calibration ???
				 * @value0: ???
				 * @value1: ???
				 * @value2: ???
				 * @value3: ???
				 */
				.name		= "GYRO-CAL",
				.data_sets	= 4,
				.data_type	= LEGO_SENSOR_DATA_S16,
			},
			[5] = {
				/**
				 * .. [#lego-ev3-gyro-mode5] This mode is not present
				 *    in older sensors (date code ending with 2 or 3).
				 *
				 * .. [#lego-ev3-gyro-mode5-value0-direction]
				 *    Clockwise is positive when looking at the
				 *    side of the sensor opposite the cable jack.
				 *
				 * @name_footnote: [#lego-ev3-gyro-mode5]_
				 * @description: Rotational Speed (2nd axis)
				 * @value0: Rotational Speed (-440 to 440)
				 * @value0_footnote: [#lego-ev3-gyro-mode5-value0-direction]_
				 * @units_description: degrees per second
				 */
				.name		= "TILT-RATE",
				.data_sets	= 1,
				.data_type	= LEGO_SENSOR_DATA_S16,
				.units		= "d/s",
			},
			[6] = {
				/**
				 * @name_footnote: [#lego-ev3-gyro-mode5]_
				 * @description: Angle (2nd axis)
				 * @value0: Angle (-32768 to 32767)
				 * @value0_footnote: [#lego-ev3-gyro-mode5-value0-direction]_
				 * @units_description: degrees
				 */
				.name		= "TILT-ANG",
				.data_sets	= 1,
				.data_type	= LEGO_SENSOR_DATA_S16,
				.units		= "deg",
			},
		},
	},
	[LEGO_EV3_INFRARED] = {
		/**
		 * @vendor_name: LEGO
		 * @vendor_part_number: 45509
		 * @vendor_part_name: EV3 Infrared Sensor
		 * @vendor_website: https://education.lego.com/en-us/products/ev3-infrared-sensor/45509
		 */
		.name		= LEGO_EV3_INFRARED_NAME,
		.type_id	= LEGO_EV3_INFRARED_TYPE_ID,
		.num_modes	= 6,
		.num_view_modes	= 3,
		.mode_info	= {
			[0] = {
				/**
				 * .. [#lego-ev3-ir-mode0-value0] 100% is
				 *    approximately 70cm/27in.
				 *
				 * @description: Proximity
				 * @value0: Distance (0 to 100)
				 * @value0_footnote: [#lego-ev3-ir-mode0-value0]_
				 * @units_description: percent
				 */
				.name		= "IR-PROX",
				.data_sets	= 1,
				.data_type	= LEGO_SENSOR_DATA_S8,
				.units		= "pct",
			},
			[1] = {
				/**
				 * .. [#lego-ev3-ir-mode1-value0] When looking
				 *    in the same direction as the sensor, -25
				 *    is far left and +25 is far right.
				 *
				 * .. [#lego-ev3-ir-mode1-value1] 100% is
				 *    approximately 200cm/78in.
				 *
				 * .. [#lego-ev3-ir-mode1-no-beacon] The absence
				 *    of a beacon on a channel can be detected
				 *    when distance == -128 (and heading == 0).
				 *
				 * @description: IR Seeker
				 * @value0: Channel 1 Heading (-25 to 25)
				 * @value0_footnote: [#lego-ev3-ir-mode1-value0]_
				 * @value1: Channel 1 Distance (-128 and 0 to 100)
				 * @value1_footnote: [#lego-ev3-ir-mode1-value1]_ [#lego-ev3-ir-mode1-no-beacon]_
				 * @value2: Channel 2 Heading (-25 to 25)
				 * @value2_footnote: [#lego-ev3-ir-mode1-value0]_
				 * @value3: Channel 2 Distance (-128 and 0 to 100)
				 * @value3_footnote: [#lego-ev3-ir-mode1-value1]_ [#lego-ev3-ir-mode1-no-beacon]_
				 * @value4: Channel 3 Heading (-25 to 25)
				 * @value4_footnote: [#lego-ev3-ir-mode1-value0]_
				 * @value5: Channel 3 Distance (-128 and 0 to 100)
				 * @value5_footnote: [#lego-ev3-ir-mode1-value1]_ [#lego-ev3-ir-mode1-no-beacon]_
				 * @value6: Channel 4 Heading (-25 to 25)
				 * @value6_footnote: [#lego-ev3-ir-mode1-value0]_
				 * @value7: Channel 4 Distance (-128 and 0 to 100)
				 * @value7_footnote: [#lego-ev3-ir-mode1-value1]_ [#lego-ev3-ir-mode1-no-beacon]_
				 * @units_description: percent
				 */
				.name		= "IR-SEEK",
				.data_sets	= 8,
				.data_type	= LEGO_SENSOR_DATA_S8,
				.units		= "pct",
			},
			[2] = {
				/**
				 * .. [#lego-ev3-ir-mode2-value0] Button values:
				 *
				 *    =======  =========================
				 *     Value    Description
				 *    =======  =========================
				 *     0        none
				 *     1        red up
				 *     2        red down
				 *     3        blue up
				 *     4        blue down
				 *     5        red up and blue up
				 *     6        red up and blue down
				 *     7        red down and blue up
				 *     8        red down and blue down
				 *     9        beacon mode on
				 *     10       red up and red down
				 *     11       blue up and blue down
				 *    =======  =========================
				 *
				 *    red == left and blue == right
				 *
				 *    Pressing more that 2 buttons at one time
				 *    is not supported. It will usually read 0.
				 *    Pressing an up/down button while beacon
				 *    mode is activated with turn off beacon mode.
				 *
				 * @description: IR Remote Control
				 * @value0: Channel 1 (0 to 11)
				 * @value0_footnote: [#lego-ev3-ir-mode2-value0]_
				 * @value1: Channel 2 (0 to 11)
				 * @value1_footnote: [#lego-ev3-ir-mode2-value0]_
				 * @value2: Channel 3 (0 to 11)
				 * @value2_footnote: [#lego-ev3-ir-mode2-value0]_
				 * @value3: Channel 4 (0 to 11)
				 * @value3_footnote: [#lego-ev3-ir-mode2-value0]_
				 * @units_description: button
				 */
				.name		= "IR-REMOTE",
				.data_sets	= 4,
				.data_type	= LEGO_SENSOR_DATA_S8,
				.units		= "btn",
			},
			[3] = {
				/**
				 * .. [#lego-ev3-ir-mode3-value0] Button values:
				 *
				 *    =============  ===========  =========  =========  =======
				 *     Value          Blue Down    Blue Up   Red Down   Red Up
				 *    =============  ===========  =========  =========  =======
				 *     262/270/384
				 *     287/279                                          X
				 *     300/292                                X
				 *     309/317                                X         X
				 *     330/322                     X
				 *     339/347                     X                    X
				 *     352/360                     X          X
				 *     377/369                     X          X         X
				 *     390/398        X
				 *     415/407        X                                 X
				 *     428/420        X                       X
				 *     437/445        X                       X         X
				 *     458/450        X            X
				 *     467/475        X            X                    X
				 *     480/488        X            X          X
				 *     505/497        X            X          X         X
				 *    =============  ===========  =========  =========  =======
				 *
				 *    X = button pressed
				 *
				 *    The most significant byte is always 0x01.
				 *    In the least significant byte, the 4 most
				 *    significant bits represent each button. Bit
				 *    7 is the blue down button, bit 6 is the
				 *    blue up button, bit 5 is the red down button,
				 *    bit 4 is the red up button. Beware that when
				 *    no buttons are pressed, bit 7 is set (value
				 *    == 384).
				 *
				 *    Example::
				 *
				 *        if (value == 384) {
				 *            // no buttons are pressed
				 *        } else {
				 *            if (value & 0x80)
				 *                // blue down button is pressed
				 *            if (value & 0x40)
				 *                // blue up button is pressed
				 *            if (value & 0x20)
				 *                // red down button is pressed
				 *            if (value & 0x10)
				 *                // red up button is pressed
				 *        }
				 *
				 *    Bits 0-3 seem to be some sort of checksum
				 *    or parity check. Bit 0 = bit 4, bit 1 =
				 *    ~(bit 5), bit 2 = ~(bit 6), bit 3 = 0 if
				 *    bits 0-2 are even or 1 if bits 0-2 are odd.
				 *
				 *    Also, when the beacon mode is active or for
				 *    about 1 second after any button is released
				 *    the value is 262/270.
				 *
				 *    This mode only works with the remote on
				 *    channel 1.
				 *
				 * @description: IR Remote Control
				 * @value0: Channel 1
				 * @value0_footnote: [#lego-ev3-ir-mode3-value0]_
				 */
				.name		= "IR-REM-A",
				.data_sets	= 1,
				.data_type	= LEGO_SENSOR_DATA_S16,
			},
			[4] = {
				/**
				 * .. [#lego-ev3-ir-mode4] ``IR-S-ALT`` mode is
				 *    not usable. When switching to this mode,
				 *    the sensor quits responding to the keep-alive
				 *    messages and the sensor resets.
				 *
				 * @name_footnote: [#lego-ev3-ir-mode4]_
				 * @description: Alternate IR Seeker ???
				 * @value0: ??? (0 to 100)
				 * @value1: ??? (0 to 100)
				 * @value2: ??? (0 to 100)
				 * @value3: ??? (0 to 100)
				 * @units_description: percent
				 */
				.name		= "IR-S-ALT",
				.data_sets	= 4,
				.data_type	= LEGO_SENSOR_DATA_S8,
				.units		= "pct",
			},
			[5] = {
				/**
				 * @description: Calibration ???
				 * @value0: ??? (0 to 1023)
				 * @value1: ??? (0 to 1023)
				 */
				.name		= "IR-CAL",
				.data_sets	= 2,
				.data_type	= LEGO_SENSOR_DATA_S32,
			},
		},
	},
	[FATCATLAB_ADC] = {
		/**
		 * @vendor_name: Fatcatlab
		 * @vendor_part_number: ADC Adapter
		 * @vendor_website: http://fatcatlab.com/product/adc-adapter
		 */
		.name		= FATCATLAB_ADC_NAME,
		.type_id	= FATCATLAB_ADC_TYPE_ID,
		.num_modes	= 3,
		.num_view_modes	= 2,
		.mode_info	= {
			[0] = {
				/**
				 * @description: Channel 1
				 * @value0: Voltage (0 to 3300)
				 * @units_description: millivolts
				 */
				.name		= "CH1-VOLTAGE",
				.data_sets	= 1,
				.data_type	= LEGO_SENSOR_DATA_S16,
				.units		= "mV",
			},
			[1] = {
				/**
				 * @description: Channel 2
				 * @value0: Voltage (0 to 3300)
				 * @units_description: millivolts
				 */
				.name		= "CH2-VOLTAGE",
				.data_sets	= 1,
				.data_type	= LEGO_SENSOR_DATA_S16,
				.units		= "mV",
			},
			[2] = {
				/**
				 * @description: Both Channels
				 * @value0: Channel 1 Voltage (0 to 3300)
				 * @value1: Channel 2 Voltage (0 to 3300)
				 * @units_description: millivolts
				 */
				.name		= "VOLTAGE",
				.data_sets	= 2,
				.data_type	= LEGO_SENSOR_DATA_S16,
				.units		= "mV",
			},
		},
	},
	[FATCATLAB_GESTURE] = {
		/**
		 * @vendor_name: Fatcatlab
		 * @vendor_part_number: Gesture Sensor
		 * @vendor_website: http://fatcatlab.com/product/gesture-sensor
		 */
		.name		= FATCATLAB_GESTURE_NAME,
		.type_id	= FATCATLAB_GESTURE_TYPE_ID,
		.num_modes	= 4,
		.num_view_modes	= 4,
		.mode_info	= {
			[0] = {
				/**
				 * .. [#fcl-gesture-mode0-value0] Gesture Values:
				 *
				 *    =======  =============
				 *     Value    Description
				 *    =======  =============
				 *     0        *none*
				 *     1        left
				 *     2        right
				 *     3        up
				 *     4        down
				 *     5        near
				 *     6        far
				 *    =======  =============
				 *
				 * @description: Gesture
				 * @value0: Gesture (0 to 6)
				 * @value0_footnote: [#fcl-gesture-mode0-value0]_
				 */
				.name		= "GESTURE",
				.data_sets	= 1,
				.data_type	= LEGO_SENSOR_DATA_S8,
			},
			[1] = {
				/**
				 * @description: Proximity
				 * @value0: Voltage (0 to 127)
				 */
				.name		= "PROXIMITY",
				.data_sets	= 1,
				.data_type	= LEGO_SENSOR_DATA_S8,
			},
			[2] = {
				/**
				 * @description: Color
				 * @value0: Red
				 * @value1: Green
				 * @value2: Blue
				 */
				.name		= "RGB-RAW",
				.data_sets	= 3,
				.data_type	= LEGO_SENSOR_DATA_S16,
			},
			[3] = {
				/**
				 * .. [#fcl-gesture-mode3] The ``CLEAR`` mode is
				 *    used to reset the value of the ``GESTURE``
				 *    mode back to zero.
				 *
				 * @description: Clear
				 * @value0: Always 1
				 * @name_footnote: [#fcl-gesture-mode3]_
				 */
				.name		= "CLEAR",
				.data_sets	= 1,
				.data_type	= LEGO_SENSOR_DATA_S8,
			},
		},
	},
	[FATCATLAB_LIGHT] = {
		/**
		 * @vendor_name: Fatcatlab
		 * @vendor_part_number: Light Sensor
		 * @vendor_website: http://fatcatlab.com/product/light-sensor
		 */
		.name		= FATCATLAB_LIGHT_NAME,
		.type_id	= FATCATLAB_LIGHT_TYPE_ID,
		.num_modes	= 1,
		.num_view_modes	= 1,
		.mode_info	= {
			[0] = {
				/**
				 * @description: Illuminance
				 * @value0: Illuminance (0 to 65535)
				 * @units_description: lux
				 */
				.name		= "ILLUMINANCE",
				.data_sets	= 1,
				.data_type	= LEGO_SENSOR_DATA_S32,
				.units		= "lx",
			},
		},
	},
	[FATCATLAB_ALTITUDE] = {
		/**
		 * @vendor_name: Fatcatlab
		 * @vendor_part_number: Altitude Sensor
		 * @vendor_website: http://fatcatlab.com/product/altitude-sensor
		 */
		.name		= FATCATLAB_ALTITUDE_NAME,
		.type_id	= FATCATLAB_ALTITUDE_TYPE_ID,
		.num_modes	= 2,
		.num_view_modes	= 2,
		.mode_info	= {
			[0] = {
				/**
				 * @description: Pressure
				 * @value0: Pressure (3000 to 11000)
				 * @units_description: hectopascals
				 */
				.name		= "PRESSURE",
				.data_sets	= 1,
				.data_type	= LEGO_SENSOR_DATA_S16,
				.units		= "hPa",
				.decimals	= 1,
			},
			[1] = {
				/**
				 * @description: Altitude
				 * @value0: Altitude (-5000 to 90000)
				 * @units_description: meters
				 */
				.name		= "ALTITUDE",
				.data_sets	= 1,
				.data_type	= LEGO_SENSOR_DATA_S32,
				.units		= "m",
				.decimals	= 1,
			},
		},
	},
	[FATCATLAB_IR] = {
		/**
		 * @vendor_name: Fatcatlab
		 * @vendor_part_number: IR Receiver
		 * @vendor_website: http://fatcatlab.com/product/ir-receiver
		 */
		.name		= FATCATLAB_IR_NAME,
		.type_id	= FATCATLAB_IR_TYPE_ID,
		.num_modes	= 1,
		.num_view_modes	= 1,
		.mode_info	= {
			[0] = {
				/**
				 * .. [#fcl-ir-mode0-value0] Button values:
				 *
				 *    ======= =========================
				 *     Value   Button
				 *    ======= =========================
				 *     0       *none*
				 *     1-9     1-9 (digits)
				 *     10      0 (zero)
				 *     11      \+ (plus)
				 *     12      \- (minus)
				 *     13      ⏮ (previous)
				 *     14      ⏭ (next)
				 *     15      ⏯ (play/pause)
				 *     21      OK
				 *     22      ↰ (back)
				 *     30      ⏻ (power)
				 *     40      MENU
				 *     50      🔇 (mute)
				 *     60      MODE
				 *    ======= =========================
				 *
				 * @description: IR Remote Control
				 * @value0: Channel 1 (0 to 60)
				 * @value0_footnote: [#fcl-ir-mode0-value0]_
				 * @units_description: button
				 */
				.name		= "IR DATA",
				.data_sets	= 1,
				.data_type	= LEGO_SENSOR_DATA_S8,
			},
		},
	},
	[FATCATLAB_9DOF] = {
		/**
		 * @vendor_name: Fatcatlab
		 * @vendor_part_number: 9DOF Sensor
		 * @vendor_website: http://fatcatlab.com/product/9dof-sensor
		 */
		.name		= FATCATLAB_9DOF_NAME,
		.type_id	= FATCATLAB_9DOF_TYPE_ID,
		.num_modes	= 3,
		.num_view_modes	= 3,
		.mode_info	= {
			[0] = {
				/**
				 * @description: Gyroscope
				 * @value0: X-axis rotational speed (-20000  to 20000)
				 * @value1: Y-axis rotational speed (-20000  to 20000)
				 * @value2: Z-axis rotational speed (-20000  to 20000)
				 * @units_description: degrees per second
				 */
				.name		= "GYRO",
				.data_sets	= 3,
				.data_type	= LEGO_SENSOR_DATA_S32,
				.units		= "d/s",
				.decimals	= 1,
			},
			[1] = {
				/**
				 * @description: Accelerometer
				 * @value0: X-axis acceleration (-16000 to 16000)
				 * @value1: Y-axis acceleration (-16000 to 16000)
				 * @value2: Z-axis acceleration (-16000 to 16000)
				 * @units_description: standard gravity
				 */
				.name		= "ACC",
				.data_sets	= 3,
				.data_type	= LEGO_SENSOR_DATA_S32,
				.units		= "g",
				.decimals	= 3,
			},
			[2] = {
				/**
				 * @description: Magnetometer
				 * @value0: X-axis magnetic flux density (-4800 to 4800)
				 * @value1: Y-axis magnetic flux density (-4800 to 4800)
				 * @value2: Z-axis magnetic flux density (-4800 to 4800)
				 * @units_description: microteslas
				 */
				.name		= "MAGNET",
				.data_sets	= 3,
				.data_type	= LEGO_SENSOR_DATA_S32,
				.units		= "uT",
			},
		},
	},
	[FATCATLAB_HUMIDITY] = {
		/**
		 * @vendor_name: Fatcatlab
		 * @vendor_part_number: Humidity Sensor
		 * @vendor_website: http://fatcatlab.com/product/humidity-sensor
		 */
		.name		= FATCATLAB_HUMIDITY_NAME,
		.type_id	= FATCATLAB_HUMIDITY_TYPE_ID,
		.num_modes	= 3,
		.num_view_modes	= 3,
		.mode_info	= {
			[0] = {
				/**
				 * @description: Temperature (Celsius)
				 * @value0: Temperature (-400 to 1250)
				 * @units_description: degrees Celsius
				 */
				.name		= "CENTIGRADE",
				.data_sets	= 1,
				.data_type	= LEGO_SENSOR_DATA_S16,
				.units		= "C",
				.decimals	= 1,
			},
			[1] = {
				/**
				 * @description: Temperature (Fahrenheit)
				 * @value0: Temperature (-400 to 2570)
				 * @units_description: degrees Fahrenheit
				 */
				.name		= "FAHRENHEIT",
				.data_sets	= 1,
				.data_type	= LEGO_SENSOR_DATA_S16,
				.units		= "F",
				.decimals	= 1,
			},
			[2] = {
				/**
				 * @description: Humidity
				 * @value0: Humidity (0 to 1000)
				 * @units_description: percent relative humidity
				 */
				.name		= "HUMIDITY",
				.data_sets	= 1,
				.data_type	= LEGO_SENSOR_DATA_S16,
				.units		= "%RH",
				.decimals	= 1,
			},
		},
	},
};
EXPORT_SYMBOL_GPL(ev3_uart_sensor_defs);
