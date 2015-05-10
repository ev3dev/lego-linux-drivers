/*
 * LEGO MINDSTORMS EV3 UART Sensor driver
 *
 * Copyright (C) 2013-2015 David Lechner <david@lechnology.com>
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
 * parser can be found in the ev3dev-kpkg repository.
 */

#include "ev3_uart_sensor.h"

const struct ev3_uart_sensor_info ev3_uart_sensor_defs[] = {
	[LEGO_EV3_COLOR] = {
		/**
		 * @vendor_name: LEGO
		 * @vendor_part_number: 45506
		 * @vendor_part_name: EV3 Color Sensor
		 */
		.name		= LEGO_EV3_COLOR_NAME,
		.type_id	= LEGO_EV3_COLOR_TYPE_ID,
		.num_modes	= 6,
		.num_view_modes	= 3,
		.mode_info	= {
			[0] = {
				/**
				 * @description: Reflected light<br />LED color: red
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
				 * @description: Ambient light<br />LED color: blue (dimly lit)
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
				 * [^color-values]: Color values:
				 *
				 * | Value | Color  |
				 * |-------|--------|
				 * | 0     | none   |
				 * | 1     | black  |
				 * | 2     | blue   |
				 * | 3     | green  |
				 * | 4     | yellow |
				 * | 5     | red    |
				 * | 6     | white  |
				 * | 7     | brown  |
				 *
				 * @description: Color<br />LED color: white (all LEDs rapidly cycling)
				 * @value0: Detected color (0 to 7)
				 * @value0_footnote: [^color-values]
				 * @units_description: color
				 */
				.name		= "COL-COLOR",
				.data_sets	= 1,
				.data_type	= LEGO_SENSOR_DATA_S8,
				.units		= "col",
			},
			[3] = {
				/**
				 * @description: Raw Reflected<br />LED color: red
				 * @value0: ??? (0 to 1020???)
				 * @value1: ??? (0 to 1020???)
				 */
				.name		= "REF-RAW",
				.data_sets	= 2,
				.data_type	= LEGO_SENSOR_DATA_S16,
			},
			[4] = {
				/**
				 * @description: Raw Color Components<br />LED color: white (all LEDs rapidly cycling)
				 * @value0: Red??? (0 to 1020???)
				 * @value1: Green??? (0 to 1020???)
				 * @value2: Blue??? (0 to 1020???)
				 * @units_description: color
				 */
				.name		= "RGB-RAW",
				.data_sets	= 3,
				.data_type	= LEGO_SENSOR_DATA_S16,
			},
			[5] = {
				/**
				 * [^cal-mode]: This mode is not usable. When in COL-CAL mode,
				 * the color sensor does not respond to the keep-alive sent from
				 * the EV3 brick. As a result, the sensor will time out and reset.
				 *
				 * @name_footnote: [^cal-mode]
				 * @description: Calibration ???<br />LED color: red, flashing every 4 seconds, then goes continous
				 * @value0: ???
				 * @value1: ???
				 * @value2: ???
				 * @value3: ???
				 */
				.name		= "COL-CAL",
				.data_sets	= 4,
				.data_type	= LEGO_SENSOR_DATA_S32,
			},
		},
	},
	[LEGO_EV3_ULTRASONIC] = {
		/**
		 * @vendor_name: LEGO
		 * @vendor_part_number: 45504
		 * @vendor_part_name: EV3 Ultrasonic Sensor
		 */
		.name		= LEGO_EV3_ULTRASONIC_NAME,
		.type_id	= LEGO_EV3_ULTRASONIC_TYPE_ID,
		.num_modes	= 7,
		.num_view_modes	= 3,
		.mode_info	= {
			[0] = {
				/**
				 * @description: Continuous measurement<br />LEDs: On, steady
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
				 * @description: Continuous measurement<br />LEDs: On, steady
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
				 * [^listen-value]: A value of `1` indicates that another ultrasonic
				 * sensor has been detected. A `1` can also be triggered by a loud
				 * noise such as clapping.
				 *
				 * @description: Listen<br />LEDs: On, blinking
				 * @value0: Presence (0-1)
				 * @value0_footnote: [^listen-value]
				 */
				.name		= "US-LISTEN",
				.data_sets	= 1,
				.data_type	= LEGO_SENSOR_DATA_S8,
			},
			[3] = {
				/**
				 * [^single-measurement]: A measurement is taken when the mode is set
				 * and `value0` will not change after this. To take another measurement
				 * set the mode again. **NOTE:** If you write the mode too frequently
				 * (e.g. every 100msec), the sensor will sometimes lock up and writing
				 * to the `mode` attribute will return an error. A delay of 250msec
				 * between each write to the mode attribute seems sufficient to keep
				 * the sensor from locking up.
				 *
				 * @description: Single measurement<br />LEDs: On momentarily when mode is set, then off
				 * @value0: Distance (0-2550)
				 * @value0_footnote: [^single-measurement]
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
				 * @description: Single measurement<br />LEDs: On momentarily when mode is set, then off
				 * @value0: Distance (0-1003)
				 * @value0_footnote: [^single-measurement]
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
				 * [^dc-mode]: Not sure what DC mode stands for.
				 * Seems to work like the continuous measurement modes.
				 *
				 * @name_footnote: [^dc-mode]
				 * @description: ???<br />LEDs: On, steady
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
				 * @name_footnote: [^dc-mode]
				 * @description: ???<br />LEDs: On, steady
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
		 */
		.name		= LEGO_EV3_GYRO_NAME,
		.type_id	= LEGO_EV3_GYRO_TYPE_ID,
		.num_modes	= 5,
		.num_view_modes	= 3,
		.mode_info	= {
			[0] = {
				/**
				 * [^angle-reset]: The angle in GYRO-ANG or GYRO-G&A modes can
				 * be reset by changing to a different mode and changing back.
				 *
				 * [^angle-overflow]: If you spin around too many times
				 * in GYRO-ANG or GYRO-G&A mode, it will get stuck at 32767.
				 *
				 * [^direction]: Clockwise is positive when looking at the side
				 * of the sensor with the arrows.
				 *
				 * @name_footnote: [^angle-reset]
				 * @description: Angle
				 * @value0: Angle (-32768 to 32767)
				 * @value0_footnote: [^angle-overflow]<sup>,</sup>[^direction]
				 * @units_description: degrees
				 */
				.name		= "GYRO-ANG",
				.data_sets	= 1,
				.data_type	= LEGO_SENSOR_DATA_S16,
				.units		= "deg",
			},
			[1] = {
				/**
				 * [^calibration]: The sensor is calibrated when the
				 * GYRO-RATE or the GYRO-G&A mode is set. If the sensor is
				 * moving when setting the mode, the calibration will be off.
				 *
				 * @name_footnote: [^calibration]
				 * @description: Rotational Speed
				 * @value0: Rotational Speed (-440 to 440)
				 * @value0_footnote: [^direction]
				 * @units_description: degrees per second
				 */
				.name		= "GYRO-RATE",
				.data_sets	= 1,
				.data_type	= LEGO_SENSOR_DATA_S16,
				.units		= "d/s",
			},
			[2] = {
				/**
				 * @description: Raw sensor value ???
				 * @value0: ??? (-1464 to 1535)
				 * @value0_footnote: [^direction]
				 */
				.name		= "GYRO-FAS",
				.data_sets	= 1,
				.data_type	= LEGO_SENSOR_DATA_S16,
			},
			[3] = {
				/**
				 * @name_footnote: [^angle-reset]<sup>,</sup>[^calibration]
				 * @description: Angle and Rotational Speed
				 * @value0: Angle (-32768 to 32767)
				 * @value0_footnote: [^angle-overflow]<sup>,</sup>[^direction]
				 * @value1: Rotational Speed (-440 to 440)
				 * @value1_footnote: [^direction]
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
		},
	},
	[LEGO_EV3_INFRARED] = {
		/**
		 * @vendor_name: LEGO
		 * @vendor_part_number: 45509
		 * @vendor_part_name: EV3 Infrared Sensor
		 */
		.name		= LEGO_EV3_INFRARED_NAME,
		.type_id	= LEGO_EV3_INFRARED_TYPE_ID,
		.num_modes	= 6,
		.num_view_modes	= 3,
		.mode_info	= {
			[0] = {
				/**
				 * [^distance-value]: 100% is approximately 70cm/27in.
				 *
				 * @description: Proximity
				 * @value0: Distance (0 to 100)
				 * @value0_footnote: [^distance-value]
				 * @units_description: percent
				 */
				.name		= "IR-PROX",
				.data_sets	= 1,
				.data_type	= LEGO_SENSOR_DATA_S8,
				.units		= "pct",
			},
			[1] = {
				/**
				 * [^heading-value]: When looking in the same direction as the
				 * sensor, -25 is far left and +25 is far right.
				 *
				 * [^no-beacon]: The absence of a beacon on a channel can be
				 * detected when distance == -128 (and heading == 0).
				 *
				 * @description: IR Seeker
				 * @value0: Channel 1 Heading (-25 to 25)
				 * @value0_footnote: [^heading-value]
				 * @value1: Channel 1 Distance (-128 and 0 to 100)
				 * @value1_footnote: [^distance-value]<sup>,</sup>[^no-beacon]
				 * @value2: Channel 2 Heading (-25 to 25)
				 * @value2_footnote: [^heading-value]
				 * @value3: Channel 2 Distance (-128 and 0 to 100)
				 * @value3_footnote: [^distance-value]<sup>,</sup>[^no-beacon]
				 * @value4: Channel 3 Heading (-25 to 25)
				 * @value4_footnote: [^heading-value]
				 * @value5: Channel 3 Distance (-128 and 0 to 100)
				 * @value5_footnote: [^distance-value]<sup>,</sup>[^no-beacon]
				 * @value6: Channel 4 Heading (-25 to 25)
				 * @value6_footnote: [^heading-value]
				 * @value7: Channel 4 Distance (-128 and 0 to 100)
				 * @value7_footnote: [^distance-value]<sup>,</sup>[^no-beacon]
				 * @units_description: percent
				 */
				.name		= "IR-SEEK",
				.data_sets	= 1,
				.data_type	= LEGO_SENSOR_DATA_S8,
				.units		= "pct",
				.data_sets	= 8,
			},
			[2] = {
				/**
				 * [^remote-mode-values]: Button values:
				 *
				 * | Value | Description            |
				 * |-------|------------------------|
				 * | 0     | none                   |
				 * | 1     | red up                 |
				 * | 2     | red down               |
				 * | 3     | blue up                |
				 * | 4     | blue down              |
				 * | 5     | red up and blue up     |
				 * | 6     | red up and blue down   |
				 * | 7     | red down and blue up   |
				 * | 8     | red down and blue down |
				 * | 9     | beacon mode on         |
				 * | 10    | red up and red down    |
				 * | 11    | blue up and blue down  |
				 *
				 * red == left and blue == right
				 *
				 * Pressing more that 2 buttons at one time is not supported.
				 * It will usually read 0. Pressing an up/down button while
				 * beacon mode is activated with turn off beacon mode.
				 *
				 * @description: IR Remote Control
				 * @value0: Channel 1 (0 to 11)
				 * @value0_footnote: [^remote-mode-values]
				 * @value1: Channel 2 (0 to 11)
				 * @value1_footnote: [^remote-mode-values]
				 * @value2: Channel 3 (0 to 11)
				 * @value2_footnote: [^remote-mode-values]
				 * @value3: Channel 4 (0 to 11)
				 * @value3_footnote: [^remote-mode-values]
				 * @units_description: button
				 */
				.name		= "IR-REMOTE",
				.data_sets	= 1,
				.data_type	= LEGO_SENSOR_DATA_S8,
				.units		= "btn",
				.data_sets	= 4,
			},
			[3] = {
				/**
				 * [^alt-remote-mode-values]: Button values:
				 *
				 * | Value   | Blue Down | Blue Up | Red Down | Red Up |
				 * |:-------:|:---------:|:-------:|:--------:|:------:|
				 * | 262/384 |           |         |          |        |
				 * | 287     |           |         |          | X      |
				 * | 300     |           |         | X        |        |
				 * | 309     |           |         | X        | X      |
				 * | 330     |           | X       |          |        |
				 * | 339     |           | X       |          | X      |
				 * | 352     |           | X       | X        |        |
				 * | 377     |           | X       | X        | X      |
				 * | 390     | X         |         |          |        |
				 * | 415     | X         |         |          | X      |
				 * | 428     | X         |         | X        |        |
				 * | 437     | X         |         | X        | X      |
				 * | 458     | X         | X       |          |        |
				 * | 467     | X         | X       |          | X      |
				 * | 480     | X         | X       | X        |        |
				 * | 505     | X         | X       | X        | X      |
				 *
				 * X = button pressed
				 *
				 * The most significant byte is always 0x01. In the least
				 * significant byte, the 4 most significant bits represent
				 * each button. Bit 7 is the blue down button, bit 6 is the
				 * blue up button, bit 5 is the red down button, bit 4 is the
				 * red up button. Beware that when no buttons are pressed,
				 * bit 7 is set (value == 384). You can test that bits 0-3
				 * are all 0 to check this.
				 *
				 *<pre><code>if ((value & 0x0F) == 0) {
				 *     // no buttons are pressed
				 * } else {
				 *     if (value & 0x80)
				 *         // blue down button is pressed
				 *     if (value & 0x40)
				 *         // blue up button is pressed
				 *     if (value & 0x20)
				 *         // red down button is pressed
				 *     if (value & 0x10)
				 *         // red up button is pressed
				 * }
				 * </code></pre>
				 *
				 * Bits 0-3 seem to be some sort of checksum or parity check.
				 * Bit 0 = bit 4, bit 1 = ~(bit 5), bit 2 = ~(bit 6),
				 * bit 3 = 0 if bits 0-2 are even or 1 if bits 0-2 are odd.
				 *
				 * Also, when the beacon mode is active or for about 1 second
				 * after any button is released the value is 262.
				 *
				 * This mode only works with the remote on channel 1.
				 *
				 * @description: IR Remote Control
				 * @value0: Channel 1
				 * @value0_footnote: [^alt-remote-mode-values]
				 */
				.name		= "IR-REM-A",
				.data_sets	= 1,
				.data_type	= LEGO_SENSOR_DATA_S16,
			},
			[4] = {
				/**
				 * [^alt-seeker-mode]: `IR-S-ALT` mode is not usable.
				 * When switching to this mode, the sensor quits responding
				 * to the keep-alive messages and the sensor resets.
				 *
				 * @name_footnote: [^alt-seeker-mode]
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
};
EXPORT_SYMBOL_GPL(ev3_uart_sensor_defs);
