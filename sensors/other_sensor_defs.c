/*
 * non-MINDSTORMS sensor drivers
 *
 * Copyright (C) 2013-2014 David Lechner <david@lechnology.com>
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
 * IMPORTANT: THIS FILE IS NOT MEANT TO BE COMPILED. IT IS ONLY FOR
 * AUTOMATICALLY GENERATING DOCUMENTATION ON NON-MINDSTORMS SENSORS.
 */

/*
 * Documentation is automatically generated from this struct, so formatting is
 * very important. Make sure any new sensors have the same layout. The comments
 * are also parsed to provide more information for the documentation. The
 * parser can be found in the Documentation/json/ directory.
 */

const struct other_sensor_info other_i2c_sensor_defs[] = {
	[MS_SENSOR_KIT_PFC8574] = {
		/**
		 * .. [#pcf8574-address] Valid addresses are 0x38..0x3F
		 *    (configurable via input pins)
		 *
		 * .. [#pcf8574-device-class] Sample usage:
		 *
		 *    Register I2C device::
		 *
		 *        echo pcf8574 0x38 > /sys/bus/i2c/devices/i2c-<port+2>/new_device
		 *
		 *    Finding device class node and initializing::
		 *
		 *        for chip in $(find /sys/class/gpio -name gpiochip*)
		 *        do
		 *            if [[ "$(cat $chip/label)" == "pcf8547" ]]
		 *            then
		 *                base=$(cat $chip/base)
		 *                # Pins are active low
		 *                for i in {0..7}
		 *                do
		 *                    gpio=$(($base + $i))
		 *                    echo $gpio > /sys/class/gpio/export
		 *                    # gpios on this chip are active low
		 *                    echo 1 > /sys/class/gpio/gpio$gpio/active_low
		 *                    # initialize direction here
		 *                done
		 *                # do whatever with the gpios
		 *            fi
		 *        done
		 *
		 * @vendor_name: mindsensors.com
		 * @vendor_part_number: PCF8574-Nx
		 * @vendor_part_name: Sensor building kit for NXT with PCF8574 IC
		 * @vendor_website: http://mindsensors.com/index.php?module=pagemaster&PAGE_user_op=view_page&PAGE_id=71
		 * @device_class: `gpio <https://www.kernel.org/doc/Documentation/gpio/>`_ [#pcf8574-device-class]_
		 * @default_address: 0x38
		 * @default_address_footnote: [#pcf8574-address]_
		 */
		.name		= "pcf8574",
	},
	[MS_SENSOR_KIT_PCF8591] = {
		/**
		 * .. [#pcf8591-address] Valid addresses are 0x48..0x4F
		 *    (configurable via input pins)
		 *
		 * .. [#pcf8591-device-class] Sample usage:
		 *
		 *    Register I2C device::
		 *
		 *        echo pcf8591 0x48 > /sys/bus/i2c/devices/i2c-<port+2>/new_device
		 *
		 *    Finding device class node::
		 *
		 *        for chip in $(find /sys/class/hwmon -name hwmon*)
		 *        do
		 *            if [[ "$(cat $chip/device/name)" == "pcf8591" ]]
		 *            then
		 *                # do whatever
		 *            fi
		 *        done
		 *
		 * @vendor_name: mindsensors.com
		 * @vendor_part_number: PCF8591-Nx
		 * @vendor_part_name: Sensor building kit for NXT with PCF8591 IC
		 * @vendor_website: http://mindsensors.com/index.php?module=pagemaster&PAGE_user_op=view_page&PAGE_id=92
		 * @device_class: `hwmon <https://wiki.archlinux.org/index.php/Lm_sensors>`_ [#pcf8591-device-class]_
		 * @default_address: 0x48
		 * @default_address_footnote: [#pcf8591-address]_
		 */
		.name		= "pcf8591",
	},
	[MS_RTC] = {
		/**
		 * .. [#ds1307-device-class] Sample usage:
		 *
		 *    Register I2C device::
		 *
		 *        echo ds1307 0x68 > /sys/bus/i2c/devices/i2c-<port+2>/new_device
		 *
		 *    Finding device class node::
		 *
		 *        for chip in $(find /sys/class/rtc -name rtc*)
		 *        do
		 *            if [[ "$(cat $chip/name)" == "ds1307" ]]
		 *            then
		 *                # do whatever
		 *            fi
		 *        done
		 *
		 * @vendor_name: mindsensors.com
		 * @vendor_part_number: RTC-Nx-v3
		 * @vendor_part_name: Realtime Clock for NXT
		 * @vendor_website: http://mindsensors.com/index.php?module=pagemaster&PAGE_user_op=view_page&PAGE_id=77
		 * @device_class: `rtc <https://www.kernel.org/doc/Documentation/rtc.txt>`_ [#ds1307-device-class]_
		 * @default_address: 0x68
		 */
		.name		= "ds1307",
	},
};
