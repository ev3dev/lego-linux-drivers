Dexter Industries BrickPi/BrickPi+
==================================

This page describes the drivers that are specific to `Dexter Industries
BrickPi and BrickPi+`__ These controllers are LEGO MINDSTORMS compatible addon
boards for Raspberry Pi.

.. note:: BrickPi is identified by hardware v1.7.3 and BrickPi+ is identified as
   v2.8.

.. __: https://www.dexterindustries.com/brickpi/


.. _brickpi_in_port_mode_info:

Input Ports
-----------

.. kernel-doc:: brickpi/brickpi_ports_in.c
    :doc: userspace

.. lego-port:: brickpi_in_port_mode_info


.. _brickpi-in-port-5:

Input Port 5
------------

.. note:: Only the original BrickPi has an Input Port 5. BrickPi+ does not.

Input Port 5 on the BrickPi is physically wired to the I2C pins on the Raspberry
Pi and I2C is supported by the ``i2c_bcm2708`` kernel module (there is no
ev3dev-specific driver for this port).

Only :ref:`nxt-i2c-sensors` and :ref:`other-i2c-sensors` work on port 5. 

.. tip:: Sensor types are listed in the :ref:`supported-sensors` table.


Configuration
~~~~~~~~~~~~~

For Input Port 5 to work with most LEGO compatible sensors, you have to make
sure that correct baudrate for the sensor is set and load the driver manually.
:ref:`nxt-i2c-sensors` should work with the slow 9600 baudrate used originally
by LEGO MINDSTORMS NXT.

You can temporarily change the baudrate on a running system. This change will
be reset after a reboot::

    sudo modprobe -r i2c_bcm2708 # remove module
    sudo modprobe i2c_bcm2708 baudrate=9600 # load module specifying baudrate

.. note:: Some devices behave erratically if baudrate is changed while they are plugged in. If needed unplug your sensor, change the baudrate and plug the sensor again.

Or, you can permanently change the baudrate by editing ``/boot/flash/config.txt``
with the following line (requires reboot to take effect)::

    dtparam=i2c_baudrate=9600


Usage
~~~~~

To use a sensor, you have to tell the I2C adapter the driver and I2C address
to use. When you write this to ``/sys/bus/i2c/devices/i2c-1/new_device``,
a new sensor device will appear in ``/sys/class/lego-sensor/`` which can be
used as you would any other sensor.

.. tip:: You can find the driver name and default I2C address in the sensor information pages.

Example: Loading Microinfinity CruizCore XG1300L driver manually::

    sudo sh -c 'echo "mi-xg1300l 0x01" > /sys/bus/i2c/devices/i2c-1/new_device'


I2C Sensor Driver
-----------------

.. kernel-doc:: brickpi/brickpi_i2c_sensor.c
    :doc: userspace


.. _brickpi_out_port_mode_info:

Output Ports
------------

.. kernel-doc:: brickpi/brickpi_ports_out.c
    :doc: userspace

.. lego-port:: brickpi_out_port_mode_info


Battery
-------

.. kernel-doc:: brickpi/brickpi_battery.c
    :doc: userspace


Line Discipline
---------------

.. kernel-doc:: brickpi/brickpi_ld.c
    :doc: userspace

