
Sensors / Input Devices
=======================

The EV3 has four input ports for connecting sensors and other devices (like
sensor multiplexers or motor controllers). If you are trying to use something
that plugs into one of those ports, you are in the right place.


.. _supported-sensors:

Supported Sensors
-----------------

This is a list of sensors that currently have drivers available in the ev3dev
kernel. The *Part Number* column links to the vendor website. The *Name*
column links to the sensor's entry in *Appendix A*.

.. lego-sensor-list::

.. The references for these footnotes come from sphinx/ev3dev_json.py as part
   of the lego-sensor-list directive.

.. [#detect-lego-nxt-touch] Only touch sensors that shipped with the NXT 2.0
   set can be automatically detected. Older touch sensors that shipped with
   the original NXT sets are missing an electrical connection (pin 2 is not
   internally connected to pin 3) and cannot be automatically detected.

.. [#detect-nxt-analog] The automatic detection algorithm detects this sensor
   as an `NXT/Analog <nxt-analog-sensors>`_ type sensor but it cannot determine
   the exact sensor type. The generic analog driver (``nxt-analog``) will be
   loaded by default for this sensor. See the :ref:`lego-port-class` for
   information on how to manually load the correct driver.

.. [#detect-other-i2c] The automatic detection algorithm detects this sensor
   as an I2C sensor and the port is automatically put into I2C mode. However,
   the sensor does not follow the LEGO MINDSTORMS convention for I2C sensors,
   so the exact type of sensor cannot be determined. See :doc:`i2c` for
   information on how to manually load the correct driver.

.. [#detect-mi-xg1300l] The automatic detection algorithm detects this sensor
   as an I2C sensor and the port is automatically put into I2C mode. However,
   this sensor only partially follows the LEGO MINDSTORMS convention for I2C
   sensors, so the driver must be loaded manually. See the sensor's page for
   more information.

.. [#detect-di-dflex] The Dexter Industries dFlex sensor cannot be automatically
   detected (because pin 2 is not connected to pin 3). In order to use this
   sensor, you must manually set the port to ``nxt-analog`` mode and then set
   the driver to ``di-dflex``.


Unsupported Sensors
-------------------

One of the goals of ev3dev is to support as many sensors as possible. In fact,
**even if a manufacturer's documentation says that a device does not work with
the EV3, chances are it will work with ev3dev.**

If you have a sensor that is not supported yet, let us know about it by
`opening an issue`_ on GitHub. For many sensors adding a driver is trivial -
even if you are not a "kernel hacker" or a "c programmer". For the non-trivial
sensors, see the `contributing page`_ for information on how to write a driver
or how to donate hardware to someone who will.

.. _opening an issue: https://github.com/ev3dev/ev3dev/issues
.. _contributing page: http://www.ev3dev.org/contributing


Using Sensors
-------------

Automatic Detection
~~~~~~~~~~~~~~~~~~~

The EV3 has "smart" sensor ports that can identify most sensors. Beware!
Full automatic detection works on EV3 only. And even on EV3, some sensors cannot
be automatically detected. See notes below in the table of supported sensors.

For sensors that cannot be automatically detected, you can manually control
the input port mode. Read more about it on :doc:`ports` page.

On FatcatLab's EVB, EV3/Analog and EV3/UART sensors can be automatically detected.
For NXT/Analog sensors, you must manually set the mode. Most NXT/I2C sensors should
be automatically detected.

On BrickPi, BrickPi+ and BrickPi3, sensors cannot be automatically detected at
all. You must manually configure the input ports for all sensors.

On PiStorms, detection of EV3/UART, EV3/Analog and NXT/I2C sensors is
semi-automatic. If you put the input port into ``ev3-uart``, ``ev3-analog``,
or ``i2c-thru`` mode, the specific type of sensor will be automatically detected
(assuming it is a LEGO compatible sensor). So you usually don't have to write to
the ``set_device`` attribute of the port for these sensors.

MINDSTORMS Compatible Sensors
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Most of the supported sensors are designed to work with LEGO MINDSTORMS. These
sensors use the `LEGO sensor class <lego-sensor-class>`_. You can find these
in sysfs at ``/sys/class/lego-sensor/``. The sensors have *modes* that select
what type of data is read from the sensor. Generally, you will select a mode
and then read the data value from the sensor. Follow the link above for more
details.

Other Sensors and Advanced Usage
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Many sensors that were not specifically designed for LEGO MINDSTORMS can be used
with ev3dev too. You can read more about the "Other" sensor types below. Also,
some MINDSTORMS compatible sensor may have advanced features that are not
accessible using the ``lego-sensor`` class.

When using sensors this way, often automatic detection will not work correctly,
therefore it is usually best to manually select the mode of input port (e.g.
``other-i2c`` or ``other-uart``).

For more information on using I2C sensors, check out :doc:`i2c`.


Types of Sensors
----------------

When dealing with sensors in ev3dev, it is useful to know how it communicates
with the EV3 brick. There are three basic kinds of communication that the input
ports can use to get information from the sensor, `Analog Sensors`_, `I2C Sensors`_
and `UART Sensors`_, plus one special type for `LEGO NXT Color Sensors`_.


Analog Sensors
~~~~~~~~~~~~~~

These are the simplest type of sensor. The measured value is converted to a
voltage (0-5VDC) that is read by the EV3.

.. _ev3-analog-sensors:

EV3/Analog Sensors
``````````````````

These are sensors that were designed specifically for the EV3 and will not work
on the NXT because the pinout is different. They contain an ID resistor so that
the EV3 can tell different types of sensors apart. The actual analog value is
measured on pin6. 

.. _nxt-analog-sensors:

NXT/Analog Sensors
``````````````````

These sensors are designed for the NXT, but also work on the EV3. The EV3 cannot
differentiate between most of these sensors though, so you have to tell it
which one your have or just use the generic driver.

.. _rcx-analog-sensors:

RCX/Analog Sensors
``````````````````

RCX sensors also fall into this category, but do not work with the EV3 - at
least not with the converter cable described in the NXT Hardware Developers
kit. This is due to a difference in the input port pins between the EV3 and
the NXT. If someone wants design a new converter cable, we could make them work.

.. _wedo-analog-sensors:

WeDo/Analog Sensors
```````````````````

WeDo sensors are also analog sensors. They are actually electrically similar to
EV3/Analog sensors (require 5V power and have ID resistor). Currently, we only
support WeDo sensors attached to a WeDo hub, but if someone would like to design
a cable and modify the ``wedo-sensor`` and ``lego-ports`` drivers, we could
easily make them work with the input ports on the EV3.


.. _nxt-color-sensors:

LEGO NXT Color Sensors
~~~~~~~~~~~~~~~~~~~~~~

The LEGO NXT Color Sensor is in a class of its own. It uses a hybrid of analog
and (non-standard) digital communications. The NXT Color Sensor is not usable
at this point in time. We can detect it with the auto-detect, but we don't
have a driver for it yet.



I2C Sensors
~~~~~~~~~~~

I2C sensors are sensors that communicate with the intelligent brick via the
`I2C protocol`_. In the NXT documentation, they are referred to a "digital"
sensors. These sensors can be sorted into two categories.

.. _I2C protocol: https://en.wikipedia.org/wiki/I2c


.. _nxt-i2c-sensors:

NXT/I2C Sensors
```````````````

These are sensors designed using LEGO's guidelines for NXT digital sensors.
NXT/I2C sensors can be automatically detected because they contain vendor and
product IDs at specific I2C registers.


.. _other-i2c-sensors:

Other/I2C Sensors
`````````````````

These can be any off-the-shelf I2C device or 3rd party sensors that are designed
for LEGO MINDSTORMS, but do not follow the guidelines of NXT/I2C sensors.



UART Sensors
~~~~~~~~~~~~

The EV3 has a `UART`_ transceiver connected to each input port that can
be used to communicate with many devices using this widely used standard.

.. _UART: https://en.wikipedia.org/wiki/Uart


.. _ev3-uart-sensors:

EV3/UART Sensors
````````````````

These is a new type of sensor that is designed specifically for the EV3 (they
don't work with the NXT).  These sensors are a bit "smarter" in that in addition
to sending the data of what they measure, they also send information about their
capabilities. This means that any new EV3/UART sensors should "just work" without
us having to write new drivers.


.. _other-uart-sensors:

Other/UART Sensors
``````````````````

In addition to sensors designed to work with EV3, any UART device can be connected.
But, be careful of voltage levels. The EV3 uses 3.3V I/O. It is safe to connect
other devices that use 3.3V or 5V I/O. But, don't connect anything with higher
voltage!


.. _lego-sensor-class:

The lego-sensor Subsytem
------------------------

.. kernel-doc:: sensors/lego_sensor_class.c
   :doc: userspace


Sensor Modules
--------------

The sensor drivers are implemented in the following modules:

EV3/Analog
~~~~~~~~~~

.. kernel-doc:: sensors/ev3_analog_sensor_core.c
   :doc: userspace

EV3/UART
~~~~~~~~

.. kernel-doc:: sensors/ev3_uart_sensor_ld.c
   :doc: userspace

.. kernel-doc:: sensors/ev3_uart_sensor_core.c
   :doc: userspace

NXT/Analog
~~~~~~~~~~

.. kernel-doc:: sensors/nxt_analog_sensor_core.c
   :doc: userspace

NXT/I2C
~~~~~~~

.. kernel-doc:: sensors/nxt_i2c_sensor_core.c
   :doc: userspace
