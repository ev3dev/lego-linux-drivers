Motors / Output Devices
=======================

The EV3 has four output ports for connecting motors and other devices (like
LEDs). If you are trying to use something that plugs into one of the output
ports, then you are in the right place. If you are working with a motor that
connects to a motor controller which plugs into an input port, you will find
information on the motor controller on the [sensors] page.

This page lists well-known devices, however any device that is rated for 9VDC,
requires 500mA or less and can handle pulse width modulation is safe to connect
directly to the output ports on the EV3.


Supported Motors
----------------

.. lego-motor-list::


.. [#motor-autodetect] Motors are only automatically detected on the LEGO
    MINDSTORMS EV3 platform. LEGO NXT Motors are detected as LEGO EV3 Large
    motors. 3rd party motors will not be correctly detected and must be
    manually specified.


Types of Motors
---------------

There are currently three basic types of classes of devices that are supported
in ev3dev. Tacho motors, DC motors and LEDs.

Tacho Motors
~~~~~~~~~~~~

Tacho motors get their name from the LMS2012 (official LEGO) source code. Tacho
is short for `tachometer`_. This is probably a bit of a misnomer because the
motor itself does not have a tachometer. Instead, the EV3 brick acts as the
tachometer. Technically speaking the motors have a `incremental rotary encoder`_
(also called quadrature encoder) that is used by the EV3 to determine the speed
and direction of rotation.

.. _tachometer: https://en.wikipedia.org/wiki/Tachometer
.. _incremental rotary encoder: https://en.wikipedia.org/wiki/Rotary_encoder#Incremental_rotary_encoder

DC Motors
~~~~~~~~~

DC motors are just "plain" motors. They do *not* have a quadrature encoder for
feedback.

LEDs
~~~~

Any 9VDC rated (i.e. it has an appropriately sized resistor) LED can be used.


Using Motors and LEDs
---------------------

Currently, only NXT and EV3 motors can be automatically detected. To use other
devices, see the [EV3 Output Port Driver] for information on how to set the
mode of the output port.

RCX compatible (aka 9V) motors and LEDs can be connected to the EV3 using a
`LEGO 8528`_ cable.

Power Functions motors and LEDs can be connected using a `LEGO 8528`_ cable plus
a `LEGO 8886`_ cable or `LEGO 8871`_ cable.

.. _LEGO 8528: http://shop.lego.com/en-US/Converter-Cables-for-LEGO-MINDSTORMS-NXT-8528
.. _LEGO 8886: http://shop.lego.com/en-US/LEGO-Power-Functions-Extension-Wire-8886
.. _LEGO 8871: http://shop.lego.com/en-US/LEGO-Power-Functions-Extension-Wire-20-8871


.. _tacho-motor-class:

tacho-motor Subsystem
---------------------

.. kernel-doc:: motors/tacho_motor_class.c
    :doc: userspace


.. _dc-motor-class:

dc-motor Subsystem
------------------

.. kernel-doc:: motors/dc_motor_class.c
    :doc: userspace


.. _servo-motor-class:

servo-motor Subsystem
---------------------

.. kernel-doc:: motors/servo_motor_class.c
    :doc: userspace


Motor Modules
-------------

EV3/EVB Motors
~~~~~~~~~~~~~~

.. kernel-doc:: ev3/legoev3_motor.c
    :doc: userspace

Generic DC Motors
~~~~~~~~~~~~~~~~~

.. kernel-doc:: motors/rcx_motor.c
    :doc: userspace

Generic LEDs
~~~~~~~~~~~~
.. kernel-doc:: motors/rcx_led.c
    :doc: userspace
