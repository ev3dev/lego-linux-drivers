Dexter Industries BrickPi3
==========================

This page describes the drivers that are specific to `Dexter Industries
BrickPi3`__ These controllers are LEGO MINDSTORMS compatible addon
boards for Raspberry Pi. Unlike the previous versions of BrickPi, the
BrickPi3 uses the SPI interface on the Raspberry Pi to communicate.

.. __: https://www.dexterindustries.com/brickpi/

You can confirm that the ``brickpi3`` module has loaded successfully by
checking ``dmesg``. You should see something like this::

    [    3.053862] brickpi3 spi0.1: Mfg: Dexter Industries
    [    3.060030] brickpi3 spi0.1: Board: BrickPi3
    [    3.063114] brickpi3 spi0.1: HW: 3.2.1
    [    3.076690] brickpi3 spi0.1: FW: 1.2.0
    [    3.090078] brickpi3 spi0.1: ID: FAD3CABB504B5354392E314BFF0F2C38


.. _brickpi3_in_port_mode_info:

Input Ports
-----------

.. kernel-doc:: brickpi3/brickpi3_ports_in.c
    :doc: userspace

.. lego-port:: brickpi3_in_port_mode_info


I2C Adapters
------------

.. kernel-doc:: brickpi3/brickpi3_i2c.c
    :doc: userspace


.. _brickpi3_out_port_mode_info:

Output Ports
------------

.. kernel-doc:: brickpi3/brickpi3_ports_out.c
    :doc: userspace

.. lego-port:: brickpi3_out_port_mode_info


LEDs
----

.. kernel-doc:: brickpi3/brickpi3_leds.c
    :doc: userspace


Battery
-------

.. kernel-doc:: brickpi3/brickpi3_battery.c
    :doc: userspace


Voltages
--------

.. kernel-doc:: brickpi3/brickpi3_iio.c
    :doc: userspace
