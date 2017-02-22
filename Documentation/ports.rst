
Input / Output Ports
====================

This page is about the kind of ports that you plug :doc:`sensors <sensors>` and
:doc:`motors <motors>` into. Ports that have more than one connection type will
have drivers that let you control the port itself. The input and output ports
on the EV3 itself and some sensor multiplexers fall into this category (see
list below). Simple multiplexers that only support one connection type will not
have a separate driver for the port.

Input Port 5 on the BrickPi is physically wired to the I2C pins on the
RaspberryPi. See BrickPi :ref:`brickpi-in-port-5` for the details.

Port drivers use the `lego-port-class`_ to provide a common interface for
interacting with individual ports. Follow the link for more information.


List of port drivers
--------------------

This is a list of port drivers that are currently available in the ev3dev
kernel.

.. lego-port-list::


.. _lego-port-class:

The lego-port Subsystem
-----------------------

.. kernel-doc:: core/lego_port_class.c
    :doc: userspace
