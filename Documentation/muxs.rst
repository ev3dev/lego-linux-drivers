Port Multiplexers
=================

Port multiplexers are used to connect more than one sensor or motor to a single
input or output port. Some multiplexers are treated as a single sensor while
others have their own port drivers.

Supported Multiplexers
----------------------

.. flat-table::
    :widths: 1 3 1 5
    :header-rows: 1

    * - Part Number
      - Name
      - Ports
      - Supported connections

    * - :cspan:`3` **HiTechnic**

    * - `NTX1060 <http://www.hitechnic.com/cgi-bin/commerce.cgi?preadd=action&key=NTX1060>`_
      - `NXT Touch Sensor Multiplexer`_
      - 4
      - LEGO NXT Touch sensor

    * - `NSX2020 <http://www.hitechnic.com/cgi-bin/commerce.cgi?preadd=action&key=NSX2020>`_
      - `HiTechnic Sensor Multiplexer`_
      - 4
      - NXT/Analog and NXT/I2C sensors

    * - :cspan:`3` **mindsensors.com**

    * - `EV3SensorMux <http://www.mindsensors.com/ev3-and-nxt/23-ev3-sensor-multiplexer-for-ev3-or-nxt>`_
      - `EV3 Sensor Multiplexer for EV3 or NXT`_
      - 3
      - LEGO EV3 sensors (no 3rd party sensors)

    * - `NXTMMX-v2 <http://www.mindsensors.com/ev3-and-nxt/21-multiplexer-for-nxtev3-motors>`_
      - `Multiplexer for NXT/EV3 Motors`_
      - 3
      - LEGO EV3/NXT motors

    * - `NXTServo-v3 <http://www.mindsensors.com/ev3-and-nxt/25-8-channel-servo-controller-for-nxt-or-ev3>`_
      - `8 Channel Servo Controller for NXT or EV3`_
      - 8
      - Hobby type RC servo motors

    * - NXTTouchMux
      - `Touch Sensor Multiplexer for NXT`_
      - 3
      - LEGO NXT Touch sensors

    * - `SPLIT-Nx-v2 <http://www.mindsensors.com/ev3-and-nxt/52-port-splitter-for-nxt-digital-sensors>`_
      - `Port Splitter for NXT Digital Sensors`_
      - 3
      - All I2C sensors


HiTechnic
-----------

NXT Touch Sensor Multiplexer
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

This multiplexer allows connecting up to 4 LEGO NXT Touch sensors to a single
input port. This device is treated as a single sensor, so there are is no
``lego-port`` driver loaded.


.. _ht_nxt_smux_port_mode_info:

HiTechnic Sensor Multiplexer
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. kernel-doc:: sensors/ht_nxt_smux.c
   :doc: userspace

.. lego-port:: ht_nxt_smux_port_mode_info

.. kernel-doc:: sensors/ht_nxt_smux_i2c_sensor.c
   :doc: userspace


Mindsensors.com
---------------

.. _ms_ev3_smux_port_mode_info:

EV3 Sensor Multiplexer for EV3 or NXT
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. kernel-doc:: sensors/ms_ev3_smux.c
   :doc: userspace

.. lego-port:: ms_ev3_smux_port_mode_info


.. _ms_nxtmmx_out_port_mode_info:

Multiplexer for NXT/EV3 Motors
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. kernel-doc:: sensors/ms_nxtmmx.c
   :doc: userspace

.. lego-port:: ms_nxtmmx_out_port_mode_info


8 Channel Servo Controller for NXT or EV3
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

This multiplexer allows connecting up to 8 hobby type RC servo motors to a
single input port. There is no port device associated with this. When connected,
It loads 8 :ref:`servo-motor-class` devices that are used to control the motors.


Touch Sensor Multiplexer for NXT
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

This multiplexer allows connecting up to 3 LEGO NXT Touch sensors to a single
input port. This device is treated as a single sensor, so there are is no
``lego-port`` driver loaded.


Port Splitter for NXT Digital Sensors
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

This multiplexer allows connecting up to 3 I2C sensors to a single input port.
This is a passive device, so there are no drivers. All connected I2C devices
must have a unique I2C address.
