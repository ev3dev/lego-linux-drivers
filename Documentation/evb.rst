
FatcatLab EVB
=============

The FatcatLab EVB is a LEGO MINDSTORMS EV3 compatible cape for BeagleBone.


Input / Output Ports
--------------------

The EVB uses the LEGO MINDSTORMS EV3 input and output port drives, so please
refer to :ref:`ev3-ports` in the EV3 section.


Input Port I2C Adapters
~~~~~~~~~~~~~~~~~~~~~~~

.. kernel-doc:: evb/evb_pru_i2c.c
    :doc: userspace




Buttons / Joystick
------------------

.. kernel-doc:: evb/evb_input.c
    :doc: userspace


Battery
-------

.. kernel-doc:: evb/evb_battery.c
    :doc: userspace


Sound
-----

.. kernel-doc:: evb/evb_sound.c
    :doc: userspace
