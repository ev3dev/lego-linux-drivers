
FatcatLab EVB
=============

The FatcatLab EVB is a LEGO MINDSTORMS EV3 compatible cape for BeagleBone.


Input / Output Ports
--------------------

.. kernel-doc:: evb/evb_ports_core.c
    :doc: userspace


.. _evb_input_port_mode_info:

Input Ports
~~~~~~~~~~~

.. kernel-doc:: evb/evb_ports_in.c
    :doc: userspace

.. lego-port:: evb_input_port_mode_info


Input Port I2C Adapters
~~~~~~~~~~~~~~~~~~~~~~~

.. kernel-doc:: evb/evb_pru_i2c.c
    :doc: userspace


.. _evb_output_port_mode_info:

Output Ports
~~~~~~~~~~~~

.. kernel-doc:: evb/evb_ports_out.c
    :doc: userspace

.. lego-port:: evb_output_port_mode_info


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
