
LEGO MINDSTORMS EV3
===================

Input / Output Ports
--------------------

.. kernel-doc:: ev3/legoev3_ports_core.c
    :doc: userspace


.. _legoev3_input_port_mode_info:

Input Ports
~~~~~~~~~~~

.. kernel-doc:: ev3/legoev3_ports_in.c
    :doc: userspace

.. lego-port:: legoev3_input_port_mode_info


.. _legoev3_output_port_mode_info:

Output Ports
~~~~~~~~~~~~

.. kernel-doc:: ev3/legoev3_ports_out.c
    :doc: userspace

.. lego-port:: legoev3_output_port_mode_info


Buttons
-------

The EV3 buttons use the standard Linux input subsystem. You can find the event
device node at ``/dev/input/by-path/platform-gpio-keys.0-event``. The buttons
are mapped to Linux key codes as shown in the table below.

.. flat-table:: Button map
    :widths: 1 1
    :header-rows: 1

    * - Button
      - Linux key code (value)

    * - Back
      - ``KEY_BACKSPACE`` (14)

    * - Center
      - ``KEY_ENTER`` (28)

    * - Up
      - ``KEY_UP`` (103)

    * - Left
      - ``KEY_LEFT`` (105)

    * - Right
      - ``KEY_RIGHT`` (106)

    * - Down
      - ``KEY_DOWN`` (108)


LEDs
----

The EV3 brick status LEDs used the standard Linux LEDs subsystem. You can
find the LED devices at ``/sys/class/leds/``. There are two bi-color LEDs.
The color for each LED is controlled separately, so there is a ``green`` and
``red`` device node for each LED. Turning both of these on at the same time
will make the LED appear orange(ish).

The LEDs are controlled by a PWM signal, so you can vary the brightness between
0 and ``max_brightness``.


Display
-------

The EV3 display uses the standard Linux frame buffer subsystem (fbdev). The
display memory can be directly accessed by reading and writing or memory mapping
``/dev/fb0``. Display attributes such as width, height and row stride can be
read using ``FBIOGET_VSCREENINFO`` and ``FBIOGET_FSCREENINFO`` ioctls.

.. note:: You must make sure that you control the active virtual console before
   drawing on the display, otherwise other programs could be writing to the
   display at the same time, causing unexpected results.


Sound
-----

.. kernel-doc:: ev3/legoev3_sound.c
    :doc: userspace


Battery
-------

.. kernel-doc:: ev3/legoev3_battery.c
    :doc: userspace
