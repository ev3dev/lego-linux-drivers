
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
      - ``KEY_ENTER`` (105)

    * - Right
      - ``KEY_ENTER`` (106)

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

- This driver is used to get information about the EV3 battery.
- It uses the `power_supply`_ subsystem.
- It registers a sysfs device node at ``/sys/class/power_supply/lego-ev3-battery/``.

.. flat-table:: Sysfs Attributes
    :widths: 1 3

    * - ``current_now``
      - Returns the battery current in microamps.
    * - ``scope``
      - Always returns ``System``.
    * - ``technology``
      - Returns ``Unknown`` or ``Li-ion`` depending on if the rechargeable
        battery is present. If the technology is ``Unknown``, you can write
        ``NiMH`` to this attribute if you are using rechargeable NiMH batteries.
    * - ``type``
      - Always returns ``Battery``.
    * - ``voltage_max_design``
      - Returns the nominal "full" battery voltage. The value returned
        depends on ``technology``.
    * - ``voltage_min_design``
      - Returns the nominal "empty" battery voltage. The value returned
        depends on ``technology``.
    * - ``voltage_now``
      - Returns the battery voltage in microvolts.

.. _power_supply: http://lxr.free-electrons.com/source/Documentation/power/power_supply_class.txt?v=4.9
