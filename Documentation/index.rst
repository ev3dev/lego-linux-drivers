=======================================
Linux Kernel Drivers for ev3dev-buster
=======================================

This is the documentation for the ev3dev-specific Linux kernel drivers that
are part of the ev3dev-buster operating system.

The drivers use `sysfs`_ to interact with user space (*user space* is anything
outside of the kernel). Each hardware device is represented by a directory
in sysfs called a **device node**. Each device node has **attributes** that
are represented by files. You monitor and control the hardware by reading and
writing these attribute files.

.. _sysfs: https://en.wikipedia.org/wiki/Sysfs

.. _userspace:

.. toctree::
   :maxdepth: 2
   :caption: User Space Interfaces

   sensors
   motors
   ports
   muxs
   ev3
   wedo
   brickpi
   brickpi3
   pistorms
   evb
   board-info
   sensor_data
   motor_data
   i2c
