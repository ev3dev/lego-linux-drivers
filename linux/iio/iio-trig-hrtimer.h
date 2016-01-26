/**
 * The industrial I/O periodic hrtimer trigger driver
 *
 * Copyright (C) Intuitive Aerial AB
 * Written by Marten Svanfeldt, marten@intuitiveaerial.com
 * Copyright (C) 2012, Analog Device Inc.
 *  Author: Lars-Peter Clausen <lars@metafoo.de>
 * Copyright (C) 2015, Intel Corporation
 *
 * Slightly modified for kernels < 4.5
 * Copyright (C) 2016 David Lechner <david@lechnology.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 */

extern struct iio_trigger *iio_trig_hrtimer_probe(const char *name);
extern int iio_trig_hrtimer_remove(struct iio_trigger *trigger);
