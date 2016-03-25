/*
 * Tacho motor helpers
 *
 * Copyright (C) 2016 David Lechner <david@lechnology.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.

 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/export.h>

#include <tacho_motor_helper.h>

/*
 * Speed helper:
 *
 * Performs averaging of speed over a time period. This is not the most
 * accurate - especially at low speeds, but is the best we can do with motor
 * controllers that just report position.
 */

/**
 * tm_speed_update - update the speed based on new position and time
 *
 * @spd: Pointer to the speed helper.
 * @pos: The new position.
 * @t: The timestamp of the new position.
 */
void tm_speed_update(struct tm_speed *spd, int pos, ktime_t t)
{
	long ds = USEC_PER_SEC * (pos - spd->pos[spd->tail]);
	ktime_t dt = ktime_sub(t, spd->time[spd->tail]);

	spd->tail++;
	spd->tail &= BUFFER_SIZE - 1;

	spd->speed = div64_s64(ds, ktime_to_us(dt));

	spd->head++;
	spd->head &= BUFFER_SIZE - 1;
	spd->pos[spd->head] = pos;
	spd->time[spd->head] = t;
}
EXPORT_SYMBOL_GPL(tm_speed_update);

/**
 * tm_speed_init - initialize the speed helper
 *
 * @spd: Pointer to the speed helper.
 * @pos: The current position.
 * @t: The timestamp of the current position.
 * @count: The calculation period in terms of # calls to tm_speed_update.
 */
void tm_speed_init(struct tm_speed *spd, int pos, ktime_t t, int count)
{
	int i;

	BUG_ON(count >= BUFFER_SIZE);
	for (i = 0; i < BUFFER_SIZE; i++) {
		spd->pos[i] = pos;
		spd->time[i] = t;
	}
	spd->speed = 0;
	spd->head = 0;
	spd->tail = BUFFER_SIZE - count;
}
EXPORT_SYMBOL_GPL(tm_speed_init);
