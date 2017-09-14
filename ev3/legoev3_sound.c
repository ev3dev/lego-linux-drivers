/*
 * LEGO MINDSTORMS EV3 sound driver
 *
 * Copyright (C) 2013-2014,2016-2017 David Lechner <david@lechnology.com>
 * Copyright (C) 2014 Franz Detro <franz.detro@gmx.de>
 *
 * This driver is a combination of:
 *
 * "Writing an ALSA Driver"
 * <https://www.kernel.org/doc/htmldocs/writing-an-alsa-driver>
 * Copyright (c) 2002-2005 Takashi Iwai <tiwai@suse.de>
 *
 * BeagleBoard/GSoC/2010 Projects/Pulse Width Modulation
 * <http://elinux.org/BeagleBoard/GSoC/2010_Projects/Pulse_Width_Modulation>
 * Copyright (c) 2010 Varun Jewalikar
 *
 * d_sound.c from LEGO® MINDSTORMS EV3
 * Copyright (C) 2010-2013 The LEGO Group
 *
 * This program is free software; you may redistribute and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

/**
 * DOC: userspace
 *
 * The ``snd-legoev3`` module provedes an `ALSA`_ driver for PCM playback and a
 * Linux input device with sound capabilities (``EV_SND``) for producing tones.
 *
 * The ALSA driver can be used with standard tools such as ``alsamixer`` and
 * ``aplay``. Tones can be produced by using the ``beep`` command, ioctls such
 * as ``KDMKTONE`` (must run on local console or as root for ioctls) or by
 * writing ``SND_*`` events to the event device (must be member of ``input``
 * group for this).
 *
 * .. _ALSA: https://en.wikipedia.org/wiki/Advanced_Linux_Sound_Architecture
 */

#include <linux/gpio/consumer.h>
#include <linux/hrtimer.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/pwm.h>
#include <linux/slab.h>
#include <linux/timer.h>

#include <sound/control.h>
#include <sound/core.h>
#include <sound/initval.h>
#include <sound/pcm.h>

#if IS_ENABLED(CONFIG_LEGOEV3_FIQ)
#include <mach/legoev3-fiq.h>
#endif

#define DRIVER_NAME	"snd-legoev3"

/*--- configuration defines ---*/

#define BUFFER_SIZE	(128*1024)
#define TONE_MIN_HZ	100
#define TONE_MAX_HZ	10000
#define MAX_VOLUME	256
#define PCM_PWM_PERIOD	(NSEC_PER_SEC / 64000)

/*--- module parameters ---*/

static unsigned int max_sample_rate = 22050;

module_param(max_sample_rate, uint, S_IRUGO);
MODULE_PARM_DESC(max_sample_rate, "Maximum sample rate, range [8000..48000].");

/*--- private declarations ---*/

struct snd_legoev3 {
	struct gpio_desc	*ena_gpio;
	struct pwm_device	*pwm;
	struct input_dev	*input_dev;
	struct snd_card		*card;
	struct snd_pcm		*pcm;
	struct work_struct	 disable_work;
	struct hrtimer		 pcm_timer;
	struct tasklet_struct	 pcm_period_tasklet;
	unsigned int		 tone_frequency;
	unsigned int		 tone_volume;
	ktime_t			 pcm_timer_period;
	size_t			 pcm_playback_ptr;
	unsigned int		 pcm_callback_count;
	unsigned int		 pcm_volume;
#if IS_ENABLED(CONFIG_LEGOEV3_FIQ)
	int			 fiq_err;
#endif
	bool			 requested_enabled;
	bool			 is_enabled;
};

/*--- common functions ---*/

static int snd_legoev3_enable(struct snd_legoev3 *chip)
{
	chip->requested_enabled = true;
	cancel_work_sync(&chip->disable_work);
	gpiod_set_value(chip->ena_gpio, 1);
	chip->is_enabled = true;

	return 0;
};

static void snd_legoev3_disable(struct snd_legoev3 *chip)
{
	chip->requested_enabled = false;
	schedule_work(&chip->disable_work);
}

static void snd_legoev3_disable_work(struct work_struct *work)
{
	struct snd_legoev3 *chip =
			container_of(work, struct snd_legoev3, disable_work);

	/*
	 * we can't call cancel_delayed_work_sync in atomic context, so we have
	 * to manage race conditions ourselves.
	 */
	if (unlikely(chip->requested_enabled))
		return;

	chip->is_enabled = false;
	gpiod_set_value(chip->ena_gpio, 0);
}

/*--- tone/beep mode ---*/

static int snd_legoev3_apply_tone_volume(struct snd_legoev3 *chip)
{
	int duty;

	/*
	 * This could overflow an int in the calculations when period is min
	 * (1/100Hz) and volume is max (256). Also using 6.25% duty cycle as
	 * 100% volume, hence the >> 3. Any higher and we get distortion in
	 * the sound.
	 */
	duty = (pwm_get_period(chip->pwm) * chip->tone_volume / MAX_VOLUME) >> 4;

	return pwm_config(chip->pwm, duty, pwm_get_period(chip->pwm));
}

static int snd_legoev3_do_tone(struct snd_legoev3 *chip, int hz)
{
	int err;

	/* check if PCM playback is active */
	if (hrtimer_is_queued(&chip->pcm_timer))
		return -EBUSY;

	if (hz <= 0) {
		err = pwm_config(chip->pwm, 0, pwm_get_period(chip->pwm));
		if (err < 0)
			return err;
		snd_legoev3_disable(chip);
		chip->tone_frequency = 0;
		return 0;
	}
	if (hz < TONE_MIN_HZ)
		hz = TONE_MIN_HZ;
	if (hz > TONE_MAX_HZ)
		hz = TONE_MAX_HZ;
	/*
	 * Just setting the period here - duty cycle is set in
	 * snd_legoev3_apply_tone_volume().
	 */
	err = pwm_config(chip->pwm, 0, NSEC_PER_SEC / hz);
	if (err < 0)
		return err;

	err = snd_legoev3_apply_tone_volume(chip);
	if (err < 0)
		return err;

	snd_legoev3_enable(chip);
	chip->tone_frequency = hz;

	return 0;
}

static int snd_legoev3_beep_event(struct input_dev *input, unsigned int type,
				unsigned int code, int hz)
{
	struct snd_legoev3 *chip = input_get_drvdata(input);

	switch(code) {
	case SND_BELL:
		if (hz)
			hz = 1000;
	case SND_TONE:
		break;
	default:
		return -1;
	}

	snd_legoev3_do_tone(chip, hz);

	return 0;
}

static int snd_legoev3_input_device_create(struct snd_card *card)
{
	struct snd_legoev3 *chip = card->private_data;
	struct input_dev *input;
	int err;

	input = input_allocate_device();
	if (IS_ERR(input))
		return PTR_ERR(input);

	input->name = card->shortname;
	input->dev.parent = chip->card->dev;
	input->event = snd_legoev3_beep_event;
	input_set_capability(input, EV_SND, SND_BELL);
	input_set_capability(input, EV_SND, SND_TONE);
	input_set_drvdata(input, chip);

	err = input_register_device(input);
	if (err < 0) {
		input_free_device(input);
		return err;
	}

	chip->input_dev = input;

	return 0;
}

/*--- ALSA PCM device ---*/

static struct snd_pcm_hardware snd_legoev3_playback_hw = {
	.info = (SNDRV_PCM_INFO_MMAP |
		 SNDRV_PCM_INFO_MMAP_VALID |
		 SNDRV_PCM_INFO_INTERLEAVED),
	.formats           = SNDRV_PCM_FMTBIT_S16_LE,
	.rates             = SNDRV_PCM_RATE_CONTINUOUS,
	.rate_min          = 8000,
	.rate_max          = 48000,
	.channels_min      = 1,
	.channels_max      = 1,
	.buffer_bytes_max  = BUFFER_SIZE,
	.period_bytes_min  = 4096,
	.period_bytes_max  = BUFFER_SIZE,
	.periods_min       = 1,
	.periods_max       = 1024,
};

/*
 * Call snd_pcm_period_elapsed in a tasklet
 * This avoids spinlock messes and long-running irq contexts
 */
static void snd_legoev3_period_elapsed_tasklet(unsigned long data)
{
	struct snd_pcm_substream *substream = (void *)data;

	snd_pcm_period_elapsed(substream);
}

static enum hrtimer_restart snd_legoev3_pcm_timer_callback(struct hrtimer *timer)
{
	struct snd_legoev3 *chip = container_of(timer, struct snd_legoev3, pcm_timer);
	struct snd_pcm_substream *substream = (void *)chip->pcm_period_tasklet.data;
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct pwm_device *pwm = chip->pwm;
	long duty;
	size_t period_size;

	hrtimer_forward_now(&chip->pcm_timer, chip->pcm_timer_period);

	/*
	 * Take care of notifying alsa every when we are done playing back
	 * a period.
	 */
	period_size = snd_pcm_lib_period_bytes(substream);
	chip->pcm_callback_count += 1;
	if (chip->pcm_callback_count >= period_size) {
		chip->pcm_callback_count %= period_size;
		tasklet_schedule(&chip->pcm_period_tasklet);
	}

	duty = *(short *)(runtime->dma_area + chip->pcm_playback_ptr);
	/* do volume scaling while value is signed */
	duty *= chip->pcm_volume;
	duty /= MAX_VOLUME;
	/* then convert to unsigned and scale to pwm period */
	duty += SHRT_MAX;
	duty *= pwm_get_period(pwm);
	duty /= USHRT_MAX;

	/*
	 * FIXME: Setting the buffer data to 0 here because there is a tendency
	 * to replay part of a sample at the end of playback. If we can figure
	 * out how to detect the end of playback, we wouldn't need to do this.
	 */
	*(short *)(runtime->dma_area + chip->pcm_playback_ptr) = 0;

	pwm_config(pwm, duty, pwm_get_period(pwm));

	chip->pcm_playback_ptr += frames_to_bytes(runtime, 1);
	if (chip->pcm_playback_ptr >= snd_pcm_lib_buffer_bytes(substream))
		chip->pcm_playback_ptr = 0;

	return HRTIMER_RESTART;
}

#if IS_ENABLED(CONFIG_LEGOEV3_FIQ)
static void snd_legoev3_period_elapsed(void *data)
{
	struct snd_legoev3 *chip = data;

	tasklet_schedule(&chip->pcm_period_tasklet);
}
#endif

static int snd_legoev3_pcm_playback_open(struct snd_pcm_substream *substream)
{
	struct snd_legoev3 *chip = snd_pcm_substream_chip(substream);
	struct snd_pcm_runtime *runtime = substream->runtime;
	int err;

	/* start at 50% duty cycle to prevent speaker pop */
	err = pwm_config(chip->pwm, PCM_PWM_PERIOD / 2, PCM_PWM_PERIOD);
	if (err < 0)
		return err;

	runtime->hw = snd_legoev3_playback_hw;
	tasklet_init(&chip->pcm_period_tasklet, snd_legoev3_period_elapsed_tasklet,
		     (unsigned long)substream);

#if IS_ENABLED(CONFIG_LEGOEV3_FIQ)
	chip->fiq_err = legoev3_fiq_ehrpwm_request();
	if (chip->fiq_err < 0)
		pr_warn("%s: Failed to get FIQ (%d), falling back to hrtimer\n",
			__func__, chip->fiq_err);
#endif

	return 0;
}

static int snd_legoev3_pcm_playback_close(struct snd_pcm_substream *substream)
{
	struct snd_legoev3 *chip = snd_pcm_substream_chip(substream);

	/* Should already be stopped, but just in case... */
	hrtimer_cancel(&chip->pcm_timer);
	tasklet_kill(&chip->pcm_period_tasklet);

#if IS_ENABLED(CONFIG_LEGOEV3_FIQ)
	if (!chip->fiq_err)
		legoev3_fiq_ehrpwm_release();
#endif

	return 0;
}

static int snd_legoev3_pcm_hw_params(struct snd_pcm_substream *substream,
				   struct snd_pcm_hw_params *hw_params)
{
	return snd_pcm_lib_malloc_pages(substream,
					params_buffer_bytes(hw_params));
}

static int snd_legoev3_pcm_hw_free(struct snd_pcm_substream *substream)
{
	return snd_pcm_lib_free_pages(substream);
}

static int snd_legoev3_pcm_prepare(struct snd_pcm_substream *substream)
{
	struct snd_legoev3 *chip = snd_pcm_substream_chip(substream);

	chip->pcm_playback_ptr = 0;
	chip->pcm_callback_count = 0;

#if IS_ENABLED(CONFIG_LEGOEV3_FIQ)
	if (!chip->fiq_err) {
		unsigned char int_period;
		int period;

		/* interrupt can occur on every 1st, 2nd or 3rd pwm pulse */
		if (substream->runtime->rate > 32000)
			int_period = 1;
		else if (substream->runtime->rate > 16000)
			int_period = 2;
		else
			int_period = 3;

		period = NSEC_PER_SEC / (substream->runtime->rate * int_period);
		pwm_config(chip->pwm, period / 2, period);
		legoev3_fiq_ehrpwm_prepare(substream,
					   chip->pcm_volume,
					   int_period,
					   snd_legoev3_period_elapsed,
					   chip);
	}
#endif

	return 0;
}

static int snd_legoev3_pcm_trigger(struct snd_pcm_substream *substream, int cmd)
{
	struct snd_legoev3 *chip = snd_pcm_substream_chip(substream);

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
		if (chip->tone_frequency || hrtimer_is_queued(&chip->pcm_timer))
			return -EBUSY;
#if IS_ENABLED(CONFIG_LEGOEV3_FIQ)
		if (legoev3_fiq_ehrpwm_int_is_enabled())
			return -EBUSY;
#endif
		snd_legoev3_enable(chip);
		chip->pcm_timer_period = ktime_set(0,
				NSEC_PER_SEC / substream->runtime->rate);
#if IS_ENABLED(CONFIG_LEGOEV3_FIQ)
		if (!chip->fiq_err)
			legoev3_fiq_ehrpwm_int_enable();
		else
#endif
		hrtimer_start(&chip->pcm_timer, chip->pcm_timer_period,
			      HRTIMER_MODE_REL);
		break;
	case SNDRV_PCM_TRIGGER_STOP:
		snd_legoev3_disable(chip);
#if IS_ENABLED(CONFIG_LEGOEV3_FIQ)
		if (!chip->fiq_err)
			legoev3_fiq_ehrpwm_int_disable();
		else
#endif
		hrtimer_cancel(&chip->pcm_timer);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static snd_pcm_uframes_t
snd_legoev3_pcm_pointer(struct snd_pcm_substream *substream)
{
	struct snd_legoev3 *chip = snd_pcm_substream_chip(substream);

#if IS_ENABLED(CONFIG_LEGOEV3_FIQ)
	if (!chip->fiq_err)
		return bytes_to_frames(substream->runtime,
				       legoev3_fiq_ehrpwm_get_playback_ptr());
#endif
	return bytes_to_frames(substream->runtime, chip->pcm_playback_ptr);
}

static struct snd_pcm_ops snd_legoev3_playback_ops = {
	.open		= snd_legoev3_pcm_playback_open,
	.close		= snd_legoev3_pcm_playback_close,
	.ioctl		= snd_pcm_lib_ioctl,
	.hw_params	= snd_legoev3_pcm_hw_params,
	.hw_free	= snd_legoev3_pcm_hw_free,
	.prepare	= snd_legoev3_pcm_prepare,
	.trigger	= snd_legoev3_pcm_trigger,
	.pointer	= snd_legoev3_pcm_pointer,
};

static int snd_legoev3_new_pcm(struct snd_legoev3 *chip)
{
	struct snd_pcm *pcm;
	int err;

	err = snd_pcm_new(chip->card, "PWM", 0, 1, 0, &pcm);
	if (err < 0)
		return err;
	pcm->private_data = chip;
	strlcpy(pcm->name, chip->card->shortname, sizeof(pcm->name));
	chip->pcm = pcm;

	snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_PLAYBACK,
		&snd_legoev3_playback_ops);

	err = snd_pcm_lib_preallocate_pages_for_all(pcm,
		SNDRV_DMA_TYPE_CONTINUOUS, snd_dma_continuous_data(GFP_KERNEL),
		BUFFER_SIZE, BUFFER_SIZE);
	if (err < 0)
		return err;

	return 0;
}

/*--- volume controls ---*/

static int snd_legoev3_volume_control_info(struct snd_kcontrol *kcontrol,
					 struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 1;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = MAX_VOLUME;

	return 0;
}

static int snd_legoev3_tone_volume_control_get(struct snd_kcontrol *kcontrol,
					     struct snd_ctl_elem_value *ucontrol)
{
	struct snd_legoev3 *chip = snd_kcontrol_chip(kcontrol);
	ucontrol->value.integer.value[0] = chip->tone_volume;

	return 0;
}

static int snd_legoev3_tone_volume_control_put(struct snd_kcontrol *kcontrol,
					     struct snd_ctl_elem_value *ucontrol)
{
	struct snd_legoev3 *chip = snd_kcontrol_chip(kcontrol);
	int newValue = ucontrol->value.integer.value[0];
	int changed = 0;

	if (newValue < 0)
		newValue = 0;
	if (newValue > MAX_VOLUME)
		newValue = MAX_VOLUME;

	if (chip->tone_volume != newValue) {
		chip->tone_volume = newValue;

		/* if tone playback is running, apply volume */
		if (chip->tone_frequency)
			snd_legoev3_apply_tone_volume(chip);

		changed = 1;
	}

	return changed;
}

static int snd_legoev3_pcm_volume_control_get(struct snd_kcontrol *kcontrol,
					    struct snd_ctl_elem_value *ucontrol)
{
	struct snd_legoev3 *chip = snd_kcontrol_chip(kcontrol);
	ucontrol->value.integer.value[0] = chip->pcm_volume;

	return 0;
}

static int snd_legoev3_pcm_volume_control_put(struct snd_kcontrol *kcontrol,
					    struct snd_ctl_elem_value *ucontrol)
{
	struct snd_legoev3 *chip = snd_kcontrol_chip(kcontrol);
	int newValue = ucontrol->value.integer.value[0];
	int changed = 0;

	if (newValue < 0)
		newValue = 0;
	if (newValue > MAX_VOLUME)
		newValue = MAX_VOLUME;

	if (chip->tone_volume != newValue) {
		chip->pcm_volume = newValue;

		/* if PCM playback is running, apply volume */
		/* FIXME: */
#if IS_ENABLED(CONFIG_LEGOEV3_FIQ)
		if (!chip->fiq_err)
			legoev3_fiq_ehrpwm_set_volume(chip->pcm_volume);
#endif

		changed = 1;
	}

	return changed;
}

static struct snd_kcontrol_new tone_volume_control = {
	.iface	= SNDRV_CTL_ELEM_IFACE_MIXER,
	.name	= "Beep Playback Volume",
	.index	= 0,
	.access	= SNDRV_CTL_ELEM_ACCESS_READWRITE,
	.info	= snd_legoev3_volume_control_info,
	.get	= snd_legoev3_tone_volume_control_get,
	.put	= snd_legoev3_tone_volume_control_put
};

static struct snd_kcontrol_new pcm_volume_control = {
	.iface	= SNDRV_CTL_ELEM_IFACE_MIXER,
	.name	= "PCM Playback Volume",
	.index	= 0,
	.access	= SNDRV_CTL_ELEM_ACCESS_READWRITE,
	.info	= snd_legoev3_volume_control_info,
	.get	= snd_legoev3_pcm_volume_control_get,
	.put	= snd_legoev3_pcm_volume_control_put
};

/*--- platform sound device ---*/

static int snd_legoev3_create(struct snd_card *card, struct pwm_device *pwm,
			    struct gpio_desc *gpio)
{
	struct snd_legoev3 *chip  = card->private_data;
	static struct snd_device_ops ops = { };
	int err;

	chip->card = card;
	chip->pwm = pwm;
	chip->ena_gpio = gpio;
	INIT_WORK(&chip->disable_work, snd_legoev3_disable_work);
	hrtimer_init(&chip->pcm_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	chip->pcm_timer.function = &snd_legoev3_pcm_timer_callback;

	err = snd_device_new(card, SNDRV_DEV_LOWLEVEL, chip, &ops);
	if (err < 0)
		return err;

	err = snd_legoev3_new_pcm(chip);
	if (err < 0)
		return err;

	err = snd_ctl_add(card, snd_ctl_new1(&tone_volume_control, chip));
	if (err < 0)
		return err;

	err = snd_ctl_add(card, snd_ctl_new1(&pcm_volume_control, chip));
	if (err < 0)
		return err;

	return 0;
}

static int snd_legoev3_probe(struct platform_device *pdev)
{
	struct snd_card *card;
	struct gpio_desc *ena_gpio;
	struct pwm_device *pwm;
	const char *label;
	int err;

	of_property_read_string(pdev->dev.of_node, "label", &label);

	ena_gpio = devm_gpiod_get(&pdev->dev, "enable", GPIOD_OUT_LOW);
	if (IS_ERR(ena_gpio)) {
		err = PTR_ERR(ena_gpio);
		if (err != -EPROBE_DEFER);
			dev_err(&pdev->dev, "Failed to get gpio!\n");
		return err;
	}

	pwm = devm_pwm_get(&pdev->dev, "speaker");
	if (IS_ERR(pwm)) {
		err = PTR_ERR(pwm);
		if (err != -EPROBE_DEFER)
			dev_err(&pdev->dev, "Failed to get pwm!\n");
		return err;
	}

	pwm_apply_args(pwm);

	/* This lets us set the pwm duty cycle in an atomic context */
	pm_runtime_irq_safe(pwm->chip->dev);

	err = pwm_enable(pwm);
	if (err < 0) {
		dev_err(&pdev->dev, "Failed to enable pwm!\n");
		return err;
	}

	/* configure maximal sample rate */
	if (max_sample_rate < 8000)
		max_sample_rate = 8000;
	else if (max_sample_rate > 48000)
		max_sample_rate = 48000;

	snd_legoev3_playback_hw.rate_max = max_sample_rate;

	err = snd_card_new(&pdev->dev, -1, "legoev3", THIS_MODULE,
			   sizeof(struct snd_legoev3), &card);
	if (err < 0) {
		dev_err(&pdev->dev, "Failed to create sound card!\n");
		goto err_snd_card_new;
	}

	strlcpy(card->driver, DRIVER_NAME, sizeof(card->driver));
	strlcpy(card->shortname, label, sizeof(card->shortname));
	sprintf(card->longname, "%s connected to %s", card->shortname,
		dev_name(pwm->chip->dev));

	err = snd_legoev3_create(card, pwm, ena_gpio);
	if (err < 0) {
		dev_err(&pdev->dev, "Failed to create sound device!\n");
		goto err_snd_legoev3_create;
	}

	err = snd_card_register(card);
	if (err < 0) {
		dev_err(&pdev->dev, "Failed to register sound card!\n");
		goto err_snd_card_register;
	}

	err = snd_legoev3_input_device_create(card);
	if (err < 0) {
		dev_err(&pdev->dev, "Failed to create beep device!\n");
		goto err_snd_legoev3_input_device_create;
	}

	dev_set_drvdata(&pdev->dev, card);

	return 0;

err_snd_legoev3_input_device_create:
err_snd_card_register:
err_snd_legoev3_create:
	snd_card_free(card);
err_snd_card_new:
	pwm_disable(pwm);
	return err;
}

static int snd_legoev3_remove(struct platform_device *pdev)
{
	struct snd_card *card = dev_get_drvdata(&pdev->dev);
	struct snd_legoev3 *chip =  card->private_data;

	input_unregister_device(chip->input_dev);
	input_free_device(chip->input_dev);
	snd_card_free(card);
	dev_set_drvdata(&pdev->dev, NULL);

	/* make sure sound is off */
	snd_legoev3_disable(chip);
	cancel_work_sync(&chip->disable_work);
	pwm_disable(chip->pwm);

	return 0;
}

static const struct of_device_id of_snd_legoev3_match[] = {
	{ .compatible = "ev3dev,ev3-sound", },
	{ }
};
MODULE_DEVICE_TABLE(of, of_snd_legoev3_match);

static struct platform_driver snd_legoev3_platform_driver = {
	.driver = {
		.name		= DRIVER_NAME,
		.of_match_table = of_snd_legoev3_match,
	},
	.probe = snd_legoev3_probe,
	.remove = snd_legoev3_remove,
};
module_platform_driver(snd_legoev3_platform_driver);

MODULE_DESCRIPTION("LEGO MINDSTORMS EV3 speaker driver");
MODULE_AUTHOR("David Lechner <david@lechnology.com>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:" DRIVER_NAME);
