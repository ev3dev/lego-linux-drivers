/*
 * LEGO Mindstorms EV3 sound driver
 *
 * Copyright (C) 2013 David Lechner <david@lechnology.com>
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

#include <linux/gpio.h>
#include <linux/hrtimer.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/pwm.h>
#include <linux/slab.h>
#include <linux/timer.h>

#include <sound/control.h>
#include <sound/core.h>
#include <sound/initval.h>
#include <sound/legoev3.h>
#include <sound/pcm.h>

#include <asm/fiq.h>
#include <mach/legoev3-fiq.h>

/*--- configuration defines ---*/

#define BUFFER_SIZE (128*1024)
#define TONE_MIN_HZ 100
#define TONE_MAX_HZ 10000
#define MAX_VOLUME  256

/*--- module parameters ---*/

static unsigned int max_sample_rate = 22050;
static unsigned int ramp_ms = 20;
static bool debug = false;

module_param(max_sample_rate, uint, S_IRUGO);
MODULE_PARM_DESC(max_sample_rate, "Maximum sample rate, range [8000..48000].");
module_param(ramp_ms, uint, S_IRUGO|S_IWUSR);
MODULE_PARM_DESC(ramp_ms, "PWM ramp time in ms. 0 turns ramping off.");
module_param(debug,  bool, S_IRUGO|S_IWUSR);
MODULE_PARM_DESC(debug, "Enable ALSA callback logging.");

struct snd_legoev3 {
	struct pwm_device    *pwm;
	struct snd_card      *card;
	struct input_dev     *input_dev;
	struct snd_pcm       *pcm;
	struct tasklet_struct pcm_period_tasklet;
	struct delayed_work   pcm_stop;
	bool                  pcm_stop_cancelled;
	unsigned              amp_gpio;

	unsigned long  tone_frequency;
	unsigned long  tone_duration;
	struct hrtimer tone_timer;

	int volume;
};

/*--- tone mode ---*/

static int snd_legoev3_apply_tone_volume(struct snd_legoev3 *chip)
{
	int duty_percent;
	
	/* use only 1/8th of volume range (taken from lms2012 source code) */
	duty_percent = ((50/8) * chip->volume) >> 8;
	if ((duty_percent == 0) && (chip->volume > 0))
		duty_percent = 1; 
	
	return pwm_config(chip->pwm, chip->pwm->period * duty_percent / 100,
			  chip->pwm->period);
}

static int snd_legoev3_do_tone(struct snd_legoev3 *chip, int hz)
{
	int err;

	if (chip->pcm->streams[0].substream_opened ||
	    chip->pcm->streams[1].substream_opened)
		return -EBUSY;

	if (hz <= 0) {
		gpio_set_value(chip->amp_gpio, 0);
		pwm_disable(chip->pwm);
		pwm_config(chip->pwm, 0, chip->pwm->period);
		chip->tone_frequency = 0;
		chip->tone_duration  = 0;
		return 0;
	}
	if (hz < TONE_MIN_HZ)
		hz = TONE_MIN_HZ;
	if (hz > TONE_MAX_HZ)
		hz = TONE_MAX_HZ;
	err = pwm_config(chip->pwm, chip->pwm->duty_cycle, NSEC_PER_SEC / hz);
	if (err < 0)
		return err;
		
	err = snd_legoev3_apply_tone_volume(chip);
	if (err < 0)
		return err;
	
	err = pwm_enable(chip->pwm);
	if (err < 0)
		return err;
	
	chip->tone_frequency = hz;
	chip->tone_duration  = 0;
	gpio_set_value(chip->amp_gpio, 1);

	return 0;
}

static void snd_legoev3_stop_tone(struct snd_legoev3 * chip)
{
	snd_legoev3_do_tone(chip, 0);
}

/**
 * snd_legoev3_cb_stop_tone - timer callback on end of tone duration
 */
static enum hrtimer_restart snd_legoev3_cb_stop_tone(struct hrtimer *pTimer)
{
	struct snd_legoev3 *chip = container_of(pTimer, struct snd_legoev3, tone_timer);

	snd_legoev3_do_tone(chip, 0);

	return HRTIMER_NORESTART;
}

/**
 * snd_legoev3_show_tone - sysfs 'tone' attribute read
 *
 * @output: frequency in Hz, 0 if no tone active
*/
static ssize_t snd_legoev3_show_tone(struct device *dev,
                                     struct device_attribute *attr,      
                                     char *buf)
{
	struct snd_card *card = dev_get_drvdata(&to_platform_device(dev)->dev);
	struct snd_legoev3 *chip = card->private_data;

	if (chip->tone_frequency)
	{
		if (chip->tone_duration)
			return snprintf(buf, PAGE_SIZE, "%lu %lu\n",
			                chip->tone_frequency, chip->tone_duration);
		else
			return snprintf(buf, PAGE_SIZE, "%lu\n", chip->tone_frequency);
	}
	
	return snprintf(buf, PAGE_SIZE, "0\n");
}

/**
 * snd_legoev3_store_tone - sysfs 'tone' attribute write
 *
 *  input:  frequency (Hz) [duration (ms)]
 *
 * frequency == 0 stops tone
 * 
 * Examples: '1000'     // 1KHz tone
 *           '440 1000' // 440 Hz tone for one second
 *           '0'        // stop tone
*/
static ssize_t snd_legoev3_store_tone(struct device *dev,
                                      struct device_attribute *attr,
                                      const char *buf, size_t count)
{
	struct snd_card *card = dev_get_drvdata(&to_platform_device(dev)->dev);
	struct snd_legoev3 *chip = card->private_data;

	const char *start = buf;
	      char *end   = (char*)buf;
	const char *last  = buf + count;

	int freq, duration = 0;
	ktime_t time;

	start = skip_spaces(end);
	if (last <= start)
		return -EINVAL;

	freq = simple_strtol(start, &end, 0);
	if (end == start)
		return -EINVAL;
	
	if (freq != 0)
	{
		if (freq < TONE_MIN_HZ)
			return -EINVAL;
		if (freq > TONE_MAX_HZ)
			return -EINVAL;
	}

	start = skip_spaces(end);	
	if (last <= start)
	{
		if (snd_legoev3_do_tone(chip, freq) == 0)
			return count;
 	
		return -EINVAL;
	}

	duration = simple_strtol(start, &end, 0);
	if (end == start)
		return -EINVAL;
		
	if (duration < 0)
		return -EINVAL;

	if (snd_legoev3_do_tone(chip, freq) == 0)
	{
		time = ktime_set(duration/1000, (duration%1000)*NSEC_PER_MSEC);
		hrtimer_start(&chip->tone_timer, time, HRTIMER_MODE_REL);

		return count;
	}

	return -EINVAL;
}

/*--- input device (beep) ---*/

static int snd_legoev3_beep_event(struct input_dev *dev, unsigned int type,
                                  unsigned int code, int hz)
{
	struct snd_legoev3 *chip = input_get_drvdata(dev);

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
	struct input_dev *dev;
	int err;

	dev = input_allocate_device();
	if (IS_ERR(dev))
		return PTR_ERR(dev);

	dev->name = "EV3 speaker beep";
	dev->dev.parent = chip->card->dev;
	dev->evbit[0] = BIT(EV_SND);
	dev->sndbit[0] = BIT(SND_BELL) | BIT(SND_TONE);
	dev->event = snd_legoev3_beep_event;

	err = input_register_device(dev);
	if (err < 0) {
		input_free_device(dev);
		return err;
	}

	input_set_drvdata(dev, chip);
	chip->input_dev = dev;

	hrtimer_init(&chip->tone_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	chip->tone_timer.function = snd_legoev3_cb_stop_tone;

	return 0;
}

/*--- ALSA PCM device ---*/

static struct snd_pcm_hardware snd_legoev3_playback_hw = {
	.info = (SNDRV_PCM_INFO_MMAP |
	         SNDRV_PCM_INFO_MMAP_VALID |
	         SNDRV_PCM_INFO_INTERLEAVED),
	.formats =          SNDRV_PCM_FMTBIT_S16_LE,
	.rates =            SNDRV_PCM_RATE_CONTINUOUS,
	.rate_min =         8000,
	.rate_max =         48000,
	.channels_min =     1,
	.channels_max =     1,
	.buffer_bytes_max = BUFFER_SIZE,
	.period_bytes_min = 4096,
	.period_bytes_max = BUFFER_SIZE,
	.periods_min =      1,
	.periods_max =      1024,
};

/*
 * Call snd_pcm_period_elapsed in a tasklet
 * This avoids spinlock messes and long-running irq contexts
 */
static void snd_legoev3_call_pcm_elapsed(unsigned long data)
{
	if (data)
	{
		struct snd_pcm_substream *substream = (void *)data;
		snd_pcm_period_elapsed(substream);
	}
}

static void snd_legoev3_period_elapsed(void* data)
{
	struct snd_legoev3 *chip = data;

	tasklet_schedule(&chip->pcm_period_tasklet);
}

/*
 * Only called when the ehrpwm interrupt is configured as regular IRQ and
 * not as a FIQ. In other words, this is for debugging (when assigned to IRQ)
 * and serves as a dummy callback during normal usage (when assigned to FIQ).
 */
//static int snd_legoev3_et_callback(struct ehrpwm_pwm *ehrpwm, void *data)
//{
//	fiq_c_handler_t handler = get_fiq_c_handler();
//
//	if (handler)
//		handler();
//
//	return 0;
//}

static int snd_legoev3_pcm_playback_open(struct snd_pcm_substream *substream)
{
	struct snd_legoev3 *chip = snd_pcm_substream_chip(substream);
	struct snd_pcm_runtime *runtime = substream->runtime;
	int err;

	if (chip->tone_frequency)
		return -EBUSY;

	err = legoev3_fiq_ehrpwm_request();
	if (err < 0)
		return err;

	runtime->hw = snd_legoev3_playback_hw;
//	err = ehrpwm_et_cb_register(chip->pwm, substream,
//				    snd_legoev3_et_callback);
	if (err < 0)
		goto err_ehrpwm_et_cb_register;

	pwm_config(chip->pwm, 0, chip->pwm->period);

	tasklet_init(&chip->pcm_period_tasklet, snd_legoev3_call_pcm_elapsed,
	             (unsigned long)substream);

	if (debug) printk(KERN_INFO "legoev3_pcm_playback_open\n");

	return 0;

err_ehrpwm_et_cb_register:
	legoev3_fiq_ehrpwm_release();
	return err;
}

static int snd_legoev3_pcm_playback_close(struct snd_pcm_substream *substream)
{
	struct snd_legoev3 *chip = snd_pcm_substream_chip(substream);
	
	if (chip)
	{
		tasklet_kill(&chip->pcm_period_tasklet);
		if (debug) printk(KERN_INFO "legoev3_pcm_playback_close\n");
	}
	legoev3_fiq_ehrpwm_release();

	return 0;
}

static int snd_legoev3_pcm_hw_params(struct snd_pcm_substream *substream,
                                     struct snd_pcm_hw_params *hw_params)
{
	return snd_pcm_lib_malloc_pages(substream, params_buffer_bytes(hw_params));
}

static int snd_legoev3_pcm_hw_free(struct snd_pcm_substream *substream)
{
	return snd_pcm_lib_free_pages(substream);
}

static int snd_legoev3_pcm_prepare(struct snd_pcm_substream *substream)
{
	struct snd_legoev3 *chip = snd_pcm_substream_chip(substream);
	int err, int_prd;

	/* interrupt can occur on every 1st, 2nd or 3rd pwm pulse */
	if (substream->runtime->rate > 32000)
		int_prd = 1;
	else if (substream->runtime->rate > 16000)
		int_prd = 2;
	else
		int_prd = 3;
	err = pwm_config(chip->pwm, chip->pwm->duty_cycle,
			 NSEC_PER_SEC / (substream->runtime->rate * int_prd));
	if (err < 0)
		return err;
	//err = ehrpwm_et_set_sel_evt(chip->pwm, ET_CTR_PRD, int_prd);
	if (err < 0)
		return err;

	if (debug)
        	printk(KERN_INFO "legoev3_pcm_prepare with sample rate=%d, factor=%d, ramp=%d\n",
		       substream->runtime->rate, int_prd, ramp_ms);

	gpio_set_value(chip->amp_gpio, 1);

	// TODO: second argument is ticks, not ns.
	legoev3_fiq_ehrpwm_prepare(substream, chip->pwm->period,
				   chip->volume, snd_legoev3_period_elapsed, chip);

	return 0;
}

static void snd_legoev3_pcm_stop(struct work_struct *work)
{
	struct snd_legoev3 *chip = container_of((struct delayed_work*)work,
	                                        struct snd_legoev3, pcm_stop);

	if (chip && !chip->pcm_stop_cancelled)
	{
		if (debug) printk(KERN_INFO "legoev3_pcm_stop\n");
		pwm_disable(chip->pwm);
		//ehrpwm_et_int_en_dis(chip->pwm, ET_DISABLE);
		gpio_set_value(chip->amp_gpio, 0);
	}
}

static int snd_legoev3_pcm_trigger(struct snd_pcm_substream *substream, int cmd)
{
	struct snd_legoev3 *chip = snd_pcm_substream_chip(substream);

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
		if (debug) printk(KERN_INFO "legoev3_pcm_trigger(start)\n");
		chip->pcm_stop_cancelled = true;
		cancel_delayed_work(&chip->pcm_stop);
		legoev3_fiq_ehrpwm_ramp(substream, 1, ramp_ms);
		//ehrpwm_et_int_en_dis(chip->pwm, ET_ENABLE);
		pwm_enable(chip->pwm);
		break;
	case SNDRV_PCM_TRIGGER_STOP:
		if (debug) printk(KERN_INFO "legoev3_pcm_trigger(stop)\n");
		legoev3_fiq_ehrpwm_ramp(substream, -1, ramp_ms);
		chip->pcm_stop_cancelled = false;
		schedule_delayed_work(&chip->pcm_stop, msecs_to_jiffies(1000));
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static snd_pcm_uframes_t
snd_legoev3_pcm_pointer(struct snd_pcm_substream *substream)
{
	return bytes_to_frames(substream->runtime,
			       legoev3_fiq_ehrpwm_get_playback_ptr());
}

static struct snd_pcm_ops snd_legoev3_playback_ops = {
	.open =        snd_legoev3_pcm_playback_open,
	.close =       snd_legoev3_pcm_playback_close,
	.ioctl =       snd_pcm_lib_ioctl,
	.hw_params =   snd_legoev3_pcm_hw_params,
	.hw_free =     snd_legoev3_pcm_hw_free,
	.prepare =     snd_legoev3_pcm_prepare,
	.trigger =     snd_legoev3_pcm_trigger,
	.pointer =     snd_legoev3_pcm_pointer,
};

static int snd_legoev3_new_pcm(struct snd_legoev3 *chip)
{
	struct snd_pcm *pcm;
	int err;

	err = snd_pcm_new(chip->card, "LEGO Mindstorms EV3", 0, 1, 0, &pcm);
	if (err < 0)
		return err;
	pcm->private_data = chip;
	strcpy(pcm->name, "LEGO Mindstorms EV3");
	chip->pcm = pcm;

	snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_PLAYBACK,
		&snd_legoev3_playback_ops);

	err = snd_pcm_lib_preallocate_pages_for_all(pcm,
		SNDRV_DMA_TYPE_CONTINUOUS, snd_dma_continuous_data (GFP_KERNEL),
		BUFFER_SIZE, BUFFER_SIZE);
	if (err < 0)
		return err;

	return 0;
}

/*--- volume control ---*/

static struct snd_kcontrol_new volume_control;

static int snd_legoev3_volume_control_info(struct snd_kcontrol *kcontrol,
                                           struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 1;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = MAX_VOLUME;
	return 0;
}

static int snd_legoev3_volume_control_get(struct snd_kcontrol *kcontrol,
                                          struct snd_ctl_elem_value *ucontrol)
{
	struct snd_legoev3 *chip = snd_kcontrol_chip(kcontrol);
	ucontrol->value.integer.value[0] = chip->volume;
	return 0;
}

static int snd_legoev3_volume_control_put(struct snd_kcontrol *kcontrol,
                                          struct snd_ctl_elem_value *ucontrol)
{
	struct snd_legoev3 *chip = snd_kcontrol_chip(kcontrol);
	int changed = 0, newValue = ucontrol->value.integer.value[0];
	
	if (newValue < 0) newValue = 0;
	if (newValue > MAX_VOLUME) newValue = MAX_VOLUME;

	if (chip->volume != newValue) {
		chip->volume = newValue;
	
		/* if tone or PCM playback is running, apply volume */
		if (chip->tone_frequency)
			snd_legoev3_apply_tone_volume(chip);
		if (chip->pcm->streams[0].substream_opened ||
		    chip->pcm->streams[1].substream_opened)
			legoev3_fiq_ehrpwm_set_volume(chip->volume);

		changed = 1;
	}

	return changed;
}

static struct snd_kcontrol_new volume_control = {
	.iface  = SNDRV_CTL_ELEM_IFACE_MIXER,
	.name   = "Playback Volume",
	.index  = 0,
	.access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
	.info   = snd_legoev3_volume_control_info,
	.get    = snd_legoev3_volume_control_get,
	.put    = snd_legoev3_volume_control_put
};

/**
 * snd_legoev3_show_volume - sysfs 'volume' attribute read
 *
 * output: volume in percent (0..100)
*/
static ssize_t snd_legoev3_show_volume(struct device *dev,
                                       struct device_attribute *attr,
                                       char *buf)
{
	struct snd_card *card = dev_get_drvdata(&to_platform_device(dev)->dev);
	struct snd_legoev3 *chip = card->private_data;

	return snprintf(buf, PAGE_SIZE, "%u\n", (chip->volume * 100)/MAX_VOLUME);
}

/**
 * snd_legoev3_store_volume - sysfs 'volume' attribute write
 *
 * input: volume in percent (0..100)
*/
static ssize_t snd_legoev3_store_volume(struct device *dev,
                                        struct device_attribute *attr,
                                        const char *buf, size_t count)
{
	struct snd_card *card = dev_get_drvdata(&to_platform_device(dev)->dev);
	struct snd_legoev3 *chip = card->private_data;

	const char *start = buf;
	      char *end   = (char*)buf;
	const char *last  = buf + count;

	int value;

	start = skip_spaces(end);
	if (last <= start)
		return -EINVAL;

	value = simple_strtol(start, &end, 0);
	if (end == start)
		return -EINVAL;

	if (value < 0)
		return -EINVAL;
	if (value > 100)
		return -EINVAL;

	chip->volume = value * MAX_VOLUME / 100;

	/* if tone or pcm playback is running, apply volume */
	if (chip->tone_frequency)
		snd_legoev3_apply_tone_volume(chip);
	if (chip->pcm->streams[0].substream_opened ||
	    chip->pcm->streams[1].substream_opened)
		legoev3_fiq_ehrpwm_set_volume(chip->volume);

	return count;
}


/*--- sysfs attributes ---*/

/**
 * snd_legoev3_show_mode - sysfs 'mode' attribute read
 *
 * output: 'tone' for tone mode
 *         'pcm'  for ALSA PCM playback mode
 *         'idle'
*/
static ssize_t snd_legoev3_show_mode(struct device *dev,
                                     struct device_attribute *attr,
                                     char *buf)
{
	struct snd_card *card = dev_get_drvdata(&to_platform_device(dev)->dev);
	struct snd_legoev3 *chip = card->private_data;

	if (chip->tone_frequency)
		return snprintf(buf, PAGE_SIZE, "tone\n");
	else if (chip->pcm->streams[0].substream_opened || 
	         chip->pcm->streams[1].substream_opened)
		return snprintf(buf, PAGE_SIZE, "pcm\n");

	return snprintf(buf, PAGE_SIZE, "idle\n");
}

static DEVICE_ATTR(mode,   0444, snd_legoev3_show_mode,   NULL);
static DEVICE_ATTR(tone,   0666, snd_legoev3_show_tone,   snd_legoev3_store_tone);
static DEVICE_ATTR(volume, 0666, snd_legoev3_show_volume, snd_legoev3_store_volume);

static struct attribute *snd_legoev3_attrs[] = {
    &dev_attr_mode.attr
  , &dev_attr_tone.attr
  , &dev_attr_volume.attr
  , NULL
};

static struct attribute_group snd_legoev3_attr_group = {
	.attrs = snd_legoev3_attrs,
};

/*--- platform sound device ---*/

static int snd_legoev3_create(struct snd_card *card,
                                        struct snd_legoev3_platform_data *pdata)
{
	struct snd_legoev3 *chip  = card->private_data;
	static struct snd_device_ops ops = { };
	int err;

	chip->card = card;
	chip->pwm = pdata->pwm;
	INIT_DELAYED_WORK(&chip->pcm_stop, snd_legoev3_pcm_stop);
	chip->pcm_stop_cancelled = false;	
	chip->amp_gpio = pdata->amp_gpio;

	err = snd_device_new(card, SNDRV_DEV_LOWLEVEL, chip, &ops);
	if (err < 0)
		return err;

	err = snd_legoev3_new_pcm(chip);
	if (err < 0)
		return err;

	err = snd_ctl_add(card, snd_ctl_new1(&volume_control, chip));
	if (err < 0)
		return err;

	return 0;
}

static int snd_legoev3_init_ehrpwm(struct pwm_device *pwm)
{
	int err;

	err = pwm_set_polarity(pwm, 0);
	if (err < 0)
		return err;
	err = pwm_config(pwm, 0, pwm->period);
	if (err < 0)
		return err;

	return 0;
}

static int snd_legoev3_probe(struct platform_device *pdev)
{
	struct snd_legoev3_platform_data *pdata;
	struct snd_card *card;
	int err;

	pdata = pdev->dev.platform_data;
	if (!pdata || !pdata->pwm_dev_name || !pdata->amp_gpio)
		return -ENXIO;

	pdata->pwm = pwm_get(&pdev->dev, NULL);
	if (IS_ERR(pdata->pwm))
		return PTR_ERR(pdata->pwm);

	err = snd_legoev3_init_ehrpwm(pdata->pwm);
	if (err < 0)
		goto err_snd_legoev3_init_ehrpwm;

	err = gpio_request_one(pdata->amp_gpio, GPIOF_OUT_INIT_LOW, "snd_ena");
	if (err)
		goto err_gpio_request_one;

	// configure maximal sample rate
	if (max_sample_rate < 8000)
		max_sample_rate = 8000;
	else if (max_sample_rate > 48000)
		max_sample_rate = 48000;
	
	snd_legoev3_playback_hw.rate_max = max_sample_rate;

	err = snd_card_create(-1, "legoev3", THIS_MODULE,
	                      sizeof(struct snd_legoev3), &card);
	if (err < 0)
		goto err_snd_card_create;

	err = snd_legoev3_create(card, pdata);
	if (err < 0)
		goto err_snd_legoev3_create;

	strcpy(card->driver, "legoev3");
	strcpy(card->shortname, "LEGO Mindstorms EV3 speaker");
	sprintf(card->longname, "%s connected to %s", card->shortname,
	        pdata->pwm_dev_name);

	err = snd_card_register(card);
	if (err < 0)
		goto err_snd_card_register;

	err = snd_legoev3_input_device_create(card);
	if (err < 0)
		goto err_snd_legoev3_input_device_create;

	dev_set_drvdata(&pdev->dev, card);

	err = sysfs_create_group(&pdev->dev.kobj, &snd_legoev3_attr_group);

	return 0;

err_snd_legoev3_input_device_create:
err_snd_card_register:
err_snd_legoev3_create:
	snd_card_free(card);
err_snd_card_create:
	gpio_free(pdata->amp_gpio);
err_gpio_request_one:
err_snd_legoev3_init_ehrpwm:
	pwm_put(pdata->pwm);
	pdata->pwm = NULL;
	return err;
}

static int snd_legoev3_remove(struct platform_device *pdev)
{
	struct snd_legoev3_platform_data *pdata = pdev->dev.platform_data;
	struct snd_card *card = dev_get_drvdata(&pdev->dev);
	struct snd_legoev3 *chip =  card->private_data;

	/* make sure sound is off */
	hrtimer_cancel(&chip->tone_timer);
	snd_legoev3_stop_tone(chip);

	chip->pcm_stop_cancelled = false;	
	cancel_delayed_work_sync(&chip->pcm_stop);

	sysfs_remove_group(&pdev->dev.kobj, &snd_legoev3_attr_group);
	input_unregister_device(chip->input_dev);
	input_free_device(chip->input_dev);
	snd_card_free(card);
	dev_set_drvdata(&pdev->dev, NULL);
	gpio_free(pdata->amp_gpio);
	pwm_put(pdata->pwm);
	pdata->pwm = NULL;

	return 0;
}

/*--- module ---*/

static struct platform_driver snd_legoev3_platform_driver = {
	.driver = {
		.name = "snd-legoev3",
	},
	.probe = snd_legoev3_probe,
	.remove = snd_legoev3_remove,
};

static int __init snd_legoev3_init(void)
{
	return platform_driver_register(&snd_legoev3_platform_driver);
}
module_init(snd_legoev3_init);

static void __exit snd_legoev3_exit(void)
{
	platform_driver_unregister(&snd_legoev3_platform_driver);
}
module_exit(snd_legoev3_exit);

MODULE_DESCRIPTION("LEGO Mindstorms EV3 speaker driver");
MODULE_AUTHOR("David Lechner <david@lechnology.com>");
MODULE_AUTHOR("Franz Detro <franz.detro@gmx.de>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:snd-legoev3");
