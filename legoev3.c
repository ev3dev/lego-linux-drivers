/*
 * LEGO Mindstorms EV3 sound driver
 *
 * Copyright (C) 2013 David Lechner <david@lechnology.com>
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
#include <linux/init.h>
#include <linux/input.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/pwm/ehrpwm.h>
#include <linux/slab.h>

#include <sound/control.h>
#include <sound/core.h>
#include <sound/initval.h>
#include <sound/legoev3.h>
#include <sound/pcm.h>

#include "legoev3.h"

#define BUFFER_SIZE (64*1024)
#define TONE_MIN_HZ 100
#define TONE_MAX_HZ 10000

struct snd_legoev3 {
	struct pwm_device *pwm;
	struct snd_card *card;
	struct input_dev *input_dev;
	struct snd_pcm *pcm;
	unsigned amp_gpio;
	char tone_busy;
	size_t playback_ptr;
	unsigned long et_callback_count;
	int global_volume;
};

static struct snd_pcm_hardware snd_legoev3_playback_hw = {
	.info = (SNDRV_PCM_INFO_MMAP |
	         SNDRV_PCM_INFO_MMAP_VALID |
	         SNDRV_PCM_INFO_INTERLEAVED),
	.formats =          SNDRV_PCM_FMTBIT_S16_LE,
	.rates =            SNDRV_PCM_RATE_8000_48000,
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

static unsigned int rates[] = { 48000 };
static struct snd_pcm_hw_constraint_list constraints_rates = {
	.count = ARRAY_SIZE(rates),
	.list = rates,
	.mask = 0,
};

static struct snd_kcontrol_new global_volume_control;

static int snd_legoev3_do_tone(struct snd_legoev3 *chip, int hz)
{
	int err;

	if (chip->pcm->streams[0].substream_opened ||
	    chip->pcm->streams[1].substream_opened)
		return -EBUSY;

	if (hz <= 0) {
		gpio_set_value(chip->amp_gpio, 0);
		pwm_stop(chip->pwm);
		pwm_set_duty_percent(chip->pwm, 0);
		chip->tone_busy = 0;
		return 0;
	}
	if (hz < TONE_MIN_HZ)
		hz = TONE_MIN_HZ;
	if (hz > TONE_MAX_HZ)
		hz = TONE_MAX_HZ;
	err = pwm_set_frequency(chip->pwm, hz);
	if (err < 0)
		return err;
	/* TODO: add volume parameter and factor in here*/
	err = pwm_set_duty_percent(chip->pwm, 50);
	if (err < 0)
		return err;
	err = pwm_start(chip->pwm);
	if (err < 0)
		return err;
	gpio_set_value(chip->amp_gpio, 1);

	return 0;
}

static void snd_legoev3_stop_tone(struct snd_legoev3 * chip)
{
	snd_legoev3_do_tone(chip, 0);
}

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

static int snd_legoev3_et_callback(struct ehrpwm_pwm *ehrpwm, void *data)
{
	struct snd_pcm_substream *substream = (struct snd_pcm_substream *)data;
	struct snd_legoev3 *chip = snd_pcm_substream_chip(substream);
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct pwm_device *pwm = chip->pwm;
	unsigned long duty_ticks;

	duty_ticks = (*(short *)(runtime->dma_area + chip->playback_ptr)
	              ^ 0x8000) * pwm->period_ticks / 0xffff *
	              chip->global_volume / 0xffff;

	pwm_set_duty_ticks(pwm, duty_ticks);

	if (chip->et_callback_count++ >= runtime->period_size * 2) {
		chip->et_callback_count %= runtime->period_size * 2;
		snd_pcm_period_elapsed(substream);
	}

	chip->playback_ptr += frames_to_bytes(runtime, 1);
	if (chip->playback_ptr >= runtime->dma_bytes)
		chip->playback_ptr = 0;

	return 0;
}

static int __devinit snd_legoev3_new_pcm(struct snd_legoev3 *chip);

static int __devinit snd_legoev3_create(struct snd_card *card,
                                        struct snd_legoev3_platform_data *pdata)
{
	struct snd_legoev3 *chip  = card->private_data;
	static struct snd_device_ops ops = { };
	int err;

	chip->card = card;
	chip->pwm = pdata->pwm;
	chip->amp_gpio = pdata->amp_gpio;

	err = snd_device_new(card, SNDRV_DEV_LOWLEVEL, chip, &ops);
	if (err < 0)
		return err;

	err = snd_legoev3_new_pcm(chip);
	if (err < 0)
		return err;

	err = snd_ctl_add(card, snd_ctl_new1(&global_volume_control, chip));
	if (err < 0)
		return err;

	return 0;
}

static int __devinit snd_legoev3_init_ehrpwm(struct pwm_device *pwm)
{
	int err;

	err = ehrpwm_tb_set_phase(pwm, 0);
	if (err < 0)
		return err;
	err = ehrpwm_tb_set_counter(pwm, 0);
	if (err < 0)
		return err;
	err = ehrpwm_tb_config_sync(pwm, TB_DISABLE, TB_SYNC_DISABLE);
	if (err < 0)
		return err;
	err = ehrpwm_tb_set_counter_mode(pwm, TB_FREEZE, TB_DOWN);
	if (err)
		return err;
	err = ehrpwm_tb_set_periodload(pwm, TB_SHADOW);
	if (err < 0)
		return err;
	err = ehrpwm_cmp_set_cmp_ctl(pwm, CC_SHADOW, CC_SHADOW, CC_CTR_ZERO,
	                             CC_CTR_ZERO);
	if (err < 0)
		return err;
	err = ehrpwm_db_set_mode(pwm, DB_DISABLE, DB_ACTIVE_HIGH, DB_DISABLE);
	if (err < 0)
		return err;
	err = ehrpwm_pc_en_dis(pwm, PC_DISABLE);
	if (err < 0)
		return err;
	err = ehrpwm_et_set_sel_evt(pwm, ET_CTR_PRD, ET_1ST);
	if (err < 0)
		return err;
	err = ehrpwm_hr_config(pwm, HR_CTR_ZERO, HR_DUTY, HR_MEP_DISABLE);
	if (err < 0)
		return err;
	err = pwm_set_duty_ticks(pwm, 0);
	if (err < 0)
		return err;

	return 0;
}

static int __devinit snd_legoev3_input_device_create(struct snd_card *card)
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

	return 0;
}

static int __devinit snd_legoev3_probe(struct platform_device *pdev)
{
	struct snd_legoev3_platform_data *pdata;
	struct snd_card *card;
	int err;

	pdata = pdev->dev.platform_data;
	if (!pdata || !pdata->pwm_dev_name || !pdata->amp_gpio)
		return -ENXIO;
	pdata->pwm = pwm_request_byname(pdata->pwm_dev_name, "snd-legoev3");
	if (IS_ERR(pdata->pwm))
		return PTR_ERR(pdata->pwm);

	err = snd_card_create(-1, "legoev3", THIS_MODULE,
	                      sizeof(struct snd_legoev3), &card);
	if (err < 0)
		goto err1;

	err = snd_legoev3_create(card, pdata);
	if (err < 0)
		goto err2;

	err = snd_legoev3_init_ehrpwm(pdata->pwm);
	if (err < 0)
		goto err2;

	strcpy(card->driver, "legoev3");
	strcpy(card->shortname, "LEGO Mindstorms EV3 speaker");
	sprintf(card->longname, "%s connected to %s", card->shortname,
	        pdata->pwm_dev_name);

	err = snd_card_register(card);
	if (err < 0)
		goto err2;

	err = snd_legoev3_input_device_create(card);
	if (err < 0)
		goto err2;

	dev_set_drvdata(&pdev->dev, card);
	return 0;

err2:
	snd_card_free(card);
err1:
	pwm_release(pdata->pwm);
	pdata->pwm = NULL;
	return err;
}

static int __devexit snd_legoev3_remove(struct platform_device *pdev)
{
	struct snd_legoev3_platform_data *pdata = pdev->dev.platform_data;
	struct snd_card *card = dev_get_drvdata(&pdev->dev);
	struct snd_legoev3 *chip =  card->private_data;

	/* TODO: make sure sound is off */
	input_unregister_device(chip->input_dev);
	input_free_device(chip->input_dev);
	snd_card_free(card);
	dev_set_drvdata(&pdev->dev, NULL);
	pwm_release(pdata->pwm);
	pdata->pwm = NULL;
	return 0;
}

static int snd_legoev3_pcm_playback_open(struct snd_pcm_substream *substream)
{
	struct snd_legoev3 *chip = snd_pcm_substream_chip(substream);
	struct snd_pcm_runtime *runtime = substream->runtime;
	int err;

	if (chip->tone_busy)
		return -EBUSY;

	err = ehrpwm_tb_set_prescalar_val(chip->pwm, TB_DIV1, TB_HS_DIV1);
	if (err < 0)
		return err;

	err = pwm_set_frequency(chip->pwm, 48000);
	if (err < 0)
		return err;

	err = snd_pcm_hw_constraint_list(substream->runtime, 0,
	                                 SNDRV_PCM_HW_PARAM_RATE,
	                                 &constraints_rates);
	if (err < 0)
		return err;

	runtime->hw = snd_legoev3_playback_hw;
	err = ehrpwm_et_cb_register(chip->pwm, substream,
				    snd_legoev3_et_callback);
	if (err < 0)
		return err;

	pwm_set_duty_ticks(chip->pwm, 0);

	return 0;
}

static int snd_legoev3_pcm_playback_close(struct snd_pcm_substream *substream)
{
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

	chip->playback_ptr = 0;
	chip->et_callback_count = 0;

	return 0;
}

static int snd_legoev3_pcm_trigger(struct snd_pcm_substream *substream, int cmd)
{
	struct snd_legoev3 *chip = snd_pcm_substream_chip(substream);

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
		ehrpwm_et_int_en_dis(chip->pwm, ET_ENABLE);
		gpio_set_value(chip->amp_gpio, 1);
		pwm_start(chip->pwm);
		break;
	case SNDRV_PCM_TRIGGER_STOP:
		gpio_set_value(chip->amp_gpio, 0);
		ehrpwm_et_int_en_dis(chip->pwm, ET_DISABLE);
		pwm_set_duty_ticks(chip->pwm, 0);
		pwm_stop(chip->pwm);
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

	return bytes_to_frames(substream->runtime, chip->playback_ptr);
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

static int __devinit snd_legoev3_new_pcm(struct snd_legoev3 *chip)
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

static int global_volume_control_info(struct snd_kcontrol *kcontrol,
                                      struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 1;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = 0xffff;
	return 0;
}

static int global_volume_control_get(struct snd_kcontrol *kcontrol,
                                     struct snd_ctl_elem_value *ucontrol)
{
	struct snd_legoev3 *chip = snd_kcontrol_chip(kcontrol);
	ucontrol->value.integer.value[0] = chip->global_volume;
	return 0;
}

static int global_volume_control_put(struct snd_kcontrol *kcontrol,
                                     struct snd_ctl_elem_value *ucontrol)
{
	struct snd_legoev3 *chip = snd_kcontrol_chip(kcontrol);
	int changed = 0;
	if (chip->global_volume != ucontrol->value.integer.value[0]) {
		chip->global_volume = ucontrol->value.integer.value[0];
		changed = 1;
	}
	return changed;
}

static struct snd_kcontrol_new global_volume_control = {
	.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
	.name = "Playback Volume",
	.index = 0,
	.access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
	.info = global_volume_control_info,
	.get = global_volume_control_get,
	.put = global_volume_control_put
};

static struct platform_driver snd_legoev3_platform_driver = {
	.driver = {
		.name = "snd-legoev3",
	},
	.probe = snd_legoev3_probe,
	.remove = __devexit_p(snd_legoev3_remove),
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
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:snd-legoev3");
MODULE_SUPPORTED_DEVICE("{{ev3dev,legoev3}}");
