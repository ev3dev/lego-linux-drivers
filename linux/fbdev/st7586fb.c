/*
 * st7856fb.c - fbdev driver for ST7856 display controller
 *
 * Layout is based on skeletonfb.c by James Simmons and Geert Uytterhoeven.
 *
 * Copyright (C) 2011 Matt Porter <matt@ohporter.com>
 * Copyright (C) 2011 Texas Instruments
 * Copyright (C) 2013-2014,2017 David Lechner <david@lechnology.com>
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License. See the file COPYING in the main directory of this archive for
 * more details.
 */

#include <linux/delay.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/errno.h>
#include <linux/fb.h>
#include <linux/gpio/consumer.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/spi/spi.h>
#include <linux/string.h>
#include <linux/uaccess.h>
#include <linux/vmalloc.h>

#define FB_ST7586_UPDATE_DELAY (HZ / 20)

/* Supported display modules */
enum {
	ST7586_DISPLAY_LEGO_EV3, /* LEGO MINDSTORMS EV3 LCD */
};

/* Init script function */
struct st7586_function {
	u16 cmd;
	u16 data;
};

/* Init script commands */
enum st7586_cmd {
	ST7586_START,
	ST7586_END,
	ST7586_CMD,
	ST7586_DATA,
	ST7586_DELAY,
	ST7586_CLR,
};

struct st7586_par {
	struct delayed_work dwork;
	struct spi_device *spi;
	struct fb_info *info;
	struct gpio_desc *rst;
	struct gpio_desc *a0;
	u8 *buf;
	u8 *display_data;
	int display_data_size;
};

struct st7586_display_data {
	const char *id;
	int xres;
	int yres;
	int width_mm;
	int height_mm;
};

/* ST7586 Commands */
#define ST7586_NOP	0x00
#define ST7586_SWRESET	0x01
#define ST7586_SLPIN	0x10
#define ST7586_SLPOUT	0x11
#define ST7586_PTLON	0x12
#define ST7586_NORON	0x13
#define ST7586_INVOFF	0x20
#define ST7586_INVON	0x21
#define ST7586_APOFF	0x22
#define ST7586_APON	0x23
#define ST7586_DISPOFF	0x28
#define ST7586_DISPON	0x29
#define ST7586_CASET	0x2A
#define ST7586_RASET	0x2B
#define ST7586_RAMWR	0x2C
#define ST7586_RAMRD	0x2E
#define ST7586_PARAREA	0x30
#define ST7586_DSPCTL	0x36
#define ST7586_DSPGRAY	0x38
#define ST7586_DSPMONO	0x39
#define ST7586_RAMENB	0x3A
#define ST7586_DSPDUTY	0xB0
#define ST7586_PARDISP	0xB4
#define ST7586_NLNINV	0xB5
#define ST7586_VOP	0xC0
#define ST7586_BIAS	0xC3
#define ST7586_BOOST	0xC4
#define ST7586_VOPOFF	0xC7
#define ST7586_ANCTL	0xD0
#define ST7586_ARDCTL	0xD7
#define ST7586_OTPRWCTL	0xE0
#define ST7586_OTPCOUT	0xE1
#define ST7586_OTPWR	0xE2
#define ST7586_OTPRD	0xE3

static const struct st7586_function st7586_cfg_script[] = {
	{ ST7586_START, ST7586_START},
	{ ST7586_CMD, ST7586_ARDCTL},
	{ ST7586_DATA, 0x9f},
	{ ST7586_CMD, ST7586_OTPRWCTL},
	{ ST7586_DATA, 0x00},
	{ ST7586_DELAY, 10},
	{ ST7586_CMD, ST7586_OTPRD},
	{ ST7586_DELAY, 20},
	{ ST7586_CMD, ST7586_OTPCOUT},
	{ ST7586_CMD, ST7586_SLPOUT},
	{ ST7586_CMD, ST7586_DISPOFF},
	{ ST7586_DELAY, 50},
	{ ST7586_CMD, ST7586_VOPOFF},
	{ ST7586_DATA, 0x00},
	{ ST7586_CMD, ST7586_VOP},
	{ ST7586_DATA, 0xE3},
	{ ST7586_DATA, 0x00},
	{ ST7586_CMD, ST7586_BIAS},
	{ ST7586_DATA, 0x02},
	{ ST7586_CMD, ST7586_BOOST},
	{ ST7586_DATA, 0x04},
	{ ST7586_CMD, ST7586_ANCTL},
	{ ST7586_DATA, 0x1d},
	{ ST7586_CMD, ST7586_NLNINV},
	{ ST7586_DATA, 0x00},
	{ ST7586_CMD, ST7586_DSPMONO},
	{ ST7586_CMD, ST7586_RAMENB},
	{ ST7586_DATA, 0x02},
	{ ST7586_CMD, ST7586_DSPCTL},
	{ ST7586_DATA, 0x00},
	{ ST7586_CMD, ST7586_DSPDUTY},
	{ ST7586_DATA, 0x7f},
	{ ST7586_CMD, ST7586_PARDISP},
	{ ST7586_DATA, 0xa0},
	{ ST7586_CMD, ST7586_PARAREA},
	{ ST7586_DATA, 0x00},
	{ ST7586_DATA, 0x00},
	{ ST7586_DATA, 0x00},
	{ ST7586_DATA, 0x77},
	{ ST7586_CMD, ST7586_INVOFF},
	{ ST7586_CMD, ST7586_CASET},
	{ ST7586_DATA, 0x00},
	{ ST7586_DATA, 0x00},
	{ ST7586_DATA, 0x00},
	{ ST7586_DATA, 0x7f},
	{ ST7586_CMD, ST7586_RASET},
	{ ST7586_DATA, 0x00},
	{ ST7586_DATA, 0x00},
	{ ST7586_DATA, 0x00},
	{ ST7586_DATA, 0x9f},
	{ ST7586_CLR, ST7586_CLR},
	{ ST7586_DELAY, 100},
	{ ST7586_END, ST7586_END},
};

static const struct st7586_display_data st7586_displays[] = {
	[ST7586_DISPLAY_LEGO_EV3] = {
		.id		= "LEGO EV3 LCD",
		.xres		= 178,
		.yres		= 128,
		.width_mm	= 37,
		.height_mm	= 27,
	},
};

static inline int _st7586_write(struct st7586_par *par, u8 data)
{
	par->buf[0] = data;

	return spi_write(par->spi, par->buf, 1);
}

static int st7586_write_data(struct st7586_par *par, u8 data)
{
	/* Set data mode */
	gpiod_set_value(par->a0, 1);

	return _st7586_write(par, data);
}

static int st7586_write_data_buf(struct st7586_par *par, u8 *txbuf, int size)
{
	/* Set data mode */
	gpiod_set_value(par->a0, 1);

	/* Write entire buffer */
	return spi_write(par->spi, txbuf, size);
}

static int st7586_write_cmd(struct st7586_par *par, u8 data)
{
	/* Set command mode */
	gpiod_set_value(par->a0, 0);

	return _st7586_write(par, data);
}

static void st7586_clear_ddrram(struct st7586_par *par)
{
	u8 *buf;

	buf = kzalloc(128*128, GFP_KERNEL);
	st7586_write_cmd(par, ST7586_RAMWR);
	st7586_write_data_buf(par, buf, 128*128);
	kfree(buf);
}

static void st7586_run_cfg_script(struct st7586_par *par)
{
	int i;

	for (i = 0; ; i++) {
		switch (st7586_cfg_script[i].cmd) {
		case ST7586_START:
			break;
		case ST7586_CLR:
			st7586_clear_ddrram(par);
			break;
		case ST7586_CMD:
			st7586_write_cmd(par, st7586_cfg_script[i].data & 0xff);
			break;
		case ST7586_DATA:
			st7586_write_data(par, st7586_cfg_script[i].data & 0xff);
			break;
		case ST7586_DELAY:
			mdelay(st7586_cfg_script[i].data);
			break;
		case ST7586_END:
			return;
		}
	}
}

static void st7586_set_addr_win(struct st7586_par *par,
				int xs, int ys, int xe, int ye)
{
	st7586_write_cmd(par, ST7586_CASET);
	st7586_write_data(par, 0x00);
	st7586_write_data(par, xs);
	st7586_write_data(par, 0x00);
	st7586_write_data(par, ((xe+2)/3)-1);
	st7586_write_cmd(par, ST7586_RASET);
	st7586_write_data(par, 0x00);
	st7586_write_data(par, ys);
	st7586_write_data(par, 0x00);
	st7586_write_data(par, ye-1);
}

static void st7586_reset(struct st7586_par *par)
{
	/* Reset controller */
	gpiod_set_value(par->rst, 1);
	mdelay(10);
	gpiod_set_value(par->rst, 0);
	mdelay(120);
}

static const u8 st7586_1bpp_lookup[] = {
	0xFF, 0x1F, 0xE3, 0x03, 0xFC, 0x1C, 0xE0, 0x00,
};

static const u8 st7586_2bpp_lookup[] = {
	0xFF, 0x9F, 0x5F, 0x1F, 0xF3, 0x93, 0x53, 0x13,
	0xEB, 0x8B, 0x4B, 0x0B, 0xE3, 0x83, 0x43, 0x03,
	0xFE, 0x9E, 0x5E, 0x1E, 0xF2, 0x92, 0x52, 0x12,
	0xEA, 0x8A, 0x4A, 0x0A, 0xE2, 0x82, 0x42, 0x02,
	0xFD, 0x9D, 0x5D, 0x1D, 0xF1, 0x91, 0x51, 0x11,
	0xE9, 0x89, 0x49, 0x09, 0xE1, 0x81, 0x41, 0x01,
	0xFC, 0x9C, 0x5C, 0x1C, 0xF0, 0x90, 0x50, 0x10,
	0xE8, 0x88, 0x48, 0x08, 0xE0, 0x80, 0x40, 0x00,
};

static void st7586_update_display(struct st7586_par *par)
{
	const int bytes_per_row_in = par->info->fix.line_length;
	const int bytes_per_row_out = (par->info->var.xres + 2) / 3;
	int i = 0;
	int row;
	u8 *in, *out;
	u16 in_val;

	switch (par->info->var.bits_per_pixel) {
	case 1:
		/* Convert 1bpp to ST7586 3 pixel per byte format */
		for (row = 0; row < par->info->var.yres; row++) {
			in = par->info->screen_base + row * bytes_per_row_in;
			out = par->display_data + row * bytes_per_row_out;
			for (i = 0; i < bytes_per_row_in;) {
				in_val = in[i++];
				*out++ = st7586_1bpp_lookup[in_val & 0x7];
				in_val >>= 3;
				*out++ = st7586_1bpp_lookup[in_val & 0x7];
				in_val >>= 3;
				in_val |= in[i++] << 2;
				*out++ = st7586_1bpp_lookup[in_val & 0x7];
				in_val >>= 3;
				*out++ = st7586_1bpp_lookup[in_val & 0x7];
				in_val >>= 3;
				*out++ = st7586_1bpp_lookup[in_val & 0x7];
				in_val >>= 3;
				in_val |= in[i++] << 1;
				*out++ = st7586_1bpp_lookup[in_val & 0x7];
				in_val >>= 3;
				*out++ = st7586_1bpp_lookup[in_val & 0x7];
				in_val >>= 3;
				*out++ = st7586_1bpp_lookup[in_val & 0x7];
			}
		}
		break;

	case 2:
		/* Convert 2bpp to ST7586 3 pixel per byte format */
		for (row = 0; row < par->info->var.yres; row++) {
			in = par->info->screen_base + row * bytes_per_row_in;
			out = par->display_data + row * bytes_per_row_out;
			for (i = 0; i < bytes_per_row_in;) {
				in_val = in[i++];
				*out++ = st7586_2bpp_lookup[in_val & 0x3F];
				in_val >>= 6;
				in_val |= in[i++] << 2;
				*out++ = st7586_2bpp_lookup[in_val & 0x3F];
				in_val >>= 6;
				in_val |= in[i++] << 4;
				*out++ = st7586_2bpp_lookup[in_val & 0x3F];
				in_val >>= 6;
				*out++ = st7586_2bpp_lookup[in_val & 0x3F];
			}
		}
		break;
	}

	st7586_set_addr_win(par, 0, 0, par->info->var.xres, par->info->var.yres);
	st7586_write_cmd(par, ST7586_RAMWR);

	/* Blast framebuffer to ST7586 internal display RAM */
	st7586_write_data_buf(par, par->display_data, par->display_data_size);
}

static void st7586_deferred_work(struct work_struct *w)
{
	struct st7586_par *par = container_of(w, struct st7586_par, dwork.work);

	st7586_update_display(par);
}

static void st7586_deferred_io(struct fb_info *info,
				 struct list_head *pagelist)
{
	struct st7586_par *par = info->par;

	st7586_update_display(par);
}

static int st7586_init_display(struct st7586_par *par)
{
	st7586_reset(par);
	st7586_run_cfg_script(par);

	/* Set row/column data window */
	st7586_set_addr_win(par, 0, 0, par->info->var.xres, par->info->var.yres);

	st7586_write_cmd(par, ST7586_DISPON);

	return 0;
}

void st7586_fillrect(struct fb_info *info, const struct fb_fillrect *rect)
{
	struct st7586_par *par = info->par;

	sys_fillrect(info, rect);
	schedule_delayed_work(&par->dwork, FB_ST7586_UPDATE_DELAY);
}

void st7586_copyarea(struct fb_info *info, const struct fb_copyarea *area)
{
	struct st7586_par *par = info->par;

	sys_copyarea(info, area);
	schedule_delayed_work(&par->dwork, FB_ST7586_UPDATE_DELAY);
}

void st7586_imageblit(struct fb_info *info, const struct fb_image *image)
{
	struct st7586_par *par = info->par;

	sys_imageblit(info, image);
	schedule_delayed_work(&par->dwork, FB_ST7586_UPDATE_DELAY);
}

static ssize_t st7586_write(struct fb_info *info, const char __user *buf,
			    size_t count, loff_t *ppos)
{
	struct st7586_par *par = info->par;
	unsigned long p = *ppos;
	unsigned long total_size;
	void *dst;
	int err = 0;

	if (info->state != FBINFO_STATE_RUNNING)
		return -EPERM;

	total_size = info->fix.smem_len;

	if (p > total_size)
		return -EFBIG;

	if (count > total_size) {
		err = -EFBIG;
		count = total_size;
	}

	if (count + p > total_size) {
		if (!err)
			err = -ENOSPC;

		count = total_size - p;
	}

	dst = (void __force *)(info->screen_base + p);

	if (copy_from_user(dst, buf, count))
		err = -EFAULT;

	if  (!err)
		*ppos += count;

	schedule_delayed_work(&par->dwork, FB_ST7586_UPDATE_DELAY);

	return (err) ? err : count;
}

static int st7586_check_var(struct fb_var_screeninfo *var, struct fb_info *info)
{
	switch (var->bits_per_pixel) {
	case 1:
		var->grayscale = false;
		break;
	case 2:
		var->grayscale = true;
		break;
	default:
		return -EINVAL;
	}

	if (var->xres != info->var.xres ||
	    var->yres != info->var.yres ||
	    var->xres_virtual != info->var.xres_virtual ||
	    var->yres_virtual != info->var.yres_virtual)
		return -EINVAL;

	var->red.offset = 0;
	var->red.length = var->bits_per_pixel;
	var->red.msb_right = 0;
	var->green = var->red;
	var->blue = var->red;
	var->transp.offset = 0;
	var->transp.length = 0;
	var->transp.msb_right = 0;

	var->width = info->var.width;
	var->height = info->var.height;

	return 0;
}

static int st7586_set_par(struct fb_info *info)
{
	u32 line_length;

	switch (info->var.bits_per_pixel) {
	case 1:
		st7586_write_cmd(info->par, ST7586_DSPMONO);
		info->fix.visual = FB_VISUAL_MONO10;
		break;
	case 2:
		st7586_write_cmd(info->par, ST7586_DSPGRAY);
		info->fix.visual = FB_VISUAL_STATIC_PSEUDOCOLOR;
		break;
	}

	line_length = (info->var.xres * info->var.bits_per_pixel + 7) / 8;
	/* align to a multiple of 12 for st7586_update_display()  */
	info->fix.line_length = (line_length + 11) / 12 * 12;

	return 0;
}

static int st7586_mmap(struct fb_info *info, struct vm_area_struct *vma)
{
	return dma_mmap_coherent(info->dev, vma, info->screen_base,
				 info->fix.smem_start, info->fix.smem_len);
}

static struct fb_ops st7586_ops = {
	.owner		= THIS_MODULE,
	.fb_read	= fb_sys_read,
	.fb_write	= st7586_write,
	.fb_check_var	= st7586_check_var,
	.fb_set_par	= st7586_set_par,
	.fb_fillrect	= st7586_fillrect,
	.fb_copyarea	= st7586_copyarea,
	.fb_imageblit	= st7586_imageblit,
	.fb_mmap	= st7586_mmap,
};

static struct fb_deferred_io st7586_defio = {
	.delay		= FB_ST7586_UPDATE_DELAY,
	.deferred_io	= st7586_deferred_io,
};

static const struct of_device_id st7586_of_ids[] = {
	{ .compatible = "lego,ev3-lcd" },
	{ }
};
MODULE_DEVICE_TABLE(of, st7586_of_ids);

static int st7586_probe(struct spi_device *spi)
{
	const struct st7586_display_data *data;
	struct st7586_par *par;
	struct fb_info *info;
	kernel_ulong_t id;
	dma_addr_t dma_addr;
	int ret;

	id = spi_get_device_id(spi)->driver_data;

	if (id >= ARRAY_SIZE(st7586_displays)) {
		dev_err(&spi->dev, "Could not find matching device\n");
		return -EINVAL;
	}
	data = &st7586_displays[id];

	info = framebuffer_alloc(sizeof(*par), &spi->dev);
	if (!info)
		return -ENOMEM;

	strncpy(info->fix.id, data->id, sizeof(info->fix.id));
	info->fix.type = FB_TYPE_PACKED_PIXELS;
	info->fix.visual = FB_VISUAL_MONO10;
	info->fix.line_length = (data->xres + 7) / 8;
	/* align to a multiple of 12 for st7586_update_display()  */
	info->fix.line_length = (info->fix.line_length + 11) / 12 * 12;
	info->fix.accel = FB_ACCEL_NONE;

	info->var.xres = data->xres;
	info->var.yres = data->yres;
	info->var.xres_virtual = info->var.xres;
	info->var.yres_virtual = info->var.yres;
	info->var.bits_per_pixel = 1;
	info->var.grayscale = true;
	info->var.red.length = info->var.bits_per_pixel;
	info->var.green.length = info->var.bits_per_pixel;
	info->var.blue.length = info->var.bits_per_pixel;
	info->var.width = data->width_mm;
	info->var.height = data->height_mm;

	info->fbops = &st7586_ops;
	info->flags = FBINFO_DEFAULT | FBINFO_VIRTFB;
	info->fbdefio = &st7586_defio;
	fb_deferred_io_init(info);

	/* make sure we allocate enough for both mono and grayscale mode */
	info->fix.smem_len = info->fix.line_length * info->var.yres * 2;
	info->screen_base = dma_alloc_coherent(info->dev, info->fix.smem_len,
					       &dma_addr, GFP_KERNEL);
	info->fix.smem_start = dma_addr;
	if (!info->screen_base) {
		ret = -ENOMEM;
		goto err1;
	}

	/*
	 * Zero memory for easy detection of the first time data is written to
	 * the framebuffer.
	 */
	memset(info->screen_base, 0, info->fix.smem_len);

	spi_set_drvdata(spi, info);

	par = info->par;

	par->display_data_size = (info->var.xres + 2) / 3 * info->var.yres;
	par->display_data = devm_kmalloc(&spi->dev, par->display_data_size,
					 GFP_KERNEL);
	if (!par->display_data) {
		ret = -ENOMEM;
		goto err1;
	}

	par->rst = devm_gpiod_get(&spi->dev, "rst", GPIOD_ASIS);
	ret = PTR_ERR_OR_ZERO(par->rst);
	if (ret) {
		if (ret != -EPROBE_DEFER)
			dev_err(&spi->dev, "Failed to get rst gpio\n");
		goto err1;
	}

	par->a0 = devm_gpiod_get(&spi->dev, "a0", GPIOD_ASIS);
	ret = PTR_ERR_OR_ZERO(par->a0);
	if (ret) {
		if (ret != -EPROBE_DEFER)
			dev_err(&spi->dev, "Failed to get a0 gpio\n");
		goto err1;
	}

	par->info = info;
	par->spi = spi;
	par->buf = devm_kmalloc(&spi->dev, 1, GFP_KERNEL);
	INIT_DELAYED_WORK(&par->dwork, st7586_deferred_work);

	ret = st7586_init_display(par);
	if (ret < 0)
		goto err2;

	ret = register_framebuffer(info);
	if (ret < 0)
		goto err2;

	return 0;

err2:
	dma_free_coherent(info->dev, info->fix.smem_len, info->screen_base,
			  info->fix.smem_start);
err1:
	framebuffer_release(info);

	return ret;
}

static int st7586_remove(struct spi_device *spi)
{
	struct fb_info *info = spi_get_drvdata(spi);
	struct st7586_par *par = info->par;

	unregister_framebuffer(info);
	cancel_delayed_work_sync(&par->dwork);
	dma_free_coherent(info->dev, info->fix.smem_len, info->screen_base,
			  info->fix.smem_start);
	framebuffer_release(info);

	return 0;
}

static const struct spi_device_id st7586_ids[] = {
	/* ids must match of compatible strings less the vendor prefix */
	{ "ev3-lcd", ST7586_DISPLAY_LEGO_EV3 },
	{ }
};
MODULE_DEVICE_TABLE(spi, st7586_ids);

static struct spi_driver st7586_driver = {
	.driver = {
		.name		= "st7586fb",
		.of_match_table	= st7586_of_ids,
	},
	.id_table	= st7586_ids,
	.probe		= st7586_probe,
	.remove		= st7586_remove,
};
module_spi_driver(st7586_driver);

MODULE_DESCRIPTION("Framebuffer driver for ST7586 display controller");
MODULE_AUTHOR("David Lechner <david@lechnology.com>");
MODULE_LICENSE("GPL");
