/*
 * TM024HBH40 2.4' TFT LCD Module
 *
 * Kim, Kyoung Ho <kimkyoungho@gmail.com>
 *
 * Copyright 2012 Kim Kyoung Ho, All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include <linux/init.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/notifier.h>
#include <linux/regulator/consumer.h>
#include <linux/platform_device.h>
#include <linux/sysfs.h>
#include <linux/gpio.h>

#include <mach/device.h>
#include <mach/lcdif.h>
#include <mach/regs-pwm.h>
#include <mach/system.h>
#include <mach/pinctrl.h>

#define REGS_PWM_BASE IO_ADDRESS(PWM_PHYS_ADDR)

#define _H_ACTIVE  240
#define _V_ACTIVE  320

#define LCD_DELAY		0xffff
#define LCD_INDEX		0xfffe
#define LCD_DATA		0xfffc

#define N_ELEMENTS(arr)		(sizeof(arr) / sizeof(arr[0]))

#define _LCDIF_PACKAGING_FORMAT_REG	0x03
#define _LCDIF_PACKAGING_FORMAT_DATA	0x0f

#define _lcdif_write(reg, data)	__raw_writel(data, REGS_LCDIF_BASE + reg)
#define _lcdif_read(reg)	__raw_readl(REGS_LCDIF_BASE + reg)

struct lcd_reg_info {
	unsigned short index;
	unsigned short data;
};

static struct mxs_platform_bl_data bl_data;
static struct clk *lcd_clk;

static const struct lcd_reg_info lcd_code_init[] = {
	// Driving ability setting
	{ 0x2E, 0x89 },	//write_data(0x00, },	//GDOFF
	{ 0x29, 0X8F },	//write_data(0x00 },	//RTN
	{ 0x2B, 0X02 },	//write_data(0x00, },	//DUM
	{ 0xE2, 0X00 },	//write_data(0x00, },	//VREF
	{ 0xE4, 0X01 },	//write_data(0x00, },	//EQ
	{ 0xE5, 0X10 },	//write_data(0x00, },	//EQ
	{ 0xE6, 0X01 },	//write_data(0x00, },	//EQ
	{ 0xE7, 0X10 },	//write_data(0x00, },	//EQ
	{ 0xE8, 0X70 },	//write_data(0x00, },	//OPON
	{ 0xF2, 0X00 },	//write_data(0x00, },	//GEN
	{ 0xEA, 0X00 },	//write_data(0x00, },	//PTBA
	{ 0xEB, 0X20 },	//write_data(0x00, },	//PTBA
	{ 0xEC, 0X3C },	//write_data(0x00, },	//STBA
	{ 0xED, 0XC8 },	//write_data(0x00, },	//STBA
	{ 0xE9, 0X38 },	//write_data(0x00, },	//OPON1
	{ 0xF1, 0X01 },	//write_data(0x00, },	//OTPS1B

	// Gamma 2.8 setting 
	{ 0x40, 0X00 },	//write_data(0x00, },	//
	{ 0x41, 0X00 },	//write_data(0x00, },	//
	{ 0x42, 0X00 },	//write_data(0x00, },	//
	{ 0x43, 0X15 },	//write_data(0x00, },	//
	{ 0x44, 0X13 },	//write_data(0x00, },	//
	{ 0x45, 0X3f },	//write_data(0x00, },	//
	{ 0x47, 0X55 },	//write_data(0x00, },	//
	{ 0x48, 0X00 },	//write_data(0x00, },	//
	{ 0x49, 0X12 },	//write_data(0x00, },	//
	{ 0x4A, 0X19 },	//write_data(0x00, },	//
	{ 0x4B, 0X19 },	//write_data(0x00, },	//
	{ 0x4C, 0X16 },	//write_data(0x00, },	//
	{ 0x50, 0X00 },	//write_data(0x00, },	//
	{ 0x51, 0X2c },	//write_data(0x00, },	//
	{ 0x52, 0X2a },	//write_data(0x00, },	//
	{ 0x53, 0X3F },	//write_data(0x00, },	//
	{ 0x54, 0X3F },	//write_data(0x00, },	//
	{ 0x55, 0X3F },	//write_data(0x00, },	//
	{ 0x56, 0X2a },	//write_data(0x00, },	//
	{ 0x57, 0X7e },	//write_data(0x00, },	//
	{ 0x58, 0X09 },	//write_data(0x00, },	//
	{ 0x59, 0X06 },	//write_data(0x00, },	//
	{ 0x5A, 0X06 },	//write_data(0x00, },	//
	{ 0x5B, 0X0d },	//write_data(0x00, },	//
	{ 0x5C, 0X1F },	//write_data(0x00, },	//
	{ 0x5D, 0XFF },	//write_data(0x00, },	//

	// Power Voltage Setting
	{ 0x1B, 0X1A },	//write_data(0x00);
	{ 0x1A, 0X02 },	//write_data(0x00);
	{ 0x24, 0X61 },	//write_data(0x00);
	{ 0x25, 0X5C },	//write_data(0x00);

	// Vcom offset
	//{ 0x23,0x8D },	  // FLICKER ADJUST
	{ 0x23, 0x62 },	//write_data(0x00,0X62);

	// Power ON Setting
	{ 0x18, 0X36 },	//write_data(0x00 },	//RADJ 70Hz
	{ 0x19, 0X01 },	//write_data(0x00,0X01 },	//OSC_EN=1
	{ 0x1F, 0X88 },	//write_data(0x00,0X88 },	// GAS=1, VOMG=00, PON=0, DK=1, XDK=0, DVDH_TRI=0, STB=0
	{ LCD_DELAY, 5 },
	{ 0x1F, 0X80 },	//write_data(0x00,0X80 },	// GAS=1, VOMG=00, PON=0, DK=0, XDK=0, DVDH_TRI=0, STB=0
	{ LCD_DELAY, 5 },
	{ 0x1F, 0X90 },	//write_data(0x00,0X90 },	// GAS=1, VOMG=00, PON=1, DK=0, XDK=0, DVDH_TRI=0, STB=0
	{ LCD_DELAY, 5 },
	{ 0x1F, 0XD4 },	//write_data(0x00,0XD4 },	// GAS=1, VOMG=10, PON=1, DK=0, XDK=1, DDVDH_TRI=0, STB=0
	{ LCD_DELAY, 5 },
	//262k/65k color selection
	{ 0x17, 0X05 },	//write_data(0x00,0X05 },	//default 0x06 262k color // 0x05 65k color

	//SET PANEL
	{ 0x36, 0X09 },	//write_data(0x00,0X09 },	//SS_P, GS_P,REV_P,BGR_P

	//Display ON Setting
	{ 0x28, 0X38 },	//write_data(0x00,0X38 },	//GON=1, DTE=1, D=1000
	{ LCD_DELAY, 40 },
	{ 0x28, 0X3C },	//write_data(0x00,0X3C },	//GON=1, DTE=1, D=1100

	// 0x13F = 320, 0xEF = 240

	//Set GRAM Area
	{ 0x02, 0X00 },	//write_data(0x00,0X00);
	{ 0x03, 0X00 },	//write_data(0x00,0X00 },	//Column Start
	{ 0x04, 0X00 },	//write_data(0x00,0X00);
	{ 0x05, 0XEF },	//write_data(0x00,0XEF },	//Column End
	{ 0x06, 0X00 },	//write_data(0x00,0X00);
	{ 0x07, 0X00 },	//write_data(0x00,0X00 },	//Row Start
	{ 0x08, 0X01 },	//write_data(0x00,0X01);
	{ 0x09, 0X3F },	//write_data(0x00,0X3F },	//Row End

	{ LCD_INDEX, 0x22 },
};

static const struct lcd_reg_info lcd_code_disp_on[] = {

};

static const struct lcd_reg_info lcd_code_disp_off[] = {

};

static inline void
_lcdif_set_packaging_format(int format)
{
	_lcdif_write(HW_LCDIF_CTRL1_CLR,
			BM_LCDIF_CTRL1_BYTE_PACKING_FORMAT);
	_lcdif_write(HW_LCDIF_CTRL1_SET,
			BF_LCDIF_CTRL1_BYTE_PACKING_FORMAT(format));
}

static inline void
_lcd_write(int is_cmd, int data)
{
	_lcdif_write(HW_LCDIF_CTRL_CLR,
			BM_LCDIF_CTRL_LCDIF_MASTER |
			BM_LCDIF_CTRL_RUN);

	_lcdif_write(HW_LCDIF_TRANSFER_COUNT,
			BF_LCDIF_TRANSFER_COUNT_V_COUNT(1) |
			BF_LCDIF_TRANSFER_COUNT_H_COUNT(1));

	_lcdif_write((is_cmd ? HW_LCDIF_CTRL_CLR : HW_LCDIF_CTRL_SET),
			BM_LCDIF_CTRL_DATA_SELECT);

	_lcdif_write(HW_LCDIF_CTRL_SET, BM_LCDIF_CTRL_RUN);

	while (_lcdif_read(HW_LCDIF_STAT) & BM_LCDIF_STAT_LFIFO_FULL)
		;

	_lcdif_write(HW_LCDIF_DATA, data);

	while (_lcdif_read(HW_LCDIF_CTRL) & BM_LCDIF_CTRL_RUN)
		;

	_lcdif_write(HW_LCDIF_CTRL1_CLR, BM_LCDIF_CTRL1_CUR_FRAME_DONE_IRQ);
}

static inline int lcd_write_index(unsigned short index)
{
	_lcd_write(true, index);
	return 0;
}

static inline int lcd_write_data(unsigned short data)
{
	_lcd_write(false, data);
	return 0;
}

int lcd_write_reg(unsigned short index, unsigned short reg)
{
	lcd_write_index(index);
	return lcd_write_data(reg);
}

static int lcd_reg_write_info(struct lcd_reg_info *codes, int size)
{
	int i;

	for (i = 0; i < size; i++) {
		if (codes[i].index == LCD_DELAY)
			msleep(codes[i].data);
		else if (codes[i].index == LCD_INDEX)
			lcd_write_index(codes[i].data);
		else if (codes[i].index == LCD_DATA)
			lcd_write_data(codes[i].data);
		else
			lcd_write_reg(codes[i].index, codes[i].data);
	}

	return size;
}

static int lcd_panel_init(void)
{
	return lcd_reg_write_info(lcd_code_init, N_ELEMENTS(lcd_code_init));
}

static int lcd_panel_display(bool on)
{
	if (on) {
		return lcd_reg_write_info(lcd_code_disp_on, N_ELEMENTS(lcd_code_disp_on));
	} else {
		return lcd_reg_write_info(lcd_code_disp_off, N_ELEMENTS(lcd_code_disp_off));
	}
}

static ssize_t show_lcd_reg(struct device *device, struct device_attribute *attr,
			char *buf)
{
	return snprintf(buf, PAGE_SIZE, "not supported\n");
}

static ssize_t store_lcd_reg(struct device *device,
			    struct device_attribute *attr,
			    const char *buf, size_t count)
{
	unsigned short index;
	unsigned short data;
	char *comma;

	comma = strchr(buf, ',');
	if (!comma) goto _end_store;

	comma++;
	while (*comma == ' ') comma++;

	index = simple_strtoul(buf, NULL, 16);
	data = simple_strtoul(comma, NULL, 16);

	if (index > 0x0 && index <= 0xe6) {
		lcd_write_reg(index, data);
		printk("write reg 0x%02x = 0x%04x\n", index, data);
	} else if (index == 0xfffe && (data > 0x00 && data <= 0xe6)) {
		lcd_write_index(data);
		printk("write index 0x%02x\n", data);
	} else if (index == 0xfffc) {
		lcd_write_data(data);
		printk("write data 0x%04x\n", data);
	} else
		goto _end_store;
	
	return count;

_end_store:
	printk("\nusage : echo \"hexvalue1, hexvalue2\"\n"
		"\twrite reg, if hexvalue1 is index, and hexvalue2 is data\n"
		"\twrite index only, if hexvalue1 = 0xfffe\n"
		"\twrite data only, if hexvalue2 = 0xfffc\n\n");

	return -1;
}

static bool init_lcd_reg = false;
//static DEVICE_ATTR(lcd_reg, S_IWUSR | S_IRUGO, show_lcd_reg, store_lcd_spi_reg);

static void
_lcdif_setup_system_panel(void)
{
	_lcdif_write(HW_LCDIF_CTRL_CLR,
			BM_LCDIF_CTRL_CLKGATE |
			BM_LCDIF_CTRL_SFTRST |
			BM_LCDIF_CTRL_LCDIF_MASTER |
			BM_LCDIF_CTRL_BYPASS_COUNT);

	_lcdif_write(HW_LCDIF_VDCTRL0_CLR,
			BM_LCDIF_VDCTRL0_VSYNC_OEB);

	_lcdif_write(HW_LCDIF_CTRL_CLR,
			BM_LCDIF_CTRL_VSYNC_MODE |
			BM_LCDIF_CTRL_WAIT_FOR_VSYNC_EDGE |
			BM_LCDIF_CTRL_DVI_MODE |
			BM_LCDIF_CTRL_DOTCLK_MODE);

	_lcdif_write(HW_LCDIF_TRANSFER_COUNT,
			BF_LCDIF_TRANSFER_COUNT_V_COUNT(_V_ACTIVE) |
			BF_LCDIF_TRANSFER_COUNT_H_COUNT(_H_ACTIVE));

	_lcdif_write(HW_LCDIF_CTRL,
			BF_LCDIF_CTRL_INPUT_DATA_SWIZZLE(
				BV_LCDIF_CTRL_INPUT_DATA_SWIZZLE__NO_SWAP) |
			BF_LCDIF_CTRL_LCD_DATABUS_WIDTH(
				BV_LCDIF_CTRL_LCD_DATABUS_WIDTH__16_BIT) |
			BF_LCDIF_CTRL_WORD_LENGTH(
				BV_LCDIF_CTRL_WORD_LENGTH__16_BIT));

	_lcdif_set_packaging_format(_LCDIF_PACKAGING_FORMAT_REG);

	_lcdif_write(HW_LCDIF_TIMING,
			BF_LCDIF_TIMING_CMD_HOLD(0x1) |
			BF_LCDIF_TIMING_CMD_SETUP(0x1) |
			BF_LCDIF_TIMING_DATA_HOLD(0x3) |
			BF_LCDIF_TIMING_DATA_SETUP(0x3));
}

//#define _TEST_LOGO

#ifdef _TEST_LOGO
static void
_lcd_test_logo(void)
{
	int len = _H_ACTIVE * _V_ACTIVE;
	int i, j, k;

	for (i = 0; i < _V_ACTIVE; i++)
		for (j = 0; j < _H_ACTIVE; j++) {
			k = j / 80;

			if (k == 0)
				lcd_write_data(0xf800);
			else if (k == 1)
				lcd_write_data(0x07e0);
			else
				lcd_write_data(0x001f);
		}
}
#endif

static int init_panel(struct device *dev, dma_addr_t phys, int memsize,
		      struct mxs_platform_fb_entry *pentry)
{
	int ret = 0;
	lcd_clk = clk_get(NULL, "lcdif");
	if (IS_ERR(lcd_clk)) {
		ret = PTR_ERR(lcd_clk);
		goto out;
	}
	ret = clk_enable(lcd_clk);
	if (ret) {
		clk_put(lcd_clk);
		goto out;
	}

	ret = clk_set_rate(lcd_clk, 1000000000 / pentry->cycle_time_ns);	/* Hz */
	if (ret) {
		clk_disable(lcd_clk);
		clk_put(lcd_clk);
		goto out;
	}

	/*
	 * Make sure we do a high-to-low transition to reset the panel.
	 * First make it low for 100 msec, hi for 10 msec, low for 10 msec,
	 * then hi.
	 */
	__raw_writel(BM_LCDIF_CTRL1_RESET, REGS_LCDIF_BASE + HW_LCDIF_CTRL1_CLR);	/* low */
	mdelay(100);
	__raw_writel(BM_LCDIF_CTRL1_RESET, REGS_LCDIF_BASE + HW_LCDIF_CTRL1_SET);	/* high */
	mdelay(10);
	__raw_writel(BM_LCDIF_CTRL1_RESET, REGS_LCDIF_BASE + HW_LCDIF_CTRL1_CLR);	/* low */

	/* For the Samsung, Reset must be held low at least 30 uSec
	 * Therefore, we'll hold it low for about 10 mSec just to be sure.
	 * Then we'll wait 1 mSec afterwards.
	 */
	mdelay(10);
	__raw_writel(BM_LCDIF_CTRL1_RESET, REGS_LCDIF_BASE + HW_LCDIF_CTRL1_SET);	/* high */
	mdelay(1);

	_lcdif_setup_system_panel();

	lcd_panel_init();

#ifdef _TEST_LOGO
	_lcd_test_logo();
#else
	_lcdif_set_packaging_format(_LCDIF_PACKAGING_FORMAT_DATA);

	_lcdif_write(HW_LCDIF_TRANSFER_COUNT,
			BF_LCDIF_TRANSFER_COUNT_V_COUNT(_V_ACTIVE) |
			BF_LCDIF_TRANSFER_COUNT_H_COUNT(_H_ACTIVE));

	_lcdif_write(HW_LCDIF_CTRL_SET, BM_LCDIF_CTRL_DATA_SELECT);

	__raw_writel(BM_LCDIF_CTRL_BYPASS_COUNT,
			     REGS_LCDIF_BASE + HW_LCDIF_CTRL_CLR);

	__raw_writel(BM_LCDIF_CTRL1_CUR_FRAME_DONE_IRQ_EN,
			REGS_LCDIF_BASE + HW_LCDIF_CTRL1_SET);	/* cur frame done irq */

	ret = mxs_lcdif_dma_init(dev, phys, memsize);
#endif
	if (ret)
		goto out;

	mxs_lcd_set_bl_pdata(pentry->bl_data);
	mxs_lcdif_notify_clients(MXS_LCDIF_PANEL_INIT, pentry);

	if (!init_lcd_reg) {
		//device_create_file(dev, &dev_attr_lcd_reg);
		init_lcd_reg = true;
	}

	return 0;

out:
	return ret;
}

static void release_panel(struct device *dev,
			  struct mxs_platform_fb_entry *pentry)
{
	/* Reset LCD panel signel. */
	__raw_writel(BM_LCDIF_CTRL1_RESET,
		REGS_LCDIF_BASE + HW_LCDIF_CTRL1_CLR);
	mdelay(100);
	mxs_lcdif_notify_clients(MXS_LCDIF_PANEL_RELEASE, pentry);
	mxs_lcdif_dma_release();
	clk_disable(lcd_clk);
	clk_put(lcd_clk);
	__raw_writel(BM_LCDIF_CTRL_CLKGATE,
		     REGS_LCDIF_BASE + HW_LCDIF_CTRL_SET);
}

static int blank_panel(int blank)
{
	int ret = 0, count;

	switch (blank) {
	case FB_BLANK_NORMAL:
	case FB_BLANK_VSYNC_SUSPEND:
	case FB_BLANK_HSYNC_SUSPEND:
	case FB_BLANK_POWERDOWN:
		__raw_writel(BM_LCDIF_CTRL1_CUR_FRAME_DONE_IRQ_EN,
				REGS_LCDIF_BASE + HW_LCDIF_CTRL1_CLR);	/* cur frame done irq */

		lcd_panel_display(false);

		break;

	case FB_BLANK_UNBLANK:
		lcd_panel_display(true);

		__raw_writel(BM_LCDIF_CTRL1_CUR_FRAME_DONE_IRQ_EN,
				REGS_LCDIF_BASE + HW_LCDIF_CTRL1_SET);	/* cur frame done irq */

		break;

	default:
		ret = -EINVAL;
	}
	return ret;
}

static struct mxs_platform_fb_entry fb_entry = {
	.name = "tm024hbh40",
	.x_res = _V_ACTIVE,
	.y_res = _H_ACTIVE,
	.bpp = 16,
	.cycle_time_ns = 150,
	.lcd_type = MXS_LCD_PANEL_SYSTEM,
	.init_panel = init_panel,
	.release_panel = release_panel,
	.blank_panel = blank_panel,
	.run_panel = mxs_lcdif_run,
	.stop_panel = mxs_lcdif_stop,
	.pan_display = mxs_lcdif_pan_display,
	.bl_data = &bl_data,
};

static struct clk *pwm_clk;

static int init_bl(struct mxs_platform_bl_data *data)
{
	int ret = 0;

	pwm_clk = clk_get(NULL, "pwm");
	if (IS_ERR(pwm_clk)) {
		ret = PTR_ERR(pwm_clk);
		return ret;
	}
	clk_enable(pwm_clk);
	mxs_reset_block(REGS_PWM_BASE, 1);

	__raw_writel(BF_PWM_ACTIVEn_INACTIVE(0) |
		     BF_PWM_ACTIVEn_ACTIVE(0),
		     REGS_PWM_BASE + HW_PWM_ACTIVEn(2));
	__raw_writel(BF_PWM_PERIODn_CDIV(6) |	/* divide by 64 */
		     BF_PWM_PERIODn_INACTIVE_STATE(2) |	/* low */
		     BF_PWM_PERIODn_ACTIVE_STATE(3) |	/* high */
		     BF_PWM_PERIODn_PERIOD(599),
		     REGS_PWM_BASE + HW_PWM_PERIODn(2));
	__raw_writel(BM_PWM_CTRL_PWM2_ENABLE, REGS_PWM_BASE + HW_PWM_CTRL_SET);

	return 0;
}

static void free_bl(struct mxs_platform_bl_data *data)
{
	__raw_writel(BF_PWM_ACTIVEn_INACTIVE(0) |
		     BF_PWM_ACTIVEn_ACTIVE(0),
		     REGS_PWM_BASE + HW_PWM_ACTIVEn(2));
	__raw_writel(BF_PWM_PERIODn_CDIV(6) |	/* divide by 64 */
		     BF_PWM_PERIODn_INACTIVE_STATE(2) |	/* low */
		     BF_PWM_PERIODn_ACTIVE_STATE(3) |	/* high */
		     BF_PWM_PERIODn_PERIOD(599),
		     REGS_PWM_BASE + HW_PWM_PERIODn(2));
	__raw_writel(BM_PWM_CTRL_PWM2_ENABLE, REGS_PWM_BASE + HW_PWM_CTRL_CLR);

	clk_disable(pwm_clk);
	clk_put(pwm_clk);
}

static int values[] = { 0, 4, 9, 14, 20, 27, 35, 45, 57, 75, 100 };

static int power[] = {
	0, 1500, 3600, 6100, 10300,
	15500, 74200, 114200, 155200,
	190100, 191000
};

static int bl_to_power(int br)
{
	int base;
	int rem;

	if (br > 100)
		br = 100;
	base = power[br / 10];
	rem = br % 10;
	if (!rem)
		return base;
	else
		return base + (rem * (power[br / 10 + 1]) - base) / 10;
}

static int set_bl_intensity(struct mxs_platform_bl_data *data,
			    struct backlight_device *bd, int suspended)
{
	int intensity = bd->props.brightness;
	int scaled_int;

	if (bd->props.power != FB_BLANK_UNBLANK)
		intensity = 0;
	if (bd->props.fb_blank != FB_BLANK_UNBLANK)
		intensity = 0;
	if (suspended)
		intensity = 0;

	/*
	 * This is not too cool but what can we do?
	 * Luminance changes non-linearly...
	 */
	if (regulator_set_current_limit
	    (data->regulator, bl_to_power(intensity), bl_to_power(intensity)))
		return -EBUSY;

	scaled_int = values[intensity / 10];
	if (scaled_int < 100) {
		int rem = intensity - 10 * (intensity / 10);	/* r = i % 10; */
		scaled_int += rem * (values[intensity / 10 + 1] -
				     values[intensity / 10]) / 10;
	}

	__raw_writel(BF_PWM_ACTIVEn_INACTIVE(scaled_int) |
		     BF_PWM_ACTIVEn_ACTIVE(0),
		     REGS_PWM_BASE + HW_PWM_ACTIVEn(2));
	__raw_writel(BF_PWM_PERIODn_CDIV(6) |	/* divide by 64 */
		     BF_PWM_PERIODn_INACTIVE_STATE(2) |	/* low */
		     BF_PWM_PERIODn_ACTIVE_STATE(3) |	/* high */
		     BF_PWM_PERIODn_PERIOD(399),
		     REGS_PWM_BASE + HW_PWM_PERIODn(2));

	//gpio_direction_output(MXS_PIN_TO_GPIO(PINID_PWM4), (instensity) ? 1 : 0);

	return 0;
}

static struct mxs_platform_bl_data bl_data = {
	.bl_max_intensity = 100,
	.bl_default_intensity = 50,
	.bl_cons_intensity = 50,
	.init_bl = init_bl,
	.free_bl = free_bl,
	.set_bl_intensity = set_bl_intensity,
};

static int __init register_devices(void)
{
	struct platform_device *pdev;
	pdev = mxs_get_device("mxs-fb", 0);
	if (pdev == NULL || IS_ERR(pdev))
		return -ENODEV;

	mxs_lcd_register_entry(&fb_entry, pdev->dev.platform_data);

	return 0;
}

subsys_initcall(register_devices);
