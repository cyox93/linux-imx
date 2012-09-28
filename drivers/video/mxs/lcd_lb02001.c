/*
 * LB02001 2.0' TFT LCD Module
 *
 * OHSUNG ELECTRONICS CO., LTD. <kimkyoungho@ohsungec.com>
 *
 * Copyright 2012 OHSUNG ELECTRONICS CO., LTD., All Rights Reserved.
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

#include <mach/device.h>
#include <mach/lcdif.h>
#include <mach/regs-pwm.h>
#include <mach/system.h>
#include <mach/pinctrl.h>

#define REGS_PWM_BASE IO_ADDRESS(PWM_PHYS_ADDR)

#define DOTCLK_H_ACTIVE  240
#define DOTCLK_H_PULSE_WIDTH 1
#define DOTCLK_HF_PORCH  2
#define DOTCLK_HB_PORCH  4 
#define DOTCLK_H_WAIT_CNT  (DOTCLK_H_PULSE_WIDTH + (3 * DOTCLK_HB_PORCH))
#define DOTCLK_H_PERIOD (DOTCLK_H_WAIT_CNT + DOTCLK_HF_PORCH + DOTCLK_H_ACTIVE)

#define DOTCLK_V_ACTIVE  320
#define DOTCLK_V_PULSE_WIDTH  1
#define DOTCLK_VF_PORCH  2
#define DOTCLK_VB_PORCH  2 
#define DOTCLK_V_WAIT_CNT (DOTCLK_V_PULSE_WIDTH + DOTCLK_VB_PORCH)
#define DOTCLK_V_PERIOD (DOTCLK_VF_PORCH + DOTCLK_V_ACTIVE + DOTCLK_V_WAIT_CNT)

#define N_ELEMENTS(arr)		(sizeof(arr) / sizeof(arr[0]))

#define LCD_ID			0
#define LCD_DELAY		0xffff
#define LCD_INDEX		0xfffe
#define LCD_DATA		0xfffc

#define LCD_SPI_START		0x70
#define LCD_SPI_ID(id)		((id & 0x01) << 2)
#define LCD_SPI_WR		(0 << 0)
#define LCD_SPI_RD		(1 << 0)
#define LCD_SPI_RS		(1 << 1)

struct lcd_reg_info {
	unsigned short index;
	unsigned short data;
};

static struct mxs_platform_bl_data bl_data;
static struct clk *lcd_clk;
static struct mxs_lcd_spi_platform_data *lcd_spi;

static const struct lcd_reg_info lcd_code_init[] = {
	{ 0x01, 0x0000 },	// driver output control
	{ 0x02, 0x0200 },	// set inverion
	{ 0x03, 0x1030 },	// set entry mode
	{ 0x08, 0x0202 },	// set back & front porch
	{ 0x09, 0x0000 },	// set scan interval
	{ 0x0a, 0x0000 },	// set display control1
	{ 0x0c, 0x0111 },	// set RGB I/F display control
	{ 0x0d, 0x0000 },	// set frame mark position
	{ 0x60, 0xa700 },	// set gate scan control
	{ 0x61, 0x0000 },	// Normally White
	{ 0x6a, 0x0000 },	// set gate scan control
	{ LCD_DELAY, 10 },	//delay 10ms
	{ 0x10, 0x1490 },	// set BT, STB & SLP
	{ 0x11, 0x0227 },	// set VCi1 & step up circuits
	{ LCD_DELAY, 80 },	//delay 80ms
	{ 0x12, 0x000c },	// set VREGOUT1
	{ LCD_DELAY, 10 },	//delay 50ms
	{ 0x13, 0x1000 },	// set VCOMAC
	{ 0x29, 0x000b },	// set VCOMH
	{ 0x2b, 0x000b },	// set frame rate
	{ LCD_DELAY, 10 },	//delay 50ms
	{ 0x20, 0x0000 },	// set Gram horizontal address
	{ 0x21, 0x0000 },	// set Gram vertical address

	//============Gamma============
	{ 0x30, 0x0000 },
	{ 0x31, 0x0406 },
	{ 0x32, 0x0002 },
	{ 0x35, 0x0402 },
	{ 0x36, 0x0004 },
	{ 0x37, 0x0507 },
	{ 0x38, 0x0103 },
	{ 0x39, 0x0707 },
	{ 0x3c, 0x0204 },
	{ 0x3d, 0x0004 },

	//=============================
	{ 0x50, 0x0000 },	// set RAM Address
	{ 0x51, 0x00ef },
	{ 0x52, 0x0000 },
	{ 0x53, 0x013f }, 
	{ LCD_DELAY, 10 },	//delay 10ms
	{ 0x07, 0x0133 },	// display on
};

static const struct lcd_reg_info lcd_code_disp_on[] = {
	{ 0x0010, 0x0080 },		// SAP, BT[3:0], AP, DSTB, SLP
	{ LCD_DELAY,  50 },		// Dis-charge capacitor power voltage
	{ 0x0010, 0x1490 },		// SAP, BT[3:0], AP, DSTB, SLP, STB
	{ 0x0011, 0x0227 },		// DC1[2:0], DC0[2:0], VC[2:0]
	{ LCD_DELAY,  80 },		// Delay 50ms
	{ 0x0012, 0x000c },		// Inernal reference voltage =Vci;
	{ LCD_DELAY,  10 },		// Delay 50ms
	{ 0x0013, 0x1000 },		// VDV[4:0] for VCOM amplitude
	{ 0x0029, 0x000b },		// VCM[5:0] for VCOMH
	{ LCD_DELAY,  10 },		// Delay 50ms
	{ 0x0007, 0x0133 },		// 262K color and display ON
};

static const struct lcd_reg_info lcd_code_disp_off[] = {
	{ 0x0007, 0x0131 },		// Set D1=0, D0=1
	{ LCD_DELAY,  10 },
	{ 0x0007, 0x0130 },		// Set D1=0, D0=0
	{ LCD_DELAY,  10 },
	{ 0x0007, 0x0000 },		// display OFF
	//************* Power OFF sequence **************//
	{ 0x0010, 0x0080 },		// SAP, BT[3:0], APE, AP, DSTB, SLP
	{ 0x0011, 0x0000 },		// DC1[2:0], DC0[2:0], VC[2:0]
	{ 0x0012, 0x0000 },		// VREG1OUT voltage
	{ 0x0013, 0x0000 },		// VDV[4:0] for VCOM amplitude
	{ LCD_DELAY,  50 },		// Dis-charge capacitor power voltage
	{ 0x0010, 0x0082 },		// SAP, BT[3:0], APE, AP, DSTB, SLP
};

static int lcd_write_spi(char start, unsigned short data, int bits)
{
	int i, mask = 0x80;

	gpio_set_value(lcd_spi->cs, 0);
	udelay(2);

	// send start
	for (i = 0; mask != 0; i++) {
		gpio_set_value(lcd_spi->sck, 0);
		udelay(1);
		gpio_set_value(lcd_spi->mosi, (start & mask) ? 1 : 0);
		udelay(1);
		gpio_set_value(lcd_spi->sck, 1);
		udelay(1);

		mask >>= 1;
	}

	// send data
	if (bits > 0) mask = (0x1 << (bits - 1));
	for (i = 0; mask != 0; i++) {
		gpio_set_value(lcd_spi->sck, 0);
		udelay(1);
		gpio_set_value(lcd_spi->mosi, (data & mask) ? 1 : 0);
		udelay(1);
		gpio_set_value(lcd_spi->sck, 1);
		udelay(1);

		mask >>= 1;
	}

	gpio_set_value(lcd_spi->sck, 1);
	gpio_set_value(lcd_spi->mosi, 1);
	udelay(2);
	gpio_set_value(lcd_spi->cs, 1);

	udelay(5);

	return 0;
}

static inline int lcd_write_index(unsigned short index)
{
	unsigned char start = LCD_SPI_START | LCD_SPI_ID(LCD_ID) | LCD_SPI_WR;

	return lcd_write_spi(start, index, 16);
}

static inline int lcd_write_data(unsigned short data)
{
	unsigned char start = LCD_SPI_START | LCD_SPI_ID(LCD_ID) | LCD_SPI_WR | LCD_SPI_RS;

	return lcd_write_spi(start, data, 16);
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

static ssize_t show_lcd_spi_reg(struct device *device, struct device_attribute *attr,
			char *buf)
{
	return snprintf(buf, PAGE_SIZE, "not supported\n");
}

char *comma;

static ssize_t store_lcd_spi_reg(struct device *device,
			    struct device_attribute *attr,
			    const char *buf, size_t count)
{
	unsigned short index;
	unsigned short data;

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
static DEVICE_ATTR(lcd_reg, S_IWUSR | S_IRUGO, show_lcd_spi_reg, store_lcd_spi_reg);

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

	lcd_panel_init();

	setup_dotclk_panel(DOTCLK_V_PULSE_WIDTH, DOTCLK_V_PERIOD,
			   DOTCLK_V_WAIT_CNT, DOTCLK_V_ACTIVE,
			   DOTCLK_H_PULSE_WIDTH, DOTCLK_H_PERIOD,
			   DOTCLK_H_WAIT_CNT, DOTCLK_H_ACTIVE, 0);

	ret = mxs_lcdif_dma_init(dev, phys, memsize);
	if (ret)
		goto out;

	mxs_lcd_set_bl_pdata(pentry->bl_data);
	mxs_lcdif_notify_clients(MXS_LCDIF_PANEL_INIT, pentry);

	if (!init_lcd_reg) {
		device_create_file(dev, &dev_attr_lcd_reg);
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
	release_dotclk_panel();
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
		__raw_writel(BM_LCDIF_CTRL_BYPASS_COUNT,
			     REGS_LCDIF_BASE + HW_LCDIF_CTRL_CLR);
		for (count = 10000; count; count--) {
			if (__raw_readl(REGS_LCDIF_BASE + HW_LCDIF_STAT) &
			    BM_LCDIF_STAT_TXFIFO_EMPTY)
				break;
			udelay(1);
		}

		lcd_panel_display(false);

		break;

	case FB_BLANK_UNBLANK:
		lcd_panel_display(true);

		__raw_writel(BM_LCDIF_CTRL_BYPASS_COUNT,
			     REGS_LCDIF_BASE + HW_LCDIF_CTRL_SET);
		break;

	default:
		ret = -EINVAL;
	}
	return ret;
}

static struct mxs_platform_fb_entry fb_entry = {
	.name = "lb02001",
	.x_res = 320,
	.y_res = 240,
	.bpp = 16,
	.cycle_time_ns = 150,
	.lcd_type = MXS_LCD_PANEL_DOTCLK,
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

#ifdef CONFIG_MACH_AUSTIN_MX23
	__raw_writel(BF_PWM_ACTIVEn_INACTIVE(0) |
		     BF_PWM_ACTIVEn_ACTIVE(0),
		     REGS_PWM_BASE + HW_PWM_ACTIVEn(4));
	__raw_writel(BF_PWM_PERIODn_CDIV(6) |	/* divide by 64 */
		     BF_PWM_PERIODn_INACTIVE_STATE(2) |	/* low */
		     BF_PWM_PERIODn_ACTIVE_STATE(3) |	/* high */
		     BF_PWM_PERIODn_PERIOD(599),
		     REGS_PWM_BASE + HW_PWM_PERIODn(4));
	__raw_writel(BM_PWM_CTRL_PWM4_ENABLE, REGS_PWM_BASE + HW_PWM_CTRL_SET);
#else
	__raw_writel(BF_PWM_ACTIVEn_INACTIVE(0) |
		     BF_PWM_ACTIVEn_ACTIVE(0),
		     REGS_PWM_BASE + HW_PWM_ACTIVEn(2));
	__raw_writel(BF_PWM_PERIODn_CDIV(6) |	/* divide by 64 */
		     BF_PWM_PERIODn_INACTIVE_STATE(2) |	/* low */
		     BF_PWM_PERIODn_ACTIVE_STATE(3) |	/* high */
		     BF_PWM_PERIODn_PERIOD(599),
		     REGS_PWM_BASE + HW_PWM_PERIODn(2));
	__raw_writel(BM_PWM_CTRL_PWM2_ENABLE, REGS_PWM_BASE + HW_PWM_CTRL_SET);
#endif

	return 0;
}

static void free_bl(struct mxs_platform_bl_data *data)
{
#ifdef CONFIG_MACH_AUSTIN_MX23
	__raw_writel(BF_PWM_ACTIVEn_INACTIVE(0) |
		     BF_PWM_ACTIVEn_ACTIVE(0),
		     REGS_PWM_BASE + HW_PWM_ACTIVEn(4));
	__raw_writel(BF_PWM_PERIODn_CDIV(6) |	/* divide by 64 */
		     BF_PWM_PERIODn_INACTIVE_STATE(2) |	/* low */
		     BF_PWM_PERIODn_ACTIVE_STATE(3) |	/* high */
		     BF_PWM_PERIODn_PERIOD(599),
		     REGS_PWM_BASE + HW_PWM_PERIODn(4));
	__raw_writel(BM_PWM_CTRL_PWM4_ENABLE, REGS_PWM_BASE + HW_PWM_CTRL_CLR);
#else
	__raw_writel(BF_PWM_ACTIVEn_INACTIVE(0) |
		     BF_PWM_ACTIVEn_ACTIVE(0),
		     REGS_PWM_BASE + HW_PWM_ACTIVEn(2));
	__raw_writel(BF_PWM_PERIODn_CDIV(6) |	/* divide by 64 */
		     BF_PWM_PERIODn_INACTIVE_STATE(2) |	/* low */
		     BF_PWM_PERIODn_ACTIVE_STATE(3) |	/* high */
		     BF_PWM_PERIODn_PERIOD(599),
		     REGS_PWM_BASE + HW_PWM_PERIODn(2));
	__raw_writel(BM_PWM_CTRL_PWM2_ENABLE, REGS_PWM_BASE + HW_PWM_CTRL_CLR);
#endif

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

#ifdef CONFIG_MACH_AUSTIN_MX23
	__raw_writel(BF_PWM_ACTIVEn_INACTIVE(scaled_int) |
		     BF_PWM_ACTIVEn_ACTIVE(0),
		     REGS_PWM_BASE + HW_PWM_ACTIVEn(4));
	__raw_writel(BF_PWM_PERIODn_CDIV(6) |	/* divide by 64 */
		     BF_PWM_PERIODn_INACTIVE_STATE(2) |	/* low */
		     BF_PWM_PERIODn_ACTIVE_STATE(3) |	/* high */
		     BF_PWM_PERIODn_PERIOD(399),
		     REGS_PWM_BASE + HW_PWM_PERIODn(4));
#else
	__raw_writel(BF_PWM_ACTIVEn_INACTIVE(scaled_int) |
		     BF_PWM_ACTIVEn_ACTIVE(0),
		     REGS_PWM_BASE + HW_PWM_ACTIVEn(2));
	__raw_writel(BF_PWM_PERIODn_CDIV(6) |	/* divide by 64 */
		     BF_PWM_PERIODn_INACTIVE_STATE(2) |	/* low */
		     BF_PWM_PERIODn_ACTIVE_STATE(3) |	/* high */
		     BF_PWM_PERIODn_PERIOD(399),
		     REGS_PWM_BASE + HW_PWM_PERIODn(2));
#endif
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

	pdev = mxs_get_device("mxs-lcd-spi", 0);
	lcd_spi = (struct mxs_lcd_spi_platform_data *)pdev->dev.platform_data;

	return 0;
}

subsys_initcall(register_devices);
