/*
 * Copyright (C) 2012 OHSUNG ELECTRONICS CO., LTD. All Rights Reserved.
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

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/spi/spi.h>
#include <linux/gpio.h>

#include <asm/setup.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>

#include <mach/hardware.h>
#include <mach/device.h>
#include <mach/pinctrl.h>
#include <mach/regs-ocotp.h>

#include "device.h"
#include "mx23evk.h"
#include "mx23_pins.h"

static void __init fixup_board(struct machine_desc *desc, struct tag *tags,
			       char **cmdline, struct meminfo *mi)
{
	mx23_set_input_clk(24000000, 24000000, 32000, 50000000);
}

#if defined(CONFIG_SND_MXS_SOC_ADC) || defined(CONFIG_SND_MXS_SOC_ADC_MODULE)
static void __init mx23evk_init_adc(void)
{
	struct platform_device *pdev;
	pdev = mxs_get_device("mxs-adc", 0);
	if (pdev == NULL)
		return;
	mxs_add_device(pdev, 3);
}
#else
static void __init mx23evk_init_adc(void)
{

}
#endif

#if defined(CONFIG_FB_MXS_LCD_LB02001)
struct mxs_lcd_spi_platform_data lcd_spi = {
	.sck = MXS_PIN_TO_GPIO(PINID_LCD_RS),
	.mosi = MXS_PIN_TO_GPIO(PINID_LCD_WR),
	.miso = -1,
	.cs = MXS_PIN_TO_GPIO(PINID_LCD_CS),
};

static void __init austin_init_lcd_spi(void)
{
	struct platform_device *pdev;

	pdev = mxs_get_device("mxs-lcd-spi", 0);
	if (pdev == NULL)
		return;

	printk("%s : pdev[%p]\n", __func__, pdev);

	if (lcd_spi.sck >= 0) {
		gpio_request(lcd_spi.sck, "LCD SPI SCK");
		gpio_direction_output(lcd_spi.sck, 1);
	}

	if (lcd_spi.mosi >= 0) {
		gpio_request(lcd_spi.mosi, "LCD SPI MOSI");
		gpio_direction_output(lcd_spi.mosi, 1);
	}

	if (lcd_spi.miso >= 0) {
		gpio_request(lcd_spi.miso, "LCD SPI MISO");
		gpio_direction_output(lcd_spi.miso, 1);
	}

	if (lcd_spi.cs >= 0) {
		gpio_request(lcd_spi.cs, "LCD SPI CS");
		gpio_direction_output(lcd_spi.cs, 1);
	}

	pdev->dev.platform_data = &lcd_spi;
	mxs_add_device(pdev, 2);
}
#else
static void __init austin_init_lcd_spi(void)
{
}
#endif

#define REGS_OCOTP_BASE	IO_ADDRESS(OCOTP_PHYS_ADDR)
int get_evk_board_version()
{
	int boardid;
	boardid = __raw_readl(REGS_OCOTP_BASE + HW_OCOTP_CUSTCAP);
	boardid &= 0x30000000;
	boardid = boardid >> 28;

	return boardid;
}
EXPORT_SYMBOL_GPL(get_evk_board_version);

static void __init mx23evk_device_init(void)
{
	mx23evk_init_adc();
	austin_init_lcd_spi();
}


static void __init austin_init_machine(void)
{
	mx23_pinctrl_init();

	/* Init iram allocate */
#ifdef CONFIG_VECTORS_PHY_ADDR
	/* reserve the first page for irq vectors table*/
	iram_init(MX23_OCRAM_PHBASE + PAGE_SIZE, MX23_OCRAM_SIZE - PAGE_SIZE);
#else
	iram_init(MX23_OCRAM_PHBASE, MX23_OCRAM_SIZE);
#endif

	mx23_gpio_init();
	mx23evk_pins_init();
	mx23_device_init();
	mx23evk_device_init();
}

MACHINE_START(MX23EVK, "OHSUNG AUSTIN MX23 board")
	.phys_io	= 0x80000000,
	.io_pg_offst	= ((0xf0000000) >> 18) & 0xfffc,
	.boot_params	= 0x40000100,
	.fixup		= fixup_board,
	.map_io		= mx23_map_io,
	.init_irq	= mx23_irq_init,
	.init_machine	= austin_init_machine,
	.timer		= &mx23_timer.timer,
MACHINE_END
