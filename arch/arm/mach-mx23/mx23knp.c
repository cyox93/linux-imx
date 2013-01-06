/*
 * Copyright (C) 2012 Kim, KyoungHo All Rights Reserved.
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
#include <linux/input.h>
#include <linux/gpio_keys.h>
#include <linux/delay.h>
#include <linux/sysdev.h>

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

#if defined(CONFIG_KEYBOARD_GPIO) || defined(CONFIG_KEYBOARD_GPIO_MODULE)
static struct gpio_keys_button mx23knp_gpio_keys_table[] = {
	{
		.code		= KEY_ENTER,
		.gpio		= MXS_PIN_TO_GPIO(PINID_GPMI_D08),
		.active_low	= 1,
		.desc		= "Enter button",
		.wakeup		= 1,
	},
	{
		.code		= KEY_UP,
		.gpio		= MXS_PIN_TO_GPIO(PINID_GPMI_D09),
		.active_low	= 1,
		.desc		= "Up button",
		.wakeup		= 1,
	},
	{
		.code		= KEY_DOWN,
		.gpio		= MXS_PIN_TO_GPIO(PINID_GPMI_D10),
		.active_low	= 1,
		.desc		= "Down button",
		.wakeup		= 1,
	},
	{
		.code		= KEY_ESC,
		.gpio		= MXS_PIN_TO_GPIO(PINID_GPMI_D11),
		.active_low	= 1,
		.desc		= "ESC(Back) button",
		.wakeup		= 1,
	},

};

static struct gpio_keys_platform_data mx23knp_gpio_keys_data = {
	.buttons = mx23knp_gpio_keys_table,
	.nbuttons = ARRAY_SIZE(mx23knp_gpio_keys_table),
};

static struct platform_device mx23knp_device_gpiokeys = {
	.name = "gpio-keys",
	.dev.platform_data = &mx23knp_gpio_keys_data,
};

static void __init mx23knp_init_keypad(void)
{
	platform_device_register(&mx23knp_device_gpiokeys);
}

static ssize_t bt_reset_store(struct sys_device *dev,
					struct sysdev_attribute *attr,
					const char *buf, size_t size)
{
	gpio_set_value(MXS_PIN_TO_GPIO(PINID_GPMI_D12), 0);
	mdelay(100);

	gpio_set_value(MXS_PIN_TO_GPIO(PINID_GPMI_D12), 1);

	return size;
}

static SYSDEV_ATTR(reset, 0200, NULL, bt_reset_store);

static struct sysdev_class bt_sysclass = {
	.name = "bt",
};

static struct sys_device bt_device = {
	.id = 0,
	.cls = &bt_sysclass,
};

static int bt_sysdev_ctrl_init(void)
{
	int err;

	err = sysdev_class_register(&bt_sysclass);
	if (!err)
		err = sysdev_register(&bt_device);
	if (!err) {
		err = sysdev_create_file(&bt_device, &attr_reset);
	}

	return err;
}

static void __init mx23knp_init_bluetooth(void)
{
	gpio_request(MXS_PIN_TO_GPIO(PINID_GPMI_D12), "Bluetooth RESET");
	gpio_direction_output(MXS_PIN_TO_GPIO(PINID_GPMI_D12), 0);
	mdelay(100);

	gpio_set_value(MXS_PIN_TO_GPIO(PINID_GPMI_D12), 1);

	bt_sysdev_ctrl_init();
}
#else
static void __init mx23knp_init_keypad(void)
{
}

static void __init mx23knp_init_bluetooth(void)
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
	mx23knp_init_keypad();
	mx23knp_init_bluetooth();
}


static void __init mx23knp_init_machine(void)
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

MACHINE_START(MX23KNP, "Freescale MX23 KNP board")
	.phys_io	= 0x80000000,
	.io_pg_offst	= ((0xf0000000) >> 18) & 0xfffc,
	.boot_params	= 0x40000100,
	.fixup		= fixup_board,
	.map_io		= mx23_map_io,
	.init_irq	= mx23_irq_init,
	.init_machine	= mx23knp_init_machine,
	.timer		= &mx23_timer.timer,
MACHINE_END
