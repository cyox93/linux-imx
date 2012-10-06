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

#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/types.h>
#include <linux/input.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/mutex.h>
#include <linux/errno.h>
#include <linux/slab.h>
#include <asm/irq.h>
#include <asm/io.h>

#include <mach/gpio.h>
#include <mach/device.h>
#include <mach/hardware.h>

/* Group (0..3) -- when multiple keys are pressed, only the
 * keys pressed in the same group are considered as pressed. This is
 * in order to workaround certain crappy HW designs that produce ghost
 * keypresses. */
#define GROUP_0		(0 << 16)
#define GROUP_1		(1 << 16)
#define GROUP_2		(2 << 16)
#define GROUP_3		(3 << 16)
#define GROUP_MASK	GROUP_3

static void mxs_kbd_gpio_tasklet(unsigned long);
static void mxs_kbd_gpio_timer(unsigned long);

static unsigned char keypad_state[8];
static DEFINE_MUTEX(kp_enable_mutex);
static int kp_enable = 1;
static int kp_cur_group = -1;

struct mxs_kbd_gpio {
	struct input_dev *input;
	struct timer_list timer;
	int irq;
	unsigned int rows;
	unsigned int cols;
	unsigned long delay;
	unsigned int debounce;
};

static DECLARE_TASKLET_DISABLED(kp_tasklet, mxs_kbd_gpio_tasklet, 0);

static int *keymap;
static unsigned int *row_gpios;
static unsigned int *col_gpios;

static void set_col_gpio_val(struct mxs_kbd_gpio *mxs_kbd_gpio, u8 value)
{
	int col;

	for (col = 0; col < mxs_kbd_gpio->cols; col++)
		gpio_set_value(col_gpios[col], value & (1 << col));
}

static u8 get_row_gpio_val(struct mxs_kbd_gpio *mxs_kbd_gpio)
{
	int row;
	u8 value = 0;

	for (row = 0; row < mxs_kbd_gpio->rows; row++) {
		if (gpio_get_value(row_gpios[row]))
			value |= (1 << row);
	}
	return value;
}

static irqreturn_t mxs_kbd_gpio_interrupt(int irq, void *dev_id)
{
	struct mxs_kbd_gpio *mxs_kbd_gpio = dev_id;

	/* disable keyboard interrupt and schedule for handling */
	int i;

	for (i = 0; i < mxs_kbd_gpio->rows; i++) {
		int gpio_irq = gpio_to_irq(row_gpios[i]);
		/*
		 * The interrupt which we're currently handling should
		 * be disabled _nosync() to avoid deadlocks waiting
		 * for this handler to complete.  All others should
		 * be disabled the regular way for SMP safety.
		 */
		if (gpio_irq == irq)
			disable_irq_nosync(gpio_irq);
		else
			disable_irq(gpio_irq);
	}

	tasklet_schedule(&kp_tasklet);

	return IRQ_HANDLED;
}

static void mxs_kbd_gpio_timer(unsigned long data)
{
	tasklet_schedule(&kp_tasklet);
}

static void mxs_kbd_gpio_scan_keypad(struct mxs_kbd_gpio *mxs_kbd_gpio, unsigned char *state)
{
	int col = 0;

	/* read the keypad status */
	/* read the keypad status */
	for (col = 0; col < mxs_kbd_gpio->cols; col++) {
		set_col_gpio_val(mxs_kbd_gpio, ~(1 << col));
		state[col] = ~(get_row_gpio_val(mxs_kbd_gpio)) & 0xff;
	}
	set_col_gpio_val(mxs_kbd_gpio, 0);
}

static inline int mxs_kbd_gpio_find_key(int col, int row)
{
	int i, key;

	key = KEY(col, row, 0);
	for (i = 0; keymap[i] != 0; i++)
		if ((keymap[i] & 0xff000000) == key)
			return keymap[i] & 0x00ffffff;
	return -1;
}

static void mxs_kbd_gpio_tasklet(unsigned long data)
{
	struct mxs_kbd_gpio *mxs_kbd_gpio_data = (struct mxs_kbd_gpio *) data;
	unsigned char new_state[8], changed, key_down = 0;
	int col, row;
	int spurious = 0;

	/* check for any changes */
	mxs_kbd_gpio_scan_keypad(mxs_kbd_gpio_data, new_state);

	/* check for changes and print those */
	for (col = 0; col < mxs_kbd_gpio_data->cols; col++) {
		changed = new_state[col] ^ keypad_state[col];
		key_down |= new_state[col];
		if (changed == 0)
			continue;

		for (row = 0; row < mxs_kbd_gpio_data->rows; row++) {
			int key;
			if (!(changed & (1 << row)))
				continue;

			printk(KERN_DEBUG "mxs-kbd-gpio: key %d-%d %s\n", col,
			       row, (new_state[col] & (1 << row)) ?
			       "pressed" : "released");

			key = mxs_kbd_gpio_find_key(col, row);
			if (key < 0) {
				printk(KERN_WARNING
				      "mxs-kbd-gpio: Spurious key event %d-%d\n",
				       col, row);
				/* We scan again after a couple of seconds */
				spurious = 1;
				continue;
			}

			if (!(kp_cur_group == (key & GROUP_MASK) ||
			      kp_cur_group == -1))
				continue;

			kp_cur_group = key & GROUP_MASK;
			input_report_key(mxs_kbd_gpio_data->input, key & ~GROUP_MASK,
					 new_state[col] & (1 << row));
		}
	}
	memcpy(keypad_state, new_state, sizeof(keypad_state));

	if (key_down) {
                int delay = HZ / 20;
		/* some key is pressed - keep irq disabled and use timer
		 * to poll the keypad */
		if (spurious)
			delay = 2 * HZ;
		mod_timer(&mxs_kbd_gpio_data->timer, jiffies + delay);
	} else {
		/* enable interrupts */
		int i;
		for (i = 0; i < mxs_kbd_gpio_data->rows; i++)
			enable_irq(gpio_to_irq(row_gpios[i]));
	}
}

#ifdef CONFIG_PM
static int mxs_kbd_gpio_suspend(struct platform_device *dev, pm_message_t state)
{
	/* Nothing yet */

	return 0;
}

static int mxs_kbd_gpio_resume(struct platform_device *dev)
{
	/* Nothing yet */

	return 0;
}
#else
#define mxs_kbd_gpio_suspend	NULL
#define mxs_kbd_gpio_resume	NULL
#endif

static int __devinit mxs_kbd_gpio_probe(struct platform_device *pdev)
{
	struct mxs_kbd_gpio *mxs_kbd_gpio;
	struct input_dev *input_dev;
	struct mxs_kbd_gpio_plat_data *pdata =  pdev->dev.platform_data;
	int i, col_idx, row_idx, irq_idx, ret;

	if (!pdata->rows || !pdata->cols || !pdata->keymap) {
		printk(KERN_ERR "No rows, cols or keymap from pdata\n");
		return -EINVAL;
	}

	mxs_kbd_gpio = kzalloc(sizeof(struct mxs_kbd_gpio), GFP_KERNEL);
	input_dev = input_allocate_device();
	if (!mxs_kbd_gpio || !input_dev) {
		kfree(mxs_kbd_gpio);
		input_free_device(input_dev);
		return -ENOMEM;
	}

	platform_set_drvdata(pdev, mxs_kbd_gpio);

	mxs_kbd_gpio->input = input_dev;

	keymap = pdata->keymap;

	if (pdata->rep)
		__set_bit(EV_REP, input_dev->evbit);

	if (pdata->delay)
		mxs_kbd_gpio->delay = pdata->delay;

	if (pdata->row_gpios && pdata->col_gpios) {
		row_gpios = pdata->row_gpios;
		col_gpios = pdata->col_gpios;
	}

	mxs_kbd_gpio->rows = pdata->rows;
	mxs_kbd_gpio->cols = pdata->cols;

	/* Cols: outputs */
	for (col_idx = 0; col_idx < mxs_kbd_gpio->cols; col_idx++) {
		if (gpio_request(col_gpios[col_idx], "mxs_kbd_gpio_col") < 0) {
			printk(KERN_ERR "Failed to request"
			       "GPIO%d for keypad\n",
			       col_gpios[col_idx]);
			goto err1;
		}
		gpio_direction_output(col_gpios[col_idx], 0);
	}
	/* Rows: inputs */
	for (row_idx = 0; row_idx < mxs_kbd_gpio->rows; row_idx++) {
		if (gpio_request(row_gpios[row_idx], "mxs_kbd_gpio_row") < 0) {
			printk(KERN_ERR "Failed to request"
			       "GPIO%d for keypad\n",
			       row_gpios[row_idx]);
			goto err3;
		}
		gpio_direction_input(row_gpios[row_idx]);
	}

	setup_timer(&mxs_kbd_gpio->timer, mxs_kbd_gpio_timer, (unsigned long)mxs_kbd_gpio);

	/* get the irq and init timer*/
	tasklet_enable(&kp_tasklet);
	kp_tasklet.data = (unsigned long) mxs_kbd_gpio;

	/* setup input device */
	__set_bit(EV_KEY, input_dev->evbit);
	for (i = 0; keymap[i] != 0; i++)
		__set_bit(keymap[i] & KEY_MAX, input_dev->keybit);
	input_dev->name = "mxs-kbd-gpio";
	input_dev->phys = "mxs-kbd-gpio/input0";
	input_dev->dev.parent = &pdev->dev;

	input_dev->id.bustype = BUS_HOST;
	input_dev->id.vendor = 0x0001;
	input_dev->id.product = 0x0001;
	input_dev->id.version = 0x0100;

	ret = input_register_device(mxs_kbd_gpio->input);
	if (ret < 0) {
		printk(KERN_ERR "Unable to register mxs-kbd-gpio input device\n");
		goto err3;
	}

	/* scan current status and enable interrupt */
	mxs_kbd_gpio_scan_keypad(mxs_kbd_gpio, keypad_state);
	for (irq_idx = 0; irq_idx < mxs_kbd_gpio->rows; irq_idx++) {
		if (request_irq(gpio_to_irq(row_gpios[irq_idx]),
				mxs_kbd_gpio_interrupt,
				IRQF_TRIGGER_FALLING,
				"mxs-kbd-gpio", mxs_kbd_gpio) < 0)
			goto err5;
	}

	return 0;
err5:
	for (i = irq_idx - 1; i >=0; i--)
		free_irq(row_gpios[i], 0);

	input_unregister_device(mxs_kbd_gpio->input);
	input_dev = NULL;
err3:
	for (i = row_idx - 1; i >=0; i--)
		gpio_free(row_gpios[i]);
err1:
	for (i = col_idx - 1; i >=0; i--)
		gpio_free(col_gpios[i]);

	kfree(mxs_kbd_gpio);
	input_free_device(input_dev);

	return -EINVAL;
}

static int __devexit mxs_kbd_gpio_remove(struct platform_device *pdev)
{
	struct mxs_kbd_gpio *mxs_kbd_gpio = platform_get_drvdata(pdev);

	/* disable keypad interrupt handling */
	tasklet_disable(&kp_tasklet);
	int i;
	for (i = 0; i < mxs_kbd_gpio->cols; i++)
		gpio_free(col_gpios[i]);
	for (i = 0; i < mxs_kbd_gpio->rows; i++) {
		gpio_free(row_gpios[i]);
		free_irq(gpio_to_irq(row_gpios[i]), 0);
	}

	del_timer_sync(&mxs_kbd_gpio->timer);
	tasklet_kill(&kp_tasklet);

	/* unregister everything */
	input_unregister_device(mxs_kbd_gpio->input);

	kfree(mxs_kbd_gpio);

	return 0;
}

static struct platform_driver mxs_kbd_gpio_driver = {
	.probe		= mxs_kbd_gpio_probe,
	.remove		= __devexit_p(mxs_kbd_gpio_remove),
	.suspend	= mxs_kbd_gpio_suspend,
	.resume		= mxs_kbd_gpio_resume,
	.driver		= {
		.name	= "mxs-kbd-gpio",
		.owner	= THIS_MODULE,
	},
};

static int __init mxs_kbd_gpio_init(void)
{
	printk(KERN_INFO "MXS GPIO Matrix Keypad Driver\n");
	return platform_driver_register(&mxs_kbd_gpio_driver);
}

static void __exit mxs_kbd_gpio_exit(void)
{
	platform_driver_unregister(&mxs_kbd_gpio_driver);
}

module_init(mxs_kbd_gpio_init);
module_exit(mxs_kbd_gpio_exit);

MODULE_AUTHOR("Kim, KyoungHo");
MODULE_DESCRIPTION("MXS GPIO Keypad Driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:mxs-kbd-gpio");
