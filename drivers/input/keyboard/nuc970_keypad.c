/* linux/driver/input/nuc970_keypad.c
 *
 * Copyright (c) 2017 Nuvoton technology corporation
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * Changelog:
 *
 */

#include <linux/init.h>
#include <linux/slab.h>

#include <linux/input.h>
#include <linux/device.h>

#include <asm/errno.h>
#include <asm/delay.h>
#include <linux/mm.h>
#include <linux/poll.h>
#include <linux/module.h>
#include <asm/io.h>
#include <linux/cdev.h>
#include <linux/interrupt.h>
#include <linux/completion.h>

#include <linux/platform_device.h>

#include <mach/map.h>
#include <mach/mfp.h>

#include <mach/gpio.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio.h>

#include <mach/map.h>
#include <mach/regs-gpio.h>
#include <mach/regs-clock.h>
#include <mach/regs-gcr.h>
#include <mach/regs-aic.h>

#include <mach/irqs.h>

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/ioport.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_irq.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/platform_data/keypad-nuc970.h>


#undef BIT
#include <linux/input.h>
#define BIT(x)  (1UL<<((x)%BITS_PER_LONG))

#define DEF_KPD_DELAY           HZ/100

#define KEY_COUNT		32

#if defined CONFIG_OF
#define CONFIG_NUC970_KEYPAD_PH 1
#endif

static struct input_dev *nuc970_keypad_input_dev;
static struct timer_list kpd_timer;
static char timer_active = 0;

static u32 old_key;
static u32 new_key;
static u32 open_cnt = 0;

u32 nuc970_key_pressing = 0;
EXPORT_SYMBOL(nuc970_key_pressing);

static int nuc970_keymap[] = {
	// row 0
	KEY_RESERVED,	KEY_LEFTALT, 	KEY_F5,         KEY_INFO,
	KEY_UP,	        KEY_F7,         KEY_TAB,	KEY_F1,

	// row 1
	KEY_ESC,	KEY_DOWN,	KEY_RIGHT,	KEY_F6,
	KEY_F2,	        KEY_1,          KEY_2,	        KEY_4,

	// row 2
	KEY_LEFT,	KEY_CAPSLOCK,   KEY_F3,          KEY_B,
	KEY_5,          KEY_6,          KEY_SPACE,       KEY_F4,

	// row 3
	KEY_RESERVED,	KEY_7,          KEY_8,	         KEY_9,
	KEY_BACKSPACE,	KEY_DOT,        KEY_0,	         KEY_ENTER,
};

#if 0
// arg = 0, from isr, 1 from timer.
static void nuc970_check_ghost_state(void)
{
	int i,j;
	u32 check = 0;
	u32 col, check_col, cmp_col;			

	for (i = 0; i < ROW_CNT; i++) 
	{
		col = (new_key >> (i*COL_CNT)) & COL_MASK;

		if ((col & check) && hweight8(col) > 1)
		{
			for(j=0; j<ROW_CNT; j++)
			{
				check_col = (new_key >> (j*COL_CNT)) & COL_MASK;
				if((col & check_col) != 0)
				{
					cmp_col = (old_key >> (j*COL_CNT)) & COL_MASK;
					new_key = new_key & ~((cmp_col ^ check_col) << (j*COL_CNT));
				}
			}
		}

		check |= col;
	}
		
}
#endif

static void read_key(unsigned long arg)
{
	u32 i;

	#if 1
	// ISR detect key press, disable irq, use timer to read following key press until released
	if (!timer_active) {
		nuc970_key_pressing = 1;

		#if defined CONFIG_NUC970_KEYPAD_PH
		for(i = 0; i < NUC970_KPD_COL_NUMBER; i++)
		{
			disable_irq_nosync(gpio_to_irq(NUC970_PH8+i));
		}
		#endif
        }
	#else
	nuc970_key_pressing = 1;
	#endif

	#if defined CONFIG_NUC970_KEYPAD_PH
	new_key = readl(REG_GPIOH_DATAIN) & (((1 << NUC970_KPD_COL_NUMBER) - 1) << 8);

	#endif

	if ((new_key & (((1 << NUC970_KPD_COL_NUMBER) - 1) << 8)) == (((1 << NUC970_KPD_COL_NUMBER) - 1) << 8)) { // all key released

		for (i = 0; i < KEY_COUNT; i++) {
			if (old_key & (1 << i)) {
				input_event(nuc970_keypad_input_dev, EV_MSC, MSC_SCAN, i);
				//printk("=== key up1 code[%d] 0x%x 0x%x \n", i, nuc970_keymap[i], new_key);

				input_report_key(nuc970_keypad_input_dev, nuc970_keymap[i], 0);     //key up
				input_sync(nuc970_keypad_input_dev);
			}
		}
		old_key = 0;
		del_timer(&kpd_timer);
		timer_active = 0;

		#if defined CONFIG_NUC970_KEYPAD_PH
		for(i = 0; i < NUC970_KPD_COL_NUMBER; i++)
		{
			enable_irq(gpio_to_irq(NUC970_PH8+i));
		}

		#endif

		nuc970_key_pressing = 0;
		return;
	}

	#if defined CONFIG_NUC970_KEYPAD_PH
	// scan key
	new_key = 0;
	for(i = 0; i < NUC970_KPD_ROW_NUMBER; i++)
	{
		writel((readl(REG_GPIOH_DIR) & ~( ((1 << NUC970_KPD_ROW_NUMBER) - 1) << 4 ) ), REG_GPIOH_DIR);
		writel((readl(REG_GPIOH_DIR) | (1 << (4+i))), REG_GPIOH_DIR);
		udelay(100);
		new_key |= ( (~(readl(REG_GPIOH_DATAIN) >> 8) & ((1 << NUC970_KPD_COL_NUMBER)- 1)) << (8*i) );
	}

	writel( ( readl(REG_GPIOH_DIR) | ( ((1 << NUC970_KPD_ROW_NUMBER) - 1) << 4 ) ), REG_GPIOH_DIR);

	#endif

	for (i = 0; i < KEY_COUNT; i++) {

		if ((new_key ^ old_key) & (1 << i)) {// key state change
			if (new_key & (1 << i)) {
				//key down
				//printk("=== key down code[%d], 0x%x, 0x%x \n", i, new_key, nuc970_keymap[i]);

				input_event(nuc970_keypad_input_dev, EV_MSC, MSC_SCAN, i);
				input_report_key(nuc970_keypad_input_dev, nuc970_keymap[i], 1);
				input_sync(nuc970_keypad_input_dev);

			} else {
				//key up
				//printk("=== key up code[%d] 0x%x, 0x%x \n", i, nuc970_keymap[i], new_key);

				input_event(nuc970_keypad_input_dev, EV_MSC, MSC_SCAN, i);
				input_report_key(nuc970_keypad_input_dev, nuc970_keymap[i], 0);
				input_sync(nuc970_keypad_input_dev);
			}

		}

	}

	old_key = new_key;


	timer_active = 1;
	if ( arg == 0 )
		mod_timer(&kpd_timer, jiffies + DEF_KPD_DELAY*1); //### to avoid key too sensitive
	else
		mod_timer(&kpd_timer, jiffies + DEF_KPD_DELAY);

	return;

}


static irqreturn_t nuc970_kpd_irq(int irq, void *dev_id) 
{
	read_key(0);

	return IRQ_HANDLED;
}

static irqreturn_t kpi_interrupr_handle__(int irq, void *dev_id)
{
	// clear ISR
	writel(readl(REG_GPIOH_ISR) | ( ((1 << 12) - 1) << 4), REG_GPIOH_ISR);
    
	return IRQ_HANDLED;
}


int nuc970_kpd_open(struct input_dev *dev)
{
	u32 i;
	int error = 0;

	if (open_cnt > 0) {
		goto exit;
	}

	new_key = old_key = 0;

	// init timer
	init_timer(&kpd_timer);
	kpd_timer.function = read_key;	/* timer handler */
	kpd_timer.data = 1;

	#if defined CONFIG_NUC970_KEYPAD_PH
	writel(readl(REG_GPIOH_ISR), REG_GPIOH_ISR); // clear source

	for(i = 0; i < NUC970_KPD_COL_NUMBER; i++)
	{
		error =  request_irq((IRQ_GPIO_START+NUC970_PH8+i), nuc970_kpd_irq, IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING | IRQF_NO_SUSPEND, "Keypad",NULL);
		if (error) {
			printk("register the PH%d keypad_irq failed!  0x%x  0x%x \n", (8+i), (IRQ_GPIO_START+NUC970_PH8+i), NUC970_PH8);
			return -EAGAIN;
		}
	}

	for(i = 0; i < NUC970_KPD_ROW_NUMBER; i++)
	{
		if(request_irq((IRQ_GPIO_START+NUC970_PH4+i),kpi_interrupr_handle__, IRQF_TRIGGER_FALLING | IRQF_NO_SUSPEND, "Keypad",NULL) != 0){
		printk("register the PH%d keypad_irq failed!  0x%x \n", (4+i), (IRQ_GPIO_START+NUC970_PH4+i) );
		return -1;
		}

		enable_irq_wake((IRQ_GPIO_START+NUC970_PH4+i));
		disable_irq_nosync((IRQ_GPIO_START+NUC970_PH4+i));
	}

	#endif

exit:
	open_cnt++;
	return 0;
}



void nuc970_kpd_close(struct input_dev *dev)
{
	u32 i;

	open_cnt--;
	if (open_cnt == 0)
	{
		//disable interrupt
		#if defined CONFIG_NUC970_KEYPAD_PH
		for(i = 0; i < CONFIG_COL_NUMBER; i++)
		{
			free_irq(gpio_to_irq(NUC970_PH8+i),NULL);
		}

		#endif
	}
	return;
}


static int nuc970_keypad_probe(struct platform_device *pdev)
{
	int i, err;

	#ifdef CONFIG_OF
	if (pdev->dev.of_node) {
		of_property_read_u32_array(pdev->dev.of_node, "row", &NUC970_KPD_ROW_NUMBER, 1);
		of_property_read_u32_array(pdev->dev.of_node, "col", &NUC970_KPD_COL_NUMBER, 1);
	}
	#else
	NUC970_KPD_COL_NUMBER = CONFIG_COL_NUMBER;
	NUC970_KPD_ROW_NUMBER = CONFIG_ROW_NUMBER;
	#endif

	if(NUC970_KPD_COL_NUMBER > 8)  NUC970_KPD_COL_NUMBER = 8;
	if(NUC970_KPD_COL_NUMBER > 4)  NUC970_KPD_ROW_NUMBER = 4;

	// init GPIO
	#if defined CONFIG_NUC970_KEYPAD_PH
	// Set Column
	writel(readl(REG_GPIOH_DIR) & ~( ((1 << NUC970_KPD_COL_NUMBER) - 1) << 8), REG_GPIOH_DIR); // input
	writel(readl(REG_GPIOH_PUEN) | ( ((1 << NUC970_KPD_COL_NUMBER) - 1) << 8), REG_GPIOH_PUEN); // pull-up

	// Set Row
	writel(readl(REG_GPIOH_DIR) | ( ((1 << NUC970_KPD_ROW_NUMBER) - 1) << 4), REG_GPIOH_DIR);  // output
	writel(readl(REG_GPIOH_PUEN) | ( ((1 << NUC970_KPD_ROW_NUMBER) - 1) << 4), REG_GPIOH_PUEN); // pull up
	writel(readl(REG_GPIOH_DATAOUT) & ~( ((1 << NUC970_KPD_ROW_NUMBER) - 1) << 4), REG_GPIOH_DATAOUT); // low

	writel(readl(REG_GPIOH_DBEN) | ( ((1 << NUC970_KPD_COL_NUMBER) - 1) << 8), REG_GPIOH_DBEN);
	#endif

	writel(0x2f, REG_GPIO_DBNCECON); // De-bounce sampling cycle select 32768 clock

	if (!(nuc970_keypad_input_dev = input_allocate_device())) {
		printk("NUC970 Keypad Drvier Allocate Memory Failed!\n");
		err = -ENOMEM;
		goto fail;
	}

	nuc970_keypad_input_dev->name = "nuc970-kpi"; //"NUC970_Keypad";
	nuc970_keypad_input_dev->phys = "input/event1";
	nuc970_keypad_input_dev->id.bustype = BUS_HOST;
	nuc970_keypad_input_dev->id.vendor  = 0x0005;
	nuc970_keypad_input_dev->id.product = 0x0001;
	nuc970_keypad_input_dev->id.version = 0x0100;

	nuc970_keypad_input_dev->open    = nuc970_kpd_open;
	nuc970_keypad_input_dev->close   = nuc970_kpd_close;

	nuc970_keypad_input_dev->evbit[0] = BIT(EV_KEY) | BIT(EV_SYN) |  BIT(EV_REP);

	for (i = 0; i < KEY_MAX; i++)
		set_bit(i+1, nuc970_keypad_input_dev->keybit);

	err = input_register_device(nuc970_keypad_input_dev);
	if (err) {
		input_free_device(nuc970_keypad_input_dev);
		return err;
	}

	// must set after input device register!!!
	nuc970_keypad_input_dev->rep[REP_DELAY] = 1000; //1000ms
	nuc970_keypad_input_dev->rep[REP_PERIOD] = 100; //ms

	return 0;

fail:
	input_free_device(nuc970_keypad_input_dev);
	return err;
}

static int nuc970_keypad_remove(struct platform_device *pdev)
{
	u32 i;

	#if defined CONFIG_NUC970_KEYPAD_PH
	for(i = 0; i < NUC970_KPD_COL_NUMBER; i++)
	{
		free_irq(gpio_to_irq(NUC970_PH8+i),NULL);
	}

	#endif

	platform_set_drvdata(pdev, NULL);
	input_free_device(nuc970_keypad_input_dev);

	return 0;
}

#define nuc970_keypad_suspend 	NULL
#define nuc970_keypad_resume	NULL

static const struct of_device_id nuc970_kpi_of_match[] = {
	{ .compatible = "nuvoton,nuc970-kpi" },
	{},
};
MODULE_DEVICE_TABLE(of, nuc970_kpi_of_match);

static struct platform_driver nuc970_keypad_driver = {
	.probe		= nuc970_keypad_probe,
	.remove		= nuc970_keypad_remove,
	.suspend	= nuc970_keypad_suspend,
	.resume		= nuc970_keypad_resume,
	.driver		= {
		.name	= "nuc970-kpi",
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(nuc970_kpi_of_match),
	},
};
module_platform_driver(nuc970_keypad_driver);

MODULE_AUTHOR("nuvoton");
MODULE_DESCRIPTION("nuc970 keypad driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:nuc970-keypad");
