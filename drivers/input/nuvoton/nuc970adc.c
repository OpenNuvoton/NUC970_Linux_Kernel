/*
 * Copyright (c) 2017 Nuvoton technology corporation.
 *
 * Wan ZongShun <mcuos.com@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation;version 2 of the License.
 *
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/async_tx.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/spinlock.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/proc_fs.h>
#include <linux/of.h>
#include <linux/of_platform.h>

#include <linux/delay.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/io.h>

#include <linux/blkdev.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/trigger.h>
#include <linux/iio/buffer.h>
#include <linux/iio/trigger_consumer.h>
#include <linux/iio/triggered_buffer.h>

#include <linux/clk.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/slab.h>

#include <mach/map.h>
#include <mach/regs-clock.h>
#include <mach/regs-gcr.h>
#include <mach/regs-adc.h>

#ifdef CONFIG_BATTREY_NUC970ADC
#include <linux/sched.h>
#include <linux/module.h>
#include <linux/param.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/slab.h>
#endif

#include "nuc970adc.h"

#define INTERNAL_VOLTAGE 2500

#if 0
#define ENTRY()					printk("[%-20s] : Enter...\n", __FUNCTION__)
#define LEAVE()					printk("[%-20s] : Leave...\n", __FUNCTION__)
#else
#define ENTRY()
#define LEAVE()
#endif

#if 0
#define ADEBUG(fmt, arg...)		printk(fmt, ##arg)
#else
#define ADEBUG(fmt, arg...)
#endif


#define NUC970_ADC_TIMEOUT	(msecs_to_jiffies(1000))

#define ADC_CHANNEL(_index, _id) {			\
	.type = IIO_VOLTAGE,				\
	.indexed = 1,					\
	.channel = _index,				\
	.address = _index,				\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),   \
	.datasheet_name = _id,				\
    .scan_index = _index,               \
    .scan_type = {                      \
        .sign = 'u',                    \
        .realbits = 12,                 \
        .storagebits = 16,              \
        .shift = 0,                     \
        .endianness = IIO_BE,           \
    },                                  \
}

volatile uint8_t IsEnableKP = 0;
volatile uint8_t IsEnableKP_wakeup = 0;
volatile uint8_t IsEnableTS = 0;
volatile uint8_t IsEnableTS_wakeup = 0;
volatile uint8_t IsEnableIIO = 0;
volatile uint8_t IsEnableBT = 0;

enum touch_state {
	TS_IDLE,						/* We are waiting next key report */
	TS_CONVERSION,			/* We are waiting for ADC to report XY coord */
	TS_DOWN,
	TS_UP,
};

enum keypad_state {
	KP_IDLE,						/* We are waiting next key report */
	KP_CONVERSION,			/* We are waiting for ADC to report keypad */
	KP_DOWN,
	KP_UP,
};

#define KP_USED (1<<1)
#define TS_USED (1<<2)
#define BT_USED (1<<3)
#define IIO_USED (1<<4)

struct key_threshold {
	u32	thl;
	u32	thh;
};

static int nuc970_keycode[] = {
	KEY_A,
	KEY_B,
	KEY_C,
	KEY_D,
	KEY_E,
	KEY_F,
	KEY_G,
	KEY_H,
};

static struct key_threshold nuc970_key_th[] = {
	{0x500,0x5ff},
	{0x600,0x6ff},
	{0x700,0xaff},
	{0xa00,0xb4f},
	{0xb50,0xbff},
	{0xc00,0xcff},
	{0xd00,0xd49},
	{0xd50,0xe00},
};

#define CLK_PCLKEN1_ADCEN (1<<24)
u32 adc_sample_cnt = CONFIG_SAMPLE_NUC970ADC;
u32 z_th = 10;
u32 penup_delay_time = 3000;

struct nuc970_adc {
	struct input_dev *input_ts;
	struct input_dev *input_kp;
	struct timer_list timer;
	int irq_num;
	spinlock_t lock;
	enum touch_state ts_state;
	enum keypad_state kp_state;
	int kp_num;
	u32 isr;
	u32 ier;
	u32 conf;
	int used_state;
	struct clk		*clk;
	struct clk		*eclk;
	struct tasklet_struct irq_tasklet;
	u32 ts_num;
#ifdef CONFIG_BATTREY_NUC970ADC
	struct device *dev;
	struct power_supply bat;
	u32 bt_data;
	u8 bt_finish;
#endif

	struct iio_dev	*indio_dev;
	struct iio_trigger	*trig;
	struct completion   completion;

	int old_using;
	int old_x;
	int old_y;
};

static void enable_menu(void)
{
	__raw_writel(__raw_readl(REG_ADC_CTL) | ADC_CTL_MST, REG_ADC_CTL); /* ADC menu convert */
}
static void nuc970_detect2touch(void)
{
	ENTRY();
	/* Disable penwdown : ts_disable_pendown  */
	__raw_writel(__raw_readl(REG_ADC_CTL) & ~(ADC_CTL_PEDEEN), REG_ADC_CTL); /* Disable pen down */
	__raw_writel(__raw_readl(REG_ADC_IER) & ~(ADC_IER_PEDEIEN), REG_ADC_IER); /* Disable pen down interrupt flag */

	/* Config touch paramters : ts_enable_touch  */
	__raw_writel(__raw_readl(REG_ADC_CONF)| ADC_CONF_TEN | ADC_CONF_ZEN | (1<<22) | (7<<3) | (3<<6), REG_ADC_CONF);
	__raw_writel(__raw_readl(REG_ADC_ISR) | ADC_ISR_MF, REG_ADC_ISR);
	__raw_writel(__raw_readl(REG_ADC_IER) |(ADC_IER_MIEN), REG_ADC_IER);
	LEAVE();
}

static void nuc970_touch2detect(void)
{
	ENTRY();
	/* Clear interrupt before enable pendown */
	__raw_writel(__raw_readl(REG_ADC_CONF) & ~(ADC_CONF_TEN | ADC_CONF_ZEN), REG_ADC_CONF);
	__raw_writel( (__raw_readl(REG_ADC_IER) & ~(ADC_IER_PEDEIEN)), REG_ADC_IER); /*Disable Interrupt */
	__raw_writel(__raw_readl(REG_ADC_CTL) | (ADC_CTL_ADEN | ADC_CTL_PEDEEN), REG_ADC_CTL); /* Enable pen down event */
	udelay(10);
	__raw_writel(ADC_ISR_PEDEF|ADC_ISR_PEUEF|ADC_ISR_TF|ADC_ISR_ZF,REG_ADC_ISR);/* Clear pen down/up interrupt status */
	__raw_writel(__raw_readl(REG_ADC_IER) | (ADC_IER_PEDEIEN), REG_ADC_IER); /*Enable Interrupt */
	LEAVE();
}


__attribute__ ((unused)) static int nuc970_kp_conversion(struct nuc970_adc *nuc970_adc)
{
	u32 val,i;
	struct key_threshold *nuc970_th = (struct key_threshold *)nuc970_key_th;
	ENTRY();
	if((nuc970_adc->isr & ADC_ISR_KPCF) && (nuc970_adc->conf & ADC_CONF_KPCEN)) {
		if((nuc970_adc->kp_state == KP_DOWN) || (nuc970_adc->kp_state == KP_CONVERSION)) {
			nuc970_adc->kp_state= KP_CONVERSION;
			val=__raw_readl(REG_ADC_KPDATA);
			ADEBUG("KPDATA=0x%08x,ARRAY_SIZE(nuc970_keycode)=%d\n",val,ARRAY_SIZE(nuc970_keycode));
			for(i=0; i<ARRAY_SIZE(nuc970_keycode); i++) {
				if(val>nuc970_th[i].thl && val<nuc970_th[i].thh) {
					ADEBUG("i=%d\n",i);
					input_report_key(nuc970_adc->input_kp,nuc970_keycode[i],1);
					input_sync(nuc970_adc->input_kp);
					nuc970_adc->kp_num=i;
				}
			}
			if(nuc970_adc->used_state & TS_USED) {
				if(nuc970_adc->ts_state==TS_IDLE)
					nuc970_touch2detect();
			}
			LEAVE();
			return true;
		}
	}
	LEAVE();
	return false;
}
__attribute__ ((unused)) static int nuc970_kp_detect_up_down(struct nuc970_adc *nuc970_adc)
{
	ENTRY();
	if(nuc970_adc->isr & ADC_ISR_KPEF) {
		if(nuc970_adc->kp_state==KP_IDLE) {
			nuc970_adc->kp_state = KP_DOWN;
			if(nuc970_adc->used_state & TS_USED) {
				nuc970_detect2touch();
			}
			udelay(100);
			enable_menu();
			return true;
		}
	}
	if(nuc970_adc->isr & ADC_ISR_KPUEF) {
		if(nuc970_adc->kp_state == KP_CONVERSION) {
			nuc970_adc->kp_state = KP_UP;
			input_report_key(nuc970_adc->input_kp,nuc970_keycode[nuc970_adc->kp_num],0);
			input_sync(nuc970_adc->input_kp);
			nuc970_adc->kp_state=KP_IDLE;
			return true;
		}
	}
	LEAVE();
	return false;
}



__attribute__ ((unused)) static void ts_wait_conversion(unsigned long param)
{
	enable_menu();
}


__attribute__ ((unused)) static int nuc970_ts_conversion(struct nuc970_adc *nuc970_adc)
{
	u32 x,y,z,z2,pressure;
	static u32 z_cnt = 0;
	ENTRY();
	if(  (nuc970_adc->isr & ADC_ISR_TF) && (nuc970_adc->conf & ADC_CONF_TEN) && (nuc970_adc->isr & ADC_ISR_ZF) && (nuc970_adc->conf & ADC_CONF_ZEN) ) {
		if((nuc970_adc->ts_state == TS_DOWN) || (nuc970_adc->ts_state == TS_CONVERSION)) {
			nuc970_adc->ts_state= TS_CONVERSION;
			x = (__raw_readl(REG_ADC_XYDATA) & 0xfff);
			y = (__raw_readl(REG_ADC_XYDATA)>>16 & 0xfff);
			z = (__raw_readl(REG_ADC_ZDATA) & 0xfff);
			z2= ((__raw_readl(REG_ADC_ZDATA)>>16) & 0xfff);
			pressure=(x*(z2-(z+1)))/(z+1);
			//ADEBUG("G=>x=0x%03x,y=0x%03x,z1=0x%03x,z2=0x%03x,mear=%d\n",x,y,z,z2,pressure);
#if 0
			if(z<=z_th && (0xfff-z2)<=z_th) /* threshold value */
#else
			if((__raw_readl(REG_ADC_ZSORT0)&0xfff)<=z_th ||
			   (__raw_readl(REG_ADC_ZSORT1)&0xfff)<=z_th ||
			   (__raw_readl(REG_ADC_ZSORT2)&0xfff)<=z_th ||
			   (__raw_readl(REG_ADC_ZSORT3)&0xfff)<=z_th ) /* threshold value */
#endif
			{
				z_cnt++;
				if(z_cnt == 2) {
					z_cnt = 0;
					nuc970_adc->old_using=0;
					input_report_key(nuc970_adc->input_ts, BTN_TOUCH, 0);
					if(nuc970_adc->ts_num++>penup_delay_time) {
						nuc970_adc->ts_state = TS_IDLE;
						del_timer(&nuc970_adc->timer);
						nuc970_touch2detect();
					}
				}
				mod_timer(&nuc970_adc->timer, jiffies + msecs_to_jiffies(20));
			} else {
				int i,xdata,ydata;
				nuc970_adc->ts_num=0;
				z_cnt = 0;
				for(i=0; i<=12; i+=4) {
					xdata = (__raw_readl(REG_ADC_XYSORT0+i) & 0xfff);
					ydata = (__raw_readl(REG_ADC_XYSORT0+i)>>16 & 0xfff);
					if(xdata==0 || xdata==0xFFF || ydata==0 || ydata==0xfff ||abs(xdata-x)>50 || abs(ydata-y)>50) {
						nuc970_adc->old_using=0;
						ADEBUG("conversion data is failed\n");
						enable_menu();
						return true;
					}
				}
				if( (nuc970_adc->old_using==1) && (abs(nuc970_adc->old_x-x)>0x200 || abs(nuc970_adc->old_y-y)>0x200)) {
					enable_menu();
					return true;
				}
				nuc970_adc->old_using=1;
				nuc970_adc->old_x=x;
				nuc970_adc->old_y=y;
				input_report_key(nuc970_adc->input_ts, BTN_TOUCH, 1);
				input_report_abs(nuc970_adc->input_ts, ABS_X,x);
				input_report_abs(nuc970_adc->input_ts, ABS_Y,y);
				input_report_abs(nuc970_adc->input_ts, ABS_PRESSURE,pressure);
				ADEBUG("x=0x%03x,y=0x%03x,z1=0x%03x,z2=0x%03x,mear=%d\n",x,y,z,z2,pressure);
				ADEBUG("zs0=0x%08x\n",__raw_readl(REG_ADC_ZSORT0));
				ADEBUG("zs1=0x%08x\n",__raw_readl(REG_ADC_ZSORT1));
				ADEBUG("zs2=0x%08x\n",__raw_readl(REG_ADC_ZSORT2));
				ADEBUG("zs3=0x%08x\n",__raw_readl(REG_ADC_ZSORT3));
				ADEBUG("xys0=0x%08x\n",__raw_readl(REG_ADC_XYSORT0));
				ADEBUG("xys1=0x%08x\n",__raw_readl(REG_ADC_XYSORT1));
				ADEBUG("xys2=0x%08x\n",__raw_readl(REG_ADC_XYSORT2));
				ADEBUG("xys3=0x%08x\n",__raw_readl(REG_ADC_XYSORT3));
				mod_timer(&nuc970_adc->timer, jiffies + msecs_to_jiffies(20));
			}
			input_sync(nuc970_adc->input_ts);
			return true;
		}
	}
	LEAVE();
	return false;
}
__attribute__ ((unused)) static int nuc970_ts_detect_down(struct nuc970_adc *nuc970_adc)
{
	ENTRY();
	if((nuc970_adc->isr & ADC_ISR_PEDEF)&&(nuc970_adc->ier & ADC_IER_PEDEIEN )) {
		if(nuc970_adc->ts_state==TS_IDLE) {
			nuc970_adc->ts_state = TS_DOWN;
			nuc970_detect2touch();
			enable_menu();
			return true;
		}
	}
	LEAVE();
	return false;
}

static void nuc970adc_irq_tasklet(struct nuc970_adc *nuc970_adc)
{
	if(IsEnableTS==1)
		if(nuc970_ts_detect_down(nuc970_adc)) return;

	if(IsEnableKP==1)
		if(nuc970_kp_detect_up_down(nuc970_adc)) return;

	if(nuc970_adc->isr & ADC_ISR_MF) {
		if(IsEnableKP==1)
			nuc970_kp_conversion(nuc970_adc);

		if(IsEnableTS==1)
			nuc970_ts_conversion(nuc970_adc);
	}
}
static irqreturn_t nuc970_adc_interrupt(int irq, void *dev_id)
{
	struct nuc970_adc *nuc970_adc = dev_id;
	nuc970_adc->isr=__raw_readl(REG_ADC_ISR);
	nuc970_adc->ier=__raw_readl(REG_ADC_IER);
	nuc970_adc->conf=__raw_readl(REG_ADC_CONF);
	ENTRY();
	//ADEBUG("isr=0x%08x,ier=0x%08x\n",nuc970_adc->isr,nuc970_adc->ier);
	if(IsEnableTS==1)
		if((nuc970_adc->isr & ADC_ISR_PEDEF)&&(nuc970_adc->ier & ADC_IER_PEDEIEN )) {
			__raw_writel(ADC_ISR_PEDEF, REG_ADC_ISR);
			goto leave;
		}



	if(IsEnableTS_wakeup==1)
		if((__raw_readl(REG_ADC_WKISR) & ADC_WKISR_WPEDEF)&&(nuc970_adc->ier & ADC_IER_WKTIEN )) {
			__raw_writel(ADC_WKISR_WPEDEF, REG_ADC_WKISR);
			goto leave;
		}

	if(IsEnableKP_wakeup==1)
		if((__raw_readl(REG_ADC_WKISR) & ADC_WKISR_WKPEF)&&(nuc970_adc->ier & ADC_IER_WKPIEN )) {
			__raw_writel(ADC_WKISR_WKPEF, REG_ADC_WKISR);
			goto leave;
		}

	if(IsEnableKP==1) {
		if(nuc970_adc->isr & ADC_ISR_KPEF) {
			__raw_writel(ADC_ISR_KPEF,REG_ADC_ISR);
			goto leave;
		}
		if(nuc970_adc->isr & ADC_ISR_KPUEF) {
			__raw_writel(ADC_ISR_KPUEF,REG_ADC_ISR);
			goto leave;
		}
	}

	if(IsEnableIIO==1 && (nuc970_adc->isr & ADC_ISR_NACF)==ADC_ISR_NACF) {
		complete(&nuc970_adc->completion);
		__raw_writel(ADC_ISR_NACF,REG_ADC_ISR);
	}

	if(nuc970_adc->isr & ADC_ISR_MF) {
		__raw_writel(ADC_ISR_MF,REG_ADC_ISR);
		if(IsEnableKP==1)
			if((nuc970_adc->isr & ADC_ISR_KPCF) && (nuc970_adc->conf & ADC_CONF_KPCEN))
				if((nuc970_adc->kp_state == KP_DOWN) || (nuc970_adc->kp_state == KP_CONVERSION)) {
					__raw_writel(ADC_ISR_KPCF,REG_ADC_ISR);
					goto leave;
				}

		if(IsEnableTS==1)
			if(  (nuc970_adc->isr & ADC_ISR_TF) && (nuc970_adc->conf & ADC_CONF_TEN) && (nuc970_adc->isr & ADC_ISR_ZF) && (nuc970_adc->conf & ADC_CONF_ZEN) )
				if((nuc970_adc->ts_state == TS_DOWN) || (nuc970_adc->ts_state == TS_CONVERSION)) {
					__raw_writel(ADC_ISR_TF|ADC_ISR_ZF,REG_ADC_ISR);
					goto leave;
				}
	}
	LEAVE();
	return IRQ_HANDLED;
leave :
	tasklet_schedule(&nuc970_adc->irq_tasklet);
	return IRQ_HANDLED;
}

__attribute__ ((unused)) static int nuc970ts_open(struct input_dev *dev)
{
	struct nuc970_adc *nuc970_adc = input_get_drvdata(dev);
	ENTRY();

	/* Set touch parameters */
	writel(__raw_readl(REG_ADC_CONF)  | (ADC_CONF_HSPEED|ADC_CONF_TEN | ADC_CONF_ZEN | (1<<22)| (7<<3) | (3<<6) | (adc_sample_cnt<<24) ), REG_ADC_CONF); /* CONF */

	/* Clear interrupt before enable pendown */
	nuc970_touch2detect();

	nuc970_adc->used_state |= TS_USED;
	LEAVE();
	return 0;
}

__attribute__ ((unused)) static void nuc970ts_close(struct input_dev *dev)
{
	struct nuc970_adc *nuc970_adc = input_get_drvdata(dev);
	ENTRY();
	/* disable trigger mode */

	spin_lock_irq(&nuc970_adc->lock);

	nuc970_adc->ts_state = TS_IDLE;

	spin_unlock_irq(&nuc970_adc->lock);

	/* Now that interrupts are shut off we can safely delete timer */
	del_timer_sync(&nuc970_adc->timer);


	nuc970_touch2detect();
	/* Diable penwdown : ts_enable_touch  */
	__raw_writel(__raw_readl(REG_ADC_CONF)  & ~(ADC_CONF_TEN | ADC_CONF_ZEN |  ADC_CONF_DISTMAVEN), REG_ADC_CONF); /* CONF */
	__raw_writel(__raw_readl(REG_ADC_CTL) & ~(ADC_CTL_PEDEEN), REG_ADC_CTL); /* Disable pen down */
	__raw_writel(__raw_readl(REG_ADC_IER) & ~(ADC_IER_PEDEIEN), REG_ADC_IER); /* Disable pen down interrupt flag */
	nuc970_adc->used_state &= ~TS_USED;
	LEAVE();
}

__attribute__ ((unused)) static int nuc970kp_open(struct input_dev *dev)
{
	struct nuc970_adc *nuc970_adc = input_get_drvdata(dev);
	ENTRY();
	__raw_writel(__raw_readl(REG_ADC_CTL) | (ADC_CTL_ADEN | ADC_CTL_PKWPEN), REG_ADC_CTL);
	__raw_writel(__raw_readl(REG_ADC_CONF)| (ADC_CONF_KPCEN|ADC_CONF_HSPEED), REG_ADC_CONF);
	__raw_writel(__raw_readl(REG_ADC_IER) | (ADC_IER_KPEIEN | ADC_IER_KPUEIEN | ADC_IER_MIEN), REG_ADC_IER);
	__raw_writel(__raw_readl(REG_ADC_ISR) | (ADC_ISR_KPCF), REG_ADC_ISR);

	nuc970_adc->used_state |= KP_USED;
	LEAVE();
	return 0;
}
__attribute__ ((unused)) static void nuc970kp_close(struct input_dev *dev)
{
	struct nuc970_adc *nuc970_adc = input_get_drvdata(dev);
	ENTRY();
	__raw_writel(__raw_readl(REG_ADC_CTL) & ~(ADC_CTL_PKWPEN), REG_ADC_CTL);
	__raw_writel(__raw_readl(REG_ADC_CONF)& ~(ADC_CONF_KPCEN), REG_ADC_CONF);
	__raw_writel(__raw_readl(REG_ADC_IER) & ~(ADC_IER_KPEIEN | ADC_IER_KPUEIEN), REG_ADC_IER);
	nuc970_adc->used_state &= ~KP_USED;
	LEAVE();
}

#ifdef CONFIG_BATTREY_NUC970ADC
#define MAX_VOLTAGE 5000
static int nuc970adc_battery_read_adc(struct nuc970_adc *nuc970_adc)
{
	u32 count=3;
	unsigned long flags;
	ENTRY();
	spin_lock_irqsave(&nuc970_adc->lock, flags);
	__raw_writel(__raw_readl(REG_ADC_CTL) | (ADC_CTL_ADEN | ADC_CTL_VBGEN), REG_ADC_CTL);
	__raw_writel(__raw_readl(REG_ADC_CONF)| (ADC_CONF_VBATEN|ADC_CONF_HSPEED), REG_ADC_CONF);
	__raw_writel(__raw_readl(REG_ADC_IER) | (ADC_IER_MIEN), REG_ADC_IER);
	__raw_writel(__raw_readl(REG_ADC_ISR) | (ADC_ISR_VBF), REG_ADC_ISR);

	if(nuc970_adc->used_state & TS_USED) {
		if(nuc970_adc->ts_state==TS_IDLE) {
			nuc970_detect2touch();
			__raw_writel(__raw_readl(REG_ADC_CONF)  & ~(ADC_CONF_TEN | ADC_CONF_ZEN |  ADC_CONF_DISTMAVEN), REG_ADC_CONF); /* CONF */
			__raw_writel(__raw_readl(REG_ADC_CTL) & ~(ADC_CTL_PEDEEN), REG_ADC_CTL); /* Disable pen down */
			__raw_writel(__raw_readl(REG_ADC_IER) & ~(ADC_IER_PEDEIEN), REG_ADC_IER); /* Disable pen down interrupt flag */
		}
	}
	if(nuc970_adc->used_state & KP_USED) {
		__raw_writel(__raw_readl(REG_ADC_CTL) & ~(ADC_CTL_PKWPEN),REG_ADC_CTL);
		__raw_writel(__raw_readl(REG_ADC_CONF)& ~(ADC_CONF_KPCEN), REG_ADC_CONF);
		__raw_writel(__raw_readl(REG_ADC_IER) & ~(ADC_IER_KPEIEN | ADC_IER_KPUEIEN), REG_ADC_IER);
	}
	while(count--) {
		enable_menu();
		while((__raw_readl(REG_ADC_ISR) & (ADC_ISR_VBF|ADC_ISR_MF))!=(ADC_ISR_VBF|ADC_ISR_MF)) udelay(500);
		__raw_writel(__raw_readl(REG_ADC_ISR) | (ADC_ISR_VBF|ADC_ISR_MF),REG_ADC_ISR);
		nuc970_adc->bt_data=__raw_readl(REG_ADC_VBADATA);
	}
	if(nuc970_adc->used_state & TS_USED) {
		if(nuc970_adc->ts_state==TS_IDLE)
			nuc970_touch2detect();
		else {
			nuc970_detect2touch();
			enable_menu();
		}
	}
	if(nuc970_adc->used_state & KP_USED) {
		__raw_writel(__raw_readl(REG_ADC_CTL) | (ADC_CTL_PKWPEN),REG_ADC_CTL);
		__raw_writel(__raw_readl(REG_ADC_CONF)| (ADC_CONF_KPCEN), REG_ADC_CONF);
		__raw_writel(__raw_readl(REG_ADC_IER) | (ADC_IER_KPEIEN | ADC_IER_KPUEIEN), REG_ADC_IER);
	}

	ADEBUG("nuc970_adc->bt_data=%08x(%d)\n",nuc970_adc->bt_data,nuc970_adc->bt_data);
	__raw_writel(__raw_readl(REG_ADC_CTL) & ~(ADC_CTL_VBGEN), REG_ADC_CTL);
	__raw_writel(__raw_readl(REG_ADC_CONF)& ~(ADC_CONF_VBATEN), REG_ADC_CONF);
	LEAVE();
	spin_unlock_irqrestore(&nuc970_adc->lock, flags);
	return nuc970_adc->bt_data;
}

static int nuc970adc_battery_read_voltage(struct nuc970_adc *di)
{
	u32 n;
	int voltage = nuc970adc_battery_read_adc(di);
	ENTRY();
	n=((INTERNAL_VOLTAGE*voltage)>>12)*4;
	if(n>MAX_VOLTAGE) n=MAX_VOLTAGE;
	LEAVE();
	return n;

}
static int nuc970adc_battery_read_present(struct nuc970_adc *di)
{
	u32 n;
	int voltage = nuc970adc_battery_read_adc(di);
	ENTRY();
	n=((INTERNAL_VOLTAGE*voltage)>>12)*4;
	if(n>MAX_VOLTAGE) n=MAX_VOLTAGE;
	LEAVE();
	return (n*100)/MAX_VOLTAGE;


}
/*
 * Return power_supply property
 */
static int nuc970adc_battery_get_property(struct power_supply *psy,
        enum power_supply_property psp,
        union power_supply_propval *val)
{
	struct nuc970_adc *di = container_of((psy),struct nuc970_adc, bat);
	ENTRY();
	ADEBUG("psp=%d\n",psp);
	switch (psp) {
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN:
		val->intval = MAX_VOLTAGE;
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = nuc970adc_battery_read_present(di);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = nuc970adc_battery_read_voltage(di);
		break;
	default:
		return -EINVAL;
	}
	if (val->intval == INT_MAX || val->intval == INT_MIN)
		return -EINVAL;
	LEAVE();
	return 0;
}

static enum power_supply_property nuc970adc_battery_props[] = {
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
};
#endif

static const struct iio_chan_spec nuc970adc_iio_channels[] = {
	ADC_CHANNEL(0, "adc0"),
	ADC_CHANNEL(1, "adc1"),
	ADC_CHANNEL(2, "adc2"),
	ADC_CHANNEL(3, "adc3"),
	ADC_CHANNEL(4, "adc4"),
	ADC_CHANNEL(5, "adc5"),
	ADC_CHANNEL(6, "adc6"),
	ADC_CHANNEL(7, "adc7"),

};
#ifdef CONFIG_IIO_NUC970ADC
static void nuc970adc_channels_remove(struct iio_dev *indio_dev)
{
	kfree(indio_dev->channels);
}

static void nuc970adc_buffer_remove(struct iio_dev *idev)
{
	ENTRY();
	iio_triggered_buffer_cleanup(idev);
	LEAVE();
}

static irqreturn_t nuc970adc_iio_trigger_handler(int irq, void *p)
{
	struct iio_poll_func *pf = p;
	struct iio_dev *indio_dev = pf->indio_dev;
	struct nuc970_adc *info = iio_device_get_drvdata(indio_dev);
	int val;
	int channel;
	unsigned long timeout;
	ENTRY();
	channel = find_first_bit(indio_dev->active_scan_mask,indio_dev->masklength);
	printk("channel=%d\n",channel);
	// enable channel
	writel((readl(REG_ADC_CONF) & ~(0x7 << 3)) | (channel << 3) | ADC_CONF_NACEN, REG_ADC_CONF);

	// enable MST
	writel(readl(REG_ADC_CTL) | ADC_CTL_MST, REG_ADC_CTL);

	timeout = wait_for_completion_interruptible_timeout(&info->completion, NUC970_ADC_TIMEOUT);
	//wait_event_interruptible(adc_wq_xfer, (adc_state_xfer != 0));

	val = readl(REG_ADC_DATA);

	iio_push_to_buffers(indio_dev, (void *)&val);
	iio_trigger_notify_done(indio_dev->trig);
	LEAVE();
	return IRQ_HANDLED;
}
#endif
static int nuc970adc_read_raw(struct iio_dev *indio_dev,
                              struct iio_chan_spec const *chan,
                              int *val, int *val2, long mask)
{

//0:VBT(A0), 1:VHS(A1), 2:A2,     3:VSENSE
//4:YM(A4),  5:YP(A5),  6:XM(A6), 7:XP(A7)

	struct nuc970_adc *nuc970_adc = iio_device_get_drvdata(indio_dev);
	unsigned long timeout;
	ENTRY();
	if (mask != IIO_CHAN_INFO_RAW)
		return -EINVAL;

	if(IsEnableTS==1 && chan->channel>=4 && chan->channel<=7) {
		*val = 0;
		printk("this channel used by touch\n");
		return -EALREADY;
	}

	if(IsEnableKP==1 && chan->channel==2) {
		*val = 0;
		printk("this channel used by keybad\n");
		return -EALREADY;
	}
	mutex_lock(&indio_dev->mlock);

	init_completion(&nuc970_adc->completion);

	writel(readl(REG_ADC_CTL) | ADC_CTL_ADEN, REG_ADC_CTL); //Enable ADC

	if(nuc970_adc->used_state & TS_USED) {
		if(nuc970_adc->ts_state==TS_IDLE) {
			nuc970_detect2touch();
			__raw_writel(__raw_readl(REG_ADC_CONF)  & ~(ADC_CONF_TEN | ADC_CONF_ZEN |  ADC_CONF_DISTMAVEN), REG_ADC_CONF); /* CONF */
			__raw_writel(__raw_readl(REG_ADC_CTL) & ~(ADC_CTL_PEDEEN), REG_ADC_CTL); /* Disable pen down */
			__raw_writel(__raw_readl(REG_ADC_IER) & ~(ADC_IER_PEDEIEN), REG_ADC_IER); /* Disable pen down interrupt flag */
		}
	}
	if(nuc970_adc->used_state & KP_USED) {
		__raw_writel(__raw_readl(REG_ADC_CTL) & ~(ADC_CTL_PKWPEN),REG_ADC_CTL);
		__raw_writel(__raw_readl(REG_ADC_CONF)& ~(ADC_CONF_KPCEN), REG_ADC_CONF);
		__raw_writel(__raw_readl(REG_ADC_IER) & ~(ADC_IER_KPEIEN | ADC_IER_KPUEIEN), REG_ADC_IER);
	}

#ifdef CONFIG_NUC970ADC_BANDGAP
	__raw_writel(__raw_readl(REG_ADC_CTL) | ADC_CTL_VBGEN, REG_ADC_CTL);  //Enable bandgap
	__raw_writel((__raw_readl(REG_ADC_CONF) & ~(0x3 << 6)), REG_ADC_CONF); //select bandgap
#endif
#ifdef CONFIG_NUC970ADC_VREF
	__raw_writel((__raw_readl(REG_ADC_CONF) & ~(0x3 << 6)), REG_ADC_CONF); //select vref
#endif
#ifdef CONFIG_NUC970ADC_I33V
	__raw_writel((__raw_readl(REG_ADC_CONF) |(0x3<<6)), REG_ADC_CONF); //select AGND33 vs AVDD33
#endif

	// enable channel
	__raw_writel((__raw_readl(REG_ADC_CONF) & ~(0x7 << 3)) | (chan->channel << 3)|ADC_CONF_NACEN, REG_ADC_CONF);

	__raw_writel(__raw_readl(REG_ADC_IER)|ADC_IER_MIEN, REG_ADC_IER);

	__raw_writel(ADC_ISR_NACF, REG_ADC_ISR);

	// enable MST
	__raw_writel(__raw_readl(REG_ADC_CTL) | ADC_CTL_MST, REG_ADC_CTL);

	timeout = wait_for_completion_interruptible_timeout(&nuc970_adc->completion, NUC970_ADC_TIMEOUT);

	*val = __raw_readl(REG_ADC_DATA);

	__raw_writel(__raw_readl(REG_ADC_CONF) & ~(ADC_CONF_NACEN),REG_ADC_CONF);

	if(nuc970_adc->used_state & TS_USED) {
		if(nuc970_adc->ts_state==TS_IDLE)
			nuc970_touch2detect();
		else {
			nuc970_detect2touch();
			enable_menu();
		}
	}
	if(nuc970_adc->used_state & KP_USED) {
		__raw_writel(__raw_readl(REG_ADC_CTL) | (ADC_CTL_PKWPEN),REG_ADC_CTL);
		__raw_writel(__raw_readl(REG_ADC_CONF)| (ADC_CONF_KPCEN), REG_ADC_CONF);
		__raw_writel(__raw_readl(REG_ADC_IER) | (ADC_IER_KPEIEN | ADC_IER_KPUEIEN), REG_ADC_IER);
	}
	mutex_unlock(&indio_dev->mlock);
	if (timeout == 0)
		return -ETIMEDOUT;
	LEAVE();
	return IIO_VAL_INT;
}

static int nuc970adc_ring_preenable(struct iio_dev *indio_dev)
{
#ifdef CONFIG_IIO_NUC970ADC
	int ret;
	ENTRY();
	ret = iio_sw_buffer_preenable(indio_dev);
	if (ret < 0)
		return ret;
	LEAVE();
#endif
	return 0;
}

static const struct iio_buffer_setup_ops nuc970adc_ring_setup_ops = {
	.preenable = &nuc970adc_ring_preenable,
	.postenable = &iio_triggered_buffer_postenable,
	.predisable = &iio_triggered_buffer_predisable,
};

static const struct iio_info nuc970adc_iio_info = {
	.read_raw = &nuc970adc_read_raw,
};

static int nuc970adc_probe(struct platform_device *pdev)
{
	struct nuc970_adc *nuc970_adc;
	struct input_dev *input_ts_dev=NULL;
	struct input_dev *input_kp_dev=NULL;
	int err;

#ifndef CONFIG_OF
#ifdef CONFIG_KEYBOARD_NUC970ADC
	IsEnableKP = 1;
#endif

#ifdef CONFIG_TOUCHSCREEN_NUC970ADC
	IsEnableTS = 1;
#endif
#ifdef CONFIG_KEYBOARD_NUC970ADC_WKUP
	IsEnableKP_wakeup = 1;
#endif

#ifdef CONFIG_TOUCHSCREEN_NUC970ADC_WKUP
	IsEnableTS_wakeup = 1;
#endif

#ifdef CONFIG_IIO_NUC970ADC
	IsEnableIIO = 1;
#endif

#ifdef CONFIG_BATTREY_NUC970ADC
	IsEnableBT = 1;
#endif
#else
	{
		const char *pstr;
		of_property_read_string(pdev->dev.of_node,"keypad-status",&pstr);
		if(pstr[0]=='d')
			IsEnableKP = 0;
		else
			IsEnableKP = 1;

		of_property_read_string(pdev->dev.of_node,"touch-status",&pstr);
		if(pstr[0]=='d')
			IsEnableTS = 0;
		else
			IsEnableTS = 1;

		of_property_read_u32_array(pdev->dev.of_node,"samplecounter", &adc_sample_cnt,1);
		if(adc_sample_cnt>255) adc_sample_cnt=255;
		of_property_read_u32_array(pdev->dev.of_node,"z_th", &z_th,1);
		of_property_read_u32_array(pdev->dev.of_node,"penup_delay_time", &penup_delay_time,1);


		of_property_read_string(pdev->dev.of_node,"iio-status",&pstr);
		if(pstr[0]=='d')
			IsEnableIIO = 0;
		else
			IsEnableIIO = 1;

		of_property_read_string(pdev->dev.of_node,"battery-status",&pstr);
		if(pstr[0]=='d')
			IsEnableBT = 0;
		else
			IsEnableBT = 1;
	}
#endif

	err =0;
	printk("%s - pdev = %s\n", __func__, pdev->name);
	ENTRY();
	nuc970_adc = kzalloc(sizeof(struct nuc970_adc), GFP_KERNEL);
	if (!nuc970_adc) {
		err = -ENOMEM;
		goto fail1;
	}

	spin_lock_init(&nuc970_adc->lock);
	nuc970_adc->eclk=clk_get(NULL, "adc_eclk");
	clk_prepare(nuc970_adc->eclk);
	clk_enable(nuc970_adc->eclk);
	nuc970_adc->clk=clk_get(NULL, "adc");
	clk_prepare(nuc970_adc->clk);
	clk_enable(nuc970_adc->clk);

	clk_set_rate(nuc970_adc->eclk, 1000000);
	nuc970_adc->kp_state = KP_IDLE;
	nuc970_adc->ts_state = TS_IDLE;

	if(IsEnableTS==1) {
		input_ts_dev = input_allocate_device();
		if (!input_ts_dev) {
			err = -ENOMEM;
			goto fail1;
		}
		nuc970_adc->input_ts = input_ts_dev;
		nuc970_adc->old_using = 0;
		input_ts_dev->name = "NUC970/N9H30 TouchScreen(ADC)";
		input_ts_dev->phys = "nuc970ts/event0";
		input_ts_dev->id.bustype = BUS_HOST;
		input_ts_dev->id.vendor  = 0x0005;
		input_ts_dev->id.product = 0x0001;
		input_ts_dev->id.version = 0x0100;
		input_ts_dev->dev.parent = &pdev->dev;
		input_ts_dev->open = nuc970ts_open;
		input_ts_dev->close = nuc970ts_close;
		input_ts_dev->evbit[0] = BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS) | BIT_MASK(EV_SYN);
		input_ts_dev->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);
		input_set_abs_params(input_ts_dev, ABS_X, 0, 0x400, 0, 0);
		input_set_abs_params(input_ts_dev, ABS_Y, 0, 0x400, 0, 0);
		input_set_abs_params(input_ts_dev, ABS_PRESSURE, 0, 0x400, 0, 0);
		input_set_drvdata(input_ts_dev, nuc970_adc);
		setup_timer(&nuc970_adc->timer, ts_wait_conversion,(unsigned long)nuc970_adc);
		if(input_register_device(nuc970_adc->input_ts))
			goto fail5;
	}

	if(IsEnableKP==1) {
		input_kp_dev = input_allocate_device();
		if (!input_kp_dev) {
			err = -ENOMEM;
			goto fail1;
		}
		nuc970_adc->input_kp = input_kp_dev;
		nuc970_adc->kp_num = -1;
		input_kp_dev->name = "NUC970/N9H30 Keypad(ADC)";
		input_kp_dev->id.bustype = BUS_HOST;
		input_kp_dev->id.vendor  = 0x0005;
		input_kp_dev->id.product = 0x0002;
		input_kp_dev->id.version = 0x0100;
		input_kp_dev->open = nuc970kp_open;
		input_kp_dev->close = nuc970kp_close;
		input_kp_dev->dev.parent = &pdev->dev;
		input_kp_dev->evbit[0] = BIT_MASK(EV_KEY) | BIT_MASK(EV_REP) | BIT_MASK(EV_MSC);
		input_kp_dev->keycode = nuc970_keycode;
		input_kp_dev->keycodesize = sizeof(unsigned char);
		input_kp_dev->keycodemax = ARRAY_SIZE(nuc970_keycode);
		input_set_drvdata(input_kp_dev, nuc970_adc);
		if(input_register_device(nuc970_adc->input_kp))
			goto fail5;
		{
			int i;
			for (i = 0; i < ARRAY_SIZE(nuc970_keycode); i++)
				set_bit(nuc970_keycode[i], input_kp_dev->keybit);
		}
	}

	if(IsEnableIIO==1) {
#ifdef CONFIG_IIO_NUC970ADC
		int ret;
		nuc970_adc->used_state |= IIO_USED ;
		nuc970_adc->indio_dev = iio_device_alloc(0);
		if (nuc970_adc->indio_dev == NULL) {
			dev_err(&pdev->dev, "failed to allocate iio device\n");
			printk("failed to allocate iio device\n");
			return -ENOMEM;
		}
		iio_device_set_drvdata(nuc970_adc->indio_dev,nuc970_adc);
		nuc970_adc->indio_dev->dev.parent = &pdev->dev;
		nuc970_adc->indio_dev->name = dev_name(&pdev->dev);
		nuc970_adc->indio_dev->modes = INDIO_DIRECT_MODE;
		nuc970_adc->indio_dev->info = &nuc970adc_iio_info;
		nuc970_adc->indio_dev->num_channels = 8;
		nuc970_adc->indio_dev->channels = nuc970adc_iio_channels;
		nuc970_adc->indio_dev->masklength = nuc970_adc->indio_dev->num_channels - 1;
		ret = iio_triggered_buffer_setup(nuc970_adc->indio_dev, &iio_pollfunc_store_time,
		                                 &nuc970adc_iio_trigger_handler, &nuc970adc_ring_setup_ops);
		if (ret) {
			nuc970adc_channels_remove(nuc970_adc->indio_dev);
			iio_device_free(nuc970_adc->indio_dev);
			return ret;
		}

		ret = iio_device_register(nuc970_adc->indio_dev);
		if (ret < 0) {
			printk("Couldn't register NC970/N9H30 ADC..\n");
			nuc970adc_channels_remove(nuc970_adc->indio_dev);
			iio_device_free(nuc970_adc->indio_dev);
			return ret;
		}
#endif
	}

	__raw_writel(__raw_readl(REG_APBIPRST1) | (1<<24), REG_APBIPRST1);
	udelay(100);
	__raw_writel(__raw_readl(REG_APBIPRST1) & ~(1<<24), REG_APBIPRST1);
	udelay(100);

	nuc970_adc->irq_num = platform_get_irq(pdev, 0);
	if (request_irq(nuc970_adc->irq_num, nuc970_adc_interrupt,IRQF_NO_SUSPEND, "nuc970adc", nuc970_adc)) {
		err = -EBUSY;
		goto fail4;
	}

	if(IsEnableBT==1) {
#ifdef CONFIG_BATTREY_NUC970ADC
		nuc970_adc->used_state |= BT_USED;
		nuc970_adc->bat.name = "NUC970/N9H30 Battery(ADC)";
		nuc970_adc->bat.type = POWER_SUPPLY_TYPE_BATTERY;
		nuc970_adc->bat.properties = nuc970adc_battery_props;
		nuc970_adc->bat.num_properties = ARRAY_SIZE(nuc970adc_battery_props);
		nuc970_adc->bat.get_property = nuc970adc_battery_get_property;
		if ( power_supply_register(nuc970_adc->dev, &nuc970_adc->bat)) {
			printk("----------------------------------------------------power failed\n");
			goto fail5;
		}
#endif
	}

	platform_set_drvdata(pdev, nuc970_adc);

	tasklet_init(&nuc970_adc->irq_tasklet,
	             (void (*)(unsigned long))nuc970adc_irq_tasklet,
	             (unsigned long)nuc970_adc);

	LEAVE();
	return 0;

fail5:
	free_irq(nuc970_adc->irq_num, nuc970_adc);
fail4:
	clk_put(nuc970_adc->clk);
fail1:
	if(!input_ts_dev) input_free_device(input_ts_dev);
	if(!input_kp_dev) input_free_device(input_kp_dev);
	kfree(nuc970_adc);
	printk("failed\n");
	LEAVE();
	return err;
}

static int nuc970adc_remove(struct platform_device *pdev)
{
	struct nuc970_adc *nuc970_adc = platform_get_drvdata(pdev);
	ENTRY();
	free_irq(nuc970_adc->irq_num, nuc970_adc);

	clk_put(nuc970_adc->clk);
	clk_put(nuc970_adc->eclk);

	if(IsEnableTS==1)
		input_unregister_device(nuc970_adc->input_ts);

	if(IsEnableKP==1)
		input_unregister_device(nuc970_adc->input_kp);

	if(IsEnableIIO==1) {
#ifdef CONFIG_IIO_NUC970ADC
		iio_device_unregister(nuc970_adc->indio_dev);
		nuc970adc_channels_remove(nuc970_adc->indio_dev);
		iio_device_free(nuc970_adc->indio_dev);
		nuc970adc_buffer_remove(nuc970_adc->indio_dev);
#endif
	}

	kfree(nuc970_adc);

	platform_set_drvdata(pdev, NULL);
	LEAVE();
	return 0;
}

static int nuc970adc_resume(struct platform_device *pdev)
{

	struct nuc970_adc *nuc970_adc = platform_get_drvdata(pdev);
	ENTRY();

	if(IsEnableTS_wakeup == 1) {
		if(nuc970_adc->used_state & TS_USED) {
			__raw_writel(__raw_readl(REG_ADC_CTL) & ~ADC_CTL_WKTEN,REG_ADC_CTL); /* Disable touch wake up */
			__raw_writel( (__raw_readl(REG_ADC_IER) & ~(ADC_IER_WKTIEN)), REG_ADC_IER); /*Disable Interrupt */
			nuc970_touch2detect();
		}
	}

	if(IsEnableKP_wakeup == 1) {
		if(nuc970_adc->used_state & KP_USED) {
			__raw_writel(__raw_readl(REG_ADC_CTL) & ~ADC_CTL_WKPEN,REG_ADC_CTL); /* Disable touch wake up */
			__raw_writel( (__raw_readl(REG_ADC_IER) & ~(ADC_IER_WKPIEN)), REG_ADC_IER); /*Disable Interrupt */
		}
	}

	if(IsEnableTS_wakeup == 1 || IsEnableKP_wakeup == 1) {
		__raw_writel(__raw_readl(REG_WKUPSER) & ~(1<<26), REG_WKUPSER);
		disable_irq_wake(IRQ_ADC);
	}

	LEAVE();
	return 0;
}

static int nuc970adc_suspend(struct platform_device *pdev,pm_message_t state)
{

	struct nuc970_adc *nuc970_adc = platform_get_drvdata(pdev);

	if(IsEnableTS_wakeup == 1 || IsEnableKP_wakeup == 1) {
		__raw_writel((1 << 26)|__raw_readl(REG_WKUPSER), REG_WKUPSER);
		enable_irq_wake(IRQ_ADC);
	}

	ENTRY();
	if(IsEnableTS_wakeup == 1) {
		if(nuc970_adc->used_state & TS_USED) {
			if( nuc970_adc->ts_state!=TS_IDLE) {
				nuc970_adc->ts_state = TS_IDLE;
				del_timer(&nuc970_adc->timer);
			}
			/* Clear interrupt before enable pendown */
			__raw_writel(__raw_readl(REG_ADC_CONF) & ~(ADC_CONF_TEN | ADC_CONF_ZEN), REG_ADC_CONF);
			__raw_writel( (__raw_readl(REG_ADC_IER) & ~(ADC_IER_WKTIEN|ADC_IER_PEDEIEN)), REG_ADC_IER); /*Disable Interrupt */
			__raw_writel(__raw_readl(REG_ADC_CTL) | (ADC_CTL_ADEN| ADC_CTL_WKTEN | ADC_CTL_PEDEEN), REG_ADC_CTL); /* Enable pen down event */
			udelay(10);
			__raw_writel(ADC_ISR_PEDEF|ADC_ISR_PEUEF|ADC_ISR_TF|ADC_ISR_ZF,REG_ADC_ISR);/* Clear pen down/up interrupt status */
			__raw_writel(ADC_WKISR_WPEDEF,REG_ADC_WKISR);  /* Clear ts wakeup up flag */
			__raw_writel(__raw_readl(REG_ADC_IER) | (ADC_IER_PEDEIEN|ADC_IER_WKTIEN), REG_ADC_IER); /*Enable Interrupt */
		}
	}


	if(IsEnableKP_wakeup == 1) {
		if(nuc970_adc->used_state & KP_USED) {
			if( nuc970_adc->kp_state!=KP_IDLE) {
				nuc970_adc->kp_state = KP_IDLE;
			}
			__raw_writel(__raw_readl(REG_ADC_CTL) | (ADC_CTL_ADEN | ADC_CTL_WKPEN|ADC_CTL_PKWPEN), REG_ADC_CTL);
			__raw_writel(ADC_WKISR_WKPEF,REG_ADC_WKISR);  /* Clear kp wakeup up flag */
			__raw_writel(__raw_readl(REG_ADC_IER) | (ADC_IER_WKPIEN), REG_ADC_IER); /*Enable Interrupt */
		}
	}
	LEAVE();
	return 0;
}

static struct platform_device_id nuc970_adc_driver_ids[] = {
	{ "nuc970-adc", 0 },
	{ },
};

static const struct of_device_id nuc970_adc_of_match[] = {
	{ .compatible = "nuvoton,nuc970-adc" },
	{},
};

static struct platform_driver nuc970adc_driver = {
	.probe		= nuc970adc_probe,
	.remove		= nuc970adc_remove,
	.resume		= nuc970adc_resume,
	.suspend		= nuc970adc_suspend,
	.driver		= {
		.name	= "nuc970-adc",
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(nuc970_adc_of_match),
	},
	.id_table	= nuc970_adc_driver_ids,
};
module_platform_driver(nuc970adc_driver);

MODULE_AUTHOR("schung <schung@nuvoton.com>");
MODULE_DESCRIPTION("NUC970/N9H30 adc touch/keypad driver!");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:nuc970-adc");
