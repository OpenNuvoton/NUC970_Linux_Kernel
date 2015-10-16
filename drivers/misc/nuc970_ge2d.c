/* linux/driver/misc/nuc970-ge2d.c
 *
 * Copyright (c) 2015 Nuvoton technology corporation
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */

#include <linux/module.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/ioport.h>
#include <linux/slab.h>
#include <linux/miscdevice.h>
#include <linux/device.h>
#include <linux/spinlock.h>
#include <linux/wait.h>
#include <linux/poll.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include <mach/map.h>
#include <linux/mman.h>

#include <mach/regs-ge.h>
#include "nuc970_ge2d.h"

#define DEBUG
#ifdef DEBUG
    #define dbgprintk  printk
#else
    #define dbgprintk(...)  
#endif

struct nuc970_ge2d {
	spinlock_t g2d_lock;
    void __iomem *regs;
	struct pinctrl *pinctrl;
	wait_queue_head_t wq;    
	struct clk *clk;
	struct clk *eclk;
    unsigned int    irq;
    u8 update;
};

struct nuc970_ge2d *ge2d_dev;

#ifdef DEBUG
static void show_2dregs(void)
{
    dbgprintk("\n[REG_GE2D_TRG] =\t 0x%08x\n", __raw_readl(REG_GE2D_TRG));
    dbgprintk("[REG_GE2D_XYSORG] =\t 0x%08x\n", __raw_readl(REG_GE2D_XYSORG));
    dbgprintk("[REG_GE2D_XYDORG] =\t 0x%08x\n", __raw_readl(REG_GE2D_XYDORG));
    dbgprintk("[REG_GE2D_MISCTL] =\t 0x%08x\n", __raw_readl(REG_GE2D_MISCTL));
    dbgprintk("[REG_GE2D_CTL] =\t 0x%08x\n", __raw_readl(REG_GE2D_CTL));
    dbgprintk("[REG_GE2D_SRCSPA] =\t 0x%08x\n", __raw_readl(REG_GE2D_SRCSPA));
    dbgprintk("[REG_GE2D_DSTSPA] =\t 0x%08x\n", __raw_readl(REG_GE2D_DSTSPA));
    dbgprintk("[REG_GE2D_SDPITCH] =\t 0x%08x\n", __raw_readl(REG_GE2D_SDPITCH));
}
#endif

static unsigned int make_color(unsigned int bpp, unsigned char r, unsigned char g, unsigned char b)
{
    unsigned int color;
    
    switch(bpp)
    {
        case 8:
            color = (r & 0xE0) | ((g & 0xE0) >> 5) | ((b & 0xC0) >> 7);     //332
        break;
        
        case 16:
            color = ((r & 0xF8) << 8) | ((g & 0xFC) << 2) | ((b & 0xF8) >> 3);     //565
        break;
        
        case 24:
        case 32:
            color = (r << 16) | (g << 8) | (b);     //888
        break;
    }
    
    return color;
}

static irqreturn_t nuc970_ge2d_interrupt(int irq, void *dev_id)
{
    dbgprintk("[%s] isr!!\n", __func__);
    ge2d_dev->update = 1;
    __raw_writel(1, REG_GE2D_INTSTS); // clear interrupt status
    
    wake_up_interruptible(&ge2d_dev->wq);        
	return IRQ_HANDLED;
}

static int nuc970_g2d_bitblt(nuc970_g2d_params *params)
{
    unsigned int cmd32 = 0xcc430000, data32;    
    unsigned int src_x = params->src_start_x;
	unsigned int src_y = params->src_start_y;
	unsigned int src_width = params->src_work_width;
	unsigned int src_height = params->src_work_height;
    unsigned int dst_width =  params->dst_work_width;
    unsigned int dst_height =  params->dst_work_height;
	unsigned int dst_x = params->dst_start_x;
	unsigned int dst_y = params->dst_start_y;
	
    dbgprintk("[%s](%d,%d)[%dx%d]-->(%d,%d)[%dx%d]\n", __func__, src_x, src_y, src_width, src_height, dst_x, dst_y, dst_width, dst_height);    
    if (src_x > dst_x) { //+X
        if (src_y > dst_y) { //+Y
        } else { //-Y
            cmd32 |= 0x08;
            src_y = src_y + src_height - 1;
            dst_y = dst_y + src_height - 1;
        }
    } else { //-X
        if (src_y > dst_y) { //+Y
            cmd32 |= 0x04; // 010
            src_x = src_x + src_width - 1;
            dst_x = dst_x + src_width - 1;
        } else { //-Y
            cmd32 |= 0xc; // 110
            src_x = src_x + src_width - 1;
            dst_x = dst_x + src_width - 1;
            src_y = src_y + src_height - 1;
            dst_y = dst_y + src_height - 1;
        }
    }
    
    if (params->alpha_mode) {
        cmd32 |= 0x00200000;
        __raw_writel(cmd32, REG_GE2D_CTL);

        data32 = __raw_readl(REG_GE2D_MISCTL) & 0x0000ffff;
        data32 |= (params->alpha_val << 16);
        __raw_writel(data32, REG_GE2D_MISCTL);
    }
    
    if (params->color_key_mode > 0) {        
        if (params->color_key_mode == 1)
            cmd32 |= 0x00008000;    // color transparency
        else
            cmd32 |= 0x00009000;    // destination pixels control transparency
        
        __raw_writel(params->color_key_val, REG_GE2D_TRNSCOLR);
        __raw_writel(0x00FFFFFF, REG_GE2D_TCMSK);
    }    
    __raw_writel(cmd32, REG_GE2D_CTL);
    
    switch(params->bpp_src)
    {
        case 8:
            cmd32 = 0x00;
            break;
        case 16:
            cmd32 = 0x10;
            break;
        case 32:
            cmd32 = 0x20;
            break;
    }
    __raw_writel(cmd32, REG_GE2D_MISCTL);
  
    __raw_writel((params->dst_full_width << 16 | params->src_full_width), REG_GE2D_SDPITCH);
    __raw_writel((src_y << 16 | src_x), REG_GE2D_SRCSPA);
    __raw_writel((dst_y << 16 | dst_x), REG_GE2D_DSTSPA);
    __raw_writel((src_height << 16 | src_width), REG_GE2D_RTGLSZ);
    
    __raw_writel(params->src_base_addr, REG_GE2D_XYSORG);
    __raw_writel(params->dst_base_addr, REG_GE2D_XYDORG);

#ifdef DEBUG    
    show_2dregs();
#endif
    
    __raw_writel(1, REG_GE2D_TRG);
    wait_event_interruptible(ge2d_dev->wq, ge2d_dev->update != 0);
    ge2d_dev->update = 0;
    
    return 0;    
}

static int nuc970_g2d_bitblt_rop(nuc970_g2d_params *params)
{
    unsigned int cmd32 = 0x00430000 | (params->rop << 24), data32;   
    unsigned int src_x = params->src_start_x;
	unsigned int src_y = params->src_start_y;
	unsigned int src_width = params->src_work_width;
	unsigned int src_height = params->src_work_height;
    unsigned int dst_width =  params->dst_work_width;
    unsigned int dst_height =  params->dst_work_height;
	unsigned int dst_x = params->dst_start_x;
	unsigned int dst_y = params->dst_start_y;
	
    dbgprintk("[%s](%d,%d)[%dx%d]-->(%d,%d)[%dx%d]\n", __func__, src_x, src_y, src_width, src_height, dst_x, dst_y, dst_width, dst_height);    
    
    if (src_x > dst_x) { //+X
        if (src_y > dst_y) { //+Y
        } else { //-Y
            cmd32 |= 0x08;
            src_y = src_y + src_height - 1;
            dst_y = dst_y + src_height - 1;
        }
    } else { //-X
        if (src_y > dst_y) { //+Y
            cmd32 |= 0x04; // 010
            src_x = src_x + src_width - 1;
            dst_x = dst_x + src_width - 1;
        } else { //-Y
            cmd32 |= 0xc; // 110
            src_x = src_x + src_width - 1;
            dst_x = dst_x + src_width - 1;
            src_y = src_y + src_height - 1;
            dst_y = dst_y + src_height - 1;
        }
    }
    
    if (params->alpha_mode) {
        cmd32 |= 0x00200000;
        __raw_writel(cmd32, REG_GE2D_CTL);

        data32 = __raw_readl(REG_GE2D_MISCTL) & 0x0000ffff;
        data32 |= (params->alpha_val << 16);
        __raw_writel(data32, REG_GE2D_MISCTL);
    }
    
    if (params->color_key_mode > 0) {        
        if (params->color_key_mode == 1)
            cmd32 |= 0x00008000;    // color transparency
        else
            cmd32 |= 0x00009000;    // destination pixels control transparency
        
        __raw_writel(params->color_key_val, REG_GE2D_TRNSCOLR);
        __raw_writel(0x00FFFFFF, REG_GE2D_TCMSK);
    }
    
    __raw_writel(cmd32, REG_GE2D_CTL);
    
    switch(params->bpp_src)
    {
        case 8:
            cmd32 = 0x00;
            break;
        case 16:
            cmd32 = 0x10;
            break;
        case 32:
            cmd32 = 0x20;
            break;
    }
    __raw_writel(cmd32, REG_GE2D_MISCTL);
  
    __raw_writel((params->dst_full_width << 16 | params->src_full_width), REG_GE2D_SDPITCH);
    __raw_writel((src_y << 16 | src_x), REG_GE2D_SRCSPA);
    __raw_writel((dst_y << 16 | dst_x), REG_GE2D_DSTSPA);
    __raw_writel((src_height << 16 | src_width), REG_GE2D_RTGLSZ);
    
    __raw_writel(params->src_base_addr, REG_GE2D_XYSORG);
    __raw_writel(params->dst_base_addr, REG_GE2D_XYDORG);
    
    if ((params->rop==0x00) || (params->rop==0xff)) {
        cmd32 = (cmd32 & 0xffff0fff) | 0x00009000;
        __raw_writel(cmd32, REG_GE2D_CTL);
    }

#ifdef DEBUG    
     show_2dregs();
#endif
    
    __raw_writel(1, REG_GE2D_TRG);
    wait_event_interruptible(ge2d_dev->wq, ge2d_dev->update != 0);
    ge2d_dev->update = 0;
    
    return 0;    
}

static int nuc970_g2d_fill_rectangle(nuc970_g2d_params *params)
{
    unsigned int cmd32 = 0xcc430060;    
    unsigned int dst_width =  params->dst_work_width;
    unsigned int dst_height =  params->dst_work_height;
	unsigned int dst_x = params->dst_start_x;
	unsigned int dst_y = params->dst_start_y;
	unsigned int color32;
    
    color32 = make_color(params->bpp_src, params->color_val[G2D_RED], params->color_val[G2D_GREEN], params->color_val[G2D_BLUE]);
    dbgprintk("[%s](%d,%d)[%dx%d],0x%08x\n", __func__, dst_x, dst_y, dst_width, dst_height, color32);    
    
    __raw_writel(color32, REG_GE2D_FGCOLR); // fill with foreground color  
    __raw_writel(cmd32, REG_GE2D_CTL);
    
    switch(params->bpp_src)
    {
        case 8:
            cmd32 = 0x00;
            break;
        case 16:
            cmd32 = 0x10;
            break;
        case 32:
            cmd32 = 0x20;
            break;
    }
    __raw_writel(cmd32, REG_GE2D_MISCTL);
  
    __raw_writel(params->dst_full_width << 16, REG_GE2D_SDPITCH);
    __raw_writel((dst_y << 16 | dst_x), REG_GE2D_DSTSPA);
    __raw_writel((dst_height << 16 | dst_width), REG_GE2D_RTGLSZ);    
        
    __raw_writel(params->src_base_addr, REG_GE2D_XYSORG);
    __raw_writel(params->dst_base_addr, REG_GE2D_XYDORG);

#ifdef DEBUG    
    show_2dregs();
#endif
    
    __raw_writel(1, REG_GE2D_TRG);
    wait_event_interruptible(ge2d_dev->wq, ge2d_dev->update != 0);
    ge2d_dev->update = 0;
    
    return 0;    
}

static int nuc970_g2d_rotation(nuc970_g2d_params *params)
{
    unsigned int cmd32 = 0xcc030000 | (params->rotate << 1);
    unsigned int src_x = params->src_start_x;
	unsigned int src_y = params->src_start_y;
	unsigned int src_width = params->src_work_width;
	unsigned int src_height = params->src_work_height;
	unsigned int dst_x = params->dst_start_x;
	unsigned int dst_y = params->dst_start_y;
	
    dbgprintk("[%s](%d,%d)[%dx%d]-->(%d,%d), r=%d\n", __func__, src_x, src_y, src_width, src_height, dst_x, dst_y, params->rotate);    
        
    __raw_writel(cmd32, REG_GE2D_CTL);
    
    switch(params->bpp_src)
    {
        case 8:
            cmd32 = 0x00;
            break;
        case 16:
            cmd32 = 0x10;
            break;
        case 32:
            cmd32 = 0x20;
            break;
    }
    __raw_writel(cmd32, REG_GE2D_MISCTL);
  
    __raw_writel((params->dst_full_width << 16 | params->src_full_width), REG_GE2D_SDPITCH);
    __raw_writel((src_y << 16 | src_x), REG_GE2D_SRCSPA);
    __raw_writel((dst_y << 16 | dst_x), REG_GE2D_DSTSPA);
    __raw_writel((src_height << 16 | src_width), REG_GE2D_RTGLSZ);
    
    __raw_writel(params->src_base_addr, REG_GE2D_XYSORG);
    __raw_writel(params->dst_base_addr, REG_GE2D_XYDORG);

#ifdef DEBUG    
     show_2dregs();
#endif
    
    __raw_writel(1, REG_GE2D_TRG);
    wait_event_interruptible(ge2d_dev->wq, ge2d_dev->update != 0);
    ge2d_dev->update = 0;
    
    return 0;    
}

static int nuc970_g2d_line(nuc970_g2d_params *params)
{
    unsigned int cmd32; 
	unsigned int x1 = params->line_x1;
    unsigned int x2 = params->line_x2;
    unsigned int y1 = params->line_y1;
    unsigned int y2 = params->line_y2;
	unsigned int color32;
    int abs_X, abs_Y, min, max;
    unsigned int step_constant, initial_error, direction_code;
    
    color32 = make_color(params->bpp_src, params->color_val[G2D_RED], params->color_val[G2D_GREEN], params->color_val[G2D_BLUE]);
    dbgprintk("[%s](%d,%d)-(%d,%d),0x%08x\n", __func__, x1, y1, x2, y2, color32);    
    
    abs_X = abs(x2-x1);   //absolute value
    abs_Y = abs(y2-y1);   //absolute value
    if (abs_X > abs_Y) {  // X major
        max = abs_X;
        min = abs_Y;

        step_constant = (((unsigned int)(2*(min-max))) << 16) | (unsigned int)(2*min);
        initial_error = (((unsigned int)(2*(min)-max)) << 16) | (unsigned int)(max);

        if (x2 > x1) { // +X direction
            if (y2 > y1) // +Y direction
                direction_code = XpYpXl;
            else // -Y direction
                direction_code = XpYmXl;
        } else { // -X direction
            if (y2 > y1) // +Y direction
                direction_code = XmYpXl;
            else // -Y direction
                direction_code = XmYmXl;
        }
    } else { // Y major
        max = abs_Y;
        min = abs_X;

        step_constant = (((unsigned int)(2*(min-max))) << 16) | (unsigned int)(2*min);
        initial_error = (((unsigned int)(2*(min)-max)) << 16) | (unsigned int)(max);

        if (x2 > x1) { // +X direction
            if (y2 > y1) // +Y direction
                direction_code = XpYpYl;
            else // -Y direction
                direction_code = XpYmYl;
        } else { // -X direction
            if (y2 > y1) // +Y direction
                direction_code = XmYpYl;
            else // -Y direction
                direction_code = XmYmYl;
        }
    }
    
    __raw_writel(step_constant, REG_GE2D_BETSC);
    __raw_writel(initial_error, REG_GE2D_BIEPC);

    cmd32 = 0x008b0000 | direction_code;
    __raw_writel(cmd32, REG_GE2D_CTL);
    
    switch(params->bpp_src)
    {
        case 8:
            cmd32 = 0x00;
            break;
        case 16:
            cmd32 = 0x10;
            break;
        case 32:
            cmd32 = 0x20;
            break;
    }
    __raw_writel(cmd32, REG_GE2D_MISCTL);
    
    __raw_writel(color32, REG_GE2D_BGCOLR);
    __raw_writel(color32, REG_GE2D_FGCOLR);
  
    __raw_writel(params->dst_full_width << 16, REG_GE2D_SDPITCH);
    __raw_writel((params->dst_full_height << 16 | params->dst_full_width), REG_GE2D_RTGLSZ);
    __raw_writel(((y1 << 16) | x1), REG_GE2D_DSTSPA);
    __raw_writel(params->src_base_addr, REG_GE2D_XYSORG);
    __raw_writel(params->dst_base_addr, REG_GE2D_XYDORG);
    
 #ifdef DEBUG    
     show_2dregs();
#endif

    __raw_writel(1, REG_GE2D_TRG);
    wait_event_interruptible(ge2d_dev->wq, ge2d_dev->update != 0);
    ge2d_dev->update = 0;

    return 0;    
}

static int nuc970_g2d_stretch(nuc970_g2d_params *params)
{
    unsigned int cmd32, data32;    
    unsigned int src_x = params->src_start_x;
	unsigned int src_y = params->src_start_y;
	unsigned int src_width = params->src_work_width;
	unsigned int src_height = params->src_work_height;
	unsigned int dst_x = params->dst_start_x;
	unsigned int dst_y = params->dst_start_y;
	
    dbgprintk("[%s](%d,%d)-->(%d,%d)[%d/%d,%d/%d]\n", __func__, src_x, src_y, dst_x, dst_y, params->scale_vfn, params->scale_vfm, params->scale_hfn, params->scale_hfm);    
    if(params->scale_mode == G2D_SCALE_DOWN)
        cmd32 = 0xcc030000;
    else
        cmd32 = 0xcc070000;
    __raw_writel(cmd32, REG_GE2D_CTL);
    
    switch(params->bpp_src)
    {
        case 8:
            cmd32 = 0x00;
            break;
        case 16:
            cmd32 = 0x10;
            break;
        case 32:
            cmd32 = 0x20;
            break;
    }
    __raw_writel(cmd32, REG_GE2D_MISCTL);
    
    data32 = (params->scale_vfn << 24) | (params->scale_vfm << 16) | (params->scale_hfn << 8) | params->scale_hfm;
    __raw_writel(data32 ,REG_GE2D_TCNTVHSF);
    
    __raw_writel((params->dst_full_width << 16 | params->src_full_width), REG_GE2D_SDPITCH);
    __raw_writel((src_y << 16 | src_x), REG_GE2D_SRCSPA);
    __raw_writel((dst_y << 16 | dst_x), REG_GE2D_DSTSPA);
    __raw_writel((src_height << 16 | src_width), REG_GE2D_RTGLSZ);
    
    __raw_writel(params->src_base_addr, REG_GE2D_XYSORG);
    __raw_writel(params->dst_base_addr, REG_GE2D_XYDORG);
    
 #ifdef DEBUG    
     show_2dregs();
#endif
    
    __raw_writel(1, REG_GE2D_TRG);
    wait_event_interruptible(ge2d_dev->wq, ge2d_dev->update != 0);
    ge2d_dev->update = 0;
    
    return 0;    
}

static int nuc970_ge2d_open(struct inode *inode, struct file *file)
{
	nuc970_g2d_params *params;
    
	params = (nuc970_g2d_params *)kmalloc(sizeof(nuc970_g2d_params), GFP_KERNEL);
	if(params == NULL){
		printk(KERN_ERR "Instance memory allocation was failed\n");
		return -1;
	}

	memset(params, 0, sizeof(nuc970_g2d_params));

	file->private_data	= (nuc970_g2d_params *)params;
	
    ge2d_dev->update = 0;
	return 0;
}

int nuc970_ge2d_release(struct inode *inode, struct file *file)
{
	nuc970_g2d_params	*params;

	params	= (nuc970_g2d_params *)file->private_data;
	if (params == NULL) {
		printk(KERN_ERR "Can't release s3c_rotator!!\n");
		return -1;
	}

	kfree(params);
	return 0;
}

static long nuc970_ge2d_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    nuc970_g2d_params	*params;
  
    spin_lock(&ge2d_dev->g2d_lock);
    
    params	= (nuc970_g2d_params*)file->private_data;
	if (copy_from_user(params, (nuc970_g2d_params*)arg, sizeof(nuc970_g2d_params)))
        return -EFAULT;
    
    spin_unlock(&ge2d_dev->g2d_lock);
     
    switch (cmd) {      
       case NUC970_GE2D_START_BITBLT:
            nuc970_g2d_bitblt(params);
		break;
       
       case NUC970_GE2D_START_BITBLT_ROP:
            nuc970_g2d_bitblt_rop(params);
		break;
       
       case NUC970_GE2D_FILL_RECTANGLE:
            nuc970_g2d_fill_rectangle(params);
		break;
       
       case NUC970_GE2D_ROTATION:
            nuc970_g2d_rotation(params);
        break;
        
        case NUC970_GE2D_LINE:
            nuc970_g2d_line(params);
        break; 
        
        case NUC970_GE2D_STRETCH:
            nuc970_g2d_stretch(params);
        break; 
    }
    
	return 0;
}

static unsigned int nuc970_ge2d_poll(struct file *file, poll_table *wait)
{
	unsigned int mask = 0;

	poll_wait(file, &ge2d_dev->wq, wait);
	if(ge2d_dev->update)
		mask |= POLLIN | POLLRDNORM;
	return mask;
}

struct file_operations nuc970_ge2d_fops =
{
	.owner		= THIS_MODULE,
	.open		= nuc970_ge2d_open,
	.release	= nuc970_ge2d_release,	
	.unlocked_ioctl	= nuc970_ge2d_ioctl,
	.poll		= nuc970_ge2d_poll,
};

static struct miscdevice nuc970_ge2d_dev = {
    .minor = MISC_DYNAMIC_MINOR,
	.name = "ge2d",
	.fops = &nuc970_ge2d_fops,
};

static int nuc970_ge2d_probe(struct platform_device *pdev)
{
    int irq_num, ret;
    struct resource *res;
    
    ge2d_dev = devm_kzalloc(&pdev->dev, sizeof(struct nuc970_ge2d), GFP_KERNEL);
	if (ge2d_dev == NULL) {
		dev_err(&pdev->dev, "failed to allocate memory for etimer device\n");
		return -ENOMEM;
	}
    
	irq_num = platform_get_irq(pdev, 0);
	if(irq_num <= 0) {
		printk(KERN_ERR "failed to get irq resouce\n");
                return -ENOENT;
	}

	ret = request_irq(irq_num, nuc970_ge2d_interrupt, IRQF_DISABLED, pdev->name, NULL);
	if (ret) {
		printk("request_irq(g2d) failed.\n");
		return ret;
	}
    
    /* get the memory region */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if(res == NULL) {
			printk(KERN_ERR "failed to get memory region resouce\n");
			return -ENOENT;
	}

	ge2d_dev->regs = ioremap(res->start, resource_size(res));
	if(ge2d_dev->regs == NULL) {
		printk(KERN_ERR "failed ioremap\n");
                return -ENOENT;
	}
 
    // configure engine clock	
	ge2d_dev->clk = clk_get(NULL, "ge2d_hclk");
	ge2d_dev->eclk = clk_get(NULL, "ge2d_eclk");
	
	if (IS_ERR(ge2d_dev->clk)) {
		pr_debug("failed to get etmr clock\n");
		ret = PTR_ERR(ge2d_dev->clk);
		goto out1;
	}

	if (IS_ERR(ge2d_dev->eclk)) {
		pr_debug("failed to get etmr eclock\n");
		ret = PTR_ERR(ge2d_dev->eclk);
		goto out1;
	}

	clk_prepare(ge2d_dev->clk);
	clk_enable(ge2d_dev->clk);
    clk_prepare(ge2d_dev->eclk);
	clk_enable(ge2d_dev->eclk);
    
    misc_register(&nuc970_ge2d_dev);
    
    spin_lock_init(&ge2d_dev->g2d_lock);
    
    ge2d_dev->pinctrl = devm_pinctrl_get(&pdev->dev);
	ge2d_dev->irq = irq_num;
    
    __raw_writel(0x00ffffff, REG_GE2D_WRPLNMSK);
        
	init_waitqueue_head(&ge2d_dev->wq);

	platform_set_drvdata(pdev, ge2d_dev);
	
	return 0;
out1:

	free_irq(ge2d_dev->irq, NULL);

	return(0);
}

static int nuc970_ge2d_remove(struct platform_device *pdev)
{
	misc_deregister(&nuc970_ge2d_dev);
    free_irq(ge2d_dev->irq, NULL);

	return 0;
}


#ifdef CONFIG_PM
static int nuc970_ge2d_suspend(struct platform_device *pdev, pm_message_t state)
{
	return 0;
}

static int nuc970_ge2d_resume(struct platform_device *pdev)
{

	return 0;
}

#else
#define nuc970_ge2d_suspend 	NULL
#define nuc970_ge2d_resume	    NULL
#endif

static struct platform_driver nuc970_ge2d_driver = {
	.driver		= {
		.owner	= THIS_MODULE,
		.name	= "nuc970-ge2d",
	},
	.probe		= nuc970_ge2d_probe,
	.remove		= nuc970_ge2d_remove,
	.suspend	= nuc970_ge2d_suspend,
	.resume		= nuc970_ge2d_resume,
};


module_platform_driver(nuc970_ge2d_driver);

MODULE_AUTHOR("Nuvoton Technology Corp.");
MODULE_ALIAS("platform:nuc970-ge2d");
MODULE_LICENSE("GPL");
