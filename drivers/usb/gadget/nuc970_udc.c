/*
 * linux/drivers/usb/gadget/nuc970_udc.c
 *
 * Nuvoton NUC970 MCU on-chip full speed USB device controllers
 *
 * Copyright (C) 2014 Nuvoton Technology Corp
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
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#include <linux/proc_fs.h>
#include <linux/prefetch.h>
#include <linux/usb/ch9.h>
#include <linux/of.h>

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/ioport.h>
#include <linux/slab.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/clk.h>
#include <linux/list.h>
#include <linux/interrupt.h>
#include <linux/usb/gadget.h>
#include <linux/platform_device.h>
#include <asm/byteorder.h>
#include <asm/io.h>
#include <asm/irq.h>

#include <mach/nuc970_usbd.h>
#include <mach/map.h>
#include <mach/regs-gcr.h>
#include <mach/regs-clock.h>
#include <mach/regs-gpio.h>
#include "nuc970_udc.h"

#define DRIVER_DESC     "NUVOTON USB Device Controller Gadget"
#define DRIVER_VERSION  "16 August 2014"
#define DRIVER_AUTHOR   "shirley <clyu2@nuvoton.com>"

static const char gadget_name [] = "nuc970-udc";
static const char driver_desc [] = DRIVER_DESC;
static const char ep0name [] = "ep0";

static const char *const ep_name[] = {
    ep0name,                                /* everyone has ep0 */
    "ep1", "ep2", "ep3", "ep4", "ep5", "ep6", "ep7", "ep8", "ep9", "ep10", "ep11", "ep12"
};

#define EP0_FIFO_SIZE           64
#define EP_FIFO_SIZE            512

static struct nuc970_udc controller;

static void udc_isr_rst(struct nuc970_udc *dev);
static void udc_isr_dma(struct nuc970_udc *dev);
static void udc_isr_ctrl_pkt(struct nuc970_udc *dev);
static void udc_isr_update_dev(struct nuc970_udc *dev);
static u32 udc_transfer(struct nuc970_ep *ep, u8* buf, size_t size, u32 mode);
static void nuc970_udc_enable(struct nuc970_udc *dev);
static void nuc970_udc_disable(struct nuc970_udc *dev);



static void nuke (struct nuc970_udc *udc, struct nuc970_ep *ep)
{
    while (!list_empty (&ep->queue))
    {
        struct nuc970_request *req;
        req = list_entry (ep->queue.next, struct nuc970_request, queue);
        list_del_init (&req->queue);
        req->req.status = -ESHUTDOWN;
        spin_unlock (&udc->lock);
        req->req.complete (&ep->ep, &req->req);
        spin_lock (&udc->lock);
    }
}


static void done(struct nuc970_ep *ep, struct nuc970_request *req, int status)
{
    struct nuc970_udc *udc = &controller;

    list_del_init(&req->queue); //del req->queue from ep->queue

    if (list_empty(&ep->queue))
    {
        if (ep->index)
            __raw_writel(0, controller.reg + REG_USBD_EPA_IRQ_ENB + 0x28*(ep->index-1));
    }
    else
    {
        __raw_writel(ep->irq_enb, controller.reg + REG_USBD_EPA_IRQ_ENB + 0x28*(ep->index-1));
    }

    if (likely (req->req.status == -EINPROGRESS))
        req->req.status = status;
    else
        status = req->req.status;

    udc = ep->dev;

    usb_gadget_unmap_request(&udc->gadget, &req->req, ep->ep_dir);
    req->req.complete(&ep->ep, &req->req);
}


static void start_write(struct nuc970_ep *ep, u8* buf,u32 length)
{
    struct nuc970_udc *dev = ep->dev;
    u32 volatile reg;

    if (dev->usb_dma_trigger)
    {
        pr_devel("*** dma trigger ***\n");
        return;
    }

    dev->usb_dma_trigger = 1;
    dev->usb_dma_cnt = length;
    dev->usb_dma_owner = ep->index;

    __raw_writel((USB_DMA_REQ | USB_RST_STS | USB_SUS_REQ | USB_FLT_DET), controller.reg + REG_USBD_IRQ_ENB);

    __raw_writel((u32)buf, controller.reg + REG_USBD_AHB_DMA_ADDR);//Tell DMA the memory physcal address
    __raw_writel(length, controller.reg + REG_USBD_DMA_CNT);

    reg = __raw_readl(controller.reg + REG_USBD_DMA_CTRL_STS);
    if ((reg & 0x40) != 0x40)
        __raw_writel((reg | 0x00000020), controller.reg + REG_USBD_DMA_CTRL_STS);

    return ;
}

static void start_read(struct nuc970_ep *ep, u8* buf, u32 length)
{
    struct nuc970_udc   *dev = ep->dev;

    if (dev->usb_dma_trigger) {
        pr_devel("*** dma trigger ***\n");
        return;
    }

    __raw_writel((USB_DMA_REQ | USB_RST_STS | USB_SUS_REQ | USB_FLT_DET), controller.reg + REG_USBD_IRQ_ENB);
    __raw_writel((u32)buf, controller.reg + REG_USBD_AHB_DMA_ADDR);//Tell DMA the memory address
    __raw_writel(length, controller.reg + REG_USBD_DMA_CNT);

    dev->usb_dma_trigger = 1;
    dev->usb_dma_cnt = length;
    dev->usb_dma_loop = (length+31)/32;
    dev->usb_dma_owner = ep->index;

    __raw_writel(__raw_readl(controller.reg + REG_USBD_DMA_CTRL_STS)|0x00000020, controller.reg + REG_USBD_DMA_CTRL_STS);

    return ;
}

static inline void clear_ep_state (struct nuc970_udc *dev)
{
    unsigned i;

    /* hardware SET_{CONFIGURATION,INTERFACE} automagic resets endpoint
    * fifos, and pending transactions mustn't be continued in any case.
    */
    for (i = 0; i < NUC970_ENDPOINTS; i++)
        nuke(dev, &dev->ep[i]);
}

/*
 *  write_packet
 */
static inline int
write_packet(struct nuc970_ep *ep, struct nuc970_request *req)
{
    struct nuc970_udc *udc = &controller;
    unsigned len, tmp;
    u8  *buf;
    u32 data, i;
    u32 max;

    buf = req->req.buf + req->req.actual;
    prefetch(buf);

    if (ep->ep_num == 0)
    { //ctrl pipe don't use DMA
        max = ep->ep.maxpacket;
        len = min(req->req.length - req->req.actual, max);

        if (len == 0)
        {
            if (req->req.zero&&!req->req.length)
                    __raw_writel(CEP_ZEROLEN, controller.reg + REG_USBD_CEP_CTRL_STAT);
        }
        else
        {
            tmp = len / 4;
            for (i=0; i<tmp; i++)
            {
                data  = *buf++;
                data |= *buf++ << 8;
                data |= *buf++ << 16;
                data |= *buf++ << 24;
                __raw_writel(data, controller.reg + REG_USBD_CEP_DATA_BUF);
            }

            tmp = len % 4;
            for (i=0; i<tmp; i++)
            {
                __raw_writeb( *buf++ & 0xff, controller.reg + REG_USBD_CEP_DATA_BUF);
            }
            __raw_writel(len, controller.reg + REG_USBD_IN_TRNSFR_CNT);
        }
        req->req.actual += len;
    }
    else
    {
        len = req->req.length - req->req.actual;

        usb_gadget_map_request(&udc->gadget, &req->req, ep->ep_dir);
        buf = (u8*)(req->req.dma + req->req.actual);

        if (len == 0)
        {
            __raw_writel((__raw_readl(controller.reg + REG_USBD_EPA_RSP_SC+0x28*(ep->index-1))&0xF7)|EP_ZERO_IN,
                         controller.reg + REG_USBD_EPA_RSP_SC+0x28*(ep->index-1));
        }
        else
        {
            len = udc_transfer(ep, buf, len, DMA_WRITE);
        }
        req->req.actual += len;
    }

    return len;
}

/*
 *  write_fifo
 */
// return:  0 = still running, 1 = completed, negative = errno
static int write_fifo(struct nuc970_ep *ep, struct nuc970_request *req)
{
    u32 len;

    len = write_packet(ep, req);

    /* last packet is often short (sometimes a zlp) */

    if (req->req.length == req->req.actual/* && !req->req.zero*/)
    {
        done(ep, req, 0);
        return 1;
    }
    else
        return 0;
}

static inline int read_packet(struct nuc970_ep *ep,u8 *buf,
                              struct nuc970_request *req, u16 cnt)
{
    struct nuc970_udc *udc = &controller;
    unsigned    len, tmp, fifo_count;
    u16 data, i;

    if (ep->ep_num == 0)
    { //ctrl pipe don't use DMA
        fifo_count = __raw_readl(controller.reg + REG_USBD_CEP_CNT);
        len = min(req->req.length - req->req.actual, fifo_count);

        tmp = len / 4;
        for (i=0; i<tmp; i++)
        {
            data = __raw_readl(controller.reg + REG_USBD_CEP_DATA_BUF);
            *buf++ = data  & 0xFF;
            *buf++ = (data & 0xFF00) >> 8;
            *buf++ = (data & 0xFF0000) >> 16;
            *buf++ = (data & 0xFF000000) >> 24;
        }

        tmp = len % 4;
        for (i=0; i<tmp; i++)
        {
            data = __raw_readb(controller.reg + REG_USBD_CEP_DATA_BUF);
            *buf++ = data&0xFF;
        }
        req->req.actual += len;

    }
    else
    {
        usb_gadget_map_request(&udc->gadget, &req->req, ep->ep_dir);
        buf = (u8*)req->req.dma;
        len = req->req.length - req->req.actual;

        if (cnt && cnt < ep->ep.maxpacket)
            len = udc_transfer(ep, buf, cnt, DMA_READ);
        else if (len)
            len = udc_transfer(ep, buf, len, DMA_READ);
        req->req.actual += len;
    }

    return len;
}

// return:  0 = still running, 1 = queue empty, negative = errno
static int read_fifo(struct nuc970_ep *ep, struct nuc970_request *req, u16 cnt)
{
    u8 *buf;
    unsigned bufferspace;
    int is_last=1;
    int fifo_count = 0;

    buf = req->req.buf + req->req.actual;
    bufferspace = req->req.length - req->req.actual;
    if (!bufferspace)
    {
        pr_err("read_fifo: Buffer full !!\n");
        return -1;
    }

    fifo_count=read_packet(ep, buf, req, cnt);

    if (req->req.length == req->req.actual)
        done(ep, req, 0);
    else if (fifo_count && fifo_count < ep->ep.maxpacket)
    {
        done(ep, req, 0);
        /* overflowed this request?  flush extra data */
        if (req->req.length != req->req.actual)
        {
            pr_devel("%s(): EOVERFLOW set\n", __FUNCTION__);
            if (req->req.short_not_ok)
                req->req.status = -EOVERFLOW;   //device read less then host write
        }
    }
    else
        is_last = 0;

    return is_last;
}

static void Get_SetupPacket(struct usb_ctrlrequest *pcrq, u32 temp)
{
    pcrq->bRequestType = (u8)temp & 0xff;
    pcrq->bRequest = (u8)(temp >> 8) & 0xff;
    pcrq->wValue = (u16)__raw_readl(controller.reg + REG_USBD_SETUP3_2);
    pcrq->wIndex = (u16)__raw_readl(controller.reg + REG_USBD_SETUP5_4);
    pcrq->wLength = (u16)__raw_readl(controller.reg + REG_USBD_SETUP7_6);

    pr_devel("setup:%x,%x,%x,%x,%x\n", pcrq->bRequestType, pcrq->bRequest, pcrq->wValue, pcrq->wIndex, pcrq->wLength);
}

void paser_irq_stat(int irq, struct nuc970_udc *dev)
{
    __raw_writel(irq, controller.reg + REG_USBD_IRQ_STAT);//clear irq bit

    switch (irq)
    {
        case USB_SOF:
            break;

        case USB_RST_STS://reset
            udc_isr_rst(dev);
            break;

        case USB_RESUME:
            __raw_writel((USB_RST_STS | USB_SUS_REQ | USB_FLT_DET), controller.reg + REG_USBD_IRQ_ENB);
            break;

        case USB_SUS_REQ:
            if (dev == NULL)
                break;

            __raw_writel((USB_RST_STS | USB_RESUME | USB_FLT_DET), controller.reg + REG_USBD_IRQ_ENB);
            break;

        case USB_HS_SETTLE:
            dev->gadget.speed = USB_SPEED_HIGH;
            dev->usb_devstate = 1;      //default state
            dev->usb_address = 0;       //zero
            __raw_writel(0x002, controller.reg + REG_USBD_CEP_IRQ_ENB);
            break;

        case USB_DMA_REQ:
            udc_isr_dma(dev);
            break;

        case USABLE_CLK:
            break;

        case USB_FLT_DET:
            if (__raw_readl(controller.reg + REG_USBD_PHY_CTL) & 0x80000000)
            {
                printk("plug in\n");
                nuc970_udc_enable(dev);
            }
            else
            {
                printk("plug out\n");
                nuc970_udc_disable(dev);
            }
            break;

        default:
            break;
    }

    return ;
}

void paser_irq_cep(int irq, struct nuc970_udc *dev, u32 IrqSt)
{
    struct nuc970_ep *ep = &dev->ep[0];
    struct nuc970_request *req;
    int is_last = 1;

    if (list_empty(&ep->queue))
        req = 0;
    else
        req = list_entry(ep->queue.next, struct nuc970_request, queue);

    switch (irq)
    {
        case CEP_SUPPKT://receive setup packet
            dev->ep0state=EP0_IDLE;
            dev->setup_ret = 0;
            udc_isr_ctrl_pkt(dev);
            break;

        case CEP_DATA_RXD:
            if (dev->ep0state == EP0_OUT_DATA_PHASE)
            {
                if (req)
                    is_last = read_fifo(ep,req, 0);

                __raw_writel(0x400, controller.reg + REG_USBD_CEP_IRQ_STAT);

                if (!is_last)
                    __raw_writel(0x440, controller.reg + REG_USBD_CEP_IRQ_ENB);//enable out token and status complete int
                else
                { //transfer finished
                    __raw_writel(0x04C, controller.reg + REG_USBD_CEP_IRQ_STAT);
                    __raw_writel(CEP_NAK_CLEAR, controller.reg + REG_USBD_CEP_CTRL_STAT);   // clear nak so that sts stage is complete
                    __raw_writel(0x400, controller.reg + REG_USBD_CEP_IRQ_ENB);     // suppkt int//enb sts completion int
                    dev->ep0state = EP0_END_XFER;
                }
            }
            return;

        case CEP_IN_TOK:
            if ((IrqSt & CEP_STS_END))
                dev->ep0state=EP0_IDLE;

            if (dev->setup_ret < 0)
            { // == -EOPNOTSUPP)
                pr_devel("CEP send zero pkt\n");
                __raw_writel(CEP_ZEROLEN, controller.reg + REG_USBD_CEP_CTRL_STAT);
                __raw_writel(0x400, controller.reg + REG_USBD_CEP_IRQ_ENB);     //enb sts completion int
            }

            else if (dev->ep0state == EP0_IN_DATA_PHASE)
            {
                if (req)
                    is_last = write_fifo(ep,req);

                if (!is_last)
                    __raw_writel(0x408, controller.reg + REG_USBD_CEP_IRQ_ENB);
                else
                {
                    if (dev->setup_ret >= 0)
                        __raw_writel(CEP_NAK_CLEAR, controller.reg + REG_USBD_CEP_CTRL_STAT);   // clear nak so that sts stage is complete
                    __raw_writel(0x402, controller.reg + REG_USBD_CEP_IRQ_ENB);     // suppkt int//enb sts completion int

                    if (dev->setup_ret < 0)//== -EOPNOTSUPP)
                        dev->ep0state=EP0_IDLE;
                    else if (dev->ep0state != EP0_IDLE)
                        dev->ep0state=EP0_END_XFER;
                }
            }

            return;

        case CEP_PING_TOK:
            __raw_writel(0x402, controller.reg + REG_USBD_CEP_IRQ_ENB);     // suppkt int//enb sts completion int
            return;

        case CEP_DATA_TXD:
            return;

        case CEP_STS_END:
            __raw_writel(0x4A, controller.reg + REG_USBD_CEP_IRQ_ENB);
            udc_isr_update_dev(dev);
            dev->ep0state=EP0_IDLE;
            dev->setup_ret = 0;
            break;

        default:
            ;
    }
    return ;
}


void paser_irq_nep(int irq, struct nuc970_ep *ep, u32 IrqSt)
{
    struct nuc970_udc *dev = ep->dev;
    struct nuc970_request *req;
    int i;
    u16 data, fifo_count, tmp, loop;
    u8 *buf;
    u32 datacnt_reg;

    if (list_empty(&ep->queue))
    {
        pr_devel("nep->queue is empty\n");
        req = 0;
    }
    else
    {
        __raw_writel(__raw_readl(controller.reg + REG_USBD_EPA_IRQ_STAT + 0x28*(ep->index-1)),
                     controller.reg + REG_USBD_EPA_IRQ_STAT + 0x28*(ep->index-1));
        req = list_entry(ep->queue.next, struct nuc970_request, queue);
    }
    pr_devel("paser_irq_nep:0x%x\n", (int)req->req.dma);
    switch (irq)
    {
        case EP_IN_TOK:
            __raw_writel(irq, controller.reg + REG_USBD_EPA_IRQ_STAT + 0x28*(ep->index-1));

            if (ep->ep_type == EP_TYPE_BLK)
            {
                if (__raw_readl(controller.reg + REG_USBD_EPA_RSP_SC+0x28*(ep->index-1))&0x40)
                { //send last packet
                    pr_devel("send last packet\n");
                    break;
                }
            }
            if (req == NULL)
            {
                __raw_writel(0, controller.reg + REG_USBD_EPA_IRQ_ENB + 0x28*(ep->index-1));
                break;
            }

            while (__raw_readl(controller.reg + REG_USBD_DMA_CTRL_STS)&0x20);//wait DMA complete
            if (dev->usb_dma_trigger)
            {
                pr_devel("IN dma triggered\n");
                while ((__raw_readl(controller.reg + REG_USBD_IRQ_STAT) & 0x20) == 0);
                __raw_writel(0x20, controller.reg + REG_USBD_IRQ_STAT);
                udc_isr_dma(dev);
            }

            write_fifo(ep,req);
            break;

        case EP_BO_SHORT_PKT:
            if (req)
            {
                if (dev->usb_dma_trigger)
                {
                    loop = __raw_readl(controller.reg + REG_USBD_EPA_DATA_CNT + 0x28*(ep->index-1))>>16;
                    pr_devel("loop=%d, %d\n", loop, dev->usb_dma_loop);
                    loop = dev->usb_dma_loop - loop;

                    if (loop)
                        req->req.actual += loop*32;//each loop 32 bytes
                    pr_devel("reset dma\n");
                    dev->usb_dma_trigger = 0;
                    //reset DMA
                    __raw_writel(0x80, controller.reg + REG_USBD_DMA_CTRL_STS);
                    __raw_writel(0x00, controller.reg + REG_USBD_DMA_CTRL_STS);
                    __raw_writel(dev->irq_enbl, controller.reg + REG_USBD_IRQ_ENB_L);
                }

                fifo_count = __raw_readl(controller.reg + REG_USBD_EPA_DATA_CNT + 0x28*(ep->index-1));

                buf = req->req.buf + req->req.actual;
                tmp = fifo_count / 4;
                for (i=0; i<tmp; i++)
                {
                    data = __raw_readl(controller.reg + REG_USBD_EPA_DATA_BUF + 0x28*(ep->index-1));
                    *buf++ = data&0xFF;
                    *buf++ = (data&0xFF00) >> 8;
                    *buf++ = (data&0xFF0000) >> 16;
                    *buf++ = (data&0xFF000000) >> 24;
                }

                tmp = fifo_count % 4;
                for (i=0; i<tmp; i++)
                {
                    data = __raw_readb(controller.reg + REG_USBD_EPA_DATA_BUF + 0x28*(ep->index-1));
                    *buf++ = data&0xFF;
                }

                if (ep->buffer_disabled)
                {
                    __raw_writel((__raw_readl(controller.reg + REG_USBD_EPA_RSP_SC + 0x28*(ep->index-1)))&0x77,
                                 controller.reg + REG_USBD_EPA_RSP_SC + 0x28*(ep->index-1));//enable buffer
                    __raw_writel((__raw_readl(controller.reg + REG_USBD_EPA_RSP_SC+0x28*(ep->index-1))&0xF7)|0x80,
                                 controller.reg + REG_USBD_EPA_RSP_SC+0x28*(ep->index-1));//disable buffer when short packet
                }
                req->req.actual += fifo_count;
                done(ep, req, 0);
            }
            else
                __raw_writel(0, controller.reg + REG_USBD_EPA_IRQ_ENB + 0x28*(ep->index-1));

            break;

        case EP_DATA_RXD:
            if (req == NULL)
            {
                __raw_writel(0, controller.reg + REG_USBD_EPA_IRQ_ENB + 0x28*(ep->index-1));
                break;
            }
            datacnt_reg = (u32)(REG_USBD_EPA_DATA_CNT + 0x28*(ep->index-1));
            if (__raw_readl(controller.reg + datacnt_reg) == 0)
                break;

            while (__raw_readl(controller.reg + REG_USBD_DMA_CTRL_STS)&0x20);//wait DMA complete
            fifo_count = __raw_readl(controller.reg + datacnt_reg);

            if (dev->usb_dma_trigger)
            {
                pr_devel("RxED dma triggered\n");
                while ((__raw_readl(controller.reg + REG_USBD_IRQ_STAT) & 0x20) == 0);
                __raw_writel(0x02, controller.reg + REG_USBD_IRQ_STAT);
                udc_isr_dma(dev);
            }
            read_fifo(ep,req, __raw_readl(controller.reg + datacnt_reg));
            break;

        default:
            pr_devel("irq: %d not handled !\n",irq);
            __raw_writel(irq, controller.reg + REG_USBD_EPA_IRQ_STAT + 0x28*(ep->index-1));
    }

    return ;
}

void paser_irq_nepint(int irq, struct nuc970_ep *ep, u32 IrqSt)
{
    struct nuc970_udc *dev = ep->dev;
    struct nuc970_request   *req;

    __raw_writel(irq, controller.reg + REG_USBD_EPA_IRQ_STAT + 0x28*(ep->index-1));

    if (list_empty(&ep->queue))
    {
        pr_devel("nepirq->queue is empty\n");
        req = 0;
        return;
    }
    else
    {
        req = list_entry(ep->queue.next, struct nuc970_request, queue);
    }

    switch (irq)
    {
        case EP_IN_TOK:
            while (__raw_readl(controller.reg + REG_USBD_DMA_CTRL_STS)&0x20);//wait DMA complete
            if (dev->usb_dma_trigger)
            {
                pr_devel("int IN dma triggered\n");
                while ((__raw_readl(controller.reg + REG_USBD_IRQ_STAT) & 0x20) == 0);
                __raw_writel(0x20, controller.reg + REG_USBD_IRQ_STAT);
                udc_isr_dma(dev);
            }
            write_fifo(ep,req);
            break;
        default:
            pr_devel("irq: %d not handled !\n",irq);
            __raw_writel(irq, controller.reg + REG_USBD_EPA_IRQ_STAT + 0x28*(ep->index-1));
    }
    return ;
}


/*
 *      nuc970_udc_irq - interrupt handler
 */
static irqreturn_t nuc970_udc_irq(int irq, void *_dev)
{
    struct nuc970_udc *dev;
    struct nuc970_ep *ep;
    u32 volatile IrqStL, IrqEnL;
    u32 volatile IrqSt, IrqEn;
    int i=0, j;

    dev=(struct nuc970_udc *)(_dev);

    IrqStL = __raw_readl(controller.reg + REG_USBD_IRQ_STAT_L); /* 0x000 register get interrupt status */
    IrqEnL = __raw_readl(controller.reg + REG_USBD_IRQ_ENB_L);

    IrqStL = IrqStL & IrqEnL ;
    if (!IrqStL)
    {
        pr_err("Not our interrupt !\n");
        return IRQ_HANDLED;
    }

    if (IrqStL & IRQ_USB_STAT)
    {
        IrqSt = __raw_readl(controller.reg + REG_USBD_IRQ_STAT);
        IrqEn = __raw_readl(controller.reg + REG_USBD_IRQ_ENB);
        __raw_writel(IrqSt, controller.reg + REG_USBD_IRQ_STAT);
        IrqSt = IrqSt & IrqEn ;
        if (IrqSt && dev->driver)
        {
            for (i=0; i<9; i++)
            {
                if ((i == 6) || (i == 7))
                    continue;
                if (IrqSt&(1<<i))
                {
                    paser_irq_stat(1<<i,dev);
                    break;
                }
            }
        }
    }//end IRQ_USB_STAT

    if (IrqStL & IRQ_CEP)
    {
        IrqSt = __raw_readl(controller.reg + REG_USBD_CEP_IRQ_STAT);
        IrqEn = __raw_readl(controller.reg + REG_USBD_CEP_IRQ_ENB);
        IrqSt = IrqSt & IrqEn ;
        __raw_writel(IrqSt, controller.reg + REG_USBD_CEP_IRQ_STAT);

        if (IrqSt && dev->driver)
        {
            if (IrqSt&CEP_STS_END)
            { //deal with STS END
                if (dev->ep0state == EP0_OUT_DATA_PHASE)
                    IrqSt &= 0x1BF7;
                paser_irq_cep(CEP_STS_END,dev,IrqSt);
            }
            for (i=0; i<13; i++)
            {
                if (i == 10)
                    continue;
                if (IrqSt&(1<<i))
                {
                    paser_irq_cep(1<<i,dev,IrqSt);
                    //break;
                }
            }
        }
    }

    if (IrqStL & IRQ_NCEP)
    {
        IrqStL >>= 2;

        for (j = 0; j < NUC970_ENDPOINTS-1; j++)
        { 
            if (IrqStL & (1 << j))
            {
                //in-token and out token interrupt can deal with one only
                IrqSt = __raw_readl(controller.reg + REG_USBD_EPA_IRQ_STAT + 0x28 * j);
                IrqEn = __raw_readl(controller.reg + REG_USBD_EPA_IRQ_ENB + 0x28 * j);

                IrqSt = IrqSt & IrqEn ;
                if (IrqSt && dev->driver)
                {
                    ep = &dev->ep[j+1];

                    for (i=12; i>=0; i--)
                    {
                        if (IrqSt&(1<<i))
                        {
                            if ((1<<i) == EP_BO_SHORT_PKT)
                                IrqSt &= 0x1FCF;//clear out token/RxED intr
                            if ((ep->ep_type == EP_TYPE_BLK) || (ep->ep_type == EP_TYPE_ISO))
                                paser_irq_nep(1<<i, ep, IrqSt);
                            else if (ep->ep_type == EP_TYPE_INT)
                                paser_irq_nepint(1<<i, ep, IrqSt);
                            break;
                        }
                    }
                }
            }
        }
    }//if end

    return IRQ_HANDLED;
}


static s32 sram_data[13][2] = {{0,0x40}};

//0-3F for Ctrl pipe
s32 get_sram_base(struct nuc970_udc *dev, u32 max)
{
    int i, cnt = 1, j;
    s32 start, end;

    for (i = 1; i < NUC970_ENDPOINTS; i++)
    {
        struct nuc970_ep *ep = &dev->ep[i];

        start = __raw_readl(controller.reg + REG_USBD_EPA_START_ADDR+0x28*(ep->index-1));
        end = __raw_readl(controller.reg + REG_USBD_EPA_END_ADDR+0x28*(ep->index-1));
        if (end - start > 0)
        {
                sram_data[cnt][0] = start;
                sram_data[cnt][1] = end + 1;
                cnt++;
        }
    }
    if (cnt == 1)
            return 0x40;
    //sorting from small to big
    j= 1;
    while ((j<cnt))
    {
        for (i=0; i<cnt -j; i++)
        {
            if (sram_data[i][0]>sram_data[i+1][0])
            {
                start = sram_data[i][0];
                end = sram_data[i][1];
                sram_data[i][0] = sram_data[i+1][0];
                sram_data[i][1] = sram_data[i+1][1];
                sram_data[i+1][0] = start;
                sram_data[i+1][1] = end;
            }
        }
        j++;
    }

    for (i = 0; i< cnt-1; i++)
    {
        if (sram_data[i+1][0] - sram_data[i][1] >= max)
            return sram_data[i][1];
    }

    if (0x800 - sram_data[cnt-1][1] >= max)
        return sram_data[cnt-1][1];

    return -ENOBUFS;
}

/*
 *  nuc970_ep_enable
 */
static int nuc970_ep_enable (struct usb_ep *_ep, const struct usb_endpoint_descriptor *desc)
{
    struct nuc970_udc *dev;
    struct nuc970_ep *ep;
    u32 max, tmp;
    unsigned long flags;
    u32 int_en_reg;
    s32 sram_addr;

    ep = container_of (_ep, struct nuc970_ep, ep);
    if (!_ep || !desc || _ep->name == ep0name || desc->bDescriptorType != USB_DT_ENDPOINT)
        return -EINVAL;
    dev = ep->dev;

    if (!dev->driver || dev->gadget.speed == USB_SPEED_UNKNOWN)
        return -ESHUTDOWN;

    max = usb_endpoint_maxp(desc);

    spin_lock_irqsave (&dev->lock, flags);
    _ep->maxpacket = max & 0x7ff;

    ep->ep.desc = desc;
    ep->bEndpointAddress = desc->bEndpointAddress;

    /* set max packet */
    if (ep->index != 0)
    {
        __raw_writel(max, controller.reg + REG_USBD_EPA_MPS + 0x28*(ep->index-1));
        ep->ep.maxpacket = max;

        sram_addr = get_sram_base(dev, max);

        if (sram_addr < 0)
            return sram_addr;

        __raw_writel(sram_addr, controller.reg + REG_USBD_EPA_START_ADDR+0x28*(ep->index-1));
        sram_addr = sram_addr + max;
        __raw_writel(sram_addr-1, controller.reg + REG_USBD_EPA_END_ADDR+0x28*(ep->index-1));
    }

    /* set type, direction, address; reset fifo counters */
    if (ep->index != 0)
    {
        ep->ep_num = desc->bEndpointAddress & ~USB_DIR_IN;
        ep->ep_dir = desc->bEndpointAddress &0x80 ? 1 : 0;
        ep->ep_type = ep->ep.desc->bmAttributes&USB_ENDPOINT_XFERTYPE_MASK;
        if (ep->ep_type == USB_ENDPOINT_XFER_ISOC)
        {
            ep->ep_type = EP_TYPE_ISO;
            ep->ep_mode = EP_MODE_FLY;
        } else if (ep->ep_type == USB_ENDPOINT_XFER_BULK)
        {
            ep->ep_type = EP_TYPE_BLK;
            ep->ep_mode = EP_MODE_AUTO;
        }
        if (ep->ep_type == USB_ENDPOINT_XFER_INT)
        {
            ep->ep_type = EP_TYPE_INT;
            ep->ep_mode = EP_MODE_MAN;
        }
        __raw_writel(0x9, controller.reg + REG_USBD_EPA_RSP_SC+0x28*(ep->index-1));//DATA0 and flush SRAM
        __raw_writel(ep->ep_num<<4|ep->ep_dir<<3|ep->ep_type<<1|1,
                     controller.reg + REG_USBD_EPA_CFG+0x28*(ep->index-1));
        __raw_writel(ep->ep_mode, controller.reg + REG_USBD_EPA_RSP_SC+0x28*(ep->index-1));

        /* enable irqs */
        int_en_reg = __raw_readl(controller.reg + REG_USBD_IRQ_ENB_L);
        __raw_writel(int_en_reg | (1<<(ep->index+1)), controller.reg + REG_USBD_IRQ_ENB_L);
        dev->irq_enbl = __raw_readl(controller.reg + REG_USBD_IRQ_ENB_L);

        if (ep->ep_type == EP_TYPE_BLK)
        {
            if (ep->ep_dir)//IN
                ep->irq_enb = 0x40;
            else
            {
                ep->irq_enb = 0x10;
                __raw_writel((__raw_readl(controller.reg + REG_USBD_EPA_RSP_SC+0x28*(ep->index-1))&0xF7)|0x80,
                             controller.reg + REG_USBD_EPA_RSP_SC + 0x28*(ep->index-1));//disable buffer when short packet
                ep->buffer_disabled = 1;
            }
        }
        else if (ep->ep_type == EP_TYPE_INT)
            ep->irq_enb = 0x40;
        else if (ep->ep_type == EP_TYPE_ISO)
        {
            if (ep->ep_dir)//IN
                ep->irq_enb = 0x40;
            else
                ep->irq_enb = 0x20;
        }
    }

    /* print some debug message */
    tmp = desc->bEndpointAddress;
    pr_devel ("enable %s(%d) ep%x%s-blk max %02x\n",
            _ep->name,ep->ep_num, tmp, desc->bEndpointAddress & USB_DIR_IN ? "in" : "out", max);

    spin_unlock_irqrestore (&dev->lock, flags);
    return 0;
}

/*
 * nuc970_ep_disable
 */
static int nuc970_ep_disable (struct usb_ep *_ep)
{
    struct nuc970_ep *ep = container_of(_ep, struct nuc970_ep, ep);
    unsigned long flags;

    if (!_ep || !ep->ep.desc)
        return -EINVAL;

    spin_lock_irqsave(&ep->dev->lock, flags);
    ep->ep.desc = 0;

    __raw_writel(0, controller.reg + REG_USBD_EPA_CFG+0x28*(ep->index-1));
    __raw_writel(0, controller.reg + REG_USBD_EPA_IRQ_ENB + 0x28*(ep->index-1));

    nuke (ep->dev, ep);

    __raw_writel(0, controller.reg + REG_USBD_EPA_START_ADDR+0x28*(ep->index-1));
    __raw_writel(0, controller.reg + REG_USBD_EPA_END_ADDR+0x28*(ep->index-1));

    spin_unlock_irqrestore(&ep->dev->lock, flags);
    pr_devel("%s disabled\n", _ep->name);
    return 0;
}

/*
 * nuc970_alloc_request
 */
static struct usb_request *nuc970_alloc_request (struct usb_ep *_ep, gfp_t mem_flags)
{
    struct nuc970_ep *ep;
    struct nuc970_request *req;

    ep = container_of (_ep, struct nuc970_ep, ep);
    if (!_ep)
        return 0;

    req = kmalloc (sizeof *req, mem_flags);
    if (!req)
        return 0;
    memset (req, 0, sizeof *req);
    INIT_LIST_HEAD (&req->queue);

    return &req->req;
}

/*
 * nuc970_free_request
 */
static void nuc970_free_request (struct usb_ep *_ep, struct usb_request *_req)
{
    struct nuc970_ep *ep;
    struct nuc970_request *req;

    ep = container_of (_ep, struct nuc970_ep, ep);
    if (!ep || !_req || (!ep->ep.desc && _ep->name != ep0name))
        return;

    req = container_of (_req, struct nuc970_request, req);
    list_del_init(&req->queue);

    WARN_ON (!list_empty (&req->queue));
    kfree (req);
}


/*
 *  nuc970_queue
 */
static int nuc970_queue(struct usb_ep *_ep, struct usb_request *_req, gfp_t gfp_flags)
{
    struct nuc970_request *req;
    struct nuc970_ep *ep;
    struct nuc970_udc *dev;
    unsigned long flags;

    local_irq_save(flags);

    req = container_of(_req, struct nuc970_request, req);

    if (unlikely (!_req || !_req->complete || !_req->buf || !list_empty(&req->queue)))
    {
        local_irq_restore(flags);
        return -EINVAL;
    }

    ep = container_of(_ep, struct nuc970_ep, ep);
    if (unlikely (!_ep || (!ep->ep.desc && ep->ep.name != ep0name)))
    {
        pr_err("nuc970_queue: inval 2\n");
        local_irq_restore(flags);
        return -EINVAL;
    }

    dev = ep->dev;
    if (unlikely (!dev->driver || dev->gadget.speed == USB_SPEED_UNKNOWN))
    {
        local_irq_restore(flags);
        pr_err("nuc970_queue: speed =%d\n",dev->gadget.speed);
        return -ESHUTDOWN;
    }

    /* iso is always one packet per request, that's the only way
     * we can report per-packet status.  that also helps with dma.
     */
    if (ep->ep.desc)
    { //clyu
        if (unlikely (ep->ep.desc->bmAttributes == USB_ENDPOINT_XFER_ISOC
                        && req->req.length > usb_endpoint_maxp(ep->ep.desc)))
        {
            local_irq_restore(flags);
            return -EMSGSIZE;
        }
    }

    _req->status = -EINPROGRESS;
    _req->actual = 0;

    /* pio or dma irq handler advances the queue. */
    if (likely (req != 0))
        list_add_tail(&req->queue, &ep->queue);

    if (ep->index==0)
    { //delayed status
        if (dev->setup_ret > 1000|| ((req->req.length==0)&&(dev->ep0state == EP0_OUT_DATA_PHASE)))
        {
            __raw_writel(CEP_NAK_CLEAR, controller.reg + REG_USBD_CEP_CTRL_STAT);   // clear nak so that sts stage is complete
            __raw_writel(0x402, controller.reg + REG_USBD_CEP_IRQ_ENB);     // suppkt int//enb sts completion int
            done(ep, req, 0);
        }
    }
    else if (ep->index > 0)
    {
        if (ep->ep_dir)
        { //IN
            if (!dev->usb_dma_trigger || (ep->index!=dev->usb_dma_owner))
                __raw_writel(ep->irq_enb, controller.reg + REG_USBD_EPA_IRQ_ENB + 0x28*(ep->index-1));
        }
        else
        { //OUT
            if (!dev->usb_dma_trigger || (ep->index!=dev->usb_dma_owner))
                __raw_writel(ep->irq_enb, controller.reg + REG_USBD_EPA_IRQ_ENB + 0x28*(ep->index-1));
        }
    }

    local_irq_restore(flags);
    return 0;
}

/*
 *  nuc970_dequeue
 */
static int nuc970_dequeue (struct usb_ep *_ep, struct usb_request *_req)
{
    struct nuc970_ep *ep;
    struct nuc970_udc *udc = &controller;
    int retval = -EINVAL;
    unsigned long flags;
    struct nuc970_request *req;

    if (!udc->driver)
        return -ESHUTDOWN;

    if (!_ep || !_req)
        return retval;
    ep = container_of (_ep, struct nuc970_ep, ep);
    udc = ep->dev;

    spin_lock_irqsave (&udc->lock, flags);
    list_for_each_entry(req, &ep->queue, queue)
    {
        if (&req->req == _req)
        {
            list_del_init (&req->queue);
            _req->status = -ECONNRESET;
            retval = 0;
            break;
        }
    }
    spin_unlock_irqrestore (&udc->lock, flags);
    pr_devel("dequeue: %d, req %p\n", retval,  &req->req);
    if (retval == 0)
    {
        pr_devel( "dequeued req %p from %s, len %d buf %p\n", req, _ep->name, _req->length, _req->buf);
        _req->complete (_ep, _req);
        done(ep, req, -ECONNRESET);
    }

    return retval;
}


/*
 * nuc970_set_halt
 */
static int nuc970_set_halt (struct usb_ep *_ep, int value)
{
    pr_devel("set halt 0x%x\n", __raw_readl(controller.reg + REG_USBD_PHY_CTL));
    return 0;
}


static const struct usb_ep_ops nuc970_ep_ops =
{
    .enable         = nuc970_ep_enable,
    .disable        = nuc970_ep_disable,

    .alloc_request  = nuc970_alloc_request,
    .free_request   = nuc970_free_request,

    .queue          = nuc970_queue,
    .dequeue        = nuc970_dequeue,

    .set_halt       = nuc970_set_halt,
};

/*
 *  nuc970_g_get_frame
 */
static int nuc970_g_get_frame (struct usb_gadget *_gadget)
{
    int tmp;
    tmp = __raw_readl(controller.reg + REG_USBD_FRAME_CNT);
    return tmp & 0xffff;
}

/*
 *  nuc970_wakeup
 */
static int nuc970_wakeup (struct usb_gadget *_gadget)
{
    return 0;
}

/*
 *  nuc970_set_selfpowered
 */
static int nuc970_set_selfpowered (struct usb_gadget *_gadget, int value)
{
    return 0;
}

static int nuc970_udc_start(struct usb_gadget *g, struct usb_gadget_driver *driver);
static int nuc970_udc_stop(struct usb_gadget *g, struct usb_gadget_driver *driver);

static const struct usb_gadget_ops nuc970_ops =
{
    .get_frame          = nuc970_g_get_frame,
    .wakeup             = nuc970_wakeup,
    .set_selfpowered    = nuc970_set_selfpowered,
    .udc_start          = nuc970_udc_start,
    .udc_stop           = nuc970_udc_stop,
};


static void nuc970_udc_enable(struct nuc970_udc *dev)
{
    dev->gadget.speed = USB_SPEED_HIGH;
    __raw_writel(__raw_readl(controller.reg + REG_USBD_PHY_CTL) | 0x100, controller.reg + REG_USBD_PHY_CTL);
}

static void nuc970_udc_disable(struct nuc970_udc *dev)
{
    __raw_writel(__raw_readl(controller.reg + REG_USBD_PHY_CTL) & ~0x100, controller.reg + REG_USBD_PHY_CTL);
    dev->gadget.speed = USB_SPEED_UNKNOWN;
}

/*
 *  nuc970_udc_start
 */
static int nuc970_udc_start(struct usb_gadget *g, struct usb_gadget_driver *driver)
{
    struct nuc970_udc *udc = to_nuc970_udc(g);

    pr_devel("nuc970_udc_start() '%s'\n", driver->driver.name);

    udc->gadget.name = gadget_name;
    udc->gadget.ops = &nuc970_ops;
    udc->gadget.max_speed = USB_SPEED_HIGH;
    udc->driver = driver;

    udc->usb_devstate=0;
    udc->usb_address = 0;
    /*
     * configure USB controller
     */
    __raw_writel(0x03, controller.reg + REG_USBD_IRQ_ENB_L);    /* enable usb, cep interrupt */
    __raw_writel((USB_RESUME | USB_RST_STS | USB_FLT_DET), controller.reg + REG_USBD_IRQ_ENB);
    __raw_writel(0, controller.reg + REG_USBD_ADDR);
    __raw_writel((CEP_SUPPKT | CEP_STS_END), controller.reg + REG_USBD_CEP_IRQ_ENB);


    nuc970_udc_enable(udc);
    return 0;
}

/*
 *  nuc970_udc_stop
 */
static int nuc970_udc_stop(struct usb_gadget *g, struct usb_gadget_driver *driver)
{
    struct nuc970_udc *udc = to_nuc970_udc(g);
    unsigned int volatile i;

    udc->driver = 0;

    pr_devel("device_release_driver\n");

    /* clear/disable all interrupts */
    __raw_writel(0, controller.reg + REG_USBD_IRQ_ENB);
    __raw_writel(0xffff, controller.reg + REG_USBD_IRQ_STAT);

    __raw_writel(0, controller.reg + REG_USBD_CEP_IRQ_ENB);
    __raw_writel(0xffff, controller.reg + REG_USBD_CEP_IRQ_STAT);

    for (i = 0; i < NUC970_ENDPOINTS-1; i++)
    { //6 endpoints
        __raw_writel(0, controller.reg + REG_USBD_EPA_IRQ_ENB + 0x28 * i);
        __raw_writel(0xffff, controller.reg + REG_USBD_EPA_IRQ_STAT + 0x28 * i);
    }

    nuc970_udc_disable(udc);
    return 0;
}

static void udc_isr_rst(struct nuc970_udc *dev)
{
    int i;

    dev->usb_devstate = 0;
    dev->usb_address = 0;
    dev->usb_less_mps = 0;

    //reset DMA
    __raw_writel(0x80, controller.reg + REG_USBD_DMA_CTRL_STS);
    __raw_writel(0x00, controller.reg + REG_USBD_DMA_CTRL_STS);

    dev->usb_devstate = 1;      //default state
    pr_devel("speed:%x\n", __raw_readl(controller.reg + REG_USBD_OPER));
    if (__raw_readl(controller.reg + REG_USBD_OPER) & 0x04)
        dev->gadget.speed = USB_SPEED_HIGH;
    else
        dev->gadget.speed = USB_SPEED_FULL;


    __raw_writel(__raw_readl(controller.reg + REG_USBD_CEP_CTRL_STAT)|CEP_FLUSH,
                 controller.reg + REG_USBD_CEP_CTRL_STAT);// flush fifo
    for (i = 1; i < NUC970_ENDPOINTS; i++)
        __raw_writel(0x09, controller.reg + REG_USBD_EPA_RSP_SC + 0x28*(i-1)); // flush fifo

    __raw_writel(0, controller.reg + REG_USBD_ADDR);
    __raw_writel(0x002, controller.reg + REG_USBD_CEP_IRQ_ENB);
}

static void udc_isr_dma(struct nuc970_udc *dev)
{
    struct nuc970_request *req;
    struct nuc970_ep *ep;
    u32 datacnt_reg;

    if (!dev->usb_dma_trigger)
    {
        pr_devel("DMA not trigger, intr?\n");
        return;
    }

    ep = &dev->ep[dev->usb_dma_owner];
    datacnt_reg = (u32)(REG_USBD_EPA_DATA_CNT+0x28*(ep->index-1));

    if (dev->usb_dma_dir == Ep_In)
        __raw_writel(0x40, controller.reg + REG_USBD_EPA_IRQ_STAT + 0x28*(ep->index-1));

    dev->usb_dma_trigger = 0;
    if (list_empty(&ep->queue))
    {
        pr_devel("DMA ep->queue is empty\n");
        req = 0;
        __raw_writel(dev->irq_enbl, controller.reg + REG_USBD_IRQ_ENB_L);
        return;
    }
    else
    {
        req = list_entry(ep->queue.next, struct nuc970_request, queue);
    }

    if (req)
    {
        if (ep->ep_type == EP_TYPE_BLK)
        {
            if (dev->usb_less_mps == 1)
            {
                __raw_writel((__raw_readl(controller.reg + REG_USBD_EPA_RSP_SC+0x28*(ep->index-1))&0xF7)|0x40,
                             controller.reg + REG_USBD_EPA_RSP_SC+0x28*(ep->index-1)); // packet end
                dev->usb_less_mps = 0;
            }
        }
        else if (ep->ep_type == EP_TYPE_INT)
        {
            __raw_writel(dev->usb_dma_cnt, controller.reg + REG_USBD_EPA_TRF_CNT+0x28*(ep->index-1));
        }
        req->req.actual += dev->usb_dma_cnt;
        if ((req->req.length == req->req.actual) || dev->usb_dma_cnt < ep->ep.maxpacket)
        {
            __raw_writel(dev->irq_enbl, controller.reg + REG_USBD_IRQ_ENB_L);
            if ((ep->ep_type == EP_TYPE_BLK) && (ep->ep_dir == 0) && dev->usb_dma_cnt < ep->ep.maxpacket)
            {
                if (ep->buffer_disabled)
                {
                    __raw_writel((__raw_readl(controller.reg + REG_USBD_EPA_RSP_SC + 0x28*(ep->index-1)))&0x77,
                                 controller.reg + REG_USBD_EPA_RSP_SC + 0x28*(ep->index-1));//enable buffer
                    __raw_writel((__raw_readl(controller.reg + REG_USBD_EPA_RSP_SC+0x28*(ep->index-1))&0xF7)|0x80,
                                 controller.reg + REG_USBD_EPA_RSP_SC+0x28*(ep->index-1));//disable buffer when short packet
                }
            }
            done(ep, req, 0);

            return;
        }
    }

    if (dev->usb_dma_dir == Ep_Out)
    {
        if (dev->usb_dma_trigger_next)
        {
            dev->usb_dma_trigger_next = 0;
            pr_devel("dma out\n");
            read_fifo(ep, req, 0);
        }
    }
    else if (dev->usb_dma_dir == Ep_In)
    {
        if (dev->usb_less_mps == 1)
            dev->usb_less_mps = 0;
        if (dev->usb_dma_trigger_next)
        {
            dev->usb_dma_trigger_next = 0;
            pr_devel("dma in\n");
            write_fifo(ep, req);
        }
    }
}


static void udc_isr_ctrl_pkt(struct nuc970_udc *dev)
{
    u32 temp;
    u32 ReqErr=0;
    struct nuc970_ep *ep = &dev->ep[0];
    struct usb_ctrlrequest crq;
    struct nuc970_request *req;
    int ret;

    if (list_empty(&ep->queue))
    {
        pr_devel("ctrl ep->queue is empty\n");
        req = 0;
    }
    else
    {
        req = list_entry(ep->queue.next, struct nuc970_request, queue);
        //pr_devel("req = %x\n", req);
    }

    temp = __raw_readl(controller.reg + REG_USBD_SETUP1_0);
    Get_SetupPacket(&crq,temp);
    dev->crq = crq;

    switch (dev->ep0state)
    {
        case EP0_IDLE:
            switch (crq.bRequest)
            {
                case USBR_SET_ADDRESS:
                    ReqErr = ((crq.bRequestType == 0) && ((crq.wValue & 0xff00) == 0)
                              && (crq.wIndex == 0) && (crq.wLength == 0)) ? 0 : 1;

                    if ((crq.wValue & 0xffff) > 0x7f)
                    { //within 7f
                        ReqErr=1;   //Devaddr > 127
                    }
                    if (dev->usb_devstate == 3)
                    {
                        ReqErr=1;   //Dev is configured
                    }
                    if (ReqErr==1)
                    {
                        break;      //break this switch loop
                    }
                    if (dev->usb_devstate == 2)
                    {
                        if (crq.wValue == 0)
                            dev->usb_devstate = 1;      //enter default state
                        dev->usb_address = crq.wValue;  //if wval !=0,use new address
                    }
                    if (dev->usb_devstate == 1)
                    {
                        if (crq.wValue != 0)
                        {
                            dev->usb_address = crq.wValue;
                            dev->usb_devstate = 2;
                        }
                    }
                    break;

                case USBR_SET_CONFIGURATION:
                    ReqErr = ((crq.bRequestType == 0) && ((crq.wValue & 0xff00) == 0) &&
                              ((crq.wValue & 0x80) == 0) && (crq.wIndex == 0) &&
                              (crq.wLength == 0)) ? 0 : 1;

                    if (dev->usb_devstate == 1)
                        ReqErr=1;
                    if (ReqErr==1)
                            break;  //break this switch loop

                    if (crq.wValue == 0)
                        dev->usb_devstate = 2;
                    else
                        dev->usb_devstate = 3;
                    break;

                case USBR_SET_INTERFACE:
                    ReqErr = ((crq.bRequestType == 0x1) && ((crq.wValue & 0xff80) == 0)
                              && ((crq.wIndex & 0xfff0) == 0) && (crq.wLength == 0)) ? 0 : 1;

                    if (!((dev->usb_devstate == 0x3) && (crq.wIndex == 0x0) && (crq.wValue == 0x0)))
                        ReqErr=1;
                    if (ReqErr == 1)
                        break;  //break this switch loop

                default:
                    ;
            }//switch end

            if (crq.bRequestType & USB_DIR_IN)
            {
                dev->ep0state = EP0_IN_DATA_PHASE;
                __raw_writel(0x08, controller.reg + REG_USBD_CEP_IRQ_ENB);
            }
            else
            {
                dev->ep0state = EP0_OUT_DATA_PHASE;
                __raw_writel(0x40, controller.reg + REG_USBD_CEP_IRQ_ENB);
            }
            ret = dev->driver->setup(&dev->gadget, &crq);
            dev->setup_ret = ret;
            if (ret < 0)
            {
                __raw_writel(0x400, controller.reg + REG_USBD_CEP_IRQ_STAT);
                __raw_writel(0x448, controller.reg + REG_USBD_CEP_IRQ_ENB);     // enable in/RxED/status complete interrupt
                __raw_writel(CEP_NAK_CLEAR, controller.reg + REG_USBD_CEP_CTRL_STAT);   //clear nak so that sts stage is complete
            }
            else if (ret > 1000)
            { //DELAYED_STATUS
                pr_devel("DELAYED_STATUS:%p\n", req);
                dev->ep0state = EP0_END_XFER;
                __raw_writel(0, controller.reg + REG_USBD_CEP_IRQ_ENB);
            }
            break;

        case EP0_STALL:
            break;
        default:
            ;
    }
    if (ReqErr == 1)
    {
        __raw_writel(CEP_SEND_STALL, controller.reg + REG_USBD_CEP_CTRL_STAT);
        dev->ep0state = EP0_STALL;
    }
}

void udc_isr_update_dev(struct nuc970_udc *dev)
{
    struct usb_ctrlrequest *pcrq = &dev->crq;

    //update this device for set requests
    switch (pcrq->bRequest)
    {
        case USBR_SET_ADDRESS:
            __raw_writel(dev->usb_address, controller.reg + REG_USBD_ADDR);
            break;

        case USBR_SET_CONFIGURATION:
            break;

        case USBR_SET_INTERFACE:
            break;

        case USBR_SET_FEATURE:
            break;

        case USBR_CLEAR_FEATURE:
            break;

        default:
            ;
    }//switch end
    return;
}


static void USB_Init(struct nuc970_udc *dev)
{
    int i, j;

    dev->usb_devstate=0;
    dev->usb_address = 0;
    /*
     * configure USB controller
     */
    __raw_writel(0x03, controller.reg + REG_USBD_IRQ_ENB_L);    /* enable usb, cep interrupt */
    __raw_writel((USB_RESUME | USB_RST_STS | USB_FLT_DET), controller.reg + REG_USBD_IRQ_ENB);

    __raw_writel(USB_HS, controller.reg + REG_USBD_OPER);//USB 2.0

    __raw_writel(0, controller.reg + REG_USBD_ADDR);
    __raw_writel((CEP_SUPPKT | CEP_STS_END), controller.reg + REG_USBD_CEP_IRQ_ENB);

    for (j = 0; j < NUC970_ENDPOINTS; j++)
    {
        dev->ep[j].ep_num = 0xff;
        dev->ep[j].ep_dir = 0xff;
        dev->ep[j].ep_type = 0xff;
    }

    /* setup endpoint information */
    INIT_LIST_HEAD (&dev->gadget.ep_list);
    for (i = 0; i < NUC970_ENDPOINTS; i++)
    {
        struct nuc970_ep *ep = &dev->ep[i];

        if (!ep_name[i])
            break;
        ep->index = i;
        ep->ep.name = ep_name[i];
        ep->ep.ops = &nuc970_ep_ops;
        list_add_tail (&ep->ep.ep_list, &dev->gadget.ep_list);

        /* maxpacket differs between ep0 and others ep */
        if (!i)
        {
            ep->ep_num = 0;
            ep->ep.maxpacket = EP0_FIFO_SIZE;
            __raw_writel(0x00000000, controller.reg + REG_USBD_CEP_START_ADDR);
            __raw_writel(0x0000003f, controller.reg + REG_USBD_CEP_END_ADDR);
        }
        else
        {
            ep->ep.maxpacket = EP_FIFO_SIZE;
            __raw_writel(0, controller.reg + REG_USBD_EPA_START_ADDR+0x28*(ep->index-1));
            __raw_writel(0, controller.reg + REG_USBD_EPA_END_ADDR+0x28*(ep->index-1));
        }
        ep->dev = dev;
        ep->ep.desc = 0;
        INIT_LIST_HEAD (&ep->queue);
    }
    dev->gadget.ep0 = &dev->ep[0].ep;
    list_del_init (&dev->ep[0].ep.ep_list);
}

static u32 udc_transfer(struct nuc970_ep *ep, u8* buf, size_t size, u32 mode)
{
    struct nuc970_udc *dev = ep->dev;
    unsigned int volatile count=0;
    int volatile loop,len=0;

    loop = size / USBD_DMA_LEN;
    if (mode == DMA_WRITE)
    {
        while (!(__raw_readl(controller.reg + REG_USBD_EPA_IRQ_STAT + (0x28* (ep->index-1))) & 0x02));
        {
            dev->usb_dma_dir = Ep_In;
            dev->usb_less_mps = 0;
            __raw_writel(0x03, controller.reg + REG_USBD_IRQ_ENB_L);
            __raw_writel((__raw_readl(controller.reg + REG_USBD_DMA_CTRL_STS)&0xe0) | 0x10 | ep->ep_num,
                         controller.reg + REG_USBD_DMA_CTRL_STS);// bulk in, write
            __raw_writel(0, controller.reg + REG_USBD_EPA_IRQ_ENB + (0x28* (ep->index-1)));

            if (loop > 0)
            {
                loop--;
                if (loop > 0)
                    dev->usb_dma_trigger_next = 1;
                start_write(ep, buf, USBD_DMA_LEN);
            }
            else
            {
                if (size >= ep->ep.maxpacket)
                {
                    count = size/ep->ep.maxpacket;
                    count *= ep->ep.maxpacket;

                    if (count < size)
                        dev->usb_dma_trigger_next = 1;
                    start_write(ep, buf, count);
                }
                else
                {
                    if (ep->ep_type == EP_TYPE_BLK)
                        dev->usb_less_mps = 1;
                    start_write(ep, buf, size);
                }
            }
        }
    }
    else if (mode == DMA_READ)
    {
        dev->usb_dma_dir = Ep_Out;
        dev->usb_less_mps = 0;
        __raw_writel(0x03, controller.reg + REG_USBD_IRQ_ENB_L);
        __raw_writel((__raw_readl(controller.reg + REG_USBD_DMA_CTRL_STS) & 0xe0)|ep->ep_num,
                     controller.reg + REG_USBD_DMA_CTRL_STS);   //read
        __raw_writel(0x1000, controller.reg + REG_USBD_EPA_IRQ_ENB + (0x28* (ep->index-1)));
        __raw_writel(__raw_readl(controller.reg + REG_USBD_IRQ_ENB_L)|(ep->index<<2),
                     controller.reg + REG_USBD_IRQ_ENB_L);

        if (loop > 0)
        {
            loop--;
            if (loop > 0)
                dev->usb_dma_trigger_next = 1;
            start_read(ep, buf, USBD_DMA_LEN);
        }
        else
        {
            if (size >= ep->ep.maxpacket)
            {
                count = size/ep->ep.maxpacket;
                count *= ep->ep.maxpacket;
                if (count < size)
                    dev->usb_dma_trigger_next = 1;
                start_read(ep, buf, count);
            }
            else
            {
                //using short packet intr to deal with
                start_read(ep, buf, size);
            }
        }
    }
    return len;
}

/*
 *  probe - binds to the platform device
 */
static int nuc970_udc_probe(struct platform_device *pdev)
{
    struct nuc970_udc *udc = &controller;
    struct device *dev = &pdev->dev;
    struct clk *clk;
    int error;

    pr_devel("nuc970_udc_probe...\n");
    dev_dbg(dev, "%s()\n", __func__);

/*****************************************************************/
	clk = clk_get(NULL, "gpio");
        if (IS_ERR(clk)) {
		pr_devel(KERN_ERR "nuc970-gpio:failed to get gpio clock source\n");
		error = PTR_ERR(clk);
		return error;
	}

	clk_prepare(clk);
	clk_enable(clk);

	__raw_writel(0x8000, REG_GPIOE_PUEN);
	__raw_writel(0x4000, REG_GPIOE_PDEN);
	__raw_writel(__raw_readl(REG_GPIOE_DIR) | 0xC000, REG_GPIOE_DIR);
	__raw_writel(__raw_readl(REG_GPIOE_DATAOUT) | 0x8000, REG_GPIOE_DATAOUT);

/*****************************************************************/
    udc->pdev = pdev;

    udc->clk = clk_get(NULL, "usbd_hclk");
    if (IS_ERR(udc->clk))
    {
        error = -ENODEV;
        dev_dbg(&pdev->dev, "no udc_clk?\n");
        goto fail1;
    }

    clk_prepare(udc->clk);
    clk_enable(udc->clk);       /* Enable the peripheral clock */

    udc->res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
    if (udc->res == NULL)
    {
        dev_err(dev, "failed to get I/O memory\n");
        error = -ENXIO;
        goto fail1;
    }

    if (!request_mem_region(udc->res->start, resource_size(udc->res), pdev->name))
    {
        dev_err(dev, "failed to request I/O memory\n");
        error = -EBUSY;
        goto fail1;
    }

    udc->reg = ioremap(udc->res->start, resource_size(udc->res));
    if (udc->reg == NULL)
    {
        dev_err(dev, "failed to remap I/O memory\n");
        error = -ENXIO;
        goto fail1;
    }

    udc->gadget.dev.parent = dev;
    platform_set_drvdata (pdev, udc);

    spin_lock_init (&udc->lock);

    __raw_writel(__raw_readl(controller.reg + REG_USBD_PHY_CTL) | 0x200, controller.reg + REG_USBD_PHY_CTL);
    // FIXME: is it possible to loop forever?
    while (1)
    {
        __raw_writel(0x20, controller.reg + REG_USBD_EPA_MPS);
        if (__raw_readl(controller.reg + REG_USBD_EPA_MPS) == 0x20)
            break;
    }

    /* initial gadget structure */
    udc->gadget.ops = &nuc970_ops;
    udc->gadget.speed = USB_SPEED_UNKNOWN;
    udc->gadget.max_speed = USB_SPEED_HIGH;//USB_SPEED_FULL;
    udc->ep0state = EP0_IDLE;
    udc->gadget.name = dev_name(dev);

    USB_Init(udc);

    udc->irq = platform_get_irq(pdev, 0);
    if (udc->irq < 0)
    {
        dev_err(dev, "Failed to get irq\n");
        error = -ENXIO;
        goto fail2;
    }
    error = request_irq(udc->irq, nuc970_udc_irq, IRQF_DISABLED, gadget_name, udc);
    if (error != 0)
    {
        dev_err(dev, "request_irq() failed\n");
        goto fail2;
    }
    error = usb_add_gadget_udc(dev, &udc->gadget);
    if (error)
        goto fail3;

    pr_devel("nuc970_udc_probe done.\n");
    return 0;
fail3:
    free_irq(udc->irq, udc);
fail2:
    iounmap(udc->reg);
fail1:
    return error;
}

/*
 *  nuc970_udc_remove
 */
static int nuc970_udc_remove(struct platform_device *pdev)
{
    struct nuc970_udc *udc = platform_get_drvdata (pdev);

    dev_dbg(&pdev->dev, "%s()\n", __func__);

    usb_del_gadget_udc(&udc->gadget);
    free_irq(udc->irq, udc);
    iounmap(udc->reg);

    __raw_writel(__raw_readl(controller.reg + REG_USBD_PHY_CTL) & ~0x200,
                 controller.reg + REG_USBD_PHY_CTL);    // phy suspend
    clk_disable(udc->clk);

    return 0;
}

#ifdef CONFIG_PM
static int nuc970_udc_suspend (struct platform_device *pdev, pm_message_t state)
{
    // TODO:
    return 0;
}

static int nuc970_udc_resume (struct platform_device *pdev)
{
    // TODO:
    return 0;
}
#else
#define nuc970_udc_suspend     NULL
#define nuc970_udc_resume      NULL
#endif

static struct platform_driver nuc970_udc_driver =
{
    .probe      = nuc970_udc_probe,
    .remove     = nuc970_udc_remove,
    .suspend    = nuc970_udc_suspend,
    .resume     = nuc970_udc_resume,
    .driver     = {
            .owner  = THIS_MODULE,
            .name   = (char *) "nuc970-usbdev",
    },
};

//insmod g_mass_storage.ko file=/dev/mmcblk0p1 stall=0 removable=1

module_platform_driver(nuc970_udc_driver);

MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL");
