/* nuc_vdi.c
 *
 * Copyright (c) Nuvoton technology corporation
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.  
 *
 *   ref => sn9c102_core.c 
 *
 */
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/list.h>
#include <linux/slab.h>
#include <linux/clk.h>
#include <linux/module.h>
#include <linux/mm.h>
#include <linux/fs.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/vmalloc.h>
#include <linux/init.h>
#include <linux/spinlock.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/clk.h>

#include <linux/videodev2.h>
#include <media/v4l2-common.h>
#include <media/v4l2-ioctl.h>
#include <media/videobuf-dma-contig.h>
#include <media/v4l2-dev.h>

#include <media/v4l2-device.h>
#include <linux/jiffies.h>

#include <asm/io.h>

#include <mach/regs-gcr.h>
#include <mach/regs-clock.h>
#include <mach/regs-lcd.h>
#include <mach/regs-gpio.h>

#include <mach/regs-cap.h>
#include <linux/time.h>

#include "nuc970_cap.h"


/*****************************************************************************/

#define NUVOTON_VIN_MODULE_NAME    "V4L2 driver for NUVOTON "                    \
			      "Image Processor and Control Chip"
#define NUVOTON_VIN_MODULE_AUTHOR  "(C) 2013-2013 SCHung"
#define NUVOTON_VIN_AUTHOR_EMAIL   "<schung@nuvoton.com>"
#define NUVOTON_VIN_MODULE_LICENSE "GPL"
#define NUVOTON_VIN_MODULE_VERSION "1:1.10"
#define NUVOTON_VIN_MODULE_VERSION_CODE  KERNEL_VERSION(1, 1, 10)

/*****************************************************************************/

static short video_nr[] = {[0 ... NUVOTON_MAX_DEVICES-1] = -1};
module_param_array(video_nr, short, NULL, 0444);
MODULE_PARM_DESC(video_nr,
		 "\n<-1|n[,...]> Specify V4L2 minor mode number."
		 "\n -1 = use next available (default)"
		 "\n  n = use minor number n (integer >= 0)"
		 "\nYou can specify up to "
		 __MODULE_STRING(NUVOTON_MAX_DEVICES) " cameras this way."
		 "\nFor example:"
		 "\nvideo_nr=-1,2,-1 would assign minor number 2 to"
		 "\nthe second registered camera and use auto for the first"
		 "\none and for every other camera."
		 "\n");

static bool force_munmap[] = {[0 ... NUVOTON_MAX_DEVICES-1] =
			       NUVOTON_FORCE_MUNMAP};
module_param_array(force_munmap, bool, NULL, 0444);
MODULE_PARM_DESC(force_munmap,
		 "\n<0|1[,...]> Force the application to unmap previously"
		 "\nmapped buffer memory before calling any VIDIOC_S_CROP or"
		 "\nVIDIOC_S_FMT ioctl's. Not all the applications support"
		 "\nthis feature. This parameter is specific for each"
		 "\ndetected camera."
		 "\n 0 = do not force memory unmapping"
		 "\n 1 = force memory unmapping (save memory)"
		 "\nDefault value is "__MODULE_STRING(NUVOTON_FORCE_MUNMAP)"."
		 "\n");

static unsigned int frame_timeout[] = {[0 ... NUVOTON_MAX_DEVICES-1] =
				       NUVOTON_FRAME_TIMEOUT};
module_param_array(frame_timeout, uint, NULL, 0644);
MODULE_PARM_DESC(frame_timeout,
		 "\n<n[,...]> Timeout for a video frame in seconds."
		 "\nThis parameter is specific for each detected camera."
		 "\nDefault value is "__MODULE_STRING(NUVOTON_FRAME_TIMEOUT)"."
		 "\n");
		 

/* Declare static vars that will be used as parameters */
//static u32 vid_limit	=  2;	
/* Video memory limit, in Mb */
/* create an initial linked list: avid_vdo_devlist */
//static LIST_HEAD(avid_vdo_devlist);
LIST_HEAD(nuvoton_vdo_devlist);

/* ---- IOCTL vidioc handling  ----*/
static int nuvoton_vidioc_querycap(struct file* file,void* priv,struct v4l2_capability *cap)
{
	struct nuvoton_vin_device* cam=priv;
	ENTRY();	
	strlcpy(cap->driver,"nuvoton_vin",sizeof(cap->driver));
	cap->version = NUVOTON_VIN_MODULE_VERSION_CODE;
	cap->capabilities = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_READWRITE | V4L2_CAP_STREAMING;
	strlcpy(cap->card, cam->v4ldev->name, sizeof(cap->card));	
	LEAVE();
	return 0;
}

#if 0
/* set the parameters of frame rate control of capture DMA of NUVOTON */
static int nuvoton_vidioc_s_parm(struct file* file,void* priv,struct v4l2_streamparm *parm){return 0;}

/* get the frame rate parameters */
static int nuvoton_vidioc_g_parm(struct file* file,void* priv,struct v4l2_streamparm *parm)
{
	ENTRY();
	LEAVE();
	return 0;	
}	
#endif

/* enum all supported formats of specific device */
#if 0
static int nuvoton_vidioc_enum_fmt(struct file *file,void  *priv,struct v4l2_fmtdesc *fmtd)
{
	ENTRY();
	if (fmtd->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;
		
	if (fmtd->index == 0) {
		strcpy(fmtd->description, "JPEG");
		fmtd->pixelformat = V4L2_PIX_FMT_JPEG;
		fmtd->flags = V4L2_FMT_FLAG_COMPRESSED;
	} else
		return -EINVAL;		

	fmtd->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	memset(&fmtd->reserved, 0, sizeof(fmtd->reserved));		
	LEAVE();
	return 0;	
}
#endif 

#if 0
/* get the parameters of frame, width, height, field, type, bytesperline, sizeimage */
static int nuvoton_vidioc_g_fmt(struct file *file,void *priv,struct v4l2_format *foramt)
{
	struct nuvoton_vin_device* cam=priv;
	struct v4l2_pix_format* pfmt = &(cam->sensor.pix_format);
	ENTRY();
	if (format->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;

	pfmt->bytesperline = 0;
	pfmt->sizeimage = pfmt->height * ((pfmt->width*pfmt->priv)/8);
	pfmt->field = V4L2_FIELD_NONE;
	memcpy(&(format->fmt.pix), pfmt, sizeof(*pfmt));		
	LEAVE();
	return 0;	
}	
#endif

/* enum all supported formats of specific device */
static int nuvoton_vidioc_enum_fmt_vid_cap(struct file *file,void  *priv,struct v4l2_fmtdesc *f)
{
	ENTRY();
	LEAVE();
	return 0;	
}

/* get the parameters of frame, width, height, field, type, bytesperline, sizeimage */
static int nuvoton_vidioc_g_fmt_vid_cap(struct file *file,void *priv,struct v4l2_format *format)
{
	struct nuvoton_vin_device* cam=priv;
	struct v4l2_pix_format* pfmt = &(cam->sensor.pix_format);
	ENTRY();
	pfmt->bytesperline = 0;
	pfmt->sizeimage = pfmt->height * ((pfmt->width*pfmt->priv)/8);
	pfmt->field = V4L2_FIELD_NONE;
	memcpy(&(format->fmt.pix), pfmt, sizeof(*pfmt));		
	LEAVE();
	return 0;	
}	

unsigned short GCD(unsigned short m1, unsigned short m2)
{
	unsigned short m;
	if(m1<m2)
	{
		m=m1; m1=m2; m2=m;
	}
	if(m1%m2==0)
		return m2;
	else
		return (GCD(m2,m1%m2));		
}

/* This ioctl is similar to vidioc_s_fmt_vid_cap(). However, this ioctl is used to query the formats
that the device supports without changing any state of the device. */
static int nuvoton_vidioc_try_fmt_vid_cap(struct file *file, void *priv,struct v4l2_format *format)
{   
	struct nuvoton_vin_device* cam=priv;
	struct nuvoton_vin_sensor* s = &cam->sensor;	
	struct v4l2_pix_format* pix;
	struct v4l2_pix_format* pfmt = &(s->pix_format);
	struct v4l2_rect* bounds = &(s->cropcap.bounds);
	struct v4l2_rect rect;	
	u32 u32GCD;	
	u32 heightM,heightN,widthM,widthN;
	u32 outfmt;
	//const u8 stream = cam->stream;
	//const u32 nbuffers = cam->nbuffers;
	//u32 i;
	//int err = 0;
	ENTRY();
	pix = &(format->fmt.pix);
	
	if (format->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;
	memcpy(&rect, &(s->_rect), sizeof(rect));

	#if 1 
		rect.width = pix->width;
		rect.height = pix->height;
	#else
	if (!s->set_crop) {
		pix->width = rect.width;
		pix->height = rect.height;
	} else {
		rect.width = pix->width;
		rect.height = pix->height;
	}
	#endif

	if (rect.width < 8)
		rect.width = 8;
	if (rect.height < 8)
		rect.height = 8;
	if (rect.width > bounds->left + bounds->width - rect.left)
		rect.width = bounds->left + bounds->width - rect.left;
	if (rect.height > bounds->top + bounds->height - rect.top)
		rect.height = bounds->top + bounds->height - rect.top;
	rect.width &= ~7L;
	rect.height &= ~7L;

	pix->width = rect.width;
	pix->height = rect.height;
	pix->priv = pfmt->priv;
	pix->colorspace = pfmt->colorspace;
	pix->bytesperline = 0;
	pix->sizeimage = pix->height * ((pix->width * pix->priv) / 8);
	pix->field = V4L2_FIELD_NONE;	
	memcpy(pfmt, pix, sizeof(*pix));
	/*Set capture format for nuvoton sensor interface */	
	__raw_writel( (__raw_readl(REG_CAP_CWS) & ~(0x0fff0fff)) | (s->cropcap.bounds.width) | (s->cropcap.bounds.height<<16),REG_CAP_CWS );
	switch(pix->pixelformat)
	{
		/* Packet YUV422 */		
		case V4L2_PIX_FMT_YUYV:
		case V4L2_PIX_FMT_RGB555:
		case V4L2_PIX_FMT_RGB565:
		case V4L2_PIX_FMT_GREY:			
			if(pix->pixelformat==V4L2_PIX_FMT_YUYV) 	
				outfmt = 0<<4;
			if(pix->pixelformat==V4L2_PIX_FMT_GREY) 	
			{
				pix->priv = 8;
				outfmt = 1<<4;			
			}
			if(pix->pixelformat==V4L2_PIX_FMT_RGB555) 	
				outfmt = 2<<4;
			if(pix->pixelformat==V4L2_PIX_FMT_RGB565) 	
				outfmt = 3<<4; //infmtord
				
			__raw_writel( (__raw_readl(REG_CAP_PAR) & ~(3<<4)) | outfmt,REG_CAP_PAR );
			__raw_writel( (__raw_readl(REG_CAP_PAR) & ~INMASK) | s->infmtord,REG_CAP_PAR );
			//VDEBUG("pix->pixelformat = V4L2_PIX_FMT_YUYV\n");
			/* Set_Cropping start position for sensor */
			__raw_writel( (__raw_readl(REG_CAP_CWSP) & ~(CAP_CWSP_CWSADDRV | CAP_CWSP_CWSADDRH)) | (s->cropstart),REG_CAP_CWSP );

			/* Packet Scaling Vertical Factor Register (LSB) */		
			VDEBUG("pix->height=%d, s->cropcap.bounds.height = %d\n",pix->height,s->cropcap.bounds.height);
			u32GCD=GCD(pix->height,s->cropcap.bounds.height);
			heightN=(pix->height/u32GCD);
			heightM=(s->cropcap.bounds.height/u32GCD);
			__raw_writel( (__raw_readl(REG_CAP_PKTSL) & ~(CAP_PKTSL_PKTSVNL | CAP_PKTSL_PKTSVML))|
					       ((heightN & 0xff)<<24|(heightM & 0xff)<<16),REG_CAP_PKTSL);

			/* Packet Scaling Vertical Factor Register (MSB) */
			__raw_writel( (__raw_readl(REG_CAP_PKTSM) & ~(CAP_PKTSM_PKTSVNH | CAP_PKTSM_PKTSVMH))|
					       ((heightN>>8)<<24|(heightM>>8)<<16),REG_CAP_PKTSM);

			/* Packet Scaling Horizontal Factor Register (LSB) */	
			VDEBUG("pix->width=%d, s->cropcap.bounds.width = %d\n",pix->width,s->cropcap.bounds.width);
			u32GCD=GCD(pix->width,s->cropcap.bounds.width);			
			widthN=(pix->width/u32GCD);
			widthM=(s->cropcap.bounds.width/u32GCD);			
			__raw_writel( (__raw_readl(REG_CAP_PKTSL) & ~(CAP_PKTSL_PKTSHNL | CAP_PKTSL_PKTSHML))|
							((widthN & 0xff)<<8 | (widthM & 0xff)<<0),REG_CAP_PKTSL);
			
			/* Packet Scaling Horizontal Factor Register (MSB) */
			__raw_writel( (__raw_readl(REG_CAP_PKTSM) & ~(CAP_PKTSM_PKTSHNH | CAP_PKTSM_PKTSHMH))|
					       ((widthN>>8)<<8 | (widthM>>8)<<0),REG_CAP_PKTSM);
						
			/* Frame Output Pixel Stride Width Register(Packet/Planar) */
			__raw_writel( (__raw_readl(REG_CAP_STRIDE)& ~CAP_STRIDE_PKTSTRIDE) | 
					       (/*PacketStride*/pix->width<<0),REG_CAP_STRIDE);
			
			cam->vpe.format=pix->pixelformat;
			cam->vpe.PacketWidth=pix->width;
			cam->vpe.PacketHeight=pix->height;
			cam->vpe.PacketEnable=1;
			cam->vpe.EngineEnable=1;
		break;	

		/* Planar YUV422 */
		case V4L2_PIX_FMT_YUV422P:
		case V4L2_PIX_FMT_YUV411P:
			if(pix->pixelformat==V4L2_PIX_FMT_YUV422P) 	outfmt = 0<<7;
			if(pix->pixelformat==V4L2_PIX_FMT_YUV411P) 	outfmt = 1<<7;	
			__raw_writel( (__raw_readl(REG_CAP_PAR) & ~(1<<7)) | outfmt,REG_CAP_PAR );
			//VDEBUG("pix->pixelformat = V4L2_PIX_FMT_YUV422P\n");
			/* Set_Cropping start position for sensor */
			__raw_writel( (__raw_readl(REG_CAP_CWSP) & ~(CAP_CWSP_CWSADDRV | CAP_CWSP_CWSADDRH)) | ( 0 | 2<<16 ),REG_CAP_CWSP );

			/* Planar Scaling Vertical Factor Register (LSB) */
			u32GCD=pix->height/s->cropcap.bounds.height;
			__raw_writel( (__raw_readl(REG_CAP_PLNSL) & ~(CAP_PLNSL_PLNSVNL | CAP_PLNSL_PLNSVML))|
					       ( ((pix->height/u32GCD)&0xff)<<24 | ((s->cropcap.bounds.height/u32GCD)&0xff)<<16),REG_CAP_PLNSL);

			/* Planar Scaling Vertical Factor Register (MSB) */
			u32GCD=pix->height/s->cropcap.bounds.height;
			__raw_writel( (__raw_readl(REG_CAP_PLNSM) & ~(CAP_PLNSM_PLNSVNH | CAP_PLNSM_PLNSVMH))|
					       ( ((pix->height/u32GCD)>>8)<<24 | ((s->cropcap.bounds.height)>>8)/u32GCD<<16),REG_CAP_PLNSM);

			/* Planar Scaling Horizontal Factor Register (LSB) */
			u32GCD=pix->width/s->cropcap.bounds.width;
			__raw_writel( (__raw_readl(REG_CAP_PLNSL) & ~(CAP_PLNSL_PLNSHNL | CAP_PLNSL_PLNSHML))|
							(((pix->width/u32GCD) & 0xff)<<8 | ((s->cropcap.bounds.width/u32GCD) & 0xff)<<0),REG_CAP_PLNSL);
			
			/* Planar Scaling Horizontal Factor Register (MSB) */			
			u32GCD=pix->width/s->cropcap.bounds.width;
			__raw_writel( (__raw_readl(REG_CAP_PLNSM) & ~(CAP_PLNSM_PLNSHNH | CAP_PLNSM_PLNSHMH))|
					       (((pix->width/u32GCD) >>8)<<8 | ((s->cropcap.bounds.width/u32GCD)>>8)<<0),REG_CAP_PLNSM);

			/* Frame Output Pixel Stride Width Register(Planar) */		
			__raw_writel( (__raw_readl(REG_CAP_STRIDE)& ~CAP_STRIDE_PLNSTRIDE) | 
					      (/*PacketStride*/pix->width<<16),REG_CAP_STRIDE);
			
			cam->vpe.format=pix->pixelformat;
			cam->vpe.PlanarWidth=pix->width;
			cam->vpe.PlanarHeight=pix->height;
			cam->vpe.PlanarEnable=1;
			cam->vpe.EngineEnable=1;
		break;
		
		#if 0  /* not surpport yet, TODO: */
		case V4L2_PIX_FMT_NV16: /* 16  Y/CbCr 4:2:2  */ break;
		case V4L2_PIX_FMT_NV61: /* 16  Y/CrCb 4:2:2  */ break;
		case V4L2_PIX_FMT_YYUV: break;   
		case V4L2_PIX_FMT_YUYV: break;  
		case V4L2_PIX_FMT_YYUV: break;  
		case V4L2_PIX_FMT_YVYU: break;  
		case V4L2_PIX_FMT_UYVY: break;  
		case V4L2_PIX_FMT_VYUY: break;  

		/* packet only Y*/
		case V4L2_PIX_FMT_GREY: break;
		
		/* Packet RGB555 */
		case V4L2_PIX_FMT_RGB555: break;
		
		/* Packet RGB565 */
		case V4L2_PIX_FMT_RGB565: break;
	
		#endif
		
		default :
			return -EINVAL;
		break;

	}
		
	__raw_writel((__raw_readl(REG_CAP_PAR) & ~(VSP_HI|HSP_HI|PCLKP_HI)) | s->polarity,REG_CAP_PAR); /* set CAP Polarity */
	__raw_writel((__raw_readl(REG_CAP_INT) | 0x10000) ,REG_CAP_INT); 	   /* Enable CAP Interrupt */
	LEAVE();               
	return 0;
}


/* FIX ME: This seems to be generic enough to be at videodev2 */
/* FIX ME: 
This function sets the output image resolution of the sensor, and save the parameters. */
static int nuvoton_vidioc_s_fmt_vid_cap(struct file *file,void *priv,struct v4l2_format *format)
{
	ENTRY();
	nuvoton_vidioc_try_fmt_vid_cap(file,priv,format);
	LEAVE();
	return 0;
}

/* ask kernel to allocate needed buffer */
static int nuvoton_vidioc_reqbufs(struct file *file, void *priv,struct v4l2_requestbuffers *rb)
{
	struct nuvoton_vin_device* cam=priv;
	//struct nuvoton_vin_sensor* s = &cam->sensor;
	u32 i;
	int err;
	ENTRY();
	if (rb->type != V4L2_BUF_TYPE_VIDEO_CAPTURE || rb->memory != V4L2_MEMORY_MMAP)
		return -EINVAL;

	if (cam->io == IO_READ) {
		VDEBUG("Close and open the device again to choose the mmap I/O method\n");
		return -EBUSY;
	}		
	
	for (i = 0; i < cam->nbuffers; i++)
		if (cam->frame[i].vma_use_count) {
			VDEBUG("VIDIOC_REQBUFS failed. Previous buffers are still mapped.\n");
			return -EBUSY;
		}
		
	if (cam->stream == STREAM_ON)
		if ((err = nuvoton_vin_stream_interrupt(cam)))
			return err;		

	nuvoton_vin_empty_framequeues(cam);	

	nuvoton_vin_release_buffers(cam);
	if (rb->count)
		rb->count = nuvoton_vin_request_buffers(cam, rb->count, IO_MMAP);

	cam->io = rb->count ? IO_MMAP : IO_NONE;			
		
	LEAVE();
	return 0;
}

/* query buffer info */
static int nuvoton_vidioc_querybuf(struct file *file, void *priv, struct v4l2_buffer *p)
{
	struct nuvoton_vin_device* cam=priv;
	ENTRY();
	if (p->type != V4L2_BUF_TYPE_VIDEO_CAPTURE ||
	    p->index >= cam->nbuffers || cam->io != IO_MMAP)
	{
		LEAVE();	
		return -EINVAL;
	}

	memcpy(p,&(cam->frame[p->index].buf),sizeof(struct v4l2_buffer));

	if (cam->frame[p->index].vma_use_count)
		p->flags |= V4L2_BUF_FLAG_MAPPED;

	if (cam->frame[p->index].state == F_DONE)
		p->flags |= V4L2_BUF_FLAG_DONE;
	else if (cam->frame[p->index].state != F_UNUSED)
		p->flags |= V4L2_BUF_FLAG_QUEUED;

	LEAVE();	
	return 0;		
}

/* put the buffer into the list */
static int nuvoton_vidioc_qbuf(struct file *file, void *priv, struct v4l2_buffer *p)
{
	struct nuvoton_vin_device* cam=priv;
	struct v4l2_buffer b;
	unsigned long lock_flags;
	ENTRY();
	memcpy(&b,p,sizeof(b));

	if (b.type != V4L2_BUF_TYPE_VIDEO_CAPTURE ||
	    b.index >= cam->nbuffers || cam->io != IO_MMAP)
		return -EINVAL;

	if (cam->frame[b.index].state != F_UNUSED)
		return -EINVAL;

	cam->frame[b.index].state = F_QUEUED;

	spin_lock_irqsave(&cam->queue_lock, lock_flags);		
	list_add_tail(&cam->frame[b.index].frame, &cam->inqueue);
	spin_unlock_irqrestore(&cam->queue_lock, lock_flags);

	VDEBUG("Frame #%lu queued", (unsigned long)b.index);
	LEAVE();
	return 0;	
}

/* de-queue a buffer from the list */
static int nuvoton_vidioc_dqbuf(struct file *file, void *priv, struct v4l2_buffer *p)
{
	struct nuvoton_vin_device* cam=priv;
	struct v4l2_buffer b;
	struct nuvoton_vin_frame_t *f;
	unsigned long lock_flags;
	long timeout;
	int err = 0;
	ENTRY();

	memcpy(&b,p,sizeof(b));
	//if (b.type != V4L2_BUF_TYPE_VIDEO_CAPTURE || cam->io != IO_MMAP)
	if (b.type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;
	if (list_empty(&cam->outqueue)) {
		if (cam->stream == STREAM_OFF)
			return -EINVAL;
//		if (filp->f_flags & O_NONBLOCK)
//			return -EAGAIN;
		if (!cam->module_param.frame_timeout) {
			err = wait_event_interruptible
			      ( cam->wait_frame,
				(!list_empty(&cam->outqueue)) ||
				(cam->state & DEV_DISCONNECTED) ||
				(cam->state & DEV_MISCONFIGURED) );
			if (err)
				return err;
		} else {
			timeout = wait_event_interruptible_timeout
				  ( cam->wait_frame,
				    (!list_empty(&cam->outqueue)) ||
				    (cam->state & DEV_DISCONNECTED) ||
				    (cam->state & DEV_MISCONFIGURED),
				    cam->module_param.frame_timeout *
				    1000 * msecs_to_jiffies(1) );
			if (timeout < 0)
			{
				return timeout;
			}else if (timeout == 0 &&
				 !(cam->state & DEV_DISCONNECTED)) {
				VDEBUG("Video frame timeout elapsed\n");
				return -EIO;
			}
		}
		if (cam->state & DEV_DISCONNECTED)
			return -ENODEV;
		if (cam->state & DEV_MISCONFIGURED)
			return -EIO;
	}
	spin_lock_irqsave(&cam->queue_lock, lock_flags);
	f = list_entry(cam->outqueue.next, struct nuvoton_vin_frame_t, frame);
	list_del(cam->outqueue.next);
	spin_unlock_irqrestore(&cam->queue_lock, lock_flags);
	f->state = F_UNUSED;
	b = f->buf;
	if (f->vma_use_count)
		b.flags |= V4L2_BUF_FLAG_MAPPED;
	memcpy(p,&b,sizeof(b));
	VDEBUG("Frame #%lu dequeued", (unsigned long)f->buf.index);
		
	LEAVE();
	return 0;
}

/* start capturing the sensor output stream */
static int nuvoton_vidioc_streamon(struct file *file, void *priv, enum v4l2_buf_type i)
{
	struct nuvoton_vin_device* cam=priv;
	ENTRY();

	/* Setting Buffer address */
	VDEBUG("cam->nbuffers=%d\n",cam->nbuffers);
	VDEBUG("cam->vpe.EngineEnable=%d\n",cam->vpe.EngineEnable);
	
	if(cam->nbuffers>0)
	{
		
		VDEBUG("cam->frame_current->bufmem=0x%08x\n",cam->frame_current->bufmem);
		VDEBUG("cam->frame_current->pbuf=0x%08x\n",cam->frame_current->pbuf);
		VDEBUG("cam->vpe.PacketEnable=%d\n",cam->vpe.PacketEnable);
		if(cam->vpe.PacketEnable==1)
		{							
			if(cam->frame_current!=NULL)				
				__raw_writel(cam->frame_current->pbuf,REG_CAP_PKTBA0);
			else
				VDEBUG("PacketEnable : cam->frame_current == NULL\n");			
		}

		VDEBUG("cam->vpe.PlanarEnable=%d\n",cam->vpe.PlanarEnable);
		if(cam->vpe.PlanarEnable==1)
		{
			//if(V4L2_PIX_FMT_YUV422P)	
			{
				if(cam->frame_current!=NULL)
				{
					__raw_writel((unsigned int)cam->frame_current->pbuf,REG_CAP_YBA);
					__raw_writel(__raw_readl(REG_CAP_YBA)+(cam->vpe.PlanarWidth*cam->vpe.PlanarHeight),REG_CAP_UBA);
					__raw_writel(__raw_readl(REG_CAP_UBA)+(cam->vpe.PlanarWidth*cam->vpe.PlanarHeight)/2,REG_CAP_VBA);
				}else
					printk("PlanarEnable : cam->frame_current == NULL\n");				
			}
		}
	}
	/* Capture engine enable and packer/planar mode enable */	
	__raw_writel( (__raw_readl(REG_CAP_CTL) & ~(CAP_CTL_CAPEN | CAP_CTL_PKTEN | CAP_CTL_PLNEN))
		| (cam->vpe.EngineEnable | (cam->vpe.PacketEnable<<6) | (cam->vpe.PlanarEnable<<5) ),REG_CAP_CTL);

	//for(j=0;j<=0x88;j+=4)
	//	printk("CAP_BA+0x%2x=0x%08x\n",j,__raw_readl(CAP_BA+j));
	cam->stream = STREAM_ON;
	LEAVE();
	return 0;
}

/* stop capturing sensor output stream */
static int nuvoton_vidioc_streamoff(struct file *file, void *priv, enum v4l2_buf_type i)
{
	ENTRY();
	LEAVE();
	return 0;
}

/* setup the device support standard */
static int nuvoton_vidioc_s_std(struct file *file, void *priv, v4l2_std_id i)
{
	ENTRY();
	LEAVE();
	return 0;
}


/* get the supported standard of the device */
#if 0
static int vidioc_g_std(struct file *file, void *priv, v4l2_std_id *i){return 0;}
#endif

/* only one input in this sample driver */
/* Enumerate the input. AVID Daytona supports several input devices
* 1. image sensor 
* 2. native image post-processor 
* 3. analog TV tuner * * */
static int nuvoton_vidioc_enum_input(struct file *file,void *priv,struct v4l2_input *inp)
{
	ENTRY();
	strcpy(inp->name, "Camera");
	inp->type = V4L2_INPUT_TYPE_CAMERA;
	LEAVE();
	return 0;
}

/* always returns 0 (we have only one input) */
static int nuvoton_vidioc_g_input(struct file *file, void *priv, unsigned int *i)	
{
	int index = 0;
	ENTRY();
	*i=index;
	LEAVE();
	return 0;
}

/* Because we have only one input, set input = 0 */
static int nuvoton_vidioc_s_input(struct file *file, void *priv, unsigned int i)
{	
	ENTRY();
	if (i != 0)
	{
		LEAVE();
		return -EINVAL;
	}
	LEAVE();
	return 0;
}

/* ----- *           controls * ----- */
/* Check if the control method is supported (via  v4l2_queryctrl)
If yes, the parameter of the control will be returned;
if not, error code will be returned. */
static int nuvoton_vidioc_queryctrl(struct file *file,void *priv,struct v4l2_queryctrl *qc)
{
	
	struct nuvoton_vin_device* cam=priv;
	struct nuvoton_vin_sensor* s = &cam->sensor;			
	ENTRY();
	
	#if 0
	for (i = 0; i < ARRAY_SIZE(s->qctrl); i++)
		if (qc->id && qc->id == s->qctrl[i].id) {
			memcpy(qc, &(s->qctrl[i]), sizeof(qc));			
			return 0;
		}
	#else
		if (qc->id && qc->id == s->qctrl.id) {
			memcpy(qc, &(s->qctrl), sizeof(qc));
			LEAVE();
			return 0;
		}
	#endif	
	LEAVE();	
	return -EINVAL;	
}

/* get the control parameter */
static int nuvoton_vidioc_g_ctrl(struct file *file, void *priv,struct v4l2_control *ctrl)
{
	struct nuvoton_vin_device* cam=priv;
	struct nuvoton_vin_sensor* s = &cam->sensor;
	int err = 0;
	ENTRY();
	if (!s->get_ctrl && !s->set_ctrl)
		return -EINVAL;

	if (!s->get_ctrl) {		
		if (ctrl->id == s->qctrl.id) {
			ctrl->value = s->_qctrl.default_value;
			goto exit;
		}
	}else
		err = s->get_ctrl(cam, ctrl);

exit:		
	LEAVE();
	return err;
}

/* set the control parameter */
static int nuvoton_vidioc_s_ctrl(struct file *file,void *priv,struct v4l2_control *ctrl)
{
	struct nuvoton_vin_device* cam=priv;
	struct nuvoton_vin_sensor* s = &cam->sensor;
	int err = 0;
	ENTRY();
	if (!s->set_ctrl)
		return -EINVAL;
		
	if (ctrl->id == s->qctrl.id) {
		if (s->qctrl.flags & V4L2_CTRL_FLAG_DISABLED)
			return -EINVAL;
		if (ctrl->value < s->qctrl.minimum || ctrl->value > s->qctrl.maximum)
			return -ERANGE;
		ctrl->value -= ctrl->value % s->qctrl.step;
	}
	
	if ((err = s->set_ctrl(cam, ctrl)))
		return err;
	s->_qctrl.default_value = ctrl->value;		
	LEAVE();
	return 0;
}

static int nuvoton_vidioc_cropcap(struct file *file, void *priv, struct v4l2_cropcap *crop)
{
	struct nuvoton_vin_device* cam=priv;	
	struct v4l2_cropcap* cc = &(cam->sensor.cropcap);
	
	cc->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	cc->pixelaspect.numerator = 1;
	cc->pixelaspect.denominator = 1;	
	memcpy(crop,cc,sizeof(struct v4l2_cropcap));
	return 0;
}

static int nuvoton_vidioc_g_crop(struct file *file, void *priv,struct v4l2_crop *crop)
{
	struct nuvoton_vin_device* cam=priv;	
	struct nuvoton_vin_sensor* s = &cam->sensor;
	ENTRY();
	crop->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	memcpy(&(crop->c), &(s->_rect), sizeof(struct v4l2_rect));
	LEAVE();	
	return 0;		
}
static int nuvoton_vidioc_s_crop(struct file *file, void *priv,const struct v4l2_crop *crop)
{
	struct nuvoton_vin_device* cam=priv;	
	struct nuvoton_vin_sensor* s = &cam->sensor;
	struct v4l2_rect* rect;
	struct v4l2_rect* bounds = &(s->cropcap.bounds);
	//const enum unvoton_vin_stream_state stream = cam->stream;
	const u8 stream = cam->stream;
	const u32 nbuffers = cam->nbuffers;
	u32 i;
	int err = 0;
	ENTRY();	
	rect = (struct v4l2_rect*)&(crop->c);
		
	if (crop->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;
		
	if (cam->module_param.force_munmap)
		for (i = 0; i < cam->nbuffers; i++)
			if (cam->frame[i].vma_use_count) {
				VDEBUG("VIDIOC_S_CROP failed. Unmap the buffers first.\n");
				return -EBUSY;
			}

	if (!s->set_crop) {
			memcpy(rect, &(s->_rect), sizeof(*rect));			
			return 0;
		}			
		
	rect->left &= ~7L;
	rect->top &= ~7L;
	if (rect->width < 8) 	rect->width = 8;
	if (rect->height < 8)	rect->height = 8;
	if (rect->width > bounds->width)		rect->width = bounds->width;
	if (rect->height > bounds->height)	rect->height = bounds->height;
	if (rect->left < bounds->left)	rect->left = bounds->left;
	if (rect->top < bounds->top)		rect->top = bounds->top;
	if (rect->left + rect->width > bounds->left + bounds->width)	rect->left = bounds->left+bounds->width - rect->width;
	if (rect->top + rect->height > bounds->top + bounds->height)	rect->top = bounds->top+bounds->height - rect->height;
	rect->width &= ~7L;
	rect->height &= ~7L;		
	
	if (cam->stream == STREAM_ON)
		if ((err = nuvoton_vin_stream_interrupt(cam)))
			return err;	
	
	
	if (cam->module_param.force_munmap || cam->io == IO_READ)
		nuvoton_vin_release_buffers(cam);
		
	if (s->set_crop)
		err += s->set_crop(cam, rect);

	if (err) { /* atomic, no rollback in ioctl() */
		cam->state |= DEV_MISCONFIGURED;
		VDEBUG("VIDIOC_S_CROP failed because of hardware problems. To use the camera, close and open %s again.\n",
						video_device_node_name(cam->v4ldev));
		return -EIO;
	}		
	
	s->pix_format.width = rect->width;
	s->pix_format.height = rect->height;
	memcpy(&(s->_rect), rect, sizeof(*rect));
	
	if ((cam->module_param.force_munmap  || cam->io == IO_READ) &&
	    nbuffers != nuvoton_vin_request_buffers(cam, nbuffers, cam->io)) {
		cam->state |= DEV_MISCONFIGURED;
		VDEBUG("VIDIOC_S_CROP failed because of not enough memory. To use the camera, close and open %s again.\n",
		    video_device_node_name(cam->v4ldev));
		return -ENOMEM;
	}
	
	if (cam->io == IO_READ)
		nuvoton_vin_empty_framequeues(cam);
	else if (cam->module_param.force_munmap)
		nuvoton_vin_requeue_outqueue(cam);

	cam->stream = stream;
	LEAVE();
	return 0;
}

static int nuvoton_vin_vidioc_s_jpegcomp(struct file *file, void *priv,const struct v4l2_jpegcompression *jc)
{
	ENTRY();
	LEAVE();
	return 0;
}

#if 0
static int vidioc_querymenu(struct file *file,void *priv,struct v4l2_querymenu *menu){return 0;}
#endif

/* ----	Initialization v4l2 ioctrl ops   ----*/
static const struct v4l2_ioctl_ops nuvoton_vdi_ioctl_ops =
{
	.vidioc_querycap	= nuvoton_vidioc_querycap,
	.vidioc_g_fmt_vid_cap	= nuvoton_vidioc_g_fmt_vid_cap,
	.vidioc_enum_fmt_vid_cap = nuvoton_vidioc_enum_fmt_vid_cap,
	.vidioc_s_fmt_vid_cap	= nuvoton_vidioc_s_fmt_vid_cap,
	.vidioc_enum_input = nuvoton_vidioc_enum_input,
	.vidioc_g_input	= nuvoton_vidioc_g_input,
	.vidioc_s_input	= nuvoton_vidioc_s_input,
	.vidioc_s_std	= nuvoton_vidioc_s_std,
	.vidioc_reqbufs	= nuvoton_vidioc_reqbufs,
	.vidioc_try_fmt_vid_cap = nuvoton_vidioc_try_fmt_vid_cap, 																	  
	.vidioc_querybuf	= nuvoton_vidioc_querybuf,
	.vidioc_qbuf	= nuvoton_vidioc_qbuf,
	.vidioc_dqbuf	= nuvoton_vidioc_dqbuf,
	.vidioc_streamon	= nuvoton_vidioc_streamon,
	.vidioc_streamoff	= nuvoton_vidioc_streamoff,
	.vidioc_queryctrl	= nuvoton_vidioc_queryctrl,
	.vidioc_g_ctrl = nuvoton_vidioc_g_ctrl,
	.vidioc_s_ctrl = nuvoton_vidioc_s_ctrl,		
	.vidioc_cropcap = nuvoton_vidioc_cropcap,	
	.vidioc_g_crop = nuvoton_vidioc_g_crop,
	.vidioc_s_crop = nuvoton_vidioc_s_crop,
	.vidioc_s_jpegcomp = nuvoton_vin_vidioc_s_jpegcomp,
};


/*****************************************************************************/
int nuvoton_vin_stream_interrupt(struct nuvoton_vin_device* cam){return 0;}

u32 nuvoton_vin_request_buffers(struct nuvoton_vin_device* cam, u32 count,enum nuvoton_vin_io_method io)
{
	struct v4l2_pix_format* p = &(cam->sensor.pix_format);	
	dma_addr_t pbuf;		
	#if 0
	struct v4l2_rect* r = &(cam->sensor.cropcap.bounds);
	const size_t imagesize = cam->module_param.force_munmap ||
				 io == IO_READ ?
				 (p->width * p->height * p->priv) / 8 :
				 (r->width * r->height * p->priv) / 8;
	#else				 				 
	const size_t imagesize = (p->width * p->height * p->priv) / 8 ;
	#endif				 
	void* buff = NULL;
	u32 i;	
	ENTRY();
	if (count > NUVOTON_MAX_FRAMES)
		count = NUVOTON_MAX_FRAMES;

	cam->nbuffers = (count);
	while (cam->nbuffers > 0) {
		if((buff = dma_alloc_writecombine(NULL, cam->nbuffers * PAGE_ALIGN(imagesize),
							&pbuf, GFP_KERNEL))!=NULL)
			break;
		cam->nbuffers--;
	}
	for (i = 0; i < cam->nbuffers; i++) {
		cam->frame[i].bufmem = buff + i*PAGE_ALIGN(imagesize);
		cam->frame[i].pbuf = pbuf + i*PAGE_ALIGN(imagesize);
		cam->frame[i].buf.index = i;
		cam->frame[i].buf.m.userptr = (unsigned long)(buff + i*PAGE_ALIGN(imagesize));
		cam->frame[i].buf.m.offset = i*PAGE_ALIGN(imagesize);
		cam->frame[i].buf.length = imagesize;
		cam->frame[i].buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		cam->frame[i].buf.sequence = 0;
		cam->frame[i].buf.field = V4L2_FIELD_NONE;
		cam->frame[i].buf.memory = V4L2_MEMORY_MMAP;
		cam->frame[i].buf.flags = 0;
		//VDEBUG("cam->frame[%d].bufmem=0x%08x\n",i,(unsigned int)cam->frame[i].bufmem);
		//VDEBUG("cam->frame[%d].pbuf=0x%08x\n",i,(unsigned int)cam->frame[i].pbuf);		
	}
	
	cam->frame_current=&cam->frame[0];	
	LEAVE();
	return (cam->nbuffers);
}


void nuvoton_vin_release_buffers(struct nuvoton_vin_device* cam)
{
	ENTRY();

	if (cam->nbuffers) {
		vfree(cam->frame[0].bufmem);
		cam->nbuffers = 0;
	}
	cam->frame_current = NULL;
	LEAVE();	
}

void nuvoton_vin_empty_framequeues(struct nuvoton_vin_device* cam)
{
	u32 i;
	ENTRY();
	INIT_LIST_HEAD(&cam->inqueue);
	INIT_LIST_HEAD(&cam->outqueue);

	for (i = 0; i < NUVOTON_MAX_FRAMES; i++) {
		cam->frame[i].state = F_UNUSED;
		cam->frame[i].buf.bytesused = 0;
	}
	LEAVE();	
}

void nuvoton_vin_requeue_outqueue(struct nuvoton_vin_device* cam)
{
	struct nuvoton_vin_frame_t *i;

	list_for_each_entry(i, &cam->outqueue, frame) {
		i->state = F_QUEUED;
		list_add(&i->frame, &cam->inqueue);
	}
	INIT_LIST_HEAD(&cam->outqueue);
}


static void nuvoton_vin_queue_unusedframes(struct nuvoton_vin_device* cam)
{
	unsigned long lock_flags;
	u32 i;

	for (i = 0; i < cam->nbuffers; i++)
		if (cam->frame[i].state == F_UNUSED) {
			cam->frame[i].state = F_QUEUED;
			spin_lock_irqsave(&cam->queue_lock, lock_flags);
			list_add_tail(&cam->frame[i].frame, &cam->inqueue);
			spin_unlock_irqrestore(&cam->queue_lock, lock_flags);
		}
}

/*****************************************************************************/

/* ----	Initialization and module stuff   ----*/
static int nuvoton_vin_init(struct nuvoton_vin_device* cam)
{
	//struct nuvoton_vin_device* s = &cam->sensor;
	//struct v4l2_control ctrl;
	//struct v4l2_queryctrl *qctrl;
	//struct v4l2_rect* rect;
	//u8 i = 0;
	//int err = 0;
	ENTRY();
	cam->state=0;
	cam->nreadbuffers = 2;
	if (!(cam->state & DEV_INITIALIZED)) {
		mutex_init(&cam->open_mutex);
		mutex_init(&cam->fileop_mutex);
		spin_lock_init(&cam->queue_lock);
		init_waitqueue_head(&cam->wait_frame);
		init_waitqueue_head(&cam->wait_stream);
		
		cam->state |= DEV_INITIALIZED;
	}

	LEAVE();	
	return 0;
}

static void nuvoton_vin_release_resources(struct kref *kref)
{
	struct nuvoton_vin_device *cam = container_of(kref, struct nuvoton_vin_device,kref);
	ENTRY();
	VDEBUG("V4L2 device %s deregistered\n",video_device_node_name(cam->v4ldev));
	video_set_drvdata(cam->v4ldev, NULL);
	video_unregister_device(cam->v4ldev);
	kfree(cam->control_buffer);
	kfree(cam);	
}

#define CLK_PMCON_SEN_OFF_ST (1<<4 )
#define CLK_HCLKEN_CAPEN 		 (1<<26)
#define CLK_HCLKEN_SENSOREN  (1<<27)
int capture_init(struct nuvoton_vin_device* cam)
{
	//u32 i32Div;
	//struct nuvoton_vin_sensor* s = &cam->sensor;

	struct clk *clk;
	#if 0	
	struct clk *clkcap,*clkaplldiv,*clkmux;
	int ret;
	#endif
	//u32 u32SensorFreq=24000000;
	ENTRY();
	

	clk = clk_get(NULL, "cap_eclk");
	if (IS_ERR(clk)) {	
		return -ENOENT;
	}
	
	clk_prepare(clk);
	clk_enable(clk);

	clk_prepare(clk_get(NULL, "cap_hclk"));
	clk_enable(clk_get(NULL, "cap_hclk"));
	
	clk_prepare(clk_get(NULL, "sensor_hclk"));
        clk_enable(clk_get(NULL, "sensor_hclk"));

#if 0
	clkmux = clk_get(NULL, "cap_eclk_mux");
        if (IS_ERR(clkmux)) {
			printk(KERN_ERR "nuc970-audio:failed to get cap clock source\n");
			ret = PTR_ERR(clkmux);
			return ret;
		}
	clkcap = clk_get(NULL, "cap_eclk");
        if (IS_ERR(clkcap)) {
			printk(KERN_ERR "nuc970-cap:failed to get cap clock source\n");
			ret = PTR_ERR(clkcap);
			return ret;
		}
	clkaplldiv = clk_get(NULL, "cap_aplldiv");
        if (IS_ERR(clkaplldiv)) {
			printk(KERN_ERR "nuc970-cap:failed to get cap clock source\n");
			ret = PTR_ERR(clkaplldiv);
			return ret;
		}		
		clk_set_parent(clkmux, clkaplldiv);
		clk_set_rate(clkcap, 24000000);
	#endif
	/* Muti-pin setting */	

	/* MFP_GPI_L : 3=SEN_CLK0, 4=SEN_PCLK, 5=SEN_HSYNC, 6=SEN_VSYNC, 7=SEN_FIFLD */
	//__raw_writel( (__raw_readl(REG_MFP_GPI_L) & ~0xFFFF000) | 0x33333000,REG_MFP_GPI_L);

	/* MFP_GPI_H : SEN_PDATA[0~7] */	
	//__raw_writel(0x33333333,REG_MFP_GPI_H);		
	
	// To-Do
	//__raw_writel((__raw_readl(REG_CLK_DIV3) & ~0x000f0000 ) | 3<<16,REG_CLK_DIV3); /* Sensor Clock Source Select UCLKOut */
	//i32Div = (66000000/u32SensorFreq)-1;	
	//if(i32Div < 0) i32Div = 0;			
	//__raw_writel((__raw_readl(REG_CLK_DIV3) & ~0x0f000000 ) | i32Div<<24,REG_CLK_DIV3);
			
	
	//printk("REG_CLK_HCLKEN=0x%08x\n",__raw_readl(REG_CLK_HCLKEN));
	//printk("REG_MFP_GPI_L=0x%08x\n",__raw_readl(REG_MFP_GPI_L));
	//printk("REG_MFP_GPI_H=0x%08x\n",__raw_readl(REG_MFP_GPI_H));
	//printk("REG_CLK_DIV3=0x%08x\n",__raw_readl(REG_CLK_DIV3));
	
	/* GPIOI7 set to high */
	__raw_writel( (__raw_readl(REG_MFP_GPI_L) & ~0xF0000000) ,REG_MFP_GPI_L);
	__raw_writel((__raw_readl(GPIO_BA+0x200) | 0x0080),(GPIO_BA+0x200)); /* GPIOI7 Output mode */
	__raw_writel((__raw_readl(GPIO_BA+0x204) | 0x0080),(GPIO_BA+0x204)); /* GPIOI7 Output to high */
	
	/* GPIOI0 set to low  */
	__raw_writel( (__raw_readl(REG_MFP_GPI_L) & ~0x0000000F) ,REG_MFP_GPI_L);
	__raw_writel((__raw_readl(GPIO_BA+0x200) | 0x0001),(GPIO_BA+0x200)); /* GPIOI0 Output mode */
	__raw_writel((__raw_readl(GPIO_BA+0x204) &~ 0x0001),(GPIO_BA+0x204)); /* GPIOI0 Output to low */	
	return 0;
}

static int nuvoton_vdi_open(struct file *filp)
{
	struct nuvoton_vin_device* cam;
	int err = 0;
	ENTRY();
	if (!down_read_trylock(&nuvoton_vin_dev_lock))
		return -EAGAIN;
	cam = video_drvdata(filp);

	if (wait_for_completion_interruptible(&cam->probe)) {
		up_read(&nuvoton_vin_dev_lock);
		return -ERESTARTSYS;
	}
	kref_get(&cam->kref);

	if (mutex_lock_interruptible(&cam->open_mutex)) {
		kref_put(&cam->kref, nuvoton_vin_release_resources);
		up_read(&nuvoton_vin_dev_lock);
		return -ERESTARTSYS;
	}
	if (cam->state & DEV_DISCONNECTED) {
		VDEBUG("Device not present\n");
		err = -ENODEV;
		goto out;
	}
	if (cam->users) {
		VDEBUG("Device %s is busy...\n",video_device_node_name(cam->v4ldev));
		VDEBUG("Simultaneous opens are not supported\n");
		if ((filp->f_flags & O_NONBLOCK) || (filp->f_flags & O_NDELAY)) {
			err = -EWOULDBLOCK;
			goto out;
		}
		VDEBUG("A blocking open() has been requested. Wait for the device to be released...\n");
		up_read(&nuvoton_vin_dev_lock);
		//err = wait_event_interruptible_exclusive(cam->wait_open,
			//			(cam->state & DEV_DISCONNECTED)
				//			 || !cam->users);
		down_read(&nuvoton_vin_dev_lock);
		if (err)
			goto out;
		if (cam->state & DEV_DISCONNECTED) {
			err = -ENODEV;
			goto out;
		}
	}
	if (cam->state & DEV_MISCONFIGURED) {
		err = nuvoton_vin_init(cam);
		if (err) {
			VDEBUG("Initialization failed again. I will retry on next open().\n");
			goto out;
		}
		cam->state &= ~DEV_MISCONFIGURED;
	}
	filp->private_data = cam;
	cam->users++;
	cam->io = IO_NONE;
	cam->stream = STREAM_OFF;
	cam->nbuffers = 0;
	cam->frame_count = 0;
	nuvoton_vin_empty_framequeues(cam);	
	VDEBUG("Video device %s is open\n",video_device_node_name(cam->v4ldev));

out:
	mutex_unlock(&cam->open_mutex);
	if (err)
		kref_put(&cam->kref, nuvoton_vin_release_resources);
	up_read(&nuvoton_vin_dev_lock);
	LEAVE();	
	return err;

}

static int nuvoton_vdi_close(struct file *filp)
{

	struct nuvoton_vin_device* cam;
	ENTRY();
	down_write(&nuvoton_vin_dev_lock);

	cam = video_drvdata(filp);

	nuvoton_vin_release_buffers(cam);	
	cam->users--;
	//printk("after cam->users=%d\n",cam->users);
	//wake_up_interruptible_nr(&cam->wait_open, 1);
	//printk("wake_up_interruptible_nr\n");
	VDEBUG("Video device %s closed", video_device_node_name(cam->v4ldev));

	kref_put(&cam->kref, nuvoton_vin_release_resources);	
	up_write(&nuvoton_vin_dev_lock);
	

	__raw_writel( __raw_readl(REG_CAP_CTL) & ~(CAP_CTL_CAPEN | CAP_CTL_PKTEN | CAP_CTL_PLNEN),REG_CAP_CTL);
	LEAVE();
	return 0;
}

static ssize_t nuvoton_vdi_read(struct file* filp, char __user * buf, size_t count, loff_t* f_pos)
{
	struct nuvoton_vin_device *cam = video_drvdata(filp);
	struct nuvoton_vin_frame_t* f, * i;
	unsigned long lock_flags;
	long timeout;
	int err = 0;	
	ENTRY();			
	if (mutex_lock_interruptible(&cam->fileop_mutex))
		return -ERESTARTSYS;

	if (cam->state & DEV_DISCONNECTED) {
		VDEBUG("Device not present");
		mutex_unlock(&cam->fileop_mutex);
		return -ENODEV;
	}

	if (cam->state & DEV_MISCONFIGURED) {
		VDEBUG("The camera is misconfigured. Close and open it again.\n");
		mutex_unlock(&cam->fileop_mutex);
		return -EIO;
	}

	if (cam->io == IO_MMAP) {
		VDEBUG("Close and open the device again to choose the read method\n");
		mutex_unlock(&cam->fileop_mutex);
		return -EBUSY;
	}	
	if (cam->io == IO_NONE) {
		if (!nuvoton_vin_request_buffers(cam, cam->nreadbuffers, IO_READ)) {
			VDEBUG("read() failed, not enough memory\n");
			mutex_unlock(&cam->fileop_mutex);
			return -ENOMEM;
		}
		cam->io = IO_READ;
		cam->stream = STREAM_ON;

		if(cam->nreadbuffers>0)
		{
			if(cam->vpe.PacketEnable==1)
			{							
				if(cam->frame_current!=NULL)				
					__raw_writel(cam->frame_current->pbuf,REG_CAP_PKTBA0);
				else
					VDEBUG("PacketEnable : cam->frame_current == NULL\n");			
			}
			if(cam->vpe.PlanarEnable==1)
			{
				//if(V4L2_PIX_FMT_YUV422P)	
				{
					if(cam->frame_current!=NULL)
					{
						__raw_writel((unsigned int)cam->frame_current->pbuf,REG_CAP_YBA);
						__raw_writel(__raw_readl(REG_CAP_YBA)+(cam->vpe.PlanarWidth*cam->vpe.PlanarHeight),REG_CAP_UBA);
						__raw_writel(__raw_readl(REG_CAP_UBA)+(cam->vpe.PlanarWidth*cam->vpe.PlanarHeight)/2,REG_CAP_VBA);
					}else
						printk("PlanarEnable : cam->frame_current == NULL\n");				
				}
			}
		}
		/* Capture engine enable and packer/planar mode enable */	
		__raw_writel( (__raw_readl(REG_CAP_CTL) & ~(CAP_CTL_CAPEN | CAP_CTL_PKTEN | CAP_CTL_PLNEN))
			| (cam->vpe.EngineEnable | (cam->vpe.PacketEnable<<6) | (cam->vpe.PlanarEnable<<5) ),REG_CAP_CTL);		
	}

	if (list_empty(&cam->inqueue)) {
		if (!list_empty(&cam->outqueue))
			nuvoton_vin_empty_framequeues(cam);
		nuvoton_vin_queue_unusedframes(cam);
	}

	if (!count) {
		mutex_unlock(&cam->fileop_mutex);
		return 0;
	}

	if (list_empty(&cam->outqueue)) {
		if (filp->f_flags & O_NONBLOCK) {
			mutex_unlock(&cam->fileop_mutex);
			return -EAGAIN;
		}
		timeout = wait_event_interruptible_timeout
			  ( cam->wait_frame,
			    (!list_empty(&cam->outqueue)) ||
			    (cam->state & DEV_DISCONNECTED) ||
			    (cam->state & DEV_MISCONFIGURED),
			    msecs_to_jiffies(
				cam->module_param.frame_timeout * 1000
			    )
			  );
		if (timeout < 0) {
			mutex_unlock(&cam->fileop_mutex);
			return timeout;
		}
		if (cam->state & DEV_DISCONNECTED) {
			mutex_unlock(&cam->fileop_mutex);
			return -ENODEV;
		}
		if (!timeout || (cam->state & DEV_MISCONFIGURED)) {
			mutex_unlock(&cam->fileop_mutex);
			return -EIO;
		}
	}

	f = list_entry(cam->outqueue.prev, struct nuvoton_vin_frame_t, frame);

	if (count > f->buf.bytesused)
		count = f->buf.bytesused;

	if (copy_to_user(buf, f->bufmem, count)) {
		err = -EFAULT;
		goto exit;
	}
	*f_pos += count;

exit:
	spin_lock_irqsave(&cam->queue_lock, lock_flags);
	list_for_each_entry(i, &cam->outqueue, frame)
		i->state = F_UNUSED;
	INIT_LIST_HEAD(&cam->outqueue);
	spin_unlock_irqrestore(&cam->queue_lock, lock_flags);

	nuvoton_vin_queue_unusedframes(cam);

	VDEBUG("Frame #%lu, bytes read: %zu\n",(unsigned long)f->buf.index, count);

	mutex_unlock(&cam->fileop_mutex);
	LEAVE();
	return err ? err : count;		
}

static void nuvoton_vin_vm_open(struct vm_area_struct* vma)
{	
	struct nuvoton_vin_frame_t* f = vma->vm_private_data;
	ENTRY();
	f->vma_use_count++;
}


static void nuvoton_vin_vm_close(struct vm_area_struct* vma)
{
	/* NOTE: buffers are not freed here */
	struct nuvoton_vin_frame_t* f = vma->vm_private_data;
	ENTRY();
	f->vma_use_count--;
	LEAVE();
}


static const struct vm_operations_struct nuvoton_vin_vm_ops = {
	.open = nuvoton_vin_vm_open,
	.close = nuvoton_vin_vm_close,
};

static int nuvoton_vdi_mmap(struct file* filp, struct vm_area_struct *vma)
{
	
	struct nuvoton_vin_device *cam = video_drvdata(filp);
	unsigned long size = vma->vm_end - vma->vm_start,start = vma->vm_start;
	void *pos;
	u32 i;
	ENTRY();
	if (mutex_lock_interruptible(&cam->fileop_mutex))
		return -ERESTARTSYS;

	if (cam->state & DEV_DISCONNECTED) {
		VDEBUG("Device not present\n");
		mutex_unlock(&cam->fileop_mutex);
		return -ENODEV;
	}

	if (cam->state & DEV_MISCONFIGURED) {
		VDEBUG("The camera is misconfigured. Close and open it again.\n");
		mutex_unlock(&cam->fileop_mutex);
		return -EIO;
	}

	if (!(vma->vm_flags & (VM_WRITE | VM_READ))) {
		mutex_unlock(&cam->fileop_mutex);
		return -EACCES;
	}

	if (cam->io != IO_MMAP ||
	    size != PAGE_ALIGN(cam->frame[0].buf.length)) {
		mutex_unlock(&cam->fileop_mutex);
		return -EINVAL;
	}

	for (i = 0; i < cam->nbuffers; i++) {
		if ((cam->frame[i].buf.m.offset>>PAGE_SHIFT) == vma->vm_pgoff)
			break;
	}
	if (i == cam->nbuffers) {
		mutex_unlock(&cam->fileop_mutex);
		return -EINVAL;
	}

	vma->vm_flags |= VM_IO | VM_DONTEXPAND | VM_DONTDUMP;
	pos = cam->frame[i].bufmem;
	#if 1
	while (size > 0) { /* size is page-aligned */
		if (vm_insert_page(vma, start, vmalloc_to_page(pos))) {
			mutex_unlock(&cam->fileop_mutex);
			return -EAGAIN;
		}
		start += PAGE_SIZE;
		pos += PAGE_SIZE;
		size -= PAGE_SIZE;
	}
	#else	
	while (size > 0)
	{
		page = vmalloc_to_pfn((void *)pos);
		if (remap_pfn_range(vma, start, page, PAGE_SIZE, PAGE_SHARED))
		{			
			return -EAGAIN;
		}
		start += PAGE_SIZE;
		pos += PAGE_SIZE;
		if (size > PAGE_SIZE)
			size -= PAGE_SIZE;
		else
			size = 0;
	}

	#endif

	vma->vm_ops = &nuvoton_vin_vm_ops;
	vma->vm_private_data = &cam->frame[i];
	nuvoton_vin_vm_open(vma);

	mutex_unlock(&cam->fileop_mutex);
	LEAVE();
	return 0;	
	
}

/* ISR for buffer handling                   *
 * for nuvoton sensor interface              *
 * type of dev_id is "struct im9815_vdo_dev" */
static irqreturn_t nuvoton_vdi_isr(int irq, void *priv)
{
	struct nuvoton_vin_device* cam=priv;	
	struct nuvoton_vin_frame_t** f=NULL;	
	ENTRY();	
	f  = &cam->frame_current;	
	if (cam->stream == STREAM_INTERRUPT) {
		VDEBUG("STREAM_INTERRUPT\n");
		cam->stream = STREAM_OFF;
		if ((*f))
			(*f)->state = F_QUEUED;
		wake_up(&cam->wait_stream);
	}

	if (cam->stream == STREAM_OFF || list_empty(&cam->inqueue))
	{
		if(cam->stream == STREAM_OFF) VDEBUG("STREAM_OFF\n");
		VDEBUG("goto resubmit\n");
		goto resubmit;
	}

	if (!(*f))
	{
		goto resubmit;
		//(*f) = list_entry(cam->inqueue.next, struct nuvoton_vin_frame_t,frame);		
	}
	spin_lock(&cam->queue_lock);
	list_move_tail(&(*f)->frame, &cam->outqueue);

	if (!list_empty(&cam->inqueue))
	{						
		(*f) = list_entry(cam->inqueue.next,struct nuvoton_vin_frame_t,frame);
		
		/* Update New frame */
		__raw_writel(__raw_readl(REG_CAP_CTL) | CAP_CTL_UPDATE,REG_CAP_CTL);
		
		if(cam->vpe.PacketEnable==1)
		{				
			/* Setting packet buffer start address */
			__raw_writel((*f)->pbuf,REG_CAP_PKTBA0);
		}
		if(cam->vpe.PlanarEnable==1)
		{
				/* Setting planar buffer Y address, U address, V address */
				__raw_writel((unsigned int)(*f)->pbuf,REG_CAP_YBA);
				__raw_writel(__raw_readl(REG_CAP_YBA)+(cam->vpe.PlanarWidth*cam->vpe.PlanarHeight),REG_CAP_UBA);
				__raw_writel(__raw_readl(REG_CAP_UBA)+(cam->vpe.PlanarWidth*cam->vpe.PlanarHeight)/2,REG_CAP_VBA);
		}			
	}	
//	else
//		(*f) = NULL;
	spin_unlock(&cam->queue_lock);

	
resubmit:				
	VDEBUG("wake_up_interruptible\n");
	wake_up_interruptible(&cam->wait_frame);
	__raw_writel(__raw_readl(REG_CAP_INT),REG_CAP_INT);
	LEAVE();
	return IRQ_NONE;
}



static struct v4l2_file_operations nuvoton_vdi_fops = 
{    
	.owner	    		= THIS_MODULE,    
	.open       		= nuvoton_vdi_open,    
	.release    		= nuvoton_vdi_close,    
	.read       		= nuvoton_vdi_read,    	  
	.ioctl      		= video_ioctl2, 		/* V4L2 ioctl handler */    
	.unlocked_ioctl 	= video_ioctl2, 		/* V4L2 ioctl handler */    
	.mmap       		= nuvoton_vdi_mmap,
};

/* ----	Initialization v4l2_file_operations  ----*/
extern int nuvoton_vin_probe(struct nuvoton_vin_device* cam);
int nuvoton_vdi_device_register(void)
{
	struct nuvoton_vin_device* cam;
	static unsigned int dev_nr;	
	int err = 0;
	ENTRY();
	
	if (!(cam = kzalloc(sizeof(struct nuvoton_vin_device), GFP_KERNEL)))
			return -ENOMEM;

	if (!(cam->control_buffer = kzalloc(4, GFP_KERNEL))) {
		VDEBUG("kmalloc() failed\n");
		err = -ENOMEM;
		goto fail;
	}
					
	if (!(cam->v4ldev = video_device_alloc())) {
			VDEBUG("video_device_alloc() failed\n");
			err = -ENOMEM;
			goto fail;
	}

	capture_init(cam); /* Set capture init for nuvoton sensor interface */
	
	//for sensor init
	if(nuvoton_vin_probe(cam)<0){  //sensor probe;
		VDEBUG("Initialization failed. I will retry on open().");
		return -EAGAIN;
	}
	
	VDEBUG("capture_init().");	
	
	/* INIT */
	
	mutex_init(&cam->open_mutex);
	mutex_init(&cam->fileop_mutex);
	init_waitqueue_head(&cam->wait_frame);
	init_waitqueue_head(&cam->wait_stream);	
	cam->nreadbuffers = 2;
	
	strcpy(cam->v4ldev->name, "NUVOTON Camera Interface");
	cam->v4ldev->current_norm	= V4L2_STD_NTSC_M;
	cam->v4ldev->fops		= &nuvoton_vdi_fops;
	cam->v4ldev->release		= video_device_release;
	cam->v4ldev->tvnorms		= V4L2_STD_525_60;		
	cam->v4ldev->ioctl_ops = &nuvoton_vdi_ioctl_ops; /* for V4L2 ioctl handler */	
	video_set_drvdata(cam->v4ldev, cam);
	
	init_completion(&cam->probe);
	
	err = video_register_device(cam->v4ldev, VFL_TYPE_GRABBER,
				    video_nr[dev_nr]);
	if (err) {
		VDEBUG("V4L2 device registration failed\n");
		if (err == -ENFILE && video_nr[dev_nr] == -1)
			VDEBUG("Free /dev/videoX node not found\n");
		video_nr[dev_nr] = -1;
		dev_nr = (dev_nr < NUVOTON_MAX_DEVICES-1) ? dev_nr+1 : 0;
		complete_all(&cam->probe);
		goto fail;
	}				

	/* Setting Interrupt(IRQ) for nuvoton sensor interface  */	
	if(dev_nr==0)
	{
		err = request_irq(IRQ_CAP, nuvoton_vdi_isr, IRQF_SHARED, "camera sensor 0", cam);
			if(err < 0)	
			{        
				VDEBUG("Interrupt(IRQ_CAP) setup failed\n");
				goto fail;
			}
			else
				VDEBUG("%s, main sensor isr is created\n", __func__);
	}
		
	VDEBUG("V4L2 device registered as %s\n",video_device_node_name(cam->v4ldev));		
	
	cam->module_param.force_munmap = force_munmap[dev_nr];
	cam->module_param.frame_timeout = frame_timeout[dev_nr];

	dev_nr = (dev_nr < NUVOTON_MAX_DEVICES-1) ? dev_nr+1 : 0;	
	kref_init(&cam->kref);
	complete_all(&cam->probe);		
	return 0;
fail:
	if (cam) {
		kfree(cam->control_buffer);
		if (cam->v4ldev)
			video_device_release(cam->v4ldev);
		kfree(cam);
	}
	return err;
}

/*****************************************************************************/ //for sensor
void nuvoton_vin_attach_sensor(struct nuvoton_vin_device* cam, struct nuvoton_vin_sensor* sensor)
{
	memcpy(&cam->sensor, sensor, sizeof(struct nuvoton_vin_sensor));
}
/*****************************************************************************/


/* This routine allocates from 1 to n_devs drivers. 
The real maximum number of virtual drivers will depend on how many drivers   will succeed.
This is limited to the maximum number of devices that   videodev supports.
Since there are 64 minors for video grabbers, this is   currently the theoretical maximum limit.
However, a further limit does   exist at videodev that forbids any driver to register more than 32 video   grabbers. 
Max number of video_devices can be registered at the same time: 32 */
static int nuvoton_cap_device_probe(struct platform_device *pdev)
{
	int ret = -ENOMEM;
	ENTRY();	
	ret = nuvoton_vdi_device_register();
	if(ret)
	{	
		printk("%s, main sensor registeration fail.\n\n", __func__);
		ret = -EPROBE_DEFER;
		goto out;
	}
	else
		printk("%s, main sensor registeration ok.\n\n", __func__);
	
out:
	LEAVE();
	return ret; 
}

static int nuvoton_cap_device_remove(struct platform_device *pdev)
{
	ENTRY();
	LEAVE();
	return 0;
}

static struct platform_device_id nuc970_cap_driver_ids[] = {	
	{ "nuc970-videoin", 0 },
	{ },
};

static struct platform_driver nuc970_cap_driver = {
		.probe		= nuvoton_cap_device_probe,
		.remove 	= nuvoton_cap_device_remove,
		.driver 	= {
			.name	= "nuc970-cap",
			.owner	= THIS_MODULE,
		},
		.id_table	= nuc970_cap_driver_ids,
	};

module_platform_driver(nuc970_cap_driver);
MODULE_DESCRIPTION("NUVOTON sensor interface");
MODULE_AUTHOR("SCHung");
MODULE_LICENSE("Dual BSD/GPL");
