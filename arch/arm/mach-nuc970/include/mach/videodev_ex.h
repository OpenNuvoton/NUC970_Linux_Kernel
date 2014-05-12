/* videodev_ex.h
 *
 * Copyright (c) Nuvoton technology corporation
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */

#ifndef __LINUX_VIDEODEV_EX_H
#define __LINUX_VIDEODEV_EX_H

#ifndef __LINUX_VIDEODEV_H
#error Please include videodev.h at first
#endif

typedef struct{
	unsigned int u32RemainBufSize;
	unsigned int u32RemainBufPhyAdr;
}S_BUF_INFO;

typedef struct 
{
	int32_t i32PipeBufNo;
	int32_t i32PipeBufSize;
	int32_t i32CurrPipePhyAddr;
}S_PIPE_INFO;


#define	VIDIOCGCAPTIME				_IOR('v',30, struct v4l2_buffer)		/* Get Capture time */
#define	VIDIOCSBRIGHTNESS			_IOW('v',31, int)
#define	VIDIOCGBRIGHTNESS			_IOR('v',32, int)
#define	VIDIOCSCONTRAST				_IOW('v',33, int)
#define	VIDIOCGCONTRAST				_IOR('v',34, int)
#define	VIDIOCSSHARPNESS			_IOW('v',35, int)
#define	VIDIOCGSHARPNESS			_IOR('v',36, int)
#define	VIDIOCSWHITEBALANCE			_IOW('v',37, int)
#define	VIDIOCGWHITEBALANCE			_IOR('v',38, int)
#define	VIDIOCSNOISEREDUCTION		_IOW('v',39, int)
#define	VIDIOCGNOISEREDUCTION		_IOR('v',40, int)
#define	VIDIOCSCOLORSATURATION		_IOW('v',41, int)
#define	VIDIOCGCOLORSATURATION		_IOR('v',42, int)
#define	VIDIOCSPREVIEW				_IOR('v',43, int)
#define	VIDIOCSFLICKERFREQ			_IOW('v',44, int)
#define	VIDIOCGSYSUPTIME			_IOR('v',45, struct timeval)/*Get system up time*/
#define	VIDIOCSIRLED				_IOR('v',46, int)			/* 0: off 1: on 2: auto*/
#define	VIDIOCGIRLEDONOFF			_IOR('v',47, int)			/* 0: off 1: on */


#define VIDIOC_G_DIFF_OFFSET		_IOR('v',48, int)			/* Get diff offset address and size */ 
#define VIDIOC_G_DIFF_SIZE			_IOR('v',49, int)
#define VIDIOC_S_MOTION_THRESHOLD	_IOW('v',50, int)			/* Set motion detection threshold */
#define VIDIOC_QUERY_SENSOR_ID		_IOR('v',51, int)
#define	VIDIOC_G_BUFINFO			_IOR('v',52, S_BUF_INFO)

#define VIDIOC_G_PACKET_INFO		_IOR('v',53, S_PIPE_INFO)			/* Get Packet offset address and size */ 
#define VIDIOC_G_PLANAR_INFO		_IOR('v',54, S_PIPE_INFO)			/* Get Planar offset address and size */ 
#define VIDIOC_S_MAP_BUF			_IOW('v',55, int)					/* Specified the mapping buffer */

//#define VIDIOC_G_PACKET_SIZE		_IOR('v',54, int)			
//#define VIDIOC_G_PACKET_ADDR		_IOR('v',55, int)			/* For encode packet pipe */

enum{
        OV_7648=1,
        PP_PO3030K,
        PP_PO2010D2,
        OV_9660,
        OV_9653,
        OV_7725=7725,
        OV_7670=7670,
        OV_2640,
        OV_6880,
        NT_99140=99140,
		NT_99141=99141,
        NT_99050=99050
};

#endif /* __LINUX_VIDEODEV_EX_H */
