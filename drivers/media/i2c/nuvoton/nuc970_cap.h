#ifndef _NUVOTON_VDI_V4L2_H
#define _NUVOTON_VDI_V4L2_H

#include <linux/version.h>
#include <linux/videodev2.h>
#include <media/v4l2-common.h>
#include <media/v4l2-ioctl.h>
#include <linux/device.h>
#include <linux/list.h>
#include <linux/spinlock.h>
#include <linux/time.h>
#include <linux/wait.h>
#include <linux/types.h>
#include <linux/param.h>
#include <linux/mutex.h>
#include <linux/rwsem.h>
#include <linux/stddef.h>
#include <linux/string.h>
#include <linux/kref.h>


#if 0
#define VDI_DEBUG_ENABLE_ENTER_LEAVE
#define VDI_DEBUG
#endif

#ifdef VDI_DEBUG
#define VDEBUG(fmt, arg...)		printk(fmt, ##arg)
#else
#define VDEBUG(fmt, arg...)
#endif

#ifdef VDI_DEBUG_ENABLE_ENTER_LEAVE
#define ENTRY()					printk("[%-20s] : Enter...\n", __FUNCTION__)
#define LEAVE()					printk("[%-20s] : Leave...\n", __FUNCTION__)
#else
#define ENTRY()
#define LEAVE()
#endif

#include "nuc970_sensor.h"

#define NUVOTON_MAX_DEVICES         2
#define NUVOTON_MAX_FRAMES 		      CONFIG_MAX_FRAME_BUFFER
#define NUVOTON_FORCE_MUNMAP        0
#define NUVOTON_FRAME_TIMEOUT       2


#define VSP_LO      0x000 /* 0 : VS pin output polarity is active low */
#define VSP_HI      0x400 /* 1 : VS pin output polarity is active high. */
#define HSP_LO      0x000 /* 0 : HS pin output polarity is active low */
#define HSP_HI      0x200 /* 1 : HS pin output polarity is active high. */
#define PCLKP_LO    0x000 /* 0 : Input video data and signals are latched by falling edge of Pixel Clock */
#define PCLKP_HI    0x100 /* 1 : Input video data and signals are latched by rising edge of Pixel Clock. */


#define INFMT_YCbCr  0x0 		/*  Sensor Input Data Format YCbCr422 */
#define INFMT_RGB565 0x1 		/*  Sensor Input Data Format RGB565 */
#define INTYPE_CCIR601 (0x0<<1) 	/*  Sensor Input Type CCIR601 */
#define INTYPE_CCIR656 (0x1<<1) 	/*  Sensor Input Type CCIR656 */
#define INORD_YUYV  (0x0<<2) 	/*  Sensor Input Data Order YUYV */
#define INORD_YVYU  (0x1<<2) 	/*  Sensor Input Data Order YVYU */
#define INORD_UYVY  (0x2<<2) 	/*  Sensor Input Data Order UYVY */
#define INORD_VYUY  (0x3<<2) 	/*  Sensor Input Data Order VYUY */
#define INMASK 0xF				/*  Sensor Input Mask */


enum nuvoton_vin_frame_state {
	F_UNUSED,
	F_QUEUED,
	F_GRABBING,
	F_DONE,
	F_ERROR,
};

struct nuvoton_vin_frame_t {
	void* bufmem;
	u32 pbuf;
	struct v4l2_buffer buf;
	enum nuvoton_vin_frame_state state;
	struct list_head frame;
	unsigned long vma_use_count;
};

enum nuvoton_vin_dev_state {
	DEV_INITIALIZED = 0x01,
	DEV_DISCONNECTED = 0x02,
	DEV_MISCONFIGURED = 0x04,
};

enum nuvoton_vin_io_method {
	IO_NONE,
	IO_READ,
	IO_MMAP,
};

enum nuvoton_vin_stream_state {
	STREAM_OFF,
	STREAM_INTERRUPT,
	STREAM_ON,
};

struct nuvoton_vin_module_param {
	u8 force_munmap;
	u16 frame_timeout;
};

static DECLARE_RWSEM(nuvoton_vin_dev_lock);

struct capture_parameter{
	u32 format;
	u32 PacketWidth;
	u32 PacketHeight;
	
	u32 PlanarWidth;
	u32 PlanarHeight;
	
	u8 PacketEnable;
	u8 PlanarEnable;
	};

struct nuvoton_vin_device {
	struct video_device* v4ldev;
	struct nuvoton_vin_sensor sensor;
	u8* control_buffer;
	u32 type;
	struct nuvoton_vin_frame_t *frame_current, frame[NUVOTON_MAX_FRAMES];
	struct list_head inqueue, outqueue;
	u32 frame_count, nbuffers, nreadbuffers;

	enum nuvoton_vin_io_method io;
	enum nuvoton_vin_stream_state stream;

	struct v4l2_jpegcompression compression;

	struct nuvoton_vin_module_param module_param;

	struct kref kref;
	enum nuvoton_vin_dev_state state;
	u8 users;

	struct completion probe;
	struct mutex open_mutex, fileop_mutex;
	spinlock_t queue_lock;
	//wait_queue_head_t wait_open;
	wait_queue_head_t wait_frame, wait_stream;

	struct capture_parameter vpe; /*sensor interface for nuvoton */
	dma_addr_t phy_addr[NUVOTON_MAX_FRAMES];
	void* vir_addr[NUVOTON_MAX_FRAMES];
};

/*****************************************************************************/
void nuvoton_vin_attach_sensor(struct nuvoton_vin_device* cam, struct nuvoton_vin_sensor* sensor);
/*****************************************************************************/

void nuvoton_vin_release_buffers(struct nuvoton_vin_device* cam);
void nuvoton_vin_empty_framequeues(struct nuvoton_vin_device* cam);
void nuvoton_vin_requeue_outqueue(struct nuvoton_vin_device* cam);
u32 nuvoton_vin_request_buffers(struct nuvoton_vin_device* cam, u32 count,enum nuvoton_vin_io_method io);
int nuvoton_vin_stream_interrupt(struct nuvoton_vin_device* cam);

#endif /*_NUVOTON_V4L2_H*/
