#ifndef _NUVOTON_SENSOR_H
#define _NUVOTON_SENSOR_H

#include <linux/delay.h>
#include <linux/i2c.h>

struct nuvoton_vin_device;
struct nuvoton_vin_sensor;

#define NUVOTON_MAX_CTRLS 							(V4L2_CID_LASTP1 - V4L2_CID_BASE + 10)
#define NUVOTON_V4L2_CID_DAC_MAGNITUDE 	(V4L2_CID_PRIVATE_BASE + 0)
#define NUVOTON_V4L2_CID_GREEN_BALANCE 	(V4L2_CID_PRIVATE_BASE + 1)

struct nuvoton_vin_sensor {
	char name[32];

	#if 0
	struct v4l2_queryctrl qctrl[NUVOTON_MAX_CTRLS];
	#else
	struct v4l2_queryctrl qctrl;
	#endif
	
	struct v4l2_cropcap cropcap;
	struct v4l2_pix_format pix_format;
	u32 polarity;
	u32 infmtord;
	u32 cropstart;
	int (*init)(struct nuvoton_vin_device*);
	int (*get_ctrl)(struct nuvoton_vin_device*, struct v4l2_control* ctrl);
	int (*set_ctrl)(struct nuvoton_vin_device*,
			const struct v4l2_control* ctrl);
	int (*set_crop)(struct nuvoton_vin_device*, const struct v4l2_rect* rect);

	/* Private */
	#if 0
	struct v4l2_queryctrl _qctrl[NUVOTON_MAX_CTRLS];
	#else
	struct v4l2_queryctrl _qctrl;
	#endif
	
	struct v4l2_rect _rect;
};

#endif