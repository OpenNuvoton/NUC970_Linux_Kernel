#include <linux/delay.h>
#include <linux/module.h>
#include "nuc970_cap.h"

static struct nuvoton_vin_sensor nt99050;

struct OV_RegValue{
	__u16	uRegAddr;
	__u8	uValue;
};

#define _REG_TABLE_SIZE(nTableName)	sizeof(nTableName)/sizeof(struct OV_RegValue)
/* NT99141, VGA, YUV422 */
static struct OV_RegValue Init_RegValue[] = 
{	
 	//[InitialSetting]
    {0x3021, 0x01},
#if 0 /* BT656 */
    {0x32F0, 0x61},{0x32F1, 0x10}, 	
#else
   {0x32F0, 0x01}, 
#endif    
    {0x3024, 0x00}, {0x3270, 0x00}, //[Gamma_MDR]
    {0x3271, 0x0D}, {0x3272, 0x19}, {0x3273, 0x2A}, {0x3274, 0x3C}, {0x3275, 0x4D}, 
    {0x3276, 0x67}, {0x3277, 0x81}, {0x3278, 0x98}, {0x3279, 0xAD}, {0x327A, 0xCE}, 
    {0x327B, 0xE0}, {0x327C, 0xED}, {0x327D, 0xFF}, {0x327E, 0xFF}, {0x3060, 0x01},
    {0x3210, 0x04}, //LSC //D
    {0x3211, 0x04}, //F
    {0x3212, 0x04}, //D
    {0x3213, 0x04}, //D
    {0x3214, 0x04}, {0x3215, 0x05}, {0x3216, 0x04}, {0x3217, 0x04}, {0x321C, 0x04},
    {0x321D, 0x05}, {0x321E, 0x04}, {0x321F, 0x03}, {0x3220, 0x00}, {0x3221, 0xA0},
    {0x3222, 0x00}, {0x3223, 0xA0}, {0x3224, 0x00}, {0x3225, 0xA0}, {0x3226, 0x80},
    {0x3227, 0x88}, {0x3228, 0x88}, {0x3229, 0x30}, {0x322A, 0xCF}, {0x322B, 0x07},
    {0x322C, 0x04}, {0x322D, 0x02}, {0x3302, 0x00},//[CC: Saturation:100%]
    {0x3303, 0x1C}, {0x3304, 0x00}, {0x3305, 0xC8}, {0x3306, 0x00}, {0x3307, 0x1C},
    {0x3308, 0x07}, {0x3309, 0xE9}, {0x330A, 0x06}, {0x330B, 0xDF}, {0x330C, 0x01},
    {0x330D, 0x38}, {0x330E, 0x00}, {0x330F, 0xC6}, {0x3310, 0x07}, {0x3311, 0x3F},
    {0x3312, 0x07}, {0x3313, 0xFC}, {0x3257, 0x50}, //CA Setting
    {0x3258, 0x10}, {0x3251, 0x01}, {0x3252, 0x50}, {0x3253, 0x9A}, {0x3254, 0x00}, 
    {0x3255, 0xd8}, {0x3256, 0x60}, {0x32C4, 0x38}, {0x32F6, 0xCF}, {0x3363, 0x37},
    {0x3331, 0x08}, {0x3332, 0x6C}, // 60
    {0x3360, 0x10}, {0x3361, 0x30}, {0x3362, 0x70}, {0x3367, 0x40}, {0x3368, 0x32}, //20
    {0x3369, 0x24}, //1D
    {0x336A, 0x1A}, {0x336B, 0x20}, {0x336E, 0x1A}, {0x336F, 0x16}, {0x3370, 0x0c},
    {0x3371, 0x12}, {0x3372, 0x1d}, {0x3373, 0x24}, {0x3374, 0x30}, {0x3375, 0x0A},
    {0x3376, 0x18}, {0x3377, 0x20}, {0x3378, 0x30}, {0x3340, 0x1C}, {0x3326, 0x03}, //Eext_DIV
    {0x3200, 0x3E}, //1E
    {0x3201, 0x3F}, {0x3109, 0x82}, //LDO Open
    {0x3106, 0x07}, {0x303F, 0x02}, {0x3040, 0xFF}, {0x3041, 0x01}, {0x3051, 0xE0},
    {0x3060, 0x01},

    {0x32BF, 0x04}, {0x32C0, 0x6A},	{0x32C1, 0x6A},	{0x32C2, 0x6A}, {0x32C3, 0x00},
		{0x32C4, 0x20}, {0x32C5, 0x20}, {0x32C6, 0x20},	{0x32C7, 0x00},	{0x32C8, 0x95},
		{0x32C9, 0x6A},	{0x32CA, 0x8A},	{0x32CB, 0x8A},	{0x32CC, 0x8A},	{0x32CD, 0x8A},
		{0x32D0, 0x01},	{0x3200, 0x3E},	{0x3201, 0x0F},	{0x302A, 0x00},	{0x302B, 0x09},
		{0x302C, 0x00},	{0x302D, 0x04},	{0x3022, 0x24},	{0x3023, 0x24},	{0x3002, 0x00},
		{0x3003, 0x00},	{0x3004, 0x00},	{0x3005, 0x00},	{0x3006, 0x02},	{0x3007, 0x83},
		{0x3008, 0x01},	{0x3009, 0xE3},

		{0x300A, 0x03},	{0x300B, 0x28},	{0x300C, 0x01},	{0x300D, 0xF4},
		
		{0x300E, 0x02},	{0x300F, 0x84},	{0x3010, 0x01},	{0x3011, 0xE4},	{0x32B8, 0x3B},
		{0x32B9, 0x2D},	{0x32BB, 0x87},	{0x32BC, 0x34},	{0x32BD, 0x38},	{0x32BE, 0x30},
		{0x3201, 0x3F},	{0x320A, 0x01},	{0x3021, 0x06},	{0x3060, 0x01},	
};

/************  I2C  *****************/
static struct i2c_client *save_client;
static char sensor_inited = 0;

static int sensor_read(u16 reg,u8 *val)
{
	int ret;
	/* We have 16-bit i2c addresses - care for endianess */
	unsigned char data[2] = { reg >> 8, reg & 0xff };

	ret = i2c_master_send(save_client, data, 2);
	if (ret < 2) {
		dev_err(&save_client->dev, "%s: i2c read error, reg: 0x%x\n",
			__func__, reg);
		return ret < 0 ? ret : -EIO;
	}

	ret = i2c_master_recv(save_client, val, 1);
	if (ret < 1) {
		dev_err(&save_client->dev, "%s: i2c read error, reg: 0x%x\n",__func__, reg);
		return ret < 0 ? ret : -EIO;
	}
	return 0;
}

static int sensor_write(u16 reg, u8 val)
{
	int ret;
	unsigned char data[3] = { reg >> 8, reg & 0xff, val };

	ret = i2c_master_send(save_client, data, 3);
	if (ret < 3) {
		dev_err(&save_client->dev, "%s: i2c write error, reg: 0x%x\n",
			__func__, reg);
		return ret < 0 ? ret : -EIO;
	}

	return 0;
}

static int sensor_probe(struct i2c_client *client,const struct i2c_device_id *did)
{
	ENTRY();
	sensor_inited = 1;
	client->flags = I2C_CLIENT_SCCB;
	save_client = client;
	LEAVE();
	return 0;
}
static int sensor_remove(struct i2c_client *client)
{	
	ENTRY();
	LEAVE();
	return 0;
}

static int nt99050_init(struct nuvoton_vin_device* cam)
{
	int err = 0;
	ENTRY();
	LEAVE();		
	return err;
}

static struct nuvoton_vin_sensor nt99050 = {
	.name = "nt99050",
	.init = &nt99050_init,
	.infmtord = (INORD_YUYV | INFMT_YCbCr | INTYPE_CCIR601),
	.polarity = (VSP_LO | HSP_LO | PCLKP_HI),
	.cropstart = ( 0 | 0<<16 ), /*( Vertical | Horizontal<<16 ) */
	.cropcap = {
		.bounds = {
			.left = 0,
			.top = 0,
			.width = 640,
			.height = 480,
		},
		.defrect = {
			.left = 0,
			.top = 0,
			.width = 800,
			.height = 480,
		},
	},
	.pix_format	 = {
		.width = 640,
		.height = 480,
		.pixelformat = V4L2_PIX_FMT_YUYV,
		.priv = 16,
		.colorspace = V4L2_COLORSPACE_JPEG,
	},
};

int nuvoton_vin_probe(struct nuvoton_vin_device* cam)
{
	int i,ret = 0;
	__u8 SensorID[2];
	struct OV_RegValue *psRegValue;	
	ENTRY();	
	nuvoton_vin_attach_sensor(cam, &nt99050);
	
	// if i2c module isn't loaded at this time
	if(!sensor_inited)
		return -1;
		
	psRegValue=Init_RegValue;
	for(i=0;i<_REG_TABLE_SIZE(Init_RegValue); i++, psRegValue++)
	{		
		printk(".");		
		ret = sensor_write((psRegValue->uRegAddr), (psRegValue->uValue));	
	} 	

	//----------Read sensor id-------------------------------------	        
	sensor_read(0x3000,&SensorID[0]);  /* Chip_Version_H 0x14 */			
	sensor_read(0x3001,&SensorID[1]);  /* Chip_Version_L 0x10 */		
	printk("\nSensor Chip_Version_H = 0x%02x(0x05) Chip_Version_L = 0x%02x(0x00)\n", SensorID[0],SensorID[1]);
	//-------------------------------------------------------------		
	printk("\n");
	if(ret>=0)
		printk("driver i2c initial done\n");
	else
		printk("driver i2c initial fail\n");	
	LEAVE();		
	return ret;	
}
static const struct i2c_device_id sensor_id[] = {{ "nt99050", 0 },{}};
MODULE_DEVICE_TABLE(i2c, sensor_id);

static struct i2c_driver sensor_i2c_driver = {
	.driver = { .name  = "nt99050", 
				.owner = THIS_MODULE,
						},
	.probe    = sensor_probe,
	.remove   = sensor_remove,	
	.id_table = sensor_id,
};
module_i2c_driver(sensor_i2c_driver);
