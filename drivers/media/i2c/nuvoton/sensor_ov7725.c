#include <linux/delay.h>
#include <linux/module.h>
#include "nuc970_cap.h"


static struct nuvoton_vin_sensor ov7725;

struct OV_RegValue{
	__u8	uRegAddr;
	__u8	uValue;
};

#define _REG_TABLE_SIZE(nTableName)	sizeof(nTableName)/sizeof(struct OV_RegValue)

static struct OV_RegValue RegValue[] = 
{	
		{0x12, 0x80}, {0x12, 0x00}, {0x3D, 0x03}, {0x17, 0x22}, {0x18, 0xA4}, 
		{0x19, 0x07}, {0x1A, 0xF0}, {0x32, 0x02}, {0x29, 0xA0}, {0x2C, 0xF0},
		{0x2A, 0x02}, {0x65, 0x20}, {0x11, 0x01}, {0x42, 0x7F}, {0x63, 0xE0}, 
		{0x64, 0xFF}, {0x66, 0x00},	{0x67, 0x48}, {0x0D, 0x41}, {0x0E, 0x01}, 
		{0x0F, 0xC5}, {0x14, 0x11}, {0x22, 0x7F}, {0x23, 0x03}, {0x24, 0x40},
		{0x25, 0x30}, {0x26, 0xA1}, {0x2B, 0x00}, {0x6B, 0xAA}, {0x13, 0xEF}, 
		{0x90, 0x05}, {0x91, 0x01}, {0x92, 0x03}, {0x93, 0x00}, {0x94, 0x90}, 
		{0x95, 0x8A}, {0x96, 0x06}, {0x97, 0x0B}, {0x98, 0x95}, {0x99, 0xA0},
		{0x9A, 0x1E}, {0x9B, 0x08}, {0x9C, 0x20}, {0x9E, 0x81}, {0xA6, 0x04}, 
		{0x7E, 0x0C}, {0x7F, 0x24}, {0x80, 0x3A}, {0x81, 0x60}, {0x82, 0x70}, 
		{0x83, 0x7E}, {0x84, 0x8A}, {0x85, 0x94}, {0x86, 0x9E}, {0x87, 0xA8},
		{0x88, 0xB4}, {0x89, 0xBE}, {0x8A, 0xCA}, {0x8B, 0xD8}, {0x8C, 0xE2}, 
		{0x8D, 0x28}, {0x46, 0x05}, {0x47, 0x00}, {0x48, 0x00}, {0x49, 0x12}, 
		{0x4A, 0x00}, {0x4B, 0x13}, {0x4C, 0x21}, {0x0C, 0x10}, {0x09, 0x00},
		{0xFF, 0xFF}, {0xFF, 0xFF}	
};

/************  I2C  *****************/
static struct i2c_client *save_client;
static char sensor_inited = 0;
__u8 sensor_read(__u8 uRegAddr)
{
		u8 val;		
		//printk("sensor_read i2c_smbus_read_byte uRegAddr=0x%x\n",uRegAddr);
		i2c_smbus_write_byte(save_client, uRegAddr);
		//printk("sensor_read i2c_smbus_write_byte\n");
		val = i2c_smbus_read_byte(save_client);		
		return val;
}

static int32_t sensor_write(__u8 uRegAddr, __u8 uData)
{
		int ret;
		ret=i2c_smbus_write_byte_data(save_client, uRegAddr, uData);			
		return ret;
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

static int ov7725_init(struct nuvoton_vin_device* cam)
{
	int err = 0;
	ENTRY();
	LEAVE();		
	return err;
}

static struct nuvoton_vin_sensor ov7725 = {
	.name = "ov7725",
	.init = &ov7725_init,
	.infmtord = (INORD_YUYV | INFMT_YCbCr | INTYPE_CCIR601),
	.polarity = (VSP_HI | HSP_LO | PCLKP_HI),
	.cropstart = ( 0 | 2<<16 ),	/*( Vertical | Horizontal<<16 ) */
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
	__u8 SensorID[4];
	struct OV_RegValue *psRegValue;
	ENTRY();	
	nuvoton_vin_attach_sensor(cam, &ov7725);
	
	// if i2c module isn't loaded at this time
	if(!sensor_inited)
		return -1;
		
	psRegValue=RegValue;
	for(i=0;i<_REG_TABLE_SIZE(RegValue); i++, psRegValue++)
	{
		int32_t ret;
		printk(".");		
		ret = sensor_write((psRegValue->uRegAddr), (psRegValue->uValue));
		if(ret<0)
		{
			VDEBUG("Wrong to write register addr = 0x%x, write data = 0x%x , ret = %d\n", (psRegValue->uRegAddr), (psRegValue->uValue), ret);					
		}	
	} 	
	//----------Read sensor id-------------------------------------	        
	SensorID[0]=sensor_read(0x0A);  /* PID 0x77 */		
	SensorID[1]=sensor_read(0x0B);  /* VER 0x21 */
	SensorID[2]=sensor_read(0x1C);  /* Manufacturer ID Byte - High  0x7F */	
	SensorID[3]=sensor_read(0x1D);  /* Manufacturer ID Byte - Low   0xA2 */
	printk("Sensor PID = 0x%02x(0x77) VER = 0x%02x(0x21) MIDH = 0x%02x(0x7F) MIDL = 0x%02x(0xA2)\n", SensorID[0],SensorID[1],SensorID[2],SensorID[3]);	
	//-------------------------------------------------------------		
	printk("\n");
	if(ret>=0)
		printk("driver i2c initial done\n");
	else
		printk("driver i2c initial fail\n");	
	LEAVE();		
	return ret;	
}

static const struct i2c_device_id sensor_id[] = {{ "ov7725", 0 },};
MODULE_DEVICE_TABLE(i2c, sensor_id);

static struct i2c_driver sensor_i2c_driver = {
	.driver = { .name  = "ov7725", 
							.owner = THIS_MODULE,
						},
	.probe    = sensor_probe,
	.remove   = sensor_remove,	
	.id_table = sensor_id,
};

module_i2c_driver(sensor_i2c_driver);
