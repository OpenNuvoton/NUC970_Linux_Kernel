/*
 * linux/arch/arm/mach-nuc970/dev.c
 *
 * Copyright (C) 2014 Nuvoton corporation.
 *
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 */

#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/interrupt.h>
#include <linux/list.h>
#include <linux/timer.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

#include <linux/mtd/physmap.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>

#include <linux/spi/spi.h>
#include <linux/spi/flash.h>
#include <linux/pwm.h>

#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/irq.h>
#include <asm/mach-types.h>

#include <mach/regs-serial.h>
#include <mach/irqs.h>
#include <mach/regs-gcr.h>
#include <mach/regs-aic.h>

#include <mach/map.h>
#include <mach/fb.h>
#include <mach/regs-lcd.h>
#include <mach/nuc970_spi.h>
#include <mach/gpio.h>

#include <linux/platform_data/i2c-nuc970.h>
#include <linux/i2c.h>
#include <linux/i2c-gpio.h>
#include <linux/i2c/i2c-hid.h>

#include "cpu.h"

/* USB EHCI Host Controller */
#ifdef CONFIG_USB_EHCI_HCD
static struct resource nuc970_ehci_resource[] = {
        [0] = {
                .start = NUC970_PA_EHCI,
                .end   = NUC970_PA_EHCI + NUC970_SZ_EHCI - 1,
                .flags = IORESOURCE_MEM,
        },
        [1] = {
                .start = IRQ_EHCI,
                .end   = IRQ_EHCI,
                .flags = IORESOURCE_IRQ,
        }
};

static u64 nuc970_device_usb_ehci_dmamask = 0xffffffffUL;

static struct platform_device nuc970_device_ehci = {
        .name		  = "nuc970-ehci",
        .id		  = -1,
        .num_resources	  = ARRAY_SIZE(nuc970_ehci_resource),
        .resource	  = nuc970_ehci_resource,
        .dev              = {
                .dma_mask = &nuc970_device_usb_ehci_dmamask,
                .coherent_dma_mask = 0xffffffffUL
        }
};
#endif
/* USB OHCI Host Controller */
#ifdef CONFIG_USB_OHCI_HCD
static struct resource nuc970_ohci_resource[] = {
        [0] = {
                .start = NUC970_PA_OHCI,
                .end   = NUC970_PA_OHCI + NUC970_SZ_OHCI - 1,
                .flags = IORESOURCE_MEM,
        },
        [1] = {
                .start = IRQ_OHCI,
                .end   = IRQ_OHCI,
                .flags = IORESOURCE_IRQ,
        }
};

static u64 nuc970_device_usb_ohci_dmamask = 0xffffffffUL;
static struct platform_device nuc970_device_ohci = {
        .name		  = "nuc970-ohci",
        .id		  = -1,
        .num_resources	  = ARRAY_SIZE(nuc970_ohci_resource),
        .resource	  = nuc970_ohci_resource,
        .dev              = {
                .dma_mask = &nuc970_device_usb_ohci_dmamask,
                .coherent_dma_mask = 0xffffffffUL
        }
};
#endif
/* USB Device (Gadget)*/
#ifdef CONFIG_USBD_NUC970
static struct resource nuc970_usbgadget_resource[] = {
        [0] = {
                .start = NUC970_PA_USBDEV,
                .end   = NUC970_PA_USBDEV + NUC970_SZ_USBDEV - 1,
                .flags = IORESOURCE_MEM,
        },
        [1] = {
                .start = IRQ_UDC,
                .end   = IRQ_UDC,
                .flags = IORESOURCE_IRQ,
        }
};

static u64 nuc970_device_udc_dmamask = 0xffffffffUL;
static struct platform_device nuc970_device_usbgadget = {
	.name		= "nuc970-usbgadget",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(nuc970_usbgadget_resource),
	.resource	= nuc970_usbgadget_resource,
	.dev              = {
		.dma_mask = &nuc970_device_udc_dmamask,
		.coherent_dma_mask = 0xffffffffUL
	}
};
#endif

/* Initial serial platform data */
static struct plat_nuc970serial_port nuc970_uart_data[] = {
        [0] = NUC970SERIAL_PORT(UART0),
        {},
};

static struct platform_device nuc970_serial_device = {
        .name			= "nuc970-uart",
        .id			= 1,
        .dev			= {
                .platform_data	= nuc970_uart_data,
        },
};


/* LCD controller*/
#ifdef CONFIG_FB_NUC970
//#define YUV422

static struct nuc970fb_display nuc970fb_lcd_info[] = {
	/* AUO A035QN02V0 320x240 TFT Panel */
	[0] = {
#ifndef YUV422
		.type		= LCM_DCCS_VA_SRC_RGB565,
#else
		.type		= LCM_DCCS_VA_SRC_YUV422,
#endif
		.width		= 320,
		.height		= 240,
		.xres		= 320,
		.yres		= 240,
		.bpp		= 16,
		.pixclock	= 200000,
		.left_margin	= 10,
		.right_margin   = 54,
		.hsync_len	= 10,
		.upper_margin	= 2,
		.lower_margin	= 4,
		.vsync_len	= 1,
#ifndef YUV422
		.dccs		= 0x0e00041a,
#else
		.dccs		= 0x0e00000a,
#endif
		.devctl		= 0x060800c0,
		.fbctrl		= 0x00a000a0,
		.scale		= 0x04000400,
	},
};


static struct nuc970fb_mach_info nuc970fb_fb_info = {
	.displays		= &nuc970fb_lcd_info[0],
	.num_displays		= ARRAY_SIZE(nuc970fb_lcd_info),
	.default_display	= 0,
	.gpio_dir		= 0x00000004,
	.gpio_dir_mask		= 0xFFFFFFFD,
	.gpio_data		= 0x00000004,
	.gpio_data_mask		= 0xFFFFFFFD,
};

static struct resource nuc970fb_lcd_resource[] = {
	[0] = {
		.start = NUC970_PA_LCD,
		.end   = NUC970_PA_LCD + NUC970_SZ_LCD - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_LCD,
		.end   = IRQ_LCD,
		.flags = IORESOURCE_IRQ,
	}
};

static u64 nuc970fb_device_lcd_dmamask = -1;
struct platform_device nuc970fb_device_lcd = {
	.name             = "nuc970-lcd",
	.id               = -1,
	.num_resources    = ARRAY_SIZE(nuc970fb_lcd_resource),
	.resource         = nuc970fb_lcd_resource,
	.dev              = {
		.dma_mask               = &nuc970fb_device_lcd_dmamask,
		.coherent_dma_mask      = -1,
		.platform_data 		= &nuc970fb_fb_info,
	}
};
#endif

/* SDIO Controller */
#ifdef CONFIG_MMC_NUC970_SD
static struct resource nuc970_sdh_resource[] = {
        [0] = {
                .start = NUC970_PA_SDH,
                .end   = NUC970_PA_SDH + NUC970_SZ_SDH - 1,
                .flags = IORESOURCE_MEM,
        },
        [1] = {
                .start = IRQ_SDH,
                .end   = IRQ_SDH,
                .flags = IORESOURCE_IRQ,
        }
};

static u64 nuc970_device_sdh_dmamask = 0xffffffffUL;
struct platform_device nuc970_device_sdh = {
        .name		  = "nuc970-sdh",
        .id		  = -1,
        .num_resources	  = ARRAY_SIZE(nuc970_sdh_resource),
        .resource	  = nuc970_sdh_resource,
	    .dev              = {
		.dma_mask = &nuc970_device_sdh_dmamask,
		.coherent_dma_mask = 0xffffffffUL
	}
};
#endif

/* NAND, eMMC Controller */
#ifdef CONFIG_MMC_NUC970_FMI
static struct resource nuc970_fmi_resource[] = {
        [0] = {
                .start = NUC970_PA_FMI,
                .end   = NUC970_PA_FMI + NUC970_SZ_FMI - 1,
                .flags = IORESOURCE_MEM,
        },
        [1] = {
                .start = IRQ_FMI,
                .end   = IRQ_FMI,
                .flags = IORESOURCE_IRQ,
        }
};

static u64 nuc970_device_fmi_dmamask = 0xffffffffUL;
struct platform_device nuc970_device_sdh = {
        .name		  = "nuc970-fmi",
        .id		  = -1,
        .num_resources	  = ARRAY_SIZE(nuc970_fmi_resource),
        .resource	  = nuc970_fmi_resource,
	    .dev              = {
		.dma_mask = &nuc970_device_fmi_dmamask,
		.coherent_dma_mask = 0xffffffffUL
	}
};
#endif


/* Ethernet MAC0 Controller */
#ifdef CONFIG_NUC970_ETH0
static struct resource nuc970_emac0_resource[] = {
        [0] = {
                .start = NUC970_PA_EMAC0,
                .end   = NUC970_PA_EMAC0 + NUC970_SZ_EMAC0 - 1,
                .flags = IORESOURCE_MEM,
        },
        [1] = {
                .start = IRQ_EMC0TX,
                .end   = IRQ_EMC0TX,
                .flags = IORESOURCE_IRQ,
        },
        [2] = {
                .start = IRQ_EMC0RX,
                .end   = IRQ_EMC0RX,
                .flags = IORESOURCE_IRQ,
        }
};

static u64 nuc970_device_emac0_dmamask = 0xffffffffUL;
struct platform_device nuc970_device_emac0 = {
        .name		  = "nuc970-emac0",
        .id		  = -1,
        .num_resources	  = ARRAY_SIZE(nuc970_emac0_resource),
        .resource	  = nuc970_emac0_resource,
	    .dev              = {
		.dma_mask = &nuc970_device_emac0_dmamask,
		.coherent_dma_mask = 0xffffffffUL
	}
};
#endif

#ifdef CONFIG_NUC970_ETH1
/* Ethernet MAC1 Controller */
static struct resource nuc970_emac1_resource[] = {
        [0] = {
                .start = NUC970_PA_EMAC1,
                .end   = NUC970_PA_EMAC1 + NUC970_SZ_EMAC1 - 1,
                .flags = IORESOURCE_MEM,
        },
        [1] = {
                .start = IRQ_EMC1TX,
                .end   = IRQ_EMC1TX,
                .flags = IORESOURCE_IRQ,
        },
        [2] = {
                .start = IRQ_EMC1RX,
                .end   = IRQ_EMC1RX,
                .flags = IORESOURCE_IRQ,
        }
};

static u64 nuc970_device_emac1_dmamask = 0xffffffffUL;
struct platform_device nuc970_device_emac1 = {
        .name		  = "nuc970-emac1",
        .id		  = -1,
        .num_resources	  = ARRAY_SIZE(nuc970_emac1_resource),
        .resource	  = nuc970_emac1_resource,
	    .dev              = {
		.dma_mask = &nuc970_device_emac1_dmamask,
		.coherent_dma_mask = 0xffffffffUL
	}
};
#endif

/* JPEG Controller */
#ifdef CONFIG_JPEG_CODEC
static struct resource nuc970_jpeg_resource[] = {
        [0] = {
                .start = NUC970_PA_JPEG,
                .end   = NUC970_PA_JPEG + NUC970_SZ_JPEG - 1,
                .flags = IORESOURCE_MEM,
        },
        [1] = {
                .start = IRQ_JPEG,
                .end   = IRQ_JPEG,
                .flags = IORESOURCE_IRQ,
        }
};

static u64 nuc970_device_jpeg_dmamask = 0xffffffffUL;
struct platform_device nuc970_device_jpeg = {
        .name		  = "nuc970-jpeg",
        .id		  = -1,
        .num_resources	  = ARRAY_SIZE(nuc970_jpeg_resource),
        .resource	  = nuc970_jpeg_resource,
	    .dev              = {
		.dma_mask = &nuc970_device_jpeg_dmamask,
		.coherent_dma_mask = 0xffffffffUL
	}
};
#endif

/* VIDEOIN */
#ifdef CONFIG_NUC970_VCAP
static struct resource nuc970_videoin_resource[] = {
        [0] = {
                .start = NUC970_PA_VIDEOIN,
                .end   = NUC970_PA_VIDEOIN + NUC970_SZ_VIDEOIN - 1,
                .flags = IORESOURCE_MEM,
        },
        [1] = {
                .start = IRQ_CAP,
                .end   = IRQ_CAP,
                .flags = IORESOURCE_IRQ,
        }
};

struct platform_device nuc970_device_videoin = {
        .name		  = "nuc970-videoin",
        .id		  = -1,
        .num_resources	  = ARRAY_SIZE(nuc970_videoin_resource),
        .resource	  = nuc970_videoin_resource,
};

#endif


/* AUDIO controller*/
#ifdef CONFIG_SND_SOC_NUC970
static u64 nuc970_device_audio_dmamask = -1;
static struct resource nuc970_i2s_resource[] = {
        [0] = {
                .start = NUC970_PA_ACTL,
                .end   = NUC970_PA_ACTL + NUC970_SZ_ACTL - 1,
                .flags = IORESOURCE_MEM,
        },
        [1] = {
                .start = IRQ_ACTL,
                .end   = IRQ_ACTL,
                .flags = IORESOURCE_IRQ,
        }

};

struct platform_device nuc970_device_audio_i2s = {
        .name		= "nuc970-audio-i2s",
        .id		= -1,
        .num_resources	= ARRAY_SIZE(nuc970_i2s_resource),
        .resource	= nuc970_i2s_resource,
        .dev              = {
                .dma_mask               = &nuc970_device_audio_dmamask,
                .coherent_dma_mask      = -1,
        }
};

struct platform_device nuc970_device_audio = {
	.name	= "nuc970-audio",
	.id		= -1,
    .dev    = {
                .dma_mask               = &nuc970_device_audio_dmamask,
                .coherent_dma_mask      = -1,
    }
};

struct platform_device nuc970_device_audio_pcm = {
	.name	= "nuc970-audio-pcm",
	.id		= 0,
    .dev    = {
                .dma_mask               = &nuc970_device_audio_dmamask,
                .coherent_dma_mask      = -1,
        }
};
#endif

/* I2C */
// port 0
#ifdef CONFIG_I2C
/* I2C clients */
static struct i2c_board_info __initdata nuc970_i2c_clients0[] =
{
#if defined(CONFIG_SENSOR_OV7725) || defined(CONFIG_SENSOR_OV7725_DEV1)
	{
		I2C_BOARD_INFO("ov7725", 0x21),
	},
#endif
#ifdef CONFIG_SND_SOC_NAU8822
	{
		I2C_BOARD_INFO("nau8822", 0x1a),
	},
#endif
};
#endif

#ifdef CONFIG_I2C_BUS_NUC970_P0

static struct resource nuc970_i2c0_resource[] = {
        [0] = {
                .start = NUC970_PA_I2C0,
                .end   = NUC970_PA_I2C0 + NUC970_SZ_I2C0 - 1,
                .flags = IORESOURCE_MEM,
        },
        [1] = {
                .start = IRQ_I2C0,
                .end   = IRQ_I2C0,
                .flags = IORESOURCE_IRQ,
        }
};

static struct nuc970_platform_i2c nuc970_i2c0_data = {
	.bus_num = 0,
	.bus_freq = 100000,
};

struct platform_device nuc970_device_i2c0 = {
        .name		  = "nuc970-i2c0",
        .id		  = -1,
        .num_resources	  = ARRAY_SIZE(nuc970_i2c0_resource),
        .resource	  = nuc970_i2c0_resource,
		.dev = {
        	.platform_data = &nuc970_i2c0_data,
    	}
};
#endif

#ifdef CONFIG_I2C_BUS_NUC970_P1
//port 1
static struct nuc970_platform_i2c nuc970_i2c1_data = {
	.bus_num = 1,
	.bus_freq = 100000,
};

static struct resource nuc970_i2c_p1_resource[] = {
        [0] = {
                .start = NUC970_PA_I2C1,
                .end   = NUC970_PA_I2C1+ NUC970_SZ_I2C1 - 1,
                .flags = IORESOURCE_MEM,
        },
        [1] = {
                .start = IRQ_I2C1,
                .end   = IRQ_I2C1,
                .flags = IORESOURCE_IRQ,
        }

};

struct platform_device nuc970_device_i2c1 = {
        .name		  = "nuc970-i2c1",
        .id		  = -1,
        .num_resources	  = ARRAY_SIZE(nuc970_i2c_p1_resource),
        .resource	  = nuc970_i2c_p1_resource,
        .dev = {
        	.platform_data = &nuc970_i2c1_data,
    	}
};
#endif

/* SPI */
/* spi device, spi flash info */
#ifdef CONFIG_SPI_NUC970_P0
static struct mtd_partition nuc970_spi_flash_partitions[] = {
        {
                .name = "SPI flash",
                .size = 0x0200000,
                .offset = 0,
        },
};

static struct flash_platform_data nuc970_spi_flash_data = {
        .name = "m25p80",
        .parts =  nuc970_spi_flash_partitions,
        .nr_parts = ARRAY_SIZE(nuc970_spi_flash_partitions),
        .type = "en25qh16",
};

static struct spi_board_info nuc970_spi_board_info[] __initdata = {
        {
                .modalias = "m25p80",
                .max_speed_hz = 30000000,
                .bus_num = 0,
                .chip_select = 0,
                .platform_data = &nuc970_spi_flash_data,
                .mode = SPI_MODE_0,
        },
};

static struct nuc970_spi_info nuc970_spiflash_data = {
        .num_cs		= 1,
        .lsb		= 0,
        .txneg		= 1,
        .rxneg		= 0,
        .divider	= 4,
        .sleep		= 0,
        .txnum		= 0,
        .txbitlen	= 8,
        .bus_num	= 0,
};

static struct resource nuc970_spi0_resource[] = {
        [0] = {
                .start = NUC970_PA_SPI0,
                .end   = NUC970_PA_SPI0 + NUC970_SZ_SPI0 - 1,
                .flags = IORESOURCE_MEM,
        },
        [1] = {
                .start = IRQ_SPI0,
                .end   = IRQ_SPI0,
                .flags = IORESOURCE_IRQ,
        }
};

struct platform_device nuc970_device_spi0 = {
        .name		  = "nuc970-spi0",
        .id		  = -1,
        .num_resources	  = ARRAY_SIZE(nuc970_spi0_resource),
        .resource	  = nuc970_spi0_resource,
        .dev		= {
                .platform_data = &nuc970_spiflash_data,
    }
};
#endif

#ifdef CONFIG_SPI_NUC970_P1
static struct resource nuc970_spi1_resource[] = {
        [0] = {
                .start = NUC970_PA_SPI1,
                .end   = NUC970_PA_SPI1 + NUC970_SZ_SPI1 - 1,
                .flags = IORESOURCE_MEM,
        },
        [1] = {
                .start = IRQ_SPI1,
                .end   = IRQ_SPI1,
                .flags = IORESOURCE_IRQ,
        }
};

struct platform_device nuc970_device_spi1 = {
        .name		  = "nuc970-spi1",
        .id		  = -1,
        .num_resources	  = ARRAY_SIZE(nuc970_spi1_resource),
        .resource	  = nuc970_spi1_resource,
};
#endif

#ifdef CONFIG_PWM_NUC970
static struct pwm_lookup board_pwm_lookup[] = {
	PWM_LOOKUP("nuc970-pwm", 0, "pwm-backlight", NULL),
};

static struct resource nuc970_pwm_resource[] = {
        [0] = {
                .start = NUC970_PA_PWM,
                .end   = NUC970_PA_PWM + NUC970_SZ_PWM - 1,
                .flags = IORESOURCE_MEM,
        },
        [1] = {
                .start = IRQ_PWM,
                .end   = IRQ_PWM,
                .flags = IORESOURCE_IRQ,
        }
};

//TODO: create mutiple instance for each channel
struct platform_device nuc970_device_pwm = {
        .name		  = "nuc970-pwm",
        .id		  = -1,
        .num_resources	  = ARRAY_SIZE(nuc970_pwm_resource),
        .resource	  = nuc970_pwm_resource,
};
#endif

#ifdef CONFIG_NUC970_WDT
static struct resource nuc970_wdt_resource[] = {
        [0] = {
                .start = NUC970_PA_WDT,
                .end   = NUC970_PA_WDT + NUC970_SZ_WDT - 1,
                .flags = IORESOURCE_MEM,
        },
        [1] = {
                .start = IRQ_WDT,
                .end   = IRQ_WDT,
                .flags = IORESOURCE_IRQ,
        }
};

struct platform_device nuc970_device_wdt = {
        .name		  = "nuc970-wdt",
        .id		  = -1,
        .num_resources	  = ARRAY_SIZE(nuc970_wdt_resource),
        .resource	  = nuc970_wdt_resource,
};
#endif

#ifdef CONFIG_NUC970_WWDT
static struct resource nuc970_wwdt_resource[] = {
        [0] = {
                .start = NUC970_PA_WWDT,
                .end   = NUC970_PA_WWDT + NUC970_SZ_WWDT - 1,
                .flags = IORESOURCE_MEM,
        },
        [1] = {
                .start = IRQ_WWDT,
                .end   = IRQ_WWDT,
                .flags = IORESOURCE_IRQ,
        }
};

struct platform_device nuc970_device_wwdt = {
        .name		  = "nuc970-wwdt",
        .id		  = -1,
        .num_resources	  = ARRAY_SIZE(nuc970_wwdt_resource),
        .resource	  = nuc970_wwdt_resource,
};
#endif

#ifdef CONFIG_GPIO_NUC970 
#ifdef CONFIG_I2C_ALGOBIT
static struct i2c_gpio_platform_data i2c_gpio_adapter_data = {   
    .sda_pin = NUC970_PG1,   
    .scl_pin = NUC970_PG0,   
    .udelay = 1, 
    .timeout = 100,   
    .sda_is_open_drain = 0,   //not support open drain mode
    .scl_is_open_drain = 0,   //not support open drain mode
};   
  
static struct platform_device i2c_gpio = {   
    .name = "i2c-gpio",   
    .id = 0,   
    .dev = {   
        .platform_data = &i2c_gpio_adapter_data,   
        },   
};
#endif
static struct resource nuc970_gpio_resource[] = {
	[0] = {
	       .start = NUC970_PA_GPIO,
	       .end = NUC970_PA_GPIO + NUC970_SZ_GPIO - 1,
	       .flags = IORESOURCE_MEM,
	       },
};

struct platform_device nuc970_device_gpio = {
	.name = "nuc970-gpio",
	.id = -1,
	.num_resources = ARRAY_SIZE(nuc970_gpio_resource),
	.resource = nuc970_gpio_resource,
};
   
#endif

static struct platform_device *nuc970_public_dev[] __initdata = {
        &nuc970_serial_device,
#ifdef CONFIG_USB_OHCI_HCD
        &nuc970_device_ohci,
#endif
#ifdef CONFIG_USB_EHCI_HCD
        &nuc970_device_ehci,
#endif
#ifdef CONFIG_FB_NUC970
        &nuc970fb_device_lcd,
#endif
#ifdef CONFIG_I2C_BUS_NUC970_P0
	&nuc970_device_i2c0,
#endif
#ifdef CONFIG_I2C_BUS_NUC970_P1
        &nuc970_device_i2c1,
#endif
#ifdef CONFIG_MMC_NUC970_SD
	&nuc970_device_sdh,
#endif
#ifdef CONFIG_MMC_NUC970_FMI
	&nuc970_device_fmi,
#endif
#ifdef CONFIG_JPEG_CODEC
	&nuc970_device_jpeg,
#endif
#ifdef CONFIG_NUC970_ETH0
	&nuc970_device_emac0,
#endif
#ifdef CONFIG_NUC970_ETH1
	&nuc970_device_emac1,
#endif
#ifdef CONFIG_PWM_NUC970
	&nuc970_device_pwm,
#endif
#ifdef CONFIG_NUC970_WDT
	&nuc970_device_wdt,
#endif
#ifdef CONFIG_NUC970_WWDT
	&nuc970_device_wwdt,
#endif
#ifdef CONFIG_NUC970_VCAP
	&nuc970_device_videoin,
#endif
#ifdef CONFIG_SND_SOC_NUC970
	&nuc970_device_audio_pcm,
	&nuc970_device_audio,
	&nuc970_device_audio_i2s,
#endif
#ifdef CONFIG_USBD_NUC970
	&nuc970_device_usbgadget,
#endif
#ifdef CONFIG_SPI_NUC970_P0
	&nuc970_device_spi0,
#endif
#ifdef CONFIG_SPI_NUC970_P1
	&nuc970_device_spi1,
#endif
#ifdef CONFIG_GPIO_NUC970
	&nuc970_device_gpio,
 #ifdef CONFIG_I2C_ALGOBIT
	&i2c_gpio,
 #endif
#endif
};

void __init nuc970_platform_init(struct platform_device **device, int size)
{
	platform_add_devices(device, size);
	platform_add_devices(nuc970_public_dev, ARRAY_SIZE(nuc970_public_dev));

#ifdef CONFIG_SPI_NUC970_P0
	/* register spi devices */
	spi_register_board_info(nuc970_spi_board_info, ARRAY_SIZE(nuc970_spi_board_info));
#endif

#ifdef CONFIG_I2C
	if(sizeof(nuc970_i2c_clients0))
		i2c_register_board_info(0, nuc970_i2c_clients0, sizeof(nuc970_i2c_clients0)/sizeof(struct i2c_board_info));
#endif	

#ifdef CONFIG_PWM_NUC970
	pwm_add_table(board_pwm_lookup, ARRAY_SIZE(board_pwm_lookup));
#endif
}

