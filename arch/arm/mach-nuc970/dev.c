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
#include <linux/pwm_backlight.h>

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
#include <mach/regs-lcd.h>
#include <mach/gpio.h>

#include <linux/i2c.h>
#include <linux/i2c-gpio.h>
#include <linux/i2c/i2c-hid.h>

#include <linux/platform_data/i2c-nuc970.h>
#include <linux/platform_data/video-nuc970fb.h>
#include <linux/platform_data/spi-nuc970.h>
#include <linux/platform_data/dma-nuc970.h>
#include <linux/platform_data/keypad-nuc970.h>

#include "cpu.h"

/* USB EHCI Host Controller */
#if defined(CONFIG_USB_EHCI_HCD) || defined(CONFIG_USB_EHCI_HCD_MODULE)
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
#if defined(CONFIG_USB_OHCI_HCD) || defined(CONFIG_USB_OHCI_HCD_MODULE)
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

/* Cryptographic Accelerator */
#if defined(CONFIG_CRYPTO_DEV_NUC970) || defined(CONFIG_CRYPTO_DEV_NUC970_MODULE)
static struct resource nuc970_crypto_resource[] = {
        [0] = {
                .start = NUC970_PA_CRYPTO,
                .end   = NUC970_PA_CRYPTO + NUC970_SZ_CRYPTO - 1,
                .flags = IORESOURCE_MEM,
        },
        [1] = {
                .start = IRQ_CRYPTO,
                .end   = IRQ_CRYPTO,
                .flags = IORESOURCE_IRQ,
        }
};

static u64 nuc970_device_crypto_dmamask = 0xffffffffUL;
static struct platform_device nuc970_device_crypto = {
        .name		  = "nuc970-crypto",
        .id		  = -1,
        .num_resources	  = ARRAY_SIZE(nuc970_crypto_resource),
        .resource	  = nuc970_crypto_resource,
        .dev              = {
                .dma_mask = &nuc970_device_crypto_dmamask,
                .coherent_dma_mask = 0xffffffffUL
        }
};

static struct platform_device nuc970_device_crypto_raw = {
	.name = "nuc970-crypto-raw",
	.id = -1,
	.resource = nuc970_crypto_resource,
};

static struct platform_device nuc970_device_prng = {
	.name = "nuvoton-rng",
	.id = -1,
	.resource = nuc970_crypto_resource,
};
#endif

/* USB Device (Gadget)*/
#if defined(CONFIG_USB_NUC970) || defined(CONFIG_USB_NUC970_MODULE)
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
	.name		= "nuc970-usbdev",
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
        [1] = NUC970SERIAL_PORT(UART1),
        [2] = NUC970SERIAL_PORT(UART2),
        [3] = NUC970SERIAL_PORT(UART3),
        [4] = NUC970SERIAL_PORT(UART4),
        [5] = NUC970SERIAL_PORT(UART5),
        [6] = NUC970SERIAL_PORT(UART6),
        [7] = NUC970SERIAL_PORT(UART7),
        [8] = NUC970SERIAL_PORT(UART8),
        [9] = NUC970SERIAL_PORT(UART9),
        [10] = NUC970SERIAL_PORT(UART10),
        {},
};

static struct platform_device nuc970_serial_device0 = {
        .name			= "nuc970-uart",
        .id			= 0,
        .dev			= {
                .platform_data	= &nuc970_uart_data[0],
        },
};

#if defined(CONFIG_NUC970_UART1) || defined(CONFIG_NUC970_UART1_MODULE)
static struct platform_device nuc970_serial_device1 = {
        .name			= "nuc970-uart",
        .id			= 1,
        .dev			= {
                .platform_data	= &nuc970_uart_data[1],
        },
};
#endif

#if defined(CONFIG_NUC970_UART2) || defined(CONFIG_NUC970_UART2_MODULE)
static struct platform_device nuc970_serial_device2 = {
        .name			= "nuc970-uart",
        .id			= 2,
        .dev			= {
                .platform_data	= &nuc970_uart_data[2],
        },
};
#endif

#if defined(CONFIG_NUC970_UART3) || defined(CONFIG_NUC970_UART3_MODULE)
static struct platform_device nuc970_serial_device3 = {
        .name			= "nuc970-uart",
        .id			= 3,
        .dev			= {
                .platform_data	= &nuc970_uart_data[3],
        },
};
#endif

#if defined(CONFIG_NUC970_UART4) || defined(CONFIG_NUC970_UART4_MODULE)
static struct platform_device nuc970_serial_device4 = {
        .name			= "nuc970-uart",
        .id			= 4,
        .dev			= {
                .platform_data	= &nuc970_uart_data[4],
        },
};
#endif

#if defined(CONFIG_NUC970_UART5) || defined(CONFIG_NUC970_UART5_MODULE)
static struct platform_device nuc970_serial_device5 = {
        .name			= "nuc970-uart",
        .id			= 5,
        .dev			= {
                .platform_data	= &nuc970_uart_data[5],
        },
};
#endif

#if defined(CONFIG_NUC970_UART6) || defined(CONFIG_NUC970_UART6_MODULE)
static struct platform_device nuc970_serial_device6 = {
        .name			= "nuc970-uart",
        .id			= 6,
        .dev			= {
                .platform_data	= &nuc970_uart_data[6],
        },
};
#endif

#if defined(CONFIG_NUC970_UART7) || defined(CONFIG_NUC970_UART7_MODULE)
static struct platform_device nuc970_serial_device7 = {
        .name			= "nuc970-uart",
        .id			= 7,
        .dev			= {
                .platform_data	= &nuc970_uart_data[7],
        },
};
#endif

#if defined(CONFIG_NUC970_UART8) || defined(CONFIG_NUC970_UART8_MODULE)
static struct platform_device nuc970_serial_device8 = {
        .name			= "nuc970-uart",
        .id			= 8,
        .dev			= {
                .platform_data	= &nuc970_uart_data[8],
        },
};
#endif

#if defined(CONFIG_NUC970_UART9) || defined(CONFIG_NUC970_UART9_MODULE)
static struct platform_device nuc970_serial_device9 = {
        .name			= "nuc970-uart",
        .id			= 9,
        .dev			= {
                .platform_data	= &nuc970_uart_data[9],
        },
};
#endif

#if defined(CONFIG_NUC970_UART10) || defined(CONFIG_NUC970_UART10_MODULE)
static struct platform_device nuc970_serial_device10 = {
        .name			= "nuc970-uart",
        .id			= 10,
        .dev			= {
                .platform_data	= &nuc970_uart_data[10],
        },
};
#endif

/* LCD controller*/
#if defined(CONFIG_FB_NUC970) || defined(CONFIG_FB_NUC970_MODULE)
static struct nuc970fb_display nuc970fb_lcd_info[] = {
#ifdef CONFIG_A025DL02_320X240
	/* AUO A035QN02V0 320x240 TFT Panel , 18bits*/
	[0] = {
		.type		= LCM_DCCS_VA_SRC_RGB565,
		.width		= 320,
		.height		= 240,
		.xres		= 320,
		.yres		= 240,
		.bpp		= 16,
		.pixclock	= 4000000,
		.left_margin	= 10,
		.right_margin   = 54,
		.hsync_len	= 10,
		.upper_margin	= 2,
		.lower_margin	= 4,
		.vsync_len	= 1,
		.dccs		= 0x0e00041a,
		.devctl		= 0x060800c0,
		.fbctrl		= 0x00a000a0,
		.scale		= 0x04000400,
	},
#endif

#ifdef CONFIG_FW070TFT_800X480
	/* FW070TFT 800x480 TFT Panel , 24bits*/
	[0] = {
#ifdef CONFIG_FB_SRCFMT_RGB888
		.type		= LCM_DCCS_VA_SRC_RGB888,
		.bpp		= 32,
#elif defined(CONFIG_FB_SRCFMT_RGB565)
		.type		= LCM_DCCS_VA_SRC_RGB565,
		.bpp		= 16,
#endif
		.width		= 800,
		.height		= 480,
		.xres		= 800,
		.yres		= 480,
		.pixclock	= 32000000,
		.left_margin	= 40,
		.right_margin   = 196,
		.hsync_len		= 20,
		.upper_margin	= 23,
		.lower_margin	= 19,
		.vsync_len		= 3,
#ifdef CONFIG_FB_SRCFMT_RGB888
		.dccs		= 0x0e00020a,
                .fbctrl		= 0x03200320,
#elif defined(CONFIG_FB_SRCFMT_RGB565)
		.dccs		= 0x0e00040a,
		.fbctrl		= 0x01900190,
#endif
#ifdef CONFIG_FB_LCD_16BIT_PIN
                .devctl		= 0x050000c0,
#elif defined(CONFIG_FB_LCD_18BIT_PIN)
                .devctl		= 0x060000c0,
#elif defined(CONFIG_FB_LCD_24BIT_PIN)
                .devctl		= 0x070000c0,
#endif
		.scale		= 0x04000400,
	},
#endif

#ifdef CONFIG_E50A2V1_800X480
	/* E50A2V1 800x480 TFT Panel , 24bits*/
	[0] = {
#ifdef CONFIG_FB_SRCFMT_RGB888
		.type		= LCM_DCCS_VA_SRC_RGB888,
		.bpp		= 32,
#elif defined(CONFIG_FB_SRCFMT_RGB565)
		.type   = LCM_DCCS_VA_SRC_RGB565,
		.bpp		= 16,
#endif
		.width		= 800,
		.height		= 480,
		.xres		= 800,
		.yres		= 480,
		.pixclock	= 20000000,
		.left_margin	= 88,
		.right_margin   = 40,
		.hsync_len		= 48,
		.upper_margin	= 32,
		.lower_margin	= 13,
		.vsync_len		= 3,
#ifdef CONFIG_FB_SRCFMT_RGB888
		.dccs		= 0x0e00020a,
                .fbctrl		= 0x03200320,
#elif defined(CONFIG_FB_SRCFMT_RGB565)
		.dccs		= 0x0e00040a,
		.fbctrl		= 0x01900190,
#endif
#ifdef CONFIG_FB_LCD_16BIT_PIN
                .devctl		= 0x050000c0,
#elif defined(CONFIG_FB_LCD_18BIT_PIN)
                .devctl		= 0x060000c0,
#elif defined(CONFIG_FB_LCD_24BIT_PIN)
                .devctl		= 0x070000c0,
#endif
		.scale		= 0x04000400,
	},
#endif

#ifdef CONFIG_ILI9431_MPU80_240x320
	/* ILI9431 240x320 MPU Panel , 16bits*/
	[0] = {
		.type   = LCM_DCCS_VA_SRC_RGB565,
		.bpp		= 16,
		.width		= 240,
		.height		= 320,
		.xres		= 240,
		.yres		= 320,
		.pixclock	= 6000000,
		.left_margin	= 6,
		.right_margin   = 10,
		.hsync_len		= 2,
		.upper_margin	= 27,
		.lower_margin	= 5,
		.vsync_len		= 11,
		.dccs		= 0x0e000400,
		.fbctrl		= 0x00780078,
                .devctl		= 0xC50000E0,
		.scale		= 0x04000400,
	},
#endif
};

static struct nuc970fb_mach_info nuc970fb_fb_info = {
	.displays		= &nuc970fb_lcd_info[0],
	.num_displays		= ARRAY_SIZE(nuc970fb_lcd_info),
	.default_display	= 0,
        .gpio_blen          = NUC970_PG3,
        .gpio_lcs           = NUC970_PG2,
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
#if defined(CONFIG_MMC_NUC970_SD) || defined(CONFIG_MMC_NUC970_SD_MODULE)
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
#if defined(CONFIG_MTD_NAND_NUC970) || defined(CONFIG_MTD_NAND_NUC970_SD_MODULE) || defined(CONFIG_MMC_NUC970_EMMC) || defined(CONFIG_MMC_NUC970_EMMC_MODULE)
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
struct platform_device nuc970_device_fmi = {
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
#if defined(CONFIG_NUC970_ETH0) || defined(CONFIG_NUC970_ETH0_MODULE)
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

#if defined(CONFIG_NUC970_ETH1) || defined(CONFIG_NUC970_ETH1_MODULE)
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
#if defined(CONFIG_NUC970_JPEG_CODEC) || defined(CONFIG_NUC970_JPEG_CODEC_MODULE)
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
#if defined(CONFIG_VIDEO_NUC970) || defined(CONFIG_VIDEO_NUC970_MODULE)
static struct resource nuc970_cap_resource[] = {
        [0] = {
                .start = NUC970_PA_CAP,
                .end   = NUC970_PA_CAP + NUC970_SZ_CAP - 1,
                .flags = IORESOURCE_MEM,
        },
        [1] = {
                .start = IRQ_CAP,
                .end   = IRQ_CAP,
                .flags = IORESOURCE_IRQ,
        }
};

struct platform_device nuc970_device_cap = {
        .name		  = "nuc970-videoin",
        .id		  = -1,
        .num_resources	  = ARRAY_SIZE(nuc970_cap_resource),
        .resource	  = nuc970_cap_resource,
};

#endif

/* ADC */
#if defined(CONFIG_NUC970_ADC) || defined(CONFIG_NUC970_ADC_MODULE)
static struct resource nuc970_adc_resource[] = {
        [0] = {
                .start = NUC970_PA_ADC,
                .end   = NUC970_PA_ADC + NUC970_SZ_ADC - 1,
                .flags = IORESOURCE_MEM,
        },
        [1] = {
                .start = IRQ_ADC,
                .end   = IRQ_ADC,
                .flags = IORESOURCE_IRQ,
        }
};

struct platform_device nuc970_device_adc = {
        .name		  = "nuc970-adc",
        .id		  = -1,
        .num_resources	  = ARRAY_SIZE(nuc970_adc_resource),
        .resource	  = nuc970_adc_resource,
};
#endif

/* Normal ADC */
#if defined(CONFIG_NUC970_NADC) || defined(CONFIG_NUC970_NADC_MODULE)
static struct resource nuc970_nadc_resource[] = {
        [0] = {
                .start = NUC970_PA_ADC,
                .end   = NUC970_PA_ADC + NUC970_SZ_ADC - 1,
                .flags = IORESOURCE_MEM,
        },
        [1] = {
                .start = IRQ_ADC,
                .end   = IRQ_ADC,
                .flags = IORESOURCE_IRQ,
        }
};

struct platform_device nuc970_device_nadc = {
        .name		  = "nuc970-nadc",
        .id		  = -1,
        .num_resources	  = ARRAY_SIZE(nuc970_nadc_resource),
        .resource	  = nuc970_nadc_resource,
};
#endif

#if defined(CONFIG_NUC970_DMA) || defined(CONFIG_NUC970_DMA_MODULE)
#define DMA_CHANNEL(_name, _base, _irq) \
	{ .name = (_name), .base = (_base), .irq = (_irq) }

static struct resource nuc970_gdma_resource[] = {
        [0] = {
                .start = NUC970_PA_GDMA,
                .end   = NUC970_PA_GDMA + NUC970_SZ_GDMA - 1,
                .flags = IORESOURCE_MEM,
        },
        [1] = {
                .start = IRQ_GDMA0,
                .end   = IRQ_GDMA0,
                .flags = IORESOURCE_IRQ,
        },
        [2] = {
                .start = IRQ_GDMA1,
                .end   = IRQ_GDMA1,
                .flags = IORESOURCE_IRQ,
        }
};

struct nuc970_dma_chan_data nuc970_dma_m2m_channels[] = {
	DMA_CHANNEL("m2m0", NUC970_VA_GDMA + 0x000, IRQ_GDMA0),
	DMA_CHANNEL("m2m1", NUC970_VA_GDMA + 0x020, IRQ_GDMA1),
};

struct nuc970_dma_platform_data nuc970_dma_m2m_data = {
	.channels		= nuc970_dma_m2m_channels,
	.num_channels		= ARRAY_SIZE(nuc970_dma_m2m_channels),
};


static struct platform_device nuc970_device_gdma = {
	.name			= "nuc970-dma-m2m",
	.id			= -1,
    .num_resources	= ARRAY_SIZE(nuc970_gdma_resource),
    .resource	= nuc970_gdma_resource,
	.dev			= {
	.platform_data	= &nuc970_dma_m2m_data,
	},
};
#endif


/* AUDIO controller*/
#if defined(CONFIG_SND_SOC_NUC970) || defined(CONFIG_SND_SOC_NUC970_MODULE)
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
	.name		= "nuc970-audio",
	.id		= -1,
};

struct platform_device nuc970_device_audio_pcm = {
	.name		= "nuc970-audio-pcm",
	.id		= 0,
};
#endif

/* I2C */
#if defined(CONFIG_I2C_BUS_NUC970_P0) || defined(CONFIG_I2C_BUS_NUC970_P0_MODULE)
// port 0
/* I2C clients */
static struct i2c_board_info __initdata nuc970_i2c_clients0[] =
{
#ifdef CONFIG_SND_SOC_NAU8822
	{I2C_BOARD_INFO("nau8822", 0x1a),},
#endif
};
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
#if defined(CONFIG_I2C_BUS_NUC970_P1) || defined(CONFIG_I2C_BUS_NUC970_P1_MODULE)
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
#if defined(CONFIG_SPI_NUC970_P0) || defined(CONFIG_SPI_NUC970_P0_MODULE)
/* spi device, spi flash info */
#ifdef CONFIG_MTD_M25P80
static struct mtd_partition nuc970_spi0_flash_partitions[] = {
 #ifdef CONFIG_BOARD_ETH2UART
         {
                .name = "kernel",
                .size = 0x0800000,
                .offset = 0x1000000,
        },
        {
                .name = "rootfs",
                .size = 0x0800000,
                .offset = 0x1800000,
        },
 #else
        {
                .name = "kernel",
                .size = 0x0800000,
                .offset = 0,
        },
        {
                .name = "rootfs",
                .size = 0x0800000,
                .offset = 0x0800000,
        },
 #endif
};
static struct flash_platform_data nuc970_spi0_flash_data = {
        .name = "m25p80",
        .parts =  nuc970_spi0_flash_partitions,
        .nr_parts = ARRAY_SIZE(nuc970_spi0_flash_partitions),
        .type = "w25q128",
};
#endif

static struct spi_board_info nuc970_spi0_board_info[] __initdata = {
#ifdef CONFIG_MTD_M25P80
        {
                .modalias = "m25p80",
                .max_speed_hz = 18750000,
                .bus_num = 0,
                .chip_select = 0,       //use SS0
                .platform_data = &nuc970_spi0_flash_data,
 #if defined(CONFIG_SPI_NUC970_P0_NORMAL)
                .mode = (SPI_MODE_0 | SPI_RX_DUAL | SPI_TX_DUAL),
 #elif defined(CONFIG_SPI_NUC970_P0_QUAD)
                .mode = (SPI_MODE_0 | SPI_TX_QUAD | SPI_RX_QUAD),
 #endif
        },
#endif

#ifdef CONFIG_SPI_SPIDEV
        {
                .modalias = "spidev",
                .max_speed_hz = 18750000,
                .bus_num = 0,
                .chip_select = 1,       //use SS1
                .mode = SPI_MODE_0,
        },
#endif
};

static struct nuc970_spi_info nuc970_spi0_platform_data = {
        .num_cs		= 2,
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
#if defined(CONFIG_MTD_M25P80) || defined(CONFIG_SPI_SPIDEV)
        .dev		= {
                .platform_data = &nuc970_spi0_platform_data,
    }
#endif
};
#endif

#if defined(CONFIG_SPI_NUC970_P1) || defined(CONFIG_SPI_NUC970_P1_MODULE)
/* spi device, spi flash info */
#ifdef CONFIG_BOARD_TOMATO

#ifdef CONFIG_SPI_SPIDEV
static struct spi_board_info nuc970_spi1_board_info[] __initdata = {
        {
                .modalias = "spidev",
                .max_speed_hz = 18750000,
                .bus_num = 1,
                .chip_select = 0,       //use SS0
                .mode = SPI_MODE_0,
        },
};
#endif

#else   //CONFIG_BOARD_TOMATO

#ifdef CONFIG_MTD_M25P80
static struct mtd_partition nuc970_spi1_flash_partitions[] = {
        {
                .name = "SPI flash",
                .size = 0x0200000,
                .offset = 0,
        },
};

static struct flash_platform_data nuc970_spi1_flash_data = {
        .name = "m25p80",
        .parts =  nuc970_spi1_flash_partitions,
        .nr_parts = ARRAY_SIZE(nuc970_spi1_flash_partitions),
        .type = "en25qh16",
};
#endif

static struct spi_board_info nuc970_spi1_board_info[] __initdata = {
#ifdef CONFIG_MTD_M25P80
        {
                .modalias = "m25p80",
                .max_speed_hz = 18750000,
                .bus_num = 1,
                .chip_select = 0,       //use SS0
                .platform_data = &nuc970_spi1_flash_data,
                .mode = SPI_MODE_0,
        },
#endif
#ifdef CONFIG_SPI_SPIDEV
        {
                .modalias = "spidev",
                .max_speed_hz = 18750000,
                .bus_num = 1,
                .chip_select = 1,       //use SS1
                .mode = SPI_MODE_0,
        },
#endif
};
#endif

static struct nuc970_spi_info nuc970_spi1_platform_data = {
        .num_cs		= 2,
        .lsb		= 0,
        .txneg		= 1,
        .rxneg		= 0,
        .divider	= 4,
        .sleep		= 0,
        .txnum		= 0,
        .txbitlen	= 8,
        .bus_num	= 1,
};

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
#if defined(CONFIG_MTD_M25P80) || defined(CONFIG_SPI_SPIDEV)
        .dev		= {
                .platform_data = &nuc970_spi1_platform_data,
		}
#endif
};
#endif

#if defined(CONFIG_KEYBOARD_NUC970) || defined(CONFIG_KEYBOARD_NUC970_MODULE)
static int nuc970_keymap[] = {
	KEY(0, 0, KEY_A),	KEY(0, 1, KEY_B),
	KEY(0, 2, KEY_C),	KEY(0, 3, KEY_D),
	KEY(0, 4, KEY_E),	KEY(0, 5, KEY_F),
	KEY(0, 6, KEY_G),	KEY(0, 7, KEY_H),

	KEY(1, 0, KEY_I),	KEY(1, 1, KEY_J),
	KEY(1, 2, KEY_K),	KEY(1, 3, KEY_L),
	KEY(1, 4, KEY_M),	KEY(1, 5, KEY_N),
	KEY(1, 6, KEY_O),	KEY(1, 7, KEY_P),

	KEY(2, 0, KEY_Q),	KEY(2, 1, KEY_R),
	KEY(2, 2, KEY_S),	KEY(2, 3, KEY_T),
	KEY(2, 4, KEY_U),	KEY(2, 5, KEY_V),
	KEY(2, 6, KEY_W),	KEY(2, 7, KEY_X),

	KEY(3, 0, KEY_Y),	KEY(3, 1, KEY_Z),
	KEY(3, 2, KEY_1),	KEY(3, 3, KEY_2),
	KEY(3, 4, KEY_3),	KEY(3, 5, KEY_4),
	KEY(3, 6, KEY_5),	KEY(3, 7, KEY_6),
};

static struct matrix_keymap_data nuc970_map_data = {
	.keymap			= nuc970_keymap,
	.keymap_size	= ARRAY_SIZE(nuc970_keymap),
};

static struct nuc970_keypad_platform_data nuc970_keypad_info = {
		.keymap_data	= &nuc970_map_data,
        .prescale		= 0x80,
        .debounce		= 0x8,
};


static struct resource nuc970_kpi_resource[] = {
	[0] = {
			.start = NUC970_PA_KPI,
			.end   = NUC970_PA_KPI + NUC970_SZ_KPI - 1,
			.flags = IORESOURCE_MEM,
	},
	[1] = {
			.start = IRQ_KPI,
			.end   = IRQ_KPI,
			.flags = IORESOURCE_IRQ,
	}
};

struct platform_device nuc970_device_kpi = {
        .name		  = "nuc970-kpi",
        .id		  = -1,
        .num_resources	  = ARRAY_SIZE(nuc970_kpi_resource),
        .resource	  = nuc970_kpi_resource,

		.dev		= {
                .platform_data = &nuc970_keypad_info,
		}
};
#endif

#if defined(CONFIG_RTC_DRV_NUC970) || defined(CONFIG_RTC_DRV_NUC970_MODULE)
static struct resource nuc970_rtc_resource[] = {
        [0] = {
                .start = NUC970_PA_RTC,
                .end   = NUC970_PA_RTC + NUC970_SZ_RTC - 1,
                .flags = IORESOURCE_MEM,
        },
        [1] = {
                .start = IRQ_RTC,
                .end   = IRQ_RTC,
                .flags = IORESOURCE_IRQ,
        },
};

struct platform_device nuc970_device_rtc = {
        .name		= "nuc970-rtc",
        .id		= -1,
        .num_resources	= ARRAY_SIZE(nuc970_rtc_resource),
        .resource	= nuc970_rtc_resource,
};
#endif

#if defined(CONFIG_NUC970_CAN0) || defined(CONFIG_NUC970_CAN0_MODULE)
static struct resource nuc970_can0_resource[] = {
        [0] = {
                .start = NUC970_PA_CAN,
                .end   = NUC970_PA_CAN + NUC970_SZ_CAN - 1,
                .flags = IORESOURCE_MEM,
        },
        [1] = {
                .start = IRQ_CAN0,
                .end   = IRQ_CAN0,
                .flags = IORESOURCE_IRQ,
        },
};

struct platform_device nuc970_device_can0 = {
        .name		= "nuc970-can0",
        .id		= -1,
        .num_resources	= ARRAY_SIZE(nuc970_can0_resource),
        .resource	= nuc970_can0_resource,
};
#endif

#if defined(CONFIG_NUC970_CAN1) || defined(CONFIG_NUC970_CAN1_MODULE)
static struct resource nuc970_can1_resource[] = {
        [0] = {
                .start = NUC970_PA_CAN1,
                .end   = NUC970_PA_CAN1 + NUC970_SZ_CAN1 - 1,
                .flags = IORESOURCE_MEM,
        },
        [1] = {
                .start = IRQ_CAN1,
                .end   = IRQ_CAN1,
                .flags = IORESOURCE_IRQ,
        },
};

struct platform_device nuc970_device_can1 = {
        .name		= "nuc970-can1",
        .id		= -1,
        .num_resources	= ARRAY_SIZE(nuc970_can1_resource),
        .resource	= nuc970_can1_resource,
};
#endif

#if defined(CONFIG_NUC970_EBI) || defined(CONFIG_NUC970_EBI_MODULE)
static struct resource nuc970_ebi_resource[] = {
        [0] = {
                .start = NUC970_PA_EBI,
                .end   = NUC970_PA_EBI + NUC970_SZ_EBI - 1,
                .flags = IORESOURCE_MEM,
        },
};

struct platform_device nuc970_device_ebi = {
        .name           = "nuc970-ebi",
        .id             = -1,
        .num_resources  = ARRAY_SIZE(nuc970_ebi_resource),
        .resource       = nuc970_ebi_resource,
};
#endif

#if defined(CONFIG_PWM_NUC970) || defined(CONFIG_PWM_NUC970_MODULE)
static struct pwm_lookup board_pwm_lookup[] = {
	PWM_LOOKUP("nuc970-pwm", 0, "pwm-backlight", NULL),
};

#if 0
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
#endif

struct platform_device nuc970_device_pwm0 = {
        .name		  = "nuc970-pwm",
        .id		  = 0,
};
struct platform_device nuc970_device_pwm1 = {
        .name		  = "nuc970-pwm",
        .id		  = 1,
};
struct platform_device nuc970_device_pwm2 = {
        .name		  = "nuc970-pwm",
        .id		  = 2,
};
struct platform_device nuc970_device_pwm3 = {
        .name		  = "nuc970-pwm",
        .id		  = 3,
};
#endif

#if defined(CONFIG_NUC970_WDT) || defined(CONFIG_NUC970_WDT_MODULE)
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

#if defined(CONFIG_NUC970_WWDT) || defined(CONFIG_NUC970_WWDT_MODULE)
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

#if defined(CONFIG_SCUART_NUC970) || defined(CONFIG_SCUART_NUC970_MODULE)
/* Initial serial platform data */
static struct plat_nuc970serial_port nuc970_scuart_data[] = {
        [0] = {
		.membase	= NUC970_VA_SC,
		.mapbase	= NUC970_PA_SC,
		.irq 		= IRQ_SMC0,
		.uartclk 	= 12000000,

        },
        [1] = {
		.membase	= (NUC970_VA_SC + 0x400),
		.mapbase	= (NUC970_PA_SC + 0x400),
		.irq 		= IRQ_SMC1,
		.uartclk 	= 12000000,

        },
        {},
};
#endif

#if defined(CONFIG_NUC970_SC) || defined(CONFIG_NUC970_SC_MODULE) || defined(CONFIG_SCUART_NUC970) || defined(CONFIG_SCUART_NUC970_MODULE)

#if defined(CONFIG_NUC970_SC) || defined(CONFIG_NUC970_SC_MODULE)
static struct resource nuc970_sc0_resource[] = {
        [0] = {
                .start = NUC970_PA_SC,
                .end   = NUC970_PA_SC + 0x400 - 1,
                .flags = IORESOURCE_MEM,
        },
        [1] = {
                .start = IRQ_SMC0,
                .end   = IRQ_SMC0,
                .flags = IORESOURCE_IRQ,
        },
};
static struct resource nuc970_sc1_resource[] = {
        [0] = {
                .start = NUC970_PA_SC + 0x400,
                .end   = NUC970_PA_SC + 0x800 - 1,
                .flags = IORESOURCE_MEM,
        },
        [1] = {
                .start = IRQ_SMC1,
                .end   = IRQ_SMC1,
                .flags = IORESOURCE_IRQ,
        },
};
#endif

struct platform_device nuc970_device_sc0 = {
        .name		  = "nuc970-sc",
        .id		  = 0,
#if defined(CONFIG_NUC970_SC) || defined(CONFIG_NUC970_SC_MODULE)
        .num_resources	  = ARRAY_SIZE(nuc970_sc0_resource),
        .resource	  = nuc970_sc0_resource,
#endif
#if defined(CONFIG_SCUART_NUC970) || defined(CONFIG_SCUART_NUC970_MODULE)
        .dev			= {
                .platform_data	= &nuc970_scuart_data[0],
        },
#endif
};
struct platform_device nuc970_device_sc1 = {
        .name		  = "nuc970-sc",
        .id		  = 1,
#if defined(CONFIG_NUC970_SC) || defined(CONFIG_NUC970_SC_MODULE)
        .num_resources	  = ARRAY_SIZE(nuc970_sc1_resource),
        .resource	  = nuc970_sc1_resource,
#endif
#if defined(CONFIG_SCUART_NUC970) || defined(CONFIG_SCUART_NUC970_MODULE)
        .dev			= {
                .platform_data	= &nuc970_scuart_data[1],
        },
#endif
};

#endif


#if defined(CONFIG_NUC970_ETIMER) || defined(CONFIG_NUC970_ETIMER_MODULE)
static struct resource nuc970_etimer_resource[] = {
        [0] = {
                .start = IRQ_ETIMER0,
                .end   = IRQ_ETIMER0,
                .flags = IORESOURCE_IRQ,
        },
        [1] = {
                .start = IRQ_ETIMER1,
                .end   = IRQ_ETIMER1,
                .flags = IORESOURCE_IRQ,
        },
        [2] = {
                .start = IRQ_ETIMER2,
                .end   = IRQ_ETIMER2,
                .flags = IORESOURCE_IRQ,
        },
        [3] = {
                .start = IRQ_ETIMER3,
                .end   = IRQ_ETIMER3,
                .flags = IORESOURCE_IRQ,
        },
};

struct platform_device nuc970_device_etimer0 = {
        .name		  = "nuc970-etimer",
        .id		  = 0,
        .num_resources	  = ARRAY_SIZE(nuc970_etimer_resource),
        .resource	  = nuc970_etimer_resource,
};
struct platform_device nuc970_device_etimer1 = {
        .name		  = "nuc970-etimer",
        .id		  = 1,
        .num_resources	  = ARRAY_SIZE(nuc970_etimer_resource),
        .resource	  = nuc970_etimer_resource,
};
struct platform_device nuc970_device_etimer2 = {
        .name		  = "nuc970-etimer",
        .id		  = 2,
        .num_resources	  = ARRAY_SIZE(nuc970_etimer_resource),
        .resource	  = nuc970_etimer_resource,
};
struct platform_device nuc970_device_etimer3 = {
        .name		  = "nuc970-etimer",
        .id		  = 3,
        .num_resources	  = ARRAY_SIZE(nuc970_etimer_resource),
        .resource	  = nuc970_etimer_resource,
};
#endif
#if defined(CONFIG_PINCTRL) || defined(CONFIG_PINCTRL_MODULE)
struct platform_device nuc970_device_pinctrl = {
        .name		  = "pinctrl-nuc970",
        .id		  = -1,
};
#endif

#if defined(CONFIG_GPIO_NUC970) || defined(CONFIG_GPIO_NUC970_MODULE)
#if defined(CONFIG_I2C_ALGOBIT) || defined(CONFIG_I2C_ALGOBIT_MODULE)
static struct i2c_board_info __initdata nuc970_i2c_clients2[] =
{
#ifdef CONFIG_SENSOR_OV7725
	{I2C_BOARD_INFO("ov7725",  0x21),},
#endif
#ifdef CONFIG_SENSOR_OV5640
	{I2C_BOARD_INFO("ov5640",  0x3c),},
#endif
#ifdef CONFIG_SENSOR_NT99141
	{I2C_BOARD_INFO("nt99141", 0x2a),},
#endif
#ifdef CONFIG_SENSOR_NT99050
	{I2C_BOARD_INFO("nt99050", 0x21),},
#endif
};
static struct i2c_gpio_platform_data i2c_gpio_adapter_data = {
    .sda_pin = NUC970_PB1,
    .scl_pin = NUC970_PB0,
    .udelay = 1,
    .timeout = 100,
    .sda_is_open_drain = 0,   //not support open drain mode
    .scl_is_open_drain = 0,   //not support open drain mode
};

static struct platform_device i2c_gpio = {
    .name = "i2c-gpio",
    .id = 2,
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
	.id = 0,
	.num_resources = ARRAY_SIZE(nuc970_gpio_resource),
	.resource = nuc970_gpio_resource,
};

#ifndef CONFIG_OF
struct platform_device nuc970_device_eint0 = {
        .name		  = "nuc970-gpio",
        .id		  = 1,
};

struct platform_device nuc970_device_eint1 = {
        .name		  = "nuc970-gpio",
        .id		  = 2,
};

struct platform_device nuc970_device_eint2 = {
        .name		  = "nuc970-gpio",
        .id		  = 3,
};

struct platform_device nuc970_device_eint3 = {
        .name		  = "nuc970-gpio",
        .id		  = 4,
};

struct platform_device nuc970_device_eint4 = {
        .name		  = "nuc970-gpio",
        .id		  = 5,
};

struct platform_device nuc970_device_eint5 = {
        .name		  = "nuc970-gpio",
        .id		  = 6,
};

struct platform_device nuc970_device_eint6 = {
        .name		  = "nuc970-gpio",
        .id		  = 7,
};

struct platform_device nuc970_device_eint7 = {
        .name		  = "nuc970-gpio",
        .id		  = 8,
};
#endif
#endif

#if defined(CONFIG_NUC970_GE2D) || defined(CONFIG_NUC970_GE2D_MODULE)
static struct resource nuc970_ge2d_resource[] = {
        [0] = {
                .start = NUC970_PA_GE,
                .end   = NUC970_PA_GE + NUC970_SZ_GE - 1,
                .flags = IORESOURCE_MEM,
        },
        [1] = {
                .start = IRQ_GE2D,
                .end   = IRQ_GE2D,
                .flags = IORESOURCE_IRQ,
        }
};

struct platform_device nuc970_device_ge2d = {
        .name		  = "nuc970-ge2d",
        .id		  = -1,
        .num_resources	  = ARRAY_SIZE(nuc970_ge2d_resource),
        .resource	  = nuc970_ge2d_resource,
};
#endif

#if defined(CONFIG_IIO_GPIO_TRIGGER) || defined(CONFIG_IIO_GPIO_TRIGGER_MODULE)
static struct resource iio_gpio_trigger_resources[] = {
	[0] = {
		.start  = IRQ_GPIO_START+NUC970_PE2,
		.end    = IRQ_GPIO_START+NUC970_PE2,
		.flags  = IORESOURCE_IRQ | IORESOURCE_IRQ_LOWEDGE,
	},
};

static struct platform_device iio_gpio_trigger = {
	.name = "iio_gpio_trigger",
	.num_resources = ARRAY_SIZE(iio_gpio_trigger_resources),
	.resource = iio_gpio_trigger_resources,
};
#endif
#if defined(CONFIG_BACKLIGHT_PWM)
static struct platform_pwm_backlight_data nuc970_backlight_data = {
	.pwm_id		= 0,
	.max_brightness	= 1,
	.dft_brightness	= 1,
	.pwm_period_ns	= 78000,
};

struct platform_device nuc970_pwm_bl = {
        .name		  = "pwm-backlight",
	.dev		= {
		.platform_data = &nuc970_backlight_data,
	},
};
#endif
static struct platform_device *nuc970_public_dev[] __initdata = {
        &nuc970_serial_device0,

#if defined(CONFIG_NUC970_UART1) || defined(CONFIG_NUC970_UART1_MODULE)
		&nuc970_serial_device1,
#endif

#if defined(CONFIG_NUC970_UART2) || defined(CONFIG_NUC970_UART2_MODULE)
		&nuc970_serial_device2,
#endif

#if defined(CONFIG_NUC970_UART3) || defined(CONFIG_NUC970_UART3_MODULE)
		&nuc970_serial_device3,
#endif

#if defined(CONFIG_NUC970_UART4) || defined(CONFIG_NUC970_UART4_MODULE)
		&nuc970_serial_device4,
#endif

#if defined(CONFIG_NUC970_UART5) || defined(CONFIG_NUC970_UART5_MODULE)
		&nuc970_serial_device5,
#endif

#if defined(CONFIG_NUC970_UART6) || defined(CONFIG_NUC970_UART6_MODULE)
		&nuc970_serial_device6,
#endif

#if defined(CONFIG_NUC970_UART7) || defined(CONFIG_NUC970_UART7_MODULE)
		&nuc970_serial_device7,
#endif

#if defined(CONFIG_NUC970_UART8) || defined(CONFIG_NUC970_UART8_MODULE)
		&nuc970_serial_device8,
#endif

#if defined(CONFIG_NUC970_UART9) || defined(CONFIG_NUC970_UART9_MODULE)
		&nuc970_serial_device9,
#endif

#if defined(CONFIG_NUC970_UART10) || defined(CONFIG_NUC970_UART10_MODULE)
		&nuc970_serial_device10,
#endif

#if defined(CONFIG_NUC970_CAN0) || defined(CONFIG_NUC970_CAN0_MODULE)
	&nuc970_device_can0,
#endif

#if defined(CONFIG_NUC970_CAN1) || defined(CONFIG_NUC970_CAN1_MODULE)
	&nuc970_device_can1,
#endif

#if defined(CONFIG_NUC970_EBI) || defined(CONFIG_NUC970_EBI_MODULE)
	&nuc970_device_ebi,
#endif

#if defined(CONFIG_USB_OHCI_HCD) || defined(CONFIG_USB_OHCI_HCD_MODULE)
        &nuc970_device_ohci,
#endif
#if defined(CONFIG_USB_EHCI_HCD) || defined(CONFIG_USB_EHCI_HCD_MODULE)
        &nuc970_device_ehci,
#endif
#if defined(CONFIG_CRYPTO_DEV_NUC970) || defined(CONFIG_CRYPTO_DEV_NUC970_MODULE)
		&nuc970_device_crypto,
		&nuc970_device_crypto_raw,
		&nuc970_device_prng,
#endif
#if defined(CONFIG_FB_NUC970) || defined(CONFIG_FB_NUC970_MODULE)
        &nuc970fb_device_lcd,
#endif
#if defined(CONFIG_I2C_BUS_NUC970_P0) || defined(CONFIG_I2C_BUS_NUC970_P0_MODULE)
	&nuc970_device_i2c0,
#endif
#if defined(CONFIG_I2C_BUS_NUC970_P1) || defined(CONFIG_I2C_BUS_NUC970_P1_MODULE)
        &nuc970_device_i2c1,
#endif
#if defined(CONFIG_NUC970_DMA) || defined(CONFIG_NUC970_DMA_MODULE)
	&nuc970_device_gdma,
#endif

#if defined(CONFIG_MMC_NUC970_SD) || defined(CONFIG_MMC_NUC970_SD_MODULE)
	&nuc970_device_sdh,
#endif
#if defined(CONFIG_MTD_NAND_NUC970) || defined(CONFIG_MTD_NAND_NUC970_MODULE) || defined(CONFIG_MMC_NUC970_EMMC) || defined(CONFIG_MMC_NUC970_EMMC_MODULE)
	&nuc970_device_fmi,
#endif
#if defined(CONFIG_NUC970_JPEG_CODEC) || defined(CONFIG_NUC970_JPEG_CODEC_MODULE)
	&nuc970_device_jpeg,
#endif
#if defined(CONFIG_NUC970_ETH0) || defined(CONFIG_NUC970_ETH0_MODULE)
	&nuc970_device_emac0,
#endif
#if defined(CONFIG_NUC970_ETH1) || defined(CONFIG_NUC970_ETH1_MODULE)
	&nuc970_device_emac1,
#endif
#if defined(CONFIG_PWM_NUC970) || defined(CONFIG_PWM_NUC970_MODULE)
	&nuc970_device_pwm0,
	&nuc970_device_pwm1,
	&nuc970_device_pwm2,
	&nuc970_device_pwm3,
#endif
#if defined(CONFIG_NUC970_WDT) || defined(CONFIG_NUC970_WDT_MODULE)
	&nuc970_device_wdt,
#endif
#if defined(CONFIG_NUC970_WWDT) || defined(CONFIG_NUC970_WWDT_MODULE)
	&nuc970_device_wwdt,
#endif
#if defined(CONFIG_VIDEO_NUC970) || defined(CONFIG_VIDEO_NUC970_MODULE)
	&nuc970_device_cap,
#endif
#if defined(CONFIG_SND_SOC_NUC970) || defined(CONFIG_SND_SOC_NUC970_MODULE)
	&nuc970_device_audio_pcm,
	&nuc970_device_audio,
	&nuc970_device_audio_i2s,
#endif
#if defined(CONFIG_USB_NUC970) || defined(CONFIG_USB_NUC970_MODULE)
	&nuc970_device_usbgadget,
#endif
#if defined(CONFIG_SPI_NUC970_P0) || defined(CONFIG_SPI_NUC970_P0_MODULE)
	&nuc970_device_spi0,
#endif
#if defined(CONFIG_SPI_NUC970_P1) || defined(CONFIG_SPI_NUC970_P1_MODULE)
	&nuc970_device_spi1,
#endif

#if defined(CONFIG_NUC970_ADC) || defined(CONFIG_NUC970_ADC_MODULE)
	&nuc970_device_adc,
#endif
#if defined(CONFIG_NUC970_NADC) || defined(CONFIG_NUC970_NADC_MODULE)
	&nuc970_device_nadc,
#endif

#if defined(CONFIG_NUC970_ETIMER) || defined(CONFIG_NUC970_ETIMER_MODULE)
	&nuc970_device_etimer0,
	&nuc970_device_etimer1,
	&nuc970_device_etimer2,
	&nuc970_device_etimer3,
#endif
#if defined(CONFIG_PINCTRL) || defined(CONFIG_PINCTRL_MODULE)
	&nuc970_device_pinctrl,
#endif
#if defined(CONFIG_KEYBOARD_NUC970) || defined(CONFIG_KEYBOARD_NUC970_MODULE)
	&nuc970_device_kpi,
#endif
#if defined(CONFIG_RTC_DRV_NUC970) || defined(CONFIG_RTC_DRV_NUC970_MODULE)
	&nuc970_device_rtc,
#endif
#if defined(CONFIG_GPIO_NUC970) || defined(CONFIG_GPIO_NUC970_MODULE)
	&nuc970_device_gpio,
	#ifndef CONFIG_OF
	&nuc970_device_eint0,
	&nuc970_device_eint1,
	&nuc970_device_eint2,
	&nuc970_device_eint3,
	&nuc970_device_eint4,
	&nuc970_device_eint5,
	&nuc970_device_eint6,
	&nuc970_device_eint7,
	#endif
#endif
#if defined(CONFIG_I2C_ALGOBIT) || defined(CONFIG_I2C_ALGOBIT_MODULE)
	&i2c_gpio,
#endif
#if defined(CONFIG_NUC970_SC) || defined(CONFIG_NUC970_SC_MODULE) || defined(CONFIG_SCUART_NUC970) || defined(CONFIG_SCUART_NUC970_MODULE)
	&nuc970_device_sc0,
	&nuc970_device_sc1,
#endif

#if defined(CONFIG_NUC970_GE2D) || defined(CONFIG_NUC970_GE2D_MODULE)
    &nuc970_device_ge2d,
#endif

#if defined(CONFIG_IIO_GPIO_TRIGGER) || defined(CONFIG_IIO_GPIO_TRIGGER_MODULE)
    &iio_gpio_trigger,
#endif
#if defined(CONFIG_BACKLIGHT_PWM)
    &nuc970_pwm_bl,
#endif
};


void __init nuc970_platform_init(struct platform_device **device, int size)
{
	platform_add_devices(device, size);
	platform_add_devices(nuc970_public_dev, ARRAY_SIZE(nuc970_public_dev));

#if defined(CONFIG_MTD_M25P80) || defined(CONFIG_SPI_SPIDEV) || defined(CONFIG_SPI_SPIDEV_MODULE)
 	/* register spi devices */
#if defined(CONFIG_SPI_NUC970_P0) || defined(CONFIG_SPI_NUC970_P0_MODULE)
	spi_register_board_info(nuc970_spi0_board_info, ARRAY_SIZE(nuc970_spi0_board_info));
#endif
#if defined(CONFIG_SPI_NUC970_P1) || defined(CONFIG_SPI_NUC970_P1_MODULE)
    spi_register_board_info(nuc970_spi1_board_info, ARRAY_SIZE(nuc970_spi1_board_info));
#endif
#endif

#if defined(CONFIG_I2C_BUS_NUC970_P0) || defined(CONFIG_I2C_BUS_NUC970_P0_MODULE)
	i2c_register_board_info(0, nuc970_i2c_clients0, sizeof(nuc970_i2c_clients0)/sizeof(struct i2c_board_info));
#endif

#if defined(CONFIG_GPIO_NUC970) || defined(CONFIG_GPIO_NUC970_MODULE)
#if defined(CONFIG_I2C_ALGOBIT) || defined(CONFIG_I2C_ALGOBIT_MODULE)
	i2c_register_board_info(2, nuc970_i2c_clients2, sizeof(nuc970_i2c_clients2)/sizeof(struct i2c_board_info));
#endif
#endif

#if defined(CONFIG_PWM_NUC970) || defined(CONFIG_PWM_NUC970_MODULE)
	pwm_add_table(board_pwm_lookup, ARRAY_SIZE(board_pwm_lookup));
#endif
}
