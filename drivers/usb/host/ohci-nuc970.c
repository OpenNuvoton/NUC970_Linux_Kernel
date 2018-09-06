/*
 * linux/driver/usb/host/ohci-nuc970.c
 *
 * Copyright (c) 2012 Nuvoton technology corporation.
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation;version 2 of the License.
 *
 */

#include <linux/platform_device.h>
#include <linux/signal.h>
#include <linux/of.h>
#include <mach/regs-gcr.h>
#include <mach/regs-gpio.h>
#include <linux/clk.h>

#include <mach/map.h>

static int  of_pm_vbus_off;
static int  of_mfp_setting;   /* D+/D- multi-function pin setting - 0: not used; 1: PE.14/PE.15; 2: PF.10  */

/**
 * usb_hcd_ppc_soc_probe - initialize On-Chip HCDs
 * Context: !in_interrupt()
 *
 * Allocates basic resources for this USB host controller.
 *
 * Store this function in the HCD's struct pci_driver as probe().
 */
static int usb_hcd_nuc970_probe(const struct hc_driver *driver,
                                struct platform_device *pdev)
{
        int retval;
        struct usb_hcd *hcd;
        struct ohci_hcd *ohci ;
		struct clk *clkmux, *clkaplldiv, *clkapll, *clkusb;
#ifdef CONFIG_OF
	    u32   val32[2];
#endif
		int ret;

        if (IS_ERR(clk_get(NULL, "usbh_hclk"))) {
                printk("clk_get error!!\n");
                return -1;
        }

        clk_prepare(clk_get(NULL, "usb_eclk"));
        clk_enable(clk_get(NULL, "usb_eclk"));

        /* enable USB Host clock */
        clk_prepare(clk_get(NULL, "usbh_hclk"));
        clk_enable(clk_get(NULL, "usbh_hclk"));

#ifdef CONFIG_OF

        devm_pinctrl_get_select_default(&pdev->dev);

		if ((__raw_readl(REG_MFP_GPE_H) & 0xFF000000) == 0x77000000)
			of_mfp_setting = 1;
		else if ((__raw_readl(REG_MFP_GPF_H) & 0x00000F00) == 0x00000700)
			of_mfp_setting = 2;
		else
			of_mfp_setting = 0;

		//printk("of_mfp_setting = %d\n", of_mfp_setting);

		if (of_property_read_u32_array(pdev->dev.of_node, "ov_active", val32, 1) == 0)
		{
			// printk("Over-current active level %s...\n", val32[0] ? "high" : "low");
			if (val32[0])
			{
        		/* set over-current active high */
        		__raw_writel(__raw_readl(NUC970_VA_OHCI+0x204) &~0x8, (volatile void __iomem *)(NUC970_VA_OHCI+0x204));
        	}
        	else
        	{
        		/* set over-current active low */
        		__raw_writel(__raw_readl(NUC970_VA_OHCI+0x204) | 0x8, (volatile void __iomem *)(NUC970_VA_OHCI+0x204));
        	}
        }

		if (of_property_read_u32_array(pdev->dev.of_node, "pm_vbus_off", val32, 1) == 0)
		{
			if (val32[0])
				of_pm_vbus_off = 1;
			else
				of_pm_vbus_off = 0;
		}
		else
		{
		    of_pm_vbus_off = 0;
		}

		/*
	 	 * Right now device-tree probed devices don't get dma_mask set.
	 	 * Since shared usb code relies on it, set it here for now.
	 	 * Once we have dma capability bindings this can go away.
	 	 */
		if (!pdev->dev.dma_mask)
		 	pdev->dev.dma_mask = &pdev->dev.coherent_dma_mask;
		if (!pdev->dev.coherent_dma_mask)
			pdev->dev.coherent_dma_mask = DMA_BIT_MASK(32);

#else

#if !defined(CONFIG_USB_NUC970_EHCI)

		/* multi-function pin select */
#if defined (CONFIG_NUC970_USBH_PWR_PE_2)
        /* set over-current active low */
        __raw_writel(__raw_readl(NUC970_VA_OHCI+0x204) | 0x8, (volatile void __iomem *)(NUC970_VA_OHCI+0x204));

    	/* initial USBH_PPWR0 & USBH_PPWR1 pin -> PE.14 & PE.15 */
    	p = devm_pinctrl_get_select(&pdev->dev, "usbh-ppwr-pe");
    	if (IS_ERR(p))
    	{
        	dev_err(&pdev->dev, "unable to reserve pin\n");
        	retval = PTR_ERR(p);
    	}
#elif defined (CONFIG_NUC970_USBH_PWR_PF_2)
        /* set over-current active low */
        __raw_writel(__raw_readl(NUC970_VA_OHCI+0x204) | 0x8, (volatile void __iomem *)(NUC970_VA_OHCI+0x204));

    	/* initial USBH_PPWR pin -> PF.10 */
    	p = devm_pinctrl_get_select(&pdev->dev, "usbh-ppwr-pf");
    	if (IS_ERR(p))
    	{
        	dev_err(&pdev->dev, "unable to reserve pin\n");
        	retval = PTR_ERR(p);
    	}
#elif defined (CONFIG_NUC970_USBH_OC_ONLY_2)
        /* set over-current active low */
        __raw_writel(__raw_readl(NUC970_VA_OHCI+0x204) | 0x8, (volatile void __iomem *)(NUC970_VA_OHCI+0x204));

    	p = devm_pinctrl_get_select(&pdev->dev, "usbh-ppwr-oc");
    	if (IS_ERR(p))
    	{
        	dev_err(&pdev->dev, "unable to reserve pin\n");
        	retval = PTR_ERR(p);
    	}
#else  //  CONFIG_NUC970_USBH_NONE_2
        /* set over-current active high */
        __raw_writel(__raw_readl(NUC970_VA_OHCI+0x204) &~0x8, (volatile void __iomem *)(NUC970_VA_OHCI+0x204));
#endif

#endif   // !CONFIG_USB_NUC970_EHCI

#endif   // CONFIG_OF

		clkmux = clk_get(NULL, "usb_eclk_mux");
        if (IS_ERR(clkmux)) {
			printk(KERN_ERR "nuc970-usb:failed to get usb clock source\n");
			ret = PTR_ERR(clkmux);
			return ret;
		}

		/* Set APLL output 96 MHz, select 48 MHz for OHCI */
		clkaplldiv = clk_get(NULL, "usb_aplldiv");
        if (IS_ERR(clkaplldiv)) {
			printk(KERN_ERR "nuc970-usb:failed to get usb clock source\n");
			ret = PTR_ERR(clkaplldiv);
			return ret;
		}

		clkapll = clk_get(NULL, "apll");
        if (IS_ERR(clkapll)) {
			printk(KERN_ERR "nuc970-usb:failed to get usb clock source\n");
			ret = PTR_ERR(clkapll);
			return ret;
		}

		clkusb = clk_get(NULL, "usb_eclk");
        if (IS_ERR(clkusb)) {
			printk(KERN_ERR "nuc970-usb:failed to get usb clock source\n");
			ret = PTR_ERR(clkusb);
			return ret;
		}

		clk_prepare(clkusb);
        clk_enable(clkusb);

        clk_set_parent(clkmux, clkaplldiv);

        clk_set_rate(clkapll, 96000000);
		clk_set_rate(clkusb, 48000000);

        /* enable PHY 0/1 */
		__raw_writel(0x160, (volatile void __iomem *)(NUC970_VA_EHCI+0xC4));
		__raw_writel(0x520, (volatile void __iomem *)(NUC970_VA_EHCI+0xC8));

        hcd = usb_create_hcd(driver, &pdev->dev, "nuc970-ohci");
        if (!hcd)
                return -ENOMEM;

        hcd->rsrc_start = pdev->resource[0].start;
        hcd->rsrc_len = pdev->resource[0].end - pdev->resource[0].start + 1;

        if (!request_mem_region(hcd->rsrc_start, hcd->rsrc_len, hcd_name)) {
                pr_debug("ohci probe request_mem_region failed");
                retval = -EBUSY;
                goto err1;
        }

        hcd->regs = ioremap(hcd->rsrc_start, hcd->rsrc_len);
        if (!hcd->regs) {
                pr_debug("ohci error mapping memory\n");
                retval = -ENOMEM;
                goto err2;
        }

        ohci = hcd_to_ohci(hcd);
        ohci_hcd_init(ohci);

        retval = usb_add_hcd(hcd, pdev->resource[1].start, IRQF_SHARED);

		//printk("Port status: 0x%x, 0x%x\n", __raw_readl((volatile void __iomem *)(NUC970_VA_EHCI+0x64)), __raw_readl((volatile void __iomem *)(NUC970_VA_EHCI+0x68)));
		//printk("Port status: 0x%x, 0x%x\n", __raw_readl((volatile void __iomem *)(NUC970_VA_OHCI+0x54)), __raw_readl((volatile void __iomem *)(NUC970_VA_OHCI+0x58)));

        if (retval == 0)
                return retval;

        pr_debug("Removing nuc970/n9h30 OHCI USB Controller\n");

        iounmap(hcd->regs);
err2:
        release_mem_region(hcd->rsrc_start, hcd->rsrc_len);
err1:

        usb_put_hcd(hcd);
        return retval;
}


/* may be called without controller electrically present */
/* may be called with controller, bus, and devices active */

/**
 * usb_hcd_ppc_soc_remove - shutdown processing for On-Chip HCDs
 * @pdev: USB Host Controller being removed
 * Context: !in_interrupt()
 *
 * Reverses the effect of usb_hcd_ppc_soc_probe().
 * It is always called from a thread
 * context, normally "rmmod", "apmd", or something similar.
 *
 */
static void usb_hcd_nuc970_remove(struct usb_hcd *hcd,
                                  struct platform_device *dev)
{
        usb_remove_hcd(hcd);

        //pr_debug("stopping PPC-SOC USB Controller\n");

        iounmap(hcd->regs);
        release_mem_region(hcd->rsrc_start, hcd->rsrc_len);
        usb_put_hcd(hcd);
}


static int ohci_nuc970_start (struct usb_hcd *hcd)
{
        struct ohci_hcd	*ohci = hcd_to_ohci (hcd);
        int ret;

        if ((ret = ohci_init(ohci)) < 0)
                return ret;

        if ((ret = ohci_run (ohci)) < 0) {
                printk("can't start %s", hcd->self.bus_name);
                ohci_stop (hcd);
                return ret;
        }

        return 0;
}


static const struct hc_driver ohci_nuc970_hc_driver = {
        .description =		hcd_name,
        .product_desc = 	"Nuvoton NUC970/N9H30 OHCI Host Controller",
        .hcd_priv_size =	sizeof(struct ohci_hcd),

        /*
         * generic hardware linkage
         */
        .irq =			ohci_irq,
        .flags =		HCD_USB11 | HCD_MEMORY,

        /*
         * basic lifecycle operations
         */
        .start =                ohci_nuc970_start,
        .stop =			ohci_stop,
	.shutdown =		ohci_shutdown,

        /*
         * managing i/o requests and associated device resources
         */
        .urb_enqueue =		ohci_urb_enqueue,
        .urb_dequeue =		ohci_urb_dequeue,
        .endpoint_disable =	ohci_endpoint_disable,

        /*
         * scheduling support
         */
        .get_frame_number =	ohci_get_frame,

        /*
         * root hub support
         */
        .hub_status_data =	ohci_hub_status_data,
        .hub_control =		ohci_hub_control,
#ifdef	CONFIG_PM
        .bus_suspend =		ohci_bus_suspend,
        .bus_resume =		ohci_bus_resume,
#endif
        .start_port_reset =	ohci_start_port_reset,
};


static int ohci_hcd_nuc970_drv_probe(struct platform_device *pdev)
{
        int ret;

        if (usb_disabled())
                return -ENODEV;

        ret = usb_hcd_nuc970_probe(&ohci_nuc970_hc_driver, pdev);
        return ret;
}

static int ohci_hcd_nuc970_drv_remove(struct platform_device *pdev)
{
        struct usb_hcd *hcd = platform_get_drvdata(pdev);

        usb_hcd_nuc970_remove(hcd, pdev);
        return 0;
}


#if defined(CONFIG_PM) && !defined(CONFIG_USB_NUC970_EHCI)

static int ohci_nuc970_pm_suspend(struct device *dev)
{
	struct usb_hcd *hcd = dev_get_drvdata(dev);
	bool do_wakeup = device_may_wakeup(dev);
	int  ret;

	ret = ohci_suspend(hcd, do_wakeup);

	/* Suspend PHY0 and PHY1; this will turn off PHY power. */
    __raw_writel(0x60, NUC970_VA_EHCI+0xC4);
    __raw_writel(0x20, NUC970_VA_EHCI+0xC8);

#ifdef CONFIG_OF

	if (of_pm_vbus_off)
	{
		if (of_mfp_setting == 1)
		{
	        __raw_writel(__raw_readl(REG_GPIOE_DATAOUT) & 0x3FFF, REG_GPIOE_DATAOUT);   // PE.14 & PE.15 output low
	        __raw_writel(__raw_readl(REG_GPIOE_DIR) | 0xC000, REG_GPIOE_DIR);           // PE.14 & PE.15 output mode
	        __raw_writel(__raw_readl(REG_MFP_GPE_H) & 0x00FFFFFF, REG_MFP_GPE_H);       // PE.14 & PE.15 GPIO mode
		}
		else if (of_mfp_setting == 2)
		{
	        __raw_writel(__raw_readl(REG_GPIOF_DATAOUT) & 0xFBFF, REG_GPIOF_DATAOUT);   // PF.10 output low
	        __raw_writel(__raw_readl(REG_GPIOF_DIR) | 0x0400, REG_GPIOF_DIR);           // PF.10 output mode
	        __raw_writel(__raw_readl(REG_MFP_GPF_H) & 0xFFFFF0FF, REG_MFP_GPF_H);       // PF.10 GPIO mode
	    }
	}

#else  /* !CONFIG_OF  */

#ifdef CONFIG_USB_NUC970_PM_VBUS_OFF_2
    /* turn off port power */
	#if defined (CONFIG_NUC970_USBH_PWR_PE_2)
		__raw_writel(__raw_readl(REG_GPIOE_DATAOUT) & 0x3FFF, REG_GPIOE_DATAOUT);   // PE.14 & PE.15 output low
		__raw_writel(__raw_readl(REG_GPIOE_DIR) | 0xC000, REG_GPIOE_DIR);           // PE.14 & PE.15 output mode
		__raw_writel(__raw_readl(REG_MFP_GPE_H) & 0x00FFFFFF, REG_MFP_GPE_H);       // PE.14 & PE.15 GPIO mode
	#elif defined (CONFIG_NUC970_USBH_PWR_PF_2)
		__raw_writel(__raw_readl(REG_GPIOF_DATAOUT) & 0xFBFF, REG_GPIOF_DATAOUT);   // PF.10 output low
		__raw_writel(__raw_readl(REG_GPIOF_DIR) | 0x0400, REG_GPIOF_DIR);           // PF.10 output mode
		__raw_writel(__raw_readl(REG_MFP_GPF_H) & 0xFFFFF0FF, REG_MFP_GPF_H);       // PF.10 GPIO mode
	#endif
#endif

#endif  /*  end of CONFIG_OF */

	return ret;
}

static int ohci_nuc970_pm_resume(struct device *dev)
{
	struct usb_hcd *hcd = dev_get_drvdata(dev);

#ifdef CONFIG_OF

	if (of_pm_vbus_off)
	{
		if (of_mfp_setting == 1)
		{
	        __raw_writel(__raw_readl(REG_MFP_GPE_H) | 0x77000000, REG_MFP_GPE_H);       // PE.14 & PE.15 for USBH_PWR
		}
		else if (of_mfp_setting == 2)
		{
	        __raw_writel(__raw_readl(REG_MFP_GPF_H) | 0x00000700, REG_MFP_GPF_H);       // PF.10 for USBH_PWR
	    }
	}

#else  /* !CONFIG_OF */

    #ifdef CONFIG_USB_NUC970_PM_VBUS_OFF_2
        #if defined (CONFIG_NUC970_USBH_PWR_PE_2)
	        __raw_writel(__raw_readl(REG_MFP_GPE_H) | 0x77000000, REG_MFP_GPE_H);       // PE.14 & PE.15 for USBH_PWR
        #elif defined (CONFIG_NUC970_USBH_PWR_PF_2)
	        __raw_writel(__raw_readl(REG_MFP_GPF_H) | 0x00000700, REG_MFP_GPF_H);       // PF.10 for USBH_PWR
        #endif
    #endif

#endif  /* CONFIG_OF */

	/* re-enable PHY0 and PHY1 */
    __raw_writel(0x160, NUC970_VA_EHCI+0xC4);
    __raw_writel(0x520, NUC970_VA_EHCI+0xC8);

	ohci_resume(hcd, false);

	return 0;
}
#else
#define ohci_nuc970_pm_suspend	NULL
#define ohci_nuc970_pm_resume	NULL
#endif

static const struct dev_pm_ops ohci_nuc970_dev_pm_ops = {
	.suspend         = ohci_nuc970_pm_suspend,
	.resume          = ohci_nuc970_pm_resume,
};


static const struct of_device_id nuc970_ohci_of_match[] = {
	{ .compatible = "nuvoton,nuc970-ohci" },
	{},
};
MODULE_DEVICE_TABLE(of, nuc970_ohci_of_match);


static struct platform_driver ohci_hcd_nuc970_driver = {
        .probe		= ohci_hcd_nuc970_drv_probe,
        .remove		= ohci_hcd_nuc970_drv_remove,

        .driver		= {
                .name	= "nuc970-ohci",
                .pm     = &ohci_nuc970_dev_pm_ops,
                .owner	= THIS_MODULE,
		        .of_match_table = of_match_ptr(nuc970_ohci_of_match),
        },
};
