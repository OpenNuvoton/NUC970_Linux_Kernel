/*
 * linux/driver/usb/host/ehci-nuc970.c
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
#include <linux/gfp.h>
#include <linux/of.h>

#include <linux/clk.h>
#include <mach/irqs.h>
#include <mach/map.h>
#include <mach/regs-gcr.h>
#include <mach/regs-aic.h>
#include <mach/regs-timer.h>
#include <mach/regs-gpio.h>

static int  of_pm_vbus_off;
static int  of_mfp_setting;   /* D+/D- multi-function pin setting - 0: not used; 1: PE.14/PE.15; 2: PF.10  */

#if 0
#include <linux/kthread.h>
static int port_dump_thread(void *__unused)
{
	while (1)
	{
		printk("EHCI: 0x%x, 0x%x, 0x%x - 0x%x 0x%x\n", __raw_readl(NUC970_VA_EHCI+0x20), __raw_readl(NUC970_VA_EHCI+0x64), __raw_readl(NUC970_VA_EHCI+0x68), __raw_readl(NUC970_VA_EHCI+0xC4), __raw_readl(NUC970_VA_EHCI+0xC8));
		// printk("OHCI suspend: 0x%x, 0x%x\n", __raw_readl(NUC970_VA_OHCI+0x50), __raw_readl(NUC970_VA_EHCI+0x54));
		msleep(5000);
	}
	return 0;
}
#endif

static int usb_nuc970_probe(const struct hc_driver *driver,
                            struct platform_device *pdev)
{
        struct usb_hcd *hcd;
        struct ehci_hcd *ehci;
        u32  physical_map_ehci;
        struct pinctrl *p;
        int retval;
#ifdef CONFIG_OF
	    u32   val32[2];
#endif

        if (IS_ERR(clk_get(NULL, "usbh_hclk"))) {
                printk("clk_get error!!\n");
                return -1;
        }

		/* Enable USB Host clock */
        clk_prepare(clk_get(NULL, "usb_eclk"));	
        clk_enable(clk_get(NULL, "usb_eclk"));
        
        clk_prepare(clk_get(NULL, "usbh_hclk"));	
        clk_enable(clk_get(NULL, "usbh_hclk"));

#ifdef CONFIG_OF

        p = devm_pinctrl_get_select_default(&pdev->dev);
        if (IS_ERR(p)) {
            return PTR_ERR(p);
        }

		if ((__raw_readl(REG_MFP_GPE_H) & 0xFF000000) == 0x77000000)
			of_mfp_setting = 1;
		else if ((__raw_readl(REG_MFP_GPF_H) & 0x00000F00) == 0x00000700)
			of_mfp_setting = 2;
		else
			of_mfp_setting = 0;
			
		//printk("of_mfp_setting = %d\n", of_mfp_setting);

		if (of_property_read_u32_array(pdev->dev.of_node, "ov_active", val32, 1) != 0) 
		{
			printk("%s - can not get map-addr!\n", __func__);
			return -EINVAL;
		}
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

		/* multi-function pin select */
#if defined (CONFIG_NUC970_USBH_PWR_PE)
        /* set over-current active low */
        __raw_writel(__raw_readl(NUC970_VA_OHCI+0x204) | 0x8, (volatile void __iomem *)(NUC970_VA_OHCI+0x204));

    	/* initial USBH_PPWR0 & USBH_PPWR1 pin -> PE.14 & PE.15 */
    	p = devm_pinctrl_get_select(&pdev->dev, "usbh-ppwr-pe");
    	if (IS_ERR(p))
    	{
        	dev_err(&pdev->dev, "unable to reserve pin\n");
        	retval = PTR_ERR(p);
    	}

#elif defined (CONFIG_NUC970_USBH_PWR_PF)
        /* set over-current active low */
        __raw_writel(__raw_readl(NUC970_VA_OHCI+0x204) | 0x8, (volatile void __iomem *)(NUC970_VA_OHCI+0x204));

    	/* initial USBH_PPWR pin -> PF.10 */
    	p = devm_pinctrl_get_select(&pdev->dev, "usbh-ppwr-pf");
    	if (IS_ERR(p))
    	{
        	dev_err(&pdev->dev, "unable to reserve pin\n");
        	retval = PTR_ERR(p);
    	}
#elif defined (CONFIG_NUC970_USBH_OC_ONLY)
        /* set over-current active low */
        __raw_writel(__raw_readl(NUC970_VA_OHCI+0x204) | 0x8, (volatile void __iomem *)(NUC970_VA_OHCI+0x204));

    	p = devm_pinctrl_get_select(&pdev->dev, "usbh-ppwr-oc");
    	if (IS_ERR(p))
    	{
        	dev_err(&pdev->dev, "unable to reserve pin\n");
        	retval = PTR_ERR(p);
    	}
#else  //  CONFIG_NUC970_USBH_NONE
        /* set over-current active high */
        __raw_writel(__raw_readl(NUC970_VA_OHCI+0x204) &~0x8, (volatile void __iomem *)(NUC970_VA_OHCI+0x204));
#endif

#endif  // CONFIG_OF

        if (pdev->resource[1].flags != IORESOURCE_IRQ) {
                pr_debug("resource[1] is not IORESOURCE_IRQ");
                retval = -ENOMEM;
        }

        hcd = usb_create_hcd(driver, &pdev->dev, dev_name(&pdev->dev));
        if (!hcd) {
                retval = -ENOMEM;
                goto err1;
        }

        hcd->rsrc_start = pdev->resource[0].start;
        hcd->rsrc_len = pdev->resource[0].end - pdev->resource[0].start + 1;

        if (!request_mem_region(hcd->rsrc_start, hcd->rsrc_len, hcd_name)) {
                pr_debug("ehci probe request_mem_region failed");
                retval = -EBUSY;
                goto err2;
        }

        hcd->regs = ioremap(hcd->rsrc_start, hcd->rsrc_len);
        if (hcd->regs == NULL) {
                pr_debug("ehci error mapping memory\n");
                retval = -EFAULT;
                goto err3;
        }

        ehci = hcd_to_ehci(hcd);
        ehci->caps = hcd->regs;
        ehci->regs = hcd->regs + 0x20;

        /* enable PHY 0/1 */
        physical_map_ehci = (u32)ehci->caps;
        __raw_writel(0x160, (volatile void __iomem *)physical_map_ehci+0xC4);
        __raw_writel(0x520, (volatile void __iomem *)physical_map_ehci+0xC8);

        /* cache this readonly data; minimize chip reads */
        ehci->hcs_params = readl(&ehci->caps->hcs_params);
        ehci->sbrn = 0x20;
        
        retval = usb_add_hcd(hcd, pdev->resource[1].start, IRQF_SHARED);

        if (retval != 0)
            goto err4;

		 // kthread_run(port_dump_thread, NULL, "khubd");

        return retval;

err4:
        iounmap(hcd->regs);
err3:
        release_mem_region(hcd->rsrc_start, hcd->rsrc_len);
err2:
        usb_put_hcd(hcd);
err1:

        return retval;
}

void usb_nuc970_remove(struct usb_hcd *hcd, struct platform_device *pdev)
{
        usb_remove_hcd(hcd);
        iounmap(hcd->regs);
        release_mem_region(hcd->rsrc_start, hcd->rsrc_len);
        usb_put_hcd(hcd);
}


static const struct hc_driver ehci_nuc970_hc_driver = {
        .description = hcd_name,
        .product_desc = "Nuvoton NUC970 EHCI Host Controller",
        .hcd_priv_size = sizeof(struct ehci_hcd),

        /*
         * generic hardware linkage
         */
        .irq = ehci_irq,
        .flags = HCD_USB2|HCD_MEMORY,

        /*
         * basic lifecycle operations
         */
        .reset = ehci_init,
        .start = ehci_run,

        .stop = ehci_stop,
	    .shutdown = ehci_shutdown,

        /*
         * managing i/o requests and associated device resources
         */
        .urb_enqueue = ehci_urb_enqueue,
        .urb_dequeue = ehci_urb_dequeue,
        .endpoint_disable = ehci_endpoint_disable,
        .endpoint_reset		= ehci_endpoint_reset,

        /*
         * scheduling support
         */
        .get_frame_number = ehci_get_frame,

        /*
         * root hub support
         */
        .hub_status_data = ehci_hub_status_data,
        .hub_control = ehci_hub_control,
#ifdef	CONFIG_PM
        .bus_suspend = ehci_bus_suspend,
        .bus_resume = ehci_bus_resume,
#endif
	    .relinquish_port = ehci_relinquish_port,
	    .port_handed_over = ehci_port_handed_over,

	    .clear_tt_buffer_complete = ehci_clear_tt_buffer_complete,
};

static int ehci_nuc970_probe(struct platform_device *pdev)
{
        printk("ehci_nuc970_probe() - name: %s\n", pdev->name);
        if (usb_disabled())
                return -ENODEV;

        return usb_nuc970_probe(&ehci_nuc970_hc_driver, pdev);
}

static int ehci_nuc970_remove(struct platform_device *pdev)
{
        struct usb_hcd *hcd = platform_get_drvdata(pdev);

        usb_nuc970_remove(hcd, pdev);

        return 0;
}

#ifdef CONFIG_PM
static int ehci_nuc970_pm_suspend(struct device *dev)
{
	struct usb_hcd *hcd = dev_get_drvdata(dev);
	bool do_wakeup = device_may_wakeup(dev);
	int  ret;

	ret = ehci_suspend(hcd, do_wakeup);

	/* Suspend PHY0 and PHY1; this will turn off PHY power. */
    __raw_writel(0x60, NUC970_VA_EHCI+0xC4);
    __raw_writel(0x20, NUC970_VA_EHCI+0xC8);

#ifdef CONFIG_OF

	if (of_pm_vbus_off)
	{
		if (of_mfp_setting == 1)
		{
	        __raw_writel(__raw_readl(REG_GPIOE_DATAOUT) & 0x3FFF, REG_GPIOE_DATAOUT);   // PE.14 & PE.15 output low
	        __raw_writel(__raw_readl(REG_GPIOE_DIR) & 0xC000, REG_GPIOE_DIR);           // PE.14 & PE.15 output mode
	        __raw_writel(__raw_readl(REG_MFP_GPE_H) & 0x00FFFFFF, REG_MFP_GPE_H);       // PE.14 & PE.15 GPIO mode
		}
		else if (of_mfp_setting == 2)
		{
	        __raw_writel(__raw_readl(REG_GPIOF_DATAOUT) & 0xFBFF, REG_GPIOF_DATAOUT);   // PF.10 output low
	        __raw_writel(__raw_readl(REG_GPIOF_DIR) & 0x0400, REG_GPIOF_DIR);           // PF.10 output mode
	        __raw_writel(__raw_readl(REG_MFP_GPF_H) & 0xFFFFF0FF, REG_MFP_GPF_H);       // PF.10 GPIO mode
	    }
	}

#else   /* !CONFIG_OF */

    #ifdef CONFIG_USB_NUC970_PM_VBUS_OFF    
        /* turn off port power */
        #if defined (CONFIG_NUC970_USBH_PWR_PE)
	        __raw_writel(__raw_readl(REG_GPIOE_DATAOUT) & 0x3FFF, REG_GPIOE_DATAOUT);   // PE.14 & PE.15 output low
	        __raw_writel(__raw_readl(REG_GPIOE_DIR) & 0xC000, REG_GPIOE_DIR);           // PE.14 & PE.15 output mode
	        __raw_writel(__raw_readl(REG_MFP_GPE_H) & 0x00FFFFFF, REG_MFP_GPE_H);       // PE.14 & PE.15 GPIO mode
        #elif defined (CONFIG_NUC970_USBH_PWR_PF)
	        __raw_writel(__raw_readl(REG_GPIOF_DATAOUT) & 0xFBFF, REG_GPIOF_DATAOUT);   // PF.10 output low
	        __raw_writel(__raw_readl(REG_GPIOF_DIR) & 0x0400, REG_GPIOF_DIR);           // PF.10 output mode
	        __raw_writel(__raw_readl(REG_MFP_GPF_H) & 0xFFFFF0FF, REG_MFP_GPF_H);       // PF.10 GPIO mode
        #endif
    #endif  /* end of CONFIG_USB_NUC970_PM_VBUS_OFF */

#endif  /* end of CONFIG_OFF */
	
	return ret;
}

static int ehci_nuc970_pm_resume(struct device *dev)
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

    #ifdef CONFIG_USB_NUC970_PM_VBUS_OFF    
        #if defined (CONFIG_NUC970_USBH_PWR_PE)
	        __raw_writel(__raw_readl(REG_MFP_GPE_H) | 0x77000000, REG_MFP_GPE_H);       // PE.14 & PE.15 for USBH_PWR
        #elif defined (CONFIG_NUC970_USBH_PWR_PF)
	        __raw_writel(__raw_readl(REG_MFP_GPF_H) | 0x00000700, REG_MFP_GPF_H);       // PF.10 for USBH_PWR
        #endif
    #endif

#endif  /* end of CONFIG_OF */

	/* re-enable PHY0 and PHY1 */
    __raw_writel(0x160, NUC970_VA_EHCI+0xC4);
    __raw_writel(0x520, NUC970_VA_EHCI+0xC8);

	ehci_resume(hcd, false);

	return 0;
}
#else
#define ehci_nuc970_pm_suspend	NULL
#define ehci_nuc970_pm_resume	NULL
#endif

static const struct dev_pm_ops ehci_nuc970_dev_pm_ops = {
	.suspend         = ehci_nuc970_pm_suspend,
	.resume          = ehci_nuc970_pm_resume,
};


static const struct of_device_id nuc970_ehci_of_match[] = {
	{ .compatible = "nuvoton,nuc970-ehci" },
	{},
};
MODULE_DEVICE_TABLE(of, nuc970_ehci_of_match);


static struct platform_driver ehci_hcd_nuc970_driver = {

        .probe = ehci_nuc970_probe,
        .remove = ehci_nuc970_remove,
        .driver = {
                .name = "nuc970-ehci",
		        .pm = &ehci_nuc970_dev_pm_ops,
                .owner= THIS_MODULE,
		        .of_match_table = of_match_ptr(nuc970_ehci_of_match),
        },
};

