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

#include <linux/clk.h>
#include <mach/irqs.h>
#include <mach/map.h>
#include <mach/regs-gcr.h>
#include <mach/regs-aic.h>
#include <mach/regs-timer.h>


static int usb_nuc970_probe(const struct hc_driver *driver,
                            struct platform_device *pdev)
{
        struct usb_hcd *hcd;
        struct ehci_hcd *ehci;
        u32  physical_map_ehci;
        struct pinctrl *p;
        int retval;

        if (IS_ERR(clk_get(NULL, "usbh_hclk"))) {
                printk("clk_get error!!\n");
                return -1;
        }

		/* multi-function pin select */
#if defined (CONFIG_NUC970_USBH_PWR_PE)
        /* set over-current active low */
        __raw_writel(__raw_readl(NUC970_VA_OHCI+0x204) | 0x8, NUC970_VA_OHCI+0x204);

    	/* initial USBH_PPWR0 & USBH_PPWR1 pin -> PE.14 & PE.15 */
    	p = devm_pinctrl_get_select(&pdev->dev, "usbh-ppwr-pe");
    	if (IS_ERR(p))
    	{
        	dev_err(&pdev->dev, "unable to reserve pin\n");
        	retval = PTR_ERR(p);
    	}
#elif defined (CONFIG_NUC970_USBH_PWR_PF)
        /* set over-current active low */
        __raw_writel(__raw_readl(NUC970_VA_OHCI+0x204) | 0x8, NUC970_VA_OHCI+0x204);

    	/* initial USBH_PPWR pin -> PF.10 */
    	p = devm_pinctrl_get_select(&pdev->dev, "usbh-ppwr-pf");
    	if (IS_ERR(p))
    	{
        	dev_err(&pdev->dev, "unable to reserve pin\n");
        	retval = PTR_ERR(p);
    	}
#elif defined (CONFIG_NUC970_USBH_OC_ONLY)
        /* set over-current active low */
        __raw_writel(__raw_readl(NUC970_VA_OHCI+0x204) | 0x8, NUC970_VA_OHCI+0x204);

    	p = devm_pinctrl_get_select(&pdev->dev, "usbh-ppwr-oc");
    	if (IS_ERR(p))
    	{
        	dev_err(&pdev->dev, "unable to reserve pin\n");
        	retval = PTR_ERR(p);
    	}
#else  //  CONFIG_NUC970_USBH_NONE
        /* set over-current active high */
        __raw_writel(__raw_readl(NUC970_VA_OHCI+0x204) &~0x8, NUC970_VA_OHCI+0x204);
#endif
		/* Enable USB Host clock */
        clk_prepare(clk_get(NULL, "usb_eclk"));	
        clk_enable(clk_get(NULL, "usb_eclk"));
        
        clk_prepare(clk_get(NULL, "usbh_hclk"));	
        clk_enable(clk_get(NULL, "usbh_hclk"));

        if (pdev->resource[1].flags != IORESOURCE_IRQ) {
                pr_debug("resource[1] is not IORESOURCE_IRQ");
                retval = -ENOMEM;
        }

        hcd = usb_create_hcd(driver, &pdev->dev, "nuc970-ehci");
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
        __raw_writel(0x160, physical_map_ehci+0xC4);
        __raw_writel(0x520, physical_map_ehci+0xC8);

        /* cache this readonly data; minimize chip reads */
        ehci->hcs_params = readl(&ehci->caps->hcs_params);
        ehci->sbrn = 0x20;

        retval = usb_add_hcd(hcd, pdev->resource[1].start, IRQF_SHARED);

        if (retval != 0)
            goto err4;

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

        /*
         * managing i/o requests and associated device resources
         */
        .urb_enqueue = ehci_urb_enqueue,
        .urb_dequeue = ehci_urb_dequeue,
        .endpoint_disable = ehci_endpoint_disable,

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
};

static int ehci_nuc970_probe(struct platform_device *pdev)
{
        //printk("ehci_nuc970_probe()\n");
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

static struct platform_driver ehci_hcd_nuc970_driver = {

        .probe = ehci_nuc970_probe,
        .remove = ehci_nuc970_remove,
        .driver = {
                .name = "nuc970-ehci",
                .owner= THIS_MODULE,
        },
};

static int __init ehci_nuc970_init(void)
{

	return platform_driver_register(&ehci_hcd_nuc970_driver);
}

static void __exit ehci_nuc970_cleanup(void)
{
	platform_driver_unregister(&ehci_hcd_nuc970_driver);

}

//module_init(ehci_nuc970_init);
//module_exit(ehci_nuc970_cleanup);
