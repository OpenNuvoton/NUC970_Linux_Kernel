/*
 *  linux/drivers/mmc/host/nuc970_sd_2port.c - Nuvoton NUC970 dual SD Driver
 *
 * Copyright (c) 2014 Nuvoton Technology Corp.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation;version 2 of the License.
 */


#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/mutex.h>
#include <linux/ioport.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/blkdev.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/dma-mapping.h>
#include <linux/clk.h>
#include <linux/atmel_pdc.h>
#include <linux/gfp.h>

#include <linux/mmc/host.h>

#include <asm/io.h>
#include <asm/irq.h>

#include <mach/map.h>
#include <mach/regs-clock.h>
#include <mach/regs-gcr.h>
#include <mach/regs-sdh.h>

//#define nuc970_sd_debug       printk
#define nuc970_sd_debug(...)

#define DRIVER_NAME    "nuc970-sdh"

#define FL_SENT_COMMAND (1 << 0)
#define FL_SENT_STOP    (1 << 1)

#define nuc970_sd_read(reg)          __raw_readl(reg)
#define nuc970_sd_write(reg, val)    __raw_writel((val), (reg))

#define MCI_BLKSIZE         512
#define MCI_MAXBLKSIZE      4095
#define MCI_BLKATONCE       255
#define MCI_BUFSIZE         (MCI_BLKSIZE * MCI_BLKATONCE)

/* Driver thread command */
#define SD_EVENT_NONE       0x00000000
#define SD_EVENT_CMD_OUT    0x00000001
#define SD_EVENT_RSP_IN     0x00000010
#define SD_EVENT_RSP2_IN    0x00000100
#define SD_EVENT_CLK_KEEP0  0x00001000
#define SD_EVENT_CLK_KEEP1  0x00010000

/* SD Port 0 */
static volatile int sd_event=0, sd_state=0, sd_state_xfer=0, sd_ri_timeout=0, sd_send_cmd=0;
static DECLARE_WAIT_QUEUE_HEAD(sd_event_wq);
static DECLARE_WAIT_QUEUE_HEAD(sd_wq);
static DECLARE_WAIT_QUEUE_HEAD(sd_wq_xfer);

/* SD Port 1 */
static volatile int sd1_event=0, sd1_state=0, sd1_state_xfer=0, sd1_ri_timeout=0, sd1_send_cmd=0;
static DECLARE_WAIT_QUEUE_HEAD(sd1_event_wq);
static DECLARE_WAIT_QUEUE_HEAD(sd1_wq);
static DECLARE_WAIT_QUEUE_HEAD(sd1_wq_xfer);

//--- Define semaphore for SDH controller
//static DEFINE_MUTEX(sdh_mutex);
//static DEFINE_MUTEX(sdh_dmac_sem);
struct semaphore sdh_fmi_sem;
struct semaphore sdh_dmac_sem;

/*
 * Low level type for this driver
 */
struct nuc970_sd_host {
    struct mmc_host *mmc;
    struct mmc_command *cmd;
    struct mmc_request *request;

    void __iomem *sd_base;
    int irq;

    int present;

    struct clk *sd_clk, *upll_clk;

    /*
     * Flag indicating when the command has been sent. This is used to
     * work out whether or not to send the stop
     */
    unsigned int flags;
    /* flag for current port */
    u32 bus_mode;
    u32 port;       // SD port 0 / 1

    /* DMA buffer used for transmitting */
    unsigned int* buffer;
    dma_addr_t physical_address;
    unsigned int total_length;

    /* Latest in the scatterlist that has been enabled for transfer, but not freed */
    int in_use_index;

    /* Latest in the scatterlist that has been enabled for transfer */
    int transfer_index;

    /* Timer for timeouts */
    struct timer_list timer;
};

struct nuc970_sd_host *sd_host, *sd1_host;

#if 0
static void dump_sdh_regs()
{
    printk("    REG_CLK_HCLKEN = 0x%x\n", nuc970_sd_read(REG_CLK_HCLKEN));
    printk("    REG_CLKDIV9 = 0x%x\n", nuc970_sd_read(REG_CLK_DIV9));
    printk("    REG_FMICSR  = 0x%x\n", nuc970_sd_read(REG_FMICSR));
    printk("    REG_DMACCSR = 0x%x\n", nuc970_sd_read(REG_DMACCSR));
    printk("    REG_SDCSR   = 0x%x\n", nuc970_sd_read(REG_SDCSR));
    printk("    REG_SDARG   = 0x%x\n", nuc970_sd_read(REG_SDARG));
    printk("    REG_SDIER   = 0x%x\n", nuc970_sd_read(REG_SDIER));
    printk("    REG_SDISR   = 0x%x\n", nuc970_sd_read(REG_SDISR));
    printk("    REG_SDRSP0  = 0x%x\n", nuc970_sd_read(REG_SDRSP0));
    printk("    REG_SDRSP1  = 0x%x\n", nuc970_sd_read(REG_SDRSP1));
    printk("    REG_SDBLEN  = 0x%x\n", nuc970_sd_read(REG_SDBLEN));
    printk("    REG_SDTMOUT = 0x%x\n", nuc970_sd_read(REG_SDTMOUT));
    printk("    REG_SDECR   = 0x%x\n", nuc970_sd_read(REG_SDECR));
}
#endif

/*
 * Reset the controller and restore most of the state
 */
static void nuc970_sd_reset_host(struct nuc970_sd_host *host)
{
    unsigned long flags;

    local_irq_save(flags);

    nuc970_sd_write(REG_DMACCSR, nuc970_sd_read(REG_DMACCSR) | DMACCSR_DMACEN | DMACCSR_SW_RST); //enable DMAC for FMI
//    nuc970_sd_write(REG_FMICSR, FMICSR_SD_EN | FMICSR_SW_RST); /* Enable SD functionality of FMI */
    nuc970_sd_write(REG_FMICSR, FMICSR_SD_EN);
    local_irq_restore(flags);
}

static void nuc970_sd_timeout_timer(unsigned long data)
{
    struct nuc970_sd_host *host;

    host = (struct nuc970_sd_host *)data;

    if (host->request)
    {
        dev_err(host->mmc->parent, "Timeout waiting end of packet\n");

        if (host->cmd && host->cmd->data)
        {
            host->cmd->data->error = -ETIMEDOUT;
        }
        else
        {
            if (host->cmd)
                host->cmd->error = -ETIMEDOUT;
            else
                host->request->cmd->error = -ETIMEDOUT;
        }
        nuc970_sd_reset_host(host);
        mmc_request_done(host->mmc, host->request);
    }
}

/*
 * Copy from sg to a dma block - used for transfers
 */
static inline void nuc970_sd_sg_to_dma(struct nuc970_sd_host *host, struct mmc_data *data)
{
    unsigned int len, i, size;
    unsigned *dmabuf = host->buffer;

    size = data->blksz * data->blocks;
    len = data->sg_len;

    /*
     * Just loop through all entries. Size might not
     * be the entire list though so make sure that
     * we do not transfer too much.
     */
    for (i = 0; i < len; i++) {
        struct scatterlist *sg;
        int amount;
        unsigned int *sgbuffer;

        sg = &data->sg[i];

        sgbuffer = kmap_atomic(sg_page(sg)) + sg->offset;
        amount = min(size, sg->length);
        size -= amount;
        {
            char *tmpv = (char *)dmabuf;
            memcpy(tmpv, sgbuffer, amount);
            tmpv += amount;
            dmabuf = (unsigned *)tmpv;
        }

        kunmap_atomic(sgbuffer);
        data->bytes_xfered += amount;

        if (size == 0)
            break;
    }

    /*
     * Check that we didn't get a request to transfer
     * more data than can fit into the SG list.
     */
    BUG_ON(size != 0);
}

/*
 * Handle after a dma read
 */
static void nuc970_sd_post_dma_read(struct nuc970_sd_host *host)
{
    struct mmc_command *cmd;
    struct mmc_data *data;
    unsigned int len, i, size;
    unsigned *dmabuf = host->buffer;

    cmd = host->cmd;
    if (!cmd) {
        nuc970_sd_debug("no command\n");
        return;
    }

    data = cmd->data;
    if (!data) {
        nuc970_sd_debug("no data\n");
        return;
    }

    size = data->blksz * data->blocks;
    len = data->sg_len;

    for (i = 0; i < len; i++) {
        struct scatterlist *sg;
        int amount;
        unsigned int *sgbuffer;

        sg = &data->sg[i];

        sgbuffer = kmap_atomic(sg_page(sg)) + sg->offset;
        amount = min(size, sg->length);
        size -= amount;
        {
            char *tmpv = (char *)dmabuf;
            memcpy(sgbuffer, tmpv, amount);
            tmpv += amount;
            dmabuf = (unsigned *)tmpv;
        }
        flush_kernel_dcache_page(sg_page(sg));
        kunmap_atomic(sgbuffer);
        data->bytes_xfered += amount;
        if (size == 0)
            break;
    }
}

/*
 * Handle transmitted data
 */
static void nuc970_sd_handle_transmitted(struct nuc970_sd_host *host)
{
    //nuc970_sd_debug("Handling the transmit\n");
    if (nuc970_sd_read(REG_SDISR) & SDISR_CRC_IF)
        nuc970_sd_write(REG_SDISR, SDISR_CRC_IF);

    /* check read/busy */
    if (host->port == 0)
        nuc970_sd_write(REG_SDCSR, nuc970_sd_read(REG_SDCSR) | SDCSR_CLK_KEEP0);
    if (host->port == 1)
        nuc970_sd_write(REG_SDCSR, nuc970_sd_read(REG_SDCSR) | SDCSR_CLK_KEEP1);
}


/*
 * Update bytes tranfered count during a write operation
 */
static void nuc970_sd_update_bytes_xfered(struct nuc970_sd_host *host)
{
    struct mmc_data *data;

    /* always deal with the effective request (and not the current cmd) */
    if (host->request->cmd && host->request->cmd->error != 0)
        return;

    if (host->request->data) {
        data = host->request->data;
        if (data->flags & MMC_DATA_WRITE) {
            /* card is in IDLE mode now */
            data->bytes_xfered = data->blksz * data->blocks;
        }
    }
}

/*-----------------------------------------------------------------------------
 * Config SIC register to select SD port.
 *---------------------------------------------------------------------------*/
static int nuc970_sd_select_port(u32 port)
{
    if (port == 0)
        nuc970_sd_write(REG_SDCSR, (nuc970_sd_read(REG_SDCSR) & 0x9fffffff)); //SD Port 0 is selected

    if (port == 1)
        nuc970_sd_write(REG_SDCSR, (nuc970_sd_read(REG_SDCSR) & 0x9fffffff)|0x20000000); //SD Port 1 is selected

#if 0
    //--- Reset SDIO controller and DMAC to keep clean status for next access.
    // Reset DMAC engine and interrupt satus
    nuc970_sd_write(REG_DMACCSR, DMACCSR_DMACEN | DMACCSR_SW_RST); //enable DMAC for FMI
    while(nuc970_sd_read(REG_DMACCSR) & DMACCSR_SW_RST);

    // Reset FMI engine and interrupt status
    nuc970_sd_write(REG_FMICSR, FMICSR_SW_RST); /* Enable SD functionality of FMI */
    while(nuc970_sd_read(REG_FMICSR) & FMICSR_SW_RST);

    // Reset SDIO engine and interrupt status
    nuc970_sd_write(REG_SDCSR, (nuc970_sd_read(REG_SDCSR) | 0x4000)); //SD reset
    while(nuc970_sd_read(REG_SDCSR) & 0x4000);
    nuc970_sd_write(REG_SDISR, 0xffffffff);
#endif

    return 0;
}



/*
 * Enable the controller
 */
static void nuc970_sd_enable(struct nuc970_sd_host *host)
{
    nuc970_sd_write(REG_DMACCSR, DMACCSR_DMACEN);   // enable DMAC for FMI
    nuc970_sd_write(REG_FMICSR, FMICSR_SD_EN);  /* Enable SD functionality of FMI */
    nuc970_sd_write(REG_SDISR, 0xffffffff);

    if (host->port == 0)
    {
        nuc970_sd_write(REG_SDIER, nuc970_sd_read(REG_SDIER) | SDIER_CD0SRC);   // select GPIO detect
        nuc970_sd_write(REG_SDCSR, (nuc970_sd_read(REG_SDCSR) & 0x9fffffff)); //SD Port 0 is selected
    }

    if (host->port == 1)
    {
        nuc970_sd_write(REG_SDIER, nuc970_sd_read(REG_SDIER) | SDIER_CD1SRC);   // select GPIO detect
        nuc970_sd_write(REG_SDCSR, (nuc970_sd_read(REG_SDCSR) & 0x9fffffff)|0x20000000); //SD Port 1 is selected
    }
    nuc970_sd_write(REG_SDCSR, (nuc970_sd_read(REG_SDCSR) & ~0xfff0000)|0x09010000);
}

/*
 * Disable the controller
 */
static void nuc970_sd_disable(struct nuc970_sd_host *host)
{
    nuc970_sd_write(REG_SDISR, 0xffffffff);
    nuc970_sd_write(REG_FMICSR, nuc970_sd_read(REG_SDCSR) & ~FMICSR_SD_EN);
}

/*
 * Send a command
 */
static void nuc970_sd_send_command(struct nuc970_sd_host *host, struct mmc_command *cmd)
{
    unsigned int volatile csr;
    unsigned int block_length;
    struct mmc_data *data = cmd->data;
    int clock_free_run_status = 0;
    unsigned int blocks;

    host->cmd = cmd;

    if (host->port == 0)
    {
        sd_host = host;
        sd_state = 0;
        sd_state_xfer = 0;
    }

    if (host->port == 1)
    {
        sd1_host = host;
        sd1_state = 0;
        sd1_state_xfer = 0;
    }

    if (down_interruptible(&sdh_fmi_sem))   // get sdio_fmi_sem for whole SD command, include data read/write.
        return;

    if (nuc970_sd_read(REG_FMICSR) != FMICSR_SD_EN)
        nuc970_sd_write(REG_FMICSR, FMICSR_SD_EN);

    nuc970_sd_select_port(host->port);
    csr = nuc970_sd_read(REG_SDCSR) & 0xff00c080;

    if (host->port == 0)
    {
        clock_free_run_status = csr | 0x80; /* clock keep */
        csr = csr & ~0x80;
    }
    if (host->port == 1)
    {
        clock_free_run_status = csr | 0x80000000; /* clock keep */
        csr = csr & ~0x80000000;
    }
    csr = csr | (cmd->opcode << 8) | SDCSR_CO_EN;   // set command code and enable command out
    if (host->port == 0)
        sd_event |= SD_EVENT_CMD_OUT;
    if (host->port == 1)
        sd1_event |= SD_EVENT_CMD_OUT;

    if (host->bus_mode == MMC_BUS_WIDTH_4)
        csr |= SDCSR_DBW;

    if (mmc_resp_type(cmd) != MMC_RSP_NONE)
    {
        /* if a response is expected then allow maximum response latancy */
        /* set 136 bit response for R2, 48 bit response otherwise */
        if (mmc_resp_type(cmd) == MMC_RSP_R2)
        {
            csr |= SDCSR_R2_EN;
            if (host->port == 0)
                sd_event |= SD_EVENT_RSP2_IN;
            if (host->port == 1)
                sd1_event |= SD_EVENT_RSP2_IN;
        }
        else
        {
            csr |= SDCSR_RI_EN;
            if (host->port == 0)
                sd_event |= SD_EVENT_RSP_IN;
            if (host->port == 1)
                sd1_event |= SD_EVENT_RSP_IN;
        }
        nuc970_sd_write(REG_SDISR, SDISR_RITO_IF);
        if (host->port == 0)
            sd_ri_timeout = 0;
        if (host->port == 1)
            sd1_ri_timeout = 0;
        nuc970_sd_write(REG_SDTMOUT, 0xffff);
    }

    if (data)
    {
        nuc970_sd_write(REG_SDIER, nuc970_sd_read(REG_SDIER) | SDIER_BLKD_IE);  //Enable SD interrupt & select GPIO detect
        block_length = data->blksz;
        blocks = data->blocks;

        nuc970_sd_write(REG_SDBLEN, block_length-1);
        if ((block_length > 512) || (blocks >= 256))
            printk("ERROR: don't support read/write 256 blocks in on CMD\n");
        else
            csr = (csr & ~0x00ff0000) | (blocks << 16);
    }
    else
    {
        nuc970_sd_write(REG_SDIER, nuc970_sd_read(REG_SDIER) & ~SDIER_BLKD_IE); //disable SD interrupt & select GPIO detect
        block_length = 0;
        blocks = 0;
    }

    /*
     * Set the arguments and send the command
     */
    nuc970_sd_debug("Sending command %d as 0x%0X, arg = 0x%08X, blocks = %d, length = %d\n",
              cmd->opcode, csr, cmd->arg, blocks, block_length);

    if (data)
    {
        data->bytes_xfered = 0;
        host->transfer_index = 0;
        host->in_use_index = 0;
        if (data->flags & MMC_DATA_READ)
        {
            /*
             * Handle a read
             */
            host->total_length = 0;
            nuc970_sd_write(REG_DMACSAR2, host->physical_address);
            nuc970_sd_debug("SDH - Reading %d bytes [phy_addr = 0x%x]\n", block_length * blocks, host->physical_address);
        }
        else if (data->flags & MMC_DATA_WRITE)
        {
            if (down_interruptible(&sdh_dmac_sem))  // get sdio_dmac_sem for data writing.
                return;
            host->total_length = block_length * blocks;
            nuc970_sd_sg_to_dma(host, data);
            nuc970_sd_debug("SDH - Transmitting %d bytes\n", host->total_length);
            nuc970_sd_write(REG_DMACSAR2, host->physical_address);
            csr = csr | SDCSR_DO_EN;
        }
    }
    /*
     * Send the command and then enable the PDC - not the other way round as
     * the data sheet says
     */
    nuc970_sd_write(REG_SDARG, cmd->arg);
    nuc970_sd_write(REG_SDCSR, csr);
    if (host->port == 0)
    {
        sd_send_cmd = 1;
        wake_up_interruptible(&sd_event_wq);
        wait_event_interruptible(sd_wq, (sd_state != 0));
    }
    if (host->port == 1)
    {
        sd1_send_cmd = 1;
        wake_up_interruptible(&sd1_event_wq);
        wait_event_interruptible(sd1_wq, (sd1_state != 0));
    }

    if (data)
    {
        if (data->flags & MMC_DATA_WRITE)
        {
            while (!(nuc970_sd_read(REG_SDISR) & SDISR_SDDAT0))
            {
                nuc970_sd_write(REG_SDCSR, nuc970_sd_read(REG_SDCSR) | 0x40);   /* clk8_oe */
                while (nuc970_sd_read(REG_SDCSR) & 0x40);
            }
            up(&sdh_dmac_sem);  // release sdio_dmac_sem for data writing.
            nuc970_sd_update_bytes_xfered(host);
        }
    }
    if (clock_free_run_status)
    {
        if (host->port == 0)
            nuc970_sd_write(REG_SDCSR, nuc970_sd_read(REG_SDCSR) | 0x80);   /* clock keep */
        if (host->port == 1)
            nuc970_sd_write(REG_SDCSR, nuc970_sd_read(REG_SDCSR) | 0x80000000); /* clock keep */
    }
    up(&sdh_fmi_sem);   // release sdio_fmi_sem for whole SD command, include data readd/write.
    mmc_request_done(host->mmc, host->request);
}

static void nuc970_sd_send_stop(struct nuc970_sd_host *host, struct mmc_command *cmd)
{
    unsigned int csr;
    unsigned int block_length;
    unsigned int blocks;

    host->cmd = cmd;

    if (host->port == 0)
    {
        sd_host = host;
        sd_state = 0;
        sd_state_xfer = 0;
    }
    if (host->port == 1)
    {
        sd1_host = host;
        sd1_state = 0;
        sd1_state_xfer = 0;
    }

    if (nuc970_sd_read(REG_FMICSR) != FMICSR_SD_EN)
        nuc970_sd_write(REG_FMICSR, FMICSR_SD_EN);

    nuc970_sd_select_port(host->port);
    csr = nuc970_sd_read(REG_SDCSR) & 0xff00c080;

    csr = csr | (cmd->opcode << 8) | SDCSR_CO_EN;   // set command code and enable command out
    if (host->port == 0)
        sd_event |= SD_EVENT_CMD_OUT;
    if (host->port == 1)
        sd1_event |= SD_EVENT_CMD_OUT;

    if (host->bus_mode == MMC_BUS_WIDTH_4)
        csr |= SDCSR_DBW;

    if (mmc_resp_type(cmd) != MMC_RSP_NONE)
    {
    /* if a response is expected then allow maximum response latancy */

        /* set 136 bit response for R2, 48 bit response otherwise */
        if (mmc_resp_type(cmd) == MMC_RSP_R2)
        {
            csr |= SDCSR_R2_EN;
            if (host->port == 0)
                sd_event |= SD_EVENT_RSP2_IN;
            if (host->port == 1)
                sd1_event |= SD_EVENT_RSP2_IN;
        }
        else
        {
            csr |= SDCSR_RI_EN;
            if (host->port == 0)
                sd_event |= SD_EVENT_RSP_IN;
            if (host->port == 1)
                sd1_event |= SD_EVENT_RSP_IN;
        }
        nuc970_sd_write(REG_SDISR, SDISR_RITO_IF);
        if (host->port == 0)
            sd_ri_timeout = 0;
        if (host->port == 1)
            sd1_ri_timeout = 0;
        nuc970_sd_write(REG_SDTMOUT, 0xffff);
    }

    nuc970_sd_write(REG_SDIER, nuc970_sd_read(REG_SDIER) & ~SDIER_BLKD_IE); //disable SD interrupt & select GPIO detect
    block_length = 0;
    blocks = 0;

    nuc970_sd_write(REG_SDARG, cmd->arg);
    nuc970_sd_write(REG_SDCSR, csr);
    if (host->port == 0)
    {
        sd_send_cmd = 1;
        wake_up_interruptible(&sd_event_wq);
    }
    if (host->port == 1)
    {
        sd1_send_cmd = 1;
        wake_up_interruptible(&sd1_event_wq);
    }
    mmc_request_done(host->mmc, host->request);
}


/*
 * Process the request
 */
static void nuc970_sd_send_request(struct nuc970_sd_host *host)
{
    if (!(host->flags & FL_SENT_COMMAND)) {
        host->flags |= FL_SENT_COMMAND;
        nuc970_sd_send_command(host, host->request->cmd);
    } else if ((!(host->flags & FL_SENT_STOP)) && host->request->stop) {
        host->flags |= FL_SENT_STOP;
        nuc970_sd_send_stop(host, host->request->stop);
    } else {
        if (host->port == 0)
        {
            sd_state = 1;
            wake_up_interruptible(&sd_wq);
        }
        if (host->port == 1)
        {
            sd1_state = 1;
            wake_up_interruptible(&sd1_wq);
        }
        del_timer(&host->timer);
    }
}

/*
 * Handle a command that has been completed
 */
static void nuc970_sd_completed_command(struct nuc970_sd_host *host, unsigned int status)
{
    struct mmc_command *cmd = host->cmd;
    struct mmc_data *data = cmd->data;
    unsigned int i, j, tmp[5], err;
    unsigned char *ptr;

    err = nuc970_sd_read(REG_SDISR);

    if ((err & SDISR_RITO_IF) || (cmd->error)) {
        nuc970_sd_write(REG_SDTMOUT, 0x0);
        nuc970_sd_write(REG_SDISR, SDISR_RITO_IF);
        cmd->error = -ETIMEDOUT;
        cmd->resp[0] = cmd->resp[1] = cmd->resp[2] = cmd->resp[3] = 0;
    } else {
        if (status & SD_EVENT_RSP_IN) {
            // if not R2
            cmd->resp[0] = (nuc970_sd_read(REG_SDRSP0) << 8)|(nuc970_sd_read(REG_SDRSP1) & 0xff);
            cmd->resp[1] = cmd->resp[2] = cmd->resp[3] = 0;
        } else if (status & SD_EVENT_RSP2_IN) {
            // if R2
            ptr = (unsigned char *)FB0_BASE_ADDR;
            for (i=0, j=0; j<5; i+=4, j++)
                tmp[j] = (*(ptr+i)<<24)|(*(ptr+i+1)<<16)|(*(ptr+i+2)<<8)|(*(ptr+i+3));
            for (i=0; i<4; i++)
                cmd->resp[i] = ((tmp[i] & 0x00ffffff)<<8)|((tmp[i+1] & 0xff000000)>>24);
        }
    }
    //nuc970_sd_debug("Event = 0x%0X [0x%08X 0x%08X] <0x%x>\n", status, cmd->resp[0], cmd->resp[1], err);

    if (!cmd->error)
    {
        if ((err & SDISR_CRC_7) == 0)
        {
            if (!(mmc_resp_type(cmd) & MMC_RSP_CRC))
            {
                cmd->error = 0;
                nuc970_sd_write(REG_SDISR, SDISR_CRC_IF);
            }
            else
            {
                cmd->error = -EIO;
                nuc970_sd_debug("Error detected and set to %d/%d (cmd = %d, retries = %d)\n",
                                 cmd->error, data ? data->error : 0, cmd->opcode, cmd->retries);
            }
        }
        else
            cmd->error = 0;

        if (data)
        {
            data->bytes_xfered = 0;
            host->transfer_index = 0;
            host->in_use_index = 0;
            if (data->flags & MMC_DATA_READ)
            {
                if (down_interruptible(&sdh_dmac_sem))  // get sdio_dmac_sem for data reading.
                    return;

                nuc970_sd_write(REG_SDCSR, nuc970_sd_read(REG_SDCSR) | SDCSR_DI_EN);
            }
            if (host->port == 0)
                wait_event_interruptible(sd_wq_xfer, (sd_state_xfer != 0));
            if (host->port == 1)
                wait_event_interruptible(sd1_wq_xfer, (sd1_state_xfer != 0));

            if (data->flags & MMC_DATA_READ)
                up(&sdh_dmac_sem);  // release sdio_dmac_sem for data reading.
        }
    }
    nuc970_sd_send_request(host);
}

/*
 * Handle an MMC request
 */
static int nuc970_sd_card_detect(struct mmc_host *mmc)
{
    struct nuc970_sd_host *host = mmc_priv(mmc);
    int ret;

    if (nuc970_sd_read(REG_FMICSR) != FMICSR_SD_EN)
        nuc970_sd_write(REG_FMICSR, FMICSR_SD_EN);

    nuc970_sd_select_port(host->port);
    if (host->port == 0)
        host->present = nuc970_sd_read(REG_SDISR) & SDISR_CDPS0;
    if (host->port == 1)
        host->present = nuc970_sd_read(REG_SDISR) & SDISR_CDPS1;

    ret = host->present ? 0 : 1;
    return ret;
}

static void nuc970_sd_request(struct mmc_host *mmc, struct mmc_request *mrq)
{
    struct nuc970_sd_host *host = mmc_priv(mmc);
    int card_present;
    host->request = mrq;
    host->flags = 0;

    /* more than 1s timeout needed with slow SD cards */
    //mod_timer(&host->timer, jiffies +  msecs_to_jiffies(2000));
    if (down_interruptible(&sdh_fmi_sem))
        return -ENOSYS;
	card_present = nuc970_sd_card_detect(mmc);
    up(&sdh_fmi_sem);

    if (card_present == 0) {
        nuc970_sd_debug("no medium present\n");
        host->request->cmd->error = -ENOMEDIUM;
        mmc_request_done(host->mmc, host->request);
    } else
        nuc970_sd_send_request(host);
}

/*
 * Set the IOS
 */
extern unsigned long get_cpu_clk(void);
static void nuc970_sd_set_ios(struct mmc_host *mmc, struct mmc_ios *ios)
{
    struct nuc970_sd_host *host = mmc_priv(mmc);
    host->bus_mode = ios->bus_width;

    if (down_interruptible(&sdh_fmi_sem))
        return;

    /* maybe switch power to the card */
    switch (ios->power_mode)
    {
        case MMC_POWER_OFF:
            //dump_sdh_regs();
            nuc970_sd_write(REG_FMICSR, nuc970_sd_read(REG_FMICSR) & ~FMICSR_SD_EN);
            __raw_writel(0x1, REG_SDECR);
            break;
        case MMC_POWER_UP:
        case MMC_POWER_ON: // enable 74 clocks
            __raw_writel(0, REG_SDECR);
	    nuc970_sd_write(REG_FMICSR, FMICSR_SD_EN);

            if (ios->clock == 0) {
                up(&sdh_fmi_sem);
                return;
            }                
            if (ios->clock <= 400000)
            {
                clk_set_rate(host->upll_clk, 100000000);
                clk_set_rate(host->sd_clk, ios->clock);
                nuc970_sd_write(REG_SDCSR, nuc970_sd_read(REG_SDCSR) | SDCSR_CLK74_OE);
                while (nuc970_sd_read(REG_SDCSR) & SDCSR_CLK74_OE);
            }
            else
                clk_set_rate(host->sd_clk, ios->clock);
            break;
        default:
            WARN_ON(1);
    }

    if (ios->bus_width == MMC_BUS_WIDTH_4)
    {
        nuc970_sd_debug("MMC: Setting controller bus width to 4\n");
        nuc970_sd_write(REG_SDCSR, nuc970_sd_read(REG_SDCSR) | SDCSR_DBW);
    }
    else
    {
        //nuc970_sd_debug("MMC: Setting controller bus width to 1\n");
        nuc970_sd_write(REG_SDCSR, nuc970_sd_read(REG_SDCSR) & ~SDCSR_DBW);
    }
    up(&sdh_fmi_sem);
}


/*
 * Handle CO, RI, and R2 event
 */
static int sd_event_thread(void *unused)
{
    int event = 0;
    int completed = 0;

    for (;;)
    {
        wait_event_interruptible(sd_event_wq, (sd_event != SD_EVENT_NONE) && (sd_send_cmd));
        //printk("sd_event_thread - sd_event_wq raised!\n");

        completed = 0;
        event = sd_event;
        sd_event = SD_EVENT_NONE;
        sd_send_cmd = 0;
        if (event & SD_EVENT_CMD_OUT)
        {
            while (1)
            {
                if (!(nuc970_sd_read(REG_SDCSR) & SDCSR_CO_EN))
                {
                    completed = 1;
                    break;
                }
            }
        }

        if (event & SD_EVENT_RSP_IN)
        {
            while (1)
            {
                if (!(nuc970_sd_read(REG_SDCSR) & SDCSR_RI_EN))
                {
                    completed = 1;
                    break;
                }

                if (nuc970_sd_read(REG_SDISR) & SDISR_RITO_IF)
                {
                    nuc970_sd_write(REG_SDTMOUT, 0x0);
                    nuc970_sd_write(REG_SDISR, SDISR_RITO_IF);

                    completed = 1;
                    sd_host->cmd->error = -ETIMEDOUT;
                    break;
                }
            }
        }

        if (event & SD_EVENT_RSP2_IN)
        {
            while (1)
            {
                if (!(nuc970_sd_read(REG_SDCSR) & SDCSR_R2_EN))
                {
                    completed = 1;
                    break;
                }
            }
        }
        if (completed)
        {
            //nuc970_sd_debug("Completed command\n");
            nuc970_sd_completed_command(sd_host, event);
        }
    }
    nuc970_sd_debug("SD0 event quit\n");
    return 0;
}

static int sd1_event_thread(void *unused)
{
    int event = 0;
    int completed = 0;

    for (;;)
    {
        wait_event_interruptible(sd1_event_wq, (sd1_event != SD_EVENT_NONE) && (sd1_send_cmd));
        //printk("sd_event_thread - sd_event_wq raised!\n");

        completed = 0;
        event = sd1_event;
        sd1_event = SD_EVENT_NONE;
        sd1_send_cmd = 0;
        if (event & SD_EVENT_CMD_OUT)
        {
            while (1)
            {
                if (!(nuc970_sd_read(REG_SDCSR) & SDCSR_CO_EN))
                {
                    completed = 1;
                    break;
                }
            }
        }

        if (event & SD_EVENT_RSP_IN)
        {
            while (1)
            {
                if (!(nuc970_sd_read(REG_SDCSR) & SDCSR_RI_EN))
                {
                    completed = 1;
                    break;
                }

                if (nuc970_sd_read(REG_SDISR) & SDISR_RITO_IF)
                {
                    nuc970_sd_write(REG_SDTMOUT, 0x0);
                    nuc970_sd_write(REG_SDISR, SDISR_RITO_IF);

                    completed = 1;
                    sd1_host->cmd->error = -ETIMEDOUT;
                    break;
                }
            }
        }

        if (event & SD_EVENT_RSP2_IN)
        {
            while (1)
            {
                if (!(nuc970_sd_read(REG_SDCSR) & SDCSR_R2_EN))
                {
                    completed = 1;
                    break;
                }
            }
        }
        if (completed)
        {
            //nuc970_sd_debug("Completed command\n");
            nuc970_sd_completed_command(sd1_host, event);
        }
    }
    nuc970_sd_debug("SD1 event quit\n");
    return 0;
}

/*
 * Handle an interrupt
 */
static irqreturn_t nuc970_sd_irq(int irq, void *devid)
{
    struct nuc970_sd_host *host = devid;
    unsigned int int_status, present;
    unsigned int reg_port_select;

    int_status = nuc970_sd_read(REG_SDISR);
    reg_port_select = (nuc970_sd_read(REG_SDCSR) & 0x60000000) >> 29;

    //printk("FMI irq: status = %08X <0x%x, csr 0x%x>\n", int_status, nuc970_sd_read(REG_MFP_GPD_L), nuc970_sd_read(REG_SDCSR));
    if (host->port == 0)
    {
        if (int_status & 0x400) /* sdio 0 interrupt */
        {
            nuc970_sd_write(REG_SDIER, nuc970_sd_read(REG_SDIER) & ~0x400);
            nuc970_sd_write(REG_SDISR, 0x400);
            mmc_signal_sdio_irq(host->mmc);
        }
    }

    if (host->port == 1)
    {
        if (int_status & 0x800) /* sdio 1 interrupt */
        {
            nuc970_sd_write(REG_SDIER, nuc970_sd_read(REG_SDIER) & ~0x800);
            nuc970_sd_write(REG_SDISR, 0x800);
            mmc_signal_sdio_irq(host->mmc);
        }
    }

    if (int_status & SDISR_BLKD_IF) {
        nuc970_sd_debug("SDH xfer done.\n");
        if (host->port != reg_port_select)
        {
            // This shared interrupt is not come from this device host. Ignore it.
            //nvt_sd_debug("SDIO %d irq: return IRQ_NONE.\n", host->port);
            return IRQ_NONE;    // interrupt was not from this device
        }

        if ((host->cmd == 0) || (host->cmd->data == 0))
        {
            nuc970_sd_debug("SD %d irq: port select = %d, found NULL pointer!!\n", host->port, reg_port_select);
            return IRQ_NONE;
        }

        if (host->cmd->data->flags & MMC_DATA_WRITE) {
            nuc970_sd_handle_transmitted(host);
        } else if (host->cmd->data->flags & MMC_DATA_READ) {
            nuc970_sd_post_dma_read(host);

            //-- - check CRC-16 error for data-in transfer
            if (int_status & SDISR_CRC_IF)
            {
                // 2014/5/16, according to comment in mmc_sd_init_card() in core/sd.c, it said
                //      "This CRC enable is located AFTER the reading of the
	            //       card registers because some SDHC cards are not able
	            //       to provide valid CRCs for non-512-byte blocks."
	            // The ACMD51 will read 8 bytes SCR register and could trigger invalid CRC-16 error.
	            // The CMD6 will read 64 bytes SSR register and could trigger invalid CRC-16 error.
	            // So, SD driver ignore CRC-16 error check for command code 51 and 6 here.
                if ((host->cmd->opcode != 51) && (host->cmd->opcode != 6))
                {
                    host->cmd->error = -EIO;
                    if ((nuc970_sd_read(REG_SDISR) & SDISR_CRC_16) == 0)
                        nuc970_sd_debug("CRC-16 error detected and set to %d (cmd = %d, retries = %d, SDIOISR=0x%08X)\n",
                                      host->cmd->error, host->cmd->opcode, host->cmd->retries, int_status);
                    nuc970_sd_write(REG_SDISR, SDISR_CRC_IF);  // clear CRC interrupt flag
                    // When CRC error is occurred, software should reset SDIO engine.
                    nuc970_sd_write(REG_SDCSR, nuc970_sd_read(REG_SDCSR) | SDCSR_SW_RST); // software reset SDIO engine
                    while (nuc970_sd_read(REG_SDCSR) & SDCSR_SW_RST); // waiting for reset completed
                }
            }
        }
        nuc970_sd_write(REG_SDISR, SDISR_BLKD_IF);
        if (host->port == 0)
        {
            sd_state_xfer = 1;
            wake_up_interruptible(&sd_wq_xfer);
        }
        if (host->port == 1)
        {
            sd1_state_xfer = 1;
            wake_up_interruptible(&sd1_wq_xfer);
        }
    }

    /*
     * we expect this irq on both insert and remove,
     * and use a short delay to debounce.
     */

    if (host->port == 0)
    {
        /* SD card port 0 detect */
        if (int_status & SDISR_CD0_IF)
        {
	        present = int_status & SDISR_CDPS0;
	        host->present = present;
	        nuc970_sd_debug("%s: card %s\n", mmc_hostname(host->mmc), present ? "remove" : "insert");
	        if (!present)
	        {
	            nuc970_sd_debug("****** Resetting SD-card bus width ******\n");
	            nuc970_sd_write(REG_SDCSR, nuc970_sd_read(REG_SDCSR) & ~SDCSR_DBW);
	        }
	        /* 0.5s needed because of early card detect switch firing */
	        mmc_detect_change(host->mmc, msecs_to_jiffies(500));
	        nuc970_sd_write(REG_SDISR, SDISR_CD0_IF);
        }
    }
    if (host->port == 1)
    {
        /* SD card port 1 detect */
        if (int_status & SDISR_CD1_IF) {
	        present = int_status & SDISR_CDPS1;
	        host->present = present;
	        nuc970_sd_debug("%s: card %s\n", mmc_hostname(host->mmc), present ? "remove" : "insert");
	        if (!present) {
	            nuc970_sd_debug("****** Resetting SD-card bus width ******\n");
	            nuc970_sd_write(REG_SDCSR, nuc970_sd_read(REG_SDCSR) & ~SDCSR_DBW);
	        }
            /* 0.5s needed because of early card detect switch firing */
            mmc_detect_change(host->mmc, msecs_to_jiffies(500));
            nuc970_sd_write(REG_SDISR, SDISR_CD1_IF);
        }
    }

    return IRQ_HANDLED;
}

static int nuc970_sd_get_ro(struct mmc_host *mmc)
{
    /* TODO: check write protect pin */
    /* if write protect, it should return >0 value */

    /* no write protect */
    return 0;

    /*
     * Board doesn't support read only detection; let the mmc core
     * decide what to do.
     */
    //return -ENOSYS;
}

static void nuc900_sd_enable_sdio_irq(struct mmc_host *mmc, int enable)
{
    struct nuc970_sd_host *host = mmc_priv(mmc);

    if (down_interruptible(&sdh_fmi_sem))
        return;

    nuc970_sd_write(REG_DMACIER, DMACIER_TABORT_IE);    //Enable target abort interrupt generation during DMA transfer
    nuc970_sd_write(REG_FMIIER, FMIIER_DTA_IE); //Enable DMAC READ/WRITE target abort interrupt generation

    if (host->port == 0)
    {
        nuc970_sd_write(REG_SDIER, nuc970_sd_read(REG_SDIER) & ~0x400);
        nuc970_sd_write(REG_SDISR, 0x400);
        if (enable)
        {
            nuc970_sd_write(REG_SDIER, nuc970_sd_read(REG_SDIER) | 0x4400); /* sdio interrupt */
            nuc970_sd_write(REG_SDCSR, nuc970_sd_read(REG_SDCSR) | 0x80);   /* clock keep */
        }
        else
        {
            nuc970_sd_write(REG_SDIER, nuc970_sd_read(REG_SDIER) & ~0x4400);/* sdio interrupt */
            nuc970_sd_write(REG_SDCSR, nuc970_sd_read(REG_SDCSR) & ~0x80);  /* clock keep */
        }
    }

    if (host->port == 1)
    {
        nuc970_sd_write(REG_SDIER, nuc970_sd_read(REG_SDIER) & ~0x800);
        nuc970_sd_write(REG_SDISR, 0x800);
        if (enable)
        {
            nuc970_sd_write(REG_SDIER, nuc970_sd_read(REG_SDIER) | 0x4800); /* sdio interrupt */
            nuc970_sd_write(REG_SDCSR, nuc970_sd_read(REG_SDCSR) | 0x80000000); /* clock keep */
        }
        else
        {
            nuc970_sd_write(REG_SDIER, nuc970_sd_read(REG_SDIER) & ~0x4800);/* sdio interrupt */
            nuc970_sd_write(REG_SDCSR, nuc970_sd_read(REG_SDCSR) & ~0x80000000);    /* clock keep */
        }
    }
    up(&sdh_fmi_sem);
}

static const struct mmc_host_ops nuc970_sd_ops = {
        .request    = nuc970_sd_request,
        .set_ios    = nuc970_sd_set_ios,
        .get_ro     = nuc970_sd_get_ro,
        .get_cd     = nuc970_sd_card_detect,
        .enable_sdio_irq = nuc900_sd_enable_sdio_irq,
};

/*
 * Probe for the device
 */
static int nuc970_sd_probe(struct platform_device *pdev)
{
    struct mmc_host *mmc, *mmc1;
    struct nuc970_sd_host *host, *host1;
    struct resource *res;
    int ret;
    struct clk *clkmux;
    struct pinctrl *p;

    res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
    if (!res)
        return -ENXIO;

    if (!request_mem_region(res->start, res->end - res->start + 1, DRIVER_NAME))
        return -EBUSY;

    sema_init(&sdh_fmi_sem, 1);
    sema_init(&sdh_dmac_sem, 1);

#if defined (CONFIG_NUC970_SD_TWO)
#if defined (CONFIG_NUC970_SD1_PE)
    /* initial SD0 -> PD0~7, SD1 pin -> PE2~9 */
    p = devm_pinctrl_get_select(&pdev->dev, "sd01-PE");

#elif defined (CONFIG_NUC970_SD1_PH)
    /* initial SD0 -> PD0~7, SD1 pin -> PH6~13 */
    p = devm_pinctrl_get_select(&pdev->dev, "sd01-PH");

#elif defined (CONFIG_NUC970_SD1_PI)
    /* initial SD0 -> PD0~7, SD1 pin -> PI5~10, 12~13 */
    p = devm_pinctrl_get_select(&pdev->dev, "sd01-PI");

#endif
    if (IS_ERR(p))
    {
        dev_err(&pdev->dev, "unable to reserve pin\n");
        ret = PTR_ERR(p);
    }
#endif

    /* SD Port 0 initial */
    mmc = mmc_alloc_host(sizeof(struct nuc970_sd_host), &pdev->dev);
    if (!mmc) {
        ret = -ENOMEM;
        dev_dbg(&pdev->dev, "couldn't allocate mmc host\n");
        goto fail6;
    }

    mmc->ops = &nuc970_sd_ops;
    mmc->f_min = 300000;
    mmc->f_max = 50000000;
    mmc->ocr_avail = MMC_VDD_27_28|MMC_VDD_28_29|MMC_VDD_29_30|MMC_VDD_30_31|MMC_VDD_31_32|MMC_VDD_32_33 | MMC_VDD_33_34;
    mmc->caps = 0;
    mmc->max_blk_size  = MCI_MAXBLKSIZE;
    mmc->max_blk_count = MCI_BLKATONCE;
    mmc->max_req_size  = MCI_BUFSIZE;
    mmc->max_segs      = MCI_BLKATONCE;
    mmc->max_seg_size  = MCI_BUFSIZE;

    host = mmc_priv(mmc);
    sd_host = host;
    host->mmc = mmc;
    host->bus_mode = 0;
    mmc->caps |= (MMC_CAP_4_BIT_DATA|MMC_CAP_SDIO_IRQ|MMC_CAP_SD_HIGHSPEED|MMC_CAP_MMC_HIGHSPEED);
    host->port = 0;     // default SD port to check

    host->buffer = dma_alloc_coherent(&pdev->dev, MCI_BUFSIZE, &host->physical_address, GFP_KERNEL);
    if (!host->buffer) {
            ret = -ENOMEM;
            dev_err(&pdev->dev, "Can't allocate transmit buffer\n");
            goto fail5;
    }

    clk_prepare(clk_get(NULL, "sdh_hclk"));
    clk_enable(clk_get(NULL, "sdh_hclk"));
    host->sd_clk = clk_get(NULL, "sdh_eclk");
    if (IS_ERR(host->sd_clk)) {
        ret = -ENODEV;
        dev_dbg(&pdev->dev, "no sd_clk?\n");
        goto fail2;
    }
    clk_prepare(host->sd_clk);
    clk_enable(host->sd_clk);       /* Enable the peripheral clock */

    clkmux = clk_get(NULL, "sdh_eclk_mux");
    if (IS_ERR(clkmux)) {
        printk(KERN_ERR "nuc970-sdh:failed to get sdh clock source\n");
        ret = PTR_ERR(clkmux);
        return ret;
    }

    host->upll_clk = clk_get(NULL, "sdh_uplldiv");
    if (IS_ERR(host->upll_clk)) {
        printk(KERN_ERR "nuc970-sdh:failed to get sdh clock source\n");
        ret = PTR_ERR(host->upll_clk);
        return ret;
    }
    clk_set_parent(clkmux, host->upll_clk);
    clk_set_rate(host->upll_clk, 33000000);

    nuc970_sd_disable(host);
    nuc970_sd_enable(host);

    /*
     * Allocate the MCI interrupt
     */
    host->irq = platform_get_irq(pdev, 0);
    ret = request_irq(host->irq, nuc970_sd_irq, IRQF_SHARED, mmc_hostname(mmc), host);
    if (ret) {
        dev_dbg(&pdev->dev, "request MCI interrupt failed\n");
        goto fail0;
    }

    /* add a thread to check CO, RI, and R2 */
    kernel_thread(sd_event_thread, NULL, 0);
    setup_timer(&host->timer, nuc970_sd_timeout_timer, (unsigned long)host);
    platform_set_drvdata(pdev, mmc);

    /*
     * Add host to MMC layer
     */
    host->present = nuc970_sd_read(REG_SDISR) & SDISR_CDPS0;
    nuc970_sd_write(REG_SDIER, nuc970_sd_read(REG_SDIER) | SDIER_CD0_IE | SDIER_CD0SRC);    //Enable SD interrupt & select GPIO detect

    mmc_add_host(mmc);
    nuc970_sd_debug("Added NUC970 SD0 driver\n");

    /* SD Port 1 initial */
    mmc1 = mmc_alloc_host(sizeof(struct nuc970_sd_host), &pdev->dev);
    if (!mmc1) {
        ret = -ENOMEM;
        dev_dbg(&pdev->dev, "couldn't allocate mmc1 host\n");
        goto fail6;
    }

    mmc1->ops = &nuc970_sd_ops;
    mmc1->f_min = 300000;
    mmc1->f_max = 50000000;
    mmc1->ocr_avail = MMC_VDD_27_28|MMC_VDD_28_29|MMC_VDD_29_30|MMC_VDD_30_31|MMC_VDD_31_32|MMC_VDD_32_33 | MMC_VDD_33_34;
    mmc1->caps = 0;
    mmc1->max_blk_size  = MCI_MAXBLKSIZE;
    mmc1->max_blk_count = MCI_BLKATONCE;
    mmc1->max_req_size  = MCI_BUFSIZE;
    mmc1->max_segs      = MCI_BLKATONCE;
    mmc1->max_seg_size  = MCI_BUFSIZE;

    host1 = mmc_priv(mmc1);
    sd1_host = host1;
    host1->mmc = mmc1;
    host1->bus_mode = 0;
    mmc1->caps |= (MMC_CAP_4_BIT_DATA|MMC_CAP_SDIO_IRQ|MMC_CAP_SD_HIGHSPEED|MMC_CAP_MMC_HIGHSPEED);
    host1->port = 1;     // default SD port to check

    host1->buffer = dma_alloc_coherent(&pdev->dev, MCI_BUFSIZE, &host1->physical_address, GFP_KERNEL);
    if (!host1->buffer) {
            ret = -ENOMEM;
            dev_err(&pdev->dev, "Can't allocate transmit buffer 1\n");
            goto fail5;
    }

    host1->sd_clk = host->sd_clk;
    host1->upll_clk = host->upll_clk;

    nuc970_sd_disable(host1);
    nuc970_sd_enable(host1);

    /*
     * Allocate the MCI interrupt
     */
    host1->irq = platform_get_irq(pdev, 0);
    ret = request_irq(host1->irq, nuc970_sd_irq, IRQF_SHARED, mmc_hostname(mmc1), host1);
    if (ret) {
        dev_dbg(&pdev->dev, "request MCI interrupt failed\n");
        goto fail0;
    }

    /* add a thread to check CO, RI, and R2 */
    kernel_thread(sd1_event_thread, NULL, 0);
    setup_timer(&host1->timer, nuc970_sd_timeout_timer, (unsigned long)host1);
    platform_set_drvdata(pdev, mmc1);

    /*
     * Add host to MMC layer
     */
    host1->present = nuc970_sd_read(REG_SDISR) & SDISR_CDPS1;
    nuc970_sd_write(REG_SDIER, nuc970_sd_read(REG_SDIER) | SDIER_CD1_IE | SDIER_CD1SRC);    //Enable SD interrupt & select GPIO detect

    mmc_add_host(mmc1);

    nuc970_sd_debug("Added NUC970 SD1 driver\n");

    return 0;

fail0:
    clk_disable(host->sd_clk);
    clk_put(host->sd_clk);
    clk_disable(host1->sd_clk);
    clk_put(host1->sd_clk);
fail2:
    if (host->buffer)
        dma_free_coherent(&pdev->dev, MCI_BUFSIZE, host->buffer, host->physical_address);
    if (host1->buffer)
        dma_free_coherent(&pdev->dev, MCI_BUFSIZE, host1->buffer, host1->physical_address);
fail5:
    mmc_free_host(mmc);
    mmc_free_host(mmc1);
fail6:
    release_mem_region(res->start, res->end - res->start + 1);
    dev_err(&pdev->dev, "probe failed, err %d\n", ret);
    return ret;
}

/*
 * Remove a device
 */
static int nuc970_sd_remove(struct platform_device *pdev)
{
    struct mmc_host *mmc = platform_get_drvdata(pdev);
    struct nuc970_sd_host *host;

    if (!mmc)
        return -1;

    host = mmc_priv(mmc);

    if (host->buffer)
        dma_free_coherent(&pdev->dev, MCI_BUFSIZE, host->buffer, host->physical_address);

    nuc970_sd_disable(host);
    del_timer_sync(&host->timer);
    mmc_remove_host(mmc);
    free_irq(host->irq, host);

    clk_disable(host->sd_clk);            /* Disable the peripheral clock */
    clk_put(host->sd_clk);

    mmc_free_host(mmc);
    platform_set_drvdata(pdev, NULL);
    nuc970_sd_debug("NUC970 SD Removed\n");
    return 0;
}

#ifdef CONFIG_PM
static int nuc970_sd_suspend(struct platform_device *pdev, pm_message_t state)
{
    struct mmc_host *mmc = platform_get_drvdata(pdev);
    //struct nuc970_sd_host *host = mmc_priv(mmc);
    int ret = 0;

    if (mmc)
        ret = mmc_suspend_host(mmc);

    return ret;
}

static int nuc970_sd_resume(struct platform_device *pdev)
{
    struct mmc_host *mmc = platform_get_drvdata(pdev);
    //struct nuc970_sd_host *host = mmc_priv(mmc);
    int ret = 0;

    if (mmc)
        ret = mmc_resume_host(mmc);

    return ret;
}
#else
#define nuc970_sd_suspend   NULL
#define nuc970_sd_resume    NULL
#endif

static struct platform_driver nuc970_sd_driver = {
        .probe      = nuc970_sd_probe,
        .remove     = nuc970_sd_remove,
        .suspend    = nuc970_sd_suspend,
        .resume     = nuc970_sd_resume,
        .driver     = {
                .name   = DRIVER_NAME,
                .owner  = THIS_MODULE,
        },
};


module_platform_driver(nuc970_sd_driver);

MODULE_DESCRIPTION("NUC970 dual SD Card Interface driver");
MODULE_AUTHOR("HPChen");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:nuc970_sd");
