/*
 *  linux/drivers/mmc/host/nuc970_emmc.c - Nuvoton NUC970 SD Driver
 *
 *  Copyright (C) 2014 Cougar Creek Computing Devices Ltd, All Rights Reserved
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */


#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
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
#include <mach/regs-fmi.h>

//#define nuc970_emmc_debug       printk
#define nuc970_emmc_debug(...)

#define DRIVER_NAME    "nuc970-fmi"

#define FL_SENT_COMMAND (1 << 0)
#define FL_SENT_STOP    (1 << 1)

#define nuc970_emmc_read(reg)          __raw_readl(reg)
#define nuc970_emmc_write(reg, val)    __raw_writel((val), (reg))

#define MCI_BLKSIZE         512
#define MCI_MAXBLKSIZE      4095
#define MCI_BLKATONCE       255
#define MCI_BUFSIZE         (MCI_BLKSIZE * MCI_BLKATONCE)

/* Driver thread command */
#define EMMC_EVENT_NONE       0x00000000
#define EMMC_EVENT_CMD_OUT    0x00000001
#define EMMC_EVENT_RSP_IN     0x00000010
#define EMMC_EVENT_RSP2_IN    0x00000100
#define EMMC_EVENT_CLK_KEEP0  0x00001000
#define EMMC_EVENT_CLK_KEEP1  0x00010000

static volatile int emmc_event=0, emmc_state=0, emmc_state_xfer=0, emmc_ri_timeout=0, emmc_send_cmd=0;
static DECLARE_WAIT_QUEUE_HEAD(emmc_event_wq);
static DECLARE_WAIT_QUEUE_HEAD(emmc_wq);
static DECLARE_WAIT_QUEUE_HEAD(emmc_wq_xfer);

/*
 * Low level type for this driver
 */
struct nuc970_emmc_host {
    struct mmc_host *mmc;
    struct mmc_command *cmd;
    struct mmc_request *request;

    void __iomem *emmc_base;
    int irq;

    int present;

    struct clk *emmc_clk, *upll_clk;

    /*
     * Flag indicating when the command has been sent. This is used to
     * work out whether or not to send the stop
     */
    unsigned int flags;
    /* flag for current port */
    u32 bus_mode;

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

struct nuc970_emmc_host *emmc_host;

/*
 * Reset the controller and restore most of the state
 */
static void nuc970_emmc_reset_host(struct nuc970_emmc_host *host)
{
    unsigned long flags;

    local_irq_save(flags);

    nuc970_emmc_write(REG_NAND_DMACCSR, DMACCSR_DMAC_EN | DMACCSR_SWRST); //enable DMAC for FMI
    nuc970_emmc_write(REG_NAND_FMICSR, FMICSR_SWRST); /* Enable emmc functionality of FMI */
    nuc970_emmc_write(REG_NAND_FMICSR, FMICSR_EMMCEN);
    local_irq_restore(flags);
}

static void nuc970_emmc_timeout_timer(unsigned long data)
{
    struct nuc970_emmc_host *host;

    host = (struct nuc970_emmc_host *)data;

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
        nuc970_emmc_reset_host(host);
        mmc_request_done(host->mmc, host->request);
    }
}

/*
 * Copy from sg to a dma block - used for transfers
 */
static inline void nuc970_emmc_sg_to_dma(struct nuc970_emmc_host *host, struct mmc_data *data)
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
static void nuc970_emmc_post_dma_read(struct nuc970_emmc_host *host)
{
    struct mmc_command *cmd;
    struct mmc_data *data;
    unsigned int len, i, size;
    unsigned *dmabuf = host->buffer;

    cmd = host->cmd;
    if (!cmd) {
        nuc970_emmc_debug("no command\n");
        return;
    }

    data = cmd->data;
    if (!data) {
        nuc970_emmc_debug("no data\n");
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
static void nuc970_emmc_handle_transmitted(struct nuc970_emmc_host *host)
{
    //nuc970_emmc_debug("Handling the transmit\n");
    if (nuc970_emmc_read(REG_EMMCISR) & EMMCISR_CRC_IF)
        nuc970_emmc_write(REG_EMMCISR, EMMCISR_CRC_IF);

    /* check read/busy */
    nuc970_emmc_write(REG_EMMCCSR, nuc970_emmc_read(REG_EMMCCSR) | EMMCCSR_CLK8_OE);
}


/*
 * Update bytes tranfered count during a write operation
 */
static void nuc970_emmc_update_bytes_xfered(struct nuc970_emmc_host *host)
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


/*
 * Enable the controller
 */
static void nuc970_emmc_enable(struct nuc970_emmc_host *host)
{
    nuc970_emmc_write(REG_NAND_DMACCSR, DMACCSR_DMAC_EN);   // enable DMAC for FMI
    nuc970_emmc_write(REG_NAND_FMICSR, FMICSR_EMMCEN);  /* Enable SD functionality of FMI */
    nuc970_emmc_write(REG_EMMCISR, 0xffffffff);

    nuc970_emmc_write(REG_EMMCCSR, (nuc970_emmc_read(REG_EMMCCSR) & ~0xfff0000)|0x09010000);
}

/*
 * Disable the controller
 */
static void nuc970_emmc_disable(struct nuc970_emmc_host *host)
{
    nuc970_emmc_write(REG_NAND_DMACCSR, DMACCSR_DMAC_EN | DMACCSR_SWRST); //enable DMAC for FMI
    nuc970_emmc_write(REG_NAND_FMICSR, FMICSR_SWRST); /* Enable EMMC functionality of FMI */
    nuc970_emmc_write(REG_EMMCISR, 0xffffffff);
    nuc970_emmc_write(REG_NAND_FMICSR, nuc970_emmc_read(REG_NAND_FMICSR) & ~FMICSR_EMMCEN);
}

/*
 * Send a command
 */
static void nuc970_emmc_send_command(struct nuc970_emmc_host *host, struct mmc_command *cmd)
{
    unsigned int volatile csr;
    unsigned int block_length;
    struct mmc_data *data = cmd->data;
    unsigned int blocks;

    host->cmd = cmd;
    emmc_host = host;
    emmc_state = 0;
    emmc_state_xfer = 0;

    if (nuc970_emmc_read(REG_NAND_FMICSR) != FMICSR_EMMCEN)
        nuc970_emmc_write(REG_NAND_FMICSR, FMICSR_EMMCEN);

    csr = nuc970_emmc_read(REG_EMMCCSR) & 0xff00c080;

    csr = csr & ~0x80;
    csr = csr | (cmd->opcode << 8) | EMMCCSR_CO_EN;   // set command code and enable command out
    emmc_event |= EMMC_EVENT_CMD_OUT;

    if (host->bus_mode == MMC_BUS_WIDTH_4)
        csr |= EMMCCSR_DBW;

    if (mmc_resp_type(cmd) != MMC_RSP_NONE)
    {
        /* if a response is expected then allow maximum response latancy */
        /* set 136 bit response for R2, 48 bit response otherwise */
        if (mmc_resp_type(cmd) == MMC_RSP_R2)
        {
            csr |= EMMCCSR_R2_EN;
            emmc_event |= EMMC_EVENT_RSP2_IN;
        }
        else
        {
            csr |= EMMCCSR_RI_EN;
            emmc_event |= EMMC_EVENT_RSP_IN;
        }
        nuc970_emmc_write(REG_EMMCISR, EMMCISR_RITO_IF);
        emmc_ri_timeout = 0;
        nuc970_emmc_write(REG_EMMCTMOUT, 0xffff);
    }

    if (data)
    {
        nuc970_emmc_write(REG_EMMCIER, nuc970_emmc_read(REG_EMMCIER) | EMMCIER_BLKD_IE);  //Enable EMMC interrupt
        block_length = data->blksz;
        blocks = data->blocks;

        nuc970_emmc_write(REG_EMMCBLEN, block_length-1);
        if ((block_length > 512) || (blocks >= 256))
            printk("ERROR: don't support read/write 256 blocks in on CMD\n");
        else
            csr = (csr & ~0x00ff0000) | (blocks << 16);
    }
    else
    {
        nuc970_emmc_write(REG_EMMCIER, nuc970_emmc_read(REG_EMMCIER) & ~EMMCIER_BLKD_IE); //disable SD interrupt
        block_length = 0;
        blocks = 0;
    }

    /*
     * Set the arguments and send the command
     */
    nuc970_emmc_debug("Sending command %d as 0x%0X, arg = 0x%08X, blocks = %d, length = %d\n",
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
            nuc970_emmc_write(REG_EMMCTMOUT, 0x3fffff);
            host->total_length = 0;
            nuc970_emmc_write(REG_NAND_DMACSAR, host->physical_address);
            nuc970_emmc_debug("EMMC - Reading %d bytes [phy_addr = 0x%x]\n", block_length * blocks, host->physical_address);
        }
        else if (data->flags & MMC_DATA_WRITE)
        {
            host->total_length = block_length * blocks;
            nuc970_emmc_sg_to_dma(host, data);
            nuc970_emmc_debug("EMMC - Transmitting %d bytes\n", host->total_length);
            nuc970_emmc_write(REG_NAND_DMACSAR, host->physical_address);
            csr = csr | EMMCCSR_DO_EN;
        }
    }
    /*
     * Send the command and then enable the PDC - not the other way round as
     * the data sheet says
     */
    nuc970_emmc_write(REG_EMMCARG, cmd->arg);
    nuc970_emmc_write(REG_EMMCCSR, csr);
    emmc_send_cmd = 1;
    wake_up_interruptible(&emmc_event_wq);
    wait_event_interruptible(emmc_wq, (emmc_state != 0));

    if (data)
    {
        if (data->flags & MMC_DATA_WRITE)
        {
            while (!(nuc970_emmc_read(REG_EMMCISR) & EMMCISR_SDDAT0))
            {
                nuc970_emmc_write(REG_EMMCCSR, nuc970_emmc_read(REG_EMMCCSR) | EMMCCSR_CLK8_OE);
                while (nuc970_emmc_read(REG_EMMCCSR) & EMMCCSR_CLK8_OE);
            }
            nuc970_emmc_update_bytes_xfered(host);
        }
    }
    mmc_request_done(host->mmc, host->request);
}

static void nuc970_emmc_send_stop(struct nuc970_emmc_host *host, struct mmc_command *cmd)
{
    unsigned int csr;
    unsigned int block_length;
    unsigned int blocks;

    host->cmd = cmd;
    emmc_host = host;
    emmc_state = 0;
    emmc_state_xfer = 0;

    if (nuc970_emmc_read(REG_NAND_FMICSR) != FMICSR_EMMCEN)
        nuc970_emmc_write(REG_NAND_FMICSR, FMICSR_EMMCEN);

    csr = nuc970_emmc_read(REG_EMMCCSR) & 0xff00c080;

    csr = csr | (cmd->opcode << 8) | EMMCCSR_CO_EN;   // set command code and enable command out
    emmc_event |= EMMC_EVENT_CMD_OUT;

    if (host->bus_mode == MMC_BUS_WIDTH_4)
        csr |= EMMCCSR_DBW;

    if (mmc_resp_type(cmd) != MMC_RSP_NONE)
    {
    /* if a response is expected then allow maximum response latancy */

        /* set 136 bit response for R2, 48 bit response otherwise */
        if (mmc_resp_type(cmd) == MMC_RSP_R2)
        {
            csr |= EMMCCSR_R2_EN;
            emmc_event |= EMMC_EVENT_RSP2_IN;
        }
        else
        {
            csr |= EMMCCSR_RI_EN;
            emmc_event |= EMMC_EVENT_RSP_IN;
        }
        nuc970_emmc_write(REG_EMMCISR, EMMCISR_RITO_IF);
        emmc_ri_timeout = 0;
        nuc970_emmc_write(REG_EMMCTMOUT, 0xffff);
    }

    nuc970_emmc_write(REG_EMMCIER, nuc970_emmc_read(REG_EMMCIER) & ~EMMCIER_BLKD_IE); //disable SD interrupt
    block_length = 0;
    blocks = 0;

    nuc970_emmc_write(REG_EMMCARG, cmd->arg);
    nuc970_emmc_write(REG_EMMCCSR, csr);
    emmc_send_cmd = 1;
    wake_up_interruptible(&emmc_event_wq);

    mmc_request_done(host->mmc, host->request);
}


/*
 * Process the request
 */
static void nuc970_emmc_send_request(struct nuc970_emmc_host *host)
{
    if (!(host->flags & FL_SENT_COMMAND)) {
        host->flags |= FL_SENT_COMMAND;
        nuc970_emmc_send_command(host, host->request->cmd);
    } else if ((!(host->flags & FL_SENT_STOP)) && host->request->stop) {
        host->flags |= FL_SENT_STOP;
        nuc970_emmc_send_stop(host, host->request->stop);
    } else {
        emmc_state = 1;
        wake_up_interruptible(&emmc_wq);
        del_timer(&host->timer);
    }
}

/*
 * Handle a command that has been completed
 */
static void nuc970_emmc_completed_command(struct nuc970_emmc_host *host, unsigned int status)
{
    struct mmc_command *cmd = host->cmd;
    struct mmc_data *data = cmd->data;
    unsigned int i, j, tmp[5], err;
    unsigned char *ptr;

    err = nuc970_emmc_read(REG_EMMCISR);

    if ((err & EMMCISR_RITO_IF) || (cmd->error)) {
        nuc970_emmc_write(REG_EMMCTMOUT, 0x0);
        nuc970_emmc_write(REG_EMMCISR, EMMCISR_RITO_IF);
        cmd->error = -ETIMEDOUT;
        cmd->resp[0] = cmd->resp[1] = cmd->resp[2] = cmd->resp[3] = 0;
    } else {
        if (status & EMMC_EVENT_RSP_IN) {
            // if not R2
            cmd->resp[0] = (nuc970_emmc_read(REG_EMMCRSP0) << 8)|(nuc970_emmc_read(REG_EMMCRSP1) & 0xff);
            cmd->resp[1] = cmd->resp[2] = cmd->resp[3] = 0;
        } else if (status & EMMC_EVENT_RSP2_IN) {
            // if R2
            ptr = (unsigned char *)REG_NAND_FB0;
            for (i=0, j=0; j<5; i+=4, j++)
                tmp[j] = (*(ptr+i)<<24)|(*(ptr+i+1)<<16)|(*(ptr+i+2)<<8)|(*(ptr+i+3));
            for (i=0; i<4; i++)
                cmd->resp[i] = ((tmp[i] & 0x00ffffff)<<8)|((tmp[i+1] & 0xff000000)>>24);
        }
    }
    //nuc970_emmc_debug("Event = 0x%0X [0x%08X 0x%08X] <0x%x>\n", status, cmd->resp[0], cmd->resp[1], err);

    if (!cmd->error)
    {
        if ((err & EMMCISR_CRC_7) == 0)
        {
            if (!(mmc_resp_type(cmd) & MMC_RSP_CRC))
            {
                cmd->error = 0;
                nuc970_emmc_write(REG_EMMCISR, EMMCISR_CRC_IF);
            }
            else
            {
                cmd->error = -EIO;
                nuc970_emmc_debug("Error detected and set to %d/%d (cmd = %d, retries = %d)\n",
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
                nuc970_emmc_write(REG_EMMCCSR, nuc970_emmc_read(REG_EMMCCSR) | EMMCCSR_DI_EN);
            }
            wait_event_interruptible(emmc_wq_xfer, (emmc_state_xfer != 0));
        }
    }
    nuc970_emmc_send_request(host);
}

/*
 * Handle an MMC request
 */
static void nuc970_emmc_request(struct mmc_host *mmc, struct mmc_request *mrq)
{
    struct nuc970_emmc_host *host = mmc_priv(mmc);
    host->request = mrq;
    host->flags = 0;

    /* more than 1s timeout needed with slow emmc cards */
    //mod_timer(&host->timer, jiffies +  msecs_to_jiffies(2000));

    nuc970_emmc_send_request(host);
}

/*
 * Set the IOS
 */
extern unsigned long get_cpu_clk(void);
static void nuc970_emmc_set_ios(struct mmc_host *mmc, struct mmc_ios *ios)
{
    struct nuc970_emmc_host *host = mmc_priv(mmc);
    host->bus_mode = ios->bus_width;

    /* maybe switch power to the card */
    switch (ios->power_mode)
    {
        case MMC_POWER_OFF:
            nuc970_emmc_write(REG_NAND_FMICSR, 0);
            break;
        case MMC_POWER_UP:
        case MMC_POWER_ON: // enable 74 clocks
            nuc970_emmc_write(REG_NAND_FMICSR, 0x2);

            if (ios->clock == 0)
                return;
                
            if (ios->clock <= 400000)
            {
                clk_set_rate(host->upll_clk, 100000000);
                clk_set_rate(host->emmc_clk, ios->clock);
                nuc970_emmc_write(REG_EMMCCSR, nuc970_emmc_read(REG_EMMCCSR) | EMMCCSR_CLK74_OE);
                while (nuc970_emmc_read(REG_EMMCCSR) & EMMCCSR_CLK74_OE);
            }
            else
                clk_set_rate(host->emmc_clk, ios->clock);
            break;
        default:
            WARN_ON(1);
    }

    if (ios->bus_width == MMC_BUS_WIDTH_4)
    {
        nuc970_emmc_debug("MMC: Setting controller bus width to 4\n");
        nuc970_emmc_write(REG_EMMCCSR, nuc970_emmc_read(REG_EMMCCSR) | EMMCCSR_DBW);
    }
    else
    {
        //nuc970_emmc_debug("MMC: Setting controller bus width to 1\n");
        nuc970_emmc_write(REG_EMMCCSR, nuc970_emmc_read(REG_EMMCCSR) & ~EMMCCSR_DBW);
    }
}


/*
 * Handle CO, RI, and R2 event
 */
static int emmc_event_thread(void *unused)
{
    int event = 0;
    int completed = 0;

    for (;;)
    {
        wait_event_interruptible(emmc_event_wq, (emmc_event != EMMC_EVENT_NONE) && (emmc_send_cmd));
        //printk("emmc_event_thread - emmc_event_wq raised!\n");

        completed = 0;
        event = emmc_event;
        emmc_event = EMMC_EVENT_NONE;
        emmc_send_cmd = 0;
        if (event & EMMC_EVENT_CMD_OUT)
        {
            while (1)
            {
                if (!(nuc970_emmc_read(REG_EMMCCSR) & EMMCCSR_CO_EN))
                {
                    completed = 1;
                    break;
                }
            }
        }

        if (event & EMMC_EVENT_RSP_IN)
        {
            while (1)
            {
                if (!(nuc970_emmc_read(REG_EMMCCSR) & EMMCCSR_RI_EN))
                {
                    completed = 1;
                    break;
                }

                if (nuc970_emmc_read(REG_EMMCISR) & EMMCISR_RITO_IF)
                {
                    nuc970_emmc_write(REG_EMMCTMOUT, 0x0);
                    nuc970_emmc_write(REG_EMMCISR, EMMCISR_RITO_IF);

                    completed = 1;
                    emmc_host->cmd->error = -ETIMEDOUT;
                    break;
                }
            }
        }

        if (event & EMMC_EVENT_RSP2_IN)
        {
            while (1)
            {
                if (!(nuc970_emmc_read(REG_EMMCCSR) & EMMCCSR_R2_EN))
                {
                    completed = 1;
                    break;
                }
            }
        }
        if (completed)
        {
            //nuc970_emmc_debug("Completed command\n");
            nuc970_emmc_completed_command(emmc_host, event);
        }
    }
    nuc970_emmc_debug("event quit\n");
    return 0;
}

/*
 * Handle an interrupt
 */
static irqreturn_t nuc970_emmc_irq(int irq, void *devid)
{
    struct nuc970_emmc_host *host = devid;
    unsigned int int_status;

    int_status = nuc970_emmc_read(REG_EMMCISR);

    //nuc970_emmc_debug("FMI irq: status = %08X\n", int_status);

    if (int_status & EMMCISR_BLKD_IF) {
        nuc970_emmc_debug("EMMC xfer done.\n");
        if (host->cmd->data->flags & MMC_DATA_WRITE) {
            nuc970_emmc_handle_transmitted(host);
        } else if (host->cmd->data->flags & MMC_DATA_READ) {
            nuc970_emmc_post_dma_read(host);
        }
        nuc970_emmc_write(REG_EMMCISR, EMMCISR_BLKD_IF);
        emmc_state_xfer = 1;
        wake_up_interruptible(&emmc_wq_xfer);
    }

    return IRQ_HANDLED;
}

static int nuc970_emmc_get_ro(struct mmc_host *mmc)
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

static const struct mmc_host_ops nuc970_emmc_ops = {
        .request    = nuc970_emmc_request,
        .set_ios    = nuc970_emmc_set_ios,
        .get_ro     = nuc970_emmc_get_ro,
};

/*
 * Probe for the device
 */
static int nuc970_emmc_probe(struct platform_device *pdev)
{
    struct mmc_host *mmc;
    struct nuc970_emmc_host *host;
    struct resource *res;
    int ret;
    struct clk *clkmux;
    struct pinctrl *p;

    res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
    if (!res)
        return -ENXIO;

    if (!request_mem_region(res->start, res->end - res->start + 1, DRIVER_NAME))
        return -EBUSY;

    mmc = mmc_alloc_host(sizeof(struct nuc970_emmc_host), &pdev->dev);
    if (!mmc) {
        ret = -ENOMEM;
        dev_dbg(&pdev->dev, "couldn't allocate mmc host\n");
        goto fail6;
    }

    mmc->ops = &nuc970_emmc_ops;
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
    emmc_host = host;
    host->mmc = mmc;
    host->bus_mode = 0;
    mmc->caps |= (MMC_CAP_4_BIT_DATA|MMC_CAP_MMC_HIGHSPEED);

    host->buffer = dma_alloc_coherent(&pdev->dev, MCI_BUFSIZE, &host->physical_address, GFP_KERNEL);
    if (!host->buffer) {
            ret = -ENOMEM;
            dev_err(&pdev->dev, "Can't allocate transmit buffer\n");
            goto fail5;
    }

    /* enable FMI clock */
    clk_prepare(clk_get(NULL, "fmi_hclk"));
    clk_enable(clk_get(NULL, "fmi_hclk"));
    /* enable NAND clock */
    clk_prepare(clk_get(NULL, "nand_hclk"));
    clk_enable(clk_get(NULL, "nand_hclk"));
    /* enable eMMC clock */
    clk_prepare(clk_get(NULL, "emmc_hclk"));
    clk_enable(clk_get(NULL, "emmc_hclk"));
    host->emmc_clk = clk_get(NULL, "emmc_eclk");
    if (IS_ERR(host->emmc_clk)) {
        ret = -ENODEV;
        dev_dbg(&pdev->dev, "no emmc_clk?\n");
        goto fail2;
    }
    clk_prepare(host->emmc_clk);
    clk_enable(host->emmc_clk);       /* Enable the peripheral clock */

    clkmux = clk_get(NULL, "emmc_eclk_mux");
    if (IS_ERR(clkmux)) {
        printk(KERN_ERR "nuc970-emmc:failed to get emmc clock source\n");
        ret = PTR_ERR(clkmux);
        return ret;
    }

    host->upll_clk = clk_get(NULL, "emmc_uplldiv");
    if (IS_ERR(host->upll_clk)) {
        printk(KERN_ERR "nuc970-emmc:failed to get emmc clock source\n");
        ret = PTR_ERR(host->upll_clk);
        return ret;
    }
    clk_set_parent(clkmux, host->upll_clk);
    clk_set_rate(host->upll_clk, 33000000);

    nuc970_emmc_disable(host);

#if defined (CONFIG_NUC970_EMMC_PC)
    /* initial SD1 pin -> PE2~9 */
    p = devm_pinctrl_get_select(&pdev->dev, "emmc-PC");

#elif defined (CONFIG_NUC970_EMMC_PI)
    /* initial SD1 pin -> PH6~13 */
    p = devm_pinctrl_get_select(&pdev->dev, "emmc-PI");

#endif
    if (IS_ERR(p))
    {
        dev_err(&pdev->dev, "unable to reserve pin\n");
        ret = PTR_ERR(p);
    }
    nuc970_emmc_enable(host);

    /*
     * Allocate the MCI interrupt
     */
    host->irq = platform_get_irq(pdev, 0);
    ret = request_irq(host->irq, nuc970_emmc_irq, IRQF_SHARED, mmc_hostname(mmc), host);
    if (ret) {
        dev_dbg(&pdev->dev, "request MCI interrupt failed\n");
        goto fail0;
    }

    /* add a thread to check CO, RI, and R2 */
    kernel_thread(emmc_event_thread, NULL, 0);
    setup_timer(&host->timer, nuc970_emmc_timeout_timer, (unsigned long)host);
    platform_set_drvdata(pdev, mmc);

    /*
     * Add host to MMC layer
     */
    host->present = 1;

    mmc_add_host(mmc);
    nuc970_emmc_debug("Added NUC970 EMMC driver\n");
    return 0;

fail0:
    clk_disable(host->emmc_clk);
    clk_put(host->emmc_clk);
fail2:
    if (host->buffer)
        dma_free_coherent(&pdev->dev, MCI_BUFSIZE, host->buffer, host->physical_address);
fail5:
    mmc_free_host(mmc);
fail6:
    release_mem_region(res->start, res->end - res->start + 1);
    dev_err(&pdev->dev, "probe failed, err %d\n", ret);
    return ret;
}

#ifdef CONFIG_PM
static int nuc970_emmc_suspend(struct platform_device *pdev, pm_message_t state)
{
    struct mmc_host *mmc = platform_get_drvdata(pdev);
    //struct nuc970_emmc_host *host = mmc_priv(mmc);
    int ret = 0;

    if (mmc)
        ret = mmc_suspend_host(mmc);

    return ret;
}

static int nuc970_emmc_resume(struct platform_device *pdev)
{
    struct mmc_host *mmc = platform_get_drvdata(pdev);
    //struct nuc970_emmc_host *host = mmc_priv(mmc);
    int ret = 0;

    if (mmc)
        ret = mmc_resume_host(mmc);

    return ret;
}
#else
#define nuc970_emmc_suspend   NULL
#define nuc970_emmc_resume    NULL
#endif

static struct platform_driver nuc970_emmc_driver = {
        .probe      = nuc970_emmc_probe,
        .suspend    = nuc970_emmc_suspend,
        .resume     = nuc970_emmc_resume,
        .driver     = {
                .name   = DRIVER_NAME,
                .owner  = THIS_MODULE,
        },
};


module_platform_driver(nuc970_emmc_driver);

MODULE_DESCRIPTION("NUC970 eMMC Card Interface driver");
MODULE_AUTHOR("HPChen");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:nuc970_emmc");
