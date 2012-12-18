/*
 * Remote processor machine-specific module for Davinci
 *
 * Copyright (C) 2011-2012 Texas Instruments, Inc.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 */

#define pr_fmt(fmt)    "%s: " fmt, __func__

#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/printk.h>
#include <linux/bitops.h>
#include <linux/platform_device.h>
#include <linux/remoteproc.h>
#include <linux/platform_data/da8xx-remoteproc.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/irq.h>

#include <mach/clock.h>   /* for davinci_clk_reset_assert/deassert() */

#include "remoteproc_internal.h"

static char *fw_name;
module_param(fw_name, charp, S_IRUGO);
MODULE_PARM_DESC(fw_name, "\n\t\tName of DSP firmware file in /lib/firmware");

/*
 * OMAP-L138 Technical References:
 * http://www.ti.com/product/omap-l138
 */
#define SYSCFG_CHIPSIG_OFFSET 0x174
#define SYSCFG_CHIPSIG_CLR_OFFSET 0x178
#define SYSCFG_CHIPINT0 (1 << 0)
#define SYSCFG_CHIPINT1 (1 << 1)
#define SYSCFG_CHIPINT2 (1 << 2)
#define SYSCFG_CHIPINT3 (1 << 3)

/**
 * struct davinci_rproc - davinci remote processor state
 * @rproc: rproc handle
 */
struct davinci_rproc {
	struct rproc *rproc;
	struct clk *dsp_clk;
};

static void __iomem *syscfg0_base;
static struct platform_device *remoteprocdev;
static struct irq_data *irq_data;
static void (*ack_fxn)(struct irq_data *data);
static int irq;

/**
 * handle_event() - inbound virtqueue message workqueue function
 *
 * This funciton is registered as a kernel thread and is scheduled by the
 * kernel handler.
 */
static irqreturn_t handle_event(int irq, void *p)
{
	struct rproc *rproc = platform_get_drvdata(remoteprocdev);

	/* Process incoming buffers on our vring */
	while (IRQ_HANDLED == rproc_vq_interrupt(rproc, 0))
		;

	/* Must allow wakeup of potenitally blocking senders: */
	rproc_vq_interrupt(rproc, 1);

	return IRQ_HANDLED;
}

/**
 * davinci_rproc_callback() - inbound virtqueue message handler
 *
 * This handler is invoked directly by the kernel whenever the remote
 * core (DSP) has modified the state of a virtqueue.  There is no
 * "payload" message indicating the virtqueue index as is the case with
 * mailbox-based implementations on OMAP4.  As such, this handler "polls"
 * each known virtqueue index for every invocation.
 */
static irqreturn_t davinci_rproc_callback(int irq, void *p)
{
	if (readl(syscfg0_base + SYSCFG_CHIPSIG_OFFSET) & SYSCFG_CHIPINT0) {
		/* Clear interrupt level source */
		writel(SYSCFG_CHIPINT0,
			syscfg0_base + SYSCFG_CHIPSIG_CLR_OFFSET);

		/*
		 * ACK intr to AINTC.
		 *
		 * It has already been ack'ed by the kernel before calling
		 * this function, but since the ARM<->DSP interrupts in the
		 * CHIPSIG register are "level" instead of "pulse" variety,
		 * we need to ack it after taking down the level else we'll
		 * be called again immediately after returning.
		 */
		ack_fxn(irq_data);

		return IRQ_WAKE_THREAD;
	}

	return IRQ_HANDLED;
}

static int davinci_rproc_start(struct rproc *rproc)
{
	struct platform_device *pdev = to_platform_device(rproc->dev.parent);
	struct device *dev = rproc->dev.parent;
	struct davinci_rproc *drproc = rproc->priv;
	struct clk *dsp_clk;
	struct resource *r;
	unsigned long host1cfg_physaddr;
	unsigned int host1cfg_offset;
	int ret;

	remoteprocdev = pdev;

	/* hw requires the start (boot) address be on 1KB boundary */
	if (rproc->bootaddr & 0x3ff) {
		dev_err(dev, "invalid boot address: must be aligned to 1KB\n");

		return -EINVAL;
	}

	r = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (IS_ERR_OR_NULL(r)) {
		dev_err(dev, "platform_get_resource() error: %ld\n",
			PTR_ERR(r));

		return PTR_ERR(r);
	}
	host1cfg_physaddr = (unsigned long)r->start;

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		dev_err(dev, "platform_get_irq(pdev, 0) error: %d\n", irq);

		return irq;
	}

	irq_data = irq_get_irq_data(irq);
	if (IS_ERR_OR_NULL(irq_data)) {
		dev_err(dev, "irq_get_irq_data(%d) error: %ld\n",
			irq, PTR_ERR(irq_data));

		return PTR_ERR(irq_data);
	}
	ack_fxn = irq_data->chip->irq_ack;

	ret = request_threaded_irq(irq, davinci_rproc_callback, handle_event,
		0, "davinci-remoteproc", drproc);
	if (ret) {
		dev_err(dev, "request_threaded_irq error: %d\n", ret);

		return ret;
	}

	syscfg0_base = ioremap(host1cfg_physaddr & PAGE_MASK, SZ_4K);
	host1cfg_offset = offset_in_page(host1cfg_physaddr);
	writel(rproc->bootaddr, syscfg0_base + host1cfg_offset);

	dsp_clk = clk_get(dev, NULL);
	if (IS_ERR_OR_NULL(dsp_clk)) {
		dev_err(dev, "clk_get error: %ld\n", PTR_ERR(dsp_clk));
		ret = PTR_ERR(dsp_clk);
		goto fail;
	}
	clk_enable(dsp_clk);
	davinci_clk_reset_deassert(dsp_clk);

	drproc->dsp_clk = dsp_clk;

	return 0;
fail:
	free_irq(irq, drproc);
	iounmap(syscfg0_base);

	return ret;
}

static int davinci_rproc_stop(struct rproc *rproc)
{
	struct davinci_rproc *drproc = rproc->priv;
	struct clk *dsp_clk = drproc->dsp_clk;

	clk_disable(dsp_clk);
	clk_put(dsp_clk);
	iounmap(syscfg0_base);
	free_irq(irq, drproc);

	return 0;
}

/* kick a virtqueue */
static void davinci_rproc_kick(struct rproc *rproc, int vqid)
{
	int timed_out;
	unsigned long timeout;

	timed_out = 0;
	timeout = jiffies + HZ/100;

	/* Poll for ack from other side first */
	while (readl(syscfg0_base + SYSCFG_CHIPSIG_OFFSET) &
		SYSCFG_CHIPINT2)
		if (time_after(jiffies, timeout)) {
			dev_err(rproc->dev.parent, "failed to receive ack\n");
			timed_out = 1;

			break;
		}

	if (!timed_out)
		/* Interupt remote proc */
		writel(SYSCFG_CHIPINT2, syscfg0_base + SYSCFG_CHIPSIG_OFFSET);
}

static struct rproc_ops davinci_rproc_ops = {
	.start = davinci_rproc_start,
	.stop = davinci_rproc_stop,
	.kick = davinci_rproc_kick,
};

static int davinci_rproc_probe(struct platform_device *pdev)
{
	struct da8xx_rproc_pdata *pdata = pdev->dev.platform_data;
	struct davinci_rproc *drproc;
	struct rproc *rproc;
	struct clk *dsp_clk;
	int ret;

	if (!fw_name) {
		dev_err(&pdev->dev, "No firmware file specified\n");

		return -EINVAL;
	}

	dev_dbg(&pdev->dev, "module param fw_name: '%s'\n", fw_name);
	pdata->firmware = fw_name;

	rproc = rproc_alloc(&pdev->dev, pdata->name, &davinci_rproc_ops,
				pdata->firmware, sizeof(*drproc));
	if (!rproc)
		return -ENOMEM;

	drproc = rproc->priv;
	drproc->rproc = rproc;

	platform_set_drvdata(pdev, rproc);

	ret = rproc_add(rproc);
	if (ret)
		goto free_rproc;

	/*
	 * rproc_add() can end up enabling the DSP's clk with the DSP
	 * *not* in reset, but davinci_rproc_start() needs the DSP to be
	 * held in reset at the time it is called.
	 */
	dsp_clk = clk_get(rproc->dev.parent, NULL);
	davinci_clk_reset_assert(dsp_clk);
	clk_put(dsp_clk);

	return 0;

free_rproc:
	dev_err(rproc->dev.parent, "rproc_add failed: %d\n", ret);
	rproc_put(rproc);

	return ret;
}

static int __devexit davinci_rproc_remove(struct platform_device *pdev)
{
	struct rproc *rproc = platform_get_drvdata(pdev);
	int ret;

	ret = rproc_del(rproc);
	if (ret)
		return ret;

	rproc_put(rproc);

	return 0;
}

static struct platform_driver davinci_rproc_driver = {
	.probe = davinci_rproc_probe,
	.remove = __devexit_p(davinci_rproc_remove),
	.driver = {
		.name = "davinci-rproc",
		.owner = THIS_MODULE,
	},
};

module_platform_driver(davinci_rproc_driver);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Davinci DA850 Remote Processor control driver");
