// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) STMicroelectronics 2024 - All Rights Reserved
 * Authors: Gwenael Treuveur <gwenael.treuveur@foss.st.com> for STMicroelectronics.
 */

#include <linux/clk.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/mailbox_client.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_reserved_mem.h>
#include <linux/platform_device.h>
#include <linux/remoteproc.h>
#include <linux/reset.h>
#include <linux/slab.h>
#include "remoteproc_internal.h"
#include "remoteproc_elf_helpers.h"

#define STM32_MBX_SHUTDOWN	"shutdown"

struct stm32_rproc;

struct stm32_m0_rproc_mem {
	char name[20];
	void __iomem *cpu_addr;
	phys_addr_t bus_addr;
	u32 dev_addr;
	size_t size;
};

struct stm32_mbox {
	const unsigned char name[10];
	struct mbox_chan *chan;
	struct mbox_client client;
	int id;
};

struct stm32_rproc {
	struct reset_control *rst;
	struct clk_bulk_data *clks;
	int num_clks;
	int wdg_irq;
	u32 nb_rmems;
	struct stm32_m0_rproc_mem *rmems;
	struct stm32_mbox mb;
};

static int stm32_m0_rproc_pa_to_da(struct rproc *rproc, phys_addr_t pa, u64 *da)
{
	unsigned int i;
	struct stm32_rproc *ddata = rproc->priv;
	struct stm32_m0_rproc_mem *p_mem;

	for (i = 0; i < ddata->nb_rmems; i++) {
		p_mem = &ddata->rmems[i];

		if (pa < p_mem->bus_addr ||
		    pa >= p_mem->bus_addr + p_mem->size)
			continue;
		*da = pa - p_mem->bus_addr + p_mem->dev_addr;
		dev_dbg(rproc->dev.parent, "pa %pa to da %llx\n", &pa, *da);
		return 0;
	}

	return -EINVAL;
}

static int stm32_m0_rproc_mem_alloc(struct rproc *rproc,
				    struct rproc_mem_entry *mem)
{
	struct device *dev = rproc->dev.parent;
	void *va;

	dev_dbg(dev, "map memory: %pad+%zx\n", &mem->dma, mem->len);
	va = (__force void *)ioremap_wc(mem->dma, mem->len);
	if (IS_ERR_OR_NULL(va)) {
		dev_err(dev, "Unable to map memory region: %pad+0x%zx\n",
			&mem->dma, mem->len);
		return -ENOMEM;
	}

	/* Update memory entry va */
	mem->va = va;

	return 0;
}

static int stm32_m0_rproc_mem_release(struct rproc *rproc,
				      struct rproc_mem_entry *mem)
{
	dev_dbg(rproc->dev.parent, "unmap memory: %pa\n", &mem->dma);
	iounmap((__force __iomem void *)mem->va);

	return 0;
}

static int read_dma_range(struct stm32_m0_rproc_mem *p_mem, const u32 *cell)
{
	int is_64 = sizeof(phys_addr_t) / sizeof(u64);
	int nb_cell = 0;

	p_mem->dev_addr = cell[nb_cell++];
	p_mem->bus_addr = cell[nb_cell++];
	if (is_64) /* 32-bit*/
		p_mem->bus_addr = ((u64)p_mem->bus_addr << 32) | cell[nb_cell++];
	p_mem->size = cell[nb_cell++];

	return nb_cell;
}

static int stm32_m0_rproc_of_memory_translations(struct platform_device *pdev,
						 struct stm32_rproc *ddata)
{
	struct device *parent, *dev = &pdev->dev;
	struct device_node *np;
	struct stm32_m0_rproc_mem *p_mems;
	const u32 *mem_range;
	int cnt, array_size, elt_size, i, j, ret = 0;

	parent = dev->parent;
	np = parent->of_node;

	/* A dma-ranges element is construct with:
	 *  - the 32-bit remote processor address
	 *  - the cpu address which depends on the cup arch (32-bit or 64-bit)
	 *  - the 32-bit remote processor memory mapping size
	 */
	elt_size = sizeof(p_mems->dev_addr) + sizeof(p_mems->bus_addr) + sizeof(u32);

	cnt = of_property_count_elems_of_size(np, "dma-ranges", elt_size);
	if (cnt <= 0) {
		dev_err(dev, "%s: dma-ranges property not defined\n", __func__);
		return -EINVAL;
	}

	p_mems = devm_kcalloc(dev, cnt, sizeof(*p_mems), GFP_KERNEL);
	if (!p_mems)
		return -ENOMEM;
	mem_range = kcalloc(cnt, elt_size, GFP_KERNEL);
	if (!mem_range)
		return -ENOMEM;

	array_size = cnt * elt_size / sizeof(u32);

	ret = of_property_read_u32_array(np, "dma-ranges",
					 (u32 *)mem_range, array_size);
	if (ret) {
		dev_err(dev, "error while get dma-ranges property: %x\n", ret);
		goto free_mem;
	}

	for (i = 0, j = 0; i < cnt; i++) {
		j += read_dma_range(&p_mems[i], &mem_range[j]);

		dev_dbg(dev, "memory range[%i]: da %#x, pa %pa, size %#zx:\n",
			i, p_mems[i].dev_addr, &p_mems[i].bus_addr,
			p_mems[i].size);
	}

	ddata->rmems = p_mems;
	ddata->nb_rmems = cnt;

free_mem:
	kfree(mem_range);
	return ret;
}

static void stm32_m0_rproc_request_shutdown(struct rproc *rproc)
{
	struct stm32_rproc *ddata = rproc->priv;
	int err, dummy_data;

	/* Request shutdown of the remote processor */
	if (rproc->state != RPROC_OFFLINE && rproc->state != RPROC_CRASHED) {
		if (ddata->mb.chan) {
			/* A dummy data is sent to allow to block on transmit. */
			err = mbox_send_message(ddata->mb.chan, &dummy_data);
			if (err < 0)
				dev_warn(&rproc->dev, "warning: remote FW shutdown without ack\n");
		}
	}
}

static int stm32_m0_rproc_prepare(struct rproc *rproc)
{
	struct device *dev = rproc->dev.parent;
	struct device_node *np = dev->of_node;
	struct of_phandle_iterator it;
	struct rproc_mem_entry *mem;
	struct reserved_mem *rmem;
	u64 da;
	int index = 0;

	/* Register associated reserved memory regions */
	of_phandle_iterator_init(&it, np, "memory-region", NULL, 0);
	while (of_phandle_iterator_next(&it) == 0) {
		rmem = of_reserved_mem_lookup(it.node);
		if (!rmem) {
			of_node_put(it.node);
			dev_err(dev, "unable to acquire memory-region\n");
			return -EINVAL;
		}

		if (stm32_m0_rproc_pa_to_da(rproc, rmem->base, &da) < 0) {
			of_node_put(it.node);
			dev_err(dev, "memory region not valid %pa\n",
				&rmem->base);
			return -EINVAL;
		}

		/* Register memory region */
		mem = rproc_mem_entry_init(dev, NULL,
					   (dma_addr_t)rmem->base,
					   rmem->size, da,
					   stm32_m0_rproc_mem_alloc,
					   stm32_m0_rproc_mem_release,
					   it.node->name);

		if (!mem) {
			of_node_put(it.node);
			return -ENOMEM;
		}

		rproc_add_carveout(rproc, mem);
		index++;
	}

	return 0;
}

static irqreturn_t stm32_rproc_wdg(int irq, void *data)
{
	struct platform_device *pdev = data;
	struct rproc *rproc = platform_get_drvdata(pdev);

	rproc_report_crash(rproc, RPROC_WATCHDOG);

	return IRQ_HANDLED;
}

static void stm32_m0_rproc_free_mbox(struct rproc *rproc)
{
	struct stm32_rproc *ddata = rproc->priv;

	if (ddata->mb.chan) {
		mbox_free_channel(ddata->mb.chan);
		ddata->mb.chan = NULL;
	}
}

static const struct stm32_mbox stm32_m0_rproc_mbox = {
	.name = STM32_MBX_SHUTDOWN,
	.id = -1,
	.client = {
		.tx_block = true,
		.tx_done = NULL,
		.tx_tout = 500, /* 500 ms time out */
	},
};

static int stm32_m0_rproc_request_mbox(struct rproc *rproc)
{
	struct stm32_rproc *ddata = rproc->priv;
	struct device *dev = &rproc->dev;
	const unsigned char *name;
	struct mbox_client *cl;

	/* Initialise mailbox structure table */
	memcpy(&ddata->mb, &stm32_m0_rproc_mbox, sizeof(stm32_m0_rproc_mbox));

	name = stm32_m0_rproc_mbox.name;
	cl = &ddata->mb.client;
	cl->dev = dev->parent;

	ddata->mb.chan = mbox_request_channel_byname(cl, name);
	if (IS_ERR(ddata->mb.chan)) {
		if (PTR_ERR(ddata->mb.chan) == -EPROBE_DEFER) {
			dev_err_probe(dev->parent, PTR_ERR(ddata->mb.chan),
				      "failed to request mailbox %s\n", name);
			return PTR_ERR(ddata->mb.chan);
		}
		dev_info(dev, "no %s mbox\n", name);
		ddata->mb.chan = NULL;
	}

	return 0;
}

static int stm32_m0_rproc_start(struct rproc *rproc)
{
	struct stm32_rproc *ddata = rproc->priv;
	int err;

	/* Enable all C3 clocks */
	err = clk_bulk_prepare_enable(ddata->num_clks, ddata->clks);
	if (err) {
		dev_err(&rproc->dev, "failed to enable the C3 clocks\n");
		goto err_start;
	}

	/* Assert the reset of the cm0 */
	err = reset_control_assert(ddata->rst);
	if (err) {
		dev_err(&rproc->dev, "failed to assert the reset\n");
		goto err_start;
	}

	/* Deassert the reset of the cm0 */
	err = reset_control_deassert(ddata->rst);
	if (err) {
		dev_err(&rproc->dev, "failed to deassert the reset\n");
		goto err_start;
	}

	return 0;

err_start:
	/* Disable all C3 clocks */
	clk_bulk_disable_unprepare(ddata->num_clks, ddata->clks);
	return err;
}

static int stm32_m0_rproc_stop(struct rproc *rproc)
{
	struct stm32_rproc *ddata = rproc->priv;
	int err;

	stm32_m0_rproc_request_shutdown(rproc);

	/* Assert the reset of the m0 */
	err = reset_control_assert(ddata->rst);
	if (err) {
		dev_err(&rproc->dev, "failed to assert the reset\n");
		return err;
	}

	/* Disable all C3 clocks */
	clk_bulk_disable_unprepare(ddata->num_clks, ddata->clks);

	return 0;
}

static const struct rproc_ops st_m0_rproc_ops = {
	.prepare	= stm32_m0_rproc_prepare,
	.start		= stm32_m0_rproc_start,
	.stop		= stm32_m0_rproc_stop,
	.load		= rproc_elf_load_segments,
	.sanity_check	= rproc_elf_sanity_check,
};

static const struct of_device_id stm32_m0_rproc_match[] = {
	{.compatible = "st,stm32mp2-m0"},
	{},
};
MODULE_DEVICE_TABLE(of, stm32_m0_rproc_match);

static int stm32_m0_rproc_parse_dt(struct platform_device *pdev,
				   struct stm32_rproc *ddata, bool *auto_boot)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	int err, irq;

	irq = platform_get_irq(pdev, 0);
	if (irq == -EPROBE_DEFER)
		return dev_err_probe(dev, irq, "failed to get interrupt\n");

	if (irq > 0) {
		err = devm_request_irq(dev, irq, stm32_rproc_wdg, 0,
				       dev_name(dev), pdev);
		if (err)
			return dev_err_probe(dev, err,
					     "failed to request wdg irq\n");

		ddata->wdg_irq = irq;

		dev_dbg(dev, "wdg irq %d registered\n", ddata->wdg_irq);
	}

	ddata->rst = devm_reset_control_get(dev, "mcu_rst");
	if (IS_ERR(ddata->rst))
		return dev_err_probe(dev, PTR_ERR(ddata->rst),
				     "failed to get mcu_reset\n");

	/* Get all clocks defined in dt node */
	err = devm_clk_bulk_get_all(dev, &ddata->clks);
	if (err < 0)
		return dev_err_probe(dev, err, "Unable to get C3 clocks\n");

	ddata->num_clks = err;

	*auto_boot = of_property_read_bool(np, "st,auto-boot");

	return 0;
}

static int stm32_m0_rproc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct stm32_rproc *ddata;
	struct device_node *np = dev->of_node;
	struct rproc *rproc;
	int ret;

	ret = dma_coerce_mask_and_coherent(dev, DMA_BIT_MASK(32));
	if (ret)
		return ret;

	rproc = rproc_alloc(dev, np->name,
			    &st_m0_rproc_ops,
			    NULL, sizeof(*ddata));

	if (!rproc)
		return -ENOMEM;

	ddata = rproc->priv;

	ret = stm32_m0_rproc_parse_dt(pdev, ddata, &rproc->auto_boot);
	if (ret)
		goto free_rproc;

	rproc->fw_format = RPROC_FW_ELF;

	ret = stm32_m0_rproc_of_memory_translations(pdev, ddata);
	if (ret)
		goto free_rproc;

	rproc->has_iommu = false;

	platform_set_drvdata(pdev, rproc);

	ret = stm32_m0_rproc_request_mbox(rproc);
	if (ret)
		goto free_resources;

	ret = rproc_add(rproc);
	if (ret)
		goto free_mb;

	return 0;

free_mb:
	stm32_m0_rproc_free_mbox(rproc);
free_resources:
	rproc_resource_cleanup(rproc);
free_rproc:
	rproc_free(rproc);

	return ret;
}

static void stm32_m0_rproc_remove(struct platform_device *pdev)
{
	struct rproc *rproc = platform_get_drvdata(pdev);

	if (atomic_read(&rproc->power) > 0)
		rproc_shutdown(rproc);

	rproc_del(rproc);
	stm32_m0_rproc_free_mbox(rproc);

	rproc_free(rproc);
}

static void stm32_m0_rproc_shutdown(struct platform_device *pdev)
{
	struct rproc *rproc = platform_get_drvdata(pdev);

	if (atomic_read(&rproc->power) > 0)
		dev_warn(&pdev->dev,
			 "Warning: remote fw is still running with possible side effect!!!\n");
}

static struct platform_driver stm32_m0_rproc_driver = {
	.probe = stm32_m0_rproc_probe,
	.remove = stm32_m0_rproc_remove,
	.shutdown = stm32_m0_rproc_shutdown,
	.driver = {
		.name = "stm32-m0-rproc",
		.of_match_table = stm32_m0_rproc_match,
	},
};
module_platform_driver(stm32_m0_rproc_driver);

MODULE_DESCRIPTION("STM32 CM0+ Remote Processor Control Driver");
MODULE_AUTHOR("Gwenael Treuveur <gwenael.treuveur@foss.st.com>");
MODULE_LICENSE("GPL");
