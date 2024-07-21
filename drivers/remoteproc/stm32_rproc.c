// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) STMicroelectronics 2018 - All Rights Reserved
 * Authors: Ludovic Barre <ludovic.barre@st.com> for STMicroelectronics.
 *          Fabien Dessenne <fabien.dessenne@st.com> for STMicroelectronics.
 */

#include <linux/arm-smccc.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/mailbox_client.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/nvmem-consumer.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_reserved_mem.h>
#include <linux/pm_wakeirq.h>
#include <linux/regmap.h>
#include <linux/remoteproc.h>
#include <linux/reset.h>
#include <linux/slab.h>
#include <linux/tee_remoteproc.h>

#include "remoteproc_internal.h"
#include "remoteproc_elf_helpers.h"

#define HOLD_BOOT		0
#define RELEASE_BOOT		1

#define MBOX_NB_VQ		2
#define MBOX_NB_MBX		4

#define STM32_SMC_RCC		0x82001000
#define STM32_SMC_REG_WRITE	0x1

#define STM32_MBX_VQ0		"vq0"
#define STM32_MBX_VQ1		"vq1"
#define STM32_MBX_SHUTDOWN	"shutdown"
#define STM32_MBX_DETACH	"detach"

#define RSC_TBL_SIZE		1024

#define CM_STATE_OFF		0
#define CM_STATE_INI		1
#define CM_STATE_CRUN		2
#define CM_STATE_CSTOP		3
#define CM_STATE_STANDBY	4
#define CM_STATE_CRASH		5

/* PWR_CPU2D2SR register definitions */
#define M33_STATE_FIELD_SHIFT   2
#define M33_STATE_FIELD_MASK    0x3

#define M33_STATE_OFF		0
#define M33_STATE_CRUN		1
#define M33_STATE_CSLEEP	2
#define M33_STATE_CSTOP		3

/*
 * Define a default index in future may come a global list of
 * firmwares which list platforms and associated firmware(s)
 */

#define STM32_MP1_PROC_ID    0
#define STM32_MP2_PROC_ID    1

struct stm32_rproc;

struct stm32_syscon {
	struct regmap *map;
	u32 reg;
	u32 mask;
};

struct stm32_rproc_mem {
	char name[20];
	void __iomem *cpu_addr;
	phys_addr_t bus_addr;
	u32 dev_addr;
	size_t size;
};

struct stm32_rproc_data {
	int proc_id;
	int (*get_info)(struct rproc *rproc);
	bool signed_fw;
};

struct stm32_mbox {
	const unsigned char name[10];
	struct mbox_chan *chan;
	struct mbox_client client;
	int vq_id;
};

struct stm32_rproc {
	struct reset_control *rst;
	struct reset_control *hold_boot_rst;
	struct stm32_syscon hold_boot;
	struct stm32_syscon pdds;
	struct stm32_syscon cm_state;
	struct stm32_syscon rsctbl;
	struct stm32_syscon boot_vec;
	int wdg_irq;
	u32 nb_rmems;
	struct stm32_rproc_mem *rmems;
	struct stm32_mbox mb[MBOX_NB_MBX];
	bool hold_boot_smc;
	bool fw_loaded;
	struct tee_rproc *trproc;
	const struct stm32_rproc_data *desc;
	void __iomem *rsc_va;
	size_t rsc_sz;
};

static u64 stm32_rproc_get_vect_table(struct rproc *rproc, const struct firmware *fw)
{
	struct device *dev = rproc->dev.parent;
	struct stm32_rproc *ddata = rproc->priv;
	const void *shdr, *name_table_shdr;
	const char *name_table;
	const u8 *elf_data = (void *)fw->data;
	const void *ehdr = elf_data;
	size_t fw_size = fw->size;
	int i;
	u8 class = fw_elf_get_class(fw);
	u32 elf_shdr_get_size = elf_size_of_shdr(class);
	u16 shnum = elf_hdr_get_e_shnum(class, ehdr);
	u16 shstrndx = elf_hdr_get_e_shstrndx(class, ehdr);

	/*
	 * If the platform does not support the boot address configuration set
	 * the boot address to default value 0.
	 */
	if (!ddata->boot_vec.map)
		return 0;

	/* Look for the vector table  in the ELF image*/
	/* First, get the section header according to the elf class */
	shdr = elf_data + elf_hdr_get_e_shoff(class, ehdr);
	/* Compute name table section header entry in shdr array */
	name_table_shdr = shdr + (shstrndx * elf_shdr_get_size);
	/* Finally, compute the name table section address in elf */
	name_table = elf_data + elf_shdr_get_sh_offset(class, name_table_shdr);

	for (i = 0; i < shnum; i++, shdr += elf_shdr_get_size) {
		u64 size = elf_shdr_get_sh_size(class, shdr);
		u64 offset = elf_shdr_get_sh_offset(class, shdr);
		u32 name = elf_shdr_get_sh_name(class, shdr);

		if (strcmp(name_table + name, ".isr_vectors"))
			continue;

		/* make sure we have the entire table */
		if (offset + size > fw_size || offset + size < size) {
			dev_err(dev, "vector table truncated\n");
			return 0;
		}

		return elf_shdr_get_sh_addr(class, shdr);
	}

	return 0;
}

static int stm32_rproc_pa_to_da(struct rproc *rproc, phys_addr_t pa, u64 *da)
{
	unsigned int i;
	struct stm32_rproc *ddata = rproc->priv;
	struct stm32_rproc_mem *p_mem;

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

static int stm32_rproc_mem_alloc(struct rproc *rproc,
				 struct rproc_mem_entry *mem)
{
	struct device *dev = rproc->dev.parent;
	void *va;

	dev_dbg(dev, "map memory: %pad+%zx\n", &mem->dma, mem->len);
	va = ioremap_wc(mem->dma, mem->len);
	if (IS_ERR_OR_NULL(va)) {
		dev_err(dev, "Unable to map memory region: %pad+0x%zx\n",
			&mem->dma, mem->len);
		return -ENOMEM;
	}

	/* Update memory entry va */
	mem->va = va;

	return 0;
}

static int stm32_rproc_mem_release(struct rproc *rproc,
				   struct rproc_mem_entry *mem)
{
	dev_dbg(rproc->dev.parent, "unmap memory: %pa\n", &mem->dma);
	iounmap(mem->va);

	return 0;
}

static int read_dma_range(struct stm32_rproc_mem *p_mem, const u32 *cell)
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

static int stm32_rproc_of_memory_translations(struct platform_device *pdev,
					      struct stm32_rproc *ddata)
{
	struct device *parent, *dev = &pdev->dev;
	struct device_node *np;
	struct stm32_rproc_mem *p_mems;
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

static int stm32_rproc_mbox_idx(struct rproc *rproc, const unsigned char *name)
{
	struct stm32_rproc *ddata = rproc->priv;
	int i;

	for (i = 0; i < ARRAY_SIZE(ddata->mb); i++) {
		if (!strncmp(ddata->mb[i].name, name, strlen(name)))
			return i;
	}
	dev_err(&rproc->dev, "mailbox %s not found\n", name);

	return -EINVAL;
}

static void stm32_rproc_request_shutdown(struct rproc *rproc)
{
	struct stm32_rproc *ddata = rproc->priv;
	int err, dummy_data, idx;

	/* Request shutdown of the remote processor */
	if (rproc->state != RPROC_OFFLINE && rproc->state != RPROC_CRASHED) {
		idx = stm32_rproc_mbox_idx(rproc, STM32_MBX_SHUTDOWN);
		if (idx >= 0 && ddata->mb[idx].chan) {
			/* A dummy data is sent to allow to block on transmit. */
			err = mbox_send_message(ddata->mb[idx].chan,
						&dummy_data);
			if (err < 0)
				dev_warn(&rproc->dev, "warning: remote FW shutdown without ack\n");
		}
	}
}

static int stm32_rproc_release(struct rproc *rproc)
{
	struct stm32_rproc *ddata = rproc->priv;
	unsigned int err = 0;

	/* To allow platform Standby power mode, set remote proc Deep Sleep. */
	if (ddata->pdds.map) {
		err = regmap_update_bits(ddata->pdds.map, ddata->pdds.reg,
					 ddata->pdds.mask, 1);
		if (err) {
			dev_err(&rproc->dev, "failed to set pdds\n");
			return err;
		}
	}

	/* Update coprocessor state to OFF if available. */
	if (ddata->desc->proc_id == STM32_MP1_PROC_ID && ddata->cm_state.map) {
		err = regmap_update_bits(ddata->cm_state.map,
					 ddata->cm_state.reg,
					 ddata->cm_state.mask,
					 CM_STATE_OFF);
		if (err) {
			dev_err(&rproc->dev, "failed to set copro state\n");
			return err;
		}
	}

	return err;
}

static int stm32_rproc_tee_elf_sanity_check(struct rproc *rproc,
					    const struct firmware *fw)
{
	struct stm32_rproc *ddata = rproc->priv;
	unsigned int ret = 0;

	if (rproc->state == RPROC_DETACHED)
		return 0;

	ret = tee_rproc_load_fw(ddata->trproc, fw);
	if (!ret)
		ddata->fw_loaded = true;

	return ret;
}

static int stm32_rproc_tee_elf_load(struct rproc *rproc,
				    const struct firmware *fw)
{
	struct stm32_rproc *ddata = rproc->priv;
	unsigned int ret;

	/*
	 * This function can be called by remote proc for recovery
	 * without the sanity check. In this case we need to load the firmware
	 * else nothing done here as the firmware has been preloaded for the
	 * sanity check to be able to parse it for the resource table
	 */
	if (ddata->fw_loaded)
		return 0;

	ret = tee_rproc_load_fw(ddata->trproc, fw);
	if (ret)
		return ret;
	ddata->fw_loaded = true;

	/* update the resource table parameters */
	if (rproc_tee_get_rsc_table(ddata->trproc)) {
		/* no resource table: reset the related fields */
		rproc->cached_table = NULL;
		rproc->table_ptr = NULL;
		rproc->table_sz = 0;
	}

	return 0;
}

static struct resource_table *
stm32_rproc_tee_elf_find_loaded_rsc_table(struct rproc *rproc,
					  const struct firmware *fw)
{
	struct stm32_rproc *ddata = rproc->priv;

	return tee_rproc_get_loaded_rsc_table(ddata->trproc);
}

static int stm32_rproc_tee_start(struct rproc *rproc)
{
	struct stm32_rproc *ddata = rproc->priv;

	return tee_rproc_start(ddata->trproc);
}

static int stm32_rproc_tee_attach(struct rproc *rproc)
{
	/* Nothing to do, remote proc already started by the secured context */
	return 0;
}

static int stm32_rproc_tee_stop(struct rproc *rproc)
{
	struct stm32_rproc *ddata = rproc->priv;
	int err;

	stm32_rproc_request_shutdown(rproc);

	err = tee_rproc_stop(ddata->trproc);
	if (err)
		return err;

	ddata->fw_loaded = false;

	return stm32_rproc_release(rproc);
}

static int stm32_rproc_prepare(struct rproc *rproc)
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

		if (stm32_rproc_pa_to_da(rproc, rmem->base, &da) < 0) {
			of_node_put(it.node);
			dev_err(dev, "memory region not valid %pa\n",
				&rmem->base);
			return -EINVAL;
		}

		/*  No need to map vdev buffer */
		if (strcmp(it.node->name, "vdev0buffer")) {
			/* Register memory region */
			mem = rproc_mem_entry_init(dev, NULL,
						   (dma_addr_t)rmem->base,
						   rmem->size, da,
						   stm32_rproc_mem_alloc,
						   stm32_rproc_mem_release,
						   it.node->name);

			if (mem)
				rproc_coredump_add_segment(rproc, da,
							   rmem->size);
		} else {
			/* Register reserved memory for vdev buffer alloc */
			mem = rproc_of_resm_mem_entry_init(dev, index,
							   rmem->size,
							   rmem->base,
							   it.node->name);
		}

		if (!mem) {
			of_node_put(it.node);
			return -ENOMEM;
		}

		rproc_add_carveout(rproc, mem);
		index++;
	}

	return 0;
}

static int stm32_rproc_parse_fw(struct rproc *rproc, const struct firmware *fw)
{
	struct stm32_rproc *ddata = rproc->priv;
	int ret;

	if (ddata->trproc)
		ret = rproc_tee_get_rsc_table(ddata->trproc);
	else
		ret = rproc_elf_load_rsc_table(rproc, fw);
	if (ret)
		dev_warn(&rproc->dev, "no resource table found for this firmware\n");

	return 0;
}

static irqreturn_t stm32_rproc_wdg(int irq, void *data)
{
	struct platform_device *pdev = data;
	struct rproc *rproc = platform_get_drvdata(pdev);

	rproc_report_crash(rproc, RPROC_WATCHDOG);

	return IRQ_HANDLED;
}

static void stm32_rproc_mb_callback(struct mbox_client *cl, void *data)
{
	struct rproc *rproc = dev_get_drvdata(cl->dev);
	struct stm32_mbox *mb = container_of(cl, struct stm32_mbox, client);

	if (rproc_vq_interrupt(rproc, mb->vq_id) == IRQ_NONE)
		dev_dbg(&rproc->dev, "no message found in vq%d\n", mb->vq_id);
}

static void stm32_rproc_free_mbox(struct rproc *rproc)
{
	struct stm32_rproc *ddata = rproc->priv;
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(ddata->mb); i++) {
		if (ddata->mb[i].chan)
			mbox_free_channel(ddata->mb[i].chan);
		ddata->mb[i].chan = NULL;
	}
}

static const struct stm32_mbox stm32_rproc_mbox[MBOX_NB_MBX] = {
	{
		.name = STM32_MBX_VQ0,
		.vq_id = 0,
		.client = {
			.rx_callback = stm32_rproc_mb_callback,
			.tx_block = false,
		},
	},
	{
		.name = STM32_MBX_VQ1,
		.vq_id = 1,
		.client = {
			.rx_callback = stm32_rproc_mb_callback,
			.tx_block = false,
		},
	},
	{
		.name = STM32_MBX_SHUTDOWN,
		.vq_id = -1,
		.client = {
			.tx_block = true,
			.tx_done = NULL,
			.tx_tout = 500, /* 500 ms time out */
		},
	},
	{
		.name = STM32_MBX_DETACH,
		.vq_id = -1,
		.client = {
			.tx_block = true,
			.tx_done = NULL,
			.tx_tout = 200, /* 200 ms time out to detach should be fair enough */
		},
	}
};

static int stm32_rproc_request_mbox(struct rproc *rproc)
{
	struct stm32_rproc *ddata = rproc->priv;
	struct device *dev = &rproc->dev;
	unsigned int i;
	int j;
	const unsigned char *name;
	struct mbox_client *cl;

	/* Initialise mailbox structure table */
	memcpy(ddata->mb, stm32_rproc_mbox, sizeof(stm32_rproc_mbox));

	for (i = 0; i < MBOX_NB_MBX; i++) {
		name = ddata->mb[i].name;

		cl = &ddata->mb[i].client;
		cl->dev = dev->parent;

		ddata->mb[i].chan = mbox_request_channel_byname(cl, name);
		if (IS_ERR(ddata->mb[i].chan)) {
			if (PTR_ERR(ddata->mb[i].chan) == -EPROBE_DEFER) {
				dev_err_probe(dev->parent,
					      PTR_ERR(ddata->mb[i].chan),
					      "failed to request mailbox %s\n",
					      name);
				goto err_probe;
			}
			dev_warn(dev, "cannot get %s mbox\n", name);
			ddata->mb[i].chan = NULL;
		}
	}

	return 0;

err_probe:
	for (j = i - 1; j >= 0; j--)
		if (ddata->mb[j].chan)
			mbox_free_channel(ddata->mb[j].chan);
	return -EPROBE_DEFER;
}

static int stm32_rproc_set_hold_boot(struct rproc *rproc, bool hold)
{
	struct stm32_rproc *ddata = rproc->priv;
	struct stm32_syscon hold_boot = ddata->hold_boot;
	struct arm_smccc_res smc_res;
	int val, err;

	/*
	 * Three ways to manage the hold boot
	 * - using SCMI: the hold boot is managed as a reset,
	 * - using Linux(no SCMI): the hold boot is managed as a syscon register
	 * - using SMC call (deprecated): use SMC reset interface
	 */

	val = hold ? HOLD_BOOT : RELEASE_BOOT;

	if (ddata->hold_boot_rst) {
		/* Use the SCMI reset controller */
		if (!hold)
			err = reset_control_deassert(ddata->hold_boot_rst);
		else
			err =  reset_control_assert(ddata->hold_boot_rst);
	} else if (IS_ENABLED(CONFIG_HAVE_ARM_SMCCC) && ddata->hold_boot_smc) {
		/* Use the SMC call */
		arm_smccc_smc(STM32_SMC_RCC, STM32_SMC_REG_WRITE,
			      hold_boot.reg, val, 0, 0, 0, 0, &smc_res);
		err = smc_res.a0;
	} else {
		/* Use syscon */
		err = regmap_update_bits(hold_boot.map, hold_boot.reg,
					 hold_boot.mask, val);
	}

	if (err)
		dev_err(&rproc->dev, "failed to set hold boot\n");

	return err;
}

static void stm32_rproc_add_coredump_trace(struct rproc *rproc)
{
	struct rproc_debug_trace *trace;
	struct rproc_dump_segment *segment;
	bool already_added;

	list_for_each_entry(trace, &rproc->traces, node) {
		already_added = false;

		list_for_each_entry(segment, &rproc->dump_segments, node) {
			if (segment->da == trace->trace_mem.da) {
				already_added = true;
				break;
			}
		}

		if (!already_added)
			rproc_coredump_add_segment(rproc, trace->trace_mem.da,
						   trace->trace_mem.len);
	}
}

static int stm32_rproc_start(struct rproc *rproc)
{
	struct stm32_rproc *ddata = rproc->priv;
	int err;

	stm32_rproc_add_coredump_trace(rproc);

	/* clear remote proc Deep Sleep */
	if (ddata->pdds.map) {
		err = regmap_update_bits(ddata->pdds.map, ddata->pdds.reg,
					 ddata->pdds.mask, 0);
		if (err) {
			dev_err(&rproc->dev, "failed to clear pdds\n");
			return err;
		}
	}

	/* Configure the boot vector register with the vector table address read in the ELF. */
	if (ddata->boot_vec.map) {
		if (rproc->bootaddr & ~ddata->boot_vec.mask) {
			dev_err(&rproc->dev, "Boot address is %#llx no aligned on mask %#x\n",
				rproc->bootaddr, ddata->boot_vec.mask);
			return err;
		}
		dev_dbg(&rproc->dev, "boot vector address = %#llx", rproc->bootaddr);
		err = regmap_update_bits(ddata->boot_vec.map, ddata->boot_vec.reg,
					 ddata->boot_vec.mask, rproc->bootaddr);
		if (err) {
			dev_err(&rproc->dev, "failed to set boot_vec\n");
			return err;
		}
	}

	err = stm32_rproc_set_hold_boot(rproc, false);
	if (err)
		return err;

	return stm32_rproc_set_hold_boot(rproc, true);
}

static int stm32_rproc_attach(struct rproc *rproc)
{
	stm32_rproc_add_coredump_trace(rproc);

	return stm32_rproc_set_hold_boot(rproc, true);
}

static int stm32_rproc_detach(struct rproc *rproc)
{
	struct stm32_rproc *ddata = rproc->priv;
	int err, idx;

	/* Inform the remote processor of the detach */
	idx = stm32_rproc_mbox_idx(rproc, STM32_MBX_DETACH);
	if (idx >= 0 && ddata->mb[idx].chan) {
		err = mbox_send_message(ddata->mb[idx].chan, "stop");
		if (err < 0)
			dev_warn(&rproc->dev, "warning: remote FW detach without ack\n");
	}

	/* Allow remote processor to auto-reboot */
	return stm32_rproc_set_hold_boot(rproc, false);
}

static int stm32_rproc_stop(struct rproc *rproc)
{
	struct stm32_rproc *ddata = rproc->priv;
	int err;

	stm32_rproc_request_shutdown(rproc);

	err = stm32_rproc_set_hold_boot(rproc, true);
	if (err)
		return err;

	err = reset_control_assert(ddata->rst);
	if (err) {
		dev_err(&rproc->dev, "failed to assert the reset\n");
		return err;
	}


	return stm32_rproc_release(rproc);
}

static void stm32_rproc_kick(struct rproc *rproc, int vqid)
{
	struct stm32_rproc *ddata = rproc->priv;
	unsigned int i;
	int err;

	if (WARN_ON(vqid >= MBOX_NB_VQ))
		return;

	for (i = 0; i < MBOX_NB_MBX; i++) {
		if (vqid != ddata->mb[i].vq_id)
			continue;
		if (!ddata->mb[i].chan)
			return;
		err = mbox_send_message(ddata->mb[i].chan, "kick");
		if (err < 0)
			dev_err(&rproc->dev, "%s: failed (%s, err:%d)\n",
				__func__, ddata->mb[i].name, err);
		return;
	}
}

static struct resource_table *
stm32_rproc_get_loaded_rsc_table(struct rproc *rproc, size_t *table_sz)
{
	struct stm32_rproc *ddata = rproc->priv;

	*table_sz = ddata->rsc_sz;
	return (struct resource_table *)ddata->rsc_va;
}

static const struct rproc_ops st_rproc_ops = {
	.prepare	= stm32_rproc_prepare,
	.start		= stm32_rproc_start,
	.stop		= stm32_rproc_stop,
	.attach		= stm32_rproc_attach,
	.detach		= stm32_rproc_detach,
	.kick		= stm32_rproc_kick,
	.load		= rproc_elf_load_segments,
	.parse_fw	= stm32_rproc_parse_fw,
	.find_loaded_rsc_table = rproc_elf_find_loaded_rsc_table,
	.get_loaded_rsc_table = stm32_rproc_get_loaded_rsc_table,
	.sanity_check	= rproc_elf_sanity_check,
	.get_boot_addr = stm32_rproc_get_vect_table,
};

static const struct rproc_ops st_rproc_tee_ops = {
	.prepare	= stm32_rproc_prepare,
	.start		= stm32_rproc_tee_start,
	.stop		= stm32_rproc_tee_stop,
	.attach		= stm32_rproc_tee_attach,
	.kick		= stm32_rproc_kick,
	.parse_fw	= stm32_rproc_parse_fw,
	.find_loaded_rsc_table = stm32_rproc_tee_elf_find_loaded_rsc_table,
	.get_loaded_rsc_table = stm32_rproc_get_loaded_rsc_table,
	.sanity_check	= stm32_rproc_tee_elf_sanity_check,
	.load		= stm32_rproc_tee_elf_load,
};

static int stm32_rproc_get_m4_info(struct rproc *rproc)
{
	struct stm32_rproc *ddata = rproc->priv;
	struct device *dev = rproc->dev.parent;
	u32 rsc_pa;
	unsigned int state;
	int ret;

	rproc->state = CM_STATE_OFF;
	ddata->rsc_va = NULL;
	ddata->rsc_sz = 0;

	/* check the Cortex-M state */
	if (!ddata->cm_state.map)
		 /* We couldn't get the coprocessor's state, assume it is not running */
		return 0;

	ret = regmap_read(ddata->cm_state.map, ddata->cm_state.reg, &state);
	if (ret)
		return ret;

	if (state == CM_STATE_OFF)
		return 0;

	rproc->state = RPROC_DETACHED;

	/* Get the resource table information */
	ret = regmap_read(ddata->rsctbl.map, ddata->rsctbl.reg, &rsc_pa);
	if (ret) {
		dev_err(dev, "failed to read rsc tbl addr\n");
		return -EINVAL;
	}

	if (!rsc_pa)
		/* no rsc table */
		return 0;

	ddata->rsc_va = devm_ioremap_wc(dev, rsc_pa, RSC_TBL_SIZE);
	if (IS_ERR_OR_NULL(ddata->rsc_va)) {
		dev_err(dev, "Unable to map memory region: %pa+%x\n", &rsc_pa, RSC_TBL_SIZE);
		return -ENOMEM;
	}

	/*
	 * Assuming the resource table fits in 1kB is fair.
	 * Notice for the detach, that this 1 kB memory area has to be reserved in the coprocessor
	 * firmware for the resource table. On detach, the remoteproc core re-initializes this
	 * entire area by overwriting it with the initial values stored in rproc->clean_table.
	 */
	ddata->rsc_sz = RSC_TBL_SIZE;

	return 0;
}

static int stm32_rproc_get_m33_info(struct rproc *rproc)
{
	struct stm32_rproc *ddata = rproc->priv;
	struct device *dev = rproc->dev.parent;
	unsigned int cm_state;
	u32 rsc_pa;
	u32 size;
	int ret;

	rproc->state = CM_STATE_OFF;
	ddata->rsc_va = NULL;
	ddata->rsc_sz = 0;

	if (!ddata->cm_state.map) {
		/* We couldn't get the coprocessor's state, assume it is not running */
		return 0;
	}

	ret = regmap_read(ddata->cm_state.map, ddata->cm_state.reg, &cm_state);
	if (ret)
		return ret;

	switch ((cm_state >> M33_STATE_FIELD_SHIFT) & M33_STATE_FIELD_MASK) {
	case M33_STATE_OFF:
		break;
	case M33_STATE_CRUN:
	case M33_STATE_CSLEEP:
	case M33_STATE_CSTOP:
		rproc->state = RPROC_DETACHED;
		break;
	default:
		return -EINVAL;
	}

	/* Get the resource table information */
	ret = nvmem_cell_read_u32(dev, "rsc-tbl-addr", &rsc_pa);
	if (ret) {
		dev_err(dev, "failed to read rsc tbl addr\n");
		return -EINVAL;
	}

	if (!rsc_pa)
		/* no rsc table */
		return 0;

	ddata->rsc_va = devm_ioremap_wc(dev, rsc_pa, RSC_TBL_SIZE);
	if (IS_ERR_OR_NULL(ddata->rsc_va)) {
		dev_err(dev, "Unable to map memory region: %pa+%x\n", &rsc_pa, RSC_TBL_SIZE);
		return -ENOMEM;
	}

	ret = nvmem_cell_read_u32(dev, "rsc-tbl-size", &size);
	if (ret) {
		dev_err(dev, "failed to read rsc tbl size\n");
		return -EINVAL;
	}
	ddata->rsc_sz = size;

	return 0;
}

static const struct stm32_rproc_data stm32_rproc_stm32pm15 = {
	.proc_id = STM32_MP1_PROC_ID,
	.get_info = stm32_rproc_get_m4_info,
	.signed_fw = false,
};

static const struct stm32_rproc_data stm32_rproc_stm32pm15_tee = {
	.proc_id = STM32_MP1_PROC_ID,
	.get_info = stm32_rproc_get_m4_info,
	.signed_fw = true,
};

static const struct stm32_rproc_data stm32_rproc_stm32pm25 = {
	.proc_id = STM32_MP2_PROC_ID,
	.get_info = stm32_rproc_get_m33_info,
	.signed_fw = false,
};

static const struct stm32_rproc_data stm32_rproc_stm32pm25_tee = {
	.proc_id = STM32_MP2_PROC_ID,
	.get_info = stm32_rproc_get_m33_info,
	.signed_fw = true,
};

static const struct of_device_id stm32_rproc_match[] = {
	{.compatible = "st,stm32mp1-m4", .data = &stm32_rproc_stm32pm15},
	{.compatible = "st,stm32mp1-m4-tee", .data = &stm32_rproc_stm32pm15_tee},
	{.compatible = "st,stm32mp2-m33", .data = &stm32_rproc_stm32pm25},
	{.compatible = "st,stm32mp2-m33-tee", .data = &stm32_rproc_stm32pm25_tee},
	{},
};
MODULE_DEVICE_TABLE(of, stm32_rproc_match);

static int stm32_rproc_get_syscon(struct device_node *np, const char *prop,
				  struct stm32_syscon *syscon)
{
	int err = 0;

	syscon->map = syscon_regmap_lookup_by_phandle(np, prop);
	if (IS_ERR(syscon->map)) {
		err = PTR_ERR(syscon->map);
		syscon->map = NULL;
		goto out;
	}

	err = of_property_read_u32_index(np, prop, 1, &syscon->reg);
	if (err)
		goto out;

	err = of_property_read_u32_index(np, prop, 2, &syscon->mask);

out:
	return err;
}

static int stm32_rproc_parse_dt(struct platform_device *pdev,
				struct stm32_rproc *ddata, bool *auto_boot)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct stm32_syscon tz;
	unsigned int tzen;
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

		if (of_property_read_bool(np, "wakeup-source")) {
			device_init_wakeup(dev, true);
			dev_pm_set_wake_irq(dev, irq);
		}

		dev_info(dev, "wdg irq registered\n");
	}

	ddata->rst = devm_reset_control_get_optional(dev, "mcu_rst");
	if (!ddata->rst) {
		/* Try legacy fallback method: get it by index */
		ddata->rst = devm_reset_control_get_by_index(dev, 0);
	}
	if (IS_ERR(ddata->rst))
		return dev_err_probe(dev, PTR_ERR(ddata->rst),
				     "failed to get mcu_reset\n");

	/*
	 * Three ways to manage the hold boot
	 * - using SCMI: the hold boot is managed as a reset
	 *    The DT "reset-mames" property should be defined with 2 items:
	 *        reset-names = "mcu_rst", "hold_boot";
	 * - using SMC call (deprecated): use SMC reset interface
	 *    The DT "reset-mames" property is optional, "st,syscfg-tz" is required
	 * - default(no SCMI, no SMC): the hold boot is managed as a syscon register
	 *    The DT "reset-mames" property is optional, "st,syscfg-holdboot" is required
	 */

	ddata->hold_boot_rst = devm_reset_control_get_optional(dev, "hold_boot");
	if (IS_ERR(ddata->hold_boot_rst))
		return dev_err_probe(dev, PTR_ERR(ddata->hold_boot_rst),
				     "failed to get hold_boot reset\n");

	if (!ddata->hold_boot_rst && IS_ENABLED(CONFIG_HAVE_ARM_SMCCC)) {
		/* Manage the MCU_BOOT using SMC call */
		err = stm32_rproc_get_syscon(np, "st,syscfg-tz", &tz);
		if (!err) {
			err = regmap_read(tz.map, tz.reg, &tzen);
			if (err) {
				dev_err(dev, "failed to read tzen\n");
				return err;
			}
			ddata->hold_boot_smc = tzen & tz.mask;
		}
	}

	if (!ddata->hold_boot_rst && !ddata->hold_boot_smc) {
		/* Default: hold boot manage it through the syscon controller */
		err = stm32_rproc_get_syscon(np, "st,syscfg-holdboot",
					     &ddata->hold_boot);
		if (err) {
			dev_err(dev, "failed to get hold boot\n");
			return err;
		}
	}

	err = stm32_rproc_get_syscon(np, "st,syscfg-pdds", &ddata->pdds);
	if (err)
		dev_info(dev, "pdds sys config not defined\n");

	*auto_boot = of_property_read_bool(np, "st,auto-boot");

	/*
	 * See if we can check the M4 status, i.e if it was started
	 * from the boot loader or not.
	 */
	err = stm32_rproc_get_syscon(np, "st,syscfg-cm-state",
				     &ddata->cm_state);
	if (err) {
		/* remember this */
		ddata->cm_state.map = NULL;
		/* no coprocessor state syscon (optional) */
		dev_warn(dev, "m4 state not supported\n");

		/* no need to go further */
		return 0;
	}

	/* See if we can get the resource table */
	err = stm32_rproc_get_syscon(np, "st,syscfg-rsc-tbl",
				     &ddata->rsctbl);
	if (err) {
		/* no rsc table syscon (optional) */
		dev_warn(dev, "rsc tbl syscon not supported\n");
	}

	/* See if we can get the optional non secure boot address register */
	err = stm32_rproc_get_syscon(np, "st,syscfg-nsvtor", &ddata->boot_vec);
	if (err)
		dev_dbg(dev, "nsvector sys config not defined\n");

	return 0;
}

static int stm32_rproc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct stm32_rproc *ddata;
	struct device_node *np = dev->of_node;
	const struct stm32_rproc_data *desc;
	struct tee_rproc *trproc = NULL;
	struct rproc *rproc;
	int ret;

	desc = device_get_match_data(&pdev->dev);

	ret = dma_coerce_mask_and_coherent(dev, DMA_BIT_MASK(32));
	if (ret)
		return ret;

	if (desc->signed_fw) {
		trproc = tee_rproc_register(dev, desc->proc_id);
		if (IS_ERR_OR_NULL(trproc)) {
			return PTR_ERR(trproc);
		/*
		 * Delegate the firmware management to the secure context.
		 * The firmware loaded has to be signed.
		 */
		dev_info(dev, "Support of signed firmware only\n");
		}
	}

	rproc = rproc_alloc(dev, np->name,
			    trproc ? &st_rproc_tee_ops : &st_rproc_ops,
			    NULL, sizeof(*ddata));
	if (!rproc) {
		ret = -ENOMEM;
		goto free_tee;
	}

	ddata = rproc->priv;
	ddata->desc = desc;
	ddata->trproc = trproc;

	ret = stm32_rproc_parse_dt(pdev, ddata, &rproc->auto_boot);
	if (ret)
		goto free_rproc;

	if (trproc) {
		/*
		 * Delegate the firmware management to the secure context. The
		 * firmware loaded has to be signed.
		 */
		dev_info(dev, "Support of signed firmware only\n");
		rproc->fw_format = RPROC_FW_TEE;
		trproc->rproc = rproc;
	} else {
		rproc->fw_format = RPROC_FW_ELF;
	}

	rproc_coredump_set_elf_info(rproc, ELFCLASS32, EM_NONE);

	ret = stm32_rproc_of_memory_translations(pdev, ddata);
	if (ret)
		goto free_rproc;

	ret = ddata->desc->get_info(rproc);
	if (ret)
		goto free_rproc;


	rproc->has_iommu = false;

	platform_set_drvdata(pdev, rproc);

	ret = stm32_rproc_request_mbox(rproc);
	if (ret)
		goto free_resources;

	ret = rproc_add(rproc);
	if (ret)
		goto free_mb;

	return 0;

free_mb:
	stm32_rproc_free_mbox(rproc);
free_resources:
	rproc_resource_cleanup(rproc);
free_rproc:
	if (device_may_wakeup(dev)) {
		dev_pm_clear_wake_irq(dev);
		device_init_wakeup(dev, false);
	}
	rproc_free(rproc);
free_tee:
	if (trproc)
		tee_rproc_unregister(trproc);

	return ret;
}

static int stm32_rproc_remove(struct platform_device *pdev)
{
	struct rproc *rproc = platform_get_drvdata(pdev);
	struct stm32_rproc *ddata = rproc->priv;
	struct device *dev = &pdev->dev;

	if (atomic_read(&rproc->power) > 0)
		rproc_shutdown(rproc);

	rproc_del(rproc);
	stm32_rproc_free_mbox(rproc);

	if (device_may_wakeup(dev)) {
		dev_pm_clear_wake_irq(dev);
		device_init_wakeup(dev, false);
	}
	rproc_free(rproc);
	if (ddata->trproc)
		tee_rproc_unregister(ddata->trproc);

	return 0;
}

static void stm32_rproc_shutdown(struct platform_device *pdev)
{
	struct rproc *rproc = platform_get_drvdata(pdev);

	if (atomic_read(&rproc->power) > 0)
		dev_warn(&pdev->dev,
			 "Warning: remote fw is still running with possible side effect!!!\n");
}

static int stm32_rproc_suspend(struct device *dev)
{
	struct rproc *rproc = dev_get_drvdata(dev);
	struct stm32_rproc *ddata = rproc->priv;

	if (device_may_wakeup(dev))
		return enable_irq_wake(ddata->wdg_irq);

	return 0;
}

static int stm32_rproc_resume(struct device *dev)
{
	struct rproc *rproc = dev_get_drvdata(dev);
	struct stm32_rproc *ddata = rproc->priv;

	if (device_may_wakeup(dev))
		return disable_irq_wake(ddata->wdg_irq);

	return 0;
}

static DEFINE_SIMPLE_DEV_PM_OPS(stm32_rproc_pm_ops,
				stm32_rproc_suspend, stm32_rproc_resume);

static struct platform_driver stm32_rproc_driver = {
	.probe = stm32_rproc_probe,
	.remove = stm32_rproc_remove,
	.shutdown = stm32_rproc_shutdown,
	.driver = {
		.name = "stm32-rproc",
		.pm = pm_ptr(&stm32_rproc_pm_ops),
		.of_match_table = stm32_rproc_match,
	},
};
module_platform_driver(stm32_rproc_driver);

MODULE_DESCRIPTION("STM32 Remote Processor Control Driver");
MODULE_AUTHOR("Ludovic Barre <ludovic.barre@st.com>");
MODULE_AUTHOR("Fabien Dessenne <fabien.dessenne@st.com>");
MODULE_LICENSE("GPL v2");

