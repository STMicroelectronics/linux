// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2022, STMicroelectronics - All Rights Reserved
 */

#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/debugfs.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>

#include "stm32_sys_bus.h"

/* ETZPC peripheral as firewall bus */
/* ETZPC registers */
#define ETZPC_DECPROT			0x10

/* ETZPC miscellaneous */
#define ETZPC_PROT_MASK			GENMASK(1, 0)
#define ETZPC_PROT_A7NS			0x3
#define ETZPC_DECPROT_SHIFT		1

#define IDS_PER_DECPROT_REGS		16
#define STM32MP15_ETZPC_ENTRIES		96
#define STM32MP13_ETZPC_ENTRIES		64

/* RIFSC peripheral as firewall bus */
/* RIFSC offset register */
#define RIFSC_RISC_SECCFGR0		0x10
#define RIFSC_RISC_PRIVCFGR0		0x30
#define RIFSC_RISC_PER0_CIDCFGR		0x100
#define RIFSC_RISC_PER0_SEMCR		0x104

/*
 * SEMCR register
 */
#define SEMCR_MUTEX			BIT(0)

/* RIFSC miscellaneous */
#define RIFSC_RISC_SCID_MASK		GENMASK(6, 4)
#define RIFSC_RISC_SEMWL_MASK		GENMASK(23, 16)
#define RIFSC_RISC_PER_ID_MASK		GENMASK(31, 24)

#define RIFSC_RISC_PERx_CID_MASK	(RIFSC_RISC_CFEN_MASK | \
					 RIFSC_RISC_SEM_EN_MASK | \
					 RIFSC_RISC_SCID_MASK | \
					 RIFSC_RISC_SEML_MASK)

#define IDS_PER_RISC_SEC_PRIV_REGS	32

/* RIF miscellaneous */
/*
 * CIDCFGR register fields
 */
#define CIDCFGR_CFEN			BIT(0)
#define CIDCFGR_SEMEN			BIT(1)
#define CIDCFGR_SEMWL(x)		BIT(RIFSC_RISC_SEML_SHIFT + (x))

#define SEMWL_SHIFT			16

/* Compartiment IDs */
#define RIF_CID0			0x0
#define RIF_CID1			0x1

#define STM32MP25_RIFSC_ENTRIES		178

struct sys_bus_data;

struct stm32_sys_bus_match_data {
	const u32 *map_table;
	unsigned int max_entries;
	int (*sys_bus_get_access)(struct sys_bus_data *pdata, struct device_node *np);
#ifdef CONFIG_DEBUG_FS
	const char *bus_name;
#endif
};

struct sys_bus_data {
	struct stm32_sys_bus_match_data *pconf;
	void __iomem *sys_bus_base;
	struct device *dev;
#ifdef CONFIG_DEBUG_FS
	struct rifsc_master_debug_data *m_dbg_data;
	struct rifsc_dev_debug_data *d_dbg_data;
#endif
};

static struct sys_bus_data *bus_data;

#ifdef CONFIG_DEBUG_FS
#define STM32MP25_RIFSC_NAWARE_ENTRIES	128
#define STM32MP25_RIFSC_MASTER_ENTRIES	16

#define RIFSC_RIMC_ATTR0	0xC10

#define RIFSC_RIMC_CIDSEL	BIT(2)
#define RIFSC_RIMC_MCID_MASK	GENMASK(6, 4)
#define RIFSC_RIMC_MSEC		BIT(8)
#define RIFSC_RIMC_MPRIV	BIT(9)

static const char *stm32mp25_rif_master_names[STM32MP25_RIFSC_MASTER_ENTRIES] = {
	"ETR",
	"SDMMC1",
	"SDMMC2",
	"SDMMC3",
	"USB3DR",
	"USBH",
	"ETH1",
	"ETH2",
	"PCIE",
	"GPU",
	"DMCIPP",
	"LTDC_L1/L2",
	"LTDC_L3",
	"LTDC_ROT",
	"VDEC",
	"VENC"
};

static const char *stm32mp25_rif_dev_names[STM32MP25_RIFSC_NAWARE_ENTRIES] = {
	"TIM1",
	"TIM2",
	"TIM3",
	"TIM4",
	"TIM5",
	"TIM6",
	"TIM7",
	"TIM8",
	"TIM10",
	"TIM11",
	"TIM12",
	"TIM13",
	"TIM14",
	"TIM15",
	"TIM16",
	"TIM17",
	"TIM20",
	"LPTIM1",
	"LPTIM2",
	"LPTIM3",
	"LPTIM4",
	"LPTIM5",
	"SPI1",
	"SPI2",
	"SPI3",
	"SPI4",
	"SPI5",
	"SPI6",
	"SPI7",
	"SPI8",
	"SPDIFRX",
	"USART1",
	"USART2",
	"USART3",
	"UART4",
	"UART5",
	"USART6",
	"UART7",
	"UART8",
	"UART9",
	"LPUART1",
	"I2C1",
	"I2C2",
	"I2C3",
	"I2C4",
	"I2C5",
	"I2C6",
	"I2C7",
	"I2C8",
	"SAI1",
	"SAI2",
	"SAI3",
	"SAI4",
	"RESERVED",
	"MDF1",
	"ADF1",
	"FDCAN",
	"HDP",
	"ADC12",
	"ADC3",
	"ETH1",
	"ETH2",
	"RESERVED",
	"USBH",
	"RESERVED",
	"RESERVED",
	"USB3DR",
	"COMBOPHY",
	"PCIE",
	"UCPD1",
	"ETHSW_DEIP",
	"ETHSW_ACM_CF",
	"ETHSW_ACM_MSGBU",
	"STGEN",
	"OCTOSPI1",
	"OCTOSPI2",
	"SDMMC1",
	"SDMMC2",
	"SDMMC3",
	"GPU",
	"LTDC_CMN",
	"DSI_CMN",
	"RESERVED",
	"RESERVED",
	"LVDS",
	"RESERVED",
	"CSI",
	"DCMIPP",
	"DCMI_PSSI",
	"VDEC",
	"VENC",
	"RESERVED",
	"RNG",
	"PKA",
	"SAES",
	"HASH",
	"CRYP1",
	"CRYP2",
	"IWDG1",
	"IWDG2",
	"IWDG3",
	"IWDG4",
	"IWDG5",
	"WWDG1",
	"WWDG2",
	"RESERVED",
	"VREFBUF",
	"DTS",
	"RAMCFG",
	"CRC",
	"SERC",
	"OCTOSPIM",
	"GICV2M",
	"RESERVED",
	"I3C1",
	"I3C2",
	"I3C3",
	"I3C4",
	"ICACHE_DCACHE",
	"LTDC_L0L1",
	"LTDC_L2",
	"LTDC_ROT",
	"DSI_TRIG",
	"DSI_RDFIFO",
	"RESERVED",
	"OTFDEC1",
	"OTFDEC2",
	"IAC",
};

struct rifsc_dev_debug_data {
	char dev_name[15];
	u8 dev_cid;
	u8 dev_sem_cids;
	u8 dev_id;
	bool dev_cid_filt_en;
	bool dev_sem_en;
	bool dev_priv;
	bool dev_sec;
};

struct rifsc_master_debug_data {
	char m_name[11];
	u8 m_cid;
	bool cidsel;
	bool m_sec;
	bool m_priv;
};

static void stm32_rifsc_fill_master_dbg_entry(struct sys_bus_data *pdata,
					      struct rifsc_master_debug_data *dbg_entry,
					      int i)
{
	u32 rimc_attr = readl_relaxed(pdata->sys_bus_base + RIFSC_RIMC_ATTR0 + 0x4 * i);

	snprintf(dbg_entry->m_name, sizeof(dbg_entry->m_name), "%s", stm32mp25_rif_master_names[i]);
	dbg_entry->m_cid = FIELD_GET(RIFSC_RIMC_MCID_MASK, rimc_attr);
	dbg_entry->cidsel = rimc_attr & RIFSC_RIMC_CIDSEL;
	dbg_entry->m_sec = rimc_attr & RIFSC_RIMC_MSEC;
	dbg_entry->m_priv = rimc_attr & RIFSC_RIMC_MPRIV;
}

static void stm32_rifsc_fill_dev_dbg_entry(struct sys_bus_data *pdata,
					   struct rifsc_dev_debug_data *dbg_entry,
					   int i)
{
	u32 cid_cfgr, sec_cfgr, priv_cfgr;
	u8 reg_id = i / IDS_PER_RISC_SEC_PRIV_REGS;
	u8 reg_offset = i % IDS_PER_RISC_SEC_PRIV_REGS;

	cid_cfgr = readl_relaxed(pdata->sys_bus_base + RIFSC_RISC_PER0_CIDCFGR + 0x8 * i);
	sec_cfgr = readl_relaxed(pdata->sys_bus_base + RIFSC_RISC_SECCFGR0 + 0x4 * reg_id);
	priv_cfgr = readl_relaxed(pdata->sys_bus_base + RIFSC_RISC_PRIVCFGR0 + 0x4 * reg_id);

	snprintf(dbg_entry->dev_name, sizeof(dbg_entry->dev_name), "%s",
		 stm32mp25_rif_dev_names[i]);
	dbg_entry->dev_id = i;
	dbg_entry->dev_cid_filt_en = cid_cfgr & CIDCFGR_CFEN;
	dbg_entry->dev_sem_en = cid_cfgr & CIDCFGR_SEMEN;
	dbg_entry->dev_cid = FIELD_GET(RIFSC_RISC_SCID_MASK, cid_cfgr);
	dbg_entry->dev_sem_cids = FIELD_GET(RIFSC_RISC_SEMWL_MASK, cid_cfgr);
	dbg_entry->dev_sec = sec_cfgr & BIT(reg_offset) ?  true : false;
	dbg_entry->dev_priv = priv_cfgr & BIT(reg_offset) ?  true : false;
}

static int stm32_rifsc_conf_dump_show(struct seq_file *s, void *data)
{
	struct sys_bus_data *debug_data = (struct sys_bus_data *)s->private;
	int i;

	seq_puts(s, "\n=============================================\n");
	seq_puts(s, "                 RIFSC dump\n");
	seq_puts(s, "=============================================\n\n");

	seq_puts(s, "\n=============================================\n");
	seq_puts(s, "                 RISUP dump\n");
	seq_puts(s, "=============================================\n");

	seq_printf(s, "\n| %-15s |", "Peripheral name");
	seq_puts(s, "| Firewall ID |");
	seq_puts(s, "| N/SECURE |");
	seq_puts(s, "| N/PRIVILEGED |");
	seq_puts(s, "| CID filtering |");
	seq_puts(s, "| Semaphore mode |");
	seq_puts(s, "| SCID |");
	seq_printf(s, "| %7s |\n", "SEMWL");

	for (i = 0; i < STM32MP25_RIFSC_NAWARE_ENTRIES; i++) {
		struct rifsc_dev_debug_data d_dbg_entry;

		stm32_rifsc_fill_dev_dbg_entry(debug_data, &d_dbg_entry, i);

		seq_printf(s, "| %-15s |", d_dbg_entry.dev_name);
		seq_printf(s, "| %-11d |", d_dbg_entry.dev_id);
		seq_printf(s, "| %-8s |", d_dbg_entry.dev_sec ? "SEC" : "NSEC");
		seq_printf(s, "| %-12s |", d_dbg_entry.dev_priv ? "PRIV" : "NPRIV");
		seq_printf(s, "| %-13s |",
			   d_dbg_entry.dev_cid_filt_en ? "enabled" : "disabled");
		seq_printf(s, "| %-14s |",
			   d_dbg_entry.dev_sem_en ? "enabled" : "disabled");
		seq_printf(s, "| %-4d |", d_dbg_entry.dev_cid);
		seq_printf(s, "| %#-7x |\n", d_dbg_entry.dev_sem_cids);
	}

	seq_puts(s, "\n=============================================\n");
	seq_puts(s, "                  RIMU dump\n");
	seq_puts(s, "=============================================\n");

	seq_puts(s, "| Master name |");
	seq_puts(s, "| CIDSEL |");
	seq_puts(s, "| MCID |");
	seq_puts(s, "| N/SECURE |");
	seq_puts(s, "| N/PRIVILEGED |\n");

	for (i = 0; i < STM32MP25_RIFSC_MASTER_ENTRIES; i++) {
		struct rifsc_master_debug_data m_dbg_entry;

		stm32_rifsc_fill_master_dbg_entry(debug_data, &m_dbg_entry, i);

		seq_printf(s, "| %-11s |", m_dbg_entry.m_name);
		seq_printf(s, "| %-6s |", m_dbg_entry.cidsel ? "CIDSEL" : "");
		seq_printf(s, "| %-4d |", m_dbg_entry.m_cid);
		seq_printf(s, "| %-8s |", m_dbg_entry.m_sec ? "SEC" : "NSEC");
		seq_printf(s, "| %-12s |\n", m_dbg_entry.m_priv ? "PRIV" : "NPRIV");
	}

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(stm32_rifsc_conf_dump);

static int stm32_sys_bus_register_debugfs(struct sys_bus_data *pdata)
{
	struct dentry *root = NULL;

	/* Only supported for RIFSC at the moment */
	if (strcmp(pdata->pconf->bus_name, "rifsc"))
		return 0;

	root = debugfs_lookup("stm32_firewall", NULL);
	if (!root)
		root = debugfs_create_dir("stm32_firewall", NULL);

	if (IS_ERR(root))
		return PTR_ERR(root);

	debugfs_create_file("rifsc", 0444, root, pdata, &stm32_rifsc_conf_dump_fops);

	return 0;
}
#endif /* CONFIG_DEBUG_FS */

static int stm32_sys_bus_get_periph_id(struct sys_bus_data *pdata, struct device_node *np, u32 *id)
{
	int err;
	u32 feature_domain_cell[2];
	u32 id_bus;

	/* Get reg from device node */
	err = of_property_read_u32_array(np, "feature-domains", feature_domain_cell, 2);
	if (err) {
		dev_err(pdata->dev, "Unable to find feature-domains property\n");
		return -ENODEV;
	}

	id_bus = feature_domain_cell[1];

	if (id_bus >= pdata->pconf->max_entries) {
		dev_err(pdata->dev, "Invalid sys bus ID for %s\n", np->full_name);
		return -EINVAL;
	}

	*id = id_bus;

	return 0;
}

static int stm32_etzpc_get_access(struct sys_bus_data *pdata, struct device_node *np)
{
	int err;
	u32 offset, reg_offset, sec_val, id;

	err = stm32_sys_bus_get_periph_id(pdata, np, &id);
	if (err)
		return err;

	/* Check access configuration, 16 peripherals per register */
	reg_offset = ETZPC_DECPROT + 0x4 * (id / IDS_PER_DECPROT_REGS);
	offset = (id % IDS_PER_DECPROT_REGS) << ETZPC_DECPROT_SHIFT;

	/* Verify peripheral is non-secure and attributed to cortex A7 */
	sec_val = (readl(pdata->sys_bus_base + reg_offset) >> offset) & ETZPC_PROT_MASK;
	if (sec_val != ETZPC_PROT_A7NS) {
		dev_dbg(pdata->dev, "Invalid bus configuration: reg_offset %#x, value %d\n",
			reg_offset, sec_val);
		return -EACCES;
	}

	return 0;
}

static bool stm32_rif_is_semaphore_available(void __iomem *addr)
{
	return !(readl(addr) & SEMCR_MUTEX);
}

static int stm32_rif_acquire_semaphore(struct sys_bus_data *pdata, int id)
{
	void __iomem *addr = pdata->sys_bus_base + RIFSC_RISC_PER0_SEMCR + 0x8 * id;

	writel(SEMCR_MUTEX, addr);

	/* Check that CID1 has the semaphore */
	if (stm32_rif_is_semaphore_available(addr) ||
	    FIELD_GET(RIFSC_RISC_SCID_MASK, (readl(addr)) != RIF_CID1))
		return -EACCES;

	return 0;
}

static int stm32_rifsc_get_access_by_node(struct sys_bus_data *pdata, struct device_node *np)
{
	int err;
	u32 id;

	err = stm32_sys_bus_get_periph_id(pdata, np, &id);
	if (err)
		return err;

	return stm32_rifsc_get_access_by_id(id);
}

int stm32_rifsc_check_access_by_id(u32 id)
{
	struct sys_bus_data *pdata = bus_data;
	u32 reg_offset, reg_id, sec_reg_value, cid_reg_value;
	void __iomem *addr;

	if (id >= STM32MP25_RIFSC_ENTRIES)
		return -EINVAL;

	/*
	 * RIFSC_RISC_PRIVCFGRx and RIFSC_RISC_SECCFGRx both handle configuration access for
	 * 32 peripherals. On the other hand, there is one _RIFSC_RISC_PERx_CIDCFGR register
	 * per peripheral
	 */
	reg_id = id / IDS_PER_RISC_SEC_PRIV_REGS;
	reg_offset = id % IDS_PER_RISC_SEC_PRIV_REGS;
	sec_reg_value = readl(pdata->sys_bus_base + RIFSC_RISC_SECCFGR0 + 0x4 * reg_id);
	cid_reg_value = readl(pdata->sys_bus_base + RIFSC_RISC_PER0_CIDCFGR + 0x8 * id);

	/* Check security configuration */
	if (sec_reg_value & BIT(reg_offset)) {
		dev_dbg(pdata->dev, "Invalid security configuration for peripheral: %d\n", id);
		return -EACCES;
	}

	/* Skip CID check if CID filtering isn't enabled */
	if (!(cid_reg_value & CIDCFGR_CFEN))
		return 0;

	if (FIELD_GET(RIFSC_RISC_SCID_MASK, cid_reg_value) == RIF_CID1)
		return 0;

	/*
	 * First check conditions for semaphore mode, which doesn't take into account static CID.
	 * CID for Cortex A35 is RIF_CID1.
	 */
	if (cid_reg_value & CIDCFGR_SEMEN) {
		addr = pdata->sys_bus_base + RIFSC_RISC_PER0_SEMCR + 0x8 * id;

		/* Check that CID1 has the semaphore */
		if (!stm32_rif_is_semaphore_available(addr) &&
		    (FIELD_GET(RIFSC_RISC_SCID_MASK, readl(addr)) == RIF_CID1)) {
			return 0;
		}
	}

	return -EACCES;
}

int stm32_rifsc_get_access_by_id(u32 id)
{
	int err;
	struct sys_bus_data *pdata = bus_data;
	u32 reg_offset, reg_id, sec_reg_value, cid_reg_value;

	if (id >= STM32MP25_RIFSC_ENTRIES)
		return -EINVAL;

	/*
	 * RIFSC_RISC_PRIVCFGRx and RIFSC_RISC_SECCFGRx both handle configuration access for
	 * 32 peripherals. On the other hand, there is one _RIFSC_RISC_PERx_CIDCFGR register
	 * per peripheral
	 */
	reg_id = id / IDS_PER_RISC_SEC_PRIV_REGS;
	reg_offset = id % IDS_PER_RISC_SEC_PRIV_REGS;
	sec_reg_value = readl(pdata->sys_bus_base + RIFSC_RISC_SECCFGR0 + 0x4 * reg_id);
	cid_reg_value = readl(pdata->sys_bus_base + RIFSC_RISC_PER0_CIDCFGR + 0x8 * id);

	/*
	 * First check conditions for semaphore mode, which doesn't take into account static CID.
	 * CID for Cortex A35 is RIF_CID1.
	 */
	if (cid_reg_value & CIDCFGR_SEMEN) {
		if (cid_reg_value & BIT(RIF_CID1 + SEMWL_SHIFT)) {
			/* Static CID is irrelevant if semaphore mode */
			goto skip_cid_check;
		} else {
			dev_dbg(pdata->dev, "Invalid bus semaphore configuration: index %d\n", id);
			return -EACCES;
		}
	}

	/* Skip CID check if CID filtering isn't enabled */
	if (!(cid_reg_value & CIDCFGR_CFEN))
		goto skip_cid_check;

	/* Coherency check with the CID configuration */
	if (FIELD_GET(RIFSC_RISC_SCID_MASK, cid_reg_value) != RIF_CID1) {
		dev_dbg(pdata->dev, "Invalid CID configuration for peripheral: %d\n", id);
		return -EACCES;
	}

skip_cid_check:
	/* Check security configuration */
	if (sec_reg_value & BIT(reg_offset)) {
		dev_dbg(pdata->dev, "Invalid security configuration for peripheral: %d\n", id);
		return -EACCES;
	}

	/*
	 * If the peripheral is in semaphore mode, take the semaphore so that
	 * the CID1 has the ownership.
	 */
	if (cid_reg_value & CIDCFGR_SEMEN) {
		err = stm32_rif_acquire_semaphore(pdata, id);
		if (err) {
			dev_err(pdata->dev, "Couldn't acquire semaphore for peripheral: %d\n", id);
			return err;
		}
	}

	return 0;
}

static void stm32_sys_bus_populate(struct sys_bus_data *pdata)
{
	struct device *parent;
	struct device_node *child;

	parent = pdata->dev;

	dev_dbg(parent, "Populating %s system bus\n", pdata->dev->driver->name);

	for_each_available_child_of_node(dev_of_node(parent), child) {
		if (pdata->pconf->sys_bus_get_access(pdata, child)) {
			/*
			 * Peripheral access not allowed.
			 * Mark the node as populated so platform bus won't probe it
			 */
			of_node_set_flag(child, OF_POPULATED);
			dev_err(parent, "%s: Peripheral will not be probed\n",
				child->full_name);
		}
	}
}

static int stm32_sys_bus_probe(struct platform_device *pdev)
{
	struct sys_bus_data *pdata;
	struct resource *res;
	void __iomem *mmio;
	struct stm32_sys_bus_match_data *mdata;
	struct device_node *np = pdev->dev.of_node;

	pdata = devm_kzalloc(&pdev->dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	mmio = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(mmio))
		return PTR_ERR(mmio);

	pdata->sys_bus_base = mmio;

	mdata = (struct stm32_sys_bus_match_data *)of_device_get_match_data(&pdev->dev);
	if (!mdata)
		return -EINVAL;

	pdata->pconf = mdata;
	pdata->dev = &pdev->dev;

	/* Todo: Use firewall framework */
	bus_data = pdata;

	platform_set_drvdata(pdev, pdata);

#ifdef CONFIG_DEBUG_FS
	stm32_sys_bus_register_debugfs(pdata);
#endif

	stm32_sys_bus_populate(pdata);

	/* Populate all available nodes */
	return of_platform_populate(np, NULL, NULL, &pdev->dev);
}

static const struct stm32_sys_bus_match_data stm32mp15_sys_bus_data = {
	.max_entries = STM32MP15_ETZPC_ENTRIES,
	.sys_bus_get_access = stm32_etzpc_get_access,
#ifdef CONFIG_DEBUG_FS
	.bus_name = "etzpc"
#endif
};

static const struct stm32_sys_bus_match_data stm32mp13_sys_bus_data = {
	.max_entries = STM32MP13_ETZPC_ENTRIES,
	.sys_bus_get_access = stm32_etzpc_get_access,
#ifdef CONFIG_DEBUG_FS
	.bus_name = "etzpc"
#endif
};

static const struct stm32_sys_bus_match_data stm32mp25_sys_bus_data = {
	.max_entries = STM32MP25_RIFSC_ENTRIES,
	.sys_bus_get_access = stm32_rifsc_get_access_by_node,
#ifdef CONFIG_DEBUG_FS
	.bus_name = "rifsc"
#endif
};

static const struct of_device_id stm32_sys_bus_of_match[] = {
	{ .compatible = "st,stm32mp15-sys-bus", .data = &stm32mp15_sys_bus_data },
	{ .compatible = "st,stm32mp13-sys-bus", .data = &stm32mp13_sys_bus_data },
	{ .compatible = "st,stm32mp25-sys-bus", .data = &stm32mp25_sys_bus_data },
	{}
};
MODULE_DEVICE_TABLE(of, stm32_sys_bus_of_match);

static struct platform_driver stm32_sys_bus_driver = {
	.probe  = stm32_sys_bus_probe,
	.driver = {
		.name = "stm32-sys-bus",
		.of_match_table = stm32_sys_bus_of_match,
	},
};

static int __init stm32_sys_bus_init(void)
{
	return platform_driver_register(&stm32_sys_bus_driver);
}
arch_initcall(stm32_sys_bus_init);

