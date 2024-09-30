// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2024, STMicroelectronics - All Rights Reserved
 */

#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/clk.h>
#include <linux/debugfs.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>

/*
 * RISAB constants
 */
#define RISAB_NUMBER_OF_PAGES_MAX	32
#define RISAB_BLOCKS_PER_PAGES		8
#define RISAB_NUMBER_OF_CID		7
#define RISAB_PAGE_SIZE			0x1000

/*
 * RISAB configuration register
 */
#define RISAB_CR	0x000
#define RISAB_CR_SRWIAD	BIT(31)
#define RISAB_CR_GLOCK	BIT(0)

/*
 * RISAB configuration lock register
 */
#define RISAB_RCFGLOCKR	0x010

/*
 * RISAB page y=0..31 security and privileged configuration registers
 */
#define RISAB_PG_SECCFGR(y)	(0x100 + 0x4 * (y))
#define RISAB_PG_PRIVCFGR(y)	(0x200 + 0x4 * (y))
#define RISAB_PG_C2PRIVCFGR(y)	(0x600 + 0x4 * (y))

/*
 * RISAB compartment x=0..6 privilege, read and write configuration registers
 */
#define RISAB_CIDPRIVCFGR(x)	(0x800 + 0x20 * (x))
#define RISAB_CIDRDCFGR(x)	(0x808 + 0x20 * (x))
#define RISAB_CIDWRCFGR(x)	(0x810 + 0x20 * (x))

/*
 * RISAB page y=0..31 CID configuration registers
 */
#define RISAB_PG_CIDCFGR(y)		(0xA00 + 0x4 * (y))
#define RISAB_PG_CIDCFGR_DCCID_MASK	GENMASK(6, 4)
#define RISAB_PG_CIDCFGR_DCEN		BIT(2)
#define RISAB_PG_CIDCFGR_CFEN		BIT(0)

/*
 * RISAB hardware configuration registers
 */
#define RISAB_HWCFGR1				0xFF0
#define RISAB_HWCFGR1_LOG_NUM_PAGE_MASK		GENMASK(27, 24)
#define RISAB_HWCFGR1_LOG_NUM_PAGE_SHIFT	24

struct risab_debug_data {
	bool sec[RISAB_BLOCKS_PER_PAGES];
	bool priv[RISAB_BLOCKS_PER_PAGES];
	bool c2priv[RISAB_BLOCKS_PER_PAGES];
	u8 dccid;
	bool dcen;
	bool cfen;
};

struct risab_generic_debug_data {
	bool srwiad;
	bool glock;
	bool rlock[RISAB_NUMBER_OF_PAGES_MAX];
	bool ppriv[RISAB_NUMBER_OF_CID][RISAB_NUMBER_OF_PAGES_MAX];
	bool prden[RISAB_NUMBER_OF_CID][RISAB_NUMBER_OF_PAGES_MAX];
	bool pwren[RISAB_NUMBER_OF_CID][RISAB_NUMBER_OF_PAGES_MAX];
};

struct risab_pdata {
	void __iomem *base;
	struct device *dev;
	struct dentry *dbg_entry;
	u32 risab_map_base;
	struct clk *clk;
	u32 nb_pages;
};

static void stm32_risab_fill_dev_dbg_entry(struct risab_pdata *pdata,
					   struct risab_debug_data *dbg_entry, int page)
{
	u32 risab_pg_c2privcfgr = readl_relaxed(pdata->base + RISAB_PG_C2PRIVCFGR(page));
	u32 risab_pg_privcfgr = readl_relaxed(pdata->base + RISAB_PG_PRIVCFGR(page));
	u32 risab_pg_seccfgr = readl_relaxed(pdata->base + RISAB_PG_SECCFGR(page));
	u32 risab_pg_cidcfgr = readl_relaxed(pdata->base + RISAB_PG_CIDCFGR(page));
	int block;

	for (block = 0; block < RISAB_BLOCKS_PER_PAGES; block++) {
		dbg_entry->sec[block] = risab_pg_seccfgr & BIT(block);
		dbg_entry->priv[block] = risab_pg_privcfgr & BIT(block);
		dbg_entry->c2priv[block] = risab_pg_c2privcfgr & BIT(block);
	}

	dbg_entry->dccid = risab_pg_cidcfgr & RISAB_PG_CIDCFGR_DCCID_MASK;
	dbg_entry->dcen = risab_pg_cidcfgr & RISAB_PG_CIDCFGR_DCEN;
	dbg_entry->cfen = risab_pg_cidcfgr & RISAB_PG_CIDCFGR_CFEN;
}

static void stm32_risab_fill_dev_generic_dbg_entry(struct risab_pdata *pdata,
						   struct risab_generic_debug_data *dbg_entry)
{
	u32 risab_rcfglockr = readl_relaxed(pdata->base + RISAB_RCFGLOCKR);
	u32 risab_cr = readl_relaxed(pdata->base + RISAB_CR);
	int page, compartment;

	dbg_entry->srwiad = risab_cr & RISAB_CR_SRWIAD;
	dbg_entry->glock = risab_cr & RISAB_CR_GLOCK;

	for (page = 0; page < pdata->nb_pages; page++)
		dbg_entry->rlock[page] = risab_rcfglockr & BIT(page);

	for (compartment = 0; compartment < RISAB_NUMBER_OF_CID; compartment++) {
		u32 risab_cidprivcfgr = readl_relaxed(pdata->base + RISAB_CIDPRIVCFGR(compartment));
		u32 risab_cidrdcfgr = readl_relaxed(pdata->base + RISAB_CIDRDCFGR(compartment));
		u32 risab_cidwrcfgr = readl_relaxed(pdata->base + RISAB_CIDWRCFGR(compartment));

		for (page = 0; page < pdata->nb_pages; page++) {
			dbg_entry->ppriv[compartment][page] = risab_cidprivcfgr & BIT(page);
			dbg_entry->prden[compartment][page] = risab_cidrdcfgr & BIT(page);
			dbg_entry->pwren[compartment][page] = risab_cidwrcfgr & BIT(page);
		}
	}
}

static int stm32_risab_conf_dump_show(struct seq_file *s, void *data)
{
	struct risab_pdata *pdata = (struct risab_pdata *)s->private;
	struct risab_generic_debug_data generic_dbg_entry;
	struct risab_debug_data dbg_entry;
	int ret, page, compartment, block;

	ret = clk_prepare_enable(pdata->clk);
	if (ret) {
		dev_err(pdata->dev, "Couldn't enable RISAB clock");
		return ret;
	}

	stm32_risab_fill_dev_generic_dbg_entry(pdata, &generic_dbg_entry);

	seq_puts(s, "=============================================\n");
	seq_printf(s, "        RISAB dump (%s)\n", pdata->dev->of_node->full_name);
	seq_puts(s, "=============================================\n");

	seq_printf(s, "Secure read/write illegal access disable (SRWIAD): %d.\n",
		   generic_dbg_entry.srwiad);
	seq_printf(s, "Global lock (GLOCK): %d.\n", generic_dbg_entry.glock);

	seq_puts(s, "| Page                       |");
	seq_puts(s, "| Res. |");
	seq_puts(s, "|     priv. (p) read (r) write (w) per compartment      |");
	seq_puts(s, "| Delegated |");
	seq_puts(s, "|    CID    |");
	seq_puts(s, "| secure (s) default priv. (p) compartment2 priv. (P) per block |\n");

	seq_puts(s, "|     start add.  end add.   |");
	seq_puts(s, "| lock |");
	seq_puts(s, "| CID0  | CID1  | CID2  | CID3  | CID4  | CID5  | CID6  |");
	seq_puts(s, "| conf. CID |");
	seq_puts(s, "| filtering |");
	seq_puts(s, "| blck0 | blck1 | blck2 | blck3 | blck4 | blck5 | blck6 | blck7 |\n");

	for (page = 0; page < pdata->nb_pages; page++) {
		stm32_risab_fill_dev_dbg_entry(pdata, &dbg_entry, page);
		seq_printf(s, "| %2d  0x%08x  0x%08x |",
			   page,
			   pdata->risab_map_base + page * RISAB_PAGE_SIZE,
			   pdata->risab_map_base + (page + 1) * RISAB_PAGE_SIZE - 1
		);
		seq_printf(s, "| %3s  |", generic_dbg_entry.rlock[page] ? "Yes" : "No");
		for (compartment = 0; compartment < RISAB_NUMBER_OF_CID; compartment++) {
			seq_printf(s, "| %1s %1s %1s ",
				   generic_dbg_entry.ppriv[compartment][page] ? "p" : " ",
				   generic_dbg_entry.prden[compartment][page] ? "r" : " ",
				   generic_dbg_entry.pwren[compartment][page] ? "w" : " "
			);
		}

		if (dbg_entry.dcen)
			seq_printf(s, "||  0x%04x   |", dbg_entry.dccid);
		else
			seq_puts(s, "|| disabled  |");

		seq_printf(s, "| %-9s |", dbg_entry.cfen ? "enabled" : "disabled");
		for (block = 0; block < RISAB_BLOCKS_PER_PAGES; block++) {
			seq_printf(s, "| %1s %1s %1s ",
				   dbg_entry.sec[block] ? "s" : " ",
				   dbg_entry.priv[block] && !dbg_entry.cfen ? "p" : " ",
				   dbg_entry.c2priv[block] ? "P" : " "
			);
		}
		seq_puts(s, "|\n");
	}

	clk_disable_unprepare(pdata->clk);

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(stm32_risab_conf_dump);

static int stm32_risab_register_debugfs(struct risab_pdata *pdata)
{
	struct dentry *root = NULL;

	root = debugfs_lookup("stm32_firewall", NULL);
	if (!root)
		root = debugfs_create_dir("stm32_firewall", NULL);

	if (IS_ERR(root))
		return PTR_ERR(root);

	pdata->dbg_entry = debugfs_create_file(pdata->dev->of_node->full_name, 0444,
					       root, pdata, &stm32_risab_conf_dump_fops);

	return 0;
}

static void stm32_risab_remove(struct platform_device *pdev)
{
	struct risab_pdata *pdata = platform_get_drvdata(pdev);

	debugfs_remove(pdata->dbg_entry);
}

static int stm32_risab_get_nb_pages(struct risab_pdata *pdata)
{
	u32 risab_hwcfgr1, nb_pages_shift;
	int ret, nb_page;

	ret = clk_prepare_enable(pdata->clk);
	if (ret) {
		dev_err(pdata->dev, "Failed to enable clk: %d\n", ret);
		return ret;
	}

	risab_hwcfgr1 = readl_relaxed(pdata->base + RISAB_HWCFGR1);
	nb_pages_shift = FIELD_GET(RISAB_HWCFGR1_LOG_NUM_PAGE_MASK,
				   risab_hwcfgr1);
	nb_page = BIT(nb_pages_shift);

	if (nb_page > RISAB_NUMBER_OF_PAGES_MAX) {
		dev_err(pdata->dev, "RISAB number of pages is greater than %d",
			RISAB_NUMBER_OF_PAGES_MAX);
		ret = -EINVAL;
		goto err_clk_disable;
	}

	ret = nb_page;

err_clk_disable:
	clk_disable_unprepare(pdata->clk);

	return ret;
}

static int stm32_risab_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct risab_pdata *pdata;
	void __iomem *mmio;
	int err, nb_pages;
	struct clk *clk;

	pdata = devm_kzalloc(&pdev->dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata)
		return -ENOMEM;

	mmio =  devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(mmio))
		return PTR_ERR(mmio);

	clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(clk)) {
		dev_err_probe(&pdev->dev, PTR_ERR(clk), "Failed to get clk");
		return PTR_ERR(clk);
	}

	pdata->base = mmio;
	pdata->clk = clk;
	pdata->dev = &pdev->dev;

	err = of_property_read_u32(np, "st,mem-map", &pdata->risab_map_base);
	if (err) {
		pr_err("Couldn't read RISAB number of pages");
		return err;
	}

	nb_pages = stm32_risab_get_nb_pages(pdata);
	if (nb_pages < 0) {
		pr_err("Couldn't read RISAB number of pages");
		return nb_pages;
	}
	pdata->nb_pages = nb_pages;

	platform_set_drvdata(pdev, pdata);

	stm32_risab_register_debugfs(pdata);

	/* Populate all available nodes */
	return of_platform_populate(np, NULL, NULL, &pdev->dev);
}

static const struct of_device_id stm32_risab_match[] = {
	{ .compatible = "st,stm32mp25-risab", },
	{}
};
MODULE_DEVICE_TABLE(of, stm32_risab_match);

static struct platform_driver stm32_risab_driver = {
	.probe  = stm32_risab_probe,
	.remove = stm32_risab_remove,
	.driver = {
		.name = "stm32-risab",
		.of_match_table = stm32_risab_match,
	},
};
module_platform_driver(stm32_risab_driver);
MODULE_AUTHOR("Theo GOUREAU <theo.goureau@foss.st.com>");
MODULE_LICENSE("GPL");
