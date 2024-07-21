// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2024, STMicroelectronics - All Rights Reserved
 */

#include <linux/bitfield.h>
#include <linux/bits.h>
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
 * RISAF registers
 */
#define _RISAF_REG_CFGR(x)		(0x40 + 0x40 * ((x) - 1))
#define _RISAF_REG_STARTR(x)		(0x44 + 0x40 * ((x) - 1))
#define _RISAF_REG_ENDR(x)		(0x48 + 0x40 * ((x) - 1))
#define _RISAF_REG_CIDCFGR(x)		(0x4c + 0x40 * ((x) - 1))
#define _RISAF_HWCFGR			0xFF0

/*
 * _RISAF_REG_CFGR register
 */
#define _RISAF_REG_CFGR_BREN_SHIFT	0
#define _RISAF_REG_CFGR_BREN		BIT(_RISAF_REG_CFGR_BREN_SHIFT)
#define _RISAF_REG_CFGR_SEC_SHIFT	8
#define _RISAF_REG_CFGR_SEC		BIT(_RISAF_REG_CFGR_SEC_SHIFT)
#define _RISAF_REG_CFGR_ENC_SHIFT	15
#define _RISAF_REG_CFGR_ENC		BIT(_RISAF_REG_CFGR_ENC_SHIFT)
#define _RISAF_REG_CFGR_PRIVC_SHIFT	16
#define _RISAF_REG_CFGR_PRIVC_MASK	GENMASK(23, 16)

/*
 * _RISAF_REG_CIDCFGR register
 */
#define _RISAF_REG_CIDCFGR_RDENC_SHIFT	0
#define _RISAF_REG_CIDCFGR_RDENC_MASK	GENMASK(7, 0)
#define _RISAF_REG_CIDCFGR_WRENC_SHIFT	16
#define _RISAF_REG_CIDCFGR_WRENC_MASK	GENMASK(23, 16)

/*
 * _RISAF_HWCFGR register
 */
#define _RISAF_HWCFGR_CFG1		GENMASK(7, 0)

#define NB_CID_SUPPORTED		8
#define CID_STRING_LENGTH		20
#define SEPARATOR_STRING_LENGTH		145

struct risaf_debug_data {
	uint reg_id;
	phys_addr_t reg_base;
	phys_addr_t reg_end;
	uint cid_priv_list;
	uint cid_r_list;
	uint cid_w_list;
	bool encrypted;
	bool enabled;
	bool sec;
};

struct risaf_pdata {
	void __iomem *base;
	struct device *dev;
	struct dentry *dbg_entry;
	phys_addr_t risaf_map_base;
	uint nb_reg;
};

static void stm32_risaf_fill_dev_dbg_entry(struct risaf_pdata *pdata,
					   struct risaf_debug_data *dbg_entry,
					   uint i)
{
	u32 cfgr, cidcfgr;

	dbg_entry->reg_id = i;

	cfgr = readl_relaxed(pdata->base + _RISAF_REG_CFGR(i));
	dbg_entry->encrypted = cfgr & _RISAF_REG_CFGR_ENC;
	dbg_entry->enabled = cfgr & _RISAF_REG_CFGR_BREN;
	dbg_entry->sec = cfgr & _RISAF_REG_CFGR_SEC;
	dbg_entry->cid_priv_list = (cfgr & _RISAF_REG_CFGR_PRIVC_MASK) >>
				   _RISAF_REG_CFGR_PRIVC_SHIFT;

	dbg_entry->reg_base = readl_relaxed(pdata->base + _RISAF_REG_STARTR(i));
	dbg_entry->reg_end = readl_relaxed(pdata->base + _RISAF_REG_ENDR(i));

	cidcfgr = readl_relaxed(pdata->base + _RISAF_REG_CIDCFGR(i));
	dbg_entry->cid_r_list = cidcfgr & _RISAF_REG_CIDCFGR_RDENC_MASK;
	dbg_entry->cid_w_list = (cidcfgr & _RISAF_REG_CIDCFGR_WRENC_MASK) >>
				_RISAF_REG_CIDCFGR_WRENC_SHIFT;
}

static void stm32_risaf_build_cid_bf_string(uint bf_list, char *string, size_t len)
{
	bool first_occurence = true;
	char i_as_char[3];
	uint i;

	memset(string, 0, strlen(string));

	if (!bf_list)
		return;

	snprintf(string, len, "CID");
	for (i = 0; i < NB_CID_SUPPORTED; i++) {
		if (bf_list & BIT(i)) {
			if (first_occurence) {
				sprintf(i_as_char, "%u", i);
				first_occurence = false;
			} else {
				sprintf(i_as_char, ",%u", i);
			}
			strncat(string, i_as_char, len);
		}
	}
}

static int stm32_risaf_conf_dump_show(struct seq_file *s, void *data)
{
	struct risaf_pdata *pdata = (struct risaf_pdata *)s->private;
	char cid_string[CID_STRING_LENGTH] = {0};
	char separator[SEPARATOR_STRING_LENGTH];
	uint i;

	pdata->nb_reg = readl_relaxed(pdata->base + _RISAF_HWCFGR) & _RISAF_HWCFGR_CFG1;
	memset(separator, '-', sizeof(separator) - 1);
	memset(separator + sizeof(separator) - 1, '\0', 1);


	seq_printf(s, "\n%s\n", separator);
	seq_puts(s, "| Region ID |");
	seq_puts(s, "| Region base addr |");
	seq_puts(s, "| Region end addr |");
	seq_puts(s, "| Enabled |");
	seq_puts(s, "| Encrypted |");
	seq_puts(s, "| N/SECURE |");
	seq_puts(s, "| CID PRIV-only list |");
	seq_puts(s, "| CID read list |");
	seq_puts(s, "| CID write list |");
	seq_printf(s, "\n%s\n", separator);

	/* Region 0 is the immutable and implicit RISAF TDCID-only base region, skip index 0 */
	for (i = 1; i <= pdata->nb_reg; i++) {
		struct risaf_debug_data d_dbg_entry;

		stm32_risaf_fill_dev_dbg_entry(pdata, &d_dbg_entry, i);
		seq_printf(s, "| Region %-2u |", d_dbg_entry.reg_id);
		seq_printf(s, "| %#-16llx |", pdata->risaf_map_base + d_dbg_entry.reg_base);
		seq_printf(s, "| %#-15llx |", pdata->risaf_map_base + d_dbg_entry.reg_end);
		seq_printf(s, "| %-7s |", d_dbg_entry.enabled ? "ENABLED" : "");
		seq_printf(s, "| %-9s |", d_dbg_entry.encrypted ? "YES" : "");
		seq_printf(s, "| %-8s |", d_dbg_entry.sec ? "SEC" : "NSEC");

		/* CID PRIV-only list */
		stm32_risaf_build_cid_bf_string(d_dbg_entry.cid_priv_list, cid_string,
						sizeof(cid_string));
		seq_printf(s, "| %-18s |", cid_string);

		/* CID read list */
		stm32_risaf_build_cid_bf_string(d_dbg_entry.cid_r_list, cid_string,
						sizeof(cid_string));
		seq_printf(s, "| %-13s |", cid_string);

		/* CID write list */
		stm32_risaf_build_cid_bf_string(d_dbg_entry.cid_w_list, cid_string,
						sizeof(cid_string));
		seq_printf(s, "| %-14s |\n", cid_string);

		seq_printf(s, "%s\n", separator);
	}

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(stm32_risaf_conf_dump);

static int stm32_risaf_register_debugfs(struct risaf_pdata *pdata)
{
	struct dentry *root = NULL;

	root = debugfs_lookup("stm32_firewall", NULL);
	if (!root)
		root = debugfs_create_dir("stm32_firewall", NULL);

	if (IS_ERR(root))
		return PTR_ERR(root);

	pdata->dbg_entry = debugfs_create_file(pdata->dev->of_node->full_name, 0444, root, pdata,
					       &stm32_risaf_conf_dump_fops);

	return 0;
}

static void stm32_risaf_remove(struct platform_device *pdev)
{
	struct risaf_pdata *pdata = platform_get_drvdata(pdev);

	debugfs_remove(pdata->dbg_entry);
}

static int stm32_risaf_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct risaf_pdata *pdata;
	void __iomem *mmio;
	int err;

	pdata = devm_kzalloc(&pdev->dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata)
		return -ENOMEM;

	mmio =  devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(mmio))
		return PTR_ERR(mmio);

	pdata->base = mmio;
	pdata->dev = &pdev->dev;

	err = of_property_read_u64_index(np, "st,mem-map", 0, &pdata->risaf_map_base);
	if (err) {
		pr_err("Couldn't read st,mem-map property");
		return err;
	}

	platform_set_drvdata(pdev, pdata);

	stm32_risaf_register_debugfs(pdata);

	/* Populate all available nodes */
	return of_platform_populate(np, NULL, NULL, &pdev->dev);
}

static const struct of_device_id stm32_risaf_match[] = {
	{ .compatible = "st,stm32mp25-risaf", },
	{ .compatible = "st,stm32mp25-risaf-enc", },
	{}
};
MODULE_DEVICE_TABLE(of, stm32_risaf_match);

static struct platform_driver stm32_risaf_driver = {
	.probe  = stm32_risaf_probe,
	.remove_new = stm32_risaf_remove,
	.driver = {
		.name = "stm32-risaf",
		.of_match_table = stm32_risaf_match,
	},
};
module_platform_driver(stm32_risaf_driver);
MODULE_AUTHOR("Gatien Chevallier <gatien.chevallier@foss.st.com>");
MODULE_LICENSE("GPL");
