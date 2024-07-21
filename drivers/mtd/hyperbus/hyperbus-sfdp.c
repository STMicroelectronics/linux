// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2023, Infineon Technologies.
 */
#include <linux/mtd/sfdp.h>
#include <linux/mtd/xip.h>
#include <linux/mtd/gen_probe.h>

/* HyperBus Commands */
#define HYPERBUS_ADDR_UNLOCK1	0x555
#define HYPERBUS_ADDR_UNLOCK2	0x2AA
#define HYPERBUS_CMD_UNLOCK1	0xAA
#define HYPERBUS_CMD_UNLOCK2	0x55
#define HYPERBUS_CMD_RDSFDP	0x90
#define HYPERBUS_CMD_RDVCR1	0xC7
#define HYPERBUS_CMD_RDVCR2	0xC9
#define HYPERBUS_CMD_ASOEXT	0xF0
#define HYPERBUS_CMD_ERSCTR	0x30

/* For Infineon S26Hx family */
#define S26HX_ADDR_MFR_ID	0x800
#define S26HX_ADDR_DEV_ID1	0x801
#define S26HX_ADDR_DEV_ID2	0x802
#define S26HX_DEV_ID1_3V0	0x006A
#define S26HX_DEV_ID1_1V8	0x007B
#define S26HX_DEV_ID2_512	0x001A
#define S26HX_DEV_ID2_01G	0x001B
#define S26HX_CFR1_UNIFORM	BIT(9)
#define S26HX_CFR1_TP4KBS	BIT(8)
#define S26HX_CFR2_SP4KBS	BIT(2)
#define ERASEINFO(s, b)		(((s) << 8) | ((b) - 1))

static inline u16 hyperbus_read(u32 addr, struct map_info *map,
				struct cfi_private *cfi)
{
	return cfi_read_query16(map, addr * cfi->interleave * cfi->device_type);
}

static inline void hyperbus_write(u8 cmd, u32 addr, struct map_info *map,
				  struct cfi_private *cfi)
{
	cfi_send_gen_cmd(cmd, addr, 0, map, cfi, cfi->device_type, NULL);
}

static int __xipram hyperbus_sfdp_present(struct map_info *map, __u32 base,
					  struct cfi_private *cfi)
{
	int osf = cfi->interleave * cfi->device_type; /* scale factor */
	map_word val[2];
	map_word sfdp[2];

	sfdp[0] = cfi_build_cmd(0x4653, map, cfi); /* "SF" */
	sfdp[1] = cfi_build_cmd(0x5044, map, cfi); /* "DP" */

	val[0] = map_read(map, base + osf * 0x0);
	val[1] = map_read(map, base + osf * 0x1);

	if (!map_word_equal(map, sfdp[0], val[0]))
		return 0;

	if (!map_word_equal(map, sfdp[1], val[1]))
		return 0;

	return 1; /* "SFDP" found */
}

int __xipram hyperbus_sfdp_mode_on(u32 base, struct map_info *map,
					  struct cfi_private *cfi)
{
	hyperbus_write(HYPERBUS_CMD_ASOEXT, 0, map, cfi);
	hyperbus_write(HYPERBUS_CMD_UNLOCK1, HYPERBUS_ADDR_UNLOCK1, map, cfi);
	hyperbus_write(HYPERBUS_CMD_UNLOCK2, HYPERBUS_ADDR_UNLOCK2, map, cfi);
	hyperbus_write(HYPERBUS_CMD_RDSFDP, HYPERBUS_ADDR_UNLOCK1, map, cfi);
	return hyperbus_sfdp_present(map, 0, cfi);
}
EXPORT_SYMBOL_GPL(hyperbus_sfdp_mode_on);

void __xipram hyperbus_sfdp_mode_off(u32 base, struct map_info *map,
					    struct cfi_private *cfi)
{
	hyperbus_write(HYPERBUS_CMD_ASOEXT, 0, map, cfi);
}
EXPORT_SYMBOL_GPL(hyperbus_sfdp_mode_off);

static u16 __xipram hyperbus_s26hx_rdvcr1(struct map_info *map,
					  struct cfi_private *cfi)
{
	hyperbus_write(HYPERBUS_CMD_UNLOCK1, HYPERBUS_ADDR_UNLOCK1, map, cfi);
	hyperbus_write(HYPERBUS_CMD_UNLOCK2, HYPERBUS_ADDR_UNLOCK2, map, cfi);
	hyperbus_write(HYPERBUS_CMD_RDVCR1, HYPERBUS_ADDR_UNLOCK1, map, cfi);
	return hyperbus_read(0, map, cfi);
}

static u16 __xipram hyperbus_s26hx_rdvcr2(struct map_info *map,
					  struct cfi_private *cfi)
{
	hyperbus_write(HYPERBUS_CMD_UNLOCK1, HYPERBUS_ADDR_UNLOCK1, map, cfi);
	hyperbus_write(HYPERBUS_CMD_UNLOCK2, HYPERBUS_ADDR_UNLOCK2, map, cfi);
	hyperbus_write(HYPERBUS_CMD_RDVCR2, HYPERBUS_ADDR_UNLOCK1, map, cfi);
	return hyperbus_read(0, map, cfi);
}

static int __xipram hyperbus_s26hx_chip_setup(struct map_info *map,
					      struct cfi_private *cfi)
{
	u16 mfr_id, dev_id1, dev_id2, cfr1v, cfr2v;
	u8 nregions, nbtm4ks, ntop4ks;
	u16 n256ks;
	u32 *erase;

	/* Read manufacturer and Device ID */
	mfr_id = hyperbus_read(S26HX_ADDR_MFR_ID, map, cfi);
	dev_id1 = hyperbus_read(S26HX_ADDR_DEV_ID1, map, cfi);
	dev_id2 = hyperbus_read(S26HX_ADDR_DEV_ID2, map, cfi);

	if (mfr_id != CFI_MFR_CYPRESS ||
	    (dev_id1 != S26HX_DEV_ID1_3V0 && dev_id1 != S26HX_DEV_ID1_1V8) ||
	    (dev_id2 != S26HX_DEV_ID2_512 && dev_id2 != S26HX_DEV_ID2_01G))
		return 0;

	/* Exit SFDP ASO then read CFR1V and CFR2V */
	hyperbus_sfdp_mode_off(0, map, cfi);
	cfr1v = hyperbus_s26hx_rdvcr1(map, cfi);
	cfr2v = hyperbus_s26hx_rdvcr2(map, cfi);

	/*
	 * Determine number of regions (nregions), number of bottom 4KB sectors
	 * (nbtm4ks), and number of top 4KB sectors (ntop4ks)
	 */
	if (cfr1v & S26HX_CFR1_UNIFORM) {
		nregions = 1;
		nbtm4ks = 0;
		ntop4ks = 0;
	} else if (cfr2v & S26HX_CFR2_SP4KBS) {
		nregions = 5;
		nbtm4ks = 16;
		ntop4ks = 16;
	} else if (cfr1v & S26HX_CFR1_TP4KBS) {
		nregions = 3;
		nbtm4ks = 0;
		ntop4ks = 32;
	} else {
		nregions = 3;
		nbtm4ks = 32;
		ntop4ks = 0;
	}

	cfi->cfiq = kmalloc(sizeof(*cfi->cfiq) + nregions * sizeof(u32),
			    GFP_KERNEL);
	if (!cfi->cfiq)
		return 0;

	memset(cfi->cfiq, 0, sizeof(struct cfi_ident));

	cfi->mfr = mfr_id;
	cfi->id = dev_id1 << 8 | dev_id2;
	cfi->cfi_mode = CFI_MODE_CFI;
	cfi->addr_unlock1 = HYPERBUS_ADDR_UNLOCK1;
	cfi->addr_unlock2 = HYPERBUS_ADDR_UNLOCK2;
	cfi->sector_erase_cmd = CMD(HYPERBUS_CMD_ERSCTR);

	cfi->cfiq->P_ID = P_ID_AMD_STD;
	cfi->cfiq->DevSize = dev_id2; /* dev_id2 indicates size in 2^N */
	cfi->cfiq->MaxBufWriteSize = 9; /* 2^9 = 512 */

	/*
	 * cfi_cmdset_0002 uses fixed timeout for word write (1ms + 1 jiffies)
	 * and sector erase (20s) that should be enough for s26hx. For buffer
	 * write, 2000us is used if cfiq->BufWriteTimeoutXXX is 0. The typical
	 * and maximum 512B page programming times are 680us and 2175us
	 * respectively. Specify proper values here to extend buffer write
	 * timeout.
	 */
	cfi->cfiq->BufWriteTimeoutTyp = 10; /* 2^10 = 1024 */
	cfi->cfiq->BufWriteTimeoutMax = 2; /* 1024 x 2^2 = 4096 */

	/* Setup erase region info */
	cfi->cfiq->NumEraseRegions = nregions;
	erase = cfi->cfiq->EraseRegionInfo;

	/* Bottom 4KB sectors and remaining portion */
	if (nbtm4ks) {
		*erase++ = ERASEINFO(SZ_4K, nbtm4ks);
		*erase++ = ERASEINFO(SZ_256K - (SZ_4K * nbtm4ks), 1);
	}

	/*
	 * The number of uniform 256KB sectors is obtained by dividing 'device
	 * size' by 256K(=2^18). Deduct overlaid sector(s) from uniform number
	 * if top and/or bottom 4KB sectors exist.
	 */
	n256ks = (1 << (cfi->cfiq->DevSize - 18)) - !!(nbtm4ks) - !!(ntop4ks);
	*erase++ = ERASEINFO(SZ_256K, n256ks);

	/* Top 4KB sectors and remaining portion */
	if (ntop4ks) {
		*erase++ = ERASEINFO(SZ_256K - (SZ_4K * ntop4ks), 1);
		*erase = ERASEINFO(SZ_4K, ntop4ks);
	}

	return 1;
}

static int __xipram hyperbus_sfdp_chip_setup(struct map_info *map,
					     struct cfi_private *cfi)
{
	/*
	 * Just call setup for S26Hx since it is only chip supported.
	 * Once other devices come up, we will implement vendor/chip specific
	 * detection and setup flow here.
	 */
	return hyperbus_s26hx_chip_setup(map, cfi);
}

static int __xipram hyperbus_sfdp_probe_chip(struct map_info *map, __u32 base,
					     unsigned long *chip_map,
					     struct cfi_private *cfi)
{
	u32 probe_addr;
	int i;

	if (base >= map->size) {
		pr_notice("Probe at base(0x%08x) past the end of the map(0x%08lx)\n",
			  base, map->size - 1);
		return 0;
	}

	probe_addr = base + cfi_build_cmd_addr(HYPERBUS_ADDR_UNLOCK1, map, cfi);
	if (probe_addr >= map->size) {
		pr_notice("Probe at base[unlock](0x%08x) past the end of the map(0x%08lx)\n",
			  probe_addr, map->size - 1);
		return 0;
	}

	if (!hyperbus_sfdp_mode_on(base, map, cfi))
		return 0;

	/*
	 * This is the first time we're called. Set up the CFI stuff accordingly
	 * and return
	 */
	if (!cfi->numchips)
		return hyperbus_sfdp_chip_setup(map, cfi);

	/* Check each previous chip to see if it's an alias */
	for (i = 0; i < (base >> cfi->chipshift); i++) {
		unsigned long start;

		/* Skip location; no valid chip at this address */
		if (!test_bit(i, chip_map))
			continue;

		start = i << cfi->chipshift;

		/*
		 * This chip should be in read mode if it's one we've already
		 * touched.
		 */
		if (hyperbus_sfdp_present(map, start, cfi)) {
			/*
			 * Eep. This chip also had the SFDP signature. Is it an
			 * alias or the new one?
			 */
			hyperbus_sfdp_mode_off(start, map, cfi);

			/* If the SFDP signature goes away, it's an alias */
			if (!hyperbus_sfdp_present(map, start, cfi)) {
				pr_debug("%s: Found an alias at 0x%x for the chip at 0x%lx\n",
					 map->name, base, start);
				return 0;
			}

			/*
			 * Yes, it's actually got SFDP for data. Most
			 * unfortunate. Stick the new chip in read mode too and
			 * if it's the same, assume it's an alias.
			 * FIXME: Use other modes to do a proper check
			 */
			hyperbus_sfdp_mode_off(base, map, cfi);

			if (hyperbus_sfdp_present(map, base, cfi)) {
				pr_debug("%s: Found an alias at 0x%x for the chip at 0x%lx\n",
					 map->name, base, start);
				return 0;
			}
		}
	}

	/*
	 * OK, if we got to here, then none of the previous chips appear to
	 * be aliases for the current one.
	 */
	set_bit((base >> cfi->chipshift), chip_map);  /* Update chip map */
	cfi->numchips++;

	/* Put it back into Read Mode */
	hyperbus_sfdp_mode_off(base, map, cfi);

	pr_info("%s: Found %d x%d devices at 0x%x in %d-bit bank\n",
		map->name, cfi->interleave, cfi->device_type * 8, base,
		map->bankwidth * 8);

	return 1;
}

static struct chip_probe hyperbus_sfdp_chip_probe = {
	.name		= "SFDP",
	.probe_chip	= hyperbus_sfdp_probe_chip
};

struct mtd_info *hyperbus_sfdp_probe(struct map_info *map)
{
	return mtd_do_chip_probe(map, &hyperbus_sfdp_chip_probe);
}
