// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) STMicroelectronics 2018, Datum Systems 2020
 * Author: Mark Carlin <mcarlin@datumsystems.com>
 */

#include <linux/clk.h>
#include <linux/dmaengine.h>
#include <linux/dma-mapping.h>
#include <linux/errno.h>
#include <linux/interrupt.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/pinctrl/consumer.h>
#include <linux/platform_device.h>
#include <linux/uio_driver.h>
#include <linux/reset.h>

// Max chip enable 
#define FMC2_MAX_CE			4

//
// FMC2 Controller Registers (RAM/NOR)
//
#define FMC2_BCR1			0x00
#define FMC2_BTR1			0x04
#define FMC2_BCR2			0x08
#define FMC2_BTR2			0x0c
#define FMC2_BCR3			0x10
#define FMC2_BTR3			0x14
#define FMC2_BCR4			0x18
#define FMC2_BTR4			0x1c
#define FMC2_PSCNTR			0x20
#define FMC2_BWTR1			0x104
#define FMC2_BWTR2			0x10c
#define FMC2_BWTR3			0x114
#define FMC2_BWTR4			0x11c

//
// NOR/SRAM Register Masks
//
// Register: FMC2_BCRx
#define FMC2_BCR1_FMC2EN		BIT(31)	// Only available on BANK1
#define FMC2_BCRx_NBLSET_MASK	GENMASK(23,22)
#define FMC2_BCRx_NBLSET(x)		(((x) & 0x3) << 22)
#define FMC2_BCRx_CCLKEN		BIT(20)
#define FMC2_BCRx_CBURSTRW		BIT(19)
#define FMC2_BCRx_CPSIZE_MASK	GENMASK(18,16)
#define FMC2_BCRx_CPSIZE(x)		(((x) & 0x7) << 16)
#define FMC2_BCRx_ASYNCWAIT		BIT(15)
#define FMC2_BCRx_EXTMOD		BIT(14)
#define FMC2_BCRx_WAITEN		BIT(13)
#define FMC2_BCRx_WREN			BIT(12)
#define FMC2_BCRx_WAITCFG		BIT(11)
#define FMC2_BCRx_WAITPOL		BIT(9)
#define FMC2_BCRx_BURSTEN		BIT(8)
#define FMC2_BCRx_FACCEN		BIT(6)
#define FMC2_BCRx_MWID_MASK		GENMASK(5,4)
#define FMC2_BCRx_MWID(x)		(((x) & 0x3) << 4)
#define FMC2_BCRx_MWID_BUSWIDTH_8	0
#define FMC2_BCRx_MWID_BUSWIDTH_16	1
#define FMC2_BCRx_MTYP_MASK		GENMASK(3,2)
#define FMC2_BCRx_MTYP(x)		(((x) & 0x3) << 2)
#define FMC2_BCRx_MUXEN			BIT(1)
#define FMC2_BCRx_MBKEN			BIT(0)

// Register: FMC2_BTRx
#define FMC2_BTRx_DATAHLD_MASK	GENMASK(31,30)
#define FMC2_BTRx_DATAHLD(x)	(((x) & 0x3) << 30)
#define FMC2_BTRx_ACCMOD_MASK	GENMASK(29,28)
#define FMC2_BTRx_ACCMOD(x)		(((x) & 0x3) << 28)
#define FMC2_BTRx_DATLAT_MASK	GENMASK(27,24)
#define FMC2_BTRx_DATLAT(x)		(((x) & 0xf) << 24)
#define FMC2_BTRx_CLKDIV_MASK	GENMASK(23,20)
#define FMC2_BTRx_CLKDIV(x)		(((x) & 0xf) << 20)
#define FMC2_BTRx_BUSTURN_MASK	GENMASK(19,16)
#define FMC2_BTRx_BUSTURN(x)	(((x) & 0xf) << 16)
#define FMC2_BTRx_DATAST_MASK	GENMASK(15,8)
#define FMC2_BTRx_DATAST(x)		(((x) & 0xff) << 8)
#define FMC2_BTRx_ADDHLD_MASK	GENMASK(7,4)
#define FMC2_BTRx_ADDHLD(x)		(((x) & 0xf) << 4)
#define FMC2_BTRx_ADDSET_MASK	GENMASK(3,0)
#define FMC2_BTRx_ADDSET(x)		(((x) & 0xf) << 0)

// Register: FMC2_BWTRx
#define FMC2_BWTRx_DATAHLD_MASK	GENMASK(31,30)
#define FMC2_BWTRx_DATAHLD(x)	(((x) & 0x3) << 30)
#define FMC2_BWTRx_ACCMOD_MASK	GENMASK(29,28)
#define FMC2_BWTRx_ACCMOD(x)	(((x) & 0x3) << 28)
#define FMC2_BWTRx_BUSTURN_MASK	GENMASK(19,16)
#define FMC2_BWTRx_BUSTURN(x)	(((x) & 0xf) << 16)
#define FMC2_BWTRx_DATAST_MASK	GENMASK(15,8)
#define FMC2_BWTRx_DATAST(x)	(((x) & 0xff) << 8)
#define FMC2_BWTRx_ADDHLD_MASK	GENMASK(7,4)
#define FMC2_BWTRx_ADDHLD(x)	(((x) & 0xf) << 4)
#define FMC2_BWTRx_ADDSET_MASK	GENMASK(3,0)
#define FMC2_BWTRx_ADDSET(x)	(((x) & 0xf) << 0)

// Register: FMC2_PCSCNTR
#define FMC2_PCSCNTR_MASK		GENMASK(19,16)
#define FMC2_PCSCNTR(x)			(((x) & 0xf) << 16)
#define FMC2_CSCOUNT_MASK		GENMASK(15,0)
#define FMC2_CSCOUNT(x)			(((x) & 0xff) << 0)

struct stm32_fmc2_timings {
	u8 data_hld;
	u8 acc_mode;
	u8 dat_lat;
	u8 clk_div;
	u8 bus_turn;
	u8 data_st;
	u8 add_hld;
	u8 add_set;
};

struct stm32_fmc2_nfc {
	struct device *dev;
	void __iomem *io_base;
	void __iomem *data_base[FMC2_MAX_CE];
	void __iomem *cmd_base[FMC2_MAX_CE];
	void __iomem *addr_base[FMC2_MAX_CE];
	int cs_used[FMC2_MAX_CE];
	struct stm32_fmc2_timings timings[FMC2_MAX_CE];
	phys_addr_t io_phys_addr;
	phys_addr_t data_phys_addr[FMC2_MAX_CE];
	
	struct uio_info uio_info;
	
	unsigned int irq;
	
	struct clk *clk;
	u8 irq_state;

	struct dma_chan *dma_tx_ch;
	struct dma_chan *dma_rx_ch;
	struct dma_chan *dma_ecc_ch;
	struct sg_table dma_data_sg;
	struct sg_table dma_ecc_sg;
	u8 *ecc_buf;
	int dma_ecc_len;

	struct completion complete;
	struct completion dma_data_complete;
	struct completion dma_ecc_complete;

	u8 cs_assigned;
	int cs_sel;
};

// IRQ handler
static irqreturn_t fmc_fpga_handler(int irq, struct uio_info *dev_info)
{
/*
	void __iomem *plx_intscr = dev_info->mem[0].internal_addr
					+ PLX9030_INTCSR;

	if ((ioread8(plx_intscr) & INT1_ENABLED_AND_ACTIVE)
	    != INT1_ENABLED_AND_ACTIVE)
		return IRQ_NONE;

	// Disable interrupt
	iowrite8(ioread8(plx_intscr) & ~INTSCR_INT1_ENABLE, plx_intscr);
*/
	return IRQ_HANDLED;
}

// Timings configuration
static void stm32_fmc2_timings_init(struct stm32_fmc2_nfc *fmc2, int cs)
{
	struct stm32_fmc2_timings *timings = &fmc2->timings[cs];

	u32 btr = readl_relaxed(fmc2->io_base + FMC2_BTR1 + (cs << 3));

	btr &= ~FMC2_BTRx_DATAHLD_MASK;
	btr |= FMC2_BTRx_DATAHLD(timings->data_hld);
	btr &= ~FMC2_BTRx_ACCMOD_MASK;
	btr |= FMC2_BTRx_ACCMOD(timings->acc_mode);
	btr &= ~FMC2_BTRx_DATLAT_MASK;
	btr |= FMC2_BTRx_DATLAT(timings->dat_lat);
	btr &= ~FMC2_BTRx_CLKDIV_MASK;
	btr |= FMC2_BTRx_CLKDIV(timings->clk_div);
	btr &= ~FMC2_BTRx_BUSTURN_MASK;
	btr |= FMC2_BTRx_BUSTURN(timings->bus_turn);
	btr &= ~FMC2_BTRx_DATAST_MASK;
	btr |= FMC2_BTRx_DATAST(timings->data_st);
	btr &= ~FMC2_BTRx_ADDHLD_MASK;
	btr |= FMC2_BTRx_ADDHLD(timings->add_hld);
	btr &= ~FMC2_BTRx_ADDSET_MASK;
	btr |= FMC2_BTRx_ADDSET(timings->add_set);
	
	writel_relaxed(btr, fmc2->io_base + FMC2_BTR1 + (cs << 3));
}

// Controller configuration
static void stm32_fmc2_setup(struct stm32_fmc2_nfc *fmc2, int cs)
{
	u32 bcr = readl_relaxed(fmc2->io_base + FMC2_BCR1 + (cs << 3));

	bcr &= ~FMC2_BCRx_NBLSET_MASK;
	bcr |= FMC2_BCRx_NBLSET(0);
	bcr &= ~FMC2_BCRx_WAITEN;
	bcr &= ~FMC2_BCRx_FACCEN;
	bcr &= ~FMC2_BCRx_MTYP_MASK;
	bcr |= FMC2_BCRx_MTYP(0);
	bcr &= ~FMC2_BCRx_MUXEN;
	
	if(fmc2->cs_used[cs])
		bcr |= FMC2_BCRx_MBKEN;
	else
		bcr &= ~FMC2_BCRx_MBKEN;

	writel_relaxed(bcr, fmc2->io_base + FMC2_BCR1 + (cs << 3));
}

// Set bus width to 16-bit or 8-bit
static void stm32_fmc2_set_buswidth_16(struct stm32_fmc2_nfc *fmc2, int cs, bool set)
{
 	u32 bcr = readl_relaxed(fmc2->io_base + FMC2_BCR1 + (cs << 3));

	bcr &= ~FMC2_BCRx_MWID_MASK;
	if (set)
		bcr |= FMC2_BCRx_MWID(FMC2_BCRx_MWID_BUSWIDTH_16);
	else
		bcr |= FMC2_BCRx_MWID(FMC2_BCRx_MWID_BUSWIDTH_8);

	writel_relaxed(bcr, fmc2->io_base + FMC2_BCR1 + (cs << 3));
}

// Controller initialization 
static void stm32_fmc2_init(struct stm32_fmc2_nfc *fmc2)
{
	u32 bcr1 = readl_relaxed(fmc2->io_base + FMC2_BCR1);

	//Enable FMC2 controller
	bcr1 |= FMC2_BCR1_FMC2EN;

	writel_relaxed(bcr1, fmc2->io_base + FMC2_BCR1);
}

// Controller timings
static void stm32_fmc2_calc_timings(struct stm32_fmc2_nfc *fmc2)
{
	int cs;
	// set all the same for now, if we need to adjust we will modify
	for(cs = 0; cs < FMC2_MAX_CE; cs++)
	{
		if(fmc2->cs_used[cs])
		{
			fmc2->timings[cs].data_hld = 1;
			fmc2->timings[cs].acc_mode = 0;
			fmc2->timings[cs].dat_lat = 15;
			fmc2->timings[cs].clk_div = 15;
			fmc2->timings[cs].bus_turn = 0;
			fmc2->timings[cs].data_st = 5;
			fmc2->timings[cs].add_hld = 1;
			fmc2->timings[cs].add_set = 5;
		}
	}
}


// FMC2 probe 
static int stm32_fmc2_parse_child(struct stm32_fmc2_nfc *fmc2,
				  struct device_node *dn, int ifpga)
{
	const int *regs;
	u32 addr, size;
	int len;
	const char* name[] = {"FPGA1", "FPGA2", "FPGA3", "FPGA4"};

	regs = of_get_property(dn, "reg", &len);
	
	if(!regs)
		return -EINVAL;
	if(len != (sizeof(int) << 1))
	{
		dev_err(fmc2->dev, "invalid reg property size\n");
		return -EINVAL;
	}
	addr = be32_to_cpu((__be32)regs[0]);
	size = be32_to_cpu((__be32)regs[1]);

	fmc2->uio_info.mem[ifpga].addr = addr;
	fmc2->uio_info.mem[ifpga].size = size;
	fmc2->uio_info.mem[ifpga].memtype = UIO_MEM_PHYS;
	fmc2->uio_info.mem[ifpga].name = name[ifpga];
	
	return 0;
}

static int stm32_fmc2_parse_dt(struct stm32_fmc2_nfc *fmc2)
{
	struct device_node *dn = fmc2->dev->of_node;
	struct device_node *child;
	int nfpgas = of_get_child_count(dn);
	int ifpga = 0;
	int ret = 0;
	
	if (!nfpgas) {
		dev_err(fmc2->dev, "FPGAs not defined\n");
		return -EINVAL;
	}

	if (nfpgas > FMC2_MAX_CE) {
		dev_err(fmc2->dev, "too many FPGAs defined\n");
		return -EINVAL;
	}

	for_each_child_of_node(dn, child) {
		ret = stm32_fmc2_parse_child(fmc2, child, ifpga);
		if (ret < 0) {
			of_node_put(child);
			return ret;
		}
		fmc2->cs_used[ifpga++] = true;
	}
	
	return ret;
}

static int stm32_fmc2_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct reset_control *rstc;
	struct stm32_fmc2_nfc *fmc2;
	struct resource *res;
	int cs, ret;
	int uioreg;
	
	fmc2 = devm_kzalloc(dev, sizeof(*fmc2), GFP_KERNEL);
	if (!fmc2)
		return -ENOMEM;

	fmc2->dev = dev;

	ret = stm32_fmc2_parse_dt(fmc2);
	if (ret)
		return ret;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	
	fmc2->io_phys_addr = res->start;
	fmc2->io_base = devm_ioremap_resource(dev, res);
	if (IS_ERR(fmc2->io_base))
		return PTR_ERR(fmc2->io_base);

	fmc2->uio_info.name="FMC-FPGA";
	fmc2->uio_info.version="0.0.1";

	fmc2->irq = platform_get_irq(pdev, 0);
	if (fmc2->irq < 0) {
		if (fmc2->irq != -EPROBE_DEFER)
			dev_err(dev, "IRQ error missing or invalid\n");
		return fmc2->irq;
	}
	fmc2->uio_info.irq = fmc2->irq;
	fmc2->uio_info.irq_flags = IRQF_SHARED;
	fmc2->uio_info.handler = fmc_fpga_handler;
	
	// Calculate timings
	stm32_fmc2_calc_timings(fmc2);

	// Setup FMC clock
	fmc2->clk = devm_clk_get(dev, NULL);
	if (IS_ERR(fmc2->clk))
		return PTR_ERR(fmc2->clk);
	ret = clk_prepare_enable(fmc2->clk);
	if (ret) {
		dev_err(dev, "can not enable the clock\n");
		return ret;
	}
	rstc = devm_reset_control_get(dev, NULL);
	if (!IS_ERR(rstc)) {
		reset_control_assert(rstc);
		reset_control_deassert(rstc);
	}
	// FMC2 registers programming
	for(cs = 0; cs < FMC2_MAX_CE; cs++)
	{
		if(fmc2->cs_used[cs])
		{
			stm32_fmc2_setup(fmc2, cs);
			stm32_fmc2_timings_init(fmc2, cs);
		}
	}	

	// Reconfigure bus width to 16-bit
	for(cs = 0; cs < FMC2_MAX_CE; cs++)
	{
		if(fmc2->cs_used[cs])
			stm32_fmc2_set_buswidth_16(fmc2, cs, true);
	}	

	// Setup UIO
	uioreg = uio_register_device(dev, &fmc2->uio_info);
	if(uioreg)
	{
		dev_err(dev, "uio_register_device failed code = %d\n", uioreg);
		return -ENODEV;
	}
	
	//ret = devm_request_irq(dev, irq, stm32_fmc2_irq, 0,
	//		       dev_name(dev), fmc2);
	//if (ret) {
	//	dev_err(dev, "failed to request irq\n");
	//	return ret;
	//}

	init_completion(&fmc2->complete);

	// DMA setup
	//ret = stm32_fmc2_dma_setup(fmc2);
	//if (ret)
	//	return ret;

	// FMC2 init routine
	stm32_fmc2_init(fmc2);
	platform_set_drvdata(pdev, fmc2);

	return 0;
}

static int stm32_fmc2_remove(struct platform_device *pdev)
{
	struct stm32_fmc2_nfc *fmc2 = platform_get_drvdata(pdev);


	//if (fmc2->dma_ecc_ch)
	//	dma_release_channel(fmc2->dma_ecc_ch);
	//if (fmc2->dma_tx_ch)
	//	dma_release_channel(fmc2->dma_tx_ch);
	//if (fmc2->dma_rx_ch)
	//	dma_release_channel(fmc2->dma_rx_ch);

	//sg_free_table(&fmc2->dma_data_sg);
	//sg_free_table(&fmc2->dma_ecc_sg);

	clk_disable_unprepare(fmc2->clk);

	return 0;
}

static int __maybe_unused stm32_fmc2_suspend(struct device *dev)
{
	struct stm32_fmc2_nfc *fmc2 = dev_get_drvdata(dev);

	clk_disable_unprepare(fmc2->clk);
	pinctrl_pm_select_sleep_state(dev);

	return 0;
}

static int __maybe_unused stm32_fmc2_resume(struct device *dev)
{
	struct stm32_fmc2_nfc *fmc2 = dev_get_drvdata(dev);
	int ret;

	pinctrl_pm_select_default_state(dev);
	ret = clk_prepare_enable(fmc2->clk);
	if (ret) {
		dev_err(dev, "can not enable the clock\n");
		return ret;
	}
	stm32_fmc2_init(fmc2);

	return 0;
}

static SIMPLE_DEV_PM_OPS(stm32_fmc2_pm_ops, stm32_fmc2_suspend,
			 stm32_fmc2_resume);

static const struct of_device_id stm32_fmc2_match[] = {
	{.compatible = "st,stm32mp15-fmc2-fpga"},
	{}
};
MODULE_DEVICE_TABLE(of, stm32_fmc2_match);

static struct platform_driver stm32_fmc2_driver = {
	.probe	= stm32_fmc2_probe,
	.remove	= stm32_fmc2_remove,
	.driver	= {
		.name = "stm32_fmc2_fpga",
		.of_match_table = stm32_fmc2_match,
		.pm = &stm32_fmc2_pm_ops,
	},
};
module_platform_driver(stm32_fmc2_driver);

MODULE_ALIAS("platform:stm32_fmc2_fpga");
MODULE_AUTHOR("Mark Carlin <mcarlin@datumsystems.com>");
MODULE_DESCRIPTION("STMicroelectronics STM32 FMC2 FPGA driver");
MODULE_LICENSE("GPL v2");
