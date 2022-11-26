// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) STMicroelectronics 2018 - All Rights Reserved
 * Author: Olivier Bideau <olivier.bideau@st.com> for STMicroelectronics.
 * Author: Gabriel Fernandez <gabriel.fernandez@st.com> for STMicroelectronics.
 */

#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/reset-controller.h>
#include <linux/slab.h>
#include <linux/spinlock.h>

#include <dt-bindings/clock/stm32mp1-clks.h>

static DEFINE_SPINLOCK(rlock);

#define RCC_OCENSETR		0x0C
#define RCC_HSICFGR		0x18
#define RCC_RDLSICR		0x144
#define RCC_PLL1CR		0x80
#define RCC_PLL1CFGR1		0x84
#define RCC_PLL1CFGR2		0x88
#define RCC_PLL2CR		0x94
#define RCC_PLL2CFGR1		0x98
#define RCC_PLL2CFGR2		0x9C
#define RCC_PLL3CR		0x880
#define RCC_PLL3CFGR1		0x884
#define RCC_PLL3CFGR2		0x888
#define RCC_PLL4CR		0x894
#define RCC_PLL4CFGR1		0x898
#define RCC_PLL4CFGR2		0x89C
#define RCC_APB1ENSETR		0xA00
#define RCC_APB2ENSETR		0xA08
#define RCC_APB3ENSETR		0xA10
#define RCC_APB4ENSETR		0x200
#define RCC_APB5ENSETR		0x208
#define RCC_AHB2ENSETR		0xA18
#define RCC_AHB3ENSETR		0xA20
#define RCC_AHB4ENSETR		0xA28
#define RCC_AHB5ENSETR		0x210
#define RCC_AHB6ENSETR		0x218
#define RCC_AHB6LPENSETR	0x318
#define RCC_RCK12SELR		0x28
#define RCC_RCK3SELR		0x820
#define RCC_RCK4SELR		0x824
#define RCC_MPCKSELR		0x20
#define RCC_ASSCKSELR		0x24
#define RCC_MSSCKSELR		0x48
#define RCC_SPI6CKSELR		0xC4
#define RCC_SDMMC12CKSELR	0x8F4
#define RCC_SDMMC3CKSELR	0x8F8
#define RCC_FMCCKSELR		0x904
#define RCC_I2C46CKSELR		0xC0
#define RCC_I2C12CKSELR		0x8C0
#define RCC_I2C35CKSELR		0x8C4
#define RCC_UART1CKSELR		0xC8
#define RCC_QSPICKSELR		0x900
#define RCC_ETHCKSELR		0x8FC
#define RCC_RNG1CKSELR		0xCC
#define RCC_RNG2CKSELR		0x920
#define RCC_GPUCKSELR		0x938
#define RCC_USBCKSELR		0x91C
#define RCC_STGENCKSELR		0xD4
#define RCC_SPDIFCKSELR		0x914
#define RCC_SPI2S1CKSELR	0x8D8
#define RCC_SPI2S23CKSELR	0x8DC
#define RCC_SPI2S45CKSELR	0x8E0
#define RCC_CECCKSELR		0x918
#define RCC_LPTIM1CKSELR	0x934
#define RCC_LPTIM23CKSELR	0x930
#define RCC_LPTIM45CKSELR	0x92C
#define RCC_UART24CKSELR	0x8E8
#define RCC_UART35CKSELR	0x8EC
#define RCC_UART6CKSELR		0x8E4
#define RCC_UART78CKSELR	0x8F0
#define RCC_FDCANCKSELR		0x90C
#define RCC_SAI1CKSELR		0x8C8
#define RCC_SAI2CKSELR		0x8CC
#define RCC_SAI3CKSELR		0x8D0
#define RCC_SAI4CKSELR		0x8D4
#define RCC_ADCCKSELR		0x928
#define RCC_MPCKDIVR		0x2C
#define RCC_DSICKSELR		0x924
#define RCC_CPERCKSELR		0xD0
#define RCC_MCO1CFGR		0x800
#define RCC_MCO2CFGR		0x804
#define RCC_BDCR		0x140
#define RCC_AXIDIVR		0x30
#define RCC_MCUDIVR		0x830
#define RCC_APB1DIVR		0x834
#define RCC_APB2DIVR		0x838
#define RCC_APB3DIVR		0x83C
#define RCC_APB4DIVR		0x3C
#define RCC_APB5DIVR		0x40
#define RCC_TIMG1PRER		0x828
#define RCC_TIMG2PRER		0x82C
#define RCC_RTCDIVR		0x44
#define RCC_DBGCFGR		0x80C

#define RCC_CLR	0x4

static const char * const ref12_parents[] = {
	"ck_hsi", "ck_hse"
};

static const char * const ref3_parents[] = {
	"ck_hsi", "ck_hse", "ck_csi"
};

static const char * const ref4_parents[] = {
	"ck_hsi", "ck_hse", "ck_csi", "i2s_ckin"
};

static const char * const cpu_src[] = {
	"ck_hsi", "ck_hse", "pll1_p", "pll1_p_div"
};

static const char * const axi_src[] = {
	"ck_hsi", "ck_hse", "pll2_p"
};

static const char * const per_src[] = {
	"ck_hsi", "ck_csi", "ck_hse"
};

static const char * const mcu_src[] = {
	"ck_hsi", "ck_hse", "ck_csi", "pll3_p"
};

static const char * const sdmmc12_src[] = {
	"ck_axi", "pll3_r", "pll4_p", "ck_hsi"
};

static const char * const sdmmc3_src[] = {
	"ck_mcu", "pll3_r", "pll4_p", "ck_hsi"
};

static const char * const fmc_src[] = {
	"ck_axi", "pll3_r", "pll4_p", "ck_per"
};

static const char * const qspi_src[] = {
	"ck_axi", "pll3_r", "pll4_p", "ck_per"
};

static const char * const eth_src[] = {
	"pll4_p", "pll3_q"
};

static const char * const rng_src[] = {
	"ck_csi", "pll4_r", "ck_lse", "ck_lsi"
};

static const char * const usbphy_src[] = {
	"ck_hse", "pll4_r", "clk-hse-div2"
};

static const char * const usbo_src[] = {
	"pll4_r", "ck_usbo_48m"
};

static const char * const stgen_src[] = {
	"ck_hsi", "ck_hse"
};

static const char * const spdif_src[] = {
	"pll4_p", "pll3_q", "ck_hsi"
};

static const char * const spi123_src[] = {
	"pll4_p", "pll3_q", "i2s_ckin", "ck_per", "pll3_r"
};

static const char * const spi45_src[] = {
	"pclk2", "pll4_q", "ck_hsi", "ck_csi", "ck_hse"
};

static const char * const spi6_src[] = {
	"pclk5", "pll4_q", "ck_hsi", "ck_csi", "ck_hse", "pll3_q"
};

static const char * const cec_src[] = {
	"ck_lse", "ck_lsi", "ck_csi"
};

static const char * const i2c12_src[] = {
	"pclk1", "pll4_r", "ck_hsi", "ck_csi"
};

static const char * const i2c35_src[] = {
	"pclk1", "pll4_r", "ck_hsi", "ck_csi"
};

static const char * const i2c46_src[] = {
	"pclk5", "pll3_q", "ck_hsi", "ck_csi"
};

static const char * const lptim1_src[] = {
	"pclk1", "pll4_p", "pll3_q", "ck_lse", "ck_lsi", "ck_per"
};

static const char * const lptim23_src[] = {
	"pclk3", "pll4_q", "ck_per", "ck_lse", "ck_lsi"
};

static const char * const lptim45_src[] = {
	"pclk3", "pll4_p", "pll3_q", "ck_lse", "ck_lsi", "ck_per"
};

static const char * const usart1_src[] = {
	"pclk5", "pll3_q", "ck_hsi", "ck_csi", "pll4_q", "ck_hse"
};

static const char * const usart234578_src[] = {
	"pclk1", "pll4_q", "ck_hsi", "ck_csi", "ck_hse"
};

static const char * const usart6_src[] = {
	"pclk2", "pll4_q", "ck_hsi", "ck_csi", "ck_hse"
};

static const char * const fdcan_src[] = {
	"ck_hse", "pll3_q", "pll4_q", "pll4_r"
};

static const char * const sai_src[] = {
	"pll4_q", "pll3_q", "i2s_ckin", "ck_per", "pll3_r"
};

static const char * const sai2_src[] = {
	"pll4_q", "pll3_q", "i2s_ckin", "ck_per", "spdif_ck_symb", "pll3_r"
};

static const char * const adc12_src[] = {
	"pll4_r", "ck_per", "pll3_q"
};

static const char * const dsi_src[] = {
	"ck_dsi_phy", "pll4_p"
};

static const char * const rtc_src[] = {
	"off", "ck_lse", "ck_lsi", "ck_hse"
};

static const char * const mco1_src[] = {
	"ck_hsi", "ck_hse", "ck_csi", "ck_lsi", "ck_lse"
};

static const char * const mco2_src[] = {
	"ck_mpu", "ck_axi", "ck_mcu", "pll4_p", "ck_hse", "ck_hsi"
};

static const char * const ck_trace_src[] = {
	"ck_axi"
};

static const struct clk_div_table axi_div_table[] = {
	{ 0, 1 }, { 1, 2 }, { 2, 3 }, { 3, 4 },
	{ 4, 4 }, { 5, 4 }, { 6, 4 }, { 7, 4 },
	{ 0 },
};

static const struct clk_div_table mcu_div_table[] = {
	{ 0, 1 }, { 1, 2 }, { 2, 4 }, { 3, 8 },
	{ 4, 16 }, { 5, 32 }, { 6, 64 }, { 7, 128 },
	{ 8, 256 }, { 9, 512 }, { 10, 512}, { 11, 512 },
	{ 12, 512 }, { 13, 512 }, { 14, 512}, { 15, 512 },
	{ 0 },
};

static const struct clk_div_table apb_div_table[] = {
	{ 0, 1 }, { 1, 2 }, { 2, 4 }, { 3, 8 },
	{ 4, 16 }, { 5, 16 }, { 6, 16 }, { 7, 16 },
	{ 0 },
};

static const struct clk_div_table ck_trace_div_table[] = {
	{ 0, 1 }, { 1, 2 }, { 2, 4 }, { 3, 8 },
	{ 4, 16 }, { 5, 16 }, { 6, 16 }, { 7, 16 },
	{ 0 },
};

#define MAX_MUX_CLK 2

struct stm32_mmux {
	u8 nbr_clk;
	struct clk_hw *hws[MAX_MUX_CLK];
	u8 saved_parent;
};

struct stm32_clk_mmux {
	struct clk_mux mux;
	struct stm32_mmux *mmux;
};

struct stm32_mgate {
	u8 nbr_clk;
	u32 flag;
};

struct stm32_clk_mgate {
	struct clk_gate gate;
	struct stm32_mgate *mgate;
	u32 mask;
};

struct clock_config {
	u32 id;
	const char *name;
	const char *parent_name;
	const char * const *parent_names;
	int num_parents;
	unsigned long flags;
	void *cfg;
	struct clk_hw * (*func)(struct device *dev,
				struct clk_hw_onecell_data *clk_data,
				void __iomem *base, spinlock_t *lock,
				const struct clock_config *cfg);
};

#define NO_ID ~0

struct gate_cfg {
	u32 reg_off;
	u8 bit_idx;
	u8 gate_flags;
};

struct fixed_factor_cfg {
	unsigned int mult;
	unsigned int div;
};

struct div_cfg {
	u32 reg_off;
	u8 shift;
	u8 width;
	u8 div_flags;
	const struct clk_div_table *table;
};

struct mux_cfg {
	u32 reg_off;
	u8 shift;
	u8 width;
	u8 mux_flags;
	u32 *table;
};

struct stm32_gate_cfg {
	struct gate_cfg		*gate;
	struct stm32_mgate	*mgate;
	const struct clk_ops	*ops;
};

struct stm32_div_cfg {
	struct div_cfg		*div;
	const struct clk_ops	*ops;
};

struct stm32_mux_cfg {
	struct mux_cfg		*mux;
	struct stm32_mmux	*mmux;
	const struct clk_ops	*ops;
};

/* STM32 Composite clock */
struct stm32_composite_cfg {
	const struct stm32_gate_cfg	*gate;
	const struct stm32_div_cfg	*div;
	const struct stm32_mux_cfg	*mux;
};

static struct clk_hw *
_clk_hw_register_gate(struct device *dev,
		      struct clk_hw_onecell_data *clk_data,
		      void __iomem *base, spinlock_t *lock,
		      const struct clock_config *cfg)
{
	struct gate_cfg *gate_cfg = cfg->cfg;

	return clk_hw_register_gate(dev,
				    cfg->name,
				    cfg->parent_name,
				    cfg->flags,
				    gate_cfg->reg_off + base,
				    gate_cfg->bit_idx,
				    gate_cfg->gate_flags,
				    lock);
}

static struct clk_hw *
_clk_hw_register_fixed_factor(struct device *dev,
			      struct clk_hw_onecell_data *clk_data,
			      void __iomem *base, spinlock_t *lock,
			      const struct clock_config *cfg)
{
	struct fixed_factor_cfg *ff_cfg = cfg->cfg;

	return clk_hw_register_fixed_factor(dev, cfg->name, cfg->parent_name,
					    cfg->flags, ff_cfg->mult,
					    ff_cfg->div);
}

static struct clk_hw *
_clk_hw_register_divider_table(struct device *dev,
			       struct clk_hw_onecell_data *clk_data,
			       void __iomem *base, spinlock_t *lock,
			       const struct clock_config *cfg)
{
	struct div_cfg *div_cfg = cfg->cfg;

	return clk_hw_register_divider_table(dev,
					     cfg->name,
					     cfg->parent_name,
					     cfg->flags,
					     div_cfg->reg_off + base,
					     div_cfg->shift,
					     div_cfg->width,
					     div_cfg->div_flags,
					     div_cfg->table,
					     lock);
}

static struct clk_hw *
_clk_hw_register_mux(struct device *dev,
		     struct clk_hw_onecell_data *clk_data,
		     void __iomem *base, spinlock_t *lock,
		     const struct clock_config *cfg)
{
	struct mux_cfg *mux_cfg = cfg->cfg;

	return clk_hw_register_mux(dev, cfg->name, cfg->parent_names,
				   cfg->num_parents, cfg->flags,
				   mux_cfg->reg_off + base, mux_cfg->shift,
				   mux_cfg->width, mux_cfg->mux_flags, lock);
}

/* MP1 Gate clock with set & clear registers */

static int mp1_gate_clk_enable(struct clk_hw *hw)
{
	if (!clk_gate_ops.is_enabled(hw))
		clk_gate_ops.enable(hw);

	return 0;
}

static void mp1_gate_clk_disable(struct clk_hw *hw)
{
	struct clk_gate *gate = to_clk_gate(hw);
	unsigned long flags = 0;

	if (clk_gate_ops.is_enabled(hw)) {
		spin_lock_irqsave(gate->lock, flags);
		writel_relaxed(BIT(gate->bit_idx), gate->reg + RCC_CLR);
		spin_unlock_irqrestore(gate->lock, flags);
	}
}

static const struct clk_ops mp1_gate_clk_ops = {
	.enable		= mp1_gate_clk_enable,
	.disable	= mp1_gate_clk_disable,
	.is_enabled	= clk_gate_is_enabled,
};

static struct clk_hw *_get_stm32_mux(struct device *dev, void __iomem *base,
				     const struct stm32_mux_cfg *cfg,
				     spinlock_t *lock)
{
	struct stm32_clk_mmux *mmux;
	struct clk_mux *mux;
	struct clk_hw *mux_hw;

	if (cfg->mmux) {
		mmux = devm_kzalloc(dev, sizeof(*mmux), GFP_KERNEL);
		if (!mmux)
			return ERR_PTR(-ENOMEM);

		mmux->mux.reg = cfg->mux->reg_off + base;
		mmux->mux.shift = cfg->mux->shift;
		mmux->mux.mask = (1 << cfg->mux->width) - 1;
		mmux->mux.flags = cfg->mux->mux_flags;
		mmux->mux.table = cfg->mux->table;
		mmux->mux.lock = lock;
		mmux->mmux = cfg->mmux;
		mux_hw = &mmux->mux.hw;
		cfg->mmux->hws[cfg->mmux->nbr_clk++] = mux_hw;

	} else {
		mux = devm_kzalloc(dev, sizeof(*mux), GFP_KERNEL);
		if (!mux)
			return ERR_PTR(-ENOMEM);

		mux->reg = cfg->mux->reg_off + base;
		mux->shift = cfg->mux->shift;
		mux->mask = (1 << cfg->mux->width) - 1;
		mux->flags = cfg->mux->mux_flags;
		mux->table = cfg->mux->table;
		mux->lock = lock;
		mux_hw = &mux->hw;
	}

	return mux_hw;
}

static struct clk_hw *_get_stm32_div(struct device *dev, void __iomem *base,
				     const struct stm32_div_cfg *cfg,
				     spinlock_t *lock)
{
	struct clk_divider *div;

	div = devm_kzalloc(dev, sizeof(*div), GFP_KERNEL);

	if (!div)
		return ERR_PTR(-ENOMEM);

	div->reg = cfg->div->reg_off + base;
	div->shift = cfg->div->shift;
	div->width = cfg->div->width;
	div->flags = cfg->div->div_flags;
	div->table = cfg->div->table;
	div->lock = lock;

	return &div->hw;
}

static struct clk_hw *_get_stm32_gate(struct device *dev, void __iomem *base,
				      const struct stm32_gate_cfg *cfg,
				      spinlock_t *lock)
{
	struct stm32_clk_mgate *mgate;
	struct clk_gate *gate;
	struct clk_hw *gate_hw;

	if (cfg->mgate) {
		mgate = devm_kzalloc(dev, sizeof(*mgate), GFP_KERNEL);
		if (!mgate)
			return ERR_PTR(-ENOMEM);

		mgate->gate.reg = cfg->gate->reg_off + base;
		mgate->gate.bit_idx = cfg->gate->bit_idx;
		mgate->gate.flags = cfg->gate->gate_flags;
		mgate->gate.lock = lock;
		mgate->mask = BIT(cfg->mgate->nbr_clk++);

		mgate->mgate = cfg->mgate;

		gate_hw = &mgate->gate.hw;

	} else {
		gate = devm_kzalloc(dev, sizeof(*gate), GFP_KERNEL);
		if (!gate)
			return ERR_PTR(-ENOMEM);

		gate->reg = cfg->gate->reg_off + base;
		gate->bit_idx = cfg->gate->bit_idx;
		gate->flags = cfg->gate->gate_flags;
		gate->lock = lock;

		gate_hw = &gate->hw;
	}

	return gate_hw;
}

static struct clk_hw *
clk_stm32_register_gate_ops(struct device *dev,
			    const char *name,
			    const char *parent_name,
			    unsigned long flags,
			    void __iomem *base,
			    const struct stm32_gate_cfg *cfg,
			    spinlock_t *lock)
{
	struct clk_init_data init = { NULL };
	struct clk_hw *hw;
	int ret;

	init.name = name;
	init.parent_names = &parent_name;
	init.num_parents = 1;
	init.flags = flags;

	init.ops = &clk_gate_ops;

	if (cfg->ops)
		init.ops = cfg->ops;

	hw = _get_stm32_gate(dev, base, cfg, lock);
	if (IS_ERR(hw))
		return ERR_PTR(-ENOMEM);

	hw->init = &init;

	ret = clk_hw_register(dev, hw);
	if (ret)
		hw = ERR_PTR(ret);

	return hw;
}

static struct clk_hw *
clk_stm32_register_composite(struct device *dev,
			     const char *name, const char * const *parent_names,
			     int num_parents, void __iomem *base,
			     const struct stm32_composite_cfg *cfg,
			     unsigned long flags, spinlock_t *lock)
{
	const struct clk_ops *mux_ops, *div_ops, *gate_ops;
	struct clk_hw *mux_hw, *div_hw, *gate_hw;

	mux_hw = NULL;
	div_hw = NULL;
	gate_hw = NULL;
	mux_ops = NULL;
	div_ops = NULL;
	gate_ops = NULL;

	if (cfg->mux) {
		mux_hw = _get_stm32_mux(dev, base, cfg->mux, lock);

		if (!IS_ERR(mux_hw)) {
			mux_ops = &clk_mux_ops;

			if (cfg->mux->ops)
				mux_ops = cfg->mux->ops;
		}
	}

	if (cfg->div) {
		div_hw = _get_stm32_div(dev, base, cfg->div, lock);

		if (!IS_ERR(div_hw)) {
			div_ops = &clk_divider_ops;

			if (cfg->div->ops)
				div_ops = cfg->div->ops;
		}
	}

	if (cfg->gate) {
		gate_hw = _get_stm32_gate(dev, base, cfg->gate, lock);

		if (!IS_ERR(gate_hw)) {
			gate_ops = &clk_gate_ops;

			if (cfg->gate->ops)
				gate_ops = cfg->gate->ops;
		}
	}

	return clk_hw_register_composite(dev, name, parent_names, num_parents,
				       mux_hw, mux_ops, div_hw, div_ops,
				       gate_hw, gate_ops, flags);
}

#define to_clk_mgate(_gate) container_of(_gate, struct stm32_clk_mgate, gate)

static int mp1_mgate_clk_enable(struct clk_hw *hw)
{
	struct clk_gate *gate = to_clk_gate(hw);
	struct stm32_clk_mgate *clk_mgate = to_clk_mgate(gate);

	clk_mgate->mgate->flag |= clk_mgate->mask;

	mp1_gate_clk_enable(hw);

	return  0;
}

static void mp1_mgate_clk_disable(struct clk_hw *hw)
{
	struct clk_gate *gate = to_clk_gate(hw);
	struct stm32_clk_mgate *clk_mgate = to_clk_mgate(gate);

	clk_mgate->mgate->flag &= ~clk_mgate->mask;

	if (clk_mgate->mgate->flag == 0)
		mp1_gate_clk_disable(hw);
}

static const struct clk_ops mp1_mgate_clk_ops = {
	.enable		= mp1_mgate_clk_enable,
	.disable	= mp1_mgate_clk_disable,
	.is_enabled	= clk_gate_is_enabled,

};

#define to_clk_mmux(_mux) container_of(_mux, struct stm32_clk_mmux, mux)

static u8 clk_mmux_get_parent(struct clk_hw *hw)
{
	return clk_mux_ops.get_parent(hw);
}

static int clk_mmux_set_parent(struct clk_hw *hw, u8 index)
{
	struct clk_mux *mux = to_clk_mux(hw);
	struct stm32_clk_mmux *clk_mmux = to_clk_mmux(mux);
	struct clk_hw *hwp;
	int ret, n;

	ret = clk_mux_ops.set_parent(hw, index);
	if (ret)
		return ret;

	hwp = clk_hw_get_parent(hw);

	for (n = 0; n < clk_mmux->mmux->nbr_clk; n++)
		if (clk_mmux->mmux->hws[n] != hw)
			clk_hw_set_parent(clk_mmux->mmux->hws[n], hwp);

	return 0;
}

static const struct clk_ops clk_mmux_ops = {
	.get_parent	= clk_mmux_get_parent,
	.set_parent	= clk_mmux_set_parent,
	.determine_rate	= __clk_mux_determine_rate,
};

static bool is_all_clk_on_switch_are_off(struct clk_hw *hw)
{
	struct clk_composite *composite = to_clk_composite(hw);
	struct clk_hw *mux_hw = composite->mux_hw;
	struct clk_mux *mux = to_clk_mux(mux_hw);
	struct stm32_clk_mmux *clk_mmux = to_clk_mmux(mux);
	int i = 0;

	for (i = 0; i < clk_mmux->mmux->nbr_clk; i++)
		if (__clk_is_enabled(clk_mmux->mmux->hws[i]->clk))
			return false;

	return true;
}

#define MMUX_SAFE_POSITION 0

static int clk_mmux_set_safe_position(struct clk_hw *hw)
{
	struct clk_composite *composite = to_clk_composite(hw);
	struct clk_hw *mux_hw = composite->mux_hw;
	struct clk_mux *mux = to_clk_mux(mux_hw);
	struct stm32_clk_mmux *clk_mmux = to_clk_mmux(mux);

	clk_mmux->mmux->saved_parent = clk_mmux_get_parent(mux_hw);
	clk_mux_ops.set_parent(mux_hw, MMUX_SAFE_POSITION);

	return 0;
}

static int clk_mmux_restore_parent(struct clk_hw *hw)
{
	struct clk_composite *composite = to_clk_composite(hw);
	struct clk_hw *mux_hw = composite->mux_hw;
	struct clk_mux *mux = to_clk_mux(mux_hw);
	struct stm32_clk_mmux *clk_mmux = to_clk_mmux(mux);

	clk_mux_ops.set_parent(mux_hw, clk_mmux->mmux->saved_parent);

	return 0;
}

static u8 clk_mmux_get_parent_safe(struct clk_hw *hw)
{
	struct clk_mux *mux = to_clk_mux(hw);
	struct stm32_clk_mmux *clk_mmux = to_clk_mmux(mux);

	clk_mmux->mmux->saved_parent = clk_mmux_get_parent(hw);

	return clk_mmux->mmux->saved_parent;
}

static int clk_mmux_set_parent_safe(struct clk_hw *hw, u8 index)
{
	struct clk_mux *mux = to_clk_mux(hw);
	struct stm32_clk_mmux *clk_mmux = to_clk_mmux(mux);

	clk_mmux_set_parent(hw, index);
	clk_mmux->mmux->saved_parent = index;

	return 0;
}

static const struct clk_ops clk_mmux_safe_ops = {
	.get_parent	= clk_mmux_get_parent_safe,
	.set_parent	= clk_mmux_set_parent_safe,
	.determine_rate	= __clk_mux_determine_rate,
};

static int mp1_mgate_clk_enable_safe(struct clk_hw *hw)
{
	struct clk_hw *composite_hw = __clk_get_hw(hw->clk);

	clk_mmux_restore_parent(composite_hw);
	mp1_mgate_clk_enable(hw);

	return  0;
}

static void mp1_mgate_clk_disable_safe(struct clk_hw *hw)
{
	struct clk_hw *composite_hw = __clk_get_hw(hw->clk);

	mp1_mgate_clk_disable(hw);

	if (is_all_clk_on_switch_are_off(composite_hw))
		clk_mmux_set_safe_position(composite_hw);
}

static const struct clk_ops mp1_mgate_clk_safe_ops = {
	.enable		= mp1_mgate_clk_enable_safe,
	.disable	= mp1_mgate_clk_disable_safe,
	.is_enabled	= clk_gate_is_enabled,
};

/* STM32 PLL */
struct clk_pll_fractional_divider {
	struct clk_hw hw;
	void __iomem *mreg;
	u8 mshift;
	u8 mwidth;
	u8 mflags;
	void __iomem *nreg;
	u8 nshift;
	u8 nwidth;
	u8 nflags;
	void __iomem *freg;
	u8 fshift;
	u8 fwidth;

	/* lock pll enable/disable registers */
	spinlock_t *lock;
};

#define to_pll_fractional_divider(_hw)\
	container_of(_hw, struct clk_pll_fractional_divider, hw)

static unsigned long clk_pll_frac_div_recalc_rate(struct clk_hw *hw,
						  unsigned long parent_rate)
{
	struct clk_pll_fractional_divider *fd = to_pll_fractional_divider(hw);
	u32 mmask = GENMASK(fd->mwidth - 1, 0) << fd->mshift;
	u32 nmask = GENMASK(fd->nwidth - 1, 0) << fd->nshift;
	u32 fmask = GENMASK(fd->fwidth - 1, 0) << fd->fshift;
	unsigned long m, n, f;
	u64 rate, frate = 0;
	u32 val;

	val = readl(fd->mreg);
	m = (val & mmask) >> fd->mshift;
	if (fd->mflags & CLK_FRAC_DIVIDER_ZERO_BASED)
		m++;

	val = readl(fd->nreg);
	n = (val & nmask) >> fd->nshift;
	if (fd->nflags & CLK_FRAC_DIVIDER_ZERO_BASED)
		n++;

	if (!n || !m)
		return parent_rate;

	rate = (u64)parent_rate * n;
	do_div(rate, m);

	val = readl(fd->freg);
	f = (val & fmask) >> fd->fshift;
	if (f) {
		frate = (u64)parent_rate * (u64)f;
		do_div(frate, (m * (1 << fd->fwidth)));
	}
	return rate + frate;
}

static const struct clk_ops clk_pll_frac_div_ops = {
	.recalc_rate	= clk_pll_frac_div_recalc_rate,
};

#define PLL_BIT_ON		0
#define PLL_BIT_RDY		1
#define PLL_MUX_SHIFT		0
#define PLL_MUX_MASK		3
#define PLL_DIVMN_OFFSET	4
#define PLL_DIVM_SHIFT		16
#define PLL_DIVM_WIDTH		6
#define PLL_DIVN_SHIFT		0
#define PLL_DIVN_WIDTH		9
#define PLL_FRAC_OFFSET		0xC
#define PLL_FRAC_SHIFT		3
#define PLL_FRAC_WIDTH		13

#define TIMEOUT 5

static int pll_enable(struct clk_hw *hw)
{
	struct clk_gate *gate = to_clk_gate(hw);
	u32 timeout = TIMEOUT;
	int bit_status = 0;

	if (clk_gate_ops.is_enabled(hw))
		return 0;

	clk_gate_ops.enable(hw);

	do {
		bit_status = !(readl_relaxed(gate->reg) & BIT(PLL_BIT_RDY));

		if (bit_status)
			udelay(120);

	} while (bit_status && --timeout);

	return bit_status;
}

static void pll_disable(struct clk_hw *hw)
{
	if (!clk_gate_ops.is_enabled(hw))
		return;

	clk_gate_ops.disable(hw);
}

const struct clk_ops pll_gate_ops = {
	.enable		= pll_enable,
	.disable	= pll_disable,
	.is_enabled	= clk_gate_is_enabled,
};

static struct clk_hw *clk_register_pll(struct device *dev, const char *name,
				       const char * const *parent_names,
				       int num_parents,
				       void __iomem *reg,
				       void __iomem *mux_reg,
				       unsigned long flags,
				       spinlock_t *lock)
{
	struct clk_pll_fractional_divider *frac_div;
	struct clk_gate *gate;
	struct clk_mux *mux;

	mux = devm_kzalloc(dev, sizeof(*mux), GFP_KERNEL);
	if (!mux)
		return ERR_PTR(-ENOMEM);

	mux->reg = mux_reg;
	mux->shift = PLL_MUX_SHIFT;
	mux->mask = PLL_MUX_MASK;
	mux->flags = CLK_MUX_READ_ONLY;
	mux->table = NULL;
	mux->lock = lock;

	gate = devm_kzalloc(dev, sizeof(*gate), GFP_KERNEL);
	if (!gate)
		return ERR_PTR(-ENOMEM);

	gate->reg = reg;
	gate->bit_idx = PLL_BIT_ON;
	gate->flags = 0;
	gate->lock = lock;

	frac_div = devm_kzalloc(dev, sizeof(*frac_div), GFP_KERNEL);
	if (!frac_div)
		return ERR_PTR(-ENOMEM);

	frac_div->mreg = reg + PLL_DIVMN_OFFSET;
	frac_div->mshift = PLL_DIVM_SHIFT;
	frac_div->mwidth = PLL_DIVM_WIDTH;
	frac_div->mflags = CLK_FRAC_DIVIDER_ZERO_BASED;
	frac_div->nreg = reg + PLL_DIVMN_OFFSET;
	frac_div->nshift = PLL_DIVN_SHIFT;
	frac_div->nwidth = PLL_DIVN_WIDTH;
	frac_div->nflags = CLK_FRAC_DIVIDER_ZERO_BASED;
	frac_div->freg = reg + PLL_FRAC_OFFSET;
	frac_div->fshift = PLL_FRAC_SHIFT;
	frac_div->fwidth = PLL_FRAC_WIDTH;

	return clk_hw_register_composite(dev, name, parent_names, num_parents,
					 &mux->hw, &clk_mux_ops,
					 &frac_div->hw, &clk_pll_frac_div_ops,
					 &gate->hw, &pll_gate_ops, flags);
}

/* Kernel Timer */
struct timer_cker {
	/* lock the kernel output divider register */
	spinlock_t *lock;
	void __iomem *apbdiv;
	void __iomem *timpre;
	struct clk_hw hw;
};

#define to_timer_cker(_hw) container_of(_hw, struct timer_cker, hw)

#define APB_DIV_MASK 0x07
#define TIM_PRE_MASK 0x01

static unsigned long __bestmult(struct clk_hw *hw, unsigned long rate,
				unsigned long parent_rate)
{
	struct timer_cker *tim_ker = to_timer_cker(hw);
	u32 prescaler;
	unsigned int mult = 0;

	prescaler = readl_relaxed(tim_ker->apbdiv) & APB_DIV_MASK;
	if (prescaler < 2)
		return 1;

	mult = 2;

	if (rate / parent_rate >= 4)
		mult = 4;

	return mult;
}

static long timer_ker_round_rate(struct clk_hw *hw, unsigned long rate,
				 unsigned long *parent_rate)
{
	unsigned long factor = __bestmult(hw, rate, *parent_rate);

	return *parent_rate * factor;
}

static int timer_ker_set_rate(struct clk_hw *hw, unsigned long rate,
			      unsigned long parent_rate)
{
	struct timer_cker *tim_ker = to_timer_cker(hw);
	unsigned long flags = 0;
	unsigned long factor = __bestmult(hw, rate, parent_rate);
	int ret = 0;

	spin_lock_irqsave(tim_ker->lock, flags);

	switch (factor) {
	case 1:
		break;
	case 2:
		writel_relaxed(0, tim_ker->timpre);
		break;
	case 4:
		writel_relaxed(1, tim_ker->timpre);
		break;
	default:
		ret = -EINVAL;
	}
	spin_unlock_irqrestore(tim_ker->lock, flags);

	return ret;
}

static unsigned long timer_ker_recalc_rate(struct clk_hw *hw,
					   unsigned long parent_rate)
{
	struct timer_cker *tim_ker = to_timer_cker(hw);
	u32 prescaler, timpre;
	u32 mul;

	prescaler = readl_relaxed(tim_ker->apbdiv) & APB_DIV_MASK;

	timpre = readl_relaxed(tim_ker->timpre) & TIM_PRE_MASK;

	if (!prescaler)
		return parent_rate;

	mul = (timpre + 1) * 2;

	return parent_rate * mul;
}

static const struct clk_ops timer_ker_ops = {
	.recalc_rate	= timer_ker_recalc_rate,
	.round_rate	= timer_ker_round_rate,
	.set_rate	= timer_ker_set_rate,

};

static struct clk_hw *clk_register_cktim(struct device *dev, const char *name,
					 const char *parent_name,
					 unsigned long flags,
					 void __iomem *apbdiv,
					 void __iomem *timpre,
					 spinlock_t *lock)
{
	struct timer_cker *tim_ker;
	struct clk_init_data init;
	struct clk_hw *hw;
	int err;

	tim_ker = devm_kzalloc(dev, sizeof(*tim_ker), GFP_KERNEL);
	if (!tim_ker)
		return ERR_PTR(-ENOMEM);

	init.name = name;
	init.ops = &timer_ker_ops;
	init.flags = flags;
	init.parent_names = &parent_name;
	init.num_parents = 1;

	tim_ker->hw.init = &init;
	tim_ker->lock = lock;
	tim_ker->apbdiv = apbdiv;
	tim_ker->timpre = timpre;

	hw = &tim_ker->hw;
	err = clk_hw_register(dev, hw);

	if (err)
		return ERR_PTR(err);

	return hw;
}

/* The divider of RTC clock concerns only ck_hse clock */
#define HSE_RTC 3

static unsigned long clk_divider_rtc_recalc_rate(struct clk_hw *hw,
						 unsigned long parent_rate)
{
	if (clk_hw_get_parent(hw) == clk_hw_get_parent_by_index(hw, HSE_RTC))
		return clk_divider_ops.recalc_rate(hw, parent_rate);

	return parent_rate;
}

static int clk_divider_rtc_set_rate(struct clk_hw *hw, unsigned long rate,
				    unsigned long parent_rate)
{
	if (clk_hw_get_parent(hw) == clk_hw_get_parent_by_index(hw, HSE_RTC))
		return clk_divider_ops.set_rate(hw, rate, parent_rate);

	return parent_rate;
}

static int clk_divider_rtc_determine_rate(struct clk_hw *hw, struct clk_rate_request *req)
{
	if (req->best_parent_hw == clk_hw_get_parent_by_index(hw, HSE_RTC))
		return clk_divider_ops.determine_rate(hw, req);

	req->rate = req->best_parent_rate;

	return 0;
}

static const struct clk_ops rtc_div_clk_ops = {
	.recalc_rate	= clk_divider_rtc_recalc_rate,
	.set_rate	= clk_divider_rtc_set_rate,
	.determine_rate = clk_divider_rtc_determine_rate
};

static int clk_div_get_duty_cycle(struct clk_hw *hw, struct clk_duty *duty)
{
	struct clk_divider *divider = to_clk_divider(hw);
	unsigned int val;

	val = readl(divider->reg) >> divider->shift;
	val &= clk_div_mask(divider->width);

	duty->num = (val + 1) / 2;
	duty->den = (val + 1);

	return 0;
}

static unsigned long clk_div_duty_cycle_recalc_rate(struct clk_hw *hw,
						    unsigned long parent_rate)
{
	return clk_divider_ops.recalc_rate(hw, parent_rate);
}

static long clk_div_duty_cycle_round_rate(struct clk_hw *hw, unsigned long rate,
					  unsigned long *prate)
{
	return clk_divider_ops.round_rate(hw, rate, prate);
}

static int clk_div_duty_cycle_set_rate(struct clk_hw *hw, unsigned long rate,
				       unsigned long parent_rate)
{
	return clk_divider_ops.set_rate(hw, rate, parent_rate);
}

static const struct clk_ops div_dc_clk_ops = {
	.recalc_rate	= clk_div_duty_cycle_recalc_rate,
	.round_rate	= clk_div_duty_cycle_round_rate,
	.set_rate	= clk_div_duty_cycle_set_rate,
	.get_duty_cycle = clk_div_get_duty_cycle,
};

struct stm32_pll_cfg {
	u32 offset;
	u32 muxoff;
	const struct clk_ops *ops;
};

static struct clk_hw *_clk_register_pll(struct device *dev,
					struct clk_hw_onecell_data *clk_data,
					void __iomem *base, spinlock_t *lock,
					const struct clock_config *cfg)
{
	struct stm32_pll_cfg *stm_pll_cfg = cfg->cfg;

	return clk_register_pll(dev, cfg->name, cfg->parent_names,
				cfg->num_parents,
				base + stm_pll_cfg->offset,
				base + stm_pll_cfg->muxoff,
				cfg->flags, lock);
}

struct stm32_cktim_cfg {
	u32 offset_apbdiv;
	u32 offset_timpre;
};

static struct clk_hw *_clk_register_cktim(struct device *dev,
					  struct clk_hw_onecell_data *clk_data,
					  void __iomem *base, spinlock_t *lock,
					  const struct clock_config *cfg)
{
	struct stm32_cktim_cfg *cktim_cfg = cfg->cfg;

	return clk_register_cktim(dev, cfg->name, cfg->parent_name, cfg->flags,
				  cktim_cfg->offset_apbdiv + base,
				  cktim_cfg->offset_timpre + base, lock);
}

static struct clk_hw *
_clk_stm32_register_gate(struct device *dev,
			 struct clk_hw_onecell_data *clk_data,
			 void __iomem *base, spinlock_t *lock,
			 const struct clock_config *cfg)
{
	return clk_stm32_register_gate_ops(dev,
				    cfg->name,
				    cfg->parent_name,
				    cfg->flags,
				    base,
				    cfg->cfg,
				    lock);
}

static struct clk_hw *
_clk_stm32_register_composite(struct device *dev,
			      struct clk_hw_onecell_data *clk_data,
			      void __iomem *base, spinlock_t *lock,
			      const struct clock_config *cfg)
{
	return clk_stm32_register_composite(dev, cfg->name, cfg->parent_names,
					    cfg->num_parents, base, cfg->cfg,
					    cfg->flags, lock);
}

#define GATE(_id, _name, _parent, _flags, _offset, _bit_idx, _gate_flags)\
{\
	.id		= _id,\
	.name		= _name,\
	.parent_name	= _parent,\
	.flags		= _flags,\
	.cfg		=  &(struct gate_cfg) {\
		.reg_off	= _offset,\
		.bit_idx	= _bit_idx,\
		.gate_flags	= _gate_flags,\
	},\
	.func		= _clk_hw_register_gate,\
}

#define FIXED_FACTOR(_id, _name, _parent, _flags, _mult, _div)\
{\
	.id		= _id,\
	.name		= _name,\
	.parent_name	= _parent,\
	.flags		= _flags,\
	.cfg		=  &(struct fixed_factor_cfg) {\
		.mult = _mult,\
		.div = _div,\
	},\
	.func		= _clk_hw_register_fixed_factor,\
}

#define DIV_TABLE(_id, _name, _parent, _flags, _offset, _shift, _width,\
		  _div_flags, _div_table)\
{\
	.id		= _id,\
	.name		= _name,\
	.parent_name	= _parent,\
	.flags		= _flags,\
	.cfg		=  &(struct div_cfg) {\
		.reg_off	= _offset,\
		.shift		= _shift,\
		.width		= _width,\
		.div_flags	= _div_flags,\
		.table		= _div_table,\
	},\
	.func		= _clk_hw_register_divider_table,\
}

#define DIV(_id, _name, _parent, _flags, _offset, _shift, _width, _div_flags)\
	DIV_TABLE(_id, _name, _parent, _flags, _offset, _shift, _width,\
		  _div_flags, NULL)

#define MUX(_id, _name, _parents, _flags, _offset, _shift, _width, _mux_flags)\
{\
	.id		= _id,\
	.name		= _name,\
	.parent_names	= _parents,\
	.num_parents	= ARRAY_SIZE(_parents),\
	.flags		= _flags,\
	.cfg		=  &(struct mux_cfg) {\
		.reg_off	= _offset,\
		.shift		= _shift,\
		.width		= _width,\
		.mux_flags	= _mux_flags,\
	},\
	.func		= _clk_hw_register_mux,\
}

#define PLL(_id, _name, _parents, _flags, _offset_p, _offset_mux)\
{\
	.id		= _id,\
	.name		= _name,\
	.parent_names	= _parents,\
	.num_parents	= ARRAY_SIZE(_parents),\
	.flags		= CLK_IGNORE_UNUSED | (_flags),\
	.cfg		=  &(struct stm32_pll_cfg) {\
		.offset = _offset_p,\
		.muxoff = _offset_mux,\
	},\
	.func		= _clk_register_pll,\
}

#define STM32_CKTIM(_name, _parent, _flags, _offset_apbdiv, _offset_timpre)\
{\
	.id		= NO_ID,\
	.name		= _name,\
	.parent_name	= _parent,\
	.flags		= _flags,\
	.cfg		=  &(struct stm32_cktim_cfg) {\
		.offset_apbdiv = _offset_apbdiv,\
		.offset_timpre = _offset_timpre,\
	},\
	.func		= _clk_register_cktim,\
}

#define STM32_TIM(_id, _name, _parent, _offset_set, _bit_idx)\
		  GATE_MP1(_id, _name, _parent, CLK_SET_RATE_PARENT,\
			   _offset_set, _bit_idx, 0)

/* STM32 GATE */
#define STM32_GATE(_id, _name, _parent, _flags, _gate)\
{\
	.id		= _id,\
	.name		= _name,\
	.parent_name	= _parent,\
	.flags		= _flags,\
	.cfg		= (struct stm32_gate_cfg *) {_gate},\
	.func		= _clk_stm32_register_gate,\
}

#define _STM32_GATE(_gate_offset, _gate_bit_idx, _gate_flags, _mgate, _ops)\
	(&(struct stm32_gate_cfg) {\
		&(struct gate_cfg) {\
			.reg_off	= _gate_offset,\
			.bit_idx	= _gate_bit_idx,\
			.gate_flags	= _gate_flags,\
		},\
		.mgate		= _mgate,\
		.ops		= _ops,\
	})

#define _STM32_MGATE(_mgate)\
	(&per_gate_cfg[_mgate])

#define _GATE(_gate_offset, _gate_bit_idx, _gate_flags)\
	_STM32_GATE(_gate_offset, _gate_bit_idx, _gate_flags,\
		    NULL, NULL)\

#define _GATE_MP1(_gate_offset, _gate_bit_idx, _gate_flags)\
	_STM32_GATE(_gate_offset, _gate_bit_idx, _gate_flags,\
		    NULL, &mp1_gate_clk_ops)\

#define _MGATE_MP1(_mgate)\
	&per_gate_cfg[_mgate]

#define GATE_MP1(_id, _name, _parent, _flags, _offset, _bit_idx, _gate_flags)\
	STM32_GATE(_id, _name, _parent, _flags,\
		   _GATE_MP1(_offset, _bit_idx, _gate_flags))

#define MGATE_MP1(_id, _name, _parent, _flags, _mgate)\
	STM32_GATE(_id, _name, _parent, _flags,\
		   _STM32_MGATE(_mgate))

#define _STM32_DIV(_div_offset, _div_shift, _div_width,\
		   _div_flags, _div_table, _ops)\
	(&(struct stm32_div_cfg) {\
		&(struct div_cfg) {\
			.reg_off	= _div_offset,\
			.shift		= _div_shift,\
			.width		= _div_width,\
			.div_flags	= _div_flags,\
			.table		= _div_table,\
		},\
		.ops		= _ops,\
	})

#define _DIV(_div_offset, _div_shift, _div_width, _div_flags, _div_table)\
	_STM32_DIV(_div_offset, _div_shift, _div_width,\
		   _div_flags, _div_table, NULL)

#define _DIV_DUTY_CYCLE(_div_offset, _div_shift, _div_width, _div_flags,\
			_div_table)\
	_STM32_DIV(_div_offset, _div_shift, _div_width,\
		   _div_flags, _div_table, &div_dc_clk_ops)

#define _DIV_RTC(_div_offset, _div_shift, _div_width, _div_flags, _div_table)\
	_STM32_DIV(_div_offset, _div_shift, _div_width,\
		   _div_flags, _div_table, &rtc_div_clk_ops)

#define _STM32_MUX(_offset, _shift, _width, _mux_flags, _mmux, _ops)\
	(&(struct stm32_mux_cfg) {\
		&(struct mux_cfg) {\
			.reg_off	= _offset,\
			.shift		= _shift,\
			.width		= _width,\
			.mux_flags	= _mux_flags,\
			.table		= NULL,\
		},\
		.mmux		= _mmux,\
		.ops		= _ops,\
	})

#define _MUX(_offset, _shift, _width, _mux_flags)\
	_STM32_MUX(_offset, _shift, _width, _mux_flags, NULL, NULL)

#define _MMUX(_mmux)		&ker_mux_cfg[_mmux]

#define PARENT(_parent)		((const char *[]) { _parent})

#define _NO_MUX			NULL
#define _NO_DIV			NULL
#define _NO_GATE		NULL

#define COMPOSITE(_id, _name, _parents, _flags, _gate, _mux, _div)\
{\
	.id		= _id,\
	.name		= _name,\
	.parent_names	= _parents,\
	.num_parents	= ARRAY_SIZE(_parents),\
	.flags		= _flags,\
	.cfg		= &(struct stm32_composite_cfg) {\
		.gate = (_gate),\
		.mux = (_mux),\
		.div = (_div),\
	},\
	.func		= _clk_stm32_register_composite,\
}

#define PCLK(_id, _name, _parent, _flags, _mgate)\
	MGATE_MP1(_id, _name, _parent, _flags, _mgate)

#define KCLK(_id, _name, _parents, _flags, _mgate, _mmux)\
	     COMPOSITE(_id, _name, _parents, CLK_OPS_PARENT_ENABLE |\
		       CLK_SET_RATE_NO_REPARENT | _flags,\
		       _MGATE_MP1(_mgate),\
		       _MMUX(_mmux),\
		       _NO_DIV)

enum {
	G_SAI1,
	G_SAI2,
	G_SAI3,
	G_SAI4,
	G_SPI1,
	G_SPI2,
	G_SPI3,
	G_SPI4,
	G_SPI5,
	G_SPI6,
	G_SPDIF,
	G_I2C1,
	G_I2C2,
	G_I2C3,
	G_I2C4,
	G_I2C5,
	G_I2C6,
	G_USART2,
	G_UART4,
	G_USART3,
	G_UART5,
	G_USART1,
	G_USART6,
	G_UART7,
	G_UART8,
	G_LPTIM1,
	G_LPTIM2,
	G_LPTIM3,
	G_LPTIM4,
	G_LPTIM5,
	G_LTDC,
	G_DSI,
	G_QSPI,
	G_FMC,
	G_SDMMC1,
	G_SDMMC2,
	G_SDMMC3,
	G_USBO,
	G_USBPHY,
	G_RNG1,
	G_RNG2,
	G_FDCAN,
	G_DAC12,
	G_CEC,
	G_ADC12,
	G_GPU,
	G_STGEN,
	G_DFSDM,
	G_ADFSDM,
	G_TIM2,
	G_TIM3,
	G_TIM4,
	G_TIM5,
	G_TIM6,
	G_TIM7,
	G_TIM12,
	G_TIM13,
	G_TIM14,
	G_MDIO,
	G_TIM1,
	G_TIM8,
	G_TIM15,
	G_TIM16,
	G_TIM17,
	G_SYSCFG,
	G_VREF,
	G_TMPSENS,
	G_PMBCTRL,
	G_HDP,
	G_IWDG2,
	G_STGENRO,
	G_DMA1,
	G_DMA2,
	G_DMAMUX,
	G_DCMI,
	G_CRYP2,
	G_HASH2,
	G_CRC2,
	G_HSEM,
	G_IPCC,
	G_GPIOA,
	G_GPIOB,
	G_GPIOC,
	G_GPIOD,
	G_GPIOE,
	G_GPIOF,
	G_GPIOG,
	G_GPIOH,
	G_GPIOI,
	G_GPIOJ,
	G_GPIOK,
	G_MDMA,
	G_ETHCK,
	G_ETHTX,
	G_ETHRX,
	G_ETHMAC,
	G_CRC1,
	G_USBH,
	G_ETHSTP,
	G_RTCAPB,
	G_TZC1,
	G_TZC2,
	G_TZPC,
	G_IWDG1,
	G_BSEC,
	G_GPIOZ,
	G_CRYP1,
	G_HASH1,
	G_BKPSRAM,
	G_DDRPERFM,

	G_LAST
};

static struct stm32_mgate mp1_mgate[G_LAST];

#define _K_GATE(_id, _gate_offset, _gate_bit_idx, _gate_flags,\
	       _mgate, _ops)\
	[_id] = {\
		&(struct gate_cfg) {\
			.reg_off	= _gate_offset,\
			.bit_idx	= _gate_bit_idx,\
			.gate_flags	= _gate_flags,\
		},\
		.mgate		= _mgate,\
		.ops		= _ops,\
	}

#define K_GATE(_id, _gate_offset, _gate_bit_idx, _gate_flags)\
	_K_GATE(_id, _gate_offset, _gate_bit_idx, _gate_flags,\
	       NULL, &mp1_gate_clk_ops)

#define K_MGATE(_id, _gate_offset, _gate_bit_idx, _gate_flags)\
	_K_GATE(_id, _gate_offset, _gate_bit_idx, _gate_flags,\
	       &mp1_mgate[_id], &mp1_mgate_clk_ops)

#define K_MGATE_SAFE(_id, _gate_offset, _gate_bit_idx, _gate_flags)\
	_K_GATE(_id, _gate_offset, _gate_bit_idx, _gate_flags,\
		&mp1_mgate[_id], &mp1_mgate_clk_safe_ops)

/* Peripheral gates */
static struct stm32_gate_cfg per_gate_cfg[G_LAST] = {
	/* Multi gates */
	K_GATE(G_MDIO,		RCC_APB1ENSETR, 31, 0),
	K_MGATE(G_DAC12,	RCC_APB1ENSETR, 29, 0),
	K_MGATE(G_CEC,		RCC_APB1ENSETR, 27, 0),
	K_MGATE(G_SPDIF,	RCC_APB1ENSETR, 26, 0),
	K_MGATE(G_I2C5,		RCC_APB1ENSETR, 24, 0),
	K_MGATE(G_I2C3,		RCC_APB1ENSETR, 23, 0),
	K_MGATE(G_I2C2,		RCC_APB1ENSETR, 22, 0),
	K_MGATE(G_I2C1,		RCC_APB1ENSETR, 21, 0),
	K_MGATE(G_UART8,	RCC_APB1ENSETR, 19, 0),
	K_MGATE(G_UART7,	RCC_APB1ENSETR, 18, 0),
	K_MGATE(G_UART5,	RCC_APB1ENSETR, 17, 0),
	K_MGATE(G_UART4,	RCC_APB1ENSETR, 16, 0),
	K_MGATE(G_USART3,	RCC_APB1ENSETR, 15, 0),
	K_MGATE(G_USART2,	RCC_APB1ENSETR, 14, 0),
	K_MGATE(G_SPI3,		RCC_APB1ENSETR, 12, 0),
	K_MGATE(G_SPI2,		RCC_APB1ENSETR, 11, 0),
	K_MGATE(G_LPTIM1,	RCC_APB1ENSETR, 9, 0),
	K_GATE(G_TIM14,		RCC_APB1ENSETR, 8, 0),
	K_GATE(G_TIM13,		RCC_APB1ENSETR, 7, 0),
	K_GATE(G_TIM12,		RCC_APB1ENSETR, 6, 0),
	K_GATE(G_TIM7,		RCC_APB1ENSETR, 5, 0),
	K_GATE(G_TIM6,		RCC_APB1ENSETR, 4, 0),
	K_GATE(G_TIM5,		RCC_APB1ENSETR, 3, 0),
	K_GATE(G_TIM4,		RCC_APB1ENSETR, 2, 0),
	K_GATE(G_TIM3,		RCC_APB1ENSETR, 1, 0),
	K_GATE(G_TIM2,		RCC_APB1ENSETR, 0, 0),

	K_MGATE(G_FDCAN,	RCC_APB2ENSETR, 24, 0),
	K_GATE(G_ADFSDM,	RCC_APB2ENSETR, 21, 0),
	K_GATE(G_DFSDM,		RCC_APB2ENSETR, 20, 0),
	K_MGATE(G_SAI3,		RCC_APB2ENSETR, 18, 0),
	K_MGATE(G_SAI2,		RCC_APB2ENSETR, 17, 0),
	K_MGATE(G_SAI1,		RCC_APB2ENSETR, 16, 0),
	K_MGATE(G_USART6,	RCC_APB2ENSETR, 13, 0),
	K_MGATE(G_SPI5,		RCC_APB2ENSETR, 10, 0),
	K_MGATE(G_SPI4,		RCC_APB2ENSETR, 9, 0),
	K_MGATE(G_SPI1,		RCC_APB2ENSETR, 8, 0),
	K_GATE(G_TIM17,		RCC_APB2ENSETR, 4, 0),
	K_GATE(G_TIM16,		RCC_APB2ENSETR, 3, 0),
	K_GATE(G_TIM15,		RCC_APB2ENSETR, 2, 0),
	K_GATE(G_TIM8,		RCC_APB2ENSETR, 1, 0),
	K_GATE(G_TIM1,		RCC_APB2ENSETR, 0, 0),

	K_GATE(G_HDP,		RCC_APB3ENSETR, 20, 0),
	K_GATE(G_PMBCTRL,	RCC_APB3ENSETR, 17, 0),
	K_GATE(G_TMPSENS,	RCC_APB3ENSETR, 16, 0),
	K_GATE(G_VREF,		RCC_APB3ENSETR, 13, 0),
	K_GATE(G_SYSCFG,	RCC_APB3ENSETR, 11, 0),
	K_MGATE(G_SAI4,		RCC_APB3ENSETR, 8, 0),
	K_MGATE(G_LPTIM5,	RCC_APB3ENSETR, 3, 0),
	K_MGATE(G_LPTIM4,	RCC_APB3ENSETR, 2, 0),
	K_MGATE(G_LPTIM3,	RCC_APB3ENSETR, 1, 0),
	K_MGATE(G_LPTIM2,	RCC_APB3ENSETR, 0, 0),

	K_GATE(G_STGENRO,	RCC_APB4ENSETR, 20, 0),
	K_MGATE(G_USBPHY,	RCC_APB4ENSETR, 16, 0),
	K_GATE(G_IWDG2,		RCC_APB4ENSETR, 15, 0),
	K_GATE(G_DDRPERFM,	RCC_APB4ENSETR, 8, 0),
	K_MGATE(G_DSI,		RCC_APB4ENSETR, 4, 0),
	K_MGATE(G_LTDC,		RCC_APB4ENSETR, 0, 0),

	K_GATE(G_STGEN,		RCC_APB5ENSETR, 20, 0),
	K_GATE(G_BSEC,		RCC_APB5ENSETR, 16, 0),
	K_GATE(G_IWDG1,		RCC_APB5ENSETR, 15, 0),
	K_GATE(G_TZPC,		RCC_APB5ENSETR, 13, 0),
	K_GATE(G_TZC2,		RCC_APB5ENSETR, 12, 0),
	K_GATE(G_TZC1,		RCC_APB5ENSETR, 11, 0),
	K_GATE(G_RTCAPB,	RCC_APB5ENSETR, 8, 0),
	K_MGATE(G_USART1,	RCC_APB5ENSETR, 4, 0),
	K_MGATE(G_I2C6,		RCC_APB5ENSETR, 3, 0),
	K_MGATE(G_I2C4,		RCC_APB5ENSETR, 2, 0),
	K_MGATE(G_SPI6,		RCC_APB5ENSETR, 0, 0),

	K_MGATE(G_SDMMC3,	RCC_AHB2ENSETR, 16, 0),
	K_MGATE(G_USBO,		RCC_AHB2ENSETR, 8, 0),
	K_MGATE(G_ADC12,	RCC_AHB2ENSETR, 5, 0),
	K_GATE(G_DMAMUX,	RCC_AHB2ENSETR, 2, 0),
	K_GATE(G_DMA2,		RCC_AHB2ENSETR, 1, 0),
	K_GATE(G_DMA1,		RCC_AHB2ENSETR, 0, 0),

	K_GATE(G_IPCC,		RCC_AHB3ENSETR, 12, 0),
	K_GATE(G_HSEM,		RCC_AHB3ENSETR, 11, 0),
	K_GATE(G_CRC2,		RCC_AHB3ENSETR, 7, 0),
	K_MGATE(G_RNG2,		RCC_AHB3ENSETR, 6, 0),
	K_GATE(G_HASH2,		RCC_AHB3ENSETR, 5, 0),
	K_GATE(G_CRYP2,		RCC_AHB3ENSETR, 4, 0),
	K_GATE(G_DCMI,		RCC_AHB3ENSETR, 0, 0),

	K_GATE(G_GPIOK,		RCC_AHB4ENSETR, 10, 0),
	K_GATE(G_GPIOJ,		RCC_AHB4ENSETR, 9, 0),
	K_GATE(G_GPIOI,		RCC_AHB4ENSETR, 8, 0),
	K_GATE(G_GPIOH,		RCC_AHB4ENSETR, 7, 0),
	K_GATE(G_GPIOG,		RCC_AHB4ENSETR, 6, 0),
	K_GATE(G_GPIOF,		RCC_AHB4ENSETR, 5, 0),
	K_GATE(G_GPIOE,		RCC_AHB4ENSETR, 4, 0),
	K_GATE(G_GPIOD,		RCC_AHB4ENSETR, 3, 0),
	K_GATE(G_GPIOC,		RCC_AHB4ENSETR, 2, 0),
	K_GATE(G_GPIOB,		RCC_AHB4ENSETR, 1, 0),
	K_GATE(G_GPIOA,		RCC_AHB4ENSETR, 0, 0),

	K_GATE(G_BKPSRAM,	RCC_AHB5ENSETR, 8, 0),
	K_MGATE(G_RNG1,		RCC_AHB5ENSETR, 6, 0),
	K_GATE(G_HASH1,		RCC_AHB5ENSETR, 5, 0),
	K_GATE(G_CRYP1,		RCC_AHB5ENSETR, 4, 0),
	K_GATE(G_GPIOZ,		RCC_AHB5ENSETR, 0, 0),

	K_GATE(G_USBH,		RCC_AHB6ENSETR, 24, 0),
	K_GATE(G_CRC1,		RCC_AHB6ENSETR, 20, 0),
	K_MGATE_SAFE(G_SDMMC2,	RCC_AHB6ENSETR, 17, 0),
	K_MGATE_SAFE(G_SDMMC1,	RCC_AHB6ENSETR, 16, 0),
	K_MGATE_SAFE(G_QSPI,	RCC_AHB6ENSETR, 14, 0),
	K_MGATE_SAFE(G_FMC,	RCC_AHB6ENSETR, 12, 0),
	K_GATE(G_ETHMAC,	RCC_AHB6ENSETR, 10, 0),
	K_GATE(G_ETHRX,		RCC_AHB6ENSETR, 9, 0),
	K_GATE(G_ETHTX,		RCC_AHB6ENSETR, 8, 0),
	K_GATE(G_ETHCK,		RCC_AHB6ENSETR, 7, 0),
	K_MGATE(G_GPU,		RCC_AHB6ENSETR, 5, 0),
	K_GATE(G_MDMA,		RCC_AHB6ENSETR, 0, 0),
	K_GATE(G_ETHSTP,	RCC_AHB6LPENSETR, 11, 0),
};

enum {
	M_SDMMC12,
	M_SDMMC3,
	M_FMC,
	M_QSPI,
	M_RNG1,
	M_RNG2,
	M_USBPHY,
	M_USBO,
	M_STGEN,
	M_SPDIF,
	M_SPI1,
	M_SPI23,
	M_SPI45,
	M_SPI6,
	M_CEC,
	M_I2C12,
	M_I2C35,
	M_I2C46,
	M_LPTIM1,
	M_LPTIM23,
	M_LPTIM45,
	M_USART1,
	M_UART24,
	M_UART35,
	M_USART6,
	M_UART78,
	M_SAI1,
	M_SAI2,
	M_SAI3,
	M_SAI4,
	M_DSI,
	M_FDCAN,
	M_ADC12,
	M_ETHCK,
	M_CKPER,
	M_LAST
};

static struct stm32_mmux ker_mux[M_LAST];

#define _K_MUX(_id, _offset, _shift, _width, _mux_flags, _mmux, _ops)\
	[_id] = {\
		&(struct mux_cfg) {\
			.reg_off	= _offset,\
			.shift		= _shift,\
			.width		= _width,\
			.mux_flags	= _mux_flags,\
			.table		= NULL,\
		},\
		.mmux		= _mmux,\
		.ops		= _ops,\
	}

#define K_MUX(_id, _offset, _shift, _width, _mux_flags)\
	_K_MUX(_id, _offset, _shift, _width, _mux_flags,\
			NULL, NULL)

#define K_MMUX(_id, _offset, _shift, _width, _mux_flags)\
	_K_MUX(_id, _offset, _shift, _width, _mux_flags,\
			&ker_mux[_id], &clk_mmux_ops)

#define K_MMUX_SAFE(_id, _offset, _shift, _width, _mux_flags)\
	_K_MUX(_id, _offset, _shift, _width, _mux_flags,\
			&ker_mux[_id], &clk_mmux_safe_ops)

static const struct stm32_mux_cfg ker_mux_cfg[M_LAST] = {
	/* Kernel multi mux */
	K_MMUX_SAFE(M_SDMMC12, RCC_SDMMC12CKSELR, 0, 3, 0),
	K_MMUX(M_SPI23, RCC_SPI2S23CKSELR, 0, 3, 0),
	K_MMUX(M_SPI45, RCC_SPI2S45CKSELR, 0, 3, 0),
	K_MMUX(M_I2C12, RCC_I2C12CKSELR, 0, 3, 0),
	K_MMUX(M_I2C35, RCC_I2C35CKSELR, 0, 3, 0),
	K_MMUX(M_LPTIM23, RCC_LPTIM23CKSELR, 0, 3, 0),
	K_MMUX(M_LPTIM45, RCC_LPTIM45CKSELR, 0, 3, 0),
	K_MMUX(M_UART24, RCC_UART24CKSELR, 0, 3, 0),
	K_MMUX(M_UART35, RCC_UART35CKSELR, 0, 3, 0),
	K_MMUX(M_UART78, RCC_UART78CKSELR, 0, 3, 0),
	K_MMUX(M_SAI1, RCC_SAI1CKSELR, 0, 3, 0),
	K_MMUX(M_ETHCK, RCC_ETHCKSELR, 0, 2, 0),
	K_MMUX(M_I2C46, RCC_I2C46CKSELR, 0, 3, 0),

	/*  Kernel simple mux */
	K_MUX(M_RNG2, RCC_RNG2CKSELR, 0, 2, 0),
	K_MUX(M_SDMMC3, RCC_SDMMC3CKSELR, 0, 3, 0),
	K_MMUX_SAFE(M_FMC, RCC_FMCCKSELR, 0, 2, 0),
	K_MMUX_SAFE(M_QSPI, RCC_QSPICKSELR, 0, 2, 0),
	K_MUX(M_USBPHY, RCC_USBCKSELR, 0, 2, 0),
	K_MUX(M_USBO, RCC_USBCKSELR, 4, 1, 0),
	K_MUX(M_SPDIF, RCC_SPDIFCKSELR, 0, 2, 0),
	K_MUX(M_SPI1, RCC_SPI2S1CKSELR, 0, 3, 0),
	K_MUX(M_CEC, RCC_CECCKSELR, 0, 2, 0),
	K_MUX(M_LPTIM1, RCC_LPTIM1CKSELR, 0, 3, 0),
	K_MUX(M_USART6, RCC_UART6CKSELR, 0, 3, 0),
	K_MUX(M_FDCAN, RCC_FDCANCKSELR, 0, 2, 0),
	K_MUX(M_SAI2, RCC_SAI2CKSELR, 0, 3, 0),
	K_MUX(M_SAI3, RCC_SAI3CKSELR, 0, 3, 0),
	K_MUX(M_SAI4, RCC_SAI4CKSELR, 0, 3, 0),
	K_MUX(M_ADC12, RCC_ADCCKSELR, 0, 2, 0),
	K_MUX(M_DSI, RCC_DSICKSELR, 0, 1, 0),
	K_MUX(M_CKPER, RCC_CPERCKSELR, 0, 2, 0),
	K_MUX(M_RNG1, RCC_RNG1CKSELR, 0, 2, 0),
	K_MUX(M_STGEN, RCC_STGENCKSELR, 0, 2, 0),
	K_MUX(M_USART1, RCC_UART1CKSELR, 0, 3, 0),
	K_MUX(M_SPI6, RCC_SPI6CKSELR, 0, 3, 0),
};

static const struct clock_config stm32mp1_clock_cfg[] = {
	/*  External / Internal Oscillators */
	GATE_MP1(CK_HSE, "ck_hse", "clk-hse", 0, RCC_OCENSETR, 8, 0),
	/* ck_csi is used by IO compensation and should be critical */
	GATE_MP1(CK_CSI, "ck_csi", "clk-csi", CLK_IS_CRITICAL,
		 RCC_OCENSETR, 4, 0),
	COMPOSITE(CK_HSI, "ck_hsi", PARENT("clk-hsi"), 0,
		  _GATE_MP1(RCC_OCENSETR, 0, 0),
		  _NO_MUX,
		  _DIV(RCC_HSICFGR, 0, 2, CLK_DIVIDER_POWER_OF_TWO |
		       CLK_DIVIDER_READ_ONLY, NULL)),
	GATE(CK_LSI, "ck_lsi", "clk-lsi", 0, RCC_RDLSICR, 0, 0),
	GATE(CK_LSE, "ck_lse", "clk-lse", 0, RCC_BDCR, 0, 0),

	FIXED_FACTOR(CK_HSE_DIV2, "clk-hse-div2", "ck_hse", 0, 1, 2),

	/* PLLs */
	PLL(PLL1, "pll1", ref12_parents, 0, RCC_PLL1CR, RCC_RCK12SELR),
	PLL(PLL2, "pll2", ref12_parents, 0, RCC_PLL2CR, RCC_RCK12SELR),
	PLL(PLL3, "pll3", ref3_parents, 0, RCC_PLL3CR, RCC_RCK3SELR),
	PLL(PLL4, "pll4", ref4_parents, 0, RCC_PLL4CR, RCC_RCK4SELR),

	/* ODF */
	COMPOSITE(PLL1_P, "pll1_p", PARENT("pll1"), CLK_SET_RATE_PARENT,
		  _GATE(RCC_PLL1CR, 4, 0),
		  _NO_MUX,
		  _DIV(RCC_PLL1CFGR2, 0, 7, 0, NULL)),

	COMPOSITE(PLL2_P, "pll2_p", PARENT("pll2"), 0,
		  _GATE(RCC_PLL2CR, 4, 0),
		  _NO_MUX,
		  _DIV(RCC_PLL2CFGR2, 0, 7, 0, NULL)),

	COMPOSITE(PLL2_Q, "pll2_q", PARENT("pll2"), 0,
		  _GATE(RCC_PLL2CR, 5, 0),
		  _NO_MUX,
		  _DIV(RCC_PLL2CFGR2, 8, 7, 0, NULL)),

	COMPOSITE(PLL2_R, "pll2_r", PARENT("pll2"), CLK_IS_CRITICAL,
		  _GATE(RCC_PLL2CR, 6, 0),
		  _NO_MUX,
		  _DIV(RCC_PLL2CFGR2, 16, 7, 0, NULL)),

	COMPOSITE(PLL3_P, "pll3_p", PARENT("pll3"), 0,
		  _GATE(RCC_PLL3CR, 4, 0),
		  _NO_MUX,
		  _DIV(RCC_PLL3CFGR2, 0, 7, 0, NULL)),

	COMPOSITE(PLL3_Q, "pll3_q", PARENT("pll3"), 0,
		  _GATE(RCC_PLL3CR, 5, 0),
		  _NO_MUX,
		  _DIV_DUTY_CYCLE(RCC_PLL3CFGR2, 8, 7, 0, NULL)),

	COMPOSITE(PLL3_R, "pll3_r", PARENT("pll3"), 0,
		  _GATE(RCC_PLL3CR, 6, 0),
		  _NO_MUX,
		  _DIV(RCC_PLL3CFGR2, 16, 7, 0, NULL)),

	COMPOSITE(PLL4_P, "pll4_p", PARENT("pll4"), 0,
		  _GATE(RCC_PLL4CR, 4, 0),
		  _NO_MUX,
		  _DIV(RCC_PLL4CFGR2, 0, 7, 0, NULL)),

	COMPOSITE(PLL4_Q, "pll4_q", PARENT("pll4"), 0,
		  _GATE(RCC_PLL4CR, 5, 0),
		  _NO_MUX,
		  _DIV(RCC_PLL4CFGR2, 8, 7, 0, NULL)),

	COMPOSITE(PLL4_R, "pll4_r", PARENT("pll4"), 0,
		  _GATE(RCC_PLL4CR, 6, 0),
		  _NO_MUX,
		  _DIV_DUTY_CYCLE(RCC_PLL4CFGR2, 16, 7, 0, NULL)),

	/* MUX system clocks */
	MUX(CK_PER, "ck_per", per_src, CLK_OPS_PARENT_ENABLE,
	    RCC_CPERCKSELR, 0, 2, 0),

	MUX(CK_MPU, "ck_mpu", cpu_src, CLK_OPS_PARENT_ENABLE |
	    CLK_SET_RATE_PARENT | CLK_IS_CRITICAL, RCC_MPCKSELR, 0, 2, 0),

	COMPOSITE(CK_AXI, "ck_axi", axi_src, CLK_IS_CRITICAL |
		  CLK_OPS_PARENT_ENABLE,
		  _NO_GATE,
		  _MUX(RCC_ASSCKSELR, 0, 2, 0),
		  _DIV(RCC_AXIDIVR, 0, 3, 0, axi_div_table)),

	COMPOSITE(CK_MCU, "ck_mcu", mcu_src, CLK_IS_CRITICAL |
		  CLK_OPS_PARENT_ENABLE,
		  _NO_GATE,
		  _MUX(RCC_MSSCKSELR, 0, 2, 0),
		  _DIV(RCC_MCUDIVR, 0, 4, 0, mcu_div_table)),

	DIV_TABLE(PCLK1, "pclk1", "ck_mcu", CLK_IGNORE_UNUSED, RCC_APB1DIVR, 0,
		  3, CLK_DIVIDER_READ_ONLY, apb_div_table),

	DIV_TABLE(PCLK2, "pclk2", "ck_mcu", CLK_IGNORE_UNUSED, RCC_APB2DIVR, 0,
		  3, CLK_DIVIDER_READ_ONLY, apb_div_table),

	DIV_TABLE(PCLK3, "pclk3", "ck_mcu", CLK_IGNORE_UNUSED, RCC_APB3DIVR, 0,
		  3, CLK_DIVIDER_READ_ONLY, apb_div_table),

	DIV_TABLE(PCLK4, "pclk4", "ck_axi", CLK_IGNORE_UNUSED, RCC_APB4DIVR, 0,
		  3, CLK_DIVIDER_READ_ONLY, apb_div_table),

	DIV_TABLE(PCLK5, "pclk5", "ck_axi", CLK_IGNORE_UNUSED, RCC_APB5DIVR, 0,
		  3, CLK_DIVIDER_READ_ONLY, apb_div_table),

	/* Kernel Timers */
	STM32_CKTIM("ck1_tim", "pclk1", 0, RCC_APB1DIVR, RCC_TIMG1PRER),
	STM32_CKTIM("ck2_tim", "pclk2", 0, RCC_APB2DIVR, RCC_TIMG2PRER),

	STM32_TIM(TIM2_K, "tim2_k", "ck1_tim", RCC_APB1ENSETR, 0),
	STM32_TIM(TIM3_K, "tim3_k", "ck1_tim", RCC_APB1ENSETR, 1),
	STM32_TIM(TIM4_K, "tim4_k", "ck1_tim", RCC_APB1ENSETR, 2),
	STM32_TIM(TIM5_K, "tim5_k", "ck1_tim", RCC_APB1ENSETR, 3),
	STM32_TIM(TIM6_K, "tim6_k", "ck1_tim", RCC_APB1ENSETR, 4),
	STM32_TIM(TIM7_K, "tim7_k", "ck1_tim", RCC_APB1ENSETR, 5),
	STM32_TIM(TIM12_K, "tim12_k", "ck1_tim", RCC_APB1ENSETR, 6),
	STM32_TIM(TIM13_K, "tim13_k", "ck1_tim", RCC_APB1ENSETR, 7),
	STM32_TIM(TIM14_K, "tim14_k", "ck1_tim", RCC_APB1ENSETR, 8),
	STM32_TIM(TIM1_K, "tim1_k", "ck2_tim", RCC_APB2ENSETR, 0),
	STM32_TIM(TIM8_K, "tim8_k", "ck2_tim", RCC_APB2ENSETR, 1),
	STM32_TIM(TIM15_K, "tim15_k", "ck2_tim", RCC_APB2ENSETR, 2),
	STM32_TIM(TIM16_K, "tim16_k", "ck2_tim", RCC_APB2ENSETR, 3),
	STM32_TIM(TIM17_K, "tim17_k", "ck2_tim", RCC_APB2ENSETR, 4),

	/* Peripheral clocks */
	PCLK(TIM2, "tim2", "pclk1", CLK_IGNORE_UNUSED, G_TIM2),
	PCLK(TIM3, "tim3", "pclk1", CLK_IGNORE_UNUSED, G_TIM3),
	PCLK(TIM4, "tim4", "pclk1", CLK_IGNORE_UNUSED, G_TIM4),
	PCLK(TIM5, "tim5", "pclk1", CLK_IGNORE_UNUSED, G_TIM5),
	PCLK(TIM6, "tim6", "pclk1", CLK_IGNORE_UNUSED, G_TIM6),
	PCLK(TIM7, "tim7", "pclk1", CLK_IGNORE_UNUSED, G_TIM7),
	PCLK(TIM12, "tim12", "pclk1", CLK_IGNORE_UNUSED, G_TIM12),
	PCLK(TIM13, "tim13", "pclk1", CLK_IGNORE_UNUSED, G_TIM13),
	PCLK(TIM14, "tim14", "pclk1", CLK_IGNORE_UNUSED, G_TIM14),
	PCLK(LPTIM1, "lptim1", "pclk1", 0, G_LPTIM1),
	PCLK(SPI2, "spi2", "pclk1", 0, G_SPI2),
	PCLK(SPI3, "spi3", "pclk1", 0, G_SPI3),
	PCLK(USART2, "usart2", "pclk1", 0, G_USART2),
	PCLK(USART3, "usart3", "pclk1", 0, G_USART3),
	PCLK(UART4, "uart4", "pclk1", 0, G_UART4),
	PCLK(UART5, "uart5", "pclk1", 0, G_UART5),
	PCLK(UART7, "uart7", "pclk1", 0, G_UART7),
	PCLK(UART8, "uart8", "pclk1", 0, G_UART8),
	PCLK(I2C1, "i2c1", "pclk1", 0, G_I2C1),
	PCLK(I2C2, "i2c2", "pclk1", 0, G_I2C2),
	PCLK(I2C3, "i2c3", "pclk1", 0, G_I2C3),
	PCLK(I2C5, "i2c5", "pclk1", 0, G_I2C5),
	PCLK(SPDIF, "spdif", "pclk1", 0, G_SPDIF),
	PCLK(CEC, "cec", "pclk1", 0, G_CEC),
	PCLK(DAC12, "dac12", "pclk1", 0, G_DAC12),
	PCLK(MDIO, "mdio", "pclk1", 0, G_MDIO),
	PCLK(TIM1, "tim1", "pclk2", CLK_IGNORE_UNUSED, G_TIM1),
	PCLK(TIM8, "tim8", "pclk2", CLK_IGNORE_UNUSED, G_TIM8),
	PCLK(TIM15, "tim15", "pclk2", CLK_IGNORE_UNUSED, G_TIM15),
	PCLK(TIM16, "tim16", "pclk2", CLK_IGNORE_UNUSED, G_TIM16),
	PCLK(TIM17, "tim17", "pclk2", CLK_IGNORE_UNUSED, G_TIM17),
	PCLK(SPI1, "spi1", "pclk2", 0, G_SPI1),
	PCLK(SPI4, "spi4", "pclk2", 0, G_SPI4),
	PCLK(SPI5, "spi5", "pclk2", 0, G_SPI5),
	PCLK(USART6, "usart6", "pclk2", 0, G_USART6),
	PCLK(SAI1, "sai1", "pclk2", 0, G_SAI1),
	PCLK(SAI2, "sai2", "pclk2", 0, G_SAI2),
	PCLK(SAI3, "sai3", "pclk2", 0, G_SAI3),
	PCLK(DFSDM, "dfsdm", "pclk2", 0, G_DFSDM),
	PCLK(FDCAN, "fdcan", "pclk2", 0, G_FDCAN),
	PCLK(LPTIM2, "lptim2", "pclk3", 0, G_LPTIM2),
	PCLK(LPTIM3, "lptim3", "pclk3", 0, G_LPTIM3),
	PCLK(LPTIM4, "lptim4", "pclk3", 0, G_LPTIM4),
	PCLK(LPTIM5, "lptim5", "pclk3", 0, G_LPTIM5),
	PCLK(SAI4, "sai4", "pclk3", 0, G_SAI4),
	PCLK(SYSCFG, "syscfg", "pclk3", 0, G_SYSCFG),
	PCLK(VREF, "vref", "pclk3", 13, G_VREF),
	PCLK(TMPSENS, "tmpsens", "pclk3", 0, G_TMPSENS),
	PCLK(PMBCTRL, "pmbctrl", "pclk3", 0, G_PMBCTRL),
	PCLK(HDP, "hdp", "pclk3", 0, G_HDP),
	PCLK(LTDC, "ltdc", "pclk4", 0, G_LTDC),
	PCLK(DSI, "dsi", "pclk4", 0, G_DSI),
	PCLK(IWDG2, "iwdg2", "pclk4", 0, G_IWDG2),
	PCLK(USBPHY, "usbphy", "pclk4", 0, G_USBPHY),
	PCLK(STGENRO, "stgenro", "pclk4", 0, G_STGENRO),
	PCLK(SPI6, "spi6", "pclk5", 0, G_SPI6),
	PCLK(I2C4, "i2c4", "pclk5", 0, G_I2C4),
	PCLK(I2C6, "i2c6", "pclk5", 0, G_I2C6),
	PCLK(USART1, "usart1", "pclk5", 0, G_USART1),
	PCLK(RTCAPB, "rtcapb", "pclk5", CLK_IS_CRITICAL, G_RTCAPB),
	PCLK(TZC1, "tzc1", "ck_axi", CLK_IGNORE_UNUSED, G_TZC1),
	PCLK(TZC2, "tzc2", "ck_axi", CLK_IGNORE_UNUSED, G_TZC2),
	PCLK(TZPC, "tzpc", "pclk5", CLK_IGNORE_UNUSED, G_TZPC),
	PCLK(IWDG1, "iwdg1", "pclk5", 0, G_IWDG1),
	PCLK(BSEC, "bsec", "pclk5", CLK_IGNORE_UNUSED, G_BSEC),
	PCLK(STGEN, "stgen", "pclk5", CLK_IGNORE_UNUSED, G_STGEN),
	PCLK(DMA1, "dma1", "ck_mcu", 0, G_DMA1),
	PCLK(DMA2, "dma2", "ck_mcu",  0, G_DMA2),
	PCLK(DMAMUX, "dmamux", "ck_mcu", 0, G_DMAMUX),
	PCLK(ADC12, "adc12", "ck_mcu", 0, G_ADC12),
	PCLK(USBO, "usbo", "ck_mcu", 0, G_USBO),
	PCLK(SDMMC3, "sdmmc3", "ck_mcu", 0, G_SDMMC3),
	PCLK(DCMI, "dcmi", "ck_mcu", 0, G_DCMI),
	PCLK(CRYP2, "cryp2", "ck_mcu", 0, G_CRYP2),
	PCLK(HASH2, "hash2", "ck_mcu", 0, G_HASH2),
	PCLK(RNG2, "rng2", "ck_mcu", 0, G_RNG2),
	PCLK(CRC2, "crc2", "ck_mcu", 0, G_CRC2),
	PCLK(HSEM, "hsem", "ck_mcu", 0, G_HSEM),
	PCLK(IPCC, "ipcc", "ck_mcu", 0, G_IPCC),
	PCLK(GPIOA, "gpioa", "ck_mcu", 0, G_GPIOA),
	PCLK(GPIOB, "gpiob", "ck_mcu", 0, G_GPIOB),
	PCLK(GPIOC, "gpioc", "ck_mcu", 0, G_GPIOC),
	PCLK(GPIOD, "gpiod", "ck_mcu", 0, G_GPIOD),
	PCLK(GPIOE, "gpioe", "ck_mcu", 0, G_GPIOE),
	PCLK(GPIOF, "gpiof", "ck_mcu", 0, G_GPIOF),
	PCLK(GPIOG, "gpiog", "ck_mcu", 0, G_GPIOG),
	PCLK(GPIOH, "gpioh", "ck_mcu", 0, G_GPIOH),
	PCLK(GPIOI, "gpioi", "ck_mcu", 0, G_GPIOI),
	PCLK(GPIOJ, "gpioj", "ck_mcu", 0, G_GPIOJ),
	PCLK(GPIOK, "gpiok", "ck_mcu", 0, G_GPIOK),
	PCLK(GPIOZ, "gpioz", "ck_axi", CLK_IGNORE_UNUSED, G_GPIOZ),
	PCLK(CRYP1, "cryp1", "ck_axi", CLK_IGNORE_UNUSED, G_CRYP1),
	PCLK(HASH1, "hash1", "ck_axi", CLK_IGNORE_UNUSED, G_HASH1),
	PCLK(RNG1, "rng1", "ck_axi", 0, G_RNG1),
	PCLK(BKPSRAM, "bkpsram", "ck_axi", CLK_IGNORE_UNUSED, G_BKPSRAM),
	PCLK(MDMA, "mdma", "ck_axi", 0, G_MDMA),
	PCLK(GPU, "gpu", "ck_axi", 0, G_GPU),
	PCLK(ETHTX, "ethtx", "ck_axi", 0, G_ETHTX),
	PCLK(ETHRX, "ethrx", "ck_axi", 0, G_ETHRX),
	PCLK(ETHMAC, "ethmac", "ck_axi", 0, G_ETHMAC),
	PCLK(CRC1, "crc1", "ck_axi", 0, G_CRC1),
	PCLK(USBH, "usbh", "ck_axi", 0, G_USBH),
	PCLK(ETHSTP, "ethstp", "ck_axi", 0, G_ETHSTP),
	PCLK(DDRPERFM, "ddrperfm", "pclk4", 0, G_DDRPERFM),

	/* Kernel clocks */
	KCLK(SDMMC1_K, "sdmmc1_k", sdmmc12_src, 0, G_SDMMC1, M_SDMMC12),
	KCLK(SDMMC2_K, "sdmmc2_k", sdmmc12_src, 0, G_SDMMC2, M_SDMMC12),
	KCLK(SDMMC3_K, "sdmmc3_k", sdmmc3_src, 0, G_SDMMC3, M_SDMMC3),
	KCLK(FMC_K, "fmc_k", fmc_src, 0, G_FMC, M_FMC),
	KCLK(QSPI_K, "qspi_k", qspi_src, 0, G_QSPI, M_QSPI),
	KCLK(RNG1_K, "rng1_k", rng_src, 0, G_RNG1, M_RNG1),
	KCLK(RNG2_K, "rng2_k", rng_src, 0, G_RNG2, M_RNG2),
	KCLK(USBPHY_K, "usbphy_k", usbphy_src, 0, G_USBPHY, M_USBPHY),
	KCLK(STGEN_K, "stgen_k", stgen_src, CLK_IS_CRITICAL, G_STGEN, M_STGEN),
	KCLK(SPDIF_K, "spdif_k", spdif_src, 0, G_SPDIF, M_SPDIF),
	KCLK(SPI1_K, "spi1_k", spi123_src, 0, G_SPI1, M_SPI1),
	KCLK(SPI2_K, "spi2_k", spi123_src, 0, G_SPI2, M_SPI23),
	KCLK(SPI3_K, "spi3_k", spi123_src, 0, G_SPI3, M_SPI23),
	KCLK(SPI4_K, "spi4_k", spi45_src, 0, G_SPI4, M_SPI45),
	KCLK(SPI5_K, "spi5_k", spi45_src, 0, G_SPI5, M_SPI45),
	KCLK(SPI6_K, "spi6_k", spi6_src, 0, G_SPI6, M_SPI6),
	KCLK(CEC_K, "cec_k", cec_src, 0, G_CEC, M_CEC),
	KCLK(I2C1_K, "i2c1_k", i2c12_src, 0, G_I2C1, M_I2C12),
	KCLK(I2C2_K, "i2c2_k", i2c12_src, 0, G_I2C2, M_I2C12),
	KCLK(I2C3_K, "i2c3_k", i2c35_src, 0, G_I2C3, M_I2C35),
	KCLK(I2C5_K, "i2c5_k", i2c35_src, 0, G_I2C5, M_I2C35),
	KCLK(I2C4_K, "i2c4_k", i2c46_src, 0, G_I2C4, M_I2C46),
	KCLK(I2C6_K, "i2c6_k", i2c46_src, 0, G_I2C6, M_I2C46),
	KCLK(LPTIM1_K, "lptim1_k", lptim1_src, 0, G_LPTIM1, M_LPTIM1),
	KCLK(LPTIM2_K, "lptim2_k", lptim23_src, 0, G_LPTIM2, M_LPTIM23),
	KCLK(LPTIM3_K, "lptim3_k", lptim23_src, 0, G_LPTIM3, M_LPTIM23),
	KCLK(LPTIM4_K, "lptim4_k", lptim45_src, 0, G_LPTIM4, M_LPTIM45),
	KCLK(LPTIM5_K, "lptim5_k", lptim45_src, 0, G_LPTIM5, M_LPTIM45),
	KCLK(USART1_K, "usart1_k", usart1_src, 0, G_USART1, M_USART1),
	KCLK(USART2_K, "usart2_k", usart234578_src, 0, G_USART2, M_UART24),
	KCLK(USART3_K, "usart3_k", usart234578_src, 0, G_USART3, M_UART35),
	KCLK(UART4_K, "uart4_k", usart234578_src, 0, G_UART4, M_UART24),
	KCLK(UART5_K, "uart5_k", usart234578_src, 0, G_UART5, M_UART35),
	KCLK(USART6_K, "uart6_k", usart6_src, 0, G_USART6, M_USART6),
	KCLK(UART7_K, "uart7_k", usart234578_src, 0, G_UART7, M_UART78),
	KCLK(UART8_K, "uart8_k", usart234578_src, 0, G_UART8, M_UART78),
	KCLK(FDCAN_K, "fdcan_k", fdcan_src, 0, G_FDCAN, M_FDCAN),
	KCLK(SAI1_K, "sai1_k", sai_src, 0, G_SAI1, M_SAI1),
	KCLK(SAI2_K, "sai2_k", sai2_src, 0, G_SAI2, M_SAI2),
	KCLK(SAI3_K, "sai3_k", sai_src, 0, G_SAI3, M_SAI3),
	KCLK(SAI4_K, "sai4_k", sai_src, 0, G_SAI4, M_SAI4),
	KCLK(ADC12_K, "adc12_k", adc12_src, 0, G_ADC12, M_ADC12),
	KCLK(DSI_K, "dsi_k", dsi_src, 0, G_DSI, M_DSI),
	KCLK(ADFSDM_K, "adfsdm_k", sai_src, 0, G_ADFSDM, M_SAI1),
	KCLK(USBO_K, "usbo_k", usbo_src, 0, G_USBO, M_USBO),
	KCLK(ETHCK_K, "ethck_k", eth_src, 0, G_ETHCK, M_ETHCK),

	/* Particulary Kernel Clocks (no mux or no gate) */
	MGATE_MP1(DFSDM_K, "dfsdm_k", "ck_mcu", 0, G_DFSDM),
	MGATE_MP1(DSI_PX, "dsi_px", "pll4_q", CLK_SET_RATE_PARENT, G_DSI),
	MGATE_MP1(LTDC_PX, "ltdc_px", "pll4_q", CLK_SET_RATE_PARENT, G_LTDC),
	MGATE_MP1(GPU_K, "gpu_k", "pll2_q", 0, G_GPU),
	MGATE_MP1(DAC12_K, "dac12_k", "ck_lsi", 0, G_DAC12),

	COMPOSITE(ETHPTP_K, "ethptp_k", eth_src, CLK_OPS_PARENT_ENABLE |
		  CLK_SET_RATE_NO_REPARENT,
		  _NO_GATE,
		  _MMUX(M_ETHCK),
		  _DIV(RCC_ETHCKSELR, 4, 4, 0, NULL)),

	/* RTC clock */
	COMPOSITE(RTC, "ck_rtc", rtc_src, CLK_OPS_PARENT_ENABLE,
		  _GATE(RCC_BDCR, 20, 0),
		  _MUX(RCC_BDCR, 16, 2, 0),
		  _DIV_RTC(RCC_RTCDIVR, 0, 6, 0, NULL)),

	/* MCO clocks */
	COMPOSITE(CK_MCO1, "ck_mco1", mco1_src, CLK_OPS_PARENT_ENABLE |
		  CLK_SET_RATE_NO_REPARENT,
		  _GATE(RCC_MCO1CFGR, 12, 0),
		  _MUX(RCC_MCO1CFGR, 0, 3, 0),
		  _DIV(RCC_MCO1CFGR, 4, 4, 0, NULL)),

	COMPOSITE(CK_MCO2, "ck_mco2", mco2_src, CLK_OPS_PARENT_ENABLE |
		  CLK_SET_RATE_NO_REPARENT,
		  _GATE(RCC_MCO2CFGR, 12, 0),
		  _MUX(RCC_MCO2CFGR, 0, 3, 0),
		  _DIV(RCC_MCO2CFGR, 4, 4, 0, NULL)),

	/* Debug clocks */
	GATE(CK_DBG, "ck_sys_dbg", "ck_axi", CLK_IGNORE_UNUSED,
	     RCC_DBGCFGR, 8, 0),

	COMPOSITE(CK_TRACE, "ck_trace", ck_trace_src,
		  CLK_OPS_PARENT_ENABLE | CLK_IGNORE_UNUSED,
		  _GATE(RCC_DBGCFGR, 9, 0),
		  _NO_MUX,
		  _DIV(RCC_DBGCFGR, 0, 3, 0, ck_trace_div_table)),
};

static const u32 stm32mp1_clock_secured[] = {
	CK_HSE,
	CK_HSI,
	CK_CSI,
	CK_LSI,
	CK_LSE,
	PLL1,
	PLL2,
	PLL1_P,
	PLL2_P,
	PLL2_Q,
	PLL2_R,
	CK_MPU,
	CK_AXI,
	SPI6,
	I2C4,
	I2C6,
	USART1,
	RTCAPB,
	TZC1,
	TZC2,
	TZPC,
	IWDG1,
	BSEC,
	STGEN,
	GPIOZ,
	CRYP1,
	HASH1,
	RNG1,
	BKPSRAM,
	RNG1_K,
	STGEN_K,
	SPI6_K,
	I2C4_K,
	I2C6_K,
	USART1_K,
	RTC,
};

static bool stm32_check_security(const struct clock_config *cfg)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(stm32mp1_clock_secured); i++)
		if (cfg->id == stm32mp1_clock_secured[i])
			return true;
	return false;
}

struct stm32_rcc_match_data {
	const struct clock_config *cfg;
	unsigned int num;
	unsigned int maxbinding;
	u32 clear_offset;
	bool (*check_security)(const struct clock_config *cfg);
};

static struct stm32_rcc_match_data stm32mp1_data = {
	.cfg		= stm32mp1_clock_cfg,
	.num		= ARRAY_SIZE(stm32mp1_clock_cfg),
	.maxbinding	= STM32MP1_LAST_CLK,
	.clear_offset	= RCC_CLR,
};

static struct stm32_rcc_match_data stm32mp1_data_secure = {
	.cfg		= stm32mp1_clock_cfg,
	.num		= ARRAY_SIZE(stm32mp1_clock_cfg),
	.maxbinding	= STM32MP1_LAST_CLK,
	.clear_offset	= RCC_CLR,
	.check_security = &stm32_check_security
};

static const struct of_device_id stm32mp1_match_data[] = {
	{
		.compatible = "st,stm32mp1-rcc",
		.data = &stm32mp1_data,
	},
	{
		.compatible = "st,stm32mp1-rcc-secure",
		.data = &stm32mp1_data_secure,
	},
	{ }
};
MODULE_DEVICE_TABLE(of, stm32mp1_match_data);

static int stm32_register_hw_clk(struct device *dev,
				 struct clk_hw_onecell_data *clk_data,
				 void __iomem *base, spinlock_t *lock,
				 const struct clock_config *cfg)
{
	struct clk_hw **hws;
	struct clk_hw *hw = ERR_PTR(-ENOENT);

	hws = clk_data->hws;

	if (cfg->func)
		hw = (*cfg->func)(dev, clk_data, base, lock, cfg);

	if (IS_ERR(hw)) {
		pr_err("Unable to register %s\n", cfg->name);
		return  PTR_ERR(hw);
	}

	if (cfg->id != NO_ID)
		hws[cfg->id] = hw;

	return 0;
}

#define STM32_RESET_ID_MASK GENMASK(15, 0)

struct stm32_reset_data {
	/* reset lock */
	spinlock_t			lock;
	struct reset_controller_dev	rcdev;
	void __iomem			*membase;
	u32				clear_offset;
};

static inline struct stm32_reset_data *
to_stm32_reset_data(struct reset_controller_dev *rcdev)
{
	return container_of(rcdev, struct stm32_reset_data, rcdev);
}

static int stm32_reset_update(struct reset_controller_dev *rcdev,
			      unsigned long id, bool assert)
{
	struct stm32_reset_data *data = to_stm32_reset_data(rcdev);
	int reg_width = sizeof(u32);
	int bank = id / (reg_width * BITS_PER_BYTE);
	int offset = id % (reg_width * BITS_PER_BYTE);

	if (data->clear_offset) {
		void __iomem *addr;

		addr = data->membase + (bank * reg_width);
		if (!assert)
			addr += data->clear_offset;

		writel(BIT(offset), addr);

	} else {
		unsigned long flags;
		u32 reg;

		spin_lock_irqsave(&data->lock, flags);

		reg = readl(data->membase + (bank * reg_width));

		if (assert)
			reg |= BIT(offset);
		else
			reg &= ~BIT(offset);

		writel(reg, data->membase + (bank * reg_width));

		spin_unlock_irqrestore(&data->lock, flags);
	}

	return 0;
}

static int stm32_reset_assert(struct reset_controller_dev *rcdev,
			      unsigned long id)
{
	return stm32_reset_update(rcdev, id, true);
}

static int stm32_reset_deassert(struct reset_controller_dev *rcdev,
				unsigned long id)
{
	return stm32_reset_update(rcdev, id, false);
}

static int stm32_reset_status(struct reset_controller_dev *rcdev,
			      unsigned long id)
{
	struct stm32_reset_data *data = to_stm32_reset_data(rcdev);
	int reg_width = sizeof(u32);
	int bank = id / (reg_width * BITS_PER_BYTE);
	int offset = id % (reg_width * BITS_PER_BYTE);
	u32 reg;

	reg = readl(data->membase + (bank * reg_width));

	return !!(reg & BIT(offset));
}

static const struct reset_control_ops stm32_reset_ops = {
	.assert		= stm32_reset_assert,
	.deassert	= stm32_reset_deassert,
	.status		= stm32_reset_status,
};

static int stm32_rcc_reset_init(struct device *dev, void __iomem *base,
				const struct of_device_id *match)
{
	const struct stm32_rcc_match_data *data = match->data;
	struct stm32_reset_data *reset_data = NULL;

	data = match->data;

	reset_data = kzalloc(sizeof(*reset_data), GFP_KERNEL);
	if (!reset_data)
		return -ENOMEM;

	spin_lock_init(&reset_data->lock);
	reset_data->membase = base;
	reset_data->rcdev.owner = THIS_MODULE;
	reset_data->rcdev.ops = &stm32_reset_ops;
	reset_data->rcdev.of_node = dev_of_node(dev);
	reset_data->rcdev.nr_resets = STM32_RESET_ID_MASK;
	reset_data->clear_offset = data->clear_offset;

	return reset_controller_register(&reset_data->rcdev);
}

static int stm32_rcc_clock_init(struct device *dev, void __iomem *base,
				const struct of_device_id *match)
{
	const struct stm32_rcc_match_data *data = match->data;
	struct clk_hw_onecell_data *clk_data;
	struct clk_hw **hws;
	int err, n, max_binding;

	max_binding =  data->maxbinding;

	clk_data = devm_kzalloc(dev, struct_size(clk_data, hws, max_binding),
				GFP_KERNEL);
	if (!clk_data)
		return -ENOMEM;

	clk_data->num = max_binding;

	hws = clk_data->hws;

	for (n = 0; n < max_binding; n++)
		hws[n] = ERR_PTR(-ENOENT);

	for (n = 0; n < data->num; n++) {
		if (data->check_security && data->check_security(&data->cfg[n]))
			continue;

		err = stm32_register_hw_clk(dev, clk_data, base, &rlock,
					    &data->cfg[n]);
		if (err) {
			dev_err(dev, "Can't register clk %s: %d\n",
				data->cfg[n].name, err);

			return err;
		}
	}

	return of_clk_add_hw_provider(dev_of_node(dev), of_clk_hw_onecell_get, clk_data);
}

static int stm32_rcc_init(struct device *dev, void __iomem *base,
			  const struct of_device_id *match_data)
{
	const struct of_device_id *match;
	int err;

	match = of_match_node(match_data, dev_of_node(dev));
	if (!match) {
		dev_err(dev, "match data not found\n");
		return -ENODEV;
	}

	/* RCC Reset Configuration */
	err = stm32_rcc_reset_init(dev, base, match);
	if (err) {
		pr_err("stm32mp1 reset failed to initialize\n");
		return err;
	}

	/* RCC Clock Configuration */
	err = stm32_rcc_clock_init(dev, base, match);
	if (err) {
		pr_err("stm32mp1 clock failed to initialize\n");
		return err;
	}

	return 0;
}

static void stm32_clk_summary_debugfs_create(struct device *dev, void __iomem *base);

static int stm32mp1_rcc_init(struct device *dev)
{
	void __iomem *base;
	int ret;

	base = of_iomap(dev_of_node(dev), 0);
	if (!base) {
		pr_err("%pOFn: unable to map resource", dev_of_node(dev));
		ret = -ENOMEM;
		goto out;
	}

	ret = stm32_rcc_init(dev, base, stm32mp1_match_data);

out:
	if (ret) {
		if (base)
			iounmap(base);

		of_node_put(dev_of_node(dev));
	} else {
		stm32_clk_summary_debugfs_create(dev, base);
	}

	return ret;
}

static int get_clock_deps(struct device *dev)
{
	static const char * const clock_deps_name[] = {
		"hsi", "hse", "csi", "lsi", "lse",
	};
	size_t deps_size = sizeof(struct clk *) * ARRAY_SIZE(clock_deps_name);
	struct clk **clk_deps;
	int i;

	clk_deps = devm_kzalloc(dev, deps_size, GFP_KERNEL);
	if (!clk_deps)
		return -ENOMEM;

	for (i = 0; i < ARRAY_SIZE(clock_deps_name); i++) {
		struct clk *clk = of_clk_get_by_name(dev_of_node(dev),
						     clock_deps_name[i]);

		if (IS_ERR(clk)) {
			if (PTR_ERR(clk) != -EINVAL && PTR_ERR(clk) != -ENOENT)
				return PTR_ERR(clk);
		} else {
			/* Device gets a reference count on the clock */
			clk_deps[i] = devm_clk_get(dev, __clk_get_name(clk));
			clk_put(clk);
		}
	}

	return 0;
}

static int stm32mp1_rcc_clocks_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	int ret = get_clock_deps(dev);

	if (!ret)
		ret = stm32mp1_rcc_init(dev);

	return ret;
}

static int stm32mp1_rcc_clocks_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *child, *np = dev_of_node(dev);

	for_each_available_child_of_node(np, child)
		of_clk_del_provider(child);

	return 0;
}

static struct platform_driver stm32mp1_rcc_clocks_driver = {
	.driver	= {
		.name = "stm32mp1_rcc",
		.of_match_table = stm32mp1_match_data,
	},
	.probe = stm32mp1_rcc_clocks_probe,
	.remove = stm32mp1_rcc_clocks_remove,
};

static int __init stm32mp1_clocks_init(void)
{
	return platform_driver_register(&stm32mp1_rcc_clocks_driver);
}
core_initcall(stm32mp1_clocks_init);

#ifdef CONFIG_DEBUG_FS

#include <linux/debugfs.h>

#define NO_STM32_MUX		0xFFFF
#define NO_STM32_DIV		0xFFFF
#define NO_STM32_GATE		0xFFFF

enum enum_gate_cfg {
	GATE_HSI,
	GATE_CSI,
	GATE_LSI,
	GATE_HSE,
	GATE_LSE,
	GATE_PLL1,
	GATE_PLL2,
	GATE_PLL3,
	GATE_PLL4,
	GATE_PLL1_DIVP,
	GATE_PLL1_DIVQ,
	GATE_PLL1_DIVR,
	GATE_PLL2_DIVP,
	GATE_PLL2_DIVQ,
	GATE_PLL2_DIVR,
	GATE_PLL3_DIVP,
	GATE_PLL3_DIVQ,
	GATE_PLL3_DIVR,
	GATE_PLL4_DIVP,
	GATE_PLL4_DIVQ,
	GATE_PLL4_DIVR,
	GATE_RTCCK,
	GATE_MCO1,
	GATE_MCO2,
	GATE_DBGCK,
	GATE_TRACECK,
	GATE_SAI1,
	GATE_SAI2,
	GATE_SAI3,
	GATE_SAI4,
	GATE_SPI1,
	GATE_SPI2,
	GATE_SPI3,
	GATE_SPI4,
	GATE_SPI5,
	GATE_SPI6,
	GATE_SPDIF,
	GATE_I2C1,
	GATE_I2C2,
	GATE_I2C3,
	GATE_I2C4,
	GATE_I2C5,
	GATE_I2C6,
	GATE_USART2,
	GATE_UART4,
	GATE_USART3,
	GATE_UART5,
	GATE_USART1,
	GATE_USART6,
	GATE_UART7,
	GATE_UART8,
	GATE_LPTIM1,
	GATE_LPTIM2,
	GATE_LPTIM3,
	GATE_LPTIM4,
	GATE_LPTIM5,
	GATE_LTDC,
	GATE_DSI,
	GATE_QSPI,
	GATE_FMC,
	GATE_SDMMC1,
	GATE_SDMMC2,
	GATE_SDMMC3,
	GATE_USBO,
	GATE_USBPHY,
	GATE_RNG1,
	GATE_RNG2,
	GATE_FDCAN,
	GATE_DAC12,
	GATE_CEC,
	GATE_ADC12,
	GATE_GPU,
	GATE_STGEN,
	GATE_DFSDM,
	GATE_ADFSDM,
	GATE_TIM2,
	GATE_TIM3,
	GATE_TIM4,
	GATE_TIM5,
	GATE_TIM6,
	GATE_TIM7,
	GATE_TIM12,
	GATE_TIM13,
	GATE_TIM14,
	GATE_MDIO,
	GATE_TIM1,
	GATE_TIM8,
	GATE_TIM15,
	GATE_TIM16,
	GATE_TIM17,
	GATE_SYSCFG,
	GATE_VREF,
	GATE_TMPSENS,
	GATE_PMBCTRL,
	GATE_HDP,
	GATE_IWDG2,
	GATE_STGENRO,
	GATE_DMA1,
	GATE_DMA2,
	GATE_DMAMUX,
	GATE_DCMI,
	GATE_CRYP2,
	GATE_HASH2,
	GATE_CRC2,
	GATE_HSEM,
	GATE_IPCC,
	GATE_GPIOA,
	GATE_GPIOB,
	GATE_GPIOC,
	GATE_GPIOD,
	GATE_GPIOE,
	GATE_GPIOF,
	GATE_GPIOG,
	GATE_GPIOH,
	GATE_GPIOI,
	GATE_GPIOJ,
	GATE_GPIOK,
	GATE_MDMA,
	GATE_ETHCK,
	GATE_ETHTX,
	GATE_ETHRX,
	GATE_ETHMAC,
	GATE_CRC1,
	GATE_USBH,
	GATE_ETHSTP,
	GATE_RTCAPB,
	GATE_TZC1,
	GATE_TZC2,
	GATE_TZPC,
	GATE_IWDG1,
	GATE_BSEC,
	GATE_GPIOZ,
	GATE_CRYP1,
	GATE_HASH1,
	GATE_BKPSRAM,
	GATE_DDRPERFM,

	GATE_NB
};

struct cs_gate_cfg {
	u16	offset;
	u8	bit_idx;
};

#define CFG_GATE(_id, _offset, _bit_idx)\
	[(_id)] = {\
		.offset = (_offset),\
		.bit_idx = (_bit_idx),\
	}

static struct cs_gate_cfg stm32mp15_gates[] = {
	CFG_GATE(GATE_HSI,		RCC_OCENSETR,	0),
	CFG_GATE(GATE_CSI,		RCC_OCENSETR,	4),
	CFG_GATE(GATE_LSI,		RCC_RDLSICR,	0),
	CFG_GATE(GATE_HSE,		RCC_OCENSETR,	8),
	CFG_GATE(GATE_LSE,		RCC_BDCR,	0),
	CFG_GATE(GATE_RTCCK,		RCC_BDCR,	20),
	CFG_GATE(GATE_PLL1,		RCC_PLL1CR,	0),
	CFG_GATE(GATE_PLL1_DIVP,	RCC_PLL1CR,	4),
	CFG_GATE(GATE_PLL1_DIVQ,	RCC_PLL1CR,	5),
	CFG_GATE(GATE_PLL1_DIVR,	RCC_PLL1CR,	6),
	CFG_GATE(GATE_PLL2,		RCC_PLL2CR,	0),
	CFG_GATE(GATE_PLL2_DIVP,	RCC_PLL2CR,	4),
	CFG_GATE(GATE_PLL2_DIVQ,	RCC_PLL2CR,	5),
	CFG_GATE(GATE_PLL2_DIVR,	RCC_PLL2CR,	6),
	CFG_GATE(GATE_PLL3,		RCC_PLL3CR,	0),
	CFG_GATE(GATE_PLL3_DIVP,	RCC_PLL3CR,	4),
	CFG_GATE(GATE_PLL3_DIVQ,	RCC_PLL3CR,	5),
	CFG_GATE(GATE_PLL3_DIVR,	RCC_PLL3CR,	6),
	CFG_GATE(GATE_PLL4,		RCC_PLL4CR,	0),
	CFG_GATE(GATE_PLL4_DIVP,	RCC_PLL4CR,	4),
	CFG_GATE(GATE_PLL4_DIVQ,	RCC_PLL4CR,	5),
	CFG_GATE(GATE_PLL4_DIVR,	RCC_PLL4CR,	6),
	CFG_GATE(GATE_MCO1,		RCC_MCO1CFGR,	12),
	CFG_GATE(GATE_MCO2,		RCC_MCO2CFGR,	12),
	CFG_GATE(GATE_DBGCK,		RCC_DBGCFGR,	8),
	CFG_GATE(GATE_TRACECK,		RCC_DBGCFGR,	9),
	CFG_GATE(GATE_MDIO,		RCC_APB1ENSETR, 31),
	CFG_GATE(GATE_DAC12,		RCC_APB1ENSETR, 29),
	CFG_GATE(GATE_CEC,		RCC_APB1ENSETR, 27),
	CFG_GATE(GATE_SPDIF,		RCC_APB1ENSETR, 26),
	CFG_GATE(GATE_I2C5,		RCC_APB1ENSETR, 24),
	CFG_GATE(GATE_I2C3,		RCC_APB1ENSETR, 23),
	CFG_GATE(GATE_I2C2,		RCC_APB1ENSETR, 22),
	CFG_GATE(GATE_I2C1,		RCC_APB1ENSETR, 21),
	CFG_GATE(GATE_UART8,		RCC_APB1ENSETR, 19),
	CFG_GATE(GATE_UART7,		RCC_APB1ENSETR, 18),
	CFG_GATE(GATE_UART5,		RCC_APB1ENSETR, 17),
	CFG_GATE(GATE_UART4,		RCC_APB1ENSETR, 16),
	CFG_GATE(GATE_USART3,		RCC_APB1ENSETR, 15),
	CFG_GATE(GATE_USART2,		RCC_APB1ENSETR, 14),
	CFG_GATE(GATE_SPI3,		RCC_APB1ENSETR, 12),
	CFG_GATE(GATE_SPI2,		RCC_APB1ENSETR, 11),
	CFG_GATE(GATE_LPTIM1,		RCC_APB1ENSETR, 9),
	CFG_GATE(GATE_TIM14,		RCC_APB1ENSETR, 8),
	CFG_GATE(GATE_TIM13,		RCC_APB1ENSETR, 7),
	CFG_GATE(GATE_TIM12,		RCC_APB1ENSETR, 6),
	CFG_GATE(GATE_TIM7,		RCC_APB1ENSETR, 5),
	CFG_GATE(GATE_TIM6,		RCC_APB1ENSETR, 4),
	CFG_GATE(GATE_TIM5,		RCC_APB1ENSETR, 3),
	CFG_GATE(GATE_TIM4,		RCC_APB1ENSETR, 2),
	CFG_GATE(GATE_TIM3,		RCC_APB1ENSETR, 1),
	CFG_GATE(GATE_TIM2,		RCC_APB1ENSETR, 0),
	CFG_GATE(GATE_FDCAN,		RCC_APB2ENSETR, 24),
	CFG_GATE(GATE_ADFSDM,		RCC_APB2ENSETR, 21),
	CFG_GATE(GATE_DFSDM,		RCC_APB2ENSETR, 20),
	CFG_GATE(GATE_SAI3,		RCC_APB2ENSETR, 18),
	CFG_GATE(GATE_SAI2,		RCC_APB2ENSETR, 17),
	CFG_GATE(GATE_SAI1,		RCC_APB2ENSETR, 16),
	CFG_GATE(GATE_USART6,		RCC_APB2ENSETR, 13),
	CFG_GATE(GATE_SPI5,		RCC_APB2ENSETR, 10),
	CFG_GATE(GATE_SPI4,		RCC_APB2ENSETR, 9),
	CFG_GATE(GATE_SPI1,		RCC_APB2ENSETR, 8),
	CFG_GATE(GATE_TIM17,		RCC_APB2ENSETR, 4),
	CFG_GATE(GATE_TIM16,		RCC_APB2ENSETR, 3),
	CFG_GATE(GATE_TIM15,		RCC_APB2ENSETR, 2),
	CFG_GATE(GATE_TIM8,		RCC_APB2ENSETR, 1),
	CFG_GATE(GATE_TIM1,		RCC_APB2ENSETR, 0),
	CFG_GATE(GATE_HDP,		RCC_APB3ENSETR, 20),
	CFG_GATE(GATE_PMBCTRL,		RCC_APB3ENSETR, 17),
	CFG_GATE(GATE_TMPSENS,		RCC_APB3ENSETR, 16),
	CFG_GATE(GATE_VREF,		RCC_APB3ENSETR, 13),
	CFG_GATE(GATE_SYSCFG,		RCC_APB3ENSETR, 11),
	CFG_GATE(GATE_SAI4,		RCC_APB3ENSETR, 8),
	CFG_GATE(GATE_LPTIM5,		RCC_APB3ENSETR, 3),
	CFG_GATE(GATE_LPTIM4,		RCC_APB3ENSETR, 2),
	CFG_GATE(GATE_LPTIM3,		RCC_APB3ENSETR, 1),
	CFG_GATE(GATE_LPTIM2,		RCC_APB3ENSETR, 0),
	CFG_GATE(GATE_STGENRO,		RCC_APB4ENSETR, 20),
	CFG_GATE(GATE_USBPHY,		RCC_APB4ENSETR, 16),
	CFG_GATE(GATE_IWDG2,		RCC_APB4ENSETR, 15),
	CFG_GATE(GATE_DDRPERFM,		RCC_APB4ENSETR, 8),
	CFG_GATE(GATE_DSI,		RCC_APB4ENSETR, 4),
	CFG_GATE(GATE_LTDC,		RCC_APB4ENSETR, 0),
	CFG_GATE(GATE_STGEN,		RCC_APB5ENSETR, 20),
	CFG_GATE(GATE_BSEC,		RCC_APB5ENSETR, 16),
	CFG_GATE(GATE_IWDG1,		RCC_APB5ENSETR, 15),
	CFG_GATE(GATE_TZPC,		RCC_APB5ENSETR, 13),
	CFG_GATE(GATE_TZC2,		RCC_APB5ENSETR, 12),
	CFG_GATE(GATE_TZC1,		RCC_APB5ENSETR, 11),
	CFG_GATE(GATE_RTCAPB,		RCC_APB5ENSETR, 8),
	CFG_GATE(GATE_USART1,		RCC_APB5ENSETR, 4),
	CFG_GATE(GATE_I2C6,		RCC_APB5ENSETR, 3),
	CFG_GATE(GATE_I2C4,		RCC_APB5ENSETR, 2),
	CFG_GATE(GATE_SPI6,		RCC_APB5ENSETR, 0),
	CFG_GATE(GATE_SDMMC3,		RCC_AHB2ENSETR, 16),
	CFG_GATE(GATE_USBO,		RCC_AHB2ENSETR, 8),
	CFG_GATE(GATE_ADC12,		RCC_AHB2ENSETR, 5),
	CFG_GATE(GATE_DMAMUX,		RCC_AHB2ENSETR, 2),
	CFG_GATE(GATE_DMA2,		RCC_AHB2ENSETR, 1),
	CFG_GATE(GATE_DMA1,		RCC_AHB2ENSETR, 0),
	CFG_GATE(GATE_IPCC,		RCC_AHB3ENSETR, 12),
	CFG_GATE(GATE_HSEM,		RCC_AHB3ENSETR, 11),
	CFG_GATE(GATE_CRC2,		RCC_AHB3ENSETR, 7),
	CFG_GATE(GATE_RNG2,		RCC_AHB3ENSETR, 6),
	CFG_GATE(GATE_HASH2,		RCC_AHB3ENSETR, 5),
	CFG_GATE(GATE_CRYP2,		RCC_AHB3ENSETR, 4),
	CFG_GATE(GATE_DCMI,		RCC_AHB3ENSETR, 0),
	CFG_GATE(GATE_GPIOK,		RCC_AHB4ENSETR, 10),
	CFG_GATE(GATE_GPIOJ,		RCC_AHB4ENSETR, 9),
	CFG_GATE(GATE_GPIOI,		RCC_AHB4ENSETR, 8),
	CFG_GATE(GATE_GPIOH,		RCC_AHB4ENSETR, 7),
	CFG_GATE(GATE_GPIOG,		RCC_AHB4ENSETR, 6),
	CFG_GATE(GATE_GPIOF,		RCC_AHB4ENSETR, 5),
	CFG_GATE(GATE_GPIOE,		RCC_AHB4ENSETR, 4),
	CFG_GATE(GATE_GPIOD,		RCC_AHB4ENSETR, 3),
	CFG_GATE(GATE_GPIOC,		RCC_AHB4ENSETR, 2),
	CFG_GATE(GATE_GPIOB,		RCC_AHB4ENSETR, 1),
	CFG_GATE(GATE_GPIOA,		RCC_AHB4ENSETR, 0),
	CFG_GATE(GATE_BKPSRAM,		RCC_AHB5ENSETR, 8),
	CFG_GATE(GATE_RNG1,		RCC_AHB5ENSETR, 6),
	CFG_GATE(GATE_HASH1,		RCC_AHB5ENSETR, 5),
	CFG_GATE(GATE_CRYP1,		RCC_AHB5ENSETR, 4),
	CFG_GATE(GATE_GPIOZ,		RCC_AHB5ENSETR, 0),
	CFG_GATE(GATE_USBH,		RCC_AHB6ENSETR, 24),
	CFG_GATE(GATE_CRC1,		RCC_AHB6ENSETR, 20),
	CFG_GATE(GATE_SDMMC2,		RCC_AHB6ENSETR, 17),
	CFG_GATE(GATE_SDMMC1,		RCC_AHB6ENSETR, 16),
	CFG_GATE(GATE_QSPI,		RCC_AHB6ENSETR, 14),
	CFG_GATE(GATE_FMC,		RCC_AHB6ENSETR, 12),
	CFG_GATE(GATE_ETHMAC,		RCC_AHB6ENSETR, 10),
	CFG_GATE(GATE_ETHRX,		RCC_AHB6ENSETR, 9),
	CFG_GATE(GATE_ETHTX,		RCC_AHB6ENSETR, 8),
	CFG_GATE(GATE_ETHCK,		RCC_AHB6ENSETR, 7),
	CFG_GATE(GATE_GPU,		RCC_AHB6ENSETR, 5),
	CFG_GATE(GATE_MDMA,		RCC_AHB6ENSETR, 0),
	CFG_GATE(GATE_ETHSTP,		RCC_AHB6LPENSETR, 11),
};

enum enum_mux_cfg {
	MUX_MPU,
	MUX_AXI,
	MUX_MCU,
	MUX_PLL12,
	MUX_PLL3,
	MUX_PLL4,
	MUX_CKPER,
	MUX_RTC,
	MUX_SDMMC12,
	MUX_SDMMC3,
	MUX_FMC,
	MUX_QSPI,
	MUX_RNG1,
	MUX_RNG2,
	MUX_USBPHY,
	MUX_USBO,
	MUX_STGEN,
	MUX_SPDIF,
	MUX_SPI1,
	MUX_SPI23,
	MUX_SPI45,
	MUX_SPI6,
	MUX_CEC,
	MUX_I2C12,
	MUX_I2C35,
	MUX_I2C46,
	MUX_LPTIM1,
	MUX_LPTIM23,
	MUX_LPTIM45,
	MUX_USART1,
	MUX_UART24,
	MUX_UART35,
	MUX_USART6,
	MUX_UART78,
	MUX_SAI1,
	MUX_SAI2,
	MUX_SAI3,
	MUX_SAI4,
	MUX_DSI,
	MUX_FDCAN,
	MUX_ADC12,
	MUX_ETHCK,
	MUX_MCO1,
	MUX_MCO2,
};

struct cs_mux_cfg {
	u16	offset;
	u8	shift;
	u8	width;
	u8	flags;
	u32	*table;
};

#define CFG_MUX(_id, _offset, _shift, _witdh, _flags)\
	[_id] = {\
		.offset = (_offset),\
		.shift = (_shift),\
		.width = (_witdh),\
		.flags = (_flags),\
	}

static const struct cs_mux_cfg stm32mp15_muxes[] = {
	CFG_MUX(MUX_PLL12,	RCC_RCK12SELR, 0, 2, 0),
	CFG_MUX(MUX_PLL3,	RCC_RCK3SELR, 0, 2, 0),
	CFG_MUX(MUX_PLL4,	RCC_RCK4SELR, 0, 2, 0),
	CFG_MUX(MUX_CKPER,	RCC_CPERCKSELR, 0, 2, 0),
	CFG_MUX(MUX_MPU,	RCC_MPCKSELR, 0, 2, 0),
	CFG_MUX(MUX_AXI,	RCC_ASSCKSELR, 0, 3, 0),
	CFG_MUX(MUX_MCU,	RCC_MSSCKSELR, 0, 2, 0),
	CFG_MUX(MUX_RTC,	RCC_BDCR, 16, 2, 0),
	CFG_MUX(MUX_SDMMC12,	RCC_SDMMC12CKSELR, 0, 3, 0),
	CFG_MUX(MUX_SPI23,	RCC_SPI2S23CKSELR, 0, 3, 0),
	CFG_MUX(MUX_SPI45,	RCC_SPI2S45CKSELR, 0, 3, 0),
	CFG_MUX(MUX_I2C12,	RCC_I2C12CKSELR, 0, 3, 0),
	CFG_MUX(MUX_I2C35,	RCC_I2C35CKSELR, 0, 3, 0),
	CFG_MUX(MUX_LPTIM23,	RCC_LPTIM23CKSELR, 0, 3, 0),
	CFG_MUX(MUX_LPTIM45,	RCC_LPTIM45CKSELR, 0, 3, 0),
	CFG_MUX(MUX_UART24,	RCC_UART24CKSELR, 0, 3, 0),
	CFG_MUX(MUX_UART35,	RCC_UART35CKSELR, 0, 3, 0),
	CFG_MUX(MUX_UART78,	RCC_UART78CKSELR, 0, 3, 0),
	CFG_MUX(MUX_SAI1,	RCC_SAI1CKSELR, 0, 3, 0),
	CFG_MUX(MUX_ETHCK,	RCC_ETHCKSELR, 0, 2, 0),
	CFG_MUX(MUX_I2C46,	RCC_I2C46CKSELR, 0, 3, 0),
	CFG_MUX(MUX_RNG2,	RCC_RNG2CKSELR, 0, 2, 0),
	CFG_MUX(MUX_SDMMC3,	RCC_SDMMC3CKSELR, 0, 3, 0),
	CFG_MUX(MUX_FMC,	RCC_FMCCKSELR, 0, 2, 0),
	CFG_MUX(MUX_QSPI,	RCC_QSPICKSELR, 0, 2, 0),
	CFG_MUX(MUX_USBPHY,	RCC_USBCKSELR, 0, 2, 0),
	CFG_MUX(MUX_USBO,	RCC_USBCKSELR, 4, 1, 0),
	CFG_MUX(MUX_SPDIF,	RCC_SPDIFCKSELR, 0, 2, 0),
	CFG_MUX(MUX_SPI1,	RCC_SPI2S1CKSELR, 0, 3, 0),
	CFG_MUX(MUX_CEC,	RCC_CECCKSELR, 0, 2, 0),
	CFG_MUX(MUX_LPTIM1,	RCC_LPTIM1CKSELR, 0, 3, 0),
	CFG_MUX(MUX_USART6,	RCC_UART6CKSELR, 0, 3, 0),
	CFG_MUX(MUX_FDCAN,	RCC_FDCANCKSELR, 0, 2, 0),
	CFG_MUX(MUX_SAI2,	RCC_SAI2CKSELR, 0, 3, 0),
	CFG_MUX(MUX_SAI3,	RCC_SAI3CKSELR, 0, 3, 0),
	CFG_MUX(MUX_SAI4,	RCC_SAI4CKSELR, 0, 3, 0),
	CFG_MUX(MUX_ADC12,	RCC_ADCCKSELR, 0, 2, 0),
	CFG_MUX(MUX_DSI,	RCC_DSICKSELR, 0, 1, 0),
	CFG_MUX(MUX_RNG1,	RCC_RNG1CKSELR, 0, 2, 0),
	CFG_MUX(MUX_STGEN,	RCC_STGENCKSELR, 0, 2, 0),
	CFG_MUX(MUX_USART1,	RCC_UART1CKSELR, 0, 3, 0),
	CFG_MUX(MUX_SPI6,	RCC_SPI6CKSELR, 0, 3, 0),
	CFG_MUX(MUX_MCO1,	RCC_MCO1CFGR, 0, 3, 0),
	CFG_MUX(MUX_MCO2,	RCC_MCO2CFGR, 0, 3, 0),
};

enum enum_div_cfg {
	DIV_PLL1DIVP,
	DIV_PLL2DIVP,
	DIV_PLL2DIVQ,
	DIV_PLL2DIVR,
	DIV_PLL3DIVP,
	DIV_PLL3DIVQ,
	DIV_PLL3DIVR,
	DIV_PLL4DIVP,
	DIV_PLL4DIVQ,
	DIV_PLL4DIVR,
	DIV_MPU,
	DIV_AXI,
	DIV_MCU,
	DIV_APB1,
	DIV_APB2,
	DIV_APB3,
	DIV_APB4,
	DIV_APB5,
	DIV_RTC,
	DIV_HSI,
	DIV_MCO1,
	DIV_MCO2,
	DIV_TRACE,
	DIV_ETHPTP,
	DIV_NB
};

struct cs_div_cfg {
	u16	offset;
	u8	shift;
	u8	width;
	u8	flags;
	const struct clk_div_table *table;
};

#define CFG_DIV(_id, _offset, _shift, _width, _flags, _table)\
	[(_id)] = {\
		.offset = (_offset),\
		.shift = (_shift),\
		.width = (_width),\
		.flags = (_flags),\
		.table = (_table),\
	}

static const struct cs_div_cfg stm32mp15_dividers[DIV_NB] = {
	CFG_DIV(DIV_MPU,	RCC_MPCKDIVR, 0, 4, 0, NULL),
	CFG_DIV(DIV_AXI,	RCC_AXIDIVR, 0, 3, 0, axi_div_table),
	CFG_DIV(DIV_MCU,	RCC_MCUDIVR, 0, 4, 0, mcu_div_table),
	CFG_DIV(DIV_APB1,	RCC_APB1DIVR, 0, 3, 0, apb_div_table),
	CFG_DIV(DIV_APB2,	RCC_APB2DIVR, 0, 3, 0, apb_div_table),
	CFG_DIV(DIV_APB3,	RCC_APB3DIVR, 0, 3, 0, apb_div_table),
	CFG_DIV(DIV_APB4,	RCC_APB4DIVR, 0, 3, 0, apb_div_table),
	CFG_DIV(DIV_APB5,	RCC_APB5DIVR, 0, 3, 0, apb_div_table),
	CFG_DIV(DIV_HSI,	RCC_HSICFGR, 0, 2, CLK_DIVIDER_POWER_OF_TWO, NULL),
	CFG_DIV(DIV_PLL1DIVP,	RCC_PLL1CFGR2, 0, 7, 0, NULL),
	CFG_DIV(DIV_PLL2DIVP,	RCC_PLL2CFGR2, 0, 7, 0, NULL),
	CFG_DIV(DIV_PLL2DIVQ,	RCC_PLL2CFGR2, 8, 7, 0, NULL),
	CFG_DIV(DIV_PLL2DIVR,	RCC_PLL2CFGR2, 16, 7, 0, NULL),
	CFG_DIV(DIV_PLL3DIVP,	RCC_PLL3CFGR2, 0, 7, 0, NULL),
	CFG_DIV(DIV_PLL3DIVQ,	RCC_PLL3CFGR2, 8, 7, 0, NULL),
	CFG_DIV(DIV_PLL3DIVR,	RCC_PLL3CFGR2, 16, 7, 0, NULL),
	CFG_DIV(DIV_PLL4DIVP,	RCC_PLL4CFGR2, 0, 7, 0, NULL),
	CFG_DIV(DIV_PLL4DIVQ,	RCC_PLL4CFGR2, 8, 7, 0, NULL),
	CFG_DIV(DIV_PLL4DIVR,	RCC_PLL4CFGR2, 16, 7, 0, NULL),
	CFG_DIV(DIV_RTC,	RCC_RTCDIVR, 0, 6, 0, NULL),
	CFG_DIV(DIV_MCO1,	RCC_MCO1CFGR, 4, 4, 0, NULL),
	CFG_DIV(DIV_MCO2,	RCC_MCO2CFGR, 4, 4, 0, NULL),
	CFG_DIV(DIV_TRACE,	RCC_DBGCFGR, 0, 3, 0, ck_trace_div_table),
	CFG_DIV(DIV_ETHPTP,	RCC_ETHCKSELR, 4, 4, 0, NULL),
};

struct clk_stm32_clock_data {
	const struct cs_gate_cfg *gates;
	const struct cs_mux_cfg *muxes;
	const struct cs_div_cfg *dividers;
};

struct clock_summary {
	struct clk_summary *clocks;
	int nb_clocks;
	void __iomem *base;
	struct clk_stm32_clock_data *clock_data;
};

struct clk_summary {
	const char *name;
	unsigned long rate;
	int enabled;
	int nb_parents;
	int gate_id;
	int mux_id;
	int div_id;
	void *data;
	unsigned long (*get_rate)(struct clock_summary *cs,
				  struct clk_summary *c,
				  unsigned long parent_rate);
	const char * const *parent_names;
};

static u8 clk_stm32_get_parent_mux(void __iomem *base,
				   struct clk_stm32_clock_data *data,
				   u16 mux_id)
{
	const struct cs_mux_cfg *mux = &data->muxes[mux_id];
	u32 mask = BIT(mux->width) - 1;
	u32 val;

	val = readl(base + mux->offset) >> mux->shift;
	val &= mask;

	return val;
}

static int clk_stm32_is_enabled_gate(void __iomem *base,
				     struct clk_stm32_clock_data *data,
				     u16 gate_id)
{
	const struct cs_gate_cfg *gate = &data->gates[gate_id];

	return (readl(base + gate->offset) & BIT(gate->bit_idx)) != 0;
}

static unsigned int _get_table_div(const struct clk_div_table *table,
				   unsigned int val)
{
	const struct clk_div_table *clkt;

	for (clkt = table; clkt->div; clkt++)
		if (clkt->val == val)
			return clkt->div;
	return 0;
}

static unsigned int _get_div(const struct clk_div_table *table,
			     unsigned int val, unsigned long flags, u8 width)
{
	if (flags & CLK_DIVIDER_ONE_BASED)
		return val;
	if (flags & CLK_DIVIDER_POWER_OF_TWO)
		return 1 << val;
	if (table)
		return _get_table_div(table, val);
	return val + 1;
}

static unsigned long clk_stm32_get_rate_divider(void __iomem *base,
						struct clk_stm32_clock_data *data,
						u16 div_id,
						unsigned long parent_rate)
{
	const struct cs_div_cfg *divider = &data->dividers[div_id];
	unsigned int val;
	unsigned int div;

	val =  readl(base + divider->offset) >> divider->shift;
	val &= clk_div_mask(divider->width);
	div = _get_div(divider->table, val, divider->flags, divider->width);

	if (!div) {
		WARN(!(divider->flags & CLK_DIVIDER_ALLOW_ZERO),
		     "%d: Zero divisor and CLK_DIVIDER_ALLOW_ZERO not set\n", div_id);
		return parent_rate;
	}

	return DIV_ROUND_UP_ULL((u64)parent_rate, div);
}

struct cs_pll {
	u32 offset;
};

static unsigned long clk_summary_pll_frac_div_recalc_rate(struct clock_summary *cs,
							  struct clk_summary *c,
							  unsigned long parent_rate)
{
	struct cs_pll *pll = (struct cs_pll *)c->data;
	struct clk_pll_fractional_divider fracdiv;
	struct clk_pll_fractional_divider *fd = &fracdiv;
	void __iomem *reg;
	u32 mmask;
	u32 nmask;
	u32 fmask;
	unsigned long m, n, f;
	u64 rate, frate = 0;
	u32 val;

	reg = cs->base + pll->offset;
	fd->mreg = reg + PLL_DIVMN_OFFSET;
	fd->mshift = PLL_DIVM_SHIFT;
	fd->mwidth = PLL_DIVM_WIDTH;
	fd->mflags = CLK_FRAC_DIVIDER_ZERO_BASED;
	fd->nreg = reg + PLL_DIVMN_OFFSET;
	fd->nshift = PLL_DIVN_SHIFT;
	fd->nwidth = PLL_DIVN_WIDTH;
	fd->nflags = CLK_FRAC_DIVIDER_ZERO_BASED;
	fd->freg = reg + PLL_FRAC_OFFSET;
	fd->fshift = PLL_FRAC_SHIFT;
	fd->fwidth = PLL_FRAC_WIDTH;

	mmask = GENMASK(fd->mwidth - 1, 0) << fd->mshift;
	nmask = GENMASK(fd->nwidth - 1, 0) << fd->nshift;
	fmask = GENMASK(fd->fwidth - 1, 0) << fd->fshift;

	val = readl(fd->mreg);
	m = (val & mmask) >> fd->mshift;
	if (fd->mflags & CLK_FRAC_DIVIDER_ZERO_BASED)
		m++;

	val = readl(fd->nreg);
	n = (val & nmask) >> fd->nshift;
	if (fd->nflags & CLK_FRAC_DIVIDER_ZERO_BASED)
		n++;

	if (!n || !m)
		return parent_rate;

	rate = (u64)parent_rate * n;
	do_div(rate, m);

	val = readl(fd->freg);
	f = (val & fmask) >> fd->fshift;
	if (f) {
		frate = (u64)parent_rate * (u64)f;
		do_div(frate, (m * (1 << fd->fwidth)));
	}

	return rate + frate;
}

static unsigned long clk_summary_hsediv2_recalc_rate(struct clock_summary *cs,
						     struct clk_summary *c,
						     unsigned long parent_rate)
{
	return parent_rate / 2;
}

static unsigned long clk_summary_osc_recalc_rate(struct clock_summary *cs,
						 struct clk_summary *c,
						 unsigned long parent_rate)
{
	struct clk *clk = __clk_lookup(c->name);

	if (clk)
		return clk_get_rate(clk);

	return 0;
}

static unsigned long clk_summary_div_recalc_rate(struct clock_summary *cs,
						 struct clk_summary *c,
						 unsigned long parent_rate)
{
	return clk_stm32_get_rate_divider(cs->base, cs->clock_data, c->div_id, parent_rate);
}

static unsigned long clk_summary_rtc_recalc_rate(struct clock_summary *cs,
						 struct clk_summary *c,
						 unsigned long parent_rate)
{
	u8 parent;

	parent = clk_stm32_get_parent_mux(cs->base, cs->clock_data, c->mux_id);
	if (parent == HSE_RTC)
		return clk_summary_div_recalc_rate(cs, c, parent_rate);

	return parent_rate;
}

struct cs_stm32_timer {
	u32 apbdiv;
	u32 timpre;
};

static unsigned long clk_stm32_timer_recalc_rate(struct clock_summary *cs,
						 struct clk_summary *c,
						 unsigned long parent_rate)
{
	struct cs_stm32_timer *tim = (struct cs_stm32_timer *)c->data;
	void __iomem *rcc_base = cs->base;
	u32 prescaler, timpre;

	prescaler = readl(rcc_base + tim->apbdiv) & APB_DIV_MASK;

	timpre = readl(rcc_base + tim->timpre) & TIM_PRE_MASK;

	if (prescaler == 0U)
		return parent_rate;

	return parent_rate * (timpre + 1U) * 2U;
}

#define CS_OSC(_name, _gate) \
{\
	.name		= _name,\
	.nb_parents	= 0,\
	.gate_id	= _gate,\
	.mux_id		= NO_STM32_MUX,\
	.div_id		= NO_STM32_DIV,\
	.get_rate	= clk_summary_osc_recalc_rate,\
}

#define CS_DIV2(_name, _parent) \
{\
	.name		= _name,\
	.nb_parents	= 1,\
	.parent_names	= PARENT(_parent),\
	.gate_id	= NO_STM32_GATE,\
	.mux_id		= NO_STM32_MUX,\
	.div_id		= NO_STM32_DIV,\
	.get_rate	= clk_summary_hsediv2_recalc_rate,\
}

#define CS_PLL(_name, _parents, _gate, _mux, _offset)\
{\
	.name		= _name,\
	.nb_parents	= ARRAY_SIZE(_parents),\
	.parent_names	= _parents,\
	.gate_id	= _gate,\
	.mux_id		= _mux,\
	.div_id		= NO_STM32_DIV,\
	.data		=  &(struct cs_pll) {\
		.offset		= _offset,\
	},\
	.get_rate	= clk_summary_pll_frac_div_recalc_rate,\
}

#define CS_DIV(_name, _parent, _div) \
{\
	.name		= _name,\
	.nb_parents	= 1,\
	.parent_names	= PARENT(_parent),\
	.div_id		= _div,\
	.gate_id	= NO_STM32_GATE,\
	.mux_id		= NO_STM32_MUX,\
	.get_rate	= clk_summary_div_recalc_rate,\
}

#define CS_MUX(_name, _parents, _mux) \
{\
	.name		= _name,\
	.nb_parents	= ARRAY_SIZE(_parents),\
	.parent_names	= _parents,\
	.mux_id		= _mux,\
	.gate_id	= NO_STM32_GATE,\
	.div_id		= NO_STM32_DIV,\
}

#define CS_GATE(_name, _parent, _gate) \
{\
	.name		= _name,\
	.nb_parents	= 1,\
	.parent_names	= PARENT(_parent),\
	.gate_id	= _gate,\
	.mux_id		= NO_STM32_MUX,\
	.div_id		= NO_STM32_DIV,\
}

#define CS_GATEDIV(_name, _parent, _gate, _div) \
{\
	.name		= _name,\
	.nb_parents	= 1,\
	.parent_names	= PARENT(_parent),\
	.gate_id	= _gate,\
	.mux_id		= NO_STM32_MUX,\
	.div_id		= _div,\
	.get_rate	= clk_summary_div_recalc_rate,\
}

#define CS_GATEMUX(_name, _parents, _gate, _mux) \
{\
	.name		= _name,\
	.nb_parents	= ARRAY_SIZE(_parents),\
	.parent_names	= _parents,\
	.gate_id	= _gate,\
	.mux_id		= _mux,\
	.div_id		= NO_STM32_DIV,\
}

#define CS_COMPOSITE(_name, _parents, _gate, _mux, _div) \
{\
	.name		= _name,\
	.nb_parents	= ARRAY_SIZE(_parents),\
	.parent_names	= _parents,\
	.gate_id	= _gate,\
	.mux_id		= _mux,\
	.div_id		= _div,\
	.get_rate	= clk_summary_div_recalc_rate,\
}

#define CS_RTC(_name, _parents, _gate, _mux, _div) \
{\
	.name		= _name,\
	.nb_parents	= ARRAY_SIZE(_parents),\
	.parent_names	= _parents,\
	.gate_id	= _gate,\
	.mux_id		= _mux,\
	.div_id		= _div,\
	.get_rate	= clk_summary_rtc_recalc_rate,\
}

#define CS_STM32_TIMER(_name, _parent, _apbdiv, _timpre) \
{\
	.name		= _name,\
	.nb_parents	= 1,\
	.parent_names	= PARENT(_parent),\
	.div_id		= NO_STM32_DIV,\
	.gate_id	= NO_STM32_GATE,\
	.mux_id		= NO_STM32_MUX,\
	.data		=  &(struct cs_stm32_timer) {\
		.apbdiv		= _apbdiv,\
		.timpre		= _timpre,\
	},\
	.get_rate	= clk_stm32_timer_recalc_rate,\
}

static struct clk_summary stm32mp15_clock_summary[] = {
	CS_OSC("ck_hsi", GATE_HSI),
	CS_OSC("ck_csi", GATE_CSI),
	CS_OSC("ck_lsi", GATE_LSI),
	CS_OSC("ck_hse", GATE_HSE),
	CS_OSC("ck_lse", GATE_LSE),
	CS_OSC("ck_usbo_48m", NO_STM32_GATE),
	CS_DIV2("clk-hse-div2", "ck_hse"),
	CS_MUX("ck_per", per_src, MUX_CKPER),

	CS_PLL("pll1", ref12_parents, GATE_PLL1, MUX_PLL12, RCC_PLL1CR),
	CS_GATEDIV("pll1_p", "pll1", GATE_PLL1_DIVP, DIV_PLL1DIVP),

	CS_DIV("pll1_p_div", "pll1_p", DIV_MPU),

	CS_PLL("pll2", ref12_parents, GATE_PLL2, MUX_PLL12, RCC_PLL2CR),
	CS_GATEDIV("pll2_p", "pll2", GATE_PLL2_DIVP, DIV_PLL2DIVP),
	CS_GATEDIV("pll2_q", "pll2", GATE_PLL2_DIVQ, DIV_PLL2DIVQ),
	CS_GATEDIV("pll2_r", "pll2", GATE_PLL2_DIVR, DIV_PLL2DIVR),

	CS_PLL("pll3", ref3_parents, GATE_PLL3, MUX_PLL3, RCC_PLL3CR),
	CS_GATEDIV("pll3_p", "pll3", GATE_PLL3_DIVP, DIV_PLL3DIVP),
	CS_GATEDIV("pll3_q", "pll3", GATE_PLL3_DIVQ, DIV_PLL3DIVQ),
	CS_GATEDIV("pll3_r", "pll3", GATE_PLL3_DIVR, DIV_PLL3DIVR),

	CS_PLL("pll4", ref4_parents, GATE_PLL4, MUX_PLL4, RCC_PLL4CR),
	CS_GATEDIV("pll4_p", "pll4", GATE_PLL4_DIVP, DIV_PLL4DIVP),
	CS_GATEDIV("pll4_q", "pll4", GATE_PLL4_DIVQ, DIV_PLL4DIVQ),
	CS_GATEDIV("pll4_r", "pll4", GATE_PLL4_DIVR, DIV_PLL4DIVR),

	CS_MUX("ck_mpu", cpu_src, MUX_MPU),
	CS_MUX("ck_axi", axi_src, MUX_AXI),
	CS_MUX("ck_mcu", mcu_src, MUX_MCU),

	CS_DIV("pclk1", "ck_mcu", DIV_APB1),
	CS_DIV("pclk2", "ck_mcu", DIV_APB2),
	CS_DIV("pclk3", "ck_mcu", DIV_APB3),
	CS_DIV("pclk4", "ck_axi", DIV_APB4),
	CS_DIV("pclk5", "ck_axi", DIV_APB5),

	CS_STM32_TIMER("ck1_tim", "pclk1", RCC_APB1DIVR, RCC_TIMG1PRER),
	CS_STM32_TIMER("ck2_tim", "pclk2", RCC_APB2DIVR, RCC_TIMG2PRER),

	CS_GATE("tim2_k", "ck1_tim", GATE_TIM2),
	CS_GATE("tim3_k", "ck1_tim", GATE_TIM3),
	CS_GATE("tim4_k", "ck1_tim", GATE_TIM4),
	CS_GATE("tim5_k", "ck1_tim", GATE_TIM5),
	CS_GATE("tim6_k", "ck1_tim", GATE_TIM6),
	CS_GATE("tim7_k", "ck1_tim", GATE_TIM7),
	CS_GATE("tim12_k", "ck1_tim", GATE_TIM12),
	CS_GATE("tim13_k", "ck1_tim", GATE_TIM13),
	CS_GATE("tim14_k", "ck1_tim", GATE_TIM14),
	CS_GATE("tim1_k", "ck2_tim", GATE_TIM1),
	CS_GATE("tim8_k", "ck2_tim", GATE_TIM8),
	CS_GATE("tim15_k", "ck2_tim", GATE_TIM15),
	CS_GATE("tim16_k", "ck2_tim", GATE_TIM16),
	CS_GATE("tim17_k", "ck2_tim", GATE_TIM17),

	CS_GATE("tim2", "pclk1", GATE_TIM2),
	CS_GATE("tim3", "pclk1", GATE_TIM3),
	CS_GATE("tim4", "pclk1", GATE_TIM4),
	CS_GATE("tim5", "pclk1", GATE_TIM5),
	CS_GATE("tim6", "pclk1", GATE_TIM6),
	CS_GATE("tim7", "pclk1", GATE_TIM7),
	CS_GATE("tim12", "pclk1", GATE_TIM12),
	CS_GATE("tim13", "pclk1", GATE_TIM13),
	CS_GATE("tim14", "pclk1", GATE_TIM14),
	CS_GATE("lptim1", "pclk1", GATE_LPTIM1),
	CS_GATE("spi2", "pclk1", GATE_SPI2),
	CS_GATE("spi3", "pclk1", GATE_SPI3),
	CS_GATE("usart2", "pclk1", GATE_USART2),
	CS_GATE("usart3", "pclk1", GATE_USART3),
	CS_GATE("uart4", "pclk1", GATE_UART4),
	CS_GATE("uart5", "pclk1", GATE_UART5),
	CS_GATE("uart7", "pclk1", GATE_UART7),
	CS_GATE("uart8", "pclk1", GATE_UART8),
	CS_GATE("i2c1", "pclk1", GATE_I2C1),
	CS_GATE("i2c2", "pclk1", GATE_I2C2),
	CS_GATE("i2c3", "pclk1", GATE_I2C3),
	CS_GATE("i2c5", "pclk1", GATE_I2C5),
	CS_GATE("spdif", "pclk1", GATE_SPDIF),
	CS_GATE("cec", "pclk1", GATE_CEC),
	CS_GATE("dac12", "pclk1", GATE_DAC12),
	CS_GATE("mdio", "pclk1", GATE_MDIO),
	CS_GATE("tim1", "pclk2", GATE_TIM1),
	CS_GATE("tim8", "pclk2", GATE_TIM8),
	CS_GATE("tim15", "pclk2", GATE_TIM15),
	CS_GATE("tim16", "pclk2", GATE_TIM16),
	CS_GATE("tim17", "pclk2", GATE_TIM17),
	CS_GATE("spi1", "pclk2", GATE_SPI1),
	CS_GATE("spi4", "pclk2", GATE_SPI4),
	CS_GATE("spi5", "pclk2", GATE_SPI5),
	CS_GATE("usart6", "pclk2", GATE_USART6),
	CS_GATE("sai1", "pclk2", GATE_SAI1),
	CS_GATE("sai2", "pclk2", GATE_SAI2),
	CS_GATE("sai3", "pclk2", GATE_SAI3),
	CS_GATE("dfsdm", "pclk2", GATE_DFSDM),
	CS_GATE("fdcan", "pclk2", GATE_FDCAN),
	CS_GATE("lptim2", "pclk3", GATE_LPTIM2),
	CS_GATE("lptim3", "pclk3", GATE_LPTIM3),
	CS_GATE("lptim4", "pclk3", GATE_LPTIM4),
	CS_GATE("lptim5", "pclk3", GATE_LPTIM5),
	CS_GATE("sai4", "pclk3", GATE_SAI4),
	CS_GATE("syscfg", "pclk3", GATE_SYSCFG),
	CS_GATE("vref", "pclk3", GATE_VREF),
	CS_GATE("tmpsens", "pclk3", GATE_TMPSENS),
	CS_GATE("pmbctrl", "pclk3", GATE_PMBCTRL),
	CS_GATE("hdp", "pclk3", GATE_HDP),
	CS_GATE("ltdc", "pclk4", GATE_LTDC),
	CS_GATE("dsi", "pclk4", GATE_DSI),
	CS_GATE("iwdg2", "pclk4", GATE_IWDG2),
	CS_GATE("usbphy", "pclk4", GATE_USBPHY),
	CS_GATE("stgenro", "pclk4", GATE_STGENRO),
	CS_GATE("spi6", "pclk5", GATE_SPI6),
	CS_GATE("i2c4", "pclk5", GATE_I2C4),
	CS_GATE("i2c6", "pclk5", GATE_I2C6),
	CS_GATE("usart1", "pclk5", GATE_USART1),
	CS_GATE("rtcapb", "pclk5", GATE_RTCAPB),
	CS_GATE("tzc1", "ck_axi", GATE_TZC1),
	CS_GATE("tzc2", "ck_axi", GATE_TZC2),
	CS_GATE("tzpc", "pclk5", GATE_TZPC),
	CS_GATE("iwdg1", "pclk5", GATE_IWDG1),
	CS_GATE("bsec", "pclk5", GATE_BSEC),
	CS_GATE("stgen", "pclk5", GATE_STGEN),
	CS_GATE("dma1", "ck_mcu", GATE_DMA1),
	CS_GATE("dma2", "ck_mcu", GATE_DMA2),
	CS_GATE("dmamux", "ck_mcu", GATE_DMAMUX),
	CS_GATE("adc12", "ck_mcu", GATE_ADC12),
	CS_GATE("usbo", "ck_mcu", GATE_USBO),
	CS_GATE("sdmmc3", "ck_mcu", GATE_SDMMC3),
	CS_GATE("dcmi", "ck_mcu", GATE_DCMI),
	CS_GATE("cryp2", "ck_mcu", GATE_CRYP2),
	CS_GATE("hash2", "ck_mcu", GATE_HASH2),
	CS_GATE("rng2", "ck_mcu", GATE_RNG2),
	CS_GATE("crc2", "ck_mcu", GATE_CRC2),
	CS_GATE("hsem", "ck_mcu", GATE_HSEM),
	CS_GATE("ipcc", "ck_mcu", GATE_IPCC),
	CS_GATE("gpioa", "ck_mcu", GATE_GPIOA),
	CS_GATE("gpiob", "ck_mcu", GATE_GPIOB),
	CS_GATE("gpioc", "ck_mcu", GATE_GPIOC),
	CS_GATE("gpiod", "ck_mcu", GATE_GPIOD),
	CS_GATE("gpioe", "ck_mcu", GATE_GPIOE),
	CS_GATE("gpiof", "ck_mcu", GATE_GPIOF),
	CS_GATE("gpiog", "ck_mcu", GATE_GPIOG),
	CS_GATE("gpioh", "ck_mcu", GATE_GPIOH),
	CS_GATE("gpioi", "ck_mcu", GATE_GPIOI),
	CS_GATE("gpioj", "ck_mcu", GATE_GPIOJ),
	CS_GATE("gpiok", "ck_mcu", GATE_GPIOK),
	CS_GATE("gpioz", "ck_axi", GATE_GPIOZ),
	CS_GATE("cryp1", "ck_axi", GATE_CRYP1),
	CS_GATE("hash1", "ck_axi", GATE_HASH1),
	CS_GATE("rng1", "ck_axi", GATE_RNG1),
	CS_GATE("bkpsram", "ck_axi", GATE_BKPSRAM),
	CS_GATE("mdma", "ck_axi", GATE_MDMA),
	CS_GATE("gpu", "ck_axi", GATE_GPU),
	CS_GATE("ethtx", "ck_axi", GATE_ETHTX),
	CS_GATE("ethrx", "ck_axi", GATE_ETHRX),
	CS_GATE("ethmac", "ck_axi", GATE_ETHMAC),
	CS_GATE("crc1", "ck_axi", GATE_CRC1),
	CS_GATE("usbh", "ck_axi", GATE_USBH),
	CS_GATE("ethstp", "ck_axi", GATE_ETHSTP),
	CS_GATE("ddrperfm", "pclk4", GATE_DDRPERFM),

	CS_GATEMUX("sdmmc1_k", sdmmc12_src, GATE_SDMMC1, MUX_SDMMC12),
	CS_GATEMUX("sdmmc2_k", sdmmc12_src, GATE_SDMMC2, MUX_SDMMC12),
	CS_GATEMUX("sdmmc3_k", sdmmc3_src, GATE_SDMMC3, MUX_SDMMC3),
	CS_GATEMUX("fmc_k", fmc_src, GATE_FMC, MUX_FMC),
	CS_GATEMUX("qspi_k", qspi_src, GATE_QSPI, MUX_QSPI),
	CS_GATEMUX("rng1_k", rng_src, GATE_RNG1, MUX_RNG1),
	CS_GATEMUX("rng2_k", rng_src, GATE_RNG2, MUX_RNG2),
	CS_GATEMUX("usbphy_k", usbphy_src, GATE_USBPHY, MUX_USBPHY),
	CS_GATEMUX("stgen_k", stgen_src, GATE_STGEN, MUX_STGEN),
	CS_GATEMUX("spdif_k", spdif_src, GATE_SPDIF, MUX_SPDIF),
	CS_GATEMUX("spi1_k", spi123_src, GATE_SPI1, MUX_SPI1),
	CS_GATEMUX("spi2_k", spi123_src, GATE_SPI2, MUX_SPI23),
	CS_GATEMUX("spi3_k", spi123_src, GATE_SPI3, MUX_SPI23),
	CS_GATEMUX("spi4_k", spi45_src, GATE_SPI4, MUX_SPI45),
	CS_GATEMUX("spi5_k", spi45_src, GATE_SPI5, MUX_SPI45),
	CS_GATEMUX("spi6_k", spi6_src, GATE_SPI6, MUX_SPI6),
	CS_GATEMUX("cec_k", cec_src, GATE_CEC, MUX_CEC),
	CS_GATEMUX("i2c1_k", i2c12_src, GATE_I2C1, MUX_I2C12),
	CS_GATEMUX("i2c2_k", i2c12_src, GATE_I2C2, MUX_I2C12),
	CS_GATEMUX("i2c3_k", i2c35_src, GATE_I2C3, MUX_I2C35),
	CS_GATEMUX("i2c5_k", i2c35_src, GATE_I2C5, MUX_I2C35),
	CS_GATEMUX("i2c4_k", i2c46_src, GATE_I2C4, MUX_I2C46),
	CS_GATEMUX("i2c6_k", i2c46_src, GATE_I2C6, MUX_I2C46),
	CS_GATEMUX("lptim1_k", lptim1_src, GATE_LPTIM1, MUX_LPTIM1),
	CS_GATEMUX("lptim2_k", lptim23_src, GATE_LPTIM2, MUX_LPTIM23),
	CS_GATEMUX("lptim3_k", lptim23_src, GATE_LPTIM3, MUX_LPTIM23),
	CS_GATEMUX("lptim4_k", lptim45_src, GATE_LPTIM4, MUX_LPTIM45),
	CS_GATEMUX("lptim5_k", lptim45_src, GATE_LPTIM5, MUX_LPTIM45),
	CS_GATEMUX("usart1_k", usart1_src, GATE_USART1, MUX_USART1),
	CS_GATEMUX("usart2_k", usart234578_src, GATE_USART2, MUX_UART24),
	CS_GATEMUX("usart3_k", usart234578_src, GATE_USART3, MUX_UART35),
	CS_GATEMUX("uart4_k", usart234578_src, GATE_UART4, MUX_UART24),
	CS_GATEMUX("uart5_k", usart234578_src, GATE_UART5, MUX_UART35),
	CS_GATEMUX("uart6_k", usart6_src, GATE_USART6, MUX_USART6),
	CS_GATEMUX("uart7_k", usart234578_src, GATE_UART7, MUX_UART78),
	CS_GATEMUX("uart8_k", usart234578_src, GATE_UART8, MUX_UART78),
	CS_GATEMUX("fdcan_k", fdcan_src, GATE_FDCAN, MUX_FDCAN),
	CS_GATEMUX("sai1_k", sai_src, GATE_SAI1, MUX_SAI1),
	CS_GATEMUX("sai2_k", sai2_src, GATE_SAI2, MUX_SAI2),
	CS_GATEMUX("sai3_k", sai_src, GATE_SAI3, MUX_SAI3),
	CS_GATEMUX("sai4_k", sai_src, GATE_SAI4, MUX_SAI4),
	CS_GATEMUX("adc12_k", adc12_src, GATE_ADC12, MUX_ADC12),
	CS_GATEMUX("dsi_k", dsi_src, GATE_DSI, MUX_DSI),
	CS_GATEMUX("adfsdm_k", sai_src, GATE_ADFSDM, MUX_SAI1),
	CS_GATEMUX("usbo_k", usbo_src, GATE_USBO, MUX_USBO),
	CS_GATEMUX("ethck_k", eth_src, GATE_ETHCK, MUX_ETHCK),

	CS_GATE("dfsdm_k", "ck_mcu", GATE_DFSDM),
	CS_GATE("dsi_px", "pll4_q", GATE_DSI),
	CS_GATE("ltdc_px", "pll4_q", GATE_LTDC),
	CS_GATE("gpu_k", "pll2_q", GATE_GPU),
	CS_GATE("dac12_k", "ck_lsi", GATE_DAC12),

	CS_COMPOSITE("ck_mco1", mco1_src, GATE_MCO1, MUX_MCO1, DIV_MCO1),
	CS_COMPOSITE("ck_mco2", mco2_src, GATE_MCO2, MUX_MCO2, DIV_MCO2),
	CS_GATE("ck_sys_dbg", "ck_axi", GATE_DBGCK),

	CS_COMPOSITE("ethptp_k", eth_src, NO_STM32_GATE, MUX_ETHCK, DIV_ETHPTP),

	CS_RTC("ck_rtc", rtc_src, GATE_RTCCK, MUX_RTC, DIV_RTC),

	CS_GATEDIV("ck_trace", "ck_axi", GATE_TRACECK, DIV_TRACE),
};

static void rcc_summary_show_one(struct seq_file *s, struct clk_summary *c,
				 int level)
{
	char enabled;

	seq_printf(s, "%*s%-*s %11lu ",
		   level * 3 + 1, "",
		   30 - level * 3,
		   c->name,
		   c->rate
		);

	switch (c->enabled) {
	case 0:
		enabled = 'N';
		break;
	case 1:
		enabled = 'Y';
		break;
	default:
		enabled = '?';
		break;
	}

	seq_printf(s, " %9c\n", enabled);
}

static int clock_summary_clk_is_enabled(struct clock_summary *cs,
					struct clk_summary *c)
{
	return clk_stm32_is_enabled_gate(cs->base, cs->clock_data, c->gate_id);
}

static const char *clock_summary_get_parent_name(struct clock_summary *cs,
						 struct clk_summary *c)
{
	int id = 0;

	if (c->nb_parents == 0)
		return NULL;

	if (c->nb_parents > 1)
		id = clk_stm32_get_parent_mux(cs->base, cs->clock_data, c->mux_id);

	return c->parent_names[id];
}

static void rcc_summary_show_subtree(struct seq_file *s, struct clk_summary *c,
				     unsigned long parent_rate, int level)
{
	struct clock_summary *cs = (struct clock_summary *)s->private;
	int i;

	if (c->get_rate)
		c->rate = c->get_rate(cs, c, parent_rate);
	else
		c->rate = parent_rate;

	c->enabled = -1;
	if (c->gate_id != NO_STM32_GATE)
		c->enabled = clock_summary_clk_is_enabled(cs, c);

	rcc_summary_show_one(s, c, level);

	for (i = 0; i < cs->nb_clocks; i++) {
		struct clk_summary *child = &cs->clocks[i];
		const char *parent_name = clock_summary_get_parent_name(cs, child);

		if (!parent_name)
			continue;

		if (!strcmp(c->name, parent_name))
			rcc_summary_show_subtree(s, child, c->rate, level + 1);
	}
}

static int rcc_summary_show(struct seq_file *s, void *data)
{
	struct clock_summary *cs = (struct clock_summary *)s->private;
	int i;

	seq_puts(s, "                                              hardware\n");
	seq_puts(s, "   clock                               rate     enable\n");
	seq_puts(s, "------------------------------------------------------\n");

	for (i = 0; i < cs->nb_clocks; i++) {
		struct clk_summary *c = &cs->clocks[i];

		if (c->nb_parents == 0)
			rcc_summary_show_subtree(s, c, 0, 0);
	}

	return 0;
}

DEFINE_SHOW_ATTRIBUTE(rcc_summary);

struct clk_stm32_clock_data stm32mp15_clock_data = {
	.gates		= stm32mp15_gates,
	.muxes		= stm32mp15_muxes,
	.dividers	= stm32mp15_dividers,
};

static struct clock_summary clock_summary_mp15 = {
	.clocks		= stm32mp15_clock_summary,
	.nb_clocks	= ARRAY_SIZE(stm32mp15_clock_summary),
	.clock_data	= &stm32mp15_clock_data,
};

static void stm32_clk_summary_debugfs_create(struct device *dev, void __iomem *base)
{
	struct dentry *rootdir = debugfs_lookup("clk", NULL);

	clock_summary_mp15.base = base;

	debugfs_create_file("stm32_clk_summary", 0444, rootdir,
			    &clock_summary_mp15, &rcc_summary_fops);
}

#else

static void stm32_clk_summary_debugfs_create(struct device *dev, void __iomem *base)
{
}

#endif
