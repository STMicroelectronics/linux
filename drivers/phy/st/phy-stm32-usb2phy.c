// SPDX-License-Identifier: GPL-2.0-only
/*
 * STMicroelectronics STM32 USB2 PHY Controller driver
 * Currently Only supported for STM32MP25
 *
 * Copyright (C) 2022 STMicroelectronics
 * Author(s): Pankaj Dev <pankaj.dev@st.com>.
 */
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/phy/phy.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/reset.h>
#include <linux/usb/role.h>
#include <linux/mfd/syscon.h>

#define PHY1CR_OFFSET		0x2400
#define PHY1TRIM1_OFFSET	0x240C
#define PHY1TRIM2_OFFSET	0x2410
#define PHY2CR_OFFSET		0x2800
#define PHY2TRIM1_OFFSET	0x2808
#define PHY2TRIM2_OFFSET	0x280C

#define PHYCR_REG      1

/* Retention mode enable (active low) */
#define SYSCFG_USB2PHY2CR_RETENABLEN2_MASK		BIT(0)
/*
 * Auto-resume mode enable. Enables auto-resume logic in USB2PHY so that the PHY automatically
 * responds to a remote wake-up without initial involvement of the Host controller.
 */
#define SYSCFG_USB2PHY2CR_AUTORSMENB2_MASK		BIT(1)
/* Controls the power down of analog blocks during Suspend and Sleep. */
#define SYSCFG_USB2PHY2CR_USB2PHY2CMN_MASK		BIT(2)
/* Controls vbus valid input of USB3 DRD controller when in Host mode */
#define SYSCFG_USB2PHY2CR_VBUSVALID_MASK		BIT(4)
/* Selects VBUS valid comparator that is used when USB3 DRD controller is in Device mode */
#define SYSCFG_USB2PHY2CR_VBUSVLDEXTSEL_MASK		BIT(5)
/* Voltage comparison result when an external voltage comparator is used */
#define SYSCFG_USB2PHY2CR_VBUSVLDEXT_MASK		BIT(6)
/*
 * 0: internal debounce logic is enabled (default).
 * Bit0: applies to utmiotg_vbusvalid,
 * Bit1: applies to pipe3_PowerPresent,
 * Bit2: applies to utmisrp_bvalid,
 * Bit3: applies to utmiotg_iddig]. (default)
 */
#define SYSCFG_USB2PHY2CR_FILTER_BYPASS_MASK		GENMASK(10, 7)
/* Voltage comparison result when an external voltage comparator is used */
#define SYSCFG_USB2PHY2CR_OTGDISABLE0_MASK		BIT(16)
/* Voltage comparison result when an external voltage comparator is used */
#define SYSCFG_USB2PHY2CR_DRVVBUS0_MASK			BIT(17)

#define SYSCFG_USB2PHYTRIM1_PLLITUNE_MASK		GENMASK(1, 0)
#define SYSCFG_USB2PHYTRIM1_PLLPTUNE_MASK		GENMASK(5, 2)
#define SYSCFG_USB2PHYTRIM1_COMPDISTUNE_MASK		GENMASK(8, 6)
#define SYSCFG_USB2PHYTRIM1_SQRXTUNE_MASK		GENMASK(11, 9)
#define SYSCFG_USB2PHYTRIM1_VDATREFTUNE_MASK		GENMASK(13, 12)
#define SYSCFG_USB2PHYTRIM1_OTGTUNE_MASK		GENMASK(16, 14)
#define SYSCFG_USB2PHYTRIM1_TXHSXVTUNE_MASK		GENMASK(18, 17)
#define SYSCFG_USB2PHYTRIM1_TXFSLSTUNE_MASK		GENMASK(22, 19)
#define SYSCFG_USB2PHYTRIM1_TXVREFTUNE_MASK		GENMASK(26, 23)
#define SYSCFG_USB2PHYTRIM1_TXRISETUNE_MASK		GENMASK(28, 27)
#define SYSCFG_USB2PHYTRIM1_TXRESTUNE_MASK		GENMASK(30, 29)

#define SYSCFG_USB2PHYTRIM2_TXPREEMPAMPTUNE_MASK	GENMASK(1, 0)
#define SYSCFG_USB2PHYTRIM2_TXPREEMPPULSETUNE_MASK	BIT(2)

struct stm32_usb2phy {
	struct phy *phy;
	struct regmap *regmap;
	struct device *dev;
	struct reset_control *rstc;
	struct clk *phyref;
	struct regulator *vdd33, *vdda18;
	enum phy_mode mode;
	u32 mask_trim1, value_trim1, mask_trim2, value_trim2;
	bool is_init;
	struct clk_hw clk48_hw;
	atomic_t en_refcnt;
	const struct stm32mp2_usb2phy_hw_data *hw_data;
};

enum stm32_usb2phy_mode {
	USB2_MODE_HOST_ONLY,
	USB2_MODE_DRD,
};

struct stm32mp2_usb2phy_hw_data {
	u32 phyrefsel_mask, phyrefsel_bitpos, cr_offset, trim1_offset, trim2_offset;
	enum stm32_usb2phy_mode valid_mode;
};

static const struct stm32mp2_usb2phy_hw_data stm32mp2_usb2phy_hwdata[] = {
	{
		.cr_offset = PHY1CR_OFFSET,
		.trim1_offset = PHY1TRIM1_OFFSET,
		.trim2_offset = PHY1TRIM2_OFFSET,
		.valid_mode = USB2_MODE_HOST_ONLY,
		.phyrefsel_mask = 0x7,
		.phyrefsel_bitpos = 4,
	},
	{
		.cr_offset = PHY2CR_OFFSET,
		.trim1_offset = PHY2TRIM1_OFFSET,
		.trim2_offset = PHY2TRIM2_OFFSET,
		.valid_mode = USB2_MODE_DRD,
		.phyrefsel_mask = 0x7,
		.phyrefsel_bitpos = 12,
	}
};

/*
 * Two phy instances are found in mp25, and some bitfields are a bit different (shift...)
 * depending on the instance. So identify the instance by using CR offset to report
 * the correct bitfields & modes to use
 */
static const struct stm32mp2_usb2phy_hw_data *stm32_usb2phy_get_hwdata(unsigned long offset)
{
	int i;

	for (i = 0; i < sizeof(stm32mp2_usb2phy_hwdata); i++)
		if (stm32mp2_usb2phy_hwdata[i].cr_offset == offset)
			break;

	if (i < sizeof(stm32mp2_usb2phy_hwdata))
		return &stm32mp2_usb2phy_hwdata[i];

	return NULL;
}

static int stm32_usb2phy_getrefsel(unsigned long rate)
{
	switch (rate) {
	case 19200000:
		return 0;
	case 20000000:
		return 1;
	case 24000000:
		return 2;
	default:
		return -EINVAL;
	}
}

static int stm32_usb2phy_regulators_enable(struct stm32_usb2phy *phy_dev)
{
	int ret;

	ret = regulator_enable(phy_dev->vdd33);
	if (ret)
		return ret;

	if (phy_dev->vdda18) {
		ret = regulator_enable(phy_dev->vdda18);
		if (ret)
			goto vdd33_disable;
	}

	return 0;

vdd33_disable:
	regulator_disable(phy_dev->vdd33);

	return ret;
}

static int stm32_usb2phy_regulators_disable(struct stm32_usb2phy *phy_dev)
{
	int ret;

	if (phy_dev->vdda18) {
		ret = regulator_disable(phy_dev->vdda18);
		if (ret)
			return ret;
	}

	ret = regulator_disable(phy_dev->vdd33);
	if (ret)
		return ret;

	return 0;
}

static int stm32_usb2phy_enable(struct stm32_usb2phy *phy_dev)
{
	int ret;
	struct device *dev = phy_dev->dev;
	unsigned long phyref_rate;
	u32 phyrefsel;
	const struct stm32mp2_usb2phy_hw_data *phy_data = phy_dev->hw_data;

	/* Check if a phy is already init or clk48 in use */
	if (atomic_inc_return(&phy_dev->en_refcnt) > 1)
		return 0;

	phyref_rate = clk_get_rate(phy_dev->phyref);

	ret = stm32_usb2phy_getrefsel(phyref_rate);
	if (ret < 0) {
		dev_err(dev, "invalid phyref clk rate: %d\n", ret);
		return ret;
	}
	phyrefsel = (u32)ret;
	dev_dbg(dev, "phyrefsel value (%d)\n", phyrefsel);

	ret = regmap_update_bits(phy_dev->regmap,
				 phy_data->cr_offset,
				 phy_data->phyrefsel_mask << phy_data->phyrefsel_bitpos,
				 phyrefsel << phy_data->phyrefsel_bitpos);
	if (ret) {
		dev_err(dev, "can't set phyrefclksel (%d)\n", ret);
		return ret;
	}

	if (phy_dev->mask_trim1) {
		ret = regmap_update_bits(phy_dev->regmap, phy_dev->hw_data->trim1_offset,
					 phy_dev->mask_trim1, phy_dev->value_trim1);
		if (ret) {
			dev_err(dev, "can't set usb2phytrim1 (%d)\n", ret);
			return ret;
		}
		dev_dbg(dev, "usb2phytrim1 mask = %x value = %x\n", phy_dev->mask_trim1,
			phy_dev->value_trim1);
	}

	if (phy_dev->mask_trim2) {
		ret = regmap_update_bits(phy_dev->regmap, phy_dev->hw_data->trim2_offset,
					 phy_dev->mask_trim2, phy_dev->value_trim2);
		if (ret) {
			dev_err(dev, "can't set usb2phytrim2 (%d)\n", ret);
			return ret;
		}
		dev_dbg(dev, "usb2phytrim2 mask = %x value = %x\n", phy_dev->mask_trim2,
			phy_dev->value_trim2);
	}

	ret = stm32_usb2phy_regulators_enable(phy_dev);
	if (ret) {
		dev_err(dev, "can't enable regulators (%d)\n", ret);
		return ret;
	}

	ret = clk_prepare_enable(phy_dev->phyref);
	if (ret) {
		dev_err(dev, "could not enable optional phyref: %d\n", ret);
		goto error_regdis;
	}

	ret = reset_control_deassert(phy_dev->rstc);
	if (ret) {
		dev_err(dev, "can't release reset (%d)\n", ret);
		goto error_clkdis;
	}

	return 0;

error_clkdis:
	clk_disable_unprepare(phy_dev->phyref);
error_regdis:
	stm32_usb2phy_regulators_disable(phy_dev);

	return ret;
}

static int stm32_usb2phy_disable(struct stm32_usb2phy *phy_dev)
{
	int ret;

	/* Check if a phy is still init or clk48 in use */
	if (atomic_dec_return(&phy_dev->en_refcnt) > 0)
		return 0;

	ret = reset_control_assert(phy_dev->rstc);
	if (ret) {
		dev_err(phy_dev->dev, "can't force reset (%d)\n", ret);
		return ret;
	}

	clk_disable_unprepare(phy_dev->phyref);

	ret = stm32_usb2phy_regulators_disable(phy_dev);
	if (ret) {
		dev_err(phy_dev->dev, "can't disable regulators (%d)\n", ret);
		return ret;
	}

	return 0;
}

static int stm32_usb2phy_suspend(struct device *dev)
{
	struct stm32_usb2phy *phy_dev = dev_get_drvdata(dev);
	int ret;

	/*
	 * Fempto-phy should be turned off since it is not needed for
	 * wakeup capability. In case usb-remote wakeup is not enabled,
	 * usb2-phy is already turned off by HCD driver using exit callback
	 */
	if (phy_dev->is_init) {
		ret = stm32_usb2phy_disable(phy_dev);
		if (ret) {
			dev_err(dev, "can't disable usb2phy (%d)\n", ret);
			return ret;
		}
	}

	return 0;
}

static int stm32_usb2phy_resume(struct device *dev)
{
	struct stm32_usb2phy *phy_dev = dev_get_drvdata(dev);
	int ret;

	/*
	 * If fempto-phy was turned off by suspend call for wakeup then needs
	 * to be turned back ON in resume. In case usb-remote wakeup is not
	 * enabled, usb2-phy is already turned ON by HCD driver using init callback
	 */
	if (phy_dev->is_init) {
		ret = stm32_usb2phy_enable(phy_dev);
		if (ret) {
			dev_err(dev, "can't enable usb2phy (%d)\n", ret);
			return ret;
		}
	}

	return 0;
}

static int stm32_usb2phy_set_mode(struct phy *phy, enum phy_mode mode, int submode)
{
	int ret;
	struct stm32_usb2phy *phy_dev = phy_get_drvdata(phy);
	const struct stm32mp2_usb2phy_hw_data *phy_data = phy_dev->hw_data;
	struct device *dev = &phy->dev;

	switch (mode) {
	case PHY_MODE_USB_HOST:
		if (phy_data->valid_mode == USB2_MODE_HOST_ONLY)
			ret = regmap_update_bits(phy_dev->regmap,
						 phy_data->cr_offset,
						 SYSCFG_USB2PHY2CR_USB2PHY2CMN_MASK,
						 0);
		else {
			if (submode == USB_ROLE_NONE) {
				ret = regmap_update_bits(phy_dev->regmap,
							 phy_data->cr_offset,
							 SYSCFG_USB2PHY2CR_USB2PHY2CMN_MASK |
							 SYSCFG_USB2PHY2CR_VBUSVLDEXT_MASK |
							 SYSCFG_USB2PHY2CR_VBUSVALID_MASK |
							 SYSCFG_USB2PHY2CR_VBUSVLDEXT_MASK, 0);
			} else {
				ret = regmap_update_bits(phy_dev->regmap,
							 phy_data->cr_offset,
							 SYSCFG_USB2PHY2CR_USB2PHY2CMN_MASK |
							 SYSCFG_USB2PHY2CR_VBUSVLDEXT_MASK |
							 SYSCFG_USB2PHY2CR_VBUSVALID_MASK,
							 SYSCFG_USB2PHY2CR_VBUSVALID_MASK);
			}
		}
		if (ret) {
			dev_err(dev, "can't set usb2phycr (%d)\n", ret);
			return ret;
		}
		break;

	case PHY_MODE_USB_DEVICE:
		if (submode == USB_ROLE_NONE) {
			ret = regmap_update_bits(phy_dev->regmap,
						 phy_data->cr_offset,
						 SYSCFG_USB2PHY2CR_USB2PHY2CMN_MASK |
						 SYSCFG_USB2PHY2CR_VBUSVALID_MASK |
						 SYSCFG_USB2PHY2CR_VBUSVLDEXTSEL_MASK |
						 SYSCFG_USB2PHY2CR_VBUSVLDEXT_MASK,
						 SYSCFG_USB2PHY2CR_USB2PHY2CMN_MASK |
						 SYSCFG_USB2PHY2CR_VBUSVLDEXTSEL_MASK);
		} else {
			ret = regmap_update_bits(phy_dev->regmap,
						 phy_data->cr_offset,
						 SYSCFG_USB2PHY2CR_USB2PHY2CMN_MASK |
						 SYSCFG_USB2PHY2CR_VBUSVALID_MASK |
						 SYSCFG_USB2PHY2CR_VBUSVLDEXTSEL_MASK |
						 SYSCFG_USB2PHY2CR_VBUSVLDEXT_MASK,
						 SYSCFG_USB2PHY2CR_USB2PHY2CMN_MASK |
						 SYSCFG_USB2PHY2CR_VBUSVLDEXTSEL_MASK |
						 SYSCFG_USB2PHY2CR_VBUSVLDEXT_MASK);
		}
		if (ret) {
			dev_err(dev, "can't set usb2phycr (%d)\n", ret);
			return ret;
		}
		break;

	default:
		return -EINVAL;
	}

	phy_dev->mode = mode;

	return 0;
}

static int stm32_usb2phy_init(struct phy *phy)
{
	int ret;
	struct stm32_usb2phy *phy_dev = phy_get_drvdata(phy);
	struct device *dev = &phy->dev;

	ret = stm32_usb2phy_enable(phy_dev);
	if (ret) {
		dev_err(dev, "can't enable usb2phy (%d)\n", ret);
		return ret;
	}

	if (phy_dev->mode != PHY_MODE_INVALID) {
		ret = stm32_usb2phy_set_mode(phy, phy_dev->mode, USB_ROLE_NONE);
		if (ret) {
			dev_err(dev, "can't set phy mode (%d)\n", ret);
			goto error_disable;
		}
	}

	phy_dev->is_init = true;

	return 0;

error_disable:
	stm32_usb2phy_disable(phy_dev);

	return ret;
}

static int stm32_usb2phy_exit(struct phy *phy)
{
	struct stm32_usb2phy *phy_dev = phy_get_drvdata(phy);
	int ret;

	ret = stm32_usb2phy_disable(phy_dev);
	if (ret) {
		dev_err(phy_dev->dev, "can't disable usb2phy (%d)\n", ret);
		return ret;
	}

	phy_dev->is_init = false;

	return 0;
}

static const struct phy_ops stm32_usb2phy_data = {
	.init = stm32_usb2phy_init,
	.exit = stm32_usb2phy_exit,
	.set_mode = stm32_usb2phy_set_mode,
	.owner = THIS_MODULE,
};

static int stm32_usb2phy_clk48_prepare(struct clk_hw *hw)
{
	struct stm32_usb2phy *phy_dev = container_of(hw, struct stm32_usb2phy,
							   clk48_hw);

	return stm32_usb2phy_enable(phy_dev);
}

static void stm32_usb2phy_clk48_unprepare(struct clk_hw *hw)
{
	struct stm32_usb2phy *phy_dev = container_of(hw, struct stm32_usb2phy,
							   clk48_hw);

	stm32_usb2phy_disable(phy_dev);
}

static unsigned long stm32_usb2phy_clk48_recalc_rate(struct clk_hw *hw,
							   unsigned long parent_rate)
{
	return 48000000;
}

static const struct clk_ops stm32_usb2phy_clk48_ops = {
	.prepare = stm32_usb2phy_clk48_prepare,
	.unprepare = stm32_usb2phy_clk48_unprepare,
	.recalc_rate = stm32_usb2phy_clk48_recalc_rate,
};

static int stm32_usb2phy_clk48_register(struct stm32_usb2phy *phy_dev)
{
	struct clk_init_data init = { };
	int ret = 0;
	char name[20];

	snprintf(name, sizeof(name), "ck_usb2phy%x_48m", phy_dev->hw_data->cr_offset);
	init.name = name;
	init.ops = &stm32_usb2phy_clk48_ops;

	phy_dev->clk48_hw.init = &init;

	ret = devm_clk_hw_register(phy_dev->dev, &phy_dev->clk48_hw);
	if (ret) {
		dev_err(phy_dev->dev, "failed to register 48m clk\n");
		return ret;
	}

	ret = devm_of_clk_add_hw_provider(phy_dev->dev, of_clk_hw_simple_get, &phy_dev->clk48_hw);
	if (ret)
		dev_err(phy_dev->dev, "adding clk provider 48m failed\n");

	return ret;
}

static int stm32_usb2phy_tuning(struct phy *phy)
{
	int ret;
	struct stm32_usb2phy *phy_dev = phy_get_drvdata(phy);
	struct device *dev = &phy->dev;
	struct device_node *np = dev->of_node;
	u8 val;

	ret = of_property_read_u8(np, "st,pll-ipath-tune", &val);
	if (ret != -EINVAL) {
		if (!ret && val <= SYSCFG_USB2PHYTRIM1_PLLITUNE_MASK) {
			phy_dev->mask_trim1 |= SYSCFG_USB2PHYTRIM1_PLLITUNE_MASK;
			phy_dev->value_trim1 |= FIELD_PREP(SYSCFG_USB2PHYTRIM1_PLLITUNE_MASK, val);
		} else {
			dev_err(dev, "Error getting pll-ipath-tune property (%d)\n", ret);
			return ret;
		}
	}

	ret = of_property_read_u8(np, "st,pll-ppath-tune", &val);
	if (ret != -EINVAL) {
		if (!ret && val <= SYSCFG_USB2PHYTRIM1_PLLPTUNE_MASK) {
			phy_dev->mask_trim1 |= SYSCFG_USB2PHYTRIM1_PLLPTUNE_MASK;
			phy_dev->value_trim1 |= FIELD_PREP(SYSCFG_USB2PHYTRIM1_PLLPTUNE_MASK, val);
		} else {
			dev_err(dev, "Error getting pll-ppath-tune property (%d)\n", ret);
			return ret;
		}
	}

	ret = of_property_read_u8(np, "st,comp-dis-tune", &val);
	if (ret != -EINVAL) {
		if (!ret && FIELD_FIT(SYSCFG_USB2PHYTRIM1_COMPDISTUNE_MASK, val)) {
			phy_dev->mask_trim1 |= SYSCFG_USB2PHYTRIM1_COMPDISTUNE_MASK;
			phy_dev->value_trim1 |= FIELD_PREP(SYSCFG_USB2PHYTRIM1_COMPDISTUNE_MASK,
							   val);
		} else {
			dev_err(dev, "Error getting comp-dis-tune property (%d)\n", ret);
			return ret;
		}
	}

	ret = of_property_read_u8(np, "st,sqrx-tune", &val);
	if (ret != -EINVAL) {
		if (!ret && FIELD_FIT(SYSCFG_USB2PHYTRIM1_SQRXTUNE_MASK, val)) {
			phy_dev->mask_trim1 |= SYSCFG_USB2PHYTRIM1_SQRXTUNE_MASK;
			phy_dev->value_trim1 |= FIELD_PREP(SYSCFG_USB2PHYTRIM1_SQRXTUNE_MASK, val);
		} else {
			dev_err(dev, "Error getting sqrx-tune property (%d)\n", ret);
			return ret;
		}
	}

	ret = of_property_read_u8(np, "st,vdatref-tune", &val);
	if (ret != -EINVAL) {
		if (!ret && FIELD_FIT(SYSCFG_USB2PHYTRIM1_VDATREFTUNE_MASK, val)) {
			phy_dev->mask_trim1 |= SYSCFG_USB2PHYTRIM1_VDATREFTUNE_MASK;
			phy_dev->value_trim1 |= FIELD_PREP(SYSCFG_USB2PHYTRIM1_VDATREFTUNE_MASK,
							   val);
		} else {
			dev_err(dev, "Error getting vdatref-tune property (%d)\n", ret);
			return ret;
		}
	}

	ret = of_property_read_u8(np, "st,otg-tune", &val);
	if (ret != -EINVAL) {
		if (!ret && FIELD_FIT(SYSCFG_USB2PHYTRIM1_OTGTUNE_MASK, val)) {
			phy_dev->mask_trim1 |= SYSCFG_USB2PHYTRIM1_OTGTUNE_MASK;
			phy_dev->value_trim1 |= FIELD_PREP(SYSCFG_USB2PHYTRIM1_OTGTUNE_MASK, val);
		} else {
			dev_err(dev, "Error getting otg-tune property (%d)\n", ret);
			return ret;
		}
	}

	ret = of_property_read_u8(np, "st,txhsxv-tune", &val);
	if (ret != -EINVAL) {
		if (!ret && FIELD_FIT(SYSCFG_USB2PHYTRIM1_TXHSXVTUNE_MASK, val)) {
			phy_dev->mask_trim1 |= SYSCFG_USB2PHYTRIM1_TXHSXVTUNE_MASK;
			phy_dev->value_trim1 |= FIELD_PREP(SYSCFG_USB2PHYTRIM1_TXHSXVTUNE_MASK,
							   val);
		} else {
			dev_err(dev, "Error getting txhsxv-tune property (%d)\n", ret);
			return ret;
		}
	}

	ret = of_property_read_u8(np, "st,txfsls-tune", &val);
	if (ret != -EINVAL) {
		if (!ret && FIELD_FIT(SYSCFG_USB2PHYTRIM1_TXFSLSTUNE_MASK, val)) {
			phy_dev->mask_trim1 |= SYSCFG_USB2PHYTRIM1_TXFSLSTUNE_MASK;
			phy_dev->value_trim1 |= FIELD_PREP(SYSCFG_USB2PHYTRIM1_TXFSLSTUNE_MASK,
							   val);
		} else {
			dev_err(dev, "Error getting txfsls-tune property (%d)\n", ret);
			return ret;
		}
	}

	ret = of_property_read_u8(np, "st,txvref-tune", &val);
	if (ret != -EINVAL) {
		if (!ret && FIELD_FIT(SYSCFG_USB2PHYTRIM1_TXVREFTUNE_MASK, val)) {
			phy_dev->mask_trim1 |= SYSCFG_USB2PHYTRIM1_TXVREFTUNE_MASK;
			phy_dev->value_trim1 |= FIELD_PREP(SYSCFG_USB2PHYTRIM1_TXVREFTUNE_MASK,
							   val);
		} else {
			dev_err(dev, "Error getting txvref-tune property (%d)\n", ret);
			return ret;
		}
	}

	ret = of_property_read_u8(np, "st,txrise-tune", &val);
	if (ret != -EINVAL) {
		if (!ret && FIELD_FIT(SYSCFG_USB2PHYTRIM1_TXRISETUNE_MASK, val)) {
			phy_dev->mask_trim1 |= SYSCFG_USB2PHYTRIM1_TXRISETUNE_MASK;
			phy_dev->value_trim1 |= FIELD_PREP(SYSCFG_USB2PHYTRIM1_TXRISETUNE_MASK,
							   val);
		} else {
			dev_err(dev, "Error getting txrise-tune property (%d)\n", ret);
			return ret;
		}
	}

	ret = of_property_read_u8(np, "st,txres-tune", &val);
	if (ret != -EINVAL) {
		if (!ret && FIELD_FIT(SYSCFG_USB2PHYTRIM1_TXRESTUNE_MASK, val)) {
			phy_dev->mask_trim1 |= SYSCFG_USB2PHYTRIM1_TXRESTUNE_MASK;
			phy_dev->value_trim1 |= FIELD_PREP(SYSCFG_USB2PHYTRIM1_TXRESTUNE_MASK,
							   val);
		} else {
			dev_err(dev, "Error getting txres-tune property (%d)\n", ret);
			return ret;
		}
	}

	ret = of_property_read_u8(np, "st,txpreempamp-tune", &val);
	if (ret != -EINVAL) {
		if (!ret && FIELD_FIT(SYSCFG_USB2PHYTRIM2_TXPREEMPAMPTUNE_MASK, val)) {
			phy_dev->mask_trim2 |= SYSCFG_USB2PHYTRIM2_TXPREEMPAMPTUNE_MASK;
			phy_dev->value_trim2 |= FIELD_PREP(SYSCFG_USB2PHYTRIM2_TXPREEMPAMPTUNE_MASK,
							   val);
		} else {
			dev_err(dev, "Error getting txpreempamp-tune property (%d)\n", ret);
			return ret;
		}
	}

	ret = of_property_read_u8(np, "st,txpreemppulse-tune", &val);
	if (ret != -EINVAL) {
		if (!ret && FIELD_FIT(SYSCFG_USB2PHYTRIM2_TXPREEMPPULSETUNE_MASK, val)) {
			phy_dev->mask_trim2 |= SYSCFG_USB2PHYTRIM2_TXPREEMPPULSETUNE_MASK;
			phy_dev->value_trim2 |= FIELD_PREP(
						SYSCFG_USB2PHYTRIM2_TXPREEMPPULSETUNE_MASK,
						val);
		} else {
			dev_err(dev, "Error getting txpreemppulse-tune property (%d)\n", ret);
			return ret;
		}
	}

	of_node_put(np);

	return 0;
}

static int stm32_usb2phy_probe(struct platform_device *pdev)
{
	struct stm32_usb2phy *phy_dev;
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct phy_provider *phy_provider;
	struct phy *phy;
	int ret;
	u32 phycr;

	phy_dev = devm_kzalloc(dev, sizeof(*phy_dev), GFP_KERNEL);
	if (!phy_dev)
		return -ENOMEM;

	phy_dev->dev = dev;
	dev_set_drvdata(dev, phy_dev);

	phy_dev->rstc = devm_reset_control_get(dev, NULL);
	if (IS_ERR(phy_dev->rstc))
		return dev_err_probe(dev, PTR_ERR(phy_dev->rstc), "failed to get USB2PHY reset\n");

	phy_dev->phyref = devm_clk_get(dev, NULL);
	if (IS_ERR(phy_dev->phyref))
		return dev_err_probe(dev, PTR_ERR(phy_dev->phyref), "failed to get phyref clk\n");

	phy_dev->regmap = syscon_regmap_lookup_by_phandle(np, "st,syscfg");
	if (IS_ERR(phy_dev->regmap))
		return dev_err_probe(dev, PTR_ERR(phy_dev->regmap),
				     "No syscfg phandle specified\n");

	ret = of_property_read_u32_index(np, "st,syscfg", PHYCR_REG, &phycr);
	if (ret)
		return dev_err_probe(dev, ret, "can't get phycr offset\n");

	phy_dev->vdd33 = devm_regulator_get_optional(dev, "vdd33");
	if (IS_ERR(phy_dev->vdd33))
		return dev_err_probe(dev, PTR_ERR(phy_dev->vdd33),
				     "failed to get vdda3v3 supply\n");

	phy_dev->vdda18 = devm_regulator_get_optional(dev, "vdda18");
	if (IS_ERR(phy_dev->vdda18)) {
		ret = PTR_ERR(phy_dev->vdda18);
		if (ret != -ENODEV)
			return dev_err_probe(dev, ret, "failed to get vdda1v8 supply\n");
		phy_dev->vdda18 = NULL;
	}

	phy_dev->hw_data = stm32_usb2phy_get_hwdata(phycr);
	if (!phy_dev->hw_data) {
		dev_err(dev, "can't get matching stm32mp2_usb2_of_data\n");
		return -EINVAL;
	}

	phy = devm_phy_create(dev, NULL, &stm32_usb2phy_data);
	if (IS_ERR(phy))
		return dev_err_probe(dev, PTR_ERR(phy), "failed to create USB2-PHY\n");

	phy_dev->phy = phy;
	phy_set_drvdata(phy, phy_dev);

	/* Configure phy tuning */
	ret = stm32_usb2phy_tuning(phy);
	if (ret)
		return dev_err_probe(dev, ret, "can't set tuning parameters\n");

	phy_provider = devm_of_phy_provider_register(dev, of_phy_simple_xlate);
	if (IS_ERR(phy_provider))
		return PTR_ERR(phy_provider);

	ret = stm32_usb2phy_clk48_register(phy_dev);
	if (ret) {
		dev_err_probe(dev, ret, "failed to register ck_usb2phy%x_48m clock: %d\n",
			      phy_dev->hw_data->cr_offset, ret);
		return ret;
	}

	return 0;
}

static DEFINE_SIMPLE_DEV_PM_OPS(stm32_usb2phy_pm_ops, stm32_usb2phy_suspend,
				stm32_usb2phy_resume);

static const struct of_device_id stm32_usb2phy_of_match[] = {
	{ .compatible = "st,stm32mp25-usb2phy" },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, stm32_usb2phy_of_match);

static struct platform_driver stm32_usb2phy_driver = {
	.probe = stm32_usb2phy_probe,
	.driver = {
		.name = "stm32-usb2phy",
		.of_match_table = stm32_usb2phy_of_match,
		.pm = pm_sleep_ptr(&stm32_usb2phy_pm_ops)
	}
};

module_platform_driver(stm32_usb2phy_driver);

MODULE_AUTHOR("Pankaj Dev <pankaj.dev@st.com>");
MODULE_DESCRIPTION("STMicroelectronics Generic USB2PHY driver for stm32");
MODULE_LICENSE("GPL v2");
