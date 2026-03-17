// SPDX-License-Identifier: (GPL-2.0+ OR BSD-3-Clause)
/*
 * platform.c - DesignWare HS OTG Controller platform driver
 *
 * Copyright (C) Matthijs Kooijman <matthijs@stdin.nl>
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/clk.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/of.h>
#include <linux/mfd/syscon.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/phy/phy.h>
#include <linux/platform_data/s3c-hsotg.h>
#include <linux/regmap.h>
#include <linux/reset.h>

#include <linux/usb/of.h>

#include "core.h"
#include "hcd.h"
#include "debug.h"

static const char dwc2_driver_name[] = "dwc2";

#define TDCD_DBNC		100

#define VBUS_CURRENT_500MA	500
#define VBUS_CURRENT_1500MA	1500
#define VBUS_CURRENT_MAX	1500

static enum power_supply_property usb_chg_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CURRENT_MAX,
};

static enum power_supply_usb_type usb_chg_psy_types[] = {
	POWER_SUPPLY_USB_TYPE_SDP,	      /* Standard Downstream Port */
	POWER_SUPPLY_USB_TYPE_DCP,	      /* Dedicated Charging Port */
	POWER_SUPPLY_USB_TYPE_CDP,	      /* Charging Downstream Port */
};

static int stm32mp2_usb2phy_batt_chg_get_property(struct power_supply *psy,
						  enum power_supply_property psp,
						  union power_supply_propval *val)
{
	struct dwc2_hsotg *hsotg = power_supply_get_drvdata(psy);

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = 1;
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		val->intval = hsotg->chg_current * 1000;
		break;
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		val->intval = VBUS_CURRENT_MAX * 1000;  /* Convert to uA */;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int stm32mp2_usb2phy_data_contact_det(struct dwc2_hsotg *hsotg)
{
	u32 reg_val;
	int retval = 0;

	/* Data contact det */
	reg_val = dwc2_readl(hsotg, GGPIO);
	reg_val |= GGPIO_STM32_OTG_GCCFG_DCDEN;
	dwc2_writel(hsotg, reg_val, GGPIO);

	/* Wait for debounce time */
	msleep(TDCD_DBNC);

	reg_val = dwc2_readl(hsotg, GGPIO);
	if (!(reg_val & GGPIO_STM32_OTG_GCCFG_FSVPLUS))
		retval = 0;
	else
		retval = -EAGAIN;

	/* reset DCDEN */
	reg_val = dwc2_readl(hsotg, GGPIO);
	reg_val &= ~GGPIO_STM32_OTG_GCCFG_DCDEN;
	dwc2_writel(hsotg, reg_val, GGPIO);

	return retval;
}

static int stm32mp2_usb2phy_primary_det(struct dwc2_hsotg *hsotg)
{
	u32 reg_val;
	int retval = 0;

	/* Primary det */
	reg_val = dwc2_readl(hsotg, GGPIO);
	reg_val |= GGPIO_STM32_OTG_GCCFG_PDEN;
	dwc2_writel(hsotg, reg_val, GGPIO);

	msleep(20);

	/* Check CHGDET bit */
	reg_val = dwc2_readl(hsotg, GGPIO);
	if (reg_val & GGPIO_STM32_OTG_GCCFG_CHGDET) {
		/* Device is connected to CDP or DCP */
		retval = 1;
	} else {
		/* Device is connected to SDP */
		dev_info(hsotg->dev, "Device connected to SDP\n");
		hsotg->batt_chg_psy_type = POWER_SUPPLY_TYPE_USB;
		hsotg->chg_current = VBUS_CURRENT_500MA;
		retval = 0;
	}

	reg_val = dwc2_readl(hsotg, GGPIO);
	reg_val &= ~GGPIO_STM32_OTG_GCCFG_PDEN;
	dwc2_writel(hsotg, reg_val, GGPIO);

	return retval;
}

static void stm32mp2_usb2phy_secondary_det(struct dwc2_hsotg *hsotg)
{
	u32 reg_val;

	/* Secondary det */
	reg_val = dwc2_readl(hsotg, GGPIO);
	reg_val |= GGPIO_STM32_OTG_GCCFG_SDEN;
	dwc2_writel(hsotg, reg_val, GGPIO);

	msleep(20);

	/* Check CHGDET bit */
	reg_val = dwc2_readl(hsotg, GGPIO);
	if (reg_val & GGPIO_STM32_OTG_GCCFG_CHGDET) {
		/* Device is connected to DCP */
		dev_info(hsotg->dev, "Device connected to DCP\n");
		hsotg->batt_chg_psy_type = POWER_SUPPLY_TYPE_USB_DCP;
		hsotg->chg_current = VBUS_CURRENT_1500MA;
	} else {
		/* Device is connected to CDP */
		dev_info(hsotg->dev, "Device connected to CDP\n");
		hsotg->batt_chg_psy_type = POWER_SUPPLY_TYPE_USB_CDP;
		hsotg->chg_current = VBUS_CURRENT_1500MA;
	}

	reg_val = dwc2_readl(hsotg, GGPIO);
	reg_val &= ~GGPIO_STM32_OTG_GCCFG_SDEN;
	dwc2_writel(hsotg, reg_val, GGPIO);
}

int stm32mp2_usb2phy_usb_chg_psy_register(struct dwc2_hsotg *hsotg)
{
	struct power_supply_desc *psy_desc = &hsotg->batt_chg_psy_desc;
	struct power_supply_config usb_chg_psy_cfg = {};
	char *psy_name;

	psy_name = devm_kasprintf(hsotg->dev, GFP_KERNEL, "psy-%s",
				  dev_name(hsotg->dev));
	if (!psy_name)
		return -ENOMEM;

	psy_desc->name = psy_name;
	psy_desc->type = POWER_SUPPLY_TYPE_USB;
	psy_desc->usb_types = usb_chg_psy_types;
	psy_desc->num_usb_types = ARRAY_SIZE(usb_chg_psy_types);
	psy_desc->properties = usb_chg_props;
	psy_desc->num_properties = ARRAY_SIZE(usb_chg_props);
	psy_desc->get_property = stm32mp2_usb2phy_batt_chg_get_property;

	hsotg->batt_chg_psy_type = POWER_SUPPLY_TYPE_USB;

	usb_chg_psy_cfg.drv_data = hsotg;
	hsotg->chg_current = VBUS_CURRENT_500MA; /* Default current */
	hsotg->psy_batt_chg = devm_power_supply_register(hsotg->dev, psy_desc,
							 &usb_chg_psy_cfg);

	if (IS_ERR(hsotg->psy_batt_chg)) {
		dev_warn(hsotg->dev, "unable to register power supply\n");
		return PTR_ERR(hsotg->psy_batt_chg);
	}

	return 0;
}

int stm32mp2_usb2phy_batt_chg_det(struct dwc2_hsotg *hsotg)
{
	int ret;

	ret = stm32mp2_usb2phy_data_contact_det(hsotg);
	if (ret)
		return ret;

	ret = stm32mp2_usb2phy_primary_det(hsotg);
	if (ret)
		stm32mp2_usb2phy_secondary_det(hsotg);

	if (hsotg->chg_current != VBUS_CURRENT_500MA)
		power_supply_changed(hsotg->psy_batt_chg);

	return 0;
}

/*
 * Check the dr_mode against the module configuration and hardware
 * capabilities.
 *
 * The hardware, module, and dr_mode, can each be set to host, device,
 * or otg. Check that all these values are compatible and adjust the
 * value of dr_mode if possible.
 *
 *                      actual
 *    HW  MOD dr_mode   dr_mode
 *  ------------------------------
 *   HST  HST  any    :  HST
 *   HST  DEV  any    :  ---
 *   HST  OTG  any    :  HST
 *
 *   DEV  HST  any    :  ---
 *   DEV  DEV  any    :  DEV
 *   DEV  OTG  any    :  DEV
 *
 *   OTG  HST  any    :  HST
 *   OTG  DEV  any    :  DEV
 *   OTG  OTG  any    :  dr_mode
 */
static int dwc2_get_dr_mode(struct dwc2_hsotg *hsotg)
{
	enum usb_dr_mode mode;

	hsotg->dr_mode = usb_get_dr_mode(hsotg->dev);
	if (hsotg->dr_mode == USB_DR_MODE_UNKNOWN)
		hsotg->dr_mode = USB_DR_MODE_OTG;

	mode = hsotg->dr_mode;

	if (dwc2_hw_is_device(hsotg)) {
		if (IS_ENABLED(CONFIG_USB_DWC2_HOST)) {
			dev_err(hsotg->dev,
				"Controller does not support host mode.\n");
			return -EINVAL;
		}
		mode = USB_DR_MODE_PERIPHERAL;
	} else if (dwc2_hw_is_host(hsotg)) {
		if (IS_ENABLED(CONFIG_USB_DWC2_PERIPHERAL)) {
			dev_err(hsotg->dev,
				"Controller does not support device mode.\n");
			return -EINVAL;
		}
		mode = USB_DR_MODE_HOST;
	} else {
		if (IS_ENABLED(CONFIG_USB_DWC2_HOST))
			mode = USB_DR_MODE_HOST;
		else if (IS_ENABLED(CONFIG_USB_DWC2_PERIPHERAL))
			mode = USB_DR_MODE_PERIPHERAL;
	}

	if (mode != hsotg->dr_mode) {
		dev_warn(hsotg->dev,
			 "Configuration mismatch. dr_mode forced to %s\n",
			mode == USB_DR_MODE_HOST ? "host" : "device");

		hsotg->dr_mode = mode;
	}

	return 0;
}

static int __dwc2_lowlevel_hw_enable(struct dwc2_hsotg *hsotg)
{
	struct platform_device *pdev = to_platform_device(hsotg->dev);
	int ret;

	ret = regulator_bulk_enable(ARRAY_SIZE(hsotg->supplies),
				    hsotg->supplies);
	if (ret)
		return ret;

	if (hsotg->utmi_clk) {
		ret = clk_prepare_enable(hsotg->utmi_clk);
		if (ret)
			goto err_dis_reg;
	}

	if (hsotg->clk) {
		ret = clk_prepare_enable(hsotg->clk);
		if (ret)
			goto err_dis_utmi_clk;
	}

	if (hsotg->uphy) {
		ret = usb_phy_init(hsotg->uphy);
	} else if (hsotg->plat && hsotg->plat->phy_init) {
		ret = hsotg->plat->phy_init(pdev, hsotg->plat->phy_type);
	} else {
		ret = phy_init(hsotg->phy);
		if (ret == 0) {
			ret = phy_power_on(hsotg->phy);
			if (ret)
				phy_exit(hsotg->phy);
		}
	}

	if (ret)
		goto err_dis_clk;

	return 0;

err_dis_clk:
	if (hsotg->clk)
		clk_disable_unprepare(hsotg->clk);

err_dis_utmi_clk:
	if (hsotg->utmi_clk)
		clk_disable_unprepare(hsotg->utmi_clk);

err_dis_reg:
	regulator_bulk_disable(ARRAY_SIZE(hsotg->supplies), hsotg->supplies);

	return ret;
}

/**
 * dwc2_lowlevel_hw_enable - enable platform lowlevel hw resources
 * @hsotg: The driver state
 *
 * A wrapper for platform code responsible for controlling
 * low-level USB platform resources (phy, clock, regulators)
 */
int dwc2_lowlevel_hw_enable(struct dwc2_hsotg *hsotg)
{
	int ret = __dwc2_lowlevel_hw_enable(hsotg);

	if (ret == 0)
		hsotg->ll_hw_enabled = true;
	return ret;
}

static int __dwc2_lowlevel_hw_disable(struct dwc2_hsotg *hsotg)
{
	struct platform_device *pdev = to_platform_device(hsotg->dev);
	int ret = 0;

	if (hsotg->uphy) {
		usb_phy_shutdown(hsotg->uphy);
	} else if (hsotg->plat && hsotg->plat->phy_exit) {
		ret = hsotg->plat->phy_exit(pdev, hsotg->plat->phy_type);
	} else {
		ret = phy_power_off(hsotg->phy);
		if (ret == 0)
			ret = phy_exit(hsotg->phy);
	}
	if (ret)
		return ret;

	if (hsotg->clk)
		clk_disable_unprepare(hsotg->clk);

	if (hsotg->utmi_clk)
		clk_disable_unprepare(hsotg->utmi_clk);

	return regulator_bulk_disable(ARRAY_SIZE(hsotg->supplies), hsotg->supplies);
}

/**
 * dwc2_lowlevel_hw_disable - disable platform lowlevel hw resources
 * @hsotg: The driver state
 *
 * A wrapper for platform code responsible for controlling
 * low-level USB platform resources (phy, clock, regulators)
 */
int dwc2_lowlevel_hw_disable(struct dwc2_hsotg *hsotg)
{
	int ret = __dwc2_lowlevel_hw_disable(hsotg);

	if (ret == 0)
		hsotg->ll_hw_enabled = false;
	return ret;
}

static void dwc2_reset_control_assert(void *data)
{
	reset_control_assert(data);
}

static int dwc2_lowlevel_hw_init(struct dwc2_hsotg *hsotg)
{
	int i, ret;

	hsotg->reset = devm_reset_control_get_optional(hsotg->dev, "dwc2");
	if (IS_ERR(hsotg->reset))
		return dev_err_probe(hsotg->dev, PTR_ERR(hsotg->reset),
				     "error getting reset control\n");

	reset_control_deassert(hsotg->reset);
	ret = devm_add_action_or_reset(hsotg->dev, dwc2_reset_control_assert,
				       hsotg->reset);
	if (ret)
		return ret;

	hsotg->reset_ecc = devm_reset_control_get_optional(hsotg->dev, "dwc2-ecc");
	if (IS_ERR(hsotg->reset_ecc))
		return dev_err_probe(hsotg->dev, PTR_ERR(hsotg->reset_ecc),
				     "error getting reset control for ecc\n");

	reset_control_deassert(hsotg->reset_ecc);
	ret = devm_add_action_or_reset(hsotg->dev, dwc2_reset_control_assert,
				       hsotg->reset_ecc);
	if (ret)
		return ret;

	/*
	 * Attempt to find a generic PHY, then look for an old style
	 * USB PHY and then fall back to pdata
	 */
	hsotg->phy = devm_phy_get(hsotg->dev, "usb2-phy");
	if (IS_ERR(hsotg->phy)) {
		ret = PTR_ERR(hsotg->phy);
		switch (ret) {
		case -ENODEV:
		case -ENOSYS:
			hsotg->phy = NULL;
			break;
		default:
			return dev_err_probe(hsotg->dev, ret, "error getting phy\n");
		}
	}

	if (!hsotg->phy) {
		hsotg->uphy = devm_usb_get_phy(hsotg->dev, USB_PHY_TYPE_USB2);
		if (IS_ERR(hsotg->uphy)) {
			ret = PTR_ERR(hsotg->uphy);
			switch (ret) {
			case -ENODEV:
			case -ENXIO:
				hsotg->uphy = NULL;
				break;
			default:
				return dev_err_probe(hsotg->dev, ret, "error getting usb phy\n");
			}
		}
	}

	hsotg->plat = dev_get_platdata(hsotg->dev);

	/* Clock */
	hsotg->clk = devm_clk_get_optional(hsotg->dev, "otg");
	if (IS_ERR(hsotg->clk))
		return dev_err_probe(hsotg->dev, PTR_ERR(hsotg->clk), "cannot get otg clock\n");

	hsotg->utmi_clk = devm_clk_get_optional(hsotg->dev, "utmi");
	if (IS_ERR(hsotg->utmi_clk))
		return dev_err_probe(hsotg->dev, PTR_ERR(hsotg->utmi_clk),
				     "cannot get utmi clock\n");

	/* Regulators */
	for (i = 0; i < ARRAY_SIZE(hsotg->supplies); i++)
		hsotg->supplies[i].supply = dwc2_hsotg_supply_names[i];

	ret = devm_regulator_bulk_get(hsotg->dev, ARRAY_SIZE(hsotg->supplies),
				      hsotg->supplies);
	if (ret)
		return dev_err_probe(hsotg->dev, ret, "failed to request supplies\n");

	return 0;
}

/**
 * dwc2_driver_remove() - Called when the DWC_otg core is unregistered with the
 * DWC_otg driver
 *
 * @dev: Platform device
 *
 * This routine is called, for example, when the rmmod command is executed. The
 * device may or may not be electrically present. If it is present, the driver
 * stops device processing. Any resources used on behalf of this device are
 * freed.
 */
static void dwc2_driver_remove(struct platform_device *dev)
{
	struct dwc2_hsotg *hsotg = platform_get_drvdata(dev);
	struct dwc2_gregs_backup *gr;
	int ret = 0;

	gr = &hsotg->gr_backup;

	/* Exit Hibernation when driver is removed. */
	if (hsotg->hibernated) {
		if (gr->gotgctl & GOTGCTL_CURMODE_HOST)
			ret = dwc2_exit_hibernation(hsotg, 0, 0, 1);
		else
			ret = dwc2_exit_hibernation(hsotg, 0, 0, 0);

		if (ret)
			dev_err(hsotg->dev,
				"exit hibernation failed.\n");
	}

	/* Exit Partial Power Down when driver is removed. */
	if (hsotg->in_ppd) {
		ret = dwc2_exit_partial_power_down(hsotg, 0, true);
		if (ret)
			dev_err(hsotg->dev,
				"exit partial_power_down failed\n");
	}

	/* Exit clock gating when driver is removed. */
	if (hsotg->params.power_down == DWC2_POWER_DOWN_PARAM_NONE &&
	    hsotg->bus_suspended && !hsotg->params.no_clock_gating) {
		if (dwc2_is_device_mode(hsotg))
			dwc2_gadget_exit_clock_gating(hsotg, 0);
		else
			dwc2_host_exit_clock_gating(hsotg, 0);
	}

	dwc2_debugfs_exit(hsotg);
	if (hsotg->hcd_enabled)
		dwc2_hcd_remove(hsotg);
	if (hsotg->gadget_enabled)
		dwc2_hsotg_remove(hsotg);

	dwc2_drd_exit(hsotg);

	if (hsotg->params.activate_stm_id_vb_detection)
		regulator_disable(hsotg->usb33d);

	if (hsotg->params.activate_stm32_bvaloval_en && !hsotg->role_sw) {
		u32 ggpio = dwc2_readl(hsotg, GGPIO);

		ggpio &= ~GGPIO_STM32_OTG_GCCFG_IDPULLUP_DIS;
		ggpio &= ~GGPIO_STM32_OTG_GCCFG_VBVALOVAL;
		dwc2_writel(hsotg, ggpio, GGPIO);
	}

	if (hsotg->ll_hw_enabled)
		dwc2_lowlevel_hw_disable(hsotg);
}

/**
 * dwc2_driver_shutdown() - Called on device shutdown
 *
 * @dev: Platform device
 *
 * In specific conditions (involving usb hubs) dwc2 devices can create a
 * lot of interrupts, even to the point of overwhelming devices running
 * at low frequencies. Some devices need to do special clock handling
 * at shutdown-time which may bring the system clock below the threshold
 * of being able to handle the dwc2 interrupts. Disabling dwc2-irqs
 * prevents reboots/poweroffs from getting stuck in such cases.
 */
static void dwc2_driver_shutdown(struct platform_device *dev)
{
	struct dwc2_hsotg *hsotg = platform_get_drvdata(dev);

	if (hsotg->ll_hw_enabled) {
		dwc2_disable_global_interrupts(hsotg);
		synchronize_irq(hsotg->irq);
		dwc2_lowlevel_hw_disable(hsotg);
	}
}

/**
 * dwc2_check_core_endianness() - Returns true if core and AHB have
 * opposite endianness.
 * @hsotg:	Programming view of the DWC_otg controller.
 */
static bool dwc2_check_core_endianness(struct dwc2_hsotg *hsotg)
{
	u32 snpsid;

	snpsid = ioread32(hsotg->regs + GSNPSID);
	if ((snpsid & GSNPSID_ID_MASK) == DWC2_OTG_ID ||
	    (snpsid & GSNPSID_ID_MASK) == DWC2_FS_IOT_ID ||
	    (snpsid & GSNPSID_ID_MASK) == DWC2_HS_IOT_ID)
		return false;
	return true;
}

/**
 * dwc2_check_core_version() - Check core version
 *
 * @hsotg: Programming view of the DWC_otg controller
 *
 */
int dwc2_check_core_version(struct dwc2_hsotg *hsotg)
{
	struct dwc2_hw_params *hw = &hsotg->hw_params;

	/*
	 * Attempt to ensure this device is really a DWC_otg Controller.
	 * Read and verify the GSNPSID register contents. The value should be
	 * 0x45f4xxxx, 0x5531xxxx or 0x5532xxxx
	 */

	hw->snpsid = dwc2_readl(hsotg, GSNPSID);
	if ((hw->snpsid & GSNPSID_ID_MASK) != DWC2_OTG_ID &&
	    (hw->snpsid & GSNPSID_ID_MASK) != DWC2_FS_IOT_ID &&
	    (hw->snpsid & GSNPSID_ID_MASK) != DWC2_HS_IOT_ID) {
		dev_err(hsotg->dev, "Bad value for GSNPSID: 0x%08x\n",
			hw->snpsid);
		return -ENODEV;
	}

	dev_dbg(hsotg->dev, "Core Release: %1x.%1x%1x%1x (snpsid=%x)\n",
		hw->snpsid >> 12 & 0xf, hw->snpsid >> 8 & 0xf,
		hw->snpsid >> 4 & 0xf, hw->snpsid & 0xf, hw->snpsid);
	return 0;
}

/**
 * dwc2_driver_probe() - Called when the DWC_otg core is bound to the DWC_otg
 * driver
 *
 * @dev: Platform device
 *
 * This routine creates the driver components required to control the device
 * (core, HCD, and PCD) and initializes the device. The driver components are
 * stored in a dwc2_hsotg structure. A reference to the dwc2_hsotg is saved
 * in the device private data. This allows the driver to access the dwc2_hsotg
 * structure on subsequent calls to driver methods for this device.
 */
static int dwc2_driver_probe(struct platform_device *dev)
{
	struct device_node *node = dev->dev.of_node;
	struct dwc2_hsotg *hsotg;
	struct resource *res;
	int retval;

	hsotg = devm_kzalloc(&dev->dev, sizeof(*hsotg), GFP_KERNEL);
	if (!hsotg)
		return -ENOMEM;

	hsotg->dev = &dev->dev;

	/*
	 * Use reasonable defaults so platforms don't have to provide these.
	 */
	if (!dev->dev.dma_mask)
		dev->dev.dma_mask = &dev->dev.coherent_dma_mask;
	retval = dma_set_coherent_mask(&dev->dev, DMA_BIT_MASK(32));
	if (retval) {
		dev_err(&dev->dev, "can't set coherent DMA mask: %d\n", retval);
		return retval;
	}

	hsotg->regs = devm_platform_get_and_ioremap_resource(dev, 0, &res);
	if (IS_ERR(hsotg->regs))
		return PTR_ERR(hsotg->regs);

	dev_dbg(&dev->dev, "mapped PA %08lx to VA %p\n",
		(unsigned long)res->start, hsotg->regs);

	retval = dwc2_lowlevel_hw_init(hsotg);
	if (retval)
		return retval;

	retval = pm_runtime_set_active(&dev->dev);
	if (retval)
		return dev_err_probe(&dev->dev, retval, "Failed to activate pm runtime\n");

	retval = devm_pm_runtime_enable(&dev->dev);
	if (retval)
		return dev_err_probe(&dev->dev, retval, "Failed to enable pm runtime\n");

	/*
	 * Disable runtime PM until the device is fully initialized.
	 * This prevents the device from being suspended prematurely.
	 */
	pm_runtime_forbid(&dev->dev);

	spin_lock_init(&hsotg->lock);

	hsotg->vbus_supply = devm_regulator_get_optional(hsotg->dev, "vbus");
	if (IS_ERR(hsotg->vbus_supply)) {
		retval = PTR_ERR(hsotg->vbus_supply);
		hsotg->vbus_supply = NULL;
		if (retval != -ENODEV)
			return retval;
	}

	retval = dwc2_lowlevel_hw_enable(hsotg);
	if (retval)
		return retval;

	hsotg->needs_byte_swap = dwc2_check_core_endianness(hsotg);

	retval = dwc2_get_dr_mode(hsotg);
	if (retval)
		goto error;

	hsotg->need_phy_for_wake =
		of_property_read_bool(dev->dev.of_node,
				      "snps,need-phy-for-wake");

	/*
	 * Before performing any core related operations
	 * check core version.
	 */
	retval = dwc2_check_core_version(hsotg);
	if (retval)
		goto error;

	/*
	 * Reset before dwc2_get_hwparams() then it could get power-on real
	 * reset value form registers.
	 */
	retval = dwc2_core_reset(hsotg, false);
	if (retval)
		goto error;

	/* Detect config values from hardware */
	retval = dwc2_get_hwparams(hsotg);
	if (retval)
		goto error;

	hsotg->irq = platform_get_irq(dev, 0);
	if (hsotg->irq < 0) {
		retval = hsotg->irq;
		goto error;
	}

	dev_dbg(hsotg->dev, "registering common handler for irq%d\n",
		hsotg->irq);
	retval = devm_request_irq(hsotg->dev, hsotg->irq,
				  dwc2_handle_common_intr, IRQF_SHARED,
				  dev_name(hsotg->dev), hsotg);
	if (retval)
		goto error;

	/*
	 * For OTG cores, set the force mode bits to reflect the value
	 * of dr_mode. Force mode bits should not be touched at any
	 * other time after this.
	 */
	dwc2_force_dr_mode(hsotg);

	retval = dwc2_init_params(hsotg);
	if (retval)
		goto error;

	if (hsotg->params.activate_stm32_otgarcr_en) {
		hsotg->params.stm32_regmap = syscon_regmap_lookup_by_phandle(node, "st,syscfg");
		if (IS_ERR(hsotg->params.stm32_regmap)) {
			retval = dev_err_probe(&dev->dev, PTR_ERR(hsotg->params.stm32_regmap),
					       "no st,syscfg node found\n");
			goto error;
		}

		retval = of_property_read_u32_index(node, "st,syscfg", 1,
						    &hsotg->params.stm32_syscfg_otgarcr_reg_off);
		if (retval) {
			retval = dev_err_probe(&dev->dev, retval, "can't get otgarcr offset\n");
			goto error;
		}
		dev_vdbg(&dev->dev, "syscfg-otgarcr-reg offset 0x%x\n",
			 hsotg->params.stm32_syscfg_otgarcr_reg_off);
	}

	if (dev->dev.dma_range_map && hsotg->params.activate_stm32_otgarcr_en) {
		regmap_set_bits(hsotg->params.stm32_regmap,
				hsotg->params.stm32_syscfg_otgarcr_reg_off,
				STM32_SYSCFG_OTGARCR_OFFSET_AREN_MASK);
	}

	if (hsotg->params.activate_stm_id_vb_detection) {
		u32 ggpio;

		hsotg->usb33d = devm_regulator_get(hsotg->dev, "usb33d");
		if (IS_ERR(hsotg->usb33d)) {
			retval = PTR_ERR(hsotg->usb33d);
			dev_err_probe(hsotg->dev, retval, "failed to request usb33d supply\n");
			goto error;
		}
		retval = regulator_enable(hsotg->usb33d);
		if (retval) {
			dev_err_probe(hsotg->dev, retval, "failed to enable usb33d supply\n");
			goto error;
		}

		ggpio = dwc2_readl(hsotg, GGPIO);
		ggpio |= GGPIO_STM32_OTG_GCCFG_IDEN;
		ggpio |= GGPIO_STM32_OTG_GCCFG_VBDEN;
		dwc2_writel(hsotg, ggpio, GGPIO);

		/* ID/VBUS detection startup time */
		usleep_range(5000, 7000);
	}


	retval = dwc2_drd_init(hsotg);
	if (retval) {
		dev_err_probe(hsotg->dev, retval, "failed to initialize dual-role\n");
		goto error_init;
	}

	if (hsotg->params.activate_stm32_bvaloval_en && !hsotg->role_sw) {
		u32 ggpio = dwc2_readl(hsotg, GGPIO);

		if (hsotg->dr_mode == USB_DR_MODE_HOST) {
			ggpio &= ~GGPIO_STM32_OTG_GCCFG_IDPULLUP_DIS;
			ggpio &= ~GGPIO_STM32_OTG_GCCFG_VBVALOVAL;
		} else if (hsotg->dr_mode == USB_DR_MODE_PERIPHERAL) {
			ggpio |= GGPIO_STM32_OTG_GCCFG_IDPULLUP_DIS;
			ggpio |= GGPIO_STM32_OTG_GCCFG_VBVALOVAL;
		}
		dwc2_writel(hsotg, ggpio, GGPIO);
	}

	if (hsotg->dr_mode != USB_DR_MODE_HOST) {
		retval = dwc2_gadget_init(hsotg);
		if (retval)
			goto error_drd;
		hsotg->gadget_enabled = 1;
	}

	if (hsotg->params.stm32_has_batt_chg_det && hsotg->dr_mode != USB_DR_MODE_HOST) {
		retval = stm32mp2_usb2phy_usb_chg_psy_register(hsotg);
		if (retval)
			goto error_drd;
	}

	/*
	 * If we need PHY for wakeup we must be wakeup capable.
	 * When we have a device that can wake without the PHY we
	 * can adjust this condition.
	 */
	if (hsotg->need_phy_for_wake)
		device_set_wakeup_capable(&dev->dev, true);

	hsotg->reset_phy_on_wake =
		of_property_read_bool(dev->dev.of_node,
				      "snps,reset-phy-on-wake");
	if (hsotg->reset_phy_on_wake && !hsotg->phy) {
		dev_warn(hsotg->dev,
			 "Quirk reset-phy-on-wake only supports generic PHYs\n");
		hsotg->reset_phy_on_wake = false;
	}

	if (hsotg->dr_mode != USB_DR_MODE_PERIPHERAL) {
		retval = dwc2_hcd_init(hsotg);
		if (retval) {
			if (hsotg->gadget_enabled)
				dwc2_hsotg_remove(hsotg);
			goto error_drd;
		}
		hsotg->hcd_enabled = 1;
	}

	platform_set_drvdata(dev, hsotg);
	hsotg->hibernated = 0;

	/* Now that the hcd is fully initialized, allow runtime PM */
	pm_runtime_allow(&dev->dev);

	dwc2_debugfs_init(hsotg);

	/* Gadget code manages lowlevel hw on its own */
	if (hsotg->dr_mode == USB_DR_MODE_PERIPHERAL)
		dwc2_lowlevel_hw_disable(hsotg);

#if IS_ENABLED(CONFIG_USB_DWC2_PERIPHERAL) || \
	IS_ENABLED(CONFIG_USB_DWC2_DUAL_ROLE)
	/* Postponed adding a new gadget to the udc class driver list */
	if (hsotg->gadget_enabled) {
		retval = usb_add_gadget_udc(hsotg->dev, &hsotg->gadget);
		if (retval) {
			hsotg->gadget.udc = NULL;
			dwc2_hsotg_remove(hsotg);
			goto error_debugfs;
		}
	}
#endif /* CONFIG_USB_DWC2_PERIPHERAL || CONFIG_USB_DWC2_DUAL_ROLE */
	return 0;

#if IS_ENABLED(CONFIG_USB_DWC2_PERIPHERAL) || \
	IS_ENABLED(CONFIG_USB_DWC2_DUAL_ROLE)
error_debugfs:
	dwc2_debugfs_exit(hsotg);
	if (hsotg->hcd_enabled)
		dwc2_hcd_remove(hsotg);
#endif
error_drd:
	dwc2_drd_exit(hsotg);

error_init:
	if (hsotg->params.activate_stm_id_vb_detection)
		regulator_disable(hsotg->usb33d);
error:
	if (hsotg->ll_hw_enabled)
		dwc2_lowlevel_hw_disable(hsotg);
	return retval;
}

static int __maybe_unused dwc2_suspend(struct device *dev)
{
	struct dwc2_hsotg *dwc2 = dev_get_drvdata(dev);
	bool is_device_mode;
	bool ll_hw_enabled = dwc2->ll_hw_enabled;
	int ret = 0;

	/*
	 * The low level HW may have been left disabled from the probe,
	 * or disabled later on (e.g. gadget driver removed. Need to re-enable
	 * temporarily, to properly manage the suspend.
	 */
	if (!ll_hw_enabled) {
		ret = __dwc2_lowlevel_hw_enable(dwc2);
		if (ret)
			return ret;
	}

	is_device_mode = dwc2_is_device_mode(dwc2);

	if (is_device_mode) {
		/*
		 * Handle the case when bus has been suspended prior to platform suspend.
		 * - When the lx_state is DWC2_L2, the HOST has suspended the bus. So the
		 * device shouldn't disconnect: dwc2_hsotg_suspend() is then a no-op. It
		 * could request to resume the bus, or the HOST may request a bus resume.
		 * Still user could disable wakeup trough sysfs, so check also if it can
		 * be powered OFF.
		 * - In case the device has been disconnected, then it should be properly
		 * suspended/disconnected. Then, temporarily exit the low power mode (clock
		 * gating, ppd), so the gadget can be suspended and resumed later on.
		 */
		if (!dwc2_is_device_connected(dwc2) || dwc2_gadget_can_poweroff_phy(dwc2)) {
			if (dwc2->in_ppd) {
				ret = dwc2_exit_partial_power_down(dwc2, 0, true);
				if (ret)
					dev_err(dwc2->dev, "exit power_down failed\n");
			}

			if (dwc2->params.power_down == DWC2_POWER_DOWN_PARAM_NONE &&
			    dwc2->bus_suspended && !dwc2->params.no_clock_gating)
				dwc2_gadget_exit_clock_gating(dwc2, 0);
		}

		dwc2_hsotg_suspend(dwc2);
	}

	dwc2_drd_suspend(dwc2);

	if (dwc2->params.activate_stm_id_vb_detection) {
		unsigned long flags;
		u32 ggpio, gotgctl;

		/*
		 * Need to force the mode to the current mode to avoid Mode
		 * Mismatch Interrupt when ID detection will be disabled.
		 */
		dwc2_force_mode(dwc2, !is_device_mode);

		spin_lock_irqsave(&dwc2->lock, flags);
		gotgctl = dwc2_readl(dwc2, GOTGCTL);
		/* bypass debounce filter, enable overrides */
		gotgctl |= GOTGCTL_DBNCE_FLTR_BYPASS;
		gotgctl |= GOTGCTL_BVALOEN | GOTGCTL_AVALOEN;
		/* Force A / B session if needed */
		if (gotgctl & GOTGCTL_ASESVLD)
			gotgctl |= GOTGCTL_AVALOVAL;
		if (gotgctl & GOTGCTL_BSESVLD)
			gotgctl |= GOTGCTL_BVALOVAL;
		dwc2_writel(dwc2, gotgctl, GOTGCTL);
		spin_unlock_irqrestore(&dwc2->lock, flags);

		ggpio = dwc2_readl(dwc2, GGPIO);
		ggpio &= ~GGPIO_STM32_OTG_GCCFG_IDEN;
		ggpio &= ~GGPIO_STM32_OTG_GCCFG_VBDEN;
		dwc2_writel(dwc2, ggpio, GGPIO);

		regulator_disable(dwc2->usb33d);
	}

	/* balance temporarily re-enabled HW */
	if (!ll_hw_enabled)
		ret = __dwc2_lowlevel_hw_disable(dwc2);

	if (dwc2->ll_hw_enabled &&
	    (dwc2_gadget_can_poweroff_phy(dwc2) || dwc2_host_can_poweroff_phy(dwc2))) {
		ret = __dwc2_lowlevel_hw_disable(dwc2);
		dwc2->phy_off_for_suspend = true;
	}

	/* If HW has been kept enabled for wakeup, enable wake irq */
	if (dwc2->ll_hw_enabled && !dwc2->phy_off_for_suspend)
		enable_irq_wake(dwc2->irq);

	return ret;
}

static int __maybe_unused dwc2_resume(struct device *dev)
{
	struct dwc2_hsotg *dwc2 = dev_get_drvdata(dev);
	bool ll_hw_enabled = dwc2->ll_hw_enabled;
	int ret = 0;

	if (dwc2->ll_hw_enabled && !dwc2->phy_off_for_suspend)
		disable_irq_wake(dwc2->irq);

	if (!ll_hw_enabled) {
		ret = __dwc2_lowlevel_hw_enable(dwc2);
		if (ret)
			return ret;
	}

	if (dev->dma_range_map && dwc2->params.activate_stm32_otgarcr_en) {
		regmap_set_bits(dwc2->params.stm32_regmap,
				dwc2->params.stm32_syscfg_otgarcr_reg_off,
				STM32_SYSCFG_OTGARCR_OFFSET_AREN_MASK);
	}

	if (dwc2->phy_off_for_suspend && dwc2->ll_hw_enabled) {
		ret = __dwc2_lowlevel_hw_enable(dwc2);
		if (ret)
			return ret;
	}
	dwc2->phy_off_for_suspend = false;

	pm_runtime_disable(dev);
	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);

	if (dwc2->params.activate_stm_id_vb_detection) {
		unsigned long flags;
		u32 ggpio, gotgctl;

		ret = regulator_enable(dwc2->usb33d);
		if (ret)
			return ret;

		ggpio = dwc2_readl(dwc2, GGPIO);
		ggpio |= GGPIO_STM32_OTG_GCCFG_IDEN;
		ggpio |= GGPIO_STM32_OTG_GCCFG_VBDEN;
		dwc2_writel(dwc2, ggpio, GGPIO);

		/* ID/VBUS detection startup time */
		usleep_range(5000, 7000);

		spin_lock_irqsave(&dwc2->lock, flags);
		gotgctl = dwc2_readl(dwc2, GOTGCTL);
		gotgctl &= ~GOTGCTL_DBNCE_FLTR_BYPASS;
		gotgctl &= ~(GOTGCTL_BVALOEN | GOTGCTL_AVALOEN |
			     GOTGCTL_BVALOVAL | GOTGCTL_AVALOVAL);
		dwc2_writel(dwc2, gotgctl, GOTGCTL);
		spin_unlock_irqrestore(&dwc2->lock, flags);
	}

	if (dwc2->params.activate_stm32_bvaloval_en && !dwc2->role_sw) {
		u32 ggpio = dwc2_readl(dwc2, GGPIO);

		if (dwc2->dr_mode == USB_DR_MODE_HOST) {
			ggpio &= ~GGPIO_STM32_OTG_GCCFG_IDPULLUP_DIS;
			ggpio &= ~GGPIO_STM32_OTG_GCCFG_VBVALOVAL;
		} else if (dwc2->dr_mode == USB_DR_MODE_PERIPHERAL) {
			ggpio |= GGPIO_STM32_OTG_GCCFG_IDPULLUP_DIS;
			ggpio |= GGPIO_STM32_OTG_GCCFG_VBVALOVAL;
		}
		dwc2_writel(dwc2, ggpio, GGPIO);
	}

	if (!dwc2->role_sw) {
		/* Need to restore FORCEDEVMODE/FORCEHOSTMODE */
		dwc2_force_dr_mode(dwc2);
	} else {
		dwc2_drd_resume(dwc2);
	}

	if (dwc2_is_device_mode(dwc2))
		ret = dwc2_hsotg_resume(dwc2);

	/* balance temporarily re-enabled HW */
	if (!ll_hw_enabled)
		ret = __dwc2_lowlevel_hw_disable(dwc2);

	return ret;
}

/*
 * Only power saving mode entry is delegated to PM runtime suspend routine.
 * This way, power saving mode is entered, only after all children have
 * supended themselves.
 *
 * All the resume activities, remains in each child driver part, e.g.:
 * - gadget resume or remote wakeup IRQ, calls directly the dwc2_gadget_exit_xxx
 * - hcd port or bus resume routines, calls directly the dwc2_host_exit_xxx
 * - debugfs runtime resume routine, calls directly...
 *
 * Moving resume activities into a runtime resume routine would have a side effect
 * in gadget mode. A resume IRQ would be triggered, upon system-wide suspend entry:
 * the (unused) root hub resumes to reconfigure the port without wakeup capability
 * (as there's no device plugged), without a real need to update dwc2 configuration.
 * But, this would abort a sequence with a suspended gadget capable of doing remote
 * wakeup, e.g. bring it out of low power.
 */
static int dwc2_runtime_suspend(struct device *dev)
{
	struct dwc2_hsotg *hsotg = dev_get_drvdata(dev);
	int ret;

	dev_dbg(hsotg->dev, "%s lx_state %d\n", __func__, hsotg->lx_state);

	if (dwc2_is_device_mode(hsotg))
		ret = dwc2_gadget_enter_lp(hsotg);
	else
		ret = dwc2_host_enter_lp(hsotg);

	if (ret)
		return ret;

	/* If the core hasn't been put into power saving modes, keep PM domain active */
	if (!(hsotg->bus_suspended || hsotg->in_ppd || hsotg->hibernated))
		return -EBUSY;

	return 0;
}

static int dwc2_runtime_idle(struct device *dev)
{
	struct dwc2_hsotg *hsotg = dev_get_drvdata(dev);

	/* RPM suspend not requested by the driver */
	if (!hsotg->rpm_suspended)
		return -EBUSY;

	/* Connected device isn't suspended */
	if (dwc2_is_device_mode(hsotg)) {
		if (dwc2_is_device_connected(hsotg) && !(dwc2_readl(hsotg, DSTS) & DSTS_SUSPSTS))
			return -EBUSY;
	}

	return 0;
}

static const struct dev_pm_ops dwc2_dev_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(dwc2_suspend, dwc2_resume)
	RUNTIME_PM_OPS(dwc2_runtime_suspend, NULL, dwc2_runtime_idle)
};

static struct platform_driver dwc2_platform_driver = {
	.driver = {
		.name = dwc2_driver_name,
		.of_match_table = dwc2_of_match_table,
		.acpi_match_table = ACPI_PTR(dwc2_acpi_match),
		.pm = &dwc2_dev_pm_ops,
	},
	.probe = dwc2_driver_probe,
	.remove_new = dwc2_driver_remove,
	.shutdown = dwc2_driver_shutdown,
};

module_platform_driver(dwc2_platform_driver);
