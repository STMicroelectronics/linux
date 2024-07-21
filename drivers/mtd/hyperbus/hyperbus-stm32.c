// SPDX-License-Identifier: GPL
/*
 * Copyright (C) 2023, STMicroelectronics - All Rights Reserved
 * Author(s): Patrice Chotard <patrice.chotard@st.com> for STMicroelectronics.
 */

#include <linux/mtd/cfi.h>
#include <linux/mtd/sfdp.h>
#include <memory/stm32-omi.h>

#define STM32_AUTOSUSPEND_DELAY -1

struct stm32_hyperbus {
	struct device *dev;
	struct hyperbus_ctlr ctlr;
	struct hyperbus_device hbdev;
	struct stm32_omi *omi;
	u32 flash_freq;		/* flash max supported frequency */
	u32 real_flash_freq;	/* real flash freq = bus_freq x prescaler */
	u32 tacc;
	u32 cs;
	u32 dlyb_cr;
	bool wzl;
};

static void stm32_hyperbus_copy_from(struct hyperbus_device *hbdev, void *to,
				     unsigned long from, ssize_t len)
{
	struct stm32_hyperbus *hyperbus =
		container_of(hbdev, struct stm32_hyperbus, hbdev);
	struct stm32_omi *omi = hyperbus->omi;

	pm_runtime_resume_and_get(omi->dev);

	memcpy_fromio(to, omi->mm_base + from, len);

	pm_runtime_mark_last_busy(omi->dev);
	pm_runtime_put_autosuspend(omi->dev);
}

static void stm32_hyperbus_set_mode(struct stm32_hyperbus *hyperbus, u8 mode)
{
	struct stm32_omi *omi = hyperbus->omi;
	void __iomem *regs_base = omi->regs_base;
	u32 cr;

	if (omi->fmode == mode)
		return;

	if (omi->fmode == CR_FMODE_MM)
		stm32_omi_abort(omi);

	cr = readl_relaxed(regs_base + OSPI_CR);
	cr &= ~CR_FMODE_MASK;
	cr |= FIELD_PREP(CR_FMODE_MASK, mode);
	writel_relaxed(cr, regs_base + OSPI_CR);
	omi->fmode = mode;
}

static u16 stm32_hyperbus_read16(struct hyperbus_device *hbdev, unsigned long addr)
{
	struct stm32_hyperbus *hyperbus =
		container_of(hbdev, struct stm32_hyperbus, hbdev);
	struct stm32_omi *omi = hyperbus->omi;
	void __iomem *regs_base = omi->regs_base;
	u16 data;

	pm_runtime_resume_and_get(omi->dev);

	stm32_hyperbus_set_mode(hyperbus, CR_FMODE_INDR);
	writel(addr, regs_base + OSPI_AR);
	stm32_omi_tx_poll(omi, (u8 *)&data, 2, true);

	/* Wait end of tx in indirect mode */
	stm32_omi_wait_cmd(omi);

	dev_dbg(hyperbus->dev, "%s: read 0x%x @ 0x%lx\n",
		__func__, data, addr >> 1);

	stm32_hyperbus_set_mode(hyperbus, CR_FMODE_MM);

	pm_runtime_mark_last_busy(omi->dev);
	pm_runtime_put_autosuspend(omi->dev);

	return data;
}

static void stm32_hyperbus_write16(struct hyperbus_device *hbdev,
				  unsigned long addr, u16 data)
{
	struct stm32_hyperbus *hyperbus =
		container_of(hbdev, struct stm32_hyperbus, hbdev);
	struct stm32_omi *omi = hyperbus->omi;
	void __iomem *regs_base = omi->regs_base;

	pm_runtime_resume_and_get(omi->dev);

	stm32_hyperbus_set_mode(hyperbus, CR_FMODE_INDW);
	writel(addr, regs_base + OSPI_AR);
	stm32_omi_tx_poll(omi, (u8 *)&data, 2, false);

	/* Wait end of tx in indirect mode */
	stm32_omi_wait_cmd(omi);

	dev_dbg(hyperbus->dev, "%s: write 0x%x @ 0x%lx\n",
		__func__, data, addr >> 1);

	stm32_hyperbus_set_mode(hyperbus, CR_FMODE_MM);

	pm_runtime_mark_last_busy(omi->dev);
	pm_runtime_put_autosuspend(omi->dev);
}

static int stm32_hyperbus_check_transfert(struct stm32_omi *omi)
{
	struct stm32_hyperbus *hyperbus = platform_get_drvdata(omi->vdev);
	struct map_info *map = &hyperbus->hbdev.map;
	struct cfi_private cfi;
	int ret;

	cfi.interleave = 1;
	cfi.device_type = CFI_DEVICETYPE_X16;

	if (omi->jedec_flash) {
		ret = hyperbus_sfdp_mode_on(0, map, &cfi);
		hyperbus_sfdp_mode_off(0, map, &cfi);
	} else {
		ret = cfi_qry_mode_on(0, map, &cfi);
		cfi_qry_mode_off(0, map, &cfi);
	}

	return !ret;
}

static int stm32_hyperbus_calibrate(struct hyperbus_device *hbdev)
{
	struct stm32_hyperbus *hyperbus =
		container_of(hbdev, struct stm32_hyperbus, hbdev);
	struct stm32_omi *omi = hyperbus->omi;
	void __iomem *regs_base = omi->regs_base;
	u32 prescaler;
	u16 period_ps = 0;
	u8 window_len = 0;
	int ret;
	bool bypass_mode = false;

	prescaler = FIELD_GET(DCR2_PRESC_MASK,
			      readl(regs_base + OSPI_DCR2));

	if (hyperbus->real_flash_freq <= STM32_DLYB_FREQ_THRESHOLD) {
		bypass_mode = true;
		period_ps = NSEC_PER_SEC / (hyperbus->real_flash_freq / 1000);
	}

	ret = stm32_omi_dlyb_init(omi, bypass_mode, period_ps);
	if (ret) {
		dev_err(hyperbus->dev, "Can't enable delay block\n");
		return 0;
	}

	if (bypass_mode || prescaler)
		/* perform only RX TAP selection */
		ret = stm32_omi_dlyb_find_tap(omi, true, &window_len);
	else
		/* perform RX/TX TAP selection */
		ret = stm32_omi_dlyb_find_tap(omi, false, &window_len);

	if (ret) {
		dev_err(omi->dev, "Calibration failed\n");
		if (!bypass_mode)
			/* stop delay block when configured in lock mode */
			stm32_omi_dlyb_stop(omi);
	}

	return ret ? 0 : 1;
}

static const struct hyperbus_ops stm32_hyperbus_ops = {
	.read16 = stm32_hyperbus_read16,
	.write16 = stm32_hyperbus_write16,
	.copy_from = stm32_hyperbus_copy_from,
	.calibrate = stm32_hyperbus_calibrate,
};

static void stm32_hyperbus_init(struct stm32_hyperbus *hyperbus)
{
	struct stm32_omi *omi = hyperbus->omi;
	void __iomem *regs_base = omi->regs_base;
	unsigned long period;
	u32 cr, dcr1, hlcr, ccr;
	u32 prescaler;

	cr = CR_EN;
	cr |= FIELD_PREP(CR_CSSEL, hyperbus->cs);
	writel_relaxed(cr, regs_base + OSPI_CR);

	/* set MTYP to HyperBus memory-map mode */
	dcr1 = FIELD_PREP(DCR1_MTYP_MASK, DCR1_MTYP_HP_MEMMODE);
	/* set DEVSIZE to memory map size */
	dcr1 |= FIELD_PREP(DCR1_DEVSIZE_MASK, ffs(omi->mm_size) - 1);
	writel_relaxed(dcr1, regs_base + OSPI_DCR1);

	prescaler = DIV_ROUND_UP(omi->clk_rate, hyperbus->flash_freq) - 1;
	if (prescaler > 255)
		prescaler = 255;

	if (prescaler)
		writel_relaxed(TCR_DHQC, regs_base + OSPI_TCR);

	writel_relaxed(FIELD_PREP(DCR2_PRESC_MASK, prescaler), regs_base + OSPI_DCR2);
	hyperbus->real_flash_freq = omi->clk_rate / (prescaler + 1);

	writel_relaxed(1, regs_base + OSPI_DLR);

	/* set access time latency */
	period = NSEC_PER_SEC / hyperbus->real_flash_freq;
	hlcr = FIELD_PREP(HLCR_TACC_MASK, DIV_ROUND_UP(hyperbus->tacc, period));
	if (hyperbus->wzl)
		hlcr |= HLCR_WZL;

	writel_relaxed(hlcr, regs_base + OSPI_HLCR);

	ccr = CCR_DQSE | CCR_DDTR | CCR_ADDTR;
	ccr |= FIELD_PREP(CCR_DMODE_MASK, CCR_DMODE_8LINES);
	ccr |= FIELD_PREP(CCR_ADSIZE_MASK, CCR_ADSIZE_32BITS);
	ccr |= FIELD_PREP(CCR_ADMODE_MASK, CCR_ADMODE_8LINES);
	writel(ccr, regs_base + OSPI_CCR);

	stm32_hyperbus_set_mode(hyperbus, CR_FMODE_MM);
}

static int __maybe_unused stm32_hyperbus_suspend(struct device *dev)
{
	struct stm32_hyperbus *hyperbus = dev_get_drvdata(dev);
	struct stm32_omi *omi = hyperbus->omi;
	void __iomem *regs_base = omi->regs_base;
	u32 cr;
	int ret;

	ret = pm_runtime_resume_and_get(omi->dev);
	if (ret < 0)
		return ret;

	/* save DLYB configuration */
	stm32_omi_dlyb_get_cr(omi, &hyperbus->dlyb_cr);

	cr = readl_relaxed(regs_base + OSPI_CR);
	cr &= ~CR_EN;
	writel_relaxed(cr, regs_base + OSPI_CR);

	pm_runtime_put_sync_suspend(omi->dev);

	pinctrl_pm_select_sleep_state(dev);

	return pm_runtime_force_suspend(omi->dev);
}

static int __maybe_unused stm32_hyperbus_resume(struct device *dev)
{
	struct stm32_hyperbus *hyperbus = dev_get_drvdata(dev);
	struct stm32_omi *omi = hyperbus->omi;
	int ret;

	ret = pm_runtime_force_resume(omi->dev);
	if (ret < 0)
		return ret;

	pinctrl_pm_select_default_state(dev);

	ret = pm_runtime_resume_and_get(omi->dev);
	if (ret < 0)
		return ret;

	/* dlyb may be restarted by bootloarder stage, so ensure it's stopped */
	stm32_omi_dlyb_stop(omi);

	stm32_hyperbus_init(hyperbus);

	/* restore DLYB configuration */
	ret = stm32_omi_dlyb_set_cr(omi, hyperbus->dlyb_cr);
	if (ret)
		return ret;

	pm_runtime_mark_last_busy(omi->dev);
	pm_runtime_put_autosuspend(omi->dev);

	return 0;
}

static const struct dev_pm_ops stm32_hyperbus_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(stm32_hyperbus_suspend, stm32_hyperbus_resume)
};

static int stm32_hyperbus_probe(struct platform_device *pdev)
{
	struct device *parent = pdev->dev.parent;
	struct device *dev = &pdev->dev;
	struct stm32_hyperbus *hyperbus;
	struct stm32_omi *omi = dev_get_drvdata(parent);
	struct device_node *flash;
	u32 value;
	int ret;

	hyperbus = devm_kzalloc(dev, sizeof(*hyperbus), GFP_KERNEL);
	if (!hyperbus)
		return -ENOMEM;

	hyperbus->omi = omi;
	hyperbus->dev = dev;
	platform_set_drvdata(pdev, hyperbus);

	omi->check_transfer = stm32_hyperbus_check_transfert;

	/* mandatory for HyperFlash */
	if (!omi->mm_size) {
		dev_err(dev, "Memory-map region not found\n");
		return -EINVAL;
	}

	/* mandatory for HyperFlash */
	if (!omi->dlyb_base) {
		dev_err(dev, "Incorrect delay block base address\n");
		return -EINVAL;
	}

	pm_runtime_enable(omi->dev);
	pm_runtime_set_autosuspend_delay(omi->dev, STM32_AUTOSUSPEND_DELAY);
	pm_runtime_use_autosuspend(omi->dev);

	ret = pm_runtime_resume_and_get(omi->dev);
	if (ret < 0)
		goto err_pm_disable;

	if (omi->rstc) {
		reset_control_assert(omi->rstc);
		udelay(2);
		reset_control_deassert(omi->rstc);
	}

	flash = of_get_next_child(parent->of_node, NULL);
	if (!flash) {
		dev_warn(&pdev->dev, "No flash node found\n");
		goto err_pm_resume;
	}

	ret = of_property_read_u32(flash, "reg", &hyperbus->cs);
	if (ret) {
		dev_err(&pdev->dev, "Can't find reg property\n");
		goto err_pm_resume;
	}

	ret = of_property_read_u32(flash, "st,max-frequency", &value);
	if (ret) {
		dev_err(&pdev->dev, "Can't find st,max-frequency property\n");
		goto err_pm_resume;
	}
	hyperbus->flash_freq = value;

	/* optional */
	ret = of_property_read_u32(flash, "st,tacc-ns", &hyperbus->tacc);
	if (ret)
		hyperbus->tacc = 0;

	hyperbus->wzl = of_property_read_bool(flash, "st,wzl");

	hyperbus->hbdev.map.size = omi->mm_size;
	hyperbus->hbdev.map.virt = omi->mm_base;

	hyperbus->ctlr.dev = dev;
	hyperbus->ctlr.ops = &stm32_hyperbus_ops;
	hyperbus->hbdev.ctlr = &hyperbus->ctlr;
	hyperbus->hbdev.np = of_get_next_child(parent->of_node, NULL);

	stm32_hyperbus_init(hyperbus);

	ret = hyperbus_register_device(&hyperbus->hbdev);
	if (ret) {
		/* disable ospi */
		writel_relaxed(0, omi->regs_base + OSPI_CR);
		goto err_pm_resume;
	}

	pm_runtime_mark_last_busy(omi->dev);
	pm_runtime_put_autosuspend(omi->dev);

	return ret;

err_pm_resume:
	pm_runtime_put_sync_suspend(omi->dev);

err_pm_disable:
	pm_runtime_force_suspend(omi->dev);

	return ret;
}

static int stm32_hyperbus_remove(struct platform_device *pdev)
{
	struct stm32_hyperbus *hyperbus = platform_get_drvdata(pdev);
	struct stm32_omi *omi = hyperbus->omi;
	int ret;

	ret = pm_runtime_resume_and_get(omi->dev);
	if (ret < 0)
		return ret;

	hyperbus_unregister_device(&hyperbus->hbdev);
	writel_relaxed(0, &omi->regs_base + OSPI_CR);
	stm32_omi_dlyb_stop(omi);

	pm_runtime_put_sync_suspend(omi->dev);
	pm_runtime_force_suspend(omi->dev);

	return 0;
}

static struct platform_driver stm32_hyperbus_driver = {
	.probe	= stm32_hyperbus_probe,
	.remove	= stm32_hyperbus_remove,
	.driver	= {
		.name = "stm32-hyperflash",
		.pm = &stm32_hyperbus_pm_ops,
	},
};
module_platform_driver(stm32_hyperbus_driver);

MODULE_DESCRIPTION("STMicroelectronics STM32 HyperBus driver");
MODULE_LICENSE("GPL");
