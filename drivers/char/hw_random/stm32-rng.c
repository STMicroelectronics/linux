// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (c) 2015, Daniel Thompson
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/hw_random.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/pm_runtime.h>
#include <linux/reset.h>
#include <linux/slab.h>

#define RNG_CR		0x00
#define RNG_CR_RNGEN	BIT(2)
#define RNG_CR_CED	BIT(5)
#define RNG_CR_CONDRST	BIT(30)
#define RNG_CR_CONFLOCK	BIT(31)

#define RNG_SR		0x04
#define RNG_SR_SEIS	BIT(6)
#define RNG_SR_CEIS	BIT(5)
#define RNG_SR_DRDY	BIT(0)

#define RNG_DR		0x08

#define RNG_NIST_CONFIG_A	0x0F00D00
#define RNG_NIST_CONFIG_B	0x1801000
#define RNG_NIST_CONFIG_MASK	GENMASK(25, 8)

#define RNG_MAX_NOISE_CLK_FREQ	3000000

struct stm32_rng_data {
	bool	has_cond_reset;
};

struct stm32_rng_private {
	struct hwrng rng;
	void __iomem *base;
	struct clk *clk;
	struct reset_control *rst;
	const struct stm32_rng_data *data;
	u32 pm_cr;
	bool ced;
};

static int stm32_rng_read(struct hwrng *rng, void *data, size_t max, bool wait)
{
	struct stm32_rng_private *priv =
	    container_of(rng, struct stm32_rng_private, rng);
	u32 sr;
	int retval = 0;

	pm_runtime_get_sync((struct device *) priv->rng.priv);

	while (max > sizeof(u32)) {
		sr = readl_relaxed(priv->base + RNG_SR);
		/* Manage timeout which is based on timer and take */
		/* care of initial delay time when enabling rng	*/
		if (!sr && wait) {
			retval = readl_relaxed_poll_timeout_atomic(priv->base
								   + RNG_SR,
								   sr, sr,
								   10, 50000);
			if (retval)
				dev_err((struct device *)priv->rng.priv,
					"%s: timeout %x!\n", __func__, sr);
		}

		/* If error detected or data not ready... */
		if (sr != RNG_SR_DRDY) {
			if (WARN_ONCE(sr & (RNG_SR_SEIS | RNG_SR_CEIS),
					"bad RNG status - %x\n", sr))
				writel_relaxed(0, priv->base + RNG_SR);
			break;
		}

		*(u32 *)data = readl_relaxed(priv->base + RNG_DR);

		retval += sizeof(u32);
		data += sizeof(u32);
		max -= sizeof(u32);
	}

	pm_runtime_mark_last_busy((struct device *) priv->rng.priv);
	pm_runtime_put_sync_autosuspend((struct device *) priv->rng.priv);

	return retval || !wait ? retval : -EIO;
}

static uint stm32_rng_clock_freq_restrain(struct hwrng *rng)
{
	struct stm32_rng_private *priv =
	    container_of(rng, struct stm32_rng_private, rng);
	unsigned long clock_rate = 0;
	uint clock_div = 0;

	clock_rate = clk_get_rate(priv->clk);

	/*
	 * Get the exponent to apply on the CLKDIV field in RNG_CR register
	 * No need to handle the case when clock-div > 0xF as it is physically
	 * impossible
	 */
	while ((clock_rate >> clock_div) > RNG_MAX_NOISE_CLK_FREQ)
		clock_div++;

	pr_debug("RNG clk rate : %lu\n", clk_get_rate(priv->clk) >> clock_div);

	return clock_div;
}

static int stm32_rng_init(struct hwrng *rng)
{
	struct stm32_rng_private *priv =
	    container_of(rng, struct stm32_rng_private, rng);
	int err;
	u32 reg;

	err = clk_prepare_enable(priv->clk);
	if (err)
		return err;

	reg = readl_relaxed(priv->base + RNG_CR);

	if (!priv->ced)
		reg |= RNG_CR_CED;

	if (priv->data->has_cond_reset) {
		uint clock_div = stm32_rng_clock_freq_restrain(rng);

		reg &= ~RNG_NIST_CONFIG_MASK;
		reg |= RNG_CR_CONDRST | RNG_NIST_CONFIG_B | clock_div;
		writel_relaxed(reg, priv->base + RNG_CR);
		reg &= ~RNG_CR_CONDRST;
		reg |= RNG_CR_CONFLOCK;
		writel_relaxed(reg, priv->base + RNG_CR);
		err = readl_relaxed_poll_timeout_atomic(priv->base + RNG_CR, reg,
							(!(reg & RNG_CR_CONDRST)),
							10, 50000);
		if (err) {
			dev_err((struct device *)priv->rng.priv,
				"%s: timeout %x!\n", __func__, reg);
			return -EINVAL;
		}
	}

	/* clear error indicators */
	writel_relaxed(0, priv->base + RNG_SR);

	reg |= RNG_CR_RNGEN;
	writel_relaxed(reg, priv->base + RNG_CR);

	err = readl_relaxed_poll_timeout_atomic(priv->base + RNG_SR, reg,
						reg & RNG_SR_DRDY,
						10, 100000);
	if (err | (reg & ~RNG_SR_DRDY)) {
		clk_disable_unprepare(priv->clk);
		dev_err((struct device *)priv->rng.priv,
			"%s: timeout:%x SR: %x!\n", __func__, err, reg);
		return -EINVAL;
	}

	return 0;
}

static int stm32_rng_remove(struct platform_device *ofdev)
{
	pm_runtime_disable(&ofdev->dev);

	return 0;
}

#ifdef CONFIG_PM
static int stm32_rng_runtime_suspend(struct device *dev)
{
	u32 reg;
	struct stm32_rng_private *priv = dev_get_drvdata(dev);

	reg = readl_relaxed(priv->base + RNG_CR);
	reg &= ~RNG_CR_RNGEN;
	priv->pm_cr = reg;
	writel_relaxed(reg, priv->base + RNG_CR);
	clk_disable_unprepare(priv->clk);

	return 0;
}

static int stm32_rng_runtime_resume(struct device *dev)
{
	u32 reg;
	struct stm32_rng_private *priv = dev_get_drvdata(dev);

	clk_prepare_enable(priv->clk);

	/* Clean error indications */
	writel_relaxed(0, priv->base + RNG_SR);

	if (priv->data->has_cond_reset) {
		/*
		 * Correct configuration in bits [29:4] must be set in the same
		 * access that set RNG_CR_CONDRST bit. Else config setting is
		 * not taken into account. CONFIGLOCK bit must also be unset but
		 * it is not handled at the moment.
		 */
		writel_relaxed(priv->pm_cr | RNG_CR_CONDRST, priv->base + RNG_CR);

		reg = readl_relaxed(priv->base + RNG_CR);
		reg |= RNG_CR_RNGEN;
		reg &= ~RNG_CR_CONDRST;
		writel_relaxed(reg, priv->base + RNG_CR);
	} else {
		reg = readl_relaxed(priv->base + RNG_CR);
		reg |= RNG_CR_RNGEN;
		writel_relaxed(reg, priv->base + RNG_CR);
	}

	return 0;
}
#endif

static const struct dev_pm_ops stm32_rng_pm_ops = {
	SET_RUNTIME_PM_OPS(stm32_rng_runtime_suspend,
			   stm32_rng_runtime_resume, NULL)
	SET_SYSTEM_SLEEP_PM_OPS(pm_runtime_force_suspend,
				pm_runtime_force_resume)
};

static const struct stm32_rng_data stm32mp13_rng_data = {
	.has_cond_reset = true,
};

static const struct stm32_rng_data stm32_rng_data = {
	.has_cond_reset = false,
};

static const struct of_device_id stm32_rng_match[] = {
	{
		.compatible = "st,stm32mp13-rng",
		.data = &stm32mp13_rng_data,
	},
	{
		.compatible = "st,stm32-rng",
		.data = &stm32_rng_data,
	},
	{},
};
MODULE_DEVICE_TABLE(of, stm32_rng_match);

static int stm32_rng_probe(struct platform_device *ofdev)
{
	const struct of_device_id *match;
	struct device *dev = &ofdev->dev;
	struct device_node *np = ofdev->dev.of_node;
	struct stm32_rng_private *priv;
	struct resource res;
	int err;

	priv = devm_kzalloc(dev, sizeof(struct stm32_rng_private), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	err = of_address_to_resource(np, 0, &res);
	if (err)
		return err;

	priv->base = devm_ioremap_resource(dev, &res);
	if (IS_ERR(priv->base))
		return PTR_ERR(priv->base);

	priv->clk = devm_clk_get(&ofdev->dev, NULL);
	if (IS_ERR(priv->clk))
		return PTR_ERR(priv->clk);

	priv->rst = devm_reset_control_get(&ofdev->dev, NULL);
	if (!IS_ERR(priv->rst)) {
		reset_control_assert(priv->rst);
		udelay(2);
		reset_control_deassert(priv->rst);
	}

	priv->ced = of_property_read_bool(np, "clock-error-detect");

	match = of_match_device(of_match_ptr(stm32_rng_match), dev);
	if (!match) {
		dev_err(dev, "no compatible OF match\n");
		return -EINVAL;
	}

	priv->data = match->data;

	dev_set_drvdata(dev, priv);

	priv->rng.name = dev_driver_string(dev);
	priv->rng.init = stm32_rng_init;
	priv->rng.read = stm32_rng_read;
	priv->rng.priv = (unsigned long) dev;
	priv->rng.quality = 900;

	pm_runtime_set_autosuspend_delay(dev, 100);
	pm_runtime_use_autosuspend(dev);
	pm_runtime_enable(dev);

	return devm_hwrng_register(dev, &priv->rng);
}

static struct platform_driver stm32_rng_driver = {
	.driver = {
		.name = "stm32-rng",
		.pm = &stm32_rng_pm_ops,
		.of_match_table = stm32_rng_match,
	},
	.probe = stm32_rng_probe,
	.remove = stm32_rng_remove,
};

module_platform_driver(stm32_rng_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Daniel Thompson <daniel.thompson@linaro.org>");
MODULE_DESCRIPTION("STMicroelectronics STM32 RNG device driver");
