// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2025, STMicroelectronics - All Rights Reserved
 */

#include <linux/arm-smccc.h>
#include <linux/clk-provider.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>

#define CLK_STM32MP2_CA35SS_POLLING_DELAY_US			100

#define STM32_SIP_CA35SS_CLK					0x82000001

/* System Monitor Call (SMC) */
enum clk_stm32mp2_ca35ss_function {
	CLK_STM32MP2_CA35SS_NO_FUNCTION,
	CLK_STM32MP2_CA35SS_SET_RATE,
	CLK_STM32MP2_CA35SS_SET_RATE_STATUS,
	CLK_STM32MP2_CA35SS_RECALC_RATE,
	CLK_STM32MP2_CA35SS_ROUND_RATE,
	CLK_STM32MP2_CA35SS_NB_FUNCTION,
};

#define STM32_SMC_OK			0x00000000U
#define STM32_SMC_NOT_SUPPORTED		0xFFFFFFFFU
#define STM32_SMC_FAILED		0xFFFFFFFEU
#define STM32_SMC_INVALID_PARAMS	0xFFFFFFFDU
#define STM32_SMC_ON_GOING		0xFFFFFFFCU
#define STM32_SMC_NO_PERM		0xFFFFFFFBU

/**
 * smc_round_rate - Find the closest applicable rate
 * @rate: The rate to round.
 * @rounded_rate: Pointer to store the rounded rate (if status is STM32_SMC_OK,
 * unchanged otherwise)
 *
 * This function performs a Secure Monitor Call (SMC) to get the rounded
 * rate.
 *
 * Return: The status or result of the SMC call.
 */
static u32 smc_round_rate(u32 rate, u32 *rounded_rate)
{
	struct arm_smccc_res smc_res;

	arm_smccc_smc(STM32_SIP_CA35SS_CLK, CLK_STM32MP2_CA35SS_ROUND_RATE,
		      rate, 0, 0, 0, 0, 0, &smc_res);

	if (smc_res.a0 == STM32_SMC_OK)
		*rounded_rate = smc_res.a1;

	return smc_res.a0;
}

/**
 * smc_recalc_rate - Recalculate the rate using SMC call
 * @rate: Pointer to store the recalculated rate (if status is STM32_SMC_OK,
 * unchanged otherwise)
 *
 * This function performs a Secure Monitor Call (SMC) to get the recalculated
 * rate.
 *
 * Return: The status or result of the SMC call.
 */
static u32 smc_recalc_rate(u32 *rate)
{
	struct arm_smccc_res smc_res;

	arm_smccc_smc(STM32_SIP_CA35SS_CLK, CLK_STM32MP2_CA35SS_RECALC_RATE,
		      0, 0, 0, 0, 0, 0, &smc_res);

	if (smc_res.a0 == STM32_SMC_OK)
		*rate = smc_res.a1;

	return smc_res.a0;
}

/**
 * smc_set_rate - Set rate using SMC call
 * @target_rate: The target rate to set.
 *
 * This function performs a Secure Monitor Call (SMC) to set the clock rate to
 * the closest rate possible to @target_rate. This is done asynchronously; see
 * smc_set_rate_status().
 *
 * Return: The status or result of the SMC call.
 */
static u32 smc_set_rate(u32 target_rate)
{
	struct arm_smccc_res smc_res;

	arm_smccc_smc(STM32_SIP_CA35SS_CLK, CLK_STM32MP2_CA35SS_SET_RATE,
		      target_rate, 0, 0, 0, 0, 0, &smc_res);

	return smc_res.a0;
}

/**
 * smc_set_rate_status - Get status from set rate (async.) using SMC call
 *
 * This function performs a Secure Monitor Call (SMC) to get the status from the
 * asynchronous set rate SMC.
 *
 * Return: The status or result of the SMC call.
 */
static u32 smc_set_rate_status(void)
{
	struct arm_smccc_res smc_res;

	arm_smccc_smc(STM32_SIP_CA35SS_CLK,
		      CLK_STM32MP2_CA35SS_SET_RATE_STATUS, 0, 0, 0, 0,
		      0, 0, &smc_res);

	return smc_res.a0;
}

struct clk_stm32mp2_ca35ss {
	struct clk_hw hw;
	void __iomem *base;
};

static int clk_stm32mp2_ca35ss_convert_status(u32 status)
{
	switch (status) {
	case STM32_SMC_OK:
		return 0;
	case STM32_SMC_NOT_SUPPORTED:
		return -EOPNOTSUPP;
	case STM32_SMC_INVALID_PARAMS:
		return -EINVAL;
	default:
		return -EPERM;
	}
}

static unsigned long clk_stm32mp2_ca35ss_recalc_rate(struct clk_hw *hw,
						     unsigned long parent_rate)
{
	u32 rate = 0;
	u32 status;

	status = smc_recalc_rate(&rate);
	if (status != STM32_SMC_OK)
		return 0;

	return rate;
}

static long clk_stm32mp2_ca35ss_round_rate(struct clk_hw *hw,
					   unsigned long target_rate,
					   unsigned long *parent_rate)
{
	u32 rounded_rate, status;

	status = smc_round_rate(target_rate, &rounded_rate);

	if (status != STM32_SMC_OK)
		return clk_stm32mp2_ca35ss_convert_status(status);

	return (long) rounded_rate;
}

static int clk_stm32mp2_ca35ss_set_rate(struct clk_hw *hw,
					unsigned long target_rate,
					unsigned long parent_rate)
{
	u32 status;

	/* CPU frequency scaling can ONLY be done by ARM-Trusted-Firmware */
	status = smc_set_rate(target_rate);

	while (status == STM32_SMC_ON_GOING) {
		udelay(CLK_STM32MP2_CA35SS_POLLING_DELAY_US);
		status = smc_set_rate_status();
	}

	return clk_stm32mp2_ca35ss_convert_status(status);
}

static const struct clk_ops clk_stm32mp2_ca35ss_ops = {
	.set_rate = clk_stm32mp2_ca35ss_set_rate,
	.round_rate = clk_stm32mp2_ca35ss_round_rate,
	.recalc_rate = clk_stm32mp2_ca35ss_recalc_rate,
};

static int clk_stm32mp2_ca35ss_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct clk_init_data init = { NULL };
	struct clk_hw *hw;
	u32 status, rate;
	int ret;

	status = smc_recalc_rate(&rate);
	if (status != STM32_SMC_OK) {
		dev_err(dev, "Failed SMC call\n");
		return -EPERM;
	}

	hw = devm_kzalloc(dev, sizeof(*hw), GFP_KERNEL);
	if (!hw)
		return -ENOMEM;

	init.name = "clk-stm32mp2-ca35ss";
	init.ops = &clk_stm32mp2_ca35ss_ops;
	hw->init = &init;

	ret = devm_clk_hw_register(dev, hw);
	if (ret) {
		dev_err(dev, "Failed to register clock hardware\n");
		return ret;
	}

	ret = devm_of_clk_add_hw_provider(dev, of_clk_hw_simple_get, hw);
	if (ret) {
		dev_err(dev, "Failed to add clock provider\n");
		clk_hw_unregister(hw);
		return ret;
	}

	return 0;
}

static const struct of_device_id clk_stm32mp2_ca35ss_of_match[] = {
	{ .compatible = "st,stm32mp2-ca35ss-clk", },
	{ /* sentinel */ }
};

static struct platform_driver clk_stm32mp2_ca35ss_driver = {
	.driver = {
		.name = "clk-stm32mp2-ca35ss",
		.of_match_table = clk_stm32mp2_ca35ss_of_match,
	},
	.probe = clk_stm32mp2_ca35ss_probe,
};

module_platform_driver(clk_stm32mp2_ca35ss_driver);

MODULE_DEVICE_TABLE(of, clk_stm32mp2_ca35ss_of_match);
MODULE_DESCRIPTION("STMicroelectronics STM32MP2 CA35SS clock driver");
MODULE_AUTHOR("Theo GOUREAU <theo.goureau@foss.st.com>");
MODULE_LICENSE("GPL");
