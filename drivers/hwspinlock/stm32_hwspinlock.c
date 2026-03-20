// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) STMicroelectronics 2018-2026
 * Author: Benjamin Gaignard <benjamin.gaignard@st.com> for STMicroelectronics.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/hwspinlock/provider.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>

#define STM32_MUTEX_COREID		1
#define STM32_MUTEX_COREID_MASK		GENMASK(11, 8)
#define STM32_MUTEX_PROCID_MASK		GENMASK(7, 0)
#define STM32_MUTEX_LOCK_BIT		BIT(31)
#define STM32_MUTEX_NUM_LOCKS		32

/*
 * While the peripheral has 8 bit for ProcID, currently there are no use cases
 * that require more than 2 values for ProcID.
 * Supporting a wide range of ProcID impacts the memory allocated by this
 * driver.
 * Let's limit the memory use by restricting the max value for ProcID.
 */
#define STM32_MUTEX_NUM_PROCID		3

#define STM32_MUTEX_PROCID(id)		((id) / STM32_MUTEX_NUM_LOCKS)
#define STM32_MUTEX_REG_RX_OFFSET(id)	(4 * ((id) % STM32_MUTEX_NUM_LOCKS))

struct stm32_hwspinlock {
	void __iomem *base;
	struct clk *clk;
};

static int stm32_hwspinlock_trylock(struct hwspinlock *lock)
{
	struct stm32_hwspinlock *hw = hwspin_lock_get_priv(lock);
	int id = hwlock_to_id(lock);
	void __iomem *lock_addr;
	u32 val, status;

	lock_addr = hw->base + STM32_MUTEX_REG_RX_OFFSET(id);
	val = STM32_MUTEX_LOCK_BIT |
	      FIELD_PREP(STM32_MUTEX_COREID_MASK, STM32_MUTEX_COREID) |
	      FIELD_PREP(STM32_MUTEX_PROCID_MASK, STM32_MUTEX_PROCID(id));

	writel(val, lock_addr);
	status = readl(lock_addr);

	return status == val;
}

static void stm32_hwspinlock_unlock(struct hwspinlock *lock)
{
	struct stm32_hwspinlock *hw = hwspin_lock_get_priv(lock);
	int id = hwlock_to_id(lock);
	void __iomem *lock_addr;
	u32 val;

	lock_addr = hw->base + STM32_MUTEX_REG_RX_OFFSET(id);
	val = FIELD_PREP(STM32_MUTEX_COREID_MASK, STM32_MUTEX_COREID) |
	      FIELD_PREP(STM32_MUTEX_PROCID_MASK, STM32_MUTEX_PROCID(id));

	writel(val, lock_addr);
}

static void stm32_hwspinlock_relax(struct hwspinlock *lock)
{
	ndelay(50);
}

static int stm32_hwspinlock_xlate(const struct of_phandle_args *hwlock_spec)
{
	u32 arg0;

	if (WARN_ON(hwlock_spec->args_count != 2))
		return -EINVAL;

	arg0 = hwlock_spec->args[0];

	if (arg0 >= STM32_MUTEX_NUM_LOCKS)
		return -EINVAL;

	return arg0 + STM32_MUTEX_NUM_LOCKS * hwlock_spec->args[1];
}

static void *stm32_hwspinlock_init_priv(int local_id, void *init_data)
{
	return init_data;
}

static const struct hwspinlock_ops stm32_hwspinlock_ops = {
	.trylock	= stm32_hwspinlock_trylock,
	.unlock		= stm32_hwspinlock_unlock,
	.relax		= stm32_hwspinlock_relax,
	.xlate		= stm32_hwspinlock_xlate,
	.init_priv	= stm32_hwspinlock_init_priv,
};

static void stm32_hwspinlock_disable_clk(void *data)
{
	struct platform_device *pdev = data;
	struct stm32_hwspinlock *hw = platform_get_drvdata(pdev);
	struct device *dev = &pdev->dev;

	pm_runtime_get_sync(dev);
	pm_runtime_disable(dev);
	pm_runtime_set_suspended(dev);
	pm_runtime_put_noidle(dev);

	clk_disable_unprepare(hw->clk);
}

static int stm32_hwspinlock_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct stm32_hwspinlock *hw;
	int ret, num_locks;

	hw = devm_kzalloc(dev, sizeof(*hw), GFP_KERNEL);
	if (!hw)
		return -ENOMEM;

	hw->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(hw->base))
		return PTR_ERR(hw->base);

	hw->clk = devm_clk_get(dev, "hsem");
	if (IS_ERR(hw->clk))
		return PTR_ERR(hw->clk);

	ret = clk_prepare_enable(hw->clk);
	if (ret) {
		dev_err(dev, "Failed to prepare_enable clock\n");
		return ret;
	}

	platform_set_drvdata(pdev, hw);

	pm_runtime_get_noresume(dev);
	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);
	pm_runtime_put(dev);

	ret = devm_add_action_or_reset(dev, stm32_hwspinlock_disable_clk, pdev);
	if (ret) {
		dev_err(dev, "Failed to register action\n");
		return ret;
	}

	num_locks = STM32_MUTEX_NUM_LOCKS * STM32_MUTEX_NUM_PROCID;
	ret = devm_hwspin_lock_register_errno(dev, &stm32_hwspinlock_ops,
					      0, num_locks, hw);

	if (ret)
		dev_err(dev, "Failed to register hwspinlock\n");

	return ret;
}

static int __maybe_unused stm32_hwspinlock_runtime_suspend(struct device *dev)
{
	struct stm32_hwspinlock *hw = dev_get_drvdata(dev);

	clk_disable_unprepare(hw->clk);

	return 0;
}

static int __maybe_unused stm32_hwspinlock_runtime_resume(struct device *dev)
{
	struct stm32_hwspinlock *hw = dev_get_drvdata(dev);

	clk_prepare_enable(hw->clk);

	return 0;
}

static const struct dev_pm_ops stm32_hwspinlock_pm_ops = {
	SET_RUNTIME_PM_OPS(stm32_hwspinlock_runtime_suspend,
			   stm32_hwspinlock_runtime_resume,
			   NULL)
};

static const struct of_device_id stm32_hwpinlock_ids[] = {
	{ .compatible = "st,stm32-hwspinlock", },
	{},
};
MODULE_DEVICE_TABLE(of, stm32_hwpinlock_ids);

static struct platform_driver stm32_hwspinlock_driver = {
	.probe		= stm32_hwspinlock_probe,
	.driver		= {
		.name	= "stm32_hwspinlock",
		.of_match_table = stm32_hwpinlock_ids,
		.pm	= &stm32_hwspinlock_pm_ops,
	},
};

static int __init stm32_hwspinlock_init(void)
{
	return platform_driver_register(&stm32_hwspinlock_driver);
}
/* board init code might need to reserve hwspinlocks for predefined purposes */
postcore_initcall(stm32_hwspinlock_init);

static void __exit stm32_hwspinlock_exit(void)
{
	platform_driver_unregister(&stm32_hwspinlock_driver);
}
module_exit(stm32_hwspinlock_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Hardware spinlock driver for STM32 SoCs");
MODULE_AUTHOR("Benjamin Gaignard <benjamin.gaignard@st.com>");
