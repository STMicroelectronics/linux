// SPDX-License-Identifier: GPL-2.0-only
/*
 * STMicroelectronics USB-C PD driver
 *
 * Copyright (C) 2026 STMicroelectronics
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/devm-helpers.h>
#include <linux/i2c.h>
#include <linux/log2.h>
#include <linux/nvmem-consumer.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>
#include <linux/reset.h>
#include <linux/usb/tcpm.h>
#include "st_tcpp.h"

#define UCPD_CFGR1 0
#define UCPD_EN          BIT(31)
#define UCPD_RXORDSETEN  GENMASK(28, 20)
#define UCPD_PSC_UCPDCLK GENMASK(19, 17)
#define UCPD_TRANSWIN    GENMASK(15, 11)
#define UCPD_IFRGAP      GENMASK(10, 6)
#define UCPD_HBITCLKDIV  GENMASK(5, 0)

#define UCPD_CFGR2 0x4
#define UCPD_WUPEN BIT(3)

#define UCPD_CFGR3 0x8
#define TRIM_CC1_RD GENMASK(3, 0)
#define TRIM_CC1_RP GENMASK(12, 9)
#define TRIM_CC2_RD GENMASK(19, 16)
#define TRIM_CC2_RP GENMASK(28, 25)

#define UCPD_CR  0xc
#define UCPD_PHYRXEN  BIT(5)
#define UCPD_ANASUBMODE GENMASK(8, 7)
#define UCPD_ANAMODE  BIT(9)
#define UCPD_CCENABLE GENMASK(11, 10)
#define UCPD_RDCH     BIT(18)
#define UCPD_CC1TCDIS BIT(20)
#define UCPD_CC2TCDIS BIT(21)

#define UCPD_IMR 0x10
#define UCPD_TXISIE BIT(0)
#define TXMSGDISCIE BIT(1)
#define TXMSGSENTIE BIT(2)
#define TXMSGABTIE  BIT(3)
#define HRSTDISCIE  BIT(4)
#define HRSTSENTIE  BIT(5)
#define TXUNDIE     BIT(6)
#define RXNEIE      BIT(8)
#define RXORDDETIE  BIT(9)
#define RXHRSTDETIE BIT(10)
#define RXOVRIE     BIT(11)
#define RXMSGENDIE  BIT(12)
#define TYPECEVT1IE BIT(14)
#define TYPECEVT2IE BIT(15)

#define UCPD_SR  0x14
#define TYPECEVT1  BIT(14)
#define TYPECEVT2  BIT(15)
#define TYPEC_VSTATE_CC1 GENMASK(17, 16)
#define TYPEC_VSTATE_CC2 GENMASK(19, 18)

#define UCPD_ICR 0x18
#define TYPECEVT1CF BIT(14)
#define TYPECEVT2CF BIT(15)

struct stm32_ucpd {
	void __iomem *base;
	struct device *dev;
	struct device *tcpp_dev;
	struct tcpm_port *tcpm_port;
	struct tcpc_dev tcpc_dev;
	struct reset_control *reset;
	struct clk_bulk_data *clks;
	int num_clks;
	unsigned long ker_clk_rate;
	struct regulator *vdd;
	struct regulator *vconn;
	int ucpd_irq;
	enum typec_cc_status cc;
	enum typec_cc_polarity polarity;
	bool vbus_on;
	bool charge_on;
	bool vconn_on;
	bool vconn_restore;
	struct gpio_desc *vconn_cc1_gpio;
	struct gpio_desc *vconn_cc2_gpio;
	struct {
		u8 cc1_rd;
		u8 cc2_rd;
		u8 cc1_3a;
		u8 cc2_3a;
		u8 cc1_1a5;
		u8 cc2_1a5;
	} trim;
};

/*
 * +-----------+---------------------+----------+----------+----------+-----------+----------+
 * | ANAMODE   | ANASUBMODE[1:0]     | Notes    | CCx=00   | CCx=01   | CCx=10    | CCx=11   |
 * +-----------+---------------------+----------+----------+----------+-----------+----------+
 * | 0: Source | 00: Disabled        | Disabled | Disabled | N/A      | N/A       | N/A      |
 * |           | 01: Default USB Rp  | -        | vRa[Def] | vRd[Def] | vOPEN[Def]| N/A      |
 * |           | 10: 1.5A Rp         | -        | vRa[1.5] | vRd[1.5] | vOPEN[1.5]| N/A      |
 * |           | 11: 3.0A Rp         | -        | vRa[3.0] | vRd[3.0] | vOPEN[3.0]| N/A      |
 * | 1: Sink   | xx                  | -        | vRa      | vRd-USB  | vRd-1.5   | vRd-3.0  |
 * +-----------+---------------------+----------+----------+----------+-----------+----------+
 */
static enum typec_cc_status ucpd_ccrd_to_tcpm(struct stm32_ucpd *ucpd, u32 cc, u32 cr)
{
	bool is_src = !FIELD_GET(UCPD_ANAMODE, cr);
	bool submode = !!FIELD_GET(UCPD_ANASUBMODE, cr);

	if (is_src) {
		if (!submode)
			return TYPEC_CC_OPEN;

		switch (cc) {
		case 0:
			return TYPEC_CC_RA;
		case 1:
			return TYPEC_CC_RD;
		case 2:
			return TYPEC_CC_OPEN;
		default:
			dev_err(ucpd->dev, "cc mode %d\n", cc);
			return TYPEC_CC_OPEN;
		}
	}

	switch (cc) {
	case 0:
		return TYPEC_CC_OPEN;
	case 1:
		return TYPEC_CC_RP_DEF;
	case 2:
		return TYPEC_CC_RP_1_5;
	case 3:
		return TYPEC_CC_RP_3_0;
	default:
		dev_err(ucpd->dev, "cc mode %d\n", cc);
		return TYPEC_CC_OPEN;
	}
}

static void ucpd_set_trim(struct stm32_ucpd *ucpd, bool ccx_3a0)
{
	u32 val = FIELD_PREP(TRIM_CC1_RD, ucpd->trim.cc1_rd) |
		  FIELD_PREP(TRIM_CC2_RD, ucpd->trim.cc2_rd);

	if (ccx_3a0)
		val |= FIELD_PREP(TRIM_CC1_RP, ucpd->trim.cc1_3a) |
			FIELD_PREP(TRIM_CC2_RP, ucpd->trim.cc2_3a);
	else
		val |= FIELD_PREP(TRIM_CC1_RP, ucpd->trim.cc1_1a5) |
			FIELD_PREP(TRIM_CC2_RP, ucpd->trim.cc2_1a5);

	writel(val, ucpd->base + UCPD_CFGR3);
}

static int stm32_ucpd_init(struct tcpc_dev *dev)
{
	struct stm32_ucpd *ucpd = container_of(dev, struct stm32_ucpd, tcpc_dev);
	unsigned long rate;
	u32 psc, hbitclkdiv;
	int ret;
	u32 val;

	ucpd_set_trim(ucpd, true);

	val = readl(ucpd->base + UCPD_CFGR1);
	val &= ~UCPD_EN;
	writel(val, ucpd->base + UCPD_CFGR1);

	/* Enable wakeup when UCPD_EN = 0. It will be disabled if not needed upon suspend */
	val = readl(ucpd->base + UCPD_CFGR2);
	writel_relaxed(val | UCPD_WUPEN, ucpd->base + UCPD_CFGR2);
	enable_irq_wake(ucpd->ucpd_irq);

	/*
	 * UCPD_PSC_UCPDCLK can vary from 0..4: power of two prescaler: 1, 2, ... 16
	 * Recommended ucpd_clk frequency is in the range of 6 to 9 MHz.
	 */
	psc = order_base_2(DIV_ROUND_UP(ucpd->ker_clk_rate, 9000000UL));
	rate = ucpd->ker_clk_rate >> psc;

	if (psc > 4 || rate < 6000000UL || rate > 9000000UL) {
		dev_err(ucpd->dev, "No prescaler found\n");
		return -EINVAL;
	}

	/* Half-bit clock to reach near 600HKz */
	hbitclkdiv = DIV_ROUND_CLOSEST(rate, 600000UL);

	/* HBITCLKDIV bitfield = 0, means no division, e.g. 1. (Used for CC debounce) */
	hbitclkdiv = hbitclkdiv ? hbitclkdiv - 1 : 0;

	dev_dbg(ucpd->dev, "rate = %lu, hbit = %lu\n", rate, rate / (hbitclkdiv + 1));

	val = readl(ucpd->base + UCPD_CFGR1);
	val |= UCPD_EN;
	val |= FIELD_PREP(UCPD_PSC_UCPDCLK, psc);
	val |= FIELD_PREP(UCPD_HBITCLKDIV, hbitclkdiv);
	writel(val, ucpd->base + UCPD_CFGR1);

	ret = tcpp_init(ucpd->tcpp_dev);
	if (ret) {
		writel(0, ucpd->base + UCPD_CFGR1);
		return ret;
	}

	writel_relaxed(TYPECEVT1IE | TYPECEVT2IE, ucpd->base + UCPD_IMR);

	return 0;
}

/*
 * stm32_ucpd_deinit() should be called with IRQ disabled, to avoid undesired IRQ when
 * disconnecting TCPP protection
 */
static void stm32_ucpd_deinit(struct tcpc_dev *dev)
{
	struct stm32_ucpd *ucpd = container_of(dev, struct stm32_ucpd, tcpc_dev);

	/*
	 * Place the UCPD lines into a safe state: switch OFF vbus gates and isolate CC lines.
	 * It avoids a device that may be plugged into the Type-C port to pull-up the CC lines
	 * at 5v and damage them (this must be avoided in case our vdd supply gets turned off
	 * upon reboot/reset).
	 */
	tcpp_deinit(ucpd->tcpp_dev);

	/* Can now disable UCPD (clear UCPD_EN), other register bits will be reset */
	writel_relaxed(0, ucpd->base + UCPD_CFGR1);
	disable_irq_wake(ucpd->ucpd_irq);
}

static int stm32_ucpd_get_vbus(struct tcpc_dev *dev)
{
	struct stm32_ucpd *ucpd = container_of(dev, struct stm32_ucpd, tcpc_dev);

	return tcpp_get_vbus(ucpd->tcpp_dev);
}

static int stm32_ucpd_set_vbus(struct tcpc_dev *dev, bool on, bool charge)
{
	struct stm32_ucpd *ucpd = container_of(dev, struct stm32_ucpd, tcpc_dev);
	int ret;

	dev_dbg(ucpd->dev, "%s: %s %s\n", __func__, str_on_off(on), str_on_off(charge));

	ret = tcpp_set_vbus(ucpd->tcpp_dev, on, charge);
	if (!ret) {
		ucpd->vbus_on = on;
		ucpd->charge_on = charge;
	}

	return ret;
}

static u32 stm32_ucpd_to_anasubmode(enum typec_cc_status cc)
{
	switch (cc) {
	case TYPEC_CC_RP_DEF:
		return FIELD_PREP(UCPD_ANASUBMODE, 1);
	case TYPEC_CC_RP_1_5:
		return FIELD_PREP(UCPD_ANASUBMODE, 2);
	case TYPEC_CC_RP_3_0:
		return FIELD_PREP(UCPD_ANASUBMODE, 3);
	default:
		return 0;
	}
}

static int __stm32_ucpd_set_cc(struct stm32_ucpd *ucpd, enum typec_cc_status cc)
{
	u32 val;

	/* Temporarily mask comparator events during DRP toggling */
	writel(0, ucpd->base + UCPD_IMR);

	switch (cc) {
	case TYPEC_CC_OPEN:
		val = readl(ucpd->base + UCPD_CR);
		val = (val & ~UCPD_CCENABLE) | UCPD_ANAMODE;
		break;
	case TYPEC_CC_RD:
		val = readl(ucpd->base + UCPD_CR);
		val |= (UCPD_ANAMODE | UCPD_CCENABLE);
		break;
	case TYPEC_CC_RP_DEF:
	case TYPEC_CC_RP_1_5:
	case TYPEC_CC_RP_3_0:
		ucpd_set_trim(ucpd, cc == TYPEC_CC_RP_3_0);
		val = readl(ucpd->base + UCPD_CR);
		val &= ~(UCPD_ANAMODE | UCPD_ANASUBMODE);
		val |= stm32_ucpd_to_anasubmode(cc) | UCPD_CCENABLE;
		break;
	default:
		dev_err(ucpd->dev, "%s: Unsupported cc status %d\n", __func__, cc);

		return -EOPNOTSUPP;
	}

	writel(val, ucpd->base + UCPD_CR);

	/*
	 * It can take up to 1ms e.g. tDRPTransition during DRP toggling to reach
	 * a stable state. Re-enable CC1/CC2 events after it.
	 */
	usleep_range(1000, 1100);
	writel(TYPECEVT1IE | TYPECEVT2IE, ucpd->base + UCPD_IMR);

	return 0;
}

static int stm32_ucpd_set_cc(struct tcpc_dev *dev, enum typec_cc_status cc)
{
	struct stm32_ucpd *ucpd = container_of(dev, struct stm32_ucpd, tcpc_dev);
	int ret = __stm32_ucpd_set_cc(ucpd, cc);

	if (!ret)
		ucpd->cc = cc;

	return ret;
}

static int stm32_ucpd_get_cc(struct tcpc_dev *dev, enum typec_cc_status *cc1,
			     enum typec_cc_status *cc2)
{
	struct stm32_ucpd *ucpd = container_of(dev, struct stm32_ucpd, tcpc_dev);
	u32 cr = readl_relaxed(ucpd->base + UCPD_CR);
	u32 sr = readl_relaxed(ucpd->base + UCPD_SR);

	*cc1 = ucpd_ccrd_to_tcpm(ucpd, FIELD_GET(TYPEC_VSTATE_CC1, sr), cr);
	*cc2 = ucpd_ccrd_to_tcpm(ucpd, FIELD_GET(TYPEC_VSTATE_CC2, sr), cr);

	dev_dbg(ucpd->dev, "%s: cc1=0x%x cc2=0x%x\n", __func__, *cc1, *cc2);

	return 0;
}

static int stm32_ucpd_set_polarity(struct tcpc_dev *dev, enum typec_cc_polarity polarity)
{
	struct stm32_ucpd *ucpd = container_of(dev, struct stm32_ucpd, tcpc_dev);

	ucpd->polarity = polarity;
	dev_dbg(ucpd->dev, "%s: polarity %d\n", __func__, polarity);

	return 0;
}

static int stm32_ucpd_set_vconn_gpio(struct stm32_ucpd *ucpd, bool on)
{
	bool cc1_on = on && ucpd->polarity == TYPEC_POLARITY_CC2;
	bool cc2_on = on && ucpd->polarity == TYPEC_POLARITY_CC1;
	int ret;

	/*
	 * Enable regulator before turning cc1/cc2 gate on. Check vconn_on, to
	 * ensure regulator enable / disable balance.
	 */
	if (on && !ucpd->vconn_on && ucpd->vconn) {
		ret = regulator_enable(ucpd->vconn);
		if (ret) {
			dev_err(ucpd->dev, "vconn enable failed %d\n", ret);
			return ret;
		}
	}

	if (ucpd->vconn_cc1_gpio) {
		ret = gpiod_set_value_cansleep(ucpd->vconn_cc1_gpio, cc1_on);
		if (ret) {
			dev_err(ucpd->dev, "vconn-cc1 %s failed: %d\n", str_enable_disable(cc1_on),
				ret);
			goto undo_vconn;
		}
		dev_dbg(ucpd->dev, "vconn-cc1 %s\n", str_enabled_disabled(cc1_on));
	}

	if (ucpd->vconn_cc2_gpio) {
		ret = gpiod_set_value_cansleep(ucpd->vconn_cc2_gpio, cc2_on);
		if (ret) {
			dev_err(ucpd->dev, "vconn-cc2 %s failed: %d\n", str_enable_disable(cc2_on),
				ret);
			goto undo_vconn;
		}
		dev_dbg(ucpd->dev, "vconn-cc2 %s\n", str_enabled_disabled(cc2_on));
	}

	/* Disable in the reverse order */
	if (!on && ucpd->vconn_on && ucpd->vconn) {
		ret = regulator_disable(ucpd->vconn);
		if (ret) {
			dev_err(ucpd->dev, "vconn disable failed %d\n", ret);
			return ret;
		}
	}

	return 0;

undo_vconn:
	if (on && !ucpd->vconn_on && ucpd->vconn)
		regulator_disable(ucpd->vconn);

	return ret;
}

static int stm32_ucpd_set_vconn(struct tcpc_dev *dev, bool on)
{
	struct stm32_ucpd *ucpd = container_of(dev, struct stm32_ucpd, tcpc_dev);
	int ret;

	dev_dbg(ucpd->dev, "%s: %s\n", __func__, str_on_off(on));

	if (!on) {
		/* Disable optional gate for CC1/CC2 Vconn before tcpp */
		ret = stm32_ucpd_set_vconn_gpio(ucpd, on);
		if (ret)
			return ret;
	}

	ret = tcpp_set_vconn(ucpd->tcpp_dev, on, ucpd->polarity);
	if (ret) {
		dev_err(ucpd->dev, "tcpp_set_vconn failed: %d\n", ret);
		return ret;
	}

	if (on) {
		/* Enable optional gate for CC1/CC2 Vconn after tcpp */
		ret = stm32_ucpd_set_vconn_gpio(ucpd, on);
		if (ret)
			return ret;
	}

	ucpd->vconn_on = on;

	return 0;
}

static int stm32_ucpd_set_pd_rx(struct tcpc_dev *dev, bool on)
{
	struct stm32_ucpd *ucpd = container_of(dev, struct stm32_ucpd, tcpc_dev);

	dev_dbg(ucpd->dev, "%s power delivery reception\n", str_enable_disable(on));

	if (on)
		return -EOPNOTSUPP;
	else
		return 0;
}

static int stm32_ucpd_set_roles(struct tcpc_dev *dev, bool attached,
				enum typec_role pwr, enum typec_data_role data)
{
	struct stm32_ucpd *ucpd = container_of(dev, struct stm32_ucpd, tcpc_dev);
	int ret;

	dev_dbg(ucpd->dev, "%s: Set role pwr %s data %s\n", __func__,
		pwr == TYPEC_SINK ? "sink" : "source",
		data == TYPEC_DEVICE ? "device" : "host");

	if (attached) {
		dev_dbg(ucpd->dev, "attached\n");
		ret = tcpp_attach(ucpd->tcpp_dev);
	} else {
		dev_dbg(ucpd->dev, "detached\n");
		ret = tcpp_detach(ucpd->tcpp_dev);
	}

	return ret;
}

static int stm32_ucpd_pd_transmit(struct tcpc_dev *dev, enum tcpm_transmit_type type,
				  const struct pd_message *msg, unsigned int negotiated_rev)
{
	struct stm32_ucpd *ucpd = container_of(dev, struct stm32_ucpd, tcpc_dev);

	dev_dbg(ucpd->dev, "%s regociated %d\n", __func__, negotiated_rev);

	return -EOPNOTSUPP;
}

static void init_tcpc_dev(struct tcpc_dev *tcpc_dev)
{
	tcpc_dev->init = stm32_ucpd_init;
	tcpc_dev->get_vbus = stm32_ucpd_get_vbus;
	tcpc_dev->set_cc = stm32_ucpd_set_cc;
	tcpc_dev->get_cc = stm32_ucpd_get_cc;
	tcpc_dev->set_polarity = stm32_ucpd_set_polarity;
	tcpc_dev->set_vconn = stm32_ucpd_set_vconn;
	tcpc_dev->set_vbus = stm32_ucpd_set_vbus;
	tcpc_dev->set_pd_rx = stm32_ucpd_set_pd_rx;
	tcpc_dev->set_roles = stm32_ucpd_set_roles;
	tcpc_dev->pd_transmit = stm32_ucpd_pd_transmit;
}

static int stm32_ucpd_read_trim(struct device *dev, const char *name, u8 *val)
{
	struct nvmem_cell *cell;
	size_t len;
	void *buf;

	cell = devm_nvmem_cell_get(dev, name);
	if (IS_ERR(cell)) {
		dev_err(dev, "cannot get cell %s\n", name);
		return PTR_ERR(cell);
	}

	buf = nvmem_cell_read(cell, &len);
	if (IS_ERR(buf)) {
		dev_err(dev, "cannot read %s\n", name);
		return PTR_ERR(buf);
	}

	*val = *(u8 *)buf;
	kfree(buf);

	return 0;
}

struct trim_desc {
	const char *name;
	u8 *value;
};

static int stm32_ucpd_get_trim_values(struct stm32_ucpd *ucpd)
{
	struct trim_desc desc[] = {
		{ "trim_1a5_cc1", &ucpd->trim.cc1_1a5 },
		{ "trim_3a_cc1",  &ucpd->trim.cc1_3a },
		{ "trim_3a_cc2",  &ucpd->trim.cc2_3a },
		{ "trim_1a5_cc2", &ucpd->trim.cc2_1a5 },
		{ "trim_rd_cc1", &ucpd->trim.cc1_rd },
		{ "trim_rd_cc2", &ucpd->trim.cc2_rd },
	};
	int i;

	for (i = 0; i < ARRAY_SIZE(desc); i++) {
		int ret = stm32_ucpd_read_trim(ucpd->dev, desc[i].name, desc[i].value);

		if (ret)
			return ret;
	}

	return 0;
}

static irqreturn_t ucpd_irq_handler(int irq, void *data)
{
	struct stm32_ucpd *ucpd = data;
	u32 sr, handled = 0;

	sr = readl_relaxed(ucpd->base + UCPD_SR);

	if (sr & TYPECEVT1)
		handled |= TYPECEVT1CF;

	if (sr & TYPECEVT2)
		handled |= TYPECEVT2CF;

	if (!handled)
		return IRQ_NONE;

	writel(handled, ucpd->base + UCPD_ICR);

	tcpm_cc_change(ucpd->tcpm_port);

	return IRQ_HANDLED;
}

static void ucpd_vbus_changed(void *data, bool vbus_present)
{
	struct stm32_ucpd *ucpd = data;

	dev_dbg(ucpd->dev, "VBUS change: %d\n", vbus_present);

	tcpm_vbus_change(ucpd->tcpm_port);
}

static int stm32_ucpd_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct i2c_client *tcpp_client;
	struct device_node *tcpp_np;
	struct stm32_ucpd *ucpd;
	int ret;

	ucpd = devm_kzalloc(dev, sizeof(*ucpd), GFP_KERNEL);
	if (!ucpd)
		return -ENOMEM;

	ucpd->dev = dev;
	platform_set_drvdata(pdev, ucpd);

	ucpd->vdd = devm_regulator_get(&pdev->dev, "ucpd");
	if (IS_ERR(ucpd->vdd))
		return dev_err_probe(&pdev->dev, PTR_ERR(ucpd->vdd), "vdd get failed\n");

	ucpd->vconn = devm_regulator_get_optional(&pdev->dev, "vconn");
	if (IS_ERR(ucpd->vconn)) {
		if (PTR_ERR(ucpd->vconn) != -ENODEV)
			return dev_err_probe(&pdev->dev, PTR_ERR(ucpd->vconn),
					     "vconn get failed\n");
		ucpd->vconn = NULL;
	}

	ucpd->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(ucpd->base))
		return PTR_ERR(ucpd->base);

	/* Both APB and KER clocks are mandatory */
	ucpd->num_clks = devm_clk_bulk_get_all(dev, &ucpd->clks);
	if (ucpd->num_clks <= 0 || ucpd->num_clks != 2) {
		ret = ucpd->num_clks < 0 ? ucpd->num_clks : !ucpd->num_clks ? -ENOENT : -EINVAL;
		return dev_err_probe(dev, ret, "clk_bulk_get failed: num=%d\n", ucpd->num_clks);
	}

	ucpd->reset = devm_reset_control_get_exclusive(dev, "ucpd");
	if (IS_ERR(ucpd->reset))
		return dev_err_probe(dev, PTR_ERR(ucpd->reset), "Failed to get UCPD reset\n");

	ret = stm32_ucpd_get_trim_values(ucpd);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to get trim values\n");

	ucpd->vconn_cc1_gpio = devm_gpiod_get_optional(dev, "vconn-cc1", GPIOD_OUT_LOW);
	if (IS_ERR(ucpd->vconn_cc1_gpio))
		return dev_err_probe(dev, PTR_ERR(ucpd->vconn_cc1_gpio), "vconn-cc1 get failed\n");

	ucpd->vconn_cc2_gpio = devm_gpiod_get_optional(dev, "vconn-cc2", GPIOD_OUT_LOW);
	if (IS_ERR(ucpd->vconn_cc2_gpio))
		return dev_err_probe(dev, PTR_ERR(ucpd->vconn_cc2_gpio), "vconn-cc2 get failed\n");

	ucpd->ucpd_irq = platform_get_irq_byname(pdev, "ucpd");
	if (ucpd->ucpd_irq < 0)
		return ucpd->ucpd_irq;

	ucpd->tcpc_dev.fwnode = device_get_named_child_node(dev, "connector");
	if (IS_ERR(ucpd->tcpc_dev.fwnode))
		return dev_err_probe(dev, PTR_ERR(ucpd->tcpc_dev.fwnode), "connector not found");

	tcpp_np = of_parse_phandle(dev->of_node, "st,tcpp", 0);
	if (!tcpp_np)
		return -EINVAL;

	tcpp_client = of_find_i2c_device_by_node(tcpp_np);
	of_node_put(tcpp_np);

	/* tcpp driver must be probed so tcpp_init() can be called from tcpm_register_port() */
	if (!tcpp_client || !i2c_get_clientdata(tcpp_client))
		return -EPROBE_DEFER;

	ucpd->tcpp_dev = &tcpp_client->dev;

	/* No irq until tcpm_port is ready */
	irq_set_status_flags(ucpd->ucpd_irq, IRQ_NOAUTOEN);
	ret = devm_request_threaded_irq(dev, ucpd->ucpd_irq, NULL, ucpd_irq_handler, IRQF_ONESHOT,
					dev_name(dev), ucpd);
	if (ret) {
		dev_err_probe(dev, ret, "failed to get irq\n");
		goto put_tcpp;
	}

	init_tcpc_dev(&ucpd->tcpc_dev);

	ret = regulator_enable(ucpd->vdd);
	if (ret) {
		dev_err_probe(dev, ret, "failed to enable regulator\n");
		goto put_tcpp;
	}

	ret = clk_bulk_prepare_enable(ucpd->num_clks, ucpd->clks);
	if (ret) {
		dev_err_probe(dev, ret, "failed to enable clocks\n");
		goto disable_regulator;
	}
	ucpd->ker_clk_rate = clk_get_rate(ucpd->clks[1].clk);

	ret = reset_control_reset(ucpd->reset);
	if (ret) {
		dev_err_probe(dev, ret, "failed to deassert reset\n");
		goto disable_clks;
	}

	ret = devm_pm_runtime_get_noresume(dev);
	if (ret) {
		dev_err_probe(dev, ret, "Failed to get runtime PM noresume\n");
		goto disable_clks;
	}

	ret = devm_pm_runtime_set_active_enabled(dev);
	if (ret < 0) {
		dev_err_probe(dev, ret, "Failed to activate and enable runtime PM\n");
		goto disable_clks;
	}

	ucpd->tcpm_port = tcpm_register_port(dev, &ucpd->tcpc_dev);
	if (IS_ERR(ucpd->tcpm_port)) {
		ret = dev_err_probe(dev, PTR_ERR(ucpd->tcpm_port), "failed to register tcpm\n");
		goto disable_clks;
	}

	tcpp_register_vbus_notifier(ucpd->tcpp_dev, ucpd_vbus_changed, ucpd);

	device_set_wakeup_capable(dev, true);

	enable_irq(ucpd->ucpd_irq);

	return 0;

disable_clks:
	clk_bulk_disable_unprepare(ucpd->num_clks, ucpd->clks);

disable_regulator:
	regulator_disable(ucpd->vdd);

put_tcpp:
	put_device(ucpd->tcpp_dev);

	return ret;
};

static void stm32_ucpd_remove(struct platform_device *pdev)
{
	struct stm32_ucpd *ucpd = platform_get_drvdata(pdev);

	disable_irq(ucpd->ucpd_irq);
	tcpp_unregister_vbus_notifier(ucpd->tcpp_dev);
	tcpm_unregister_port(ucpd->tcpm_port);
	stm32_ucpd_deinit(&ucpd->tcpc_dev);
	clk_bulk_disable_unprepare(ucpd->num_clks, ucpd->clks);
	regulator_disable(ucpd->vdd);
	put_device(ucpd->tcpp_dev);
};

static void stm32_ucpd_shutdown(struct platform_device *pdev)
{
	struct stm32_ucpd *ucpd = platform_get_drvdata(pdev);

	disable_irq(ucpd->ucpd_irq);
	stm32_ucpd_set_vconn(&ucpd->tcpc_dev, false);
	stm32_ucpd_deinit(&ucpd->tcpc_dev);
}

static int stm32_ucpd_suspend(struct device *dev)
{
	struct stm32_ucpd *ucpd = dev_get_drvdata(dev);

	/* Already enabled for wakeup: just return */
	if (device_may_wakeup(dev))
		return 0;

	disable_irq(ucpd->ucpd_irq);

	/*
	 * Make sure that the state machine is halted
	 * to avoid CC read access during toggling.
	 */
	tcpm_port_suspend(ucpd->tcpm_port);

	/* backup vconn_on flag */
	ucpd->vconn_restore = ucpd->vconn_on;
	stm32_ucpd_set_vconn(&ucpd->tcpc_dev, false);
	stm32_ucpd_deinit(&ucpd->tcpc_dev);
	clk_bulk_disable_unprepare(ucpd->num_clks, ucpd->clks);
	regulator_disable(ucpd->vdd);

	return 0;
}

static int stm32_ucpd_resume(struct device *dev)
{
	struct stm32_ucpd *ucpd = dev_get_drvdata(dev);
	int ret;

	/* Kept active: just return */
	if (device_may_wakeup(dev))
		return 0;

	ret = regulator_enable(ucpd->vdd);
	if (ret)
		return ret;

	ret = clk_bulk_prepare_enable(ucpd->num_clks, ucpd->clks);
	if (ret) {
		regulator_disable(ucpd->vdd);
		return ret;
	}

	stm32_ucpd_init(&ucpd->tcpc_dev);
	__stm32_ucpd_set_cc(ucpd, ucpd->cc);
	stm32_ucpd_set_vconn(&ucpd->tcpc_dev, ucpd->vconn_restore);
	tcpp_set_vbus(ucpd->tcpp_dev, ucpd->vbus_on, ucpd->charge_on);

	/* tcpm must be running to handle cc_change */
	tcpm_port_resume(ucpd->tcpm_port);

	enable_irq(ucpd->ucpd_irq);

	return 0;
}

static const struct dev_pm_ops stm32_ucpd_pm_ops = {
	SYSTEM_SLEEP_PM_OPS(stm32_ucpd_suspend, stm32_ucpd_resume)
};

static const struct of_device_id stm32_ucpd_of_match[] = {
	{ .compatible = "st,stm32mp25-ucpd" },
	{},
};
MODULE_DEVICE_TABLE(of, stm32_ucpd_of_match);

static struct platform_driver stm32_ucpd_driver = {
	.probe = stm32_ucpd_probe,
	.remove	= stm32_ucpd_remove,
	.shutdown = stm32_ucpd_shutdown,
	.driver = {
		.name = "stm32-ucpd",
		.pm = &stm32_ucpd_pm_ops,
		.of_match_table = stm32_ucpd_of_match,
	},
};
module_platform_driver(stm32_ucpd_driver);

MODULE_DESCRIPTION("USB Type-C Power Delivery");
MODULE_AUTHOR("Christian Bruel <christian.bruel@foss.st.com>");
MODULE_LICENSE("GPL");
