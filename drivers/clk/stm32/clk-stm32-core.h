/* SPDX-License-Identifier: GPL-2.0  */
/*
 * Copyright (C) STMicroelectronics 2022 - All Rights Reserved
 * Author: Gabriel Fernandez <gabriel.fernandez@foss.st.com> for STMicroelectronics.
 */

#include <linux/clk-provider.h>

struct stm32_rcc_match_data;

struct stm32_mux_cfg {
	u16	offset;
	u8	shift;
	u8	width;
	u8	flags;
	u32	*table;
	u8	ready;
};

struct stm32_gate_cfg {
	u16	offset;
	u8	bit_idx;
	u8	set_clr;
};

struct stm32_div_cfg {
	u16	offset;
	u8	shift;
	u8	width;
	u8	flags;
	u8	ready;
	const struct clk_div_table *table;
};

struct stm32_composite_cfg {
	int	mux;
	int	gate;
	int	div;
};

#define NO_ID 0xFFFF0000

#define NO_STM32_MUX		0xFFFF
#define NO_STM32_DIV		0xFFFF
#define NO_STM32_GATE		0xFFFF

struct clock_config {
	unsigned long	id;
	int		sec_id;
	void		*clock_cfg;

	struct clk_hw *(*func)(struct device *dev,
			       const struct stm32_rcc_match_data *data,
			       void __iomem *base,
			       spinlock_t *lock,
			       const struct clock_config *cfg);
};

struct clk_stm32_clock_data {
	void __iomem			*base;
	u16 *gate_cpt;
	const struct stm32_gate_cfg	*gates;
	const struct stm32_mux_cfg	*muxes;
	const struct stm32_div_cfg	*dividers;
	struct clk_hw *(*is_multi_mux)(struct clk_hw *hw);
};

struct clock_summary {
	struct clk_summary *clocks;
	int nb_clocks;
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

	bool (*is_enabled)(struct clk_stm32_clock_data *data,
			   struct clk_summary *c);
	u8 (*get_parent)(struct clk_stm32_clock_data *data,
			 struct clk_summary *c);
	unsigned long (*get_rate)(struct clk_stm32_clock_data *data,
				  struct clk_summary *c,
				  unsigned long parent_rate);
	const char * const *parent_names;
};

struct stm32_rcc_match_data {
	struct clk_hw_onecell_data	*hw_clks;
	unsigned int			num_clocks;
	const struct clock_config	*tab_clocks;
	unsigned int			maxbinding;
	struct clk_stm32_clock_data	*clock_data;
	int (*multi_mux)(void __iomem *base, const struct clock_config *cfg);
	int (*check_security)(void __iomem *base, const struct clock_config *cfg);
	u32 clear_offset;
	u32				reset_us;

	struct clock_summary		*clock_summary;
};

int stm32_rcc_reset_init(struct device *dev, const struct of_device_id *match,
			 void __iomem *base);

int stm32_rcc_init(struct device *dev, const struct of_device_id *match_data,
		   void __iomem *base);

/* MUX define */
#define MUX_NO_RDY		0xFF
#define MUX_SAFE		BIT(7)

/* DIV define */
#define DIV_NO_RDY		0xFF

struct clk_stm32_gate {
	u16 gate_id;
	struct clk_hw hw;
	void __iomem *base;
	struct clk_stm32_clock_data *clock_data;
	spinlock_t *lock; /* spin lock */
};

#define to_clk_stm32_gate(_hw) container_of(_hw, struct clk_stm32_gate, hw)

struct clk_stm32_mux {
	u16 mux_id;
	struct clk_hw hw;
	void __iomem *base;
	struct clk_stm32_clock_data *clock_data;
	spinlock_t *lock; /* spin lock */
};

#define to_clk_stm32_mux(_hw) container_of(_hw, struct clk_stm32_mux, hw)

struct clk_stm32_div {
	u16 div_id;
	struct clk_hw hw;
	void __iomem *base;
	struct clk_stm32_clock_data *clock_data;
	spinlock_t *lock; /* spin lock */
};

#define to_clk_stm32_divider(_hw) container_of(_hw, struct clk_stm32_div, hw)

struct clk_stm32_composite {
	u16 gate_id;
	u16 mux_id;
	u16 div_id;
	struct clk_hw hw;
	void __iomem *base;
	struct clk_stm32_clock_data *clock_data;
	spinlock_t *lock; /* spin lock */
};

#define to_clk_stm32_composite(_hw) container_of(_hw, struct clk_stm32_composite, hw)

void clk_stm32_endisable_gate(void __iomem *base,
			      struct clk_stm32_clock_data *data, u16 gate_id,
			      int enable);
int clk_stm32_is_enabled_gate(void __iomem *base,
			      struct clk_stm32_clock_data *data, u16 gate_id);
u8 clk_stm32_get_parent_mux(void __iomem *base,
			    struct clk_stm32_clock_data *data, u16 mux_id);
int clk_stm32_set_parent_mux(void __iomem *base,
			     struct clk_stm32_clock_data *data, u16 mux_id,
			     u8 index);
int clk_stm32_set_rate_divider(void __iomem *base,
			       struct clk_stm32_clock_data *data,
			       u16 div_id, unsigned long rate,
			       unsigned long parent_rate);
unsigned long clk_stm32_get_rate_divider(void __iomem *base,
					 struct clk_stm32_clock_data *data,
					 u16 div_id, unsigned long parent_rate);

void clk_stm32_gate_endisable(struct clk_hw *hw, int enable);
int clk_stm32_gate_enable(struct clk_hw *hw);
void clk_stm32_gate_disable(struct clk_hw *hw);
int clk_stm32_gate_is_enabled(struct clk_hw *hw);

u8 clk_stm32_mux_get_parent(struct clk_hw *hw);
int clk_stm32_mux_set_parent(struct clk_hw *hw, u8 index);

void clk_stm32_composite_gate_endisable(struct clk_hw *hw, int enable);
int clk_stm32_composite_gate_enable(struct clk_hw *hw);
void clk_stm32_composite_gate_disable(struct clk_hw *hw);
int clk_stm32_composite_is_enabled(struct clk_hw *hw);
u8 clk_stm32_composite_get_parent(struct clk_hw *hw);
int clk_stm32_composite_set_parent(struct clk_hw *hw, u8 index);
unsigned long clk_stm32_composite_recalc_rate(struct clk_hw *hw,
					      unsigned long parent_rate);
long clk_stm32_composite_round_rate(struct clk_hw *hw, unsigned long rate,
				    unsigned long *prate);
int clk_stm32_composite_set_rate(struct clk_hw *hw, unsigned long rate,
				 unsigned long parent_rate);

struct clk_hw *clk_stm32_gate_register(struct device *dev,
				       const struct stm32_rcc_match_data *data,
				       void __iomem *base,
				       spinlock_t *lock,
				       const struct clock_config *cfg);

struct clk_hw *clk_stm32_div_register(struct device *dev,
				      const struct stm32_rcc_match_data *data,
				      void __iomem *base,
				      spinlock_t *lock,
				      const struct clock_config *cfg);

struct clk_hw *clk_stm32_mux_register(struct device *dev,
				      const struct stm32_rcc_match_data *data,
				      void __iomem *base,
				      spinlock_t *lock,
				      const struct clock_config *cfg);

struct clk_hw *clk_stm32_composite_register(struct device *dev,
					    const struct stm32_rcc_match_data *data,
					    void __iomem *base,
					    spinlock_t *lock,
					    const struct clock_config *cfg);

extern const struct clk_ops clk_stm32_gate_ops;
extern const struct clk_ops clk_stm32_divider_ops;
extern const struct clk_ops clk_stm32_mux_ops;
extern const struct clk_ops clk_stm32_composite_ops;

#define PARENT(_parent)	((const char *[]) { _parent})

#define CLK_STM32_GATE(_name, _parent, _flags, _gate_id)\
struct clk_stm32_gate _name = {\
	.gate_id = _gate_id,\
	.hw.init = CLK_HW_INIT(#_name, _parent, &clk_stm32_gate_ops, _flags),\
}

#define CLK_STM32_MUX(_name, _parents, _flags, _mux_id)\
struct clk_stm32_mux _name = {\
	.mux_id = _mux_id,\
	.hw.init = CLK_HW_INIT_PARENTS(#_name, _parents, &clk_stm32_mux_ops, _flags),\
}

#define CLK_STM32_DIV(_name, _parent, _flags, _div_id)\
struct clk_stm32_div _name = {\
	.div_id = _div_id,\
	.hw.init = CLK_HW_INIT(#_name, _parent, &clk_stm32_divider_ops, _flags),\
}

#define CLK_STM32_COMPOSITE(_name, _parents, _flags, _gate_id, _mux_id, _div_id)\
struct clk_stm32_composite _name = {\
	.gate_id = _gate_id,\
	.mux_id = _mux_id,\
	.div_id = _div_id,\
	.hw.init = CLK_HW_INIT_PARENTS(#_name, _parents, &clk_stm32_composite_ops, _flags),\
}

#define STM32_CLOCK_CFG(_binding, _clk, _sec_id, _struct, _register)\
{\
	.id		= (_binding),\
	.sec_id		= (_sec_id),\
	.clock_cfg	= (_struct) {_clk},\
	.func		= (_register),\
}

#define STM32_GATE_CFG(_binding, _clk, _sec_id)\
	STM32_CLOCK_CFG(_binding, &(_clk), _sec_id, struct clk_stm32_gate *,\
			&clk_stm32_gate_register)

#define STM32_COMPOSITE_CFG(_binding, _clk, _sec_id)\
	STM32_CLOCK_CFG(_binding, &(_clk), _sec_id, struct clk_stm32_composite *,\
			&clk_stm32_composite_register)
