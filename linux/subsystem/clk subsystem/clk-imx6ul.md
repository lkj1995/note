### struct clk

```C
struct clk {
	struct list_head	node;
	struct module		*owner;
	const char		*name;
	unsigned long		rate;
	unsigned long		maxrate;	/* H/W supported max rate */
	u8			usecount;
	u8			lpsc;
	u8			gpsc;
	u8			domain;
	u32			flags;
	struct clk              *parent;
	struct list_head	children; 	/* list of children */
	struct list_head	childnode;	/* parent's child list node */
	struct pll_data         *pll_data;
	u32                     div_reg;
	unsigned long (*recalc) (struct clk *);
	int (*set_rate) (struct clk *clk, unsigned long rate);
	int (*round_rate) (struct clk *clk, unsigned long rate);
	int (*reset) (struct clk *clk, bool reset);
	void (*clk_enable) (struct clk *clk);
	void (*clk_disable) (struct clk *clk);
	int (*set_parent) (struct clk *clk, struct clk *parent);
};

```

### struct of_clk_provider

```C
struct of_clk_provider {
	struct list_head link;

	struct device_node *node;
	struct clk *(*get)(struct of_phandle_args *clkspec, void *data);
	struct clk_hw *(*get_hw)(struct of_phandle_args *clkspec, void *data);
	void *data;
};
```

### struct clk_hw

```
struct clk_hw {
	struct clk_core *core;
	struct clk *clk;
	const struct clk_init_data *init;
};
```

### struct clk_core

```C
struct clk_core {
	const char		*name;
	const struct clk_ops	*ops;
	struct clk_hw		*hw;
	struct module		*owner;
	struct clk_core		*parent;
	const char		**parent_names;
	struct clk_core		**parents;
	u8			num_parents;
	u8			new_parent_index;
	unsigned long		rate;
	unsigned long		req_rate;
	unsigned long		new_rate;
	struct clk_core		*new_parent;
	struct clk_core		*new_child;
	unsigned long		flags;
	bool			orphan;
	unsigned int		enable_count;
	unsigned int		prepare_count;
	unsigned long		min_rate;
	unsigned long		max_rate;
	unsigned long		accuracy;
	int			phase;
	struct hlist_head	children;
	struct hlist_node	child_node;
	struct hlist_head	clks;
	unsigned int		notifier_count;
#ifdef CONFIG_DEBUG_FS
	struct dentry		*dentry;
	struct hlist_node	debug_node;
#endif
	struct kref		ref;
};
```

### struct clk_init_data

```C
struct clk_init_data {
	const char		*name;
	const struct clk_ops	*ops;
	const char		* const *parent_names;
	u8			num_parents;
	unsigned long		flags;
};
```

### pllx_bypass_sels[]

```c
	static const char *pll_bypass_src_sels[] = { "osc", "dummy", };
	static const char *pll1_bypass_sels[] = { "pll1", "pll1_bypass_src", };
	static const char *pll2_bypass_sels[] = { "pll2", "pll2_bypass_src", };
	static const char *pll3_bypass_sels[] = { "pll3", "pll3_bypass_src", };
	static const char *pll4_bypass_sels[] = { "pll4", "pll4_bypass_src", };
	static const char *pll5_bypass_sels[] = { "pll5", "pll5_bypass_src", };
	static const char *pll6_bypass_sels[] = { "pll6", "pll6_bypass_src", };
	static const char *pll7_bypass_sels[] = { "pll7", "pll7_bypass_src", };
```

### 1 dts

```C
/* 
 * 1. 在imx6ull.dtsi中找到node
 * 2. 根据compatible，找到了clk-imx6ul.c,开始分析clk-imx6ul.c。  
 */

clks: ccm@020c4000 {
	compatible = "fsl,imx6ul-ccm";
	reg = <0x020c4000 0x4000>;
	interrupts = <GIC_SPI 87 IRQ_TYPE_LEVEL_HIGH>,
			 	 <GIC_SPI 88 IRQ_TYPE_LEVEL_HIGH>;
	#clock-cells = <1>;
	clocks = <&ckil>, <&osc>, <&ipp_di0>, <&ipp_di1>;
	clock-names = "ckil", "osc", "ipp_di0", "ipp_di1";
};    


clocks {
	#address-cells = <1>;
	#size-cells = <0>;

	/* 外部低速晶振，固定clk */
	ckil: clock@0 {
		compatible = "fixed-clock";
		reg = <0>;
		#clock-cells = <0>;
		clock-frequency = <32768>;
		clock-output-names = "ckil";
	};
	/* 外部高速晶振，固定clk */
	osc: clock@1 {
		compatible = "fixed-clock";
		reg = <1>;
		#clock-cells = <0>;
		clock-frequency = <24000000>;
		clock-output-names = "osc";
	};

    /* PAD的外部输入时钟,没找到在哪个PAD */
	ipp_di0: clock@2 {
		compatible = "fixed-clock";
		reg = <2>;
		#clock-cells = <0>;
		clock-frequency = <0>;
		clock-output-names = "ipp_di0";
	};
        
		 /* PAD的外部输入时钟,没找到在哪个PAD */
	ipp_di1: clock@3 {
		compatible = "fixed-clock";
		reg = <3>;
		#clock-cells = <0>;
		clock-frequency = <0>;
		clock-output-names = "ipp_di1";
	};
};
```

### 2 imx6ul_clocks_init()

```c
/* 可猜测出来,如果dt中有属性"fsl,imx6ul-ccm"，会调用 imx6ul_clocks_init */
CLK_OF_DECLARE(imx6ul, "fsl,imx6ul-ccm", imx6ul_clocks_init);


static void __init imx6ul_clocks_init(struct device_node *ccm_node)
{
	struct device_node *np;
	void __iomem *base;
	int i;

    /* 注册了一个名为 "dummy" 的 provider */
	clks[IMX6UL_CLK_DUMMY] = imx_clk_fixed("dummy", 0);

    /* 已经提前解析"ckil" "osc"等4个node,并生成了clk_provider,找到对应的clk并返回 */
	clks[IMX6UL_CLK_CKIL] = of_clk_get_by_name(ccm_node, "ckil");
	clks[IMX6UL_CLK_OSC] = of_clk_get_by_name(ccm_node, "osc");
	/* ipp_di clock is external input */
	clks[IMX6UL_CLK_IPP_DI0] = of_clk_get_by_name(ccm_node, "ipp_di0");
	clks[IMX6UL_CLK_IPP_DI1] = of_clk_get_by_name(ccm_node, "ipp_di1");

    /* 没有用到 */
	np = of_find_compatible_node(NULL, NULL, "fsl,imx6ul-anatop");
	base = of_iomap(np, 0);
	WARN_ON(!base);

	clks[IMX6UL_PLL1_BYPASS_SRC] = imx_clk_mux("pll1_bypass_src", base + 0x00, 14, 1, pll_bypass_src_sels, ARRAY_SIZE(pll_bypass_src_sels));
	clks[IMX6UL_PLL2_BYPASS_SRC] = imx_clk_mux("pll2_bypass_src", base + 0x30, 14, 1, pll_bypass_src_sels, ARRAY_SIZE(pll_bypass_src_sels));
	clks[IMX6UL_PLL3_BYPASS_SRC] = imx_clk_mux("pll3_bypass_src", base + 0x10, 14, 1, pll_bypass_src_sels, ARRAY_SIZE(pll_bypass_src_sels));
	clks[IMX6UL_PLL4_BYPASS_SRC] = imx_clk_mux("pll4_bypass_src", base + 0x70, 14, 1, pll_bypass_src_sels, ARRAY_SIZE(pll_bypass_src_sels));
	clks[IMX6UL_PLL5_BYPASS_SRC] = imx_clk_mux("pll5_bypass_src", base + 0xa0, 14, 1, pll_bypass_src_sels, ARRAY_SIZE(pll_bypass_src_sels));
	clks[IMX6UL_PLL6_BYPASS_SRC] = imx_clk_mux("pll6_bypass_src", base + 0xe0, 14, 1, pll_bypass_src_sels, ARRAY_SIZE(pll_bypass_src_sels));
	clks[IMX6UL_PLL7_BYPASS_SRC] = imx_clk_mux("pll7_bypass_src", base + 0x20, 14, 1, pll_bypass_src_sels, ARRAY_SIZE(pll_bypass_src_sels));

	clks[IMX6UL_CLK_PLL1] = imx_clk_pllv3(IMX_PLLV3_SYS,	 "pll1", "osc", base + 0x00, 0x7f);
	clks[IMX6UL_CLK_PLL2] = imx_clk_pllv3(IMX_PLLV3_PLL2,    "pll2", "osc", base + 0x30, 0x1);
	clks[IMX6UL_CLK_PLL3] = imx_clk_pllv3(IMX_PLLV3_USB,	 "pll3", "osc", base + 0x10, 0x3);
	clks[IMX6UL_CLK_PLL4] = imx_clk_pllv3(IMX_PLLV3_AV,	 "pll4", "osc", base + 0x70, 0x7f);
	clks[IMX6UL_CLK_PLL5] = imx_clk_pllv3(IMX_PLLV3_AV,	 "pll5", "osc", base + 0xa0, 0x7f);
	clks[IMX6UL_CLK_PLL6] = imx_clk_pllv3(IMX_PLLV3_ENET,	 "pll6", "osc", base + 0xe0, 0x3);
	clks[IMX6UL_CLK_PLL7] = imx_clk_pllv3(IMX_PLLV3_USB,	 "pll7", "osc", base + 0x20, 0x3);

	clks[IMX6UL_PLL1_BYPASS] = imx_clk_mux_flags("pll1_bypass", base + 0x00, 16, 1, pll1_bypass_sels, ARRAY_SIZE(pll1_bypass_sels), CLK_SET_RATE_PARENT);
	clks[IMX6UL_PLL2_BYPASS] = imx_clk_mux_flags("pll2_bypass", base + 0x30, 16, 1, pll2_bypass_sels, ARRAY_SIZE(pll2_bypass_sels), CLK_SET_RATE_PARENT);
	clks[IMX6UL_PLL3_BYPASS] = imx_clk_mux_flags("pll3_bypass", base + 0x10, 16, 1, pll3_bypass_sels, ARRAY_SIZE(pll3_bypass_sels), CLK_SET_RATE_PARENT);
	clks[IMX6UL_PLL4_BYPASS] = imx_clk_mux_flags("pll4_bypass", base + 0x70, 16, 1, pll4_bypass_sels, ARRAY_SIZE(pll4_bypass_sels), CLK_SET_RATE_PARENT);
	clks[IMX6UL_PLL5_BYPASS] = imx_clk_mux_flags("pll5_bypass", base + 0xa0, 16, 1, pll5_bypass_sels, ARRAY_SIZE(pll5_bypass_sels), CLK_SET_RATE_PARENT);
	clks[IMX6UL_PLL6_BYPASS] = imx_clk_mux_flags("pll6_bypass", base + 0xe0, 16, 1, pll6_bypass_sels, ARRAY_SIZE(pll6_bypass_sels), CLK_SET_RATE_PARENT);
	clks[IMX6UL_PLL7_BYPASS] = imx_clk_mux_flags("pll7_bypass", base + 0x20, 16, 1, pll7_bypass_sels, ARRAY_SIZE(pll7_bypass_sels), CLK_SET_RATE_PARENT);

	/* Do not bypass PLLs initially */
	clk_set_parent(clks[IMX6UL_PLL1_BYPASS], clks[IMX6UL_CLK_PLL1]);
	clk_set_parent(clks[IMX6UL_PLL2_BYPASS], clks[IMX6UL_CLK_PLL2]);
	clk_set_parent(clks[IMX6UL_PLL3_BYPASS], clks[IMX6UL_CLK_PLL3]);
	clk_set_parent(clks[IMX6UL_PLL4_BYPASS], clks[IMX6UL_CLK_PLL4]);
	clk_set_parent(clks[IMX6UL_PLL5_BYPASS], clks[IMX6UL_CLK_PLL5]);
	clk_set_parent(clks[IMX6UL_PLL6_BYPASS], clks[IMX6UL_CLK_PLL6]);
	clk_set_parent(clks[IMX6UL_PLL7_BYPASS], clks[IMX6UL_CLK_PLL7]);

	clks[IMX6UL_CLK_PLL1_SYS]	= imx_clk_fixed_factor("pll1_sys",	"pll1_bypass", 1, 1);
	clks[IMX6UL_CLK_PLL2_BUS]	= imx_clk_gate("pll2_bus",	"pll2_bypass", base + 0x30, 13);
	clks[IMX6UL_CLK_PLL3_USB_OTG]	= imx_clk_gate("pll3_usb_otg",	"pll3_bypass", base + 0x10, 13);
	clks[IMX6UL_CLK_PLL4_AUDIO]	= imx_clk_gate("pll4_audio",	"pll4_bypass", base + 0x70, 13);
	clks[IMX6UL_CLK_PLL5_VIDEO]	= imx_clk_gate("pll5_video",	"pll5_bypass", base + 0xa0, 13);
	clks[IMX6UL_CLK_PLL6_ENET]	= imx_clk_gate("pll6_enet",	"pll6_bypass", base + 0xe0, 13);
	clks[IMX6UL_CLK_PLL7_USB_HOST]	= imx_clk_gate("pll7_usb_host",	"pll7_bypass", base + 0x20, 13);

	/*
	 * Bit 20 is the reserved and read-only bit, we do this only for:
	 * - Do nothing for usbphy clk_enable/disable
	 * - Keep refcount when do usbphy clk_enable/disable, in that case,
	 * the clk framework many need to enable/disable usbphy's parent
	 */
	clks[IMX6UL_CLK_USBPHY1] = imx_clk_gate("usbphy1", "pll3_usb_otg",  base + 0x10, 20);
	clks[IMX6UL_CLK_USBPHY2] = imx_clk_gate("usbphy2", "pll7_usb_host", base + 0x20, 20);

	/*
	 * usbphy*_gate needs to be on after system boots up, and software
	 * never needs to control it anymore.
	 */
	clks[IMX6UL_CLK_USBPHY1_GATE] = imx_clk_gate("usbphy1_gate", "dummy", base + 0x10, 6);
	clks[IMX6UL_CLK_USBPHY2_GATE] = imx_clk_gate("usbphy2_gate", "dummy", base + 0x20, 6);

	/*					name		   parent_name	   reg		idx */
	clks[IMX6UL_CLK_PLL2_PFD0] = imx_clk_pfd("pll2_pfd0_352m", "pll2_bus",	   base + 0x100, 0);
	clks[IMX6UL_CLK_PLL2_PFD1] = imx_clk_pfd("pll2_pfd1_594m", "pll2_bus",	   base + 0x100, 1);
	clks[IMX6UL_CLK_PLL2_PFD2] = imx_clk_pfd("pll2_pfd2_396m", "pll2_bus",	   base + 0x100, 2);
	clks[IMX6UL_CLK_PLL2_PFD3] = imx_clk_pfd("pll2_pfd3_594m", "pll2_bus",	   base + 0x100, 3);
	clks[IMX6UL_CLK_PLL3_PFD0] = imx_clk_pfd("pll3_pfd0_720m", "pll3_usb_otg", base + 0xf0,  0);
	clks[IMX6UL_CLK_PLL3_PFD1] = imx_clk_pfd("pll3_pfd1_540m", "pll3_usb_otg", base + 0xf0,  1);
	clks[IMX6UL_CLK_PLL3_PFD2] = imx_clk_pfd("pll3_pfd2_508m", "pll3_usb_otg", base + 0xf0,	 2);
	clks[IMX6UL_CLK_PLL3_PFD3] = imx_clk_pfd("pll3_pfd3_454m", "pll3_usb_otg", base + 0xf0,	 3);

	clks[IMX6UL_CLK_ENET_REF] = clk_register_divider_table(NULL, "enet_ref", "pll6_enet", 0,
			base + 0xe0, 0, 2, 0, clk_enet_ref_table, &imx_ccm_lock);
	clks[IMX6UL_CLK_ENET2_REF] = clk_register_divider_table(NULL, "enet2_ref", "pll6_enet", 0,
			base + 0xe0, 2, 2, 0, clk_enet_ref_table, &imx_ccm_lock);

	clks[IMX6UL_CLK_ENET2_REF_125M] = imx_clk_gate("enet_ref_125m", "enet2_ref", base + 0xe0, 20);
	clks[IMX6UL_CLK_ENET_PTP_REF]	= imx_clk_fixed_factor("enet_ptp_ref", "pll6_enet", 1, 20);
	clks[IMX6UL_CLK_ENET_PTP]	= imx_clk_gate("enet_ptp", "enet_ptp_ref", base + 0xe0, 21);

	clks[IMX6UL_CLK_PLL4_POST_DIV]  = clk_register_divider_table(NULL, "pll4_post_div", "pll4_audio",
		 CLK_SET_RATE_PARENT | CLK_SET_RATE_GATE, base + 0x70, 19, 2, 0, post_div_table, &imx_ccm_lock);
	clks[IMX6UL_CLK_PLL4_AUDIO_DIV] = clk_register_divider(NULL, "pll4_audio_div", "pll4_post_div",
		 CLK_SET_RATE_PARENT | CLK_SET_RATE_GATE, base + 0x170, 15, 1, 0, &imx_ccm_lock);
	clks[IMX6UL_CLK_PLL5_POST_DIV]  = clk_register_divider_table(NULL, "pll5_post_div", "pll5_video",
		 CLK_SET_RATE_PARENT | CLK_SET_RATE_GATE, base + 0xa0, 19, 2, 0, post_div_table, &imx_ccm_lock);
	clks[IMX6UL_CLK_PLL5_VIDEO_DIV] = clk_register_divider_table(NULL, "pll5_video_div", "pll5_post_div",
		 CLK_SET_RATE_PARENT | CLK_SET_RATE_GATE, base + 0x170, 30, 2, 0, video_div_table, &imx_ccm_lock);

	/*						   name		parent_name	 mult  div */
	clks[IMX6UL_CLK_PLL2_198M] = imx_clk_fixed_factor("pll2_198m", "pll2_pfd2_396m", 1,	2);
	clks[IMX6UL_CLK_PLL3_80M]  = imx_clk_fixed_factor("pll3_80m",  "pll3_usb_otg",   1,	6);
	clks[IMX6UL_CLK_PLL3_60M]  = imx_clk_fixed_factor("pll3_60m",  "pll3_usb_otg",   1,	8);
	clks[IMX6UL_CLK_GPT_3M]	   = imx_clk_fixed_factor("gpt_3m",	"osc",		 1,	8);

	np = ccm_node;
	base = of_iomap(np, 0);
	WARN_ON(!base);

	clks[IMX6UL_CA7_SECONDARY_SEL]	  = imx_clk_mux("ca7_secondary_sel", base + 0xc, 3, 1, ca7_secondary_sels, ARRAY_SIZE(ca7_secondary_sels));
	clks[IMX6UL_CLK_STEP]		  = imx_clk_mux("step", base + 0x0c, 8, 1, step_sels, ARRAY_SIZE(step_sels));
	clks[IMX6UL_CLK_PLL1_SW] 	  = imx_clk_mux_glitchless("pll1_sw",   base + 0x0c, 2,  1, pll1_sw_sels, ARRAY_SIZE(pll1_sw_sels));
	clks[IMX6UL_CLK_AXI_ALT_SEL]	  = imx_clk_mux("axi_alt_sel",		base + 0x14, 7,  1, axi_alt_sels, ARRAY_SIZE(axi_alt_sels));
	clks[IMX6UL_CLK_AXI_SEL]	  = imx_clk_mux_flags("axi_sel",	base + 0x14, 6,  1, axi_sels, ARRAY_SIZE(axi_sels), 0);
	clks[IMX6UL_CLK_PERIPH_PRE]	  = imx_clk_mux_bus("periph_pre",       base + 0x18, 18, 2, periph_pre_sels, ARRAY_SIZE(periph_pre_sels));
	clks[IMX6UL_CLK_PERIPH2_PRE]	  = imx_clk_mux_bus("periph2_pre",      base + 0x18, 21, 2, periph2_pre_sels, ARRAY_SIZE(periph2_pre_sels));
	clks[IMX6UL_CLK_PERIPH_CLK2_SEL]  = imx_clk_mux_bus("periph_clk2_sel",  base + 0x18, 12, 2, periph_clk2_sels, ARRAY_SIZE(periph_clk2_sels));
	clks[IMX6UL_CLK_PERIPH2_CLK2_SEL] = imx_clk_mux_bus("periph2_clk2_sel", base + 0x18, 20, 1, periph2_clk2_sels, ARRAY_SIZE(periph2_clk2_sels));
	clks[IMX6UL_CLK_EIM_SLOW_SEL]	  = imx_clk_mux("eim_slow_sel", base + 0x1c, 29, 2, eim_slow_sels, ARRAY_SIZE(eim_slow_sels));
	clks[IMX6UL_CLK_GPMI_SEL]	  = imx_clk_mux("gpmi_sel",     base + 0x1c, 19, 1, gpmi_sels, ARRAY_SIZE(gpmi_sels));
	clks[IMX6UL_CLK_BCH_SEL]	  = imx_clk_mux("bch_sel",	base + 0x1c, 18, 1, bch_sels, ARRAY_SIZE(bch_sels));
	clks[IMX6UL_CLK_USDHC2_SEL]	  = imx_clk_mux("usdhc2_sel",   base + 0x1c, 17, 1, usdhc_sels, ARRAY_SIZE(usdhc_sels));
	clks[IMX6UL_CLK_USDHC1_SEL]	  = imx_clk_mux("usdhc1_sel",   base + 0x1c, 16, 1, usdhc_sels, ARRAY_SIZE(usdhc_sels));
	clks[IMX6UL_CLK_SAI3_SEL]	  = imx_clk_mux("sai3_sel",     base + 0x1c, 14, 2, sai_sels, ARRAY_SIZE(sai_sels));
	clks[IMX6UL_CLK_SAI2_SEL]         = imx_clk_mux("sai2_sel",     base + 0x1c, 12, 2, sai_sels, ARRAY_SIZE(sai_sels));
	clks[IMX6UL_CLK_SAI1_SEL]	  = imx_clk_mux("sai1_sel",     base + 0x1c, 10, 2, sai_sels, ARRAY_SIZE(sai_sels));
	clks[IMX6UL_CLK_QSPI1_SEL]	  = imx_clk_mux("qspi1_sel",    base + 0x1c, 7,  3, qspi1_sels, ARRAY_SIZE(qspi1_sels));
	clks[IMX6UL_CLK_PERCLK_SEL]	  = imx_clk_mux("perclk_sel",	base + 0x1c, 6,  1, perclk_sels, ARRAY_SIZE(perclk_sels));
	clks[IMX6UL_CLK_CAN_SEL]	  = imx_clk_mux("can_sel",	base + 0x20, 8,  2, can_sels, ARRAY_SIZE(can_sels));
	if (clk_on_imx6ull())
		clks[IMX6UL_CLK_ESAI_SEL] = imx_clk_mux("esai_sel",	base + 0x20, 19, 2, esai_sels, ARRAY_SIZE(esai_sels));
	clks[IMX6UL_CLK_UART_SEL]	  = imx_clk_mux("uart_sel",	base + 0x24, 6,  1, uart_sels, ARRAY_SIZE(uart_sels));
	clks[IMX6UL_CLK_ENFC_SEL]	  = imx_clk_mux("enfc_sel",	base + 0x2c, 15, 3, enfc_sels, ARRAY_SIZE(enfc_sels));
	clks[IMX6UL_CLK_LDB_DI0_SEL]	  = imx_clk_mux("ldb_di0_sel",	base + 0x2c, 9,  3, ldb_di0_sels, ARRAY_SIZE(ldb_di0_sels));
	clks[IMX6UL_CLK_SPDIF_SEL]	  = imx_clk_mux("spdif_sel",	base + 0x30, 20, 2, spdif_sels, ARRAY_SIZE(spdif_sels));
	if (clk_on_imx6ul()) {
		clks[IMX6UL_CLK_SIM_PRE_SEL] 	  = imx_clk_mux("sim_pre_sel",	base + 0x34, 15, 3, sim_pre_sels, ARRAY_SIZE(sim_pre_sels));
		clks[IMX6UL_CLK_SIM_SEL]	  = imx_clk_mux("sim_sel", 	base + 0x34, 9, 3, sim_sels, ARRAY_SIZE(sim_sels));
	} else {
		clks[IMX6UL_CLK_EPDC_PRE_SEL]	  = imx_clk_mux("epdc_pre_sel",	base + 0x34, 15, 3, epdc_pre_sels, ARRAY_SIZE(epdc_pre_sels));
		clks[IMX6UL_CLK_EPDC_SEL]	  = imx_clk_mux("epdc_sel",	base + 0x34, 9, 3, epdc_sels, ARRAY_SIZE(epdc_sels));
	}
	clks[IMX6UL_CLK_ECSPI_SEL]	  = imx_clk_mux("ecspi_sel",	base + 0x38, 18, 1, ecspi_sels, ARRAY_SIZE(ecspi_sels));
	clks[IMX6UL_CLK_LCDIF_PRE_SEL]	  = imx_clk_mux("lcdif_pre_sel", base + 0x38, 15, 3, lcdif_pre_sels, ARRAY_SIZE(lcdif_pre_sels));
	clks[IMX6UL_CLK_LCDIF_SEL]	  = imx_clk_mux("lcdif_sel",	base + 0x38, 9, 3, lcdif_sels, ARRAY_SIZE(lcdif_sels));
	clks[IMX6UL_CLK_CSI_SEL] 	  = imx_clk_mux_flags("csi_sel", base + 0x3c, 9, 2, csi_sels, ARRAY_SIZE(csi_sels), CLK_SET_RATE_PARENT);

	clks[IMX6UL_CLK_LDB_DI0_DIV_SEL]  = imx_clk_mux("ldb_di0", base + 0x20, 10, 1, ldb_di0_div_sels, ARRAY_SIZE(ldb_di0_div_sels));
	clks[IMX6UL_CLK_LDB_DI1_DIV_SEL]  = imx_clk_mux("ldb_di1", base + 0x20, 11, 1, ldb_di1_div_sels, ARRAY_SIZE(ldb_di1_div_sels));

	clks[IMX6UL_CLK_LDB_DI0_DIV_3_5] = imx_clk_fixed_factor("ldb_di0_div_3_5", "ldb_di0_sel", 2, 7);
	clks[IMX6UL_CLK_LDB_DI0_DIV_7]	 = imx_clk_fixed_factor("ldb_di0_div_7",   "ldb_di0_sel", 1, 7);
	clks[IMX6UL_CLK_LDB_DI1_DIV_3_5] = imx_clk_fixed_factor("ldb_di1_div_3_5", "qspi1_sel", 2, 7);
	clks[IMX6UL_CLK_LDB_DI1_DIV_7]	 = imx_clk_fixed_factor("ldb_di1_div_7",   "qspi1_sel", 1, 7);

	clks[IMX6UL_CLK_PERIPH]  = imx_clk_busy_mux("periph",  base + 0x14, 25, 1, base + 0x48, 5, periph_sels, ARRAY_SIZE(periph_sels));
	clks[IMX6UL_CLK_PERIPH2] = imx_clk_busy_mux("periph2", base + 0x14, 26, 1, base + 0x48, 3, periph2_sels, ARRAY_SIZE(periph2_sels));

	clks[IMX6UL_CLK_PERIPH_CLK2]	= imx_clk_divider("periph_clk2",   "periph_clk2_sel",	base + 0x14, 27, 3);
	clks[IMX6UL_CLK_PERIPH2_CLK2]	= imx_clk_divider("periph2_clk2",  "periph2_clk2_sel",	base + 0x14, 0,  3);
	clks[IMX6UL_CLK_IPG]		= imx_clk_divider("ipg",	   "ahb",		base + 0x14, 8,	 2);
	clks[IMX6UL_CLK_LCDIF_PODF]	= imx_clk_divider("lcdif_podf",	   "lcdif_pred",	base + 0x18, 23, 3);
	clks[IMX6UL_CLK_QSPI1_PDOF]	= imx_clk_divider("qspi1_podf",	   "qspi1_sel",		base + 0x1c, 26, 3);
	clks[IMX6UL_CLK_EIM_SLOW_PODF]	= imx_clk_divider("eim_slow_podf", "eim_slow_sel",	base + 0x1c, 23, 3);
	clks[IMX6UL_CLK_PERCLK]		= imx_clk_divider("perclk",	   "perclk_sel",	base + 0x1c, 0,  6);
	clks[IMX6UL_CLK_CAN_PODF]	= imx_clk_divider("can_podf",	   "can_sel",		base + 0x20, 2,  6);
	clks[IMX6UL_CLK_GPMI_PODF]	= imx_clk_divider("gpmi_podf",	   "gpmi_sel",		base + 0x24, 22, 3);
	clks[IMX6UL_CLK_BCH_PODF]	= imx_clk_divider("bch_podf",	   "bch_sel",		base + 0x24, 19, 3);
	clks[IMX6UL_CLK_USDHC2_PODF]	= imx_clk_divider("usdhc2_podf",   "usdhc2_sel",	base + 0x24, 16, 3);
	clks[IMX6UL_CLK_USDHC1_PODF]	= imx_clk_divider("usdhc1_podf",   "usdhc1_sel",	base + 0x24, 11, 3);
	clks[IMX6UL_CLK_UART_PODF]	= imx_clk_divider("uart_podf",	   "uart_sel",		base + 0x24, 0,  6);
	clks[IMX6UL_CLK_SAI3_PRED]	= imx_clk_divider("sai3_pred",	   "sai3_sel",		base + 0x28, 22, 3);
	clks[IMX6UL_CLK_SAI3_PODF]	= imx_clk_divider("sai3_podf",	   "sai3_pred",		base + 0x28, 16, 6);
	clks[IMX6UL_CLK_SAI1_PRED]	= imx_clk_divider("sai1_pred",	   "sai1_sel",		base + 0x28, 6,	 3);
	clks[IMX6UL_CLK_SAI1_PODF]	= imx_clk_divider("sai1_podf",	   "sai1_pred",		base + 0x28, 0,	 6);
	if (clk_on_imx6ull()) {
		clks[IMX6UL_CLK_ESAI_PRED]	= imx_clk_divider("esai_pred",     "esai_sel",		base + 0x28, 9,  3);
		clks[IMX6UL_CLK_ESAI_PODF]	= imx_clk_divider("esai_podf",     "esai_pred",		base + 0x28, 25, 3);
	}
	clks[IMX6UL_CLK_ENFC_PRED]	= imx_clk_divider("enfc_pred",	   "enfc_sel",		base + 0x2c, 18, 3);
	clks[IMX6UL_CLK_ENFC_PODF]	= imx_clk_divider("enfc_podf",	   "enfc_pred",		base + 0x2c, 21, 6);
	clks[IMX6UL_CLK_SAI2_PRED]	= imx_clk_divider("sai2_pred",	   "sai2_sel",		base + 0x2c, 6,	 3);
	clks[IMX6UL_CLK_SAI2_PODF]	= imx_clk_divider("sai2_podf",	   "sai2_pred",		base + 0x2c, 0,  6);
	clks[IMX6UL_CLK_SPDIF_PRED]	= imx_clk_divider("spdif_pred",	   "spdif_sel",		base + 0x30, 25, 3);
	clks[IMX6UL_CLK_SPDIF_PODF]	= imx_clk_divider("spdif_podf",	   "spdif_pred",	base + 0x30, 22, 3);
	if (clk_on_imx6ul())
		clks[IMX6UL_CLK_SIM_PODF]	= imx_clk_divider("sim_podf",	   "sim_pre_sel",	base + 0x34, 12, 3);
	else
		clks[IMX6UL_CLK_EPDC_PODF]	= imx_clk_divider("epdc_podf",	   "epdc_pre_sel",	base + 0x34, 12, 3);
	clks[IMX6UL_CLK_ECSPI_PODF]	= imx_clk_divider("ecspi_podf",	   "ecspi_sel",		base + 0x38, 19, 6);
	clks[IMX6UL_CLK_LCDIF_PRED]	= imx_clk_divider("lcdif_pred",	   "lcdif_pre_sel",	base + 0x38, 12, 3);
	clks[IMX6UL_CLK_CSI_PODF]       = imx_clk_divider("csi_podf",      "csi_sel",           base + 0x3c, 11, 3);

	clks[IMX6UL_CLK_ARM]		= imx_clk_busy_divider("arm",	    "pll1_sw",	base +	0x10, 0,  3,  base + 0x48, 16);
	clks[IMX6UL_CLK_MMDC_PODF]	= imx_clk_busy_divider("mmdc_podf", "periph2",	base +  0x14, 3,  3,  base + 0x48, 2);
	clks[IMX6UL_CLK_AXI_PODF]	= imx_clk_busy_divider("axi_podf",  "axi_sel",	base +  0x14, 16, 3,  base + 0x48, 0);
	clks[IMX6UL_CLK_AHB]		= imx_clk_busy_divider("ahb",	    "periph",	base +  0x14, 10, 3,  base + 0x48, 1);

	/* CCGR0 */
	clks[IMX6UL_CLK_AIPSTZ1]	= imx_clk_gate2("aips_tz1",	"ahb",		base + 0x68,	0);
	clks[IMX6UL_CLK_AIPSTZ2]	= imx_clk_gate2("aips_tz2",	"ahb",		base + 0x68,	2);
	clks[IMX6UL_CLK_APBHDMA]	= imx_clk_gate2("apbh_dma",	"bch_podf",	base + 0x68,	4);
	clks[IMX6UL_CLK_ASRC_IPG]	= imx_clk_gate2_shared("asrc_ipg",	"ahb",	base + 0x68,	6, &share_count_asrc);
	clks[IMX6UL_CLK_ASRC_MEM]	= imx_clk_gate2_shared("asrc_mem",	"ahb",	base + 0x68,	6, &share_count_asrc);
	if (clk_on_imx6ul()) {
		clks[IMX6UL_CLK_CAAM_MEM]	= imx_clk_gate2("caam_mem",	"ahb",		base + 0x68,	8);
		clks[IMX6UL_CLK_CAAM_ACLK]	= imx_clk_gate2("caam_aclk",	"ahb",		base + 0x68,	10);
		clks[IMX6UL_CLK_CAAM_IPG]	= imx_clk_gate2("caam_ipg",	"ipg",		base + 0x68,	12);
	} else {
		clks[IMX6UL_CLK_DCP_CLK]	= imx_clk_gate2("dcp",		"ahb",		base + 0x68,	10);
		clks[IMX6UL_CLK_ENET]		= imx_clk_gate2("enet",		"ipg",		base + 0x68,	12);
		clks[IMX6UL_CLK_ENET_AHB]	= imx_clk_gate2("enet_ahb",	"ahb",		base + 0x68,	12);
	}
	clks[IMX6UL_CLK_CAN1_IPG]	= imx_clk_gate2("can1_ipg",	"ipg",		base + 0x68,	14);
	clks[IMX6UL_CLK_CAN1_SERIAL]	= imx_clk_gate2("can1_serial",	"can_podf",	base + 0x68,	16);
	clks[IMX6UL_CLK_CAN2_IPG]	= imx_clk_gate2("can2_ipg",	"ipg",		base + 0x68,	18);
	clks[IMX6UL_CLK_CAN2_SERIAL]	= imx_clk_gate2("can2_serial",	"can_podf",	base + 0x68,	20);
	clks[IMX6UL_CLK_GPT2_BUS]	= imx_clk_gate2("gpt2_bus",	"perclk",	base + 0x68,	24);
	clks[IMX6UL_CLK_GPT2_SERIAL]	= imx_clk_gate2("gpt2_serial",	"perclk",	base + 0x68,	26);
	clks[IMX6UL_CLK_UART2_IPG]	= imx_clk_gate2("uart2_ipg",	"ipg",		base + 0x68,	28);
	clks[IMX6UL_CLK_UART2_SERIAL]	= imx_clk_gate2("uart2_serial",	"uart_podf",	base + 0x68,	28);
	if (clk_on_imx6ul())
		clks[IMX6UL_CLK_AIPSTZ3]	= imx_clk_gate2("aips_tz3",	"ahb",		base + 0x68,	30);

	/* CCGR1 */
	clks[IMX6UL_CLK_ECSPI1]		= imx_clk_gate2("ecspi1",	"ecspi_podf",	base + 0x6c,	0);
	clks[IMX6UL_CLK_ECSPI2]		= imx_clk_gate2("ecspi2",	"ecspi_podf",	base + 0x6c,	2);
	clks[IMX6UL_CLK_ECSPI3]		= imx_clk_gate2("ecspi3",	"ecspi_podf",	base + 0x6c,	4);
	clks[IMX6UL_CLK_ECSPI4]		= imx_clk_gate2("ecspi4",	"ecspi_podf",	base + 0x6c,	6);
	clks[IMX6UL_CLK_ADC2]		= imx_clk_gate2("adc2",		"ipg",		base + 0x6c,	8);
	clks[IMX6UL_CLK_UART3_IPG]	= imx_clk_gate2("uart3_ipg",	"ipg",		base + 0x6c,	10);
	clks[IMX6UL_CLK_UART3_SERIAL]	= imx_clk_gate2("uart3_serial",	"uart_podf",	base + 0x6c,	10);
	clks[IMX6UL_CLK_EPIT1]		= imx_clk_gate2("epit1",	"perclk",	base + 0x6c,	12);
	clks[IMX6UL_CLK_EPIT2]		= imx_clk_gate2("epit2",	"perclk",	base + 0x6c,	14);
	clks[IMX6UL_CLK_ADC1]		= imx_clk_gate2("adc1",		"ipg",		base + 0x6c,	16);
	clks[IMX6UL_CLK_GPT1_BUS]	= imx_clk_gate2("gpt1_bus",	"perclk",	base + 0x6c,	20);
	clks[IMX6UL_CLK_GPT1_SERIAL]	= imx_clk_gate2("gpt1_serial",	"perclk",	base + 0x6c,	22);
	clks[IMX6UL_CLK_UART4_IPG]	= imx_clk_gate2("uart4_ipg",	"ipg",		base + 0x6c,	24);
	clks[IMX6UL_CLK_UART4_SERIAL]	= imx_clk_gate2("uart4_serail",	"uart_podf",	base + 0x6c,	24);

	/* CCGR2 */
	if (clk_on_imx6ull()) {
		clks[IMX6UL_CLK_ESAI_EXTAL]	= imx_clk_gate2_shared("esai_extal",	"esai_podf",	base + 0x70,	0, &share_count_esai);
		clks[IMX6UL_CLK_ESAI_IPG]	= imx_clk_gate2_shared("esai_ipg",	"ahb",		base + 0x70,	0, &share_count_esai);
		clks[IMX6UL_CLK_ESAI_MEM]	= imx_clk_gate2_shared("esai_mem",	"ahb",		base + 0x70,	0, &share_count_esai);
	}
	clks[IMX6UL_CLK_CSI]		= imx_clk_gate2("csi",		"csi_podf",		base + 0x70,	2);
	clks[IMX6UL_CLK_I2C1]		= imx_clk_gate2("i2c1",		"perclk",	base + 0x70,	6);
	clks[IMX6UL_CLK_I2C2]		= imx_clk_gate2("i2c2",		"perclk",	base + 0x70,	8);
	clks[IMX6UL_CLK_I2C3]		= imx_clk_gate2("i2c3",		"perclk",	base + 0x70,	10);
	clks[IMX6UL_CLK_OCOTP]		= imx_clk_gate2("ocotp",	"ipg",		base + 0x70,	12);
	clks[IMX6UL_CLK_IOMUXC]		= imx_clk_gate2("iomuxc",	"lcdif_podf",	base + 0x70,	14);
	clks[IMX6UL_CLK_LCDIF_APB]	= imx_clk_gate2("lcdif_apb",	"axi",		base + 0x70,	28);
	clks[IMX6UL_CLK_PXP]		= imx_clk_gate2("pxp",		"axi",		base + 0x70,	30);

	/* CCGR3 */
	clks[IMX6UL_CLK_UART5_IPG]	= imx_clk_gate2("uart5_ipg",	"ipg",		base + 0x74,	2);
	clks[IMX6UL_CLK_UART5_SERIAL]	= imx_clk_gate2("uart5_serial",	"uart_podf",	base + 0x74,	2);
	if (clk_on_imx6ul()) {
		clks[IMX6UL_CLK_ENET]		= imx_clk_gate2("enet",		"ipg",		base + 0x74,	4);
		clks[IMX6UL_CLK_ENET_AHB]	= imx_clk_gate2("enet_ahb",	"ahb",		base + 0x74,	4);
	} else {
		clks[IMX6UL_CLK_EPDC_ACLK]	= imx_clk_gate2("epdc_aclk",	"axi",		base + 0x74,	4);
		clks[IMX6UL_CLK_EPDC_PIX]	= imx_clk_gate2("epdc_pix",	"epdc_podf",	base + 0x74,	4);
	}
	clks[IMX6UL_CLK_UART6_IPG]	= imx_clk_gate2("uart6_ipg",	"ipg",		base + 0x74,	6);
	clks[IMX6UL_CLK_UART6_SERIAL]	= imx_clk_gate2("uart6_serial",	"uart_podf",	base + 0x74,	6);
	clks[IMX6UL_CLK_LCDIF_PIX]	= imx_clk_gate2("lcdif_pix",	"lcdif_podf",	base + 0x74,	10);
	clks[IMX6UL_CLK_QSPI]		= imx_clk_gate2("qspi1",	"qspi1_podf",	base + 0x74,	14);
	clks[IMX6UL_CLK_WDOG1]		= imx_clk_gate2("wdog1",	"ipg",		base + 0x74,	16);
	clks[IMX6UL_CLK_MMDC_P0_FAST]	= imx_clk_gate("mmdc_p0_fast", "mmdc_podf", base + 0x74,	20);
	clks[IMX6UL_CLK_MMDC_P0_IPG]	= imx_clk_gate2("mmdc_p0_ipg",	"ipg",		base + 0x74,	24);
	clks[IMX6UL_CLK_AXI]		= imx_clk_gate("axi",	"axi_podf",	base + 0x74,	28);

	/* CCGR4 */
	clks[IMX6UL_CLK_PER_BCH]	= imx_clk_gate2("per_bch",	"bch_podf",	base + 0x78,	12);
	clks[IMX6UL_CLK_PWM1]		= imx_clk_gate2("pwm1",		"perclk",	base + 0x78,	16);
	clks[IMX6UL_CLK_PWM2]		= imx_clk_gate2("pwm2",		"perclk",	base + 0x78,	18);
	clks[IMX6UL_CLK_PWM3]		= imx_clk_gate2("pwm3",		"perclk",	base + 0x78,	20);
	clks[IMX6UL_CLK_PWM4]		= imx_clk_gate2("pwm4",		"perclk",	base + 0x78,	22);
	clks[IMX6UL_CLK_GPMI_BCH_APB]	= imx_clk_gate2("gpmi_bch_apb",	"bch_podf",	base + 0x78,	24);
	clks[IMX6UL_CLK_GPMI_BCH]	= imx_clk_gate2("gpmi_bch",	"gpmi_podf",	base + 0x78,	26);
	clks[IMX6UL_CLK_GPMI_IO]	= imx_clk_gate2("gpmi_io",	"enfc_podf",	base + 0x78,	28);
	clks[IMX6UL_CLK_GPMI_APB]	= imx_clk_gate2("gpmi_apb",	"bch_podf",	base + 0x78,	30);

	/* CCGR5 */
	clks[IMX6UL_CLK_ROM]		= imx_clk_gate2("rom",		"ahb",		base + 0x7c,	0);
	clks[IMX6UL_CLK_SDMA]		= imx_clk_gate2("sdma",		"ahb",		base + 0x7c,	6);
	clks[IMX6UL_CLK_KPP]		= imx_clk_gate2("kpp",		"ipg",		base + 0x7c,	8);
	clks[IMX6UL_CLK_WDOG2]		= imx_clk_gate2("wdog2",	"ipg",		base + 0x7c,	10);
	clks[IMX6UL_CLK_SPBA]		= imx_clk_gate2("spba",		"ipg",		base + 0x7c,	12);
	clks[IMX6UL_CLK_SPDIF]		= imx_clk_gate2_shared("spdif",		"spdif_podf",	base + 0x7c,	14, &share_count_audio);
	clks[IMX6UL_CLK_SPDIF_GCLK]	= imx_clk_gate2_shared("spdif_gclk",	"ipg",		base + 0x7c,	14, &share_count_audio);
	clks[IMX6UL_CLK_SAI3]		= imx_clk_gate2_shared("sai3",		"sai3_podf",	base + 0x7c,	22, &share_count_sai3);
	clks[IMX6UL_CLK_SAI3_IPG]	= imx_clk_gate2_shared("sai3_ipg",	"ipg",		base + 0x7c,	22, &share_count_sai3);
	clks[IMX6UL_CLK_UART1_IPG]	= imx_clk_gate2("uart1_ipg",	"ipg",		base + 0x7c,	24);
	clks[IMX6UL_CLK_UART1_SERIAL]	= imx_clk_gate2("uart1_serial",	"uart_podf",	base + 0x7c,	24);
	clks[IMX6UL_CLK_UART7_IPG]	= imx_clk_gate2("uart7_ipg",	"ipg",		base + 0x7c,	26);
	clks[IMX6UL_CLK_UART7_SERIAL]	= imx_clk_gate2("uart7_serial",	"uart_podf",	base + 0x7c,	26);
	clks[IMX6UL_CLK_SAI1]		= imx_clk_gate2_shared("sai1",		"sai1_podf",	base + 0x7c,	28, &share_count_sai1);
	clks[IMX6UL_CLK_SAI1_IPG]	= imx_clk_gate2_shared("sai1_ipg",	"ipg",		base + 0x7c,	28, &share_count_sai1);
	clks[IMX6UL_CLK_SAI2]		= imx_clk_gate2_shared("sai2",		"sai2_podf",	base + 0x7c,	30, &share_count_sai2);
	clks[IMX6UL_CLK_SAI2_IPG]	= imx_clk_gate2_shared("sai2_ipg",	"ipg",		base + 0x7c,	30, &share_count_sai2);

	/* CCGR6 */
	clks[IMX6UL_CLK_USBOH3]		= imx_clk_gate2("usboh3",	"ipg",		 base + 0x80,	0);
	clks[IMX6UL_CLK_USDHC1]		= imx_clk_gate2("usdhc1",	"usdhc1_podf",	 base + 0x80,	2);
	clks[IMX6UL_CLK_USDHC2]		= imx_clk_gate2("usdhc2",	"usdhc2_podf",	 base + 0x80,	4);
	if (clk_on_imx6ul()) {
		clks[IMX6UL_CLK_SIM1]		= imx_clk_gate2("sim1",		"sim_sel",	 base + 0x80,	6);
		clks[IMX6UL_CLK_SIM2]		= imx_clk_gate2("sim2",		"sim_sel",	 base + 0x80,	8);
	}
	clks[IMX6UL_CLK_EIM]		= imx_clk_gate2("eim",		"eim_slow_podf", base + 0x80,	10);
	clks[IMX6UL_CLK_PWM8]		= imx_clk_gate2("pwm8",		"perclk",	 base + 0x80,	16);
	clks[IMX6UL_CLK_UART8_IPG]	= imx_clk_gate2("uart8_ipg",	"ipg",		 base + 0x80,	14);
	clks[IMX6UL_CLK_UART8_SERIAL]	= imx_clk_gate2("uart8_serial", "uart_podf",	 base + 0x80,	14);
	if (clk_on_imx6ull())
		clks[IMX6UL_CLK_AIPSTZ3]	= imx_clk_gate2("aips_tz3",	"ahb",		 base + 0x80,	18);
	clks[IMX6UL_CLK_WDOG3]		= imx_clk_gate2("wdog3",	"ipg",		 base + 0x80,	20);
	clks[IMX6UL_CLK_I2C4]		= imx_clk_gate2("i2c4",		"perclk",	 base + 0x80,	24);
	clks[IMX6UL_CLK_PWM5]		= imx_clk_gate2("pwm5",		"perclk",	 base + 0x80,	26);
	clks[IMX6UL_CLK_PWM6]		= imx_clk_gate2("pwm6",		"perclk",	 base +	0x80,	28);
	clks[IMX6UL_CLK_PWM7]		= imx_clk_gate2("pwm7",		"perclk",	 base + 0x80,	30);

	/* mask handshake of mmdc */
	writel_relaxed(BM_CCM_CCDR_MMDC_CH0_MASK, base + CCDR);

	imx_check_clocks(clks, ARRAY_SIZE(clks));

	clk_data.clks = clks;
	clk_data.clk_num = ARRAY_SIZE(clks);
	of_clk_add_provider(np, of_clk_src_onecell_get, &clk_data);

	/*
	 * Lower the AHB clock rate before changing the parent clock source,
	 * as AHB clock rate can NOT be higher than 133MHz, but its parent
	 * will be switched from 396MHz PFD to 528MHz PLL in order to increase
	 * AXI clock rate, so we need to lower AHB rate first to make sure at
	 * any time, AHB rate is <= 133MHz.
	 */
	clk_set_rate(clks[IMX6UL_CLK_AHB], 99000000);

	/* Change periph_pre clock to pll2_bus to adjust AXI rate to 264MHz */
	clk_set_parent(clks[IMX6UL_CLK_PERIPH_CLK2_SEL], clks[IMX6UL_CLK_PLL3_USB_OTG]);
	clk_set_parent(clks[IMX6UL_CLK_PERIPH], clks[IMX6UL_CLK_PERIPH_CLK2]);
	clk_set_parent(clks[IMX6UL_CLK_PERIPH_PRE], clks[IMX6UL_CLK_PLL2_BUS]);
	clk_set_parent(clks[IMX6UL_CLK_PERIPH], clks[IMX6UL_CLK_PERIPH_PRE]);

	/* Make sure AHB rate is 132MHz  */
	clk_set_rate(clks[IMX6UL_CLK_AHB], 132000000);

	/* set perclk to from OSC */
	clk_set_parent(clks[IMX6UL_CLK_PERCLK_SEL], clks[IMX6UL_CLK_OSC]);

        /* Set the UART parent if needed */
        if (uart_from_osc)
		imx_clk_set_parent(clks[IMX6UL_CLK_UART_SEL], clks[IMX6UL_CLK_OSC]);
        else
		imx_clk_set_parent(clks[IMX6UL_CLK_UART_SEL], clks[IMX6UL_CLK_PLL3_80M]);

	clk_set_rate(clks[IMX6UL_CLK_ENET_REF], 50000000);
	clk_set_rate(clks[IMX6UL_CLK_ENET2_REF], 50000000);
	clk_set_rate(clks[IMX6UL_CLK_CSI], 24000000);
	clk_set_rate(clks[IMX6UL_CLK_PLL3_PFD2], 320000000);

	/* keep all the clks on just for bringup */
	for (i = 0; i < ARRAY_SIZE(clks_init_on); i++)
		clk_prepare_enable(clks[clks_init_on[i]]);

	if (IS_ENABLED(CONFIG_USB_MXS_PHY)) {
		clk_prepare_enable(clks[IMX6UL_CLK_USBPHY1_GATE]);
		clk_prepare_enable(clks[IMX6UL_CLK_USBPHY2_GATE]);
	}

	clk_set_parent(clks[IMX6UL_CLK_CAN_SEL], clks[IMX6UL_CLK_PLL3_60M]);
	if (clk_on_imx6ul())
		clk_set_parent(clks[IMX6UL_CLK_SIM_PRE_SEL], clks[IMX6UL_CLK_PLL3_USB_OTG]);
	else
		clk_set_parent(clks[IMX6UL_CLK_EPDC_PRE_SEL], clks[IMX6UL_CLK_PLL3_PFD2]);

	clk_set_parent(clks[IMX6UL_CLK_ENFC_SEL], clks[IMX6UL_CLK_PLL2_PFD2]);

	/* Lower the AHB clock rate before changing the clock source. */
	imx_clk_set_rate(clks[IMX6UL_CLK_AHB], 99000000);

	/* Change periph_pre clock to pll2_bus to adjust AXI rate to 264MHz */
	imx_clk_set_parent(clks[IMX6UL_CLK_PERIPH_CLK2_SEL], clks[IMX6UL_CLK_PLL3_USB_OTG]);
	imx_clk_set_parent(clks[IMX6UL_CLK_PERIPH], clks[IMX6UL_CLK_PERIPH_CLK2]);
	imx_clk_set_parent(clks[IMX6UL_CLK_PERIPH_PRE], clks[IMX6UL_CLK_PLL2_BUS]);
	imx_clk_set_parent(clks[IMX6UL_CLK_PERIPH], clks[IMX6UL_CLK_PERIPH_PRE]);

	imx_clk_set_rate(clks[IMX6UL_CLK_AHB], 132000000);
}

```

#### 2-1 of_clk_get_by_name()

```C
/*因为其他时钟的处理相同，因此只分析时钟 "osc" */
struct clk *of_clk_get_by_name(struct device_node *np, const char *name)
{
    /* 
     * 传入的参数
     * 1. np: ccm@020c4000
     * 2. full_name: ccm
     * 3. name: osc
     */
	return __of_clk_get_by_name(np, np->full_name, name);
}



static struct clk *__of_clk_get_by_name(struct device_node *np,
					const char *dev_id,
					const char *name)
{
	struct clk *clk = ERR_PTR(-ENOENT);

	/* Walk up the tree of devices looking for a clock that matches */
	while (np) {
		int index = 0;

        /* 找到 "osc" 在 "clock-names" 的 index 偏移为1 */
		if (name)
			index = of_property_match_string(np, "clock-names", name);
        
        /* 
         * 传入的参数
         * 1. np: ccm@020c4000
         * 2. index：1
         * 2. dev_id: ccm
         * 3. name: osc
         * 找到"osc"的clk_provider,并返回其clk
         */
		clk = __of_clk_get(np, index, dev_id, name);
        
        np = np->parent;
	}

	return clk;
}
```

#### 2-1-1 __of_clk_get()

```C
static struct clk *__of_clk_get(struct device_node *np, int index,
			       const char *dev_id, const char *con_id)
{
	struct of_phandle_args clkspec;
	struct clk *clk;
	int rc;
	
    /* 
     * "clocks" 保存了phandle和长度为"#clock-cells"的参数,
     * 根据index偏移,找到phandle对应的node("ckil" node),
     * 并保存phandle 后面对应长度的所有参数(为0，无参数)。
     */
	rc = of_parse_phandle_with_args(np, "clocks", "#clock-cells", index,
					&clkspec);

    /*
     * 传入参数
     * 1. clkspec->node : "ckil" node
     * 2. dev_id：ccm
     * 3. con_id：ckil
     * 在全局list中,找到对应的clk_provider,返回其clk
     */
	clk = __of_clk_get_from_provider(&clkspec, dev_id, con_id);

	return clk;
}
```

#### 2-2 imx_clk_mux()

```C
/*
 * 只分析PLL1,传入参数
 * 1. "pll1_bypass_src"
 * 2. base + 0x00
 * 3. 14
 * 4. 1 
 * 5. pll_bypass_src_sels 
 * 6. ARRAY_SIZE(pll_bypass_src_sels)
 */
static inline struct clk *imx_clk_mux(const char *name, void __iomem *reg,
		u8 shift, u8 width, const char **parents, int num_parents)
{
	return clk_register_mux(NULL, name, parents, num_parents,
			CLK_SET_RATE_NO_REPARENT | CLK_SET_PARENT_GATE,
			reg, shift, width, 0, &imx_ccm_lock);
}
```
