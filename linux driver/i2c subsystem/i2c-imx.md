### dts 的 i2c 

```C
/* imx6ull.dtsi */
i2c1: i2c@021a0000 {
	#address-cells = <1>;
	#size-cells = <0>;
	compatible = "fsl,imx6ul-i2c", "fsl,imx21-i2c";
	reg = <0x021a0000 0x4000>;
	interrupts = <GIC_SPI 36 IRQ_TYPE_LEVEL_HIGH>;
	clocks = <&clks IMX6UL_CLK_I2C1>;
	status = "disabled";
};

/* 100ask_imx6ull-14x14.dts */
&i2c1 {
    clock-frequency = <100000>;
    pinctrl-names = "default";
    pinctrl-0 = <&pinctrl_i2c1>;
    status = "okay";
};
```

### struct imx_i2c_hwdata

```C
/*board类型*/
enum imx_i2c_type {
	IMX1_I2C,
	IMX21_I2C,
	VF610_I2C,
};

/*分频值对应关系*/
static struct imx_i2c_clk_pair imx_i2c_clk_div[] = {
	{ 22,	0x20 }, { 24,	0x21 }, { 26,	0x22 }, { 28,	0x23 },
	{ 30,	0x00 },	{ 32,	0x24 }, { 36,	0x25 }, { 40,	0x26 },
	{ 42,	0x03 }, { 44,	0x27 },	{ 48,	0x28 }, { 52,	0x05 },
	{ 56,	0x29 }, { 60,	0x06 }, { 64,	0x2A },	{ 72,	0x2B },
	{ 80,	0x2C }, { 88,	0x09 }, { 96,	0x2D }, { 104,	0x0A },
	{ 112,	0x2E }, { 128,	0x2F }, { 144,	0x0C }, { 160,	0x30 },
	{ 192,	0x31 },	{ 224,	0x32 }, { 240,	0x0F }, { 256,	0x33 },
	{ 288,	0x10 }, { 320,	0x34 },	{ 384,	0x35 }, { 448,	0x36 },
	{ 480,	0x13 }, { 512,	0x37 }, { 576,	0x14 },	{ 640,	0x38 },
	{ 768,	0x39 }, { 896,	0x3A }, { 960,	0x17 }, { 1024,	0x3B },
	{ 1152,	0x18 }, { 1280,	0x3C }, { 1536,	0x3D }, { 1792,	0x3E },
	{ 1920,	0x1B },	{ 2048,	0x3F }, { 2304,	0x1C }, { 2560,	0x1D },
	{ 3072,	0x1E }, { 3840,	0x1F }
};

struct imx_i2c_hwdata {
	enum imx_i2c_type	devtype;
	unsigned		regshift;
	struct imx_i2c_clk_pair	*clk_div;
	unsigned		ndivs;
	unsigned		i2sr_clr_opcode;
	unsigned		i2cr_ien_opcode;
};
```

### i2c_adap_imx_init()

```C
static const struct imx_i2c_hwdata imx21_i2c_hwdata = {
	.devtype		= IMX21_I2C,
	.regshift		= IMX_I2C_REGSHIFT,
	.clk_div		= imx_i2c_clk_div,
	.ndivs			= ARRAY_SIZE(imx_i2c_clk_div),
	.i2sr_clr_opcode	= I2SR_CLR_OPCODE_W0C,
	.i2cr_ien_opcode	= I2CR_IEN_OPCODE_1,

};

static const struct of_device_id i2c_imx_dt_ids[] = {
	{ .compatible = "fsl,imx1-i2c", .data = &imx1_i2c_hwdata, },
    
    /*在dts存在同名compatible，因此match匹配*/
	{ .compatible = "fsl,imx21-i2c", .data = &imx21_i2c_hwdata, },
	{ .compatible = "fsl,vf610-i2c", .data = &vf610_i2c_hwdata, },
	{ /* sentinel */ }
};

static struct platform_driver i2c_imx_driver = {
	.probe = i2c_imx_probe,
	.remove = i2c_imx_remove,
	.driver = {
		.name = DRIVER_NAME,
		.pm = I2C_IMX_PM_OPS,
		.of_match_table = i2c_imx_dt_ids,
	},
	.id_table = imx_i2c_devtype,
};


static int __init i2c_adap_imx_init(void)
{
	return platform_driver_register(&i2c_imx_driver);
}
```

### struct imx_i2c_struct

```C
struct imx_i2c_struct {
	struct i2c_adapter	adapter;
	struct clk		*clk;
	void __iomem		*base;
	wait_queue_head_t	queue;
	unsigned long		i2csr;
	unsigned int		disable_delay;
	int			stopped;
	unsigned int		ifdr; /* IMX_I2C_IFDR */
	unsigned int		cur_clk;
	unsigned int		bitrate;
	const struct imx_i2c_hwdata	*hwdata;
	struct i2c_bus_recovery_info rinfo;

	struct pinctrl *pinctrl;
	struct pinctrl_state *pinctrl_pins_default;
	struct pinctrl_state *pinctrl_pins_gpio;

	struct imx_i2c_dma	*dma;
};
```

### struct i2c_algorithm

```C
/**
 * struct i2c_algorithm - represent I2C transfer method
 * @master_xfer: Issue a set of i2c transactions to the given I2C adapter
 *   defined by the msgs array, with num messages available to transfer via
 *   the adapter specified by adap.
 * @smbus_xfer: Issue smbus transactions to the given I2C adapter. If this
 *   is not present, then the bus layer will try and convert the SMBus calls
 *   into I2C transfers instead.
 * @functionality: Return the flags that this algorithm/adapter pair supports
 *   from the I2C_FUNC_* flags.
 * @reg_slave: Register given client to I2C slave mode of this adapter
 * @unreg_slave: Unregister given client from I2C slave mode of this adapter
 *
 * The following structs are for those who like to implement new bus drivers:
 * i2c_algorithm is the interface to a class of hardware solutions which can
 * be addressed using the same bus algorithms - i.e. bit-banging or the PCF8584
 * to name two of the most common.
 *
 * The return codes from the @master_xfer field should indicate the type of
 * error code that occurred during the transfer, as documented in the kernel
 * Documentation file Documentation/i2c/fault-codes.
 */
struct i2c_algorithm {
	/* If an adapter algorithm can't do I2C-level access, set master_xfer
	   to NULL. If an adapter algorithm can do SMBus access, set
	   smbus_xfer. If set to NULL, the SMBus protocol is simulated
	   using common I2C messages */
	/* master_xfer should return the number of messages successfully
	   processed, or a negative value on error */
	int (*master_xfer)(struct i2c_adapter *adap, struct i2c_msg *msgs,
			   int num);
	int (*smbus_xfer) (struct i2c_adapter *adap, u16 addr,
			   unsigned short flags, char read_write,
			   u8 command, int size, union i2c_smbus_data *data);

	/* To determine what the adapter supports */
	u32 (*functionality) (struct i2c_adapter *);

#if IS_ENABLED(CONFIG_I2C_SLAVE)
	int (*reg_slave)(struct i2c_client *client);
	int (*unreg_slave)(struct i2c_client *client);
#endif
};


static struct i2c_algorithm i2c_imx_algo = {
	.master_xfer	= i2c_imx_xfer,
	.functionality	= i2c_imx_func,
};
```

### 1 i2c_imx_probe()

```C
/**
 * struct imxi2c_platform_data - structure of platform data for MXC I2C driver
 * @bitrate:	Bus speed measured in Hz
 *
 **/
struct imxi2c_platform_data {
	u32 bitrate;
};


static int i2c_imx_probe(struct platform_device *pdev)
{
    /*
     * i2c_bus_type的match匹配后，会执行probe，
     * 此时需要确认最佳匹配的类型，用于提取出私有内容data，
     * 用于初始化adapter，每个设备树描述的i2c都会调用该probe
     * 进行构造adapter。
     */
	const struct of_device_id *of_id = 
        of_match_device(i2c_imx_dt_ids,&pdev->dev);
    
    
	struct imx_i2c_struct *i2c_imx;
	struct resource *res;
	struct imxi2c_platform_data *pdata = dev_get_platdata(&pdev->dev);
	void __iomem *base;
	int irq, ret;
	dma_addr_t phy_addr;
    
   	dev_dbg(&pdev->dev, "<%s>\n", __func__);

    /*查找of_node是否存在irp相关属性，并提取其中断号*/
	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		dev_err(&pdev->dev, "can't get irq number\n");
		return irq;
	}
    
	/*获取of_node的resource*/
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
    /*将该i2c控制器的寄存器物理首地址，转为虚拟地址*/
	base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(base))
		return PTR_ERR(base);

    /*记录上述虚拟地址*/
	phy_addr = (dma_addr_t)res->start;
    
    /*
     * 申请 包含adapter的imx_i2c_struct 内存
     * 根据厂商的私有数据，构造adapter
     */
	i2c_imx = devm_kzalloc(&pdev->dev, sizeof(*i2c_imx), GFP_KERNEL);
	if (!i2c_imx)
		return -ENOMEM;

    /*找到私有数据 imx_i2c_hwdata */
	if (of_id)
		i2c_imx->hwdata = of_id->data;
	else
		i2c_imx->hwdata = (struct imx_i2c_hwdata *)
				platform_get_device_id(pdev)->driver_data;

	/* Setup i2c_imx driver structure */
    /*构建adapter和保存寄存器基址 */
	strlcpy(i2c_imx->adapter.name, pdev->name, sizeof(i2c_imx->adapter.name));
	i2c_imx->adapter.owner		= THIS_MODULE;
	i2c_imx->adapter.algo		= &i2c_imx_algo;
	i2c_imx->adapter.dev.parent	= &pdev->dev;
	i2c_imx->adapter.nr		= pdev->id;
	i2c_imx->adapter.dev.of_node	= pdev->dev.of_node;
	i2c_imx->base			= base;

	/* Get I2C clock */
    /*获取时钟源，确认配置的时钟值*/
	i2c_imx->clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(i2c_imx->clk)) {
		dev_err(&pdev->dev, "can't get I2C clock\n");
		return PTR_ERR(i2c_imx->clk);
	}
    
    /*根据获取值，配置和使能i2c时钟*/
	ret = clk_prepare_enable(i2c_imx->clk);
	if (ret) {
		dev_err(&pdev->dev, "can't enable I2C clock, ret=%d\n", ret);
		return ret;
	}
    
	/* Request IRQ */
    /*配置和开启irq*/
	ret = devm_request_irq(&pdev->dev, irq, i2c_imx_isr,
			       IRQF_NO_SUSPEND, pdev->name, i2c_imx);
	if (ret) {
		dev_err(&pdev->dev, "can't claim irq %d\n", irq);
		goto clk_disable;
	}
    
 	/* Init queue */
	init_waitqueue_head(&i2c_imx->queue);   
    
	/* Set up adapter data */
    /*保存包含adapter的数据，在dev私有指针*/
	i2c_set_adapdata(&i2c_imx->adapter, i2c_imx);

	/* Set up platform driver data */
	platform_set_drvdata(pdev, i2c_imx);

	/* Set up clock divider */
    /* 
     * 先设置默认的rate,再尝试读取设备树的属性"clock-frequency",
     * 获取其内容值，保存在i2c_imx->bitrate
     */
	i2c_imx->bitrate = IMX_I2C_BIT_RATE;
	ret = of_property_read_u32(pdev->dev.of_node,
				   "clock-frequency", &i2c_imx->bitrate);
	if (ret < 0 && pdata && pdata->bitrate)
		i2c_imx->bitrate = pdata->bitrate;

	/* Set up chip registers to defaults */
    /*设置 i2c_cr和i2c_sr寄存器为默认值*/
	imx_i2c_write_reg(i2c_imx->hwdata->i2cr_ien_opcode ^ I2CR_IEN,
			i2c_imx, IMX_I2C_I2CR);
	imx_i2c_write_reg(i2c_imx->hwdata->i2sr_clr_opcode, i2c_imx, IMX_I2C_I2SR);

	/* Init optional bus recovery function */
    /*可选功能，当i2c功能设为gpio功能,自动恢复引脚设置*/
	ret = i2c_imx_init_recovery_info(i2c_imx, pdev);
	/* Give it another chance if pinctrl used is not ready yet */
	if (ret == -EPROBE_DEFER)
		goto rpm_disable;

	/* Add I2C adapter */
    /*
     * 已经根据厂商数据，完成了对adapter的配置。
     * 这时添加一个adapter，并找到adapter下的子节点，
     * 也将client一并注册。
     */
	ret = i2c_add_numbered_adapter(&i2c_imx->adapter);
	if (ret < 0)
		goto rpm_disable;


	dev_info(&i2c_imx->adapter.dev, "IMX I2C adapter registered\n");


	return 0;   /* Return OK */
}

```

### 1-1 of_match_device()

```C
/*
 * Struct used for matching a device
 */
struct of_device_id {
	char	name[32];
	char	type[32];
	char	compatible[128];
	const void *data;
};

/**
 * of_match_device - Tell if a struct device matches an of_device_id list
 * @ids: array of of device match structures to search in
 * @dev: the of device structure to match against
 *
 * Used by a driver to check whether an platform_device present in the
 * system is in its list of supported devices.
 */
const struct of_device_id *of_match_device(const struct of_device_id *matches,
					   const struct device *dev)
{
	if ((!matches) || (!dev->of_node))
		return NULL;
	return of_match_node(matches, dev->of_node);
}



/**
 * of_match_node - Tell if a device_node has a matching of_match structure
 *	@matches:	array of of device match structures to search in
 *	@node:		the of device structure to match against
 *
 *	Low level utility function used by device matching.
 */
const struct of_device_id *of_match_node(const struct of_device_id *matches,
					 const struct device_node *node)
{
	const struct of_device_id *match;
	unsigned long flags;

	raw_spin_lock_irqsave(&devtree_lock, flags);
	match = __of_match_node(matches, node);
	raw_spin_unlock_irqrestore(&devtree_lock, flags);
	return match;
}


/*在compatible中，找到最优匹配的字符串，排在前面的匹配度更高，如"fsl,imx21-i2c"*/
static const struct of_device_id *__of_match_node(
                       const struct of_device_id *matches,
					   const struct device_node *node)
{
	const struct of_device_id *best_match = NULL;
	int score, best_score = 0;

	if (!matches)
		return NULL;

	for (; matches->name[0] || matches->type[0] || matches->compatible[0]; matches++) {
        /*匹配最优*/
		score = __of_device_is_compatible(node, matches->compatible,
						  matches->type, matches->name);
		if (score > best_score) {
			best_match = matches;
			best_score = score;
		}
	}

	return best_match;
}
```

### 1-2 devm_clk_get()

```C
struct clk *devm_clk_get(struct device *dev, const char *id)
{
	struct clk **ptr, *clk;

	ptr = devres_alloc(devm_clk_release, sizeof(*ptr), GFP_KERNEL);
	if (!ptr)
		return ERR_PTR(-ENOMEM);

	clk = clk_get(dev, id);
	if (!IS_ERR(clk)) {
		*ptr = clk;
		devres_add(dev, ptr);
	} else {
		devres_free(ptr);
	}

	return clk;
}

/*获取clk结构相关内容*/
struct clk *clk_get(struct device *dev, const char *con_id)
{
	const char *dev_id = dev ? dev_name(dev) : NULL;
	struct clk *clk;

	if (dev) {
		clk = __of_clk_get_by_name(dev->of_node, dev_id, con_id);
		if (!IS_ERR(clk) || PTR_ERR(clk) == -EPROBE_DEFER)
			return clk;
	}

	return clk_get_sys(dev_id, con_id);
}
```

### struct clk

```C
struct clk {
    /*指的是芯片的总时钟*/
	struct clk_core	*core;
	const char *dev_id;
	const char *con_id;
	unsigned long min_rate;
	unsigned long max_rate;
	struct hlist_node clks_node;
};
```

### struct of_clk_provider

```C
/**
 * struct of_clk_provider - Clock provider registration structure
 * @link: Entry in global list of clock providers
 * @node: Pointer to device tree node of clock provider
 * @get: Get clock callback.  Returns NULL or a struct clk for the
 *       given clock specifier
 * @data: context pointer to be passed into @get callback
 */
struct of_clk_provider {
	struct list_head link;

	struct device_node *node;
	struct clk *(*get)(struct of_phandle_args *clkspec, void *data);
	struct clk_hw *(*get_hw)(struct of_phandle_args *clkspec, void *data);
	void *data;
};
```

### 1-2-1 __of_clk_get_by_name()

```C
static struct clk *__of_clk_get_by_name(struct device_node *np,
					const char *dev_id,
					const char *name)
{
	struct clk *clk = ERR_PTR(-ENOENT);

	/* Walk up the tree of devices looking for a clock that matches */
	while (np) {
		int index = 0;

		/*
		 * For named clocks, first look up the name in the
		 * "clock-names" property.  If it cannot be found, then
		 * index will be an error code, and of_clk_get() will fail.
		 */
		if (name)
			index = of_property_match_string(np, "clock-names", name);
        /*
         * 根据属性"clocks",找到父clks，并提取clk内容
         */
		clk = __of_clk_get(np, index, dev_id, name);
		if (!IS_ERR(clk)) {
			break;
		} else if (name && index >= 0) {
			if (PTR_ERR(clk) != -EPROBE_DEFER)
				pr_err("ERROR: could not get clock %s:%s(%i)\n",
					np->full_name, name ? name : "", index);
			return clk;
		}

		/*
		 * No matching clock found on this node.  If the parent node
		 * has a "clock-ranges" property, then we can try one of its
		 * clocks.
		 */
		np = np->parent;
		if (np && !of_get_property(np, "clock-ranges", NULL))
			break;
	}

	return clk;
}
```

### 1-2-1-1 __of_clk_get()

```C
static struct clk *__of_clk_get(struct device_node *np, int index,
			       const char *dev_id, const char *con_id)
{
	struct of_phandle_args clkspec;
	struct clk *clk;
	int rc;

	if (index < 0)
		return ERR_PTR(-EINVAL);

    /*
     * 1.找到np节点下，名为"clocks"的属性。
     * 并根据index，偏移(#clock-cells * index)个内容。
     * 记录在 of_phandle_args 结构中。
     * 如i2c节点下： clocks = <&clks IMX6UL_CLK_I2C1>;
     */
	rc = of_parse_phandle_with_args(np, "clocks", "#clock-cells", index,
					&clkspec);
	if (rc)
		return ERR_PTR(rc);

	clk = __of_clk_get_from_provider(&clkspec, dev_id, con_id);
	of_node_put(clkspec.np);

	return clk;
}
```

### 1-2-1-1-1 __of_clk_get_from_provider()

```C
struct clk *__of_clk_get_from_provider(struct of_phandle_args *clkspec,
				       const char *dev_id, const char *con_id)
{
	struct of_clk_provider *provider;
	struct clk *clk = ERR_PTR(-EPROBE_DEFER);
	struct clk_hw *hw;

	if (!clkspec)
		return ERR_PTR(-EINVAL);

	/* Check if we have such a provider in our array */
	mutex_lock(&of_clk_mutex);
    
    /*
     * 遍历整个of_clk_providers list,找到匹配的node
     * 由此可见，肯定存在一个platform_driver用于匹配clks
     */
	list_for_each_entry(provider, &of_clk_providers, link) {
        
        /*匹配成功，为同个node节点*/
		if (provider->node == clkspec->np) {
			hw = __of_clk_get_hw_from_provider(provider, clkspec);
			clk = __clk_create_clk(hw, dev_id, con_id);
		}

		if (!IS_ERR(clk)) {
			if (!__clk_get(clk)) {
				__clk_free_clk(clk);
				clk = ERR_PTR(-ENOENT);
			}

			break;
		}
	}
	mutex_unlock(&of_clk_mutex);

	return clk;
}
```

### struct i2c_bus_recovery_info

```C
/**
 * struct i2c_bus_recovery_info - I2C bus recovery information
 * @recover_bus: Recover routine. Either pass driver's recover_bus() routine, or
 *	i2c_generic_scl_recovery() or i2c_generic_gpio_recovery().
 * @get_scl: This gets current value of SCL line. Mandatory for generic SCL
 *      recovery. Used internally for generic GPIO recovery.
 * @set_scl: This sets/clears SCL line. Mandatory for generic SCL recovery. Used
 *      internally for generic GPIO recovery.
 * @get_sda: This gets current value of SDA line. Optional for generic SCL
 *      recovery. Used internally, if sda_gpio is a valid GPIO, for generic GPIO
 *      recovery.
 * @prepare_recovery: This will be called before starting recovery. Platform may
 *	configure padmux here for SDA/SCL line or something else they want.
 * @unprepare_recovery: This will be called after completing recovery. Platform
 *	may configure padmux here for SDA/SCL line or something else they want.
 * @scl_gpio: gpio number of the SCL line. Only required for GPIO recovery.
 * @sda_gpio: gpio number of the SDA line. Only required for GPIO recovery.
 */
struct i2c_bus_recovery_info {
	int (*recover_bus)(struct i2c_adapter *);

	int (*get_scl)(struct i2c_adapter *);
	void (*set_scl)(struct i2c_adapter *, int val);
	int (*get_sda)(struct i2c_adapter *);

	void (*prepare_recovery)(struct i2c_adapter *);
	void (*unprepare_recovery)(struct i2c_adapter *);

	/* gpio recovery */
	int scl_gpio;
	int sda_gpio;
};
```

### 1-2 i2c_imx_init_recovery_info()

```C
/*
 * 大概意思就是说：当i2c功能转换回普通的gpio功能，可自动进行恢复，前提是已经在设备树进行描述
 * 如 pinctrl-name="default" "gpio";
 *    pinctrl-0=<&node1 &node2>;
 */

/*
 * We switch SCL and SDA to their GPIO function and do some bitbanging
 * for bus recovery. These alternative pinmux settings can be
 * described in the device tree by a separate pinctrl state "gpio". If
 * this is missing this is not a big problem, the only implication is
 * that we can't do bus recovery.
 */
static int i2c_imx_init_recovery_info(struct imx_i2c_struct *i2c_imx,
		struct platform_device *pdev)
{
	struct i2c_bus_recovery_info *rinfo = &i2c_imx->rinfo;

	i2c_imx->pinctrl = devm_pinctrl_get(&pdev->dev);
	if (!i2c_imx->pinctrl || IS_ERR(i2c_imx->pinctrl)) {
		dev_info(&pdev->dev, "can't get pinctrl, bus recovery not supported\n");
		return PTR_ERR(i2c_imx->pinctrl);
	}

	i2c_imx->pinctrl_pins_default = pinctrl_lookup_state(i2c_imx->pinctrl,
			PINCTRL_STATE_DEFAULT);
	i2c_imx->pinctrl_pins_gpio = pinctrl_lookup_state(i2c_imx->pinctrl,
			"gpio");
	rinfo->sda_gpio = of_get_named_gpio(pdev->dev.of_node, "sda-gpios", 0);
	rinfo->scl_gpio = of_get_named_gpio(pdev->dev.of_node, "scl-gpios", 0);

	if (rinfo->sda_gpio == -EPROBE_DEFER ||
	    rinfo->scl_gpio == -EPROBE_DEFER) {
		return -EPROBE_DEFER;
	} else if (!gpio_is_valid(rinfo->sda_gpio) ||
		   !gpio_is_valid(rinfo->scl_gpio) ||
		   IS_ERR(i2c_imx->pinctrl_pins_default) ||
		   IS_ERR(i2c_imx->pinctrl_pins_gpio)) {
		dev_dbg(&pdev->dev, "recovery information incomplete\n");
		return 0;
	}

	dev_dbg(&pdev->dev, "using scl-gpio %d and sda-gpio %d for recovery\n",
			rinfo->sda_gpio, rinfo->scl_gpio);

	rinfo->prepare_recovery = i2c_imx_prepare_recovery;
	rinfo->unprepare_recovery = i2c_imx_unprepare_recovery;
	rinfo->recover_bus = i2c_generic_gpio_recovery;
	i2c_imx->adapter.bus_recovery_info = rinfo;

	return 0;
}
```

### 1-3 i2c_add_numbered_adapter

```C
/**
 * i2c_add_numbered_adapter - declare i2c adapter, use static bus number
 * @adap: the adapter to register (with adap->nr initialized)
 * Context: can sleep
 *
 * This routine is used to declare an I2C adapter when its bus number
 * matters.  For example, use it for I2C adapters from system-on-chip CPUs,
 * or otherwise built in to the system's mainboard, and where i2c_board_info
 * is used to properly configure I2C devices.
 *
 * If the requested bus number is set to -1, then this function will behave
 * identically to i2c_add_adapter, and will dynamically assign a bus number.
 * 根据adapter->nr的序号，注册adapter，
 * 如果想让系统帮动态分配一个number,将nr设置为-1即可。
 *
 * If no devices have pre-been declared for this bus, then be sure to
 * register the adapter before any dynamically allocated ones.  Otherwise
 * the required bus ID may not be available.
 * 
 * When this returns zero, the specified adapter became available for
 * clients using the bus number provided in adap->nr.  Also, the table
 * of I2C devices pre-declared using i2c_register_board_info() is scanned,
 * and the appropriate driver model device nodes are created.  Otherwise, a
 * negative errno value is returned.
 */
int i2c_add_numbered_adapter(struct i2c_adapter *adap)
{
    /* 动态分配 */
	if (adap->nr == -1) /* -1 means dynamically assign bus id */
		return i2c_add_adapter(adap);

	return __i2c_add_numbered_adapter(adap);
}
```

### 1-3-1 i2c_add_adapter()

```C
/**
 * i2c_add_adapter - declare i2c adapter, use dynamic bus number
 * @adapter: the adapter to add
 * Context: can sleep
 *
 * This routine is used to declare an I2C adapter when its bus number
 * doesn't matter or when its bus number is specified by an dt alias.
 * Examples of bases when the bus number doesn't matter: I2C adapters
 * dynamically added by USB links or PCI plugin cards.
 *
 * When this returns zero, a new bus number was allocated and stored
 * in adap->nr, and the specified adapter became available for clients.
 * Otherwise, a negative errno value is returned.
 */
int i2c_add_adapter(struct i2c_adapter *adapter)
{
	struct device *dev = &adapter->dev;
	int id;

	if (dev->of_node) {
		id = of_alias_get_id(dev->of_node, "i2c");
		if (id >= 0) {
			adapter->nr = id;
			return __i2c_add_numbered_adapter(adapter);
		}
	}

	mutex_lock(&core_lock);
    /*根据idr号来作唯一id*/
	id = idr_alloc(&i2c_adapter_idr, adapter,
		       __i2c_first_dynamic_bus_num, 0, GFP_KERNEL);
	mutex_unlock(&core_lock);
	if (WARN(id < 0, "couldn't get idr"))
		return id;

	adapter->nr = id;

	return i2c_register_adapter(adapter);
}
```

### 1-3-2 __i2c_add_numbered_adapter()

```C
/**
 * __i2c_add_numbered_adapter - i2c_add_numbered_adapter where nr is never -1
 * @adap: the adapter to register (with adap->nr initialized)
 * Context: can sleep
 *
 * See i2c_add_numbered_adapter() for details.
 */
static int __i2c_add_numbered_adapter(struct i2c_adapter *adap)
{
	int id;

	mutex_lock(&core_lock);
	/*申请一个idr用于快速查找*/
	id = idr_alloc(&i2c_adapter_idr, adap, adap->nr, adap->nr + 1, GFP_KERNEL);
	mutex_unlock(&core_lock);
	if (WARN(id < 0, "couldn't get idr"))
		return id == -ENOSPC ? -EBUSY : id;

	return  (adap);
}
```

### 1-3-1-1 i2c_register_adapter()

```C
static int i2c_register_adapter(struct i2c_adapter *adap)
{
	int res = -EINVAL;

	/* Can't register until after driver model init */
	if (WARN_ON(!is_registered)) {
		res = -EAGAIN;
		goto out_list;
	}

	/* Sanity checks */
	if (WARN(!adap->name[0], "i2c adapter has no name"))
		goto out_list;

    /*adapter一定要提供算法*/
	if (!adap->algo) {
		pr_err("adapter '%s': no algo supplied!\n", adap->name);
		goto out_list;
	}

	if (!adap->lock_ops)
		adap->lock_ops = &i2c_adapter_lock_ops;

	rt_mutex_init(&adap->bus_lock);
	rt_mutex_init(&adap->mux_lock);
	mutex_init(&adap->userspace_clients_lock);
	INIT_LIST_HEAD(&adap->userspace_clients);

	/* Set default timeout to 1 second if not already set */
	if (adap->timeout == 0)
		adap->timeout = HZ;

    /*该dev为adapter，命名为"i2c-%d"*/
	dev_set_name(&adap->dev, "i2c-%d", adap->nr);
    
    /*挂载到i2c_bus总线的device下*/
	adap->dev.bus = &i2c_bus_type;
    
    /*类型是adapter*/
	adap->dev.type = &i2c_adapter_type;
    
    /*注册和初始化device*/
	res = device_register(&adap->dev);
	if (res) {
		pr_err("adapter '%s': can't register device (%d)\n", adap->name, res);
		goto out_list;
	}

	dev_dbg(&adap->dev, "adapter [%s] registered\n", adap->name);

	pm_runtime_no_callbacks(&adap->dev);
	pm_suspend_ignore_children(&adap->dev, true);
	pm_runtime_enable(&adap->dev);

#ifdef CONFIG_I2C_COMPAT
	res = class_compat_create_link(i2c_adapter_compat_class, &adap->dev,
				       adap->dev.parent);
	if (res)
		dev_warn(&adap->dev,
			 "Failed to create compatibility class link\n");
#endif

	i2c_init_recovery(adap);

	/* create pre-declared device nodes */
    /*在dts，根据adapter节点信息，创建子节点client*/
	of_i2c_register_devices(adap);
	i2c_acpi_register_devices(adap);
	i2c_acpi_install_space_handler(adap);

	if (adap->nr < __i2c_first_dynamic_bus_num)
		i2c_scan_static_board_info(adap);

	/* Notify drivers */
	mutex_lock(&core_lock);
    /*
     * 为i2c_bus_type的device，执行 __process_new_adapter 
     * 指针空检查，非空执行i2c_driver->attach_adapter
     */
	bus_for_each_drv(&i2c_bus_type, NULL, adap, __process_new_adapter);
	mutex_unlock(&core_lock);

	return 0;

out_list:
	mutex_lock(&core_lock);
	idr_remove(&i2c_adapter_idr, adap->nr);
	mutex_unlock(&core_lock);
	return res;
}
```

### 1-3-1-1-1 of_i2c_register_devices()

```C
/*根据dts和adapter信息，创建和初始化所有client*/
static void of_i2c_register_devices(struct i2c_adapter *adap)
{
	struct device_node *bus, *node;
	struct i2c_client *client;

	/* Only register child devices if the adapter has a node pointer set */
	if (!adap->dev.of_node)
		return;

	dev_dbg(&adap->dev, "of_i2c: walking child nodes\n");

    /*尝试在节点获取"i2c-bus"*/
	bus = of_get_child_by_name(adap->dev.of_node, "i2c-bus");
    
    /*这里不懂*/
	if (!bus)
		bus = of_node_get(adap->dev.of_node);

    /*获取i2c节点下的子节点信息，并为所有子节点创建client*/
	for_each_available_child_of_node(bus, node) {
        /*检查标志位，表明该client已经被创建*/
		if (of_node_test_and_set_flag(node, OF_POPULATED))
			continue;

		client = of_i2c_register_device(adap, node);
		if (IS_ERR(client)) {
			dev_warn(&adap->dev,
				 "Failed to create I2C device for %s\n",
				 node->full_name);
			of_node_clear_flag(node, OF_POPULATED);
		}
	}

	of_node_put(bus);
}
```

### 1-3-1-1-1-1 of_i2c_register_device()

```C
/*根据dts和adapter信息，创建和初始化所有client*/
static struct i2c_client *of_i2c_register_device(struct i2c_adapter *adap,
						 struct device_node *node)
{
	struct i2c_client *result;
	struct i2c_board_info info = {};
	struct dev_archdata dev_ad = {};
	const __be32 *addr_be;
	u32 addr;
	int len;

	dev_dbg(&adap->dev, "of_i2c: register %s\n", node->full_name);

	if (of_modalias_node(node, info.type, sizeof(info.type)) < 0) {
		dev_err(&adap->dev, "of_i2c: modalias failure on %s\n",
			node->full_name);
		return ERR_PTR(-EINVAL);
	}

    /* 在dts获得设备地址 如 reg = <0x1a>; */
	addr_be = of_get_property(node, "reg", &len);
	if (!addr_be || (len < sizeof(*addr_be))) {
		dev_err(&adap->dev, "of_i2c: invalid reg on %s\n",
			node->full_name);
		return ERR_PTR(-EINVAL);
	}

    /*大小端转换*/
	addr = be32_to_cpup(addr_be);
    
    /*是否为10bit地址*/
	if (addr & I2C_TEN_BIT_ADDRESS) {
		addr &= ~I2C_TEN_BIT_ADDRESS;
		info.flags |= I2C_CLIENT_TEN;
	}

    /*是否为从机设备*/
	if (addr & I2C_OWN_SLAVE_ADDRESS) {
		addr &= ~I2C_OWN_SLAVE_ADDRESS;
		info.flags |= I2C_CLIENT_SLAVE;
	}

	if (i2c_check_addr_validity(addr, info.flags)) {
		dev_err(&adap->dev, "of_i2c: invalid addr=%x on %s\n",
			addr, node->full_name);
		return ERR_PTR(-EINVAL);
	}

    /*保存设备地址信息*/
	info.addr = addr;
    /*保存node*/
	info.of_node = of_node_get(node);
	info.archdata = &dev_ad;

    /*尝试获取属性"wakeup-source"*/
	if (of_get_property(node, "wakeup-source", NULL))
		info.flags |= I2C_CLIENT_WAKE;
	
    /*根据上面获取的dts信息和adapter，创建和初始化所有client*/
	result = i2c_new_device(adap, &info);
	if (result == NULL) {
		dev_err(&adap->dev, "of_i2c: Failure registering %s\n",
			node->full_name);
		of_node_put(node);
		return ERR_PTR(-EINVAL);
	}
	return result;
}
```

### 1-3-1-1-1-1-1 i2c_new_device()

```C
/*根据adapter和i2c_board_info,创建和初始化一个client(包含初始化其client->dev)*/

/**
 * i2c_new_device - instantiate an i2c device
 * @adap: the adapter managing the device
 * @info: describes one I2C device; bus_num is ignored
 * Context: can sleep
 *
 * Create an i2c device. Binding is handled through driver model
 * probe()/remove() methods.  A driver may be bound to this device when we
 * return from this function, or any later moment (e.g. maybe hotplugging will
 * load the driver module).  This call is not appropriate for use by mainboard
 * initialization logic, which usually runs during an arch_initcall() long
 * before any i2c_adapter could exist.
 *
 * This returns the new i2c client, which may be saved for later use with
 * i2c_unregister_device(); or NULL to indicate an error.
 */
struct i2c_client *
i2c_new_device(struct i2c_adapter *adap, struct i2c_board_info const *info)
{
	struct i2c_client	*client;
	int			status;

    /*申请一个 i2c_client 内存*/
	client = kzalloc(sizeof *client, GFP_KERNEL);
	if (!client)
		return NULL;

    /*client所属的adapter*/
	client->adapter = adap;

	client->dev.platform_data = info->platform_data;

	if (info->archdata)
		client->dev.archdata = *info->archdata;

	client->flags = info->flags;
	client->addr = info->addr;
	client->irq = info->irq;

	strlcpy(client->name, info->type, sizeof(client->name));

	status = i2c_check_addr_validity(client->addr, client->flags);
	if (status) {
		dev_err(&adap->dev, "Invalid %d-bit I2C address 0x%02hx\n",
			client->flags & I2C_CLIENT_TEN ? 10 : 7, client->addr);
		goto out_err_silent;
	}

	/* Check for address business */
	status = i2c_check_addr_busy(adap, i2c_encode_flags_to_addr(client));
	if (status)
		goto out_err;

	client->dev.parent = &client->adapter->dev;
	client->dev.bus = &i2c_bus_type;
	client->dev.type = &i2c_client_type;
	client->dev.of_node = info->of_node;
    
    /*应该是用于生成在firmware的节点*/
	client->dev.fwnode = info->fwnode;

    /*client命名为"%d-%04x" "总线-设备地址"*/
	i2c_dev_set_name(adap, client);
	status = device_register(&client->dev);
	if (status)
		goto out_err;

	dev_dbg(&adap->dev, "client [%s] registered with bus id %s\n",
		client->name, dev_name(&client->dev));

	return client;

out_err:
	dev_err(&adap->dev,
		"Failed to register i2c client %s at 0x%02x (%d)\n",
		client->name, client->addr, status);
out_err_silent:
	kfree(client);
	return NULL;
}
```

### 1-3-1-1-2 __process_new_adapter()

```C
static int __process_new_adapter(struct device_driver *d, void *data)
{
	return i2c_do_add_adapter(to_i2c_driver(d), data);
}


static int i2c_do_add_adapter(struct i2c_driver *driver,
			      struct i2c_adapter *adap)
{
	/* Detect supported devices on that bus, and instantiate them */
    /* 添加了adapter后，会对i2c-bus上的device进行检测，是否能够match */
	i2c_detect(adap, driver);

	/* Let legacy drivers scan this bus for matching devices */
	if (driver->attach_adapter) {
		dev_warn(&adap->dev, "%s: attach_adapter method is deprecated\n",
			 driver->driver.name);
		dev_warn(&adap->dev,
			 "Please use another way to instantiate your i2c_client\n");
		/* We ignore the return code; if it fails, too bad */
		driver->attach_adapter(adap);
	}
	return 0;
}
```

### 1 总结

1. i2c-imx.c文件中的platform_driver，注册在platform_bus下，该platform_driver的probe实际作用就是用来解析该imx类型的soc，根据名字选择具体系列的soc，读取私有数据，读取dts，构造一个adapter，并将其挂载到i2c_bus_type总线下，如：命名为"i2c-0" "i2c-1"。

2. 然后在adapter的of_node下，解析其子of_node（即client），读取dts，构造n个client，也挂载到i2c_bus_type总线下，如：命名为“0-0039”(0代表着该设备在i2c哪条总线下，0039代表着设备地址。4位的长度用途是，可能是7bit设备地址，也可能是10bit设备地址)

   

### 2 i2c_imx_xfer()

```C
/*硬件发送函数*/
static int i2c_imx_xfer(struct i2c_adapter *adapter,
						struct i2c_msg *msgs, int num)
{
	unsigned int i, temp;
	int result;
	bool is_lastmsg = false;
	bool enable_runtime_pm = false;
	struct imx_i2c_struct *i2c_imx = i2c_get_adapdata(adapter);

	dev_dbg(&i2c_imx->adapter.dev, "<%s>\n", __func__);


	if (!pm_runtime_enabled(i2c_imx->adapter.dev.parent)) {
		pm_runtime_enable(i2c_imx->adapter.dev.parent);
		enable_runtime_pm = true;
	}

	result = pm_runtime_get_sync(i2c_imx->adapter.dev.parent);
	if (result < 0)
		goto out;

	/* Start I2C transfer */
	result = i2c_imx_start(i2c_imx);
	if (result) {
		if (i2c_imx->adapter.bus_recovery_info) {
			i2c_recover_bus(&i2c_imx->adapter);
			result = i2c_imx_start(i2c_imx);
		}
	}

	if (result)
		goto fail0;

	/* read/write data */
	for (i = 0; i < num; i++) {
		if (i == num - 1)
			is_lastmsg = true;

		if (i) {
			dev_dbg(&i2c_imx->adapter.dev,
				"<%s> repeated start\n", __func__);
			temp = imx_i2c_read_reg(i2c_imx, IMX_I2C_I2CR);
			temp |= I2CR_RSTA;
			imx_i2c_write_reg(temp, i2c_imx, IMX_I2C_I2CR);
			result = i2c_imx_bus_busy(i2c_imx, 1);
			if (result)
				goto fail0;
		}
		dev_dbg(&i2c_imx->adapter.dev,
			"<%s> transfer message: %d\n", __func__, i);
		/* write/read data */
#ifdef CONFIG_I2C_DEBUG_BUS
		temp = imx_i2c_read_reg(i2c_imx, IMX_I2C_I2CR);
		dev_dbg(&i2c_imx->adapter.dev,
			"<%s> CONTROL: IEN=%d, IIEN=%d, MSTA=%d, MTX=%d, TXAK=%d, RSTA=%d\n",
			__func__,
			(temp & I2CR_IEN ? 1 : 0), (temp & I2CR_IIEN ? 1 : 0),
			(temp & I2CR_MSTA ? 1 : 0), (temp & I2CR_MTX ? 1 : 0),
			(temp & I2CR_TXAK ? 1 : 0), (temp & I2CR_RSTA ? 1 : 0));
		temp = imx_i2c_read_reg(i2c_imx, IMX_I2C_I2SR);
		dev_dbg(&i2c_imx->adapter.dev,
			"<%s> STATUS: ICF=%d, IAAS=%d, IBB=%d, IAL=%d, SRW=%d, IIF=%d, RXAK=%d\n",
			__func__,
			(temp & I2SR_ICF ? 1 : 0), (temp & I2SR_IAAS ? 1 : 0),
			(temp & I2SR_IBB ? 1 : 0), (temp & I2SR_IAL ? 1 : 0),
			(temp & I2SR_SRW ? 1 : 0), (temp & I2SR_IIF ? 1 : 0),
			(temp & I2SR_RXAK ? 1 : 0));
#endif
		if (msgs[i].flags & I2C_M_RD)
			result = i2c_imx_read(i2c_imx, &msgs[i], is_lastmsg);
		else {
			if (i2c_imx->dma && msgs[i].len >= DMA_THRESHOLD)
				result = i2c_imx_dma_write(i2c_imx, &msgs[i]);
			else
				result = i2c_imx_write(i2c_imx, &msgs[i]);
		}
		if (result)
			goto fail0;
	}

fail0:
	/* Stop I2C transfer */
	i2c_imx_stop(i2c_imx);

	pm_runtime_mark_last_busy(i2c_imx->adapter.dev.parent);
	pm_runtime_put_autosuspend(i2c_imx->adapter.dev.parent);

out:
	if (enable_runtime_pm)
		pm_runtime_disable(i2c_imx->adapter.dev.parent);

	dev_dbg(&i2c_imx->adapter.dev, "<%s> exit with: %s: %d\n", __func__,
		(result < 0) ? "error" : "success msg",
			(result < 0) ? result : num);
	return (result < 0) ? result : num;
}

```

### 2 i2c_imx_start()

```C

static int i2c_imx_start(struct imx_i2c_struct *i2c_imx)
{
	unsigned int temp = 0;
	int result;

	dev_dbg(&i2c_imx->adapter.dev, "<%s>\n", __func__);
	
    /*
     * 用意：一个adapter可以匹配多种速度的client，
     * 因此需要在发送之前配置频率分频寄存器。
     */
	i2c_imx_set_clk(i2c_imx);

    /*设置 I2C Frequency Divider Register*/
	imx_i2c_write_reg(i2c_imx->ifdr, i2c_imx, IMX_I2C_IFDR);
    
	/* Enable I2C controller */
    /*清空 I2C Status Register*/
	imx_i2c_write_reg(i2c_imx->hwdata->i2sr_clr_opcode, i2c_imx, IMX_I2C_I2SR);
    
    /*清空 I2C Control Register*/
	imx_i2c_write_reg(i2c_imx->hwdata->i2cr_ien_opcode, i2c_imx, IMX_I2C_I2CR);

	/* Wait controller to be stable */
    /*大概的休眠时间50-150us，哈哈这个好搞笑*/
	usleep_range(50, 150);

	/* Start I2C transaction */
    /*读改写，让芯片产生一个start信号*/
	temp = imx_i2c_read_reg(i2c_imx, IMX_I2C_I2CR);
	temp |= I2CR_MSTA;
	imx_i2c_write_reg(temp, i2c_imx, IMX_I2C_I2CR);
    
    /*检查仲裁是否成功*/
	result = i2c_imx_bus_busy(i2c_imx, 1);
	if (result)
		return result;
	i2c_imx->stopped = 0;

	temp |= I2CR_IIEN | I2CR_MTX | I2CR_TXAK;
	temp &= ~I2CR_DMAEN;
	imx_i2c_write_reg(temp, i2c_imx, IMX_I2C_I2CR);
	return result;
}

```

