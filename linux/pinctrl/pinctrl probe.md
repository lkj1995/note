```shell
# dec -I dtb  -O dts imx6ull-mmc-npi.dtb > 11.dts /*反编译设备树*/
# vim 11.dts 
# /iomux
#  761             iomuxc@20e0000 {
#  762                 compatible = "fsl,imx6ul-iomuxc";
```

可知存在属性为 fsl,imx6ul-iomuxc，可与drv匹配成功。

### imx6ul_pinctrl_driver

```c
/*厂商预先写好，该芯片的pin*/
static const struct pinctrl_pin_desc imx6ul_pinctrl_pads[] = {  
	IMX_PINCTRL_PIN(MX6UL_PAD_RESERVE0),
	IMX_PINCTRL_PIN(MX6UL_PAD_RESERVE1),
	IMX_PINCTRL_PIN(MX6UL_PAD_RESERVE2),
	IMX_PINCTRL_PIN(MX6UL_PAD_RESERVE3),
	IMX_PINCTRL_PIN(MX6UL_PAD_RESERVE4),
	IMX_PINCTRL_PIN(MX6UL_PAD_RESERVE5),
	IMX_PINCTRL_PIN(MX6UL_PAD_RESERVE6),
	IMX_PINCTRL_PIN(MX6UL_PAD_RESERVE7),
    ...
};

static const struct imx_pinctrl_soc_info imx6ul_pinctrl_info = {
	.pins = imx6ul_pinctrl_pads,/*引脚描述*/
	.npins = ARRAY_SIZE(imx6ul_pinctrl_pads), /*pin数量*/
	.gpr_compatible = "fsl,imx6ul-iomuxc-gpr",
};

static const struct of_device_id imx6ul_pinctrl_of_match[] = {
    /*dt中存在"fsl,imx6ul-iomuxc"，能够和drv进行match,再执行probe函数*/
	{ .compatible = "fsl,imx6ul-iomuxc", .data = &imx6ul_pinctrl_info, },
	{ .compatible = "fsl,imx6ull-iomuxc-snvs", .data = &imx6ull_snvs_pinctrl_info, },
	{ /* sentinel */ }
};

static struct platform_driver imx6ul_pinctrl_driver = {
	.driver = {
		.name = "imx6ul-pinctrl",
		.of_match_table = of_match_ptr(imx6ul_pinctrl_of_match),
	},
	.probe = imx6ul_pinctrl_probe,/*dev和drv进行match后，执行该probe*/
};
```

### imx6ul_pinctrl_probe

```C
static int imx6ul_pinctrl_probe(struct platform_device *pdev)
{
	const struct imx_pinctrl_soc_info *pinctrl_info;
	/*找到最匹配的compatible，获取其私有数据，即获取 imx_pinctrl_soc_info */
	pinctrl_info = of_device_get_match_data(&pdev->dev);
	if (!pinctrl_info)
		return -ENODEV;

	return imx_pinctrl_probe(pdev, pinctrl_info);
}
```

### of_device_get_match_data

```C
const void *of_device_get_match_data(const struct device *dev)
{
	const struct of_device_id *match;
    /**/
	match = of_match_device(dev->driver->of_match_table, dev);
	if (!match)
		return NULL;

	return match->data;
}
```

### of_match_device

```C
/*
 * Struct used for matching a device
 */
struct of_device_id {
        char    name[32];
        char    type[32];
        char    compatible[128];
        const void *data;
};

/**
 * of_match_device - Tell if a struct device matches an of_device_id list
 * @matches: array of of device match structures to search in
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
```

```C
struct pinctrl_desc {
	const char *name;
    
    /*引脚的枚举与命名，pins指向一个数组，npins描述该数组的大小*/
	const struct pinctrl_pin_desc *pins;
	unsigned int npins;
    
    /*用于获取pin group的相关内容，如group的数量，某个group的pin描述，数量等*/
	const struct pinctrl_ops *pctlops;
    
    /*选择并使能对应的funciton的复用，如pinctrl-0的复用iic*/
	const struct pinmux_ops *pmxops;
    
    /*配置某个pin或某group pin，如上下拉*/
	const struct pinconf_ops *confops;
    
	...
};
```

### imx_pinctrl_soc_info

```C
struct imx_pinctrl_soc_info {
    /*
     * 保存在 /sys/bus/platform/devices/20e0000.iomuxc 
     * 即生成了 platform_device->dev
     */
	struct device *dev;
    
    /*初始默认已写好，所有pin配置值，及数量*/
	const struct pinctrl_pin_desc *pins;
	unsigned int npins;
    
	struct imx_pin_reg *pin_regs;
    
    /*group数组，描述该节点下，每个group的名字，pin数量，及每个pin的具体配置值*/
	struct imx_pin_group *groups;
    /*group的数量*/
	unsigned int ngroups; 
    
	unsigned int group_index;
    
    /*记录了iomuxc下有多少个group及每个group的名字，对应 imx_pin_group*/
	struct imx_pmx_func *functions; 
    
    /*等于1*/
	unsigned int nfunctions;
    
	unsigned int flags;
	const char *gpr_compatible;
};
```

### imx_pin_reg

```C
/**
 * struct imx_pin_reg - describe a pin reg map
 * @mux_reg: mux register offset
 * @conf_reg: config register offset
 */
struct imx_pin_reg {
	s16 mux_reg;  /*mux寄存器偏移值*/
	s16 conf_reg; /*config寄存器偏移值*/
};
```

### pinctrl_pin_desc

```C
/**
 * struct pinctrl_pin_desc - boards/machines provide information on their
 * pins, pads or other muxable units in this struct
 * @number: unique pin number from the global pin number space
 * @name: a name for this pin
 * @drv_data: driver-defined per-pin data. pinctrl core does not touch this
 */
struct pinctrl_pin_desc {
	unsigned number;
	const char *name;
	void *drv_data;
};
```

### imx_pinctrl

```C
/**
 * @dev: a pointer back to containing device
 * @base: the offset to the controller in virtual memory
 */
struct imx_pinctrl {
	struct device *dev;
	struct pinctrl_dev *pctl;
	void __iomem *base; /*保存resource的虚拟地址*/
	void __iomem *input_sel_base;
	const struct imx_pinctrl_soc_info *info;
};
```

### imx_pmx_func

```C
/**
 * struct imx_pmx_func - describes IMX pinmux functions
 * @name: the name of this specific function
 * @groups: corresponding pin groups
 * @num_groups: the number of groups
 */
struct imx_pmx_func {
	const char *name;
	const char **groups;
	unsigned num_groups;
};
```

### imx_pin_group

```C
/**
 * struct imx_pin_group - describes a single i.MX pin
 * @pin: the pin_id of this pin
 * @mux_mode: the mux mode for this pin.
 * @input_reg: the select input register offset for this pin if any
 *	0 if no select input setting needed.
 * @input_val: the select input value for this pin.
 * @configs: the config for this pin.
 */
struct imx_pin {
	unsigned int pin;
	unsigned int mux_mode;
	u16 input_reg;
	unsigned int input_val;
	unsigned long config;
};

/**
 * struct imx_pin_group - describes an IMX pin group
 * @name: the name of this specific pin group
 * @npins: the number of pins in this group array, i.e. the number of
 *	elements in .pins so we can iterate over that array
 * @pin_ids: array of pin_ids. pinctrl forces us to maintain such an array
 * @pins: array of pins
 */
struct imx_pin_group {
	const char *name;/*该组的名字*/
	unsigned npins;/*该组的pin数量*/
	unsigned int *pin_ids;
	struct imx_pin *pins;/*每个pin的具体配置值，offset，mux，config*/
};

```

### pinctrl_dev

```C
/**
 * struct pinctrl_dev - pin control class device
 * @node: node to include this pin controller in the global pin controller list
 * @desc: the pin controller descriptor supplied when initializing this pin
 *	controller
 * @pin_desc_tree: each pin descriptor for this pin controller is stored in
 *	this radix tree
 * @gpio_ranges: a list of GPIO ranges that is handled by this pin controller,
 *	ranges are added to this list at runtime
 * @dev: the device entry for this pin controller
 * @owner: module providing the pin controller, used for refcounting
 * @driver_data: driver data for drivers registering to the pin controller
 *	subsystem
 * @p: result of pinctrl_get() for this device
 * @hog_default: default state for pins hogged by this device
 * @hog_sleep: sleep state for pins hogged by this device
 * @mutex: mutex taken on each pin controller specific action
 * @device_root: debugfs root for this device
 */
struct pinctrl_dev {
	struct list_head node;
	struct pinctrl_desc *desc;
	struct radix_tree_root pin_desc_tree;
	struct list_head gpio_ranges;
	struct device *dev;
	struct module *owner;
	void *driver_data;
	struct pinctrl *p;
	struct pinctrl_state *hog_default;
	struct pinctrl_state *hog_sleep;
	struct mutex mutex;
#ifdef CONFIG_DEBUG_FS
	struct dentry *device_root;
#endif
};
```

### imx_pinctrl_probe

```C
int imx_pinctrl_probe(struct platform_device *pdev,
		      struct imx_pinctrl_soc_info *info)
{
	struct regmap_config config = { .name = "gpr" };
	struct device_node *dev_np = pdev->dev.of_node;
	struct pinctrl_desc *imx_pinctrl_desc;
	struct device_node *np;
	struct imx_pinctrl *ipctl;
	struct resource *res;
	struct regmap *gpr;
	int ret, i;

	if (!info || !info->pins || !info->npins) {
		dev_err(&pdev->dev, "wrong pinctrl info\n");
		return -EINVAL;
	}
	info->dev = &pdev->dev; /*记录dev*/

	if (info->gpr_compatible) {
		gpr = syscon_regmap_lookup_by_compatible(info->gpr_compatible);
		if (!IS_ERR(gpr))
			regmap_attach_dev(&pdev->dev, gpr, &config);
	}

	/* Create state holders etc for this driver */
    /*申请 imx_pinctrl 内存*/
	ipctl = devm_kzalloc(&pdev->dev, sizeof(*ipctl), GFP_KERNEL);
	if (!ipctl)
		return -ENOMEM;

    /*申请 imx_pin_reg * npins 内存， 记录寄存器偏移值*/
	info->pin_regs = devm_kmalloc(&pdev->dev, sizeof(*info->pin_regs) *
				      info->npins, GFP_KERNEL);
	if (!info->pin_regs)
		return -ENOMEM;

	for (i = 0; i < info->npins; i++) {
		info->pin_regs[i].mux_reg = -1;
		info->pin_regs[i].conf_reg = -1;
	}

    /*返回dev的resource数组地址*/
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
    
    /*转换成虚拟地址*/
	ipctl->base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(ipctl->base))
		return PTR_ERR(ipctl->base);

    /*寻找属性 "fsl,input-sel",false跳过*/
	if (of_property_read_bool(dev_np, "fsl,input-sel")) {
		np = of_parse_phandle(dev_np, "fsl,input-sel", 0);
		if (!np) {
			dev_err(&pdev->dev, "iomuxc fsl,input-sel property not found\n");
			return -EINVAL;
		}

		ipctl->input_sel_base = of_iomap(np, 0);
		of_node_put(np);
		if (!ipctl->input_sel_base) {
			dev_err(&pdev->dev,
				"iomuxc input select base address not found\n");
			return -ENOMEM;
		}
	}

    /*创建 pinctrl_desc 内存*/
	imx_pinctrl_desc = devm_kzalloc(&pdev->dev, sizeof(*imx_pinctrl_desc),
					GFP_KERNEL);
	if (!imx_pinctrl_desc)
		return -ENOMEM;

	imx_pinctrl_desc->name = dev_name(&pdev->dev);
    
    /*imx6ul_pinctrl_info，芯片的所有引脚枚举*/
	imx_pinctrl_desc->pins = info->pins;
    /*pin的总数量*/
	imx_pinctrl_desc->npins = info->npins;
    
    /*厂商写好的ops*/
	imx_pinctrl_desc->pctlops = &imx_pctrl_ops,
    /*mux相关ops，复用*/
	imx_pinctrl_desc->pmxops = &imx_pmx_ops,
    /*config相关ops，如上下拉*/
	imx_pinctrl_desc->confops = &imx_pinconf_ops,
    
	imx_pinctrl_desc->owner = THIS_MODULE,
    
	/*当前info只记录了基本内容，展开并记录在imx_pin_group和imx_pmx_func*/
	ret = imx_pinctrl_probe_dt(pdev, info);
	if (ret) {
		dev_err(&pdev->dev, "fail to probe dt properties\n");
		return ret;
	}

	ipctl->info = info;/*记录info*/
	ipctl->dev = info->dev;/*记录dev*/
    
     /* pdev->driver_data = ipctl; */
	platform_set_drvdata(pdev, ipctl);
    
	ipctl->pctl = devm_pinctrl_register(&pdev->dev,
					    imx_pinctrl_desc, ipctl);
	if (IS_ERR(ipctl->pctl)) {
		dev_err(&pdev->dev, "could not register IMX pinctrl driver\n");
		return PTR_ERR(ipctl->pctl);
	}

	dev_info(&pdev->dev, "initialized IMX pinctrl driver\n");

	return 0;
}
```

### imx_pinctrl_probe_dt

```C
static int imx_pinctrl_probe_dt(struct platform_device *pdev,
				struct imx_pinctrl_soc_info *info)
{
	struct device_node *np = pdev->dev.of_node;
	struct device_node *child;
	u32 nfuncs = 0;
	u32 i = 0;
	bool flat_funcs;

	if (!np)
		return -ENODEV;

	flat_funcs = imx_pinctrl_dt_is_flat_functions(np);/*此时根节点为iomuxc*/
    /*
     * false：如果子节点不存在，孙节点存在"fsl,pins"
     * ture:子节点存在"fsl,pins"
     */
	if (flat_funcs) {
		nfuncs = 1;
	} else {/*会进行此分支，false*/
		nfuncs = of_get_child_count(np);/*遍历所有子节点，记录数量，等于1*/
		if (nfuncs <= 0) {
			dev_err(&pdev->dev, "no functions defined\n");
			return -EINVAL;
		}
	}

	info->nfunctions = nfuncs;/*应该理解为 board 更准确*/
    /*申请 imx_pmx_func 内存，*/
	info->functions = devm_kzalloc(&pdev->dev, nfuncs * sizeof(struct imx_pmx_func),
					GFP_KERNEL);
	if (!info->functions)
		return -ENOMEM;

	info->group_index = 0;
	if (flat_funcs) {
		info->ngroups = of_get_child_count(np);
	} else {/*进入此分支*/
		info->ngroups = 0;
		for_each_child_of_node(np, child)
			info->ngroups += of_get_child_count(child);/*子节点总数量*/
	}
    
    /*分配 imx_pin_group * N 个组的内存*/
	info->groups = devm_kzalloc(&pdev->dev, info->ngroups * sizeof(struct imx_pin_group),
					GFP_KERNEL);
	if (!info->groups)
		return -ENOMEM;

	if (flat_funcs) {
		imx_pinctrl_parse_functions(np, info, 0);
	} else {/*进入此分支*/
		for_each_child_of_node(np, child)
			imx_pinctrl_parse_functions(child, info, i++);
	}

	return 0;
}
```

### imx_pinctrl_parse_functions

```C
static int imx_pinctrl_parse_functions(struct device_node *np,
				       struct imx_pinctrl_soc_info *info,
				       u32 index)
{
	struct device_node *child;
	struct imx_pmx_func *func;
	struct imx_pin_group *grp;
	u32 i = 0;

	dev_dbg(info->dev, "parse function(%d): %s\n", index, np->name);

	func = &info->functions[index];

	/* Initialise function */
	func->name = np->name;/*名字为"iomuxc"*/
	func->num_groups = of_get_child_count(np);/*获得子节点个数*/
	if (func->num_groups == 0) {
		dev_err(info->dev, "no groups defined in %s\n", np->full_name);
		return -EINVAL;
	}
    /*分配char* 数组，保存子节点group的名字*/
	func->groups = devm_kzalloc(info->dev,
			func->num_groups * sizeof(char *), GFP_KERNEL);

    
	for_each_child_of_node(np, child) {
        /* 记录每个group的名字在imx_pmx_func->group的char*指针中 */
		func->groups[i] = child->name; 
		grp = &info->groups[info->group_index++];
		imx_pinctrl_parse_groups(child, grp, info, i++);
	}

	return 0;
}
```

### imx_pinctrl_parse_groups

```C
/*
 * Each pin represented in fsl,pins consists of 5 u32 PIN_FUNC_ID and
 * 1 u32 CONFIG, so 24 types in total for each pin.
 */
#define FSL_PIN_SIZE 24
#define SHARE_FSL_PIN_SIZE 20

static int imx_pinctrl_parse_groups(struct device_node *np,
				    struct imx_pin_group *grp,
				    struct imx_pinctrl_soc_info *info,
				    u32 index)
{
	int size, pin_size;
	const __be32 *list;
	int i;
	u32 config;

	dev_dbg(info->dev, "group(%d): %s\n", index, np->name);

    /* 
     * FSL_PIN_SIZE为24字节，即使用6个4字节来描述一个pin
     * 记录关于offset，mux，config信息
     */
	if (info->flags & SHARE_MUX_CONF_REG)
		pin_size = SHARE_FSL_PIN_SIZE;
	else
		pin_size = FSL_PIN_SIZE;
	/* Initialise group */
	grp->name = np->name;/*记录子节点的group名字*/

	/*
	 * the binding format is fsl,pins = <PIN_FUNC_ID CONFIG ...>,
	 * do sanity check and calculate pins number
	 */
    /*寻找属性"fsl,pins"的内容，并记录长度*/
	list = of_get_property(np, "fsl,pins", &size);
	if (!list) {
		dev_err(info->dev, "no fsl,pins property in node %s\n", np->full_name);
		return -EINVAL;
	}

	/* we do not check return since it's safe node passed down */
	if (!size || size % pin_size) {
		dev_err(info->dev, "Invalid fsl,pins property in node %s\n", np->full_name);
		return -EINVAL;
	}
    
    /*有多少个4字节的十六进制数*/
	grp->npins = size / pin_size;
    
    /*就要创建多少个imx_pin来记录该pin需要描述的内容*/
	grp->pins = devm_kzalloc(info->dev, grp->npins * sizeof(struct imx_pin),
				GFP_KERNEL);
    
    /*ids*/
	grp->pin_ids = devm_kzalloc(info->dev, grp->npins * sizeof(unsigned int),
				GFP_KERNEL);
    
	if (!grp->pins || ! grp->pin_ids)
		return -ENOMEM;

	for (i = 0; i < grp->npins; i++) {
		u32 mux_reg = be32_to_cpu(*list++);
		u32 conf_reg;
		unsigned int pin_id;
		struct imx_pin_reg *pin_reg;
		struct imx_pin *pin = &grp->pins[i];

		if (!(info->flags & ZERO_OFFSET_VALID) && !mux_reg)
			mux_reg = -1;

		if (info->flags & SHARE_MUX_CONF_REG) {
			conf_reg = mux_reg;
		} else {
			conf_reg = be32_to_cpu(*list++);
			if (!conf_reg)
				conf_reg = -1;
		}

        /*
         * 提取，并记录信息
         * fsl,pin = <0x88, 0x318, 0x0, 0x5, 0x0, 0x10b0>;
         */
		pin_id = (mux_reg != -1) ? mux_reg / 4 : conf_reg / 4;
		pin_reg = &info->pin_regs[pin_id];
		pin->pin = pin_id;
		grp->pin_ids[i] = pin_id;
		pin_reg->mux_reg = mux_reg;
		pin_reg->conf_reg = conf_reg;
		pin->input_reg = be32_to_cpu(*list++);
		pin->mux_mode = be32_to_cpu(*list++);
		pin->input_val = be32_to_cpu(*list++);

		/* SION bit is in mux register */
		config = be32_to_cpu(*list++);
		if (config & IMX_PAD_SION)
			pin->mux_mode |= IOMUXC_CONFIG_SION;
		pin->config = config & ~IMX_PAD_SION;

		dev_dbg(info->dev, "%s: 0x%x 0x%08lx", info->pins[pin_id].name,
				pin->mux_mode, pin->config);
	}

	return 0;
}
```

### devm_pinctrl_register

```C
/**
 * devm_pinctrl_register() - Resource managed version of pinctrl_register().
 * @dev: parent device for this pin controller
 * @pctldesc: descriptor for this pin controller
 * @driver_data: private pin controller data for this pin controller
 *
 * Returns an error pointer if pincontrol register failed. Otherwise
 * it returns valid pinctrl handle.
 *
 * The pinctrl device will be automatically released when the device is unbound.
 */
struct pinctrl_dev *devm_pinctrl_register(struct device *dev,
					  struct pinctrl_desc *pctldesc,
					  void *driver_data)
{
	struct pinctrl_dev **ptr, *pctldev;

	ptr = devres_alloc(devm_pinctrl_dev_release, sizeof(*ptr), GFP_KERNEL);
	if (!ptr)
		return ERR_PTR(-ENOMEM);

	pctldev = pinctrl_register(pctldesc, dev, driver_data);
	if (IS_ERR(pctldev)) {
		devres_free(ptr);
		return pctldev;
	}

	*ptr = pctldev;
	devres_add(dev, ptr);

	return pctldev;
}
```

### pinctrl_register

```C
/**
 * pinctrl_register() - register a pin controller device
 * @pctldesc: descriptor for this pin controller
 * @dev: parent device for this pin controller
 * @driver_data: private pin controller data for this pin controller
 */
struct pinctrl_dev *pinctrl_register(struct pinctrl_desc *pctldesc,
				    struct device *dev, void *driver_data)
{
	struct pinctrl_dev *pctldev;
	int ret;

	if (!pctldesc)
		return ERR_PTR(-EINVAL);
	if (!pctldesc->name)
		return ERR_PTR(-EINVAL);
    
    /*申请 pinctrl_dev 内存*/
	pctldev = kzalloc(sizeof(*pctldev), GFP_KERNEL);
	if (pctldev == NULL) {
		dev_err(dev, "failed to alloc struct pinctrl_dev\n");
		return ERR_PTR(-ENOMEM);
	}

	/* Initialize pin control device struct */
	pctldev->owner = pctldesc->owner;
	pctldev->desc = pctldesc;
    
    /* pinctrl_dev->driver_data = pinctrl->desc; */
	pctldev->driver_data = driver_data; 
	INIT_RADIX_TREE(&pctldev->pin_desc_tree, GFP_KERNEL);
	INIT_LIST_HEAD(&pctldev->gpio_ranges);
    
	pctldev->dev = dev;
	mutex_init(&pctldev->mutex);

	/* check core ops for sanity */
	ret = pinctrl_check_ops(pctldev);
	if (ret) {
		dev_err(dev, "pinctrl ops lacks necessary functions\n");
		goto out_err;
	}

	/* If we're implementing pinmuxing, check the ops for sanity */
	if (pctldesc->pmxops) {
		ret = pinmux_check_ops(pctldev);
		if (ret)
			goto out_err;
	}

	/* If we're implementing pinconfig, check the ops for sanity */
	if (pctldesc->confops) {
		ret = pinconf_check_ops(pctldev);
		if (ret)
			goto out_err;
	}

	/* Register all the pins */
	dev_dbg(dev, "try to register %d pins ...\n",  pctldesc->npins);
    
    /*每个pin都申请一个pin_desc结构，并挂到pin_desc_tree*/
	ret = pinctrl_register_pins(pctldev, pctldesc->pins, pctldesc->npins);
	if (ret) {
		dev_err(dev, "error during pin registration\n");
		pinctrl_free_pindescs(pctldev, pctldesc->pins,
				      pctldesc->npins);
		goto out_err;
	}

	mutex_lock(&pinctrldev_list_mutex);
    /*挂入全局pinctrldev_list进行管理*/
	list_add_tail(&pctldev->node, &pinctrldev_list);
	mutex_unlock(&pinctrldev_list_mutex);

    
	pctldev->p = pinctrl_get(pctldev->dev);

	if (!IS_ERR(pctldev->p)) {
		pctldev->hog_default =
			pinctrl_lookup_state(pctldev->p, PINCTRL_STATE_DEFAULT);
		if (IS_ERR(pctldev->hog_default)) {
			dev_dbg(dev, "failed to lookup the default state\n");
		} else {
			if (pinctrl_select_state(pctldev->p,
						pctldev->hog_default))
				dev_err(dev,
					"failed to select default state\n");
		}

		pctldev->hog_sleep =
			pinctrl_lookup_state(pctldev->p,
						    PINCTRL_STATE_SLEEP);
		if (IS_ERR(pctldev->hog_sleep))
			dev_dbg(dev, "failed to lookup the sleep state\n");
	}

	pinctrl_init_device_debugfs(pctldev);

	return pctldev;

out_err:
	mutex_destroy(&pctldev->mutex);
	kfree(pctldev);
	return ERR_PTR(ret);
}
```

### pinctrl_register_pins

```C
/*
 * 根据pinctrl_pin_desc中的内容，申请并初始化对应数量的pin_desc
 * 并按pin号，顺序挂入pinctrl_dev的pin_desc_tree上进行管理
 */
static int pinctrl_register_pins(struct pinctrl_dev *pctldev,
				 struct pinctrl_pin_desc const *pins,
				 unsigned num_descs)
{
	unsigned i;
	int ret = 0;

	for (i = 0; i < num_descs; i++) {
		ret = pinctrl_register_one_pin(pctldev, &pins[i]);
		if (ret)
			return ret;
	}

	return 0;
}
```

### pinctrl_register_one_pin

```C
static int pinctrl_register_one_pin(struct pinctrl_dev *pctldev,
				    const struct pinctrl_pin_desc *pin)
{
	struct pin_desc *pindesc;
    
	/*
	 * pinctrl_pin_desc 为const数组，已经是固定内容，pin number都是唯一的
	 * 检查pin->number是否已经存在pin_desc_tree里
	 */
	pindesc = pin_desc_get(pctldev, pin->number);
	if (pindesc != NULL) {
		dev_err(pctldev->dev, "pin %d already registered\n",
			pin->number);
		return -EINVAL;
	}
	
    /*申请 pin_desc 内存*/
	pindesc = kzalloc(sizeof(*pindesc), GFP_KERNEL);
	if (pindesc == NULL) {
		dev_err(pctldev->dev, "failed to alloc struct pin_desc\n");
		return -ENOMEM;
	}

	/* Set owner */
    /*该pin_desc归属某个pinctrl_dev*/
	pindesc->pctldev = pctldev;

	/* Copy basic pin info */
	if (pin->name) { /*name是固定内容*/
		pindesc->name = pin->name;
	} else {
		pindesc->name = kasprintf(GFP_KERNEL, "PIN%u", pin->number);
		if (pindesc->name == NULL) {
			kfree(pindesc);
			return -ENOMEM;
		}
		pindesc->dynamic_name = true;/*使用动态命名*/
	}

	pindesc->drv_data = pin->drv_data;/*私有数据，由前面可知，为空*/
	
    /*将pindesc按照pin->number的顺序，插入pin_desc_tree*/
	radix_tree_insert(&pctldev->pin_desc_tree, pin->number, pindesc);
	pr_debug("registered pin %d (%s) on %s\n",
		 pin->number, pindesc->name, pctldev->desc->name);
	return 0;
}
```

### pinctrl_get

```C
/**
 * pinctrl_get() - retrieves the pinctrl handle for a device
 * @dev: the device to obtain the handle for
 */
struct pinctrl *pinctrl_get(struct device *dev)
{
	struct pinctrl *p;

	if (WARN_ON(!dev))
		return ERR_PTR(-EINVAL);

	/*
	 * See if somebody else (such as the device core) has already
	 * obtained a handle to the pinctrl for this device. In that case,
	 * return another pointer to it.
	 */
	p = find_pinctrl(dev); /*查找该device中是否有pinctrl*/
	if (p != NULL) {
		dev_dbg(dev, "obtain a copy of previously claimed pinctrl\n");
		kref_get(&p->users);
		return p;
	}

	return create_pinctrl(dev);/*不存在，申请新的pinctrl*/
}
```

### find_pinctrl

```C
static struct pinctrl *find_pinctrl(struct device *dev)
{
	struct pinctrl *p;

	mutex_lock(&pinctrl_list_mutex);
	list_for_each_entry(p, &pinctrl_list, node)
		if (p->dev == dev) {
			mutex_unlock(&pinctrl_list_mutex);
			return p;
		}

	mutex_unlock(&pinctrl_list_mutex);
	return NULL;
}
```

### pinctrl

```C
/**
 * struct pinctrl_state - a pinctrl state for a device
 * @node: list node for struct pinctrl's @states field
 * @name: the name of this state
 * @settings: a list of settings for this state
 */
struct pinctrl_state {
	struct list_head node;
	const char *name;
	struct list_head settings;
};

struct kref {
        atomic_t refcount;
};

/**
 * struct pinctrl - per-device pin control state holder
 * @node: global list node
 * @dev: the device using this pin control handle
 * @states: a list of states for this device
 * @state: the current state
 * @dt_maps: the mapping table chunks dynamically parsed from device tree for
 *	this device, if any
 * @users: reference count
 */
struct pinctrl {
	struct list_head node;
	struct device *dev;
	struct list_head states;
	struct pinctrl_state *state;
	struct list_head dt_maps;
	struct kref users;
};
```

### create_pinctrl

```C
static struct pinctrl *create_pinctrl(struct device *dev)
{
	struct pinctrl *p;
	const char *devname;
	struct pinctrl_maps *maps_node;
	int i;
	struct pinctrl_map const *map;
	int ret;

	/*
	 * create the state cookie holder struct pinctrl for each
	 * mapping, this is what consumers will get when requesting
	 * a pin control handle with pinctrl_get()
	 */
    /*申请 pinctrl 内存*/
	p = kzalloc(sizeof(*p), GFP_KERNEL);
	if (p == NULL) {
		dev_err(dev, "failed to alloc struct pinctrl\n");
		return ERR_PTR(-ENOMEM);
	}
	p->dev = dev;/*记录所属dev*/
	INIT_LIST_HEAD(&p->states);
	INIT_LIST_HEAD(&p->dt_maps);

	ret = pinctrl_dt_to_map(p);
	if (ret < 0) {
		kfree(p);
		return ERR_PTR(ret);
	}

	devname = dev_name(dev);

	mutex_lock(&pinctrl_maps_mutex);
	/* Iterate over the pin control maps to locate the right ones */
	for_each_maps(maps_node, i, map) {
		/* Map must be for this device */
		if (strcmp(map->dev_name, devname))
			continue;

		ret = add_setting(p, map);
		/*
		 * At this point the adding of a setting may:
		 *
		 * - Defer, if the pinctrl device is not yet available
		 * - Fail, if the pinctrl device is not yet available,
		 *   AND the setting is a hog. We cannot defer that, since
		 *   the hog will kick in immediately after the device
		 *   is registered.
		 *
		 * If the error returned was not -EPROBE_DEFER then we
		 * accumulate the errors to see if we end up with
		 * an -EPROBE_DEFER later, as that is the worst case.
		 */
		if (ret == -EPROBE_DEFER) {
			pinctrl_free(p, false);
			mutex_unlock(&pinctrl_maps_mutex);
			return ERR_PTR(ret);
		}
	}
	mutex_unlock(&pinctrl_maps_mutex);

	if (ret < 0) {
		/* If some other error than deferral occured, return here */
		pinctrl_free(p, false);
		return ERR_PTR(ret);
	}

	kref_init(&p->users);

	/* Add the pinctrl handle to the global list*/
    /*pinctrl放到全局pinctrl_list进行管理*/
	mutex_lock(&pinctrl_list_mutex);
	list_add_tail(&p->node, &pinctrl_list);
	mutex_unlock(&pinctrl_list_mutex);

	return p;
}
```

### pinctrl_dt_to_map

```C
int pinctrl_dt_to_map(struct pinctrl *p)
{
	struct device_node *np = p->dev->of_node;
	int state, ret;
	char *propname;
	struct property *prop;
	const char *statename;
	const __be32 *list;
	int size, config;
	phandle phandle;
	struct device_node *np_config;

	/* CONFIG_OF enabled, p->dev not instantiated from DT */
	if (!np) {
		if (of_have_populated_dt()) /*确认of_root不为空*/
			dev_dbg(p->dev,
				"no of_node; not parsing pinctrl DT\n");
		return 0;
	}

	/* We may store pointers to property names within the node */
    /*增加kobj的引用计数*/
	of_node_get(np);

	/* For each defined state ID */
	for (state = 0; ; state++) {
		/* Retrieve the pinctrl-* property */
        /*检索 pinctrl-**/
		propname = kasprintf(GFP_KERNEL, "pinctrl-%d", state);
        
        /*找到其内容及size长度*/
		prop = of_find_property(np, propname, &size);
		kfree(propname);/*使用完毕，释放内存*/
		if (!prop) {
			if (state == 0) {
				of_node_put(np);
				return -ENODEV;
			}
			break;
		}
		list = prop->value;/*内容为某个节点的名字 <&state_0_node_a>*/
		size /= sizeof(*list);

		/* Determine whether pinctrl-names property names the state */
        /*在iomuxc节点作为根节点，找到第state个字符串 如"idle"的内容*/
		ret = of_property_read_string_index(np, "pinctrl-names",
						    state, &statename);
		/*
		 * If not, statename is just the integer state ID. But rather
		 * than dynamically allocate it and have to free it later,
		 * just point part way into the property name for the string.
		 */
		if (ret < 0) {
			/* strlen("pinctrl-") == 8 */
			statename = prop->name + 8;
		}

		/* For every referenced pin configuration node in it */
		for (config = 0; config < size; config++) {
			phandle = be32_to_cpup(list++);

			/* Look up the pin configuration node */
			np_config = of_find_node_by_phandle(phandle);
			if (!np_config) {
				dev_err(p->dev,
					"prop %s index %i invalid phandle\n",
					prop->name, config);
				ret = -EINVAL;
				goto err;
			}

			/* Parse the node */
			ret = dt_to_map_one_config(p, statename, np_config);
			of_node_put(np_config);
			if (ret < 0)
				goto err;
		}

		/* No entries in DT? Generate a dummy state table entry */
		if (!size) {
			ret = dt_remember_dummy_state(p, statename);
			if (ret < 0)
				goto err;
		}
	}

	return 0;

err:
	pinctrl_dt_free_maps(p);
	return ret;
}
```

### dt_to_map_one_config

```C
static int dt_to_map_one_config(struct pinctrl *p, const char *statename,
				struct device_node *np_config)
{
	struct device_node *np_pctldev;
	struct pinctrl_dev *pctldev;
	const struct pinctrl_ops *ops;
	int ret;
	struct pinctrl_map *map;
	unsigned num_maps;

	/* Find the pin controller containing np_config */
	np_pctldev = of_node_get(np_config);
	for (;;) {
		np_pctldev = of_get_next_parent(np_pctldev);
		if (!np_pctldev || of_node_is_root(np_pctldev)) {
			dev_info(p->dev, "could not find pctldev for node %s, deferring probe\n",
				np_config->full_name);
			of_node_put(np_pctldev);
			/* OK let's just assume this will appear later then */
			return -EPROBE_DEFER;
		}
		pctldev = get_pinctrl_dev_from_of_node(np_pctldev);
		if (pctldev)
			break;
		/* Do not defer probing of hogs (circular loop) */
		if (np_pctldev == p->dev->of_node) {
			of_node_put(np_pctldev);
			return -ENODEV;
		}
	}
	of_node_put(np_pctldev);

	/*
	 * Call pinctrl driver to parse device tree node, and
	 * generate mapping table entries
	 */
	ops = pctldev->desc->pctlops;
	if (!ops->dt_node_to_map) {
		dev_err(p->dev, "pctldev %s doesn't support DT\n",
			dev_name(pctldev->dev));
		return -ENODEV;
	}
	ret = ops->dt_node_to_map(pctldev, np_config, &map, &num_maps);
	if (ret < 0)
		return ret;

	/* Stash the mapping table chunk away for later use */
	return dt_remember_or_free_map(p, statename, pctldev, map, num_maps);
}
```





- 要记住，此时的probe中，dtb已经提取并转换成device_node（iomuxc节点生成platform_device，但其子节点不会生成platform_device）

- 看5.10源码时发现，imx_pinctrl_soc_info结构体发生了很大的变化，因此看回4.9

