```shell
# dec -I dtb  -O dts imx6ull-mmc-npi.dtb > 11.dts /*反编译设备树*/
# vim 11.dts 
# /iomux
#  761             iomuxc@20e0000 {
#  762                 compatible = "fsl,imx6ul-iomuxc";
```

可知存在属性为 fsl,imx6ul-iomuxc，可与drv匹配成功,此过程为构造及初始化pinctrl，因为该节点iomuxc并不是真正的硬件，它的工作是为后续真正的dev和drv匹配提前做准备。

### struct imx6ul_pinctrl_driver

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

### 1 imx6ul_pinctrl_probe()

```C
static int imx6ul_pinctrl_probe(struct platform_device *pdev)
{
	const struct imx_pinctrl_soc_info *pinctrl_info;
	/*
	 * 其实在match已经进行过匹配了，这一步是为了获取 imx6ul_pinctrl_info 
	 * 里面存有了该soc的全部pin的信息（数量，引脚名字）
	 */
	pinctrl_info = of_device_get_match_data(&pdev->dev);
	if (!pinctrl_info)
		return -ENODEV;

	return imx_pinctrl_probe(pdev, pinctrl_info);
}
```

### 1-1 of_device_get_match_data()

```C
const void *of_device_get_match_data(const struct device *dev)
{
	const struct of_device_id *match;
    /*获取 imx6ul_pinctrl_info */
	match = of_match_device(dev->driver->of_match_table, dev);
	if (!match)
		return NULL;

	return match->data;
}
```

### 1-1-1 of_match_device()

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

### struct pinctrl_desc

```C
struct pinctrl_desc {
	const char *name;
    
    /*指向一个数组，记录引脚的枚举号与名字*/
	const struct pinctrl_pin_desc *pins;
    
    /*描述该数组的大小*/
	unsigned int npins;
    
    /*要先提取info，再解析设备树后，调用这些ops就可以根据state配置引脚*/
	const struct pinctrl_ops *pctlops;
	const struct pinmux_ops *pmxops;
	const struct pinconf_ops *confops;
    
	...
};
```

### struct  pinctrl_pin_desc

```C
struct pinctrl_pin_desc {
        unsigned number; /*厂商定义的pin号*/
        const char *name;/*该pin对应的名字*/
        void *drv_data;
};
```

### struct imx_pinctrl_soc_info

```C
struct imx_pinctrl_soc_info {
    /*
     * 保存在 /sys/bus/platform/devices/20e0000.iomuxc 
     * 即生成了 platform_device->dev
     */
	struct device *dev;
    
    /*指向一个数组，记录引脚的枚举号与名字*/
	const struct pinctrl_pin_desc *pins;
    /*pin总数量*/
	unsigned int npins;
    
	struct imx_pin_reg *pin_regs;
    
    /*指向一个数组，描述group的名字，pin数量，及每个pin的具体配置值*/
	struct imx_pin_group *groups;
    /*group的数量*/
	unsigned int ngroups; 
    
	unsigned int group_index;
    
    /*指向一个数组，记录了该func下的必要信息，func名字，group的名字，数量，用于cat */
	struct imx_pmx_func *functions;    
    /*等func的数量，等于1*/
	unsigned int nfunctions;
    
	unsigned int flags;
	const char *gpr_compatible;
};
```

### struct imx_pin_reg

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

### struct imx_pinctrl

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

### struct imx_pmx_func

```C
/**
 * struct imx_pmx_func - describes IMX pinmux functions
 * @name: the name of this specific function
 * @groups: corresponding pin groups
 * @num_groups: the number of groups
 */
struct imx_pmx_func {
	const char *name; /*imx6ul_evk，只有一个func*/
	const char **groups; /*用数组记录所有组的名字，我估计是用于cat 显示*/
	unsigned num_groups;/*func下，组的数量*/
};
```

### struct imx_pin_group

```C
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

### struct imx_pin

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
```

### struct pinctrl_dev

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

### 1-2 imx_pinctrl_probe()

```C
/*
 * 利用厂商写死的imx_pinctrl_soc_info，先生成pinctrl_desc
 * 再把dt解析，补充初始化imx_pinctrl_soc_info剩余的成员
 * 这样的好处是，厂商只定义了引脚，至于怎么用引脚的功能，就可以自行写dt
 * 效果就是，开机运行的过程，去解析dt，动态生成所需的外设功能
 */
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
    /*记录device*/
	info->dev = &pdev->dev; 

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

    /*
     * 申请 imx_pin_reg * npins 内存
     * 我猜测这个用于保存每个引脚的mux和conf的基地址
     */
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

    /*寻找属性 "fsl,input-sel",返回false，跳过*/
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

    /*pinctrl_desc的名字和dev的一致*/
	imx_pinctrl_desc->name = dev_name(&pdev->dev);
    
    /*
     * pins： 芯片引脚的pin号，名字
     * npins：pin数量
     */
	imx_pinctrl_desc->pins = info->pins;
	imx_pinctrl_desc->npins = info->npins;
    
    /*厂商写好的ops*/
	imx_pinctrl_desc->pctlops = &imx_pctrl_ops,
    /*mux相关ops，复用*/
	imx_pinctrl_desc->pmxops = &imx_pmx_ops,
    /*config相关ops，如上下拉*/
	imx_pinctrl_desc->confops = &imx_pinconf_ops,
    
	imx_pinctrl_desc->owner = THIS_MODULE,
    
	/* 当前info只记录了pin号，名字，数量
	 * 因此还需解析设备树的节点（设备树已经解析过了，只需获取device_node）
	 * 然后存入pinctrl_desc的imx_pin_group（组的信息及组的pin信息）
	 * 和imx_pmx_func（组的名字）
	 */
	ret = imx_pinctrl_probe_dt(pdev, info);
	if (ret) {
		dev_err(&pdev->dev, "fail to probe dt properties\n");
		return ret; 
	}
    
    /* 
     * platform_set_drvdata() --->  pdev->driver_data = ipctl; 
     * 这时候，通过platform_device就能找到imx_pinctrl
     * 再通过imx_pinctrl找到imx_pinctrl_soc_info
     */
	ipctl->info = info;
	ipctl->dev = info->dev;
	platform_set_drvdata(pdev, ipctl);
    
    /*
     * 注册并构造 pinctrl_dev
     */
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

### 1-2-1 imx_pinctrl_probe_dt()

```C
/*
 * 由下分析可知，一次分配好func下group的数组，然后根据偏移值
 * 再对每个group的pin一次分配数组，然后解析dt，并把内容填入group和pin的结构体
 * 完成后，imx_pinctrl_soc_info就保存了所有配置信息，解析完毕。
 * 下一步，则进行外设配置。
 */
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
    
    /*
     * false：如果子节点不存在，孙节点存在"fsl,pins"
     * ture:子节点存在"fsl,pins"
     */
	flat_funcs = imx_pinctrl_dt_is_flat_functions(np);

	if (flat_funcs) {
		nfuncs = 1;
	} else {/*会进行此分支，false*/
        
        /*遍历iomuxc子节点，记录func数量，等于1*/
		nfuncs = of_get_child_count(np);
		if (nfuncs <= 0) {
			dev_err(&pdev->dev, "no functions defined\n");
			return -EINVAL;
		}
	}
    
    /*保存func相关信息在 imx_pinctrl_soc_info*/
	info->nfunctions = nfuncs;
    /*申请 imx_pmx_func 内存*/
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
            /*func下的group数量*/
			info->ngroups += of_get_child_count(child);
	}
    
    /*分配 imx_pin_group 数组，用于保存dt解析的func下的所有group信息*/
	info->groups = devm_kzalloc(&pdev->dev, info->ngroups * sizeof(struct imx_pin_group),
					GFP_KERNEL);
	if (!info->groups)
		return -ENOMEM;

	if (flat_funcs) {
		imx_pinctrl_parse_functions(np, info, 0);
	} else {/*进入此分支*/
		for_each_child_of_node(np, child)/*对iomux节点下的每个func，都进行解析*/
			imx_pinctrl_parse_functions(child, info, i++);
	}

	return 0;
}
```

### 1-2-1-1 imx_pinctrl_parse_functions()

```C
/*解析单个func下的每个group*/
static int imx_pinctrl_parse_functions(struct device_node *np,
				       struct imx_pinctrl_soc_info *info,
				       u32 index)
{
	struct device_node *child;
	struct imx_pmx_func *func;
	struct imx_pin_group *grp;
	u32 i = 0;

	dev_dbg(info->dev, "parse function(%d): %s\n", index, np->name);
	
    /*前一个函数申请的内存，会遍历每个func*/
	func = &info->functions[index];

	/* Initialise function */
    /*func的名字，"imx6ul_evk"*/
	func->name = np->name;
    /*获得func下group个数*/
	func->num_groups = of_get_child_count(np);
	if (func->num_groups == 0) {
		dev_err(info->dev, "no groups defined in %s\n", np->full_name);
		return -EINVAL;
	}
    /*分配char*数组，保存所有group的名字*/
	func->groups = devm_kzalloc(info->dev,
			func->num_groups * sizeof(char *), GFP_KERNEL);

    
	for_each_child_of_node(np, child) { /*解析group下的每个in*/
        
        /* 在func "imx6ul_evk"，记录每个group的名字 */         
		func->groups[i] = child->name; 
		grp = &info->groups[info->group_index++];
		imx_pinctrl_parse_groups(child, grp, info, i++);
	}

	return 0;
}
```

### 1-2-1-1-1 imx_pinctrl_parse_groups()

```C
/*解析当前group下的pin，生成结构体保存fsl,pin的内容*/
/*
 * Each pin represented in fsl,pins consists of 5 u32 PIN_FUNC_ID and
 * 1 u32 CONFIG, so 24 types in total for each pin.
 * 5个PIN_FUNC_ID和1个CONFIG
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
     * FSL_PIN_SIZE = 24字节，即使用6个4字节来描述一个pin
     * 记录了关于offset，mux，config信息
     */
	if (info->flags & SHARE_MUX_CONF_REG)
		pin_size = SHARE_FSL_PIN_SIZE;
	else
		pin_size = FSL_PIN_SIZE;
    
	/* Initialise group */
    /*解析device_node，等同group，只是用处不同*/
	grp->name = np->name;

	/*
	 * the binding format is fsl,pins = <PIN_FUNC_ID CONFIG ...>,
	 * do sanity check and calculate pins number
	 */
    /*当该group存在"fsl,pins"，表示该group有pin需要解析，否则进行下一个group解析*/
	list = of_get_property(np, "fsl,pins", &size);
	if (!list) {
		dev_err(info->dev, "no fsl,pins property in node %s\n", np->full_name);
		return -EINVAL;
	}

	/* we do not check return since it's safe node passed down */
	if (!size || size % pin_size) {/*不等于24字节的倍数，不符合描述的长度*/
		dev_err(info->dev, "Invalid fsl,pins property in node %s\n", np->full_name);
		return -EINVAL;
	}
    
    /*记录该group有多少个pin需要描述*/
	grp->npins = size / pin_size;
    
    /*创建imx_pin数组，用于保存pin描述的内容*/
	grp->pins = devm_kzalloc(info->dev, grp->npins * sizeof(struct imx_pin),
				GFP_KERNEL);
    
    /*创建ids数组，这个不重要*/
	grp->pin_ids = devm_kzalloc(info->dev, grp->npins * sizeof(unsigned int),
				GFP_KERNEL);
    
	if (!grp->pins || ! grp->pin_ids)
		return -ENOMEM;

	for (i = 0; i < grp->npins; i++) {/*遍历所有pin，每个pin偏移24字节*/
        /*记录第1个__be32*/
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
            /*记录第2个__be32*/
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
		pin->input_reg = be32_to_cpu(*list++);/*记录第3个__be32*/
		pin->mux_mode = be32_to_cpu(*list++);/*记录第4个__be32*/
		pin->input_val = be32_to_cpu(*list++);/*记录第5个__be32*/

		/* SION bit is in mux register */
		config = be32_to_cpu(*list++);/*记录第6个__be32*/
		if (config & IMX_PAD_SION)
			pin->mux_mode |= IOMUXC_CONFIG_SION;
		pin->config = config & ~IMX_PAD_SION;

		dev_dbg(info->dev, "%s: 0x%x 0x%08lx", info->pins[pin_id].name,
				pin->mux_mode, pin->config);
	}

	return 0;
}
```

### 1-2-2 devm_pinctrl_register()

```C
/*ipctl->pctl = devm_pinctrl_register(&pdev->dev,imx_pinctrl_desc, ipctl);*/

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

### 1-2-2-1 pinctrl_register()

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
        
    /* 
     * pinctrl_dev->desc = pincrtl_desc;
     * 
     */    
	pctldev->desc = pctldesc;
    
    /* 
     * pinctrl_dev->driver_data = imx_pinctrl; 
     * 此时通过pinctrl_dev可找到imx_pinctrl_soc_info
     */
	pctldev->driver_data = driver_data; 
    
    /*初始化链表*/
	INIT_RADIX_TREE(&pctldev->pin_desc_tree, GFP_KERNEL);
	INIT_LIST_HEAD(&pctldev->gpio_ranges);
    
    /*保存了iomuxc这个device*/
	pctldev->dev = dev;
	mutex_init(&pctldev->mutex);

	/* check core ops for sanity */
    /*检查pctldev->desc的 三个ops是否存在，防止未定义*/
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
    
    /*
     * 注意，pinctrl_desc只是保存了pin号和名字，及ops
     * 对于每个pin，利用厂商写死的pinctrl_pin_desc
     * 申请一个pin_desc结构，并挂到pin_desc_tree进行管理
     */
	ret = pinctrl_register_pins(pctldev, pctldesc->pins, pctldesc->npins);
	if (ret) {
		dev_err(dev, "error during pin registration\n");
		pinctrl_free_pindescs(pctldev, pctldesc->pins,
				      pctldesc->npins);
		goto out_err;
	}

	mutex_lock(&pinctrldev_list_mutex);
    /*
     * 挂入全局pinctrldev_list进行管理，用于查找
     * 应该系统中只有一个pinctrl_dev
     * 我觉得这个应该也会用于cat
     */
	list_add_tail(&pctldev->node, &pinctrldev_list);
	mutex_unlock(&pinctrldev_list_mutex);

    /*
     * 查找的方法：存在一个pinctrl_list全局变量
     * 遍历pinctrl_list的pinctrl，如果该dev地址是一样的
     * 则说明找到了pinctrl，并返回该结构地址。
     * 
     * 如果是iomuxc节点调用的probe，pinctrl此时应该还不存在
     * 会生成一个新的pinctrl，然后保存在pinctrl_dev中
     * 此时就可以利用pinctrl_dev找pinctrl了
     */
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
        
		/*记录pinctrl中sleep_state，是为了能快速调用进入睡眠吗？*/
		pctldev->hog_sleep = 
			pinctrl_lookup_state(pctldev->p,
						    PINCTRL_STATE_SLEEP);
		if (IS_ERR(pctldev->hog_sleep))
			dev_dbg(dev, "failed to lookup the sleep state\n");
	}

    /*
     * 这个可能用于显示在文件系统
     * 里面已经包含了所有pin的名字，序号
     * func的信息，group的信息，pin的信息
     * config，mux的值等
    */
	pinctrl_init_device_debugfs(pctldev);

	return pctldev;

out_err:
	mutex_destroy(&pctldev->mutex);
	kfree(pctldev);
	return ERR_PTR(ret);
}
```

### 1-2-2-1-1 pinctrl_register_pins()

```C
/*
 * 根据pinctrl_pin_desc，申请并初始化对应数量的pin_desc
 * 并按pin号，顺序挂入pinctrl_dev的pin_desc_tree上进行管理
 * 由此可知，pinctrl_dev记录了soc的所有pin号和名字
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

### 1-2-2-1-1-1 pinctrl_register_one_pin()

```C
/*申请一个pin_desc描述pinctrl_pin_desc，并按序号挂入pinctrl_dev的tree下*/
static int pinctrl_register_one_pin(struct pinctrl_dev *pctldev,
				    const struct pinctrl_pin_desc *pin)
{
	struct pin_desc *pindesc;
    
	/*
	 * 检查pin号是否已存在pin_desc_tree里
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
    /*该pin_desc归属的pinctrl_dev*/
	pindesc->pctldev = pctldev;

	/* Copy basic pin info */
	if (pin->name) { /*pin的名字，为字符串*/
		pindesc->name = pin->name;
	} else {/*如果没名字，使用动态命名，如pin18*/
		pindesc->name = kasprintf(GFP_KERNEL, "PIN%u", pin->number);
		if (pindesc->name == NULL) {
			kfree(pindesc);
			return -ENOMEM;
		}
		pindesc->dynamic_name = true;
	}

	pindesc->drv_data = pin->drv_data;/*私有数据，由前面可知，为空*/
	
    /*将pindesc按照pin->number的顺序，插入pin_desc_tree*/
	radix_tree_insert(&pctldev->pin_desc_tree, pin->number, pindesc);
	pr_debug("registered pin %d (%s) on %s\n",
		 pin->number, pindesc->name, pctldev->desc->name);
	return 0;
}
```

### 1-2-2-1-1-2 pinctrl_get()

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
    /*查找全局pinctrl_list中是否有pinctrl对应传入的dev，即该device自己的pinctrl*/
	p = find_pinctrl(dev); 
	if (p != NULL) {
		dev_dbg(dev, "obtain a copy of previously claimed pinctrl\n");
		kref_get(&p->users);
		return p;
	}

    /*没找到，则为该device申请新的pinctrl*/
	return create_pinctrl(dev);
}
```

### 1-2-2-1-1-2-1 find_pinctrl()

```C
static struct pinctrl *find_pinctrl(struct device *dev)
{
	struct pinctrl *p;

	mutex_lock(&pinctrl_list_mutex);
	list_for_each_entry(p, &pinctrl_list, node)/*遍历pinctrl_list*/
		if (p->dev == dev) {/*是否这个pinctrl属于这个dev*/
			mutex_unlock(&pinctrl_list_mutex);
			return p;
		}

	mutex_unlock(&pinctrl_list_mutex);
	return NULL;
}
```

### struct pinctrl

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

### 1-2-2-1-1-2-2 create_pinctrl()

```C
/*传入的dev属于iomuxc节点，上面创建的与dev关联的pinctrl_dev，也是属于iomuxc节点*/
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
    /*
     * 记录所属device，为啥不保存在 struct dev_pin_info  *pins;
     * 是执行really_probe的时候才会创建pins的内存。
     * 这时候查找全局list来匹配dev，再放入pins
     */
	p->dev = dev;
    
    

    /*初始化链表，用于保存pinctrl-name中的各个状态，如"default"*/
	INIT_LIST_HEAD(&p->states);
    
    /*初始化链表，用于保存pinctrl_map*/
	INIT_LIST_HEAD(&p->dt_maps);

    /*
     * 取出 pinctrl_dev--> imx_pinctrl --> 
     * imx_pinctrl_soc_info --> imx_pin_group --> imx_pin
     * 生成保存config值的pinctrl_map
     */
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

### 1-2-2-1-1-2-2-1 pinctrl_dt_to_map()

```C
int pinctrl_dt_to_map(struct pinctrl *p)
{
    /* 
     * pinctrl_name = "default"; pinctrl-0 = <&node0>;
     * 先前解析的是iomuxc中的group，此时需要解析上面的node
     * 对应真正的device，而不是platform_device的iomuxc
     */
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
    /*
     * state：对应phandle数量
     * config：对应引用的node数量
     */
	for (state = 0; ; state++) {
		/* Retrieve the pinctrl-* property */
        
        /*如：构造 pinctrl-0 字符串*/
		propname = kasprintf(GFP_KERNEL, "pinctrl-%d", state);
        
        /*找到属性"pinctrl-0"，获取内容及size长度*/
		prop = of_find_property(np, propname, &size);
        
		kfree(propname);/*使用完毕，释放内存*/
        
		if (!prop) {/*如果没有该属性，则退出*/
			if (state == 0) {
				of_node_put(np);
				return -ENODEV;
			}
			break;
		}
        
        /*内容：phandle值（如 0x1f），引用了iomuxc中的某个group*/
		list = prop->value;
        
        /*引用了多少个node（phandle）*/
		size /= sizeof(*list);

		/* Determine whether pinctrl-names property names the state */
        /*处理状态，找到第state个字符串 如"idle" "sleep"*/
		ret = of_property_read_string_index(np, "pinctrl-names",
						    state, &statename);
		/*
		 * If not, statename is just the integer state ID. But rather
		 * than dynamically allocate it and have to free it later,
		 * just point part way into the property name for the string.
		 */
		if (ret < 0) { /*如果有pinctrl-0，但没有pinctrl-name，则指向'0'这个字符*/
			/* strlen("pinctrl-") == 8 */
			statename = prop->name + 8;
		}

		/* For every referenced pin configuration node in it */
        /*
         * state：对应phandle数量，config：对应引用的node数量
         * pinctrl-0中，为当前state选中的group
         * 里面的每个pin创建pinctrl_map结构
         */
		for (config = 0; config < size; config++) {
            
            /*list++,如果该 pinctrl-* 后续还有其他phandle的话*/
			phandle = be32_to_cpup(list++);

			/* Look up the pin configuration node */
            /*根据phandle找到对应的节点device_node*/
			np_config = of_find_node_by_phandle(phandle);
			if (!np_config) {
				dev_err(p->dev,
					"prop %s index %i invalid phandle\n",
					prop->name, config);
				ret = -EINVAL;
				goto err;
			}

			/* Parse the node */
            /*为当前state的当前phandle，创建一个pinctrl_map用于记录*/
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

### 1-2-2-1-1-2-2-1-1 dt_to_map_one_config()

```C
/*根据group及其pin，创建pinctrl_map */
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
    /*增加kobj的引用计数*/
	np_pctldev = of_node_get(np_config);
    
	for (;;) {
        /*获取其parent，应该是iomuxc节点,该pinctrl_dev保存了生成map的必要信息*/
		np_pctldev = of_get_next_parent(np_pctldev);
		if (!np_pctldev || of_node_is_root(np_pctldev)) {
			dev_info(p->dev, "could not find pctldev for node %s, deferring probe\n",
				np_config->full_name);
			of_node_put(np_pctldev);
			/* OK let's just assume this will appear later then */
			return -EPROBE_DEFER;
		}
        
        /*
         * 寻找全局pinctrldev_list
         * 找到 pctldev->dev->of_node == np_pctldev
         * 这里有个疑惑，是imx6ul_evk下，每个group都会生成一个pinctrl_dev吗
         */
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
    /*获取pinctrl ops，这个函数由芯片厂商写，处理自家芯片的dt配置*/
	ops = pctldev->desc->pctlops;
    
	if (!ops->dt_node_to_map) {
		dev_err(p->dev, "pctldev %s doesn't support DT\n",
			dev_name(pctldev->dev));
		return -ENODEV;
	}
    /*
     * 执行imx_dt_node_to_map,
     * 利用需要生成map的节点device_node(np_config)
     * 创建一个pinctrl_map，解析和生成需要pinctrl_dev
     */
	ret = ops->dt_node_to_map(pctldev, np_config, &map, &num_maps);
	if (ret < 0)
		return ret;

	/* Stash the mapping table chunk away for later use */
	return dt_remember_or_free_map(p, statename, pctldev, map, num_maps);
}
```

### struct pinctrl_map_mux

```C
struct pinctrl_map_mux {
	const char *group;
	const char *function;
};
```

### struct pinctrl_map_configs

```C
struct pinctrl_map_configs {
	const char *group_or_pin;
	unsigned long *configs;
	unsigned num_configs;
};
```

### enum pinctrl_map_type

```C
enum pinctrl_map_type {
	PIN_MAP_TYPE_INVALID,
	PIN_MAP_TYPE_DUMMY_STATE,
	PIN_MAP_TYPE_MUX_GROUP,
	PIN_MAP_TYPE_CONFIGS_PIN,
	PIN_MAP_TYPE_CONFIGS_GROUP,
};
```

### struct pinctrl_map

```C
struct pinctrl_map {
	const char *dev_name;
	const char *name;
	enum pinctrl_map_type type;
	const char *ctrl_dev_name;
	union {
		struct pinctrl_map_mux mux;
		struct pinctrl_map_configs configs;
	} data;
};
```

### struct pinctrl_state

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

```

### struct pinctrl_setting

```C
/**
 * struct pinctrl_setting - an individual mux or config setting
 * @node: list node for struct pinctrl_settings's @settings field
 * @type: the type of setting
 * @pctldev: pin control device handling to be programmed. Not used for
 *   PIN_MAP_TYPE_DUMMY_STATE.
 * @dev_name: the name of the device using this state
 * @data: Data specific to the setting type
 */
struct pinctrl_setting {
	struct list_head node;
	enum pinctrl_map_type type;
	struct pinctrl_dev *pctldev;
	const char *dev_name;
	union {
		struct pinctrl_setting_mux mux;
		struct pinctrl_setting_configs configs;
	} data;
};
```

### struct pinctrl_setting_mux

```

/**
 * struct pinctrl_setting_mux - setting data for MAP_TYPE_MUX_GROUP
 * @group: the group selector to program
 * @func: the function selector to program
 */
struct pinctrl_setting_mux {
	unsigned group;
	unsigned func;
};
```

### struct pinctrl_setting_configs

```C
/**
 * struct pinctrl_setting_configs - setting data for MAP_TYPE_CONFIGS_*
 * @group_or_pin: the group selector or pin ID to program
 * @configs: a pointer to an array of config parameters/values to program into
 *	hardware. Each individual pin controller defines the format and meaning
 *	of config parameters.
 * @num_configs: the number of entries in array @configs
 */
struct pinctrl_setting_configs {
	unsigned group_or_pin;
	unsigned long *configs;
	unsigned num_configs;
};
```

### 1-2-2-1-1-2-2-1-1-1 imx_dt_node_to_map()

```C
/*传入的np为需要生成map的group*/
static int imx_dt_node_to_map(struct pinctrl_dev *pctldev,
			struct device_node *np,
			struct pinctrl_map **map, unsigned *num_maps)
{
    /*
     * ipctl = pctldev->driver_data;
     * 获得私有指针指向的imx_pinctrl 
     * 取出imx_pinctrl_soc_info，用于生成pinctrl_map
     */
	struct imx_pinctrl *ipctl = pinctrl_dev_get_drvdata(pctldev);
	const struct imx_pinctrl_soc_info *info = ipctl->info;
	const struct imx_pin_group *grp;
	struct pinctrl_map *new_map;
	struct device_node *parent;
	int map_num = 1;/*记住这个是1开始，生成n+1个pinctrl_map*/
	int i, j;

	/*
	 * first find the group of this node and check if we need create
	 * config maps for pins
	 */
    /*
     * 遍历整个imx_pinctrl_soc_info->groups[i].name
     * 找到名字匹配的group，取出对应组的imx_pin_group结构
     * 用于生成pinctrl_map
     */
	grp = imx_pinctrl_find_group_by_name(info, np->name);
	if (!grp) {
		dev_err(info->dev, "unable to find group for node %s\n",
			np->name);
		return -EINVAL;
	}

     /*
      * #define IMX_NO_PAD_CTL  0x80000000   //no pin config need
      * grp->npins：该group的pin数量，也表示生成map的数量
      */
	for (i = 0; i < grp->npins; i++) {
		if (!(grp->pins[i].config & IMX_NO_PAD_CTL))
			map_num++;
	}

    /*申请 n+1个pinctrl_map 内存*/
	new_map = kmalloc(sizeof(struct pinctrl_map) * map_num, GFP_KERNEL);
	if (!new_map)
		return -ENOMEM;

    /*二级指针，用于返回*/
	*map = new_map;
	*num_maps = map_num;

	/* create mux map */
    /*取parent，获得function*/
	parent = of_get_parent(np);
	if (!parent) {
		kfree(new_map);
		return -EINVAL;
	}
    
    /*标记为 PIN_MAP_TYPE_MUX_GROUP*/
	new_map[0].type = PIN_MAP_TYPE_MUX_GROUP;
    
    /*"imx6ul_evk"*/
	new_map[0].data.mux.function = parent->name;
    
    /*如："i2cgrp"*/
	new_map[0].data.mux.group = np->name;
    
	of_node_put(parent);

	/* create config map */
	new_map++;
    
    /*记录该group的每个pin的配置值*/
	for (i = j = 0; i < grp->npins; i++) {
		if (!(grp->pins[i].config & IMX_NO_PAD_CTL)) {
            /*标记为pin*/
			new_map[j].type = PIN_MAP_TYPE_CONFIGS_PIN;
            
            /*
             * 保存在imx_pinctrl_soc_info的imx_pin_group
             * 里面的imx_pin的pin成员，指的是pin号
             * 根据pin号，在pinctrl_dev的pin_desc_tree中
             * 找到pin号对应的名字，并返回
             */
			new_map[j].data.configs.group_or_pin =
					pin_get_name(pctldev, grp->pins[i].pin);
            
            /*
             * 如：MX6UL_PAD_UART1_RTS_B__GPIO1_IO19 0x17059
             * config值：0x17059
             */
			new_map[j].data.configs.configs = &grp->pins[i].config;
			new_map[j].data.configs.num_configs = 1;
			j++;
		}
	}

	dev_dbg(pctldev->dev, "maps: function %s group %s num %d\n",
		(*map)->data.mux.function, (*map)->data.mux.group, map_num);

	return 0;
}
```

### 1-2-2-1-1-2-2-2 add_setting()

```C
/*
 * 关系：pinctrl->state -> state1 -> setting1
 *                               -> setting2               
 *                     -> state2 -> setting3
 *                               -> setting4
 *          
 * 为pinctrl_state中的每个map都创建一个setting，在create_pinctrl中被轮循调用
 */
static int add_setting(struct pinctrl *p, struct pinctrl_map const *map)
{
	struct pinctrl_state *state;
	struct pinctrl_setting *setting;
	int ret;

	state = find_state(p, map->name);
	if (!state)/*构造pinctrl情况，此state为空，创建state*/
		state = create_state(p, map->name);
	if (IS_ERR(state))
		return PTR_ERR(state);

	if (map->type == PIN_MAP_TYPE_DUMMY_STATE)
		return 0;

    /*申请 setting 内存*/
	setting = kzalloc(sizeof(*setting), GFP_KERNEL);
	if (setting == NULL) {
		dev_err(p->dev,
			"failed to alloc struct pinctrl_setting\n");
		return -ENOMEM;
	}
	
    /*复制该类型。如:PIN_MAP_TYPE_MUX_GROUP*/
	setting->type = map->type;

	setting->pctldev = get_pinctrl_dev_from_devname(map->ctrl_dev_name);
	if (setting->pctldev == NULL) {
		kfree(setting);
		/* Do not defer probing of hogs (circular loop) */
		if (!strcmp(map->ctrl_dev_name, map->dev_name))
			return -ENODEV;
		/*
		 * OK let us guess that the driver is not there yet, and
		 * let's defer obtaining this pinctrl handle to later...
		 */
		dev_info(p->dev, "unknown pinctrl device %s in map entry, deferring probe",
			map->ctrl_dev_name);
		return -EPROBE_DEFER;
	}

	setting->dev_name = map->dev_name;
	
    /*将map转换成setting*/
	switch (map->type) {
	case PIN_MAP_TYPE_MUX_GROUP:
		ret = pinmux_map_to_setting(map, setting);
		break;
	case PIN_MAP_TYPE_CONFIGS_PIN:
	case PIN_MAP_TYPE_CONFIGS_GROUP:
		ret = pinconf_map_to_setting(map, setting);
		break;
	default:
		ret = -EINVAL;
		break;
	}
	if (ret < 0) {
		kfree(setting);
		return ret;
	}

    /*加入state->node 链表，表示该state有多少种setting*/
	list_add_tail(&setting->node, &state->settings);

	return 0;
}
```

### 1-2-2-1-1-2-2-2-1 create_state()

```C
static struct pinctrl_state *create_state(struct pinctrl *p,
					  const char *name)
{
	struct pinctrl_state *state;

    /*申请 state 内存*/
	state = kzalloc(sizeof(*state), GFP_KERNEL);
	if (state == NULL) {
		dev_err(p->dev,
			"failed to alloc struct pinctrl_state\n");
		return ERR_PTR(-ENOMEM);
	}

	state->name = name;
	INIT_LIST_HEAD(&state->settings);

    /*加入 pinctrl->states 链表，表示为该device有多少种state*/
	list_add_tail(&state->node, &p->states);

	return state;
}
```

### 1-2-2-1-1-2-2-2-2 pinmux_map_to_setting

```C
int pinmux_map_to_setting(struct pinctrl_map const *map,
			  struct pinctrl_setting *setting)
{
	struct pinctrl_dev *pctldev = setting->pctldev;
	const struct pinmux_ops *pmxops = pctldev->desc->pmxops;
	char const * const *groups;
	unsigned num_groups;
	int ret;
	const char *group;

	if (!pmxops) {
		dev_err(pctldev->dev, "does not support mux function\n");
		return -EINVAL;
	}

    /*function name转换为index*/
	ret = pinmux_func_name_to_selector(pctldev, map->data.mux.function);
	if (ret < 0) {
		dev_err(pctldev->dev, "invalid function %s in map table\n",
			map->data.mux.function);
		return ret;
	}
    /*记录index*/
	setting->data.mux.func = ret;

    /*该func下有哪些group，保存在groups*/
	ret = pmxops->get_function_groups(pctldev, setting->data.mux.func,
					  &groups, &num_groups);
	if (ret < 0) {
		dev_err(pctldev->dev, "can't query groups for function %s\n",
			map->data.mux.function);
		return ret;
	}
	if (!num_groups) {
		dev_err(pctldev->dev,
			"function %s can't be selected on any group\n",
			map->data.mux.function);
		return -EINVAL;
	}
    /*判断group是否能够复用mux为该func*/
	if (map->data.mux.group) {
		group = map->data.mux.group;
		ret = match_string(groups, num_groups, group);
		if (ret < 0) {
			dev_err(pctldev->dev,
				"invalid group \"%s\" for function \"%s\"\n",
				group, map->data.mux.function);
			return ret;
		}
	} else {
		group = groups[0];
	} 

    /*group name转换为index*/
	ret = pinctrl_get_group_selector(pctldev, group);
	if (ret < 0) {
		dev_err(pctldev->dev, "invalid group %s in map table\n",
			map->data.mux.group);
		return ret;
	}
    /*记录index*/
	setting->data.mux.group = ret;

	return 0;
}
```

### 1-2-2-1-1-2-2-2-2 pinconf_map_to_setting

```C
int pinconf_map_to_setting(struct pinctrl_map const *map,
			  struct pinctrl_setting *setting)
{
	struct pinctrl_dev *pctldev = setting->pctldev;
	int pin;

	switch (setting->type) {
	case PIN_MAP_TYPE_CONFIGS_PIN:
		pin = pin_get_from_name(pctldev,
					map->data.configs.group_or_pin);
		if (pin < 0) {
			dev_err(pctldev->dev, "could not map pin config for \"%s\"",
				map->data.configs.group_or_pin);
			return pin;
		}
		setting->data.configs.group_or_pin = pin;
		break;
	case PIN_MAP_TYPE_CONFIGS_GROUP:
		pin = pinctrl_get_group_selector(pctldev,
					 map->data.configs.group_or_pin);
		if (pin < 0) {
			dev_err(pctldev->dev, "could not map group config for \"%s\"",
				map->data.configs.group_or_pin);
			return pin;
		}
		setting->data.configs.group_or_pin = pin;
		break;
	default:
		return -EINVAL;
	}

	setting->data.configs.num_configs = map->data.configs.num_configs;
	setting->data.configs.configs = map->data.configs.configs;

	return 0;
}
```



