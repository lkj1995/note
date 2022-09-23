### 作用

- 引脚的枚举与命名：描述使用了哪些pin，及pin的组，状态等信息
- 引脚复用（multiplexing）：如gpio复用为iic，uart等功能
- 引脚配置（configuration）：如上下拉，开漏，驱动能力，阻值等

### dts中描述

```shell
# iomuxc@20e0000 {
#     compatible = "fsl,imx6ul-iomuxc";
#     reg = <0x020e0000 0x4000>;
# };
#
# iomuxc_snvs: iomuxc-snvs@02290000 {
#	  compatible = "fsl,imx6ull-iomuxc-snvs";
#	  reg = <0x02290000 0x10000>;
# };
#
```

### struct imx6ul_pinctrl_driver

```c
/*厂商预先写好，该芯片的pin*/
static const struct pinctrl_pin_desc imx6ul_pinctrl_pads[] = {  
	IMX_PINCTRL_PIN(MX6UL_PAD_RESERVE0),
	IMX_PINCTRL_PIN(MX6UL_PAD_RESERVE1),
	IMX_PINCTRL_PIN(MX6UL_PAD_RESERVE2),
    ...
};

static const struct imx_pinctrl_soc_info imx6ul_pinctrl_info = {
    
    /*引脚描述*/
	.pins = imx6ul_pinctrl_pads,
        
     /*pin数量*/   
	.npins = ARRAY_SIZE(imx6ul_pinctrl_pads), 
    
    /*属性名*/
	.gpr_compatible = "fsl,imx6ul-iomuxc-gpr",
};

static const struct of_device_id imx6ul_pinctrl_of_match[] = {
	{ 
      .compatible = "fsl,imx6ul-iomuxc",
      .data = &imx6ul_pinctrl_info, 
    },
	{ 
       .compatible = "fsl,imx6ull-iomuxc-snvs", 
       .data = &imx6ull_snvs_pinctrl_info, 
    },
};

static struct platform_driver imx6ul_pinctrl_driver = {
	.driver = {
		.name = "imx6ul-pinctrl",
		.of_match_table = of_match_ptr(imx6ul_pinctrl_of_match),
	},
	.probe = imx6ul_pinctrl_probe,
};
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

### 1 imx6ul_pinctrl_probe()

```C
static int imx6ul_pinctrl_probe(struct platform_device *pdev)
{
	const struct imx_pinctrl_soc_info *pinctrl_info;
    
	 /* 获取 imx6ul_pinctrl_info，(soc的全部pin信息,如数量，引脚名字)*/	 
	pinctrl_info = of_device_get_match_data(&pdev->dev);

	return imx_pinctrl_probe(pdev, pinctrl_info);
}



/*
 * 利用厂商写的imx_pinctrl_soc_info，先生成pinctrl_desc
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

	/*申请 imx_pinctrl 内存*/
	ipctl = devm_kzalloc(&pdev->dev, sizeof(*ipctl), GFP_KERNEL);


	if (!(info->flags & IMX8_USE_SCU)) {
        
		/*获取"reg" resource*/
		res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
		ipctl->base = devm_ioremap_resource(&pdev->dev, res);
	}

    /*申请* pinctrl_desc */
	imx_pinctrl_desc = devm_kzalloc(&pdev->dev, sizeof(*imx_pinctrl_desc),
					GFP_KERNEL);

    /*设置 pinctrl_desc */
	imx_pinctrl_desc->name = dev_name(&pdev->dev);
    /* pins： 芯片引脚的pin号，名字 npins：pin数量 */    
	imx_pinctrl_desc->pins = info->pins;
	imx_pinctrl_desc->npins = info->npins;
	imx_pinctrl_desc->pctlops = &imx_pctrl_ops;
	imx_pinctrl_desc->pmxops = &imx_pmx_ops;
	imx_pinctrl_desc->confops = &imx_pinconf_ops;
	imx_pinctrl_desc->owner = THIS_MODULE;
    
    
	/* 当前info只记录了pin号，名字，数量
	 * 因此还需解析设备树的节点（设备树已经解析过了，只需获取device_node）
	 * 然后存入pinctrl_desc的imx_pin_group（组的信息及组的pin信息）
	 * 和imx_pmx_func（组的名字）
	 */	
	ret = imx_pinctrl_probe_dt(pdev, info);

    /* 
     * platform_set_drvdata() --->  pdev->driver_data = ipctl; 
     * 这时候，通过platform_device就能找到imx_pinctrl
     * 再通过imx_pinctrl找到imx_pinctrl_soc_info
     */
	ipctl->info = info;
	ipctl->dev = info->dev;
	platform_set_drvdata(pdev, ipctl);
    
    /*  根据pinctrl_desc信息，申请pinctrl_dev，并为"iomuxc"节点申请一个pinctrl */    
	ipctl->pctl = devm_pinctrl_register(&pdev->dev,
					    imx_pinctrl_desc, ipctl);

	return 0;
}
```

### 1-1 imx_pinctrl_probe_dt()

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
    /*获取of_node，准备获取compatible及compatible内容*/
	struct device_node *np = pdev->dev.of_node;
	struct device_node *child;
	u32 nfuncs = 0;
	u32 i = 0;
	bool flat_funcs;

    
    
     /* 返回0，子节点无"fsl,pins"，孙节点存在"fsl,pins" */
	flat_funcs = imx_pinctrl_dt_is_flat_functions(np);

	if (flat_funcs) {
		nfuncs = 1;
	} else {/*会进行此分支，false*/
        
        /*遍历iomuxc，记录func数量，只有"imx6ul-evk"1个子节点 */
		nfuncs = of_get_child_count(np);
	}
    
    /*保存func相关信息在 imx_pinctrl_soc_info */
	info->nfunctions = nfuncs;
    /*申请 imx_pmx_func 内存*/
	info->functions = devm_kzalloc(&pdev->dev, 
               nfuncs * sizeof(struct imx_pmx_func),  GFP_KERNEL);
    
	info->group_index = 0;
	if (flat_funcs) {
		info->ngroups = of_get_child_count(np);
	} else {/*进入此分支*/
		info->ngroups = 0;
        
         /* 计算func下的group数量，即 "imx6ul-evk" 节点下的子节点。*/
		for_each_child_of_node(np, child)
			info->ngroups += of_get_child_count(child);
        
	}
    
    /*
     * 分配 imx_pin_group 数组，保存func下的所有group信息，
     * 即每个子节点的信息存都放在 imx_pin_group 。
     */
	info->groups = devm_kzalloc(&pdev->dev, 
         info->ngroups * sizeof(struct imx_pin_group), GFP_KERNEL);


	if (flat_funcs) {
		imx_pinctrl_parse_functions(np, info, 0);
	} else {/*进入此分支*/
        
         /*解析iomux节点下的每个func，只有一个"imx6ul-evk" */
		for_each_child_of_node(np, child)
			imx_pinctrl_parse_functions(child, info, i++);
	}

	return 0;
}
```

### 1-1-1  imx_pinctrl_parse_functions()

```C
/*解析此func下的每个group*/
static int imx_pinctrl_parse_functions(struct device_node *np,
				       struct imx_pinctrl_soc_info *info,
				       u32 index)
{
	struct device_node *child;
	struct imx_pmx_func *func;
	struct imx_pin_group *grp;
	u32 i = 0;

	
    /*前面申请的内存，会遍历设置每个func*/
	func = &info->functions[index];

    /*func的名字，"imx6ul_evk"*/
	func->name = np->name;
    
    /*获得func下group个数*/
	func->num_groups = of_get_child_count(np);
    
    /*分配char*数组，保存所有group的名字*/
	func->groups = devm_kzalloc(info->dev,
			func->num_groups * sizeof(char *), GFP_KERNEL);

    
    /*解析func下的每个group*/
	for_each_child_of_node(np, child) { 
        
        /* 记录每个group的名字 */         
		func->groups[i] = child->name; 
		grp = &info->groups[info->group_index++];
		imx_pinctrl_parse_groups(child, grp, info, i++);
	}

	return 0;
}
```

### 1-1-1-1 imx_pinctrl_parse_groups()

```C
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


    /* 
     * FSL_PIN_SIZE = 24字节，即使用6个4字节来描述一个pin
     * 记录了关于offset，mux，config信息
     */
	pin_size = FSL_PIN_SIZE;
    


    /*格式检查，不符合fsl,pins = <PIN_FUNC_ID CONFIG ...>;返回错误 */
	list = of_get_property(np, "fsl,pins", &size);
	if (!list) {
		return -EINVAL;
	}


    /*不等于24字节的倍数，不符合规定的描述长度*/
	if (!size || size % pin_size) {
		return -EINVAL;
	}
    
    /*记录该group有多少个pin需要描述*/
	grp->npins = size / pin_size;
    
    /*创建imx_pin数组，用于保存pin描述的内容*/
	grp->pins = devm_kzalloc(info->dev, 
               grp->npins * sizeof(struct imx_pin), GFP_KERNEL);
    
    

    /*
     * 解析group下的所有pin，每个pin偏移24字节
     * 如：fsl,pin = <0x88, 0x318, 0x0, 0x5, 0x0, 0x10b0>;
     */       
	for (i = 0; i < grp->npins; i++) {
        /*记录第1个__be32*/
		u32 mux_reg = be32_to_cpu(*list++);
		u32 conf_reg;
		unsigned int pin_id;
		struct imx_pin_reg *pin_reg;
		struct imx_pin *pin = &grp->pins[i];


       /*记录第2个__be32*/
	   conf_reg = be32_to_cpu(*list++);

		pin_id = (mux_reg != -1) ? mux_reg / 4 : conf_reg / 4;
		pin_reg = &info->pin_regs[pin_id];
		pin->pin = pin_id;
		grp->pin_ids[i] = pin_id;
		pin_reg->mux_reg = mux_reg;
		pin_reg->conf_reg = conf_reg;
        
        /*记录第3+6个__be32*/
		pin->input_reg = be32_to_cpu(*list++);
		pin->mux_mode = be32_to_cpu(*list++);
		pin->input_val = be32_to_cpu(*list++); 
		config = be32_to_cpu(*list++);
        
	}

	return 0;
}
```

### 1-2 devm_pinctrl_register()

```C
/* 作为iomuxc，也是一个pinctrl的用户，因此调用了此函数 */
struct pinctrl_dev *devm_pinctrl_register(struct device *dev,
					  struct pinctrl_desc *pctldesc,
					  void *driver_data)
{
	struct pinctrl_dev **ptr, *pctldev;

    /* 根据 pinctrl_desc 注册 pinctrl_dev */
	pctldev = pinctrl_register(pctldesc, dev, driver_data);

	return pctldev;
}



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

    
    /*申请 pinctrl_dev */
	pctldev = kzalloc(sizeof(*pctldev), GFP_KERNEL);

    
	pctldev->owner = pctldesc->owner;
	pctldev->desc = pctldesc;
    
    /* pinctrl_dev->driver_data = imx_pinctrl;  */
	pctldev->driver_data = driver_data;
	INIT_RADIX_TREE(&pctldev->pin_desc_tree, GFP_KERNEL);
	INIT_LIST_HEAD(&pctldev->gpio_ranges);
	pctldev->dev = dev;         

       
   
     /* pinctrl_pin_desc转换成pin_desc，放入pinctrl_dev的pin list进行管理*/ 
	ret = pinctrl_register_pins(pctldev, pctldesc->pins, pctldesc->npins);
  
     /* 将 pinctrl_dev 挂入dev list，用于查找 */
	list_add_tail(&pctldev->node, &pinctrldev_list);

	/* 申请一个pinctrl */
	pctldev->p = pinctrl_get(pctldev->dev);
	
	return pctldev;
}
```

### 1-2-1 pinctrl_register_pins()

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
	
    /*每个pin，都会申请，设置一个pin_desc*/
	for (i = 0; i < num_descs; i++) {
		ret = pinctrl_register_one_pin(pctldev, &pins[i]);
		if (ret)
			return ret;
	}
	return 0;
}



/*申请一个pin_desc描述pinctrl_pin_desc，并按序号挂入pinctrl_dev的tree下*/
static int pinctrl_register_one_pin(struct pinctrl_dev *pctldev,
				    const struct pinctrl_pin_desc *pin)
{
	struct pin_desc *pindesc;
    
	
	/* 检查pin号是否存在pinctrl_dev的list */
	pindesc = pin_desc_get(pctldev, pin->number);
	
    /*不存在，申请 pin_desc 内存*/
	pindesc = kzalloc(sizeof(*pindesc), GFP_KERNEL);


    /*该pin_desc归属的pinctrl_dev*/
	pindesc->pctldev = pctldev;

	/* 设置pin_desc */
	if (pin->name) {
		pindesc->name = pin->name;
	} else {
		pindesc->name = kasprintf(GFP_KERNEL, "PIN%u", pin->number);
		if (pindesc->name == NULL) {
			kfree(pindesc);
			return -ENOMEM;
		}
		pindesc->dynamic_name = true;
	}
	pindesc->drv_data = pin->drv_data;
	
    /* 按照pin->number排序，挂入pinctrl_dev的list */
	radix_tree_insert(&pctldev->pin_desc_tree, pin->number, pindesc);

	return 0;
}
```

### 1-2-2 pinctrl_get()

```C
/* 该pinctrl属于"iomuxc"节点 */
struct pinctrl *pinctrl_get(struct device *dev)
{
	struct pinctrl *p;
    
	p = find_pinctrl(dev); 
	if (p != NULL) {
		dev_dbg(dev, "obtain a copy of previously claimed pinctrl\n");
		kref_get(&p->users);
		return p;
	}

    /*没找到，申请新的pinctrl*/
	return create_pinctrl(dev);
}
```



### 总结

- 在"iomuxc"节点中，申请一个pinctrl_desc，并根据imx_pinctrl_soc_info设置pinctrl_desc。
- 然后申请一个pinctrl_dev，并根据pinctrl_desc设置pinctrl_dev。
  - 申请，设置pinctrl，将"iomuxc"节点下的“pinctrl-names”的state，”pinctrl-0 = <&node1 &node2>”的config，和node中的”fsl,pins = xxx; ”的map进行处理，最后将map转换成settings。
  - 申请，设置pin_desc，并保存在pinctrl_dev的list，后续会被

