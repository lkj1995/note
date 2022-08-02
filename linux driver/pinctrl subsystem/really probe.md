### 调用顺序

```C
device_attach() 
	__device_attach() 
		__device_attach_driver() 
			driver_probe_device()
				really_probe()
/*在dev与drv进行probe时的调用顺序，会在really_probe()中注册pinctrl*/	
```

### struct device

```c
struct device {
 ...
#ifdef CONFIG_PINCTRL
    /*在device中，使用pins成员，描述引脚的信息*/
	struct dev_pin_info	*pins; 
#endif
 ...
};
```

### struct dev_pin_info

```C
struct dev_pin_info {
	struct pinctrl *p;/*自定义state会添加到p*/
    
    /*dts中描述的 pinctrl-name = "default";*/
	struct pinctrl_state *default_state;
    
	struct pinctrl_state *init_state;
#ifdef CONFIG_PM
     /*同上，"sleep"*/
	struct pinctrl_state *sleep_state;
     /*同上，"idle"*/
	struct pinctrl_state *idle_state;
#endif
};
```

### struct pinctrl_ops

```C
static const struct pinctrl_ops imx_pctrl_ops = {
	.get_groups_count = imx_get_groups_count,
	.get_group_name = imx_get_group_name,
	.get_group_pins = imx_get_group_pins,
	.pin_dbg_show = imx_pin_dbg_show,
	.dt_node_to_map = imx_dt_node_to_map,
	.dt_free_map = imx_dt_free_map,
};
```

### struct pinmux_ops

```C
static const struct pinmux_ops imx_pmx_ops = {
	.get_functions_count = imx_pmx_get_funcs_count,
	.get_function_name = imx_pmx_get_func_name,
	.get_function_groups = imx_pmx_get_groups,
	.set_mux = imx_pmx_set,
	.gpio_request_enable = imx_pmx_gpio_request_enable,
	.gpio_disable_free = imx_pmx_gpio_disable_free,
	.gpio_set_direction = imx_pmx_gpio_set_direction,
};
```

### struct pinconf_ops

```C
static const struct pinconf_ops imx_pinconf_ops = {
	.pin_config_get = imx_pinconf_get,
	.pin_config_set = imx_pinconf_set,
	.pin_config_dbg_show = imx_pinconf_dbg_show,
	.pin_config_group_dbg_show = imx_pinconf_group_dbg_show,
};
```

### 1 really_probe()

```C
static int really_probe(struct device *dev, struct device_driver *drv)
{
	...
        
	/* If using pinctrl, bind pins now before probing */
    /*为某个设备绑定所需的引脚pin*/    
	ret = pinctrl_bind_pins(dev);
	...
}


int pinctrl_bind_pins(struct device *dev)
{
	int ret;

    /*申请 dev_pin_info 内存*/
	dev->pins = devm_kzalloc(dev, sizeof(*(dev->pins)), GFP_KERNEL);

    /*
     * 获取该device所属的pinctrl，
     * 如果不存在，则创建pinctrl，并且生成map，
     * 再将map转换成setting
     * 返回的pinctrl保存在dev_pin_info中
     */
	dev->pins->p = devm_pinctrl_get(dev);

    
	/*获取default状态，并保存在device->pins->default_state上*/
	dev->pins->default_state = pinctrl_lookup_state(dev->pins->p,
					PINCTRL_STATE_DEFAULT);
	if (IS_ERR(dev->pins->default_state)) {
		dev_dbg(dev, "no default pinctrl state\n");
		ret = 0;
		goto cleanup_get;
	}
	/*获取init状态，并保存在device->pins->init_state上*/
	dev->pins->init_state = pinctrl_lookup_state(dev->pins->p,
					PINCTRL_STATE_INIT);
	if (IS_ERR(dev->pins->init_state)) {

		/*优先选择default状态*/
		ret = pinctrl_select_state(dev->pins->p,
					   dev->pins->default_state);
	} else {
		ret = pinctrl_select_state(dev->pins->p, dev->pins->init_state);
	}

	return 0;
}

```

### 1-1 devm_pinctrl_get()

```C
struct pinctrl *devm_pinctrl_get(struct device *dev)
{
	struct pinctrl **ptr, *p;

	p = pinctrl_get(dev);

	return p;
}


struct pinctrl *pinctrl_get(struct device *dev)
{
	struct pinctrl *p;

    /* 
     * pinctrl_list中寻找pinctrl，
     * 第一次不存在，执行create_pinctrl() 。
     */
	p = find_pinctrl(dev);

	return create_pinctrl(dev);
}


static struct pinctrl *create_pinctrl(struct device *dev)
{
	struct pinctrl *p;
	const char *devname;
	struct pinctrl_maps *maps_node;
	int i;
	struct pinctrl_map const *map;
	int ret;


    /* 申请 pinctrl */
	p = kzalloc(sizeof(*p), GFP_KERNEL);

	p->dev = dev;
	INIT_LIST_HEAD(&p->states);
	INIT_LIST_HEAD(&p->dt_maps);

    /* 
     * 设置 pinctrl,对使用的pinctrl进行展开
     * 1. state：pinctrl-names = "idle" "default";
     *	  解析每个状态state。
     * 2. config: pinctrl-0 = <&NODE1, &NODE2>;
     *			  pinctrl-1 = <&NODE3, &NODE4>;
     *    解析当前状态的每个NODE。
     * 3. map： NODE1 = { 
     *			 fsl,pins =  <
     *              MX6UL_PAD_UART3_CTS_B__FLEXCAN1_TX  0x000010B0
     *              MX6UL_PAD_UART3_RTS_B__FLEXCAN1_RX  0x000010B0
     *           >;}
     *    解析当前NODE中的pin信息。         
     */		
	ret = pinctrl_dt_to_map(p);

	devname = dev_name(dev);

	/**/
	for_each_maps(maps_node, i, map) {
		/* Map must be for this device */
		if (strcmp(map->dev_name, devname))
			continue;		
		
        /* 每个map转换成setting，生成index */
		ret = add_setting(p, map);

	}

	/*pinctrl绑定node后，放入list，可使用node进行查找*/
	list_add_tail(&p->node, &pinctrl_list);

	return p;
}
```

### 1-1-1 pinctrl_dt_to_map()

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

   
     /* state："pinctrl-names"中的 "idle" "default" "sleep"等 */
	for (state = 0; ; state++) {
        
        /* 构造如 "pinctrl-0" 的字符串 */
		propname = kasprintf(GFP_KERNEL, "pinctrl-%d", state);
        
        /*找到名为"pinctrl-0"的属性，获取内容及prop和长度size*/
		prop = of_find_property(np, propname, &size);
           
        /*内容：phandle值（如 0x1f），引用了iomuxc中的某个group*/
		list = prop->value;
        
        /*引用了多少个node（phandle数量）*/
		size /= sizeof(*list);

		/* Determine whether pinctrl-names property names the state */
        /*处理状态，找到第state个字符串 如"idle" "sleep"*/
		ret = of_property_read_string_index(np, "pinctrl-names",
						    state, &statename);

        
        /*
         * 处理每个state中的所有config，为每个
         * config创建pinctrl_map，即多少组引脚
         * 需要被记录到pinctrl_map。
         */
		for (config = 0; config < size; config++) {
            
            /*list++, 指向下一个phandle*/
			phandle = be32_to_cpup(list++);

			/* Look up the pin configuration node */
            /*根据phandle找到"iomuxc"中的某个group*/
			np_config = of_find_node_by_phandle(phandle);

             
            /* 生成1个pinctrl_map*/
			ret = dt_to_map_one_config(p, statename, np_config);

		}
	}

	return 0;
}
```

### 1-1-1-1 dt_to_map_one_config()

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

    
	for (;;) {
               
        /* 获取parent("iomuxc")node */
		np_pctldev = of_get_next_parent(np_config);
        
        /* 在 pinctrldev_list，根据of_node找到父亲的 pinctrl_dev */        
		pctldev = get_pinctrl_dev_from_of_node(np_pctldev);     
	}

    /* 获取pinctrl_desc的fops，已在iomuxc节点中设置 */
	ops = pctldev->desc->pctlops;
    

    /*
     * 执行 imx_dt_node_to_map()，
     * 读取soc_info构造的信息，根据pin数量，
     * 申请n+1个pinctrl_map,设置pinctrl_map
     */
	ret = ops->dt_node_to_map(pctldev, np_config, &map, &num_maps);
	if (ret < 0)
		return ret;

	/* Stash the mapping table chunk away for later use */
	return dt_remember_or_free_map(p, statename, pctldev, map, num_maps);
}
```

### 1-1-1-1-1 imx_dt_node_to_map()

```C

static int imx_dt_node_to_map(struct pinctrl_dev *pctldev,
			struct device_node *np,
			struct pinctrl_map **map, unsigned *num_maps)
{
    
     /*获得私有指针指向的imx_pinctrl  */
	struct imx_pinctrl *ipctl = pinctrl_dev_get_drvdata(pctldev);
    
    /* 取出imx_pinctrl_soc_info */
	const struct imx_pinctrl_soc_info *info = ipctl->info;
	const struct imx_pin_group *grp;
	struct pinctrl_map *new_map;
	struct device_node *parent;
    
    /*生成n+1个pinctrl_map*/
	int map_num = 1;
	int i, j;

    
     /* 在soc info，根据名字匹配imx_pin_group */
	grp = imx_pinctrl_find_group_by_name(info, np->name);

     /* grp->npins：根据group的pin数量，确定需要的map数量*/
	for (i = 0; i < grp->npins; i++) {
		if (!(grp->pins[i].config & IMX_NO_PAD_CTL))
			map_num++;
	}

    /*申请 n+1个 pinctrl_map 内存*/
	new_map = kmalloc(sizeof(struct pinctrl_map) * map_num, GFP_KERNEL);


    /*二级指针，用于返回*/
	*map = new_map;
	*num_maps = map_num;


    /* 取parent，"iomuxc"节点 */
	parent = of_get_parent(np);
 
    /*
     * func  为 "imx6ul_evk"
     * group 为 "i2cgrp"
     */
	new_map[0].type = PIN_MAP_TYPE_MUX_GROUP;
	new_map[0].data.mux.function = parent->name;
	new_map[0].data.mux.group = np->name;
    

	/* 偏移下一个map */
	new_map++;
    
    /*设置 [1]~[grp->npins]个map */
	for (i = j = 0; i < grp->npins; i++) {
		if (!(grp->pins[i].config & IMX_NO_PAD_CTL)) {
            
			new_map[j].type = PIN_MAP_TYPE_CONFIGS_PIN;
            
            /*
             * 1. 保存在imx_pinctrl_soc_info的imx_pin_group
             *     里面的imx_pin的pin成员，指的是pin号
             * 2. 可根据pin号，在pinctrl_dev的pin_desc_tree中
             *    找到pin号对应的名字，并返回
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
	return 0;
}
```

### 1-2 pinctrl_select_state()

```C
/**
 * pinctrl_select_state() - select/activate/program a pinctrl state to HW
 * @p: the pinctrl handle for the device that requests configuration
 * @state: the state handle to select/activate/program
 */
int pinctrl_select_state(struct pinctrl *p, struct pinctrl_state *state)
{
	struct pinctrl_setting *setting, *setting2;
	struct pinctrl_state *old_state = p->state;
	int ret;

	if (p->state == state)
		return 0;

	if (p->state) {
		/*
		 * For each pinmux setting in the old state, forget SW's record
		 * of mux owner for that pingroup. Any pingroups which are
		 * still owned by the new state will be re-acquired by the call
		 * to pinmux_enable_setting() in the loop below.
		 */
		list_for_each_entry(setting, &p->state->settings, node) {
			if (setting->type != PIN_MAP_TYPE_MUX_GROUP)
				continue;
			pinmux_disable_setting(setting);
		}
	}

	p->state = NULL;

	/* Apply all the settings for the new state */
    /*根据新的state，来选择对应setting进行更新*/
	list_for_each_entry(setting, &state->settings, node) {
		switch (setting->type) {
		case PIN_MAP_TYPE_MUX_GROUP:
			ret = pinmux_enable_setting(setting);
			break;
		case PIN_MAP_TYPE_CONFIGS_PIN:
		case PIN_MAP_TYPE_CONFIGS_GROUP:
			ret = pinconf_apply_setting(setting);
			break;
		default:
			ret = -EINVAL;
			break;
		}

		if (ret < 0) {
			goto unapply_new_state;
		}
	}

	p->state = state;

	return 0;

unapply_new_state:
	dev_err(p->dev, "Error applying setting, reverse things back\n");

	list_for_each_entry(setting2, &state->settings, node) {
		if (&setting2->node == &setting->node)
			break;
		/*
		 * All we can do here is pinmux_disable_setting.
		 * That means that some pins are muxed differently now
		 * than they were before applying the setting (We can't
		 * "unmux a pin"!), but it's not a big deal since the pins
		 * are free to be muxed by another apply_setting.
		 */
		if (setting2->type == PIN_MAP_TYPE_MUX_GROUP)
			pinmux_disable_setting(setting2);
	}

	/* There's no infinite recursive loop here because p->state is NULL */
	if (old_state)
		pinctrl_select_state(p, old_state);

	return ret;
}
```

### 1-2-1 pinmux_enable_setting()

```C
int pinmux_enable_setting(struct pinctrl_setting const *setting)
{
	struct pinctrl_dev *pctldev = setting->pctldev;
	const struct pinctrl_ops *pctlops = pctldev->desc->pctlops;
	const struct pinmux_ops *ops = pctldev->desc->pmxops;
	int ret = 0;
	const unsigned *pins = NULL;
	unsigned num_pins = 0;
	int i;
	struct pin_desc *desc;

	if (pctlops->get_group_pins)
		ret = pctlops->get_group_pins(pctldev, setting->data.mux.group,
					      &pins, &num_pins);

	if (ret) {
		const char *gname;

		/* errors only affect debug data, so just warn */
		gname = pctlops->get_group_name(pctldev,
						setting->data.mux.group);
		dev_warn(pctldev->dev,
			 "could not get pins for group %s\n",
			 gname);
		num_pins = 0;
	}

	/* Try to allocate all pins in this group, one by one */
	for (i = 0; i < num_pins; i++) {
		ret = pin_request(pctldev, pins[i], setting->dev_name, NULL);
		if (ret) {
			const char *gname;
			const char *pname;

			desc = pin_desc_get(pctldev, pins[i]);
			pname = desc ? desc->name : "non-existing";
			gname = pctlops->get_group_name(pctldev,
						setting->data.mux.group);
			dev_err(pctldev->dev,
				"could not request pin %d (%s) from group %s "
				" on device %s\n",
				pins[i], pname, gname,
				pinctrl_dev_get_name(pctldev));
			goto err_pin_request;
		}
	}

	/* Now that we have acquired the pins, encode the mux setting */
	for (i = 0; i < num_pins; i++) {
		desc = pin_desc_get(pctldev, pins[i]);
		if (desc == NULL) {
			dev_warn(pctldev->dev,
				 "could not get pin desc for pin %d\n",
				 pins[i]);
			continue;
		}
		desc->mux_setting = &(setting->data.mux);
	}

	ret = ops->set_mux(pctldev, setting->data.mux.func,
			   setting->data.mux.group);

	if (ret)
		goto err_set_mux;

	return 0;

err_set_mux:
	for (i = 0; i < num_pins; i++) {
		desc = pin_desc_get(pctldev, pins[i]);
		if (desc)
			desc->mux_setting = NULL;
	}
err_pin_request:
	/* On error release all taken pins */
	while (--i >= 0)
		pin_free(pctldev, pins[i], NULL);

	return ret;
}
```

### 1-2-2 pinconf_apply_setting()

```C
int pinconf_apply_setting(struct pinctrl_setting const *setting)
{
	struct pinctrl_dev *pctldev = setting->pctldev;
	const struct pinconf_ops *ops = pctldev->desc->confops;
	int ret;

	if (!ops) {
		dev_err(pctldev->dev, "missing confops\n");
		return -EINVAL;
	}

	switch (setting->type) {
	case PIN_MAP_TYPE_CONFIGS_PIN:
		if (!ops->pin_config_set) {
			dev_err(pctldev->dev, "missing pin_config_set op\n");
			return -EINVAL;
		}
		ret = ops->pin_config_set(pctldev,
				setting->data.configs.group_or_pin,
				setting->data.configs.configs,
				setting->data.configs.num_configs);
		if (ret < 0) {
			dev_err(pctldev->dev,
				"pin_config_set op failed for pin %d\n",
				setting->data.configs.group_or_pin);
			return ret;
		}
		break;
	case PIN_MAP_TYPE_CONFIGS_GROUP:
		if (!ops->pin_config_group_set) {
			dev_err(pctldev->dev,
				"missing pin_config_group_set op\n");
			return -EINVAL;
		}
		ret = ops->pin_config_group_set(pctldev,
				setting->data.configs.group_or_pin,
				setting->data.configs.configs,
				setting->data.configs.num_configs);
		if (ret < 0) {
			dev_err(pctldev->dev,
				"pin_config_group_set op failed for group %d\n",
				setting->data.configs.group_or_pin);
			return ret;
		}
		break;
	default:
		return -EINVAL;
	}

	return 0;
}
```

### 1-2-1-1 imx_pmx_set()

```C
static int imx_pmx_set(struct pinctrl_dev *pctldev, unsigned selector,
		       unsigned group)
{
	struct imx_pinctrl *ipctl = pinctrl_dev_get_drvdata(pctldev);
	const struct imx_pinctrl_soc_info *info = ipctl->info;
	const struct imx_pin_reg *pin_reg;
	unsigned int npins, pin_id;
	int i;
	struct imx_pin_group *grp;

	/*
	 * Configure the mux mode for each pin in the group for a specific
	 * function.
	 */
	grp = &info->groups[group];
	npins = grp->npins;

	dev_dbg(ipctl->dev, "enable function %s group %s\n",
		info->functions[selector].name, grp->name);

	for (i = 0; i < npins; i++) {
		struct imx_pin *pin = &grp->pins[i];
		pin_id = pin->pin;
		pin_reg = &info->pin_regs[pin_id];

		if (pin_reg->mux_reg == -1) {
			dev_dbg(ipctl->dev, "Pin(%s) does not support mux function\n",
				info->pins[pin_id].name);
			continue;
		}

		if (info->flags & SHARE_MUX_CONF_REG) {
			u32 reg;
			reg = readl(ipctl->base + pin_reg->mux_reg);
			reg &= ~(0x7 << 20);
			reg |= (pin->mux_mode << 20);
			writel(reg, ipctl->base + pin_reg->mux_reg);
		} else {
			writel(pin->mux_mode, ipctl->base + pin_reg->mux_reg);
		}
		dev_dbg(ipctl->dev, "write: offset 0x%x val 0x%x\n",
			pin_reg->mux_reg, pin->mux_mode);

		/*
		 * If the select input value begins with 0xff, it's a quirky
		 * select input and the value should be interpreted as below.
		 *     31     23      15      7        0
		 *     | 0xff | shift | width | select |
		 * It's used to work around the problem that the select
		 * input for some pin is not implemented in the select
		 * input register but in some general purpose register.
		 * We encode the select input value, width and shift of
		 * the bit field into input_val cell of pin function ID
		 * in device tree, and then decode them here for setting
		 * up the select input bits in general purpose register.
		 */
		if (pin->input_val >> 24 == 0xff) {
			u32 val = pin->input_val;
			u8 select = val & 0xff;
			u8 width = (val >> 8) & 0xff;
			u8 shift = (val >> 16) & 0xff;
			u32 mask = ((1 << width) - 1) << shift;
			/*
			 * The input_reg[i] here is actually some IOMUXC general
			 * purpose register, not regular select input register.
			 */
			val = readl(ipctl->base + pin->input_reg);
			val &= ~mask;
			val |= select << shift;
			writel(val, ipctl->base + pin->input_reg);
		} else if (pin->input_reg) {
			/*
			 * Regular select input register can never be at offset
			 * 0, and we only print register value for regular case.
			 */
			if (ipctl->input_sel_base)
				writel(pin->input_val, ipctl->input_sel_base +
						pin->input_reg);
			else
				writel(pin->input_val, ipctl->base +
						pin->input_reg);
			dev_dbg(ipctl->dev,
				"==>select_input: offset 0x%x val 0x%x\n",
				pin->input_reg, pin->input_val);
		}
	}

	return 0;
}
```

### 1-2-2-1 imx_pinconf_set()

```C
static int imx_pinconf_set(struct pinctrl_dev *pctldev,
			     unsigned pin_id, unsigned long *configs,
			     unsigned num_configs)
{
	struct imx_pinctrl *ipctl = pinctrl_dev_get_drvdata(pctldev);
	const struct imx_pinctrl_soc_info *info = ipctl->info;
	const struct imx_pin_reg *pin_reg = &info->pin_regs[pin_id];
	int i;

	if (pin_reg->conf_reg == -1) {
		dev_err(info->dev, "Pin(%s) does not support config function\n",
			info->pins[pin_id].name);
		return -EINVAL;
	}

	dev_dbg(ipctl->dev, "pinconf set pin %s\n",
		info->pins[pin_id].name);

	for (i = 0; i < num_configs; i++) {
		if (info->flags & SHARE_MUX_CONF_REG) {
			u32 reg;
			reg = readl(ipctl->base + pin_reg->conf_reg);
			reg &= ~0xffff;
			reg |= configs[i];
			writel(reg, ipctl->base + pin_reg->conf_reg);
		} else {
			writel(configs[i], ipctl->base + pin_reg->conf_reg);
		}
		dev_dbg(ipctl->dev, "write: offset 0x%x val 0x%lx\n",
			pin_reg->conf_reg, configs[i]);
	} /* for each config */

	return 0;
}
```



### 总结

```c
&i2c1 {
    clock-frequency = <100000>;
    pinctrl-names = "default";
    pinctrl-0 = <&pinctrl_i2c1>;
    status = "okay";
};

pinctrl_i2c1: i2c1grp {
    fsl,pins = <
         MX6UL_PAD_UART4_TX_DATA__I2C1_SCL 0x4001b8b0
         MX6UL_PAD_UART4_RX_DATA__I2C1_SDA 0x4001b8b0
     >;
};
```

- 为了方便理解，使用上述DTS的i2c client进行说明。

- 当driver与device已成功match，调用probe初始化之前，会先调用really_probe()，对设备进行相应的处理，其中包含了pinctrl子系统的处理 。
- 第一步，申请pinctrl，并根据DTS，寻找" pinctrl-* "，遍历每个state。当前只有一个state，为"default"。
- 第二步，根据当前state = "default"，寻找" pinctrl-0 " 内容，遍历每个config(指的是phandle)，当前只有一个config， 为 <&pinctrl_i2c1>。
- 第三步，根据当前config，找到属性名为"fsl,pins"，读取父节点("iomuxc")的pinctrl_dev，使用pinctrl_dev->fops，找到已经构造完毕的soc_info->groups，根据节点名字，获取其groups和pins，利用该信息，申请对应的map，设置map，当前map为2个，申请2+1个（有1个用于function）
  - " MX6UL_PAD_UART4_TX_DATA__I2C1_SCL   0x4001b8b0 "
  -  " MX6UL_PAD_UART4_RX_DATA__I2C1_SDA  0x4001b8b0 "

- 第四步，此时先申请state，并挂入pinctrl->states的list中，申请多个setting，将所有map转换成setting，保存在state->settings的list中。

- 可看申请内存的过程，pinctrl -> pinctrl_map -> state -> settings。

