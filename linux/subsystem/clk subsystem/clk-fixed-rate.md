### 1 init

```C
static const struct of_device_id of_fixed_clk_ids[] = {
	{ .compatible = "fixed-clock" },
	{ }
};

static struct platform_driver of_fixed_clk_driver = {
	.driver = {
		.name = "of_fixed_clk",
		.of_match_table = of_fixed_clk_ids,
	},
	.probe = of_fixed_clk_probe,
	.remove = of_fixed_clk_remove,
};

/* 这个意思可能是driver必须被包含进kernel */
builtin_platform_driver(of_fixed_clk_driver);
```

### 2 dts

```c
/*在imx6ull.dtsi中*/
osc: clock@1 {
	compatible = "fixed-clock";
	reg = <1>;
	#clock-cells = <0>;
	clock-frequency = <24000000>;
	clock-output-names = "osc";
};
```

###  3 of_fixed_clk_probe()

```C
static int of_fixed_clk_probe(struct platform_device *pdev)
{
	struct clk *clk;

	/*
	 * This function is not executed when of_fixed_clk_setup
	 * succeeded.
	 */
	clk = _of_fixed_clk_setup(pdev->dev.of_node);

	platform_set_drvdata(pdev, clk);

	return 0;
}

```

#### 3-1 _of_fixed_clk_setup()

```C
static struct clk *_of_fixed_clk_setup(struct device_node *node)
{
	struct clk *clk;
	const char *clk_name = node->name;
	u32 rate;
	u32 accuracy = 0;
	int ret;
	
    /* 读出rate,为24Mhz */
	if (of_property_read_u32(node, "clock-frequency", &rate))
		return ERR_PTR(-EIO);

	of_property_read_u32(node, "clock-accuracy", &accuracy);
	
    /* 读出name,为"osc" */
	of_property_read_string(node, "clock-output-names", &clk_name);

    /* 
     * 1.申请设置注册 clk_hw
     * 2.申请设置注册 clk_core
     * 3.申请设置 clk
     * 4.三者建立联系,并返回clk
     */
	clk = clk_register_fixed_rate_with_accuracy(NULL, clk_name, NULL,
						    0, rate, accuracy);

	/* 根据 clk_hw 注册 provider */
	ret = of_clk_add_provider(node, of_clk_src_simple_get, clk);

	return clk;
}
```

#### 3-1-1 clk_register_fixed_rate_with_accuracy()

```C
clk_register_fixed_rate_with_accuracy(); 
--> clk_hw_register_fixed_rate_with_accuracy();


struct clk_hw *clk_hw_register_fixed_rate_with_accuracy(struct device *dev,
		const char *name, const char *parent_name, unsigned long flags,
		unsigned long fixed_rate, unsigned long fixed_accuracy)
{
	struct clk_fixed_rate *fixed;
	struct clk_hw *hw;
	struct clk_init_data init;
	int ret;

	/* 申请 clk_fixed_rate */
	fixed = kzalloc(sizeof(*fixed), GFP_KERNEL);

	/* 
	 * 设置 init，使用了局部变量，
	 * 因调用完clk_hw_register后，init就没用了 
	 */
	init.name = name;//"osc"
	init.ops = &clk_fixed_rate_ops;
	init.flags = flags | CLK_IS_BASIC;//父clk
	init.parent_names = (parent_name ? &parent_name: NULL);//没有parent
	init.num_parents = (parent_name ? 1 : 0);//为0

	fixed->fixed_rate = fixed_rate; //为24M
	fixed->fixed_accuracy = fixed_accuracy; //为0
	fixed->hw.init = &init;//指向刚初始化的init

	/* 注册clk_hw */
	hw = &fixed->hw;
	ret = clk_hw_register(dev, hw);

	return hw;
}						    
```

#### 3-1-1-1 clk_hw_register()

```C
clk_hw_register() --> clk_register()


struct clk *clk_register(struct device *dev, struct clk_hw *hw)
{
	int i, ret;
	struct clk_core *core;

    /* 申请 clk_core */
	core = kzalloc(sizeof(*core), GFP_KERNEL);

	/* 设置 clk_core */
	core->name = kstrdup_const(hw->init->name, GFP_KERNEL);//"osc"
	core->ops = hw->init->ops;
	if (dev && dev->driver)
		core->owner = dev->driver->owner;
	core->hw = hw;
	core->flags = hw->init->flags;
	core->num_parents = hw->init->num_parents;//0
	core->min_rate = 0;
	core->max_rate = ULONG_MAX;
	hw->core = core;

	/* 负值,跳过*/
	core->parent_names = kcalloc(core->num_parents, sizeof(char *),
					GFP_KERNEL);
	/* 负值，这里不会返回*/
	if (!core->parent_names) {
		ret = -ENOMEM;
		goto fail_parent_names;
	}


	/* 为0,不执行 */
	for (i = 0; i < core->num_parents; i++) {
		core->parent_names[i] = kstrdup_const(hw->init->parent_names[i],
						GFP_KERNEL);
		if (!core->parent_names[i]) {
			ret = -ENOMEM;
			goto fail_parent_names_copy;
		}
	}

	/* 负值,跳过 */
	core->parents = kcalloc(core->num_parents, sizeof(*core->parents),
				GFP_KERNEL);
	if (!core->parents) {
		ret = -ENOMEM;
		goto fail_parents;
	};

	INIT_HLIST_HEAD(&core->clks);
	
    /* 申请设置 clk */
	hw->clk = __clk_create_clk(hw, NULL, NULL);

    /* 往上注册 clk_core */
	ret = __clk_core_init(core);
	if (!ret)
		return hw->clk;

	__clk_free_clk(hw->clk);
	hw->clk = NULL;

}
```

#### 3-1-1-1-1 __clk_create_clk()

```C
struct clk *__clk_create_clk(struct clk_hw *hw, const char *dev_id,
			     const char *con_id)
{
	struct clk *clk;

	/* 申请 clk */
	clk = kzalloc(sizeof(*clk), GFP_KERNEL);

	clk->core = hw->core;
	clk->dev_id = dev_id;//NULL
	clk->con_id = con_id;//NULL
	clk->max_rate = ULONG_MAX;

    /*clk 插入core->list */
	hlist_add_head(&clk->clks_node, &hw->core->clks);

	return clk;
}

```

#### 3-1-2 of_clk_add_provider()

```C
int of_clk_add_provider(struct device_node *np,
			struct clk *(*clk_src_get)(struct of_phandle_args *clkspec,
						   void *data),
			void *data)
{
	struct of_clk_provider *cp;
	int ret;

    /* 申请 of_clk_provider */
	cp = kzalloc(sizeof(struct of_clk_provider), GFP_KERNEL);

	cp->node = of_node_get(np); // "osc node"
	cp->data = data;// clk
	cp->get = clk_src_get; // of_clk_src_simple_get

    /* of_clk_provider 挂入全局的list */
	list_add(&cp->link, &of_clk_providers);
	

	/* 有关 'assigned-{clocks/clock-parents/clock-rates}' 属性处理 */
	ret = of_clk_set_defaults(np, true);

	return ret;
}
```

- 最终成功添加了"osc"的clk_provider。
- 从设备树可以知道，添加了4个clk_provider。

