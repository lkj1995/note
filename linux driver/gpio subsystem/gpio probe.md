### dts描述

```c
gpio1: gpio@0209c000 {
	compatible = "fsl,imx6ul-gpio", "fsl,imx35-gpio";
	reg = <0x0209c000 0x4000>;
	interrupts = <GIC_SPI 66 IRQ_TYPE_LEVEL_HIGH>,
				<GIC_SPI 67 IRQ_TYPE_LEVEL_HIGH>;
	gpio-controller;
	#gpio-cells = <2>;
	interrupt-controller;
	#interrupt-cells = <2>;
};
/*

| 66 | gpio1 |  Combined interrupt indication for GPIO1 signal 0 throughout|
| 67 | gpio1 |  Combined interrupt indication for GPIO1 signal 16 throughout|

两个中断号属于gpio1

*/
```

### gpio_mxc_init()
```C
static const struct platform_device_id mxc_gpio_devtype[] = {
	{
		.name = "imx1-gpio",
		.driver_data = IMX1_GPIO,
	}, {
		.name = "imx21-gpio",
		.driver_data = IMX21_GPIO,
	}, {
		.name = "imx31-gpio",
		.driver_data = IMX31_GPIO,
	}, {
		.name = "imx35-gpio",
		.driver_data = IMX35_GPIO,
	},
};

static const struct of_device_id mxc_gpio_dt_ids[] = {
    
	{ .compatible = "fsl,imx1-gpio", .data = &mxc_gpio_devtype[IMX1_GPIO], },
    
	{ .compatible = "fsl,imx21-gpio", .data = &mxc_gpio_devtype[IMX21_GPIO], },
    
	{ .compatible = "fsl,imx31-gpio", .data = &mxc_gpio_devtype[IMX31_GPIO], }, 
    
    /*在dts存在此属性，因此与dev匹配，作为gpio-controler*/
	{ .compatible = "fsl,imx35-gpio", .data = &mxc_gpio_devtype[IMX35_GPIO], },
	{ /* sentinel */ }
};

static struct platform_driver mxc_gpio_driver = {
	.driver		= {
		.name	= "gpio-mxc",
		.of_match_table = mxc_gpio_dt_ids,
	},
	.probe		= mxc_gpio_probe,
	.id_table	= mxc_gpio_devtype,
};

static int __init gpio_mxc_init(void)
{
    /* 申请名为 "gpio-mxc" 的 platform driver */
	return platform_driver_register(&mxc_gpio_driver); 
}
subsys_initcall(gpio_mxc_init);
```

### struct mxc_gpio_port

```C
struct mxc_gpio_port {
	struct list_head node;
	void __iomem *base;
	int irq;
	int irq_high;
	struct irq_domain *domain;
	struct gpio_chip gc;
	u32 both_edges;
};
```

### struct gpio_chip

```C
struct gpio_chip {
	const char		*label;
	struct gpio_device	*gpiodev;
	struct device		*parent;
	struct module		*owner;

	int			(*request)(struct gpio_chip *chip,
						unsigned offset);
	void			(*free)(struct gpio_chip *chip,
						unsigned offset);
	int			(*get_direction)(struct gpio_chip *chip,
						unsigned offset);
	int			(*direction_input)(struct gpio_chip *chip,
						unsigned offset);
	int			(*direction_output)(struct gpio_chip *chip,
						unsigned offset, int value);
	int			(*get)(struct gpio_chip *chip,
						unsigned offset);
	void			(*set)(struct gpio_chip *chip,
						unsigned offset, int value);
	void			(*set_multiple)(struct gpio_chip *chip,
						unsigned long *mask,
						unsigned long *bits);
	int			(*set_debounce)(struct gpio_chip *chip,
						unsigned offset,
						unsigned debounce);
	int			(*set_single_ended)(struct gpio_chip *chip,
						unsigned offset,
						enum single_ended_mode mode);

	int			(*to_irq)(struct gpio_chip *chip,
						unsigned offset);

	void			(*dbg_show)(struct seq_file *s,
						struct gpio_chip *chip);
	int			base;
	u16			ngpio;
	const char		*const *names;
	bool			can_sleep;
	bool			irq_not_threaded;

#if IS_ENABLED(CONFIG_GPIO_GENERIC)
	unsigned long (*read_reg)(void __iomem *reg);
	void (*write_reg)(void __iomem *reg, unsigned long data);
	unsigned long (*pin2mask)(struct gpio_chip *gc, unsigned int pin);
	void __iomem *reg_dat;
	void __iomem *reg_set;
	void __iomem *reg_clr;
	void __iomem *reg_dir;
	int bgpio_bits;
	spinlock_t bgpio_lock;
	unsigned long bgpio_data;
	unsigned long bgpio_dir;
#endif

#ifdef CONFIG_GPIOLIB_IRQCHIP
	/*
	 * With CONFIG_GPIOLIB_IRQCHIP we get an irqchip inside the gpiolib
	 * to handle IRQs for most practical cases.
	 */
	struct irq_chip		*irqchip;
	struct irq_domain	*irqdomain;
	unsigned int		irq_base;
	irq_flow_handler_t	irq_handler;
	unsigned int		irq_default_type;
	int			irq_parent;
	bool			irq_need_valid_mask;
	unsigned long		*irq_valid_mask;
	struct lock_class_key	*lock_key;
#endif

#if defined(CONFIG_OF_GPIO)
	/*
	 * If CONFIG_OF is enabled, then all GPIO controllers described in the
	 * device tree automatically may have an OF translation
	 */
	struct device_node *of_node;
	int of_gpio_n_cells;
	int (*of_xlate)(struct gpio_chip *gc,
			const struct of_phandle_args *gpiospec, u32 *flags);
#endif
};
```

### struct gpio_desc

```C
struct gpio_desc {
	struct gpio_device	*gdev;
	unsigned long		flags;
/* flag symbols are bit numbers */
#define FLAG_REQUESTED	0
#define FLAG_IS_OUT	1
#define FLAG_EXPORT	2	/* protected by sysfs_lock */
#define FLAG_SYSFS	3	/* exported via /sys/class/gpio/control */
#define FLAG_ACTIVE_LOW	6	/* value has active low */
#define FLAG_OPEN_DRAIN	7	/* Gpio is open drain type */
#define FLAG_OPEN_SOURCE 8	/* Gpio is open source type */
#define FLAG_USED_AS_IRQ 9	/* GPIO is connected to an IRQ */
#define FLAG_IS_HOGGED	11	/* GPIO is hogged */

	/* Connection label */
	const char		*label;
	/* Name of the GPIO */
	const char		*name;
};
```

### gpio reg

```C
#define GPIO_DR			(mxc_gpio_hwdata->dr_reg)
#define GPIO_GDIR		(mxc_gpio_hwdata->gdir_reg)
#define GPIO_PSR		(mxc_gpio_hwdata->psr_reg)
#define GPIO_ICR1		(mxc_gpio_hwdata->icr1_reg)
#define GPIO_ICR2		(mxc_gpio_hwdata->icr2_reg)
#define GPIO_IMR		(mxc_gpio_hwdata->imr_reg)
#define GPIO_ISR		(mxc_gpio_hwdata->isr_reg)
#define GPIO_EDGE_SEL		(mxc_gpio_hwdata->edge_sel_reg)

#define GPIO_INT_LOW_LEV	(mxc_gpio_hwdata->low_level)
#define GPIO_INT_HIGH_LEV	(mxc_gpio_hwdata->high_level)
#define GPIO_INT_RISE_EDGE	(mxc_gpio_hwdata->rise_edge)
#define GPIO_INT_FALL_EDGE	(mxc_gpio_hwdata->fall_edge)
#define GPIO_INT_BOTH_EDGES	0x4
```



### struct mxc_gpio_hwdata

```C
static struct mxc_gpio_hwdata imx35_gpio_hwdata = {
	.dr_reg		= 0x00,
	.gdir_reg	= 0x04,
	.psr_reg	= 0x08,
	.icr1_reg	= 0x0c,
	.icr2_reg	= 0x10,
	.imr_reg	= 0x14,
	.isr_reg	= 0x18,
	.edge_sel_reg	= 0x1c,
	.low_level	= 0x00,
	.high_level	= 0x01,
	.rise_edge	= 0x02,
	.fall_edge	= 0x03,
};

/* device type dependent stuff */
struct mxc_gpio_hwdata {
	unsigned dr_reg;
	unsigned gdir_reg;
	unsigned psr_reg;
	unsigned icr1_reg;
	unsigned icr2_reg;
	unsigned imr_reg;
	unsigned isr_reg;
	int edge_sel_reg;
	unsigned low_level;
	unsigned high_level;
	unsigned rise_edge;
	unsigned fall_edge;
};
```

### enum mxc_gpio_hwtype

```C
enum mxc_gpio_hwtype {
	IMX1_GPIO,	/* runs on i.mx1 */
	IMX21_GPIO,	/* runs on i.mx21 and i.mx27 */
	IMX31_GPIO,	/* runs on i.mx31 */
	IMX35_GPIO,	/* runs on all other i.mx */
};
```

### struct gpio_device

```C
struct gpio_device {
	int			id;
	struct device		dev;
	struct cdev		chrdev;
	struct device		*mockdev;
	struct module		*owner;
	struct gpio_chip	*chip;
	struct gpio_desc	*descs;
	int			base;
	u16			ngpio;
	char			*label;
	void			*data;
	struct list_head        list;

#ifdef CONFIG_PINCTRL
	/*
	 * If CONFIG_PINCTRL is enabled, then gpio controllers can optionally
	 * describe the actual pin range which they serve in an SoC. This
	 * information would be used by pinctrl subsystem to configure
	 * corresponding pins for gpio usage.
	 */
	struct list_head pin_ranges;
#endif
};
```

### 1 mxc_gpio_probe()

```C
static int mxc_gpio_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct mxc_gpio_port *port;
	struct resource *iores;
	int irq_base;
	int err;

    /*确认soc型号,根据型号获取gpio相关寄存器的地址 */
	mxc_gpio_get_hw(pdev);
	
    /*申请 mxc_gpio_port */
	port = devm_kzalloc(&pdev->dev, sizeof(*port), GFP_KERNEL);

    /* 
     * 1. 获取"reg"属性内容，gpio寄存器起始地址，数量，    
     * 2. 将其转换为虚拟地址，用于后续访问。
     */
	iores = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	port->base = devm_ioremap_resource(&pdev->dev, iores);
    
	/* 
	 * 获取两个中断号,在imx6ull的gpio外设中，
	 * goiox_0~goiox_15为第一组中断，
	 * goiox_16~goiox_32为第二组中断。
	 */
	port->irq_high = platform_get_irq(pdev, 1);
	port->irq = platform_get_irq(pdev, 0);

    /* 获取时钟值 可选项 */
    port->clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(port->clk))
		port->clk = NULL;
    
	/*
	 * 1. 设置IMR寄存器，禁止中断
	 * 2. 设置ISR寄存器，清空状态
	 */
	writel(0, port->base + GPIO_IMR);
	writel(~0, port->base + GPIO_ISR);

    /*此型号每个gpio外设只有一个irq*/
	if (mxc_gpio_hwtype == IMX21_GPIO) { 
		irq_set_chained_handler(port->irq, mx2_gpio_irq_handler);
	} else {
        
        /* 注册gpiox的2个中断号 */       
		/* setup one handler for each entry */
		irq_set_chained_handler_and_data(port->irq,
						 mx3_gpio_irq_handler, port);
        
		if (port->irq_high > 0)
			/* setup handler for GPIO 16 to 31 */
			irq_set_chained_handler_and_data(port->irq_high,
							 mx3_gpio_irq_handler,
							 port);
        
	}

    /*
     * 根据soc的信息，设置gpio_chip
     * 1. 设置gpio的寄存器地址
     */
	err = bgpio_init(&port->gc, &pdev->dev, 4,
			 port->base + GPIO_PSR,
			 port->base + GPIO_DR, NULL,
			 port->base + GPIO_GDIR, NULL,
			 BGPIOF_READ_OUTPUT_REG_SET);

    /* 获取"gpio-ranges"属性 */
	if (of_property_read_bool(np, "gpio-ranges")) {
		port->gc.request = gpiochip_generic_request;
		port->gc.free = gpiochip_generic_free;
	}

    /* 2. 设置中间层操作函数，如何使用gpio寄存器 */
	port->gc.request = mxc_gpio_request;
	port->gc.free = mxc_gpio_free;
	port->gc.parent = &pdev->dev;    
	port->gc.to_irq = mxc_gpio_to_irq;
    /* 
     * 获取别名的id,每个gpio有32个引脚号，如gpio1_2，base为33 
     * 如果设置成-1，在这里自动分配
     */
	port->gc.base = (pdev->id < 0) ? of_alias_get_id(np, "gpio") * 32 :
					     pdev->id * 32;

    
	err = devm_gpiochip_add_data(&pdev->dev, &port->gc, port);


	irq_base = irq_alloc_descs(-1, 0, 32, numa_node_id());
	if (irq_base < 0) {
		err = irq_base;
		goto out_bgio;
	}

	port->domain = irq_domain_add_legacy(np, 32, irq_base, 0,
					     &irq_domain_simple_ops, NULL);
	if (!port->domain) {
		err = -ENODEV;
		goto out_irqdesc_free;
	}

	/* gpio-mxc can be a generic irq chip */
	err = mxc_gpio_init_gc(port, irq_base);
	if (err < 0)
		goto out_irqdomain_remove;

	list_add_tail(&port->node, &mxc_gpio_ports);

	return 0;

out_irqdomain_remove:
	irq_domain_remove(port->domain);
out_irqdesc_free:
	irq_free_descs(irq_base, 32);
out_bgio:
	dev_info(&pdev->dev, "%s failed with errno %d\n", __func__, err);
	return err;
}
```

### 1-1 bgpio_init()

```C
/*
 * bgpio_init(&port->gc, &pdev->dev, 4, port->base + GPIO_PSR, 
 *	     port->base + GPIO_DR, NULL, port->base + GPIO_GDIR, NULL,
 *	           BGPIOF_READ_OUTPUT_REG_SET); 
 * 虚拟基址 + 寄存器偏移值 = 某个寄存器的具体虚拟地址
*/
int bgpio_init(struct gpio_chip *gc, struct device *dev,
	       unsigned long sz, void __iomem *dat, void __iomem *set,
	       void __iomem *clr, void __iomem *dirout, void __iomem *dirin,
	       unsigned long flags)
{
	int ret;

	if (!is_power_of_2(sz))
		return -EINVAL;
	
    /* gpio寄存器的宽度 4 * 8 = 32 */
	gc->bgpio_bits = sz * 8;
	if (gc->bgpio_bits > BITS_PER_LONG)
		return -EINVAL;

	spin_lock_init(&gc->bgpio_lock);
	gc->parent = dev;
	gc->label = dev_name(dev); /*使用device的名字*/
	gc->base = -1;
	gc->ngpio = gc->bgpio_bits; /*同样也是32，代表每个gpio控制器的pin数量*/
	gc->request = bgpio_request; /*函数，判断传入的pin号是否大于gc->ngpio的范围 */

    /*记录了如何读取gpio值，和设置gpio值*/
	ret = bgpio_setup_io(gc, dat, set, clr, flags);
	if (ret)
		return ret;

    /*，提前准备确认bit位数，并记录对应bit位数的读写函数，bgpio_read32 bgpio_write32*/
	ret = bgpio_setup_accessors(dev, gc, flags & BGPIOF_BIG_ENDIAN,
				    flags & BGPIOF_BIG_ENDIAN_BYTE_ORDER);
	if (ret)
		return ret;

    /*记录了如何设置gpio方向*/
	ret = bgpio_setup_direction(gc, dirout, dirin, flags);
	if (ret)
		return ret;

	gc->bgpio_data = gc->read_reg(gc->reg_dat);
	if (gc->set == bgpio_set_set &&
			!(flags & BGPIOF_UNREADABLE_REG_SET))
        /*保存当前gpio值，该成员用于保护，驱动间接获取*/
		gc->bgpio_data = gc->read_reg(gc->reg_set);
	if (gc->reg_dir && !(flags & BGPIOF_UNREADABLE_REG_DIR))
        /*保存当前gpio方向，该成员用于保护，驱动间接获取*/
		gc->bgpio_dir = gc->read_reg(gc->reg_dir);

	return ret;
}
```

### 1-2 devm_gpiochip_add_data()

```C
/* 传入参数 (&pdev->dev, &port->gc, mxc_gpio_port); */
int devm_gpiochip_add_data(struct device *dev, struct gpio_chip *chip,
			   void *data)
{    
	int ret;

	ret = gpiochip_add_data(chip, data);

	return 0;
}



int gpiochip_add_data(struct gpio_chip *chip, void *data)
{
	unsigned long	flags;
	int		status = 0;
	unsigned	i;
	int		base = chip->base;
	struct gpio_device *gdev;

	/******** 申请 gpio_device ********/
	gdev = kzalloc(sizeof(*gdev), GFP_KERNEL);

    
    
    /****** 设置 gpio_device ******/
	gdev->dev.bus = &gpio_bus_type;
	gdev->chip = chip;
	chip->gpiodev = gdev;
	if (chip->parent) {
		gdev->dev.parent = chip->parent;
		gdev->dev.of_node = chip->parent->of_node;
	}

#ifdef CONFIG_OF_GPIO
	/* If the gpiochip has an assigned OF node this takes precedence */
	if (chip->of_node)
		gdev->dev.of_node = chip->of_node;
#endif

	gdev->id = ida_simple_get(&gpio_ida, 0, 0, GFP_KERNEL);
   
    /*设置 gpio_device->dev 的名字 */
	dev_set_name(&gdev->dev, "gpiochip%d", gdev->id);
    
    /*初始化device*/
	device_initialize(&gdev->dev);
	dev_set_drvdata(&gdev->dev, gdev);
	if (chip->parent && chip->parent->driver)
		gdev->owner = chip->parent->driver->owner;
	else if (chip->owner)
		/* TODO: remove chip->owner */
		gdev->owner = chip->owner;
	else
		gdev->owner = THIS_MODULE;
    
	/****** 设置 gpio_device 完成 ******/
    

    
    
    
    /****** 根据gpio的引脚数量，申请 n个 gpio_desc *******/
	gdev->descs = kcalloc(chip->ngpio, sizeof(gdev->descs[0]), GFP_KERNEL);

	/* 记录每个gpio引脚数量 */
	gdev->ngpio = chip->ngpio;
	gdev->data = data;
	gdev->base = base;
        
	/* 
	 * 根据 [base, base + ngpio - 1] 排序，
     * 将gpio_deivce 插入 global chips list。
	 */
	status = gpiodev_add_to_list(gdev);


    /******* 设置 n个 gpio_desc  *******/
	for (i = 0; i < chip->ngpio; i++) {
        
		struct gpio_desc *desc = &gdev->descs[i];		
		desc->gdev = gdev;
        
		/*
		 * 一般来说，芯片会把引脚复位为上拉输入，保证最低功耗。
		 * linux处理gpio第一步，就是要先设置引脚方向，如不这样做的话，
		 * 会把错误的方向反馈给sysfs，此时用户cat出来的值是不正确的。
		 */
		if (chip->get_direction) {
            /*如为null，说明还没设置*/
			int dir = chip->get_direction(chip, i);

			if (!dir)
                /*记录引脚方向为输出，应该是把FLAG_IS_OUT保存在desc->flags*/
				set_bit(FLAG_IS_OUT, &desc->flags);
		} else if (!chip->direction_input) {
			set_bit(FLAG_IS_OUT, &desc->flags);
		}
	}

#ifdef CONFIG_PINCTRL
	INIT_LIST_HEAD(&gdev->pin_ranges);/*使用pinctrl*/
#endif

    /* 为每个gpio_desc分配名字 */
	status = gpiochip_set_desc_names(chip);
	status = gpiochip_irqchip_init_valid_mask(chip);

    
	status = of_gpiochip_add(chip);


	/*
	 * By first adding the chardev, and then adding the device,
	 * we get a device node entry in sysfs under
	 * /sys/bus/gpio/devices/gpiochipN/dev that can be used for
	 * coldplug of device nodes and other udev business.
	 * We can do this only if gpiolib has been initialized.
	 * Otherwise, defer until later.
	 */
	if (gpiolib_initialized) {
		status = gpiochip_setup_dev(gdev);
		if (status)
			goto err_remove_chip;
	}
	return 0;
}
```

### 1-2-1 of_gpiochip_add()

```C
int of_gpiochip_add(struct gpio_chip *chip)
{
	int status;

    /* 设置xlate函数 */
	if (!chip->of_xlate) { 
        /* 2个cells 描述gpio引脚 */
		chip->of_gpio_n_cells = 2;
		chip->of_xlate = of_gpio_simple_xlate;
	}

    /* 并没用到 gpio-ranges,往下不分析 */
	status = of_gpiochip_add_pin_range(chip);
	if (status)
		return status;

	/* If the chip defines names itself, these take precedence */
	if (!chip->names)
		of_gpiochip_set_names(chip);

	of_node_get(chip->of_node);

	return of_gpiochip_scan_gpios(chip);
}
```

### 2 of_gpiochip_add_pin_range()

```C
/*
 * 由此得出一个结论，pinctrl子系统中，存在多个pinctrl_dev。
 * 在 imx6ul-evk 节点里，多少个子节点就多少个pinctrl_dev，如下就有两个。
 * pinctrl_flexcan1: flexcan1grp{
 *     fsl,pins = <
 *              MX6UL_PAD_UART3_CTS_B__FLEXCAN1_TX         0x000010B0
 *              MX6UL_PAD_UART3_RTS_B__FLEXCAN1_RX         0x000010B0
 *      >;
 * };
 * 
 * pinctrl_i2c1: i2c1grp {
 *     fsl,pins = <
 *              MX6UL_PAD_UART4_TX_DATA__I2C1_SCL 0x4001b8b0
 *              MX6UL_PAD_UART4_RX_DATA__I2C1_SDA 0x4001b8b0
 *     >;
 * }; 
 */

static int of_gpiochip_add_pin_range(struct gpio_chip *chip)
{
	struct device_node *np = chip->of_node;
	struct of_phandle_args pinspec;
	struct pinctrl_dev *pctldev;
	int index = 0, ret;
	const char *name;
	static const char group_names_propname[] = "gpio-ranges-group-names";
	struct property *group_names;

	if (!np)
		return 0;

    /*找不到该属性名*/
	group_names = of_find_property(np, group_names_propname, NULL);

	for (;; index++) {
        
        /*
         * 寻找当前node的"gpio-ranges"属性
         * 并把参数保存在pinspec，其中参数为3个
         */
		ret = of_parse_phandle_with_fixed_args(np, "gpio-ranges", 3,
				index, &pinspec);
		if (ret)
			break;
        
		/* 
		 * 根据gpio-ranges的第1个参数 pinctrlA
		 * gpio-ranges = <&pinctrlA 0 128 12>;
		 * 在全局list中，pctldev->dev->of_node == pinspec.np
		 * 找到匹配的pinctrl_dev。
		 */
		pctldev = of_pinctrl_get(pinspec.np);
		of_node_put(pinspec.np);
		if (!pctldev)
			return -EPROBE_DEFER;

		if (pinspec.args[2]) { /* [2] 保存了转换的pin数量 */
			if (group_names) {
				of_property_read_string_index(np,
						group_names_propname,
						index, &name);
				if (strlen(name)) {
					pr_err("%s: Group name of numeric GPIO ranges must be the empty string.\n",
						np->full_name);
					break;
				}
			}
            
            /*建立gpio引脚号与pinctrl引脚号的映射关系*/
			/* npins != 0: linear range */
			ret = gpiochip_add_pin_range(chip,
					pinctrl_dev_get_devname(pctldev),
					pinspec.args[0],
					pinspec.args[1],
					pinspec.args[2]);
			if (ret)
				return ret;
		} else {
			/* npins == 0: special range */
			if (pinspec.args[1]) {
				pr_err("%s: Illegal gpio-range format.\n",
					np->full_name);
				break;
			}

			if (!group_names) {
				pr_err("%s: GPIO group range requested but no %s property.\n",
					np->full_name, group_names_propname);
				break;
			}

			ret = of_property_read_string_index(np,
						group_names_propname,
						index, &name);
			if (ret)
				break;

			if (!strlen(name)) {
				pr_err("%s: Group name of GPIO group range cannot be the empty string.\n",
				np->full_name);
				break;
			}

			ret = gpiochip_add_pingroup_range(chip, pctldev,
						pinspec.args[0], name);
			if (ret)
				return ret;
		}
	}

	return 0;
}

```

###  2-1 of_parse_phandle_with_fixed_args()

```C
/**
 * of_parse_phandle_with_fixed_args() - Find a node pointed by phandle in a list
 * @np:		pointer to a device tree node containing a list
 * @list_name:	property name that contains a list
 * @cell_count: number of argument cells following the phandle
 * @index:	index of a phandle to parse out
 * @out_args:	optional pointer to output arguments structure (will be filled)
 *
 * This function is useful to parse lists of phandles and their arguments.
 * Returns 0 on success and fills out_args, on error returns appropriate
 * errno value.
 *
 * Caller is responsible to call of_node_put() on the returned out_args->np
 * pointer.
 *
 * Example:
 *
 * phandle1: node1 {
 * }
 *
 * phandle2: node2 {
 * }
 *
 * node3 {
 *	list = <&phandle1 0 2 &phandle2 2 3>;
 * }
 *
 * To get a device_node of the `node2' node you may call this:
 * of_parse_phandle_with_fixed_args(node3, "list", 2, 1, &args);
 */
int of_parse_phandle_with_fixed_args(const struct device_node *np,
				const char *list_name, int cell_count,
				int index, struct of_phandle_args *out_args)
{
	if (index < 0)
		return -EINVAL;
	return __of_parse_phandle_with_args(np, list_name, NULL, cell_count,
					   index, out_args);
}


/*************************************************************/


static int __of_parse_phandle_with_args(const struct device_node *np,
					const char *list_name,
					const char *cells_name,
					int cell_count, int index,
					struct of_phandle_args *out_args)
{
	struct of_phandle_iterator it;
	int rc, cur_index = 0;

	/* Loop over the phandles until all the requested entry is found */
	of_for_each_phandle(&it, rc, np, list_name, cells_name, cell_count) {
		/*
		 * All of the error cases bail out of the loop, so at
		 * this point, the parsing is successful. If the requested
		 * index matches, then fill the out_args structure and return,
		 * or return -ENOENT for an empty entry.
		 */
		rc = -ENOENT;
		if (cur_index == index) {
			if (!it.phandle)
				goto err;

			if (out_args) {
				int c;

				c = of_phandle_iterator_args(&it,
							     out_args->args,
							     MAX_PHANDLE_ARGS);
				out_args->np = it.node;
				out_args->args_count = c;
			} else {
				of_node_put(it.node);
			}

			/* Found it! return success */
			return 0;
		}

		cur_index++;
	}

	/*
	 * Unlock node before returning result; will be one of:
	 * -ENOENT : index is for empty phandle
	 * -EINVAL : parsing error on data
	 */

 err:
	of_node_put(it.node);
	return rc;
}
```

### 2-1-1 gpiochip_add_pin_range()

```C
/**
 * gpiochip_add_pin_range() - add a range for GPIO <-> pin mapping
 * @chip: the gpiochip to add the range for
 * @pinctrl_name: the dev_name() of the pin controller to map to
 * @gpio_offset: the start offset in the current gpio_chip number space
 * @pin_offset: the start offset in the pin controller number space
 * @npins: the number of pins from the offset of each pin space (GPIO and
 *	pin controller) to accumulate in this range
 */
int gpiochip_add_pin_range(struct gpio_chip *chip, const char *pinctl_name,
			   unsigned int gpio_offset, unsigned int pin_offset,
			   unsigned int npins)
{
	struct gpio_pin_range *pin_range;
	struct gpio_device *gdev = chip->gpiodev;
	int ret;

    /*申请了 gpio_pin_range 结构内存 */
	pin_range = kzalloc(sizeof(*pin_range), GFP_KERNEL);
	if (!pin_range) {
		chip_err(chip, "failed to allocate pin ranges\n");
		return -ENOMEM;
	}

	/* Use local offset as range ID */
	pin_range->range.id = gpio_offset;
	pin_range->range.gc = chip;
	pin_range->range.name = chip->label;
    
    /*
     * gdev->base：某个gpio控制器的偏移，
     *    如gpio4，base = 4 * 32 = 128
     * gpio_offset：gpio子系统的引脚编号。
     * gdev->base + gpio_offset：
     *    代表了gpio子系统的引脚编号对应pinctrl引脚编号的关系。
     */
	pin_range->range.base = gdev->base + gpio_offset; 
    
    /*pinctrl子系统的起始编号*/
	pin_range->range.pin_base = pin_offset;
    
    /*要转换的引脚数量*/
	pin_range->range.npins = npins;
	pin_range->pctldev = pinctrl_find_and_add_gpio_range(pinctl_name,
			&pin_range->range);
	if (IS_ERR(pin_range->pctldev)) {
		ret = PTR_ERR(pin_range->pctldev);
		chip_err(chip, "could not create pin range\n");
		kfree(pin_range);
		return ret;
	}
	chip_dbg(chip, "created GPIO range %d->%d ==> %s PIN %d->%d\n",
		 gpio_offset, gpio_offset + npins - 1,
		 pinctl_name,
		 pin_offset, pin_offset + npins - 1);

    /* 将gpio_pin_range 保存到gpiodev的list中 */
	list_add_tail(&pin_range->node, &gdev->pin_ranges);

	return 0;
}
```

### 总结

- 多个gpio-controller在dts进行描述，通过 名为 “platform” bus 的 driver 进行 probe。

  - 确认soc型号，获取其gpio寄存器信息。
  - 申请 gpiochip，根据soc信息设置 gpiochip，设置gpio寄存器组的地址，设置中间层函数（写方向，写电平，读状态）
  - 申请gpio_device，根据gpiochip设置gpio_device。
    - 根据引脚数量申请gpio_desc，设置gpio_desc，并全部挂入gpio_device的list。

  - 当存在"gpio-ranges"属性，则gpio将与pinctrl建立联系，这样的情况下，用户节点（如led，key）就不需要写入"pinctrl-names"，"pinctrl-0"这样的属性了。
  - 不当存在"gpio-ranges"属性，则需用户节点写入"pinctrl-names"，"pinctrl-0"这样的属性，这样会提前进行really probe，解析和设置pinctrl。


- gpio子系统中，会对soc的所有gpio顺序排序，这样的好处是，只需提供gpio号，就能找到gpio-controller对应的gpio_device，并找到pin对应的gpio_desc。

