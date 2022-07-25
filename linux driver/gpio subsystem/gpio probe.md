- 在gpio子系统中，芯片厂商会注册n个platfrom_driver，以用于匹配dts中对应的gpio-controler

- driver的匹配名称为‘’ imx35-gpio ‘’

- 因此匹配符合规则的gpio节点，如 imx6ull.dtsi 中的 gpio1~5 共用一个driver处理

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

gpio2: gpio@020a0000 {
	compatible = "fsl,imx6ul-gpio", "fsl,imx35-gpio";
	reg = <0x020a0000 0x4000>;
	interrupts = <GIC_SPI 68 IRQ_TYPE_LEVEL_HIGH>,
				<GIC_SPI 69 IRQ_TYPE_LEVEL_HIGH>;
	gpio-controller;
	#gpio-cells = <2>;
	interrupt-controller;
	#interrupt-cells = <2>;
};

gpio3: gpio@020a4000 {
	compatible = "fsl,imx6ul-gpio", "fsl,imx35-gpio";
	reg = <0x020a4000 0x4000>;
	interrupts = <GIC_SPI 70 IRQ_TYPE_LEVEL_HIGH>,
				<GIC_SPI 71 IRQ_TYPE_LEVEL_HIGH>;
	gpio-controller;
	#gpio-cells = <2>;
	interrupt-controller;
	#interrupt-cells = <2>;
};

gpio4: gpio@020a8000 {
	compatible = "fsl,imx6ul-gpio", "fsl,imx35-gpio";
	reg = <0x020a8000 0x4000>;
	interrupts = <GIC_SPI 72 IRQ_TYPE_LEVEL_HIGH>,
				 <GIC_SPI 73 IRQ_TYPE_LEVEL_HIGH>;
	gpio-controller;
	#gpio-cells = <2>;
	interrupt-controller;
	#interrupt-cells = <2>;
};

gpio5: gpio@020ac000 {
	compatible = "fsl,imx6ul-gpio", "fsl,imx35-gpio";
	reg = <0x020ac000 0x4000>;
	interrupts = <GIC_SPI 74 IRQ_TYPE_LEVEL_HIGH>,
				<GIC_SPI 75 IRQ_TYPE_LEVEL_HIGH>;
	gpio-controller;
	#gpio-cells = <2>;
	interrupt-controller;
	#interrupt-cells = <2>;
};
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
	}, {
		/* sentinel */
	}
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
    /*注册一个 platform drv*/
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
/**
 * struct gpio_chip - abstract a GPIO controller
 * @label: a functional name for the GPIO device, such as a part
 *	number or the name of the SoC IP-block implementing it.
 * @gpiodev: the internal state holder, opaque struct
 * @parent: optional parent device providing the GPIOs
 * @owner: helps prevent removal of modules exporting active GPIOs
 * @request: optional hook for chip-specific activation, such as
 *	enabling module power and clock; may sleep
 * @free: optional hook for chip-specific deactivation, such as
 *	disabling module power and clock; may sleep
 * @get_direction: returns direction for signal "offset", 0=out, 1=in,
 *	(same as GPIOF_DIR_XXX), or negative error
 * @direction_input: configures signal "offset" as input, or returns error
 * @direction_output: configures signal "offset" as output, or returns error
 * @get: returns value for signal "offset", 0=low, 1=high, or negative error
 * @set: assigns output value for signal "offset"
 * @set_multiple: assigns output values for multiple signals defined by "mask"
 * @set_debounce: optional hook for setting debounce time for specified gpio in
 *	interrupt triggered gpio chips
 * @set_single_ended: optional hook for setting a line as open drain, open
 *	source, or non-single ended (restore from open drain/source to normal
 *	push-pull mode) this should be implemented if the hardware supports
 *	open drain or open source settings. The GPIOlib will otherwise try
 *	to emulate open drain/source by not actively driving lines high/low
 *	if a consumer request this. The driver may return -ENOTSUPP if e.g.
 *	it supports just open drain but not open source and is called
 *	with LINE_MODE_OPEN_SOURCE as mode argument.
 * @to_irq: optional hook supporting non-static gpio_to_irq() mappings;
 *	implementation may not sleep
 * @dbg_show: optional routine to show contents in debugfs; default code
 *	will be used when this is omitted, but custom code can show extra
 *	state (such as pullup/pulldown configuration).
 * @base: identifies the first GPIO number handled by this chip;
 *	or, if negative during registration, requests dynamic ID allocation.
 *	DEPRECATION: providing anything non-negative and nailing the base
 *	offset of GPIO chips is deprecated. Please pass -1 as base to
 *	let gpiolib select the chip base in all possible cases. We want to
 *	get rid of the static GPIO number space in the long run.
 * @ngpio: the number of GPIOs handled by this controller; the last GPIO
 *	handled is (base + ngpio - 1).
 * @names: if set, must be an array of strings to use as alternative
 *      names for the GPIOs in this chip. Any entry in the array
 *      may be NULL if there is no alias for the GPIO, however the
 *      array must be @ngpio entries long.  A name can include a single printk
 *      format specifier for an unsigned int.  It is substituted by the actual
 *      number of the gpio.
 * @can_sleep: flag must be set iff get()/set() methods sleep, as they
 *	must while accessing GPIO expander chips over I2C or SPI. This
 *	implies that if the chip supports IRQs, these IRQs need to be threaded
 *	as the chip access may sleep when e.g. reading out the IRQ status
 *	registers.
 * @irq_not_threaded: flag must be set if @can_sleep is set but the
 *	IRQs don't need to be threaded
 * @read_reg: reader function for generic GPIO
 * @write_reg: writer function for generic GPIO
 * @pin2mask: some generic GPIO controllers work with the big-endian bits
 *	notation, e.g. in a 8-bits register, GPIO7 is the least significant
 *	bit. This callback assigns the right bit mask.
 * @reg_dat: data (in) register for generic GPIO
 * @reg_set: output set register (out=high) for generic GPIO
 * @reg_clk: output clear register (out=low) for generic GPIO
 * @reg_dir: direction setting register for generic GPIO
 * @bgpio_bits: number of register bits used for a generic GPIO i.e.
 *	<register width> * 8
 * @bgpio_lock: used to lock chip->bgpio_data. Also, this is needed to keep
 *	shadowed and real data registers writes together.
 * @bgpio_data:	shadowed data register for generic GPIO to clear/set bits
 *	safely.
 * @bgpio_dir: shadowed direction register for generic GPIO to clear/set
 *	direction safely.
 * @irqchip: GPIO IRQ chip impl, provided by GPIO driver
 * @irqdomain: Interrupt translation domain; responsible for mapping
 *	between GPIO hwirq number and linux irq number
 * @irq_base: first linux IRQ number assigned to GPIO IRQ chip (deprecated)
 * @irq_handler: the irq handler to use (often a predefined irq core function)
 *	for GPIO IRQs, provided by GPIO driver
 * @irq_default_type: default IRQ triggering type applied during GPIO driver
 *	initialization, provided by GPIO driver
 * @irq_parent: GPIO IRQ chip parent/bank linux irq number,
 *	provided by GPIO driver
 * @irq_need_valid_mask: If set core allocates @irq_valid_mask with all
 *	bits set to one
 * @irq_valid_mask: If not %NULL holds bitmask of GPIOs which are valid to
 *	be included in IRQ domain of the chip
 * @lock_key: per GPIO IRQ chip lockdep class
 *
 * A gpio_chip can help platforms abstract various sources of GPIOs so
 * they can all be accessed through a common programing interface.
 * Example sources would be SOC controllers, FPGAs, multifunction
 * chips, dedicated GPIO expanders, and so on.
 *
 * Each chip controls a number of signals, identified in method calls
 * by "offset" values in the range 0..(@ngpio - 1).  When those signals
 * are referenced through calls like gpio_get_value(gpio), the offset
 * is calculated by subtracting @base from the gpio number.
 */
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

### 1 mxc_gpio_probe()

```C
static int mxc_gpio_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct mxc_gpio_port *port;
	struct resource *iores;
	int irq_base;
	int err;

    /*记录soc的gpio寄存器偏移值*/
	mxc_gpio_get_hw(pdev);
	
    /*申请 mxc_gpio_port 结构内存*/
	port = devm_kzalloc(&pdev->dev, sizeof(*port), GFP_KERNEL);
	if (!port)
		return -ENOMEM;
	
    /*获取reg属性信息，如gpio控制器基地址，寄存器数量*/
	iores = platform_get_resource(pdev, IORESOURCE_MEM, 0);
    
    /*转换为虚拟地址并保存*/
	port->base = devm_ioremap_resource(&pdev->dev, iores);
	if (IS_ERR(port->base))
		return PTR_ERR(port->base);

	port->irq_high = platform_get_irq(pdev, 1);
	port->irq = platform_get_irq(pdev, 0);
	if (port->irq < 0)
		return port->irq;

	/* disable the interrupt and clear the status */
	writel(0, port->base + GPIO_IMR);/*禁止中断*/
	writel(~0, port->base + GPIO_ISR);/*清空状态寄存器*/

	if (mxc_gpio_hwtype == IMX21_GPIO) {
		/*
		 * Setup one handler for all GPIO interrupts. Actually setting
		 * the handler is needed only once, but doing it for every port
		 * is more robust and easier.
		 */
		irq_set_chained_handler(port->irq, mx2_gpio_irq_handler);
	} else {
		/* setup one handler for each entry */
		irq_set_chained_handler_and_data(port->irq,
						 mx3_gpio_irq_handler, port);
		if (port->irq_high > 0)
			/* setup handler for GPIO 16 to 31 */
			irq_set_chained_handler_and_data(port->irq_high,
							 mx3_gpio_irq_handler,
							 port);
	}

    /*记录了使用gpio（保存了读，写及方向寄存器地址，设置好如何操作寄存器的函数等）*/
	err = bgpio_init(&port->gc, &pdev->dev, 4,
			 port->base + GPIO_PSR,
			 port->base + GPIO_DR, NULL,
			 port->base + GPIO_GDIR, NULL,
			 BGPIOF_READ_OUTPUT_REG_SET);
	if (err)
		goto out_bgio;

    
	if (of_property_read_bool(np, "gpio-ranges")) {
		port->gc.request = gpiochip_generic_request;
		port->gc.free = gpiochip_generic_free;
	}

	port->gc.to_irq = mxc_gpio_to_irq;
	port->gc.base = (pdev->id < 0) ? of_alias_get_id(np, "gpio") * 32 :
					     pdev->id * 32;

	err = devm_gpiochip_add_data(&pdev->dev, &port->gc, port);
	if (err)
		goto out_bgio;

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

### 1-1 mxc_gpio_get_hw()

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

static enum mxc_gpio_hwtype mxc_gpio_hwtype;
static struct mxc_gpio_hwdata *mxc_gpio_hwdata;



static void mxc_gpio_get_hw(struct platform_device *pdev)
{
    /*
     * 获取当前soc的信息，厂商已定义好
     */
	const struct of_device_id *of_id = 
			of_match_device(mxc_gpio_dt_ids, &pdev->dev);
	enum mxc_gpio_hwtype hwtype;
	
    /*保存 of_device_id 索引*/
	if (of_id)
		pdev->id_entry = of_id->data;
 
    /*
     * .driver_data = IMX35_GPIO
     * 匹配类型为：IMX35_GPIO
     */
	hwtype = pdev->id_entry->driver_data; 
	
    /*已经匹配过了，不能重复匹配*/
	if (mxc_gpio_hwtype) { 
		/*
		 * The driver works with a reasonable presupposition,
		 * that is all gpio ports must be the same type when
		 * running on one soc.
		 */
		BUG_ON(mxc_gpio_hwtype != hwtype);
		return;
	}
    
	/*记录了gpio寄存器基值的偏移值*/
	if (hwtype == IMX35_GPIO)
		mxc_gpio_hwdata = &imx35_gpio_hwdata;
	else if (hwtype == IMX31_GPIO)
		mxc_gpio_hwdata = &imx31_gpio_hwdata;
	else
		mxc_gpio_hwdata = &imx1_imx21_gpio_hwdata;

	mxc_gpio_hwtype = hwtype;
}
```

### 1-2 bgpio_init()

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

### 1-2-1 bgpio_setup_io()

```C
/*
 * Create the device and allocate the resources.  For setting GPIO's there are
 * three supported configurations:
 *
 *	- single input/output register resource (named "dat").
 *	- set/clear pair (named "set" and "clr").
 *	- single output register resource and single input resource ("set" and
 *	dat").
 *
 * For the single output register, this drives a 1 by setting a bit and a zero
 * by clearing a bit.  For the set clr pair, this drives a 1 by setting a bit
 * in the set register and clears it by setting a bit in the clear register.
 * The configuration is detected by which resources are present.
 *
 * For setting the GPIO direction, there are three supported configurations:
 *
 *	- simple bidirection GPIO that requires no configuration.
 *	- an output direction register (named "dirout") where a 1 bit
 *	indicates the GPIO is an output.
 *	- an input direction register (named "dirin") where a 1 bit indicates
 *	the GPIO is an input.
 */
static int bgpio_setup_io(struct gpio_chip *gc,
			  void __iomem *dat,
			  void __iomem *set,
			  void __iomem *clr,
			  unsigned long flags)
{
    /*传入 (gc, port->base + GPIO_PSR, port->base + GPIO_DR, NULL) */
    
	/*记录状态寄存器，用于读电平*/
	gc->reg_dat = dat;
	if (!gc->reg_dat)
		return -EINVAL;

   
	if (set && clr) {
		gc->reg_set = set;
		gc->reg_clr = clr;
		gc->set = bgpio_set_with_clear;
		gc->set_multiple = bgpio_set_multiple_with_clear;
	} else if (set && !clr) {
        
        /*记录控制寄存器，用于设置电平*/
		gc->reg_set = set;
        
        /*
         * 具体操作函数
         * gc->bgpio_data |= mask; or gc->bgpio_data &= ~mask; 
         * gc->write_reg(gc->reg_set, gc->bgpio_data); 
         */
		gc->set = bgpio_set_set;
		gc->set_multiple = bgpio_set_multiple_set;
	} else if (flags & BGPIOF_NO_OUTPUT) {
		gc->set = bgpio_set_none;
		gc->set_multiple = NULL;
	} else {
		gc->set = bgpio_set;
		gc->set_multiple = bgpio_set_multiple;
	}

	if (!(flags & BGPIOF_UNREADABLE_REG_SET) &&
	    (flags & BGPIOF_READ_OUTPUT_REG_SET))
		gc->get = bgpio_get_set;
	else
		gc->get = bgpio_get;

	return 0;
}
```

### 1-2-2 bgpio_setup_direction()

```C
/*记录了如何操作gpio方向*/
static int bgpio_setup_direction(struct gpio_chip *gc,
				 void __iomem *dirout,
				 void __iomem *dirin,
				 unsigned long flags)
{
    /* IO不能同时输入输出 */
	if (dirout && dirin) {
		return -EINVAL;
	} else if (dirout) {
        
        /*保存gpio dr寄存器地址*/
		gc->reg_dir = dirout; 
        
        /* 
         * bgpio_dir_out()函数用法：设置方向，然后写入dr寄存器
         *   gc->bgpio_dir |= gc->pin2mask(gc, gpio);
         *   gc->write_reg(gc->reg_dir, gc->bgpio_dir);
         */
		gc->direction_output = bgpio_dir_out;
        
        /*同上*/
		gc->direction_input = bgpio_dir_in;
        
        /* Return 0 if output, 1 of input */
		gc->get_direction = bgpio_get_dir;
        
	} else if (dirin) {
		gc->reg_dir = dirin;
		gc->direction_output = bgpio_dir_out_inv;
		gc->direction_input = bgpio_dir_in_inv;
		gc->get_direction = bgpio_get_dir_inv;
	} else {
		if (flags & BGPIOF_NO_OUTPUT)
			gc->direction_output = bgpio_dir_out_err;
		else
			gc->direction_output = bgpio_simple_dir_out;
		gc->direction_input = bgpio_simple_dir_in;
	}

	return 0;
}

```

### 1-3 devm_gpiochip_add_data()

```C
/**
 * devm_gpiochip_add_data() - Resource manager piochip_add_data()
 * @dev: the device pointer on which irq_chip belongs to.
 * @chip: the chip to register, with chip->base initialized
 * Context: potentially before irqs will work
 *
 * Returns a negative errno if the chip can't be registered, such as
 * because the chip->base is invalid or already associated with a
 * different chip.  Otherwise it returns zero as a success code.
 *
 * The gpio chip automatically be released when the device is unbound.
 */
int devm_gpiochip_add_data(struct device *dev, struct gpio_chip *chip,
			   void *data)
{
    /* 传入参数 (&pdev->dev, &port->gc, port); */
	struct gpio_chip **ptr;
	int ret;

	ptr = devres_alloc(devm_gpio_chip_release, sizeof(*ptr),
			     GFP_KERNEL);
	if (!ptr)
		return -ENOMEM;

	ret = gpiochip_add_data(chip, data);
	if (ret < 0) {
		devres_free(ptr);
		return ret;
	}

	*ptr = chip;
	devres_add(dev, ptr);

	return 0;
}
```

### struct gpio_device

```C
/**
 * struct gpio_device - internal state container for GPIO devices
 * @id: numerical ID number for the GPIO chip
 * @dev: the GPIO device struct
 * @chrdev: character device for the GPIO device
 * @mockdev: class device used by the deprecated sysfs interface (may be
 * NULL)
 * @owner: helps prevent removal of modules exporting active GPIOs
 * @chip: pointer to the corresponding gpiochip, holding static
 * data for this device
 * @descs: array of ngpio descriptors.
 * @ngpio: the number of GPIO lines on this GPIO device, equal to the size
 * of the @descs array.
 * @base: GPIO base in the DEPRECATED global Linux GPIO numberspace, assigned
 * at device creation time.
 * @label: a descriptive name for the GPIO device, such as the part number
 * or name of the IP component in a System on Chip.
 * @data: per-instance data assigned by the driver
 * @list: links gpio_device:s together for traversal
 *
 * This state container holds most of the runtime variable data
 * for a GPIO device and can hold references and live on after the
 * GPIO chip has been removed, if it is still being used from
 * userspace.
 */
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

### 1-3-1 gpiochip_add_data()

```C
/**
 * gpiochip_add_data() - register a gpio_chip
 * @chip: the chip to register, with chip->base initialized
 * Context: potentially before irqs will work
 *
 * Returns a negative errno if the chip can't be registered, such as
 * because the chip->base is invalid or already associated with a
 * different chip.  Otherwise it returns zero as a success code.
 *
 * When gpiochip_add_data() is called very early during boot, so that GPIOs
 * can be freely used, the chip->parent device must be registered before
 * the gpio framework's arch_initcall().  Otherwise sysfs initialization
 * for GPIOs will fail rudely.
 *
 * gpiochip_add_data() must only be called after gpiolib initialization,
 * ie after core_initcall().
 *
 * If chip->base is negative, this requests dynamic assignment of
 * a range of valid GPIOs.
 */
int gpiochip_add_data(struct gpio_chip *chip, void *data)
{
	unsigned long	flags;
	int		status = 0;
	unsigned	i;
	int		base = chip->base;
	struct gpio_device *gdev;

	/*
	 * First: allocate and populate the internal stat container, and
	 * set up the struct device.
	 */
	gdev = kzalloc(sizeof(*gdev), GFP_KERNEL);
	if (!gdev)
		return -ENOMEM;
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
	if (gdev->id < 0) {
		status = gdev->id;
		goto err_free_gdev;
	}
    /*设置 gpio_device->dev 的名字*/
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

    /*根据引脚数量，创建n个 gpio_desc 结构，用于描述引脚*/
	gdev->descs = kcalloc(chip->ngpio, sizeof(gdev->descs[0]), GFP_KERNEL);
	if (!gdev->descs) {
		status = -ENOMEM;
		goto err_free_gdev;
	}

	if (chip->ngpio == 0) {
		chip_err(chip, "tried to insert a GPIO chip with zero lines\n");
		status = -EINVAL;
		goto err_free_descs;
	}
	
    /*创建内存并复制一份一样的字符串*/
	if (chip->label)
		gdev->label = kstrdup(chip->label, GFP_KERNEL);
	else
		gdev->label = kstrdup("unknown", GFP_KERNEL);
	if (!gdev->label) {
		status = -ENOMEM;
		goto err_free_descs;
	}

	gdev->ngpio = chip->ngpio;
    
    /* 保存了mxc_gpio_port结构 */
	gdev->data = data;

	spin_lock_irqsave(&gpio_lock, flags);

	/*
	 * TODO: this allocates a Linux GPIO number base in the global
	 * GPIO numberspace for this chip. In the long run we want to
	 * get *rid* of this numberspace and use only descriptors, but
	 * it may be a pipe dream. It will not happen before we get rid
	 * of the sysfs interface anyways.
	 */
	if (base < 0) {
		base = gpiochip_find_base(chip->ngpio);
		if (base < 0) {
			status = base;
			spin_unlock_irqrestore(&gpio_lock, flags);
			goto err_free_label;
		}
		/*
		 * TODO: it should not be necessary to reflect the assigned
		 * base outside of the GPIO subsystem. Go over drivers and
		 * see if anyone makes use of this, else drop this and assign
		 * a poison instead.
		 */
		chip->base = base;
	}
	gdev->base = base;
    
	/*根据chip->base的某些规则排序，将gpio_deivce存放到全局list中*/
	status = gpiodev_add_to_list(gdev);
	if (status) {
		spin_unlock_irqrestore(&gpio_lock, flags);
		goto err_free_label;
	}

	spin_unlock_irqrestore(&gpio_lock, flags);

    /*对每个gpio_desc默认标记为输出*/
	for (i = 0; i < chip->ngpio; i++) {
		struct gpio_desc *desc = &gdev->descs[i];
		
        /*每个desc保存父device*/
		desc->gdev = gdev;
		/*
		 * REVISIT: most hardware initializes GPIOs as inputs
		 * (often with pullups enabled) so power usage is
		 * minimized. Linux code should set the gpio direction
		 * first thing; but until it does, and in case
		 * chip->get_direction is not set, we may expose the
		 * wrong direction in sysfs.
		 */
        
		/*
		 * 翻译：一般来说，芯片会把引脚复位为上拉输入，保证最低功耗。
		 * linux处理gpio第一步，就是要先设置引脚方向，如不这样做的话，
		 * 会把错误的方向反馈给sysfs，此时用户cat出来的值是不正确的。
		 */
		if (chip->get_direction) {
			/*
			 * If we have .get_direction, set up the initial
			 * direction flag from the hardware.
			 */
            /*如为null，说明还没设置*/
			int dir = chip->get_direction(chip, i);

			if (!dir)
                /*记录引脚方向为输出，应该是把FLAG_IS_OUT保存在desc->flags*/
				set_bit(FLAG_IS_OUT, &desc->flags);
		} else if (!chip->direction_input) {
			/*
			 * If the chip lacks the .direction_input callback
			 * we logically assume all lines are outputs.
			 */
			set_bit(FLAG_IS_OUT, &desc->flags);
		}
	}

#ifdef CONFIG_PINCTRL
	INIT_LIST_HEAD(&gdev->pin_ranges);/*使用pinctrl*/
#endif

    /*为每个gpio_desc分配一个名字，gc->names为空返回0*/
	status = gpiochip_set_desc_names(chip);
	if (status)
		goto err_remove_from_list;

	status = gpiochip_irqchip_init_valid_mask(chip);
	if (status)
		goto err_remove_from_list;

	status = of_gpiochip_add(chip);
	if (status)
		goto err_remove_chip;

	acpi_gpiochip_add(chip);

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

err_remove_chip:
	acpi_gpiochip_remove(chip);
	gpiochip_free_hogs(chip);
	of_gpiochip_remove(chip);
	gpiochip_irqchip_free_valid_mask(chip);
err_remove_from_list:
	spin_lock_irqsave(&gpio_lock, flags);
	list_del(&gdev->list);
	spin_unlock_irqrestore(&gpio_lock, flags);
err_free_label:
	kfree(gdev->label);
err_free_descs:
	kfree(gdev->descs);
err_free_gdev:
	ida_simple_remove(&gpio_ida, gdev->id);
	/* failures here can mean systems won't boot... */
	pr_err("%s: GPIOs %d..%d (%s) failed to register\n", __func__,
	       gdev->base, gdev->base + gdev->ngpio - 1,
	       chip->label ? : "generic");
	kfree(gdev);
	return status;
}
```

### 1-3-1-1 gpiodev_add_to_list()

```C
	/*根据chip->base的某些规则排序，将gpio_deivce存放到全局list中*/
/*
 * Add a new chip to the global chips list, keeping the list of chips sorted
 * by range(means [base, base + ngpio - 1]) order.
 *
 * Return -EBUSY if the new chip overlaps with some other chip's integer
 * space.
 */
static int gpiodev_add_to_list(struct gpio_device *gdev)
{
	struct gpio_device *prev, *next;

	if (list_empty(&gpio_devices)) {
		/* initial entry in list */
		list_add_tail(&gdev->list, &gpio_devices);
		return 0;
	}

	next = list_entry(gpio_devices.next, struct gpio_device, list);
	if (gdev->base + gdev->ngpio <= next->base) {
		/* add before first entry */
		list_add(&gdev->list, &gpio_devices);
		return 0;
	}

	prev = list_entry(gpio_devices.prev, struct gpio_device, list);
	if (prev->base + prev->ngpio <= gdev->base) {
		/* add behind last entry */
		list_add_tail(&gdev->list, &gpio_devices);
		return 0;
	}

	list_for_each_entry_safe(prev, next, &gpio_devices, list) {
		/* at the end of the list */
		if (&next->list == &gpio_devices)
			break;

		/* add between prev and next */
		if (prev->base + prev->ngpio <= gdev->base
				&& gdev->base + gdev->ngpio <= next->base) {
			list_add(&gdev->list, &prev->list);
			return 0;
		}
	}

	dev_err(&gdev->dev, "GPIO integer space overlap, cannot add chip\n");
	return -EBUSY;
}
```

### 1-3-1-2 of_gpiochip_add()

```C
int of_gpiochip_add(struct gpio_chip *chip)
{
	int status;

	if ((!chip->of_node) && (chip->parent))
		chip->of_node = chip->parent->of_node;

	if (!chip->of_node)
		return 0;

    /*为空，默认赋值*/
	if (!chip->of_xlate) { 
		chip->of_gpio_n_cells = 2;
		chip->of_xlate = of_gpio_simple_xlate;
	}

	if (chip->of_gpio_n_cells > MAX_PHANDLE_ARGS)
		return -EINVAL;

    
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

### 1-2-1-2-1 of_gpio_simple_xlate()

```C
/**
 * of_gpio_simple_xlate - translate gpio_spec to the GPIO number and flags
 * @gc:		pointer to the gpio_chip structure
 * @np:		device node of the GPIO chip
 * @gpio_spec:	gpio specifier as found in the device tree
 * @flags:	a flags pointer to fill in
 *
 * This is simple translation function, suitable for the most 1:1 mapped
 * gpio chips. This function performs only one sanity check: whether gpio
 * is less than ngpios (that is specified in the gpio_chip).
 */
int of_gpio_simple_xlate(struct gpio_chip *gc,
			 const struct of_phandle_args *gpiospec, u32 *flags)
{
	/*
	 * We're discouraging gpio_cells < 2, since that way you'll have to
	 * write your own xlate function (that will have to retrieve the GPIO
	 * number and the flags from a single gpio cell -- this is possible,
	 * but not recommended).
	 */
	if (gc->of_gpio_n_cells < 2) {
		WARN_ON(1);
		return -EINVAL;
	}

	if (WARN_ON(gpiospec->args_count < gc->of_gpio_n_cells))
		return -EINVAL;

	if (gpiospec->args[0] >= gc->ngpio)
		return -EINVAL;

	if (flags)
		*flags = gpiospec->args[1];

	return gpiospec->args[0];
}
```

### 1-2-1-2-2 of_gpiochip_add_pin_range()

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

### 1-2-1-2-2-1 of_parse_phandle_with_fixed_args()

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

### 1-2-1-2-2-2 gpiochip_add_pin_range()

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
