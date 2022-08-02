### 1 tty driver注册

#### 1-1 imx_serial_init()

```C
static struct uart_driver imx_reg = {
	.owner          = THIS_MODULE,
	.driver_name    = DRIVER_NAME,
	.dev_name       = DEV_NAME,
	.major          = SERIAL_IMX_MAJOR,
	.minor          = MINOR_START,
	.nr             = ARRAY_SIZE(imx_ports),
	.cons           = IMX_CONSOLE,
};

static int __init imx_serial_init(void)
{
    /* 1. 初始化一个uart_driver，并根据串口数量，创建多个port
     * 2. 分配设置注册一个tty_driver，申请相关指针
     */
	int ret = uart_register_driver(&imx_reg);

	if (ret)
		return ret;

    /*注册platform driver*/
	ret = platform_driver_register(&serial_imx_driver);
	if (ret != 0)
		uart_unregister_driver(&imx_reg);

	return ret;
}
```

#### struct uart_driver

````C
struct uart_driver {
	struct module		*owner;
	const char		*driver_name;
	const char		*dev_name;
	int			 major;
	int			 minor;
	int			 nr;
	struct console		*cons;

	/*
	 * these are private; the low level driver should not
	 * touch these; they should be initialised to NULL
	 */
	struct uart_state	*state;
	struct tty_driver	*tty_driver;
};
````

#### struct tty_driver

```C
struct tty_driver {
	int	magic;		/* magic number for this structure */
	struct kref kref;	/* Reference management */
	struct cdev **cdevs;
	struct module	*owner;
	const char	*driver_name;
	const char	*name;
	int	name_base;	/* offset of printed name */
	int	major;		/* major device number */
	int	minor_start;	/* start of minor device number */
	unsigned int	num;	/* number of devices allocated */
	short	type;		/* type of tty driver */
	short	subtype;	/* subtype of tty driver */
	struct ktermios init_termios; /* Initial termios */
	unsigned long	flags;		/* tty driver flags */
	struct proc_dir_entry *proc_entry; /* /proc fs entry */
	struct tty_driver *other; /* only used for the PTY driver */

	/*
	 * Pointer to the tty data structures
	 */
	struct tty_struct **ttys;
	struct tty_port **ports;
	struct ktermios **termios;
	void *driver_state;

	/*
	 * Driver methods
	 */

	const struct tty_operations *ops;
	struct list_head tty_drivers;
};

```

#### struct uart_state 

```C
/*
 * This is the state information which is persistent across opens.
 */
struct uart_state {
	struct tty_port		port;

	enum uart_pm_state	pm_state;
	struct circ_buf		xmit;

	atomic_t		refcount;
	wait_queue_head_t	remove_wait;
	struct uart_port	*uart_port;
};
```

#### 1-2 uart_register_driver()

````C
/* 注册一个uart内核层的driver */
/**
 *	uart_register_driver - register a driver with the uart core layer
 *	@drv: low level driver structure
 *
 *	Register a uart driver with the core driver.  We in turn register
 *	with the tty layer, and initialise the core driver per-port state.
 *  先注册一个uart内核层的uart_driver，紧接着在tty层注册一个uart_port
 *	We have a proc file in /proc/tty/driver which is named after the
 *	normal driver.
 *
 *	drv->port should be NULL, and the per-port structures should be
 *	registered using uart_add_one_port after this call has succeeded.
 */
int uart_register_driver(struct uart_driver *drv)
{
    struct tty_driver *normal;
    
	/*
	 * 为每个串口都分配一个state，
	 * 当前drv->nr数量为8，已提前写好。
	 */
	drv->state = kzalloc(sizeof(struct uart_state) * drv->nr, GFP_KERNEL);
    
    /*申请tty driver*/
    normal = alloc_tty_driver(drv->nr);
    
    /*初始化tty driver*/
	normal->driver_name	= drv->driver_name;
	normal->name		= drv->dev_name;
	normal->major		= drv->major;
	normal->minor_start	= drv->minor;
	normal->type		= TTY_DRIVER_TYPE_SERIAL;
	normal->subtype		= SERIAL_TYPE_NORMAL;
	normal->init_termios	= tty_std_termios;
	normal->init_termios.c_cflag = B9600 | CS8 | CREAD | HUPCL | CLOCAL;
	normal->init_termios.c_ispeed = normal->init_termios.c_ospeed = 9600;
	normal->flags		= TTY_DRIVER_REAL_RAW | TTY_DRIVER_DYNAMIC_DEV;
	normal->driver_state    = drv;
    
    /*设置tty_driver 的 fops*/
	tty_set_operations(normal, &uart_ops);    
    
  	/*
	 * Initialise the UART state(s).
	 * 初始化state中的每个port
	 */
	for (i = 0; i < drv->nr; i++) {
		struct uart_state *state = drv->state + i;
		struct tty_port *port = &state->port;

		tty_port_init(port);
        /*设置port 的 fops*/
		port->ops = &uart_port_ops;
	}  
    
    /*注册 tty driver*/
    retval = tty_register_driver(normal);
    
}
````

#### 1-2-1 alloc_tty_driver()

```C

/**
 * __tty_alloc_driver -- allocate tty driver
 * @lines: count of lines this driver can handle at most
 * @owner: module which is repsonsible for this driver
 * @flags: some of TTY_DRIVER_* flags, will be set in driver->flags
 *
 * This should not be called directly, some of the provided macros should be
 * used instead. Use IS_ERR and friends on @retval.
 */
struct tty_driver *__tty_alloc_driver(unsigned int lines, struct module *owner,
		unsigned long flags)
{
	struct tty_driver *driver;
	unsigned int cdevs = 1;
	int err;

	if (!lines || (flags & TTY_DRIVER_UNNUMBERED_NODE && lines > 1))
		return ERR_PTR(-EINVAL);

    /*申请 tty_driver */
	driver = kzalloc(sizeof(struct tty_driver), GFP_KERNEL);
	if (!driver)
		return ERR_PTR(-ENOMEM);

	kref_init(&driver->kref);
	driver->magic = TTY_DRIVER_MAGIC;
    
    /*串口的数量*/
	driver->num = lines;
	driver->owner = owner;
	driver->flags = flags;

    /*为每个串口分配了 ttys 和 termios 指针*/
	if (!(flags & TTY_DRIVER_DEVPTS_MEM)) {
		driver->ttys = kcalloc(lines, sizeof(*driver->ttys),
				GFP_KERNEL);
		driver->termios = kcalloc(lines, sizeof(*driver->termios),
				GFP_KERNEL);
		if (!driver->ttys || !driver->termios) {
			err = -ENOMEM;
			goto err_free_all;
		}
	}
	/*为每个串口分配了 ports 指针*/
	if (!(flags & TTY_DRIVER_DYNAMIC_ALLOC)) {
		driver->ports = kcalloc(lines, sizeof(*driver->ports),
				GFP_KERNEL);
		if (!driver->ports) {
			err = -ENOMEM;
			goto err_free_all;
		}
        /*串口的数量*/
		cdevs = lines;
	}
    
	/*为每个串口分配了 cdevs 指针*/
	driver->cdevs = kcalloc(cdevs, sizeof(*driver->cdevs), GFP_KERNEL);
	if (!driver->cdevs) {
		err = -ENOMEM;
		goto err_free_all;
	}

	return driver;
err_free_all:
	kfree(driver->ports);
	kfree(driver->ttys);
	kfree(driver->termios);
	kfree(driver->cdevs);
	kfree(driver);
	return ERR_PTR(err);
}
```

### 2 serial_imx_driver的match

```C
/**  dts  **/
uart1: serial@02020000 {
	compatible = "fsl,imx6ul-uart", "fsl,imx6q-uart", "fsl,imx21-uart";
	reg = <0x02020000 0x4000>;
	interrupts = <GIC_SPI 26 IRQ_TYPE_LEVEL_HIGH>;
	clocks = <&clks IMX6UL_CLK_UART1_IPG>,
			 <&clks IMX6UL_CLK_UART1_SERIAL>;
	clock-names = "ipg", "per";
	status = "disabled";
};


/**  driver  **/
#define IMX21_UTS 0xb4 /* UART Test Register on all other i.mx*/

static struct imx_uart_data imx_uart_devdata[] = {
	...
	[IMX21_UART] = {
		.uts_reg = IMX21_UTS,
		.devtype = IMX21_UART,
	},
	...
};

static const struct of_device_id imx_uart_dt_ids[] = {
	{ .compatible = "fsl,imx6q-uart", .data = &imx_uart_devdata[IMX6Q_UART], },
	{ .compatible = "fsl,imx53-uart", .data = &imx_uart_devdata[IMX53_UART], },
	{ .compatible = "fsl,imx1-uart", .data = &imx_uart_devdata[IMX1_UART], },
	{ .compatible = "fsl,imx21-uart", .data = &imx_uart_devdata[IMX21_UART], },
	{ /* sentinel */ }
};
```

###  3 serial_imx_driver的probe

```C
static int serial_imx_probe(struct platform_device *pdev)
{
	struct imx_port *sport;
	void __iomem *base;
	int ret = 0, reg;
	struct resource *res;
	int txirq, rxirq, rtsirq;

    /*申请 1个 imx_port*/
	sport = devm_kzalloc(&pdev->dev, sizeof(*sport), GFP_KERNEL);
	
	/*确认此节点为uart：id是否为"serial"*/
	ret = serial_imx_probe_dt(sport, pdev);

	/*获取"reg"属性，寄存器地址*/
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
    /*虚拟地址转换*/
	base = devm_ioremap_resource(&pdev->dev, res);

	/*获取中断号*/
	rxirq = platform_get_irq(pdev, 0);
	txirq = platform_get_irq(pdev, 1);
	rtsirq = platform_get_irq(pdev, 2);

    /*初始化port及timer*/
	sport->port.dev = &pdev->dev;
	sport->port.mapbase = res->start;
	sport->port.membase = base;
	sport->port.type = PORT_IMX,
	sport->port.iotype = UPIO_MEM;
	sport->port.irq = rxirq;
	sport->port.fifosize = 32;
    /*初始化 ops*/
	sport->port.ops = &imx_pops;
	sport->port.rs485_config = imx_rs485_config;
	sport->port.rs485.flags =
		SER_RS485_RTS_ON_SEND | SER_RS485_RX_DURING_TX;
	sport->port.flags = UPF_BOOT_AUTOCONF;
	init_timer(&sport->timer);
	sport->timer.function = imx_timeout;
	sport->timer.data     = (unsigned long)sport;

    
	/*获取时钟源ipg及per的值，用于计算uart clk*/
	sport->clk_ipg = devm_clk_get(&pdev->dev, "ipg");
	sport->clk_per = devm_clk_get(&pdev->dev, "per");

	
    /*获取uart的clk*/
	sport->port.uartclk = clk_get_rate(sport->clk_per);

    /*记录这个port在全局指针*/
    imx_ports[sport->port.line] = sport;
    
	/*添加一个port*/
	return uart_add_one_port(&imx_reg, &sport->port);
}
```

#### 3-1 uart_add_one_port()

```C
/**
 *	uart_add_one_port - attach a driver-defined port structure
 *	@drv: pointer to the uart low level driver structure for this port
 *	@uport: uart port structure to use for this port.
 *
 *	This allows the driver to register its own uart_port structure
 *	with the core driver.  The main purpose is to allow the low
 *	level uart drivers to expand uart_port, rather than having yet
 *	more levels of structures.
 */
int uart_add_one_port(struct uart_driver *drv, struct uart_port *uport)
{
    
	tty_dev = tty_port_register_device_attr(port, drv->tty_driver,
			uport->line, uport->dev, port, uport->tty_groups);
    
}
```

#### 3-1-1 tty_port_register_device_attr()

```C
/**
 * tty_port_register_device_attr - register tty device
 * @port: tty_port of the device
 * @driver: tty_driver for this device
 * @index: index of the tty
 * @device: parent if exists, otherwise NULL
 * @drvdata: Driver data to be set to device.
 * @attr_grp: Attribute group to be set on device.
 *
 * It is the same as tty_register_device_attr except the provided @port is
 * linked to a concrete tty specified by @index. Use this or tty_port_install
 * (or both). Call tty_port_link_device as a last resort.
 */
struct device *tty_port_register_device_attr(struct tty_port *port,
		struct tty_driver *driver, unsigned index,
		struct device *device, void *drvdata,
		const struct attribute_group **attr_grp)
{
    /*port记录在tty_driver*/
	tty_port_link_device(port, driver, index);
    
    /*
     * 1. 注册设置一个cdev，设置tty_fops
     * 2. cdev记录在tty driver
     * 3. 分配设置注册一个device
     */
	return tty_register_device_attr(driver, index, device, drvdata,
			attr_grp);
}
```

#### 3-1-2 tty_register_device_attr()

```C
struct device *tty_register_device_attr(struct tty_driver *driver,
				   unsigned index, struct device *device,
				   void *drvdata,
				   const struct attribute_group **attr_grp)
{
   /*
    * 申请，注册cdev，tty_driver记录cdev
    * cdev记录tty_fops
    */
	retval = tty_cdev_add(driver, devt, index, 1);
    
    /*申请device*/
    dev = kzalloc(sizeof(*dev), GFP_KERNEL);
    
    /*设置device*/
	dev->devt = devt;
    /*挂在tty_class*/
	dev->class = tty_class;
	dev->parent = device;
	dev->release = tty_device_create_release;
	dev_set_name(dev, "%s", name);
	dev->groups = attr_grp;
	dev_set_drvdata(dev, drvdata);

    /*注册device*/
	retval = device_register(dev);    
}
```

### 4 open操作

```C
/* 1. 用户调用 open("/dev/ttymxc0") */
  /* 1-1. cdev子系统的open处理 */

/* 2. 调用了 tty_open() */
  /* 2-1. 根据设备号在全局list找到 tty_driver */	
  /* 2-2. 申请tty_struct,并绑定到 tty_driver */

/* 3. 调用了 line discipline 的 open */
/* 3-1. 不知做了啥 */

/* 4. tty_driver->cdev[0].open()处理，即 uart_open() */
    /* 4-1. 保存uart_state(即port)到tty_struct */
	/* 4-2. 接着调用了 port 的 fops uart_port_activate() */

/* 5. 调用了 uart_ops 的 startup(), 即 imx_startup */ 
/* 5-1. 配置uart外设的寄存器，如配置和使能CLK，DMA，IRQ*/
```

### 5 read 操作

