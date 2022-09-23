### struct uart_driver

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

### struct tty_driver

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

### struct uart_state 

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

### 1 imx_serial_init()

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
    /* 
     * 1. 初始化一个uart_driver，根据串口数量，创建多个port
     * 2. 分配设置注册一个tty_driver，申请相关指针
     */
	int ret = uart_register_driver(&imx_reg);

    /* 注册platform driver */
	ret = platform_driver_register(&serial_imx_driver);

	return ret;
}
```

#### 1-1 uart_register_driver()

````C
int uart_register_driver(struct uart_driver *drv)
{
    struct tty_driver *normal;
    
	
	/* 为每个uart分配state，当前drv->nr为8 */
	drv->state = kzalloc(sizeof(struct uart_state) * drv->nr, GFP_KERNEL);
    
    /* 1. 申请和设置 tty_driver */
    normal = alloc_tty_driver(drv->nr);
    
    /* 2. 继续设置 tty_driver */
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
    
    /* 设置 tty_driver->fops */
	tty_set_operations(normal, &uart_ops);    
    
	for (i = 0; i < drv->nr; i++) {
		struct uart_state *state = drv->state + i;
		struct tty_port *port = &state->port;
        
		/* 设置 state->port */
		tty_port_init(port);
        
        /* 设置 state->port->fops */
		port->ops = &uart_port_ops;
	}  
    
    /*注册 tty driver*/
    retval = tty_register_driver(normal);
    
}
````

#### 1-1-1 tty_alloc_driver()

```C
#define tty_alloc_driver(lines, flags) \
		__tty_alloc_driver(lines, THIS_MODULE, flags)

struct tty_driver *__tty_alloc_driver(unsigned int lines, struct module *owner,
		unsigned long flags)
{
	struct tty_driver *driver;
	unsigned int cdevs = 1;
	int err;


    /* 1. 申请 tty_driver */
	driver = kzalloc(sizeof(struct tty_driver), GFP_KERNEL);

	/* 2. 设置 tty_driver */
	kref_init(&driver->kref);
	driver->magic = TTY_DRIVER_MAGIC;
	driver->num = lines;
	driver->owner = owner;
	driver->flags = flags;

    /* 2-1. 根据line数量,申请 *ttys *termios *poat *cdev 指针 */
	if (!(flags & TTY_DRIVER_DEVPTS_MEM)) {       
		driver->ttys = 
            kcalloc(lines, sizeof(*driver->ttys), GFP_KERNEL);       
		driver->termios = 
            kcalloc(lines, sizeof(*driver->termios), GFP_KERNEL);
	}
	if (!(flags & TTY_DRIVER_DYNAMIC_ALLOC)) {
		driver->ports = kcalloc(lines, sizeof(*driver->ports),
				GFP_KERNEL);
		cdevs = lines;
	}    
	driver->cdevs = kcalloc(cdevs, sizeof(*driver->cdevs), GFP_KERNEL);

	return driver;
}
```

#### 1-1-2 tty_register_driver()

```C
int tty_register_driver(struct tty_driver *driver)
{
	int error;
	int i;
	dev_t dev;
	struct device *d;

	/* 注册设备号域，长度为uart数量 */
	dev = MKDEV(driver->major, driver->minor_start);
	error = register_chrdev_region(dev, driver->num, driver->name);

	/* 注册cdev，当前跳过 */
	if (driver->flags & TTY_DRIVER_DYNAMIC_ALLOC) {
		error = tty_cdev_add(driver, dev, 0, driver->num);
	}

    /* tty_drivers保存到全局链表 */
	list_add(&driver->tty_drivers, &tty_drivers);

    /* 需其他drv调用 tty_register_device() 来生成device，当前跳过 */
	if (!(driver->flags & TTY_DRIVER_DYNAMIC_DEV)) {
		for (i = 0; i < driver->num; i++) {
			d = tty_register_device(driver, i, NULL);
			if (IS_ERR(d)) {
				error = PTR_ERR(d);
				goto err_unreg_devs;
			}
		}
	}
    /* proc文件系统相关 */
	proc_tty_register_driver(driver);
	driver->flags |= TTY_DRIVER_INSTALLED;
	return 0;
}

```

### 2 match

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

###  3 serial_imx_probe()

```C
static int serial_imx_probe(struct platform_device *pdev)
{
	struct imx_port *sport;
	void __iomem *base;
	int ret = 0, reg;
	struct resource *res;
	int txirq, rxirq, rtsirq;

    /*申请 imx_port */
	sport = devm_kzalloc(&pdev->dev, sizeof(*sport), GFP_KERNEL);
	
	/* 获取别名号 */
	ret = serial_imx_probe_dt(sport, pdev);

	/*获取"reg"属性的寄存器首地址，虚拟地址转换 */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	base = devm_ioremap_resource(&pdev->dev, res);

	/*获取中断号*/
	rxirq = platform_get_irq(pdev, 0);
	txirq = platform_get_irq(pdev, 1);
	rtsirq = platform_get_irq(pdev, 2);

    /* 初始化port及timer */
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
    
    
  	sport->gpios = mctrl_gpio_init(&sport->port, 0);
    
	/*获取时钟源 "ipg" "per" 频率 */
	sport->clk_ipg = devm_clk_get(&pdev->dev, "ipg");
	sport->clk_per = devm_clk_get(&pdev->dev, "per");

    /* 计算uart的时钟 */
	sport->port.uartclk = clk_get_rate(sport->clk_per);
	
    /* 使能"ipg"时钟 */
    ret = clk_prepare_enable(sport->clk_ipg);
    
	/* 读改写，在 request_irq() 之前将中断关闭 */
	reg = readl_relaxed(sport->port.membase + UCR1);
	reg &= ~(UCR1_ADEN | UCR1_TRDYEN | UCR1_IDEN | UCR1_RRDYEN |
		 UCR1_TXMPTYEN | UCR1_RTSDEN);
	writel_relaxed(reg, sport->port.membase + UCR1);  
    
    clk_disable_unprepare(sport->clk_ipg);
    
    /* 开启rx和tx中断，注册回调函数 */
	ret = devm_request_irq(&pdev->dev, rxirq, imx_rxint, 0,
				    dev_name(&pdev->dev), sport);
	ret = devm_request_irq(&pdev->dev, txirq, imx_txint, 0,
				    dev_name(&pdev->dev), sport);

    
    /* 记录这个imx_port在全局指针 */
    imx_ports[sport->port.line] = sport;
    
    /* 保存私有数据 */
    platform_set_drvdata(pdev, sport);
    
	/* 向 uart_driver 添加一个 uart_port */
	return uart_add_one_port(&imx_reg, &sport->port);
}
```

#### 3-1 serial_imx_probe_dt()

```C
static int serial_imx_probe_dt(struct imx_port *sport,
		struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	int ret;

    /* 保存私有数据，即uartx寄存器偏移值 */
	sport->devdata = of_device_get_match_data(&pdev->dev);

	/* 获取别名，确认uart号 */
	ret = of_alias_get_id(np, "serial");
	sport->port.line = ret;

	return 0;
}

```

#### 3-2 uart_add_one_port()

```C
int uart_add_one_port(struct uart_driver *drv, struct uart_port *uport)
{
	struct uart_state *state;
	struct tty_port *port;
	int ret = 0;
	struct device *tty_dev;
	int num_groups;
	
    /* 根据uart号，寻找对应的state */
	state = drv->state + uport->line;
	port = &state->port;

	atomic_set(&state->refcount, 1);
	init_waitqueue_head(&state->remove_wait);
    /* state 保存 uart_port */
	state->uart_port = uport;
	uport->state = state;

	state->pm_state = UART_PM_STATE_UNDEFINED;
	uport->cons = drv->cons;
    /* 次设备号 */
	uport->minor = drv->tty_driver->minor_start + uport->line;

	/* 设置flag,操作寄存器 */
	uart_configure_port(drv, state, uport);

	/* attr属性 */
	num_groups = 2;
	if (uport->attr_group)
		num_groups++;

	uport->tty_groups = kcalloc(num_groups, sizeof(*uport->tty_groups),
				    GFP_KERNEL);
	uport->tty_groups[0] = &tty_dev_attr_group;
	if (uport->attr_group)
		uport->tty_groups[1] = uport->attr_group;

	tty_dev = tty_port_register_device_attr(port, drv->tty_driver,
			uport->line, uport->dev, port, uport->tty_groups);
	if (likely(!IS_ERR(tty_dev))) {
		device_set_wakeup_capable(tty_dev, 1);
	}

	uport->flags &= ~UPF_DEAD;

 out:
	mutex_unlock(&port->mutex);
	mutex_unlock(&port_mutex);

	return ret;
}
```

#### 3-2-1 uart_configure_port()

```C
static void
uart_configure_port(struct uart_driver *drv, struct uart_state *state,
		    struct uart_port *port)
{
	unsigned int flags;

	flags = 0;
	if (port->flags & UPF_AUTO_IRQ)
		flags |= UART_CONFIG_IRQ;
	if (port->flags & UPF_BOOT_AUTOCONF) {
		if (!(port->flags & UPF_FIXED_TYPE)) {
			port->type = PORT_UNKNOWN;
			flags |= UART_CONFIG_TYPE;
		}
		port->ops->config_port(port, flags);
	}

	if (port->type != PORT_UNKNOWN) {
		unsigned long flags;

		/* Power up port for set_mctrl() */
		uart_change_pm(state, UART_PM_STATE_ON);

		/* 寄存器操作 */
		spin_lock_irqsave(&port->lock, flags);
		port->ops->set_mctrl(port, port->mctrl & TIOCM_DTR);
		spin_unlock_irqrestore(&port->lock, flags);


		if (port->cons && !(port->cons->flags & CON_ENABLED))
			register_console(port->cons);
        
		if (!uart_console(port))
			uart_change_pm(state, UART_PM_STATE_OFF);
	}
}

```

#### 3-2-2 tty_port_register_device_attr()

```C
struct device *tty_port_register_device_attr(struct tty_port *port,
		struct tty_driver *driver, unsigned index,
		struct device *device, void *drvdata,
		const struct attribute_group **attr_grp)
{
    /* port记录在tty_driver */
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

#### 3-2-2-1 tty_port_link_device()

```C
void tty_port_link_device(struct tty_port *port,
		struct tty_driver *driver, unsigned index)
{
	driver->ports[index] = port;
}
```

#### 3-2-2-2 tty_register_device_attr()

```C
struct device *tty_register_device_attr(struct tty_driver *driver,
				   unsigned index, struct device *device,
				   void *drvdata,
				   const struct attribute_group **attr_grp)
{
	char name[64];
    /* 获取设备号 */
	dev_t devt = MKDEV(driver->major, driver->minor_start) + index;
	struct device *dev = NULL;
	int retval = -ENODEV;
	bool cdev = false;

	/* 设置 "/dev/ttymxc[0~8]" 命名 */
	if (driver->type == TTY_DRIVER_TYPE_PTY)
		pty_line_name(driver, index, name);
	else
		tty_line_name(driver, index, name);

    /* 申请设置注册 cdev */
	if (!(driver->flags & TTY_DRIVER_DYNAMIC_ALLOC)) {
		retval = tty_cdev_add(driver, devt, index, 1);
		cdev = true;
	}
    
	/* 申请 device */
	dev = kzalloc(sizeof(*dev), GFP_KERNEL);

	/* 设置 device */
	dev->devt = devt;
	dev->class = tty_class;
	dev->parent = device;
	dev->release = tty_device_create_release;
	dev_set_name(dev, "%s", name);
	dev->groups = attr_grp;
	dev_set_drvdata(dev, drvdata);
    
	/* 注册 device */
	retval = device_register(dev);

	return dev;
}
```

#### 3-2-2-2-1 tty_cdev_add()

```C
static int tty_cdev_add(struct tty_driver *driver, dev_t dev,
		unsigned int index, unsigned int count)
{
	int err;

	/* 申请 cdev */
	driver->cdevs[index] = cdev_alloc();
    
	/* 设置 cdev->ops */
	driver->cdevs[index]->ops = &tty_fops;
	driver->cdevs[index]->owner = driver->owner;
    
    /* 绑定设备号，注册cdev */
	err = cdev_add(driver->cdevs[index], dev, count);

	return err;
}
```

### 4 tty_open()

```C
static int tty_open(struct inode *inode, struct file *filp)
{
	struct tty_struct *tty;
	int noctty, retval;
	dev_t device = inode->i_rdev;
	unsigned saved_flags = filp->f_flags;

	nonseekable_open(inode, filp);

retry_open:
    /* 
     * 1. 申请 tty_file_private 
     * 2. file->private_data = tty_file_private;
     */
	retval = tty_alloc_file(filp);

	/* 如首次打开,file没保存tty, 执行 tty_open_by_driver() */
	tty = tty_open_current_tty(device, filp);
    /* 根据设备号device，找到对应的 tty_driver */
	if (!tty)
		tty = tty_open_by_driver(device, inode, filp);

	/* file 保存 tty */
	tty_add_file(tty, filp);

	check_tty_count(tty, __func__);
	tty_debug_hangup(tty, "opening (count=%d)\n", tty->count);

    /* 调用 uart_ops */
	if (tty->ops->open)
		retval = tty->ops->open(tty, filp);
	else
		retval = -ENODEV;
	filp->f_flags = saved_flags;


	clear_bit(TTY_HUPPED, &tty->flags);
	return 0;
}
```

#### 4-1 uart_open()

```C
static int uart_open(struct tty_struct *tty, struct file *filp)
{
	struct uart_driver *drv = tty->driver->driver_state;
	int retval, line = tty->index;
	struct uart_state *state = drv->state + line;

	tty->driver_data = state;

    /* 
     * port->ops->activate(port, tty);
     * 即 uart_port_activate()
     */
	retval = tty_port_open(&state->port, tty, filp);
	if (retval > 0)
		retval = 0;

	return retval;
}
```

#### 4-1-1 uart_port_activate()

```C
uart_port_activate() --> uart_port_startup() --> imx_startup()

imx_startup: 配置uart外设的寄存器，如配置和使能CLK，DMA，IRQ.
```

### 5 tty_read()

```C
static irqreturn_t imx_rxint(int irq, void *dev_id)
{
	struct imx_port *sport = dev_id;
	unsigned int rx, flg, ignored = 0;
	struct tty_port *port = &sport->port.state->port;
	unsigned long flags, temp;

	spin_lock_irqsave(&sport->port.lock, flags);

	while (readl(sport->port.membase + USR2) & USR2_RDR) {
		flg = TTY_NORMAL;
		sport->port.icount.rx++;

		rx = readl(sport->port.membase + URXD0);

		temp = readl(sport->port.membase + USR2);
		if (temp & USR2_BRCD) {
			writel(USR2_BRCD, sport->port.membase + USR2);
			if (uart_handle_break(&sport->port))
				continue;
		}

		if (uart_handle_sysrq_char(&sport->port, (unsigned char)rx))
			continue;

		if (unlikely(rx & URXD_ERR)) {
			if (rx & URXD_BRK)
				sport->port.icount.brk++;
			else if (rx & URXD_PRERR)
				sport->port.icount.parity++;
			else if (rx & URXD_FRMERR)
				sport->port.icount.frame++;
			if (rx & URXD_OVRRUN)
				sport->port.icount.overrun++;

			if (rx & sport->port.ignore_status_mask) {
				if (++ignored > 100)
					goto out;
				continue;
			}

			rx &= (sport->port.read_status_mask | 0xFF);

			if (rx & URXD_BRK)
				flg = TTY_BREAK;
			else if (rx & URXD_PRERR)
				flg = TTY_PARITY;
			else if (rx & URXD_FRMERR)
				flg = TTY_FRAME;
			if (rx & URXD_OVRRUN)
				flg = TTY_OVERRUN;

#ifdef SUPPORT_SYSRQ
			sport->port.sysrq = 0;
#endif
		}

		if (sport->port.ignore_status_mask & URXD_DUMMY_READ)
			goto out;

		if (tty_insert_flip_char(port, rx, flg) == 0)
			sport->port.icount.buf_overrun++;
	}

out:
	spin_unlock_irqrestore(&sport->port.lock, flags);
	tty_flip_buffer_push(port);
	return IRQ_HANDLED;
}
```

