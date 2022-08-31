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
### struct imx_i2c_hwdata

```C
enum imx_i2c_type { /*board类型*/
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

### 1 i2c_adap_imx_init()

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
    /* 注册 platform driver */
	return platform_driver_register(&i2c_imx_driver);
}
```

### 2 match

```C
/* imx6ull.dtsi */
i2c1: i2c@021a0000 {
	#address-cells = <1>;
	#size-cells = <0>;
	compatible = "fsl,imx6ul-i2c", "fsl,imx21-i2c"; /*match "fsl,imx21-i2c"*/
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


### 3 probe

```C
struct imxi2c_platform_data {
	u32 bitrate;
};


static int i2c_imx_probe(struct platform_device *pdev)
{
    /*
     * 1 i2c_bus_type的match匹配后，会执行probe，
     *   此时需要确认最佳匹配的类型，用于提取出私有内容data，
     *   用于初始化adapter。
     * 2 每个设备树的i2cx控制器，调用同个drv，构造出多个adapter。
     */
	const struct of_device_id *of_id = 
        of_match_device(i2c_imx_dt_ids,&pdev->dev);
    
    
	struct imx_i2c_struct *i2c_imx;
	struct resource *res;
	struct imxi2c_platform_data *pdata = dev_get_platdata(&pdev->dev);
	void __iomem *base;
	int irq, ret;
	dma_addr_t phy_addr;
    

    /* 查找"interrupt"属性的第0个hwirq，并建立映射，返回virq */
	irq = platform_get_irq(pdev, 0);
    
	/*获取"reg"属性*/
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
    
    /*将i2c寄存器物理地址，转为虚拟地址*/
	base = devm_ioremap_resource(&pdev->dev, res);

    /* 记录物理地址 */
	phy_addr = (dma_addr_t)res->start;
    
   
    /* 申请包含 adapter 的 imx_i2c_struct 内存 */    
	i2c_imx = devm_kzalloc(&pdev->dev, sizeof(*i2c_imx), GFP_KERNEL);


    /*找到私有数据 imx_i2c_hwdata */
	if (of_id)
		i2c_imx->hwdata = of_id->data;
	else
		i2c_imx->hwdata = (struct imx_i2c_hwdata *)
				platform_get_device_id(pdev)->driver_data;


    /*设置 adapter */
	strlcpy(i2c_imx->adapter.name, pdev->name, sizeof(i2c_imx->adapter.name));
	i2c_imx->adapter.owner			= THIS_MODULE;
	i2c_imx->adapter.algo			= &i2c_imx_algo;
	i2c_imx->adapter.dev.parent		= &pdev->dev;
	i2c_imx->adapter.nr				= pdev->id;
	i2c_imx->adapter.dev.of_node	= pdev->dev.of_node;
	i2c_imx->base					= base;


    /* 获取i2c时钟 */
	i2c_imx->clk = devm_clk_get(&pdev->dev, NULL);

    /* 配置和使能i2c时钟 */
	ret = clk_prepare_enable(i2c_imx->clk);
    

    /* 根据virq, 申请中断回调函数 i2c_imx_isr */
	ret = devm_request_irq(&pdev->dev, irq, i2c_imx_isr,
			       IRQF_NO_SUSPEND, pdev->name, i2c_imx);

    
	init_waitqueue_head(&i2c_imx->queue);       
	i2c_set_adapdata(&i2c_imx->adapter, i2c_imx);
	platform_set_drvdata(pdev, i2c_imx);


    /* 
     * 1. 获取"clock-frequency"属性内容配置分频值。
     * 2. 不存在则使用默认配置。
     */
	i2c_imx->bitrate = IMX_I2C_BIT_RATE;
	ret = of_property_read_u32(pdev->dev.of_node,
				   "clock-frequency", &i2c_imx->bitrate);
	if (ret < 0 && pdata && pdata->bitrate)
		i2c_imx->bitrate = pdata->bitrate;


    /*设置 i2c_cr和i2c_sr寄存器为默认值*/
	imx_i2c_write_reg(i2c_imx->hwdata->i2cr_ien_opcode ^ I2CR_IEN,
			i2c_imx, IMX_I2C_I2CR);
	imx_i2c_write_reg(i2c_imx->hwdata->i2sr_clr_opcode, i2c_imx, IMX_I2C_I2SR);


    /* option，当i2c功能切换为gpio功能,可自动恢复引脚设置 */
	ret = i2c_imx_init_recovery_info(i2c_imx, pdev);


    /*
     * 1. 已经根据厂商数据，完成了对adapter的配置。
     * 2. 注册 adapter到i2c子系统，
     *   2-1. 找到adapter下的子节点(client)。
     *   2-2. 解析dt，并注册。
     */
	ret = i2c_add_numbered_adapter(&i2c_imx->adapter);

	return 0;   /* Return OK */
}

```

#### 3-1 i2c_add_numbered_adapter()

```C
/* 根据adapter->nr的序号，注册adapter。如果想让系统动态分配一个number,将nr设置为-1。*//
int i2c_add_numbered_adapter(struct i2c_adapter *adap)
{
    /* 动态分配 */
	if (adap->nr == -1) /* -1 means dynamically assign bus id */
		return i2c_add_adapter(adap);
	
    /* 最终都会调用 __i2c_add_numbered_adapter */
	return __i2c_add_numbered_adapter(adap);
}
```

#### 3-1-1 i2c_register_adapter()

```C
static int __i2c_add_numbered_adapter(struct i2c_adapter *adap)
{

	return i2c_register_adapter(adap);
}


static int i2c_register_adapter(struct i2c_adapter *adap)
{
	int res = -EINVAL;

    /* adapter一定要提供算法 */
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

    /* adapter节点，命名为"i2c-%d" */
	dev_set_name(&adap->dev, "i2c-%d", adap->nr);
    
    /* 挂入i2c_bus总线 */
	adap->dev.bus = &i2c_bus_type;
	adap->dev.type = &i2c_adapter_type;
    
    /* 注册 device */
	res = device_register(&adap->dev);
    
    /* 解析apapter子节点，创建和设置 client */
	of_i2c_register_devices(adap);

   
     /* 添加了1个新的adapter,对i2c总线下的 i2c_client 尝试进行 match  */     
	bus_for_each_drv(&i2c_bus_type, NULL, adap, __process_new_adapter);


	return 0;
}
```

#### 3-1-1-1 of_i2c_register_devices()

```C
static void of_i2c_register_devices(struct i2c_adapter *adap)
{
	struct device_node *bus, *node;
	struct i2c_client *client;
   
    /* 遍历子节点, 创建和设置client */
	for_each_available_child_of_node(bus, node) {
        
        /* 检查标志位，防止重复创建 */
		if (of_node_test_and_set_flag(node, OF_POPULATED))
			continue;
		
        /* 注册一个client */
		client = of_i2c_register_device(adap, node);
	}

	of_node_put(bus);
}
```

#### 3-1-1-1-1 of_i2c_register_device()

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

	/* 获取"compatible"内容，作为client名字 */
	if (of_modalias_node(node, info.type, sizeof(info.type)) < 0) {
		return ERR_PTR(-EINVAL);
	}

    /* 获取从设备地址 如 reg = <0x1a>; */
	addr_be = of_get_property(node, "reg", &len);
	if (!addr_be || (len < sizeof(*addr_be))) {
		return ERR_PTR(-EINVAL);
	}

    /*大小端转换*/
	addr = be32_to_cpup(addr_be);
    
    
    /* 是否为10bit地址 */
	if (addr & I2C_TEN_BIT_ADDRESS) {
		addr &= ~I2C_TEN_BIT_ADDRESS;
		info.flags |= I2C_CLIENT_TEN;
	}

    /* 是否为从机设备 */
	if (addr & I2C_OWN_SLAVE_ADDRESS) {
		addr &= ~I2C_OWN_SLAVE_ADDRESS;
		info.flags |= I2C_CLIENT_SLAVE;
	}

    /* 检查地址有效性 */
	if (i2c_check_addr_validity(addr, info.flags)) {
		return ERR_PTR(-EINVAL);
	}

    /* 保存从设备地址 */
	info.addr = addr;
	info.of_node = of_node_get(node);
	info.archdata = &dev_ad;

    /* 获取属性"wakeup-source" */
	if (of_get_property(node, "wakeup-source", NULL))
		info.flags |= I2C_CLIENT_WAKE;
	
    /* 根据info，申请注册 client */
	result = i2c_new_device(adap, &info);

	return result;
}
```

#### 3-1-1-1-1-1 i2c_new_device()

```C
struct i2c_client * 
    i2c_new_device(struct i2c_adapter *adap, struct i2c_board_info const *info)
{
	struct i2c_client	*client;
	int			status;

    /* 1. 申请 i2c_client */
	client = kzalloc(sizeof *client, GFP_KERNEL);


    /* 2. 设置client */
	client->adapter = adap;
	client->dev.platform_data = info->platform_data;

	if (info->archdata)
		client->dev.archdata = *info->archdata;

	client->flags = info->flags;
	client->addr = info->addr;
	client->irq = info->irq;

	strlcpy(client->name, info->type, sizeof(client->name));

	status = i2c_check_addr_validity(client->addr, client->flags);


	/* 检查地址是否已经被使用 */
	status = i2c_check_addr_busy(adap, i2c_encode_flags_to_addr(client));


	client->dev.parent = &client->adapter->dev;
	client->dev.bus = &i2c_bus_type;
	client->dev.type = &i2c_client_type;
	client->dev.of_node = info->of_node;
    
    /* 用于生成firmware节点 */
	client->dev.fwnode = info->fwnode;

    /* client命名为"%d-%04x" "adapter-设备地址" */
	i2c_dev_set_name(adap, client);
    
    /* dev注册 */
	status = device_register(&client->dev);

	return client;
}
```

#### 3-1-2  __process_new_adapter()

```C
static int __process_new_adapter(struct device_driver *d, void *data)
{
	return i2c_do_add_adapter(to_i2c_driver(d), data);
}


static int i2c_do_add_adapter(struct i2c_driver *driver,
			      struct i2c_adapter *adap)
{
    /* 添加了adapter后，会对i2c-bus上的device进行检测，是否能够match */
	i2c_detect(adap, driver);

	/* Let legacy drivers scan this bus for matching devices */
	if (driver->attach_adapter) {
		driver->attach_adapter(adap);
	}
	return 0;
}
```

#### 3-1-2-1 i2c_detect()

```C
static int i2c_detect(struct i2c_adapter *adapter, struct i2c_driver *driver)
{
	const unsigned short *address_list;
	struct i2c_client *temp_client;
	int i, err = 0;
	int adap_id = i2c_adapter_id(adapter);

	address_list = driver->address_list;
	if (!driver->detect || !address_list)
		return 0;

	/* Warn that the adapter lost class based instantiation */
	if (adapter->class == I2C_CLASS_DEPRECATED) {
		return 0;
	}

	/* Stop here if the classes do not match */
	if (!(adapter->class & driver->class))
		return 0;

	/* Set up a temporary client to help detect callback */
	temp_client = kzalloc(sizeof(struct i2c_client), GFP_KERNEL);
	temp_client->adapter = adapter;

	for (i = 0; address_list[i] != I2C_CLIENT_END; i += 1) {
		temp_client->addr = address_list[i];
		err = i2c_detect_address(temp_client, driver);
		if (unlikely(err))
			break;
	}

	kfree(temp_client);
	return err;
}
```

### 3 i2c_imx_xfer()

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

### 4 i2c_imx_start()

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

### 总结

- i2c-imx.c文件中的platform_driver，注册在platform_bus下，该platform_driver的probe实际作用就是用来解析该imx类型的soc，根据名字选择具体系列的soc，读取私有数据，读取dts，构造一个adapter，并将其挂载到i2c_bus_type总线下，如：命名为"i2c-0" "i2c-1"。

- 然后在adapter的of_node下，解析其子of_node（即client），读取dts，构造n个client，也挂载到i2c_bus_type总线下，如：命名为“0-0039”(0代表着该设备在i2c哪条总线下，0039代表着设备地址。4位的长度用途是，可能是7bit设备地址，也可能是10bit设备地址)

