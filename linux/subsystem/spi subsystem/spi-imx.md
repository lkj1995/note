### 1 init

```C
static struct spi_imx_devtype_data imx6ul_ecspi_devtype_data = {
	.intctrl = mx51_ecspi_intctrl,
	.config = mx51_ecspi_config,
	.trigger = mx51_ecspi_trigger,
	.rx_available = mx51_ecspi_rx_available,
	.reset = mx51_ecspi_reset,
	.devtype = IMX6UL_ECSPI,
};

static const struct of_device_id spi_imx_dt_ids[] = {
 	/* 匹配该属性 */
	{ .compatible = "fsl,imx6ul-ecspi", .data = &imx6ul_ecspi_devtype_data, },
	{ /* sentinel */ }
};


static struct platform_driver spi_imx_driver = {
	.driver = {
		   .name = DRIVER_NAME,
		   .of_match_table = spi_imx_dt_ids,
		   .pm = IMX_SPI_PM,
	},
	.id_table = spi_imx_devtype,
	.probe = spi_imx_probe,
	.remove = spi_imx_remove,
};

/* 进行了init，并创建了platform driver */
module_platform_driver(spi_imx_driver);
```

### 2 probe

```C
static int spi_imx_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	const struct of_device_id *of_id =
			of_match_device(spi_imx_dt_ids, &pdev->dev);
	struct spi_imx_master *mxc_platform_info =
			dev_get_platdata(&pdev->dev);
	struct spi_master *master;
	struct spi_imx_data *spi_imx;
	struct resource *res;
	int i, ret, irq;

	/* 
	 * 申请 设置spi_master 
	 * 申请 spi_imx_master
	 */
	master = spi_alloc_master(&pdev->dev, sizeof(struct spi_imx_data));

	platform_set_drvdata(pdev, master);
	
    /* 继续设置spi_master */
	master->bits_per_word_mask = SPI_BPW_RANGE_MASK(1, 32);
	master->bus_num = np ? -1 : pdev->id;

    /* 取出spi_imx_master，并进行设置 */
	spi_imx = spi_master_get_devdata(master);
	spi_imx->bitbang.master = master;
	spi_imx->dev = &pdev->dev;

	spi_imx->devtype_data = of_id ? of_id->data :
		(struct spi_imx_devtype_data *)pdev->id_entry->driver_data;

    /* device没提供该内容，跳过 */
	if (mxc_platform_info) {
		master->num_chipselect = mxc_platform_info->num_chipselect;
		master->cs_gpios = devm_kzalloc(&master->dev,
			sizeof(int) * master->num_chipselect, GFP_KERNEL);
		if (!master->cs_gpios)
			return -ENOMEM;

		for (i = 0; i < master->num_chipselect; i++)
			master->cs_gpios[i] = mxc_platform_info->chipselect[i];
 	}
	
    /* 设置 spi_imx_master->bitbang */
	spi_imx->bitbang.chipselect = spi_imx_chipselect;
	spi_imx->bitbang.setup_transfer = spi_imx_setupxfer;
	spi_imx->bitbang.txrx_bufs = spi_imx_transfer;
	spi_imx->bitbang.master->setup = spi_imx_setup;
	spi_imx->bitbang.master->cleanup = spi_imx_cleanup;
	spi_imx->bitbang.master->prepare_message = spi_imx_prepare_message;
	spi_imx->bitbang.master->unprepare_message = spi_imx_unprepare_message;
	spi_imx->bitbang.master->mode_bits = SPI_CPOL | SPI_CPHA | SPI_CS_HIGH;
	if (is_imx35_cspi(spi_imx) || is_imx51_ecspi(spi_imx))
		spi_imx->bitbang.master->mode_bits |= SPI_LOOP;

	init_completion(&spi_imx->xfer_done);

    /* 获取寄存器基地址,并转换成虚拟地址 */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	spi_imx->base = devm_ioremap_resource(&pdev->dev, res);
    /* 寄存器物理基地址 */
	spi_imx->base_phys = res->start;
	
    /* 调用中断子系统函数，解析dt并获取irq */
	irq = platform_get_irq(pdev, 0);
	
    /* 注册中断回调函数 */
	ret = devm_request_irq(&pdev->dev, irq, spi_imx_isr, 0,
			       dev_name(&pdev->dev), spi_imx);

	/* 获取时钟频率 "ipg"和"per" */
	spi_imx->clk_ipg = devm_clk_get(&pdev->dev, "ipg");
	spi_imx->clk_per = devm_clk_get(&pdev->dev, "per");

	/* 使能时钟 "ipg"和"per" */
	ret = clk_prepare_enable(spi_imx->clk_per);
	ret = clk_prepare_enable(spi_imx->clk_ipg);

	spi_imx->spi_clk = clk_get_rate(spi_imx->clk_per);

    /* 条件成立 */
	if (is_imx51_ecspi(spi_imx)) {
        /* dma 初始化和设置 */
		ret = spi_imx_sdma_init(&pdev->dev, spi_imx, master);
	}
	
    /* 确保 recv fifo 为空 */
	spi_imx->devtype_data->reset(spi_imx);
	
    /* 关闭所有spi的中断 */
	spi_imx->devtype_data->intctrl(spi_imx, 0);

	master->dev.of_node = pdev->dev.of_node;
    
    /*
     * 1. 使用gpio子系统，解析dt的"cs-gpios"属性，确认有多少个cs
     * 2. 并解析spi_master下的子节点，为每个申请spi_device，
     *    解析dt，设置最大频率，相位，名字，时钟极性等
     * 3. 将所有spi_device挂入spi_master的list。
     */
	ret = spi_bitbang_start(&spi_imx->bitbang);


	/* 有关"gpio-ranges",暂时不分析 */
	ret = devm_gpio_request(&pdev->dev, master->cs_gpios[i],
					DRIVER_NAME);
	

	return ret;
}
```

#### 2-1 spi_alloc_master()

```C
struct spi_master *spi_alloc_master(struct device *dev, unsigned size)
{
	struct spi_master	*master;


	/* 申请 spi_master 和 私有数据：sizeof(struct spi_imx_data) */
	master = kzalloc(size + sizeof(*master), GFP_KERNEL);

    /* 注册 device */
	device_initialize(&master->dev);
	master->bus_num = -1;
	master->num_chipselect = 1;
	master->dev.class = &spi_master_class;
	master->dev.parent = dev;
	pm_suspend_ignore_children(&master->dev, true);
    
    /* 保存私有数据在driver_data */
	spi_master_set_devdata(master, &master[1]);

	return master;
}
```

#### 2-2 mx51_ecspi_reset()

```C
static void mx51_ecspi_reset(struct spi_imx_data *spi_imx)
{
	/* 
	 * readl(spi_imx->base + MX51_ECSPI_STAT) & MX51_ECSPI_STAT_RR; 
	 * 如果 recv fifo 有数据，则读取fifo，目的是清空它
	 * 作用：确认 recv fifo 为空
	 */
	while (mx51_ecspi_rx_available(spi_imx))
		readl(spi_imx->base + MXC_CSPIRXDATA);
}
```

#### 2-3 spi_bitbang_start()

```C
int spi_bitbang_start(struct spi_bitbang *bitbang)
{
    /* 获取刚申请的 spi_master */
	struct spi_master *master = bitbang->master;
	int ret;

	/* 已设置,跳过 */
	if (!master->mode_bits)
		master->mode_bits = SPI_CPOL | SPI_CPHA | bitbang->flags;

	/* 继续设置 spi_master */
	master->prepare_transfer_hardware = spi_bitbang_prepare_hardware;
	master->unprepare_transfer_hardware = spi_bitbang_unprepare_hardware;
	master->transfer_one = spi_bitbang_transfer_one;
	master->set_cs = spi_bitbang_set_cs;

    /* 已设置,跳过 */
	if (!bitbang->txrx_bufs) {
		bitbang->use_dma = 0;
		bitbang->txrx_bufs = spi_bitbang_bufs;
		if (!master->setup) {
			if (!bitbang->setup_transfer)
				bitbang->setup_transfer =
					 spi_bitbang_setup_transfer;
			master->setup = spi_bitbang_setup;
			master->cleanup = spi_bitbang_cleanup;
		}
	}

	/* driver may get busy before register() returns, especially
	 * if someone registered boardinfo for devices
	 */
	ret = spi_register_master(spi_master_get(master));
	if (ret)
		spi_master_put(master);

	return 0;
}
```

#### 2-3-1 spi_register_master

```C
int spi_register_master(struct spi_master *master)
{
	static atomic_t		dyn_bus_id = ATOMIC_INIT((1<<15) - 1);
	struct device		*dev = master->dev.parent;
	struct boardinfo	*bi;
	int			status = -ENODEV;
	int			dynamic = 0;

	if (!dev)
		return -ENODEV;
	
    /* 解析dt,获取 cs-gpios 数量，cs使用的引脚号 */
	status = of_spi_register_master(master);

	 /* 添加 device */
	status = device_add(&master->dev);

    /* 初始化spi_master传输队列 */
	status = spi_master_initialize_queue(master);

    
	of_register_spi_devices(master);

	return status;
}
```

#### 2-3-1-1 of_spi_register_master()

```C
static int of_spi_register_master(struct spi_master *master)
{
	int nb, i, *cs;
	struct device_node *np = master->dev.of_node;

	if (!np)
		return 0;
	
    /* 1. 使用gpio子系统，解析dt。
     * 2. 找到 gpio-controller 节点, 确认"#gpio-cells"长度，为2", 
     *    说明使用2个数来描述一个gpio引脚。 
     * 3. 统计 2 * nb 的总个数，如 
     *	  cs-gpios = <&gpio1, 0 active_low 1 active_high>
     *    可知使用了2个cs引脚，分别是 gpio1_io0, gpio1_io1
     */
	nb = of_gpio_named_count(np, "cs-gpios");
	master->num_chipselect = max_t(int, nb, master->num_chipselect);

    
	/* 根据cs引脚数量申请内存 */
	cs = devm_kzalloc(&master->dev,
			  sizeof(int) * master->num_chipselect,
			  GFP_KERNEL);
    
    /* 指针指向该分配区域 */
	master->cs_gpios = cs;

	if (!master->cs_gpios)
		return -ENOMEM;

	for (i = 0; i < master->num_chipselect; i++)
		cs[i] = -ENOENT;
	
    /* 获取gpio号 */
	for (i = 0; i < nb; i++)
		cs[i] = of_get_named_gpio(np, "cs-gpios", i);

	return 0;
}
```

#### 2-3-1-2 of_register_spi_devices()

```C
static void of_register_spi_devices(struct spi_master *master)
{
	struct spi_device *spi;
	struct device_node *nc;
	
    /* 取出spi_master子节点 */
	for_each_available_child_of_node(master->dev.of_node, nc) {        
		spi = of_register_spi_device(master, nc);
	}
}


static struct spi_device *
of_register_spi_device(struct spi_master *master, struct device_node *nc)
{
	struct spi_device *spi;
	int rc;
	u32 value;

    /* 申请一个 spi_device */
	spi = spi_alloc_device(master);


	/* 记录子节点的"compatible"提供的名字 */
	rc = of_modalias_node(nc, spi->modalias,
				sizeof(spi->modalias));


	/* 第几个片选 */
	rc = of_property_read_u32(nc, "reg", &value);
	spi->chip_select = value;

	/* 记录是否为 相位 时钟极性 cs有效电平 3线模式 lsb */
	if (of_find_property(nc, "spi-cpha", NULL))
		spi->mode |= SPI_CPHA;
	if (of_find_property(nc, "spi-cpol", NULL))
		spi->mode |= SPI_CPOL;
	if (of_find_property(nc, "spi-cs-high", NULL))
		spi->mode |= SPI_CS_HIGH;
	if (of_find_property(nc, "spi-3wire", NULL))
		spi->mode |= SPI_3WIRE;
	if (of_find_property(nc, "spi-lsb-first", NULL))
		spi->mode |= SPI_LSB_FIRST;

	/* 记录是否为dspi或qspi */
	if (!of_property_read_u32(nc, "spi-tx-bus-width", &value)) {
		switch (value) {
		case 1:
			break;
		case 2:
			spi->mode |= SPI_TX_DUAL;
			break;
		case 4:
			spi->mode |= SPI_TX_QUAD;
			break;
		default:
			break;
		}
	}

	if (!of_property_read_u32(nc, "spi-rx-bus-width", &value)) {
		switch (value) {
		case 1:
			break;
		case 2:
			spi->mode |= SPI_RX_DUAL;
			break;
		case 4:
			spi->mode |= SPI_RX_QUAD;
			break;
		default:
			break;
		}
	}

	/* 记录设备的最大通讯频率 */
	rc = of_property_read_u32(nc, "spi-max-frequency", &value);
	spi->max_speed_hz = value;

	/* Store a pointer to the node in the device structure */
	of_node_get(nc);
	spi->dev.of_node = nc;

	/* Register the new device */
	rc = spi_add_device(spi);

	return spi;

}

```

#### 2-3-1-2-1 spi_add_device()

```C
int spi_add_device(struct spi_device *spi)
{
	static DEFINE_MUTEX(spi_add_lock);
	struct spi_master *master = spi->master;
	struct device *dev = master->dev.parent;
	int status;


	/* Set the bus ID string */
	spi_dev_set_name(spi);

	/* 根据子节点的"reg"内容作索引，找到具体引脚号 */
	if (master->cs_gpios)
		spi->cs_gpio = master->cs_gpios[spi->chip_select];


    /* 做一些配置检查，比如dual quad检查，并设置cs为无效电平 */
	status = spi_setup(spi);

	/* Device may be bound to an active driver when this returns */
	status = device_add(&spi->dev);

	return status;
}

```

#### 2-3-1-2-1-1 spi_setup()

````C
int spi_setup(struct spi_device *spi)
{
	unsigned	bad_bits, ugly_bits;
	int		status;


	/* help drivers fail *cleanly* when they need options
	 * that aren't supported with their current master
	 */
	bad_bits = spi->mode & ~spi->master->mode_bits;
	ugly_bits = bad_bits &
		    (SPI_TX_DUAL | SPI_TX_QUAD | SPI_RX_DUAL | SPI_RX_QUAD);
	if (ugly_bits) {
		dev_warn(&spi->dev,
			 "setup: ignoring unsupported mode bits %x\n",
			 ugly_bits);
		spi->mode &= ~ugly_bits;
		bad_bits &= ~ugly_bits;
	}
	if (bad_bits) {
		dev_err(&spi->dev, "setup: unsupported mode bits %x\n",
			bad_bits);
		return -EINVAL;
	}
	
    /* 默认设为8，用户层可修改 */
	if (!spi->bits_per_word)
		spi->bits_per_word = 8;

	status = __spi_validate_bits_per_word(spi->master, spi->bits_per_word);


    /* 子节点没有设置最大频率，默认用master的 */
	if (!spi->max_speed_hz)
		spi->max_speed_hz = spi->master->max_speed_hz;

	if (spi->master->setup)
		status = spi->master->setup(spi);
	
    /* 设置cs为无效状态 */
	spi_set_cs(spi, false);

	return status;
}
````

