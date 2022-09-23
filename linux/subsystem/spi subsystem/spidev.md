### struct spidev_data

```C
struct spidev_data {
	dev_t			devt;
	spinlock_t		spi_lock;
	struct spi_device	*spi;
	struct list_head	device_entry;

	/* TX/RX buffers are NULL unless this device is open (users > 0) */
	struct mutex		buf_lock;
	unsigned		users;
	u8			*tx_buffer;
	u8			*rx_buffer;
	u32			speed_hz;
};
```

### 1 dt

```C
ecspi1: ecspi@02008000 {
    #address-cells = <1>;
    #size-cells = <0>;
    compatible = "fsl,imx6ul-ecspi", "fsl,imx51-ecspi";
    reg = <0x02008000 0x4000>;
    interrupts = <GIC_SPI 31 IRQ_TYPE_LEVEL_HIGH>;
    clocks = <&clks IMX6UL_CLK_ECSPI1>,
     	<&clks IMX6UL_CLK_ECSPI1>;
    clock-names = "ipg", "per";
    dmas = <&sdma 3 7 1>, <&sdma 4 7 2>;
    dma-names = "rx", "tx";
    status = "disabled";   
};


&ecspi1 {
    pinctrl-names = "default";
    pinctrl-0 = <&pinctrl_ecspi1>;
    
    fsl,spi-num-chipselects = <2>;
    cs-gpio = <&gpio4 26 GPIO_ACTIVE_LOW>, <&gpio4 24 GPIO_ACTIVE_LOW>;
    status = "okay"; 
    
    spidev0: spi@0 {
        label = "eeprom";
        /* compatible使用该命名就可以匹配该driver */
        compatible = "rohm,dh2228fv";   
        reg = <0>;
        spi-max-frequency = <5000000>;
    };    
    
};
```

### 2 spidev_init()

```C
static int __init spidev_init(void)
{
	int status;

	/* 注册设备号 */
	status = register_chrdev(SPIDEV_MAJOR, "spi", &spidev_fops);

	/* 注册 spidev class */
	spidev_class = class_create(THIS_MODULE, "spidev");


	status = spi_register_driver(&spidev_spi_driver);
	if (status < 0) {
		class_destroy(spidev_class);
		unregister_chrdev(SPIDEV_MAJOR, spidev_spi_driver.driver.name);
	}
	return status;
}
```

### 3 spidev_probe()

```C
/*
 * 1. 在此之前，soc厂商的driver已提前解析dt，
 *    并生成spi_master和spi_device在spi子系统。
 * 2. spi_bus_type的match会被触发。
 * 3. match成功后，会调用spi_bus_type的probe函数。
 * 4. 会将dev和drv进行绑定，probe中会查找被match的node
      对应哪个spi_device,传参给drvier的probe。
 * 5. 然后就到执行下面的函数了。
 */
static int spidev_probe(struct spi_device *spi)
{
	struct spidev_data	*spidev;
	int			status;
	unsigned long		minor;


	/* 申请 spidev_data */
	spidev = kzalloc(sizeof(*spidev), GFP_KERNEL);

	/* 设置 spidev_data */
	spidev->spi = spi;
	spin_lock_init(&spidev->spi_lock);
	mutex_init(&spidev->buf_lock);

	INIT_LIST_HEAD(&spidev->device_entry);


    
    /* 寻找空闲的minor */
	minor = find_first_zero_bit(minors, N_SPI_MINORS);
	if (minor < N_SPI_MINORS) {
		struct device *dev;

		spidev->devt = MKDEV(SPIDEV_MAJOR, minor);
		dev = device_create(spidev_class, &spi->dev, spidev->devt,
				    spidev, "spidev%d.%d",
				    spi->master->bus_num, spi->chip_select);
		status = PTR_ERR_OR_ZERO(dev);
	} else {
		dev_dbg(&spi->dev, "no minor number available!\n");
		status = -ENODEV;
	}
    

    /* 标记该minor已被使用 */
	set_bit(minor, minors);
        
     /* 添加到spidev的全局变量list */
	list_add(&spidev->device_entry, &device_list);

    
	spidev->speed_hz = spi->max_speed_hz;
	spi_set_drvdata(spi, spidev);


	return status;
}

```

### 4 spidev_read()

```C
static ssize_t
spidev_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{
	struct spidev_data	*spidev;
	ssize_t			status = 0;

	/* chipselect only toggles at start or end of operation */
	if (count > bufsiz)
		return -EMSGSIZE;

	spidev = filp->private_data;

	mutex_lock(&spidev->buf_lock);
	status = spidev_sync_read(spidev, count);
	if (status > 0) {
		unsigned long	missing;
		/* 将读到的数据返回给user */
		missing = copy_to_user(buf, spidev->rx_buffer, status);
		if (missing == status)
			status = -EFAULT;
		else
			status = status - missing;
	}
	mutex_unlock(&spidev->buf_lock);

	return status;
}
```

### 5 spidev_write()

```C

static ssize_t
spidev_write(struct file *filp, const char __user *buf,
		size_t count, loff_t *f_pos)
{
	struct spidev_data	*spidev;
	ssize_t			status = 0;
	unsigned long		missing;

	/* chipselect only toggles at start or end of operation */
	if (count > bufsiz)
		return -EMSGSIZE;

	spidev = filp->private_data;

	/* 读取user要写的数据 */
	missing = copy_from_user(spidev->tx_buffer, buf, count);
	if (missing == 0)
        /* 最终调用了spi子系统的spi_sync() */
		status = spidev_sync_write(spidev, count);
	else
		status = -EFAULT;


	return status;
}
```

#### 5-1 spidev_sync_write()

```C
spidev_sync_write() --> spidev_sync() --> spi_sync()--> __spi_sync()  
-->  __spi_queued_transfer() --> kthread_insert_work()

    
/* 一层一层将消息封装起来，然后挂入worklist,并唤醒线程进行处理，控制spi外设进行发送 */    
static void kthread_insert_work(struct kthread_worker *worker,
				struct kthread_work *work,
				struct list_head *pos)
{
	kthread_insert_work_sanity_check(worker, work);

	list_add_tail(&work->node, pos);
    /* 添加一个work */
	work->worker = worker;
    
    /* 唤醒spi的内核线程 */
	if (!worker->current_work && likely(worker->task))   
		wake_up_process(worker->task);
}
```

### 总结

- 此驱动为简单的spi驱动，并无涉及dma，中断，如果需要复杂的处理，要其他驱动代替。