### struct i2c_driver

```C
struct i2c_driver {
	unsigned int class;

	/* Notifies the driver that a new bus has appeared. You should avoid
	 * using this, it will be removed in a near future.
	 */
	int (*attach_adapter)(struct i2c_adapter *) __deprecated;

	/* Standard driver model interfaces */
	int (*probe)(struct i2c_client *, const struct i2c_device_id *);
	int (*remove)(struct i2c_client *);

	/* driver model interfaces that don't relate to enumeration  */
	void (*shutdown)(struct i2c_client *);

	void (*alert)(struct i2c_client *, enum i2c_alert_protocol protocol,
		      unsigned int data);

	/* a ioctl like command that can be used to perform specific functions
	 * with the device.
	 */
	int (*command)(struct i2c_client *client, unsigned int cmd, void *arg);

	struct device_driver driver;
	const struct i2c_device_id *id_table;

	/* Device detection callback for automatic device creation */
	int (*detect)(struct i2c_client *, struct i2c_board_info *);
	const unsigned short *address_list;
	struct list_head clients;
};
```

### struct i2c_board_info

```c
struct i2c_board_info {
	char		type[I2C_NAME_SIZE];
	unsigned short	flags;
	unsigned short	addr;
	void		*platform_data;
	struct dev_archdata	*archdata;
	struct device_node *of_node;
	struct fwnode_handle *fwnode;
	int		irq;
};
```

### 1 i2c_add_driver()

```C
#define i2c_add_driver(driver) \
        i2c_register_driver(THIS_MODULE, driver)


int i2c_register_driver(struct module *owner, struct i2c_driver *driver)
{
	int res;

    /*1. 设置 i2c_driver 其他的部分 */
	driver->driver.owner = owner;
	driver->driver.bus = &i2c_bus_type;
	INIT_LIST_HEAD(&driver->clients);


    /* 初始化driver，进行probe，第一种匹配规则 */
	res = driver_register(&driver->driver);


    /*第二种匹配规则*/
	i2c_for_each_dev(driver, __process_new_driver);

	return 0;
}
```

#### 1-1 driver_register()

```c
driver_register() --> bus_add_driver() --> driver_attach()

int driver_attach(struct device_driver *drv)
{
	return bus_for_each_dev(drv->bus, NULL, drv, __driver_attach);
}

/* 如果是dummy_driver执行的操作，此时并没有device,会直接返回 */
int bus_for_each_dev(struct bus_type *bus, struct device *start,
		     void *data, int (*fn)(struct device *, void *))
{

	while ((dev = next_device(&i)) && !error)
        /* 执行 __driver_attach() */
		error = fn(dev, data);

}
```

#### 1-1-1 __driver_attach()

```C
static int __driver_attach(struct device *dev, void *data)
{

    /* match 匹配 */
	ret = driver_match_device(drv, dev);
	
    /* device绑定driver，并进行driver->probe */
    if (!dev->driver)
		driver_probe_device(drv, dev);

}

static inline int driver_match_device(struct device_driver *drv,
				      struct device *dev)
{
    /* 执行 i2c_device_match */
	return drv->bus->match ? drv->bus->match(dev, drv) : 1;
}
```

#### 1-1-1-1 i2c_device_match()

```c
static int i2c_device_match(struct device *dev, struct device_driver *drv)
{
	struct i2c_client	*client = i2c_verify_client(dev);
	struct i2c_driver	*driver;


	/* Attempt an OF style match */
	if (of_driver_match_device(dev, drv))
		return 1;

	driver = to_i2c_driver(drv);

	if (driver->id_table)
		return i2c_match_id(driver->id_table, client) != NULL;

	return 0;
}
```

#### 1-1-1-2 driver_probe_device()

```C
driver_probe_device() --> really_probe() --> i2c_device_probe()

/* 
 * 1. i2c_bus_type的probe。
 * 2. really_probe中，保存了dev—>driver = drv 
 */
static int i2c_device_probe(struct device *dev)
{
    /* 确认是device 是 i2c_client类型 */
	struct i2c_client	*client = i2c_verify_client(dev);
	struct i2c_driver	*driver;
	int status;

	/* 获取中断号 */
	irq = of_irq_get(dev->of_node, 0);
	client->irq = irq;
	
    /* 如果匹配,match会设置好device的driver */
	driver = to_i2c_driver(dev->driver);

    /* 执行driver的probe */
	status = driver->probe(client, i2c_match_id(driver->id_table, client));

	return 0;
}

```

#### 1-2 i2c_for_each_dev()

```C
/*第二种匹配规则*/
int i2c_for_each_dev(void *data, int (*fn)(struct device *, void *))
{
	int res;
    
    /* fn: __process_new_driver  data: i2c_driver */            
	res = bus_for_each_dev(&i2c_bus_type, NULL, data, fn);

	return res;
}
```

#### 1-2-1 bus_for_each_dev()

```C
int bus_for_each_dev(struct bus_type *bus, struct device *start,
		     void *data, int (*fn)(struct device *, void *))
{
	struct klist_iter i;
	struct device *dev;
	int error = 0;


    /* 取出 i2c bus 的 klist_devices */
	klist_iter_init_node(&bus->p->klist_devices, &i,
			     (start ? &start->p->knode_bus : NULL));
	while ((dev = next_device(&i)) && !error)
         /* 每个dev都执行 __process_new_driver */
		error = fn(dev, data);
	klist_iter_exit(&i);
	return error;
}
```

#### 1-2-1-1 i2c_do_add_adapter()  

```C
__process_new_driver() --> i2c_do_add_adapter(data, to_i2c_adapter(dev))

static int i2c_do_add_adapter(struct i2c_driver *driver,
			      struct i2c_adapter *adap)
{
	
	i2c_detect(adap, driver);

	/* Let legacy drivers scan this bus for matching devices */
	if (driver->attach_adapter) {
		driver->attach_adapter(adap);
	}
	return 0;
}
```

#### 1-2-1-1-1 i2c_detect()

```c
static int i2c_detect(struct i2c_adapter *adapter, struct i2c_driver *driver)
{
	const unsigned short *address_list;
	struct i2c_client *temp_client;
	int i, err = 0;
    
    /*	return adap->nr; */
	int adap_id = i2c_adapter_id(adapter);

    /**/
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

    /* 申请临时的client */
	temp_client = kzalloc(sizeof(struct i2c_client), GFP_KERNEL);
	temp_client->adapter = adapter;

	for (i = 0; address_list[i] != I2C_CLIENT_END; i += 1) {
		temp_client->addr = address_list[i];
		err = i2c_detect_address(temp_client, driver);
	}
    
	return err;
}

```

#### 1-2-1-1-1-1 i2c_detect_address()

```C
static int i2c_detect_address(struct i2c_client *temp_client,
			      struct i2c_driver *driver)
{
	struct i2c_board_info info;
	struct i2c_adapter *adapter = temp_client->adapter;
	int addr = temp_client->addr;
	int err;

	/* Make sure the address is valid */
	err = i2c_check_7bit_addr_validity_strict(addr);


    /* 如果adapter里已经存在了同地址的client,跳过 */
	if (i2c_check_addr_busy(adapter, addr))
		return 0;

	/* Make sure there is something at this address */
	if (!i2c_default_probe(adapter, addr))
		return 0;

	/* Finally call the custom detection function */
	memset(&info, 0, sizeof(struct i2c_board_info));
	info.addr = addr;
    
    /*调用detect*/
	err = driver->detect(temp_client, &info);

	/* Consistency check */
	if (info.type[0] == '\0') {
	} else {
		struct i2c_client *client;

		/* Detection succeeded, instantiate the device */
		if (adapter->class & I2C_CLASS_DEPRECATED)
		client = i2c_new_device(adapter, &info);
		if (client)
			list_add_tail(&client->detected, &driver->clients);
	}
	return 0;
}
```

### 3 i2c_new_device()

```C
struct i2c_client *
i2c_new_device(struct i2c_adapter *adap, struct i2c_board_info const *info)
{
	struct i2c_client	*client;
	int			status;

	client = kzalloc(sizeof *client, GFP_KERNEL);
	if (!client)
		return NULL;

	client->adapter = adap;

	client->dev.platform_data = info->platform_data;

	if (info->archdata)
		client->dev.archdata = *info->archdata;

	client->flags = info->flags;
	client->addr = info->addr;
	client->irq = info->irq;

	strlcpy(client->name, info->type, sizeof(client->name));

	status = i2c_check_addr_validity(client->addr, client->flags);
	if (status) {
		dev_err(&adap->dev, "Invalid %d-bit I2C address 0x%02hx\n",
			client->flags & I2C_CLIENT_TEN ? 10 : 7, client->addr);
		goto out_err_silent;
	}

	/* Check for address business */
	status = i2c_check_addr_busy(adap, i2c_encode_flags_to_addr(client));
	if (status)
		goto out_err;

	client->dev.parent = &client->adapter->dev;
	client->dev.bus = &i2c_bus_type;
	client->dev.type = &i2c_client_type;
	client->dev.of_node = info->of_node;
	client->dev.fwnode = info->fwnode;

	i2c_dev_set_name(adap, client);
	status = device_register(&client->dev);
	if (status)
		goto out_err;

	dev_dbg(&adap->dev, "client [%s] registered with bus id %s\n",
		client->name, dev_name(&client->dev));

	return client;

out_err:
	dev_err(&adap->dev,
		"Failed to register i2c client %s at 0x%02x (%d)\n",
		client->name, client->addr, status);
out_err_silent:
	kfree(client);
	return NULL;
}
```



### 总结

- 特定芯片驱动中，创建i2c_driver并进行一定的初始化，然后调用i2c_add_driver。

- 再进行总线绑定，尝试进行与device的match，其中有两种方法进行match。
  - 方法1：dt 的 client 搜索匹配。
    - 在i2c_imx.c中，已经进行了dt的解析，构造的 adapter deivce 和client device挂入了i2c bus。
    - 调用i2c bus的match函数，在i2c bus上，取出 client device  进行规则匹配
    - 成功后，device绑定driver，调用i2c bus的probe函数，最终调用dev->driver->probe。
    - 再进行细节就是，特定芯片的寄存器配置，open，读写操作。
    
  - 方法2：设备地址的搜索匹配。
    - 在i2c_driver初始化时，添加了想要进行匹配的设备地址在i2c_driver->address_list。
    
    - 取出i2c bus上的 client device，作用是利用client，就可以把adapter device全部遍历一遍。
    
    - 创建一个临时client device，将address_list的地址一个个取出，利用adapter发送函数，尝试detect是否有该设备地址的设备（前提是 i2c driver 要提供detect函数）。
    
    - 当有产生应答，并且该设备地址没有在adapter创建了i2c client ，则认为检测成功。
    
    - 新创建i2c client并挂入对应的adapter，如果address_list还有其他的，则继续进行。
    
      

​	  
