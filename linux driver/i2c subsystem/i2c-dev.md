### struct i2c_adapter

```C
/*
 * i2c_adapter is the structure used to identify a physical i2c bus along
 * with the access algorithms necessary to access it.
 */
struct i2c_adapter {
	struct module *owner;
	unsigned int class;		  /* classes to allow probing for */
	const struct i2c_algorithm *algo; /* the algorithm to access the bus */
	void *algo_data;

	/* data fields that are valid for all devices	*/
	const struct i2c_lock_operations *lock_ops;
	struct rt_mutex bus_lock;
	struct rt_mutex mux_lock;

	int timeout;			/* in jiffies */
	int retries;
	struct device dev;		/* the adapter device */

    /*第几个adapter*/
	int nr;
	char name[48];
	struct completion dev_released;

	struct mutex userspace_clients_lock;
	struct list_head userspace_clients;

	struct i2c_bus_recovery_info *bus_recovery_info;
	const struct i2c_adapter_quirks *quirks;
};
```

### struct i2c_client

```C
struct i2c_client {
	unsigned short flags;		/* div., see below		*/
	unsigned short addr;		/* chip address - NOTE: 7bit	*/
					/* addresses are stored in the	*/
					/* _LOWER_ 7 bits		*/
	char name[I2C_NAME_SIZE];
	struct i2c_adapter *adapter;	/* the adapter we sit on	*/
	struct device dev;		/* the device structure		*/
	int irq;			/* irq issued by device		*/
	struct list_head detected;
#if IS_ENABLED(CONFIG_I2C_SLAVE)
	i2c_slave_cb_t slave_cb;	/* callback for slave mode	*/
#endif
};
```

### struct i2c_msg

```C
struct i2c_msg {
	__u16 addr;	/* slave address			*/
	__u16 flags;
#define I2C_M_RD		0x0001	/* read data, from slave to master */
					/* I2C_M_RD is guaranteed to be 0x0001! */
#define I2C_M_TEN		0x0010	/* this is a ten bit chip address */
#define I2C_M_RECV_LEN		0x0400	/* length will be first received byte */
#define I2C_M_NO_RD_ACK		0x0800	/* if I2C_FUNC_PROTOCOL_MANGLING */
#define I2C_M_IGNORE_NAK	0x1000	/* if I2C_FUNC_PROTOCOL_MANGLING */
#define I2C_M_REV_DIR_ADDR	0x2000	/* if I2C_FUNC_PROTOCOL_MANGLING */
#define I2C_M_NOSTART		0x4000	/* if I2C_FUNC_NOSTART */
#define I2C_M_STOP		0x8000	/* if I2C_FUNC_PROTOCOL_MANGLING */
	__u16 len;		/* msg length				*/
	__u8 *buf;		/* pointer to msg data			*/
};
```
### struct i2c_rdwr_ioctl_data 

```C
/* This is the structure as used in the I2C_RDWR ioctl call */
struct i2c_rdwr_ioctl_data {
	struct i2c_msg __user *msgs;	/* pointers to i2c_msgs */
	__u32 nmsgs;			/* number of i2c_msgs */
};
```

###  struct i2c_dev

```C
struct i2c_dev {
	struct list_head list;
	struct i2c_adapter *adap;
	struct device *dev;
	struct cdev cdev;
};
```


### 1 i2c_dev_init()

```C
static int __init i2c_dev_init(void)
{
	int res;

    /*
     * #define I2C_MINORS    MINORMASK
     * #define MINORMASK	((1U << MINORBITS) - 1)
     * 注册i2c的设备号域
     */
	res = register_chrdev_region(MKDEV(I2C_MAJOR, 0), I2C_MINORS, "i2c");


    /* 注册 i2c class */
	i2c_dev_class = class_create(THIS_MODULE, "i2c-dev");
	i2c_dev_class->dev_groups = i2c_groups;

	/* Keep track of adapters which will be added or removed later */
    /*注册一个通知块到i2cbus的通知链*/
	res = bus_register_notifier(&i2c_bus_type, &i2cdev_notifier);


    /*
     * 翻译：立即绑定已经存在的adapters(已经在i2c-imx.c的probe被解析)
     * 为每个adapter创建一个i2c_dev,初始化i2c_dev->dev,及绑定设备号，设置fops
     */
	i2c_for_each_dev(NULL, i2cdev_attach_adapter);

	return 0;
}
```

#### 1-1 i2c_for_each_dev()

```C
/*从这里判断，是不是i2c-imx的driver会先被执行*/

int i2c_for_each_dev(void *data, int (*fn)(struct device *, void *))
{
	int res;

	mutex_lock(&core_lock);
	res = bus_for_each_dev(&i2c_bus_type, NULL, data, fn);
	mutex_unlock(&core_lock);

	return res;
}


/*
 * (遍历挂在bus list的devices, 对每个dev都调用@fn，并传参@data 
 *  如果@start非空,则遍历的起点在@start)
 * (每次调用完@fn都检查其返回值，如出现非0值，则直接break循环，并返回该值)
 */
int bus_for_each_dev(struct bus_type *bus, struct device *start,
		     void *data, int (*fn)(struct device *, void *))
{
	struct klist_iter i;
	struct device *dev;
	int error = 0;

	if (!bus || !bus->p)
		return -EINVAL;

	klist_iter_init_node(&bus->p->klist_devices, &i,
			     (start ? &start->p->knode_bus : NULL));
	while ((dev = next_device(&i)) && !error)
        /* 调用了 i2cdev_attach_adapter */
		error = fn(dev, data);
	klist_iter_exit(&i);
	return error;
}
```

#### 1-2 i2cdev_attach_adapter()

```C
static const struct file_operations i2cdev_fops = {
	.owner		= THIS_MODULE,
	.llseek		= no_llseek,
	.read		= i2cdev_read,
	.write		= i2cdev_write,
	.unlocked_ioctl	= i2cdev_ioctl,
	.open		= i2cdev_open,
	.release	= i2cdev_release,
};

/*
 * 在i2c-imx.c中，已经将dts的adapter和client解析和初始化，
 * 并放入了i2c_bus总线的dev_klist。
 */
static int i2cdev_attach_adapter(struct device *dev, void *dummy)
{
	struct i2c_adapter *adap;
	struct i2c_dev *i2c_dev;
	int res;

   
	if (dev->type != &i2c_adapter_type)
		return 0;
    
    /*
     * #define to_i2c_adapter(d) container_of(d, struct i2c_adapter, dev)
     * 获取dev绑定的i2c_adapter，每个在dts的i2c节点，都会有一个adapter
     */
	adap = to_i2c_adapter(dev);

    /*申请 i2c_dev，该结构包含了cdev*/
	i2c_dev = get_free_i2c_dev(adap);
	if (IS_ERR(i2c_dev))
		return PTR_ERR(i2c_dev);
	
    /*初始化cdev, 设置通用的 i2cdev_fops 操作函数*/
	cdev_init(&i2c_dev->cdev, &i2cdev_fops);
	i2c_dev->cdev.owner = THIS_MODULE;
    
    /*
     * cdev绑定1个设备号,i2c的主设备号一致,占用的次设备号由nr决定。
     * adap->nr：对应了第几个i2c_adapter
     */
	res = cdev_add(&i2c_dev->cdev, MKDEV(I2C_MAJOR, adap->nr), 1);
	if (res)
		goto error_cdev;

	/* register this i2c device with the driver core */
    /*初始化dev，并添加到sysfs*/
	i2c_dev->dev = device_create(i2c_dev_class, &adap->dev,
				     MKDEV(I2C_MAJOR, adap->nr), NULL,
				     "i2c-%d", adap->nr);
	if (IS_ERR(i2c_dev->dev)) {
		res = PTR_ERR(i2c_dev->dev);
		goto error;
	}

	pr_debug("i2c-dev: adapter [%s] registered as minor %d\n",
		 adap->name, adap->nr);
	return 0;
error:
	cdev_del(&i2c_dev->cdev);
error_cdev:
	put_i2c_dev(i2c_dev);
	return res;
}
```


#### 1-2-1 get_free_i2c_dev()

```C
static struct i2c_dev *get_free_i2c_dev(struct i2c_adapter *adap)
{
	struct i2c_dev *i2c_dev;

	if (adap->nr >= I2C_MINORS) {
		printk(KERN_ERR "i2c-dev: Out of device minors (%d)\n",
		       adap->nr);
		return ERR_PTR(-ENODEV);
	}

	/*申请 i2c_dev 内存, 主要意义是申请 cdev 内存*/
	i2c_dev = kzalloc(sizeof(*i2c_dev), GFP_KERNEL);
	if (!i2c_dev)
		return ERR_PTR(-ENOMEM);
	i2c_dev->adap = adap;

	spin_lock(&i2c_dev_list_lock);
    /*加入全局链表*/
	list_add_tail(&i2c_dev->list, &i2c_dev_list);
	spin_unlock(&i2c_dev_list_lock);
	return i2c_dev;
}
```

### 2 i2cdev_open()

```C
static int i2cdev_open(struct inode *inode, struct file *file)
{
    /*i2C子系统对应一个major，一个adapter对应一个minor*/
	unsigned int minor = iminor(inode);
	struct i2c_client *client;
	struct i2c_adapter *adap;

    /*
     * 1. i2c_adapter创建后，会被记录在idr，其中nr号与idr唯一对应。
     * 2. 因此idr中存放了一个全局变量i2c_adapter_idr，
     *    可根据nr号也就是minior，去找对应的i2c_adapter。
     */
	adap = i2c_get_adapter(minor);
    
    /*
     * 申请 i2c_client 内存，等待被read write ioctl等函数使用，
     * 因为此module会被应用层打开(如i2c_tools)。
     * client也属于一种device。
     */
	client = kzalloc(sizeof(*client), GFP_KERNEL);

    /*该client命名为 "i2c-dev %d"*/
	snprintf(client->name, I2C_NAME_SIZE, "i2c-dev %d", adap->nr);

    /*记录对应的adapter*/
	client->adapter = adap;
    /*存放在file私有指针，结束会被销毁*/
	file->private_data = client;

	return 0;
}
```

#### 2-1 i2c_get_adapter()

```C
struct i2c_adapter *i2c_get_adapter(int nr)
{
	struct i2c_adapter *adapter;

	mutex_lock(&core_lock);
    /*在i2c_adapter_idr中，根据idr机制，通过nr找到adapter*/
	adapter = idr_find(&i2c_adapter_idr, nr);
	if (!adapter)
		goto exit;

	if (try_module_get(adapter->owner))
		get_device(&adapter->dev);
	else
		adapter = NULL;

 exit:
	mutex_unlock(&core_lock);
	return adapter;
}
```

### 3 i2cdev_ioctl()

```C
static long i2cdev_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    /*取出open时保存的 i2c_client*/
	struct i2c_client *client = file->private_data;
	unsigned long funcs;


    /*命令类型*/
	switch (cmd) {
	case I2C_SLAVE:
	case I2C_SLAVE_FORCE:
		if ((arg > 0x3ff) ||
		    (((client->flags & I2C_M_TEN) == 0) && arg > 0x7f))
			return -EINVAL;
            /*
             * 检查adapter是否有注册的client，但是没有匹配driver
             * 方法：检查client->dev->driver是否为空
             */
		if (cmd == I2C_SLAVE && i2cdev_check_addr(client->adapter, arg))
			return -EBUSY;
		/* REVISIT: address could become busy later */
		client->addr = arg;
		return 0;
            
	case I2C_TENBIT:
		if (arg)
			client->flags |= I2C_M_TEN;
		else
			client->flags &= ~I2C_M_TEN;
		return 0;
            
	case I2C_PEC:
		/*
		 * Setting the PEC flag here won't affect kernel drivers,
		 * which will be using the i2c_client node registered with
		 * the driver model core.  Likewise, when that client has
		 * the PEC flag already set, the i2c-dev driver won't see
		 * (or use) this setting.
		 */
		if (arg)
			client->flags |= I2C_CLIENT_PEC;
		else
			client->flags &= ~I2C_CLIENT_PEC;
		return 0;
            
	case I2C_FUNCS:
		funcs = i2c_get_functionality(client->adapter);
		return put_user(funcs, (unsigned long __user *)arg);
    
	case I2C_RDWR:
		return i2cdev_ioctl_rdwr(client, arg);

	case I2C_SMBUS:
		return i2cdev_ioctl_smbus(client, arg);

	case I2C_RETRIES:
		client->adapter->retries = arg;
		break;
            
	case I2C_TIMEOUT:
		/* For historical reasons, user-space sets the timeout
		 * value in units of 10 ms.
		 */
		client->adapter->timeout = msecs_to_jiffies(arg * 10);
		break;
            
	default:
		/* NOTE:  returning a fault code here could cause trouble
		 * in buggy userspace code.  Some old kernel bugs returned
		 * zero in this case, and userspace code might accidentally
		 * have depended on that bug.
		 */
		return -ENOTTY;
	}
	return 0;
}
```

#### 3-1 i2cdev_ioctl_rdwr()

```C
static noinline int i2cdev_ioctl_rdwr(struct i2c_client *client,
		unsigned long arg)
{
    /*user传入的是i2c_msg数组，及多少个i2c_msg*/
	struct i2c_rdwr_ioctl_data rdwr_arg;
	struct i2c_msg *rdwr_pa;
	u8 __user **data_ptrs;
	int i, res;

    /*
     * 复制指向msg数组首地址，和nmsg(msg数量)，
     * 此时数组的内容还没复制过来，还需进一步操作。
     */
	if (copy_from_user(&rdwr_arg,
			   (struct i2c_rdwr_ioctl_data __user *)arg,
			   sizeof(rdwr_arg)))
		return -EFAULT;

	/* Put an arbitrary limit on the number of messages that can
	 * be sent at once */
    /*I2C_RDWR_IOCTL_MAX_MSGS：42，可发送的数量限制*/
	if (rdwr_arg.nmsgs > I2C_RDWR_IOCTL_MAX_MSGS)
		return -EINVAL;

    /* 将 rdwr_arg.msgs 数组指针的地址值复制出来 */
	rdwr_pa = memdup_user(rdwr_arg.msgs,
			      rdwr_arg.nmsgs * sizeof(struct i2c_msg));
	if (IS_ERR(rdwr_pa))
		return PTR_ERR(rdwr_pa);

    /*申请 nmsgs 个指针，用于保存user的i2c_msg*/
	data_ptrs = kmalloc(rdwr_arg.nmsgs * sizeof(u8 __user *), GFP_KERNEL);
	if (data_ptrs == NULL) {
		kfree(rdwr_pa);
		return -ENOMEM;
	}

	res = 0;
	for (i = 0; i < rdwr_arg.nmsgs; i++) {
		/* Limit the size of the message to a sane amount */
		if (rdwr_pa[i].len > 8192) { /*msg内容长度限制*/
			res = -EINVAL;
			break;
		}
        
		/*记录i2c_msg 地址， 再将数组指针的每个指针保存的地址值内容提取，即msg内容*/
		data_ptrs[i] = (u8 __user *)rdwr_pa[i].buf;
		rdwr_pa[i].buf = memdup_user(data_ptrs[i], rdwr_pa[i].len);
		if (IS_ERR(rdwr_pa[i].buf)) {
			res = PTR_ERR(rdwr_pa[i].buf);
			break;
		}


		if (rdwr_pa[i].flags & I2C_M_RECV_LEN) {
			if (!(rdwr_pa[i].flags & I2C_M_RD) ||
			    rdwr_pa[i].buf[0] < 1 ||
			    rdwr_pa[i].len < rdwr_pa[i].buf[0] +
					     I2C_SMBUS_BLOCK_MAX) {
				res = -EINVAL;
				break;
			}

			rdwr_pa[i].len = rdwr_pa[i].buf[0];
		}
	}
    /*某一步失败，全部释放*/
	if (res < 0) {
		int j;
		for (j = 0; j < i; ++j)
			kfree(rdwr_pa[j].buf);
		kfree(data_ptrs);
		kfree(rdwr_pa);
		return res;
	}

    /* 调用i2c-core.c的函数 */
	res = i2c_transfer(client->adapter, rdwr_pa, rdwr_arg.nmsgs);
	
    while (i-- > 0) {
		if (res >= 0 && (rdwr_pa[i].flags & I2C_M_RD)) {
			if (copy_to_user(data_ptrs[i], rdwr_pa[i].buf,
					 rdwr_pa[i].len))
				res = -EFAULT;
		}
		kfree(rdwr_pa[i].buf);
	}
	kfree(data_ptrs);
	kfree(rdwr_pa);
	return res;
}
```

#### 3-1-1 i2c_transfer()

```C
int i2c_transfer(struct i2c_adapter *adap, struct i2c_msg *msgs, int num)
{
	int ret;

	if (adap->algo->master_xfer) {
#ifdef DEBUG
		for (ret = 0; ret < num; ret++) {
			dev_dbg(&adap->dev,
				"master_xfer[%d] %c, addr=0x%02x, len=%d%s\n",
				ret, (msgs[ret].flags & I2C_M_RD) ? 'R' : 'W',
				msgs[ret].addr, msgs[ret].len,
				(msgs[ret].flags & I2C_M_RECV_LEN) ? "+" : "");
		}
#endif

		if (in_atomic() || irqs_disabled()) {
			ret = i2c_trylock_bus(adap, I2C_LOCK_SEGMENT);
			if (!ret)
				/* I2C activity is ongoing. */
				return -EAGAIN;
		} else {
			i2c_lock_bus(adap, I2C_LOCK_SEGMENT);
		}

		ret = __i2c_transfer(adap, msgs, num);
		i2c_unlock_bus(adap, I2C_LOCK_SEGMENT);

		return ret;
	} else {
		dev_dbg(&adap->dev, "I2C level transfers not supported\n");
		return -EOPNOTSUPP;
	}
}



int __i2c_transfer(struct i2c_adapter *adap, struct i2c_msg *msgs, int num)
{
	unsigned long orig_jiffies;
	int ret, try;

	if (adap->quirks && i2c_check_for_quirks(adap, msgs, num))
		return -EOPNOTSUPP;

	/* i2c_trace_msg gets enabled when tracepoint i2c_transfer gets
	 * enabled.  This is an efficient way of keeping the for-loop from
	 * being executed when not needed.
	 */
	if (static_key_false(&i2c_trace_msg)) {
		int i;
		for (i = 0; i < num; i++)
			if (msgs[i].flags & I2C_M_RD)
				trace_i2c_read(adap, &msgs[i], i);
			else
				trace_i2c_write(adap, &msgs[i], i);
	}

	/* Retry automatically on arbitration loss */
	orig_jiffies = jiffies;
	for (ret = 0, try = 0; try <= adap->retries; try++) {
		ret = adap->algo->master_xfer(adap, msgs, num);
		if (ret != -EAGAIN)
			break;
		if (time_after(jiffies, orig_jiffies + adap->timeout))
			break;
	}

	if (static_key_false(&i2c_trace_msg)) {
		int i;
		for (i = 0; i < ret; i++)
			if (msgs[i].flags & I2C_M_RD)
				trace_i2c_reply(adap, &msgs[i], i);
		trace_i2c_result(adap, i, ret);
	}

	return ret;
}
```

#### 3-2 i2cdev_ioctl_smbus()

```
static noinline int i2cdev_ioctl_smbus(struct i2c_client *client,
		unsigned long arg)
{
	struct i2c_smbus_ioctl_data data_arg;
	union i2c_smbus_data temp = {};
	int datasize, res;

	if (copy_from_user(&data_arg,
			   (struct i2c_smbus_ioctl_data __user *) arg,
			   sizeof(struct i2c_smbus_ioctl_data)))
		return -EFAULT;
	if ((data_arg.size != I2C_SMBUS_BYTE) &&
	    (data_arg.size != I2C_SMBUS_QUICK) &&
	    (data_arg.size != I2C_SMBUS_BYTE_DATA) &&
	    (data_arg.size != I2C_SMBUS_WORD_DATA) &&
	    (data_arg.size != I2C_SMBUS_PROC_CALL) &&
	    (data_arg.size != I2C_SMBUS_BLOCK_DATA) &&
	    (data_arg.size != I2C_SMBUS_I2C_BLOCK_BROKEN) &&
	    (data_arg.size != I2C_SMBUS_I2C_BLOCK_DATA) &&
	    (data_arg.size != I2C_SMBUS_BLOCK_PROC_CALL)) {
		dev_dbg(&client->adapter->dev,
			"size out of range (%x) in ioctl I2C_SMBUS.\n",
			data_arg.size);
		return -EINVAL;
	}
	/* Note that I2C_SMBUS_READ and I2C_SMBUS_WRITE are 0 and 1,
	   so the check is valid if size==I2C_SMBUS_QUICK too. */
	if ((data_arg.read_write != I2C_SMBUS_READ) &&
	    (data_arg.read_write != I2C_SMBUS_WRITE)) {
		dev_dbg(&client->adapter->dev,
			"read_write out of range (%x) in ioctl I2C_SMBUS.\n",
			data_arg.read_write);
		return -EINVAL;
	}

	/* Note that command values are always valid! */

	if ((data_arg.size == I2C_SMBUS_QUICK) ||
	    ((data_arg.size == I2C_SMBUS_BYTE) &&
	    (data_arg.read_write == I2C_SMBUS_WRITE)))
		/* These are special: we do not use data */
		return i2c_smbus_xfer(client->adapter, client->addr,
				      client->flags, data_arg.read_write,
				      data_arg.command, data_arg.size, NULL);

	if (data_arg.data == NULL) {
		dev_dbg(&client->adapter->dev,
			"data is NULL pointer in ioctl I2C_SMBUS.\n");
		return -EINVAL;
	}

	if ((data_arg.size == I2C_SMBUS_BYTE_DATA) ||
	    (data_arg.size == I2C_SMBUS_BYTE))
		datasize = sizeof(data_arg.data->byte);
	else if ((data_arg.size == I2C_SMBUS_WORD_DATA) ||
		 (data_arg.size == I2C_SMBUS_PROC_CALL))
		datasize = sizeof(data_arg.data->word);
	else /* size == smbus block, i2c block, or block proc. call */
		datasize = sizeof(data_arg.data->block);

	if ((data_arg.size == I2C_SMBUS_PROC_CALL) ||
	    (data_arg.size == I2C_SMBUS_BLOCK_PROC_CALL) ||
	    (data_arg.size == I2C_SMBUS_I2C_BLOCK_DATA) ||
	    (data_arg.read_write == I2C_SMBUS_WRITE)) {
		if (copy_from_user(&temp, data_arg.data, datasize))
			return -EFAULT;
	}
	if (data_arg.size == I2C_SMBUS_I2C_BLOCK_BROKEN) {
		/* Convert old I2C block commands to the new
		   convention. This preserves binary compatibility. */
		data_arg.size = I2C_SMBUS_I2C_BLOCK_DATA;
		if (data_arg.read_write == I2C_SMBUS_READ)
			temp.block[0] = I2C_SMBUS_BLOCK_MAX;
	}
	res = i2c_smbus_xfer(client->adapter, client->addr, client->flags,
	      data_arg.read_write, data_arg.command, data_arg.size, &temp);
	if (!res && ((data_arg.size == I2C_SMBUS_PROC_CALL) ||
		     (data_arg.size == I2C_SMBUS_BLOCK_PROC_CALL) ||
		     (data_arg.read_write == I2C_SMBUS_READ))) {
		if (copy_to_user(data_arg.data, &temp, datasize))
			return -EFAULT;
	}
	return res;
}

```



### 总结

- i2c-dev.c的init中，查找i2c_bus总线的adapter deivce，并生成对应的i2c_dev，主要作用是提供cdev的fops操作函数。

- client有6种形式生成

  - 第一种：由dt描述，在i2c-imx.c的 platform driver。

    - 对 adapter node 进行 match。

    - 进行probe。

      - 解析dt，生成多个adapter device。

      - 在adapter device 子节点下，生成多个client device。

      - 并将device全部挂载在i2c bus。

    - 生成的client device会与特定的drvier进行match。
    
  - 第二种：
    ```C
    // 创建一个i2c_client, .name = "eeprom", .addr=0x50, .adapter是i2c-3
    # echo eeprom 0x50 > /sys/bus/i2c/devices/i2c-3/new_device
      
    // 删除一个i2c_client
    # echo 0x50 > /sys/bus/i2c/devices/i2c-3/delete_device
    
    //我怀疑这种形式也是利用第三种方法实现!!
    
  - 第三种：
  
    - 应用层open(”/dev/i2c-0”)，即打开一个adapter device，open会分配一个临时的i2c_client。并使用ioctl配置client的设备地址，再使用ioctl发起传输。
  
  - 第四种：
     
     - i2c_driver提供了detect()函数和设备地址address_list，利用i2c_detect来匹配。
     
  - 第五种:
  
     - 由i2c-dev.c的open函数生成，并使用ioctl配置client的设备地址，发起传输。
  
  - 第六种：
  
     - i2c_new_device:不管设备存不存在，直接生成i2c client
     - i2c_new_probed_device：确认设备存在才会生成i2c client

