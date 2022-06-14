### 数据结构

```C
struct cdev {
        struct kobject kobj;
        struct module *owner; //THIS_MODULE
        const struct file_operations *ops;
        struct list_head list; //字符链表
        dev_t dev; //设备号
        unsigned int count;//同主设备号下，次设备号的数量
} __randomize_layout;
```



```C
/**
 * cdev_init() - initialize a cdev structure
 * @cdev: the structure to initialize
 * @fops: the file_operations for this device
 *
 * Initializes @cdev, remembering @fops, making it ready to add to the
 * system with cdev_add().
 */
void cdev_init(struct cdev *cdev, const struct file_operations *fops)
{
	memset(cdev, 0, sizeof *cdev);
	INIT_LIST_HEAD(&cdev->list);/*初始化链表，指向自身*/
	kobject_init(&cdev->kobj, &ktype_cdev_default);/*暂时未学*/
	cdev->ops = fops;/*记录匹配cdev与fops的实际驱动操作处理*/
}
```



```C
/**
 * cdev_add() - add a char device to the system
 * @p: the cdev structure for the device
 * @dev: the first device number for which this device is responsible
 * @count: the number of consecutive minor numbers corresponding to this
 *         device
 *
 * cdev_add() adds the device represented by @p to the system, making it
 * live immediately.  A negative error code is returned on failure.
 */
int cdev_add(struct cdev *p, dev_t dev, unsigned count)
{
	int error;

	p->dev = dev;/*记录设备号*/
	p->count = count;/*记录次设备号数量*/

	if (WARN_ON(dev == WHITEOUT_DEV))
		return -EBUSY;

	error = kobj_map(cdev_map, dev, count, NULL,
			 exact_match, exact_lock, p);/*主要操作，解析如下*/
	if (error)
		return error;

	kobject_get(p->kobj.parent);/*增加对象的引用计数*/

	return 0;
}

```

### 类似chrdevs的数据结构处理

```c
struct kobj_map {
	struct probe {
		struct probe *next;
		dev_t dev;
		unsigned long range;
		struct module *owner;
		kobj_probe_t *get;
		int (*lock)(dev_t, void *);
		void *data;
	} *probes[255];
	struct mutex *lock;
};
```

```C
int kobj_map(struct kobj_map *domain, dev_t dev, unsigned long range,
	     struct module *module, kobj_probe_t *probe,
	     int (*lock)(dev_t, void *), void *data)
{
    /*计算需要多少个主设备号才能存下次设备号的数量，一般来说这个n = 1*/
	unsigned n = MAJOR(dev + range - 1) - MAJOR(dev) + 1;
	unsigned index = MAJOR(dev);
	unsigned i;
	struct probe *p;

	if (n > 255)
		n = 255;
    /*申请n个probe结构体内存*/
	p = kmalloc_array(n, sizeof(struct probe), GFP_KERNEL);
	if (p == NULL)
		return -ENOMEM;

	for (i = 0; i < n; i++, p++) {
		p->owner = module; /*null*/
		p->get = probe;
		p->lock = lock;
		p->dev = dev;  /*设备号*/
		p->range = range;/*数量*/
		p->data = data; /*保存probe对应的cdev*/
	}
	mutex_lock(domain->lock);
	for (i = 0, p -= n; i < n; i++, p++, index++) {
        /*hash处理，根据主设备号找到合适的数组下标*/
		struct probe **s = &domain->probes[index % 255]; 
        /*根据次设备号数量，按顺序插入*/
		while (*s && (*s)->range < range)  
			s = &(*s)->next;
		p->next = *s;
		*s = p;
	}
	mutex_unlock(domain->lock);
	return 0;
}
```

