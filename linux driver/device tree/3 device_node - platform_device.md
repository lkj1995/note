### struct platform_device

```C
struct pdev_archdata {
#ifdef CONFIG_ARCH_OMAP
        struct omap_device *od;
#endif
};

struct platform_device {
	const char	*name;
	int		id;
	bool		id_auto;
	struct device	dev;
	u64		platform_dma_mask;
	struct device_dma_parameters dma_parms;
	u32		num_resources;
	struct resource	*resource;

	const struct platform_device_id	*id_entry;
	char *driver_override; /* Driver name to force a match */

	/* MFD cell pointer */
	struct mfd_cell *mfd_cell;

	/* arch specific additions */
	struct pdev_archdata	archdata;
};
```
### struct of_device_id

```c
struct of_device_id {
        char    name[32];
        char    type[32];
        char    compatible[128];
        const void *data;
};
```

### 1 of_platform_default_populate_init()

```C
/* 
 * 把该函数地址放入 .initcall3s.init 段中
 * 在start_kernel()进入此段，将段中的函数按顺序全部进行调用
 */           
static int __init of_platform_default_populate_init(void)
{
	struct device_node *node;
                
	/* Populate everything else. */
	of_platform_default_populate(NULL, NULL, NULL);

	return 0;
}
arch_initcall_sync(of_platform_default_populate_init);

#define arch_initcall_sync(fn)         
			__define_initcall(fn, 3s)

#define __define_initcall(fn, id)  
			___define_initcall(fn, id, .initcall##id)

#define ___define_initcall(fn, id, __sec) \
        static initcall_t __initcall_##fn##id __used \
                __attribute__((__section__(#__sec ".init"))) = fn;
```

### 1-1 of_platform_default_populate()

```C
int of_platform_default_populate(struct device_node *root,
				 const struct of_dev_auxdata *lookup,
				 struct device *parent)
{
	return of_platform_populate(root, of_default_bus_match_table, 
                                lookup, parent);
}

/*传入的参数(NULL, of_default_bus_match_table, NULL, NULL)*/
int of_platform_populate(struct device_node *root,
			const struct of_device_id *matches,
			const struct of_dev_auxdata *lookup,
			struct device *parent)
{   
	struct device_node *child;
	int rc = 0;
    
	/*找到of_root根节点*/
	root = root ? of_node_get(root) : of_find_node_by_path("/");

	/*将每个node进行处理(当做总线node，如i2c)*/
	for_each_child_of_node(root, child) {
		rc = of_platform_bus_create(child, matches, lookup, parent, true);
	}
    
	return rc;    
}
```

### 1-1-1 for_each_child_of_node()

```C
#define for_each_child_of_node(parent, child) \
	for (child = of_get_next_child(parent, NULL); child != NULL; \
	     child = of_get_next_child(parent, child))


/**
 *	of_get_next_child - Iterate a node childs
 *	@node:	parent node
 *	@prev:	previous child of the parent node, or NULL to get first
 *
 *	Returns a node pointer with refcount incremented, use of_node_put() on
 *	it when done. Returns NULL when prev is the last child. Decrements the
 *	refcount of prev.
 */
struct device_node *of_get_next_child(const struct device_node *node,
	struct device_node *prev)
{
	struct device_node *next;
	unsigned long flags;
    
     /* 返回兄弟节点或子节点，遍历所有的节点 */     
	next = __of_get_next_child(node, prev);
    
	return next;
}
```

### 1-1-2 of_platform_bus_create()

```C
/* 传入的参数 (根node的子node, of_default_bus_match_table, null, null, 1) */

static int of_platform_bus_create(struct device_node *bus,
				  const struct of_device_id *matches,
				  const struct of_dev_auxdata *lookup,
				  struct device *parent, bool strict)
{
	const struct of_dev_auxdata *auxdata;
	struct device_node *child;
	struct platform_device *dev;
	const char *bus_id = NULL;
	void *platform_data = NULL;
	int rc = 0;


    /*
     * 创建 platform_device 的前提
     * 1. 存在compatible属性 
     * 2. 节点和子节点都含有compatible属性，而且存在
     *    "simple-bus", "simple-mfd", "isa", "arm,amba-bus"
     */
	if (strict && (!of_get_property(bus, "compatible", NULL))) {
		return 0;
	}
    
    
	dev = of_platform_device_create_pdata(bus, bus_id, 
                                          platform_data, parent);
    if (!dev || !of_match_node(matches, bus))
		return 0;

    
	for_each_child_of_node(bus, child) {     
        /*递归*/
		rc = of_platform_bus_create(child, matches, lookup, 
                                      &dev->dev, strict);
	}
	return rc;
}
```

### 1-1-2-1 of_platform_device_create_pdata()

```C
struct bus_type platform_bus_type = {
        .name           = "platform",
        .dev_groups     = platform_dev_groups,
        .match          = platform_match,
        .uevent         = platform_uevent,
        .dma_configure  = platform_dma_configure,
        .pm             = &platform_dev_pm_ops,
};


/*传入参数（某个节点node, NULL, NULL, NULL）*/
static struct platform_device *of_platform_device_create_pdata(
					struct device_node *np,
					const char *bus_id,
					void *platform_data,
					struct device *parent)
{
	struct platform_device *dev;
    
     /* 
      * 检查status属性，用于确认该node是否使用    
      * 1. "okey": 返回1，并设置flag（OF_POPULATED）
      * 2. 其他：返回0
      */     
	if (!of_device_is_available(np) ||
	    of_node_test_and_set_flag(np, OF_POPULATED))
		return NULL;

    /*申请platform_device,将属性irq,mem,io转换为resource*/
	dev = of_device_alloc(np, bus_id, parent);
	if (!dev)
		goto err_clear_flag;

    /*设置platform_device*/
	dev->dev.coherent_dma_mask = DMA_BIT_MASK(32);
	if (!dev->dev.dma_mask)
		dev->dev.dma_mask = &dev->dev.coherent_dma_mask;
    
    /*挂载到总线"platform"*/
	dev->dev.bus = &platform_bus_type;
	dev->dev.platform_data = platform_data;
	of_msi_configure(&dev->dev, dev->dev.of_node);

    
    /* 注册，调用device_add() */
	if (of_device_add(dev) != 0) {
		platform_device_put(dev);
		goto err_clear_flag;
	}

	return dev;
}
```

### 1-1-2-1-1 of_device_alloc

```C
/**
 * of_device_alloc - Allocate and initialize an of_device
 * @np: device node to assign to device
 * @bus_id: Name to assign to the device.  May be null to use default name.
 * @parent: Parent device.
 */
struct platform_device *of_device_alloc(struct device_node *np,
				  const char *bus_id,
				  struct device *parent)
{
	struct platform_device *dev;
	int rc, i, num_reg = 0, num_irq;
	struct resource *res, temp_res;
    
    /*申请 platform_device结构体 + name 大小的内存，并初始化*/
	dev = platform_device_alloc("", PLATFORM_DEVID_NONE);
  
    /*计算所需resources的数量*/
	while (of_address_to_resource(np, num_reg, &temp_res) == 0)
		num_reg++;
    
    /*计算该node中断数量*/
	num_irq = of_irq_count(np);


	if (num_irq || num_reg) {
        
        /*分配resource数组,存放"reg"属性和"interrupts"属性的内容*/
		res = kcalloc(num_irq + num_reg, sizeof(*res), GFP_KERNEL);
     
        /*记录数量和数组地址*/
		dev->num_resources = num_reg + num_irq;
		dev->resource = res;
        
		for (i = 0; i < num_reg; i++, res++) {
            /*把"reg"转换成resource*/
			rc = of_address_to_resource(np, i, res);
			WARN_ON(rc);
		}
        
        /*把"interrupts"转换成resource*/
		if (of_irq_to_resource_table(np, res, num_irq) != num_irq)
			pr_debug("not all legacy IRQ resources mapped for %pOFn\n",
				 np);
	}
    
    /* platform_device和device_node建立关系 */
	dev->dev.of_node = of_node_get(np); 
	dev->dev.fwnode = &np->fwnode;
    
	return dev;
}
```

### 总结

- 系统启动后，会对device_node转换成platform_device，在of_root节点下的node，都认为是总线node，会挂载在”platform”总线下。
  - 根节点的子节点的compatible内容："simple-bus", "simple-mfd", "isa", "arm,amba-bus"，并且孙node也有compatible属性，于是孙node会被转换成platform_device。
  - 如i2c节点，子节点不会生成platform_device，子node会被父节点adapter，生成i2c_client。
  - 因此子系统全部生成了platform_device,子系统下的子node，由子系统的platfrom_driver去构建生成具体的deivce。
  - “reg” ，“inttrupts” 属性会被转换成resource，保存在platform_device。
  - 如需要使用其他的属性，可通过platform_device->dev->of_node节点去匹配。



