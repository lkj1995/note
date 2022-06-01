### of_platform_default_populate_init

```C
arch_initcall_sync(of_platform_default_populate_init);

#define arch_initcall_sync(fn)          __define_initcall(fn, 3s)

#define __define_initcall(fn, id)   ___define_initcall(fn, id, .initcall##id)

#define ___define_initcall(fn, id, __sec) \
        static initcall_t __initcall_##fn##id __used \
                __attribute__((__section__(#__sec ".init"))) = fn;
/* 
 * 把该函数地址放入了 .initcall3s.init 段中
 * 在start_kernel中，会进入此段，将段中的函数按顺序全部进行调用
 */

static int __init of_platform_default_populate_init(void)
{
	struct device_node *node;

	device_links_supplier_sync_state_pause();

	if (!of_have_populated_dt()) /*检查of_root是否为空*/
		return -ENODEV;

	/*
	 * Handle certain compatibles explicitly, since we don't want to create
	 * platform_devices for every node in /reserved-memory with a
	 * "compatible",
	 */
	for_each_matching_node(node, reserved_mem_matches)
		of_platform_device_create(node, NULL, NULL);

	node = of_find_node_by_path("/firmware");
	if (node) {
		of_platform_populate(node, NULL, NULL, NULL);
		of_node_put(node);
	}

	/* Populate everything else. */
	fw_devlink_pause();
	of_platform_default_populate(NULL, NULL, NULL);
	fw_devlink_resume();

	return 0;
}
```



### of_have_populated_dt

```C
static inline bool of_have_populated_dt(void)
{
	return of_root != NULL;
}
```



### reserved_mem_matches

```c
static const struct of_device_id reserved_mem_matches[] = {
	{ .compatible = "qcom,rmtfs-mem" },
	{ .compatible = "qcom,cmd-db" },
	{ .compatible = "ramoops" },
	{}
};
```

### of_device_id

```c
/*
 * Struct used for matching a device
 */
struct of_device_id {
        char    name[32];
        char    type[32];
        char    compatible[128];
        const void *data;
};
```



### for_each_matching_node

```C
#define for_each_matching_node(dn, matches) \
	for (dn = of_find_matching_node(NULL, matches); dn; \
	     dn = of_find_matching_node(dn, matches))
	     
	     
static inline struct device_node *of_find_matching_node(
	struct device_node *from,
	const struct of_device_id *matches)
{
	return of_find_matching_node_and_match(from, matches, NULL);
}
```

### of_find_matching_node_and_match

```C
/**
 *	of_find_matching_node_and_match - Find a node based on an of_device_id
 *					  match table.
 *	@from:		The node to start searching from or NULL, the node
 *			you pass will not be searched, only the next one
 *			will; typically, you pass what the previous call
 *			returned. of_node_put() will be called on it
 *	@matches:	array of of device match structures to search in
 *	@match		Updated to point at the matches entry which matched
 *
 *	Returns a node pointer with refcount incremented, use
 *	of_node_put() on it when done.
 */
struct device_node *of_find_matching_node_and_match(struct device_node *from,
					const struct of_device_id *matches,
					const struct of_device_id **match)
{
	struct device_node *np;
	const struct of_device_id *m;
	unsigned long flags;

	if (match)
		*match = NULL;

	raw_spin_lock_irqsave(&devtree_lock, flags);
	for_each_of_allnodes_from(from, np) {
		m = __of_match_node(matches, np);
		if (m && of_node_get(np)) {
			if (match)
				*match = m;
			break;
		}
	}
	of_node_put(from);
	raw_spin_unlock_irqrestore(&devtree_lock, flags);
	return np;
}
```

### of_default_bus_match_table

```C
/*当节点存在这些属性，则可以转换成platform_device*/
const struct of_device_id of_default_bus_match_table[] = { 
	{ .compatible = "simple-bus", },
	{ .compatible = "simple-mfd", },
	{ .compatible = "isa", },
#ifdef CONFIG_ARM_AMBA
	{ .compatible = "arm,amba-bus", },
#endif /* CONFIG_ARM_AMBA */
	{} /* Empty terminated list */
};
```

### of_skipped_node_table

```C
static const struct of_device_id of_skipped_node_table[] = {
	{ .compatible = "operating-points-v2", },
	{} /* Empty terminated list */
};
```



### of_platform_default_populate

```C
int of_platform_default_populate(struct device_node *root,
				 const struct of_dev_auxdata *lookup,
				 struct device *parent)
{
	return of_platform_populate(root, of_default_bus_match_table, lookup,
				    parent);
}
```

### of_platform_populate

```C
/**
 * of_platform_populate() - Populate platform_devices from device tree data
 * @root: parent of the first level to probe or NULL for the root of the tree
 * @matches: match table, NULL to use the default
 * @lookup: auxdata table for matching id and platform_data with device nodes
 * @parent: parent to hook devices from, NULL for toplevel
 *
 * Similar to of_platform_bus_probe(), this function walks the device tree
 * and creates devices from nodes.  It differs in that it follows the modern
 * convention of requiring all device nodes to have a 'compatible' property,
 * and it is suitable for creating devices which are children of the root
 * node (of_platform_bus_probe will only create children of the root which
 * are selected by the @matches argument).
 *
 * New board support should be using this function instead of
 * of_platform_bus_probe().
 *
 * Returns 0 on success, < 0 on failure.
 */
/*传入的参数(NULL, of_default_bus_match_table, NULL, NULL)*/
int of_platform_populate(struct device_node *root,
			const struct of_device_id *matches,
			const struct of_dev_auxdata *lookup,
			struct device *parent)
{
	struct device_node *child;
	int rc = 0;
    
    /*输入参数为NULL，找到of_root*/
	root = root ? of_node_get(root) : of_find_node_by_path("/");
	if (!root)
		return -EINVAL;

	pr_debug("%s()\n", __func__);
	pr_debug(" starting at: %pOF\n", root);

	device_links_supplier_sync_state_pause();
    
    /*把从of_root开始的每个节点取出来赋值给child*/
	for_each_child_of_node(root, child) {
		rc = of_platform_bus_create(child, matches, lookup, parent, true);
		if (rc) {
			of_node_put(child);
			break;
		}
	}
	device_links_supplier_sync_state_resume();

	of_node_set_flag(root, OF_POPULATED_BUS);

	of_node_put(root);
	return rc;
}
```

###  of_find_node_by_path

```C
static inline struct device_node *of_find_node_by_path(const char *path)
{
	return of_find_node_opts_by_path(path, NULL);
}
```

### of_find_node_opts_by_path

```C
/**
 *	of_find_node_opts_by_path - Find a node matching a full OF path
 *	@path: Either the full path to match, or if the path does not
 *	       start with '/', the name of a property of the /aliases
 *	       node (an alias).  In the case of an alias, the node
 *	       matching the alias' value will be returned.
 *	@opts: Address of a pointer into which to store the start of
 *	       an options string appended to the end of the path with
 *	       a ':' separator.
 *
 *	Valid paths:
 *		/foo/bar	Full path
 *		foo		Valid alias
 *		foo/bar		Valid alias + relative path
 *
 *	Returns a node pointer with refcount incremented, use
 *	of_node_put() on it when done.
 */
struct device_node *of_find_node_opts_by_path(const char *path, const char **opts)
{
    /*其余不重要*/
    
    /*path等于"/"，增加of_root的kobj引用计数，并返回of_root地址*/
	if (strcmp(path, "/") == 0)
		return of_node_get(of_root);

    /*其余不重要*/
}
```

### for_each_child_of_node

```C
#define for_each_child_of_node(parent, child) \
	for (child = of_get_next_child(parent, NULL); child != NULL; \
	     child = of_get_next_child(parent, child))
```

### of_get_next_child

```c
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

	raw_spin_lock_irqsave(&devtree_lock, flags);
    
    /*返回下一个子节点并增加引用计数，减少prev节点的引用计数*/
	next = __of_get_next_child(node, prev);
    
	raw_spin_unlock_irqrestore(&devtree_lock, flags);
	return next;
}
```

### __of_get_next_child

```C
static struct device_node *__of_get_next_child(const struct device_node *node,
						struct device_node *prev)
{
	struct device_node *next;

	if (!node)
		return NULL;
    
    /*
     * 如果传入空，说明无兄弟节点，则获取其子节点，否则获取该节点的兄弟节点
     * 此时可遍历了所有的节点
     */
	next = prev ? prev->sibling : node->child;
   
	for (; next; next = next->sibling)
		if (of_node_get(next))
			break;
	of_node_put(prev);
	return next;
}
```

### of_platform_bus_create

```C
/**
 * of_platform_bus_create() - Create a device for a node and its children.
 * @bus: device node of the bus to instantiate
 * @matches: match table for bus nodes
 * @lookup: auxdata table for matching id and platform_data with device nodes
 * @parent: parent for new device, or NULL for top level.
 * @strict: require compatible property
 *
 * Creates a platform_device for the provided device_node, and optionally
 * recursively create devices for all the child nodes.
 */
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

	/* Make sure it has a compatible property */
    /*
     * 创建 platform_device 的前提
     * 1. 存在compatible属性 
     * 2. 节点和子节点都含有compatible属性，而且存在
     *    "simple-bus","simple-mfd","isa","arm,amba-bus"
     *    此时子节点可创建 platform_device
     */
	if (strict && (!of_get_property(bus, "compatible", NULL))) {
		pr_debug("%s() - skipping %pOF, no compatible prop\n",
			 __func__, bus);
		return 0;
	}

	/* Skip nodes for which we don't want to create devices */
    /*跳过compatible属性为operating-points-v2的node*/
	if (unlikely(of_match_node(of_skipped_node_table, bus))) {
		pr_debug("%s() - skipping %pOF node\n", __func__, bus);
		return 0;
	}
    
    /*检查flag，如果该device_node已生成platform_device,不能重复生成，退出*/
	if (of_node_check_flag(bus, OF_POPULATED_BUS)) {
		pr_debug("%s() - skipping %pOF, already populated\n",
			__func__, bus);
		return 0;
	}

	auxdata = of_dev_lookup(lookup, bus);/*lookup为NULL，返回0*/
	if (auxdata) {/*不执行*/
		bus_id = auxdata->name;
		platform_data = auxdata->platform_data;
	}
    
    /*检查是否存在compatible属性，并且属性内容为arm,primecell*/
	if (of_device_is_compatible(bus, "arm,primecell")) {
		/*
		 * Don't return an error here to keep compatibility with older
		 * device tree files.
		 */
		of_amba_device_create(bus, bus_id, platform_data, parent);
		return 0;
	}

	dev = of_platform_device_create_pdata(bus, bus_id, platform_data, parent);
	if (!dev || !of_match_node(matches, bus))
		return 0;

	for_each_child_of_node(bus, child) {
		pr_debug("   create child: %pOF\n", child);
		rc = of_platform_bus_create(child, matches, lookup, &dev->dev, strict);
		if (rc) {
			of_node_put(child);
			break;
		}
	}
	of_node_set_flag(bus, OF_POPULATED_BUS);
	return rc;
}
```

### of_platform_device_create_pdata

```C
/**
 * of_platform_device_create_pdata - Alloc, initialize and register an of_device
 * @np: pointer to node to create device for
 * @bus_id: name to assign device
 * @platform_data: pointer to populate platform_data pointer with
 * @parent: Linux device model parent device.
 *
 * Returns pointer to created platform device, or NULL if a device was not
 * registered.  Unavailable devices will not get registered.
 */
/*传入参数（某个节点node, NULL, NULL, NULL）*/
static struct platform_device *of_platform_device_create_pdata(
					struct device_node *np,
					const char *bus_id,
					void *platform_data,
					struct device *parent)
{
	struct platform_device *dev;
    
    /*检查node的status属性，内容是否为okey*/
	if (!of_device_is_available(np) ||
	    of_node_test_and_set_flag(np, OF_POPULATED))/*添加OF_POPULATED标志*/
		return NULL;

    /*创建一个platform_device,初始化，并与device_node建立关系*/
	dev = of_device_alloc(np, bus_id, parent);
	if (!dev)
		goto err_clear_flag;

	dev->dev.coherent_dma_mask = DMA_BIT_MASK(32);
	if (!dev->dev.dma_mask)
		dev->dev.dma_mask = &dev->dev.coherent_dma_mask;
	dev->dev.bus = &platform_bus_type;
	dev->dev.platform_data = platform_data;
	of_msi_configure(&dev->dev, dev->dev.of_node);

	if (of_device_add(dev) != 0) {/*继续研究这个函数*/
		platform_device_put(dev);
		goto err_clear_flag;
	}

	return dev;

err_clear_flag:
	of_node_clear_flag(np, OF_POPULATED);
	return NULL;
}
```

### of_device_is_available

```C
/**
 *  of_device_is_available - check if a device is available for use
 *
 *  @device: Node to check for availability
 *
 *  Returns true if the status property is absent or set to "okay" or "ok",
 *  false otherwise
 */
bool of_device_is_available(const struct device_node *device)
{
	unsigned long flags;
	bool res;

	raw_spin_lock_irqsave(&devtree_lock, flags);
	res = __of_device_is_available(device);
	raw_spin_unlock_irqrestore(&devtree_lock, flags);
	return res;

}
```

### __of_device_is_available

```C
/**
 *  __of_device_is_available - check if a device is available for use
 *
 *  @device: Node to check for availability, with locks already held
 *
 *  Returns true if the status property is absent or set to "okay" or "ok",
 *  false otherwise
 */
static bool __of_device_is_available(const struct device_node *device)
{
	const char *status;
	int statlen;

	if (!device)
		return false;

	status = __of_get_property(device, "status", &statlen);
	if (status == NULL)
		return true;

	if (statlen > 0) {
		if (!strcmp(status, "okay") || !strcmp(status, "ok"))
			return true;
	}

	return false;
}
```

### of_device_alloc

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
    
    /*创建 platform_device结构体 + name 大小的内存，并初始化*/
	dev = platform_device_alloc("", PLATFORM_DEVID_NONE);
	if (!dev)
		return NULL;

	/* count the io and irq resources */
	while (of_address_to_resource(np, num_reg, &temp_res) == 0)
		num_reg++;
	num_irq = of_irq_count(np);

	/* Populate the resource table */
	if (num_irq || num_reg) {
		res = kcalloc(num_irq + num_reg, sizeof(*res), GFP_KERNEL);
		if (!res) {
			platform_device_put(dev);
			return NULL;
		}

		dev->num_resources = num_reg + num_irq;
		dev->resource = res;
		for (i = 0; i < num_reg; i++, res++) {
			rc = of_address_to_resource(np, i, res);
			WARN_ON(rc);
		}
		if (of_irq_to_resource_table(np, res, num_irq) != num_irq)
			pr_debug("not all legacy IRQ resources mapped for %pOFn\n",
				 np);
	}

	dev->dev.of_node = of_node_get(np); /*platform_device和vice_node建立关系*/
	dev->dev.fwnode = &np->fwnode;
    
   /*
    * struct device platform_bus = {
    *    .init_name      = "platform",
    * };
    */  
	dev->dev.parent = parent ? : &platform_bus;

	if (bus_id)
		dev_set_name(&dev->dev, "%s", bus_id);/*参数设置名字，kobj*/
	else
		of_device_make_bus_id(&dev->dev);/*根据reg参数，生成一个名字，kobj*/

	return dev;
}
```

### platform_device_alloc

```c
/**
 * platform_device_alloc - create a platform device
 * 创建一个platform_device结构
 * @name: base name of the device we're adding
 * @id: instance id
 *
 * Create a platform device object which can have other objects attached
 * to it, and which will have attached objects freed when it is released.
 */
struct platform_device *platform_device_alloc(const char *name, int id)
{
	struct platform_object *pa;

	pa = kzalloc(sizeof(*pa) + strlen(name) + 1, GFP_KERNEL);
	if (pa) {
		strcpy(pa->name, name);/*name保存在platform_device的后面*/
		pa->pdev.name = pa->name; /*记录*/
		pa->pdev.id = id; 
		device_initialize(&pa->pdev.dev);/*初始化device结构体的成员*/
		pa->pdev.dev.release = platform_device_release;/*通用释放函数*/
		setup_pdev_dma_masks(&pa->pdev);
	}

	return pa ? &pa->pdev : NULL;
}
```

### platform_object

```C
struct platform_object {
        struct platform_device pdev;
        char name[];
};
```

