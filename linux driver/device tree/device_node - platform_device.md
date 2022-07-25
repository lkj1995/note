### platform_device

```C
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

### pdev_archdata

```
struct pdev_archdata {
#ifdef CONFIG_ARCH_OMAP
        struct omap_device *od;
#endif
};
```

### omap_device

```c
struct omap_device {
        struct platform_device          *pdev;
        struct omap_hwmod               **hwmods;
        unsigned long                   _driver_status;
        u8                              hwmods_cnt;
        u8                              _state;
        u8                              flags;
};
```

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
	 * 正确的处理每个node，并不是每个node都需要生成platform_device
	 */
	for_each_matching_node(node, reserved_mem_matches)
		of_platform_device_create(node, NULL, NULL);

	node = of_find_node_by_path("/firmware");/*根据该路径找到此节点*/
	if (node) {
		of_platform_populate(node, NULL, NULL, NULL);
		of_node_put(node);
	}

	/* Populate everything else. */
	fw_devlink_pause();
	of_platform_default_populate(NULL, NULL, NULL);/*主要是这个处理*/
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
/*
 * Struct used for matching a device
 */
struct of_device_id {
        char    name[32];
        char    type[32];
        char    compatible[128];
        const void *data;
};

static const struct of_device_id reserved_mem_matches[] = {
	{ .compatible = "qcom,rmtfs-mem" },
	{ .compatible = "qcom,cmd-db" },
	{ .compatible = "ramoops" },
	{}
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
/*当节点的子节点存在这些属性，则可以生成platform_device*/
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
    
    /*如不为空，则增加kobj引用计数，否则找到of_root*/
	root = root ? of_node_get(root) : of_find_node_by_path("/");
	if (!root)
		return -EINVAL;

	pr_debug("%s()\n", __func__);
	pr_debug(" starting at: %pOF\n", root);

	device_links_supplier_sync_state_pause();
    
    /*把从root开始的每个节点取出来赋值给child*/
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
 *		foo		    Valid alias
 *		foo/bar		Valid alias + relative path
 *
 *	Returns a node pointer with refcount incremented, use
 *	of_node_put() on it when done.
 */
struct device_node *of_find_node_opts_by_path(const char *path, const char **opts)
{
	struct device_node *np = NULL;
	struct property *pp;
	unsigned long flags;
	const char *separator = strchr(path, ':');/*检查path中是否存在字符':'*/

	if (opts)
		*opts = separator ? separator + 1 : NULL;
    
    /*path等于"/"，增加of_root的kobj引用计数，并返回of_root地址*/
	if (strcmp(path, "/") == 0)
		return of_node_get(of_root);

	/* The path could begin with an alias */
    /*当path为"/firmware"，不符合此条件，跳过*/
	if (*path != '/') {
		int len;
		const char *p = separator;

		if (!p)
			p = strchrnul(path, '/');
		len = p - path;

		/* of_aliases must not be NULL */
		if (!of_aliases)
			return NULL;

		for_each_property_of_node(of_aliases, pp) {
			if (strlen(pp->name) == len && !strncmp(pp->name, path, len)) {
				np = of_find_node_by_path(pp->value);
				break;
			}
		}
		if (!np)
			return NULL;
		path = p;
	}

	/* Step down the tree matching path components */
	raw_spin_lock_irqsave(&devtree_lock, flags);
	if (!np)/*如果为空，则从of_root开始寻找*/
		np = of_node_get(of_root);
	np = __of_find_node_by_full_path(np, path);/*根据路径中的名字找到对应的node*/
	raw_spin_unlock_irqrestore(&devtree_lock, flags);
	return np;
}
```

### __of_find_node_by_full_path

```C
struct device_node *__of_find_node_by_full_path(struct device_node *node,
						const char *path)
{
	const char *separator = strchr(path, ':');

	while (node && *path == '/') {
		struct device_node *tmp = node;

		path++; /* Increment past '/' delimiter */
		node = __of_find_node_by_path(node, path);/*根据路径中的名字找到了node*/
		of_node_put(tmp);
		path = strchrnul(path, '/');
		if (separator && separator < path)
			break;
	}
	return node;
}
```

### strchr

```C
/**
 * strchr - Find the first occurrence of a character in a string
 * @s: The string to be searched
 * @c: The character to search for
 *
 * Note that the %NUL-terminator is considered part of the string, and can
 * be searched for.
 */
char *strchr(const char *s, int c)
{
	for (; *s != (char)c; ++s)
		if (*s == '\0')
			return NULL;
	return (char *)s;
}
```

### strchrnul

```C
/**
 * strchrnul - Find and return a character in a string, or end of string
 * @s: The string to be searched
 * @c: The character to search for
 *
 * Returns pointer to first occurrence of 'c' in s. If c is not found, then
 * return a pointer to the null byte at the end of s.
 */
char *strchrnul(const char *s, int c)
{
	while (*s && *s != (char)c)
		s++;
	return (char *)s;
}
```

### __of_find_node_by_path

```C
struct device_node *__of_find_node_by_path(struct device_node *parent,
						const char *path)
{
	struct device_node *child;
	int len;

	len = strcspn(path, "/:");
	if (!len)
		return NULL;
    
    /*遍历parent及它的每个child*/
	__for_each_child_of_node(parent, child) {
		const char *name = kbasename(child->full_name);/*过滤字符'/'*/
		if (strncmp(path, name, len) == 0 && (strlen(name) == len))
			return child;/*假如传入了"/firmware",则会变成"firmware"*/
	}
	return NULL;
}
```

### strcspn

```C
/**
 * strcspn - Calculate the length of the initial substring of @s which does not contain letters in @reject
 * @s: The string to be searched
 * @reject: The string to avoid
 */
size_t strcspn(const char *s, const char *reject)
{
	const char *p;
	const char *r;
	size_t count = 0;

	for (p = s; *p != '\0'; ++p) {
		for (r = reject; *r != '\0'; ++r) {
			if (*p == *r)
				return count;
		}
		++count;
	}
	return count;
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
	
    /*
     * 检查该节点下的子节点。如iic下的子节点
     * 是否存在，"simple-bus","simple-mfd","isa","arm,amba-bus"
     * 存在则能生成platform_device
     */
    if (!dev || !of_match_node(matches, bus))
		return 0;

	for_each_child_of_node(bus, child) {
		pr_debug("   create child: %pOF\n", child);
		rc = of_platform_bus_create(child, matches, lookup, &dev->dev, strict);/*递归*/
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
struct bus_type platform_bus_type = {
        .name           = "platform",
        .dev_groups     = platform_dev_groups,
        .match          = platform_match,
        .uevent         = platform_uevent,
        .dma_configure  = platform_dma_configure,
        .pm             = &platform_dev_pm_ops,
};

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
	dev->dev.bus = &platform_bus_type;/**/
	dev->dev.platform_data = platform_data;
	of_msi_configure(&dev->dev, dev->dev.of_node);

	if (of_device_add(dev) != 0) {/*调用device_add,添加kobj*/
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
 *  检查status属性是否为okey
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
 *  检查status属性是否为okey
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
    
    /*获取status属性的内容*/
	status = __of_get_property(device, "status", &statlen);    

    /*不存在这个属性*/
	if (status == NULL)
		return true;

	if (statlen > 0) {/*内容是否为"okey"或"ok"*/
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
    /*计算所需resources的数量*/
	while (of_address_to_resource(np, num_reg, &temp_res) == 0)
		num_reg++;
    
    /*计算该node中断数量*/
	num_irq = of_irq_count(np);

	/* Populate the resource table */
    /*分配resource数组*/
	if (num_irq || num_reg) {
		res = kcalloc(num_irq + num_reg, sizeof(*res), GFP_KERNEL);
		if (!res) {
			platform_device_put(dev);
			return NULL;
		}
        /*记录数量和分配的resource数组*/
		dev->num_resources = num_reg + num_irq;
		dev->resource = res;
		for (i = 0; i < num_reg; i++, res++) {
            /*把这些地址转化成resource结构体*/
			rc = of_address_to_resource(np, i, res);
			WARN_ON(rc);
		}
		if (of_irq_to_resource_table(np, res, num_irq) != num_irq)
			pr_debug("not all legacy IRQ resources mapped for %pOFn\n",
				 np);
	}
    
    /*platform_device和vice_node建立关系*/
	dev->dev.of_node = of_node_get(np); 
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

### of_device_add

```C
int of_device_add(struct platform_device *ofdev)
{
	BUG_ON(ofdev->dev.of_node == NULL);

	/* name and id have to be set so that the platform bus doesn't get
	 * confused on matching */
	ofdev->name = dev_name(&ofdev->dev);
	ofdev->id = PLATFORM_DEVID_NONE;

	/*
	 * If this device has not binding numa node in devicetree, that is
	 * of_node_to_nid returns NUMA_NO_NODE. device_add will assume that this
	 * device is on the same node as the parent.
	 */
	set_dev_node(&ofdev->dev, of_node_to_nid(ofdev->dev.of_node));

	return device_add(&ofdev->dev); /*初始化device，添加kobj，建立对应关系*/
}
```

### of_address_to_resource

```C
/**
 * of_address_to_resource - Translate device tree address and return as resource
 *
 * Note that if your address is a PIO address, the conversion will fail if
 * the physical address can't be internally converted to an IO token with
 * pci_address_to_pio(), that is because it's either called too early or it
 * can't be matched to any host bridge IO space
 */
int of_address_to_resource(struct device_node *dev, int index,
			   struct resource *r)
{
	const __be32	*addrp;
	u64		size;
	unsigned int	flags;
	const char	*name = NULL;

	addrp = of_get_address(dev, index, &size, &flags);
	if (addrp == NULL)
		return -EINVAL;

	/* Get optional "reg-names" property to add a name to a resource */
	of_property_read_string_index(dev, "reg-names",	index, &name);

	return __of_address_to_resource(dev, addrp, size, flags, name, r);
}
```

### of_get_address

```c
const __be32 *of_get_address(struct device_node *dev, int index, u64 *size,
		    unsigned int *flags)
{
	const __be32 *prop;
	unsigned int psize;
	struct device_node *parent;
	struct of_bus *bus;
	int onesize, i, na, ns;

	/* Get parent & match bus type */
	parent = of_get_parent(dev);
	if (parent == NULL)
		return NULL;
	bus = of_match_bus(parent);
	bus->count_cells(dev, &na, &ns);
	of_node_put(parent);
	if (!OF_CHECK_ADDR_COUNT(na))
		return NULL;

	/* Get "reg" or "assigned-addresses" property */
	prop = of_get_property(dev, bus->addresses, &psize);
	if (prop == NULL)
		return NULL;
	psize /= 4;

	onesize = na + ns;
	for (i = 0; psize >= onesize; psize -= onesize, prop += onesize, i++)
		if (i == index) {
			if (size)
				*size = of_read_number(prop + na, ns);
			if (flags)
				*flags = bus->get_flags(prop);
			return prop;
		}
	return NULL;
}
```

### of_property_read_string_index

```c
/**
 * of_property_read_string_index() - Find and read a string from a multiple
 * strings property.
 * 找到字符串属性名，并将其内容读出
 * @np:		device node from which the property value is to be read.
 * @propname:	name of the property to be searched.
 * @index:	index of the string in the list of strings
 * @out_string:	pointer to null terminated return string, modified only if
 *		return value is 0.
 *
 * Search for a property in a device tree node and retrieve a null
 * terminated string value (pointer to data, not a copy) in the list of strings
 * contained in that property.
 * Returns 0 on success, -EINVAL if the property does not exist, -ENODATA if
 * property does not have a value, and -EILSEQ if the string is not
 * null-terminated within the length of the property data.
 *
 * The out_string pointer is modified only if a valid string can be decoded.
 */
static inline int of_property_read_string_index(const struct device_node *np,
						const char *propname,
						int index, const char **output)
{
	int rc = of_property_read_string_helper(np, propname, output, 1, index);
	return rc < 0 ? rc : 0;
}

```

### __of_address_to_resource

```C
static int __of_address_to_resource(struct device_node *dev,
		const __be32 *addrp, u64 size, unsigned int flags,
		const char *name, struct resource *r)
{
	u64 taddr;

    /*根据不同类型，获取其物理地址*/
	if (flags & IORESOURCE_MEM)
		taddr = of_translate_address(dev, addrp);
	else if (flags & IORESOURCE_IO)
		taddr = of_translate_ioport(dev, addrp, size);
	else
		return -EINVAL;

	if (taddr == OF_BAD_ADDR)
		return -EINVAL;
	memset(r, 0, sizeof(struct resource));

	r->start = taddr;
	r->end = taddr + size - 1;
	r->flags = flags;
	r->name = name ? name : dev->full_name;

	return 0;
}
```



## 总结

1. 系统启动后，从 .initcall3s.init 段执行初始化函数。
2. 获取of_root,展开所有的兄弟和子节点device_node
3. 检查符合条件的节点，生成platform_device
4. 根据device_node，初始化platform_device
5. 提取每个device_node的reg的address，size和IRQ
6. 生成对应resource个数，初始化resource

