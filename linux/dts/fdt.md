#### fdt ：Flattened Device Tree data format

### fdt_header

```c
struct fdt_header {

	fdt32_t magic;			     /* magic word FDT_MAGIC 0xd00dfeed*/
    
	/*
	 * 整个dtb的大小，包含memory reservation block, structure block，strings block
	 * 及用于两个块之间的内存对齐的间隙大小
	 */
	fdt32_t totalsize;		     /* total size of DT block */
    
    /*structure block的偏移位置*/
	fdt32_t off_dt_struct;		 /* offset to structure */
    
    /*strings block的偏移位置*/
	fdt32_t off_dt_strings;		 /* offset to strings */
    
    /*memory reservation block的偏移位置*/
	fdt32_t off_mem_rsvmap;		 /* offset to memory reserve map */
    
    /*顾名思义，不同的版本解析的格式不同*/
	fdt32_t version;		     /* format version */
    
    /*兼容的版本*/
	fdt32_t last_comp_version;	 /* last compatible version */

	/* version 2 fields below */
	fdt32_t boot_cpuid_phys;	 /* Which physical CPU id we'rebooting on */
	
	/* version 3 fields below */
    /*strings block的字节大小*/
	fdt32_t size_dt_strings;	 /* size of the strings block */

	/* version 17 fields below */
    /*structure block的字节大小*/
	fdt32_t size_dt_struct;		 /* size of the structure block */
};
```

### setup_machine_fdt

```C
/**
 * setup_machine_fdt - Machine setup when an dtb was passed to the kernel
 * 当dtb传递到内核后，机器会被启动
 * @dt:	virtual address pointer to dt blob
 * @dt为指向dtb的虚拟地址
 * If a dtb was passed to the kernel, then use it to choose the correct
 * machine_desc and to setup the system.
 * 如果dtb已传递到内核，选择正确的machine_desc启动系统
 */
const struct machine_desc * __init setup_machine_fdt(void *dt)
{
	const struct machine_desc *mdesc;
	unsigned long dt_root;

	if (!early_init_dt_scan(dt))/*dtb内容检查，保存和设置属性*/
		return NULL;

	mdesc = of_flat_dt_match_machine(NULL, arch_get_next_mach);
	if (!mdesc)
		machine_halt();

	dt_root = of_get_flat_dt_root();/*root节点为0 return 0;*/
	arc_set_early_base_baud(dt_root);/*设置baud频率*/

	return mdesc;
}
```

```C
bool __init early_init_dt_scan(void *params)
{
	bool status;
    /*dtb关键内容进行检查*/
	status = early_init_dt_verify(params);
	if (!status)
		return false;
    /*检查chosen，保存cell属性，设置memory属性*/
	early_init_dt_scan_nodes();
	return true;
}
```

```C
bool __init early_init_dt_verify(void *params)
{
	if (!params)
		return false;

	/* check device tree validity */
    /*对整个dtb的关键部分进行检查，magic，size，bounds等*/
	if (fdt_check_header(params))
		return false;

	/* Setup flat device-tree pointer */
    /*记录deb的地址，并进行crc32校验*/
	initial_boot_params = params;
	of_fdt_crc32 = crc32_be(~0, initial_boot_params,
				fdt_totalsize(initial_boot_params));
	return true;
}
```

```C
int fdt_check_header(const void *fdt)
{
	size_t hdrsize;

	if (fdt_magic(fdt) != FDT_MAGIC)/*magic检查，固定内容*/
		return -FDT_ERR_BADMAGIC;
	if (!can_assume(LATEST)) {
        /*
         * #define FDT_FIRST_SUPPORTED_VERSION     0x02
         * #define FDT_LAST_SUPPORTED_VERSION      0x11
         * #define fdt_version(fdt)   (fdt_get_header(fdt, version))
         * 因结构体成员可能是不对齐的，所以返回对齐的内容
         * #define fdt_last_comp_version(fdt) (fdt_get_header(fdt, last_comp_version))
         */
		if ((fdt_version(fdt) < FDT_FIRST_SUPPORTED_VERSION)/*版本小于0x02*/
		    || (fdt_last_comp_version(fdt) >
			FDT_LAST_SUPPORTED_VERSION)) /*版本大于0x11*/
			return -FDT_ERR_BADVERSION;
		if (fdt_version(fdt) < fdt_last_comp_version(fdt)) /*小于兼容的版本，错误*/
			return -FDT_ERR_BADVERSION;
	}
	hdrsize = fdt_header_size(fdt);/*获取size*/
	if (!can_assume(VALID_DTB)) {

		if ((fdt_totalsize(fdt) < hdrsize) /*总size比handler还小，不合理*/
		    || (fdt_totalsize(fdt) > INT_MAX)) /*#define INT_MAX  ((int)(~0U>>1)) 有符号*/
			return -FDT_ERR_TRUNCATED;

		/* Bounds check memrsv block */
        /*(off >= hdrsize) && (off <= totalsize); memrsv块的边界检查*/
		if (!check_off_(hdrsize, fdt_totalsize(fdt),
				fdt_off_mem_rsvmap(fdt)))
			return -FDT_ERR_TRUNCATED;
	}

	if (!can_assume(VALID_DTB)) {
		/* Bounds check structure block */
		if (!can_assume(LATEST) && fdt_version(fdt) < 17) {
			if (!check_off_(hdrsize, fdt_totalsize(fdt),
					fdt_off_dt_struct(fdt))) /*off_dt_struct的边界检查*/
				return -FDT_ERR_TRUNCATED;
		} else { /*版本大于17，新格式*/
			if (!check_block_(hdrsize, fdt_totalsize(fdt),
					  fdt_off_dt_struct(fdt),
					  fdt_size_dt_struct(fdt)))
				return -FDT_ERR_TRUNCATED;
		}

		/* Bounds check strings block */
		if (!check_block_(hdrsize, fdt_totalsize(fdt),
				  fdt_off_dt_strings(fdt),
				  fdt_size_dt_strings(fdt)))
			return -FDT_ERR_TRUNCATED;
	}

	return 0;
}
```

```C
void __init early_init_dt_scan_nodes(void)
{
	int rc = 0;

	/* Retrieve various information from the /chosen node */
    /*
     * 检索 chose的bootargs 
     * early_init_dt_scan_chosen：回调函数
     * boot_command_line：不知在哪被填充
     */
	rc = of_scan_flat_dt(early_init_dt_scan_chosen, boot_command_line);
	if (!rc)
		pr_warn("No chosen node found, continuing without\n");

	/* Initialize {size,address}-cells info */
    /*提取#size-cells和#address-cells，并保存*/
	of_scan_flat_dt(early_init_dt_scan_root, NULL);

	/* Setup memory, calling early_init_dt_add_memory_arch */
    /*提取memory的reg，并具体设置*/
	of_scan_flat_dt(early_init_dt_scan_memory, NULL);
}

```

```C
/**
 * of_scan_flat_dt - scan flattened tree blob and call callback on each.
 * 从btb中，遍历所有的节点，将节点的名字，偏移，嵌套深度，私有数据，传递给it回调函数
 * @it: callback function
 * @data: context data pointer
 *
 * This function is used to scan the flattened device-tree, it is
 * used to extract the memory information at boot before we can
 * unflatten the tree
 * 用于平面的设备树的处理，提取信息，再把设备树进行展开成立体结构（即链表）
 */
int __init of_scan_flat_dt(int (*it)(unsigned long node,
				     const char *uname, int depth,
				     void *data),
			   void *data)
{
	const void *blob = initial_boot_params;
	const char *pathp;
	int offset, rc = 0, depth = -1;

	if (!blob)
		return 0;

	for (offset = fdt_next_node(blob, -1, &depth);
	     offset >= 0 && depth >= 0 && !rc;
	     offset = fdt_next_node(blob, offset, &depth)) {

		pathp = fdt_get_name(blob, offset, NULL);
		rc = it(offset, pathp, depth, data);
	}
	return rc;
}
```

```C
/**
 * of_flat_dt_match_machine - Iterate match tables to find matching machine.
 *
 * @default_match: A machine specific ptr to return in case of no match.
 * @get_next_compat: callback function to return next compatible match table.
 *
 * Iterate through machine match tables to find the best match for the machine
 * compatible string in the FDT.
 */
const void * __init of_flat_dt_match_machine(const void *default_match,
		const void * (*get_next_compat)(const char * const**))
{
	const void *data = NULL;
	const void *best_data = default_match;
	const char *const *compat;
	unsigned long dt_root;
	unsigned int best_score = ~1, score = 0;

	dt_root = of_get_flat_dt_root(); /*return 0;*/
	while ((data = get_next_compat(&compat))) {
		score = of_flat_dt_match(dt_root, compat);
		if (score > 0 && score < best_score) {
			best_data = data;
			best_score = score;
		}
	}
	if (!best_data) {
		const char *prop;
		int size;

		pr_err("\n unrecognized device tree list:\n[ ");

		prop = of_get_flat_dt_prop(dt_root, "compatible", &size);
		if (prop) {
			while (size > 0) {
				printk("'%s' ", prop);
				size -= strlen(prop) + 1;
				prop += strlen(prop) + 1;
			}
		}
		printk("]\n\n");
		return NULL;
	}

	pr_info("Machine model: %s\n", of_flat_dt_get_machine_name());

	return best_data;
}
```


****

****

![img](./img/dtb_structure_block.jpg "dtb_structure_block")

### device_node

```C
struct device_node {
	const char *name;
   
	phandle phandle; /*ID句柄*/
	const char *full_name;
	struct fwnode_handle fwnode;    /*fwnode的ops*/

	struct	property *properties;
	struct	property *deadprops;	/* removed properties */
	struct	device_node *parent;
	struct	device_node *child;
	struct	device_node *sibling;
#if defined(CONFIG_OF_KOBJ)
	struct	kobject kobj;
#endif
	unsigned long _flags;
	void	*data;
#if defined(CONFIG_SPARC)
	unsigned int unique_id;
	struct of_irq_controller *irq_trans;
#endif
};
```

### property

```C
struct property {
	char	*name;
	int	    length;
	void	*value;
	struct property *next;
#if defined(CONFIG_OF_DYNAMIC) || defined(CONFIG_SPARC)
	unsigned long _flags;
#endif
#if defined(CONFIG_OF_PROMTREE)
	unsigned int unique_id;
#endif
#if defined(CONFIG_OF_KOBJ)
	struct bin_attribute attr;
#endif
};
```

### fdt_node_header(dtb中的node)

```C
struct fdt_node_header {
        fdt32_t tag;
        char name[0];
};
```

### fdt_property(dtb中的property)

```C
struct fdt_property {
        fdt32_t tag;
        fdt32_t len;
        fdt32_t nameoff;
        char data[0];
};
```



```C
#define FDT_BEGIN_NODE	0x1		/* Start node: full name */
#define FDT_END_NODE	0x2		/* End node */
#define FDT_PROP	    0x3		/* Property: name off,size,content */  
#define FDT_NOP		    0x4		/* nop */
#define FDT_END		    0x9
```

### unflatten_device_tree

```C
/**
 * unflatten_device_tree - create tree of device_nodes from flat blob
 *
 * unflattens the device-tree passed by the firmware, creating the
 * tree of struct device_node. It also fills the "name" and "type"
 * pointers of the nodes so the normal device-tree walking functions
 * can be used.
 */
void __init unflatten_device_tree(void)
{
	__unflatten_device_tree(initial_boot_params, NULL, &of_root,
				early_init_dt_alloc_memory_arch, false);

	/* Get pointer to "/chosen" and "/aliases" nodes for use everywhere */
	of_alias_scan(early_init_dt_alloc_memory_arch);

	unittest_unflatten_overlay_base();
}
```

### __unflatten_device_tree

```C
/**
 * __unflatten_device_tree - create tree of device_nodes from flat blob
 *
 * unflattens a device-tree, creating the
 * tree of struct device_node. It also fills the "name" and "type"
 * pointers of the nodes so the normal device-tree walking functions
 * can be used.
 * @blob: The blob to expand
 * @dad: Parent device node
 * @mynodes: The device_node tree created by the call
 * @dt_alloc: An allocator that provides a virtual address to memory
 * for the resulting tree
 * @detached: if true set OF_DETACHED on @mynodes
 *
 * Returns NULL on failure or the memory chunk containing the unflattened
 * device tree on success.
 */
void *__unflatten_device_tree(const void *blob,
			      struct device_node *dad,
			      struct device_node **mynodes,
			      void *(*dt_alloc)(u64 size, u64 align),
			      bool detached)
{
	int size;
	void *mem;

	pr_debug(" -> unflatten_device_tree()\n");

	if (!blob) {
		pr_debug("No device tree pointer\n");
		return NULL;
	}

	pr_debug("Unflattening device tree:\n");
	pr_debug("magic: %08x\n", fdt_magic(blob));
	pr_debug("size: %08x\n", fdt_totalsize(blob));
	pr_debug("version: %08x\n", fdt_version(blob));

	if (fdt_check_header(blob)) { /*magic.size,bounds检查*/
		pr_err("Invalid device tree blob header\n");
		return NULL;
	}

	/* First pass, scan for size */
    /*第一遍扫描，计算dtb的structure block数量和其名字长度大小，总共所需的内存空间*/
	size = unflatten_dt_nodes(blob, NULL, dad, NULL);
	if (size < 0)
		return NULL;
    
    /*
     * #define ALIGN(x, a)           __ALIGN_KERNEL((x), (a))
     * #define __ALIGN_KERNEL(x, a)  __ALIGN_KERNEL_MASK(x, (typeof(x))(a) - 1)
     * #define __ALIGN_KERNEL_MASK(x, mask)  (((x) + (mask)) & ~(mask))
     * 字节对齐，如不足则补全
     */
	size = ALIGN(size, 4);
	pr_debug("  size is %d, allocating...\n", size);

	/* Allocate memory for the expanded device tree */
    /*分配所需的内存，其中4应该是magic的大小，下面可推测*/
	mem = dt_alloc(size + 4, __alignof__(struct device_node));
	if (!mem)
		return NULL;
    
    /*清0*/
	memset(mem, 0, size);
    /*最后的4字节用作magic，进行了大小端的转换处理，并赋值*/
	*(__be32 *)(mem + size) = cpu_to_be32(0xdeadbeef);

	pr_debug("  unflattening %p...\n", mem);

	/* Second pass, do actual unflattening */
    /*传入已经申请的mem*/
	unflatten_dt_nodes(blob, mem, dad, mynodes);
	if (be32_to_cpup(mem + size) != 0xdeadbeef)
		pr_warn("End of tree marker overwritten: %08x\n",
			be32_to_cpup(mem + size));

	if (detached && mynodes) {
		of_node_set_flag(*mynodes, OF_DETACHED);
		pr_debug("unflattened tree is detached\n");
	}

	pr_debug(" <- unflatten_device_tree()\n");
	return mem;
}
```

### unflatten_dt_nodes

```C
/**
 * unflatten_dt_nodes - Alloc and populate a device_node from the flat tree
 * 
 * @blob: The parent device tree blob
 * @mem: Memory chunk to use for allocating device nodes and properties
 * @dad: Parent struct device_node
 * @nodepp: The device_node tree created by the call
 *
 * It returns the size of unflattened device tree or error code
 */
/* 第一次: unflatten_dt_nodes(blob, NULL, NULL, NULL); */
/* 第二次: unflatten_dt_nodes(blob, mem, NULL, &of_root); */
static int unflatten_dt_nodes(const void *blob,
			      void *mem,
			      struct device_node *dad,
			      struct device_node **nodepp)
{
	struct device_node *root;
	int offset = 0, depth = 0, initial_depth = 0;
#define FDT_MAX_DEPTH	64 /*最深64个节点*/
	struct device_node *nps[FDT_MAX_DEPTH];
	void *base = mem;
	bool dryrun = !base;

	if (nodepp)
		*nodepp = NULL;

	/*
	 * We're unflattening device sub-tree if @dad is valid. There are
	 * possibly multiple nodes in the first level of depth. We need
	 * set @depth to 1 to make fdt_next_node() happy as it bails
	 * immediately when negative @depth is found. Otherwise, the device
	 * nodes except the first one won't be unflattened successfully.
	 */
	if (dad)
		depth = initial_depth = 1;

	root = dad;
	nps[depth] = dad;

 
	for (offset = 0;
	     offset >= 0 && depth >= initial_depth;
	     offset = fdt_next_node(blob, offset, &depth)) {
		if (WARN_ON_ONCE(depth >= FDT_MAX_DEPTH))
			continue;

		if (!IS_ENABLED(CONFIG_OF_KOBJ) && /*为ture，不成立*/
		    !of_fdt_device_is_available(blob, offset))
			continue;
        
        /*
         * 计算每一个device_node+name的大小
         * 如dryrun为0，则会对mem的device_node和name进行初始化
         */
		if (!populate_node(blob, offset, &mem, nps[depth],
				   &nps[depth+1], dryrun))
			return mem - base;

        /*
         * 此时of_root还没内容，则将初始化的第一个device_node
         * 赋予给of_root
         */
		if (!dryrun && nodepp && !*nodepp)
			*nodepp = nps[depth+1];
		if (!dryrun && !root)
			root = nps[depth+1];
	}

	if (offset < 0 && offset != -FDT_ERR_NOTFOUND) {
		pr_err("Error %d processing FDT\n", offset);
		return -EINVAL;
	}

	/*
	 * Reverse the child list. Some drivers assumes node order matches .dts
	 * node order
	 */
	if (!dryrun)
		reverse_nodes(root);

	return mem - base;
}
```


### populate_node

```C
static bool populate_node(const void *blob,
			  int offset,
			  void **mem,
			  struct device_node *dad,
			  struct device_node **pnp,
			  bool dryrun)
{
	struct device_node *np;
	const char *pathp;
	unsigned int l, allocl;
    /*获取到offset偏移的节点的名字，并返回其字符串长度l*/
	pathp = fdt_get_name(blob, offset, &l);
	if (!pathp) {
		*pnp = NULL;
		return false;/*已经遍历完成*/
	}

	allocl = ++l; /*加上'\0'*/

    /*
     * unflatten_dt_alloc只是做了所需申请内存统计，并没有真的申请
     * mem是二级指针，mem实际就是size，累计内存大小
     * size = sizeof(struct device_node) + allocl;
     * np = mem;
     * mem += size;
     */
	np = unflatten_dt_alloc(mem, sizeof(struct device_node) + allocl,
				__alignof__(struct device_node));
    
    
    /*
     * 根据上个调用的函数unflatten_dt_nodes
     * 第二次时，已经申请了mem，此时np则会持续往下，定位到每个node
     * 对每个node进行结构内容进行初始化
     */    
	if (!dryrun) { /*顾名思义，!dryrun，则需要做实际操作*/
		char *fn;
		of_node_init(np);/*设置通用ops，ktype*/
        
        /* 
         * 申请的内存，数据放置格式为 ：device_node在前，string在后
         * 长度为 n*（device_node + string）
         * full_name 指向 string 的位置
         */
		np->full_name = fn = ((char *)np) + sizeof(*np);
        
		memcpy(fn, pathp, l);/*并复制node的名字到string位置*/

		if (dad != NULL) { /*记录兄弟，父，子*/
			np->parent = dad;
			np->sibling = dad->child;
			dad->child = np;
		}
	}

	populate_properties(blob, offset, mem, np, pathp, dryrun);
	if (!dryrun) {
        /*device_node上挂了很多property，找到'name'属性，作为device_node->name的内容*/
		np->name = of_get_property(np, "name", NULL);/*找到*/
		if (!np->name)
			np->name = "<NULL>"; /*找到了'name'属性，但是property->value为空*/
	}

	*pnp = np;/*返回刚刚初始化完成的device_node*/
	return true; /*只要节点没遍历完，就会一直返回ture*/
}
```

### fdt_get_name

```C
const char *fdt_get_name(const void *fdt, int nodeoffset, int *len)
{
    /*第nodeoffset个偏移的structure block内容指针*/
	const struct fdt_node_header *nh = fdt_offset_ptr_(fdt, nodeoffset);
	const char *nameptr;
	int err;

	if (((err = fdt_ro_probe_(fdt)) < 0)
	    || ((err = fdt_check_node_offset_(fdt, nodeoffset)) < 0))
			goto fail;

	nameptr = nh->name;
    /*旧版本命名前缀带有'/'*/
	if (!can_assume(LATEST) && fdt_version(fdt) < 0x10) {
		/*
		 * For old FDT versions, match the naming conventions of V16:
		 * give only the leaf name (after all /). The actual tree
		 * contents are loosely checked.
		 */
		const char *leaf;
		leaf = strrchr(nameptr, '/');/*找到最后一次出现'/'的地方*/
		if (leaf == NULL) {
			err = -FDT_ERR_BADSTRUCTURE;
			goto fail;
		}
		nameptr = leaf+1;  /*偏移1*/
	}

	if (len)
		*len = strlen(nameptr); /*名字最后带'/0'，可以计算出长度*/

	return nameptr;

 fail:
	if (len)
		*len = err;
	return NULL;
}
```


### unflatten_dt_alloc

```C
static void *unflatten_dt_alloc(void **mem, unsigned long size,
				       unsigned long align)
{
	void *res;

	*mem = PTR_ALIGN(*mem, align);
	res = *mem; /*改变前先保存*/
	*mem += size; /*累加*/

	return res;
}
```


### fdt_next_node

```C
/*获得当前 device-tree structure 节点之后的下一个 device-tree structure 节点*/
int fdt_next_node(const void *fdt, int offset, int *depth)
{
	int nextoffset = 0;
	uint32_t tag;

	if (offset >= 0)
		if ((nextoffset = fdt_check_node_offset_(fdt, offset)) < 0)
			return nextoffset; /*不是FDT_BEGIN_NODE，直接返回*/

	do {/*当node为FDT_BEGIN_NODE标签，进入此循环*/
		offset = nextoffset;
		tag = fdt_next_tag(fdt, offset, &nextoffset);/*获取下一个tag*/

		switch (tag) {
		case FDT_PROP:
		case FDT_NOP:
			break;

		case FDT_BEGIN_NODE:
			if (depth) 
				(*depth)++;/*存在节点头，深度+1，循环结束，返回*/
			break;

		case FDT_END_NODE: /*深度-1*/
			if (depth && ((--(*depth)) < 0))
				return nextoffset; 
			break;

		case FDT_END:
			if ((nextoffset >= 0)
			    || ((nextoffset == -FDT_ERR_TRUNCATED) && !depth))
				return -FDT_ERR_NOTFOUND;
			else
				return nextoffset;
		}
	} while (tag != FDT_BEGIN_NODE);

	return offset;
}
```

### fdt_check_node_offset_

```C
int fdt_check_node_offset_(const void *fdt, int offset)
{
	if (!can_assume(VALID_INPUT)
	    && ((offset < 0) || (offset % FDT_TAGSIZE)))
		return -FDT_ERR_BADOFFSET;

	if (fdt_next_tag(fdt, offset, &offset) != FDT_BEGIN_NODE) /*符合FDT_BEGIN_NODE*/
		return -FDT_ERR_BADOFFSET;

	return offset;/*返回了下一个node*/
}
```

### fdt_next_tag

```C
/*
 * return：返回tag类型
 */
uint32_t fdt_next_tag(const void *fdt, int startoffset, int *nextoffset)
{
	const fdt32_t *tagp, *lenp;
	uint32_t tag;
	int offset = startoffset;
	const char *p;

	*nextoffset = -FDT_ERR_TRUNCATED;
    /*
     * 找到第n个structure，第三个参数传入作用是，检查大小为FDT_TAGSIZE，是否会读写越界
     * 此时tagp指向的就是tag，可以直接读
     */
	tagp = fdt_offset_ptr(fdt, offset, FDT_TAGSIZE);
    
    
	if (!can_assume(VALID_DTB) && !tagp) /*到了末尾*/
		return FDT_END; /* premature end */
    
    /*转大小端*/
	tag = fdt32_to_cpu(*tagp);
    
    /*偏移fdt32_t大小*/
	offset += FDT_TAGSIZE;

	*nextoffset = -FDT_ERR_BADSTRUCTURE;
	switch (tag) {
	case FDT_BEGIN_NODE:/*node头*/
		/* skip name */
		do {
			p = fdt_offset_ptr(fdt, offset++, 1); /*offset++，意味着会越过'/0'*/    
		} while (p && (*p != '\0'));
		if (!can_assume(VALID_DTB) && !p) /*到了末尾*/
			return FDT_END; /* premature end */
		break;

	case FDT_PROP:/*属性*/
        /*找到对应要处理的structure首地址，并越界检查*/
		lenp = fdt_offset_ptr(fdt, offset, sizeof(*lenp));
		if (!can_assume(VALID_DTB) && !lenp)
			return FDT_END; /* premature end */
            
		/* skip-name offset, length and value */
            
        /*
         * 因上面+FDT_TAGSIZE，所以要减回去
         * *lenp：属性字符串的大小
         * fdt_property->name[0] 不占空间
         */    
		offset += sizeof(struct fdt_property) - FDT_TAGSIZE
			+ fdt32_to_cpu(*lenp);
            
		if (!can_assume(LATEST) &&
		    fdt_version(fdt) < 0x10 && fdt32_to_cpu(*lenp) >= 8 &&
		    ((offset - fdt32_to_cpu(*lenp)) % 8) != 0)
			offset += 4; /*不清楚什么意思，但是最后呈现的*/
		break;

	case FDT_END:
	case FDT_END_NODE:
	case FDT_NOP:
		break;

	default:
		return FDT_END;
	}
    /*检查*/
	if (!fdt_offset_ptr(fdt, startoffset, offset - startoffset))
		return FDT_END; /* premature end */

    /*
     * 每个node的长度可能不是对齐的，需要align补全
     * nextoffset保存了下一个node的开始，即tag type
     */
	*nextoffset = FDT_TAGALIGN(offset);
	return tag; /*返回node tag类型*/
}
```

### fdt_offset_ptr

```C
/*这个函数只是做了一些保护检查，真正执行处理的是 fdt_offset_ptr_ */
const void *fdt_offset_ptr(const void *fdt, int offset, unsigned int len)
{
	unsigned int uoffset = offset;
    /*找到dtb的strucure位置，并偏移offset*/
	unsigned int absoffset = offset + fdt_off_dt_struct(fdt);

	if (offset < 0)
		return NULL;

	if (!can_assume(VALID_INPUT))
		if ((absoffset < uoffset) /*溢出*/
		    || ((absoffset + len) < absoffset) /*溢出*/
		    || (absoffset + len) > fdt_totalsize(fdt))/*越界*/
			return NULL;

	if (can_assume(LATEST) || fdt_version(fdt) >= 0x11)
		if (((uoffset + len) < uoffset) /*溢出*/
		    || ((offset + len) > fdt_size_dt_struct(fdt)))
			return NULL;

	return fdt_offset_ptr_(fdt, offset);
}
```

### fdt_offset_ptr_

```C
/*structure block的大小*/
#define fdt_off_dt_struct(fdt)          (fdt_get_header(fdt, off_dt_struct))

static inline const void *fdt_offset_ptr_(const void *fdt, int offset)
{
   return (const char *)fdt + fdt_off_dt_struct(fdt) + offset;
   /*找到第一个structure的位置，offset为偏移值，可以偏移多个structure block，也可以偏移成员位置*/
}
```

### of_node_init

```C
static inline void of_node_init(struct device_node *node)
{
#if defined(CONFIG_OF_KOBJ)
	kobject_init(&node->kobj, &of_node_ktype); /*设置ktype为of_node_ktype*/
#endif
	node->fwnode.ops = &of_fwnode_ops; /*设置ops为of_fwnode_ops*/
}
```

### of_fwnode_ops

```C
const struct fwnode_operations of_fwnode_ops = {
	.get = of_fwnode_get,
	.put = of_fwnode_put,
	.device_is_available = of_fwnode_device_is_available,
	.device_get_match_data = of_fwnode_device_get_match_data,
	.property_present = of_fwnode_property_present,
	.property_read_int_array = of_fwnode_property_read_int_array,
	.property_read_string_array = of_fwnode_property_read_string_array,
	.get_name = of_fwnode_get_name,
	.get_name_prefix = of_fwnode_get_name_prefix,
	.get_parent = of_fwnode_get_parent,
	.get_next_child_node = of_fwnode_get_next_child_node,
	.get_named_child_node = of_fwnode_get_named_child_node,
	.get_reference_args = of_fwnode_get_reference_args,
	.graph_get_next_endpoint = of_fwnode_graph_get_next_endpoint,
	.graph_get_remote_endpoint = of_fwnode_graph_get_remote_endpoint,
	.graph_get_port_parent = of_fwnode_graph_get_port_parent,
	.graph_parse_endpoint = of_fwnode_graph_parse_endpoint,
	.add_links = of_fwnode_add_links,
};
```

### of_node_ktype

```C
struct kobj_type of_node_ktype = {
        .release = of_node_release,
};
```


### of_get_property

```C
/*
 * Find a property with a given name for a given node
 * and return the value.
 */
const void *of_get_property(const struct device_node *np, const char *name,
                            int *lenp)
{
        struct property *pp = of_find_property(np, name, lenp);

        return pp ? pp->value : NULL;
}
```

### of_find_property

```C
struct property *of_find_property(const struct device_node *np,
				  const char *name,
				  int *lenp)
{
	struct property *pp;
	unsigned long flags;

	raw_spin_lock_irqsave(&devtree_lock, flags);
	pp = __of_find_property(np, name, lenp);
	raw_spin_unlock_irqrestore(&devtree_lock, flags);

	return pp;
}
```

### __of_find_property

```C
static struct property *__of_find_property(const struct device_node *np,
					   const char *name, int *lenp)
{
	struct property *pp;

	if (!np)
		return NULL;
    /*利用链表遍历device_node的每个pp，找到同名*/
	for (pp = np->properties; pp; pp = pp->next) {
		if (of_prop_cmp(pp->name, name) == 0) {
			if (lenp)
				*lenp = pp->length; /*返回其长度*/
			break;
		}
	}

	return pp;
}
```

### strcasecmp

```C
/*of_prop_cmp 底层调用的就是 strcasecmp*/
int strcasecmp(const char *s1, const char *s2)
{
        int c1, c2;

        do {
                c1 = tolower(*s1++);/*转换小写*/
                c2 = tolower(*s2++);
        } while (c1 == c2 && c1 != 0);
        return c1 - c2;
}
```
### populate_properties

```C
static void populate_properties(const void *blob,
				int offset,
				void **mem,
				struct device_node *np,
				const char *nodename,
				bool dryrun)
{
	struct property *pp, **pprev = NULL;
	int cur;
	bool has_name = false;

	pprev = &np->properties;
	for (cur = fdt_first_property_offset(blob, offset);
	     cur >= 0;
	     cur = fdt_next_property_offset(blob, cur)) {
		const __be32 *val;
		const char *pname;
		u32 sz;

		val = fdt_getprop_by_offset(blob, cur, &pname, &sz);
		if (!val) {
			pr_warn("Cannot locate property at 0x%x\n", cur);
			continue;
		}

		if (!pname) {
			pr_warn("Cannot find property name at 0x%x\n", cur);
			continue;
		}

		if (!strcmp(pname, "name"))
			has_name = true;

		pp = unflatten_dt_alloc(mem, sizeof(struct property),
					__alignof__(struct property));
		if (dryrun)
			continue;

		/* We accept flattened tree phandles either in
		 * ePAPR-style "phandle" properties, or the
		 * legacy "linux,phandle" properties.  If both
		 * appear and have different values, things
		 * will get weird. Don't do that.
		 */
		if (!strcmp(pname, "phandle") ||
		    !strcmp(pname, "linux,phandle")) {
			if (!np->phandle)
				np->phandle = be32_to_cpup(val);
		}

		/* And we process the "ibm,phandle" property
		 * used in pSeries dynamic device tree
		 * stuff
		 */
		if (!strcmp(pname, "ibm,phandle"))
			np->phandle = be32_to_cpup(val);

		pp->name   = (char *)pname;
		pp->length = sz;
		pp->value  = (__be32 *)val;
		*pprev     = pp;
		pprev      = &pp->next;
	}

	/* With version 0x10 we may not have the name property,
	 * recreate it here from the unit name if absent
	 */
	if (!has_name) {
		const char *p = nodename, *ps = p, *pa = NULL;
		int len;

		while (*p) {
			if ((*p) == '@')
				pa = p;
			else if ((*p) == '/')
				ps = p + 1;
			p++;
		}

		if (pa < ps)
			pa = p;
		len = (pa - ps) + 1;
		pp = unflatten_dt_alloc(mem, sizeof(struct property) + len,
					__alignof__(struct property));
		if (!dryrun) {
			pp->name   = "name";
			pp->length = len;
			pp->value  = pp + 1;
			*pprev     = pp;
			pprev      = &pp->next;
			memcpy(pp->value, ps, len - 1);
			((char *)pp->value)[len - 1] = 0;
			pr_debug("fixed up name for %s -> %s\n",
				 nodename, (char *)pp->value);
		}
	}

	if (!dryrun)
		*pprev = NULL;
}
```

### fdt_first_property_offset

```C
int fdt_first_property_offset(const void *fdt, int nodeoffset)
{
	int offset;

	if ((offset = fdt_check_node_offset_(fdt, nodeoffset)) < 0)
		return offset;

	return nextprop_(fdt, offset);
}
```

### nextprop_

```C
static int nextprop_(const void *fdt, int offset)
{
	uint32_t tag;
	int nextoffset;

	do {
		tag = fdt_next_tag(fdt, offset, &nextoffset);

		switch (tag) {
		case FDT_END:
			if (nextoffset >= 0)
				return -FDT_ERR_BADSTRUCTURE;
			else
				return nextoffset;

		case FDT_PROP:
			return offset;
		}
		offset = nextoffset;
	} while (tag == FDT_NOP);

	return -FDT_ERR_NOTFOUND;
}
```

