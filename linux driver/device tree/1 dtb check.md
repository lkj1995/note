## properties标准

### compatible

- 属性名：compatible

- 值类型：stringlist

- 描述：包含1个或多个字符串

- 格式：“厂商,型号”

- 示例：

  -  compatible = "fsl,mpc8641", "ns16550";  
  - 寻找符合第一个字符串的driver，如失败，则会匹配第二个。

  

### model 

- 属性名：model

- 值类型：string

- 描述：
- 格式：“厂商,型号”

- 示例：
  - model = "fsl,MPC8349EMITX";  

### phandle 

- 属性名：phandle 

- 值类型：u32

- 描述：唯一标识符，用于在设备树来识别该node.一般不需显式出现，dtc会自动插入该属性
- 格式：phandle = <num>;

- 示例：
  - phandle = <1>; 

### status 

- 属性名：status 

- 值类型：string

| 类型       | 描述                                                         |
| ---------- | ------------------------------------------------------------ |
| "okay"     | 该设备可操作。                                               |
| "disabled" | 该设备当前不可操作，但可能在之后能够操作，如热拔插。         |
| "reserved" | 该设备可操作，但不应该被使用，被其他软件部件控制。           |
| "fail"     | 说明该设备不可操作，一个错误码被记录在设备中，被修复前都无法再操作。 |
| "fail-sss" | 同上，sss为错误码。                                          |
- 格式：status = <string>;

- 示例：
  - status= “okay“



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

### setup_arch

```C
void __init setup_arch(char **cmdline_p)
{
	const struct machine_desc *mdesc;

	setup_processor();
    /*只关注这个入口，__atags_pointer 保存了tag或dtb的地址*/
	mdesc = setup_machine_fdt(__atags_pointer);
	if (!mdesc)
		mdesc = setup_machine_tags(__atags_pointer, __machine_arch_type);
    
}
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

### early_init_dt_scan

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

### early_init_dt_verify

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

### fdt_check_header

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

### early_init_dt_scan_nodes

```C
void __init early_init_dt_scan_nodes(void)
{
	int rc = 0;

	/* Retrieve various information from the /chosen node */
    /*
     * 检索 chosen属性的bootargs 
     * early_init_dt_scan_chosen：回调函数
     * 内容保存在 boot_command_line 全局指针中
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

### of_scan_flat_dt

```C
/**
 * of_scan_flat_dt - scan flattened tree blob and call callback on each.
 * 从btb中，遍历所有的节点，将节点的名字，偏移，嵌套深度，私有数据，传递给it回调函数
 * @it: callback function
 * @data: context data pointer
 * 保存内容在data
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

	for (offset = fdt_next_node(blob, -1, &depth);/*一层一层的node寻找*/
	     offset >= 0 && depth >= 0 && !rc;
	     offset = fdt_next_node(blob, offset, &depth)) {

		pathp = fdt_get_name(blob, offset, NULL);/*获取node的long_name*/
		rc = it(offset, pathp, depth, data);
	}
	return rc;
}
```

### fdt_next_node

```C
int fdt_next_node(const void *fdt, int offset, int *depth)
{
	int nextoffset = 0;
	uint32_t tag;

	if (offset >= 0)
		if ((nextoffset = fdt_check_node_offset_(fdt, offset)) < 0)
			return nextoffset;

	do {
		offset = nextoffset;
		tag = fdt_next_tag(fdt, offset, &nextoffset);

		switch (tag) {
		case FDT_PROP:
		case FDT_NOP:
			break;

		case FDT_BEGIN_NODE:
			if (depth)
				(*depth)++;
			break;

		case FDT_END_NODE:
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

### early_init_dt_scan_chosen

```C
int __init early_init_dt_scan_chosen(unsigned long node, const char *uname,
				     int depth, void *data)
{
	int l;
	const char *p;
	const void *rng_seed;

	pr_debug("search \"chosen\", depth: %d, uname: %s\n", depth, uname);

	if (depth != 1 || !data ||
	    (strcmp(uname, "chosen") != 0 && strcmp(uname, "chosen@0") != 0))
		return 0;

	early_init_dt_check_for_initrd(node);

	/* Retrieve command line */
	p = of_get_flat_dt_prop(node, "bootargs", &l);/*获取该node的属性bootargs*/
	if (p != NULL && l > 0)
		strlcpy(data, p, min(l, COMMAND_LINE_SIZE));/*保存bootargs字符串内容*/

	/*
	 * CONFIG_CMDLINE is meant to be a default in case nothing else
	 * managed to set the command line, unless CONFIG_CMDLINE_FORCE
	 * is set in which case we override whatever was found earlier.
	 */
#ifdef CONFIG_CMDLINE
#if defined(CONFIG_CMDLINE_EXTEND)
	strlcat(data, " ", COMMAND_LINE_SIZE);
	strlcat(data, CONFIG_CMDLINE, COMMAND_LINE_SIZE);
#elif defined(CONFIG_CMDLINE_FORCE)
	strlcpy(data, CONFIG_CMDLINE, COMMAND_LINE_SIZE);
#else
	/* No arguments from boot loader, use kernel's  cmdl*/
	if (!((char *)data)[0])
		strlcpy(data, CONFIG_CMDLINE, COMMAND_LINE_SIZE);
#endif
#endif /* CONFIG_CMDLINE */

	pr_debug("Command line is: %s\n", (char *)data);

	rng_seed = of_get_flat_dt_prop(node, "rng-seed", &l);
	if (rng_seed && l > 0) {
		add_bootloader_randomness(rng_seed, l);

		/* try to clear seed so it won't be found. */
		fdt_nop_property(initial_boot_params, node, "rng-seed");

		/* update CRC check value */
		of_fdt_crc32 = crc32_be(~0, initial_boot_params,
				fdt_totalsize(initial_boot_params));
	}

	/* break now */
	return 1;
}
```

### early_init_dt_scan_root

```C
/**
 * early_init_dt_scan_root - fetch the top level address and size cells
 * 提取根节点的address cells和size cells
 */
int __init early_init_dt_scan_root(unsigned long node, const char *uname,
				   int depth, void *data)
{
	const __be32 *prop;

	if (depth != 0)
		return 0;

	dt_root_size_cells = OF_ROOT_NODE_SIZE_CELLS_DEFAULT;
	dt_root_addr_cells = OF_ROOT_NODE_ADDR_CELLS_DEFAULT;

	prop = of_get_flat_dt_prop(node, "#size-cells", NULL);/*在节点找属性#size-cells*/
	if (prop)
		dt_root_size_cells = be32_to_cpup(prop);/*记录*/
	pr_debug("dt_root_size_cells = %x\n", dt_root_size_cells);

	prop = of_get_flat_dt_prop(node, "#address-cells", NULL);/*在节点找属性#address-cells*/
	if (prop)
		dt_root_addr_cells = be32_to_cpup(prop);/*记录*/
	pr_debug("dt_root_addr_cells = %x\n", dt_root_addr_cells);

	/* break now */
	return 1;
}

```

### early_init_dt_scan_memory

```C
/**
 * early_init_dt_scan_memory - Look for and parse memory nodes
 */
int __init early_init_dt_scan_memory(unsigned long node, const char *uname,
				     int depth, void *data)
{
    /*获取device_type属性内容*/
	const char *type = of_get_flat_dt_prop(node, "device_type", NULL); 
	const __be32 *reg, *endp;
	int l;
	bool hotpluggable;

	/* We are scanning "memory" nodes only */
	if (type == NULL || strcmp(type, "memory") != 0)/*内容必须为memory*/
		return 0;

	reg = of_get_flat_dt_prop(node, "linux,usable-memory", &l);
	if (reg == NULL)
		reg = of_get_flat_dt_prop(node, "reg", &l);/*获取reg属性内容*/
	if (reg == NULL)
		return 0;

	endp = reg + (l / sizeof(__be32));
	hotpluggable = of_get_flat_dt_prop(node, "hotpluggable", NULL);

	pr_debug("memory scan node %s, reg size %d,\n", uname, l);

	while ((endp - reg) >= (dt_root_addr_cells + dt_root_size_cells)) {
		u64 base, size;

		base = dt_mem_next_cell(dt_root_addr_cells, &reg);  /*提取地址内容*/
		size = dt_mem_next_cell(dt_root_size_cells, &reg);  /*提取长度内容*/

		if (size == 0)
			continue;
		pr_debug(" - %llx ,  %llx\n", (unsigned long long)base,
		    (unsigned long long)size);

		early_init_dt_add_memory_arch(base, size);

		if (!hotpluggable)
			continue;

		if (early_init_dt_mark_hotplug_memory_arch(base, size))
			pr_warn("failed to mark hotplug range 0x%llx - 0x%llx\n",
				base, base + size);
	}

	return 0;
}
```

### of_get_flat_dt_prop

```C
/**
 * of_get_flat_dt_prop - Given a node in the flat blob, return the property ptr
 * 在fdb中获取node，返回其属性的地址指针
 * This function can be used within scan_flattened_dt callback to get
 * access to properties
 */
const void *__init of_get_flat_dt_prop(unsigned long node, const char *name,
				       int *size)
{   
    /*
     * initial_boot_params：根root的node
     * node：以root的node作起始地址，指向具体哪个node的偏移位置
     * name:寻找该node的哪个属性，用name寻找
     * size：返回属性内容的字符串长度
     */
	return fdt_getprop(initial_boot_params, node, name, size);
}
```

### fdt_getprop

```C
const void *fdt_getprop(const void *fdt, int nodeoffset,
                        const char *name, int *lenp)
{
        return fdt_getprop_namelen(fdt, nodeoffset, name, strlen(name), lenp);
}
```

### fdt_getprop_namelen

```C
const void *fdt_getprop_namelen(const void *fdt, int nodeoffset,
				const char *name, int namelen, int *lenp)
{
	int poffset;
	const struct fdt_property *prop;

	prop = fdt_get_property_namelen_(fdt, nodeoffset, name, namelen, lenp,
					 &poffset);
	if (!prop)
		return NULL;

	/* Handle realignment */
	if (!can_assume(LATEST) && fdt_version(fdt) < 0x10 &&
	    (poffset + sizeof(*prop)) % 8 && fdt32_ld(&prop->len) >= 8)
		return prop->data + 4; 
	return prop->data;/*返回属性名字的地址*/
}
```

### fdt_get_property_namelen_

```C
static const struct fdt_property *fdt_get_property_namelen_(const void *fdt,
						            int offset,
						            const char *name,
						            int namelen,
							    int *lenp,
							    int *poffset)
{
	for (offset = fdt_first_property_offset(fdt, offset);
	     (offset >= 0);
	     (offset = fdt_next_property_offset(fdt, offset))) {
		const struct fdt_property *prop;

		prop = fdt_get_property_by_offset_(fdt, offset, lenp);
		if (!can_assume(LIBFDT_FLAWLESS) && !prop) {
			offset = -FDT_ERR_INTERNAL;
			break;
		}
		if (fdt_string_eq_(fdt, fdt32_ld(&prop->nameoff),
				   name, namelen)) {
			if (poffset)
				*poffset = offset;
			return prop;
		}
	}

	if (lenp)
		*lenp = offset;
	return NULL;
}
```

### arch_get_next_mach

```C
static const void * __init arch_get_next_mach(const char *const **match)
{
    /* __arch_info_begin 存放在 init.arch.info段 */
	static const struct machine_desc *mdesc = __arch_info_begin;
	const struct machine_desc *m = mdesc;

	if (m >= __arch_info_end)
		return NULL;

	mdesc++;
	*match = m->dt_compat;
	return m;
}
```

### of_flat_dt_match_machine

```C
/**
 * of_flat_dt_match_machine - Iterate match tables to find matching machine.
 * 在根节点的compatible寻找最合适的机器名，score越小越匹配
 * @default_match: A machine specific ptr to return in case of no match.
 * @get_next_compat: callback function to return next compatible match table.
 *                   传入的函数为 arch_get_next_mach
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

### fdt_first_property_offset

```C
int fdt_first_property_offset(const void *fdt, int nodeoffset)
{
	int offset;
    /* 检查传入的node，tag是否为FDT_BEGIN_NODE*/
	if ((offset = fdt_check_node_offset_(fdt, nodeoffset)) < 0)
		return offset;

	return nextprop_(fdt, offset);/*确认为node内部，开始寻找该node的property*/
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

		case FDT_PROP: /*找到属性的标签，返回其地址偏移*/
			return offset;
		}
		offset = nextoffset;
	} while (tag == FDT_NOP);/*当找到FDT_NOP，跳过，继续往下找*/

	return -FDT_ERR_NOTFOUND;
}
```

## 总结

****

1. 启动过程: start_kernel() -> setup_arch() -> setup_machine_fdt()
2. 在head.S中，搜索tag或dtb的地址，并保存在 __atags_pointer，传入setup_machine_fdt()进行处理

3. 对dtb的内容进行校验检查，如magic数检查 ，size大小检查，crc计算，版本是否合适等
4. 提取root节点的属性，如chosen属性的bootargs，#size-cells属性，#address-cells属性，memory节点的device_type属性等。
5. 并执行其it回调函数，做相应的处理，比如保存等。



