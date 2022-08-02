![img](./img/dtb_structure_block.jpg "dtb_structure_block")

### struct device_node

```C
struct of_irq_controller {
	unsigned int (*irq_build)(struct device_node *, 
                                unsigned int, void *);
	void  *data;
};


struct device_node {
	/*如果改node存在name属性，该指针指向其内容，否则设置为"<NULL>"*/
    const char *name;
    
    /*ID句柄*/
	phandle phandle;
    
    /*节点的名字*/
	const char *full_name; 
    
    /*fwnode的ops*/
	struct fwnode_handle fwnode;    
    
    /*node的属性列表*/
	struct	property *properties;
    
    /* removed properties */
	struct	property *deadprops;	
    
    /*父node*/
	struct	device_node *parent;
    
    /*子node*/
	struct	device_node *child;
    
    /*兄弟node*/
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

### struct property

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

### struct fdt_node_header

```C
/* dtb中的node */
struct fdt_node_header {
        fdt32_t tag;
        char name[0];
};
```

### struct fdt_property

```C
/* dtb中的property */
struct fdt_property {
        fdt32_t tag;
        fdt32_t len;
        fdt32_t nameoff;
        char data[0];
};
```

### 1 unflatten_device_tree

```C
#define FDT_BEGIN_NODE	0x1		/* Start node: full name */
#define FDT_END_NODE	0x2		/* End node */
#define FDT_PROP	    0x3		/* Property: name off,size,content */  
#define FDT_NOP		    0x4		/* nop */
#define FDT_END		    0x9     /*整个structure block的结束*/

/*
 * unflatten_device_tree - create tree of device_nodes from flat blob
 * 将扁平的node转换成有层次结构的deivce_node
 * unflattens the device-tree passed by the firmware, creating the
 * tree of struct device_node. It also fills the "name" and "type"
 * pointers of the nodes so the normal device-tree walking functions
 * can be used.
 */
void __init unflatten_device_tree(void)
{
	__unflatten_device_tree(initial_boot_params, NULL, &of_root,
				early_init_dt_alloc_memory_arch, false);

	/* Get pointer to "/chosen" and "/aliases" nodes for use everywhere */     /*提取chosen和aliases节点，方便 use everywhere */
	of_alias_scan(early_init_dt_alloc_memory_arch);

	unittest_unflatten_overlay_base();
} 
```

### 1-1 __unflatten_device_tree

```C
/* 根据dtb生成嵌套的device_nodes节点 */
void *__unflatten_device_tree(const void *blob,
			      struct device_node *dad,
			      struct device_node **mynodes,
			      void *(*dt_alloc)(u64 size, u64 align),
			      bool detached)
{
	int size;
	void *mem;

    /*
     * 1. 第一遍扫描，计算dtb的structure block数量和
     *    其名字长度大小，总内存大小。
     * 2. 由此可知，device_node后面紧接着node的字符串
     *    名字，存放在连续的内存中。
     */
	size = unflatten_dt_nodes(blob, NULL, dad, NULL);
 
     /* 字节对齐，不足则填充 */
	size = ALIGN(size, 4);
   
	/* Allocate memory for the expanded device tree */
    /* 
     * 1. 分配内存，其中 + 4 为magic的大小，放于末尾。
     * 2. 其中需对齐的大小为struct device_node
     */
	mem = dt_alloc(size + 4, __alignof__(struct device_node));
	if (!mem)
		return NULL;
    
    /*最后4字节用作magic*/
	*(__be32 *)(mem + size) = cpu_to_be32(0xdeadbeef);


	/* Second pass, do actual unflattening */
    /* 传入已经申请的内存mem，展开设备树 */
	unflatten_dt_nodes(blob, mem, dad, mynodes);
    
	return mem;
}
```

### 1-1-1 unflatten_dt_nodes

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
    
/*节点的子节点，最大深度64*/    
#define FDT_MAX_DEPTH	64 
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

    
 	/* 当遍历完所有node后，depth为0，退出循环 */
	for (offset = 0;
	     offset >= 0 && depth >= initial_depth;
	     offset = fdt_next_node(blob, offset, &depth)) {
        
        /* 如果深度大于64， 忽略该node及其子node */
		if (WARN_ON_ONCE(depth >= FDT_MAX_DEPTH))
			continue;
        
        /*
         * 计算每个 device_node + 该node名字长度
         * dryrun = 0，实际初始化 device_nodee
         * dryrun = 1，只进行内存大小的统计
         */
		if (!populate_node(blob, offset, &mem, nps[depth],
				   &nps[depth+1], dryrun))
			return mem - base;

        /*
         * 第二次，传入的nodepp为of_root，
         * of_root为全局指针，为空时，初始化的第一个
         * device_node赋给of_root作为起始节点
         */
		if (!dryrun && nodepp && !*nodepp)
			*nodepp = nps[depth+1];
		if (!dryrun && !root)
			root = nps[depth+1];
	}

	return mem - base;
}
```


### 1-1-1-1 populate_node

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
    
    /*获取offset偏移大小的节点名pathp，和长度l*/
	pathp = fdt_get_name(blob, offset, &l);
	if (!pathp) {
		*pnp = NULL;
		return false;/*已经遍历完成*/
	}

	allocl = ++l; /*加上'\0'*/

    /*
     * 1. 只是做了所需申请内存统计
     * 2. mem是二级指针，mem实际就是size，累计内存大小
     * 3. size = sizeof(struct device_node) + allocl;
     *    np = mem;
     *    mem += size;
     */
	np = unflatten_dt_alloc(mem, sizeof(struct device_node) + allocl,
				__alignof__(struct device_node));
    
    
    /*
     * 根据上个调用的函数unflatten_dt_nodes
     * 第二次时，已经申请了mem，此时np则会持续往下，定位到每个node
     * 对每个node进行结构内容进行初始化
     * !dryrun，则需要做实际操作
     */    
	if (!dryrun) {
		char *fn;
        
        /*设置通用fops，ktype*/
		of_node_init(np);
        
        /* 
         * 申请的内存，数据放置格式为 ：device_node在前，string在后
         * 长度为 n*（device_node + string）
         * full_name 指向 string 的位置
         */
		np->full_name = fn = ((char *)np) + sizeof(*np);
        
        /*并复制node的名字到string位置*/
		memcpy(fn, pathp, l);

        /*记录兄弟，父，子*/
		if (dad != NULL) { 
			np->parent = dad;
			np->sibling = dad->child;
			dad->child = np;
		}
	}

    /*属性解析*/
	populate_properties(blob, offset, mem, np, pathp, dryrun);
    
	if (!dryrun) {
        
        /*找到name属性内容，赋给property->name*/
		np->name = of_get_property(np, "name", NULL);
        
        /*找不到name属性，property->name设置为<NULL>*/
		if (!np->name)
			np->name = "<NULL>"; 
	}
    
	/*返回刚刚初始化完成的device_node*/
	*pnp = np;
    
    /*只要节点没遍历完，就会一直返回ture*/
	return true; 
}
```

### 1-1-1-1-1 fdt_get_name

```C
const char *fdt_get_name(const void *fdt, int nodeoffset, int *len)
{
    /*第nodeoffset个偏移的structure block内容指针*/
	const struct fdt_node_header *nh = fdt_offset_ptr_(fdt, nodeoffset);
	const char *nameptr;
	int err;


	nameptr = nh->name;
    
    /*旧版本命名前缀带有'/'*/
	if (!can_assume(LATEST) && fdt_version(fdt) < 0x10) {
		const char *leaf;
        
        /*找到最后一次出现'/'的地方*/
		leaf = strrchr(nameptr, '/');
        
        /*偏移1*/
		nameptr = leaf+1;  
	}
    
    
	/*名字最后带'/0'，可直接计算长度*/
	if (len)
		*len = strlen(nameptr); 

	return nameptr;
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
/*获得下一个 structure block 的节点*/
int fdt_next_node(const void *fdt, int offset, int *depth)
{
	int nextoffset = 0;
	uint32_t tag;
    
    /*
     * 不是FDT_BEGIN_NODE，直接返回，说明传入的offset不是node
     */
	if (offset >= 0)
		if ((nextoffset = fdt_check_node_offset_(fdt, offset)) < 0)
			return nextoffset; 

	do {
        /*
         * 当node的tag为FDT_BEGIN_NODE，此时已越过一个tag
         */
		offset = nextoffset;
		tag = fdt_next_tag(fdt, offset, &nextoffset);/*获取下一个tag*/

		switch (tag) {
		case FDT_PROP:/*越过属性FDT_PROP和空FDT_NOP*/
		case FDT_NOP:
			break;

		case FDT_BEGIN_NODE:
			if (depth) 
				(*depth)++;/*找到子节点的FDT_BEGIN_NODE，深度+1，循环结束，返回子节点*/
			break;

		case FDT_END_NODE: 
            /*
             * 深度-1，因为起始深度为0，如此时不为0，说明嵌套于子节点中，继续遍历寻找
             * 否则说明，该root节点的子节点都已经遍历结束
             */
			if (depth && ((--(*depth)) < 0))
				return nextoffset; 
			break;

		case FDT_END:/*整个structure block 结束*/
			if ((nextoffset >= 0)
			    || ((nextoffset == -FDT_ERR_TRUNCATED) && !depth))
				return -FDT_ERR_NOTFOUND;
			else
				return nextoffset;
		}
	} while (tag != FDT_BEGIN_NODE); /*当前node的下一个子node*/

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
    
	/*符合FDT_BEGIN_NODE*/
	if (fdt_next_tag(fdt, offset, &offset) != FDT_BEGIN_NODE) 
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
    /*遍历device_node->properties链表的属性，找到同名*/
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
    
    /*
     * 1. 遍历获取node的属性
     * 2. 当到遍历完所有属性后，返回负值，则退出循环
     */
	for (cur = fdt_first_property_offset(blob, offset);
	     cur >= 0;
	     cur = fdt_next_property_offset(blob, cur)) {
		const __be32 *val;
		const char *pname;
		u32 sz;
        
        /*找到nameoff位置的字符串，并返回其长度*/
		val = fdt_getprop_by_offset(blob, cur, &pname, &sz);
        
        /*存在name属性*/
		if (!strcmp(pname, "name"))
			has_name = true;
        
        /*累计mem，即统计device_node的所需同时，也统计property*/
		pp = unflatten_dt_alloc(mem, sizeof(struct property),
					__alignof__(struct property));
        
        /* dryrun = 1，只统计不处理 */
		if (dryrun)
			continue;

		/* We accept flattened tree phandles either in
		 * ePAPR-style "phandle" properties, or the
		 * legacy "linux,phandle" properties.  If both
		 * appear and have different values, things
		 * will get weird. Don't do that.
		 */
        
        /*如果出现phandle属性，则会赋值给property->phandle*/
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
        
		/*记录属性名*/
		pp->name   = (char *)pname; 
        
        /*属性内容长度*/
		pp->length = sz; 
        
        /*属性的内容*/
		pp->value  = (__be32 *)val; 
        
		*pprev     = pp;
		pprev      = &pp->next;
	}

    /* 补全 phandle属性 */
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
        /*记录phandle属性所需大小*/
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
    /*先确认当前node是否符合FDT_BEGIN_NODE,不符合，返回错误码*/
	if ((offset = fdt_check_node_offset_(fdt, nodeoffset)) < 0)
		return offset;
    /*符合，寻找其node的属性的FDT_PROP*/
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

		case FDT_PROP:/*找到属性的标签，返回其地址偏移*/
			return offset;
		}
		offset = nextoffset;
	} while (tag == FDT_NOP);/*当找到FDT_NOP，跳过，继续往下找*/

	return -FDT_ERR_NOTFOUND;
}
```

### fdt_next_property_offset

```C
int fdt_next_property_offset(const void *fdt, int offset)
{
	if ((offset = fdt_check_prop_offset_(fdt, offset)) < 0)
		return offset;

	return nextprop_(fdt, offset);
}
```

### fdt_check_prop_offset_

```C
int fdt_check_prop_offset_(const void *fdt, int offset)
{
	if (!can_assume(VALID_INPUT)
	    && ((offset < 0) || (offset % FDT_TAGSIZE)))
		return -FDT_ERR_BADOFFSET;

	if (fdt_next_tag(fdt, offset, &offset) != FDT_PROP)
		return -FDT_ERR_BADOFFSET;

	return offset;
}
```

### fdt_getprop_by_offset

```C
const void *fdt_getprop_by_offset(const void *fdt, int offset,
				  const char **namep, int *lenp)
{
	const struct fdt_property *prop;

	prop = fdt_get_property_by_offset_(fdt, offset, lenp);
	if (!prop)
		return NULL;
	if (namep) {
		const char *name;
		int namelen;

		if (!can_assume(VALID_INPUT)) {
			name = fdt_get_string(fdt, fdt32_ld(&prop->nameoff),
					      &namelen);
			if (!name) {
				if (lenp)
					*lenp = namelen;
				return NULL;
			}
			*namep = name;
		} else {
			*namep = fdt_string(fdt, fdt32_ld(&prop->nameoff));
		}
	}

	/* Handle realignment */
	if (!can_assume(LATEST) && fdt_version(fdt) < 0x10 &&
	    (offset + sizeof(*prop)) % 8 && fdt32_ld(&prop->len) >= 8)
		return prop->data + 4; 
	return prop->data;/*返回data[0],即内容*/
}
```

### fdt_get_property_by_offset_

```C
static const struct fdt_property *fdt_get_property_by_offset_(const void *fdt,
						              int offset,
						              int *lenp)
{
	int err;
	const struct fdt_property *prop;

	if (!can_assume(VALID_INPUT) &&
	    (err = fdt_check_prop_offset_(fdt, offset)) < 0) {
		if (lenp)
			*lenp = err;
		return NULL;
	}

	prop = fdt_offset_ptr_(fdt, offset);

	if (lenp)
		*lenp = fdt32_ld(&prop->len);/*反回其内容的长度*/

	return prop;/*检查无误后，返回fdt_property结构体的首地址*/
}
```

### 总结

- 将dtb展开成层次结构，解析和记录每个node及子node，将node名及长度，phandle句柄，父，子，兄弟，保存在device_node中。

  - FDT_BEGIN_NODE：节点的起始位置。

  - FDT_END_NODE：节点的结束位置，这样就能对每个node进行区分。

  - FDT_END：解析过程的结束位置。

- 在每个device_node解析完成后，会解析node内部的property，将property名及长度，property内容，保存在property中。

  - FDT_PROP：属性的起始位置

- 其中在device_node或property字符串名的内存，会创建在对应结构体后面。

- 由此可见，保存property，但不对内容做具体分析，对device_node层次来说无意义（除了”name”,”chosen”,”alias”,”phandle”，这些会被提前解析处理）。
- 下一步，会将device_node生成platform_device，最终等到driver解析处理property，赋予特定意义。

