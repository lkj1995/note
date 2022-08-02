### 1 异常向量表的安装

#### 1-1 系统启动

```assembly
@@ arch\arm\kernel\head.S @@
@ 1. 取出处理器ID，并跳转到__lookup_processor_type
mrc	p15, 0, r9, c0, c0		@ get processor id
bl	__lookup_processor_type		@ r5=procinfo r9=cpuid
	
@ 2. 跳转到__create_page_tables，创建虚拟地址的页表	
bl	__create_page_tables

@ 3. 使能和打开mmu，并返回r13保存的地址，即__mmap_switched
ldr	r13, =__mmap_switched

b	__enable_mmu
b	__turn_mmu_on
   mov	r3, r13
   ret	r3

@@ arch\arm\kernel\head-common.S @@
@ 执行__mmap_switched,最终跳转到start_kernel
__mmap_switched :
 ...
b	start_kernel
```

#### 1-2 复制向量表

```C
/* 1. init\main.c */
start_kernel();

/* 2. arch\arm\kernel\setup.c */
setup_arch(&command_line);

/* 3. arch\arm\mm\mmu.c */
paging_init(mdesc); 


static void __init devicemaps_init(const struct machine_desc *mdesc)
{
	/*
	 * Allocate the vector page early.
	 */
    /*分配新向量表*/
	vectors = early_alloc(PAGE_SIZE * 2);
    
	/*
	 * 从代码将向量表复制到新向量表
	 * Copy the vectors, stubs and kuser helpers (in entry-armv.S)
	 * into the vector page, mapped at 0xffff0000, and ensure these
	 * are visible to the instruction stream.
	 *
	 *  early_trap_init() {
	 *   代码位于__vectors_start段                  
	 *   memcpy((void *)vectors, __vectors_start, 
	 *                              __vectors_end - __vectors_start);
	 *   memcpy((void *)vectors + 0x1000, __stubs_start, 
	 *                              __stubs_end - __stubs_start);
	 * }
	 */
	early_trap_init(vectors);
    
	/*
	 * 映射新向量表到虚拟地址0xffff0000
	 * Create a mapping for the machine vectors at the high-vectors
	 * location (0xffff0000).  If we aren't using high-vectors, also
	 * create a mapping at the low-vectors virtual address.
	 */
	map.pfn = __phys_to_pfn(virt_to_phys(vectors));
	map.virtual = 0xffff0000;
	map.length = PAGE_SIZE;    
    
}
```

#### 1-3 向量表位置

````c
/*
 * 从 early_trap_init() 中直到，代码位于 __vectors_start段 
 */
	.section .vectors, "ax", %progbits
.L__vectors_start:
	W(b)	vector_rst
	W(b)	vector_und
	W(ldr)	pc, .L__vectors_start + 0x1000
	W(b)	vector_pabt
	W(b)	vector_dabt
	W(b)	vector_addrexcptn
	W(b)	vector_irq
	W(b)	vector_fiq

        
/* 链接脚本 arch/arm/kernel/vmlinux.lds */
 __vectors_start = .;
 .vectors 0xffff0000 : AT(__vectors_start) {
  *(.vectors)
 }
 . = __vectors_start + SIZEOF(.vectors);
 __vectors_end = .;
 __stubs_start = .;
 .stubs ADDR(.vectors) + 0x1000 : AT(__stubs_start) {
  *(.stubs)
 }

/*从上可知，此段代码(*.vectors)会被链接脚本连接到__vectors_start段*/
````

#### 1-4 中断发生

```assembly
/*
 * Interrupt dispatcher
 */
	vector_stub	irq, IRQ_MODE, 4
	
	@ 在usr mode发送中断时，调用 __irq_usr
	.long	__irq_usr				@  0  (USR_26 / USR_32)
	.long	__irq_invalid			@  1  (FIQ_26 / FIQ_32)
	.long	__irq_invalid			@  2  (IRQ_26 / IRQ_32)
	
	@ 在svc mode发送中断时，调用 __irq_svc
	.long	__irq_svc				@  3  (SVC_26 / SVC_32)
	.long	__irq_invalid			@  4
	.long	__irq_invalid			@  5
	.long	__irq_invalid			@  6
	.long	__irq_invalid			@  7
	.long	__irq_invalid			@  8
	.long	__irq_invalid			@  9
	.long	__irq_invalid			@  a
	.long	__irq_invalid			@  b
	.long	__irq_invalid			@  c
	.long	__irq_invalid			@  d
	.long	__irq_invalid			@  e
	.long	__irq_invalid			@  f
	



@ 如1-3中的 "vector_\name" --> "vector_irq"
vector_\name:
	.if \correction
	sub	lr, lr, #\correction
	.endif

	@
	@ Save r0, lr_<exception> (parent PC) and spsr_<exception>
	@ (parent CPSR)
	@
	stmia	sp, {r0, lr}		@ save r0, lr
	mrs	lr, spsr
	str	lr, [sp, #8]		@ save spsr

	@
	@ Prepare for SVC32 mode.  IRQs remain disabled.
	@
	mrs	r0, cpsr
	eor	r0, r0, #(\mode ^ SVC_MODE | PSR_ISETSTATE)
	msr	spsr_cxsf, r0

	@
	@ the branch table must immediately follow this code
	@ 发生中断时，判断当前处于哪种mode
	@
	and	lr, lr, #0x0f
	
 THUMB(	adr	r0, 1f			)
 THUMB(	ldr	lr, [r0, lr, lsl #2]	)
	mov	r0, sp
	
	@ 分支跳转，如 __irq_usr 或 __irq_svc
 ARM(	ldr	lr, [pc, lr, lsl #2]	)
	movs	pc, lr			@ branch to handler in SVC mode
ENDPROC(vector_\name)	
```

#### 1-5 分支处理

```assembly
@@ 分支 __irq_usr
 .align	5
__irq_usr:
	usr_entry                 @ 保存现场
	kuser_cmpxchg_check  
	irq_handler               @ 处理
	get_thread_info tsk
	mov	why, #0
	b	ret_to_user_from_irq  @ 恢复现场
 UNWIND(.fnend		)
ENDPROC(__irq_usr)


@@ 分支 __irq_svc
	.align	5
__irq_svc:
	svc_entry            @ 保存现场
	irq_handler          @ 处理

#ifdef CONFIG_PREEMPT
	ldr	r8, [tsk, #TI_PREEMPT]		@ get preempt count
	ldr	r0, [tsk, #TI_FLAGS]		@ get flags
	teq	r8, #0				@ if preempt count != 0
	movne	r0, #0				@ force flags to 0
	tst	r0, #_TIF_NEED_RESCHED
	blne	svc_preempt
#endif

	svc_exit r5, irq = 1			@ return from exception 恢复现场
 UNWIND(.fnend		)
ENDPROC(__irq_svc)
```

#### 1-6 处理函数(gic提供)

```assembly
/*
 * Interrupt handling.
 */
	.macro	irq_handler
#ifdef CONFIG_MULTI_IRQ_HANDLER 
	ldr	r1, =handle_arch_irq  @ 处理函数：handle_arch_irq
	mov	r0, sp
	badr	lr, 9997f
	ldr	pc, [r1]
#else
	arch_irq_handler_default
#endif
```

### 2 gic处理流程

#### 1-1 dts的配置

```C
/* 在imx6ull.dtsi的节点，对"arm,cortex-a7-gic"进行 probe */
intc: interrupt-controller@00a01000 {
	compatible = "arm,cortex-a7-gic";
	#interrupt-cells = <3>;
	interrupt-controller;
	reg = <0x00a01000 0x1000>, /*分发器地址*/
		      <0x00a02000 0x100>; /*cpu interface 地址*/
};
```

#### 1-2 init的调用

```C
/*在 irq-gic.c 中定义了一系列的结构体 */
IRQCHIP_DECLARE(gic_400, "arm,gic-400", gic_of_init);
IRQCHIP_DECLARE(arm11mp_gic, "arm,arm11mp-gic", gic_of_init);
IRQCHIP_DECLARE(arm1176jzf_dc_gic, "arm,arm1176jzf-devchip-gic", gic_of_init);
IRQCHIP_DECLARE(cortex_a15_gic, "arm,cortex-a15-gic", gic_of_init);
IRQCHIP_DECLARE(cortex_a9_gic, "arm,cortex-a9-gic", gic_of_init);
IRQCHIP_DECLARE(cortex_a7_gic, "arm,cortex-a7-gic", gic_of_init);
IRQCHIP_DECLARE(msm_8660_qgic, "qcom,msm-8660-qgic", gic_of_init);
IRQCHIP_DECLARE(msm_qgic2, "qcom,msm-qgic2", gic_of_init);
IRQCHIP_DECLARE(pl390, "arm,pl390", gic_of_init);


/* 该函数被调用，此时会进行match,最终匹配"arm,cortex-a7-gic" */
int __init
gic_of_init(struct device_node *node, struct device_node *parent)
{
	struct gic_chip_data *gic;
	int irq, ret;

	if (WARN_ON(!node))
		return -ENODEV;

	if (WARN_ON(gic_cnt >= CONFIG_ARM_GIC_MAX_NR))
		return -EINVAL;
	
	gic = &gic_data[gic_cnt];
	
    /* 从dts中，获取分发器和cpu interface虚拟地址 */
	ret = gic_of_setup(gic, node);

	/* 将实际执行的irq函数存到指针 handle_arch_irq */
	ret = __gic_init_bases(gic, -1, &node->fwnode);

	if (parent) {
		irq = irq_of_parse_and_map(node, 0);
		gic_cascade_irq(gic_cnt, irq);
	}
    
	/* 递增，用到时才寻找数组中的空位，进行init */
	gic_cnt++;
	return 0;
}
```

##### 1-2-1 gic_of_setup()

```C
static int gic_of_setup(struct gic_chip_data *gic, struct device_node *node)
{
	if (!gic || !node)
		return -EINVAL;
	
    /* dts中获取分发器地址，并转成虚拟地址 */
	gic->raw_dist_base = of_iomap(node, 0);
	if (WARN(!gic->raw_dist_base, "unable to map gic dist registers\n"))
		goto error;
    
	/* dts中获取 cpu interface 地址，并转成虚拟地址 */
	gic->raw_cpu_base = of_iomap(node, 1);
	if (WARN(!gic->raw_cpu_base, "unable to map gic cpu registers\n"))
		goto error;

	if (of_property_read_u32(node, "cpu-offset", &gic->percpu_offset))
		gic->percpu_offset = 0;

	return 0;

error:
	gic_teardown(gic);

	return -ENOMEM;
}
```

##### 1-2-2 __gic_init_bases()

```C
static struct irq_chip gic_chip = {
	.irq_mask		= gic_mask_irq,
	.irq_unmask		= gic_unmask_irq,
	.irq_eoi		= gic_eoi_irq,
	.irq_set_type		= gic_set_type,
	.irq_get_irqchip_state	= gic_irq_get_irqchip_state,
	.irq_set_irqchip_state	= gic_irq_set_irqchip_state,
	.flags			= IRQCHIP_SET_TYPE_MASKED |
				  IRQCHIP_SKIP_SET_WAKE |
				  IRQCHIP_MASK_ON_SUSPEND,
};


static int __init __gic_init_bases(struct gic_chip_data *gic,
				   int irq_start,
				   struct fwnode_handle *handle)
{
    ... 
        
    /* 设置指针 */
	set_handle_irq(gic_handle_irq);
    
	if (static_key_true(&supports_deactivate) && gic == &gic_data[0]) {
		name = kasprintf(GFP_KERNEL, "GICv2");
        
        /* v2版本，初始化 irq_chip */
		gic_init_chip(gic, NULL, name, true);
        
	} else {
		name = kasprintf(GFP_KERNEL, "GIC-%d", (int)(gic-&gic_data[0]));
		gic_init_chip(gic, NULL, name, false);
	}    
    
	ret = gic_init_bases(gic, irq_start, handle);    
    ...
}

/***********************set_handle_irq()************************************/

void __init set_handle_irq(void (*handle_irq)(struct pt_regs *))
{
	if (handle_arch_irq)
		return;
	
    /*从这里可以看到 1-6 中被调用的指针，实际的调用函数为 gic_handle_irq */
	handle_arch_irq = handle_irq;
}


/***********************gic_init_bases()************************************/

static const struct irq_domain_ops gic_irq_domain_hierarchy_ops = {
    /*解析设备树属性*/
	.translate = gic_irq_domain_translate,
	.alloc = gic_irq_domain_alloc,
	.free = irq_domain_free_irqs_top,
};


static int gic_init_bases(struct gic_chip_data *gic, int irq_start,
			  struct fwnode_handle *handle)
{
    ... 
    
    /* 创建了 irq_domain */    
 	if (handle) {		/* DT/ACPI */
		gic->domain = irq_domain_create_linear(handle, gic_irqs,
						       &gic_irq_domain_hierarchy_ops,
						       gic);
	}    
    
    ...
}
```

##### 1-2-3  gic_irq_domain_alloc()

```C


static int gic_irq_domain_alloc(struct irq_domain *domain, unsigned int virq,
				unsigned int nr_irqs, void *arg)
{
	int i, ret;
	irq_hw_number_t hwirq;
	unsigned int type = IRQ_TYPE_NONE;
	struct irq_fwspec *fwspec = arg;

    /*解析和读取dts的节点*/
	ret = gic_irq_domain_translate(domain, fwspec, &hwirq, &type);
	if (ret)
		return ret;

    /*
     * 1. 将所有的 hwirq 与 virq 建立联系，并保存在 irq_domain 
     * 2. 设置irq_desc
     */
	for (i = 0; i < nr_irqs; i++)
		gic_irq_domain_map(domain, virq + i, hwirq + i);

	return 0;
}

/**********************gic_irq_domain_map()*************************/
/**
 * struct irq_desc - interrupt descriptor
 * @handle_irq:		highlevel irq-events handler
 * @action:		the irq action chain
 * @kobj:		kobject used to represent this struct in sysfs
 */
struct irq_desc {
    ...
        
	struct irq_data		irq_data;   /*私有数据*/
	irq_flow_handler_t	handle_irq; /* 保存 handle_fasteoi_irq */
	struct irqaction	*action;	/* 用户的irq回调函数 */
    
	struct kobject		kobj;
    
    ...
}

static int gic_irq_domain_map(struct irq_domain *d, unsigned int irq,
				irq_hw_number_t hw)
{
	/* 记录 hwirq 和 virq 在 irq_domain，并提供chip，host_data私有数据,handle函数 */
	irq_domain_set_info(d, irq, hw, &gic->chip, d->host_data,
				 handle_fasteoi_irq, NULL, NULL);
    
    /* 设置irq_desc */
	irq_set_probe(irq);

}
```

##### 1-2-4 gic_irq_domain_translate()

```C
/*
 * 	gpc: gpc@020dc000 {
 * 		compatible = "fsl,imx6ul-gpc", "fsl,imx6q-gpc";
 * 		reg = <0x020dc000 0x4000>;
 *		interrupt-controller;
 *		#interrupt-cells = <3>;
 *      意思为从GIC_SPI开始的第89个中断，因此转换后为121号
 *		interrupts = <GIC_SPI 89 IRQ_TYPE_LEVEL_HIGH>;
 *		interrupt-parent = <&intc>;
 *		fsl,mf-mix-wakeup-irq = <0xfc00000 0x7d00 0x0 0x1400640>;
 *	}; 
 */

/*解析和读取dts的节点，如上述所示，有引用 &intc 的节点 */
static int gic_irq_domain_translate(struct irq_domain *d,
				    struct irq_fwspec *fwspec,
				    unsigned long *hwirq,
				    unsigned int *type)
{
	if (is_of_node(fwspec->fwnode)) {
		if (fwspec->param_count < 3)
			return -EINVAL;

		/* Get the interrupt number and add 16 to skip over SGIs */
        /* 0~15为SGI软件中断 */
		*hwirq = fwspec->param[1] + 16;

		/*
		 * For SPIs, we need to add 16 more to get the GIC irq
		 * ID number
		 */
        /* 
         * 16~31为SPI共享中断，如果是GIC_SPI(该宏为0)，则需再偏移16
         * 因此结果：89+16+16=121号
         */
		if (!fwspec->param[0])
			*hwirq += 16;

		*type = fwspec->param[2] & IRQ_TYPE_SENSE_MASK;
		return 0;
	}

	if (is_fwnode_irqchip(fwspec->fwnode)) {
		if(fwspec->param_count != 2)
			return -EINVAL;

		*hwirq = fwspec->param[0];
		*type = fwspec->param[1];
		return 0;
	}

	return -EINVAL;
}
```

### 3 调用过程

```c
/* 1. 根据 dtb 构造 device_node */

/* 2. 根据 device_node 构造 platform_device */
of_platform_default_populate_init();
	of_platform_default_populate();
		of_platform_populate();
			of_platform_bus_create();
				of_platform_device_create_pdata();
					of_device_alloc();
```

#### 3-1 of_device_alloc()

```C
struct platform_device *of_device_alloc(struct device_node *np,
				  const char *bus_id,
				  struct device *parent)
{
	...
    /*分配platform_device*/ 
	dev = platform_device_alloc("", -1);
    
   	/*统计中断的数量*/     
	num_irq = of_irq_count(np);
    
    /* 根据irq的信息,为每个irq构造一个resource */
	if (of_irq_to_resource_table(np, res, num_irq) != num_irq)
		pr_debug("not all legacy IRQ resources mapped for %s\n",
				np->name); 
	...
}
```

##### 3-1-1 of_irq_to_resource_table()

```C
/**
 * of_irq_to_resource_table - Fill in resource table with node's IRQ info
 * @dev: pointer to device tree node
 * @res: array of resources to fill in
 * @nr_irqs: the number of IRQs (and upper bound for num of @res elements)
 *
 * Returns the size of the filled in table (up to @nr_irqs).
 */
int of_irq_to_resource_table(struct device_node *dev, struct resource *res,
		int nr_irqs)
{
	int i;
	
    /* 根据irq的信息,为每个irq构造一个resource */
	for (i = 0; i < nr_irqs; i++, res++)
		if (!of_irq_to_resource(dev, i, res))
			break;

	return i;
}
```

##### 3-1-2 of_irq_to_resource()

```C
/**
 * of_irq_to_resource - Decode a node's IRQ and return it as a resource
 * @dev: pointer to device tree node
 * @index: zero-based index of the irq
 * @r: pointer to resource structure to return result into.
 */
int of_irq_to_resource(struct device_node *dev, int index, struct resource *r)
{
	int irq = irq_of_parse_and_map(dev, index);

	/* Only dereference the resource if both the
	 * resource and the irq are valid. */
	if (r && irq) {
		const char *name = NULL;

		memset(r, 0, sizeof(*r));
		/*
		 * Get optional "interrupt-names" property to add a name
		 * to the resource.
		 */
		of_property_read_string_index(dev, "interrupt-names", index,
					      &name);

		r->start = r->end = irq;
		r->flags = IORESOURCE_IRQ | irqd_get_trigger_type(irq_get_irq_data(irq));
		r->name = name ? name : of_node_full_name(dev);
	}

	return irq;
}
```

##### 3-1-3 irq_of_parse_and_map()

```C
#define MAX_PHANDLE_ARGS 16
struct of_phandle_args {
	struct device_node *np;
	int args_count;
	uint32_t args[MAX_PHANDLE_ARGS];
};

unsigned int irq_of_parse_and_map(struct device_node *dev, int index)
{
	struct of_phandle_args oirq;

	if (of_irq_parse_one(dev, index, &oirq))
		return 0;

	return irq_create_of_mapping(&oirq);
}



/**
 * of_irq_parse_one - Resolve an interrupt for a device
 * @device: the device whose interrupt is to be resolved
 * @index: index of the interrupt to resolve
 * @out_irq: structure of_irq filled by this function
 *
 * This function resolves an interrupt for a node by walking the interrupt tree,
 * finding which interrupt controller node it is attached to, and returning the
 * interrupt specifier that can be used to retrieve a Linux IRQ number.
 */
int of_irq_parse_one(struct device_node *device, int index, struct of_phandle_args *out_irq)
{
	struct device_node *p;
	const __be32 *intspec, *tmp, *addr;
	u32 intsize, intlen;
	int i, res;

	pr_debug("of_irq_parse_one: dev=%s, index=%d\n", of_node_full_name(device), index);

	/* OldWorld mac stuff is "special", handle out of line */
	if (of_irq_workarounds & OF_IMAP_OLDWORLD_MAC)
		return of_irq_parse_oldworld(device, index, out_irq);

	/* Get the reg property (if any) */
	addr = of_get_property(device, "reg", NULL);

	/* Try the new-style interrupts-extended first */
	res = of_parse_phandle_with_args(device, "interrupts-extended",
					"#interrupt-cells", index, out_irq);
	if (!res)
		return of_irq_parse_raw(addr, out_irq);

	/* Get the interrupts property */
	intspec = of_get_property(device, "interrupts", &intlen);
	if (intspec == NULL)
		return -EINVAL;

	intlen /= sizeof(*intspec);

	pr_debug(" intspec=%d intlen=%d\n", be32_to_cpup(intspec), intlen);

	/* Look for the interrupt parent. */
	p = of_irq_find_parent(device);
	if (p == NULL)
		return -EINVAL;

	/* Get size of interrupt specifier */
	tmp = of_get_property(p, "#interrupt-cells", NULL);
	if (tmp == NULL) {
		res = -EINVAL;
		goto out;
	}
	intsize = be32_to_cpu(*tmp);

	pr_debug(" intsize=%d intlen=%d\n", intsize, intlen);

	/* Check index */
	if ((index + 1) * intsize > intlen) {
		res = -EINVAL;
		goto out;
	}

	/* Copy intspec into irq structure */
	intspec += index * intsize;
	out_irq->np = p;
	out_irq->args_count = intsize;
	for (i = 0; i < intsize; i++)
		out_irq->args[i] = be32_to_cpup(intspec++);

	/* Check if there are any interrupt-map translations to process */
	res = of_irq_parse_raw(addr, out_irq);
 out:
	of_node_put(p);
	return res;
}
```

##### 3-1-4 
