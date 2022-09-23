### 1 异常向量表的安装

#### 1-1 系统启动

```assembly
@ arch\arm\kernel\head.S 

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
 * 从 early_trap_init() 中知道，代码位于 __vectors_start段 
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
	
	@ 在usr mode发生中断时，调用 __irq_usr
	.long	__irq_usr				@  0  (USR_26 / USR_32)
	.long	__irq_invalid			@  1  (FIQ_26 / FIQ_32)
	.long	__irq_invalid			@  2  (IRQ_26 / IRQ_32)
	
	@ 在svc mode发生中断时，调用 __irq_svc
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

### 2 interrupt-controller 节点的初始化

#### 2-1 dts的配置

```C
intc: interrupt-controller@00a01000 {
    
    /* 在imx6ull.dtsi的节点，对"arm,cortex-a7-gic"进行 probe */
	compatible = "arm,cortex-a7-gic";
    
    /* 
     * 中断类型：如SPI(共享外设中断) PPI(私有外设中断)
     * 中断号：x
     * 触发类型：上/下边沿，高低电平
     */
	#interrupt-cells = <3>; 
	interrupt-controller;
   
    /*
     * 1. 分发器地址
     * 2. cpu interface 地址
     */    
	reg = <0x00a01000 0x1000>, 
		      <0x00a02000 0x100>;
};

```

#### 2-2 init的调用

```C
/*
 * 在 irq-gic.c 中定义了一系列的结构体
 * 将宏 IRQCHIP_DECLARE 展开后,
 * struct of_device_id cortex_a7_gic = {
 * 	   .compatible = "arm,cortex-a7-gic",
 *     .data = gic_of_init,
 * };
 * 该代码会放在"__irqchip_of_table"段
 */
IRQCHIP_DECLARE(cortex_a7_gic, "arm,cortex-a7-gic", gic_of_init);
 ...

     
/* 在start_kernel中被调用 */     
void __init irqchip_init(void)
{
     /*
      * of_device_id 已提前放入"__irqchip_of_table"段， 
      * 因此该结构会被传入进行处理。
      */
	of_irq_init(__irqchip_of_table);
	acpi_probe_device_table(irqchip);
}
     
     

/* 
 * 1.扫描dts中每个node
 * 1-1. 找到所有存在属性 "interrupt-controller"的node
 * 1-2. 分配of_intc_desc，根据当前node，设置of_intc_desc
 * 1-3. 处理每个of_intc_desc，调用私有函数 gic_of_init()
 * 1-4. 构造完毕，释放所有of_intc_desc
*/
void __init of_irq_init(const struct of_device_id *matches)
{
	const struct of_device_id *match;
	struct device_node *np, *parent = NULL;
	struct of_intc_desc *desc, *temp_desc;
	struct list_head intc_desc_list, intc_parent_list;


    /* 将有"interrupt-controller"属性的node都进行处理 */
	for_each_matching_node_and_match(np, matches, &match) {
		if (!of_find_property(np, "interrupt-controller", NULL) ||
				!of_device_is_available(np))
			continue;

		/* 申请 of_intc_desc */
		desc = kzalloc(sizeof(*desc), GFP_KERNEL);

		/* 
		 * 设置of_intc_desc 
		 * 1. 记录 gic_of_init
		 * 2. 记录自身node和父node
		 * 3. 将of_intc_desc保存到list
		 */
		desc->irq_init_cb = match->data;
		desc->dev = of_node_get(np);
		desc->interrupt_parent = of_irq_find_parent(np);
		if (desc->interrupt_parent == np)
			desc->interrupt_parent = NULL;
		list_add_tail(&desc->list, &intc_desc_list);
	}

	/* 简单理解为，为每个intc的node执行 gic_of_init() */
	while (!list_empty(&intc_desc_list)) {

		list_for_each_entry_safe(desc, temp_desc, &intc_desc_list, list) {
			int ret;
			           
            /* 标记该dt已生成device */
			of_node_set_flag(desc->dev, OF_POPULATED);

			/* 调用私有函数处理，即：gic_of_init() */
			ret = desc->irq_init_cb(desc->dev,
						desc->interrupt_parent);
		}
	}
}


     

```

#### 2-2-1 gic_of_init()

```c
/* 该函数被调用，此时会进行match,最终匹配"arm,cortex-a7-gic" */
int __init
gic_of_init(struct device_node *node, struct device_node *parent)
{
	struct gic_chip_data *gic;
	int irq, ret;
	
    /* 全局数组，因soc可能会有多个gic */
	gic = &gic_data[gic_cnt];
	
    /* 从dts中，获取分发器地址和cpu接口地址，并转换为虚拟地址 */
	ret = gic_of_setup(gic, node);

	/* 指针 handle_arch_irq 指向实际的irq函数：gic_handle_irq() */
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

##### 2-2-1-1 gic_of_setup()

```C
static int gic_of_setup(struct gic_chip_data *gic, struct device_node *node)
{
	
    /* 
     * 在dts中获取地址，并转成虚拟地址。 
     * 1. dispatcher(分发器)地址
     * 2. cpu interface(cpu接口)地址
     */
	gic->raw_dist_base = of_iomap(node, 0);
	gic->raw_cpu_base = of_iomap(node, 1);

	return 0;
}
```

##### 2-2-1-2 __gic_init_bases

```C
static struct irq_chip gic_chip = {
	.irq_mask		= gic_mask_irq,
	.irq_unmask		= gic_unmask_irq,
	.irq_eoi		= gic_eoi_irq,
	.irq_set_type		= gic_set_type,
	.irq_get_irqchip_state	= gic_irq_get_irqchip_state,
	.irq_set_irqchip_state	= gic_irq_set_irqchip_state,
	.flags		= IRQCHIP_SET_TYPE_MASKED |
				  IRQCHIP_SKIP_SET_WAKE |
				  IRQCHIP_MASK_ON_SUSPEND,
};


static int __init __gic_init_bases(struct gic_chip_data *gic,
				   int irq_start,
				   struct fwnode_handle *handle)
{
    ... 
        
    /* 设置指针, handle_arch_irq = handle_irq; */
	set_handle_irq(gic_handle_irq);
           
     /* v2版本，初始化 irq_chip */
	 gic_init_chip(gic, NULL, name, true);
    
	 ret = gic_init_bases(gic, irq_start, handle);    
    ...
}


/***********************gic_init_bases()************************************/

static const struct irq_domain_ops gic_irq_domain_hierarchy_ops = {
    
     /* 解析设备树属性,type(SPI), hwirq(15), flag(trigger edge) */
	.translate = gic_irq_domain_translate,
    
    /* 保存 hwirq 和 virq 关系在 irq_domain */
	.alloc = gic_irq_domain_alloc,
	.free = irq_domain_free_irqs_top,
};


static int gic_init_bases(struct gic_chip_data *gic, int irq_start,
			  struct fwnode_handle *handle)
{
    ... 
    
    /* 
     * 申请，设置irq_domain 
     * 1. 保存通用ops
     * 2. 保存私有数据gic_chip
     * 3. 添加到全局list
     */    
 	if (handle) {		/* DT/ACPI */
		gic->domain = irq_domain_create_linear(handle, gic_irqs,
						       &gic_irq_domain_hierarchy_ops,
						       gic);
	}      
    ...
}
```


### 3 节点使用&intc

```c
/* 1. 根据 dt 构造 device_node */

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
    
   	/*统计使用的中断数量*/     
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
int of_irq_to_resource(struct device_node *dev, int index, struct resource *r)
{
	int irq = irq_of_parse_and_map(dev, index);

	if (r && irq) {
		const char *name = NULL;

        /* 清空 */
		memset(r, 0, sizeof(*r));
        
        /* 设置中断号，类型为中断 */
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

    /*解析DT，并保存在 of_phandle_args */
	if (of_irq_parse_one(dev, index, &oirq))
		return 0;
	
    /* */
	return irq_create_of_mapping(&oirq);
}
```

###### 3-1-3-1 of_irq_parse_one()

```C
/* 
 * 1. 解析第 index 个 "interrupts" 属性。 
 * 2. 将内容保存在 of_phandle_args 中并返回。
 */
int of_irq_parse_one(struct device_node *device, int index, struct of_phandle_args *out_irq)
{
	struct device_node *p;
	const __be32 *intspec, *tmp, *addr;
	u32 intsize, intlen;
	int i, res;

   
	/*分支1：尝试解析新属性"interrupts-extended" */
	res = of_parse_phandle_with_args(device, "interrupts-extended",
					"#interrupt-cells", index, out_irq);
	if (!res)
		return of_irq_parse_raw(addr, out_irq);

      
	/* 分支2：解析"interrupts"属性 */
	intspec = of_get_property(device, "interrupts", &intlen);

    /* 中断的个数 */
	intlen /= sizeof(*intspec);

	/* 寻找"interrupt-parent"属性，指向父node的phandle */
	p = of_irq_find_parent(device);

	/* 获取cells长度 */
	tmp = of_get_property(p, "#interrupt-cells", NULL);
	intsize = be32_to_cpu(*tmp);

	/* 
	 * 如 interrupts = <18 IRQ_TYPE_EDGE_FALLING>
	 * 偏移第index个位置
     */
	intspec += index * intsize;
    
	out_irq->np = p;
	out_irq->args_count = intsize;
    
    /*保存中断号，触发flag*/
	for (i = 0; i < intsize; i++)
		out_irq->args[i] = be32_to_cpup(intspec++);

	return res;
}
```

###### 3-1-3-2 irq_create_of_mapping()

```C
unsigned int irq_create_of_mapping(struct of_phandle_args *irq_data)
{
	struct irq_fwspec fwspec;

    /* 
     * 将 of_phandle_args 内容复制到 irq_fwspec 
     * 主要是：cells长度，中断号，中断flag
     */
	of_phandle_args_to_fwspec(irq_data, &fwspec);
    
    /* 创建中断映射 */
	return irq_create_fwspec_mapping(&fwspec);
}



unsigned int irq_create_fwspec_mapping(struct irq_fwspec *fwspec)
{
	struct irq_domain *domain;
	struct irq_data *irq_data;
	irq_hw_number_t hwirq;
	unsigned int type = IRQ_TYPE_NONE;
	int virq;

    /* 寻找 irq_domain */
	if (fwspec->fwnode) {
		domain = irq_find_matching_fwspec(fwspec, DOMAIN_BUS_WIRED);
		if (!domain)
			domain = irq_find_matching_fwspec(fwspec, DOMAIN_BUS_ANY);
	} else {
		domain = irq_default_domain;
	}


    /* 调用 gic_irq_domain_translate()，获取hwirq和type */
	if (irq_domain_translate(domain, fwspec, &hwirq, &type))
		return 0;


	/* 检查是否已经建立映射 */
	virq = irq_find_mapping(domain, hwirq);
    

    
	/*没有，则创建映射*/
	if (irq_domain_is_hierarchy(domain)) {
        /* 寻找空闲的virq，并调用alloc设置irq_desc */
		virq = irq_domain_alloc_irqs(domain, 1, NUMA_NO_NODE, fwspec);
		if (virq <= 0)
			return 0;
	} else {
		/* Create mapping */
		virq = irq_create_mapping(domain, hwirq);
		if (!virq)
			return virq;
	}


	return virq;
}
```

######  3-1-3-2-1 gic_irq_domain_translate()

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
    /* gic */
	if (is_of_node(fwspec->fwnode)) {

        /* 0~15为SGI软件中断 */
		*hwirq = fwspec->param[1] + 16;

        /* 
         * 16~31为SPI共享中断，如果是GIC_SPI(该宏为0)，则需再偏移16
         * 因此结果：89+16+16=121号
         */
		if (!fwspec->param[0])
			*hwirq += 16;

		*type = fwspec->param[2] & IRQ_TYPE_SENSE_MASK;
		return 0;
	}

    /* 子节点 */
	if (is_fwnode_irqchip(fwspec->fwnode)) {
		if(fwspec->param_count != 2)
			return -EINVAL;

        /*返回中断号，中断类型*/
		*hwirq = fwspec->param[0];
		*type = fwspec->param[1];
		return 0;
	}

	return -EINVAL;
}
```

###### 3-1-3-2-2 gic_irq_domain_alloc 

```c
static int gic_irq_domain_alloc(struct irq_domain *domain, unsigned int virq,
				unsigned int nr_irqs, void *arg)
{
	int i, ret;
	irq_hw_number_t hwirq;
	unsigned int type = IRQ_TYPE_NONE;
	struct irq_fwspec *fwspec = arg;

    /*解析和读取dts的节点*/
	ret = gic_irq_domain_translate(domain, fwspec, &hwirq, &type);


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

##### irq_set_probe()

```C
```

### 4 hwirq和virq映射关系

### request_irq()

```C
static inline int __must_check
request_irq(unsigned int irq, irq_handler_t handler, unsigned long flags,
	    const char *name, void *dev)
{
	return request_threaded_irq(irq, handler, NULL, flags, name, dev);
}



int request_threaded_irq(unsigned int irq, irq_handler_t handler,
			 irq_handler_t thread_fn, unsigned long irqflags,
			 const char *devname, void *dev_id)
{
	struct irqaction *action;
	struct irq_desc *desc;
	int retval;


	/*共享中断必须有dev_id*/
	if (((irqflags & IRQF_SHARED) && !dev_id) ||
	    (!(irqflags & IRQF_SHARED) && (irqflags & IRQF_COND_SUSPEND)) ||
	    ((irqflags & IRQF_NO_SUSPEND) && (irqflags & IRQF_COND_SUSPEND)))
		return -EINVAL;

    
	desc = irq_to_desc(irq);


	if (!irq_settings_can_request(desc) ||
	    WARN_ON(irq_settings_is_per_cpu_devid(desc)))
		return -EINVAL;

	if (!handler) {
		if (!thread_fn)
			return -EINVAL;
		handler = irq_default_primary_handler;
	}

	action = kzalloc(sizeof(struct irqaction), GFP_KERNEL);
	if (!action)
		return -ENOMEM;

	action->handler = handler;
	action->thread_fn = thread_fn;
	action->flags = irqflags;
	action->name = devname;
	action->dev_id = dev_id;



	retval = __setup_irq(irq, desc, action);

	return retval;
}
```

#### 总结

1 异常向量表的安装

- linux的链接脚本会将向量表存入 __vectors_start 段。
- 系统启动后，创建虚拟内存空间(0xffff0000)，复制 __vectors_start 段到该虚拟空间。
- 设置中断跳转函数，当产生irq时，会跳转到该函数进行分支处理。

2 “intc”中断子系统的初始化

- 在编译时，会将一些 of_device_id 存入 __irqchip_of_table 段，系统启动后会检查是否能够match。
- 然后就会调用gic_of_init()，初始化1个或多个interrupt-controller节点。
- 解析dt节点"intc"，申请和设置irq_chip(soc的某个gic的寄存器组操作)。
- 申请和设置irq_domain(管理hwirq和virq)。当子节点调用request_irq()，会调用irq_domain的函数，建立映射关系。

3 使用”intc”子系统的子节点的dt预处理

- 在”soc”节点中，因存在属性"simple-bus"，子节点会生成platform_device。
- 其中就包含了如"uart1" “gpio1” “sai1” 外设，某些子节点使用了中断子系统"intc"。
- 生成platform_device中，会调用到of_deivce_alloc()。
  - 统计使用的irq数量。
  - 解析子节点在dt中的"interrupts"属性的内容，<SPI，hwirq，flag>
  - 寻找空闲virq，根据virq申请irq_desc，建立与hwirq的映射关系，并保存在irq_domain。
  - 申请resource，将“reg”属性的内容保存在resource

4 当子节点driver调用request_irq

- 根据传入的virq，找到对应的irq_desc，进行设置。
- 分配irqaction结构，填充字段。
  - 将回调函数(可能多个)，放入irq_desc->action->handler。
  - 将线程函数放入irq_desc->action->thread_fn。
  - 将dev_id保存(用于共享中断)。
- 线程化的内存申请，或者在workqueue添加回调相关的函数。

5 产生中断的处理(gic_handle_irq)

- mask中断。

- 唤醒中断子线程(如果有线程化)。workqueue不需要唤醒，会一直存在。
- 调用gic中断处理函数，让cpu读取gic寄存器，获取产生哪个中断hwirq。
- 根据hwirq找到virq，如gpio，则会调用gpio的中断处理函数。
- 然后通过gpio寄存器，获取是哪个pin产生的中断，确认是哪种GPIO类型中断，并且该类型中断是否有对应的virq注册，如存在则调用action的回调函数。
- 如果有多少回调函数，则会一个个全部执行。
- 如果是共享中断，在回调函数里，还会再进一步查询，是哪个外设产生了该pin的中断。
- 处理完上半部中断，umask中断并退出，此时不影响其他中断的发送和处理。
- 当中断子线程或workqueue有条件时进行调度，进行中断下半部的处理。



如果是uart这种外设的中断，中断线只有一根，如uart的中断号为26，则产生中断后上半部的处理(合理猜测)，如果有调用过request_irq，则会遍历查找所有注册了的virq，确认该virq对应的uart中断类型，是否产生了该类型的中断类型，如接收完成中断，发送空中断，tx/rx dma中断等，找到对应的回调函数执行。

