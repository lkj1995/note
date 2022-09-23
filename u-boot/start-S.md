## 操作步骤

- 将u-boot编译后进行分析。

- 打开u-boot根目录下的u-boot.lds和u-boot.map。

- u-boot.lds可查看段的存放顺序。
- u-boot.map可查看段映射到主存的实际地址和所有.o文件链接的地址位置，还有段的大小和.o文件的大小。
- u-boot.map可知，u-boot的代码起始地址(.text .data .rodata .bss等)，保存在0x87800000。

## 第一个执行的函数 reset
```assembly
@ \arch\arm\cpu\armv7\start.S
reset:
	b	save_boot_params    #  ---[1]
	
save_boot_params_ret:	@没想到吧!又跳回来了
	mrs	r0, cpsr
	and	r1, r0, #0x1f		@ mask mode bits
	teq	r1, #0x1a		@ test for HYP mode
	bicne	r0, r0, #0x1f		@ clear all mode bits
	orrne	r0, r0, #0x13		@ set SVC mode     ---[2]
	orr	r0, r0, #0xc0		@ disable FIQ and IRQ  ---[3]
	msr	cpsr,r0	

	@  -- [4] start ----
	mrc	p15, 0, r0, c1, c0, 0	@ Read CP15 SCTLR Register
	bic	r0, #CR_V		@ V = 0
	mcr	p15, 0, r0, c1, c0, 0	@ Write CP15 SCTLR Register
	@ Set vector address in CP15 VBAR register
	ldr	r0, =_start 
	mcr	p15, 0, r0, c12, c0, 0	@Set VBAR
	@  -- [4] end ----                 

	bl	cpu_init_cp15	 @  --[5]      
	bl	cpu_init_crit    @  --[6]
	bl	_main            @  --[7]
```

1. 又跳转回去了，" save_boot_params  ->  save_boot_params_ret "。

2. 通过CPSR寄存器，设置为 SVC mode。

3. 通过CPSR寄存器，禁止 FIQ 和 IRQ。

4. 设置 SCTLR 寄存器 的 V bit 为0，设置vector(中断向量表)地址为_start。

5. 通过CP15寄存器，设置 cache, MMU, TLBs。

### 6. cpu_init_crit 

   ```assembly
   @ "\arch\arm\cpu\armv7\lowlevel_init.S"
   WEAK(lowlevel_init)
   
   	ldr	sp, =CONFIG_SYS_INIT_SP_ADDR  @设置临时stack地址 ---[6-1]
   	bic	sp, sp, #7 /* 8-byte alignment for ABI compliance */
   	
   	sub	sp, sp, #GD_SIZE
   	bic	sp, sp, #7
   	mov	r9, sp	
   	
   	push	{ip, lr}
   	bl	s_init  @ ---[6-2]
   	pop	{ip, pc}	
   	
   ENDPROC(lowlevel_init)	
   ```

#### [6-1]

   ```C
   /* 1 设置的stack地址 */
   #define CONFIG_SYS_INIT_SP_ADDR \
   	(CONFIG_SYS_INIT_RAM_ADDR + CONFIG_SYS_INIT_SP_OFFSET)
   
   /* 1-1 */
   #define CONFIG_SYS_INIT_RAM_ADDR	IRAM_BASE_ADDR
   /* 1-2-1 imx6ull内部RAM起始地址,"OCRAM 128KB" */
   #define IRAM_BASE_ADDR			0x00900000 
   
   
   /* 1-2 */
   #define CONFIG_SYS_INIT_SP_OFFSET \
   	(CONFIG_SYS_INIT_RAM_SIZE - GENERATED_GBL_DATA_SIZE)
   /* 1-2-1 */
   #define CONFIG_SYS_INIT_RAM_SIZE	IRAM_SIZE
   /* 1-2-1-1 大小128KB */
   #define IRAM_SIZE                    0x00020000
   /* 1-2-1-2 */
   #define GENERATED_GBL_DATA_SIZE      256
   
   /* 计算,即内部RAM的末尾的256个字节 */
   sp = CONFIG_SYS_INIT_SP_ADDR = 
       0x00900000 + 0x00020000 - 256 = 0x0091FF00
       
   /* 再减去 #GD_SIZE */    
   #define GD_SIZE    248  
       
   /* 最终sp指向的地址为 */    
   sp = 0x0091FF00 - 248 = 0x0091FE08  
   ```

#### [6-2]  s_init

   ```C
   /* \arch\arm\cpu\armv7\mx6\soc.c */
   void s_init(void)
   {
       struct anatop_regs *anatop = (struct anatop_regs *)ANATOP_BASE_ADDR;
       struct mxc_ccm_reg *ccm = (struct mxc_ccm_reg *)CCM_BASE_ADDR;
       u32 mask480;
       u32 mask528;
       u32 reg, periph1, periph2;
       
   #if defined(CONFIG_ANDROID_SUPPORT) /*安卓。。。*/
            /* Enable RTC */
   	writel(0x21, 0x020cc038);
   #endif
       
        /* 判断为imx6ull,啥也没做 */
       if (is_mx6sx() || is_mx6ul() || is_mx6ull() || is_mx6sll())
           return;
   }     
   ```

### 7. _main
   ```assembly
   " \arch\arm\lib\crt0.S "
   main:
        ldr	r0, =(CONFIG_SYS_INIT_SP_ADDR) @ ---[7-1]
        bic	r0, r0, 
        mov	sp, r0
        bl	board_init_f_alloc_reserve @ ---[7-2]
   
   	 mov	sp, r0   @ ---[7-3]
   	 mov	r9, r0   @ ---[7-4]
   	 bl	board_init_f_init_reserve @ ---[7-5] 
   	 
   	 mov	r0, #0  
   	 bl	board_init_f @ ---[7-6]
   	
   	
   	
   	
   /****************此时RAM的空间分布*******************/
   
      -------------- 0x00900000
      |            |  
      |            |
      |            |
      |            |    
      |            |    
      |            |    
      |            |    
      -------------- 0x0091FA00   <-- sp2  
      |global_data |    
      |            |
      -------------- 0x0091FB00
      |            |    
      |early malloc|
      |            |    
      -------------- 0x0091FF00   <-- sp1
      |   stack    |    
      |            |    
      -------------- 0x0091FFFF    
   ```

   - [7-1] sp = r0 = 0x0091FF00，并将形参r0传入了board_init_f_alloc_reserve()

#### [7-2] board_init_f_alloc_reserve

~~~C
 ```C
 /* \include\generated\autoconf.h */
 #define SYS_MALLOC_F_LEN  0x400
 
 /* \common\init\board_init.c */
 ulong board_init_f_alloc_reserve(ulong top)
 {
      
 #if CONFIG_VAL(SYS_MALLOC_F_LEN)
     /* 再分配一个区域用于u-boot的 malloc功能 */
 	top -= CONFIG_VAL(SYS_MALLOC_F_LEN);
 #endif
     
 	/*
 	 * 从[7]的RAM内存分布图看出, 
 	 * 往地址减小的方向移动 top-sizeof(struct global_data),
      * 并且进行16字节对齐
 	 */
 	top = rounddown(top-sizeof(struct global_data), 16);
 
     /* 返回 0x0091FA00 */
 	return top;
 }
~~~

   - [7-3]  将sp指针更新到sp2的位置。
   
   - [7-4]  在文件定义了gd指针指向了r9，在这里 struct global_data 区域的地址保存到r9，这样后续就可以直接通过gd这个指针访问 struct global_data 的内存区域了。
   
     ```C
     /* \arch\arm\include\asm\global_data.h */
     #define DECLARE_GLOBAL_DATA_PTR		register volatile gd_t *gd asm ("r9")

#### [7-5] board_init_f_init_reserve

     ```C
     /* \common\init\board_init.c */
     void board_init_f_init_reserve(ulong base)
     {
         /* 
          * 这里很神奇, 首先当前sp指向的是 gd结构体的区域，第一个成员是 bd_t *bd
          * 然后 gd_ptr 这个指针占用的就是 gd->bd 这个成员的位置。
          */
     	struct global_data *gd_ptr;
     
     	/* global_data结构体的区域清0  */
     	gd_ptr = (struct global_data *)base;
     	/* zero the area */
     	memset(gd_ptr, '\0', sizeof(*gd));
         
     	/* set GD unless architecture did it already */
     #if !defined(CONFIG_ARM)
     	arch_setup_gd(gd_ptr);
     #endif
         
     	/* 往地址增加的方向偏移，并16字节对齐，这时偏移到了earl malloc区域 */
     	base += roundup(sizeof(struct global_data), 16);
     
     #if CONFIG_VAL(SYS_MALLOC_F_LEN)
     	/*  使用gd结构的malloc_base成员,记录early malloc区域  */
     	gd->malloc_base = base;
     	/* 偏移到 stack 区域 */
     	base += CONFIG_VAL(SYS_MALLOC_F_LEN);
     #endif
     }

#### [7-6] struct global_data —– gd_t

```C
/* 先列举出 gd 结构体 */ 
 
 /* include/asm-generic/global_data.h */
 typedef struct global_data {
 	bd_t *bd;
 	unsigned long flags;
 	unsigned int baudrate;
 	unsigned long cpu_clk;		/* CPU clock in Hz!		*/
 	unsigned long bus_clk;
 	/* We cannot bracket this with CONFIG_PCI due to mpc5xxx */
 	unsigned long pci_clk;
 	unsigned long mem_clk;
 #if defined(CONFIG_LCD) || defined(CONFIG_VIDEO)
 	unsigned long fb_base;		/* Base address of framebuffer mem */
 #endif
 #if defined(CONFIG_POST)
 	unsigned long post_log_word;	/* Record POST activities */
 	unsigned long post_log_res;	/* success of POST test */
 	unsigned long post_init_f_time;	/* When post_init_f started */
 #endif
 #ifdef CONFIG_BOARD_TYPES
 	unsigned long board_type;
 #endif
 	unsigned long have_console;	/* serial_init() was called */
 #if CONFIG_IS_ENABLED(PRE_CONSOLE_BUFFER)
 	unsigned long precon_buf_idx;	/* Pre-Console buffer index */
 #endif
 	unsigned long env_addr;		/* Address  of Environment struct */
 	unsigned long env_valid;	/* Environment valid? enum env_valid */
 	unsigned long env_has_init;	/* Bitmask of boolean of struct env_location offsets */
 	int env_load_location;
 
 	unsigned long ram_top;		/* Top address of RAM used by U-Boot */
 	unsigned long relocaddr;	/* Start address of U-Boot in RAM */
 	phys_size_t ram_size;		/* RAM size */
 	unsigned long mon_len;		/* monitor len */
 	unsigned long irq_sp;		/* irq stack pointer */
 	unsigned long start_addr_sp;	/* start_addr_stackpointer */
 	unsigned long reloc_off;
 	struct global_data *new_gd;	/* relocated global data */
 
 #ifdef CONFIG_DM
 	struct udevice	*dm_root;	/* Root instance for Driver Model */
 	struct udevice	*dm_root_f;	/* Pre-relocation root instance */
 	struct list_head uclass_root;	/* Head of core tree */
 #endif
 #ifdef CONFIG_TIMER
 	struct udevice	*timer;		/* Timer instance for Driver Model */
 #endif
 
 	const void *fdt_blob;		/* Our device tree, NULL if none */
 	void *new_fdt;			/* Relocated FDT */
 	unsigned long fdt_size;		/* Space reserved for relocated FDT */
 #ifdef CONFIG_OF_LIVE
 	struct device_node *of_root;
 #endif
 	struct jt_funcs *jt;		/* jump table */
 	char env_buf[32];		/* buffer for env_get() before reloc. */
 #ifdef CONFIG_TRACE
 	void		*trace_buff;	/* The trace buffer */
 #endif
 #if defined(CONFIG_SYS_I2C)
 	int		cur_i2c_bus;	/* current used i2c bus */
 #endif
 #ifdef CONFIG_SYS_I2C_MXC
 	void *srdata[10];
 #endif
 	unsigned int timebase_h;
 	unsigned int timebase_l;
 #if CONFIG_VAL(SYS_MALLOC_F_LEN)
 	unsigned long malloc_base;	/* base address of early malloc() */
 	unsigned long malloc_limit;	/* limit address */
 	unsigned long malloc_ptr;	/* current address */
 #endif
 #ifdef CONFIG_PCI
 	struct pci_controller *hose;	/* PCI hose for early use */
 	phys_addr_t pci_ram_top;	/* top of region accessible to PCI */
 #endif
 #ifdef CONFIG_PCI_BOOTDELAY
 	int pcidelay_done;
 #endif
 	struct udevice *cur_serial_dev;	/* current serial device */
 	struct arch_global_data arch;	/* architecture-specific data */
 #ifdef CONFIG_CONSOLE_RECORD
 	struct membuff console_out;	/* console output */
 	struct membuff console_in;	/* console input */
 #endif
 #ifdef CONFIG_DM_VIDEO
 	ulong video_top;		/* Top of video frame buffer area */
 	ulong video_bottom;		/* Bottom of video frame buffer area */
 #endif
 #ifdef CONFIG_BOOTSTAGE
 	struct bootstage_data *bootstage;	/* Bootstage information */
 	struct bootstage_data *new_bootstage;	/* Relocated bootstage info */
 #endif
 #ifdef CONFIG_LOG
 	int log_drop_count;		/* Number of dropped log messages */
 	int default_log_level;		/* For devices with no filters */
 	struct list_head log_head;	/* List of struct log_device */
 	int log_fmt;			/* Mask containing log format info */
 #endif
 } gd_t;
```

##### [7-6-1]  board_init_f

```C
void board_init_f(ulong boot_flags)
{
     /* 这时候可以直接用gd了 */
 	gd->flags = boot_flags;//传入之前r0被清0了,所以 boot_flags 为 NULL
 	gd->have_console = 0; //默认无控制台console
 
    /* initcall,调用数组的一系列初始化函数 */
 	if (initcall_run_list(init_sequence_f))
 		hang();
 
 #if !defined(CONFIG_ARM) && !defined(CONFIG_SANDBOX) && \
 		!defined(CONFIG_EFI_APP) && !CONFIG_IS_ENABLED(X86_64)
 	/* NOTREACHED - jump_to_copy() does not return */
 	hang();
 #endif
}
```

   

