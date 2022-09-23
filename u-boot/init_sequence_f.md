### init_fnc_t init_sequence_f[]

```C
/* 条件编译的一系列函数,全部顺序执行 */
static const init_fnc_t init_sequence_f[] = {
	setup_mon_len,
	fdtdec_setup,
	initf_malloc,
	log_init, /*不分析*/
	initf_bootstage, /*不分析*/
	initf_console_record, /*不分析*/
	arch_fsp_init, /*不分析*/
	arch_cpu_init, /* basic arch cpu dependent setup */
	mach_cpu_init,/* SoC/machine dependent CPU setup */
	initf_dm, /* 设备模型初始化 */
	arch_cpu_init_dm,
	board_early_init_f, /* 板子早期的配置,imx6ull初始化串口io */
	get_clocks,		/* get CPU and bus clocks (etc.) */
	timer_init,		/* initialize timer */
	board_postclk_init,
	env_init,		/* initialize environment */
	init_baud_rate,		/* initialze baudrate settings */
	serial_init,		/* serial communications setup */
	console_init_f,		/* stage 1 init of console */
	display_options,	/* say that we are here */
	display_text_info,	/* show debugging info if required */
	checkcpu,
	print_cpuinfo,		/* display cpu info (and speed) */
	embedded_dtb_select,
	show_board_info,
	INIT_FUNC_WATCHDOG_INIT
	misc_init_f,
	INIT_FUNC_WATCHDOG_RESET
	init_func_i2c,
	init_func_vid,
	init_func_spi,
	announce_dram_init,
	dram_init,		/* configure available RAM banks */
	post_init_f,
	INIT_FUNC_WATCHDOG_RESET
	testdram,
	INIT_FUNC_WATCHDOG_RESET
	init_post,
	INIT_FUNC_WATCHDOG_RESET
	/*
	 * Now that we have DRAM mapped and working, we can
	 * relocate the code and continue running from DRAM.
	 *
	 * Reserve memory at end of RAM for (top down in that order):
	 *  - area that won't get touched by U-Boot and Linux (optional)
	 *  - kernel log buffer
	 *  - protected RAM
	 *  - LCD framebuffer
	 *  - monitor code
	 *  - board info struct
	 */
	setup_dest_addr,
#ifdef CONFIG_PRAM
	reserve_pram,
#endif
	reserve_round_4k,
#ifdef CONFIG_ARM
	reserve_mmu,
#endif
	reserve_video,
	reserve_trace,
	reserve_uboot,
	reserve_malloc,
	reserve_board,
	setup_machine,
	reserve_global_data,
	reserve_fdt,
	reserve_bootstage,
	reserve_arch,
	reserve_stacks,
	dram_init_banksize,
	show_dram_config,
#if defined(CONFIG_M68K) || defined(CONFIG_MIPS) || defined(CONFIG_PPC) || \
	defined(CONFIG_SH)
	setup_board_part1,
#endif
#if defined(CONFIG_PPC) || defined(CONFIG_M68K)
	INIT_FUNC_WATCHDOG_RESET
	setup_board_part2,
#endif
	display_new_sp,
#ifdef CONFIG_OF_BOARD_FIXUP
	fix_fdt,
#endif
	INIT_FUNC_WATCHDOG_RESET
	reloc_fdt,
	reloc_bootstage,
	setup_reloc,
#if defined(CONFIG_X86) || defined(CONFIG_ARC)
	copy_uboot_to_ram,
	do_elf_reloc_fixups,
	clear_bss,
#endif
#if defined(CONFIG_XTENSA)
	clear_bss,
#endif
#if !defined(CONFIG_ARM) && !defined(CONFIG_SANDBOX) && \
		!CONFIG_IS_ENABLED(X86_64)
	jump_to_copy,
#endif
	NULL,
};
```

### 1 setup_mon_len()

```C
/* 记录 u-boot code 的总长度 = __bss_end - _start */
static int setup_mon_len(void)
{
#if defined(__ARM__) || defined(__MICROBLAZE__)
	gd->mon_len = (ulong)&__bss_end - (ulong)_start;
#endif    
	return 0;
}
```

### 2 fdtdec_setup()

```
1.dtb集成到uboot的bin文件内部
1-1.如何使能 
  需要打开CONFIG_OF_EMBED宏来使能。
1-2.编译说明 
  在这种方式下，在编译uboot的过程中，也会编译dtb。
1-3. 最终位置 
  注意：最终dtb是包含到了uboot的bin文件内部的。 
  dtb会位于uboot的.dtb.init.rodata段中，并且在代码中可以通过__dtb_dt_begin符号获取其符号。 
  因为这种方式不够灵活，文档上也不推荐，所以后续也不具体研究，简单了解一下即可。
```

### 2 initf_malloc()

```C
int initf_malloc(void)
{
#if CONFIG_VAL(SYS_MALLOC_F_LEN)
	assert(gd->malloc_base);	/* Set up by crt0.S */
    
    /* 设置malloc的限制值 0x400 */
	gd->malloc_limit = CONFIG_VAL(SYS_MALLOC_F_LEN);
	gd->malloc_ptr = 0;
#endif

	return 0;
}
```

