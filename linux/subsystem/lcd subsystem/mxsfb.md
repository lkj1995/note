### struct fb_info

```C
struct fb_info {
	atomic_t count;
	int node;
	int flags;
	struct mutex lock;		/* Lock for open/release/ioctl funcs */
	struct mutex mm_lock;		/* Lock for fb_mmap and smem_* fields */
	struct fb_var_screeninfo var;	/* Current var */
	struct fb_fix_screeninfo fix;	/* Current fix */
	struct fb_monspecs monspecs;	/* Current Monitor specs */
	struct work_struct queue;	/* Framebuffer event queue */
	struct fb_pixmap pixmap;	/* Image hardware mapper */
	struct fb_pixmap sprite;	/* Cursor hardware mapper */
	struct fb_cmap cmap;		/* Current cmap */
	struct list_head modelist;      /* mode list */
	struct fb_videomode *mode;	/* current mode */

#ifdef CONFIG_FB_BACKLIGHT
	/* assigned backlight device */
	/* set before framebuffer registration, 
	   remove after unregister */
	struct backlight_device *bl_dev;

	/* Backlight level curve */
	struct mutex bl_curve_mutex;	
	u8 bl_curve[FB_BACKLIGHT_LEVELS];
#endif
#ifdef CONFIG_FB_DEFERRED_IO
	struct delayed_work deferred_work;
	struct fb_deferred_io *fbdefio;
#endif

	struct fb_ops *fbops;
	struct device *device;		/* This is the parent */
	struct device *dev;		/* This is this fb device */
	int class_flag;                    /* private sysfs flags */
#ifdef CONFIG_FB_TILEBLITTING
	struct fb_tile_ops *tileops;    /* Tile Blitting */
#endif
	union {
		char __iomem *screen_base;	/* Virtual address */
		char *screen_buffer;
	};
	unsigned long screen_size;	/* Amount of ioremapped VRAM or 0 */ 
	void *pseudo_palette;		/* Fake palette of 16 colors */ 
#define FBINFO_STATE_RUNNING	0
#define FBINFO_STATE_SUSPENDED	1
	u32 state;			/* Hardware state i.e suspend */
	void *fbcon_par;                /* fbcon use-only private area */
	/* From here on everything is device dependent */
	void *par;
	/* we need the PCI or similar aperture base/size not
	   smem_start/size as smem_start may just be an object
	   allocated inside the aperture so may not actually overlap */
	struct apertures_struct {
		unsigned int count;
		struct aperture {
			resource_size_t base;
			resource_size_t size;
		} ranges[0];
	} *apertures;

	bool skip_vt_switch; /* no VT switch on suspend/resume required */
};
```

### struct fb_var_screeninfo

```C
struct fb_var_screeninfo {
	__u32 xres;			/* visible resolution		*/
	__u32 yres;
	__u32 xres_virtual;		/* virtual resolution		*/
	__u32 yres_virtual;
	__u32 xoffset;			/* offset from virtual to visible */
	__u32 yoffset;			/* resolution			*/

	__u32 bits_per_pixel;		/* guess what			*/
	__u32 grayscale;		/* 0 = color, 1 = grayscale,	*/
					/* >1 = FOURCC			*/
	struct fb_bitfield red;		/* bitfield in fb mem if true color, */
	struct fb_bitfield green;	/* else only length is significant */
	struct fb_bitfield blue;
	struct fb_bitfield transp;	/* transparency			*/	

	__u32 nonstd;			/* != 0 Non standard pixel format */

	__u32 activate;			/* see FB_ACTIVATE_*		*/

	__u32 height;			/* height of picture in mm    */
	__u32 width;			/* width of picture in mm     */

	__u32 accel_flags;		/* (OBSOLETE) see fb_info.flags */

	/* Timing: All values in pixclocks, except pixclock (of course) */
	__u32 pixclock;			/* pixel clock in ps (pico seconds) */
	__u32 left_margin;		/* time from sync to picture	*/
	__u32 right_margin;		/* time from picture to sync	*/
	__u32 upper_margin;		/* time from sync to picture	*/
	__u32 lower_margin;
	__u32 hsync_len;		/* length of horizontal sync	*/
	__u32 vsync_len;		/* length of vertical sync	*/
	__u32 sync;			/* see FB_SYNC_*		*/
	__u32 vmode;			/* see FB_VMODE_*		*/
	__u32 rotate;			/* angle we rotate counter clockwise */
	__u32 colorspace;		/* colorspace for FOURCC-based modes */
	__u32 reserved[4];		/* Reserved for future compatibility */
};
```

### 1 match

```C
static const struct of_device_id mxsfb_dt_ids[] = {
	{ .compatible = "fsl,imx23-lcdif", .data = &mxsfb_devtype[0], },
	{ .compatible = "fsl,imx28-lcdif", .data = &mxsfb_devtype[1], },
	{ .compatible = "fsl,imx6sx-lcdif", .data = &mxsfb_devtype[2], },
	{ /* sentinel */ }
};


static struct platform_driver mxsfb_driver = {
	.probe = mxsfb_probe,
	.remove = mxsfb_remove,
	.shutdown = mxsfb_shutdown,
	.id_table = mxsfb_devtype,
	.driver = {
		   .name = DRIVER_NAME,
		   .of_match_table = mxsfb_dt_ids,
		   .pm = &mxsfb_pm_ops,
	},
};

/*等同于调用了 xxx_init() 和 xxx_exit(),并注册了 platform_driver */
module_platform_driver(mxsfb_driver);
```

### 2 probe

```C
static int mxsfb_probe(struct platform_device *pdev)
{
	const struct of_device_id *of_id =
			of_match_device(mxsfb_dt_ids, &pdev->dev);
	struct resource *res;
	struct mxsfb_info *host;
	struct fb_info *fb_info;
	struct pinctrl *pinctrl;
    
    /* 在设备树上，获得第0偏移的hwirq,并创建virq映射 */
	int irq = platform_get_irq(pdev, 0);
	int gpio, ret;
	int rst_gpio;

    /* 保存私有数据 */
	if (of_id)
		pdev->id_entry = of_id->data;

    /* 获取"reg"属性内容：寄存器地址及数量大小 */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);

	/* 申请 mxsfb_info, 包含fb_info指针 */
	host = devm_kzalloc(&pdev->dev, 
                        sizeof(struct mxsfb_info), GFP_KERNEL);

	/* 申请 fb_info ,第一个参数还可以申请一段私有内存 */
	fb_info = framebuffer_alloc(0, &pdev->dev);

    /* 设置 mxsfb_info */
	host->fb_info = fb_info;
	fb_info->par = host;

    /* 根据中断号,"lcdif sync intrrupt"，注册中断回调函数 */
	ret = devm_request_irq(&pdev->dev, irq, mxsfb_irq_handler, 0,
			  dev_name(&pdev->dev), host);

	/* 寄存器的虚拟地址映射 */
	host->base = devm_ioremap_resource(&pdev->dev, res);

	host->pdev = pdev;
	platform_set_drvdata(pdev, host);
    
    /* 获取一些私有数据 */
	host->devdata = &mxsfb_devdata[pdev->id_entry->driver_data];

    /* 获取时钟 */
	host->clk_pix = devm_clk_get(&host->pdev->dev, "pix");
	host->clk_axi = devm_clk_get(&host->pdev->dev, "axi");
	host->clk_disp_axi = devm_clk_get(&host->pdev->dev, "disp_axi");


    /*
     * 1. 设置操作函数 fb_info->fops。
     * 2. 设置固定参数 fb_info->fix。
     * 3. 解析dt,设置可变参数 fb_info->var。
     * 4. 申请32M连续的framebuffer内存，清0。
     * 5. 检查lcd控制器是否开启，如开启，则读相关寄存器，新创建一个
     *    fb_videomode，并保存一套参数在新创建的 fb_videomode，
     *    最后将 fb_videomode 挂入 fb_info->modelist。   
     */
	ret = mxsfb_init_fbinfo(host);

	
	/* 获取pinctrl,将引脚切换为"default"状态 */
	if (!host->dispdrv) {
		pinctrl = devm_pinctrl_get_select_default(&pdev->dev);
	}

    /* 尚未使能 */
	if (!host->enabled) {
        
        /* LCDC_CTRL set，失能lcdif控制器 */
		writel(0, host->base + LCDC_CTRL);
        
        /* 根据fb_info信息，设置lcd的寄存器 */
		mxsfb_set_par(fb_info);
        
        /* 使能lcd控制器 */
		mxsfb_enable_controller(fb_info);
        
		pm_runtime_get_sync(&host->pdev->dev);
	}

    /* 注册 fb_info */
	ret = register_framebuffer(fb_info);

	/* 暂时不研究 */
	mxsfb_overlay_init(host);

   
        /* 100ask，复位了lcd */
        printk("100ask, %s %s %d\n", __FILE__, __FUNCTION__, __LINE__);
        rst_gpio = of_get_named_gpio(pdev->dev.of_node, "reset-gpios", 0);
        if (gpio_is_valid(rst_gpio)) {
                ret = gpio_request(rst_gpio, "lcdif_rst");
                if (ret < 0) {
                        dev_err(&pdev->dev,
                                "Failed to request GPIO:%d, ERRNO:%d\n",
                                (s32)rst_gpio, ret);
                } else {
                        gpio_direction_output(rst_gpio, 0);
                        msleep(2);
                        gpio_direction_output(rst_gpio, 1);
                        dev_info(&pdev->dev,  "Success reset LCDIF\n");
                }
        }


	return 0;

}

```

#### 2-1 framebuffer_alloc()

```C
struct fb_info *framebuffer_alloc(size_t size, struct device *dev)
{
#define BYTES_PER_LONG (BITS_PER_LONG/8)
#define PADDING (BYTES_PER_LONG - (sizeof(struct fb_info) % BYTES_PER_LONG))
    
    /* 计算 fb_info 大小 */
	int fb_info_size = sizeof(struct fb_info);
	struct fb_info *info;
	char *p;

	if (size)
		fb_info_size += PADDING;
	
    /* 申请 fb_info + 私有内存 */
	p = kzalloc(fb_info_size + size, GFP_KERNEL);

	if (!p)
		return NULL;

	info = (struct fb_info *) p;

    /* info->par 记录私有内存的位置 */
	if (size)
		info->par = p + fb_info_size;

	info->device = dev;

#ifdef CONFIG_FB_BACKLIGHT
	mutex_init(&info->bl_curve_mutex);
#endif

	return info;
#undef PADDING
#undef BYTES_PER_LONG
}
```

#### 2-2 mxsfb_init_fbinfo()

```C
/*
 * 1. 设置操作函数 fb_info->fops。
 * 2. 设置固定参数 fb_info->fix。
 * 3. 解析dt,设置可变参数 fb_info->var。
 * 4. 申请32M连续的framebuffer内存，清0。
 * 5. 检查lcd控制器是否开启，如开启，则读相关寄存器，新创建一个
 *    fb_videomode，并保存一套参数在新创建的 fb_videomode，
 *    最后将 fb_videomode 挂入 fb_info->modelist。   
 */
static int mxsfb_init_fbinfo(struct mxsfb_info *host)
{
	int ret;
	struct fb_info *fb_info = host->fb_info;
	struct fb_var_screeninfo *var = &fb_info->var;
	struct fb_modelist *modelist;

    /* 设置fops */
	fb_info->fbops = &mxsfb_ops;
    
	fb_info->flags = FBINFO_FLAG_DEFAULT | FBINFO_READS_FAST;
    
    /* 设置固定参数fix */
	fb_info->fix.type = FB_TYPE_PACKED_PIXELS;
	fb_info->fix.ypanstep = 1;
	fb_info->fix.ywrapstep = 1;
	fb_info->fix.visual = FB_VISUAL_TRUECOLOR,
	fb_info->fix.accel = FB_ACCEL_NONE;

    /* 解析dt,将多组配置值保存于fb_videomode，并挂入fb_info->modelist */
	ret = mxsfb_init_fbinfo_dt(host);

	/* id 命名 */
	if (host->id < 0)
		sprintf(fb_info->fix.id, "mxs-lcdif");
	else
		sprintf(fb_info->fix.id, "mxs-lcdif%d", host->id);

    /* 上述已解析dt,生成fb_videomode */
	if (!list_empty(&fb_info->modelist)) {
		/* first video mode in the modelist as default video mode  */
		modelist = list_first_entry(&fb_info->modelist,
				struct fb_modelist, list);
        /* fb_videomode 内容复制到 fb_info->var(可变参数) */
		fb_videomode_to_var(var, &modelist->mode);
	}
	/* save the sync value getting from dtb */
	host->sync = fb_info->var.sync;
	
    /* 设置var */
	var->nonstd = 0;
	var->activate = FB_ACTIVATE_NOW;
	var->accel_flags = 0;
	var->vmode = FB_VMODE_NONINTERLACED;

	/* 确认分辨率，RGB类型(rgb888)，并根据RGB种类，设置偏移值，长度，对齐 */
	mxsfb_check_var(var, fb_info);

    /*24bit = 3字节， 3 * 1024 = 总字节大小 */
	fb_info->fix.line_length =
		fb_info->var.xres * (fb_info->var.bits_per_pixel >> 3);
    /* 32M的 frambuffer 缓存区大小 */
	fb_info->fix.smem_len = SZ_32M;

	/* dma相关的内存申请函数,分配32M大小的连续内存，并清0 */
	if (mxsfb_map_videomem(fb_info) < 0)
		return -ENOMEM;
	
    /* 
     * 如果lcdif已经开启了，则读取寄存器内容，
     * 新创建一个videomode，将寄存器参数保存到新的videomode，
     * 并将其挂入fb_info->modelist。
     * 否则执行memset，将显存清0。
     */
	if (mxsfb_restore_mode(host))
		memset((char *)fb_info->screen_base, 0, fb_info->fix.smem_len);

	return 0;
}
```

#### 2-2-1 mxsfb_init_fbinfo_dt()

```C

static int mxsfb_init_fbinfo_dt(struct mxsfb_info *host)
{
	struct fb_info *fb_info = host->fb_info;
	struct fb_var_screeninfo *var = &fb_info->var;
	struct device *dev = &host->pdev->dev;
	struct device_node *np = host->pdev->dev.of_node;
	struct device_node *display_np;
	struct device_node *timings_np;
	struct display_timings *timings = NULL;
	const char *disp_dev, *disp_videomode;
	u32 width;
	int i;
	int ret = 0;
	
	host->id = of_alias_get_id(np, "lcdif");
	
    /* 根据"display"属性的phandle,获得display_np节点 */
	display_np = of_parse_phandle(np, "display", 0);

    /* 读取display_np节点下的属性 "bus-width" 总线宽度：24 */
	ret = of_property_read_u32(display_np, "bus-width", &width);

	/* 总线宽度记录 */
	switch (width) {
	case 8:
		host->ld_intf_width = STMLCDIF_8BIT;
		break;
	case 16:
		host->ld_intf_width = STMLCDIF_16BIT;
		break;
	case 18:
		host->ld_intf_width = STMLCDIF_18BIT;
		break;
	case 24:
		host->ld_intf_width = STMLCDIF_24BIT;
		break;
	default:
		dev_err(dev, "invalid bus-width value\n");
		ret = -EINVAL;
		goto put_display_node;
	}
	
    /* 读取"bits-per-pixel" RGB色深：888 */
	ret = of_property_read_u32(display_np, "bits-per-pixel",
				   &var->bits_per_pixel);

	/* 解析多组时序相关的屏幕参数值，保存在display_timings */
	timings = of_get_display_timings(display_np);

    /* 获取 "display-timings" 节点 */
	timings_np = of_find_node_by_name(display_np,
					  "display-timings");

    /* 1个子节点 */
	for (i = 0; i < of_get_child_count(timings_np); i++) {
		struct videomode vm;
		struct fb_videomode fb_vm;
        
		/* 将 display_timing 内容转存在 videomode */
		ret = videomode_from_timings(timings, &vm, i);
	
        
		ret = fb_videomode_from_videomode(&vm, &fb_vm);


		if (!(vm.flags & DISPLAY_FLAGS_DE_HIGH))
			fb_vm.sync |= FB_SYNC_OE_LOW_ACT;
		if (vm.flags & DISPLAY_FLAGS_PIXDATA_NEGEDGE)
			fb_vm.sync |= FB_SYNC_CLK_LAT_FALL;
        /* 
         * 先寻找是否已经有相同的fb_videomode, 
         * 没有的话则申请内存，并挂入fb_info->modelist中 
         */
		fb_add_videomode(&fb_vm, &fb_info->modelist);
	}

	return ret;
}

```

#### 2-2-1-1 of_get_display_timings()

```C
/*
	display_timings {
		num 记录有多少个display_timing
		native_mode 记录着当前使用的配置项
		display_timing[] 记录着多个timing,用于多种视频输出配置
	}		
*/
struct display_timings *of_get_display_timings(struct device_node *np)
{
	struct device_node *timings_np;
	struct device_node *entry;
	struct device_node *native_mode;
	struct display_timings *disp;

	if (!np)
		return NULL;
	
    /* 获取子节点"display-timings" */
	timings_np = of_get_child_by_name(np, "display-timings");

	/* 申请1个 display_timings */
	disp = kzalloc(sizeof(*disp), GFP_KERNEL);

	/* 获取子节点"native-mode" */
	entry = of_parse_phandle(timings_np, "native-mode", 0);

	native_mode = entry;
	
    /* 1个child, "timing0" */
	disp->num_timings = of_get_child_count(timings_np);

	/* 申请n个 display_timing 指针 */
	disp->timings = kzalloc(sizeof(struct display_timing *) *
				disp->num_timings, GFP_KERNEL);

	
	disp->num_timings = 0;
	disp->native_mode = 0;

    /* 解析dt，多组配置信息保存在display_timing[],当前只有一个 */
	for_each_child_of_node(timings_np, entry) {
		struct display_timing *dt;
		int r;
		
        /* 申请1个 display_timing */
		dt = kzalloc(sizeof(*dt), GFP_KERNEL);

		/* 解析dt，保存配置信息 */
		r = of_parse_display_timing(entry, dt);

		if (native_mode == entry)
			disp->native_mode = disp->num_timings;

        /* 保存display_timing 在 display_timings */
		disp->timings[disp->num_timings] = dt;
		disp->num_timings++;
	}
	return disp;

}
```

#### 2-2-1-1-1 of_parse_display_timing()

```C
static int of_parse_display_timing(const struct device_node *np,
		struct display_timing *dt)
{
	u32 val = 0;
	int ret = 0;

	memset(dt, 0, sizeof(*dt));
	
    /* 解析dt，保存配置内容 */
	ret |= parse_timing_property(np, "hback-porch", &dt->hback_porch);
	ret |= parse_timing_property(np, "hfront-porch", &dt->hfront_porch);
	ret |= parse_timing_property(np, "hactive", &dt->hactive);
	ret |= parse_timing_property(np, "hsync-len", &dt->hsync_len);
	ret |= parse_timing_property(np, "vback-porch", &dt->vback_porch);
	ret |= parse_timing_property(np, "vfront-porch", &dt->vfront_porch);
	ret |= parse_timing_property(np, "vactive", &dt->vactive);
	ret |= parse_timing_property(np, "vsync-len", &dt->vsync_len);
	ret |= parse_timing_property(np, "clock-frequency", &dt->pixelclock);

    
    /* 极性flag
     * pixel clk active
     * data enable active
     * hsync active
     * vsync active
     */    
	dt->flags = 0;
	if (!of_property_read_u32(np, "vsync-active", &val))
		dt->flags |= val ? DISPLAY_FLAGS_VSYNC_HIGH :
				DISPLAY_FLAGS_VSYNC_LOW;
	if (!of_property_read_u32(np, "hsync-active", &val))
		dt->flags |= val ? DISPLAY_FLAGS_HSYNC_HIGH :
				DISPLAY_FLAGS_HSYNC_LOW;
	if (!of_property_read_u32(np, "de-active", &val))
		dt->flags |= val ? DISPLAY_FLAGS_DE_HIGH :
				DISPLAY_FLAGS_DE_LOW;
	if (!of_property_read_u32(np, "pixelclk-active", &val))
		dt->flags |= val ? DISPLAY_FLAGS_PIXDATA_POSEDGE :
				DISPLAY_FLAGS_PIXDATA_NEGEDGE;

	if (of_property_read_bool(np, "interlaced"))
		dt->flags |= DISPLAY_FLAGS_INTERLACED;
	if (of_property_read_bool(np, "doublescan"))
		dt->flags |= DISPLAY_FLAGS_DOUBLESCAN;
	if (of_property_read_bool(np, "doubleclk"))
		dt->flags |= DISPLAY_FLAGS_DOUBLECLK;

	if (ret) {
		pr_err("%s: error reading timing properties\n",
			of_node_full_name(np));
		return -EINVAL;
	}

	return 0;
}
```

#### 2-2-1-2 videomode_from_timings()

```C
int videomode_from_timings(const struct display_timings *disp,
			  struct videomode *vm, unsigned int index)
{
	struct display_timing *dt;

    /* 根据index，获得一个配置列表 display_timing */
	dt = display_timings_get(disp, index);

	/* 将 display_timing 内容转存在 videomode */
	videomode_from_timing(dt, vm);

	return 0;
}


void videomode_from_timing(const struct display_timing *dt,
			  struct videomode *vm)
{
	vm->pixelclock = dt->pixelclock.typ;
	vm->hactive = dt->hactive.typ;
	vm->hfront_porch = dt->hfront_porch.typ;
	vm->hback_porch = dt->hback_porch.typ;
	vm->hsync_len = dt->hsync_len.typ;

	vm->vactive = dt->vactive.typ;
	vm->vfront_porch = dt->vfront_porch.typ;
	vm->vback_porch = dt->vback_porch.typ;
	vm->vsync_len = dt->vsync_len.typ;

    /* 极性flag
     * pixel clk active
     * data enable active
     * hsync active
     * vsync active
     */
	vm->flags = dt->flags;
}
```

#### 2-2-1-3 fb_videomode_from_videomode()

```C
int fb_videomode_from_videomode(const struct videomode *vm,
				struct fb_videomode *fbmode)
{
	unsigned int htotal, vtotal;

    /*设置 fb_videomode */
	fbmode->xres = vm->hactive;
	fbmode->left_margin = vm->hback_porch;
	fbmode->right_margin = vm->hfront_porch;
	fbmode->hsync_len = vm->hsync_len;

	fbmode->yres = vm->vactive;
	fbmode->upper_margin = vm->vback_porch;
	fbmode->lower_margin = vm->vfront_porch;
	fbmode->vsync_len = vm->vsync_len;

	/* prevent division by zero in KHZ2PICOS macro */
	fbmode->pixclock = vm->pixelclock ?
			KHZ2PICOS(vm->pixelclock / 1000) : 0;

	fbmode->sync = 0;
	fbmode->vmode = 0;
	if (vm->flags & DISPLAY_FLAGS_HSYNC_HIGH)
		fbmode->sync |= FB_SYNC_HOR_HIGH_ACT;
	if (vm->flags & DISPLAY_FLAGS_VSYNC_HIGH)
		fbmode->sync |= FB_SYNC_VERT_HIGH_ACT;
	if (vm->flags & DISPLAY_FLAGS_INTERLACED)
		fbmode->vmode |= FB_VMODE_INTERLACED;
	if (vm->flags & DISPLAY_FLAGS_DOUBLESCAN)
		fbmode->vmode |= FB_VMODE_DOUBLE;
	fbmode->flag = 0;

	htotal = vm->hactive + vm->hfront_porch + vm->hback_porch +
		 vm->hsync_len;
	vtotal = vm->vactive + vm->vfront_porch + vm->vback_porch +
		 vm->vsync_len;
	/* prevent division by zero */
	if (htotal && vtotal) {
        /* 刷新时间 */
		fbmode->refresh = vm->pixelclock / (htotal * vtotal);
	/* a mode must have htotal and vtotal != 0 or it is invalid */
	} else {
		fbmode->refresh = 0;
		return -EINVAL;
	}

	return 0;
}
```

#### 2-2-1-4 fb_add_videomode()

```C
/* 先寻找是否已经有相同的fb_videomode,没有的话则申请内存，并挂入fb_info中 */
int fb_add_videomode(const struct fb_videomode *mode, struct list_head *head)
{
	struct list_head *pos;
	struct fb_modelist *modelist;
	struct fb_videomode *m;
	int found = 0;

	list_for_each(pos, head) {
		modelist = list_entry(pos, struct fb_modelist, list);
		m = &modelist->mode;
		if (fb_mode_is_equal(m, mode)) {
			found = 1;
			break;
		}
	}
	if (!found) {
        /*申请 fb_modelist */
		modelist = kmalloc(sizeof(struct fb_modelist),
						  GFP_KERNEL);
		/* 这里为什么有这种操作，是复制吗 */
		modelist->mode = *mode;
        
        /* 将 fb_videomode 挂入 fb_info->modelist 中 */
		list_add(&modelist->list, head);
	}
	return 0;
}
```

#### 2-2-2 mxsfb_check_var()

```C
/* 确认分辨率，RGB类型，并配置不同种类RGB的偏移值，长度，对齐 */
static int mxsfb_check_var(struct fb_var_screeninfo *var,
		struct fb_info *fb_info)
{
	struct mxsfb_info *host = fb_info->par;
	const struct fb_bitfield *rgb = NULL;

    /* 分辨率检查 */
	if (var->xres < MIN_XRES)
		var->xres = MIN_XRES;
	if (var->yres < MIN_YRES)
		var->yres = MIN_YRES;

	if (var->xres_virtual > var->xres) {
		dev_dbg(fb_info->device, "stride not supported\n");
		return -EINVAL;
	}

	if (var->xres_virtual < var->xres)
		var->xres_virtual = var->xres;
	if (var->yres_virtual < var->yres)
		var->yres_virtual = var->yres;

    /* 24bit的强制为32bit */
	if ((var->bits_per_pixel != 32) && (var->bits_per_pixel != 16))
		var->bits_per_pixel = 32;

    /* 记录rgb类型 */
	switch (var->bits_per_pixel) {
	case 16:
		/* always expect RGB 565 */
		rgb = def_rgb565;
		break;
	case 32:
		switch (host->ld_intf_width) {
		case STMLCDIF_8BIT:
			pr_debug("Unsupported LCD bus width mapping\n");
			return -EINVAL;
		case STMLCDIF_16BIT:
			/* 24 bit to 18 bit mapping */
			rgb = def_rgb666;
			break;
		case STMLCDIF_18BIT:
			if (pixfmt_is_equal(var, def_rgb666))
				/* 24 bit to 18 bit mapping */
				rgb = def_rgb666;
			else
				rgb = def_rgb888;
			break;
		case STMLCDIF_24BIT:
			/* real 24 bit */
			rgb = def_rgb888;
			break;
		default:
			/*
			 * 32-bit output is possible through I/O muxing, if this
			 * option is available on chip. Currently not
			 * implemented.
			 */
			pr_debug("Currently unsupported output colour depth: %u\n",
				 host->ld_intf_width);
			return -EINVAL;
		}
		break;
	default:
		pr_debug("Unsupported colour depth: %u\n", var->bits_per_pixel);
		return -EINVAL;
	}

	/*
	 * Copy the RGB parameters for this display
	 * from the machine specific parameters.
	 */
    /* 
     * 确认了如 rgb888/565 等类型, 设置一个提前写好的fb_bitfield，
     * 里面已经定义好了每种颜色在32bit/16bit的偏移值，长度，左/右对齐
     */
	var->red    = rgb[RED];
	var->green  = rgb[GREEN];
	var->blue   = rgb[BLUE];
	var->transp = rgb[TRANSP];

	return 0;
}

```

#### 2-2-3 mxsfb_map_videomem()

```C
static int mxsfb_map_videomem(struct fb_info *fbi)
{
	if (fbi->fix.smem_len < fbi->var.yres_virtual * fbi->fix.line_length)
		fbi->fix.smem_len = fbi->var.yres_virtual *
				    fbi->fix.line_length;

    /* 分配一段连续的内存 */
	fbi->screen_base = dma_alloc_writecombine(fbi->device,
				fbi->fix.smem_len,
				(dma_addr_t *)&fbi->fix.smem_start,
				GFP_DMA | GFP_KERNEL);
    
	/* 32M */
	fbi->screen_size = fbi->fix.smem_len;

	/* 内存清0 */
	memset((char *)fbi->screen_base, 0, fbi->fix.smem_len);

	return 0;
}

```

#### 2-2-4 mxsfb_restore_mode()

```C

static int mxsfb_restore_mode(struct mxsfb_info *host)
{
	struct fb_info *fb_info = host->fb_info;
	unsigned line_count;
	unsigned period;
	unsigned long pa, fbsize;
	int bits_per_pixel, ofs;
	u32 transfer_count, vdctrl0, vdctrl2, vdctrl3, vdctrl4, ctrl;
	struct fb_videomode vmode;

    /* 使能时钟 */
	clk_enable_axi(host);
	clk_enable_disp_axi(host);


	/* Only restore the mode when the controller is running */
    /* 读 LCDC_CTRL 寄存器 */
	ctrl = readl(host->base + LCDC_CTRL);
    /* 确认 lcdif控制器已经启动 */
	if (!(ctrl & CTRL_RUN))
		return -EINVAL;

	memset(&vmode, 0, sizeof(vmode));
	
    /* 
     * 读LCDC_VDCTRL0~4寄存器，如果lcd的控制器已经启动，
     * 说明里面的参数已经配置好了，需要保存这套配置，因此全
     * 部读出来，将参数保存到新创建的fb_videomode，并将
     * videomode挂入fb_info->modelist。
     */
	vdctrl0 = readl(host->base + LCDC_VDCTRL0);
	vdctrl2 = readl(host->base + LCDC_VDCTRL2);
	vdctrl3 = readl(host->base + LCDC_VDCTRL3);
	vdctrl4 = readl(host->base + LCDC_VDCTRL4);

	transfer_count = readl(host->base + host->devdata->transfer_count);

	vmode.xres = TRANSFER_COUNT_GET_HCOUNT(transfer_count);
	vmode.yres = TRANSFER_COUNT_GET_VCOUNT(transfer_count);

	switch (CTRL_GET_WORD_LENGTH(ctrl)) {
	case 0:
		bits_per_pixel = 16;
		break;
	case 3:
		bits_per_pixel = 32;
		break;
	case 1:
	default:
		return -EINVAL;
	}

	fb_info->var.bits_per_pixel = bits_per_pixel;

	vmode.pixclock = KHZ2PICOS(clk_get_rate(host->clk_pix) / 1000U);
	vmode.hsync_len = get_hsync_pulse_width(host, vdctrl2);
	vmode.left_margin = GET_HOR_WAIT_CNT(vdctrl3) - vmode.hsync_len;
	vmode.right_margin = VDCTRL2_GET_HSYNC_PERIOD(vdctrl2) - vmode.hsync_len -
		vmode.left_margin - vmode.xres;
	vmode.vsync_len = VDCTRL0_GET_VSYNC_PULSE_WIDTH(vdctrl0);
	period = readl(host->base + LCDC_VDCTRL1);
	vmode.upper_margin = GET_VERT_WAIT_CNT(vdctrl3) - vmode.vsync_len;
	vmode.lower_margin = period - vmode.vsync_len - vmode.upper_margin - vmode.yres;

	vmode.vmode = FB_VMODE_NONINTERLACED;

	vmode.sync = 0;
	if (vdctrl0 & VDCTRL0_HSYNC_ACT_HIGH)
		vmode.sync |= FB_SYNC_HOR_HIGH_ACT;
	if (vdctrl0 & VDCTRL0_VSYNC_ACT_HIGH)
		vmode.sync |= FB_SYNC_VERT_HIGH_ACT;

	
	fb_add_videomode(&vmode, &fb_info->modelist);

	host->ld_intf_width = CTRL_GET_BUS_WIDTH(ctrl);
	host->dotclk_delay = VDCTRL4_GET_DOTCLK_DLY(vdctrl4);

	fb_info->fix.line_length = vmode.xres * (bits_per_pixel >> 3);

	pa = readl(host->base + host->devdata->cur_buf);
	fbsize = fb_info->fix.line_length * vmode.yres;
	if (pa < fb_info->fix.smem_start)
		return -EINVAL;
	if (pa + fbsize > fb_info->fix.smem_start + fb_info->fix.smem_len)
		return -EINVAL;
	ofs = pa - fb_info->fix.smem_start;
	if (ofs) {
		memmove(fb_info->screen_base, fb_info->screen_base + ofs, fbsize);
		writel(fb_info->fix.smem_start, host->base + host->devdata->next_buf);
	}

	line_count = fb_info->fix.smem_len / fb_info->fix.line_length;
	fb_info->fix.ypanstep = 1;
	fb_info->fix.ywrapstep = 1;

	host->enabled = 1;

	return 0;
}

```

#### 2-3 mxsfb_dispdrv_init()

```C
/* 看起来就是进行了命名操作，不重要的处理。。 */
static int mxsfb_dispdrv_init(struct platform_device *pdev,
			      struct fb_info *fbi)
{
	struct mxsfb_info *host = fbi->par;
	struct mxc_dispdrv_setting setting;
	struct device *dev = &pdev->dev;
	char disp_dev[32];

	if (!strlen(host->disp_dev))
		return 0;

	memset(&setting, 0x0, sizeof(setting));
	setting.fbi = fbi;
	memcpy(disp_dev, host->disp_dev, strlen(host->disp_dev));
	disp_dev[strlen(host->disp_dev)] = '\0';

	/* Use videomode name from dtb, if any given */
	if (host->disp_videomode) {
		setting.dft_mode_str = kmalloc(NAME_LEN, GFP_KERNEL);
		if (setting.dft_mode_str) {
			memset(setting.dft_mode_str, 0x0, NAME_LEN);
			memcpy(setting.dft_mode_str, host->disp_videomode,
			       strlen(host->disp_videomode));
		}
	}

	host->dispdrv = mxc_dispdrv_gethandle(disp_dev, &setting);

	kfree(setting.dft_mode_str);

	if (IS_ERR(host->dispdrv))
		return -EPROBE_DEFER;
	else
		dev_info(dev, "registered mxc display driver %s\n",
			 disp_dev);

	return 0;
}

```

#### 2-4 mxsfb_set_par()

```c
static int mxsfb_set_par(struct fb_info *fb_info)
{
	struct mxsfb_info *host = fb_info->par;
	u32 ctrl, vdctrl0, vdctrl4;
	int line_size, fb_size;
	int reenable = 0;
	static u32 equal_bypass = 0;


	if (likely(equal_bypass > 1)) {
		/* If parameter no change, don't reconfigure. */
		if (mxsfb_par_equal(fb_info, host))
			return 0;
	} else
		equal_bypass++;


	/* 计算行字节数 */
	line_size =  fb_info->var.xres * (fb_info->var.bits_per_pixel >> 3);
	fb_info->fix.line_length = line_size;
    
    /* 屏幕总字节数 */
	fb_size = fb_info->var.yres_virtual * line_size;

    /* 屏幕总字节数 > 缓存区大小,错误 */
	if (fb_size > fb_info->fix.smem_len) {
		dev_err(&host->pdev->dev, "exceeds the fb buffer size limit!\n");
		return -ENOMEM;
	}

    /* 如果lcd控制器已经开启了,则需关闭它，并清空fifo，再进行配置 */
	if (host->enabled) {
		reenable = 1;
		mxsfb_disable_controller(fb_info);
	}

	/* [LCDC_CTRL1] 
	 * bit21[0x1]:清空fifo
	 * 
	 * 写入[LCDC_CTRL1]
	 */
	writel(CTRL1_FIFO_CLEAR, host->base + LCDC_CTRL1 + REG_SET);

    /* [LCDC_CTRL]  
     * bit19   [0x1]:使用DOTCLK模式,必须置1
     * bit5    [0x1]:作为bus master,必须置1  
     * bit11~10[0x3]:总线宽度，设置为24bit
     * bit9~8  [0x3]:rgb长度，使用rgb888/rgb666
     * bit1    [clear]:使用rgb888
     */
	ctrl = CTRL_BYPASS_COUNT | CTRL_MASTER |
		CTRL_SET_BUS_WIDTH(host->ld_intf_width);

	switch (fb_info->var.bits_per_pixel) {
	case 16:
		dev_dbg(&host->pdev->dev, "Setting up RGB565 mode\n");
		ctrl |= CTRL_SET_WORD_LENGTH(0);
		writel(CTRL1_SET_BYTE_PACKAGING(0xf), host->base + LCDC_CTRL1);
		break;
	case 32:
		dev_dbg(&host->pdev->dev, "Setting up RGB888/666 mode\n");
		ctrl |= CTRL_SET_WORD_LENGTH(3);
		switch (host->ld_intf_width) {
		case STMLCDIF_8BIT:
			dev_dbg(&host->pdev->dev,
					"Unsupported LCD bus width mapping\n");
			return -EINVAL;
		case STMLCDIF_16BIT:
			/* 24 bit to 18 bit mapping */
			ctrl |= CTRL_DF24; /* ignore the upper 2 bits in
					    *  each colour component
					    */
			break;
		case STMLCDIF_18BIT:
			if (pixfmt_is_equal(&fb_info->var, def_rgb666))
				/* 24 bit to 18 bit mapping */
				ctrl |= CTRL_DF24; /* ignore the upper 2 bits in
						    *  each colour component
						    */
			break;
		case STMLCDIF_24BIT:
			/* real 24 bit */
			break;
		}
		/* do not use packed pixels = one pixel per word instead */
		writel(CTRL1_SET_BYTE_PACKAGING(0x7), host->base + LCDC_CTRL1);
		break;
	default:
		dev_dbg(&host->pdev->dev, "Unhandled color depth of %u\n",
				fb_info->var.bits_per_pixel);
		return -EINVAL;
	}

    /* 写入[LCDC_CTRL] */
	writel(ctrl, host->base + LCDC_CTRL);

    /* 
     * [LCDC_TRANSFER_COUNT]
     * bit31~16[600]:  列像素点数量,设置为600
     * bit15~0 [1024]: 行像素点数量，设置为1024
     *
     * 写入[LCDC_TRANSFER_COUNT]
     */
	writel(TRANSFER_COUNT_SET_VCOUNT(fb_info->var.yres) |
			TRANSFER_COUNT_SET_HCOUNT(fb_info->var.xres),
			host->base + host->devdata->transfer_count);
	
    /*
     * [LCDC_VDCTRL0]
     * bit28[1]:使用DOTCLK模式，需开启使能引脚
     * bit27[0]:VSYNC有效电平,设置为下降沿有效
     * bit26[0]:HSYNC有效电平,设置为下降沿有效
     * bit25[0]:DOTCLK有效电平,设置为下降沿有效
     * bit24[1]:ENANLE有效电平,设置为上升沿有效
     */
	vdctrl0 = VDCTRL0_ENABLE_PRESENT |	/* always in DOTCLOCK mode */
		VDCTRL0_VSYNC_PERIOD_UNIT |
		VDCTRL0_VSYNC_PULSE_WIDTH_UNIT |
		VDCTRL0_SET_VSYNC_PULSE_WIDTH(fb_info->var.vsync_len);
	/* use the saved sync to avoid wrong sync information */
    
	if (host->sync & FB_SYNC_HOR_HIGH_ACT)
		vdctrl0 |= VDCTRL0_HSYNC_ACT_HIGH;
	if (host->sync & FB_SYNC_VERT_HIGH_ACT)
		vdctrl0 |= VDCTRL0_VSYNC_ACT_HIGH;
#ifndef CONFIG_FB_IMX64_DEBUG
	if (!(host->sync & FB_SYNC_OE_LOW_ACT))
		vdctrl0 |= VDCTRL0_ENABLE_ACT_HIGH;
#endif
	if (host->sync & FB_SYNC_CLK_LAT_FALL)
		vdctrl0 |= VDCTRL0_DOTCLK_ACT_FALLING;

    /* 写入[LCDC_VDCTRL0] */
	writel(vdctrl0, host->base + LCDC_VDCTRL0);

	/* 
	 * [LCDC_VDCTRL1]
	 * bit31~0:设置VSYNC垂直的总周期：
	 *             上空白值+下空白值+有效值+垂直同步信号脉冲宽度 
	 *
	 * 写入[LCDC_VDCTRL1]
	 */
	writel(fb_info->var.upper_margin + fb_info->var.vsync_len +
		fb_info->var.lower_margin + fb_info->var.yres,
		host->base + LCDC_VDCTRL1);

	/* 
	 * [LCDC_VDCTRL2]
	 * bit31~18:设置水平同步信号脉冲宽度
	 * bit17~0: 设置HSYNC水平的总周期：
	 *                左空白值+右空白值+有效值+水平同步信号脉冲宽度 
     *
     * 写入[LCDC_VDCTRL2]
     */
	writel(set_hsync_pulse_width(host, fb_info->var.hsync_len) |
		VDCTRL2_SET_HSYNC_PERIOD(fb_info->var.left_margin +
		fb_info->var.hsync_len + fb_info->var.right_margin +
		fb_info->var.xres),
		host->base + LCDC_VDCTRL2);

 	/*
	 * [LCDC_VDCTRL3]
	 * bit27~16：水平方向上的等待时钟数,即左右空白值 =thb + thp
     * bit15~0:  垂直方向上的等待时钟数,上下空白值 = tvb + tvp
     *
     * 写入[LCDC_VDCTRL3]
     */    
	writel(SET_HOR_WAIT_CNT(fb_info->var.left_margin +
		fb_info->var.hsync_len) |
		SET_VERT_WAIT_CNT(fb_info->var.upper_margin +
			fb_info->var.vsync_len),
		host->base + LCDC_VDCTRL3);

    /*
     * [LCDC_VDCTRL4]
     * bit18[1]:使用VSHYNC、HSYNC、DOTCLK模式
     * bit15~0:DOTCLK总时间
     *
     * 写入[LCDC_VDCTRL4]
     */
	vdctrl4 = SET_DOTCLK_H_VALID_DATA_CNT(fb_info->var.xres);
	if (mxsfb_is_v4(host))
		vdctrl4 |= VDCTRL4_SET_DOTCLK_DLY(host->dotclk_delay);
	writel(vdctrl4, host->base + LCDC_VDCTRL4);

    /* 下一帧显存的地址 */
	writel(fb_info->fix.smem_start +
			fb_info->fix.line_length * fb_info->var.yoffset,
			host->base + host->devdata->next_buf);

    /* 重新使能lcd控制器 */
	if (reenable)
		mxsfb_enable_controller(fb_info);

	/* Clear activate as not Reconfiguring framebuffer again */
	if ((fb_info->var.activate & FB_ACTIVATE_FORCE) &&
		(fb_info->var.activate & FB_ACTIVATE_MASK) == FB_ACTIVATE_NOW)
		fb_info->var.activate = FB_ACTIVATE_NOW;

	host->var = fb_info->var;
	return 0;
}

```

#### 2-5 mxsfb_enable_controller()

```c
static void mxsfb_enable_controller(struct fb_info *fb_info)
{
	struct mxsfb_info *host = fb_info->par;
	u32 reg;
	int ret;

   
	if (host->dispdrv && host->dispdrv->drv->setup) {
		ret = host->dispdrv->drv->setup(host->dispdrv, fb_info);
		if (ret < 0) {
			dev_err(&host->pdev->dev, "failed to setup"
				"dispdrv:%s\n", host->dispdrv->drv->name);
			return;
		}
		host->sync = fb_info->var.sync;
	}

    
	if (host->reg_lcd) {
		ret = regulator_enable(host->reg_lcd);
	}

	if (host->dispdrv && host->dispdrv->drv->enable) {
		ret = host->dispdrv->drv->enable(host->dispdrv, fb_info);
		if (ret < 0)
			dev_err(&host->pdev->dev, "failed to enable "
				"dispdrv:%s\n", host->dispdrv->drv->name);
	}


        /* 设置"pix"时钟频率 */
		ret = clk_set_rate(host->clk_pix,
				PICOS2KHZ(fb_info->var.pixclock) * 1000U);

        /* 使能"pix"时钟 */
		clk_enable_pix(host);

    /**/
	writel(CTRL2_OUTSTANDING_REQS__REQ_16,
		host->base + LCDC_V4_CTRL2 + REG_SET);

	/* if it was disabled, re-enable the mode again */
	writel(CTRL_DOTCLK_MODE, host->base + LCDC_CTRL + REG_SET);

	/* enable the SYNC signals first, then the DMA engine */
	reg = readl(host->base + LCDC_VDCTRL4);
	reg |= VDCTRL4_SYNC_SIGNALS_ON;
	writel(reg, host->base + LCDC_VDCTRL4);

	writel(CTRL_MASTER, host->base + LCDC_CTRL + REG_SET);
	writel(CTRL_RUN, host->base + LCDC_CTRL + REG_SET);

	/* Recovery on underflow */
	writel(CTRL1_RECOVERY_ON_UNDERFLOW, host->base + LCDC_CTRL1 + REG_SET);

	host->enabled = 1;

}
```

#### 2-6 register_framebuffer()

```C
int register_framebuffer(struct fb_info *fb_info)
{
	int ret;

	mutex_lock(&registration_lock);
	ret = do_register_framebuffer(fb_info);
	mutex_unlock(&registration_lock);

	return ret;
}


static int do_register_framebuffer(struct fb_info *fb_info)
{
	int i, ret;
	struct fb_event event;
	struct fb_videomode mode;


	/* fb 使用数量+1 */
	num_registered_fb++;
    
    /* 寻找空闲的registered_fb指针index */
	for (i = 0 ; i < FB_MAX; i++)
		if (!registered_fb[i])
			break;
    
	fb_info->node = i;
	atomic_set(&fb_info->count, 1);
	mutex_init(&fb_info->lock);
	mutex_init(&fb_info->mm_lock);
    
	/* 创建fbx device */
	fb_info->dev = device_create(fb_class, fb_info->device,
				     MKDEV(FB_MAJOR, i), NULL, "fb%d", i);
    
	/* 创建属性文件 */
	fb_init_device(fb_info);

	if (fb_info->pixmap.addr == NULL) {
		fb_info->pixmap.addr = kmalloc(FBPIXMAPSIZE, GFP_KERNEL);
		if (fb_info->pixmap.addr) {
			fb_info->pixmap.size = FBPIXMAPSIZE;
			fb_info->pixmap.buf_align = 1;
			fb_info->pixmap.scan_align = 1;
			fb_info->pixmap.access_align = 32;
			fb_info->pixmap.flags = FB_PIXMAP_DEFAULT;
		}
	}	
	fb_info->pixmap.offset = 0;

	if (!fb_info->pixmap.blit_x)
		fb_info->pixmap.blit_x = ~(u32)0;

	if (!fb_info->pixmap.blit_y)
		fb_info->pixmap.blit_y = ~(u32)0;

	if (!fb_info->modelist.prev || !fb_info->modelist.next)
		INIT_LIST_HEAD(&fb_info->modelist);

	if (fb_info->skip_vt_switch)
		pm_vt_switch_required(fb_info->dev, false);
	else
		pm_vt_switch_required(fb_info->dev, true);

    /* 根据fb_info->var 生成 fb_videomode,并挂入fb_info->modelist */
	fb_var_to_videomode(&mode, &fb_info->var);
	fb_add_videomode(&mode, &fb_info->modelist);
	registered_fb[i] = fb_info;

	event.info = fb_info;
	if (!lockless_register_fb)
		console_lock();
	if (!lock_fb_info(fb_info)) {
		if (!lockless_register_fb)
			console_unlock();
		return -ENODEV;
	}

	fb_notifier_call_chain(FB_EVENT_FB_REGISTERED, &event);
	unlock_fb_info(fb_info);
	if (!lockless_register_fb)
		console_unlock();
	return 0;
}
```

#### 2-7 mxsfb_overlay_init()

```C

static void mxsfb_overlay_init(struct mxsfb_info *fbi)
{
	int ret;
	struct mxsfb_layer *ofb = &fbi->overlay;
	struct fb_videomode ofb_vm;

	ofb->dev = &fbi->pdev->dev;
    
    /* 再申请一个fb_info */
	ofb->ol_fb = framebuffer_alloc(0, ofb->dev);

	
	init_mxsfb_overlay(fbi, ofb);

	/* add videomode to overlay fb */
	fb_var_to_videomode(&ofb_vm, &fbi->fb_info->var);
	ret = fb_add_videomode(&ofb_vm, &ofb->ol_fb->modelist);
	if (ret) {
		dev_err(ofb->dev, "add vm to ofb failed\n");
		goto fb_release;
	}

	ret = register_framebuffer(ofb->ol_fb);
	if (ret) {
		dev_err(ofb->dev, "failed to register overlay\n");
		goto fb_release;
	}

	ret = mxsfb_overlay_map_video_memory(fbi, ofb);
	if (ret) {
		dev_err(ofb->dev, "failed to map video mem for overlay\n");
		goto fb_unregister;
	}

	/* setup the initial params for overlay fb */
	overlayfb_check_var(&ofb->ol_fb->var, ofb->ol_fb);
	overlayfb_set_par(ofb->ol_fb);

	ofb->registered = 1;

	return;

fb_unregister:
	unregister_framebuffer(ofb->ol_fb);
fb_release:
	framebuffer_release(ofb->ol_fb);
}

```

#### 2-7-1 init_mxsfb_overlay()

```C
static void init_mxsfb_overlay(struct mxsfb_info *fbi,
			       struct mxsfb_layer *ofb)
{
	dev_dbg(&fbi->pdev->dev, "AS overlay init\n");

	ofb->ol_fb->fix.type		= FB_TYPE_PACKED_PIXELS;
	ofb->ol_fb->fix.xpanstep	= 0;
	ofb->ol_fb->fix.ypanstep	= 1;
	ofb->ol_fb->fix.ywrapstep 	= 1;
	ofb->ol_fb->fix.visual		= FB_VISUAL_TRUECOLOR;
	ofb->ol_fb->fix.accel		= FB_ACCEL_NONE;

	ofb->ol_fb->var.activate	= FB_ACTIVATE_NXTOPEN;
	ofb->ol_fb->var.xres		= fbi->fb_info->var.xres;
	ofb->ol_fb->var.yres		= fbi->fb_info->var.yres;
	ofb->ol_fb->var.xres_virtual	= fbi->fb_info->var.xres_virtual;
	ofb->ol_fb->var.yres_virtual	= fbi->fb_info->var.yres;
	ofb->ol_fb->var.bits_per_pixel	= fbi->fb_info->var.bits_per_pixel;
	ofb->ol_fb->var.vmode		= FB_VMODE_NONINTERLACED;
	ofb->ol_fb->var.nonstd		= 0;

	/* Copy timings of primary fb */
	ofb->ol_fb->var.pixclock	    = fbi->fb_info->var.pixclock;
	ofb->ol_fb->var.left_margin	    = fbi->fb_info->var.left_margin;
	ofb->ol_fb->var.right_margin	= fbi->fb_info->var.right_margin;
	ofb->ol_fb->var.upper_margin	= fbi->fb_info->var.upper_margin;
	ofb->ol_fb->var.lower_margin	= fbi->fb_info->var.lower_margin;
	ofb->ol_fb->var.hsync_len	    = fbi->fb_info->var.hsync_len;
	ofb->ol_fb->var.vsync_len	    = fbi->fb_info->var.vsync_len;

	ofb->ol_fb->fbops = &overlay_fb_ops;
	ofb->ol_fb->node  = -1;
	ofb->ol_fb->par	  = ofb;
	INIT_LIST_HEAD(&ofb->ol_fb->modelist);

	ofb->id = 0;
	ofb->ops = &ofb_ops;
	atomic_set(&ofb->usage, 0);
	ofb->blank_state = -1;
	ofb->global_alpha = 255;
	ofb->fbi = fbi;

	sprintf(ofb->ol_fb->fix.id, "FG");
}
```

### 3 用户层使用

```C
static const struct file_operations fb_fops = {
	.owner =	THIS_MODULE,
	.read =		fb_read,
	.write =	fb_write,
	.unlocked_ioctl = fb_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = fb_compat_ioctl,
#endif
	.mmap =		fb_mmap,
	.open =		fb_open,
	.release =	fb_release,
#ifdef HAVE_ARCH_FB_UNMAPPED_AREA
	.get_unmapped_area = get_fb_unmapped_area,
#endif
#ifdef CONFIG_FB_DEFERRED_IO
	.fsync =	fb_deferred_io_fsync,
#endif
	.llseek =	default_llseek,
};


static struct fb_ops mxsfb_ops = {
	.owner = THIS_MODULE,
	.fb_check_var = mxsfb_check_var,
	.fb_set_par = mxsfb_set_par,
	.fb_setcolreg = mxsfb_setcolreg,
	.fb_ioctl = mxsfb_ioctl,
	.fb_blank = mxsfb_blank,
	.fb_pan_display = mxsfb_pan_display,
	.fb_mmap = mxsfb_mmap,
	.fb_fillrect = cfb_fillrect,
	.fb_copyarea = cfb_copyarea,
	.fb_imageblit = cfb_imageblit,
};
```

#### 3-1 open 

```C
/* 初始化没有open函数,啥也没做 */
static int fb_open(struct inode *inode, struct file *file)
__acquires(&info->lock)
__releases(&info->lock)
{
    /* 获取次设备号,即fb_info对应的index */
	int fbidx = iminor(inode);
	struct fb_info *info;
	int res = 0;

    /* 获取fb_info */
	info = get_fb_info(fbidx);

	/* 保存在私有指针,后续使用 */
	file->private_data = info;
    
	if (info->fbops->fb_open) {
		res = info->fbops->fb_open(info,1);
	}   
	return res;
}
```

3-2 



### 总结

- mxsfb.c调用了fbmem.c中提供的framebuffer子系统函数。

- 注册了platform_driver，对lcdif节点进行match。

- probe
  - 调用中断子系统函数，解析”interrupt”属性，获取hwirq。
  
  - 解析”reg”属性，获取lcdif寄存器组地址及数量。
  
  - 申请fb_info。
  
  - 使能中断，并注册中断回调函数(默认进行线程化)。
  
  - 解析dt有关时钟的属性（“pix”,”axi”），获取时钟频率，使能pix时钟，aix好像已经提前被使能了。
  
  - 设置fb_info
  
    - 设置固定参数，设置fops
  
    - 申请32M显存，清0。
  
    - 申请timings和timing，解析dt，并将多组timing(LCD的配置值)挂入timimgs。
  
    - tinimgs -> videomode ->fb_videomode -> var，经过转换后，完成对fb_info的设置。
  
  - 调用pinctrl子系统，配置引脚。
  
  - 根据fb_info，配置lcdif寄存器(时序，分辨率，显存地址，rgb类型等)，使能lcd控制器。
  
  - 注册fb_info到framebuffer子系统。
  

- 用户层简单使用

- ```c
  static struct fb_var_screeninfo var;
  unsigned char *fb_addr_base;
  unsigned int pixel_width, line_width, screen_size;
  
  
  void framebuffer_init(void)
  {
      int fd;
      
  	fd = open("/dev/fb0", O_RDWR);
  	ioctl(fd, FBIOGET_VSCREENINFO, &var);
  
  	pixel_width = var.bits_per_pixel / 8;
  	line_width  = var.xres * pixel_width;
  	screen_size = var.yres * line_width;
  
  	fb_addr_base = mmap(NULL, screen_size, PORT_READ|PORT_WRITE, MAP_SHARED, fd, 0);
  }
  
  
  void framebuffer_draw_point(int x, int y, unsigned int color)
  {
      unsigned int *p = 
          (unsigned int *)(fb_addr_base + x * pixel_width + y * line_width);
      if(p)
      	*p = color;
  }

