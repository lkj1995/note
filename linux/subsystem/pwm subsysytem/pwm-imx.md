### struct imx_chip

```C
struct imx_chip {
	struct clk	*clk_per;
	struct clk	*clk_ipg;

	void __iomem	*mmio_base;

	struct pwm_chip	chip;

	int (*config)(struct pwm_chip *chip,
		struct pwm_device *pwm, int duty_ns, int period_ns);
	void (*set_enable)(struct pwm_chip *chip, bool enable);
};
```

### struct pwm_chip

```C
struct pwm_chip {
	struct device *dev;
	struct list_head list;
	const struct pwm_ops *ops;
	int base;
	unsigned int npwm;

	struct pwm_device *pwms;

	struct pwm_device * (*of_xlate)(struct pwm_chip *pc,
					const struct of_phandle_args *args);
	unsigned int of_pwm_n_cells;
	bool can_sleep;
};
```

### struct pwm_device

```C
struct pwm_device {
	const char *label;
	unsigned long flags;
	unsigned int hwpwm;
	unsigned int pwm;
	struct pwm_chip *chip;
	void *chip_data;

	struct pwm_args args;
	struct pwm_state state;
};
```

### 1 dt

```C
pwm1: pwm@02080000 {
    compatible = "fsl,imx6ul-pwm", "fsl,imx27-pwm";
    reg = <0x02080000 0x4000>;
    interrupts = <GIC_SPI 83 IRQ_TYPE_LEVEL_HIGH>;
    clocks = <&clks IMX6UL_CLK_PWM1>,
    	<&clks IMX6UL_CLK_PWM1>;
    clock-names = "ipg", "per";
    #pwm-cells = <2>;
};


&pwm1 {
    pinctrl-names = "default";
    pinctrl-0 = <&pinctrl_pwm1>;
    status = "okay";
};
```

### 2 init

```C
static const struct of_device_id imx_pwm_dt_ids[] = {
	{ .compatible = "fsl,imx1-pwm", .data = &imx_pwm_data_v1, },
    
    /* 这个compatible与dt匹配 */
	{ .compatible = "fsl,imx27-pwm", .data = &imx_pwm_data_v2, },
	{ /* sentinel */ }
};

static struct platform_driver imx_pwm_driver = {
	.driver		= {
		.name	= "imx-pwm",
		.of_match_table = imx_pwm_dt_ids,
	},
	.probe		= imx_pwm_probe,
	.remove		= imx_pwm_remove,
};

module_platform_driver(imx_pwm_driver);
```

### 3 probe

```C
static int imx_pwm_probe(struct platform_device *pdev)
{
	const struct of_device_id *of_id =
			of_match_device(imx_pwm_dt_ids, &pdev->dev);
	const struct imx_pwm_data *data;
	struct imx_chip *imx;
	struct resource *r;
	int ret = 0;


	/* 申请 imx_chip */
	imx = devm_kzalloc(&pdev->dev, sizeof(*imx), GFP_KERNEL);

	/* 获取时钟频率 */
	imx->clk_per = devm_clk_get(&pdev->dev, "per");
	imx->clk_ipg = devm_clk_get(&pdev->dev, "ipg");

	/* 设置 pwm_chip */
	imx->chip.ops = &imx_pwm_ops;
	imx->chip.dev = &pdev->dev;
	imx->chip.base = -1;
	imx->chip.npwm = 1;
	imx->chip.can_sleep = true;
	
    /* 获取pwm寄存器物理基地址，转换成虚拟地址 */
	r = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	imx->mmio_base = devm_ioremap_resource(&pdev->dev, r);

	/* soc私有函数 	
	 * imx_pwm_config_v2()
	 * imx_pwm_set_enable_v2()
	 */
	data = of_id->data;
	imx->config = data->config;
	imx->set_enable = data->set_enable;

    
	ret = pwmchip_add(&imx->chip);

	/* 保存私有数据 */
	platform_set_drvdata(pdev, imx);
	return 0;
}

```

#### 3-1 pwmchip_add()

```C
int pwmchip_add(struct pwm_chip *chip)
{
	return pwmchip_add_with_polarity(chip, PWM_POLARITY_NORMAL);
}



int pwmchip_add_with_polarity(struct pwm_chip *chip,
			      enum pwm_polarity polarity)
{
	struct pwm_device *pwm;
	unsigned int i;
	int ret;

	/* 
	 * base = -1,npwm = 1
     * bitmap申请一片region
	 */
	ret = alloc_pwms(chip->base, chip->npwm);

	/* 申请 npwm 个 pwm_device */
	chip->pwms = kcalloc(chip->npwm, sizeof(*pwm), GFP_KERNEL);

	/* bitmap位置 */
	chip->base = ret;

    /* 设置 pwm_device */
	for (i = 0; i < chip->npwm; i++) {
		pwm = &chip->pwms[i];

		pwm->chip = chip;
		pwm->pwm = chip->base + i;
		pwm->hwpwm = i;
		pwm->state.polarity = polarity;

		if (chip->ops->get_state)
			chip->ops->get_state(chip, pwm, &pwm->state);

        /* pwm device 插入 radix tree */
		radix_tree_insert(&pwm_tree, pwm->pwm, pwm);
	}

    /* 标记为已占用 */
	bitmap_set(allocated_pwms, chip->base, chip->npwm);

    /* pwm chip 插入 list */
	INIT_LIST_HEAD(&chip->list);
	list_add(&chip->list, &pwm_chips);

	ret = 0;

    /* 		
     * chip->of_xlate = of_pwm_simple_xlate;
	 * chip->of_pwm_n_cells = 2;
	 */
	if (IS_ENABLED(CONFIG_OF))
		of_pwmchip_add(chip);

    /* /sys/ 下创建调试信息 */
	pwmchip_sysfs_export(chip);

out:
	mutex_unlock(&pwm_lock);
	return ret;
}

```

#### 3-1-1 alloc_pwms()

```C
#define MAX_PWMS 1024

/* bitmap相关,申请1024个bitmap,即 1024bit / 1byte = 128个字节 */
static DECLARE_BITMAP(allocated_pwms, MAX_PWMS);


static int alloc_pwms(int pwm, unsigned int count)
{
	unsigned int from = 0;
	unsigned int start;

	if (pwm >= MAX_PWMS)
		return -EINVAL;

	if (pwm >= 0)
		from = pwm;
	
    /*  申请一片空闲bitmap，长度为count */
	start = bitmap_find_next_zero_area(allocated_pwms, MAX_PWMS, from,
					   count, 0);

	if (pwm >= 0 && start != pwm)
		return -EEXIST;

	if (start + count > MAX_PWMS)
		return -ENOSPC;

	return start;
}
```

### 4 pwm_request()

```C
struct pwm_device *pwm_request(int pwm, const char *label)
{
	struct pwm_device *dev;
	int err;

	
    /* 根据pwm号,在 radix tree 寻找 pwm_device */
	dev = pwm_to_device(pwm);

    /* 没提供request,无作用 */
	err = pwm_device_request(dev, label);

	return dev;
}
```

### 5 pwm_config()

```C
static inline int pwm_config(struct pwm_device *pwm, int duty_ns,
			     int period_ns)
{
	struct pwm_state state;

	pwm_get_state(pwm, &state);
	state.duty_cycle = duty_ns;
	state.period = period_ns;
    
       
    /* 	 
     * 调用了 imx_pwm_config
     * pwm->chip->ops->config(pwm->chip, pwm,
	 *				     state->duty_cycle,
	 *					     state->period);
	 */
	return pwm_apply_state(pwm, &state);
}



static int imx_pwm_config(struct pwm_chip *chip,
		struct pwm_device *pwm, int duty_ns, int period_ns)
{
	struct imx_chip *imx = to_imx_chip(chip);
	int ret;

	ret = clk_prepare_enable(imx->clk_per);
	ret = clk_prepare_enable(imx->clk_ipg);


	ret = imx->config(chip, pwm, duty_ns, period_ns);

	clk_disable_unprepare(imx->clk_ipg);
	clk_disable_unprepare(imx->clk_per);

	return ret;
}


/* 实际寄存器操作 */
static int imx_pwm_config_v2(struct pwm_chip *chip,
		struct pwm_device *pwm, int duty_ns, int period_ns)
{
	struct imx_chip *imx = to_imx_chip(chip);
	struct device *dev = chip->dev;
	unsigned long long c;
	unsigned long period_cycles, duty_cycles, prescale;
	unsigned int period_ms;
	bool enable = pwm_is_enabled(pwm);
	int wait_count = 0, fifoav;
	u32 cr, sr;

	/*
	 * i.MX PWMv2 has a 4-word sample FIFO.
	 * In order to avoid FIFO overflow issue, we do software reset
	 * to clear all sample FIFO if the controller is disabled or
	 * wait for a full PWM cycle to get a relinquished FIFO slot
	 * when the controller is enabled and the FIFO is fully loaded.
	 */
	if (enable) {
		sr = readl(imx->mmio_base + MX3_PWMSR);
		fifoav = sr & MX3_PWMSR_FIFOAV_MASK;
		if (fifoav == MX3_PWMSR_FIFOAV_4WORDS) {
			period_ms = DIV_ROUND_UP(pwm_get_period(pwm),
						 NSEC_PER_MSEC);
			msleep(period_ms);

			sr = readl(imx->mmio_base + MX3_PWMSR);
			if (fifoav == (sr & MX3_PWMSR_FIFOAV_MASK))
				dev_warn(dev, "there is no free FIFO slot\n");
		}
	} else {
		writel(MX3_PWMCR_SWR, imx->mmio_base + MX3_PWMCR);
		do {
			usleep_range(200, 1000);
			cr = readl(imx->mmio_base + MX3_PWMCR);
		} while ((cr & MX3_PWMCR_SWR) &&
			 (wait_count++ < MX3_PWM_SWR_LOOP));

		if (cr & MX3_PWMCR_SWR)
			dev_warn(dev, "software reset timeout\n");
	}

	c = clk_get_rate(imx->clk_per);
	c = c * period_ns;
	do_div(c, 1000000000);
	period_cycles = c;

	prescale = period_cycles / 0x10000 + 1;

	period_cycles /= prescale;
	c = (unsigned long long)period_cycles * duty_ns;
	do_div(c, period_ns);
	duty_cycles = c;

	/*
	 * according to imx pwm RM, the real period value should be
	 * PERIOD value in PWMPR plus 2.
	 */
	if (period_cycles > 2)
		period_cycles -= 2;
	else
		period_cycles = 0;

	writel(duty_cycles, imx->mmio_base + MX3_PWMSAR);
	writel(period_cycles, imx->mmio_base + MX3_PWMPR);

	cr = MX3_PWMCR_PRESCALER(prescale) |
		MX3_PWMCR_DOZEEN | MX3_PWMCR_WAITEN |
		MX3_PWMCR_DBGEN | MX3_PWMCR_CLKSRC_IPG_HIGH;

	if (enable)
		cr |= MX3_PWMCR_EN;

	writel(cr, imx->mmio_base + MX3_PWMCR);

	return 0;
}
```

### 6 pwm_enable()

```C
static inline int pwm_enable(struct pwm_device *pwm)
{
	struct pwm_state state;


	pwm_get_state(pwm, &state);
	state.enabled = true;
    
    /*
     * 调用了 imx_pwm_enable()
     * err = pwm->chip->ops->enable(pwm->chip, pwm);
     */
	return pwm_apply_state(pwm, &state);
}


static int imx_pwm_enable(struct pwm_chip *chip, struct pwm_device *pwm)
{
	struct imx_chip *imx = to_imx_chip(chip);
	int ret;

	ret = clk_prepare_enable(imx->clk_per);
	ret = clk_prepare_enable(imx->clk_ipg);

	imx->set_enable(chip, true);

	return 0;
}


static void imx_pwm_set_enable_v2(struct pwm_chip *chip, bool enable)
{
	struct imx_chip *imx = to_imx_chip(chip);
	u32 val;

	val = readl(imx->mmio_base + MX3_PWMCR);

	if (enable)
		val |= MX3_PWMCR_EN;
	else
		val &= ~MX3_PWMCR_EN;

	writel(val, imx->mmio_base + MX3_PWMCR);
}

```

### 7 pwm_set_polarity()

```C
static inline int pwm_set_polarity(struct pwm_device *pwm,
				   enum pwm_polarity polarity)
{
	struct pwm_state state;

	if (!pwm)
		return -EINVAL;

	pwm_get_state(pwm, &state);
	if (state.polarity == polarity)
		return 0;

	/*
	 * Changing the polarity of a running PWM without adjusting the
	 * dutycycle/period value is a bit risky (can introduce glitches).
	 * Return -EBUSY in this case.
	 * Note that this is allowed when using pwm_apply_state() because
	 * the user specifies all the parameters.
	 */
	if (state.enabled)
		return -EBUSY;

	state.polarity = polarity;
	return pwm_apply_state(pwm, &state);
}
```

