### dt节点

```C
gpio-keys {
    compatible = "gpio-keys";
    pinctrl-names = "default";

    user1 {
        label = "User1 Button";
        gpios = <&gpio5 1 GPIO_ACTIVE_LOW>;
        gpio-key,wakeup;
        linux,code = <KEY_1>;
    };
        
    user2 {
        label = "User2 Button";
        gpios = <&gpio4 14 GPIO_ACTIVE_LOW>;
        gpio-key,wakeup;
        linux,code = <KEY_2>;
    };
};
```

### struct gpio_keys_platform_data

```c
struct gpio_keys_platform_data {
	struct gpio_keys_button *buttons;
	int nbuttons;
	unsigned int poll_interval;
	unsigned int rep:1;
	int (*enable)(struct device *dev);
	void (*disable)(struct device *dev);
	const char *name;
};
```

### struct gpio_keys_button

```C
struct gpio_keys_button {
	unsigned int code;
	int gpio;
	int active_low;
	const char *desc;
	unsigned int type;
	int wakeup;
	int debounce_interval;
	bool can_disable;
	int value;
	unsigned int irq;
	struct gpio_desc *gpiod;
};
```

### 1 gpio_keys_probe()

```C
static int gpio_keys_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	const struct gpio_keys_platform_data *pdata = 
        							dev_get_platdata(dev);
	struct gpio_keys_drvdata *ddata;
	struct input_dev *input;
	size_t size;
	int i, error;
	int wakeup = 0;

    /*首次执行probe，pdata为NULL*/
	if (!pdata) {
        
        /* 将节点和子节点全部解析后，保存在新申请的pdata */
		pdata = gpio_keys_get_devtree_pdata(dev);

	}
	
    /* 类的继承，再封装一层 */
	size = sizeof(struct gpio_keys_drvdata) +
			pdata->nbuttons * sizeof(struct gpio_button_data);    
    
    /* 申请 ddata */
	ddata = devm_kzalloc(dev, size, GFP_KERNEL);

	/* 申请 input_dev */
	input = devm_input_allocate_device(dev);

	/* 设置 ddata */
	ddata->pdata = pdata;
	ddata->input = input;
	mutex_init(&ddata->disable_lock);

    /*
     * ddata 保存在 input_dev->device 
     * 和 platform_dev->device
     */
	platform_set_drvdata(pdev, ddata);
	input_set_drvdata(input, ddata);

    /* 设置input_dev */
	input->name = pdata->name ? : pdev->name;
	input->phys = "gpio-keys/input0";
	input->dev.parent = &pdev->dev;
	input->open = gpio_keys_open;
	input->close = gpio_keys_close;

	input->id.bustype = BUS_HOST;
	input->id.vendor = 0x0001;
	input->id.product = 0x0001;
	input->id.version = 0x0100;

	/* 重复触发功能，标记到input_dev */
	if (pdata->rep)
		__set_bit(EV_REP, input->evbit);

    
	for (i = 0; i < pdata->nbuttons; i++) {
		const struct gpio_keys_button *button = &pdata->buttons[i];
		struct gpio_button_data *bdata = &ddata->data[i];

        /* 
         * 1. 根据选用户的gpio号，使用gpio子系统，配置gpiox_iox 
         * 2. 并开启该gpiox_iox的中断
         */
		error = gpio_keys_setup_key(pdev, input, bdata, button);

		if (button->wakeup)
			wakeup = 1;
	}

    /* 注册attr */
	error = sysfs_create_group(&pdev->dev.kobj,
                               &gpio_keys_attr_group);

    /* 注册input_dev */
	error = input_register_device(input);

    
	device_init_wakeup(&pdev->dev, wakeup);

	return 0;
}
```

### 1-1 gpio_keys_get_devtree_pdata()

```C
/*
 * Translate OpenFirmware node properties into platform_data
 */
static struct gpio_keys_platform_data *
gpio_keys_get_devtree_pdata(struct device *dev)
{
	struct device_node *node, *pp;
	struct gpio_keys_platform_data *pdata;
	struct gpio_keys_button *button;
	int error;
	int nbuttons;
	int i;

	node = dev->of_node;

    /*获取有效的子节点数量*/
	nbuttons = of_get_available_child_count(node);


    /*
     * 多个button，需要多个gpio_keys_button来描述 
     * 整个gpio-keys，只需要1个gpio_keys_platform_data
     */
	pdata = devm_kzalloc(dev,
			     sizeof(*pdata) + nbuttons * sizeof(*button),
			     GFP_KERNEL);

    /*偏移到gpio_keys_butto[],这是技巧 */
	pdata->buttons = (struct gpio_keys_button *)(pdata + 1);
    
    /*记录button数量*/
	pdata->nbuttons = nbuttons;

    /* 是否需要重复触发的功能 */
	pdata->rep = !!of_get_property(node, "autorepeat", NULL);

    /* 获取"label"属性,命名pdata */
	of_property_read_string(node, "label", &pdata->name);

	i = 0;
	for_each_available_child_of_node(node, pp) {
		enum of_gpio_flags flags;
		
        /*取出第i个*/
		button = &pdata->buttons[i++];

        /* 
         * 分支1：
         * 1. 这里调用了gpio子系统api，gpio-controller节点
         *    中已经生成了所有引脚对应的gpio_desc。因此调用api，
         *    通过当前节点的"gpios"属性,很容易找到flags和gpio号。
         * 2. 从传入的参数index=0可知，在dt中child节点只会
         *    解析第1个gpio，如果有多个，后面的会被忽略。
         * 3. 该gpio号为 32 * gpiox + iox
         *    如：gpio1_io5,32*1+5=37，返回的gpio引脚号为37
         *    好处就是方便寻找到gpio-controller对应的gpio_dev。
         */
		button->gpio = of_get_gpio_flags(pp, 0, &flags);
       
        /* 确认dt中的flag描述的有效电平 */
		button->active_low = flags & OF_GPIO_ACTIVE_LOW;
		        
	    /* 分支2：如果是使用irq子系统的属性描述 */
		button->irq = irq_of_parse_and_map(pp, 0);

        /* ps：这两种方法都会使用中断的功能 */
        
        /* 读取属性"linux,code",确认按键码，如 "KEY_A" */
		if (of_property_read_u32(pp, "linux,code", &button->code)) {
			dev_err(dev, "Button without keycode: 0x%x\n",
				button->gpio);
			return ERR_PTR(-EINVAL);
		}
        
		/*获取"label"属性，命名每个button */
		button->desc = of_get_property(pp, "label", NULL);

        /* button类型,默认为EV_KEY */
		if (of_property_read_u32(pp, "linux,input-type", &button->type))
			button->type = EV_KEY;

        /* 该button是否能唤醒系统 */
		button->wakeup = of_property_read_bool(pp, "wakeup-source") ||
				 /* legacy name */
				 of_property_read_bool(pp, "gpio-key,wakeup");

		button->can_disable = !!of_get_property(pp, "linux,can-disable", NULL);

        /* 读取消抖时间属性,默认设为5 */
		if (of_property_read_u32(pp, "debounce-interval",
					 &button->debounce_interval))
			button->debounce_interval = 5;
	}

	return pdata;
}
```

### 1-2 gpio_keys_setup_key()

```C
static int gpio_keys_setup_key(struct platform_device *pdev,
				struct input_dev *input,
				struct gpio_button_data *bdata,
				const struct gpio_keys_button *button)
{
	const char *desc = button->desc ? button->desc : "gpio_keys";
	struct device *dev = &pdev->dev;
	irq_handler_t isr;
	unsigned long irqflags;
	int irq;
	int error;

    /* 设置bdata */
	bdata->input = input;
	bdata->button = button;
	spin_lock_init(&bdata->lock);

	/* 因为是通过gpio号来寻找gpio_desc,需先确认是否在有效范围 */
	if (gpio_is_valid(button->gpio)) {
		unsigned flags = GPIOF_IN;

		if (button->active_low)
			flags |= GPIOF_ACTIVE_LOW;

        /* 调用gpiod_request(),用于"gpio-ranges",不分析 */
		error = devm_gpio_request_one(&pdev->dev, button->gpio, 
                                      flags, desc);

        /* 通过排序的gpio号，在gpio_devices上找对应的gpio_desc */
		bdata->gpiod = gpio_to_desc(button->gpio);

		/* 消抖设置 */
		if (button->debounce_interval) {
            /* 需要gpio_chip有这个消抖处理函数 */
			error = gpiod_set_debounce(bdata->gpiod,
					button->debounce_interval * 1000);

            /*否则使用软件消抖*/
			if (error < 0)
				bdata->software_debounce =
						button->debounce_interval;
		}

        /* 如果使用的是gpio子系统，会调用gpiod_to_irq()申请irq */
		if (button->irq) {
			bdata->irq = button->irq;
		} else {
			irq = gpiod_to_irq(bdata->gpiod);
			}
			bdata->irq = irq;
		}

    	/* 初始化一个workqueue */
		INIT_DELAYED_WORK(&bdata->work, gpio_keys_gpio_work_func);

		isr = gpio_keys_gpio_isr;
		irqflags = IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING;

	} else {
		if (!button->irq) {
			dev_err(dev, "No IRQ specified\n");
			return -EINVAL;
		}
		bdata->irq = button->irq;

		if (button->type && button->type != EV_KEY) {
			dev_err(dev, "Only EV_KEY allowed for IRQ buttons.\n");
			return -EINVAL;
		}

		bdata->release_delay = button->debounce_interval;
    	/*初始化和添加一个定时器,用于中断回调函数*/
		setup_timer(&bdata->release_timer,
			    gpio_keys_irq_timer, (unsigned long)bdata);

		isr = gpio_keys_irq_isr;
		irqflags = 0;
	}
	
    /* 保存type和code */
	input_set_capability(input, button->type ?: EV_KEY, button->code);

	/*
	 * If platform has specified that the button can be disabled,
	 * we don't want it to share the interrupt line.
	 */
	if (!button->can_disable)
		irqflags |= IRQF_SHARED;

    /* 注册中断回调函数isr，最终调用request_thread_irq() */
	error = devm_request_any_context_irq(&pdev->dev, bdata->irq,
					     isr, irqflags, desc, bdata);

	return 0;
}

```

### 1-2-1 gpiod_to_irq()

```C

/**
 * gpiod_to_irq() - return the IRQ corresponding to a GPIO
 * @desc: gpio whose IRQ will be returned (already requested)
 *
 * Return the IRQ corresponding to the passed GPIO, or an error code in case of
 * error.
 */
int gpiod_to_irq(const struct gpio_desc *desc)
{
	struct gpio_chip *chip;
	int offset;

	/*
	 * Cannot VALIDATE_DESC() here as gpiod_to_irq() consumer semantics
	 * requires this function to not return zero on an invalid descriptor
	 * but rather a negative error number.
	 */
	if (!desc || IS_ERR(desc) || !desc->gdev || !desc->gdev->chip)
		return -EINVAL;

	chip = desc->gdev->chip;
	offset = gpio_chip_hwgpio(desc);
    
	if (chip->to_irq) {
        /*调用 mxc_gpio_to_irq(),通过hwirq找到virq */
		int retirq = chip->to_irq(chip, offset);
		return retirq;
	}
	return -ENXIO;
}

```

```c
static int mxc_gpio_to_irq(struct gpio_chip *gc, unsigned offset)
{
	struct mxc_gpio_port *port = gpiochip_get_data(gc);

	return irq_find_mapping(port->domain, offset);
}



/* 通过hwirq找到virq */
unsigned int irq_find_mapping(struct irq_domain *domain,
			      irq_hw_number_t hwirq)
{
	struct irq_data *data;

	/* Look for default domain if nececssary */
	if (domain == NULL)
		domain = irq_default_domain;
	if (domain == NULL)
		return 0;

	if (hwirq < domain->revmap_direct_max_irq) {
		data = irq_domain_get_irq_data(domain, hwirq);
		if (data && data->hwirq == hwirq)
			return hwirq;
	}

	/* linear形式保存的映射关系,通过hwirq寻找virq */
	if (hwirq < domain->revmap_size)
		return domain->linear_revmap[hwirq];

	/* tree形式保存的映射关系,通过hwirq寻找virq */
	data = radix_tree_lookup(&domain->revmap_tree, hwirq);

	return data ? data->irq : 0;
}
```

### 2 gpio_keys_irq_isr()

```C
/* 当产生gpio中断时 */
static irqreturn_t gpio_keys_irq_isr(int irq, void *dev_id)
{
	struct gpio_button_data *bdata = dev_id;
	const struct gpio_keys_button *button = bdata->button;
	struct input_dev *input = bdata->input;
	unsigned long flags;


	if (!bdata->key_pressed) {
		if (bdata->button->wakeup)
			pm_wakeup_event(bdata->input->dev.parent, 0);

        /* 发送input子系统事件 */
		input_event(input, EV_KEY, button->code, 1);
        /* 再发送同步事件 */
		input_sync(input);

        /*如果没有延时,则发送立即事件，代表按键释放 */
		if (!bdata->release_delay) {
			input_event(input, EV_KEY, button->code, 0);
			input_sync(input);
			goto out;
		}

		bdata->key_pressed = true;
	}

    /*延迟一会再发送事件，代表按键释放 回调函数：gpio_keys_irq_timer() */
	if (bdata->release_delay)
		mod_timer(&bdata->release_timer,
			jiffies + msecs_to_jiffies(bdata->release_delay));

	return IRQ_HANDLED;
}

```

### 2-1 gpio_keys_irq_timer()

```C
static void gpio_keys_irq_timer(unsigned long _data)
{
	struct gpio_button_data *bdata = (struct gpio_button_data *)_data;
	struct input_dev *input = bdata->input;
	unsigned long flags;

	/*延时到了，直接发送事件，代表按键释放 */
	if (bdata->key_pressed) {
		input_event(input, EV_KEY, bdata->button->code, 0);
		input_sync(input);
		bdata->key_pressed = false;
	}

}
```

### 3 gpio_keys_gpio_isr

```C
static irqreturn_t gpio_keys_gpio_isr(int irq, void *dev_id)
{
	struct gpio_button_data *bdata = dev_id;


	if (bdata->button->wakeup)
		pm_stay_awake(bdata->input->dev.parent);

    /*
     * 延时处理上报事件,如果存在按键抖动，此时又会进入此中断，
     * 延时时间又重新覆盖，直到不再抖动，此时就会延时完毕，
     * 执行回调函数 gpio_keys_gpio_work_func()
     */
	mod_delayed_work(system_wq,
			 &bdata->work,
			 msecs_to_jiffies(bdata->software_debounce));

	return IRQ_HANDLED;
}
```

### 3-1 gpio_keys_gpio_work_func()

```C
static void gpio_keys_gpio_work_func(struct work_struct *work)
{
	struct gpio_button_data *bdata =
		container_of(work, struct gpio_button_data, work.work);

	gpio_keys_gpio_report_event(bdata);

	if (bdata->button->wakeup)
		pm_relax(bdata->input->dev.parent);
}
```

### 3-1-1 gpio_keys_gpio_report_event()

```C
static void gpio_keys_gpio_report_event(struct gpio_button_data *bdata)
{
	const struct gpio_keys_button *button = bdata->button;
	struct input_dev *input = bdata->input;
	unsigned int type = button->type ?: EV_KEY;
	int state;

    /* 读取引脚电平 */
	state = gpiod_get_value_cansleep(bdata->gpiod);
	if (state < 0) {
		dev_err(input->dev.parent,
			"failed to get gpio state: %d\n", state);
		return;
	}

    /* 根据引脚当前电平上报事件 */
	if (type == EV_ABS) {
		if (state)
			input_event(input, type, button->code, button->value);
	} else {
		input_event(input, type, button->code, state);
	}
	input_sync(input);
}
```

### 4 input_register_device()

```C
int input_register_device(struct input_dev *dev)
{
	struct input_devres *devres = NULL;
	struct input_handler *handler;
	unsigned int packet_size;
	const char *path;
	int error;

    /* 动态自动释放 */
	if (dev->devres_managed) {
		devres = devres_alloc(devm_input_device_unregister,
				      sizeof(struct input_devres), GFP_KERNEL);
		devres->input = dev;
	}

	/* 每个input event都要支持EV_SYN */
	__set_bit(EV_SYN, dev->evbit);

	/* 计算事件预估数量，提供handler预估，需申请多少缓存 */
	packet_size = input_estimate_events_per_packet(dev);
	if (dev->hint_events_per_packet < packet_size)
		dev->hint_events_per_packet = packet_size;

    /* 根据数量，申请 input_value */
	dev->max_vals = dev->hint_events_per_packet + 2;
	dev->vals = kcalloc(dev->max_vals, sizeof(*dev->vals), GFP_KERNEL);


    /* 使能repeat */
	if (!dev->rep[REP_DELAY] && !dev->rep[REP_PERIOD])
		input_enable_softrepeat(dev, 250, 33);

	if (!dev->getkeycode)
		dev->getkeycode = input_default_getkeycode;

	if (!dev->setkeycode)
		dev->setkeycode = input_default_setkeycode;

	error = device_add(&dev->dev);


	path = kobject_get_path(&dev->dev.kobj, GFP_KERNEL);
	pr_info("%s as %s\n",
		dev->name ? dev->name : "Unspecified device",
		path ? path : "N/A");
	kfree(path);

	/* 把设备挂到全局的input子系统设备链表 input_dev_list */ 
	list_add_tail(&dev->node, &input_dev_list);
    

   /*  input设备在增加到input_dev_list链表上之后，会查找 
    * input_handler_list事件处理链表上的handler进行匹配，所有的
    * input device 都挂在input_dev_list上，所有类型的事件都挂在 
    * input_handler_list上，进行match。
    */ 
	list_for_each_entry(handler, &input_handler_list, node)
		input_attach_handler(dev, handler);

	if (dev->devres_managed) {
		devres_add(dev->dev.parent, devres);
	}
	return 0;
}
```

### 总结

- 先解析dt中"gpio-keys"节点，申请，并将解析的内容gpio_keys_platform_data。
- 再解析"gpio-keys"节点的子节点，根据子节点数量，申请多个gpio_keys_button[]，将每个button(子节点)解析的内容保存，并将gpio_keys_button[]保存在 gpio_keys_platform_data->buttons。

- 申请input_dev，并根据上述解析的内容，设置和注册input_dev。( 调用 input_register_device() 函数 )

- 调用gpio子系统的函数，通过gpio号，找到gpio_desc，配置好引脚。

- 再通过gpio_desc的hwirq，寻找virq，开启和设置中断，配置好回调函数。

- 当发送中断后，并经过消抖处理（利用workqueqe和timer），最终上报按键事件和同步事件。

  
