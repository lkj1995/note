### gpiod_get()

```C
/*
 * 1. 函数只适用于只有一个gpio的情况，index为0，即list = <&phandle1 1 2>;
 * 2. 如果想使用phandle2，list = <&phandle1 1 2 &phandle2 3 3>;
 * 使用 gpiod_get_index(dev, con_id, 1, flags);
 */
/**
 * gpiod_get - obtain a GPIO for a given GPIO function
 * @dev:	GPIO consumer, can be NULL for system-global GPIOs
 * @con_id:	function within the GPIO consumer
 * @flags:	optional GPIO initialization flags
 *
 * Return the GPIO descriptor corresponding to the function con_id of device
 * dev, -ENOENT if no GPIO has been assigned to the requested function, or
 * another IS_ERR() code if an error occurred while trying to acquire the GPIO.
 */
struct gpio_desc *__must_check gpiod_get(struct device *dev, const char *con_id,
					 enum gpiod_flags flags)
{
	return gpiod_get_index(dev, con_id, 0, flags);
}
```

### 1 gpiod_get_index()

```C
/**
 * gpiod_get_index - obtain a GPIO from a multi-index GPIO function
 * @dev:	GPIO consumer, can be NULL for system-global GPIOs
 * @con_id:	function within the GPIO consumer
 * @idx:	index of the GPIO to obtain in the consumer
 * @flags:	optional GPIO initialization flags
 *
 * This variant of gpiod_get() allows to access GPIOs other than the first
 * defined one for functions that define several GPIOs.
 *
 * Return a valid GPIO descriptor, -ENOENT if no GPIO has been assigned to the
 * requested function and/or index, or another IS_ERR() code if an error
 * occurred while trying to acquire the GPIO.
 */
struct gpio_desc *__must_check gpiod_get_index(struct device *dev,
					       const char *con_id,
					       unsigned int idx,
					       enum gpiod_flags flags)
{
	struct gpio_desc *desc = NULL;
	int status;
	enum gpio_lookup_flags lookupflags = 0;

	dev_dbg(dev, "GPIO lookup for consumer %s\n", con_id);

	if (dev) {
		/* Using device tree?*/
		if (IS_ENABLED(CONFIG_OF) && dev->of_node) {/*使用设备树*/
			dev_dbg(dev, "using device tree for GPIO lookup\n");
            
            /*
             * 找到该node下"<con_id>-gpios"的属性
             * 保存其内容，并找到chip，根据引脚号找到最终的desc
             * lookupflags：保存了引脚的flag
             */
			desc = of_find_gpio(dev, con_id, idx, &lookupflags);
            
		} else if (ACPI_COMPANION(dev)) {
			dev_dbg(dev, "using ACPI for GPIO lookup\n");
			desc = acpi_find_gpio(dev, con_id, idx, flags, &lookupflags);
		}
	}

	/*
	 * Either we are not using DT or ACPI, or their lookup did not return
	 * a result. In that case, use platform lookup as a fallback.
	 */
	if (!desc || desc == ERR_PTR(-ENOENT)) {
		dev_dbg(dev, "using lookup tables for GPIO lookup\n");
		desc = gpiod_find(dev, con_id, idx, &lookupflags);
	}

	if (IS_ERR(desc)) {
		dev_dbg(dev, "lookup for GPIO %s failed\n", con_id);
		return desc;
	}
	
    
    
	status = gpiod_request(desc, con_id);
	if (status < 0)
		return ERR_PTR(status);

	status = gpiod_configure_flags(desc, con_id, lookupflags, flags);
	if (status < 0) {
		dev_dbg(dev, "setup of GPIO %s failed\n", con_id);
		gpiod_put(desc);
		return ERR_PTR(status);
	}

	return desc;
}
```

### 1-1 of_find_gpio()

```C
/* 截取dts，使用gpiod_get()的dev为消费者
&usdhc1 {
    pinctrl-names = "default";
    pinctrl-0 = <&pinctrl_usdhc1>;
    cd-gpios = <&gpio1 19 GPIO_ACTIVE_LOW>;
    keep-power-in-suspend;
    enable-sdio-wakeup;
    bus-width = <4>;
    status = "okay";
};
*/

static const char * const gpio_suffixes[] = { "gpios", "gpio" };


struct gpio_desc *of_find_gpio(struct device *dev, const char *con_id,
			       unsigned int idx,
			       enum gpio_lookup_flags *flags)
{
	char prop_name[32]; /* 32 is max size of property name */
	enum of_gpio_flags of_flags;
	struct gpio_desc *desc;
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(gpio_suffixes); i++) {
		if (con_id)/* 合成，如 "led-gpios" 或 "led-gpio" 的属性名 */
			snprintf(prop_name, sizeof(prop_name), "%s-%s", con_id,
				 gpio_suffixes[i]);
		else/* 合成，如 "gpios" 或 "gpio" 的属性名 */
			snprintf(prop_name, sizeof(prop_name), "%s",
				 gpio_suffixes[i]);

        /* 
         * 1. 根据属性名(led-gpios)查找匹配，保存内容在gpiospec
         * 2. 根据gpiospec的phandle找到chip，即：有gpio-controller的节点。
         * 3. 根据gpiospec的引脚号，找到在chip中的desc数组偏移位置， 
         *    即对应的引脚号的desc
         * 4. 并返回了flags "OF_GPIO_ACTIVE_LOW"
         */
		desc = of_get_named_gpiod_flags(dev->of_node, prop_name, idx,
						&of_flags);
		if (!IS_ERR(desc) || (PTR_ERR(desc) != -ENOENT))
			break;
	}

	if (IS_ERR(desc))
		return desc;
	
    /*判断，并返回其flags*/
	if (of_flags & OF_GPIO_ACTIVE_LOW)
		*flags |= GPIO_ACTIVE_LOW;

	if (of_flags & OF_GPIO_SINGLE_ENDED) {
		if (of_flags & OF_GPIO_ACTIVE_LOW)
			*flags |= GPIO_OPEN_DRAIN;
		else
			*flags |= GPIO_OPEN_SOURCE;
	}

	return desc;
}
```

### struct of_phandle_args

```C
#define MAX_PHANDLE_ARGS 16
struct of_phandle_args {
        struct device_node *np;
        int args_count;
        uint32_t args[MAX_PHANDLE_ARGS];
};
```

### 1-1-1 of_get_named_gpiod_flags()

```C
/**
 * of_get_named_gpiod_flags() - Get a GPIO descriptor and flags for GPIO API
 * @np:		device node to get GPIO from
 * @propname:	property name containing gpio specifier(s)
 * @index:	index of the GPIO
 * @flags:	a flags pointer to fill in
 *
 * Returns GPIO descriptor to use with Linux GPIO API, or one of the errno
 * value on the error condition. If @flags is not NULL the function also fills
 * in flags for the GPIO.
 */
struct gpio_desc *of_get_named_gpiod_flags(struct device_node *np,
		     const char *propname, int index, enum of_gpio_flags *flags)
{
	struct of_phandle_args gpiospec;
	struct gpio_chip *chip;
	struct gpio_desc *desc;
	int ret;

    /*
     * 根据propname属性内容，第index个，找到对应的gpio控制器，并记录内容
     * propname = <&phandle1 1 2 &phandle2 3>;
     * 记录： 1. #gpio-cells为2
     *       2. arg[]保存1，2
     *       3. node为phandle1
     */  
	ret = of_parse_phandle_with_args(np, propname, "#gpio-cells", index,
					 &gpiospec);
	if (ret) {
		pr_debug("%s: can't parse '%s' property of node '%s[%d]'\n",
			__func__, propname, np->full_name, index);
		return ERR_PTR(ret);
	}
    
	/*
	 * 在全局gpio_devices->list中，根据gpiospec匹配chip
	 * 匹配方法：检查两者的node是否一致，并执行of_xlate检查
	 */
	chip = of_find_gpiochip_by_xlate(&gpiospec);
	if (!chip) {
		desc = ERR_PTR(-EPROBE_DEFER);
		goto out;
	}

    /*
     * 根据gpiospec保存的引脚号，找到chip中对应的desc数组的偏移位置，返回对应的desc
     * 并返回了flags，如"ACTIVE_LOW"
     */
	desc = of_xlate_and_get_gpiod_flags(chip, &gpiospec, flags);
	if (IS_ERR(desc))
		goto out;

	pr_debug("%s: parsed '%s' property of node '%s[%d]' - status (%d)\n",
		 __func__, propname, np->full_name, index,
		 PTR_ERR_OR_ZERO(desc));

out:
	of_node_put(gpiospec.np);

	return desc;
}
```

### 1-1-1-1 of_parse_phandle_with_args()

```c
/* 
 * 作用：如下图所示，根据传入的参数，确认node3中的list内容，找到phandle2(node2)并记录
 * 根据#list-cells长度，读取后面的参数内容，并返回#list-cells的值
 */

/**
 * of_parse_phandle_with_args() - Find a node pointed by phandle in a list
 * @np:		pointer to a device tree node containing a list
 * @list_name:	property name that contains a list
 * @cells_name:	property name that specifies phandles' arguments count
 * @index:	index of a phandle to parse out
 * @out_args:	optional pointer to output arguments structure (will be filled)
 *
 * This function is useful to parse lists of phandles and their arguments.
 * Returns 0 on success and fills out_args, on error returns appropriate
 * errno value.
 *
 * Caller is responsible to call of_node_put() on the returned out_args->np
 * pointer.
 *
 * Example:
 *
 * phandle1: node1 {
 *	#list-cells = <2>;
 * }
 *
 * phandle2: node2 {
 *	#list-cells = <1>;
 * }
 *
 * node3 {
 *	list = <&phandle1 1 2 &phandle2 3>;
 * }
 *
 * To get a device_node of the `node2' node you may call this:
 * of_parse_phandle_with_args(node3, "list", "#list-cells", 1, &args);
 */
int of_parse_phandle_with_args(const struct device_node *np, const char *list_name,
				const char *cells_name, int index,
				struct of_phandle_args *out_args)
{
	if (index < 0)
		return -EINVAL;
	return __of_parse_phandle_with_args(np, list_name, cells_name, 0,
					    index, out_args);
}


static int __of_parse_phandle_with_args(const struct device_node *np,
					const char *list_name,
					const char *cells_name,
					int cell_count, int index,
					struct of_phandle_args *out_args)
{
	struct of_phandle_iterator it;
	int rc, cur_index = 0;

	/* Loop over the phandles until all the requested entry is found */
	of_for_each_phandle(&it, rc, np, list_name, cells_name, cell_count) {
		/*
		 * All of the error cases bail out of the loop, so at
		 * this point, the parsing is successful. If the requested
		 * index matches, then fill the out_args structure and return,
		 * or return -ENOENT for an empty entry.
		 */
		rc = -ENOENT;
        
        /*找到符合条件的index,如:传入0代表第一个'node0' */
		if (cur_index == index) { 
			if (!it.phandle)
				goto err;

			if (out_args) {
				int c;
                
				/*
				 * 存储phandle后的参数。
				 * cells决定了保存多少个参数，并返回cells的值。
				 */
				c = of_phandle_iterator_args(&it,
							     out_args->args,
							     MAX_PHANDLE_ARGS);
                /*保存phandle=是哪个node*/
				out_args->np = it.node;
                /*保存数量*/
				out_args->args_count = c;
			} else {
				of_node_put(it.node);
			}

			/* Found it! return success */
			return 0;
		}

		cur_index++;
	}

	/*
	 * Unlock node before returning result; will be one of:
	 * -ENOENT : index is for empty phandle
	 * -EINVAL : parsing error on data
	 */

 err:
	of_node_put(it.node);
	return rc;
}
```

### 1-1-1-2 of_find_gpiochip_by_xlate()

```c
static struct gpio_chip *of_find_gpiochip_by_xlate(
					struct of_phandle_args *gpiospec)
{
	return gpiochip_find(gpiospec, of_gpiochip_match_node_and_xlate);
}

/**
 * gpiochip_find() - iterator for locating a specific gpio_chip
 * @data: data to pass to match function
 * @callback: Callback function to check gpio_chip
 *
 * Similar to bus_find_device.  It returns a reference to a gpio_chip as
 * determined by a user supplied @match callback.  The callback should return
 * 0 if the device doesn't match and non-zero if it does.  If the callback is
 * non-zero, this function will return to the caller and not iterate over any
 * more gpio_chips.
 */
struct gpio_chip *gpiochip_find(void *data,
				int (*match)(struct gpio_chip *chip,
					     void *data))
{
	struct gpio_device *gdev;
	struct gpio_chip *chip = NULL;
	unsigned long flags;

	spin_lock_irqsave(&gpio_lock, flags);
    /* 从全局链表gpio_devices->list中匹配 */
	list_for_each_entry(gdev, &gpio_devices, list)
		if (gdev->chip && match(gdev->chip, data)) {
			chip = gdev->chip;
			break;
		}

	spin_unlock_irqrestore(&gpio_lock, flags);

	return chip;
}
```

### 1-1-1-2-1 of_gpiochip_match_node_and_xlate()

```C
/* chip->match() */
static int of_gpiochip_match_node_and_xlate(struct gpio_chip *chip, void *data)
{
	struct of_phandle_args *gpiospec = data;
	/*匹配of_node,xlate*/
	return chip->gpiodev->dev.of_node == gpiospec->np &&
				chip->of_xlate(chip, gpiospec, NULL) >= 0;
}
```

### 1-1-2 of_xlate_and_get_gpiod_flags()

```C
static struct gpio_desc *of_xlate_and_get_gpiod_flags(struct gpio_chip *chip,
					struct of_phandle_args *gpiospec,
					enum of_gpio_flags *flags)
{
	int ret;

    /*参数长度要一致*/
	if (chip->of_gpio_n_cells != gpiospec->args_count)
		return ERR_PTR(-EINVAL);

    /*返回引脚号*/
	ret = chip->of_xlate(chip, gpiospec, flags);
	if (ret < 0)
		return ERR_PTR(ret);

    /*根据引脚号找到对应在chip保存的desc数组中的偏移位置*/
	return gpiochip_get_desc(chip, ret);
}
```

### of_gpio_simple_xlate()

```C
/*
 * 在of_gpiochip_add被赋值为：chip->of_xlate = of_gpio_simple_xlate;
 */

/**
 * of_gpio_simple_xlate - translate gpio_spec to the GPIO number and flags
 * @gc:		pointer to the gpio_chip structure
 * @np:		device node of the GPIO chip
 * @gpio_spec:	gpio specifier as found in the device tree
 * @flags:	a flags pointer to fill in
 *
 * This is simple translation function, suitable for the most 1:1 mapped
 * gpio chips. This function performs only one sanity check: whether gpio
 * is less than ngpios (that is specified in the gpio_chip).
 */
int of_gpio_simple_xlate(struct gpio_chip *gc,
			 const struct of_phandle_args *gpiospec, u32 *flags)
{
	/*
	 * We're discouraging gpio_cells < 2, since that way you'll have to
	 * write your own xlate function (that will have to retrieve the GPIO
	 * number and the flags from a single gpio cell -- this is possible,
	 * but not recommended).
	 */
	if (gc->of_gpio_n_cells < 2) {
		WARN_ON(1);
		return -EINVAL;
	}

	if (WARN_ON(gpiospec->args_count < gc->of_gpio_n_cells))
		return -EINVAL;

    /*当前引脚号不能大于引脚总数量*/
	if (gpiospec->args[0] >= gc->ngpio)
		return -EINVAL;
	
    /*保存flag，如"ACTIVE_LOW"*/
	if (flags)
		*flags = gpiospec->args[1];

    /*返回引脚号*/
	return gpiospec->args[0];
}
```

### 1-1-2-1 gpiochip_get_desc()

```C
/**
 * Get the GPIO descriptor corresponding to the given hw number for this chip.
 */
struct gpio_desc *gpiochip_get_desc(struct gpio_chip *chip,
				    u16 hwnum)
{
	struct gpio_device *gdev = chip->gpiodev;

	if (hwnum >= gdev->ngpio)
		return ERR_PTR(-EINVAL);

	return &gdev->descs[hwnum];
}
```

### 1-2 gpiod_request()

```C
int gpiod_request(struct gpio_desc *desc, const char *label)
{
	int status = -EPROBE_DEFER;
	struct gpio_device *gdev;

	VALIDATE_DESC(desc);
	gdev = desc->gdev;

	if (try_module_get(gdev->owner)) {
		status = __gpiod_request(desc, label);
		if (status < 0)
			module_put(gdev->owner);
		else
			get_device(&gdev->dev);
	}

	if (status)
		gpiod_dbg(desc, "%s: status %d\n", __func__, status);

	return status;
}


/* These "optional" allocation calls help prevent drivers from stomping
 * on each other, and help provide better diagnostics in debugfs.
 * They're called even less than the "set direction" calls.
 */
static int __gpiod_request(struct gpio_desc *desc, const char *label)
{
	struct gpio_chip	*chip = desc->gdev->chip;
	int			status;
	unsigned long		flags;

	spin_lock_irqsave(&gpio_lock, flags);

	/* NOTE:  gpio_request() can be called in early boot,
	 * before IRQs are enabled, for non-sleeping (SOC) GPIOs.
	 */
	/*设置label("led-gpios的led"),如果为空设置为"?"*/
	if (test_and_set_bit(FLAG_REQUESTED, &desc->flags) == 0) {
		desc_set_label(desc, label ? : "?");
		status = 0;
	} else {
		status = -EBUSY;
		goto done;
	}

	if (chip->request) {
		/* chip->request may sleep */
		spin_unlock_irqrestore(&gpio_lock, flags);
		status = chip->request(chip, gpio_chip_hwgpio(desc));
		spin_lock_irqsave(&gpio_lock, flags);

		if (status < 0) {
			desc_set_label(desc, NULL);
			clear_bit(FLAG_REQUESTED, &desc->flags);
			goto done;
		}
	}
    /*获得gpio方向*/
	if (chip->get_direction) {
		/* chip->get_direction may sleep */
		spin_unlock_irqrestore(&gpio_lock, flags);
		gpiod_get_direction(desc);
		spin_lock_irqsave(&gpio_lock, flags);
	}
done:
	spin_unlock_irqrestore(&gpio_lock, flags);
	return status;
}
```





### 1-2-2 gpiod_get_direction()

```C
/**
 * gpiod_get_direction - return the current direction of a GPIO
 * @desc:	GPIO to get the direction of
 *
 * Return GPIOF_DIR_IN or GPIOF_DIR_OUT, or an error code in case of error.
 *
 * This function may sleep if gpiod_cansleep() is true.
 */
int gpiod_get_direction(struct gpio_desc *desc)
{
	struct gpio_chip	*chip;
	unsigned		offset;
	int			status = -EINVAL;

	chip = gpiod_to_chip(desc);
    
    /* 
     * return desc - &desc->gdev->descs[0]; 
     * 返回偏移位置，类型为
     */
	offset = gpio_chip_hwgpio(desc);

	if (!chip->get_direction)
		return status;
	
    /*执行获取方向函数*/
	status = chip->get_direction(chip, offset);
	if (status > 0) {
		/* GPIOF_DIR_IN, or other positive */
		status = 1;
		clear_bit(FLAG_IS_OUT, &desc->flags);
	}
	if (status == 0) {
		/* GPIOF_DIR_OUT */
		set_bit(FLAG_IS_OUT, &desc->flags);
	}
	return status;
}
```







### 1-3 gpiod_configure_flags()

```C
/**
 * gpiod_configure_flags - helper function to configure a given GPIO
 * @desc:	gpio whose value will be assigned
 * @con_id:	function within the GPIO consumer
 * @lflags:	gpio_lookup_flags - returned from of_find_gpio() or
 *		of_get_gpio_hog()
 * @dflags:	gpiod_flags - optional GPIO initialization flags
 *
 * Return 0 on success, -ENOENT if no GPIO has been assigned to the
 * requested function and/or index, or another IS_ERR() code if an error
 * occurred while trying to acquire the GPIO.
 */
static int gpiod_configure_flags(struct gpio_desc *desc, const char *con_id,
		unsigned long lflags, enum gpiod_flags dflags)
{
	int status;

    /*设备树设置好的flag*/
	if (lflags & GPIO_ACTIVE_LOW)
		set_bit(FLAG_ACTIVE_LOW, &desc->flags);
	if (lflags & GPIO_OPEN_DRAIN)
		set_bit(FLAG_OPEN_DRAIN, &desc->flags);
	if (lflags & GPIO_OPEN_SOURCE)
		set_bit(FLAG_OPEN_SOURCE, &desc->flags);

    /*传入的flag，如果为空，无需修改，直接退出*/
	/* No particular flag request, return here... */
	if (!(dflags & GPIOD_FLAGS_BIT_DIR_SET)) {
		pr_debug("no flags found for %s\n", con_id);
		return 0;
	}

	/* Process flags */
	if (dflags & GPIOD_FLAGS_BIT_DIR_OUT)
		status = gpiod_direction_output(desc,
					      dflags & GPIOD_FLAGS_BIT_DIR_VAL);
	else
		status = gpiod_direction_input(desc);

	return status;
}
