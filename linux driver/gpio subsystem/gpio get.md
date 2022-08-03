### struct of_phandle_args

```C
#define MAX_PHANDLE_ARGS 16
struct of_phandle_args {
        struct device_node *np;
        int args_count;
        uint32_t args[MAX_PHANDLE_ARGS];
};
```

### 1 gpiod_get()

```C
/*
 * 1. 函数适用于只有一个gpio，index为0，即list = <&phandle1 1 2>;
 * 2. 如果想使用phandle2，list = <&phandle1 1 2 &phandle2 3 3>;
 *    则为 gpiod_get_index(dev, con_id, 1, flags);
 */
struct gpio_desc *__must_check gpiod_get(struct device *dev, const char *con_id,
					 enum gpiod_flags flags)
{
	return gpiod_get_index(dev, con_id, 0, flags);
}


struct gpio_desc *__must_check gpiod_get_index(struct device *dev,
					       const char *con_id,
					       unsigned int idx,
					       enum gpiod_flags flags)
{
	struct gpio_desc *desc = NULL;
	int status;
	enum gpio_lookup_flags lookupflags = 0;


	if (dev) {
        
		/*使用dt*/
		if (IS_ENABLED(CONFIG_OF) && dev->of_node) {
           
            /* 
             * 解析dt,找到gpio-controller，根据解析的引脚号，
             * 找到对应 gpio-controller 的 gpio_desc。 
             */ 
			desc = of_find_gpio(dev, con_id, idx, &lookupflags);            
		} 
	}

    /* 有关"gpio-ranges",没什么用 */  
	status = gpiod_request(desc, con_id);

	/* 根据传入flag设置方向，flag为null会设为输入 */
	status = gpiod_configure_flags(desc, con_id, lookupflags, flags);

	return desc;
}
```

### 1-1 of_find_gpio()

```C
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
        
        /* 如 "led-gpios" 或 "led-gpio" 的属性名 */
		if (con_id)
			snprintf(prop_name, sizeof(prop_name), "%s-%s", con_id,
				 gpio_suffixes[i]);
        
        /* 合成，如 "gpios" 或 "gpio" 的属性名 */
		else 
			snprintf(prop_name, sizeof(prop_name), "%s",
				 gpio_suffixes[i]);

        /* 
         * 1. 根据属性名(led-gpios)查找匹配，保存内容在gpio_desc
         * 2. 根据gpiospec的phandle找到chip，即：有gpio-controller的节点。
         * 3. 根据gpiospec的引脚号，找到在chip中的desc数组偏移位置， 
         *    即对应的引脚号的desc
         * 4. 并返回了flags "OF_GPIO_ACTIVE_LOW"
         */
		desc = of_get_named_gpiod_flags(dev->of_node, prop_name, idx,
						&of_flags);

	}
	
    /* 如果是低电平有效，设置flag */
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

### 1-1-1 of_get_named_gpiod_flags()

```C
/* 
 * 1. 在user的node(如led)，解析dt有关gpio的内容，
 * 2. 寻找要调用的gpio-controller，进而找到gpiochip
 * 3. 通过dt提供的引脚号，找到对应引脚的gpio_desc
 */
struct gpio_desc *of_get_named_gpiod_flags(struct device_node *np,
		     const char *propname, int index, enum of_gpio_flags *flags)
{
	struct of_phandle_args gpiospec;
	struct gpio_chip *chip;
	struct gpio_desc *desc;
	int ret;

    /*
     * propname：当前node的属性名
     * "#gpio-cells"：父node的属性名
     * index：第index个phandle
     * 举例：propname = <&phandle1 1 2 &phandle2 3>;
     *       1. 当#gpio-cells为2，index为0时，
     *       2. gpiospec->arg[0] = 1 | [1] = 2;
     *          gpiospec->args_count = 2;
     *          gpiospec->np = phandle1;
     */  
	ret = of_parse_phandle_with_args(np, propname, "#gpio-cells", index,
					 &gpiospec);

    	
	 /* 根据gpiospec，遍历 gpio_devices，找到匹配的gpiochip */
	chip = of_find_gpiochip_by_xlate(&gpiospec);


	/* 根据dt中，"xx-gpios"属性提供的引脚号，找到对应引脚的gpio_desc */
	desc = of_xlate_and_get_gpiod_flags(chip, &gpiospec, flags);

	return desc;
}
```

### 1-2 gpiod_request()

```C
/* 
 * desc：对应引脚号的gpio_desc 
 * lable："led" 
*/
int gpiod_request(struct gpio_desc *desc, const char *label)
{
	int status = -EPROBE_DEFER;
	struct gpio_device *gdev;

	gdev = desc->gdev;

	status = __gpiod_request(desc, label);

	return status;
}



static int __gpiod_request(struct gpio_desc *desc, const char *label)
{
	struct gpio_chip	*chip = desc->gdev->chip;
	int			status;
	unsigned long		flags;


	/* 设置label("led-gpios的led").如果为NULL,设置为"?" */
	desc_set_label(desc, label ? : "?");
	status = 0;
  
    /* 没有"gpio-ranges"属性，无作用 */
	if (chip->request) {    
        /* mxc_gpio_request */
        status = chip->request(chip, gpio_chip_hwgpio(desc));
    }
    
    /*获得gpio方向*/
	if (chip->get_direction) {
		gpiod_get_direction(desc);
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

    /* 
     * 将解析dt的flag，记录在对应引脚的gpio_desc->flags 
     * 用途是实现逻辑电平的统一，无关实际物理电平。
     */
	if (lflags & GPIO_ACTIVE_LOW)
		set_bit(FLAG_ACTIVE_LOW, &desc->flags);
    
	if (lflags & GPIO_OPEN_DRAIN)
		set_bit(FLAG_OPEN_DRAIN, &desc->flags);
    
	if (lflags & GPIO_OPEN_SOURCE)
		set_bit(FLAG_OPEN_SOURCE, &desc->flags);

    
    /*
     * gpio_get解析传入的flag，引脚设置方向
     * GPIOD_FLAGS_BIT_DIR_OUT： 输出
     * others： 输入
     */
	if (dflags & GPIOD_FLAGS_BIT_DIR_OUT)       
		status = gpiod_direction_output(desc,
					      dflags & GPIOD_FLAGS_BIT_DIR_VAL);
	else
		status = gpiod_direction_input(desc);

	return status;
}

```

### 总结

- gpiod_get_index（index）

  - 根据传入参数index，确认要使用“xxx-gpios”的第几个pin。
  - 解析dt属性，获取使用了哪个gpio-controller，使用了哪个引脚号。
  - 找到gpio-controller节点，获取已配置好的gpio_device。
  - 根据引脚号，找到对应挂在gpio_device的gpio_desc，返回给user。

  

- gpiod_set_value()，gpiod_direction_output()，gpiod_direction_input() 等

  - 因为已经获得gpio_desc，则反向找到gpio_device的gpiochip。

  - 利用gpiochip提供的中间层函数，操作gpio，如设置电平，设置方向。
  
  
  
- 在dt中，ngpios意思是：gpio寄存器的可使用的数量，即寄存器是32位宽，但只有18位对应了控制引脚，1个gpio-controller控制18个pin，设置"ngpios = <18>，通知driver只有18个pin。

