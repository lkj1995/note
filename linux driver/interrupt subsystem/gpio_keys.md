### dts的节点

```
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
/**
 * struct gpio_keys_platform_data - platform data for gpio_keys driver
 * @buttons:		pointer to array of &gpio_keys_button structures
 *			describing buttons attached to the device
 * @nbuttons:		number of elements in @buttons array
 * @poll_interval:	polling interval in msecs - for polling driver only
 * @rep:		enable input subsystem auto repeat
 * @enable:		platform hook for enabling the device
 * @disable:		platform hook for disabling the device
 * @name:		input device name
 */
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
/**
 * struct gpio_keys_button - configuration parameters
 * @code:		input event code (KEY_*, SW_*)
 * @gpio:		%-1 if this key does not support gpio
 * @active_low:		%true indicates that button is considered
 *			depressed when gpio is low
 * @desc:		label that will be attached to button's gpio
 * @type:		input event type (%EV_KEY, %EV_SW, %EV_ABS)
 * @wakeup:		configure the button as a wake-up source
 * @debounce_interval:	debounce ticks interval in msecs
 * @can_disable:	%true indicates that userspace is allowed to
 *			disable button via sysfs
 * @value:		axis value for %EV_ABS
 * @irq:		Irq number in case of interrupt keys
 * @gpiod:		GPIO descriptor
 */
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

### gpio_keys_probe()

```C
static int gpio_keys_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	const struct gpio_keys_platform_data *pdata = dev_get_platdata(dev);
	struct gpio_keys_drvdata *ddata;
	struct input_dev *input;
	size_t size;
	int i, error;
	int wakeup = 0;

	if (!pdata) {
		pdata = gpio_keys_get_devtree_pdata(dev);
		if (IS_ERR(pdata))
			return PTR_ERR(pdata);
	}

	size = sizeof(struct gpio_keys_drvdata) +
			pdata->nbuttons * sizeof(struct gpio_button_data);
	ddata = devm_kzalloc(dev, size, GFP_KERNEL);
	if (!ddata) {
		dev_err(dev, "failed to allocate state\n");
		return -ENOMEM;
	}

	input = devm_input_allocate_device(dev);
	if (!input) {
		dev_err(dev, "failed to allocate input device\n");
		return -ENOMEM;
	}

	ddata->pdata = pdata;
	ddata->input = input;
	mutex_init(&ddata->disable_lock);

	platform_set_drvdata(pdev, ddata);
	input_set_drvdata(input, ddata);

	input->name = pdata->name ? : pdev->name;
	input->phys = "gpio-keys/input0";
	input->dev.parent = &pdev->dev;
	input->open = gpio_keys_open;
	input->close = gpio_keys_close;

	input->id.bustype = BUS_HOST;
	input->id.vendor = 0x0001;
	input->id.product = 0x0001;
	input->id.version = 0x0100;

	/* Enable auto repeat feature of Linux input subsystem */
	if (pdata->rep)
		__set_bit(EV_REP, input->evbit);

	for (i = 0; i < pdata->nbuttons; i++) {
		const struct gpio_keys_button *button = &pdata->buttons[i];
		struct gpio_button_data *bdata = &ddata->data[i];

		error = gpio_keys_setup_key(pdev, input, bdata, button);
		if (error)
			return error;

		if (button->wakeup)
			wakeup = 1;
	}

	error = sysfs_create_group(&pdev->dev.kobj, &gpio_keys_attr_group);
	if (error) {
		dev_err(dev, "Unable to export keys/switches, error: %d\n",
			error);
		return error;
	}

	error = input_register_device(input);
	if (error) {
		dev_err(dev, "Unable to register input device, error: %d\n",
			error);
		goto err_remove_group;
	}

	device_init_wakeup(&pdev->dev, wakeup);

	return 0;

err_remove_group:
	sysfs_remove_group(&pdev->dev.kobj, &gpio_keys_attr_group);
	return error;
}

```

### gpio_keys_get_devtree_pdata()

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

    /*取出node节点*/
	node = dev->of_node;
	if (!node)
		return ERR_PTR(-ENODEV);

    /*获取有效的子节点数量*/
	nbuttons = of_get_available_child_count(node);
	if (nbuttons == 0)
		return ERR_PTR(-ENODEV);

    /* 申请一个 gpio_keys_platform_data 和 button数量 * gpio_keys_button */
	pdata = devm_kzalloc(dev,
			     sizeof(*pdata) + nbuttons * sizeof(*button),
			     GFP_KERNEL);
	if (!pdata)
		return ERR_PTR(-ENOMEM);
	
    /*偏移到gpio_keys_button[]位置，保存其结构位置*/
	pdata->buttons = (struct gpio_keys_button *)(pdata + 1);
    /*记录button数量*/
	pdata->nbuttons = nbuttons;

    /*涉及input subsystem */
	pdata->rep = !!of_get_property(node, "autorepeat", NULL);

    /*获取"label"属性内容，作为名字 */
	of_property_read_string(node, "label", &pdata->name);

	i = 0;
	for_each_available_child_of_node(node, pp) {
		enum of_gpio_flags flags;
		
        /*取出第i个，进行初始化*/
		button = &pdata->buttons[i++];

        /*获取"gpios"属性, 取出flag和gpio号*/
		button->gpio = of_get_gpio_flags(pp, 0, &flags);
		if (button->gpio < 0) {
			error = button->gpio;
			if (error != -ENOENT) {
				if (error != -EPROBE_DEFER)
					dev_err(dev,
						"Failed to get gpio flags, error: %d\n",
						error);
				return ERR_PTR(error);
			}
		} else {
            /*
             * 对flag进行与操作，当存在OF_GPIO_ACTIVE_LOW时，
             * active_low为真，则当物理低电平时，逻辑表示为高电平
            */
			button->active_low = flags & OF_GPIO_ACTIVE_LOW;
		}
        
	   /*解析和重映射当前子节点的中断号*/
		button->irq = irq_of_parse_and_map(pp, 0);

		if (!gpio_is_valid(button->gpio) && !button->irq) {
			dev_err(dev, "Found button without gpios or irqs\n");
			return ERR_PTR(-EINVAL);
		}

        /* input event code */
		if (of_property_read_u32(pp, "linux,code", &button->code)) {
			dev_err(dev, "Button without keycode: 0x%x\n",
				button->gpio);
			return ERR_PTR(-EINVAL);
		}
        
		/*获取"label"属性内容，作为名字 */
		button->desc = of_get_property(pp, "label", NULL);

		if (of_property_read_u32(pp, "linux,input-type", &button->type))
			button->type = EV_KEY;

		button->wakeup = of_property_read_bool(pp, "wakeup-source") ||
				 /* legacy name */
				 of_property_read_bool(pp, "gpio-key,wakeup");

		button->can_disable = !!of_get_property(pp, "linux,can-disable", NULL);

		if (of_property_read_u32(pp, "debounce-interval",
					 &button->debounce_interval))
			button->debounce_interval = 5;
	}

	if (pdata->nbuttons == 0)
		return ERR_PTR(-EINVAL);

	return pdata;
}