### dts中的描述

```c
/* 
 * 如下内容描述此node，此内容风格为芯片厂商自定
 * 会包含不同厂商的ops来解析该node的内容
 */
pincontroller{
    
/* 
 * 复用：					  
 *   单pin：某个引脚的复用 
 *   group：某组引脚的复用  
 * 配置：
 */       
	state_0_node_a { 				  
		fsl,pins = <
			MX6UL_PAD_UART1_TX_DATA__UART1_DCE_TX 0x1b0b1
			MX6UL_PAD_UART1_RX_DATA__UART1_DCE_RX 0x1b0b1
		>;         
	};                
    state_1_node_a {  
		...          
	};
	state_1_node_b {
		...
	};    
};
```

### pincontroller作用

- 引脚的枚举与命名：描述使用了哪些pin，及pin的组，状态等信息
- 引脚复用（multiplexing）：如gpio复用为iic，uart等功能
- 引脚配置（configuration）：如上下拉，开漏，驱动能力，阻值等

配置一个pinctrl_desc，并且使用pinctrl_register，与pinctrl_dev建立关系。

### pinctrl_desc

```C
/**
 * struct pinctrl_desc - pin controller descriptor, register this to pin
 * control subsystem
 * 用来描述，复用，配置pinctrl，并将其注册到pinctrl子系统
 * @name: name for the pin controller
 * @pins: an array of pin descriptors describing all the pins handled by
 *	this pin controller
 * @npins: number of descriptors in the array, usually just ARRAY_SIZE()
 *	of the pins field above
 * @pctlops: pin control operation vtable, to support global concepts like
 *	grouping of pins, this is optional.
 * @pmxops: pinmux operations vtable, if you support pinmuxing in your driver
 * @confops: pin config operations vtable, if you support pin configuration in
 *	your driver
 * @owner: module providing the pin controller, used for refcounting
 * @num_custom_params: Number of driver-specific custom parameters to be parsed
 *	from the hardware description
 * @custom_params: List of driver_specific custom parameters to be parsed from
 *	the hardware description
 * @custom_conf_items: Information how to print @params in debugfs, must be
 *	the same size as the @custom_params, i.e. @num_custom_params
 * @link_consumers: If true create a device link between pinctrl and its
 *	consumers (i.e. the devices requesting pin control states). This is
 *	sometimes necessary to ascertain the right suspend/resume order for
 *	example.
 */
struct pinctrl_desc {
	const char *name;
    
    /*指向一个数组，记录引脚的枚举号与名字*/
	const struct pinctrl_pin_desc *pins;
    
    /*描述该数组的大小*/
	unsigned int npins;
    
    /*要先提取info，再解析设备树后，调用这些ops就可以根据state配置引脚*/
	const struct pinctrl_ops *pctlops;
	const struct pinmux_ops *pmxops;
	const struct pinconf_ops *confops;
    
	struct module *owner;
#ifdef CONFIG_GENERIC_PINCONF
	unsigned int num_custom_params;
	const struct pinconf_generic_params *custom_params;
	const struct pin_config_item *custom_conf_items;
#endif
	bool link_consumers;
};
```

### pinctrl_pin_desc

```C
/**
 * struct pinctrl_pin_desc - boards/machines provide information on their
 * pins, pads or other muxable units in this struct
 * @number: unique pin number from the global pin number space
 * @name: a name for this pin
 * @drv_data: driver-defined per-pin data. pinctrl core does not touch this
 */
struct pinctrl_pin_desc {
	unsigned number; 
	const char *name;
	void *drv_data;
};
```

### pinctrl_ops

```C
/**
 * struct pinctrl_ops - global pin control operations, to be implemented by
 * pin controller drivers.
 * @get_groups_count: Returns the count of total number of groups registered.
 * @get_group_name: return the group name of the pin group
 * @get_group_pins: return an array of pins corresponding to a certain
 *	group selector @pins, and the size of the array in @num_pins
 * @pin_dbg_show: optional debugfs display hook that will provide per-device
 *	info for a certain pin in debugfs
 * @dt_node_to_map: parse a device tree "pin configuration node", and create
 *	mapping table entries for it. These are returned through the @map and
 *	@num_maps output parameters. This function is optional, and may be
 *	omitted for pinctrl drivers that do not support device tree.
 * @dt_free_map: free mapping table entries created via @dt_node_to_map. The
 *	top-level @map pointer must be freed, along with any dynamically
 *	allocated members of the mapping table entries themselves. This
 *	function is optional, and may be omitted for pinctrl drivers that do
 *	not support device tree.
 */
struct pinctrl_ops {
    /*获取pin group的数量*/
	int (*get_groups_count) (struct pinctrl_dev *pctldev);
    
    /*获取某个group的name*/
	const char *(*get_group_name) (struct pinctrl_dev *pctldev,
				       unsigned selector);
    
    /*获取某个group的pin的描述和pin数量*/
	int (*get_group_pins) (struct pinctrl_dev *pctldev,
			       unsigned selector,
			       const unsigned **pins,
			       unsigned *num_pins);
	void (*pin_dbg_show) (struct pinctrl_dev *pctldev, struct seq_file *s,
			  unsigned offset);
    
    /*
     * 解析dt的"pin configuration node"
     * 转换成pinctrl_map table
	 */
	int (*dt_node_to_map) (struct pinctrl_dev *pctldev, 
			       struct device_node *np_config,
			       struct pinctrl_map **map, unsigned *num_maps);
	void (*dt_free_map) (struct pinctrl_dev *pctldev,
			     struct pinctrl_map *map, unsigned num_maps);
};
```

### pinmux_ops

```C
/**
 * struct pinmux_ops - pinmux operations, to be implemented by pin controller
 * drivers that support pinmuxing
 * @request: called by the core to see if a certain pin can be made
 *	available for muxing. This is called by the core to acquire the pins
 *	before selecting any actual mux setting across a function. The driver
 *	is allowed to answer "no" by returning a negative error code
 * @free: the reverse function of the request() callback, frees a pin after
 *	being requested
 * @get_functions_count: returns number of selectable named functions available
 *	in this pinmux driver
 * @get_function_name: return the function name of the muxing selector,
 *	called by the core to figure out which mux setting it shall map a
 *	certain device to
 * @get_function_groups: return an array of groups names (in turn
 *	referencing pins) connected to a certain function selector. The group
 *	name can be used with the generic @pinctrl_ops to retrieve the
 *	actual pins affected. The applicable groups will be returned in
 *	@groups and the number of groups in @num_groups
 * @set_mux: enable a certain muxing function with a certain pin group. The
 *	driver does not need to figure out whether enabling this function
 *	conflicts some other use of the pins in that group, such collisions
 *	are handled by the pinmux subsystem. The @func_selector selects a
 *	certain function whereas @group_selector selects a certain set of pins
 *	to be used. On simple controllers the latter argument may be ignored
 * @gpio_request_enable: requests and enables GPIO on a certain pin.
 *	Implement this only if you can mux every pin individually as GPIO. The
 *	affected GPIO range is passed along with an offset(pin number) into that
 *	specific GPIO range - function selectors and pin groups are orthogonal
 *	to this, the core will however make sure the pins do not collide.
 * @gpio_disable_free: free up GPIO muxing on a certain pin, the reverse of
 *	@gpio_request_enable
 * @gpio_set_direction: Since controllers may need different configurations
 *	depending on whether the GPIO is configured as input or output,
 *	a direction selector function may be implemented as a backing
 *	to the GPIO controllers that need pin muxing.
 * @strict: do not allow simultaneous use of the same pin for GPIO and another
 *	function. Check both gpio_owner and mux_owner strictly before approving
 *	the pin request.
 */
struct pinmux_ops {
	int (*request) (struct pinctrl_dev *pctldev, unsigned offset);
	int (*free) (struct pinctrl_dev *pctldev, unsigned offset);
    
    /*获取function的数量，即"default" "idle"这种function的总数量*/
	int (*get_functions_count) (struct pinctrl_dev *pctldev); 
    
    /*获取function的name*/
	const char *(*get_function_name) (struct pinctrl_dev *pctldev,
					  unsigned selector);
    
    /*获取对应function的group 即 pinctrl-0 = <&state_0_node_a>; 中的state_0_node_a*/
	int (*get_function_groups) (struct pinctrl_dev *pctldev,
				  unsigned selector,
				  const char * const **groups,
				  unsigned *num_groups);
    
    /*配置要选择的function的复用*/
	int (*set_mux) (struct pinctrl_dev *pctldev, unsigned func_selector,
			unsigned group_selector);
    
	int (*gpio_request_enable) (struct pinctrl_dev *pctldev,
				    struct pinctrl_gpio_range *range,
				    unsigned offset);
	void (*gpio_disable_free) (struct pinctrl_dev *pctldev,
				   struct pinctrl_gpio_range *range,
				   unsigned offset);
	int (*gpio_set_direction) (struct pinctrl_dev *pctldev,
				   struct pinctrl_gpio_range *range,
				   unsigned offset,
				   bool input);
	bool strict;
};
```

### pinconf_ops

```C
/**
 * struct pinconf_ops - pin config operations, to be implemented by
 * pin configuration capable drivers.
 * @is_generic: for pin controllers that want to use the generic interface,
 *	this flag tells the framework that it's generic.
 * @pin_config_get: get the config of a certain pin, if the requested config
 *	is not available on this controller this should return -ENOTSUPP
 *	and if it is available but disabled it should return -EINVAL
 * @pin_config_set: configure an individual pin
 * @pin_config_group_get: get configurations for an entire pin group; should
 *	return -ENOTSUPP and -EINVAL using the same rules as pin_config_get.
 * @pin_config_group_set: configure all pins in a group
 * @pin_config_dbg_show: optional debugfs display hook that will provide
 *	per-device info for a certain pin in debugfs
 * @pin_config_group_dbg_show: optional debugfs display hook that will provide
 *	per-device info for a certain group in debugfs
 * @pin_config_config_dbg_show: optional debugfs display hook that will decode
 *	and display a driver's pin configuration parameter
 */
struct pinconf_ops {
#ifdef CONFIG_GENERIC_PINCONF
	bool is_generic;
#endif
    
    /*获取该pin的配置信息*/
	int (*pin_config_get) (struct pinctrl_dev *pctldev,
			       unsigned pin,
			       unsigned long *config);
    
    /*config该pin*/
	int (*pin_config_set) (struct pinctrl_dev *pctldev,
			       unsigned pin,
			       unsigned long *configs,
			       unsigned num_configs);
    
    /*get configurations for an entire pin group*/
	int (*pin_config_group_get) (struct pinctrl_dev *pctldev,
				     unsigned selector,
				     unsigned long *config);
    
    /*configure all pins in a group*/
	int (*pin_config_group_set) (struct pinctrl_dev *pctldev,
				     unsigned selector,
				     unsigned long *configs,
				     unsigned num_configs);
    
	void (*pin_config_dbg_show) (struct pinctrl_dev *pctldev,
				     struct seq_file *s,
				     unsigned offset);
	void (*pin_config_group_dbg_show) (struct pinctrl_dev *pctldev,
					   struct seq_file *s,
					   unsigned selector);
	void (*pin_config_config_dbg_show) (struct pinctrl_dev *pctldev,
					    struct seq_file *s,
					    unsigned long config);
};
```

### pinctrl_map

```C
enum pinctrl_map_type {
	PIN_MAP_TYPE_INVALID,
	PIN_MAP_TYPE_DUMMY_STATE,
	PIN_MAP_TYPE_MUX_GROUP,
	PIN_MAP_TYPE_CONFIGS_PIN,
	PIN_MAP_TYPE_CONFIGS_GROUP,
};

/**
 * struct pinctrl_map - boards/machines shall provide this map for devices
 * @dev_name: the name of the device using this specific mapping, the name
 *	must be the same as in your struct device*. If this name is set to the
 *	same name as the pin controllers own dev_name(), the map entry will be
 *	hogged by the driver itself upon registration
 * @name: the name of this specific map entry for the particular machine.
 *	This is the parameter passed to pinmux_lookup_state()
 * @type: the type of mapping table entry
 * @ctrl_dev_name: the name of the device controlling this specific mapping,
 *	the name must be the same as in your struct device*. This field is not
 *	used for PIN_MAP_TYPE_DUMMY_STATE
 * @data: Data specific to the mapping type
 */
struct pinctrl_map {
	const char *dev_name;
	const char *name;
	enum pinctrl_map_type type;
	const char *ctrl_dev_name;
	union {
		struct pinctrl_map_mux mux;
		struct pinctrl_map_configs configs;
	} data;
};
```

### pinctrl_map_mux

```C
/**
 * struct pinctrl_map_mux - mapping table content for MAP_TYPE_MUX_GROUP
 * @group: the name of the group whose mux function is to be configured. This
 *	field may be left NULL, and the first applicable group for the function
 *	will be used.
 * @function: the mux function to select for the group
 */
struct pinctrl_map_mux {  /*group这组pin，需要配置的mux*/
	const char *group;/*字符串名字标识，表示某group*/
	const char *function; /*同上*/
};
```

### pinctrl_map_configs

```C
/**
 * struct pinctrl_map_configs - mapping table content for MAP_TYPE_CONFIGS_*
 * @group_or_pin: the name of the pin or group whose configuration parameters
 *	are to be configured.
 * @configs: a pointer to an array of config parameters/values to program into
 *	hardware. Each individual pin controller defines the format and meaning
 *	of config parameters.
 * @num_configs: the number of entries in array @configs
 */
struct pinctrl_map_configs {
	const char *group_or_pin;/*某组pin或某个pin，字符串名字标识*/
	unsigned long *configs; /*指向一个数组，包含配置的值*/
	unsigned num_configs;
};
```

- function：iomuxc下的子节点都是为function，但是在imx6ull中，“imx6ul_evk“节点为function（其实理解为该单个board使用到的外设group节点），“imx6ul_evk“节点下有很多个外设group节点，每个group下有很多个pin
