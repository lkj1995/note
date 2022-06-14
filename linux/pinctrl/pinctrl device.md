### dts中的描述

```C
/*该node描述在device的pins成员中*/
device { 
    
    /*该设备的可切换的状态*/
	pinctrl-names = "default", "sleep"; 
    
    /*某个pin*/
	pinctrl-0 = <&state_0_node_a>; 
	pinctrl-1 = <&state_1_node_a &state_1_node_b>;
}
```

### 转换过程

```
dt的<&state_0_node_a> -> pinctrl_map -> pinctrl_setting

将 pinctrl_setting 存入 pinctrl_state.setting
```

### device

```c
struct device {
 ...
#ifdef CONFIG_PINCTRL
	struct dev_pin_info	*pins; /*在device中，使用pins成员，描述pin信息*/
#endif
 ...
};
```

### dev_pin_info

```C
/**
 * struct dev_pin_info - pin state container for devices
 * @p: pinctrl handle for the containing device
 * @default_state: the default state for the handle, if found
 * @init_state: the state at probe time, if found
 * @sleep_state: the state at suspend time, if found
 * @idle_state: the state at idle (runtime suspend) time, if found
 */
struct dev_pin_info {
	struct pinctrl *p;/*自定义state会添加到p*/
    
    /*dts中描述的 pinctrl-name = "default";*/
	struct pinctrl_state *default_state;
    
	struct pinctrl_state *init_state;
#ifdef CONFIG_PM
     /*同上，"sleep"*/
	struct pinctrl_state *sleep_state;
     /*同上，"idle"*/
	struct pinctrl_state *idle_state;
#endif
};
```

### pinctrl

```C
/**
 * struct pinctrl - per-device pin control state holder
 * @node: global list node
 * @dev: the device using this pin control handle
 * @states: a list of states for this device
 * @state: the current state
 * @dt_maps: the mapping table chunks dynamically parsed from device tree for
 *	this device, if any
 * @users: reference count
 */
struct pinctrl { 
	struct list_head node;
	struct device *dev;
    
    /*"init","default"等状态和自定义状态，都会存放到该链表*/
	struct list_head states;
    
	struct pinctrl_state *state;
	struct list_head dt_maps;    /*关联 pinctrl_map结构*/
	struct kref users;
};
```

### pinctrl_state

```C
/**
 * struct pinctrl_state - a pinctrl state for a device
 * @node: list node for struct pinctrl's @states field
 * @name: the name of this state
 * @settings: a list of settings for this state
 */
struct pinctrl_state {
	struct list_head node;
	const char *name;
	struct list_head settings; 
};

```

### pinctrl_setting

```C
/**
 * struct pinctrl_setting - an individual mux or config setting
 * @node: list node for struct pinctrl_settings's @settings field
 * @type: the type of setting
 * @pctldev: pin control device handling to be programmed. Not used for
 *   PIN_MAP_TYPE_DUMMY_STATE.
 * @dev_name: the name of the device using this state
 * @data: Data specific to the setting type
 */
struct pinctrl_setting {
	struct list_head node;
	enum pinctrl_map_type type;
	struct pinctrl_dev *pctldev;
	const char *dev_name;
	union {
		struct pinctrl_setting_mux mux;
		struct pinctrl_setting_configs configs;
	} data;
};
```

