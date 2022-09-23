### 1 input_init() 

```c
static char *input_devnode(struct device *dev, umode_t *mode)
{
	return kasprintf(GFP_KERNEL, "input/%s", dev_name(dev));
}

struct class input_class = {
	.name		= "input",
	.devnode	= input_devnode,
};

static int __init input_init(void)
{
	int err;
	
    /*注册一个input class,放在/sys/class/*/
	err = class_register(&input_class);

	/*在proc创建 目录和文件*/
	err = input_proc_init();


    /*注册设备号, 主设备号：13 次设备号数量：1024*/
	err = register_chrdev_region(MKDEV(INPUT_MAJOR, 0),
				     INPUT_MAX_CHAR_DEVICES, "input");

	return 0;
}
```

### 1-1 input_proc_init()

```C
static int __init input_proc_init(void)
{
	struct proc_dir_entry *entry;

    /*使用procfs，创建 /proc/bus/input 子目录 */
	proc_bus_input_dir = proc_mkdir("bus/input", NULL);

    
    /*创建文件 /proc/bus/input/devices */
	entry = proc_create("devices", 0, proc_bus_input_dir,
			    &input_devices_fileops);

    
	/*创建文件 /proc/bus/input/handlers */
	entry = proc_create("handlers", 0, proc_bus_input_dir,
			    &input_handlers_fileops);


	return 0;

}
```

