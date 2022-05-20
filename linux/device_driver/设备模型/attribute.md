### 基本概念

- 内核空间和用户空间进行信息交互的一种方法。

- 例如某个driver定义了一个变量，却希望用户空间程序可以修改该变量，以控制driver的运行行为，那么就可以将该变量以sysfs attribute的形式开放出来

- 在不同的层，都有自己的属性文件，比如bus，kobject

- 分类

  - 普通attribute

    ```C
    /* include/linux/sysfs.h, line 26 */
    struct attribute {
       const char *name;
       umode_t mode;
    #ifdef CONFIG_DEBUG_LOCK_ALLOC
       bool ignore_lockdep:1;
       struct lock_class_key   *key;
       struct lock_class_key   skey;
    #endif
    };
    ```

    

  - 二进制attribute

    ```C
    /* include/linux/sysfs.h, line 100 */
    struct bin_attribute {
       struct attribute    attr;
       size_t          size;
       void *private;
       ssize_t (*read)(struct file *, struct kobject *, 
           struct bin_attribute *, char *, loff_t, size_t);
       ssize_t (*write)(struct file *,struct kobject *, 
           struct bin_attribute *, char *, loff_t, size_t);
       int (*mmap)(struct file *, struct kobject *, 
           struct bin_attribute *attr, struct vm_area_struct *vma);
    };
    ```

  - 所有的文件系统，都会定义一个struct file_operations变量，用于描述本文件系统的操作接口，sysfs也不例外

    ```C
    /* fs/sysfs/file.c, line 472 */
    const struct file_operations sysfs_file_operations = {
        .read       = sysfs_read_file,
        .write      = sysfs_write_file,
        .llseek     = generic_file_llseek,
        .open       = sysfs_open_file,
        .release    = sysfs_release,
        .poll       = sysfs_poll,
    };
    ```

    - 当执行对attribute文件read的操作时，会调用上述接口（sysfs_read_file）

### attribute_group
```C
/**
 * struct attribute_group - data structure used to declare an attribute group.
 * @name:	Optional: Attribute group name
 *		If specified, the attribute group will be created in
 *		a new subdirectory with this name.
 * @is_visible:	Optional: Function to return permissions associated with an
 *		attribute of the group. Will be called repeatedly for each
 *		non-binary attribute in the group. Only read/write
 *		permissions as well as SYSFS_PREALLOC are accepted. Must
 *		return 0 if an attribute is not visible. The returned value
 *		will replace static permissions defined in struct attribute.
 * @is_bin_visible:
 *		Optional: Function to return permissions associated with a
 *		binary attribute of the group. Will be called repeatedly
 *		for each binary attribute in the group. Only read/write
 *		permissions as well as SYSFS_PREALLOC are accepted. Must
 *		return 0 if a binary attribute is not visible. The returned
 *		value will replace static permissions defined in
 *		struct bin_attribute.
 * @attrs:	Pointer to NULL terminated list of attributes.
 * @bin_attrs:	Pointer to NULL terminated list of binary attributes.
 *		Either attrs or bin_attrs or both must be provided.
 */
struct attribute_group {
	const char		*name;
	umode_t			(*is_visible)(struct kobject *,
					      struct attribute *, int);
	umode_t			(*is_bin_visible)(struct kobject *,
						  struct bin_attribute *, int);
	struct attribute	**attrs;
	struct bin_attribute	**bin_attrs;
};
```



### 创建方法

1. 如device，利用device的struct attribute_group ** groups成员

   ```C
   static DEVICE_ATTR(power, 0644, show_power, store_power);
   
     static struct attribute *dev_attrs[] = {
   	&dev_attr_type.attr,
   	&dev_attr_power.attr,
   	NULL,
     };
   
     static struct attribute_group dev_attr_group = {
   	.attrs = dev_attrs,
     };
   
     static const struct attribute_group *dev_attr_groups[] = {
   	&dev_attr_group,
   	NULL,
     };
   
   
   dev->groups = dev_attr_groups; /*初始化*/
   device_register(dev); /*这样register的时候会自动添加*/
   ```

   

2. 手动调用device_create_file来添加attr

   ```C
   static DEVICE_ATTR(power, 0644, show_power, store_power);
   
   device_create_file(&dev, &dev_attr_power);
   ```

   
