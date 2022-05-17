### bus_type

```C
/**
 * struct bus_type - The bus type of the device
 *
 * @name:	The name of the bus.
 * @dev_name:	Used for subsystems to enumerate devices like ("foo%u", dev->id).
 * @dev_root:	Default device to use as the parent.
 * @bus_groups:	Default attributes of the bus.
 * @dev_groups:	Default attributes of the devices on the bus.
 * @drv_groups: Default attributes of the device drivers on the bus.
 * @match:	Called, perhaps multiple times, whenever a new device or driver
 *		is added for this bus. It should return a positive value if the
 *		given device can be handled by the given driver and zero
 *		otherwise. It may also return error code if determining that
 *		the driver supports the device is not possible. In case of
 *		-EPROBE_DEFER it will queue the device for deferred probing.
 * @uevent:	Called when a device is added, removed, or a few other things
 *		that generate uevents to add the environment variables.
 * @probe:	Called when a new device or driver add to this bus, and callback
 *		the specific driver's probe to initial the matched device.
 * @sync_state:	Called to sync device state to software state after all the
 *		state tracking consumers linked to this device (present at
 *		the time of late_initcall) have successfully bound to a
 *		driver. If the device has no consumers, this function will
 *		be called at late_initcall_sync level. If the device has
 *		consumers that are never bound to a driver, this function
 *		will never get called until they do.
 * @remove:	Called when a device removed from this bus.
 * @shutdown:	Called at shut-down time to quiesce the device.
 *
 * @online:	Called to put the device back online (after offlining it).
 * @offline:	Called to put the device offline for hot-removal. May fail.
 *
 * @suspend:	Called when a device on this bus wants to go to sleep mode.
 * @resume:	Called to bring a device on this bus out of sleep mode.
 * @num_vf:	Called to find out how many virtual functions a device on this
 *		bus supports.
 * @dma_configure:	Called to setup DMA configuration on a device on
 *			this bus.
 * @pm:		Power management operations of this bus, callback the specific
 *		device driver's pm-ops.
 * @iommu_ops:  IOMMU specific operations for this bus, used to attach IOMMU
 *              driver implementations to a bus and allow the driver to do
 *              bus-specific setup
 * @p:		The private data of the driver core, only the driver core can
 *		touch this.
 * @lock_key:	Lock class key for use by the lock validator
 * @need_parent_lock:	When probing or removing a device on this bus, the
 *			device core should lock the device's parent.
 *
 * A bus is a channel between the processor and one or more devices. For the
 * purposes of the device model, all devices are connected via a bus, even if
 * it is an internal, virtual, "platform" bus. Buses can plug into each other.
 * A USB controller is usually a PCI device, for example. The device model
 * represents the actual connections between buses and the devices they control.
 * A bus is represented by the bus_type structure. It contains the name, the
 * default attributes, the bus' methods, PM operations, and the driver core's
 * private data.
 */
struct bus_type {
	const char		*name;/*在sysfs的名称*/
	const char		*dev_name;/*如设备未命名，则根据规则 bus->dev_name+device ID 命名*/
	struct device		*dev_root; /*默认的父设备*/
	const struct attribute_group **bus_groups;  /*bus的attr_group*/
	const struct attribute_group **dev_groups;  /*device的attr_group*/
	const struct attribute_group **drv_groups;  /*driver的attr_group*/
    /*当bus有新设备或驱动加入时被调用，进行匹配，如name，apci，dts*/
	int (*match)(struct device *dev, struct device_driver *drv); 
	int (*uevent)(struct device *dev, struct kobj_uevent_env *env);
	int (*probe)(struct device *dev);
	void (*sync_state)(struct device *dev);
	int (*remove)(struct device *dev); /*当bus被移除的时被调用*/
	void (*shutdown)(struct device *dev);

	int (*online)(struct device *dev);
	int (*offline)(struct device *dev); 

	int (*suspend)(struct device *dev, pm_message_t state);/*挂起，进入睡眠时被调用*/
	int (*resume)(struct device *dev);/*退出睡眠时被调用*/

	int (*num_vf)(struct device *dev); /*虚拟设备数量*/

	int (*dma_configure)(struct device *dev);/*用于bus的dma配置*/

	const struct dev_pm_ops *pm;/*bus的电源管理的ops*/

	const struct iommu_ops *iommu_ops; 

	struct subsys_private *p; /*私有指针，只有驱动内核才能使用*/
	struct lock_class_key lock_key;

	bool need_parent_lock;
};
```

### subsys_private

```C
/**
 * struct subsys_private - structure to hold the private to the driver core portions of the bus_type/class structure.
 *
 * @subsys - the struct kset that defines this subsystem
 * @devices_kset - the subsystem's 'devices' directory
 * @interfaces - list of subsystem interfaces associated
 * @mutex - protect the devices, and interfaces lists.
 *
 * @drivers_kset - the list of drivers associated
 * @klist_devices - the klist to iterate over the @devices_kset
 * @klist_drivers - the klist to iterate over the @drivers_kset
 * @bus_notifier - the bus notifier list for anything that cares about things
 *                 on this bus.
 * @bus - pointer back to the struct bus_type that this structure is associated
 *        with.
 *
 * @glue_dirs - "glue" directory to put in-between the parent device to
 *              avoid namespace conflicts
 * @class - pointer back to the struct class that this structure is associated
 *          with.
 *
 * This structure is the one that is the actual kobject allowing struct
 * bus_type/class to be statically allocated safely.  Nothing outside of the
 * driver core should ever touch these fields.
 */
struct subsys_private {
	struct kset subsys;/*bus自己的节点*/
	struct kset *devices_kset;/*挂到此指针的设备kset*/
	struct list_head interfaces;
	struct mutex mutex;

	struct kset *drivers_kset;/*挂到此指针的驱动kset*/
	struct klist klist_devices;/*保存了本bus下所有的device指针*/
	struct klist klist_drivers;/*保存了本bus下所有的device_driver指针*/
	struct blocking_notifier_head bus_notifier;
	unsigned int drivers_autoprobe:1;
	struct bus_type *bus;

	struct kset glue_dirs; 
	struct class *class;/*指向被分配的class*/
};
```

### bus_attribute

```C
struct bus_attribute {
        struct attribute        attr;
        ssize_t (*show)(struct bus_type *bus, char *buf);
        ssize_t (*store)(struct bus_type *bus, const char *buf, size_t count);
};
```

### bus_register

```C
/**
 * bus_register - register a driver-core subsystem
 * @bus: bus to register
 *
 * Once we have that, we register the bus with the kobject
 * infrastructure, then register the children subsystems it has:
 * the devices and drivers that belong to the subsystem.
 */
int bus_register(struct bus_type *bus)
{
	int retval;
	struct subsys_private *priv;
	struct lock_class_key *key = &bus->lock_key;
    /*申请一个私有结构subsys_private的内存*/
	priv = kzalloc(sizeof(struct subsys_private), GFP_KERNEL); 
	if (!priv)
		return -ENOMEM;

	priv->bus = bus; /*指向回bus作记录*/
	bus->p = priv; /*记录创建的subsys_private结构*/
    /*操作了bus_notifier，不懂。。*/
	BLOCKING_INIT_NOTIFIER_HEAD(&priv->bus_notifier);
    /*对本bus的kobj命名*/
	retval = kobject_set_name(&priv->subsys.kobj, "%s", bus->name);
	if (retval)
		goto out;
    /*bus已写好的处理函数*/
	priv->subsys.kobj.kset = bus_kset;
	priv->subsys.kobj.ktype = &bus_ktype;
	priv->drivers_autoprobe = 1;
    /*将priv->subsys注册到内核中，向sysfs中添加对应的目录*/
	retval = kset_register(&priv->subsys);
	if (retval)
		goto out;
    /*创建一个bus_uevent的属性文件*/
	retval = bus_create_file(bus, &bus_attr_uevent);
	if (retval)
		goto bus_uevent_fail;
   /*创建一个devices的kset，add目录和属性文件*/
	priv->devices_kset = kset_create_and_add("devices", NULL,
						 &priv->subsys.kobj);
	if (!priv->devices_kset) {
		retval = -ENOMEM;
		goto bus_devices_fail;
	}
   /*创建一个drivers的kset，add目录和属性到sysfs*/
	priv->drivers_kset = kset_create_and_add("drivers", NULL,
						 &priv->subsys.kobj);
	if (!priv->drivers_kset) {
		retval = -ENOMEM;
		goto bus_drivers_fail;
	}

	INIT_LIST_HEAD(&priv->interfaces);
	__mutex_init(&priv->mutex, "subsys mutex", key);/*初始化互斥量*/
	klist_init(&priv->klist_devices, klist_devices_get, klist_devices_put);
	klist_init(&priv->klist_drivers, NULL, NULL); /*初始化klist*/
    /*
     * 在bus下添加drivers_probe和drivers_autoprobe属性文件
     * drivers_probe：允许用户空间主动出发指定bus下的device_driver的probe动作
     * drivers_autoprobe：是否在device或device_driver添加到内核时，自动执行probe*/
	retval = add_probe_files(bus);
	if (retval)
		goto bus_probe_files_fail;
    /*创建bus_gruop自身的属性文件*/
	retval = bus_add_groups(bus, bus->bus_groups);
	if (retval)
		goto bus_groups_fail;

	pr_debug("bus: '%s': registered\n", bus->name);
	return 0;

bus_groups_fail:
	remove_probe_files(bus);
bus_probe_files_fail:
	kset_unregister(bus->p->drivers_kset);
bus_drivers_fail:
	kset_unregister(bus->p->devices_kset);
bus_devices_fail:
	bus_remove_file(bus, &bus_attr_uevent);
bus_uevent_fail:
	kset_unregister(&bus->p->subsys);
out:
	kfree(bus->p);
	bus->p = NULL;
	return retval;
}
```

### bus_create_file

```C
int bus_create_file(struct bus_type *bus, struct bus_attribute *attr)
{
	int error;
	if (bus_get(bus)) { /*本质就是调用kobj_get*/
        /*根据提供的attr或attr_group创建属性文件*/
		error = sysfs_create_file(&bus->p->subsys.kobj, &attr->attr);
		bus_put(bus);
	} else
		error = -EINVAL;
	return error;
}
```

### bus_get

```C
static struct bus_type *bus_get(struct bus_type *bus)
{
        if (bus) {
                kset_get(&bus->p->subsys); /*bus的kobj，本质就是调用kobj_get*/
                return bus;
        }
        return NULL;
}
```

### kset_get

```C
static inline struct kset *kset_get(struct kset *k)
{
        return k ? to_kset(kobject_get(&k->kobj)) : NULL;
}
```

### to_kset

```C
static inline struct kset *to_kset(struct kobject *kobj)
{
         return kobj ? container_of(kobj, struct kset, kobj) : NULL;
}
```

### klist_init

```C
/**
 * klist_init - Initialize a klist structure.
 * @k: The klist we're initializing.
 * @get: The get function for the embedding object (NULL if none)
 * @put: The put function for the embedding object (NULL if none)
 *
 * Initialises the klist structure.  If the klist_node structures are
 * going to be embedded in refcounted objects (necessary for safe
 * deletion) then the get/put arguments are used to initialise
 * functions that take and release references on the embedding
 * objects.
 */
void klist_init(struct klist *k, void (*get)(struct klist_node *),
		void (*put)(struct klist_node *))
{
	INIT_LIST_HEAD(&k->k_list);
	spin_lock_init(&k->k_lock);
	k->get = get;
	k->put = put;
}
```

### add_probe_files

```C
static int add_probe_files(struct bus_type *bus)
{
	int retval;
    /*创建属性文件*/
	retval = bus_create_file(bus, &bus_attr_drivers_probe);
	if (retval)
		goto out;
    /*创建autoprobe属性文件*/
	retval = bus_create_file(bus, &bus_attr_drivers_autoprobe);
	if (retval)
		bus_remove_file(bus, &bus_attr_drivers_probe);
out:
	return retval;
}
```

```C
static int bus_add_groups(struct bus_type *bus,
			  const struct attribute_group **groups)
{
	return sysfs_create_groups(&bus->p->subsys.kobj, groups);
}
```

