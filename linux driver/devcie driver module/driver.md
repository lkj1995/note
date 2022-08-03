### device_driver

```c
/**
 * struct device_driver - The basic device driver structure
 * @name:	Name of the device driver.
 * @bus:	The bus which the device of this driver belongs to.
 * @owner:	The module owner.
 * @mod_name:	Used for built-in modules.
 * @suppress_bind_attrs: Disables bind/unbind via sysfs.
 * @probe_type:	Type of the probe (synchronous or asynchronous) to use.
 * @of_match_table: The open firmware table.
 * @acpi_match_table: The ACPI match table.
 * @probe:	Called to query the existence of a specific device,
 *		whether this driver can work with it, and bind the driver
 *		to a specific device.
 * @sync_state:	Called to sync device state to software state after all the
 *		state tracking consumers linked to this device (present at
 *		the time of late_initcall) have successfully bound to a
 *		driver. If the device has no consumers, this function will
 *		be called at late_initcall_sync level. If the device has
 *		consumers that are never bound to a driver, this function
 *		will never get called until they do.
 * @remove:	Called when the device is removed from the system to
 *		unbind a device from this driver.
 * @shutdown:	Called at shut-down time to quiesce the device.
 * @suspend:	Called to put the device to sleep mode. Usually to a
 *		low power state.
 * @resume:	Called to bring a device from sleep mode.
 * @groups:	Default attributes that get created by the driver core
 *		automatically.
 * @dev_groups:	Additional attributes attached to device instance once the
 *		it is bound to the driver.
 * @pm:		Power management operations of the device which matched
 *		this driver.
 * @coredump:	Called when sysfs entry is written to. The device driver
 *		is expected to call the dev_coredump API resulting in a
 *		uevent.
 * @p:		Driver core's private data, no one other than the driver
 *		core can touch this.
 *
 * The device driver-model tracks all of the drivers known to the system.
 * The main reason for this tracking is to enable the driver core to match
 * up drivers with new devices. Once drivers are known objects within the
 * system, however, a number of other things become possible. Device drivers
 * can export information and configuration variables that are independent
 * of any specific device.
 */
struct device_driver {
	const char		*name;
	struct bus_type		*bus;

	struct module		*owner;
	const char		*mod_name;	/* used for built-in modules */

	bool suppress_bind_attrs;	/* disables bind/unbind via sysfs */
	enum probe_type probe_type;

	const struct of_device_id	*of_match_table;
	const struct acpi_device_id	*acpi_match_table;

	int (*probe) (struct device *dev);
	void (*sync_state)(struct device *dev);
	int (*remove) (struct device *dev);
	void (*shutdown) (struct device *dev);
	int (*suspend) (struct device *dev, pm_message_t state);
	int (*resume) (struct device *dev);
	const struct attribute_group **groups;
	const struct attribute_group **dev_groups;

	const struct dev_pm_ops *pm;
	void (*coredump) (struct device *dev);

	struct driver_private *p;
};
```

### driver_private

```C
struct driver_private {
	struct kobject kobj;
	struct klist klist_devices;
	struct klist_node knode_bus;
	struct module_kobject *mkobj;
	struct device_driver *driver;
};
```

### 1 driver_register

```c
/**
 * driver_register - register driver with bus
 * @drv: driver to register
 *
 * We pass off most of the work to the bus_add_driver() call,
 * since most of the things we have to do deal with the bus
 * structures.
 */
int driver_register(struct device_driver *drv)
{
	int ret;
	struct device_driver *other;
    
     /*bus的p不能为空，因需要bus->p->subsys->kobj作为父kobj*/
	if (!drv->bus->p) {
		pr_err("Driver '%s' was unable to register with bus_type '%s' because the bus was not initialized.\n",
			   drv->name, drv->bus->name);
		return -EINVAL;
	}
    
    /*是不是意味着只能二选一?*/
	if ((drv->bus->probe && drv->probe) ||
	    (drv->bus->remove && drv->remove) ||
	    (drv->bus->shutdown && drv->shutdown))
		pr_warn("Driver '%s' needs updating - please use "
			"bus_type methods\n", drv->name);

    /*查询bus中是否存在同名的driver*/
	other = driver_find(drv->name, drv->bus);
	if (other) { /*存在同名，已被注册*/
		pr_err("Error: Driver '%s' is already registered, "
			"aborting...\n", drv->name);
		return -EBUSY;
	}

	ret = bus_add_driver(drv);
	if (ret)
		return ret;
    /*创建driver的groups成员多组属性文件*/
	ret = driver_add_groups(drv, drv->groups);
	if (ret) {
		bus_remove_driver(drv);
		return ret;
	}
	kobject_uevent(&drv->p->kobj, KOBJ_ADD); /*通知用户层*/

	return ret;
}
```

### 1-1 driver_find

```C
/**
 * driver_find - locate driver on a bus by its name.
 * @name: name of the driver.
 * @bus: bus to scan for the driver.
 *
 * Call kset_find_obj() to iterate over list of drivers on
 * a bus to find driver by name. Return driver if found.
 *
 * This routine provides no locking to prevent the driver it returns
 * from being unregistered or unloaded while the caller is using it.
 * The caller is responsible for preventing this.
 */
struct device_driver *driver_find(const char *name, struct bus_type *bus)
{   
    /*在kset的list中寻找同名name的kobj，如找到则返回其地址*/
	struct kobject *k = kset_find_obj(bus->p->drivers_kset, name);
	struct driver_private *priv;

	if (k) {
		/* Drop reference added by kset_find_obj() */
		kobject_put(k);
		priv = to_driver(k); /*根据kobj获取driver_private*/
		return priv->driver; /*获取其driver*/
	}
	return NULL;
}
```

### 1-2 bus_add_driver

```C
/**
 * bus_add_driver - Add a driver to the bus.
 * @drv: driver.
 */
int bus_add_driver(struct device_driver *drv)
{
	struct bus_type *bus;
	struct driver_private *priv;
	int error = 0;

    /*获取需挂载的bus*/
	bus = bus_get(drv->bus); 
	if (!bus)
		return -EINVAL;

	pr_debug("bus: '%s': add driver %s\n", bus->name, drv->name);

    /*申请driver_private结构*/
	priv = kzalloc(sizeof(*priv), GFP_KERNEL);
	if (!priv) {
		error = -ENOMEM;
		goto out_put_bus;
	}
    /*记录及初始化*/
	klist_init(&priv->klist_devices, NULL, NULL);
	priv->driver = drv;
	drv->p = priv;
	priv->kobj.kset = bus->p->drivers_kset; /*父kset*/ 
    
    /*创建节点，ktype为通用driver_ktype*/
    error = kobject_init_and_add(&priv->kobj, &driver_ktype, NULL,
				     "%s", drv->name); 
	if (error)
		goto out_unregister;
    /*把driver挂载到bus_klist_drivers下*/
	klist_add_tail(&priv->knode_bus, &bus->p->klist_drivers);
	if (drv->bus->p->drivers_autoprobe) {
		error = driver_attach(drv);
		if (error)
			goto out_unregister;
	}
	module_add_driver(drv->owner, drv);
 
    /*创建driver_uevent属性文件*/ 
	error = driver_create_file(drv, &driver_attr_uevent);
	if (error) {
		printk(KERN_ERR "%s: uevent attr (%s) failed\n",
			__func__, drv->name);
	}
    
    /*创建driver的drv_group成员多组属性文件*/
	error = driver_add_groups(drv, bus->drv_groups);
	if (error) {
		/* How the hell do we get out of this pickle? Give up */
		printk(KERN_ERR "%s: driver_create_groups(%s) failed\n",
			__func__, drv->name);
	}

	if (!drv->suppress_bind_attrs) {
        /*创建bind和ubind属性文件*/
		error = add_bind_files(drv);
		if (error) {
			/* Ditto */
			printk(KERN_ERR "%s: add_bind_files(%s) failed\n",
				__func__, drv->name);
		}
	}

	return 0;

out_unregister:
	kobject_put(&priv->kobj);
	/* drv->p is freed in driver_release()  */
	drv->p = NULL;
out_put_bus:
	bus_put(bus);
	return error;
}
```

### driver_attach

```C
/**
 * driver_attach - try to bind driver to devices.
 * 尝试绑定设备和驱动
 * @drv: driver.
 *
 * Walk the list of devices that the bus has on it and try to
 * match the driver with each one.  If driver_probe_device()
 * returns 0 and the @dev->driver is set, we've found a
 * compatible pair.
 */
int driver_attach(struct device_driver *drv)
{
	return bus_for_each_dev(drv->bus, NULL, drv, __driver_attach);
}
```

### bus_for_each_dev

```C
/**
 * bus_for_each_dev - device iterator.
 * @bus: bus type.
 * @start: device to start iterating from.
 * @data: data for the callback.
 * @fn: function to be called for each device.
 *
 * Iterate over @bus's list of devices, and call @fn for each,
 * passing it @data. If @start is not NULL, we use that device to
 * begin iterating from.
 *
 * We check the return of @fn each time. If it returns anything
 * other than 0, we break out and return that value.
 *
 * NOTE: The device that returns a non-zero value is not retained
 * in any way, nor is its refcount incremented. If the caller needs
 * to retain this data, it should do so, and increment the reference
 * count in the supplied callback.
 */
int bus_for_each_dev(struct bus_type *bus, struct device *start,
		     void *data, int (*fn)(struct device *, void *))
{
	struct klist_iter i;
	struct device *dev;
	int error = 0;

	if (!bus || !bus->p)
		return -EINVAL;

	klist_iter_init_node(&bus->p->klist_devices, &i,
			     (start ? &start->p->knode_bus : NULL));
	while (!error && (dev = next_device(&i)))
		error = fn(dev, data);
	klist_iter_exit(&i);
	return error;
}
```

### __driver_attach

```C
static int __driver_attach(struct device *dev, void *data)
{
	struct device_driver *drv = data;
	int ret;

	/*
	 * Lock device and try to bind to it. We drop the error
	 * here and always return 0, because we need to keep trying
	 * to bind to devices and some drivers will return an error
	 * simply if it didn't support the device.
	 *
	 * driver_probe_device() will spit a warning if there
	 * is an error.
	 */

	ret = driver_match_device(drv, dev);
	if (ret == 0) { /*匹配失败*/
		/* no match */
		return 0;
	} else if (ret == -EPROBE_DEFER) {
		dev_dbg(dev, "Device match requests probe deferral\n");
		driver_deferred_probe_add(dev);
	} else if (ret < 0) {
		dev_dbg(dev, "Bus failed to match device: %d\n", ret);
		return ret;
	} /* ret > 0 means positive match */

	if (driver_allows_async_probing(drv)) {
		/*
		 * Instead of probing the device synchronously we will
		 * probe it asynchronously to allow for more parallelism.
		 *
		 * We only take the device lock here in order to guarantee
		 * that the dev->driver and async_driver fields are protected
		 */
		dev_dbg(dev, "probing driver %s asynchronously\n", drv->name);
		device_lock(dev);
		if (!dev->driver) {
			get_device(dev);
			dev->p->async_driver = drv;
			async_schedule_dev(__driver_attach_async_helper, dev);
		}
		device_unlock(dev);
		return 0;
	}

	device_driver_attach(drv, dev);/*调用driver->probe进行绑定*/

	return 0;
}
```

### driver_match_device

```C
static inline int driver_match_device(struct device_driver *drv,
                                      struct device *dev)
{
        /*执行match进行匹配*/
        return drv->bus->match ? drv->bus->match(dev, drv) : 1;
}
```

### device_driver_attach

```C
/**
 * device_driver_attach - attach a specific driver to a specific device
 * driver和device绑定操作
 * @drv: Driver to attach
 * @dev: Device to attach it to
 *
 * Manually attach driver to a device. Will acquire both @dev lock and
 * @dev->parent lock if needed.
 */
int device_driver_attach(struct device_driver *drv, struct device *dev)
{
	int ret = 0;

	__device_driver_lock(dev, dev->parent);

	/*
	 * If device has been removed or someone has already successfully
	 * bound a driver before us just skip the driver probe call.
	 */
	if (!dev->p->dead && !dev->driver)/*device被移除或已经绑定过了，就不进行绑定操作*/
		ret = driver_probe_device(drv, dev);

	__device_driver_unlock(dev, dev->parent);

	return ret;
}
```

### driver_probe_device

```C

/**
 * driver_probe_device - attempt to bind device & driver together
 * @drv: driver to bind a device to
 * @dev: device to try to bind to the driver
 *
 * This function returns -ENODEV if the device is not registered,
 * 1 if the device is bound successfully and 0 otherwise.
 *
 * This function must be called with @dev lock held.  When called for a
 * USB interface, @dev->parent lock must be held as well.
 *
 * If the device has a parent, runtime-resume the parent before driver probing.
 */
int driver_probe_device(struct device_driver *drv, struct device *dev)
{
	int ret = 0;

	if (!device_is_registered(dev))/*检查是否device已注册*/
		return -ENODEV;

	pr_debug("bus: '%s': %s: matched device %s with driver %s\n",
		 drv->bus->name, __func__, dev_name(dev), drv->name);

	pm_runtime_get_suppliers(dev);
	if (dev->parent)
		pm_runtime_get_sync(dev->parent);

	pm_runtime_barrier(dev);
	if (initcall_debug)
		ret = really_probe_debug(dev, drv);
	else
		ret = really_probe(dev, drv); /*绑定操作，执行probe函数*/
	pm_request_idle(dev);

	if (dev->parent)
		pm_runtime_put(dev->parent);

	pm_runtime_put_suppliers(dev);
	return ret;
}
```

### really_probe

```C
static int really_probe(struct device *dev, struct device_driver *drv)
{
	int ret = -EPROBE_DEFER;
	int local_trigger_count = atomic_read(&deferred_trigger_count);
	bool test_remove = IS_ENABLED(CONFIG_DEBUG_TEST_DRIVER_REMOVE) &&
			   !drv->suppress_bind_attrs;

	if (defer_all_probes) {
		/*
		 * Value of defer_all_probes can be set only by
		 * device_block_probing() which, in turn, will call
		 * wait_for_device_probe() right after that to avoid any races.
		 */
		dev_dbg(dev, "Driver %s force probe deferral\n", drv->name);
		driver_deferred_probe_add(dev);
		return ret;
	}

	ret = device_links_check_suppliers(dev);
	if (ret == -EPROBE_DEFER)
		driver_deferred_probe_add_trigger(dev, local_trigger_count);
	if (ret)
		return ret;

	atomic_inc(&probe_count);
	pr_debug("bus: '%s': %s: probing driver %s with device %s\n",
		 drv->bus->name, __func__, drv->name, dev_name(dev));
	if (!list_empty(&dev->devres_head)) {
		dev_crit(dev, "Resources present before probing\n");
		ret = -EBUSY;
		goto done;
	}

re_probe:
	dev->driver = drv;

	/* If using pinctrl, bind pins now before probing */
	ret = pinctrl_bind_pins(dev);
	if (ret)
		goto pinctrl_bind_failed;

	if (dev->bus->dma_configure) {
		ret = dev->bus->dma_configure(dev);
		if (ret)
			goto probe_failed;
	}

	if (driver_sysfs_add(dev)) {
		pr_err("%s: driver_sysfs_add(%s) failed\n",
		       __func__, dev_name(dev));
		goto probe_failed;
	}

	if (dev->pm_domain && dev->pm_domain->activate) {
		ret = dev->pm_domain->activate(dev);
		if (ret)
			goto probe_failed;
	}

	if (dev->bus->probe) {/*二选一，优先bus_probe*/
		ret = dev->bus->probe(dev);
		if (ret)
			goto probe_failed;
	} else if (drv->probe) {
		ret = drv->probe(dev); /*platform_drv_probe*/
		if (ret)
			goto probe_failed;
	}

	if (device_add_groups(dev, drv->dev_groups)) {
		dev_err(dev, "device_add_groups() failed\n");
		goto dev_groups_failed;
	}

	if (dev_has_sync_state(dev) &&
	    device_create_file(dev, &dev_attr_state_synced)) {
		dev_err(dev, "state_synced sysfs add failed\n");
		goto dev_sysfs_state_synced_failed;
	}

	if (test_remove) {
		test_remove = false;

		device_remove_file(dev, &dev_attr_state_synced);
		device_remove_groups(dev, drv->dev_groups);

		if (dev->bus->remove)
			dev->bus->remove(dev);
		else if (drv->remove)
			drv->remove(dev);

		devres_release_all(dev);
		driver_sysfs_remove(dev);
		dev->driver = NULL;
		dev_set_drvdata(dev, NULL);
		if (dev->pm_domain && dev->pm_domain->dismiss)
			dev->pm_domain->dismiss(dev);
		pm_runtime_reinit(dev);

		goto re_probe;
	}

	pinctrl_init_done(dev);

	if (dev->pm_domain && dev->pm_domain->sync)
		dev->pm_domain->sync(dev);

	driver_bound(dev);
	ret = 1;
	pr_debug("bus: '%s': %s: bound device %s to driver %s\n",
		 drv->bus->name, __func__, dev_name(dev), drv->name);
	goto done;

dev_sysfs_state_synced_failed:
	device_remove_groups(dev, drv->dev_groups);
dev_groups_failed:
	if (dev->bus->remove)
		dev->bus->remove(dev);
	else if (drv->remove)
		drv->remove(dev);
probe_failed:
	if (dev->bus)
		blocking_notifier_call_chain(&dev->bus->p->bus_notifier,
					     BUS_NOTIFY_DRIVER_NOT_BOUND, dev);
pinctrl_bind_failed:
	device_links_no_driver(dev);
	devres_release_all(dev);
	arch_teardown_dma_ops(dev);
	driver_sysfs_remove(dev);
	dev->driver = NULL;
	dev_set_drvdata(dev, NULL);
	if (dev->pm_domain && dev->pm_domain->dismiss)
		dev->pm_domain->dismiss(dev);
	pm_runtime_reinit(dev);
	dev_pm_set_driver_flags(dev, 0);

	switch (ret) {
	case -EPROBE_DEFER:
		/* Driver requested deferred probing */
		dev_dbg(dev, "Driver %s requests probe deferral\n", drv->name);
		driver_deferred_probe_add_trigger(dev, local_trigger_count);
		break;
	case -ENODEV:
	case -ENXIO:
		pr_debug("%s: probe of %s rejects match %d\n",
			 drv->name, dev_name(dev), ret);
		break;
	default:
		/* driver matched but the probe failed */
		pr_warn("%s: probe of %s failed with error %d\n",
			drv->name, dev_name(dev), ret);
	}
	/*
	 * Ignore errors returned by ->probe so that the next driver can try
	 * its luck.
	 */
	ret = 0;
done:
	atomic_dec(&probe_count);
	wake_up_all(&probe_waitqueue);
	return ret;
}
```

### platform_drv_probe

```C
static int platform_drv_probe(struct device *_dev)
{
	struct platform_driver *drv = to_platform_driver(_dev->driver);
	struct platform_device *dev = to_platform_device(_dev);
	int ret;

	ret = of_clk_set_defaults(_dev->of_node, false);
	if (ret < 0)
		return ret;

	ret = dev_pm_domain_attach(_dev, true);
	if (ret)
		goto out;

	if (drv->probe) {
		ret = drv->probe(dev);
		if (ret)
			dev_pm_domain_detach(_dev, true);
	}

out:
	if (drv->prevent_deferred_probe && ret == -EPROBE_DEFER) {
		dev_warn(_dev, "probe deferral not supported\n");
		ret = -ENXIO;
	}

	return ret;
}
```

