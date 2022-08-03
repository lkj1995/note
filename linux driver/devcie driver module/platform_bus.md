### platform_driver_register

```C
#define platform_driver_register(drv) \
        __platform_driver_register(drv, THIS_MODULE)
```

### __platform_driver_register

```C
/**
 * __platform_driver_register - register a driver for platform-level devices
 * @drv: platform driver structure
 * @owner: owning module/driver
 */
int __platform_driver_register(struct platform_driver *drv,
				struct module *owner)
{
	drv->driver.owner = owner;
	drv->driver.bus = &platform_bus_type;
	drv->driver.probe = platform_drv_probe;
	drv->driver.remove = platform_drv_remove;
	drv->driver.shutdown = platform_drv_shutdown;

	return driver_register(&drv->driver);
}
```

### platform_match

```C
/**
 * platform_match - bind platform device to platform driver.
 * @dev: device.
 * @drv: driver.
 *
 * Platform device IDs are assumed to be encoded like this:
 * "<name><instance>", where <name> is a short description of the type of
 * device, like "pci" or "floppy", and <instance> is the enumerated
 * instance of the device, like '0' or '42'.  Driver IDs are simply
 * "<name>".  So, extract the <name> from the platform_device structure,
 * and compare it against the name of the driver. Return whether they match
 * or not.
 */
static int platform_match(struct device *dev, struct device_driver *drv)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct platform_driver *pdrv = to_platform_driver(drv);

	/* When driver_override is set, only bind to the matching driver */
    /*首先匹配*/
	if (pdev->driver_override)
		return !strcmp(pdev->driver_override, drv->name);

	/* Attempt an OF style match first */
    /*
     * 设备树比较
     * drv->of_match_table
     * pdev->dev->
     */
	if (of_driver_match_device(dev, drv))
		return 1;

	/* Then try ACPI style match */
	if (acpi_driver_match_device(dev, drv))
		return 1;

	/* Then try to match against the id table */
	if (pdrv->id_table)
		return platform_match_id(pdrv->id_table, pdev) != NULL;

	/* fall-back to driver name match */
    /*最后比较两者的名字*/
	return (strcmp(pdev->name, drv->name) == 0);
}
```





## 总结

1. 当注册platform_device时，会调用platform_bus的match函数进行匹配查找
2. 是不是意味着，调用platform_device和platform_driver，都会自动挂载到名为"platform"的总线
3. a. 比较 platform_dev.driver_override 和 platform_driver.drv->name
   b. 比较 platform_dev.dev.of_node的compatible属性 和 platform_driver.drv->of_match_table
   c. 比较 platform_dev.name 和 platform_driver.id_table
   d. 比较 platform_dev.name 和 platform_driver.drv->name
4. 符合match并返回成功值，此时会调用platform_driver的probe函数进行初始化

