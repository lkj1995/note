### 1 i2c_init()

```C
static struct i2c_driver dummy_driver = {
	.driver.name	= "dummy",
	.probe		= dummy_probe,
	.remove		= dummy_remove,
	.id_table	= dummy_id,
};


static int __init i2c_init(void)
{
	int retval;

	retval = of_alias_get_highest_id("i2c");

	down_write(&__i2c_board_lock);
	if (retval >= __i2c_first_dynamic_bus_num)
		__i2c_first_dynamic_bus_num = retval + 1;
	up_write(&__i2c_board_lock);

    /*生成i2c_bus总线*/
	retval = bus_register(&i2c_bus_type);
	if (retval)
		return retval;

    /*这里关联了其他i2c相关的driver,必须这个driver先执行*/
	is_registered = true;

#ifdef CONFIG_I2C_COMPAT
    /*创建一个名为"i2c-adapter"的kobj*/
	i2c_adapter_compat_class = class_compat_register("i2c-adapter");
	if (!i2c_adapter_compat_class) {
		retval = -ENOMEM;
		goto bus_err;
	}
#endif
    /*单纯的注册了driver,因为总线刚创建，无其他操作*/
	retval = i2c_add_driver(&dummy_driver);
	if (retval)
		goto class_err;

	if (IS_ENABLED(CONFIG_OF_DYNAMIC))
		WARN_ON(of_reconfig_notifier_register(&i2c_of_notifier));
	if (IS_ENABLED(CONFIG_ACPI))
		WARN_ON(acpi_reconfig_notifier_register(&i2c_acpi_notifier));

	return 0;

class_err:
#ifdef CONFIG_I2C_COMPAT
	class_compat_unregister(i2c_adapter_compat_class);
bus_err:
#endif
	is_registered = false;
	bus_unregister(&i2c_bus_type);
	return retval;
}

postcore_initcall(i2c_init);/*系统启动时，会先执行该init*/
```

### 总结

1. 重要的工作在init函数完成，创建了i2c_bus总线和一个在i2c_bus总线的dummy_driver。
2. 重要的操作函数都被导出，可用于其他driver。
