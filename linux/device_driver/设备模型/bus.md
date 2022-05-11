### 基本概念

- bus是处理器与1个或多个设备之间的通道，所有设备可以都通过总线相连。

- 数据结构

  ```C
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
  	const char		*name;
  	const char		*dev_name;
  	struct device		*dev_root;
  	const struct attribute_group **bus_groups;
  	const struct attribute_group **dev_groups;
  	const struct attribute_group **drv_groups;
  
  	int (*match)(struct device *dev, struct device_driver *drv);
  	int (*uevent)(struct device *dev, struct kobj_uevent_env *env);
  	int (*probe)(struct device *dev);
  	void (*sync_state)(struct device *dev);
  	int (*remove)(struct device *dev);
  	void (*shutdown)(struct device *dev);
  
  	int (*online)(struct device *dev);
  	int (*offline)(struct device *dev);
  
  	int (*suspend)(struct device *dev, pm_message_t state);
  	int (*resume)(struct device *dev);
  
  	int (*num_vf)(struct device *dev);
  
  	int (*dma_configure)(struct device *dev);
  
  	const struct dev_pm_ops *pm;
  
  	const struct iommu_ops *iommu_ops;
  
  	struct subsys_private *p;
  	struct lock_class_key lock_key;
  
  	bool need_parent_lock;
  };
  ```

  - 操作

    ```C
    /*注册bus*/
    /**
     * bus_register - register a driver-core subsystem
     * @bus: bus to register
     *
     * Once we have that, we register the bus with the kobject
     * infrastructure, then register the children subsystems it has:
     * the devices and drivers that belong to the subsystem.
     */
    int bus_register(struct bus_type *bus);
    ```

  
  - 属性
  
    ```c
    struct bus_attribute {
            struct attribute        attr;
            ssize_t (*show)(struct bus_type *bus, char *buf);
            ssize_t (*store)(struct bus_type *bus, const char *buf, size_t count);
    };
    ```
  
    - 初始化宏
  
      ```
      ```
  
      