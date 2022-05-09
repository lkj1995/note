### 基本概念

- 数据结构

```C
/* Kobject: include/linux/kobject.h line 60 */
struct kobject {
    //该Kobject的名称，同时也是sysfs中的目录名称。由于Kobject添加到Kernel时，
    //需要根据名字注册到sysfs中。
    const char *name;
    
    //将Kobject加入到Kset中的list_head
    struct list_head    entry;
    
    //指向parent kobject，以此形成层次结构
    struct kobject      *parent;
    
    //该kobject属于的Kset。可以为NULL。
    struct kset     *kset;
    
    //该Kobject属于的kobj_type，每个Kobject必须有一个ktype
    struct kobj_type    *ktype;
    
    struct sysfs_dirent *sd;
    
    //用于原子操作的引用计数
    struct kref     kref;
    
    //指示该Kobject是否已经初始化
    unsigned int state_initialized:1; 
    
    //指示该Kobject是否已在sysfs中呈现
    unsigned int state_in_sysfs:1;
    
    //记录是否已经向用户空间发送ADD uevent
    unsigned int state_add_uevent_sent:1;
    unsigned int state_remove_uevent_sent:1;
    
    //如果该字段为1，则表示忽略所有上报的uevent事件
    unsigned int uevent_suppress:1;
};
```

### Kobject和Ktype机制理解

1. Kobject保存了一个引用计数，当计数值为0时，会自动释放，因此内存必须是动态分配。
2. 释放的时候，还需释放包含Kobject的数据结构（驱动开发者定义的），因此需要通过Ktype来实现，调用release，因为驱动开发者才知道该Kobject嵌入到哪里了。
