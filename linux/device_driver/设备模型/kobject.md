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

- Kobject保存了一个引用计数，当计数值为0时，会自动释放，因此内存必须是动态分配。

- 释放的时候，还需释放包含Kobject的数据结构（驱动开发者定义的），因此需要通过Ktype来实现，调用Ktype的release函数来对整个结构体进行析构（kobject是一个基类，因此其可嵌入其他结构，使用container_of()来获取）



### 初始化

```c
void kobject_init();
void kobject_set_name();
```

### 引用计数的操作

```c
struct kobject* kobject_get(struct kobject* kobj);
void kobject_put(struct kobject* kobj);
```

### 添加到Kset

- kobject不必在sysfs中表示，但kset中的每一个kobject都将在sysfs中表述

- 创建一个对象时，通常要把kobject添加到kset

  - 先把kobject的kset成员指向目的kset

  - 然后将kobject传递到函数

    ```c
    int kboject_add(struct kobject* kobj);
    ```

    
