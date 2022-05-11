### 基本概念

- 数据结构

```C
/* include/linux/kobject.h, line 108 */
struct kobj_type {
     //回调函数，用于释放该种类型kobject的数据结构的内存
     void (*release)(struct kobject *kobj);
 
     //该种类型的kobject的文件系统（sysfs）接口
     const struct sysfs_ops *sysfs_ops;
     
     //该种类型的kobject的attribute列表（二级指针说明可能包含），
     //attribute是一个文件，用于开放给用户空间去配置驱动的，
     struct attribute **default_attrs;
 
     const struct kobj_ns_type_operations *(*child_ns_type)(struct kobject *kobj);
    
    //和文件系统（sysfs）的命名空间有关
     const void *(*namespace)(struct kobject *kobj);
 
};
```

