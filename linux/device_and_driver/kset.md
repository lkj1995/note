### 基本概念

- 数据结构

```C
/* include/linux/kobject.h, line 159 */
struct kset {
     //保存该kset下所有的kobject链表
     struct list_head list; 
    
     //自旋锁
     spinlock_t list_lock;
    
     //kset自己的kboject
     struct kobject kobj;   
    
     //uevent的操作集，当kobject需要上报uevent的时候，
     //调用它从属的kest的uevent_ops,添加环境变量，过滤uevent。 
     //因此kobject不属于任何一个kset，是没有uevent_ops进行上报操作的。
     const struct kset_uevent_ops *uevent_ops; 
      
};
```

