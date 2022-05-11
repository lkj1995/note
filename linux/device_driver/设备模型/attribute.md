### 基本概念

- 内核空间和用户空间进行信息交互的一种方法。
- 例如某个driver定义了一个变量，却希望用户空间程序可以修改该变量，以控制driver的运行行为，那么就可以将该变量以sysfs attribute的形式开放出来

- 在不同的层，都有自己的属性文件，比如bus，kobject

- 分类

  - 普通attribute

    ```
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

    ```
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

    ```
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