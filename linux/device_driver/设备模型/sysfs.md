### 基本概念

- sysfs基于armfs位于内存的文件系统，用于将Kobject数据结构导出到用户空间，以文件目录结构形式，提供访问支持。

- 每个Kobject，都对应着sysfs中的一个目录，因此将Kobject添加到Kernel时，会调用create_dir创建目录。

- 数据结构

  ```
  /* include/linux/sysfs.h, line 124 */
  struct sysfs_ops {
      ssize_t (*show)(struct kobject *, struct attribute *,char *);
      ssize_t (*store)(struct kobject *,struct attribute *,const char *, size_t);
      const void *(*namespace)(struct kobject *, const struct attribute *);
  };
  ```

  
