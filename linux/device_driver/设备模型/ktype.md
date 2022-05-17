### 数据结构

```C
struct kobj_type {
		void (*release)(struct kobject *kobj);/*处理对象终结的回调函数*/
        const struct sysfs_ops *sysfs_ops;/*kobj的sysfs接口*/
        struct attribute **default_attrs;/*use default_groups instead*/
        const struct attribute_group **default_groups;
        const struct kobj_ns_type_operations *(*child_ns_type)(struct kobject *kobj);
        const void *(*namespace)(struct kobject *kobj);
        void (*get_ownership)(struct kobject *kobj, kuid_t *uid, kgid_t *gid);
 };
```

```C
struct sysfs_ops {
    ssize_t (*show)(struct kobject *, struct attribute *,char *);
    ssize_t (*store)(struct kobject *,struct attribute *,const char *, size_t);
    const void *(*namespace)(struct kobject *, const struct attribute *);
};
```

```C
/**
 * struct attribute_group - data structure used to declare an attribute group.
 * @name:       Optional: Attribute group name
 *              If specified, the attribute group will be created in
 *              a new subdirectory with this name.
 * @is_visible: Optional: Function to return permissions associated with an
 *              attribute of the group. Will be called repeatedly for each
 *              non-binary attribute in the group. Only read/write
 *              permissions as well as SYSFS_PREALLOC are accepted. Must
 *              return 0 if an attribute is not visible. The returned value
 *              will replace static permissions defined in struct attribute.
 * @is_bin_visible:
 *              Optional: Function to return permissions associated with a
 *              binary attribute of the group. Will be called repeatedly
 *              for each binary attribute in the group. Only read/write
 *              permissions as well as SYSFS_PREALLOC are accepted. Must
 *              return 0 if a binary attribute is not visible. The returned
 *              value will replace static permissions defined in
 *              struct bin_attribute.
 * @attrs:      Pointer to NULL terminated list of attributes.
 * @bin_attrs:  Pointer to NULL terminated list of binary attributes.
 *              Either attrs or bin_attrs or both must be provided.
 */
struct attribute_group {
        const char              *name;
        umode_t                 (*is_visible)(struct kobject *,
                                                struct attribute *, int);
        umode_t                 (*is_bin_visible)(struct kobject *,
                                                    struct bin_attribute *, int);
        struct attribute        **attrs;
        struct bin_attribute    **bin_attrs;
};
```

