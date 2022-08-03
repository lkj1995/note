### kset

```C
/**
 * struct kset - a set of kobjects of a specific type, belonging to a specific subsystem.
 *
 * A kset defines a group of kobjects.  They can be individually
 * different "types" but overall these kobjects all want to be grouped
 * together and operated on in the same manner.  ksets are used to
 * define the attribute callbacks and other common events that happen to
 * a kobject.
 *
 * @list: the list of all kobjects for this kset
 * @list_lock: a lock for iterating over the kobjects
 * @kobj: the embedded kobject for this kset (recursion, isn't it fun...)
 * @uevent_ops: the set of uevent operations for this kset.  These are
 * called whenever a kobject has something happen to it so that the kset
 * can add new environment variables, or filter out the uevents if so
 * desired.
 */
struct kset {
	struct list_head list; /*该kset的子kobj*/
	spinlock_t list_lock; /*自旋锁*/
	struct kobject kobj; /*kobj*/
    /*uevent的ops, 可发送挂载在list的子kobj的uevent*/
	const struct kset_uevent_ops *uevent_ops; 
} __randomize_layout;
```

### kset_uevent_ops

```C
struct kset_uevent_ops {
	int (* const filter)(struct kset *kset, struct kobject *kobj);
	const char *(* const name)(struct kset *kset, struct kobject *kobj);
	int (* const uevent)(struct kset *kset, struct kobject *kobj,
		      struct kobj_uevent_env *env);
};
```

### kobj_uevent_env

```C
struct kobj_uevent_env {
	char *argv[3];
	char *envp[UEVENT_NUM_ENVP];
	int envp_idx;
	char buf[UEVENT_BUFFER_SIZE];
	int buflen;
};
```

### kset_create_and_add

```C
/**
 * kset_create_and_add() - Create a struct kset dynamically and add it to sysfs.
 *
 * @name: the name for the kset
 * @uevent_ops: a struct kset_uevent_ops for the kset
 * @parent_kobj: the parent kobject of this kset, if any.
 *
 * This function creates a kset structure dynamically and registers it
 * with sysfs.  When you are finished with this structure, call
 * kset_unregister() and the structure will be dynamically freed when it
 * is no longer being used.
 *
 * If the kset was not able to be created, NULL will be returned.
 */
struct kset *kset_create_and_add(const char *name,
				 const struct kset_uevent_ops *uevent_ops,
				 struct kobject *parent_kobj)
{
	struct kset *kset;
	int error;
    /*动态创建kset，并初始化name，uevent_ops，通用kype*/
	kset = kset_create(name, uevent_ops, parent_kobj);
    
	if (!kset)
		return NULL;
	error = kset_register(kset);/*初始化并创建kernfs目录和属性文件*/
	if (error) {
		kfree(kset);
		return NULL;
	}
	return kset;
}
```

### kset_create

```C
/**
 * kset_create() - Create a struct kset dynamically.
 *
 * @name: the name for the kset
 * @uevent_ops: a struct kset_uevent_ops for the kset
 * @parent_kobj: the parent kobject of this kset, if any.
 *
 * This function creates a kset structure dynamically.  This structure can
 * then be registered with the system and show up in sysfs with a call to
 * kset_register().  When you are finished with this structure, if
 * kset_register() has been called, call kset_unregister() and the
 * structure will be dynamically freed when it is no longer being used.
 *
 * If the kset was not able to be created, NULL will be returned.
 */
static struct kset *kset_create(const char *name,
				const struct kset_uevent_ops *uevent_ops,
				struct kobject *parent_kobj)
{
	struct kset *kset;
	int retval;

	kset = kzalloc(sizeof(*kset), GFP_KERNEL);/*申请kset结构内存*/
	if (!kset)
		return NULL;
	retval = kobject_set_name(&kset->kobj, "%s", name);/*kset的kobj命名*/
	if (retval) {
		kfree(kset);
		return NULL;
	}
	kset->uevent_ops = uevent_ops; /*记录传入的uevent_ops*/
	kset->kobj.parent = parent_kobj; /*记录传入的parent_kobj*/

	/*
	 * The kobject of this kset will have a type of kset_ktype and belong to
	 * no kset itself.  That way we can properly free it when it is
	 * finished being used.
	 */
	kset->kobj.ktype = &kset_ktype; /*通用的ktype*/
	kset->kobj.kset = NULL;

	return kset;
}
```

```C
static struct kobj_type kset_ktype = {
        .sysfs_ops      = &kobj_sysfs_ops,
        .release        = kset_release,
        .get_ownership  = kset_get_ownership,
};
```

### kset_register

```c
/**
 * kset_register() - Initialize and add a kset.
 * @k: kset.
 */
int kset_register(struct kset *k)
{
	int err;

	if (!k)
		return -EINVAL;
    /*初始化kobj，list，spin_lock*/
	kset_init(k);
    /*
     * 如存在kset，kobj加入list
     * 创建kernfs_node目录，及其attr属性文件
     */	
	err = kobject_add_internal(&k->kobj); 
	if (err)
		return err;
	kobject_uevent(&k->kobj, KOBJ_ADD);/*发送一个ADD的uevent通知用户空间*/
	return 0;
}
```

```C
/**
 * kset_init() - Initialize a kset for use.
 * @k: kset
 */
void kset_init(struct kset *k)
{
	kobject_init_internal(&k->kobj); /*初始化kobj*/
	INIT_LIST_HEAD(&k->list);
	spin_lock_init(&k->list_lock);
}
```

