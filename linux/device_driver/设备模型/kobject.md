### kobject

```C
struct kobject {
          const char              *name; /*sysfs中的目录名*/
          struct list_head        entry;/*用于Kobj挂到Kset->list中管理*/
          struct kobject          *parent;/*指向父kobj，形成层次结构*/
          struct kset             *kset;/*该kobj所属的Kset*/
          struct kobj_type        *ktype;/*每个Kobj对应一个ktype*/
          struct kernfs_node      *sd; /*对应sysfs对象，基于kernfs_node实现*/
          struct kref             kref;/*引用计数，当大于0时必须存在*/
#ifdef CONFIG_DEBUG_KOBJECT_RELEASE
          struct delayed_work     release;
#endif
          unsigned int state_initialized:1;/*该Kobj是否初始化*/
          unsigned int state_in_sysfs:1;/*Kobj是否已存在sysfs*/
    
          /*当发送KOBJ_ADD时，置位，提示已向用户空间发送ADD*/
          unsigned int state_add_uevent_sent:1;
          /*当发送KOBJ_REMOVE时，置位，提示已向用户空间发送REMOVE*/
          unsigned int state_remove_uevent_sent:1;
    
          unsigned int uevent_suppress:1;/*为1，则忽略所有上报的uevent*/
};
```

### kernfs_node

```C
/*
 * kernfs_node - the building block of kernfs hierarchy.  Each and every
 * kernfs node is represented by single kernfs_node.  Most fields are
 * private to kernfs and shouldn't be accessed directly by kernfs users.
 *
 * As long as s_count reference is held, the kernfs_node itself is
 * accessible.  Dereferencing elem or any other outer entity requires
 * active reference.
 */
struct kernfs_node {
	atomic_t		count;
	atomic_t		active;
#ifdef CONFIG_DEBUG_LOCK_ALLOC
	struct lockdep_map	dep_map;
#endif
	/*
	 * Use kernfs_get_parent() and kernfs_name/path() instead of
	 * accessing the following two fields directly.  If the node is
	 * never moved to a different parent, it is safe to access the
	 * parent directly.
	 */
	struct kernfs_node	*parent;
	const char		*name;

	struct rb_node		rb;

	const void		*ns;	/* namespace tag */
	unsigned int		hash;	/* ns + name hash */
	union {
		struct kernfs_elem_dir		dir;
		struct kernfs_elem_symlink	symlink;
		struct kernfs_elem_attr		attr;
	};

	void			*priv;

	/*
	 * 64bit unique ID.  On 64bit ino setups, id is the ino.  On 32bit,
	 * the low 32bits are ino and upper generation.
	 */
	u64			id;

	unsigned short		flags;
	umode_t			mode;
	struct kernfs_iattrs	*iattr;
};
```

### kernfs_elem_attr

```C
struct kernfs_elem_attr {
	const struct kernfs_ops	*ops;
	struct kernfs_open_node	*open;
	loff_t			size;
	struct kernfs_node	*notify_next;	/* for kernfs_notify() */
};
```

### get

```C
/**
 * kobject_get() - Increment refcount for object.
 * @kobj: object.
 */
struct kobject *kobject_get(struct kobject *kobj)
{
	if (kobj) {
		if (!kobj->state_initialized) /*未初始化*/
			WARN(1, KERN_WARNING
				"kobject: '%s' (%p): is not initialized, yet kobject_get() is being called.\n",
			     kobject_name(kobj), kobj);
		kref_get(&kobj->kref); /*kobj->kref += 1*/
	}
	return kobj;
}
```

### put

```C
/**
 * kobject_put() - Decrement refcount for object.
 * @kobj: object.
 *
 * Decrement the refcount, and if 0, call kobject_cleanup().
 */
void kobject_put(struct kobject *kobj)
{
	if (kobj) {
		if (!kobj->state_initialized)
			WARN(1, KERN_WARNING
				"kobject: '%s' (%p): is not initialized, yet kobject_put() is being called.\n",
			     kobject_name(kobj), kobj);
        /*kobj->kref -= 1,如果减到0，执行kobject_release*/
		kref_put(&kobj->kref, kobject_release);
        
	}
}
```



### kobject_create

```C
/**
 * kobject_create() - Create a struct kobject dynamically.
 * 动态申请内存
 * This function creates a kobject structure dynamically and sets it up
 * to be a "dynamic" kobject with a default release function set up.
 *
 * If the kobject was not able to be created, NULL will be returned.
 * The kobject structure returned from here must be cleaned up with a
 * call to kobject_put() and not kfree(), as kobject_init() has
 * already been called on this structure.
 */
struct kobject *kobject_create(void)
{
	struct kobject *kobj;

	kobj = kzalloc(sizeof(*kobj), GFP_KERNEL); /*动态申请kobj结构空间*/
	if (!kobj)
		return NULL;

	kobject_init(kobj, &dynamic_kobj_ktype);
	return kobj;
}
```

### kobject_init

```c
/**
 * kobject_init() - Initialize a kobject structure.
 * @kobj: pointer to the kobject to initialize
 * @ktype: pointer to the ktype for this kobject.
 *
 * This function will properly initialize a kobject such that it can then
 * be passed to the kobject_add() call.
 *
 * After this function is called, the kobject MUST be cleaned up by a call
 * to kobject_put(), not by a call to kfree directly to ensure that all of
 * the memory is cleaned up properly.
 */
void kobject_init(struct kobject *kobj, struct kobj_type *ktype)
{
	char *err_str;

	if (!kobj) { /*kobj空指针检查*/
		err_str = "invalid kobject pointer!";
		goto error;
	}
	if (!ktype) {/*ktype空指针检查*/
		err_str = "must have a ktype to be initialized properly!\n";
		goto error;
	}
	if (kobj->state_initialized) {/*不能初始化已经初始化的kobj*/
		/* do not error out as sometimes we can recover */
		pr_err("kobject (%p): tried to init an initialized object, something is seriously wrong.\n",
		       kobj);
		dump_stack();
	}
	kobject_init_internal(kobj);/*初始化kobj结构体*/
	kobj->ktype = ktype; /*将ktype记录到kobj->ktype指针*/
	return;

error:
	pr_err("kobject (%p): %s\n", kobj, err_str);
	dump_stack();
}

```

### kobject_init_internal

```C
static void kobject_init_internal(struct kobject *kobj)
{
	if (!kobj)/*kobj空指针检查*/
		return;
	kref_init(&kobj->kref); /*kref引用计数设置为1*/
	INIT_LIST_HEAD(&kobj->entry);/*初始化链表，prev和next都指向自身*/
	kobj->state_in_sysfs = 0;/*尚未存在sysfs中，设为0*/ 
	kobj->state_add_uevent_sent = 0; 
	kobj->state_remove_uevent_sent = 0;
	kobj->state_initialized = 1;/*kobj已初始化,置1*/
}
```

### kobject_set_name

```C
/**
 * kobject_set_name() - Set the name of a kobject.
 * @kobj: struct kobject to set the name of
 * @fmt: format string used to build the name
 *
 * This sets the name of the kobject.  If you have already added the
 * kobject to the system, you must call kobject_rename() in order to
 * change the name of the kobject.
 */
int kobject_set_name(struct kobject *kobj, const char *fmt, ...)
{
	va_list vargs;
	int retval;

	va_start(vargs, fmt);/*可变形参格式的命名*/
	retval = kobject_set_name_vargs(kobj, fmt, vargs);/*申请内存，设置kobj->name指针*/
	va_end(vargs);

	return retval;
}
```

### kobject_add

```C
/**
 * kobject_add() - The main kobject add function.
 * @kobj: the kobject to add
 * @parent: pointer to the parent of the kobject.
 * @fmt: format to name the kobject with.
 *
 * The kobject name is set and added to the kobject hierarchy in this
 * function.
 *
 * If @parent is set, then the parent of the @kobj will be set to it.
 * If @parent is NULL, then the parent of the @kobj will be set to the
 * kobject associated with the kset assigned to this kobject.  If no kset
 * is assigned to the kobject, then the kobject will be located in the
 * root of the sysfs tree.
 *
 * Note, no "add" uevent will be created with this call, the caller should set
 * up all of the necessary sysfs files for the object and then call
 * kobject_uevent() with the UEVENT_ADD parameter to ensure that
 * userspace is properly notified of this kobject's creation.
 *
 * Return: If this function returns an error, kobject_put() must be
 *         called to properly clean up the memory associated with the
 *         object.  Under no instance should the kobject that is passed
 *         to this function be directly freed with a call to kfree(),
 *         that can leak memory.
 *
 *         If this function returns success, kobject_put() must also be called
 *         in order to properly clean up the memory associated with the object.
 *
 *         In short, once this function is called, kobject_put() MUST be called
 *         when the use of the object is finished in order to properly free
 *         everything.
 */
int kobject_add(struct kobject *kobj, struct kobject *parent,
		const char *fmt, ...)
{
	va_list args;
	int retval;

	if (!kobj)
		return -EINVAL;

	if (!kobj->state_initialized) { /*需要初始化才能执行add*/
		pr_err("kobject '%s' (%p): tried to add an uninitialized object, something is seriously wrong.\n",
		       kobject_name(kobj), kobj);
		dump_stack();
		return -EINVAL;
	}
	va_start(args, fmt);
	retval = kobject_add_varg(kobj, parent, fmt, args);/*继续传递*/
	va_end(args);

	return retval;
}
```

### kobject_add_varg

```C
static __printf(3, 0) int kobject_add_varg(struct kobject *kobj,
					   struct kobject *parent,
					   const char *fmt, va_list vargs)
{
	int retval;

	retval = kobject_set_name_vargs(kobj, fmt, vargs);/*申请内存，设置kobj->name指针*/
	if (retval) {
		pr_err("kobject: can not set name properly!\n");
		return retval;
	}
	kobj->parent = parent; /*记录parent，可能为null*/
    /*
     * 如存在kset，kobj加入list
     * 创建kernfs_node目录，及其attr属性文件
     */
	return kobject_add_internal(kobj);
    
}
```

### kobject_add_internal

```C
static int kobject_add_internal(struct kobject *kobj)
{
	int error = 0;
	struct kobject *parent;

	if (!kobj)/*检查*/
		return -ENOENT;

	if (!kobj->name || !kobj->name[0]) {/*无法初始化未命名的kobj*/
		WARN(1,
		     "kobject: (%p): attempted to be registered with empty name!\n",
		     kobj);
		return -EINVAL;
	}
    /*因为属于parent下，增加其parent的kobj引用计数。存在子引用，父不能被清理*/
	parent = kobject_get(kobj->parent);

	/* join kset if set, use it as parent if we do not already have one */
	if (kobj->kset) {
		if (!parent) /*如不存在parent，但有kset，增加kset的kobj引用计数*/
			parent = kobject_get(&kobj->kset->kobj); 
       
		kobj_kset_join(kobj);/*kobj加入kset链表，并增加kset的kobj的引用计数*/
		kobj->parent = parent; /*或是kset，或是传参的parent*/
	}

	pr_debug("kobject: '%s' (%p): %s: parent: '%s', set: '%s'\n",
		 kobject_name(kobj), kobj, __func__,
		 parent ? kobject_name(parent) : "<NULL>",
		 kobj->kset ? kobject_name(&kobj->kset->kobj) : "<NULL>");

	error = create_dir(kobj);
	if (error) {
		kobj_kset_leave(kobj); /*创建dir失败，在kset的链表，删除kobj*/
		kobject_put(parent); /*减少引用计数*/
		kobj->parent = NULL; /*此时可能指向了kset，清空*/

		/* be noisy on error issues */
		if (error == -EEXIST)
			pr_err("%s failed for %s with -EEXIST, don't try to register things with the same name in the same directory.\n",
			       __func__, kobject_name(kobj));
		else
			pr_err("%s failed for %s (error: %d parent: %s)\n",
			       __func__, kobject_name(kobj), error,
			       parent ? kobject_name(parent) : "'none'");
	} else
		kobj->state_in_sysfs = 1; /*成功，标记kobj已经生成对应的kenfs_node节点,并记录到sysfs*/

	return error;
}
```

### kobj_kset_join

```C
/* add the kobject to its kset's list */
static void kobj_kset_join(struct kobject *kobj)
{
	if (!kobj->kset)/*检查*/
		return;

	kset_get(kobj->kset);/*增加kset的kobj的引用计数*/
	spin_lock(&kobj->kset->list_lock);/*上锁*/
	list_add_tail(&kobj->entry, &kobj->kset->list);/*把kobj的它指向的kset的链表进行管理*/
	spin_unlock(&kobj->kset->list_lock);/*解锁*/
}
```

### create_dir

```C
static int create_dir(struct kobject *kobj)
{
	const struct kobj_type *ktype = get_ktype(kobj);
	const struct kobj_ns_type_operations *ops;
	int error;
    /*
     * 对kobj创建一个对应的kenfs_node，并显示到用户空间
     * kobj->sd = 新创建的kenfs_node;
     * kenfs_node->priv = kobj;
    */
	error = sysfs_create_dir_ns(kobj, kobject_namespace(kobj));
	if (error)
		return error;
    /*
     * 根据上面创建的目录里，继承目录的uid，gid
     * 对传入的attr数组，每个attr结构生成一个kenfs_node属性文件
     * 根据命名ns初始化，kenfs_node->priv = attribute
    */
	error = populate_dir(kobj); 
	if (error) {
		sysfs_remove_dir(kobj);/*创建属性文件失败，释放kobj创建的目录*/
		return error;
	}
     /*为啥需要创建两种attribute和attribute_group，难道是要准备删除attribute*/
	if (ktype) {
    	/*
    	 * 根据上面创建的目录里，继承目录的uid，gid
     	 *  对传入的gruup中的attr数组，每个attr结构生成一个kenfs_node属性文件
     	 *  根据命名ns初始化，kenfs_node->priv = attribute
    	 */
		error = sysfs_create_groups(kobj, ktype->default_groups);
		if (error) {
			sysfs_remove_dir(kobj);
			return error;
		}
	}

	/*
	 * @kobj->sd may be deleted by an ancestor going away.  Hold an
	 * extra reference so that it stays until @kobj is gone.
	 */
	sysfs_get(kobj->sd);

	/*
	 * If @kobj has ns_ops, its children need to be filtered based on
	 * their namespace tags.  Enable namespace support on @kobj->sd.
	 */
	ops = kobj_child_ns_ops(kobj);
	if (ops) {
		BUG_ON(ops->type <= KOBJ_NS_TYPE_NONE);
		BUG_ON(ops->type >= KOBJ_NS_TYPES);
		BUG_ON(!kobj_ns_type_registered(ops->type));

		sysfs_enable_ns(kobj->sd);
	}

	return 0;
}
```

### kobject_namespace

```C
/**
 * kobject_namespace() - Return @kobj's namespace tag.
 * @kobj: kobject in question
 *
 * Returns namespace tag of @kobj if its parent has namespace ops enabled
 * and thus @kobj should have a namespace tag associated with it.  Returns
 * %NULL otherwise.
 */
const void *kobject_namespace(struct kobject *kobj)
{
	const struct kobj_ns_type_operations *ns_ops = kobj_ns_ops(kobj);

	if (!ns_ops || ns_ops->type == KOBJ_NS_TYPE_NONE)
		return NULL;

	return kobj->ktype->namespace(kobj);/*函数指针，应该是在驱动层被定义，返回一个字符串指针*/
}
```

### sysfs_create_dir_ns


```C
/**
 * sysfs_create_dir_ns - create a directory for an object with a namespace tag
 * 根据kobj的
 * @kobj: object we're creating directory for
 * @ns: the namespace tag to use
 */
int sysfs_create_dir_ns(struct kobject *kobj, const void *ns)
{
	struct kernfs_node *parent, *kn;
	kuid_t uid;
	kgid_t gid;

	if (WARN_ON(!kobj))
		return -EINVAL;

	if (kobj->parent)/*如存在parent，则在parent目录下创建子目录*/
		parent = kobj->parent->sd;
	else/*否则，在root目录创建子目录，root目录为/sys/ */
		parent = sysfs_root_kn;/*全局指针，应该在linux内核运行中被初始化指向sys*/

	if (!parent)/*检查父目录是否为空*/
		return -ENOENT;
    /*调用了kobj->ktype->get_ownership ，应该是获取uid和gid*/
	kobject_get_ownership(kobj, &uid, &gid); 

    /*
     * 申请kernfs_node结构内存
     * kn->parent = parent;
     * nfs_node->priv = kobj;
     * 激活node在用户空间显示
    */
	kn = kernfs_create_dir_ns(parent, kobject_name(kobj),
				  S_IRWXU | S_IRUGO | S_IXUGO, uid, gid,
				  kobj, ns);
	if (IS_ERR(kn)) {
		if (PTR_ERR(kn) == -EEXIST)
			sysfs_warn_dup(parent, kobject_name(kobj));
		return PTR_ERR(kn);
	}

	kobj->sd = kn; /*创建kn完成后，记录在kobj->sd上*/
	return 0;
}
```

```C
static inline const char *kobject_name(const struct kobject *kobj)
{
        return kobj->name;/*返回kobj的name成员内容*/
}
```


```C
/**
 * kernfs_create_dir_ns - create a directory
 * @parent: parent in which to create a new directory (根据parent创建子目录的位置)
 * @name: name of the new directory             
 * @mode: mode of the new directory
 * @uid: uid of the new directory
 * @gid: gid of the new directory
 * @priv: opaque data associated with the new directory (nfs_node的priv保存了kobj)
 * @ns: optional namespace tag of the directory
 *
 * Returns the created node on success, ERR_PTR() value on failure.
 */
struct kernfs_node *kernfs_create_dir_ns(struct kernfs_node *parent,
					 const char *name, umode_t mode,
					 kuid_t uid, kgid_t gid,
					 void *priv, const void *ns)
{
	struct kernfs_node *kn;
	int rc;

	/* allocate */
	kn = kernfs_new_node(parent, name, mode | S_IFDIR,
			     uid, gid, KERNFS_DIR); 
    /*
     * 申请kernfs_node结构内存
     * kn->parent = parent;
     * 标记为S_IFREG为文件类型，标记为S_IFDIR为目录类型
    */
	if (!kn)
		return ERR_PTR(-ENOMEM);

	kn->dir.root = parent->dir.root;
	kn->ns = ns; /*记录namespace*/
	kn->priv = priv; /* nfs_node->priv = kobj; */

	/* link in */
	rc = kernfs_add_one(kn); /*添加并激活它，这样在用户空间就能查看到该node，或是目录，或是文件*/
	if (!rc)
		return kn;/*成功，返回创建的kn地址*/

	kernfs_put(kn);/*清除引用计数*/
	return ERR_PTR(rc);
}
```



```C
/*
 * populate_dir - populate directory with attributes.
 * @kobj: object we're working on.
 *
 * Most subsystems have a set of default attributes that are associated
 * with an object that registers with them.  This is a helper called during
 * object registration that loops through the default attributes of the
 * subsystem and creates attributes files for them in sysfs.
 */
static int populate_dir(struct kobject *kobj)
{
	struct kobj_type *t = get_ktype(kobj); /*获取kobj的ktype */
	struct attribute *attr;
	int error = 0;
	int i;

	if (t && t->default_attrs) {  /*至少存在1个default_attrs*/
		for (i = 0; (attr = t->default_attrs[i]) != NULL; i++) { /*对其进行遍历，进行属性文件的创建*/
			error = sysfs_create_file(kobj, attr);
			if (error)
				break;
		}
	}
	return error;
}
```

```C
static inline int __must_check sysfs_create_file(struct kobject *kobj,
                                                 const struct attribute *attr)
{
        return sysfs_create_file_ns(kobj, attr, NULL);
}
```

```C
/**
 * sysfs_create_file_ns - create an attribute file for an object with custom ns
 * 根据attr的ns，创建属性文件
 * @kobj: object we're creating for
 * @attr: attribute descriptor
 * @ns: namespace the new file should belong to
 */
int sysfs_create_file_ns(struct kobject *kobj, const struct attribute *attr,
			 const void *ns)
{
	kuid_t uid;
	kgid_t gid;

	if (WARN_ON(!kobj || !kobj->sd || !attr))
		return -EINVAL;

	kobject_get_ownership(kobj, &uid, &gid); /*获取kobj的kenfs_node的uid和gid，并继承*/
	return sysfs_add_file_mode_ns(kobj->sd, attr, false, attr->mode,
				      uid, gid, ns); /*继续传递*/

}
```

```C
int sysfs_add_file_mode_ns(struct kernfs_node *parent,
                           /*因为是在/sys/xxx/下创建属性目录，所以父目录为kobj->sd*/
			   const struct attribute *attr, bool is_bin,
			   umode_t mode, kuid_t uid, kgid_t gid, const void *ns)
{
	struct lock_class_key *key = NULL;
	const struct kernfs_ops *ops;
	struct kernfs_node *kn;
	loff_t size;

	if (!is_bin) {/*进入此分支*/
        /*上述分析可知，parent->priv保存了kobj，kobj->sd保存了kernfs_node*/
		struct kobject *kobj = parent->priv; 
		const struct sysfs_ops *sysfs_ops = kobj->ktype->sysfs_ops;/*留到ktype再分析*/

		/* every kobject with an attribute needs a ktype assigned */
		if (WARN(!sysfs_ops, KERN_ERR
			 "missing sysfs attribute operations for kobject: %s\n",
			 kobject_name(kobj)))
			return -EINVAL;
         /*show和store的处理，read_write，read_only，write_only*/
		if (sysfs_ops->show && sysfs_ops->store) {         
			if (mode & SYSFS_PREALLOC)
				ops = &sysfs_prealloc_kfops_rw; /*通用操作*/
			else
				ops = &sysfs_file_kfops_rw;
		} else if (sysfs_ops->show) {
			if (mode & SYSFS_PREALLOC)
				ops = &sysfs_prealloc_kfops_ro;
			else
				ops = &sysfs_file_kfops_ro;
		} else if (sysfs_ops->store) {
			if (mode & SYSFS_PREALLOC)
				ops = &sysfs_prealloc_kfops_wo;
			else
				ops = &sysfs_file_kfops_wo;
		} else
			ops = &sysfs_file_kfops_empty;

		size = PAGE_SIZE;
	} else {
		struct bin_attribute *battr = (void *)attr;

		if (battr->mmap)
			ops = &sysfs_bin_kfops_mmap;
		else if (battr->read && battr->write)
			ops = &sysfs_bin_kfops_rw;
		else if (battr->read)
			ops = &sysfs_bin_kfops_ro;
		else if (battr->write)
			ops = &sysfs_bin_kfops_wo;
		else
			ops = &sysfs_file_kfops_empty;

		size = battr->size;
	}

#ifdef CONFIG_DEBUG_LOCK_ALLOC
	if (!attr->ignore_lockdep)
		key = attr->key ?: (struct lock_class_key *)&attr->skey;
#endif
    /*
     * 申请kernfs_node结构内存
     * kn->parent = parent;
     * kn->priv = ktype->attribute;
     * 传递进去的ops被记录
     */
	kn = __kernfs_create_file(parent, attr->name, mode & 0777, uid, gid,
				  size, ops, (void *)attr, ns, key);
	if (IS_ERR(kn)) {
		if (PTR_ERR(kn) == -EEXIST)
			sysfs_warn_dup(parent, attr->name);
		return PTR_ERR(kn);
	}
	return 0;
}
```

```C
/**
 * __kernfs_create_file - kernfs internal function to create a file
 * @parent: directory to create the file in
 * @name: name of the file
 * @mode: mode of the file
 * @uid: uid of the file
 * @gid: gid of the file
 * @size: size of the file
 * @ops: kernfs operations for the file (刚上面初始化的，如sysfs_file_kfops_rw)
 * @priv: private data for the file   (为每个attribute生成一个属性文件，这是其中一个attribute结构)
 * ns为驱动用户进行传递进来的名字，通过传入的kobj->ktype->attribute获取得到
 * @ns: optional namespace tag of the file
 * @key: lockdep key for the file's active_ref, %NULL to disable lockdep
 *
 * Returns the created node on success, ERR_PTR() value on error.
 */
struct kernfs_node *__kernfs_create_file(struct kernfs_node *parent,
					 const char *name,
					 umode_t mode, kuid_t uid, kgid_t gid,
					 loff_t size,
					 const struct kernfs_ops *ops,
					 void *priv, const void *ns,
					 struct lock_class_key *key)
{
	struct kernfs_node *kn;
	unsigned flags;
	int rc;

	flags = KERNFS_FILE;
    /*
     * 申请kernfs_node结构内存
     * kn->parent = parent;
     * 标记为S_IFREG为文件类型，标记为S_IFDIR为目录类型
    */
	kn = kernfs_new_node(parent, name, (mode & S_IALLUGO) | S_IFREG,
			     uid, gid, flags);
	if (!kn)
		return ERR_PTR(-ENOMEM);

	kn->attr.ops = ops; /*传递进来的属性的ops被记录*/
	kn->attr.size = size;
	kn->ns = ns; /*namespace*/
	kn->priv = priv;/* kn->priv = ktype->attribute; */

#ifdef CONFIG_DEBUG_LOCK_ALLOC
	if (key) {
		lockdep_init_map(&kn->dep_map, "kn->active", key, 0);
		kn->flags |= KERNFS_LOCKDEP;
	}
#endif

	/*
	 * kn->attr.ops is accesible only while holding active ref.  We
	 * need to know whether some ops are implemented outside active
	 * ref.  Cache their existence in flags.
	 */
	if (ops->seq_show) /*在被调用时，只有在flag中存在的ref才能被使用，这里对可调用的函数进行ref*/
		kn->flags |= KERNFS_HAS_SEQ_SHOW;
	if (ops->mmap)
		kn->flags |= KERNFS_HAS_MMAP;
	if (ops->release)
		kn->flags |= KERNFS_HAS_RELEASE;

	rc = kernfs_add_one(kn);/*添加并激活它，这样在用户空间就能查看到该node，或是目录，或是文件*/
	if (rc) {
		kernfs_put(kn);/*失败，解除引用计数，应该会进行内存释放*/
		return ERR_PTR(rc);
	}
	return kn;
}

```



```C
 /**
  * sysfs_create_groups - given a directory kobject, create a bunch of attribute groups
  * @kobj:       The kobject to create the group on
  * @groups:     The attribute groups to create, NULL terminated
  *
  * This function creates a bunch of attribute groups.  If an error occurs when
  * creating a group, all previously created groups will be removed, unwinding
  * everything back to the original state when this function was called.
  * It will explicitly warn and error if any of the attribute files being
  * created already exist.
  *
  * Returns 0 on success or error code from sysfs_create_group on failure.
  */
 int sysfs_create_groups(struct kobject *kobj,
                         const struct attribute_group **groups)
 {
         return internal_create_groups(kobj, 0, groups);
 }

```

```C
static int internal_create_groups(struct kobject *kobj, int update,
				  const struct attribute_group **groups)
{
	int error = 0;
	int i;

	if (!groups)/*检查*/
		return 0;

	for (i = 0; groups[i]; i++) {
		error = internal_create_group(kobj, update, groups[i]);
		if (error) { /*但凡出现一个错误，就会对之前配置好的全部删除*/
			while (--i >= 0)
				sysfs_remove_group(kobj, groups[i]);
			break;
		}
	}
	return error;
}
```

```C
static int internal_create_group(struct kobject *kobj, int update,
				 const struct attribute_group *grp)
{
	struct kernfs_node *kn;
	kuid_t uid;
	kgid_t gid;
	int error;

	if (WARN_ON(!kobj || (!update && !kobj->sd)))
		return -EINVAL;

	/* Updates may happen before the object has been instantiated */
	if (unlikely(update && !kobj->sd))
		return -EINVAL;
	if (!grp->attrs && !grp->bin_attrs) { /*attr没有被设置*/
		WARN(1, "sysfs: (bin_)attrs not set by subsystem for group: %s/%s\n",
			kobj->name, grp->name ?: "");
		return -EINVAL;
	}
	kobject_get_ownership(kobj, &uid, &gid); /*获取父目录的uid和gid并继承*/
	if (grp->name) { /*name检查*/
		if (update) { /*这个update应该是更新kernfs_node对应attr的属性文件内容*/
			kn = kernfs_find_and_get(kobj->sd, grp->name);
			if (!kn) {
				pr_warn("Can't update unknown attr grp name: %s/%s\n",
					kobj->name, grp->name);
				return -EINVAL;
			}
		} else { /*不是更新，是创建*/
       		/*
    		 * 申请kernfs_node结构内存
     		 * kn->parent = parent;
     		 * nfs_node->priv = kobj;
     		 * 激活node在用户空间显示
   		 	 */         
			kn = kernfs_create_dir_ns(kobj->sd, grp->name,
						  S_IRWXU | S_IRUGO | S_IXUGO,
						  uid, gid, kobj, NULL);
			if (IS_ERR(kn)) {
				if (PTR_ERR(kn) == -EEXIST)
					sysfs_warn_dup(kobj->sd, grp->name);
				return PTR_ERR(kn);
			}
		}
	} else
		kn = kobj->sd;
	kernfs_get(kn); /*增加引用计数*/
	error = create_files(kn, kobj, uid, gid, grp, update);
	if (error) {
		if (grp->name)
			kernfs_remove(kn);
	}
	kernfs_put(kn);

	if (grp->name && update)
		kernfs_put(kn);

	return error;
}
```

```C
static int create_files(struct kernfs_node *parent, struct kobject *kobj,
			kuid_t uid, kgid_t gid,
			const struct attribute_group *grp, int update)
{
	struct attribute *const *attr;
	struct bin_attribute *const *bin_attr;
	int error = 0, i;

	if (grp->attrs) {/*至少存在1个attr*/
		for (i = 0, attr = grp->attrs; *attr && !error; i++, attr++) { 
            /*遍历，并检查错误error*/
			umode_t mode = (*attr)->mode;

			/*
			 * In update mode, we're changing the permissions or
			 * visibility.  Do this by first removing then
			 * re-adding (if required) the file.
			 */
			if (update)
				kernfs_remove_by_name(parent, (*attr)->name);
			if (grp->is_visible) {
				mode = grp->is_visible(kobj, *attr, i);
				if (!mode)
					continue;
			}

			WARN(mode & ~(SYSFS_PREALLOC | 0664),
			     "Attribute %s: Invalid permissions 0%o\n",
			     (*attr)->name, mode); /*警告出现了不该出现的标记，但下面会进行处理*/

			mode &= SYSFS_PREALLOC | 0664; /*与掩码操作*/
			error = sysfs_add_file_mode_ns(parent, *attr, false,
						       mode, uid, gid, NULL);
			if (unlikely(error))
				break;
		}
		if (error) {
			remove_files(parent, grp);
			goto exit;
		}
	}

	if (grp->bin_attrs) {
		for (i = 0, bin_attr = grp->bin_attrs; *bin_attr; i++, bin_attr++) {
			umode_t mode = (*bin_attr)->attr.mode;

			if (update)
				kernfs_remove_by_name(parent,
						(*bin_attr)->attr.name);
			if (grp->is_bin_visible) {
				mode = grp->is_bin_visible(kobj, *bin_attr, i);
				if (!mode)
					continue;
			}

			WARN(mode & ~(SYSFS_PREALLOC | 0664),
			     "Attribute %s: Invalid permissions 0%o\n",
			     (*bin_attr)->attr.name, mode);

			mode &= SYSFS_PREALLOC | 0664;
			error = sysfs_add_file_mode_ns(parent,
					&(*bin_attr)->attr, true,
					mode,
					uid, gid, NULL);
			if (error)
				break;
		}
		if (error)
			remove_files(parent, grp);
	}
exit:
	return error;
}
```



```C
int sysfs_add_file_mode_ns(struct kernfs_node *parent,
                           /*当前父目录的kernfs_node*/
			   const struct attribute *attr, bool is_bin,
                           /*gruop中保存的第n个attr*/
			   umode_t mode, kuid_t uid, kgid_t gid, const void *ns)
                 /*传入的mode*/
{
	struct lock_class_key *key = NULL;
	const struct kernfs_ops *ops;
	struct kernfs_node *kn;
	loff_t size;

	if (!is_bin) {
		struct kobject *kobj = parent->priv;
		const struct sysfs_ops *sysfs_ops = kobj->ktype->sysfs_ops;

		/* every kobject with an attribute needs a ktype assigned */
		if (WARN(!sysfs_ops, KERN_ERR
			 "missing sysfs attribute operations for kobject: %s\n",
			 kobject_name(kobj)))
			return -EINVAL;
         /*类似上面的函数操作了*/
		if (sysfs_ops->show && sysfs_ops->store) {
			if (mode & SYSFS_PREALLOC)
				ops = &sysfs_prealloc_kfops_rw;
			else
				ops = &sysfs_file_kfops_rw;
		} else if (sysfs_ops->show) {
			if (mode & SYSFS_PREALLOC)
				ops = &sysfs_prealloc_kfops_ro;
			else
				ops = &sysfs_file_kfops_ro;
		} else if (sysfs_ops->store) {
			if (mode & SYSFS_PREALLOC)
				ops = &sysfs_prealloc_kfops_wo;
			else
				ops = &sysfs_file_kfops_wo;
		} else
			ops = &sysfs_file_kfops_empty;

		size = PAGE_SIZE;
	} else {
		struct bin_attribute *battr = (void *)attr;

		if (battr->mmap)
			ops = &sysfs_bin_kfops_mmap;
		else if (battr->read && battr->write)
			ops = &sysfs_bin_kfops_rw;
		else if (battr->read)
			ops = &sysfs_bin_kfops_ro;
		else if (battr->write)
			ops = &sysfs_bin_kfops_wo;
		else
			ops = &sysfs_file_kfops_empty;

		size = battr->size;
	}

#ifdef CONFIG_DEBUG_LOCK_ALLOC
	if (!attr->ignore_lockdep)
		key = attr->key ?: (struct lock_class_key *)&attr->skey;
#endif
    /*
     * 申请kernfs_node结构内存
     * kn->parent = parent;
     * kn->priv = ktype->attribute;
     * 传递进去的ops被记录
     */
	kn = __kernfs_create_file(parent, attr->name, mode & 0777, uid, gid,
				  size, ops, (void *)attr, ns, key);
	if (IS_ERR(kn)) {
		if (PTR_ERR(kn) == -EEXIST)
			sysfs_warn_dup(parent, attr->name);
		return PTR_ERR(kn);
	}
	return 0;
}
```

```C
const struct kobj_ns_type_operations *kobj_child_ns_ops(struct kobject *parent)
{
        const struct kobj_ns_type_operations *ops = NULL;

        if (parent && parent->ktype && parent->ktype->child_ns_type)
                ops = parent->ktype->child_ns_type(parent);

        return ops;
}
```







