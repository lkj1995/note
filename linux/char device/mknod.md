### init_special_inode()

```c
/*在/dev/中创建一个设备文件，并初始化文件的inode*/

void init_special_inode(struct inode *inode, umode_t mode, dev_t rdev)
{
        inode->i_mode = mode;
        if (S_ISCHR(mode)) {
                inode->i_fop = &def_chr_fops; /*字符设备的通用操作，如下说明*/
                inode->i_rdev = rdev;/*记录设备号*/
        } else if (S_ISBLK(mode)) {
                inode->i_fop = &def_blk_fops;
                inode->i_rdev = rdev;
        } else if (S_ISFIFO(mode))
                inode->i_fop = &pipefifo_fops;
        else if (S_ISSOCK(mode))
                ;       /* leave it no_open_fops */
        else
                printk(KERN_DEBUG "init_special_inode: bogus i_mode (%o) for"
                                  " inode %s:%lu\n", mode, inode->i_sb->s_id,
                                  inode->i_ino);
}
EXPORT_SYMBOL(init_special_inode);
```

### 1 chrdev_open()

```C
 const struct file_operations def_chr_fops = {
         .open = chrdev_open,
         .llseek = noop_llseek,
 };


/*
 * Called every time a character special file is opened
 */
static int chrdev_open(struct inode *inode, struct file *filp)
{
	const struct file_operations *fops;
	struct cdev *p;
	struct cdev *new = NULL;
	int ret = 0;

	spin_lock(&cdev_lock);
    
    /*第一次inode未记录，cdev是为空*/
	p = inode->i_cdev; 
	if (!p) {
		struct kobject *kobj;
		int idx;
		spin_unlock(&cdev_lock);
        
        /*通过设备号寻找对应的kobj*/
		kobj = kobj_lookup(cdev_map, inode->i_rdev, &idx);
        
		if (!kobj)
			return -ENXIO;
        
        /*根据kobj找到cdev*/
		new = container_of(kobj, struct cdev, kobj);
		spin_lock(&cdev_lock);
        
		/* Check i_cdev again in case somebody beat us to it while
		   we dropped the lock. */
        /*防止解锁时，其他竞争者操作它，如open，此时可能cdev已经被记录了*/
		p = inode->i_cdev;
        
        
        /*无竞争者，记录找到的cdev*/
		if (!p) { 
			inode->i_cdev = p = new;
            
             /*把cdev加入inode链表，是不是代表着还会有其他cdev共用一个inode?*/
			list_add(&inode->i_devices, &p->list);
			new = NULL;
            
		} else if (!cdev_get(p))
			ret = -ENXIO;
	} else if (!cdev_get(p))
		ret = -ENXIO;
	spin_unlock(&cdev_lock);
	cdev_put(new);
	if (ret)
		return ret;

	ret = -ENXIO;
    
    /*获取cdev的ops*/
	fops = fops_get(p->ops);
	if (!fops)
		goto out_cdev_put;
    
	/*保存到file，方便接下来使用*/
	replace_fops(filp, fops);
	if (filp->f_op->open) {
        
        /*实际调用用户的open函数*/
		ret = filp->f_op->open(inode, filp); 
		if (ret)
			goto out_cdev_put;
	}

	return 0;

 out_cdev_put:
	cdev_put(p);
	return ret;
}
```

### 1-1 kobj_lookup()

```C
struct kobject *kobj_lookup(struct kobj_map *domain, dev_t dev, int *index)
{
	struct kobject *kobj;
	struct probe *p;
	unsigned long best = ~0UL;

retry:
	mutex_lock(domain->lock);
	for (p = domain->probes[MAJOR(dev) % 255]; p; p = p->next) {
		struct kobject *(*probe)(dev_t, int *, void *);
		struct module *owner; 
        /*cdev指针*/
		void *data; 
        
		/*dev大于当前要找的，继续遍历*/
		if (p->dev > dev || p->dev + p->range - 1 < dev) 
			continue;
		if (p->range - 1 >= best)
			break;
		if (!try_module_get(p->owner))
			continue;
		owner = p->owner;
		data = p->data;
		probe = p->get;
		best = p->range - 1;
		*index = dev - p->dev;
		if (p->lock && p->lock(dev, data) < 0) {
			module_put(owner);
			continue;
		}
		mutex_unlock(domain->lock);
		kobj = probe(dev, index, data);
		/* Currently ->owner protects _only_ ->probe() itself. */
		module_put(owner);
		if (kobj)
			return kobj;
		goto retry;
	}
	mutex_unlock(domain->lock);
	return NULL;
}
```

### 总结

- 用户空间执行open，创建了一个inode。
- 因class为字符设备，f_op选择了字符类型的def_chr_fops
- 先进入通用的chrdev_open进行处理
- 如果第一次打开，则根据设备号找到cdev，并记录在inode。
- 如果已经打开过，直接使用。
- inode提供全局的进程使用，file则是当前进程使用。
- 最终执行用户的open函数。