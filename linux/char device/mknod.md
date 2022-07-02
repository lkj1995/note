- 在/dev/中创建一个设备文件，并初始化文件的inode
- 主要做的事

```c
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

```C
 const struct file_operations def_chr_fops = {
         .open = chrdev_open,
         .llseek = noop_llseek,
 };
```
#### 当用户空间执行open的时候，主要做了如下操作
- 首先，创建了对应的dev/xxx ，即创建了一个inode

- 根据参数，记录dev_t设备号

- 因class为字符类，f_op指向def_chr_fops的通用操作

- cdev默认为空，为后续open作记录填充

  

- 用户进程调用open(dev/xxx)，获取了一个inode

- 获取inode的cdev

- 如果是第一次打开，cdev为空

  - 则会进入通用的字符open函数处理
  - 根据设备号在cdev_map（kobj_map）中寻找符合的kobj
  - 然后再用container_of获取cdev
  - 上自旋锁，再检查inode->cdev指针，是否有其他用户在此过程操作了open
    - 如果没有人操作，则inode->cdev记录找到的cdev
  
- 如果是多次打开

  - 则可以直接调用open

- 解除自旋锁，调用cdev_put，减少kobj引用次数

- inode->cdev复制给file->cdev

- 判断open函数不为空，则执行驱动实际的open操作

- 结束操作


```C
/*
 * Called every time a character special file is opened
 */
static int chrdev_open(struct inode *inode, struct file *filp)
{
	const struct file_operations *fops;
	struct cdev *p;
	struct cdev *new = NULL;
	int ret = 0;

	spin_lock(&cdev_lock);/*上锁*/
	p = inode->i_cdev; /*获取cdev，此时cdev是为空的，因为inode还没记录到*/
	if (!p) {
		struct kobject *kobj;
		int idx;
		spin_unlock(&cdev_lock);
		kobj = kobj_lookup(cdev_map, inode->i_rdev, &idx);/*通过设备号寻找对应的kobj*/
		if (!kobj)
			return -ENXIO;
		new = container_of(kobj, struct cdev, kobj);/*根据kobj找回结构体cdev*/
		spin_lock(&cdev_lock);
		/* Check i_cdev again in case somebody beat us to it while
		   we dropped the lock. */
		p = inode->i_cdev;/*防止解锁时，其他竞争者操作了它，如open，此时可能cdev已经被记录了*/
		if (!p) { /*无竞争者，记录找到的cdev*/
			inode->i_cdev = p = new;
             /*把cdev加入inode链表，是不是代表着还会有其他cdev共用一个inode?*/
			list_add(&inode->i_devices, &p->list);
			new = NULL;
		} else if (!cdev_get(p)) /*有竞争者，不知道啥意思。。*/
			ret = -ENXIO;
	} else if (!cdev_get(p))
		ret = -ENXIO;
	spin_unlock(&cdev_lock);
	cdev_put(new);
	if (ret)
		return ret;

	ret = -ENXIO;
	fops = fops_get(p->ops); /*获取cdev的ops*/
	if (!fops)
		goto out_cdev_put;

	replace_fops(filp, fops);/*保存到file*/
	if (filp->f_op->open) {
		ret = filp->f_op->open(inode, filp); /*调用驱动开发者编写的open函数*/
		if (ret)
			goto out_cdev_put;
	}

	return 0;

 out_cdev_put:
	cdev_put(p);
	return ret;
}
```

```C
struct kobject *kobj_lookup(struct kobj_map *domain, dev_t dev, int *index)
{
	struct kobject *kobj;
	struct probe *p;
	unsigned long best = ~0UL;

retry:
	mutex_lock(domain->lock);/*上锁*/
	for (p = domain->probes[MAJOR(dev) % 255]; p; p = p->next) {
		struct kobject *(*probe)(dev_t, int *, void *); /*probe指针*/
		struct module *owner;  
		void *data; /*cdev指针*/

		if (p->dev > dev || p->dev + p->range - 1 < dev) /*dev大于当前要找的，继续遍历*/
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

