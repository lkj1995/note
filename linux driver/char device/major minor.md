### struct char_device_struct

```C
/*次设备号长度*/
#define MINORBITS 20
#define MINORMASK ((1U << MINORBITS) - 1)

/*获取主设备号*/
#define MAJOR(dev) ((unsigned int) ((dev) >> MINORBITS))

/*获取次设备号*/
#define MINOR(dev) ((unsigned int) ((dev) & MINORMASK))

/*合成设备号*/
#define MKDEV(ma,mi) (((ma) << MINORBITS) | (mi))  

#define CHRDEV_MAJOR_HASH_SIZE 255
  
static struct char_device_struct {
    
		struct char_device_struct *next;
    		
    	/*主设备号*/
		unsigned int major;		
    
    	/*次设备号*/
		unsigned int baseminor;
    
    	/*次设备号数量*/
		int minorct;
    
    	/*名称*/
		char name[64];
		struct cdev *cdev;              /* will die */
} *chrdevs[CHRDEV_MAJOR_HASH_SIZE];
```

### 1 register_chrdev_region()

```C
/**
 * register_chrdev_region() - register a range of device numbers
 * @from: the first in the desired range of device numbers; must include
 *        the major number.
 * @count: the number of consecutive device numbers required
 * @name: the name of the device or driver.
 *
 * Return value is zero on success, a negative error code on failure.
 */
int register_chrdev_region(dev_t from, unsigned count, const char *name)
{
    /*
     * from：主设备号
     * count：申请的数量
     * name：设备号命名
     */
	struct char_device_struct *cd;
	dev_t to = from + count;
	dev_t n, next;

    /*为每个主设备号+数量，合成的设备号申请位置*/
	for (n = from; n < to; n = next) {
		next = MKDEV(MAJOR(n)+1, 0);
		if (next > to)
			next = to;
		cd = __register_chrdev_region(MAJOR(n), MINOR(n),
			       next - n, name);
		if (IS_ERR(cd))
			goto fail;
	}
	return 0;
fail:
	to = n;
	for (n = from; n < to; n = next) {
		next = MKDEV(MAJOR(n)+1, 0);
		kfree(__unregister_chrdev_region(MAJOR(n), MINOR(n), next - n));
	}
	return PTR_ERR(cd);
}

```

### 1-1 __register_chrdev_region()

```C

/*
 * Register a single major with a specified minor range.
 *
 * If major == 0 this functions will dynamically allocate a major and return
 * its number.
 *
 * If major > 0 this function will attempt to reserve the passed range of
 * minors and will return zero on success.
 *
 * Returns a -ve errno on failure.
 */

/* 
 * major为0，动态申请设备号。 
 * major大于0，根据传入的‘设备号’和‘次设备号数量’参数，静态申请设备号。
 */
static struct char_device_struct *
__register_chrdev_region(unsigned int major, unsigned int baseminor,
			   int minorct, const char *name)
{
	struct char_device_struct *cd, **cp;
	int ret = 0;
	int i;
	
    /*申请 char_device_struct 内存*/
	cd = kzalloc(sizeof(struct char_device_struct), GFP_KERNEL);
	if (cd == NULL)
		return ERR_PTR(-ENOMEM);

	mutex_lock(&chrdevs_lock);

	/* temporary */
    /*动态分配*/
	if (major == 0) {
		for (i = ARRAY_SIZE(chrdevs)-1; i > 0; i--) {
			/*查找存放指针的全局数组，找到空的位置，从倒序遍历*/
            if (chrdevs[i] == NULL)
				break;
		}

		if (i < CHRDEV_MAJOR_DYN_END)
			pr_warn("CHRDEV \"%s\" major number %d goes below the dynamic allocation range\n",
				name, i);
		
        /*没空位了，返回*/
		if (i == 0) {
			ret = -EBUSY;
			goto out;
		}
        /*记录刚找到的空指针索引*/
		major = i;
	}
	
    /*记录主设备号，次设备号，数量，名字*/
	cd->major = major;
	cd->baseminor = baseminor;
	cd->minorct = minorct;
	strlcpy(cd->name, name, sizeof(cd->name));

    /*
     * return major % CHRDEV_MAJOR_HASH_SIZE; 
     */
	i = major_to_index(major);

	for (cp = &chrdevs[i]; *cp; cp = &(*cp)->next)
        /*
         * 1. major小于当前遍历的major，可以插入该位置。
         * 2. major等于当前遍历的major，确认minor范围，
         *  --(minor]-------------------------
         *  --------(list minor range]--------
         *  -------------------------(minor]--  
         * 在该minor的前面或者后面,留意开区间和闭区间。
         */
		if ((*cp)->major > major ||
		    ((*cp)->major == major &&
		     (((*cp)->baseminor >= baseminor) ||
		      ((*cp)->baseminor + (*cp)->minorct > baseminor))))
			break;

	/* Check for overlapping minor ranges.  */
    /*检查 minor 申请的数量范围是否越界 */
	if (*cp && (*cp)->major == major) {
		int old_min = (*cp)->baseminor;
		int old_max = (*cp)->baseminor + (*cp)->minorct - 1;
		int new_min = baseminor;
		int new_max = baseminor + minorct - 1;

		/* New driver overlaps from the left.  */
		if (new_max >= old_min && new_max <= old_max) {
			ret = -EBUSY;
			goto out;
		}

		/* New driver overlaps from the right.  */
		if (new_min <= old_max && new_min >= old_min) {
			ret = -EBUSY;
			goto out;
		}
	}
    
	/*找到合适的，记录在list后面*/
	cd->next = *cp;
	*cp = cd;
	mutex_unlock(&chrdevs_lock);
	return cd;
out:
	mutex_unlock(&chrdevs_lock);
	kfree(cd);
	return ERR_PTR(ret);
}

```

