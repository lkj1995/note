### 数据结构

```C
  #define CHRDEV_MAJOR_HASH_SIZE 255
  
  static struct char_device_struct {
          struct char_device_struct *next;
          unsigned int major;/*主设备号*/
          unsigned int baseminor;/*次设备号*/
          int minorct;/*次设备号数量*/
          char name[64];/*名称*/
          struct cdev *cdev;              /* will die */
  } *chrdevs[CHRDEV_MAJOR_HASH_SIZE];
```

- 整体为一个数组，每个元素为链表展开
- 用于申请和记录设备号
- 区分为动态申请设备号和自定设备号申请

### 申请设备号过程

```c
/*
 * Register a single major with a specified minor range.
 *
 * If major == 0 this function will dynamically allocate an unused major.
 * 如果major为0，则动态分配未使用的设备号
 * If major > 0 this function will attempt to reserve the range of minors
 * 如果major大于0，则根据传入指定的‘设备号’和‘次设备号数量’参数进行申请
 * with given major.
 *
 */
static struct char_device_struct * 
__register_chrdev_region(unsigned int major, unsigned int baseminor,
			   int minorct, const char *name)
{
	struct char_device_struct *cd, *curr, *prev = NULL;
	int ret;
	int i;
     /*如主设备号大于宏返回错误，实际没有这么2^10这么大，有限制*/
	if (major >= CHRDEV_MAJOR_MAX) {
		pr_err("CHRDEV \"%s\" major requested (%u) is greater than the maximum (%u)\n",
		       name, major, CHRDEV_MAJOR_MAX-1);
		return ERR_PTR(-EINVAL);
	}
     /*即要在[baseminor,255]之内的范围数量*/
	if (minorct > MINORMASK + 1 - baseminor) {
		pr_err("CHRDEV \"%s\" minor range requested (%u-%u) is out of range of maximum range (%u-%u) for a single major\n",
			name, baseminor, baseminor + minorct - 1, 0, MINORMASK);
		return ERR_PTR(-EINVAL);
	}
    /*申请char_device_struct结构体内存*/
	cd = kzalloc(sizeof(struct char_device_struct), GFP_KERNEL);
	if (cd == NULL)
		return ERR_PTR(-ENOMEM);

	mutex_lock(&chrdevs_lock);
    /*动态分配*/
	if (major == 0) {
		/*
		 * 先在[255，234]之间找空的next，如寻找失败，进行下一步。
		 * 在511~384内，数值经hash处理后寻找，即 x % 255，
		 * 遍历每个元素的链表，确认其内不存在重复的主设备号。
		 * 如 chrdevs[511 % 255].major 不能等于 511。
		*/
		ret = find_dynamic_major();
		if (ret < 0) {
			pr_err("CHRDEV \"%s\" dynamic allocation region is full\n",
			       name);
			goto out;
		}
		major = ret;
	}
    
	ret = -EBUSY;
	i = major_to_index(major);/*此时的major或是动态或是指定的*/
	for (curr = chrdevs[i]; curr; prev = curr, curr = curr->next) {
		if (curr->major < major)
			continue;

		if (curr->major > major)
			break;
         /*
          * 当curr->major = major会执行判断次设备号，指定的设备号才会出现相等的情况
          *  因major_to_index动态分配，不允许chrdevs[i]中存在相同的设备号
         */
		if (curr->baseminor + curr->minorct <= baseminor)
			continue; /*baseminor比链表的要大，还要往后遍历*/
         
		if (curr->baseminor >= baseminor + minorct)
			break; /*baseminor比链表的要小，而且申请数量小于curr->baseminor，无越界*/

		goto out;
	}
    /*记录主设备号，次设备号，数量，名字*/
	cd->major = major;
	cd->baseminor = baseminor;
	cd->minorct = minorct;
	strlcpy(cd->name, name, sizeof(cd->name));
    
	if (!prev) { /*如果只有0个或者1个*/
		cd->next = curr;
		chrdevs[i] = cd;
	} else {  /*其余情况*/
		cd->next = prev->next;
		prev->next = cd;
	}

	mutex_unlock(&chrdevs_lock);/*解锁*/
	return cd; /*结束，成功分配*/
out:
	mutex_unlock(&chrdevs_lock);
	kfree(cd);
	return ERR_PTR(ret);
}
```



