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
    - 如果有操作，则增加kobj引用次数

- 如果是多次打开

  - 增加kobj引用次数

- 解除自旋锁，调用cdev_put，减少kobj引用次数

- inode->cdev复制给file->cdev

- 判断open函数不为空，则执行驱动实际的open操作。

- 结束。。。

