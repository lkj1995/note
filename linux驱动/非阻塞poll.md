# 非阻塞poll

```c
struct pollfd {
         int fd;        //指定某个驱动file
         short events;  //选择想要的事件
         short revents; //返回发生的事件
  };
```

```c
asmlinkage long sys_poll(struct pollfd __user *ufds, unsigned int nfds,long timeout_msecs)
{
        if (timeout_msecs > 0)    //参数timeout>0
　　　　{
　　       timeout_jiffies = msecs_to_jiffies(timeout_msecs);  
           //通过频率来计算timeout时间需要多少计数值
　　　　}
　　　　else
　　　　{
          timeout_jiffies = timeout_msecs;    
          //如果timeout时间为0,直接赋值
       }
  return do_sys_poll(ufds, nfds, &timeout_jiffies);   //调用do_sys_poll
}
```

```c
#define N_STACK_PPS ((sizeof(stack_pps) - sizeof(struct poll_list))  / \
			sizeof(struct pollfd))

static int do_sys_poll(struct pollfd __user *ufds, unsigned int nfds,
		struct timespec64 *end_time)
{
	struct poll_wqueues table;
	int err = -EFAULT, fdcount, len;
	/* Allocate small arguments on the stack to save memory and be
	   faster - use long to make sure the buffer is aligned properly
	   on 64 bit archs to avoid unaligned access */
	long stack_pps[POLL_STACK_ALLOC/sizeof(long)];
	struct poll_list *const head = (struct poll_list *)stack_pps;
 	struct poll_list *walk = head;
 	unsigned long todo = nfds;

	if (nfds > rlimit(RLIMIT_NOFILE))
		return -EINVAL;

	len = min_t(unsigned int, nfds, N_STACK_PPS);
	for (;;) {
		walk->next = NULL;
		walk->len = len;
		if (!len)
			break;

		if (copy_from_user(walk->entries, ufds + nfds-todo,
					sizeof(struct pollfd) * walk->len))
			goto out_fds;

		todo -= walk->len;
		if (!todo)
			break;

		len = min(todo, POLLFD_PER_PAGE);
		walk = walk->next = kmalloc(struct_size(walk, entries, len),
					    GFP_KERNEL);
		if (!walk) {
			err = -ENOMEM;
			goto out_fds;
		}
	}

	poll_initwait(&table);  // 初始化table  table->pt->qproc = __pollwait;
	fdcount = do_poll(head, &table, end_time);
	poll_freewait(&table);

	for (walk = head; walk; walk = walk->next) {
		struct pollfd *fds = walk->entries;
		int j;

		for (j = 0; j < walk->len; j++, ufds++)
			if (__put_user(fds[j].revents, &ufds->revents))
				goto out_fds;
  	}

	err = fdcount;
out_fds:
	walk = head->next;
	while (walk) {
		struct poll_list *pos = walk;
		walk = walk->next;
		kfree(pos);
	}

	return err;
}
```

```c
 void poll_initwait(struct poll_wqueues *pwq)
 {
         init_poll_funcptr(&pwq->pt, __pollwait);
         pwq->polling_task = current;
         pwq->triggered = 0;
         pwq->error = 0;
         pwq->table = NULL;
         pwq->inline_index = 0;
 }
```

```c
static int do_poll(struct poll_list *list, struct poll_wqueues *wait,
		   struct timespec64 *end_time)
{
	poll_table* pt = &wait->pt;
	ktime_t expire, *to = NULL;
	int timed_out = 0, count = 0;
	u64 slack = 0;
	__poll_t busy_flag = net_busy_loop_on() ? POLL_BUSY_LOOP : 0;
	unsigned long busy_start = 0;

	/* Optimise the no-wait case */
	if (end_time && !end_time->tv_sec && !end_time->tv_nsec) {
		pt->_qproc = NULL;
		timed_out = 1;
	}

	if (end_time && !timed_out)
		slack = select_estimate_accuracy(end_time);

	for (;;) {
		struct poll_list *walk;
		bool can_busy_loop = false;

		for (walk = list; walk != NULL; walk = walk->next) {
			struct pollfd * pfd, * pfd_end;

			pfd = walk->entries;
			pfd_end = pfd + walk->len;
			for (; pfd != pfd_end; pfd++) {
				/*
				 * Fish for events. If we found one, record it
				 * and kill poll_table->_qproc, so we don't
				 * needlessly register any other waiters after
				 * this. They'll get immediately deregistered
				 * when we break out and return.
				 */
				if (do_pollfd(pfd, pt, &can_busy_loop,
					      busy_flag)) {  // 把该进程挂入poll_wait指定的队列里
					count++;     //如果返回非0值，则说明event被触发，则需返回
					pt->_qproc = NULL;
					/* found something, stop busy polling */
					busy_flag = 0;
					can_busy_loop = false;
				}
			}
		}
		/*
		 * All waiters have already been registered, so don't provide
		 * a poll_table->_qproc to them on the next loop iteration.
		 */
		pt->_qproc = NULL;
		if (!count) {
			count = wait->error;
			if (signal_pending(current))  //被信号打断，则返回负值
				count = -ERESTARTNOHAND;
		}
		if (count || timed_out) // 超时或者设备有数据要返回给应用层
			break;

		/* only if found POLL_BUSY_LOOP sockets && not out of time */
		if (can_busy_loop && !need_resched()) {
			if (!busy_start) {
				busy_start = busy_loop_current_time();
				continue;
			}
			if (!busy_loop_timeout(busy_start))
				continue;
		}
		busy_flag = 0;

		/*
		 * If this is the first loop and we have a timeout
		 * given, then we convert to ktime_t and set the to
		 * pointer to the expiry value.
		 */
		if (end_time && !to) {
			expire = timespec64_to_ktime(*end_time);
			to = &expire;
		}
        //暂无数据返回，让进程指定时间值休眠
		if (!poll_schedule_timeout(wait, TASK_INTERRUPTIBLE, to, slack))
			timed_out = 1;
	}
	return count;
}
```

```c
#define POLLFD_PER_PAGE  ((PAGE_SIZE-sizeof(struct poll_list)) / sizeof(struct pollfd))

/*
 * Fish for pollable events on the pollfd->fd file descriptor. We're only
 * interested in events matching the pollfd->events mask, and the result
 * matching that mask is both recorded in pollfd->revents and returned. The
 * pwait poll_table will be used by the fd-provided poll handler for waiting,
 * if pwait->_qproc is non-NULL.
 */
static inline __poll_t do_pollfd(struct pollfd *pollfd, poll_table *pwait,
				     bool *can_busy_poll,
				     __poll_t busy_flag)
{
	int fd = pollfd->fd;
	__poll_t mask = 0, filter;
	struct fd f;

	if (fd < 0)
		goto out;
	mask = EPOLLNVAL;
	f = fdget(fd);
	if (!f.file)
		goto out;

	/* userland u16 ->events contains POLL... bitmap */
	filter = demangle_poll(pollfd->events) | EPOLLERR | EPOLLHUP;
	pwait->_key = filter | busy_flag;  
	mask = vfs_poll(f.file, pwait); //调用驱动里的poll函数，即开发者添加的xxx_poll函数
	if (mask & busy_flag)           //如果驱动有数据让应用层读的话，就返回非0, 否侧返回0
		*can_busy_poll = true;      //至少有一个，让应用层感兴趣的event发生
	mask &= filter;		/* Mask out unneeded events. 屏蔽不需要的event */ 
	fdput(f);

out:
	/* ... and so does ->revents */
	pollfd->revents = mangle_poll(mask);
	return mask;
}
```

```c
 static int poll_schedule_timeout(struct poll_wqueues *pwq, int state,
                          ktime_t *expires, unsigned long slack)
{
        int rc = -EINTR;

        set_current_state(state);
        if (!pwq->triggered)
                rc = schedule_hrtimeout_range(expires, slack, HRTIMER_MODE_ABS);
        __set_current_state(TASK_RUNNING);

        /*
         * Prepare for the next iteration.
         *
         * The following smp_store_mb() serves two purposes.  First, it's
         * the counterpart rmb of the wmb in pollwake() such that data
         * written before wake up is always visible after wake up.
         * Second, the full barrier guarantees that triggered clearing
         * doesn't pass event check of the next iteration.  Note that
         * this problem doesn't exist for the first iteration as
         * add_wait_queue() has full barrier semantics.
         */
        smp_store_mb(pwq->triggered, 0);

        return rc;
}
```

```c
/* Add a new entry */
static void __pollwait(struct file *filp, wait_queue_head_t *wait_address,
                                poll_table *p)
{
        struct poll_wqueues *pwq = container_of(p, struct poll_wqueues, pt);
        //根据table项找到该头部，即其队列
        struct poll_table_entry *entry = poll_get_entry(pwq);
        if (!entry)
            return;
        entry->filp = get_file(filp);
        entry->wait_address = wait_address;
        entry->key = p->_key;
        init_waitqueue_func_entry(&entry->wait, pollwake);
        entry->wait.private = pwq;
        add_wait_queue(wait_address, &entry->wait); //把wait_address加入队列
}
```

#### 网上总结

1. poll > sys_poll > do_sys_poll > poll_initwait，poll_initwait函数注册一下回调函数__pollwait，它就是我们的驱2. 动程序执行poll_wait时，真正被调用的函数。

2. 接下来执行file->f_op->poll，即我们驱动程序里自己实现的poll函数。它会调用poll_wait把自己挂入某个队列，这个队列也是我们的驱动自己定义的。

   它还判断一下设备是否就绪。

3. 如果设备未就绪，do_sys_poll里会让进程休眠一定时间。

4. 进程被唤醒的条件有两个 :1、“一定时间”到了 ；2、二是被驱动程序唤醒。驱动程序发现条件就绪时，就把“某个队列”上挂着的进程唤醒，这个队列，就是前面通过poll_wait把本进程挂过去的队列。

   如果驱动程序没有去唤醒进程，那么chedule_timeout(__timeou)超时后，会重复2、3动作，直到应用程序的poll调用传入的时间到达。

#### 自我总结

1. 经过两天的研究，好像知道了那么一回事。

2. 首先是应用层调用poll机制，包含了感兴趣的event，fd，超时时间的参数传入。

3. 然后进行调用

- sys_poll(计算按jiffes时间为单位的次数，传入do_sys_poll) 。

- do_sys_poll(初始化指向了__poll_wait，并执行do_poll)。

- do_poll(轮询执行驱动层自定义的xxx_poll函数)，等待返回值。

  - 无event

    - 进行短休眠（即 短休眠 * n = 应用层要求超时时间），短休眠时间到达，被唤醒（唤醒的队列包含应用层的进程，或者还有驱动层的进程，应该驱动层的进程先执行），再进入xxx_poll。

  - 有event。

    - 提取出感兴趣部分，返回给应用层，结束休眠。

    - 如果提取不到感兴趣的，进行短休眠，休眠结束后，再进入xxx_poll。

- xxx_poll(执行了poll_wait函数，此时指向了__poll_wait) 

- __poll_wait(把本进程挂入在驱动层定义的队列) 

- 返回后，if判断条件是否达到，记录event类型，返回到 3-3
