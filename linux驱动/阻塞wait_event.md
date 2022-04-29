# 阻塞wait_event

```c
struct list_head {
    struct list_head *next, *prev;
};
```

```c
struct wait_queue_head {
       spinlock_t              lock;
        struct list_head        head;
};
typedef struct wait_queue_head wait_queue_head_t;
```

```C
/**
* wait_event - sleep until a condition gets true
* @wq_head: the waitqueue to wait on
* @condition: a C expression for the event to wait for
*
* The process is put to sleep (TASK_UNINTERRUPTIBLE) until the
* @condition evaluates to true. The @condition is checked each time
* the waitqueue @wq_head is woken up.
*
* wake_up() has to be called after changing any variable that could
* change the result of the wait condition.
*/
#define wait_event(wq_head, condition)                                          \
do {                                                                            \
        might_sleep();                                                          \
       if (condition)                                                          \
                break;                                                          \
        __wait_event(wq_head, condition);                                       \
} while (0)
```

```c
/*
 * Note: we use "set_current_state()" _after_ the wait-queue add,
 * because we need a memory barrier there on SMP, so that any
 * wake-function that tests for the wait-queue being active
 * will be guaranteed to see waitqueue addition _or_ subsequent
 * tests in this thread will see the wakeup having taken place.
 *
 * The spin_unlock() itself is semi-permeable and only protects
 * one way (it only protects stuff inside the critical region and
 * stops them from bleeding out - it would still allow subsequent
 * loads to move into the critical region).
 */
void
prepare_to_wait(struct wait_queue_head *wq_head, struct wait_queue_entry *wq_entry, int state)
 {
        unsigned long flags;

        wq_entry->flags &= ~WQ_FLAG_EXCLUSIVE;
        spin_lock_irqsave(&wq_head->lock, flags);      //自旋锁
        if (list_empty(&wq_entry->entry))       
                __add_wait_queue(wq_head, wq_entry);
        set_current_state(state);
        spin_unlock_irqrestore(&wq_head->lock, flags); //自旋锁
}
EXPORT_SYMBOL(prepare_to_wait); //外部符号

static inline void __add_wait_queue(struct wait_queue_head *wq_head, struct wait_queue_entry *wq_entry)
{
        list_add(&wq_entry->entry, &wq_head->head);
}
```

```C
/**
 * finish_wait - clean up after waiting in a queue
 * @wq_head: waitqueue waited on
 * @wq_entry: wait descriptor
 *
 * Sets current thread back to running state and removes
 * the wait descriptor from the given waitqueue if still
 * queued.
 */
void finish_wait(struct wait_queue_head *wq_head, struct wait_queue_entry *wq_entry)
{
        unsigned long flags;

        __set_current_state(TASK_RUNNING);
        /*
         * We can check for list emptiness outside the lock
         * IFF:
         *  - we use the "careful" check that verifies both
         *    the next and prev pointers, so that there cannot
         *    be any half-pending updates in progress on other
         *    CPU's that we haven't seen yet (and that might
         *    still change the stack area.
         * and
         *  - all other users take the lock (ie we can only
         *    have _one_ other CPU that looks at or modifies
         *    the list).
         */
        if (!list_empty_careful(&wq_entry->entry)) {
                spin_lock_irqsave(&wq_head->lock, flags);
                list_del_init(&wq_entry->entry); //从队列头中删除entry
                spin_unlock_irqrestore(&wq_head->lock, flags);
        }
 }
 EXPORT_SYMBOL(finish_wait);
```

```C
void
prepare_to_wait_exclusive(struct wait_queue_head *wq_head, struct wait_queue_entry *wq_entry, int state)
{
        unsigned long flags;

        wq_entry->flags |= WQ_FLAG_EXCLUSIVE; //设置标志位 WQ_FLAG_EXCLUSIVE
        spin_lock_irqsave(&wq_head->lock, flags);
        if (list_empty(&wq_entry->entry))
                __add_wait_queue_entry_tail(wq_head, wq_entry);
        set_current_state(state);
        spin_unlock_irqrestore(&wq_head->lock, flags);
}
```

```
static void __wake_up_common_lock(struct wait_queue_head *wq_head, unsigned int mode,
                        int nr_exclusive, int wake_flags, void *key)
{
        unsigned long flags;
        wait_queue_entry_t bookmark;

        bookmark.flags = 0;
        bookmark.private = NULL;
        bookmark.func = NULL;
        INIT_LIST_HEAD(&bookmark.entry);

        do {
                spin_lock_irqsave(&wq_head->lock, flags);
                nr_exclusive = __wake_up_common(wq_head, mode, nr_exclusive,
                                                wake_flags, key, &bookmark);
                spin_unlock_irqrestore(&wq_head->lock, flags);
        } while (bookmark.flags & WQ_FLAG_BOOKMARK);
}

/**
* __wake_up - wake up threads blocked on a waitqueue.
* @wq_head: the waitqueue
* @mode: which threads
* @nr_exclusive: how many wake-one or wake-many threads to wake up
* @key: is directly passed to the wakeup function
*
* If this function wakes up a task, it executes a full memory barrier before
* accessing the task state.
*/
void __wake_up(struct wait_queue_head *wq_head, unsigned int mode,
                        int nr_exclusive, void *key)
{
        __wake_up_common_lock(wq_head, mode, nr_exclusive, 0, key);
}

#define wake_up(x)                      __wake_up(x, TASK_NORMAL, 1, NULL)
#define wake_up_nr(x, nr)               __wake_up(x, TASK_NORMAL, nr, NULL)
#define wake_up_all(x)                  __wake_up(x, TASK_NORMAL, 0, NULL)


#define wake_up_interruptible(x)        __wake_up(x, TASK_INTERRUPTIBLE, 1, NULL)
#define wake_up_interruptible_nr(x, nr) __wake_up(x, TASK_INTERRUPTIBLE, nr, NULL)
#define wake_up_interruptible_all(x)    __wake_up(x, TASK_INTERRUPTIBLE, 0, NULL)
```



```c
//检查条件不符合后，将entry项插入队列，并设置为interruptable。
//调度之前，一定要再次检查条件是否已经发生，如果条件尚未发生，则可进行调度。
//当条件发生了，则需要队列中删除entry，并设置状态回running，否则将无法唤醒。
//当在if和调度之间发生，则不影响，因为调度器会正确的设置好。
```

```c
//防止wake_up调用后，等待队列中进程数量很庞大，则会影响性能，
//使用WQ_FLAG_EXCLUSIVE标志，则只会唤醒第一个具有该标志的进程
```

