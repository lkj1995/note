### Notifier chains 

```
/*
 * Notifier chains are of four types:
 *
 *	Atomic notifier chains: Chain callbacks run in interrupt/atomic
 *		context. Callouts are not allowed to block.
 *	Blocking notifier chains: Chain callbacks run in process context.
 *		Callouts are allowed to block.
 *	Raw notifier chains: There are no restrictions on callbacks,
 *		registration, or unregistration.  All locking and protection
 *		must be provided by the caller.
 *	SRCU notifier chains: A variant of blocking notifier chains, with
 *		the same restrictions.
```

- 三种类型的通知链