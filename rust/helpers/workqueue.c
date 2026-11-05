// SPDX-License-Identifier: GPL-2.0

#include <linux/sched/prio.h>
#include <linux/workqueue.h>

__rust_helper void rust_helper_init_work_with_key(struct work_struct *work,
						  work_func_t func,
						  bool onstack,
						  const char *name,
						  struct lock_class_key *key)
{
	__init_work(work, onstack);
	work->data = (atomic_long_t)WORK_DATA_INIT();
	lockdep_init_map(&work->lockdep_map, name, key, 0);
	INIT_LIST_HEAD(&work->entry);
	work->func = func;
}

__rust_helper struct workqueue_struct *
rust_helper_alloc_ordered_workqueue(const char *name, unsigned int flags)
{
	return alloc_ordered_workqueue("%s", flags, name);
}

__rust_helper struct workqueue_struct *
rust_helper_alloc_unbound_workqueue(const char *name, bool highpri)
{
	return alloc_workqueue("%s", WQ_UNBOUND | (highpri ? WQ_HIGHPRI : 0), 0, name);
}

__rust_helper int rust_helper_workqueue_restrict(struct workqueue_struct *wq,
						 const struct cpumask *cpus,
						 bool ordered, bool highpri)
{
	struct workqueue_attrs *attrs = alloc_workqueue_attrs();
	int ret;

	if (!attrs)
		return -ENOMEM;
	cpumask_copy(attrs->cpumask, cpus);
	attrs->nice = highpri ? MIN_NICE : 0;
	attrs->ordered = ordered;
	ret = apply_workqueue_attrs(wq, attrs);
	free_workqueue_attrs(attrs);
	return ret;
}
