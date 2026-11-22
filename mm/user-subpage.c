// SPDX-License-Identifier: GPL-2.0-only
/* Reference-counted user data slots in native folios. */
#include <linux/highmem.h>
#include <linux/export.h>
#include <linux/mm.h>
#include <linux/memcontrol.h>
#include <linux/sched/mm.h>
#include <linux/mm_subpage.h>
#include <linux/mman.h>
#include <linux/security.h>
#include <linux/cpuset.h>
#include <linux/hash.h>
#include <linux/rhashtable.h>
#include <linux/page_ext.h>
#include <linux/rcupdate.h>
#include <linux/slab.h>
#include <linux/workqueue.h>

#include "user-subpage-internal.h"
#include "internal.h"

#define SUBPAGES_PER_FOLIO (PAGE_SIZE / MM_SUBPAGE_SIZE)
static bool subpage_shift_valid(unsigned int shift)
{
	return shift >= MM_SUBPAGE_SHIFT && shift < PAGE_SHIFT;
}

static unsigned long subpage_mask(unsigned int shift)
{
	return GENMASK((PAGE_SIZE >> shift) - 1, 0);
}

static unsigned long offset_mask(unsigned int shift, unsigned int offset)
{
	if (offset >= PAGE_SIZE || !IS_ALIGNED(offset, 1U << shift))
		return 0;
	return BIT(offset >> shift);
}

struct mm_subpage_owner {
	struct rcu_head rcu;
	struct work_struct free_work;
	struct mm_subpage_pool *pool;
	struct folio *folio;
	struct anon_vma *anon_root;
	struct mm_subpage_owner *migration_target;
	/* Allocated only when slots need independent logical identities. */
	struct mm_subpage_rmap *rmap;
	struct list_head all;
	struct list_head available;
	unsigned long used;
	unsigned long zeroed;
	u8 shift;
	bool detached;
	bool migrating;
	bool droppable;
	struct mm_subpage slots[SUBPAGES_PER_FOLIO];
};

struct mm_subpage_ext {
	struct mm_subpage_owner __rcu *owner;
	atomic_t file_maps[SUBPAGES_PER_FOLIO];
	atomic_t file_leaves;
	/* Folio lock protects validity; fits the 16K/64K layout padding. */
	u16 shmem_missing;
};
static_assert(SUBPAGES_PER_FOLIO == 1 || sizeof(struct mm_subpage_ext) ==
	      ALIGN(sizeof(void *) + sizeof(atomic_t) * (SUBPAGES_PER_FOLIO + 1),
		    sizeof(void *)));
static_assert(SUBPAGES_PER_FOLIO <= sizeof(u16) * BITS_PER_BYTE);

/*
 * The record is reserved only when this boot may create alternative-granule
 * mms (on arm64, unless booted with user4k=off). Otherwise page_ext may still exist for other
 * users, but mm_subpage_ext_ops has no offset in it and must not be read.
 */
static bool __init subpage_ext_needed(void)
{
	return arch_mm_alt_granule_enabled();
}

struct page_ext_operations mm_subpage_ext_ops = {
	.size = sizeof(struct mm_subpage_ext),
	.need = subpage_ext_needed,
};

static struct page_ext *subpage_ext_get(const struct page *page)
{
	return arch_mm_alt_granule_enabled() ? page_ext_get(page) : NULL;
}

static struct page_ext *subpage_ext_from_phys(phys_addr_t phys)
{
	return arch_mm_alt_granule_enabled() ? page_ext_from_phys(phys) : NULL;
}

unsigned long mm_subpage_shmem_missing(struct page *page)
{
	struct page_ext *ext;
	struct mm_subpage_ext *subext;
	unsigned long missing;

	/* Without alternative granules no 4K unit is ever missing. */
	if (!arch_mm_alt_granule_enabled())
		return 0;
	ext = subpage_ext_get(page);
	/* A referenced native page keeps its initialized section online. */
	VM_BUG_ON(!ext);
	subext = page_ext_data(ext, &mm_subpage_ext_ops);
	missing = subext->shmem_missing;
	page_ext_put(ext);
	return missing;
}

void mm_subpage_shmem_set_missing(struct page *page, unsigned long missing)
{
	struct page_ext *ext;
	struct mm_subpage_ext *subext;

	VM_BUG_ON(missing & ~GENMASK(SUBPAGES_PER_FOLIO - 1, 0));
	if (!arch_mm_alt_granule_enabled())
		return;
	ext = subpage_ext_get(page);
	VM_BUG_ON(!ext);
	subext = page_ext_data(ext, &mm_subpage_ext_ops);
	subext->shmem_missing = missing;
	page_ext_put(ext);
}

/* Only alternative PTE mappings contribute; native rmap counts stay unchanged. */
static void subpage_file_map_update(phys_addr_t phys, unsigned long size, int delta)
{
	struct page_ext *ext;
	struct mm_subpage_ext *subext;
	unsigned int index, nr, i;
	int count;

	VM_BUG_ON(!is_power_of_2(size) || size < MM_SUBPAGE_SIZE ||
		  size >= PAGE_SIZE || !IS_ALIGNED(phys, size));
	ext = subpage_ext_from_phys(phys);
	if (WARN_ON_ONCE(!ext))
		return;
	subext = page_ext_data(ext, &mm_subpage_ext_ops);
	index = offset_in_page(phys) >> MM_SUBPAGE_SHIFT;
	nr = size >> MM_SUBPAGE_SHIFT;
	for (i = index; i < index + nr; i++) {
		count = atomic_add_return(delta, &subext->file_maps[i]);
		VM_WARN_ON(count < 0);
	}
	/* Rmap counts one mapping, regardless of its coverage in 4K units. */
	count = atomic_add_return(delta, &subext->file_leaves);
	VM_WARN_ON(count < 0);
	page_ext_put(ext);
}

void __mm_subpage_file_map_add(phys_addr_t phys, unsigned long size)
{
	subpage_file_map_update(phys, size, 1);
}

void __mm_subpage_file_map_del(phys_addr_t phys, unsigned long size)
{
	subpage_file_map_update(phys, size, -1);
}

/* Snapshot for statistics only, never an ownership or pinning decision. */
unsigned int mm_subpage_file_mapcounts(struct page *page, struct mm_subpage_mapcounts *counts)
{
	struct page_ext *ext = subpage_ext_get(page);
	struct mm_subpage_ext *subext;
	unsigned int i, leaves;

	/* Without alternative granules there are no alternative leaves. */
	if (!ext) {
		WARN_ON_ONCE(arch_mm_alt_granule_enabled());
		memset(counts, 0, sizeof(*counts));
		return 0;
	}
	subext = page_ext_data(ext, &mm_subpage_ext_ops);
	for (i = 0; i < SUBPAGES_PER_FOLIO; i++)
		counts->slots[i] = max(atomic_read(&subext->file_maps[i]), 0);
	leaves = max(atomic_read(&subext->file_leaves), 0);
	page_ext_put(ext);
	return leaves;
}

static int owner_bind(struct mm_subpage_owner *owner)
{
	struct page_ext *ext = subpage_ext_get(&owner->folio->page);
	struct mm_subpage_ext *subext;
	int ret;

	if (!ext)
		return -EOPNOTSUPP;
	subext = page_ext_data(ext, &mm_subpage_ext_ops);
	ret = cmpxchg(&subext->owner, NULL, owner) ? -EEXIST : 0;
	page_ext_put(ext);
	return ret;
}

static void owner_unbind(struct mm_subpage_owner *owner)
{
	struct page_ext *ext = subpage_ext_get(&owner->folio->page);
	struct mm_subpage_ext *subext;

	/* The allocated, referenced folio prevents its section going offline. */
	VM_BUG_ON(!ext);
	subext = page_ext_data(ext, &mm_subpage_ext_ops);
	VM_BUG_ON(rcu_access_pointer(subext->owner) != owner);
	rcu_assign_pointer(subext->owner, NULL);
	page_ext_put(ext);
}

struct mm_subpage *mm_subpage_get_from_phys(phys_addr_t phys)
{
	struct mm_subpage_owner *owner;
	struct mm_subpage_ext *subext;
	struct mm_subpage *slot = NULL;
	struct page_ext *ext;
	unsigned int index;

	if (!IS_ALIGNED(phys, MM_SUBPAGE_SIZE))
		return NULL;
	ext = subpage_ext_from_phys(phys);
	if (!ext)
		return NULL;
	subext = page_ext_data(ext, &mm_subpage_ext_ops);
	owner = rcu_dereference(subext->owner);
	index = owner ? offset_in_page(phys) >> owner->shift : 0;
	/* Keep the native allocation alive before publishing another slot ref. */
	if (owner && folio_try_get(owner->folio)) {
		if (refcount_inc_not_zero_acquire(&owner->slots[index].refs))
			slot = &owner->slots[index];
		else
			folio_put(owner->folio);
	}
	page_ext_put(ext);
	return slot;
}

struct cow_pool_key {
	struct mem_cgroup *memcg;
	int nid;
	u32 bucket;
};

struct mm_subpage_pool {
	struct rcu_head rcu;
	spinlock_t lock;
	refcount_t refs;
	struct list_head owners;
	struct list_head available;
	struct mm_subpage_owner *unissued;
	unsigned long backing_pages;
	unsigned long all_slots;
	u8 shift;
	bool closed;
	/* Only a per-mm COW pool owns this charge-domain reference. */
	struct mem_cgroup *cow_memcg;
	int cow_nid;
	bool cow_shared;
	/* Allocated only for shared COW pools; protected by pool->lock. */
	unsigned long *cow_free_slots;
	struct cow_pool_key cow_key;
	struct rhash_head cow_hash;
	struct list_head cow_domain;
	/* Protected by this pool's lock, never used after registry detachment. */
	struct mm_subpage_cow_pools *cow_registry;
};

#define COW_POOL_BITS 4
struct mm_subpage_cow_pools {
	struct rhashtable domains;
	spinlock_t lock;
	struct list_head pools;
	refcount_t refs;
	struct rcu_head rcu;
	struct work_struct free_work;
};

static const struct rhashtable_params cow_domain_params = {
	.head_offset = offsetof(struct mm_subpage_pool, cow_hash),
	.key_offset = offsetof(struct mm_subpage_pool, cow_key),
	.key_len = sizeof(struct cow_pool_key),
	.nelem_hint = 16,
	.automatic_shrinking = true,
};

static void cow_registry_free_work(struct work_struct *work)
{
	struct mm_subpage_cow_pools *pools =
		container_of(work, struct mm_subpage_cow_pools, free_work);

	WARN_ON_ONCE(!list_empty(&pools->pools));
	rhashtable_destroy(&pools->domains);
	kfree(pools);
}

static void cow_registry_free_rcu(struct rcu_head *rcu)
{
	struct mm_subpage_cow_pools *pools =
		container_of(rcu, struct mm_subpage_cow_pools, rcu);

	queue_work(system_dfl_wq, &pools->free_work);
}

static void cow_registry_put(struct mm_subpage_cow_pools *pools)
{
	if (refcount_dec_and_test(&pools->refs))
		call_rcu(&pools->rcu, cow_registry_free_rcu);
}

static void pool_put_raw(struct mm_subpage_pool *pool);
static void owner_remove(struct mm_subpage_owner *owner);
static void owner_free(struct mm_subpage_owner *owner, bool put_folio);

/* The caller retains a pool reference independently of registry membership. */
static void cow_pool_detach_locked(struct mm_subpage_pool *pool)
{
	struct mm_subpage_cow_pools *pools = pool->cow_registry;

	lockdep_assert_held(&pool->lock);
	if (!pools)
		return;
	pool->cow_registry = NULL;
	/* A failed prospective insertion may not have entered the hash/list. */
	rhashtable_remove_fast(&pools->domains, &pool->cow_hash, cow_domain_params);
	spin_lock(&pools->lock);
	list_del_init(&pool->cow_domain);
	spin_unlock(&pools->lock);
	pool_put_raw(pool); /* Removed (or unpublished prospective) membership. */
	cow_registry_put(pools);
}

static void cow_pool_retire_empty(struct mm_subpage_pool *pool)
{
	struct mm_subpage_owner *owner = NULL;
	unsigned long flags;

	if (!pool->cow_shared)
		return;
	spin_lock_irqsave(&pool->lock, flags);
	if (!pool->backing_pages ||
	    (pool->backing_pages == 1 && pool->unissued)) {
		pool->closed = true;
		cow_pool_detach_locked(pool);
		/* A failed allocation may leave wholly unissued supply, not a
		 * live domain. Retire it under the same reservation lock.
		 */
		owner = pool->unissued;
		if (owner)
			owner_remove(owner);
	}
	spin_unlock_irqrestore(&pool->lock, flags);
	if (owner)
		owner_free(owner, true);
}

static void owner_init(struct mm_subpage_owner *owner, struct mm_subpage_pool *pool,
		       struct folio *folio)
{
	unsigned int i;

	owner->pool = pool;
	owner->shift = pool->shift;
	owner->folio = folio;
	INIT_LIST_HEAD(&owner->available);
	for (i = 0; i < (PAGE_SIZE >> pool->shift); i++) {
		owner->slots[i].owner = owner;
		owner->slots[i].index = i;
		owner->slots[i].shift = pool->shift;
		refcount_set(&owner->slots[i].refs, 0);
		atomic_set(&owner->slots[i].mapcount, 0);
	}
}

struct mm_subpage_pool *mm_subpage_pool_create_granule(gfp_t gfp, unsigned int shift)
{
	struct mm_subpage_pool *pool;

	BUILD_BUG_ON(PAGE_SHIFT <= MM_SUBPAGE_SHIFT);
	BUILD_BUG_ON(SUBPAGES_PER_FOLIO > BITS_PER_LONG);
	if (!subpage_shift_valid(shift))
		return NULL;
	pool = kzalloc(sizeof(*pool), gfp | __GFP_ACCOUNT);
	if (!pool)
		return NULL;
	pool->shift = shift;
	pool->all_slots = subpage_mask(shift);
	spin_lock_init(&pool->lock);
	refcount_set(&pool->refs, 1);
	INIT_LIST_HEAD(&pool->owners);
	INIT_LIST_HEAD(&pool->available);
	return pool;
}

struct mm_subpage_pool *mm_subpage_pool_create(gfp_t gfp)
{
	return mm_subpage_pool_create_granule(gfp, MM_SUBPAGE_SHIFT);
}

void mm_subpage_pool_get(struct mm_subpage_pool *pool)
{
	refcount_inc(&pool->refs);
}

static void pool_put_raw(struct mm_subpage_pool *pool)
{
	if (!refcount_dec_and_test(&pool->refs))
		return;
	WARN_ON_ONCE(!pool->closed || !list_empty(&pool->owners));
	mem_cgroup_put(pool->cow_memcg);
	kfree(pool->cow_free_slots);
	kfree_rcu(pool, rcu);
}

void mm_subpage_pool_put(struct mm_subpage_pool *pool)
{
	/* Failed allocations must not leave an empty per-domain cache behind. */
	cow_pool_retire_empty(pool);
	pool_put_raw(pool);
}

/* Per-mm pools retain each still-backed charge/node/shard domain separately. */
struct mm_subpage_pool *mm_subpage_cow_pool_get_charged(struct mm_struct *mm,
			struct mm_struct *charge_mm, struct vm_area_struct *vma,
			unsigned long address)
{
	struct mm_subpage_cow_pools *pools, *new_pools;
	struct mm_subpage_pool *pool, *new;
	struct cow_pool_key key = {};
	struct mem_cgroup *memcg;
	unsigned long flags;
	unsigned int attempt;
	void *result;
	int nid;

	if ((vma->vm_flags & (VM_LOCKED | VM_DROPPABLE | VM_SPECIAL)) ||
	    num_online_nodes() != 1 || num_node_state(N_MEMORY) != 1)
		return NULL;
#ifdef CONFIG_NUMA
	if (vma_policy(vma) || current->mempolicy)
		return NULL;
#endif
	nid = first_node(node_states[N_MEMORY]);
	if (!node_isset(nid, cpuset_current_mems_allowed))
		return NULL;
	/* Acquire initialization published by the mm's cmpxchg below. */
	pools = smp_load_acquire(&mm->cow_subpage_pool);
	if (!pools) {
		new_pools = kzalloc(sizeof(*new_pools), GFP_KERNEL | __GFP_ACCOUNT);
		if (!new_pools)
			return NULL;
		spin_lock_init(&new_pools->lock);
		INIT_LIST_HEAD(&new_pools->pools);
		refcount_set(&new_pools->refs, 1);
		INIT_WORK(&new_pools->free_work, cow_registry_free_work);
		if (rhashtable_init(&new_pools->domains, &cow_domain_params)) {
			kfree(new_pools);
			return NULL;
		}
		pools = cmpxchg(&mm->cow_subpage_pool, NULL, new_pools);
		if (!pools)
			pools = new_pools;
		else
			cow_registry_put(new_pools);
	}
	memcg = get_mem_cgroup_from_mm(charge_mm);
	key.memcg = memcg;
	key.nid = nid;
	key.bucket = hash_long(address >> PAGE_SHIFT, COW_POOL_BITS);
	for (attempt = 0; attempt < 3; attempt++) {
		rcu_read_lock();
		pool = rhashtable_lookup(&pools->domains, &key, cow_domain_params);
		if (pool && !refcount_inc_not_zero(&pool->refs))
			pool = NULL;
		rcu_read_unlock();
		if (pool) {
			mem_cgroup_put(memcg);
			return pool;
		}
		new = mm_subpage_pool_create_granule(GFP_KERNEL, mm_page_shift(mm));
		if (!new)
			break;
		new->cow_free_slots = kcalloc(SUBPAGES_PER_FOLIO,
					    sizeof(*new->cow_free_slots),
					    GFP_KERNEL | __GFP_ACCOUNT);
		if (!new->cow_free_slots) {
			mm_subpage_pool_close(new);
			mm_subpage_pool_put(new);
			break;
		}
#ifdef CONFIG_MEMCG
		if (memcg)
			css_get(&memcg->css);
#endif
		new->cow_memcg = memcg;
		new->cow_nid = nid;
		new->cow_shared = true;
		new->cow_key = key;
		INIT_LIST_HEAD(&new->cow_domain);
		new->cow_registry = pools;
		/* Hold caller, prospective membership and registry lifetime refs
		 * before a concurrent lookup can observe this pool.
		 */
		mm_subpage_pool_get(new);
		refcount_inc(&pools->refs);
		spin_lock_irqsave(&new->lock, flags);
		result = rhashtable_lookup_get_insert_fast(&pools->domains,
						 &new->cow_hash, cow_domain_params);
		if (!result) {
			spin_lock(&pools->lock);
			list_add_tail(&new->cow_domain, &pools->pools);
			spin_unlock(&pools->lock);
		}
		spin_unlock_irqrestore(&new->lock, flags);
		if (!result) {
			mem_cgroup_put(memcg);
			return new;
		}
		/* Do not dereference an unreferenced object returned by insert. */
		mm_subpage_pool_close(new);
		mm_subpage_pool_put(new);
		if (IS_ERR(result))
			break;
	}
	mem_cgroup_put(memcg);
	return NULL;
}

struct mm_subpage_pool *mm_subpage_cow_pool_get(struct mm_struct *mm,
			struct vm_area_struct *vma, unsigned long address)
{
	return mm_subpage_cow_pool_get_charged(mm, mm, vma, address);
}

bool mm_subpage_cow_folio_matches(struct mm_subpage_pool *pool, struct folio *folio)
{
	bool matches;

	/* The folio charge may be reparented while this pool remains alive. */
	rcu_read_lock();
	matches = pool->cow_memcg == folio_memcg(folio) &&
		  pool->cow_nid == folio_nid(folio);
	rcu_read_unlock();
	return matches;
}

void mm_subpage_cow_pool_exit(struct mm_struct *mm)
{
	struct mm_subpage_cow_pools *pools = xchg(&mm->cow_subpage_pool, NULL);
	struct mm_subpage_pool *pool;
	unsigned long flags;

	/* No faults remain; outstanding owners/callbacks never use this mm. */
	if (!pools)
		return;
	for (;;) {
		spin_lock_irqsave(&pools->lock, flags);
		pool = list_first_entry_or_null(&pools->pools,
					       struct mm_subpage_pool, cow_domain);
		if (pool)
			mm_subpage_pool_get(pool);
		spin_unlock_irqrestore(&pools->lock, flags);
		if (!pool)
			break;
		/* Lock order is pool -> registry list, never the reverse. */
		mm_subpage_pool_close(pool);
		mm_subpage_pool_put(pool);
	}
	cow_registry_put(pools);
}

/* Counts include unavailable (busy/migrating) owners, so zero is conclusive.
 * A nonzero count still requires all the normal owner eligibility checks.
 */
static void cow_update_free(struct mm_subpage_pool *pool, unsigned long mask, int delta)
{
	unsigned int i;

	lockdep_assert_held(&pool->lock);
	if (!pool->cow_shared)
		return;
	for_each_set_bit(i, &mask, SUBPAGES_PER_FOLIO) {
		VM_BUG_ON(delta < 0 && !pool->cow_free_slots[i]);
		pool->cow_free_slots[i] += delta;
	}
}

static bool cow_has_free(struct mm_subpage_pool *pool, unsigned long eligible)
{
	unsigned int i;

	lockdep_assert_held(&pool->lock);
	if (!pool->cow_shared)
		return true;
	for_each_set_bit(i, &eligible, SUBPAGES_PER_FOLIO)
		if (pool->cow_free_slots[i])
			return true;
	return false;
}

static void owner_remove(struct mm_subpage_owner *owner)
{
	lockdep_assert_held(&owner->pool->lock);
	owner->detached = true;
	cow_update_free(owner->pool, owner->pool->all_slots & ~owner->used, -1);
	list_del(&owner->all);
	list_del_init(&owner->available);
	if (owner->pool->unissued == owner)
		owner->pool->unissued = NULL;
	owner->pool->backing_pages--;
	if (!owner->pool->backing_pages && owner->pool->cow_shared) {
		owner->pool->closed = true;
		cow_pool_detach_locked(owner->pool);
	}
}

static void owner_free_rmap(struct mm_subpage_owner *owner)
{
	unsigned int i;

	if (!owner->rmap)
		return;
	for (i = 0; i < (PAGE_SIZE >> owner->shift); i++)
		if (owner->rmap[i].root)
			put_anon_vma(owner->rmap[i].root);
	kfree(owner->rmap);
}

static void owner_free_metadata(struct mm_subpage_owner *owner)
{
	struct mm_subpage_pool *pool = owner->pool;

	if (owner->anon_root)
		put_anon_vma(owner->anon_root);
	owner_free_rmap(owner);
	kfree_rcu(owner, rcu);
	mm_subpage_pool_put(pool);
}

static void owner_free_work(struct work_struct *work)
{
	owner_free_metadata(container_of(work, struct mm_subpage_owner, free_work));
}

static void owner_free(struct mm_subpage_owner *owner, bool put_folio)
{
	owner_unbind(owner);
	if (put_folio) {
		if (owner->anon_root)
			free_swap_cache(owner->folio);
		/* Last slot reference, or wholly unissued supply. */
		folio_put(owner->folio);
	}
	if (owner->anon_root || owner->rmap) {
		/* Any final anon-vma put can sleep, including a linear owner's. */
		INIT_WORK(&owner->free_work, owner_free_work);
		queue_work(system_dfl_wq, &owner->free_work);
	} else {
		owner_free_metadata(owner);
	}
}

struct anon_vma *mm_subpage_anon_root(const struct mm_subpage *subpage)
{
	VM_BUG_ON_FOLIO(!folio_test_locked(subpage->owner->folio), subpage->owner->folio);
	return subpage->owner->anon_root;
}

void mm_subpage_hold_anon_root(struct mm_subpage *subpage, struct anon_vma *root,
			       bool droppable)
{
	struct mm_subpage_owner *owner = subpage->owner;

	VM_BUG_ON_FOLIO(!folio_test_locked(owner->folio), owner->folio);
	VM_BUG_ON(owner->anon_root || root != root->root);
	get_anon_vma(root);
	owner->droppable = droppable;
	owner->anon_root = root;
}

static struct mm_subpage_rmap slot_rmap(struct mm_subpage *slot)
{
	struct mm_subpage_owner *owner = slot->owner;

	lockdep_assert_held(&owner->pool->lock);
	if (owner->rmap)
		return owner->rmap[slot->index];
	return (struct mm_subpage_rmap) {
		.root = owner->anon_root,
		.pos = { .index = owner->folio->index,
			 .offset = mm_subpage_offset(slot) },
		.offset = mm_subpage_offset(slot),
	};
}

static bool rmap_matches(struct mm_subpage_rmap map, struct vm_area_struct *vma,
			 unsigned long address)
{
	return map.root == vma->anon_vma->root &&
		vm_page_offset_equal(map.pos, vma_anon_page_offset_at(vma, address));
}

bool mm_subpage_match_rmap(struct mm_subpage *slot, struct vm_area_struct *vma,
			  unsigned long address)
{
	struct mm_subpage_pool *pool = slot->owner->pool;
	unsigned long flags;
	bool matches;

	spin_lock_irqsave(&pool->lock, flags);
	matches = slot->rmap_bound && rmap_matches(slot_rmap(slot), vma, address);
	spin_unlock_irqrestore(&pool->lock, flags);
	return matches;
}

static void *owner_kcalloc(struct mm_subpage_owner *owner, size_t nr, size_t size)
{
	struct mem_cgroup *memcg, *saved;
	void *maps;

	rcu_read_lock();
	do {
		memcg = mem_cgroup_from_virt(owner) ?: root_mem_cgroup;
	} while (memcg && !mem_cgroup_tryget(memcg));
	rcu_read_unlock();
	saved = set_active_memcg(memcg);
	maps = kcalloc(nr, size, GFP_NOWAIT | __GFP_NOWARN | __GFP_ACCOUNT);
	set_active_memcg(saved);
	mem_cgroup_put(memcg);
	return maps;
}

struct subpage_retired_root {
	struct work_struct work;
	struct anon_vma *root;
};

static void subpage_put_retired_root(struct work_struct *work)
{
	struct subpage_retired_root *retired =
		container_of(work, struct subpage_retired_root, work);

	put_anon_vma(retired->root);
	kfree(retired);
}

/* The folio lock serializes binding and sidecar creation against migration. */
int mm_subpage_bind_rmap(struct mm_subpage *slot, struct vm_area_struct *vma,
			unsigned long address)
{
	struct mm_subpage_owner *owner = slot->owner;
	struct mm_subpage_pool *pool = owner->pool;
	struct mm_subpage_rmap *maps = NULL;
	struct subpage_retired_root *retired = NULL;
	unsigned long flags;
	unsigned int i;
	int ret = 0;

	VM_BUG_ON_FOLIO(!folio_test_locked(owner->folio), owner->folio);
	spin_lock_irqsave(&pool->lock, flags);
	if (slot->rmap_bound) {
		ret = rmap_matches(slot_rmap(slot), vma, address) ? 0 : -EINVAL;
		goto unlock;
	}
	if (!owner->rmap && !rmap_matches(slot_rmap(slot), vma, address)) {
		spin_unlock_irqrestore(&pool->lock, flags);
		/* The caller may also hold a PTE lock. Never enter reclaim here. */
		maps = owner_kcalloc(owner, PAGE_SIZE >> owner->shift, sizeof(*maps));
		if (!maps)
			return -ENOMEM;
		spin_lock_irqsave(&pool->lock, flags);
		for (i = 0; i < (PAGE_SIZE >> owner->shift); i++) {
			maps[i].offset = i << owner->shift;
			if (!owner->slots[i].rmap_bound)
				continue;
			maps[i] = slot_rmap(&owner->slots[i]);
			get_anon_vma(maps[i].root);
		}
		owner->rmap = maps;
	}
	if (owner->rmap) {
		struct mm_subpage_rmap *map = &owner->rmap[slot->index];

		if (map->root && map->root != vma->anon_vma->root) {
			/* A final anon_vma put may sleep; the caller can hold a PTL. */
			spin_unlock_irqrestore(&pool->lock, flags);
			retired = owner_kcalloc(owner, 1, sizeof(*retired));
			if (!retired)
				return -ENOMEM;
			INIT_WORK(&retired->work, subpage_put_retired_root);
			spin_lock_irqsave(&pool->lock, flags);
			retired->root = map->root;
		}
		if (map->root != vma->anon_vma->root) {
			get_anon_vma(vma->anon_vma->root);
			map->root = vma->anon_vma->root;
		}
		map->pos = vma_anon_page_offset_at(vma, address);
		map->offset = mm_subpage_offset(slot);
	}
	slot->rmap_bound = true;
unlock:
	spin_unlock_irqrestore(&pool->lock, flags);
	if (retired)
		queue_work(system_dfl_wq, &retired->work);
	return ret;
}

/* Preflight publication without making an unexposed owner look published.
 * A fresh linear slot needs no sidecar: add_new_anon_rmap will set its anchor.
 * Otherwise bind the independent virtual position before any PTE lock.
 */
int mm_subpage_prepare_anon_rmap(struct mm_subpage *slot,
			       struct vm_area_struct *vma, unsigned long address)
{
	struct mm_subpage_owner *owner = slot->owner;

	VM_BUG_ON_FOLIO(!folio_test_locked(owner->folio), owner->folio);
	if (!owner->anon_root && !owner->rmap &&
	    mm_subpage_offset(slot) == vma_page_offset_at(vma, address).offset)
		return 0;
	return mm_subpage_bind_rmap(slot, vma, address);
}

static bool subpage_move_allowed(struct mm_subpage *slot, bool swap)
{
	lockdep_assert_held(&slot->owner->pool->lock);
	return !slot->owner->migrating && !slot->pins &&
		atomic_read(&slot->mapcount) == (swap ? 0 : 1) &&
		(swap || (slot->rmap_bound && slot->exclusive));
}

int mm_subpage_move_prepare(struct mm_subpage_move *move, struct mm_subpage *slot,
		struct vm_area_struct *src, unsigned long src_address,
		struct vm_area_struct *dst, unsigned long dst_address, bool swap)
{
	struct mm_subpage_owner *owner = slot->owner;
	struct mm_subpage_rmap *maps, old;
	unsigned long flags;
	unsigned int i;
	int ret = -EBUSY;

	VM_BUG_ON_FOLIO(!folio_test_locked(owner->folio), owner->folio);
	memset(move, 0, sizeof(*move));
	if (!src->anon_vma || !dst->anon_vma || !owner->anon_root ||
	    mm_page_shift(src->vm_mm) != slot->shift ||
	    mm_page_shift(dst->vm_mm) != slot->shift ||
	    owner->droppable != !!(dst->vm_flags & VM_DROPPABLE))
		return -EINVAL;
	spin_lock_irqsave(&owner->pool->lock, flags);
	if (!subpage_move_allowed(slot, swap) ||
	    (slot->rmap_bound && !rmap_matches(slot_rmap(slot), src, src_address)))
		goto unlock;
	spin_unlock_irqrestore(&owner->pool->lock, flags);

	/* Allocate before touching either PTE. The folio lock serializes writers. */
	if (!owner->rmap) {
		maps = owner_kcalloc(owner, PAGE_SIZE >> owner->shift, sizeof(*maps));
		if (!maps)
			return -ENOMEM;
		spin_lock_irqsave(&owner->pool->lock, flags);
		for (i = 0; i < (PAGE_SIZE >> owner->shift); i++) {
			maps[i].offset = i << owner->shift;
			if (!owner->slots[i].rmap_bound)
				continue;
			maps[i] = slot_rmap(&owner->slots[i]);
			get_anon_vma(maps[i].root);
		}
		owner->rmap = maps;
		spin_unlock_irqrestore(&owner->pool->lock, flags);
	}
	old = owner->rmap[slot->index];
	move->root_changed = old.root != dst->anon_vma->root;
	if (move->root_changed && old.root) {
		move->retired = owner_kcalloc(owner, 1, sizeof(*move->retired));
		if (!move->retired)
			return -ENOMEM;
		INIT_WORK(&move->retired->work, subpage_put_retired_root);
		move->retired->root = old.root;
	}
	if (move->root_changed)
		get_anon_vma(dst->anon_vma->root);
	move->slot = slot;
	move->swap = swap;
	move->map = (struct mm_subpage_rmap) {
		.root = dst->anon_vma->root,
		.pos = vma_anon_page_offset_at(dst, dst_address),
		.offset = mm_subpage_offset(slot),
	};
	return 0;
unlock:
	spin_unlock_irqrestore(&owner->pool->lock, flags);
	return ret;
}

int mm_subpage_move_commit(struct mm_subpage_move *move)
{
	struct mm_subpage *slot = move->slot;
	struct mm_subpage_owner *owner = slot->owner;
	unsigned long flags;
	int ret = -EBUSY;

	VM_BUG_ON_FOLIO(!folio_test_locked(owner->folio), owner->folio);
	spin_lock_irqsave(&owner->pool->lock, flags);
	/* Recheck pins after source-PTE clearing and the matching TLB barrier. */
	if (subpage_move_allowed(slot, move->swap)) {
		owner->rmap[slot->index] = move->map;
		slot->rmap_bound = true;
		move->committed = true;
		ret = 0;
	}
	spin_unlock_irqrestore(&owner->pool->lock, flags);
	return ret;
}

void mm_subpage_move_finish(struct mm_subpage_move *move)
{
	if (move->committed) {
		if (move->retired)
			queue_work(system_dfl_wq, &move->retired->work);
	} else {
		/* The locked destination VMA still holds this root. */
		if (move->root_changed)
			put_anon_vma(move->map.root);
		kfree(move->retired);
	}
}

int mm_subpage_snapshot_rmap(struct folio *folio, struct mm_subpage_rmap *maps)
{
	struct page_ext *ext = subpage_ext_get(&folio->page);
	struct mm_subpage_ext *subext;
	struct mm_subpage_owner *owner;
	unsigned long flags;
	unsigned int i;
	int nr = -1;

	VM_BUG_ON_FOLIO(!folio_test_locked(folio), folio);
	if (!ext)
		return nr;
	subext = page_ext_data(ext, &mm_subpage_ext_ops);
	owner = rcu_dereference(subext->owner);
	if (!owner)
		goto out;
	spin_lock_irqsave(&owner->pool->lock, flags);
	if (!owner->detached && owner->rmap) {
		nr = 0;
		for (i = 0; i < (PAGE_SIZE >> owner->shift); i++) {
			if (!owner->slots[i].rmap_bound)
				continue;
			maps[nr] = owner->rmap[i];
			get_anon_vma(maps[nr].root);
			nr++;
		}
	}
	spin_unlock_irqrestore(&owner->pool->lock, flags);
out:
	page_ext_put(ext);
	return nr;
}

/* A referenced folio plus a stable VMA suffices; no folio lock required. */
bool mm_subpage_address_in_vma(const struct folio *folio,
		const struct vm_area_struct *vma, unsigned long *address)
{
	struct page_ext *ext = subpage_ext_get(&folio->page);
	struct mm_subpage_ext *subext;
	struct mm_subpage_owner *owner;
	unsigned long flags;
	unsigned int i;
	bool nonlinear = false;

	if (!ext)
		return false;
	subext = page_ext_data(ext, &mm_subpage_ext_ops);
	owner = rcu_dereference(subext->owner);
	if (!owner)
		goto out;
	spin_lock_irqsave(&owner->pool->lock, flags);
	if (!owner->detached && owner->rmap) {
		nonlinear = true;
		*address = -EFAULT;
		for (i = 0; vma->anon_vma && i < (PAGE_SIZE >> owner->shift); i++) {
			if (!owner->slots[i].rmap_bound ||
			    owner->rmap[i].root != vma->anon_vma->root)
				continue;
			*address = vma_address_at_anon_offset(vma, owner->rmap[i].pos);
			if (*address != -EFAULT)
				break;
		}
	}
	spin_unlock_irqrestore(&owner->pool->lock, flags);
out:
	page_ext_put(ext);
	return nonlinear;
}

bool mm_subpage_is_droppable(const struct mm_subpage *subpage)
{
	return subpage->owner->droppable;
}

void mm_subpage_pool_close(struct mm_subpage_pool *pool)
{
	struct mm_subpage_owner *owner;
	unsigned long flags;

	spin_lock_irqsave(&pool->lock, flags);
	pool->closed = true;
	cow_pool_detach_locked(pool);
	/* At most one newly supplied owner has no issued slots. Close is O(1). */
	owner = pool->unissued;
	if (owner)
		owner_remove(owner);
	spin_unlock_irqrestore(&pool->lock, flags);
	if (owner)
		owner_free(owner, true);
}

static bool cow_owner_eligible(struct mm_subpage_pool *pool, struct folio *folio)
{
	/* All allocation and retry paths must exclude poisoned backing. */
	if (folio_test_hwpoison(folio))
		return false;
	if (!pool->cow_shared)
		return true;
	/* A previously unlocked owner can be mlocked or fully MADV_FREE'd
	 * after entering the pool. Never place a new live slot in such backing.
	 */
	return !folio_test_mlocked(folio) && !folio_test_unevictable(folio) &&
		!folio_test_ksm(folio) &&
		(!folio_test_anon(folio) || folio_test_swapbacked(folio)) &&
		mm_subpage_cow_folio_matches(pool, folio);
}

/* Pressure-only fallback after a non-OOM speculative allocation failed.
 * Return true when existing compatible backing can be retried. Unlike the
 * normal bounded search, inspect all owners before allowing an OOM-capable
 * allocation: a busy slot farther down the list may require no new charge.
 */
static bool cow_wait_busy(struct mm_subpage_pool *pool, unsigned long eligible,
			  bool require_all, unsigned int count)
{
	struct mm_subpage_owner *owner;
	struct folio *wait = NULL;
	unsigned long flags;

	spin_lock_irqsave(&pool->lock, flags);
	if (!pool->closed && pool->cow_shared && cow_has_free(pool, eligible)) {
		list_for_each_entry(owner, &pool->available, available) {
			if (!owner->migrating && (~owner->used & eligible) &&
			    (!require_all || !(owner->used & eligible)) &&
		    (count == 1 || hweight_long(~owner->used & eligible) >= count) &&
			    cow_owner_eligible(pool, owner->folio) &&
			    !folio_test_swapcache(owner->folio) &&
			    !folio_test_writeback(owner->folio)) {
				wait = owner->folio;
				folio_get(wait);
				break;
			}
		}
	}
	spin_unlock_irqrestore(&pool->lock, flags);
	if (wait) {
		folio_wait_locked(wait);
		folio_put(wait);
		return true;
	}
	return false;
}

bool mm_subpage_cow_wait_busy(struct mm_subpage_pool *pool)
{
	return cow_wait_busy(pool, pool->all_slots, false, 1);
}

bool mm_subpage_cow_wait_busy_at(struct mm_subpage_pool *pool, unsigned int offset)
{
	return cow_wait_busy(pool, offset_mask(pool->shift, offset), false, 1);
}

bool mm_subpage_cow_wait_busy_mask(struct mm_subpage_pool *pool, unsigned long mask)
{
	return cow_wait_busy(pool, mask, true, 1);
}

bool mm_subpage_cow_wait_busy_count(struct mm_subpage_pool *pool, unsigned int count)
{
	if (!count || count > hweight_long(pool->all_slots))
		return false;
	return cow_wait_busy(pool, pool->all_slots, false, count);
}

static struct mm_subpage_owner *find_available(struct mm_subpage_pool *pool,
					      unsigned long eligible, bool require_all,
					      unsigned int count)
{
	struct mm_subpage_owner *owner;
	unsigned int visited = 0;

	lockdep_assert_held(&pool->lock);
	if (!cow_has_free(pool, eligible))
		return NULL;
	list_for_each_entry(owner, &pool->available, available) {
		/* A fixed-offset stream must not scan every partially used owner
		 * on each fault. New COW supply is inserted first, so a bounded
		 * miss can always make progress using a fresh folio.
		 */
		if (pool->cow_shared && visited++ == 32) {
			/* The next search starts past this incompatible prefix. A
			 * phased offset stream must not strand older reusable slots.
			 */
			list_rotate_to_front(&owner->available, &pool->available);
			break;
		}
		/* A nonpresent PTE may still need any of this folio's slot bytes. */
		if (!owner->migrating && (~owner->used & eligible) &&
		    (!require_all || !(owner->used & eligible)) &&
		    (count == 1 || hweight_long(~owner->used & eligible) >= count) &&
		    (!pool->cow_shared || !folio_test_locked(owner->folio)) &&
		    cow_owner_eligible(pool, owner->folio) &&
		    !folio_test_swapcache(owner->folio) &&
		    !folio_test_writeback(owner->folio))
			return owner;
	}
	return NULL;
}

static int pool_add_folio(struct mm_subpage_pool *pool, struct folio *folio,
			  unsigned long eligible, bool require_all,
			  unsigned int count, gfp_t gfp)
{
	struct mm_subpage_owner *owner;
	unsigned long flags;
	int ret = 0;

	/* Only an unexposed order-0 allocation can become a slot owner. */
	if (folio_order(folio) || folio_ref_count(folio) != 1 ||
	    folio->mapping || folio_mapped(folio) || folio_test_lru(folio) ||
	    folio_test_private(folio) || folio_test_swapcache(folio) ||
	    folio_test_hwpoison(folio))
		return -EINVAL;
	owner = kzalloc(sizeof(*owner), gfp | __GFP_ACCOUNT);
	if (!owner)
		return -ENOMEM;
	owner_init(owner, pool, folio);
	/* Reject duplicate ownership before touching any existing data. */
	ret = owner_bind(owner);
	if (ret) {
		kfree(owner);
		return ret;
	}
	/* Initialise even unissued bytes before any future whole-folio operation. */
	clear_user_subpage_range(&folio->page, 0, PAGE_SIZE);
	__folio_mark_uptodate(folio);
	owner->zeroed = pool->all_slots;
	spin_lock_irqsave(&pool->lock, flags);
	if (pool->closed)
		ret = -ESHUTDOWN;
	else if (pool->unissued || find_available(pool, eligible, require_all, count))
		/* Another allocator supplied backing while we allocated metadata.
		 * Preserve the single-unissued-owner invariant even when a shared
		 * COW lookup skips a concurrently locked or rotating list prefix.
		 */
		ret = -EEXIST;
	else {
		mm_subpage_pool_get(pool);
		list_add_tail(&owner->all, &pool->owners);
		cow_update_free(pool, pool->all_slots, 1);
		if (pool->cow_shared)
			list_add(&owner->available, &pool->available);
		else
			list_add_tail(&owner->available, &pool->available);
		pool->backing_pages++;
		pool->unissued = owner;
	}
	spin_unlock_irqrestore(&pool->lock, flags);
	if (ret) {
		owner_unbind(owner);
		kfree_rcu(owner, rcu);
	}
	return ret;
}

int mm_subpage_pool_add_folio(struct mm_subpage_pool *pool, struct folio *folio,
			      gfp_t gfp)
{
	return pool_add_folio(pool, folio, pool->all_slots, false, 1, gfp);
}

int mm_subpage_pool_add_folio_at(struct mm_subpage_pool *pool, struct folio *folio,
				 unsigned int offset, gfp_t gfp)
{
	unsigned long eligible = offset_mask(pool->shift, offset);

	if (!eligible)
		return -EINVAL;
	return pool_add_folio(pool, folio, eligible, false, 1, gfp);
}

int mm_subpage_pool_add_folio_mask(struct mm_subpage_pool *pool, struct folio *folio,
				 unsigned long mask, gfp_t gfp)
{
	if (!mask || (mask & ~pool->all_slots))
		return -EINVAL;
	return pool_add_folio(pool, folio, mask, true, 1, gfp);
}

int mm_subpage_pool_add_folio_count(struct mm_subpage_pool *pool, struct folio *folio,
				  unsigned int count, gfp_t gfp)
{
	if (!count || count > hweight_long(pool->all_slots))
		return -EINVAL;
	return pool_add_folio(pool, folio, pool->all_slots, false, count, gfp);
}

unsigned long mm_subpage_pool_backing_pages(struct mm_subpage_pool *pool)
{
	unsigned long pages, flags;

	spin_lock_irqsave(&pool->lock, flags);
	pages = pool->backing_pages;
	spin_unlock_irqrestore(&pool->lock, flags);
	return pages;
}

/* Caller holds pool lock; each issued slot owns one native folio reference. */
static struct mm_subpage *reserve_owner_slot(struct mm_subpage_owner *owner,
					    unsigned int index, bool *needs_zero)
{
	struct mm_subpage_pool *pool = owner->pool;
	struct mm_subpage *subpage;

	lockdep_assert_held(&pool->lock);
	if (pool->unissued == owner)
		/* Transfer the supplied reference to the first reservation. */
		pool->unissued = NULL;
	else
		folio_get(owner->folio);
	__set_bit(index, &owner->used);
	cow_update_free(pool, BIT(index), -1);
	if (needs_zero)
		*needs_zero = !test_bit(index, &owner->zeroed);
	__clear_bit(index, &owner->zeroed);
	if (owner->used == pool->all_slots)
		list_del_init(&owner->available);
	subpage = &owner->slots[index];
	WRITE_ONCE(subpage->lazyfree, false);
	/* The used bit keeps backing alive while data is initialised. */
	VM_BUG_ON(refcount_read(&subpage->refs));
	VM_BUG_ON(atomic_read(&subpage->mapcount));
	return subpage;
}

static struct mm_subpage *subpage_reserve(struct mm_subpage_pool *pool,
					 unsigned long eligible, bool *needs_zero, bool lock_folio)
{
	struct mm_subpage_owner *owner;
	struct mm_subpage *subpage;
	unsigned long flags;
	unsigned int index;

retry:
	spin_lock_irqsave(&pool->lock, flags);
	if (pool->closed) {
		subpage = ERR_PTR(-ESHUTDOWN);
		goto out;
	}
	owner = find_available(pool, eligible, false, 1);
	if (!owner) {
		subpage = ERR_PTR(-EAGAIN);
		goto out;
	}
	if (lock_folio) {
		if (!folio_trylock(owner->folio)) {
			struct folio *wait;

			if (pool->cow_shared) {
				/* Another COW fault won this folio after the lookup.
				 * Try other backing, or supply another folio, rather
				 * than making parallel faults queue on its lock.
				 */
				spin_unlock_irqrestore(&pool->lock, flags);
				cond_resched();
				goto retry;
			}
			wait = owner->folio;
			folio_get(wait);
			spin_unlock_irqrestore(&pool->lock, flags);
			folio_wait_locked(wait);
			folio_put(wait);
			goto retry;
		}
		/* Reclaim may have acquired the folio after find_available's check. */
		if (folio_test_swapcache(owner->folio) || folio_test_writeback(owner->folio) ||
		    !cow_owner_eligible(pool, owner->folio)) {
			folio_unlock(owner->folio);
			spin_unlock_irqrestore(&pool->lock, flags);
			return ERR_PTR(-EAGAIN);
		}
	}
	index = __ffs(~owner->used & eligible);
	subpage = reserve_owner_slot(owner, index, needs_zero);
out:
	spin_unlock_irqrestore(&pool->lock, flags);
	return subpage;
}

/* One owner/folio lock; output remains indexed by the caller's logical mask. */
static int subpage_alloc_mask_locked(struct mm_subpage_pool *pool, unsigned long mask,
				    bool any_offset, struct mm_subpage **slots)
{
	struct mm_subpage_owner *owner;
	unsigned long flags, zero = 0, available;
	unsigned int i, physical, count = hweight_long(mask);
	bool needs_zero;

	if (!mask || (mask & ~pool->all_slots))
		return -EINVAL;
retry:
	spin_lock_irqsave(&pool->lock, flags);
	if (pool->closed) {
		spin_unlock_irqrestore(&pool->lock, flags);
		return -ESHUTDOWN;
	}
	owner = find_available(pool, any_offset ? pool->all_slots : mask,
			       !any_offset, any_offset ? count : 1);
	if (!owner) {
		spin_unlock_irqrestore(&pool->lock, flags);
		return -EAGAIN;
	}
	if (!folio_trylock(owner->folio)) {
		/* Parallel faults use another owner rather than queue behind it. */
		spin_unlock_irqrestore(&pool->lock, flags);
		cond_resched();
		goto retry;
	}
	if (folio_test_swapcache(owner->folio) || folio_test_writeback(owner->folio) ||
	    !cow_owner_eligible(pool, owner->folio)) {
		folio_unlock(owner->folio);
		spin_unlock_irqrestore(&pool->lock, flags);
		return -EAGAIN;
	}
	available = ~owner->used & pool->all_slots;
	for_each_set_bit(i, &mask, SUBPAGES_PER_FOLIO) {
		physical = any_offset ? __ffs(available) : i;
		available &= ~BIT(physical);
		slots[i] = reserve_owner_slot(owner, physical, &needs_zero);
		if (needs_zero)
			zero |= BIT(i);
	}
	spin_unlock_irqrestore(&pool->lock, flags);
	for_each_set_bit(i, &mask, SUBPAGES_PER_FOLIO) {
		if (zero & BIT(i))
			clear_user_subpage_range(&owner->folio->page,
					 mm_subpage_offset(slots[i]), 1U << pool->shift);
		/* Zeroing must precede publication, including reused slots. */
		refcount_set_release(&slots[i]->refs, 1);
	}
	return 0;
}

int mm_subpage_alloc_mask_locked(struct mm_subpage_pool *pool, unsigned long mask,
				struct mm_subpage **slots)
{
	return subpage_alloc_mask_locked(pool, mask, false, slots);
}

int mm_subpage_alloc_any_mask_locked(struct mm_subpage_pool *pool, unsigned long mask,
				    struct mm_subpage **slots)
{
	return subpage_alloc_mask_locked(pool, mask, true, slots);
}

static struct mm_subpage *subpage_alloc(struct mm_subpage_pool *pool, unsigned long eligible, bool lock_folio)
{
	bool needs_zero;
	struct mm_subpage *subpage = subpage_reserve(pool, eligible, &needs_zero, lock_folio);

	if (IS_ERR(subpage))
		return subpage;
	/* Fresh supply was already zeroed as a whole; reused slots must be cleared. */
	if (needs_zero)
		clear_user_subpage_range(&subpage->owner->folio->page,
				 mm_subpage_offset(subpage), mm_subpage_size(subpage));
	/* Data stores must precede publication in a PTE or another CPU's reference. */
	refcount_set_release(&subpage->refs, 1);
	return subpage;
}

struct mm_subpage *mm_subpage_alloc(struct mm_subpage_pool *pool)
{
	return subpage_alloc(pool, pool->all_slots, false);
}

struct mm_subpage *mm_subpage_alloc_at(struct mm_subpage_pool *pool, unsigned int offset)
{
	unsigned long eligible = offset_mask(pool->shift, offset);

	if (!eligible)
		return ERR_PTR(-EINVAL);
	return subpage_alloc(pool, eligible, false);
}

static void subpage_release_locked(struct mm_subpage *subpage, unsigned long flags)
{
	struct mm_subpage_owner *owner = subpage->owner;
	struct mm_subpage_pool *pool = owner->pool;
	struct folio *folio = owner->folio;
	bool release;

	lockdep_assert_held(&pool->lock);
	VM_BUG_ON(refcount_read(&subpage->refs));
	VM_BUG_ON(atomic_read(&subpage->mapcount));
	VM_BUG_ON(subpage->pins);
	WRITE_ONCE(subpage->exclusive, false);
	if (!owner->migrating) {
		WRITE_ONCE(subpage->lazyfree, false);
		subpage->rmap_bound = false;
	}
	__clear_bit(subpage->index, &owner->used);
	cow_update_free(pool, BIT(subpage->index), 1);
	release = !owner->used && !owner->migrating;
	if (release)
		owner_remove(owner);
	else if (!pool->closed && !owner->migrating && list_empty(&owner->available))
		list_add(&owner->available, &pool->available);
	spin_unlock_irqrestore(&pool->lock, flags);
	if (release)
		owner_free(owner, true);
	else
		folio_put(folio);
}

static void subpage_release(struct mm_subpage *subpage)
{
	unsigned long flags;

	spin_lock_irqsave(&subpage->owner->pool->lock, flags);
	subpage_release_locked(subpage, flags);
}

static struct mm_subpage *subpage_copy(struct mm_subpage_pool *pool,
				      const struct mm_subpage *source, unsigned long eligible, bool lock_folio)
{
	struct mm_subpage *dest;
	int err;

	if (pool->shift != mm_subpage_shift(source))
		return ERR_PTR(-EINVAL);
	dest = subpage_reserve(pool, eligible, NULL, lock_folio);

	if (IS_ERR(dest))
		return dest;
	err = copy_user_subpage_range(&dest->owner->folio->page, mm_subpage_offset(dest),
				&source->owner->folio->page, mm_subpage_offset(source),
				mm_subpage_size(source));
	if (err) {
		/* Never publish a reservation whose copy did not finish. */
		if (lock_folio)
			folio_unlock(dest->owner->folio);
		subpage_release(dest);
		return ERR_PTR(err);
	}
	refcount_set_release(&dest->refs, 1);
	return dest;
}

struct mm_subpage *mm_subpage_copy(struct mm_subpage_pool *pool,
				 const struct mm_subpage *source)
{
	return subpage_copy(pool, source, pool->all_slots, false);
}

struct mm_subpage *mm_subpage_copy_at(struct mm_subpage_pool *pool,
				    const struct mm_subpage *source, unsigned int offset)
{
	unsigned long eligible = offset_mask(pool->shift, offset);

	if (!eligible)
		return ERR_PTR(-EINVAL);
	return subpage_copy(pool, source, eligible, false);
}

struct mm_subpage *mm_subpage_alloc_at_locked(struct mm_subpage_pool *pool,
					    unsigned int offset)
{
	unsigned long eligible = offset_mask(pool->shift, offset);

	return eligible ? subpage_alloc(pool, eligible, true) : ERR_PTR(-EINVAL);
}

struct mm_subpage *mm_subpage_copy_at_locked(struct mm_subpage_pool *pool,
			const struct mm_subpage *source, unsigned int offset)
{
	unsigned long eligible = offset_mask(pool->shift, offset);

	return eligible ? subpage_copy(pool, source, eligible, true) : ERR_PTR(-EINVAL);
}

struct mm_subpage *mm_subpage_alloc_locked(struct mm_subpage_pool *pool)
{
	return subpage_alloc(pool, pool->all_slots, true);
}

struct mm_subpage *mm_subpage_copy_locked(struct mm_subpage_pool *pool,
					const struct mm_subpage *source)
{
	return subpage_copy(pool, source, pool->all_slots, true);
}

/* The page-extension RCU section also protects the pool until its lock is held. */
static struct mm_subpage *restore_bound_slot(struct folio *folio, unsigned int offset,
					      unsigned int shift)
{
	struct page_ext *ext = subpage_ext_get(&folio->page);
	struct mm_subpage_owner *owner;
	struct mm_subpage_pool *pool;
	struct mm_subpage_ext *subext;
	struct mm_subpage *slot = NULL;
	unsigned int index;
	unsigned long flags;

	if (!ext)
		return ERR_PTR(-EOPNOTSUPP);
	subext = page_ext_data(ext, &mm_subpage_ext_ops);
	owner = rcu_dereference(subext->owner);
	if (!owner)
		goto out;
	if ((shift && shift != owner->shift) || !offset_mask(owner->shift, offset)) {
		slot = ERR_PTR(-EINVAL);
		goto out;
	}
	index = offset >> owner->shift;
	pool = owner->pool;
	spin_lock_irqsave(&pool->lock, flags);
	if (owner->detached) {
		slot = ERR_PTR(-EAGAIN);
		goto unlock;
	}
	slot = &owner->slots[index];
	if (test_bit(index, &owner->used)) {
		folio_get(folio);
		if (!refcount_inc_not_zero_acquire(&slot->refs)) {
			/* Its final put or initial publication has not finished. */
			folio_put(folio);
			slot = ERR_PTR(-EAGAIN);
		}
		goto unlock;
	}
	VM_BUG_ON(refcount_read(&slot->refs) || atomic_read(&slot->mapcount));
	if (pool->unissued == owner)
		pool->unissued = NULL;
	else
		folio_get(folio);
	__set_bit(index, &owner->used);
	cow_update_free(pool, BIT(index), -1);
	__clear_bit(index, &owner->zeroed);
	if (owner->used == pool->all_slots)
		list_del_init(&owner->available);
	/* The folio lock and uptodate state stabilise the restored bytes. */
	refcount_set_release(&slot->refs, 1);
unlock:
	spin_unlock_irqrestore(&pool->lock, flags);
out:
	page_ext_put(ext);
	return slot;
}

struct mm_subpage *mm_subpage_restore_granule(struct folio *folio, unsigned int offset,
					   unsigned int shift, gfp_t gfp)
{
	struct mm_subpage_pool *pool;
	struct mm_subpage_owner *owner;
	struct mm_subpage *slot;
	unsigned int index;
	int err;

	VM_BUG_ON_FOLIO(!folio_test_locked(folio), folio);
	if (!subpage_shift_valid(shift) || !offset_mask(shift, offset) || folio_order(folio) || !folio_test_swapcache(folio) ||
	    !folio_test_uptodate(folio) || folio_test_ksm(folio) ||
	    (folio->mapping && !folio_test_anon(folio)) ||
	    (folio_test_anon(folio) && PageAnonExclusive(&folio->page)))
		return ERR_PTR(-EINVAL);
	index = offset >> shift;
	slot = restore_bound_slot(folio, offset, shift);
	if (slot)
		return slot;

	/* A reclaimed folio has no surviving owner; do not initialise its data. */
	pool = mm_subpage_pool_create_granule(gfp, shift);
	if (!pool)
		return ERR_PTR(-ENOMEM);
	pool->closed = true;
	owner = kzalloc(sizeof(*owner), gfp | __GFP_ACCOUNT);
	if (!owner) {
		mm_subpage_pool_put(pool);
		return ERR_PTR(-ENOMEM);
	}
	owner_init(owner, pool, folio);
	owner->used = BIT(index);
	refcount_set(&owner->slots[index].refs, 1);
	list_add(&owner->all, &pool->owners);
	pool->backing_pages = 1;
	folio_get(folio);
	err = owner_bind(owner);
	if (err) {
		folio_put(folio);
		list_del(&owner->all);
		pool->backing_pages = 0;
		kfree(owner);
		mm_subpage_pool_put(pool);
		return ERR_PTR(err == -EEXIST ? -EAGAIN : err);
	}
	/* The private closed pool's initial reference belongs to this owner. */
	return &owner->slots[index];
}

struct mm_subpage *mm_subpage_restore(struct folio *folio, unsigned int offset, gfp_t gfp)
{
	return mm_subpage_restore_granule(folio, offset, MM_SUBPAGE_SHIFT, gfp);
}

/* Preserve the metadata charge when migration runs outside its owning task. */
static struct mm_subpage_owner *owner_alloc_migration(struct mm_subpage_owner *owner)
{
	struct mem_cgroup *memcg, *saved;
	struct mm_subpage_owner *new;

	rcu_read_lock();
	do {
		memcg = mem_cgroup_from_virt(owner) ?: root_mem_cgroup;
	} while (memcg && !mem_cgroup_tryget(memcg));
	rcu_read_unlock();
	saved = set_active_memcg(memcg);
	/* Folio locks are held: do not recurse into reclaim or trigger memcg OOM. */
	new = kzalloc(sizeof(*new), GFP_NOWAIT | __GFP_NOWARN | __GFP_ACCOUNT);
	if (new && owner->rmap) {
		new->rmap = kcalloc(PAGE_SIZE >> owner->shift, sizeof(*new->rmap),
				   GFP_NOWAIT | __GFP_NOWARN | __GFP_ACCOUNT);
		if (!new->rmap) {
			kfree(new);
			new = NULL;
		}
	}
	set_active_memcg(saved);
	mem_cgroup_put(memcg);
	return new;
}

/* A migration hold pins metadata, not the native folio: the driver pins it. */
int mm_subpage_migrate_prepare(struct folio *src, struct folio *dst)
{
	struct page_ext *ext;
	struct mm_subpage_ext *subext;
	struct mm_subpage_owner *owner, *new;
	struct mm_subpage_pool *pool;
	unsigned long flags;
	int ret = 0;
	unsigned int i;

	VM_BUG_ON_FOLIO(!folio_test_locked(src), src);
	VM_BUG_ON_FOLIO(!folio_test_locked(dst), dst);
	ext = subpage_ext_get(&src->page);
	if (!ext)
		return 0;
	subext = page_ext_data(ext, &mm_subpage_ext_ops);
	owner = rcu_dereference(subext->owner);
	if (!owner) {
		page_ext_put(ext);
		return 0;
	}
	pool = owner->pool;
	spin_lock_irqsave(&pool->lock, flags);
	if (owner->detached) {
		owner = NULL;
	} else if (src == dst || folio_order(src) || folio_order(dst) ||
		   owner->migrating || pool->unissued == owner) {
		ret = -EBUSY;
	} else {
		for (i = 0; i < (PAGE_SIZE >> owner->shift); i++)
			if (owner->slots[i].pins)
				ret = -EBUSY;
		if (!ret)
			owner->migrating = true;
	}
	spin_unlock_irqrestore(&pool->lock, flags);
	page_ext_put(ext);
	if (!owner || ret)
		return ret;

	/* Async migration must not recurse into reclaim while holding folio locks. */
	new = owner_alloc_migration(owner);
	if (!new) {
		ret = -ENOMEM;
		goto undo;
	}
	owner_init(new, pool, dst);
	new->migrating = true;
	new->zeroed = owner->zeroed;
	spin_lock_irqsave(&pool->lock, flags);
	for (i = 0; i < (PAGE_SIZE >> owner->shift); i++) {
		new->slots[i].rmap_bound = owner->slots[i].rmap_bound;
		if (new->rmap) {
			new->rmap[i] = owner->rmap[i];
			if (new->rmap[i].root)
				get_anon_vma(new->rmap[i].root);
		}
	}
	spin_unlock_irqrestore(&pool->lock, flags);

	new->droppable = owner->droppable;
	new->anon_root = owner->anon_root;
	if (new->anon_root)
		get_anon_vma(new->anon_root);
	mm_subpage_pool_get(pool);
	spin_lock_irqsave(&pool->lock, flags);
	list_add_tail(&new->all, &pool->owners);
	cow_update_free(pool, pool->all_slots, 1);
	pool->backing_pages++;
	ret = owner_bind(new);
	if (ret)
		owner_remove(new);
	else
		owner->migration_target = new;
	spin_unlock_irqrestore(&pool->lock, flags);
	if (!ret)
		return 0;
	if (new->anon_root)
		put_anon_vma(new->anon_root);
	owner_free_rmap(new);
	kfree(new);
	mm_subpage_pool_put(pool);
undo:
	mm_subpage_migrate_finish(src);
	return ret;
}

bool mm_subpage_migrating(struct folio *folio)
{
	struct page_ext *ext = subpage_ext_get(&folio->page);
	struct mm_subpage_owner *owner;
	struct mm_subpage_ext *subext;
	bool active = false;

	VM_BUG_ON_FOLIO(!folio_test_locked(folio), folio);
	if (!ext)
		return false;
	subext = page_ext_data(ext, &mm_subpage_ext_ops);
	owner = rcu_dereference(subext->owner);
	if (owner)
		active = READ_ONCE(owner->migrating);
	page_ext_put(ext);
	return active;
}

/* Source folio/PTE locked: writable GETs cannot race this final snapshot. */
void mm_subpage_migrate_save_lazyfree(struct mm_subpage *subpage)
{
	struct mm_subpage_owner *owner = subpage->owner;

	VM_BUG_ON_FOLIO(!owner->migrating || !owner->migration_target, owner->folio);
	WRITE_ONCE(owner->migration_target->slots[subpage->index].lazyfree,
		   READ_ONCE(subpage->lazyfree));
}

struct mm_subpage *mm_subpage_migrate_restore(struct folio *folio, unsigned int offset)
{
	struct mm_subpage *slot;

	VM_BUG_ON_FOLIO(!folio_test_locked(folio), folio);
	VM_BUG_ON_FOLIO(!offset_mask(MM_SUBPAGE_SHIFT, offset) || !mm_subpage_migrating(folio), folio);
	/* No allocation or data initialisation can fail during migration recovery. */
	slot = restore_bound_slot(folio, offset, 0);
	VM_BUG_ON_FOLIO(IS_ERR_OR_NULL(slot), folio);
	return slot;
}

void mm_subpage_migrate_finish(struct folio *folio)
{
	struct page_ext *ext = subpage_ext_get(&folio->page);
	struct mm_subpage_owner *owner;
	struct mm_subpage_ext *subext;
	struct mm_subpage_pool *pool;
	unsigned long flags;
	unsigned int i;
	bool release = false;

	VM_BUG_ON_FOLIO(!folio_test_locked(folio), folio);
	if (!ext)
		return;
	subext = page_ext_data(ext, &mm_subpage_ext_ops);
	owner = rcu_dereference(subext->owner);
	if (!owner)
		goto out;
	pool = owner->pool;
	spin_lock_irqsave(&pool->lock, flags);
	if (owner->migrating) {
		VM_BUG_ON(owner->detached);
		owner->migrating = false;
		owner->migration_target = NULL;
		for (i = 0; i < (PAGE_SIZE >> owner->shift); i++)
			if (!(owner->used & BIT(i)))
				owner->slots[i].rmap_bound = false;
		release = !owner->used;
		if (release)
			owner_remove(owner);
		else if (!pool->closed && owner->used != pool->all_slots &&
			 list_empty(&owner->available))
			list_add(&owner->available, &pool->available);
	}
	spin_unlock_irqrestore(&pool->lock, flags);
out:
	page_ext_put(ext);
	if (release)
		/* No slot references remain; only the migration driver pins backing. */
		owner_free(owner, false);
}

void mm_subpage_get(struct mm_subpage *subpage)
{
	folio_get(subpage->owner->folio);
	refcount_inc(&subpage->refs);
}

void mm_subpage_put(struct mm_subpage *subpage)
{
	struct mm_subpage_owner *owner = subpage->owner;
	struct mm_subpage_pool *pool = owner->pool;
	struct folio *folio = owner->folio;
	unsigned long flags;

	/*
	 * Snapshot the backing while our slot reference still retains owner.
	 * A non-final decrement can race the final put and metadata reclamation;
	 * only our physical folio reference survives that decrement.
	 */
	if (refcount_dec_and_lock_irqsave(&subpage->refs, &pool->lock, &flags))
		subpage_release_locked(subpage, flags);
	else
		folio_put(folio);
}

void mm_subpage_set_exclusive(struct mm_subpage *subpage)
{
	struct mm_subpage_pool *pool = subpage->owner->pool;
	unsigned long flags;

	VM_BUG_ON_FOLIO(!folio_test_locked(subpage->owner->folio), subpage->owner->folio);
	spin_lock_irqsave(&pool->lock, flags);
	VM_BUG_ON(atomic_read(&subpage->mapcount) > 1 || !refcount_read(&subpage->refs));
	WRITE_ONCE(subpage->exclusive, true);
	spin_unlock_irqrestore(&pool->lock, flags);
}

bool mm_subpage_is_exclusive(const struct mm_subpage *subpage)
{
	return READ_ONCE(subpage->exclusive);
}

static bool subpage_try_reuse(struct mm_subpage *subpage, unsigned int references)
{
	struct mm_subpage_pool *pool = subpage->owner->pool;
	unsigned long flags;
	bool reusable;

	VM_BUG_ON_FOLIO(!folio_test_locked(subpage->owner->folio), subpage->owner->folio);
	spin_lock_irqsave(&pool->lock, flags);
	reusable = atomic_read(&subpage->mapcount) == 1 &&
		refcount_read(&subpage->refs) == references && !subpage->pins;
	if (reusable)
		WRITE_ONCE(subpage->exclusive, true);
	spin_unlock_irqrestore(&pool->lock, flags);
	return reusable;
}

bool mm_subpage_try_reuse_swap(struct mm_subpage *subpage)
{
	return subpage_try_reuse(subpage, 1);
}

bool mm_subpage_try_reuse_anon(struct mm_subpage *subpage)
{
	return subpage_try_reuse(subpage, 2);
}

bool mm_subpage_can_zap(struct mm_subpage *subpage)
{
	struct mm_subpage_pool *pool = subpage->owner->pool;
	unsigned long flags;
	bool eligible;

	spin_lock_irqsave(&pool->lock, flags);
	eligible = !subpage->owner->migrating && !subpage->pins &&
		atomic_read(&subpage->mapcount) == 1 &&
		refcount_read(&subpage->refs) == 2;
	spin_unlock_irqrestore(&pool->lock, flags);
	return eligible;
}

bool mm_subpage_try_share(struct mm_subpage *subpage)
{
	struct mm_subpage_pool *pool = subpage->owner->pool;
	unsigned long flags;
	bool shared;

	spin_lock_irqsave(&pool->lock, flags);
	shared = !subpage->pins;
	if (shared)
		WRITE_ONCE(subpage->exclusive, false);
	spin_unlock_irqrestore(&pool->lock, flags);
	return shared;
}

bool mm_subpage_is_lazyfree(const struct mm_subpage *subpage)
{
	return READ_ONCE(subpage->lazyfree);
}

void mm_subpage_cancel_lazyfree(struct mm_subpage *subpage)
{
	struct mm_subpage_pool *pool = subpage->owner->pool;
	unsigned long flags;

	spin_lock_irqsave(&pool->lock, flags);
	WRITE_ONCE(subpage->lazyfree, false);
	spin_unlock_irqrestore(&pool->lock, flags);
}

bool mm_subpage_mark_lazyfree(struct mm_subpage *subpage)
{
	struct mm_subpage_pool *pool = subpage->owner->pool;
	unsigned long flags;
	bool eligible;

	VM_BUG_ON_FOLIO(!folio_test_locked(subpage->owner->folio), subpage->owner->folio);
	spin_lock_irqsave(&pool->lock, flags);
	eligible = atomic_read(&subpage->mapcount) == 1 &&
		refcount_read(&subpage->refs) == 2 && !subpage->pins;
	if (eligible)
		WRITE_ONCE(subpage->lazyfree, true);
	spin_unlock_irqrestore(&pool->lock, flags);
	return eligible;
}

bool mm_subpage_all_lazyfree(struct mm_subpage *subpage)
{
	struct mm_subpage_owner *owner = subpage->owner;
	unsigned long flags;
	unsigned int i;
	bool all = true;

	spin_lock_irqsave(&owner->pool->lock, flags);
	for (i = 0; i < SUBPAGES_PER_FOLIO; i++) {
		if (!(owner->used & BIT(i)))
			continue;
		if (!READ_ONCE(owner->slots[i].lazyfree)) {
			all = false;
			break;
		}
	}
	spin_unlock_irqrestore(&owner->pool->lock, flags);
	return all;
}

bool mm_subpage_discardable(struct mm_subpage *subpage)
{
	struct mm_subpage_pool *pool = subpage->owner->pool;
	unsigned long flags;
	bool eligible;

	spin_lock_irqsave(&pool->lock, flags);
	eligible = (subpage->owner->droppable || READ_ONCE(subpage->lazyfree)) &&
		!subpage->pins &&
		refcount_read(&subpage->refs) == atomic_read(&subpage->mapcount) + 1;
	spin_unlock_irqrestore(&pool->lock, flags);
	return eligible;
}

int mm_subpage_pin(struct mm_subpage *subpage)
{
	struct mm_subpage_owner *owner = subpage->owner;
	unsigned long flags;
	int ret = -EAGAIN;

	spin_lock_irqsave(&owner->pool->lock, flags);
	if (!owner->anon_root || owner->migrating || !subpage->exclusive ||
	    !atomic_read(&subpage->mapcount))
		goto out;
	ret = try_grab_folio(owner->folio, 1, FOLL_PIN);
	if (ret)
		goto out;
	mm_subpage_get(subpage);
	subpage->pins++;
out:
	spin_unlock_irqrestore(&owner->pool->lock, flags);
	return ret;
}

void mm_subpage_unpin(struct mm_subpage *subpage)
{
	struct mm_subpage_owner *owner = subpage->owner;
	unsigned long flags;

	spin_lock_irqsave(&owner->pool->lock, flags);
	VM_BUG_ON(!subpage->pins || !subpage->exclusive);
	subpage->pins--;
	spin_unlock_irqrestore(&owner->pool->lock, flags);
	unpin_folio(owner->folio);
	mm_subpage_put(subpage);
}

unsigned int mm_subpage_pincount(const struct mm_subpage *subpage)
{
	return READ_ONCE(subpage->pins);
}

int mm_subpage_mapcount(const struct mm_subpage *subpage)
{
	return atomic_read(&subpage->mapcount);
}

struct folio *mm_subpage_folio(const struct mm_subpage *subpage)
{
	return subpage->owner->folio;
}

unsigned int mm_subpage_shift(const struct mm_subpage *subpage)
{
	return subpage->shift;
}

unsigned int mm_subpage_size(const struct mm_subpage *subpage)
{
	return 1U << subpage->shift;
}

unsigned int mm_subpage_offset(const struct mm_subpage *subpage)
{
	return subpage->index << subpage->shift;
}

phys_addr_t mm_subpage_phys(const struct mm_subpage *subpage)
{
	return PFN_PHYS(folio_pfn(subpage->owner->folio)) + mm_subpage_offset(subpage);
}

/*
 * Round the mm's aggregate reservation, never each VMA: splitting a VMA
 * must not create extra charges or refunds. mmap writers serialize updates;
 * munmap may retain the lock downgraded while finishing its refund.
 */
static int mm_commit_delta(struct mm_struct *mm, long pages,
		unsigned long *next, long *delta)
{
	unsigned long old = mm->committed_user_pages;
	unsigned int shift = mm_page_shift(mm);

	mmap_assert_locked(mm);
	if (pages < 0 && WARN_ON_ONCE(pages < -(long)old))
		return -EINVAL;
	if (pages > 0 && (unsigned long)pages > LONG_MAX - old)
		return -ENOMEM;
	*next = old + pages;
	if (shift > PAGE_SHIFT) {
		unsigned int extra = shift - PAGE_SHIFT;

		/* A logical reservation may span several native commit units. */
		if (*next > (LONG_MAX >> extra) || old > (LONG_MAX >> extra))
			return -ENOMEM;
		*delta = (long)(*next << extra) - (long)(old << extra);
	} else {
		unsigned long per_native = 1UL << (PAGE_SHIFT - shift);

		*delta = DIV_ROUND_UP(*next, per_native) - DIV_ROUND_UP(old, per_native);
	}
	return 0;
}

int mm_account_memory(struct mm_struct *mm, struct mm_struct *policy_mm, long pages)
{
	unsigned long next;
	long delta;
	int ret;

	if (mm_page_size(mm) == PAGE_SIZE)
		return security_vm_enough_memory_mm(policy_mm, pages);
	ret = mm_commit_delta(mm, pages, &next, &delta);
	if (ret)
		return ret;
	ret = security_vm_enough_memory_mm(policy_mm, delta);
	if (!ret)
		mm->committed_user_pages = next;
	return ret;
}

void mm_acct_memory(struct mm_struct *mm, long pages)
{
	unsigned long next;
	long delta;

	if (mm_page_size(mm) == PAGE_SIZE) {
		vm_acct_memory(pages);
		return;
	}
	if (WARN_ON_ONCE(mm_commit_delta(mm, pages, &next, &delta)))
		return;
	vm_acct_memory(delta);
	mm->committed_user_pages = next;
}

void mm_unacct_memory(struct mm_struct *mm, unsigned long pages)
{
	mm_acct_memory(mm, -(long)pages);
}

/* Shared ownership for consumers whose native page spans contain user slots. */
struct mm_subpage_refs {
	refcount_t refs;
	unsigned int count;
	unsigned int capacity;
	struct mm_subpage *slots[];
};

struct mm_subpage_refs *mm_subpage_refs_get(struct mm_subpage_refs *refs)
{
	if (refs)
		refcount_inc(&refs->refs);
	return refs;
}
EXPORT_SYMBOL_GPL(mm_subpage_refs_get);

void mm_subpage_refs_put(struct mm_subpage_refs *refs)
{
	unsigned int i;

	if (!refs || !refcount_dec_and_test(&refs->refs))
		return;
	for (i = 0; i < refs->count; i++)
		mm_subpage_put(refs->slots[i]);
	kfree(refs);
}
EXPORT_SYMBOL_GPL(mm_subpage_refs_put);

static struct mm_subpage_refs *subpage_refs_new(unsigned int capacity, gfp_t gfp)
{
	struct mm_subpage_refs *refs = kmalloc(struct_size(refs, slots, capacity), gfp);

	if (!refs)
		return NULL;
	refcount_set(&refs->refs, 1);
	refs->count = 0;
	refs->capacity = capacity;
	return refs;
}

static int subpage_refs_prepare(struct mm_subpage_refs **refsp,
				unsigned int needed, gfp_t gfp)
{
	struct mm_subpage_refs *old = *refsp, *refs;
	unsigned int i;

	if (old && refcount_read(&old->refs) == 1 && old->capacity >= needed)
		return 0;
	refs = subpage_refs_new(max(needed, 4U), gfp);
	if (!refs)
		return -ENOMEM;
	if (old) {
		for (i = 0; i < old->count; i++) {
			mm_subpage_get(old->slots[i]);
			refs->slots[refs->count++] = old->slots[i];
		}
	}
	*refsp = refs;
	mm_subpage_refs_put(old);
	return 0;
}

int mm_subpage_refs_add(struct mm_subpage_refs **refsp, struct page *page,
			unsigned int offset, unsigned int len, gfp_t gfp)
{
	struct mm_subpage *slots[PAGE_SIZE / MM_SUBPAGE_SIZE];
	struct mm_subpage_refs *refs = *refsp;
	unsigned int pos, count = 0, i, j;
	int ret = 0;
	bool found = false;

	if (!len || folio_order(page_folio(page)) || !folio_test_anon(page_folio(page)))
		return 0;
	if (offset >= PAGE_SIZE || len > PAGE_SIZE - offset)
		return -EINVAL;
	for (pos = round_down(offset, MM_SUBPAGE_SIZE); pos < offset + len;
	     pos += MM_SUBPAGE_SIZE) {
		struct mm_subpage *slot = mm_subpage_get_from_phys(page_to_phys(page) + pos);

		if (!slot) {
			/* A native anonymous allocation has no quarter owner. */
			ret = found ? -EFAULT : 0;
			goto out;
		}
		found = true;
		for (j = 0; refs && j < refs->count; j++)
			if (refs->slots[j] == slot)
				break;
		if (refs && j < refs->count) {
			mm_subpage_put(slot);
			continue;
		}
		for (j = 0; j < count; j++)
			if (slots[j] == slot)
				break;
		if (j < count)
			mm_subpage_put(slot);
		else
			slots[count++] = slot;
	}
	if (!count)
		return 0;
	if (refs && refs->count > UINT_MAX - count) {
		ret = -ENOMEM;
		goto out;
	}
	ret = subpage_refs_prepare(refsp, (refs ? refs->count : 0) + count, gfp);
	if (ret)
		goto out;
	refs = *refsp;
	for (i = 0; i < count; i++)
		refs->slots[refs->count++] = slots[i];
	return 0;
out:
	for (i = 0; i < count; i++)
		mm_subpage_put(slots[i]);
	return ret;
}
EXPORT_SYMBOL_GPL(mm_subpage_refs_add);

int mm_subpage_refs_prune(struct mm_subpage_refs **refsp,
			 bool (*keep)(const struct mm_subpage *, void *),
			 void *context, gfp_t gfp)
{
	struct mm_subpage_refs *refs = *refsp, *new;
	unsigned int i, count = 0;

	if (!refs)
		return 0;
	for (i = 0; i < refs->count; i++)
		count += keep(refs->slots[i], context);
	if (count == refs->count)
		return 0;
	if (!count) {
		*refsp = NULL;
		mm_subpage_refs_put(refs);
		return 0;
	}
	if (refcount_read(&refs->refs) == 1) {
		for (i = 0, count = 0; i < refs->count; i++) {
			if (keep(refs->slots[i], context))
				refs->slots[count++] = refs->slots[i];
			else
				mm_subpage_put(refs->slots[i]);
		}
		refs->count = count;
		return 0;
	}
	new = subpage_refs_new(count, gfp);
	if (!new)
		return -ENOMEM;
	for (i = 0; i < refs->count; i++) {
		if (!keep(refs->slots[i], context))
			continue;
		mm_subpage_get(refs->slots[i]);
		new->slots[new->count++] = refs->slots[i];
	}
	*refsp = new;
	mm_subpage_refs_put(refs);
	return 0;
}
EXPORT_SYMBOL_GPL(mm_subpage_refs_prune);
