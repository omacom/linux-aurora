/* SPDX-License-Identifier: GPL-2.0-only OR MIT */
/* Offline VFS boundary fixture. Production replacement function is appended unchanged. */
#define _GNU_SOURCE
#include <assert.h>
#include <errno.h>
#include <fcntl.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#define GFP_KERNEL 0
#ifndef O_LARGEFILE
#define O_LARGEFILE 0
#endif
#define IS_ERR(p) ((uintptr_t)(p) >= (uintptr_t)(-4095))
#define PTR_ERR(p) ((long)(intptr_t)(p))
#define PTR_ERR_OR_ZERO(p) (IS_ERR(p) ? PTR_ERR(p) : 0)
#define ERR_PTR(n) ((void *)(intptr_t)(n))
#define QSTR_INIT(n, l) { .name = (n), .len = (l) }
#define CAP ((1U << 20) + 32)
typedef uint64_t u64;
/* Supply the existing kernel ABI name to the host-compiled production code. */
typedef unsigned int umode_t;
struct inode {
	int type;
};
struct dentry {
	struct inode inode;
	bool linked;
};
struct path {
	struct dentry *dentry;
	void *mnt;
};
struct file {
	struct path f_path;
	unsigned char bytes[CAP];
	size_t len;
	int refs;
};
struct cred {
	int unused;
};
struct qstr {
	const char *name;
	size_t len;
};
struct renamedata {
	void *mnt_idmap;
	struct dentry *old_parent, *old_dentry, *new_parent, *new_dentry;
	void *delegated_inode;
	unsigned int flags;
};
static struct {
	void *fs;
} init_task;
static struct cred credential;
static struct file directory, temporary_file, canonical;
static struct dentry root_dentry;
static int mount_token, idmap_token;
static int stages[20], mounts, locks, credentials, roots, allocations;
static int attempts, unlinks, renames, temp_syncs, dir_syncs, temp_present,
	collision_count;
static int cleanup_warning, commit_warning, write_error, zero_write,
	write_calls;
static size_t write_at, short_limit;
static bool exists;
static bool publish_at_rename;
static char events[1024];
static void event(char c)
{
	size_t n = strlen(events);

	assert(n + 1 < sizeof(events));
	events[n] = c;
	events[n + 1] = 0;
}
enum {
	ALLOC = 1,
	CRED,
	PARENT_OPEN,
	TEMP_OPEN,
	MOUNT,
	RENAME_LOCK,
	SECURITY_RENAME,
	RENAME,
	TEMP_SYNC,
	DIR_SYNC,
	CLEANUP_LOCK,
	SECURITY_UNLINK,
	UNLINK,
	TARGET_TYPE,
	STALE_SOURCE
};
static bool fixture_path(const char *name)
{
	return strstr(name, "apple-sep-catacomb-") ||
		strstr(name, "aurora-sep-keybag.bin") ||
		strstr(name, "aurora-sep-refkey.bin") ||
		strstr(name, "aurora-sep-refkey-v2.bin");
}
void fixture_reset(void)
{
	memset(&directory, 0, sizeof(directory));
	memset(&temporary_file, 0, sizeof(temporary_file));
	memset(&canonical, 0, sizeof(canonical));
	directory.f_path.mnt = &mount_token;
	directory.f_path.dentry = calloc(1, sizeof(struct dentry));
	assert(directory.f_path.dentry);
	directory.f_path.dentry->inode.type = 2;
	temporary_file.f_path.dentry = calloc(1, sizeof(struct dentry));
	assert(temporary_file.f_path.dentry);
	temporary_file.f_path.dentry->inode.type = 1;
	canonical.f_path.dentry = calloc(1, sizeof(struct dentry));
	assert(canonical.f_path.dentry);
	canonical.f_path.dentry->inode.type = 1;
	temporary_file.f_path.mnt = canonical.f_path.mnt = &mount_token;
	memset(stages, 0, sizeof(stages));
	mounts = locks = credentials = roots = allocations = 0;
	attempts = unlinks = renames = temp_syncs = dir_syncs = temp_present =
		collision_count = 0;
	cleanup_warning = commit_warning = write_error = zero_write =
		write_calls = 0;
	write_at = short_limit = 0;
	exists = false;
	publish_at_rename = false;
	events[0] = 0;
}
void fixture_done(void)
{
	assert(!mounts && !locks && !credentials && !roots && !allocations);
	assert(!directory.refs && !temporary_file.refs && !canonical.refs);
	free(directory.f_path.dentry);
	free(temporary_file.f_path.dentry);
	free(canonical.f_path.dentry);
}
void fixture_set(const void *p, size_t n)
{
	assert(n <= CAP);
	memcpy(canonical.bytes, p, n);
	canonical.len = n;
	exists = true;
}
void fixture_race_publication(void)
{
	publish_at_rename = true;
}
size_t fixture_get(void *p, size_t n)
{
	if (!exists)
		return 0;
	assert(n >= canonical.len);
	memcpy(p, canonical.bytes, canonical.len);
	return canonical.len;
}
void fixture_fault(int stage, int error)
{
	assert(stage > 0 && stage < 20);
	stages[stage] = error;
}
void fixture_write_fault(size_t at, int error, int zero)
{
	write_at = at;
	write_error = error;
	zero_write = zero;
}
void fixture_short(size_t n)
{
	short_limit = n;
}
void fixture_collisions(int n)
{
	collision_count = n;
}
int fixture_count(int n)
{
	switch (n) {
	case 0:
		return renames;
	case 1:
		return temp_syncs;
	case 2:
		return dir_syncs;
	case 3:
		return temp_present;
	case 4:
		return unlinks;
	case 5:
		return commit_warning;
	case 6:
		return cleanup_warning;
	case 7:
		return attempts;
	default:
		abort();
	}
}
const char *fixture_events(void)
{
	return events;
}
static void *kstrndup(const char *p, size_t n, int g)
{
	assert(g == GFP_KERNEL);
	if (stages[ALLOC])
		return NULL;
	char *s = malloc(n + 1);

	assert(s);
	memcpy(s, p, n);
	s[n] = 0;
	allocations++;
	return s;
}
static void kfree(void *p)
{
	free(p);
	allocations--;
	assert(allocations >= 0);
}
static struct cred *prepare_kernel_cred(void *task)
{
	assert(task == &init_task);
	return stages[CRED] ? NULL : &credential;
}
static void task_lock(void *p)
{
	assert(p == &init_task);
}
static void task_unlock(void *p)
{
	assert(p == &init_task);
}
static void get_fs_root(void *fs, struct path *p)
{
	(void)fs;
	*p = (struct path){ &root_dentry, &mount_token };
	roots++;
}
static void path_put(struct path *p)
{
	assert(p->dentry == &root_dentry);
	roots--;
}
static const struct cred *override_creds(struct cred *p)
{
	assert(p == &credential);
	credentials++;
	return &credential;
}
static struct cred *revert_creds(const struct cred *p)
{
	assert(p == &credential);
	credentials--;
	return &credential;
}
static void put_cred(struct cred *p)
{
	assert(p == &credential);
}
static struct file *file_open_root(const struct path *p, const char *name,
				   int flags, umode_t mode)
{
	assert(credentials == 1);
	if (p->dentry == &root_dentry) {
		assert(!strcmp(name, "/var/lib"));
		assert(flags == (O_RDONLY | O_DIRECTORY | O_LARGEFILE) &&
		       mode == 0);
		if (stages[PARENT_OPEN])
			return ERR_PTR(-stages[PARENT_OPEN]);
		directory.refs++;
		return &directory;
	}
	assert(p == &directory.f_path);
	assert(flags ==
	       (O_WRONLY | O_CREAT | O_EXCL | O_NOFOLLOW | O_LARGEFILE));
	assert(mode == 0600);
	assert(strlen(name) == 41 && !strncmp(name, ".sep-", 5));
	attempts++;
	if (collision_count-- > 0)
		return ERR_PTR(-EEXIST);
	if (stages[TEMP_OPEN])
		return ERR_PTR(-stages[TEMP_OPEN]);
	assert(!temp_present);
	temp_present = 1;
	temporary_file.refs++;
	temporary_file.f_path.dentry->linked = true;
	temporary_file.len = 0;
	event('C');
	return &temporary_file;
}
static void get_random_bytes(void *p, size_t n)
{
	memset(p, attempts + 1, n);
}
static ssize_t kernel_write(struct file *f, const void *buf, size_t n,
			    loff_t *pos)
{
	assert(f == &temporary_file || f == &canonical);
	assert(!mounts);
	assert(*pos >= 0 && (size_t)*pos + n <= CAP);
	write_calls++;
	if ((write_error || zero_write) && (size_t)*pos >= write_at)
		return zero_write ? 0 : -write_error;
	if (short_limit && n > short_limit)
		n = short_limit;
	if ((write_error || zero_write) && (size_t)*pos < write_at &&
	    n > write_at - (size_t)*pos)
		n = write_at - (size_t)*pos;
	memcpy(f->bytes + *pos, buf, n);
	*pos += n;
	if ((size_t)*pos > f->len)
		f->len = *pos;
	event('W');
	return n;
}
static int vfs_fsync(struct file *f, int data)
{
	assert(!data && !locks && !mounts);
	if (f == &directory) {
		dir_syncs++;
		event('D');
		return -stages[DIR_SYNC];
	}
	assert(f == &temporary_file || f == &canonical);
	temp_syncs++;
	event('F');
	return -stages[TEMP_SYNC];
}
static int mnt_want_write(void *m)
{
	assert(m == &mount_token);
	if (stages[MOUNT])
		return -stages[MOUNT];
	assert(!mounts);
	mounts++;
	return 0;
}
static void mnt_drop_write(void *m)
{
	assert(m == &mount_token && mounts == 1 && !locks);
	mounts--;
}
static void *file_mnt_idmap(struct file *f)
{
	assert(f == &directory);
	return &idmap_token;
}
static struct inode *file_inode(struct file *f)
{
	return &f->f_path.dentry->inode;
}
static int start_renaming_dentry(struct renamedata *rd, int flags,
				 struct dentry *old, struct qstr *name)
{
	assert(credentials == 1 && mounts == 1 && !locks && flags == 0);
	assert(rd->mnt_idmap == &idmap_token);
	assert(rd->old_parent == directory.f_path.dentry &&
	       rd->new_parent == directory.f_path.dentry);
	assert(old == temporary_file.f_path.dentry);
	assert(name->len == strlen(name->name) && fixture_path(name->name));
	if (stages[RENAME_LOCK])
		return -stages[RENAME_LOCK];
	if (stages[STALE_SOURCE])
		return -EINVAL;
	locks++;
	if (publish_at_rename) {
		fixture_set("competing", 9);
		publish_at_rename = false;
	}
	rd->old_dentry = old;
	rd->new_dentry = canonical.f_path.dentry;
	rd->new_dentry->linked = exists;
	rd->new_dentry->inode.type = stages[TARGET_TYPE] ? stages[TARGET_TYPE] :
							   1;
	return 0;
}
static bool d_is_positive(struct dentry *d)
{
	return d->linked;
}
static bool d_is_reg(struct dentry *d)
{
	return d->inode.type == 1;
}
static int security_path_rename(const struct path *a, struct dentry *b,
				const struct path *c, struct dentry *d,
				unsigned int flags)
{
	assert(a == &directory.f_path && c == a &&
	       b == temporary_file.f_path.dentry &&
	       d == canonical.f_path.dentry && !flags && locks);
	return -stages[SECURITY_RENAME];
}
static int vfs_rename(struct renamedata *rd)
{
	assert(locks && mounts && credentials);
	assert(strchr(events, 'F'));
	assert(rd->delegated_inode == NULL && rd->flags == 0);
	if (stages[RENAME])
		return -stages[RENAME];
	canonical.len = temporary_file.len;
	memcpy(canonical.bytes, temporary_file.bytes, canonical.len);
	exists = true;
	temp_present = 0;
	renames++;
	event('R');
	return 0;
}
static void end_renaming(struct renamedata *rd)
{
	assert(rd->old_parent == directory.f_path.dentry && locks == 1);
	locks--;
}
static struct dentry *start_removing_dentry(struct dentry *parent,
					    struct dentry *child)
{
	assert(parent == directory.f_path.dentry &&
	       child == temporary_file.f_path.dentry && mounts && !locks);
	if (stages[CLEANUP_LOCK])
		return ERR_PTR(-stages[CLEANUP_LOCK]);
	if (stages[STALE_SOURCE])
		return ERR_PTR(-EINVAL);
	locks++;
	return child;
}
static int security_path_unlink(const struct path *p, struct dentry *d)
{
	assert(p == &directory.f_path && d == temporary_file.f_path.dentry &&
	       locks);
	return -stages[SECURITY_UNLINK];
}
static int vfs_unlink(void *idmap, struct inode *inode, struct dentry *d,
		      void *delegated)
{
	assert(idmap == &idmap_token && inode == file_inode(&directory) &&
	       d == temporary_file.f_path.dentry && !delegated && locks &&
	       mounts);
	if (stages[UNLINK])
		return -stages[UNLINK];
	assert(temp_present);
	temp_present = 0;
	unlinks++;
	event('U');
	return 0;
}
static void end_removing(struct dentry *d)
{
	assert(d == temporary_file.f_path.dentry && locks == 1);
	locks--;
}
static void fput(struct file *f)
{
	assert(f->refs > 0);
	f->refs--;
}
/* Leaves for the existing production parent-sync operation used on retry. */
static struct dentry *dget_parent(struct dentry *d)
{
	assert(d == canonical.f_path.dentry);
	roots++;
	return directory.f_path.dentry;
}
static void dput(struct dentry *d)
{
	assert(d == directory.f_path.dentry && roots > 0);
	roots--;
}
static struct file *dentry_open(const struct path *p, int flags,
			       const struct cred *cred)
{
	assert(p->mnt == &mount_token && p->dentry == directory.f_path.dentry);
	assert(flags == (O_RDONLY | O_DIRECTORY) && cred == &credential);
	if (stages[PARENT_OPEN])
		return ERR_PTR(-stages[PARENT_OPEN]);
	directory.refs++;
	return &directory;
}
#define pr_err(...) (++commit_warning)
#define pr_warn(...) (++cleanup_warning)
/* Existing C/Rust I/O ABI for testing the actual Catacomb reader and old writer. */
void *sep_store_open_ro(const char *p)
{
	assert(fixture_path(p));
	if (stages[16])
		return ERR_PTR(-stages[16]);
	if (!exists)
		return ERR_PTR(-ENOENT);
	canonical.refs++;
	return &canonical;
}
void *sep_store_open_trunc(const char *p)
{
	(void)p;
	canonical.len = 0;
	exists = true;
	canonical.refs++;
	return &canonical;
}
void *sep_store_open(const char *p)
{
	return sep_store_open_trunc(p);
}
void *sep_store_open_host(const char *p, bool *created)
{
	*created = !exists;
	return sep_store_open_trunc(p);
}
void *sep_store_open_block(const char *p, int w)
{
	(void)p;
	(void)w;
	abort();
}
void sep_store_close(void *p)
{
	fput(p);
}
long long sep_store_size(void *p)
{
	return ((struct file *)p)->len;
}
long sep_store_read(void *p, long long off, void *buf, size_t n)
{
	struct file *f = p;

	if ((size_t)off >= f->len)
		return 0;
	if (n > f->len - off)
		n = f->len - off;
	memcpy(buf, f->bytes + off, n);
	return n;
}
long sep_store_write(void *p, long long off, const void *buf, size_t n)
{
	loff_t pos = off;

	return kernel_write(p, buf, n, &pos);
}
int sep_store_sync(void *p)
{
	return vfs_fsync(p, 0);
}
int sep_random_bytes(void *p, size_t n)
{
	memset(p, 1, n);
	return 0;
}
