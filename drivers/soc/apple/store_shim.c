/* SPDX-License-Identifier: GPL-2.0-only OR MIT */
/* Copyright 2026 Dj */
/*
 * Backing-store file access. In C because the O_* flags are macro-derived and
 * absent from the Rust bindings.
 *
 * These files are the driver's own. Opens run under kernel credentials so
 * access never depends on which task triggered the operation: a keyctl(2) call
 * from an unprivileged process reaches unseal in that process's context, and
 * without this the driver could not read its own root-only store.
 */

#include <linux/cred.h>
#include <linux/blkdev.h>
#include <linux/err.h>
#include <linux/file.h>
#include <linux/fs.h>
#include <linux/fs_struct.h>
#include <linux/namei.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/types.h>

#include "shim.h"

static struct file *open_as_kernel(const char *path, int flags, umode_t mode)
{
	const struct cred *old;
	struct cred *kern;
	struct file *f;
	struct path root;

	kern = prepare_kernel_cred(&init_task);
	if (!kern)
		return ERR_PTR(-ENOMEM);

	task_lock(&init_task);
	get_fs_root(init_task.fs, &root);
	task_unlock(&init_task);

	old = override_creds(kern);
	f = file_open_root(&root, path, flags, mode);
	put_cred(revert_creds(old));
	path_put(&root);

	return f;
}

/* Opens the store, creating if absent. Mode 0600: the file is the driver's own. */
void *sep_store_open(const char *path)
{
	struct file *f = open_as_kernel(path, O_RDWR | O_CREAT | O_LARGEFILE, 0600);

	return IS_ERR(f) ? NULL : f;
}

/*
 * As sep_store_open(), but reports whether this call created the file, so that
 * only a new file is ever initialised. Returns an error pointer on failure.
 */
void *sep_store_open_host(const char *path, bool *created)
{
	struct file *f;

	*created = false;
	f = open_as_kernel(path, O_RDWR | O_CREAT | O_EXCL | O_LARGEFILE, 0600);
	if (PTR_ERR_OR_ZERO(f) == -EEXIST)
		return open_as_kernel(path, O_RDWR | O_LARGEFILE, 0);
	if (!IS_ERR(f))
		*created = true;
	return f;
}

/*
 * Opens truncated to empty, for callers that rewrite the whole file each time,
 * so a shorter record cannot leave stale trailing bytes from a larger one.
 */
void *sep_store_open_trunc(const char *path)
{
	struct file *f = open_as_kernel(path, O_RDWR | O_CREAT | O_TRUNC | O_LARGEFILE, 0600);

	return IS_ERR(f) ? NULL : f;
}

/*
 * Opens an existing file read-only, NULL if absent. O_RDONLY and no O_CREAT so
 * the seed cannot be created, truncated or written by mistake.
 */
void *sep_store_open_ro(const char *path)
{
	struct file *f = open_as_kernel(path, O_RDONLY | O_LARGEFILE, 0);

	return IS_ERR(f) ? NULL : f;
}

void *sep_store_open_block(const char *path, int writable)
{
	struct file *f = open_as_kernel(path,
					writable ? O_RDWR | O_LARGEFILE : O_RDONLY | O_LARGEFILE,
					0);

	if (IS_ERR(f))
		return NULL;
	if (!S_ISBLK(file_inode(f)->i_mode) ||
	    (writable && bdev_read_only(file_bdev(f)))) {
		filp_close(f, NULL);
		return NULL;
	}
	return f;
}

void sep_store_close(void *handle)
{
	if (handle)
		filp_close((struct file *)handle, NULL);
}

long long sep_store_size(void *handle)
{
	struct file *f = handle;

	if (S_ISBLK(file_inode(f)->i_mode))
		return bdev_nr_bytes(file_bdev(f));
	return i_size_read(file_inode(f));
}

long sep_store_read(void *handle, long long off, void *buf, size_t len)
{
	struct file *f = handle;
	loff_t pos = off;

	return kernel_read(f, buf, len, &pos);
}

long sep_store_write(void *handle, long long off, const void *buf,
			    size_t len)
{
	struct file *f = handle;
	loff_t pos = off;

	return kernel_write(f, buf, len, &pos);
}

int sep_store_sync(void *handle)
{
	return vfs_fsync((struct file *)handle, 0);
}

/*
 * Flushes the directory that holds an open store file. fsync() of the file
 * itself is not guaranteed to make its name in that directory durable, so a
 * file created this boot could be missing after a crash. Opened under kernel
 * credentials, as the file was.
 */
int sep_store_sync_parent(void *handle)
{
	struct file *f = handle;
	struct cred *kern;
	struct file *dir;
	struct path parent;
	int ret;

	kern = prepare_kernel_cred(&init_task);
	if (!kern)
		return -ENOMEM;

	/* The file holds its mount; the parent dentry is pinned until dput(). */
	parent.mnt = f->f_path.mnt;
	parent.dentry = dget_parent(f->f_path.dentry);
	dir = dentry_open(&parent, O_RDONLY | O_DIRECTORY, kern);
	dput(parent.dentry);
	put_cred(kern);
	if (IS_ERR(dir))
		return PTR_ERR(dir);

	ret = vfs_fsync(dir, 0);
	fput(dir);
	return ret;
}

/*
 * Removes a driver-owned file under init's root and kernel credentials, the
 * same way the opens above run. The parent directory stays. A negative
 * dentry is ENOENT. Passes NULL for the delegation out-parameter: this file
 * is not NFS-exported, and a lease break is not waited on.
 */
int sep_store_unlink(const char *path)
{
	const char *slash, *base;
	char *dir;
	size_t dirlen;
	struct cred *kern;
	const struct cred *old;
	struct path root;
	struct path parent;
	struct dentry *dentry;
	struct qstr name;
	int ret;

	if (!path || path[0] != '/')
		return -EINVAL;
	slash = strrchr(path, '/');
	if (!slash || slash == path || !slash[1])
		return -EINVAL;
	base = slash + 1;
	dirlen = slash - path;
	dir = kmalloc(dirlen + 1, GFP_KERNEL);
	if (!dir)
		return -ENOMEM;
	memcpy(dir, path, dirlen);
	dir[dirlen] = '\0';

	kern = prepare_kernel_cred(&init_task);
	if (!kern) {
		kfree(dir);
		return -ENOMEM;
	}
	task_lock(&init_task);
	get_fs_root(init_task.fs, &root);
	task_unlock(&init_task);

	old = override_creds(kern);
	ret = vfs_path_lookup(root.dentry, root.mnt, dir, LOOKUP_DIRECTORY,
			      &parent);
	kfree(dir);
	if (ret)
		goto out_cred;

	ret = mnt_want_write(parent.mnt);
	if (ret)
		goto out_parent;

	name = QSTR(base);
	inode_lock_nested(d_inode(parent.dentry), I_MUTEX_PARENT);
	dentry = lookup_one(mnt_idmap(parent.mnt), &name, parent.dentry);
	if (IS_ERR(dentry)) {
		ret = PTR_ERR(dentry);
		goto unlock;
	}
	if (d_is_negative(dentry))
		ret = -ENOENT;
	else
		ret = vfs_unlink(mnt_idmap(parent.mnt), d_inode(parent.dentry),
				 dentry, NULL);
	dput(dentry);
unlock:
	inode_unlock(d_inode(parent.dentry));
	mnt_drop_write(parent.mnt);
out_parent:
	path_put(&parent);
out_cred:
	put_cred(revert_creds(old));
	path_put(&root);
	put_cred(kern);
	return ret;
}
