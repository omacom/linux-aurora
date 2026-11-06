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
#include <linux/mount.h>
#include <linux/namei.h>
#include <linux/random.h>
#include <linux/sched.h>
#include <linux/security.h>
#include <linux/slab.h>
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

static int store_write_all(struct file *file, const void *buf, size_t len,
			   loff_t *pos)
{
	while (len) {
		ssize_t n = kernel_write(file, buf, len, pos);

		if (n < 0)
			return n;
		if (!n)
			return -EIO;
		buf = (const char *)buf + n;
		len -= n;
	}
	return 0;
}

/* Remove only the temporary dentry we opened, never a name reused by another task. */
static int store_remove_temp(struct file *dir, struct file *temp)
{
	struct dentry *dentry;
	int err;

	err = mnt_want_write(dir->f_path.mnt);
	if (err)
		return err;
	dentry = start_removing_dentry(dir->f_path.dentry, temp->f_path.dentry);
	if (IS_ERR(dentry)) {
		err = PTR_ERR(dentry);
	} else {
		err = security_path_unlink(&dir->f_path, dentry);
		if (!err)
			err = vfs_unlink(file_mnt_idmap(dir), file_inode(dir), dentry, NULL);
		end_removing(dentry);
	}
	mnt_drop_write(dir->f_path.mnt);
	return err;
}

/*
 * Durable full-file replacement, without truncating the previous generation.
 * The caller owns both byte slices until return. Only the final pathname is
 * replaced; symlinks and non-regular destinations are refused under rename's
 * directory lock. A successful rename is the visibility commit point: a later
 * directory fsync failure returns an error but cannot safely undo that rename.
 */
static int store_replace(const char *path, const void *head, size_t head_len,
			 const void *body, size_t body_len)
{
	const char *base = strrchr(path, '/');
	const struct cred *old;
	struct cred *kern;
	struct file *dir, *temp = NULL;
	struct path root;
	struct renamedata rd = {};
	struct qstr name;
	char *parent;
	char temporary[48];
	u64 nonce[2];
	loff_t pos = 0;
	bool renamed = false;
	int err, cleanup, attempt;

	if (!base || base == path || !base[1])
		return -EINVAL;
	parent = kstrndup(path, base - path, GFP_KERNEL);
	if (!parent)
		return -ENOMEM;
	kern = prepare_kernel_cred(&init_task);
	if (!kern) {
		kfree(parent);
		return -ENOMEM;
	}
	task_lock(&init_task);
	get_fs_root(init_task.fs, &root);
	task_unlock(&init_task);
	old = override_creds(kern);
	dir = file_open_root(&root, parent, O_RDONLY | O_DIRECTORY | O_LARGEFILE, 0);
	path_put(&root);
	kfree(parent);
	if (IS_ERR(dir)) {
		err = PTR_ERR(dir);
		goto out_creds;
	}

	/* Exclusive creation never adopts or truncates a previous temporary file. */
	for (attempt = 0; attempt < 4; attempt++) {
		get_random_bytes(nonce, sizeof(nonce));
		snprintf(temporary, sizeof(temporary), ".sep-%016llx%016llx.tmp",
			 (unsigned long long)nonce[0], (unsigned long long)nonce[1]);
		temp = file_open_root(&dir->f_path, temporary,
			O_WRONLY | O_CREAT | O_EXCL | O_NOFOLLOW | O_LARGEFILE, 0600);
		if (PTR_ERR_OR_ZERO(temp) != -EEXIST)
			break;
	}
	if (IS_ERR(temp)) {
		err = PTR_ERR(temp);
		temp = NULL;
		goto out_dir;
	}
	err = store_write_all(temp, head, head_len, &pos);
	if (!err)
		err = store_write_all(temp, body, body_len, &pos);
	if (!err)
		err = vfs_fsync(temp, 0);
	if (err)
		goto out_temp;

	/* Do not hold a mount write claim across kernel_write's own freeze lock. */
	err = mnt_want_write(dir->f_path.mnt);
	if (err)
		goto out_temp;
	rd.mnt_idmap = file_mnt_idmap(dir);
	rd.old_parent = dir->f_path.dentry;
	rd.new_parent = dir->f_path.dentry;
	name = (struct qstr)QSTR_INIT(base + 1, strlen(base + 1));
	err = start_renaming_dentry(&rd, 0, temp->f_path.dentry, &name);
	if (!err) {
		if (d_is_positive(rd.new_dentry) && !d_is_reg(rd.new_dentry))
			err = -EINVAL;
		else
			err = security_path_rename(&dir->f_path, rd.old_dentry,
						   &dir->f_path, rd.new_dentry, 0);
		if (!err)
			err = vfs_rename(&rd);
		end_renaming(&rd);
	}
	mnt_drop_write(dir->f_path.mnt);
	if (!err) {
		renamed = true;
		err = vfs_fsync(dir, 0);
		if (err)
			pr_err("apple-sep: replacement is visible but parent directory sync failed (%d)\n", err);
	}
out_temp:
	if (!renamed) {
		cleanup = store_remove_temp(dir, temp);
		if (cleanup)
			pr_warn("apple-sep: temporary store cleanup failed (%d)\n", cleanup);
	}
	fput(temp);
out_dir:
	fput(dir);
out_creds:
	put_cred(revert_creds(old));
	return err;
}

int sep_store_replace(const char *path, const void *head, size_t head_len,
		      const void *body, size_t body_len)
{
	return store_replace(path, head, head_len, body, body_len);
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
