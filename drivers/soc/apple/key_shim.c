/* SPDX-License-Identifier: GPL-2.0-only OR MIT */
/* Application-key files own their Rust context, independently of registration. */
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/mutex.h>
#include <linux/slab.h>

#include "shim.h"

struct sep_key_device {
	struct miscdevice misc;
	struct mutex lock;
	void *ctx;
	int (*open)(void *ctx, void **file);
	void (*release)(void *file);
	long (*ioctl)(void *file, unsigned int cmd, unsigned long arg);
};

struct sep_key_file {
	void *ctx;
	void (*release)(void *file);
	long (*ioctl)(void *file, unsigned int cmd, unsigned long arg);
};

static int sep_key_open(struct inode *inode, struct file *file)
{
	struct sep_key_device *d = container_of(file->private_data,
						struct sep_key_device, misc);
	struct sep_key_file *f;
	int ret;

	f = kzalloc(sizeof(*f), GFP_KERNEL);
	if (!f)
		return -ENOMEM;
	/*
	 * misc_open serializes registration lifetime. Rust clones the device Arc
	 * here; subsequent ioctls and release never borrow the registration.
	 */
	mutex_lock(&d->lock);
	ret = d->ctx ? d->open(d->ctx, &f->ctx) : -ENODEV;
	f->release = d->release;
	f->ioctl = d->ioctl;
	mutex_unlock(&d->lock);
	if (ret) {
		kfree(f);
		return ret;
	}
	file->private_data = f;
	return 0;
}

static int sep_key_release(struct inode *inode, struct file *file)
{
	struct sep_key_file *f = file->private_data;

	f->release(f->ctx);
	kfree(f);
	return 0;
}

static long sep_key_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct sep_key_file *f = file->private_data;

	/* Rust serializes request transitions, never a complete blocking ioctl. */
	return f->ioctl(f->ctx, cmd, arg);
}

static const struct file_operations sep_key_fops = {
	.owner = THIS_MODULE,
	.open = sep_key_open,
	.release = sep_key_release,
	.unlocked_ioctl = sep_key_ioctl,
	.compat_ioctl = compat_ptr_ioctl,
};

void *sep_key_register(void *ctx, int (*open)(void *, void **),
		       void (*release)(void *),
		       long (*ioctl)(void *, unsigned int, unsigned long))
{
	struct sep_key_device *d;

	d = kzalloc(sizeof(*d), GFP_KERNEL);
	if (!d)
		return NULL;
	mutex_init(&d->lock);
	d->ctx = ctx;
	d->open = open;
	d->release = release;
	d->ioctl = ioctl;
	d->misc.minor = MISC_DYNAMIC_MINOR;
	d->misc.name = "sep-keys";
	d->misc.mode = 0600;
	d->misc.fops = &sep_key_fops;
	if (misc_register(&d->misc)) {
		mutex_destroy(&d->lock);
		kfree(d);
		return NULL;
	}
	return d;
}

void sep_key_unregister(void *device)
{
	struct sep_key_device *d = device;

	misc_deregister(&d->misc);
	mutex_lock(&d->lock);
	d->ctx = NULL;
	mutex_unlock(&d->lock);
	/* Each successful open owns its context and copies of the callbacks. */
	mutex_destroy(&d->lock);
	kfree(d);
}
