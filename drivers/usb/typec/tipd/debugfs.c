// SPDX-License-Identifier: GPL-2.0
/*
 * TI TPS6598x / Apple CD321x / SN201202x debugfs interface
 *
 * /sys/kernel/debug/usb/tipd/<device>/command
 *	Only with CONFIG_TYPEC_TPS6598X_DEBUG_COMMANDS.
 *	write: 4CC command followed by 0..64 bytes for DATA1. The command is
 *	       executed as a whole under the driver's lock. Only a small set
 *	       of commands used to talk to a connected Apple device (as
 *	       tuxvdmtool does) is accepted.
 *	read:  the 64 bytes of DATA1 that the last completed command of this
 *	       open file left behind.
 *
 * /sys/kernel/debug/usb/tipd/<device>/regs
 *	read-only register access: pread() with the register as offset
 *	(0x00..0x7f) and the length (1..64 bytes) as count.
 * /sys/kernel/debug/usb/tipd/<device>/replug
 *	Only on a port tipd can replug its partner on (an SN201202x whose DT
 *	gives apple,debug-lock-key, with a USB role switch).
 *	write: the seconds (1..10) the partner is disconnected for. One replug,
 *	       queued as the driver queues its own after a sleep (see
 *	       tipd_port_replug()); -EBUSY while one is queued and not yet
 *	       running. A write while one runs queues another after it.
 *
 * An open "command" file that put the controller into "DBMa" leaves that
 * mode (DBMa 0, then LOCK 0000) when it is released, unless the controller
 * was in "DBMa" already when the driver probed: that mode belongs to
 * whoever set it up before Linux (e.g. a boot loader arming Debug USB).
 */

#include <linux/debugfs.h>
#include <linux/export.h>
#include <linux/fs.h>
#include <linux/kstrtox.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/uaccess.h>
#include <linux/usb.h>

#include "tps6598x.h"

#define TPS_REG_MAX	0x7f

/*
 * Whether an open "command" file holds the controller in "DBMa". Read
 * without tps->lock, by runtime suspend: remove unregisters only once the
 * bus has woken the controller, so no runtime suspend runs then.
 */
bool tipd_debugfs_dbma_held(struct tps6598x *tps)
{
	struct tipd_debugfs *debugfs = READ_ONCE(tps->debugfs);

	return debugfs && READ_ONCE(debugfs->dbma_holders);
}
EXPORT_SYMBOL_GPL(tipd_debugfs_dbma_held);

struct tipd_debugfs_cmd {
	char cmd[4];
	u8 min_len;
	u8 max_len;
	u32 timeout_ms;
};

static const struct tipd_debugfs_cmd tipd_debugfs_cmds[] = {
	{ "LOCK", 4, 4, 1000 },		/* unlock key, 0000 locks again */
	{ "DBMa", 1, 1, 1000 },		/* 1: enter, 0: leave debug mode */
	{ "VDMs", 5, 29, 200 },		/* SOP type/count + 1..7 VDOs */
	{ "DVEn", 4, 28, 1000 },	/* local end of a serial/debug route */
	{ "DISC", 0, 1, 1000 },		/* simulated disconnect */
};

struct tipd_debugfs_file {
	struct tps6598x *tps;
	bool holds_dbma;
	bool have_result;
	u8 result[TPS_MAX_LEN];
};

static struct dentry *tipd_debugfs_root;
static DEFINE_MUTEX(tipd_debugfs_root_lock);

static const struct tipd_debugfs_cmd *tipd_debugfs_find_cmd(const u8 *cmd)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(tipd_debugfs_cmds); i++)
		if (!memcmp(tipd_debugfs_cmds[i].cmd, cmd, 4))
			return &tipd_debugfs_cmds[i];

	return NULL;
}

/* Leave the debug mode entered through debugfs. Called with tps->lock held. */
static void tipd_debugfs_leave_dbma(struct tps6598x *tps)
{
	static const u8 lock_key[4];
	u8 dbma = 0;
	int ret;

	lockdep_assert_held(&tps->lock);

	if (tps->dbma_at_probe)
		return;

	ret = tps6598x_exec_cmd_tmo(tps, "DBMa", sizeof(dbma), &dbma, 0, NULL,
				    1000, 0);
	if (ret)
		dev_warn(tps->dev, "debugfs: failed to leave \"DBMa\": %d\n", ret);

	ret = tps6598x_exec_cmd_tmo(tps, "LOCK", sizeof(lock_key), lock_key,
				    0, NULL, 1000, 0);
	if (ret)
		dev_warn(tps->dev, "debugfs: failed to lock: %d\n", ret);
}

static int tipd_debugfs_command_open(struct inode *inode, struct file *file)
{
	struct tipd_debugfs_file *f;

	f = kzalloc_obj(*f);
	if (!f)
		return -ENOMEM;

	f->tps = inode->i_private;
	file->private_data = f;

	return 0;
}

static int tipd_debugfs_command_release(struct inode *inode, struct file *file)
{
	struct tipd_debugfs_file *f = file->private_data;
	struct dentry *dentry = file->f_path.dentry;
	struct tps6598x *tps = f->tps;

	/*
	 * debugfs calls ->release even after the file was removed, when tps
	 * may be gone. tipd_debugfs_unregister() has then left "DBMa" itself.
	 */
	if (f->holds_dbma && !debugfs_file_get(dentry)) {
		scoped_guard(mutex, &tps->lock) {
			if (!--tps->debugfs->dbma_holders)
				tipd_debugfs_leave_dbma(tps);
		}
		debugfs_file_put(dentry);
	}

	kfree(f);

	return 0;
}

static ssize_t tipd_debugfs_command_write(struct file *file,
					  const char __user *ubuf,
					  size_t count, loff_t *ppos)
{
	struct tipd_debugfs_file *f = file->private_data;
	struct tps6598x *tps = f->tps;
	const struct tipd_debugfs_cmd *cmd;
	u8 buf[4 + TPS_MAX_LEN];
	u8 out[TPS_MAX_LEN] = { };
	size_t len;
	int ret;

	if (count < 4 || count > sizeof(buf))
		return -EINVAL;

	if (copy_from_user(buf, ubuf, count))
		return -EFAULT;

	cmd = tipd_debugfs_find_cmd(buf);
	if (!cmd)
		return -EOPNOTSUPP;

	len = count - 4;
	if (len < cmd->min_len || len > cmd->max_len)
		return -EINVAL;
	if (!memcmp(cmd->cmd, "VDMs", 4) && (len - 1) % 4)
		return -EINVAL;
	if (!memcmp(cmd->cmd, "DVEn", 4) && len % 4)
		return -EINVAL;
	if (!memcmp(cmd->cmd, "DBMa", 4) && buf[4] > 1)
		return -EINVAL;

	guard(mutex)(&tps->lock);

	/* Preserve inherited mode and the leases held by other open files. */
	if ((!memcmp(cmd->cmd, "DBMa", 4) && !buf[4] &&
	     (tps->dbma_at_probe ||
	      tps->debugfs->dbma_holders > (unsigned int)f->holds_dbma)) ||
	    (!memcmp(cmd->cmd, "LOCK", 4) && !memchr_inv(buf + 4, 0, 4) &&
	     (tps->dbma_at_probe || tps->debugfs->dbma_holders)))
		return -EBUSY;

	ret = tps6598x_exec_cmd_tmo(tps, cmd->cmd, len, len ? &buf[4] : NULL,
				    sizeof(out), out, cmd->timeout_ms, 0);

	/* DATA1 was read back if the command ran, even if its task failed */
	f->have_result = !ret ||
			 (ret == -ETIMEDOUT && out[0] == TPS_TASK_TIMEOUT) ||
			 (ret == -EPERM && out[0] == TPS_TASK_REJECTED);
	if (f->have_result)
		memcpy(f->result, out, sizeof(out));

	if (!ret && !memcmp(cmd->cmd, "DBMa", 4)) {
		if (buf[4] && !(out[0] & 0xf) && !f->holds_dbma) {
			f->holds_dbma = true;
			tps->debugfs->dbma_holders++;
		} else if (!buf[4] && f->holds_dbma) {
			f->holds_dbma = false;
			tps->debugfs->dbma_holders--;
		}
	}

	return ret ?: count;
}

static ssize_t tipd_debugfs_command_read(struct file *file, char __user *ubuf,
					 size_t count, loff_t *ppos)
{
	struct tipd_debugfs_file *f = file->private_data;

	guard(mutex)(&f->tps->lock);
	if (!f->have_result)
		return -ENODATA;

	return simple_read_from_buffer(ubuf, count, ppos, f->result,
				       sizeof(f->result));
}

static const struct file_operations tipd_debugfs_command_fops = {
	.owner = THIS_MODULE,
	.open = tipd_debugfs_command_open,
	.release = tipd_debugfs_command_release,
	.write = tipd_debugfs_command_write,
	.read = tipd_debugfs_command_read,
	.llseek = default_llseek,
};

static ssize_t tipd_debugfs_regs_read(struct file *file, char __user *ubuf,
				      size_t count, loff_t *ppos)
{
	struct tps6598x *tps = file->private_data;
	u8 buf[TPS_MAX_LEN];
	int ret;

	if (*ppos < 0 || *ppos > TPS_REG_MAX)
		return -EINVAL;
	if (!count)
		return 0;
	if (count > sizeof(buf))
		return -EINVAL;

	scoped_guard(mutex, &tps->lock)
		ret = tps6598x_block_read(tps, *ppos, buf, count);
	if (ret)
		return ret;

	/* The offset selects a register; it does not advance. */
	if (copy_to_user(ubuf, buf, count))
		return -EFAULT;

	return count;
}

#define TIPD_DEBUGFS_REPLUG_MAX_S	10

static ssize_t tipd_debugfs_replug_write(struct file *file, const char __user *ubuf,
					 size_t count, loff_t *ppos)
{
	struct tps6598x *tps = file->private_data;
	u8 seconds;
	int ret;

	ret = kstrtou8_from_user(ubuf, count, 0, &seconds);
	if (ret)
		return ret;
	if (!seconds || seconds > TIPD_DEBUGFS_REPLUG_MAX_S)
		return -EINVAL;
	ret = tipd_replug_request(tps, seconds);
	return ret ? ret : count;
}

static const struct file_operations tipd_debugfs_replug_fops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.write = tipd_debugfs_replug_write,
	.llseek = noop_llseek,
};

static const struct file_operations tipd_debugfs_regs_fops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.read = tipd_debugfs_regs_read,
	.llseek = default_llseek,
};

void tipd_debugfs_register(struct tps6598x *tps)
{
	struct tipd_debugfs *dbg;

	scoped_guard(mutex, &tipd_debugfs_root_lock) {
		if (!tipd_debugfs_root)
			tipd_debugfs_root = debugfs_create_dir("tipd", usb_debug_root);
	}

	dbg = kzalloc_obj(*dbg);
	if (!dbg)
		return;

	dbg->dentry = debugfs_create_dir(dev_name(tps->dev), tipd_debugfs_root);
	tps->debugfs = dbg;
	if (IS_ENABLED(CONFIG_TYPEC_TPS6598X_DEBUG_COMMANDS))
		debugfs_create_file("command", 0600, dbg->dentry, tps,
				    &tipd_debugfs_command_fops);
	debugfs_create_file("regs", 0400, dbg->dentry, tps,
			    &tipd_debugfs_regs_fops);
	if (tipd_replug_supported(tps) && tps->role_sw)
		debugfs_create_file("replug", 0200, dbg->dentry, tps,
				    &tipd_debugfs_replug_fops);
}

void tipd_debugfs_unregister(struct tps6598x *tps)
{
	struct tipd_debugfs *dbg = tps->debugfs;

	if (!dbg)
		return;

	/* Waits for running file operations, later ones fail with -EIO */
	debugfs_remove_recursive(dbg->dentry);

	scoped_guard(mutex, &tps->lock) {
		if (dbg->dbma_holders)
			tipd_debugfs_leave_dbma(tps);
		tps->debugfs = NULL;
	}

	kfree(dbg);
}

void tipd_debugfs_exit(void)
{
	debugfs_remove(tipd_debugfs_root);
}
