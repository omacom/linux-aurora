// SPDX-License-Identifier: GPL-2.0-only OR MIT
static const char original[] = "previous validated generation";
static const char header[] = "0123456789abcdef";
static const char body[] = "replacement opaque biometric generation";
static const char path[] = "/var/lib/apple-sep-catacomb-user.bin";
static int passed;
static void setup(void)
{
	fixture_reset();
	fixture_set(original, sizeof(original));
}
static void unchanged(void)
{
	assert(exists && canonical.len == sizeof(original) &&
	       !memcmp(canonical.bytes, original, sizeof(original)));
	assert(!renames && !dir_syncs);
}
static void complete(void)
{
	assert(exists && canonical.len == sizeof(header) - 1 + sizeof(body));
	assert(!memcmp(canonical.bytes, header, sizeof(header) - 1));
	assert(!memcmp(canonical.bytes + sizeof(header) - 1, body,
		       sizeof(body)));
}
static int replace(void)
{
	return sep_store_replace(path, header, sizeof(header) - 1, body,
				 sizeof(body));
}
static void done(const char *name)
{
	fixture_done();
	passed++;
	printf("PASS %s\n", name);
}
int main(void)
{
	struct {
		int stage, error;
		const char *name;
	} failures[] = {
		{ ALLOC, ENOMEM, "parent allocation" },
		{ CRED, ENOMEM, "kernel credential allocation" },
		{ PARENT_OPEN, EACCES, "parent open denied" },
		{ PARENT_OPEN, ENOENT, "parent missing" },
		{ TEMP_OPEN, ENOSPC, "temporary create ENOSPC" },
		{ TEMP_OPEN, EROFS, "temporary create EROFS" },
		{ TEMP_SYNC, EIO, "temporary fsync EIO" },
		{ TEMP_SYNC, ENOSPC, "temporary fsync ENOSPC" },
		{ MOUNT, EROFS, "mount write denied" },
		{ RENAME_LOCK, ENOMEM, "rename lookup allocation" },
		{ SECURITY_RENAME, EPERM, "path security rejects rename" },
		{ RENAME, EIO, "vfs rename EIO" },
		{ RENAME, EWOULDBLOCK, "vfs rename delegation" },
		{ RENAME, EBUSY, "vfs rename mountpoint" },
	};
	for (size_t i = 0; i < sizeof(failures) / sizeof(*failures); i++) {
		setup();
		fixture_fault(failures[i].stage, failures[i].error);
		assert(replace() == -failures[i].error);
		unchanged();
		done(failures[i].name);
	}
	setup();
	fixture_write_fault(0, EIO, 0);
	assert(replace() == -EIO);
	unchanged();
	assert(!temp_present);
	done("header write EIO");
	setup();
	fixture_write_fault(7, ENOSPC, 0);
	assert(replace() == -ENOSPC);
	unchanged();
	done("partial header then ENOSPC");
	setup();
	fixture_write_fault(16, EIO, 0);
	assert(replace() == -EIO);
	unchanged();
	done("payload write EIO");
	setup();
	fixture_write_fault(22, EIO, 0);
	assert(replace() == -EIO);
	unchanged();
	done("partial payload then EIO");
	setup();
	fixture_write_fault(0, 0, 1);
	assert(replace() == -EIO);
	unchanged();
	done("zero-progress header");
	setup();
	fixture_write_fault(18, 0, 1);
	assert(replace() == -EIO);
	unchanged();
	done("zero-progress payload");
	setup();
	fixture_fault(TARGET_TYPE, 2);
	assert(replace() == -EINVAL);
	unchanged();
	done("directory target refused");
	setup();
	fixture_fault(TARGET_TYPE, 3);
	assert(replace() == -EINVAL);
	unchanged();
	done("symlink target refused");
	setup();
	fixture_fault(STALE_SOURCE, 1);
	assert(replace() == -EINVAL);
	unchanged();
	assert(temp_present && cleanup_warning);
	done("source moved; cleanup does not remove reused name");
	for (int i = CLEANUP_LOCK; i <= UNLINK; i++) {
		setup();
		fixture_write_fault(16, EIO, 0);
		fixture_fault(i, EPERM);
		assert(replace() == -EIO);
		unchanged();
		assert(temp_present && cleanup_warning);
		done(i == CLEANUP_LOCK	  ? "cleanup lookup failure" :
		     i == SECURITY_UNLINK ? "cleanup security failure" :
					    "cleanup unlink failure");
	}
	setup();
	fixture_collisions(4);
	assert(replace() == -EEXIST);
	unchanged();
	assert(attempts == 4 && !unlinks && !temp_present);
	done("exclusive-create collision exhaustion");
	setup();
	fixture_collisions(2);
	assert(!replace());
	complete();
	assert(attempts == 3);
	done("exclusive-create collision retry");
	setup();
	fixture_short(3);
	assert(!replace());
	complete();
	assert(write_calls > 2 && !temp_present && temp_syncs == 1 &&
	       dir_syncs == 1);
	assert(strstr(events, "FRD"));
	done("short writes loop to durable replacement");
	setup();
	fixture_fault(DIR_SYNC, EIO);
	assert(replace() == -EIO);
	complete();
	assert(renames == 1 && !unlinks && commit_warning);
	done("directory fsync failure leaves complete new generation visible");
	setup();
	fixture_fault(DIR_SYNC, EIO);
	assert(replace() == -EIO);
	fixture_fault(DIR_SYNC, 0);
	assert(!replace());
	complete();
	assert(renames == 2 && !temp_present);
	done("retry after ambiguous directory fsync");
	fixture_reset();
	assert(!replace());
	complete();
	done("first creation durable");
	setup();
	assert(sep_store_replace("invalid", header, 16, body, sizeof(body)) ==
	       -EINVAL);
	unchanged();
	done("invalid path rejected before writes");
	printf("C cases: %d passed, 0 failed\n", passed);
	return 0;
}
