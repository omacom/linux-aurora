# SEP host-file publication fault tests

The atomic suite compiles the production C replacement primitive, Rust StoreFile wrapper and catacomb writer against a synthetic VFS. It checks short writes, allocation/open/security/rename/fsync failures, preservation of the old generation before rename, and a complete visible generation after a failed directory fsync. The C boundary tests reject symlinks and non-regular destinations, temporary-name collisions, and unsafe cleanup. The suite also compiles a deliberate regression and requires its named behavioral assertion to fail. No real files, SEP state or hardware are opened.

Run `python3 -I tools/aurora-sep/tests/persistence/persistence-tests.py --suite atomic --output /tmp/sep-atomic`. CI pins Rust 1.85.0. These host fixtures do not establish power-loss durability on physical storage or SEP anti-replay safety.

A crash after temporary-file creation and before rename can leave a `.sep-<hex>.tmp` file in `/var/lib`, up to 1 MiB per interrupted catacomb write. Ordinary error paths attempt to unlink their own temporary file; there is no automatic cleanup after a crash. A later write uses a fresh exclusive temporary name and never adopts the leftover. These files can contain biometric material: inspect and remove confirmed stale temporary files only while SEP persistence is inactive. Successful publication leaves no temporary file.

Only full-file replacement is implemented here. Header-tail copying and exclusive destination creation belong with their later callers and tests; this suite does not claim to exercise those operations.
