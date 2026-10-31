# Capture buffer cleanup regression

Run `python3 tools/aurora-sep/tests/capture-error-wipe/run.py` from a kernel checkout. Set `RUSTC` if the host Rust compiler is not on PATH. This test needs Python 3 and a host Rust compiler; it does not compile a kernel, access a sensor, or need root.

The runner extracts the actual `read_capture`, CRC implementation, `Capture` wrapper, and its destructor from the selected `sensor.rs`. Only the allocator and SPI call are synthetic. Fixture constants are checked against the selected production source. The fake allocator records initialized bytes at deallocation so each error path can establish whether the owner wiped the receive buffer.

Nine cases cover short/oversized input, allocation and resize failures, SPI failures after partial/full writes, invalid CRC, and successful buffer ownership/cleanup. The unpatched production source reproduces three failures. To reproduce that control, save `git show HEAD:drivers/soc/apple/sensor.rs` before applying the patch and pass `--source /path/to/baseline-sensor.rs --expect-failures 3`.

This is a source-level memory-lifetime regression, not physical capture or power-loss evidence. It observes the buffer before the allocator releases it and does not assert properties of the real allocator, SPI controller, or firmware. All simulated capture data is synthetic.
