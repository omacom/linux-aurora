# SPDX-License-Identifier: GPL-2.0
TEST_GEN_PROGS := page-contract proc-args gem-userptr
TEST_GEN_PROGS_EXTENDED := iommufd-init coarse-shmem-init bpf-uptr-init vhost-log-init uprobes-init user-events-init uffd-move-init swap-fixture-init exec-fixture-init mixed-process memory-interfaces advice reclaim-advice proc-pages proc-memory core-fixture-init context-stress-init mlock-fixture-init lazyfree-fixture-init thread-fixture-init splice-fixture-init dio-fixture-init uring-fixture-init gem-fixture-init compat-fixture-init granule-proc-init granule-swap-init aslr-fixture-init cpu-granule-init user16k-smoke-init cpu-granule16-init granule-threeway-init compat-mlock-init mglru-granule-init memcg-pressure-init memcg-accounting-init runtime-app-init fex-contract-init exec-policy-init uffd-wp-init shmem-uffd-swap-init compat-uffd-init shmem-hole-init shmem-thp-init shmem-collapse-init shmem-pin-init shmem-race-init shmem-pressure-init
TEST_GEN_PROGS_EXTENDED += vfio-fixture-init sysv-shm-init perf-ring-init
TEST_GEN_PROGS_EXTENDED += aio-ring-init numa-policy-init numa-balancing-init cow-pool
TEST_GEN_PROGS_EXTENDED += mte-core-init
CFLAGS += -Wall -Wextra -O2 -ffixed-x18
TEST_GEN_FILES += compat-probe-4k compat-probe-64k compat-probe-dynamic compat-interp32 compat-aslr-probe compat-mlock-probe
TEST_GEN_FILES += fex-x86-contract libfex-abi.so compat-policy-probe compat-uffd-probe compat-vfio-probe
TEST_GEN_FILES += compat-shm-probe compat-perf-probe compat-numa-probe compat-numa-balancing-probe
TEST_GEN_PROGS_EXTENDED += anon-hole-refill anon-hole-refill-race anon-hole-refill-reclaim
TEST_FILES += README.hole-refill README.hole-refill-race README.hole-refill-reclaim
TEST_FILES += compat-script runtime-app.py
TEST_GEN_PROGS_EXTENDED += same-quarter file-cow-consumers same-native
TEST_FILES += README.any-quarter check-any-quarter.py
TEST_GEN_PROGS_EXTENDED += first-write clipped-firstwrite
TEST_FILES += README.first-write
TEST_GEN_PROGS_EXTENDED += pinned-fork
TEST_FILES += README.pinned-fork
TEST_GEN_PROGS_EXTENDED += pagetable-retention pagetable-concurrent
TEST_FILES += README.pagetable-backing
TEST_GEN_PROGS_EXTENDED += domain-cycle
TEST_FILES += README.domain-cycle
include ../../lib.mk

$(OUTPUT)/page-contract: page-contract.c x18-probe.S

$(OUTPUT)/domain-cycle $(OUTPUT)/pagetable-concurrent $(OUTPUT)/first-write $(OUTPUT)/thread-fixture-init $(OUTPUT)/user16k-smoke-init $(OUTPUT)/cow-pool $(OUTPUT)/anon-hole-refill-race $(OUTPUT)/anon-hole-refill-reclaim: LDLIBS += -pthread

# Freestanding ELF32 probes exercise the compat ABI without an armhf sysroot.
AARCH32_CC ?= clang --target=arm-linux-gnueabihf
AARCH32_FLAGS := -march=armv8-a -marm -mfloat-abi=soft -ffreestanding \
	-fno-builtin -fno-stack-protector -O2 -nostdlib -static -fuse-ld=lld
$(OUTPUT)/compat-probe-4k: compat-probe.c
	$(AARCH32_CC) $(AARCH32_FLAGS) -Wl,-z,max-page-size=4096 $< -o $@
$(OUTPUT)/compat-probe-64k: compat-probe.c
	$(AARCH32_CC) $(AARCH32_FLAGS) -Wl,-z,max-page-size=65536 $< -o $@
$(OUTPUT)/compat-interp32: compat-probe.c
	$(AARCH32_CC) $(AARCH32_FLAGS) -Wl,-z,max-page-size=65536,--image-base=0x10000000 $< -o $@
$(OUTPUT)/compat-probe-dynamic: compat-probe.c
	$(AARCH32_CC) $(filter-out -static,$(AARCH32_FLAGS)) -no-pie \
		-Wl,-z,max-page-size=65536,--dynamic-linker=/interp32 $< -o $@

$(OUTPUT)/compat-aslr-probe: compat-aslr-probe.c
	$(AARCH32_CC) $(AARCH32_FLAGS) -Wl,-z,max-page-size=65536 $< -o $@

$(OUTPUT)/compat-mlock-probe: compat-mlock-probe.c
	$(AARCH32_CC) $(AARCH32_FLAGS) -Wl,-z,max-page-size=65536 $< -o $@

# The disposable PID1 fixture also runs with the native 64K process ABI.
$(OUTPUT)/mglru-granule-init $(OUTPUT)/memcg-pressure-init $(OUTPUT)/memcg-accounting-init $(OUTPUT)/runtime-app-init $(OUTPUT)/fex-contract-init: LDFLAGS += -static -Wl,-z,max-page-size=65536

# FEX fixture: the AArch64 emulator runs the freestanding x86-64 contract.
X86_64_CC ?= clang --target=x86_64-linux-gnu
$(OUTPUT)/fex-x86-contract: fex-x86-contract.c
	$(X86_64_CC) -O2 -fno-builtin -fno-stack-protector -nostdlib -static \
		-fuse-ld=lld -Wl,-z,max-page-size=4096,-e,_start $< -o $@

$(OUTPUT)/libfex-abi.so: fex-native-abi.c
	$(CC) $(CFLAGS) -shared -fPIC -Wl,-z,max-page-size=65536 $< -o $@

$(OUTPUT)/exec-policy-init: LDFLAGS += -static -Wl,-z,max-page-size=65536
$(OUTPUT)/exec-policy-init: LDLIBS += -pthread
$(OUTPUT)/mte-core-init: LDFLAGS += -static -Wl,-z,max-page-size=65536
$(OUTPUT)/compat-policy-probe: compat-policy-probe.c
	$(AARCH32_CC) $(AARCH32_FLAGS) -Wl,-z,max-page-size=65536 $< -o $@

# Optional Windows payload; requires Wine/FEX in a disposable guest to execute.
# Keep its toolchain out of the ordinary Linux selftest build.
WINDOWS_CC ?= clang --target=x86_64-pc-windows-msvc
WINDOWS_DLLTOOL ?= llvm-dlltool
WINDOWS_LD ?= lld-link
.PHONY: wine-contract
wine-contract: $(OUTPUT)/wine-page-contract.exe
$(OUTPUT)/wine-kernel32.lib: wine-kernel32.def
	$(WINDOWS_DLLTOOL) -m i386:x86-64 -d $< -l $@
$(OUTPUT)/wine-page-contract.obj: wine-page-contract.c
	$(WINDOWS_CC) -O2 -Wall -Wextra -Werror -ffreestanding -fno-stack-protector -c $< -o $@
$(OUTPUT)/wine-page-contract.exe: $(OUTPUT)/wine-page-contract.obj $(OUTPUT)/wine-kernel32.lib
	$(WINDOWS_LD) /entry:mainCRTStartup /subsystem:console /nodefaultlib /out:$@ $^
EXTRA_CLEAN += $(OUTPUT)/wine-page-contract.exe $(OUTPUT)/wine-page-contract.obj $(OUTPUT)/wine-kernel32.lib

$(OUTPUT)/uffd-wp-init: LDFLAGS += -static -Wl,-z,max-page-size=65536
$(OUTPUT)/uffd-wp-init: LDLIBS += -pthread

$(OUTPUT)/shmem-uffd-swap-init: LDFLAGS += -static -Wl,-z,max-page-size=65536

$(OUTPUT)/compat-uffd-init: LDFLAGS += -static -Wl,-z,max-page-size=65536
$(OUTPUT)/compat-uffd-probe: compat-uffd-probe.c
	$(AARCH32_CC) $(AARCH32_FLAGS) -Wl,-z,max-page-size=65536 $< -o $@

$(OUTPUT)/shmem-hole-init: LDFLAGS += -static -Wl,-z,max-page-size=65536
$(OUTPUT)/shmem-hole-init: LDLIBS += -pthread

$(OUTPUT)/shmem-thp-init: LDFLAGS += -static -Wl,-z,max-page-size=65536
$(OUTPUT)/shmem-thp-init: LDLIBS += -pthread

$(OUTPUT)/shmem-collapse-init: LDFLAGS += -static -Wl,-z,max-page-size=65536

$(OUTPUT)/shmem-pin-init: LDFLAGS += -static -Wl,-z,max-page-size=65536
$(OUTPUT)/shmem-pin-init: LDLIBS += -pthread

$(OUTPUT)/shmem-race-init $(OUTPUT)/shmem-pressure-init: LDFLAGS += -static -Wl,-z,max-page-size=65536
$(OUTPUT)/shmem-race-init: LDLIBS += -pthread

$(OUTPUT)/uffd-move-init: LDFLAGS += -static -Wl,-z,max-page-size=65536
$(OUTPUT)/uffd-move-init: LDLIBS += -pthread

$(OUTPUT)/user-events-init: LDFLAGS += -static -Wl,-z,max-page-size=65536

$(OUTPUT)/uprobes-init: LDFLAGS += -static -Wl,-z,max-page-size=65536

$(OUTPUT)/vhost-log-init: LDFLAGS += -static -Wl,-z,max-page-size=65536

$(OUTPUT)/bpf-uptr-init: LDFLAGS += -static -Wl,-z,max-page-size=65536

$(OUTPUT)/coarse-shmem-init: LDFLAGS += -static -Wl,-z,max-page-size=65536

$(OUTPUT)/iommufd-init: LDFLAGS += -static -Wl,-z,max-page-size=65536
$(OUTPUT)/iommufd-init: LDLIBS += -pthread
$(OUTPUT)/iommufd-init: CFLAGS += $(KHDR_INCLUDES)

$(OUTPUT)/gem-userptr: CFLAGS += $(KHDR_INCLUDES)

# Privileged recording-domain fixture; run only in a disposable test VM.
$(OUTPUT)/vfio-fixture-init: LDFLAGS += -static -Wl,-z,max-page-size=65536
$(OUTPUT)/compat-vfio-probe: compat-vfio-probe.c
	$(AARCH32_CC) $(AARCH32_FLAGS) -Wl,-z,max-page-size=65536 $< -o $@

$(OUTPUT)/sysv-shm-init: LDFLAGS += -static -Wl,-z,max-page-size=65536
$(OUTPUT)/compat-shm-probe: compat-shm-probe.c
	$(AARCH32_CC) $(AARCH32_FLAGS) -Wl,-z,max-page-size=65536 $< -o $@

# Disposable guests exercise perf layout and software sampling across mm granules.
$(OUTPUT)/perf-ring-init: LDFLAGS += -static -Wl,-z,max-page-size=65536
$(OUTPUT)/compat-perf-probe: compat-perf-probe.c
	$(AARCH32_CC) $(AARCH32_FLAGS) -Wl,-z,max-page-size=65536 $< -o $@

$(OUTPUT)/aio-ring-init: LDFLAGS += -static -Wl,-z,max-page-size=65536

# Requires a disposable guest with two memory nodes and CONFIG_COMPAT.
$(OUTPUT)/numa-policy-init: LDFLAGS += -static -Wl,-z,max-page-size=65536
$(OUTPUT)/compat-numa-probe: compat-numa-probe.c
	$(AARCH32_CC) $(AARCH32_FLAGS) -Wl,-z,max-page-size=65536 $< -o $@

$(OUTPUT)/numa-balancing-init: LDFLAGS += -static -Wl,-z,max-page-size=65536
$(OUTPUT)/numa-balancing-init: LDLIBS += -pthread

$(OUTPUT)/compat-numa-balancing-probe: compat-numa-balancing-probe.c
	$(AARCH32_CC) $(AARCH32_FLAGS) -Wl,-z,max-page-size=65536 $< -o $@
