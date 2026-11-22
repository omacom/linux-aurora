/* SPDX-License-Identifier: GPL-2.0-only */
#define _GNU_SOURCE
#include <errno.h>
#include <signal.h>
#include <setjmp.h>
#include <sys/socket.h>
#include <fcntl.h>
#include <linux/userfaultfd.h>
#include <poll.h>
#include <pthread.h>
#include <stdatomic.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/auxv.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/mount.h>
#include <sys/prctl.h>
#include <sys/reboot.h>
#include <sys/syscall.h>
#include <sys/wait.h>
#include <unistd.h>
#define SET_EXEC_PAGE_SIZE 0x41555001
#define CHECK(x)                                                             \
	do {                                                                 \
		if (!(x)) {                                                  \
			printf("not ok - uffd line=%d errno=%d\n", __LINE__, \
			       errno);                                       \
			exit(1);                                             \
		}                                                            \
	} while (0)
struct writer {
	volatile unsigned char *address;
	unsigned char value;
	atomic_int done, tid;
};
static void *write_byte(void *opaque)
{
	struct writer *w = opaque;
	atomic_store(&w->tid, syscall(SYS_gettid));
	*w->address = w->value;
	atomic_store(&w->done, 1);
	return NULL;
}
struct forker {
	unsigned char *base;
	size_t page_size;
	pid_t pid;
};
static void *fork_byte(void *opaque)
{
	struct forker *f = opaque;
	f->pid = fork();
	if (!f->pid) {
		alarm(10);
		f->base[f->page_size + 17] = 0xa5;
		_exit(f->base[f->page_size + 17] != 0xa5);
	}
	return NULL;
}
static struct uffd_msg read_event(int fd)
{
	struct pollfd p = { .fd = fd, .events = POLLIN };
	struct uffd_msg msg;
	CHECK(poll(&p, 1, 10000) == 1 && (p.revents & POLLIN));
	CHECK(read(fd, &msg, sizeof(msg)) == sizeof(msg));
	return msg;
}
static void protect(int fd, void *start, size_t len, unsigned long mode)
{
	struct uffdio_writeprotect wp = { .range = { (uintptr_t)start, len },
					  .mode = mode };
	CHECK(!ioctl(fd, UFFDIO_WRITEPROTECT, &wp));
}
static void resolve_write(int fd, unsigned char *base, size_t ps, unsigned page,
			  unsigned char value, int exact)
{
	struct writer w = { .address = base + page * ps + 17, .value = value };
	struct pollfd pollfd = { .fd = fd, .events = POLLIN };
	struct uffd_msg msg;
	pthread_t thread;
	CHECK(!pthread_create(&thread, NULL, write_byte, &w));
	CHECK(poll(&pollfd, 1, 10000) == 1 && (pollfd.revents & POLLIN));
	CHECK(read(fd, &msg, sizeof(msg)) == sizeof(msg));
	CHECK(msg.event == UFFD_EVENT_PAGEFAULT);
	CHECK(msg.arg.pagefault.flags ==
	      (UFFD_PAGEFAULT_FLAG_WP | UFFD_PAGEFAULT_FLAG_WRITE));
	CHECK(msg.arg.pagefault.address ==
	      (uintptr_t)(base + page * ps + (exact ? 17 : 0)));
	CHECK(msg.arg.pagefault.feat.ptid == (unsigned int)atomic_load(&w.tid));
	CHECK(!atomic_load(&w.done));
	protect(fd, base + page * ps, ps, UFFDIO_WRITEPROTECT_MODE_DONTWAKE);
	struct uffdio_range range = { (uintptr_t)(base + page * ps), ps };
	CHECK(!ioctl(fd, UFFDIO_WAKE, &range));
	CHECK(!pthread_join(thread, NULL) && atomic_load(&w.done));
	CHECK(base[page * ps + 17] == value);
	CHECK(poll(&pollfd, 1, 0) == 0);
}
static void fill(int fd, void *dst, void *src, size_t len, unsigned long mode)
{
	struct uffdio_copy copy = { .dst = (uintptr_t)dst,
				    .src = (uintptr_t)src,
				    .len = len,
				    .mode = mode };
	CHECK(!ioctl(fd, UFFDIO_COPY, &copy) && copy.copy == (long long)len);
}
static void zero(int fd, void *dst, size_t len)
{
	struct uffdio_zeropage z = { .range = { (uintptr_t)dst, len } };
	CHECK(!ioctl(fd, UFFDIO_ZEROPAGE, &z) && z.zeropage == (long long)len);
}
static uint64_t physical(void *address, size_t ps)
{
	uint64_t entry;
	int fd = open("/proc/self/pagemap", O_RDONLY);
	CHECK(fd >= 0 &&
	      pread(fd, &entry, 8, (uintptr_t)address / ps * 8) == 8);
	CHECK(!close(fd) && (entry & (1ULL << 63)));
	return (entry & ((1ULL << 55) - 1)) * ps;
}
static void missing(size_t ps, size_t native)
{
	size_t len = 4 * native;
	unsigned char *map = mmap(NULL, len + native, PROT_READ | PROT_WRITE,
				  MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
	unsigned char *src = mmap(NULL, len + native, PROT_READ | PROT_WRITE,
				  MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
	CHECK(map != MAP_FAILED && src != MAP_FAILED);
	unsigned char *p = map + ps;
	CHECK(!mlock2(p, len, MLOCK_ONFAULT));
	for (size_t i = 0; i < len; i++)
		src[i] = (i / ps) ^ (i * 7 + 3);
	int fd = syscall(SYS_userfaultfd,
			 O_CLOEXEC | O_NONBLOCK | UFFD_USER_MODE_ONLY);
	CHECK(fd >= 0);
	struct uffdio_api api = { .api = UFFD_API,
				  .features = UFFD_FEATURE_PAGEFAULT_FLAG_WP };
	CHECK(!ioctl(fd, UFFDIO_API, &api));
	struct uffdio_register reg = { .range = { (uintptr_t)p, len },
				       .mode = UFFDIO_REGISTER_MODE_MISSING |
					       UFFDIO_REGISTER_MODE_WP };
	CHECK(!ioctl(fd, UFFDIO_REGISTER, &reg));
	CHECK((reg.ioctls &
	       ((1ULL << _UFFDIO_COPY) | (1ULL << _UFFDIO_ZEROPAGE))) ==
	      ((1ULL << _UFFDIO_COPY) | (1ULL << _UFFDIO_ZEROPAGE)));
	/* A write blocks only on its missing leaf; COPY_MODE_WP causes a second event. */
	struct writer w = { .address = p + 17, .value = 0xd3 };
	pthread_t thread;
	CHECK(!pthread_create(&thread, NULL, write_byte, &w));
	struct uffd_msg msg = read_event(fd);
	CHECK(msg.event == UFFD_EVENT_PAGEFAULT &&
	      msg.arg.pagefault.address == (uintptr_t)p &&
	      msg.arg.pagefault.flags == UFFD_PAGEFAULT_FLAG_WRITE &&
	      !atomic_load(&w.done));
	fill(fd, p, src, ps, UFFDIO_COPY_MODE_WP);
	msg = read_event(fd);
	CHECK(msg.event == UFFD_EVENT_PAGEFAULT &&
	      msg.arg.pagefault.address == (uintptr_t)p &&
	      msg.arg.pagefault.flags ==
		      (UFFD_PAGEFAULT_FLAG_WRITE | UFFD_PAGEFAULT_FLAG_WP));
	protect(fd, p, ps, 0);
	CHECK(!pthread_join(thread, NULL) && atomic_load(&w.done));
	src[17] = w.value;
	CHECK(!memcmp(p, src, ps));
	zero(fd, p + ps, ps);
	for (size_t i = 0; i < ps; i++)
		CHECK(p[ps + i] == 0);
	p[ps + 17] = 0xe4;
	CHECK(!memcmp(p, src, ps));
	/* A batch starts at a user-page boundary, not necessarily a native one. */
	fill(fd, p + 2 * ps, src + 2 * ps, len - 2 * ps, 0);
	CHECK(!memcmp(p + 2 * ps, src + 2 * ps, len - 2 * ps));
	if (ps < native) {
		unsigned char *a =
			(void *)(((uintptr_t)(p + 2 * ps) + native - 1) &
				 ~(native - 1));
		uint64_t start = physical(a, ps);
		CHECK(start != 0 && !(start & (native - 1)));
		for (size_t i = ps; i < native; i += ps)
			CHECK(physical(a + i, ps) == start + i);
	}
	struct uffdio_copy copy = { .dst = (uintptr_t)p,
				    .src = (uintptr_t)src,
				    .len = ps };
	CHECK(ioctl(fd, UFFDIO_COPY, &copy) == -1 && errno == EEXIST &&
	      copy.copy == -EEXIST);
	CHECK(!munlock(p, len));
	/* Preserve the successfully copied prefix when the next PTE already exists. */
	CHECK(!madvise(p, ps, MADV_DONTNEED));
	copy = (struct uffdio_copy){ .dst = (uintptr_t)p,
				     .src = (uintptr_t)src,
				     .len = 2 * ps };
	CHECK(ioctl(fd, UFFDIO_COPY, &copy) == -1 && errno == EAGAIN &&
	      copy.copy == (long long)ps);
	CHECK(!memcmp(p, src, ps) && p[ps + 17] == 0xe4);
	/* An invalid source stops at its leaf, without installing it or touching neighbours. */
	CHECK(!madvise(p + 2 * ps, 2 * ps, MADV_DONTNEED));
	CHECK(!mprotect(src + 3 * ps, ps, PROT_NONE));
	copy = (struct uffdio_copy){ .dst = (uintptr_t)(p + 2 * ps),
				     .src = (uintptr_t)(src + 2 * ps),
				     .len = 2 * ps };
	CHECK(ioctl(fd, UFFDIO_COPY, &copy) == -1 && errno == EAGAIN &&
	      copy.copy == (long long)ps);
	CHECK(!memcmp(p + 2 * ps, src + 2 * ps, ps));
	CHECK(!mprotect(src + 3 * ps, ps, PROT_READ | PROT_WRITE));
	/* A fresh source forces the unlocked copy retry. */
	CHECK(!madvise(src + 3 * ps, ps, MADV_DONTNEED));
	fill(fd, p + 3 * ps, src + 3 * ps, ps, 0);
	for (size_t i = 0; i < ps; i++)
		CHECK(p[3 * ps + i] == 0);
	CHECK(!memcmp(p + 4 * ps, src + 4 * ps, len - 4 * ps));
	pid_t child = fork();
	CHECK(child >= 0);
	if (!child) {
		p[2 * ps + 9] ^= 0xff;
		_exit(p[2 * ps + 9] == src[2 * ps + 9]);
	}
	int status;
	CHECK(waitpid(child, &status, 0) == child && WIFEXITED(status) &&
	      !WEXITSTATUS(status));
	CHECK(!memcmp(p + 2 * ps, src + 2 * ps, ps));
	CHECK(!close(fd));
	CHECK(!munmap(map, len + native) && !munmap(src, len + native));
	printf("ok - uffd ABI %zuK missing copy/WP, zero, packed batch, partial EEXIST/EFAULT, source retry and COW\n",
	       ps / 1024);
}

struct reader {
	volatile unsigned char *address;
	unsigned char value;
	atomic_int done;
};
static void *read_byte(void *opaque)
{
	struct reader *r = opaque;
	r->value = *r->address;
	atomic_store(&r->done, 1);
	return NULL;
}
static void resume_minor(int fd, void *start, size_t len, unsigned long mode)
{
	struct uffdio_continue c = { .range = { (uintptr_t)start, len },
				     .mode = mode };
	CHECK(!ioctl(fd, UFFDIO_CONTINUE, &c) && c.mapped == (long long)len);
}
static void file_minor(size_t ps, size_t native)
{
	for (int shared = 0; shared < 2; shared++) {
		int memfd = memfd_create("uffd-minor-granule", 0);
		CHECK(memfd >= 0 && !ftruncate(memfd, 4 * native));
		unsigned char *alias = mmap(NULL, 4 * native,
					    PROT_READ | PROT_WRITE, MAP_SHARED,
					    memfd, 0);
		unsigned char *p = mmap(NULL, 3 * ps, PROT_READ | PROT_WRITE,
					shared ? MAP_SHARED : MAP_PRIVATE,
					memfd, ps);
		CHECK(alias != MAP_FAILED && p != MAP_FAILED);
		CHECK(!mlock2(p, 3 * ps, MLOCK_ONFAULT));
		for (size_t i = 0; i < 4 * native; i++)
			alias[i] = (i / ps) ^ (i * 3 + 7);
		int fd = syscall(SYS_userfaultfd,
				 O_CLOEXEC | O_NONBLOCK | UFFD_USER_MODE_ONLY);
		CHECK(fd >= 0);
		struct uffdio_api api = {
			.api = UFFD_API,
			.features = UFFD_FEATURE_MINOR_SHMEM |
				    UFFD_FEATURE_PAGEFAULT_FLAG_WP
		};
		CHECK(!ioctl(fd, UFFDIO_API, &api));
		struct uffdio_register reg = {
			.range = { (uintptr_t)p, 3 * ps },
			.mode = UFFDIO_REGISTER_MODE_MINOR |
				UFFDIO_REGISTER_MODE_WP
		};
		CHECK(!ioctl(fd, UFFDIO_REGISTER, &reg));
		CHECK(reg.ioctls & (1ULL << _UFFDIO_CONTINUE));

		for (unsigned i = 0; i < 3; i++) {
			struct reader r = { .address = p + i * ps + 17 };
			pthread_t thread;
			CHECK(!pthread_create(&thread, NULL, read_byte, &r));
			struct uffd_msg msg = read_event(fd);
			CHECK(msg.event == UFFD_EVENT_PAGEFAULT &&
			      msg.arg.pagefault.address ==
				      (uintptr_t)(p + i * ps) &&
			      msg.arg.pagefault.flags ==
				      UFFD_PAGEFAULT_FLAG_MINOR &&
			      !atomic_load(&r.done));
			alias[(i + 1) * ps + 17] ^= 0x3c;
			resume_minor(fd, p + i * ps, ps,
				     UFFDIO_CONTINUE_MODE_WP |
					     UFFDIO_CONTINUE_MODE_DONTWAKE);
			struct uffdio_range range = { (uintptr_t)(p + i * ps),
						      ps };
			CHECK(!ioctl(fd, UFFDIO_WAKE, &range));
			CHECK(!pthread_join(thread, NULL) &&
			      atomic_load(&r.done));
			CHECK(r.value == alias[(i + 1) * ps + 17]);
			CHECK(physical(p + i * ps, ps) ==
			      physical(alias + (i + 1) * ps, ps));
			/* Independent write protection survives installing the file leaf. */
			struct writer w = { .address = p + i * ps + 17,
					    .value = 0xa1 + i };
			CHECK(!pthread_create(&thread, NULL, write_byte, &w));
			msg = read_event(fd);
			CHECK(msg.event == UFFD_EVENT_PAGEFAULT &&
			      msg.arg.pagefault.address ==
				      (uintptr_t)(p + i * ps) &&
			      msg.arg.pagefault.flags ==
				      (UFFD_PAGEFAULT_FLAG_WP |
				       UFFD_PAGEFAULT_FLAG_WRITE));
			protect(fd, p + i * ps, ps, 0);
			CHECK(!pthread_join(thread, NULL) &&
			      atomic_load(&w.done));
			CHECK(p[i * ps + 17] == w.value);
			CHECK(shared ? alias[(i + 1) * ps + 17] == w.value :
				       alias[(i + 1) * ps + 17] == r.value);
		}
		struct pollfd pollfd = { .fd = fd, .events = POLLIN };
		CHECK(poll(&pollfd, 1, 0) == 0);
		CHECK(!munlock(p, 3 * ps));
		CHECK(!madvise(p, 3 * ps, MADV_DONTNEED));
		resume_minor(fd, p, 3 * ps, 0);
		CHECK(!memcmp(p, alias + ps, 3 * ps));
		CHECK(!madvise(p, 3 * ps, MADV_DONTNEED));
		CHECK(!ftruncate(memfd, ps + 17));
		resume_minor(fd, p, ps, 0);
		struct uffdio_continue c = { .range = { (uintptr_t)(p + ps),
							ps } };
		CHECK(ioctl(fd, UFFDIO_CONTINUE, &c) == -1 && errno == EFAULT &&
		      c.mapped == -EFAULT);
		CHECK(!close(fd) && !munmap(p, 3 * ps) &&
		      !munmap(alias, 4 * native) && !close(memfd));
		printf("ok - uffd ABI %zuK shmem %s minor/WP, byte offsets, alias coherence, COW, batch continue and EOF\n",
		       ps / 1024, shared ? "shared" : "private");
	}
}

static uint64_t pagemap(void *address, size_t ps)
{
	uint64_t entry;
	int fd = open("/proc/self/pagemap", O_RDONLY);
	CHECK(fd >= 0 &&
	      pread(fd, &entry, 8, (uintptr_t)address / ps * 8) == 8);
	CHECK(!close(fd));
	return entry;
}
static sigjmp_buf bus_env;
static void *bus_address;
static void bus_handler(int sig, siginfo_t *info, void *context)
{
	(void)context;
	if (sig != SIGBUS)
		_exit(91);
	bus_address = info->si_addr;
	siglongjmp(bus_env, 1);
}
static void markers(size_t ps)
{
	for (int async = 0; async < 2; async++) {
		unsigned char *p = mmap(NULL, 3 * ps, PROT_READ | PROT_WRITE,
					MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
		CHECK(p != MAP_FAILED);
		int fd = syscall(SYS_userfaultfd,
				 O_CLOEXEC | O_NONBLOCK | UFFD_USER_MODE_ONLY);
		CHECK(fd >= 0);
		struct uffdio_api api = {
			.api = UFFD_API,
			.features = UFFD_FEATURE_PAGEFAULT_FLAG_WP |
				    UFFD_FEATURE_THREAD_ID |
				    UFFD_FEATURE_WP_UNPOPULATED |
				    (async ? UFFD_FEATURE_WP_ASYNC : 0)
		};
		CHECK(!ioctl(fd, UFFDIO_API, &api));
		struct uffdio_register reg = {
			.range = { (uintptr_t)p, 3 * ps },
			.mode = UFFDIO_REGISTER_MODE_WP
		};
		CHECK(!ioctl(fd, UFFDIO_REGISTER, &reg));
		protect(fd, p, 3 * ps, UFFDIO_WRITEPROTECT_MODE_WP);
		for (unsigned i = 0; i < 3; i++)
			CHECK(pagemap(p + i * ps, ps) & (1ULL << 57));
		CHECK(*(volatile unsigned char *)(p + ps + 17) == 0);
		if (async)
			p[ps + 17] = 0x67;
		else
			resolve_write(fd, p, ps, 1, 0x67, 0);
		CHECK(p[ps + 17] == 0x67);
		CHECK(pagemap(p, ps) & (1ULL << 57));
		CHECK(!(pagemap(p + ps, ps) & (1ULL << 57)));
		CHECK(pagemap(p + 2 * ps, ps) & (1ULL << 57));
		struct pollfd pollfd = { .fd = fd, .events = POLLIN };
		CHECK(poll(&pollfd, 1, 0) == 0);
		CHECK(!close(fd));
		p[17] = 0x68;
		p[2 * ps + 17] = 0x69;
		CHECK(!munmap(p, 3 * ps));
		printf("ok - uffd ABI %zuK unpopulated WP async=%d per-leaf pagemap state and close\n",
		       ps / 1024, async);
	}
	unsigned char *p = mmap(NULL, 3 * ps, PROT_READ | PROT_WRITE,
				MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
	CHECK(p != MAP_FAILED);
	int fd = syscall(SYS_userfaultfd,
			 O_CLOEXEC | O_NONBLOCK | UFFD_USER_MODE_ONLY);
	CHECK(fd >= 0);
	struct uffdio_api api = { .api = UFFD_API,
				  .features = UFFD_FEATURE_POISON };
	CHECK(!ioctl(fd, UFFDIO_API, &api));
	struct uffdio_register reg = { .range = { (uintptr_t)p, 3 * ps },
				       .mode = UFFDIO_REGISTER_MODE_MISSING };
	CHECK(!ioctl(fd, UFFDIO_REGISTER, &reg) &&
	      (reg.ioctls & (1ULL << _UFFDIO_POISON)));
	zero(fd, p, ps);
	zero(fd, p + 2 * ps, ps);
	struct uffdio_poison poison = { .range = { (uintptr_t)(p + ps), ps } };
	CHECK(!ioctl(fd, UFFDIO_POISON, &poison) &&
	      poison.updated == (long long)ps);
	CHECK(ioctl(fd, UFFDIO_POISON, &poison) == -1 && errno == EEXIST &&
	      poison.updated == -EEXIST);
	struct sigaction sa = { .sa_sigaction = bus_handler,
				.sa_flags = SA_SIGINFO },
			 old;
	CHECK(!sigemptyset(&sa.sa_mask) && !sigaction(SIGBUS, &sa, &old));
	if (!sigsetjmp(bus_env, 1)) {
		(void)*(volatile unsigned char *)(p + ps + 17);
		CHECK(0);
	}
	CHECK(bus_address == p + ps + 17);
	CHECK(!sigaction(SIGBUS, &old, NULL));
	p[17] = 0x81;
	p[2 * ps + 17] = 0x82;
	CHECK(!madvise(p + ps, ps, MADV_FREE));
	CHECK(!close(fd));
	CHECK(p[ps + 17] == 0);
	CHECK(p[17] == 0x81 && p[2 * ps + 17] == 0x82);
	CHECK(!munmap(p, 3 * ps));
	printf("ok - uffd ABI %zuK poison: SIGBUS, MADV_FREE recovery, neighbours\n",
	       ps / 1024);
}
static void marker_window(size_t ps, size_t native)
{
	unsigned char *map = mmap(NULL, 2 * native, PROT_READ | PROT_WRITE,
				  MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
	unsigned char *a =
		(void *)(((uintptr_t)map + native - 1) & ~(native - 1));
	int fd = syscall(SYS_userfaultfd,
			 O_CLOEXEC | O_NONBLOCK | UFFD_USER_MODE_ONLY);
	struct uffdio_api api = { .api = UFFD_API,
				  .features = UFFD_FEATURE_WP_ASYNC |
					      UFFD_FEATURE_WP_UNPOPULATED };
	struct uffdio_register reg = { .range = { (uintptr_t)a, native },
				       .mode = UFFDIO_REGISTER_MODE_WP };
	uint64_t start;

	CHECK(map != MAP_FAILED && fd >= 0);
	CHECK(!ioctl(fd, UFFDIO_API, &api) && !ioctl(fd, UFFDIO_REGISTER, &reg));
	protect(fd, a, native, UFFDIO_WRITEPROTECT_MODE_WP);
	/* The first touch of the window is a write, so markers are its only neighbours. */
	a[ps + 17] = 0x5a;
	start = physical(a, ps);
	CHECK(start && !(start & (native - 1)));
	for (size_t i = 0; i < native; i += ps) {
		CHECK(physical(a + i, ps) == start + i);
		CHECK(((pagemap(a + i, ps) >> 57) & 1) == (i != ps));
	}
	/* An async WP fault on a populated neighbour only clears its own bit. */
	a[0] = 0xa5;
	for (size_t i = 0; i < native; i += ps) {
		CHECK(physical(a + i, ps) == start + i);
		CHECK(((pagemap(a + i, ps) >> 57) & 1) == (i > ps));
	}
	CHECK(a[0] == 0xa5 && a[ps + 17] == 0x5a && !close(fd) &&
	      !munmap(map, 2 * native));
	printf("ok - uffd ABI %zuK unpopulated WP first write: one native page, neighbours WP\n",
	       ps / 1024);
}
struct pager_request {
	uintptr_t address;
	size_t size;
};
static void send_pager(int sock, int fd, struct pager_request *request)
{
	union {
		struct cmsghdr align;
		char bytes[CMSG_SPACE(sizeof(int))];
	} control = { 0 };
	struct iovec iov = { .iov_base = request, .iov_len = sizeof(*request) };
	struct msghdr msg = { .msg_iov = &iov,
			      .msg_iovlen = 1,
			      .msg_control = control.bytes,
			      .msg_controllen = sizeof(control) };
	struct cmsghdr *c = CMSG_FIRSTHDR(&msg);
	c->cmsg_level = SOL_SOCKET;
	c->cmsg_type = SCM_RIGHTS;
	c->cmsg_len = CMSG_LEN(sizeof(int));
	memcpy(CMSG_DATA(c), &fd, sizeof(fd));
	CHECK(sendmsg(sock, &msg, 0) == sizeof(*request));
}
static unsigned char pager_pattern(size_t i)
{
	return (i / 4096) ^ (i * 11 + 5);
}
static int pager_worker(int sock, size_t expected)
{
	CHECK(getauxval(AT_PAGESZ) == expected);
	alarm(30);
	struct pager_request request;
	union {
		struct cmsghdr align;
		char bytes[CMSG_SPACE(sizeof(int))];
	} control = { 0 };
	struct iovec iov = { .iov_base = &request, .iov_len = sizeof(request) };
	struct msghdr msg = { .msg_iov = &iov,
			      .msg_iovlen = 1,
			      .msg_control = control.bytes,
			      .msg_controllen = sizeof(control) };
	CHECK(recvmsg(sock, &msg, 0) == sizeof(request));
	struct cmsghdr *c = CMSG_FIRSTHDR(&msg);
	CHECK(c && c->cmsg_level == SOL_SOCKET && c->cmsg_type == SCM_RIGHTS &&
	      c->cmsg_len == CMSG_LEN(sizeof(int)));
	int fd;
	memcpy(&fd, CMSG_DATA(c), sizeof(fd));
	size_t ps = request.size;
	unsigned char *source = mmap(NULL, 4 * ps, PROT_READ | PROT_WRITE,
				     MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
	CHECK(source != MAP_FAILED);
	for (size_t i = 0; i < 4 * ps; i++)
		source[i] = pager_pattern(i);
	for (unsigned i = 0; i < 3; i++) {
		struct uffd_msg fault = read_event(fd);
		CHECK(fault.event == UFFD_EVENT_PAGEFAULT &&
		      fault.arg.pagefault.address == request.address + i * ps &&
		      !fault.arg.pagefault.flags);
		if (i == 1)
			zero(fd, (void *)(request.address + ps), ps);
		else
			fill(fd, (void *)(request.address + i * ps),
			     source + i * ps, (i == 2 ? 2 : 1) * ps, 0);
	}
	CHECK(!close(fd) && !close(sock) && !munmap(source, 4 * ps));
	return 0;
}
static void cross_pager(size_t ps, size_t native, int shared)
{
	size_t pager_ps = ps == native ? 4096 : native;
	int socks[2];
	CHECK(!socketpair(AF_UNIX, SOCK_SEQPACKET, 0, socks));
	pid_t child = fork();
	CHECK(child >= 0);
	if (!child) {
		char fd_arg[32], ps_arg[32];
		CHECK(!close(socks[0]));
		snprintf(fd_arg, sizeof(fd_arg), "%d", socks[1]);
		snprintf(ps_arg, sizeof(ps_arg), "%zu", pager_ps);
		CHECK(!prctl(SET_EXEC_PAGE_SIZE, pager_ps, 0UL, 0UL, 0UL));
		execl("/init", "/init", "--pager", fd_arg, ps_arg, NULL);
		_exit(3);
	}
	CHECK(!close(socks[1]));
	int memfd = shared ? memfd_create("remote-uffd-shmem", 0) : -1;
	if (shared)
		CHECK(memfd >= 0 && !ftruncate(memfd, 4 * ps));
	unsigned char *p = mmap(
		NULL, 4 * ps, PROT_READ | PROT_WRITE,
		shared ? MAP_SHARED : MAP_PRIVATE | MAP_ANONYMOUS, memfd, 0);
	CHECK(p != MAP_FAILED);
	int fd = syscall(SYS_userfaultfd,
			 O_CLOEXEC | O_NONBLOCK | UFFD_USER_MODE_ONLY);
	CHECK(fd >= 0);
	struct uffdio_api api = { .api = UFFD_API };
	CHECK(!ioctl(fd, UFFDIO_API, &api));
	struct uffdio_register reg = { .range = { (uintptr_t)p, 4 * ps },
				       .mode = UFFDIO_REGISTER_MODE_MISSING };
	CHECK(!ioctl(fd, UFFDIO_REGISTER, &reg));
	struct pager_request request = { .address = (uintptr_t)p, .size = ps };
	send_pager(socks[0], fd, &request);
	for (size_t i = 0; i < 4 * ps; i++)
		CHECK(p[i] == (i >= ps && i < 2 * ps ? 0 : pager_pattern(i)));
	int status;
	CHECK(waitpid(child, &status, 0) == child && WIFEXITED(status) &&
	      !WEXITSTATUS(status));
	CHECK(!close(fd) && !close(socks[0]) && !munmap(p, 4 * ps));
	if (shared)
		CHECK(!close(memfd));
	printf("ok - uffd target %zuK remote pager %zuK shared=%d SCM_RIGHTS copy/zero/batch uses target granule\n",
	       ps / 1024, pager_ps / 1024, shared);
}

static void private_missing(size_t ps)
{
	int memfd = memfd_create("uffd-private-hole", 0);
	CHECK(memfd >= 0 && !ftruncate(memfd, 4 * ps));
	unsigned char *p = mmap(NULL, 3 * ps, PROT_READ | PROT_WRITE,
				MAP_PRIVATE, memfd, ps);
	unsigned char *src = mmap(NULL, 3 * ps, PROT_READ | PROT_WRITE,
				  MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
	CHECK(p != MAP_FAILED && src != MAP_FAILED);
	for (size_t i = 0; i < 3 * ps; i++)
		src[i] = pager_pattern(i);
	int fd = syscall(SYS_userfaultfd,
			 O_CLOEXEC | O_NONBLOCK | UFFD_USER_MODE_ONLY);
	CHECK(fd >= 0);
	struct uffdio_api api = { .api = UFFD_API };
	CHECK(!ioctl(fd, UFFDIO_API, &api));
	struct uffdio_register reg = { .range = { (uintptr_t)p, 3 * ps },
				       .mode = UFFDIO_REGISTER_MODE_MISSING };
	CHECK(!ioctl(fd, UFFDIO_REGISTER, &reg) &&
	      (reg.ioctls & (1ULL << _UFFDIO_COPY)));
	struct reader r = { .address = p + 17 };
	pthread_t thread;
	CHECK(!pthread_create(&thread, NULL, read_byte, &r));
	struct uffd_msg msg = read_event(fd);
	CHECK(msg.event == UFFD_EVENT_PAGEFAULT &&
	      msg.arg.pagefault.address == (uintptr_t)p &&
	      !msg.arg.pagefault.flags);
	fill(fd, p, src, 2 * ps, 0);
	CHECK(!pthread_join(thread, NULL) && r.value == src[17]);
	zero(fd, p + 2 * ps, ps);
	CHECK(!memcmp(p, src, 2 * ps));
	unsigned char bytes[256];
	for (size_t off = 0; off < 4 * ps; off += sizeof(bytes)) {
		CHECK(pread(memfd, bytes, sizeof(bytes), off) == sizeof(bytes));
		for (unsigned i = 0; i < sizeof(bytes); i++)
			CHECK(bytes[i] == 0);
	}
	CHECK(!madvise(p, ps, MADV_DONTNEED));
	CHECK(!madvise(src, 3 * ps, MADV_DONTNEED));
	fill(fd, p, src, ps, 0);
	for (size_t i = 0; i < ps; i++)
		CHECK(p[i] == 0);
	CHECK(!madvise(p, 3 * ps, MADV_DONTNEED));
	CHECK(!ftruncate(memfd, ps + 17));
	struct uffdio_copy copy = { .dst = (uintptr_t)(p + ps),
				    .src = (uintptr_t)src,
				    .len = ps };
	CHECK(ioctl(fd, UFFDIO_COPY, &copy) == -1 && errno == EFAULT &&
	      copy.copy == -EFAULT);
	fill(fd, p, src, ps, 0);
	CHECK(!close(fd) && !close(memfd) && !munmap(p, 3 * ps) &&
	      !munmap(src, 3 * ps));
	printf("ok - uffd ABI %zuK private shmem missing COPY/ZERO, unchanged file bytes, source retry and EOF\n",
	       ps / 1024);
}

static int register_shared(void *p, size_t len)
{
	int fd = syscall(SYS_userfaultfd,
			 O_CLOEXEC | O_NONBLOCK | UFFD_USER_MODE_ONLY);
	CHECK(fd >= 0);
	struct uffdio_api api = { .api = UFFD_API,
				  .features = UFFD_FEATURE_MISSING_SHMEM |
					      UFFD_FEATURE_MINOR_SHMEM |
					      UFFD_FEATURE_PAGEFAULT_FLAG_WP };
	CHECK(!ioctl(fd, UFFDIO_API, &api));
	struct uffdio_register reg = { .range = { (uintptr_t)p, len },
				       .mode = UFFDIO_REGISTER_MODE_MISSING |
					       UFFDIO_REGISTER_MODE_MINOR |
					       UFFDIO_REGISTER_MODE_WP };
	CHECK(!ioctl(fd, UFFDIO_REGISTER, &reg));
	CHECK((reg.ioctls &
	       ((1ULL << _UFFDIO_COPY) | (1ULL << _UFFDIO_ZEROPAGE) |
		(1ULL << _UFFDIO_CONTINUE))) ==
	      ((1ULL << _UFFDIO_COPY) | (1ULL << _UFFDIO_ZEROPAGE) |
	       (1ULL << _UFFDIO_CONTINUE)));
	return fd;
}
static void shared_missing(size_t ps, size_t native)
{
	int memfd = memfd_create("uffd-shared-holes", 0);
	CHECK(memfd >= 0 && !ftruncate(memfd, 8 * native));
	unsigned char *p = mmap(NULL, 4 * ps, PROT_READ | PROT_WRITE,
				MAP_SHARED, memfd, ps);
	unsigned char *q = mmap(NULL, 4 * ps, PROT_READ | PROT_WRITE,
				MAP_SHARED, memfd, ps);
	unsigned char *src = mmap(NULL, 4 * ps, PROT_READ | PROT_WRITE,
				  MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
	CHECK(p != MAP_FAILED && q != MAP_FAILED && src != MAP_FAILED);
	for (size_t i = 0; i < 4 * ps; i++)
		src[i] = pager_pattern(i);
	int fd = register_shared(p, 4 * ps), fd2 = register_shared(q, 4 * ps);
	for (unsigned i = 0; i < 2; i++) {
		struct reader r = { .address = p + i * ps + 17 };
		pthread_t thread;
		CHECK(!pthread_create(&thread, NULL, read_byte, &r));
		struct uffd_msg msg = read_event(fd);
		CHECK(msg.event == UFFD_EVENT_PAGEFAULT &&
		      msg.arg.pagefault.address == (uintptr_t)(p + i * ps) &&
		      !msg.arg.pagefault.flags);
		if (!i)
			fill(fd, p, src, ps, UFFDIO_COPY_MODE_WP);
		else
			zero(fd, p + ps, ps);
		CHECK(!pthread_join(thread, NULL) && atomic_load(&r.done));
		CHECK(r.value == (i ? 0 : src[17]));
		if (!i) {
			struct uffdio_continue c = {
				.range = { (uintptr_t)(p + ps), ps }
			};
			CHECK(ioctl(fd, UFFDIO_CONTINUE, &c) == -1 &&
			      errno == EFAULT && c.mapped == -EFAULT);
		}
	}
	fill(fd, p + 2 * ps, src + 2 * ps, 2 * ps, 0);
	CHECK(!memcmp(p, src, ps) && !memcmp(p + 2 * ps, src + 2 * ps, 2 * ps));
	for (size_t i = 0; i < ps; i++)
		CHECK(p[ps + i] == 0);
	for (unsigned i = 0; i < 4; i++) {
		struct reader r = { .address = q + i * ps + 17 };
		pthread_t thread;
		CHECK(!pthread_create(&thread, NULL, read_byte, &r));
		struct uffd_msg msg = read_event(fd2);
		CHECK(msg.event == UFFD_EVENT_PAGEFAULT &&
		      msg.arg.pagefault.address == (uintptr_t)(q + i * ps) &&
		      msg.arg.pagefault.flags == UFFD_PAGEFAULT_FLAG_MINOR);
		resume_minor(fd2, q + i * ps, ps, 0);
		CHECK(!pthread_join(thread, NULL) && r.value == p[i * ps + 17]);
		CHECK(physical(p + i * ps, ps) == physical(q + i * ps, ps));
	}
	CHECK(!memcmp(p, q, 4 * ps));
	if (ps < native)
		CHECK(physical(p + ps, ps) == physical(p, ps) + ps);
	struct writer w = { .address = p + 17, .value = 0x9a };
	pthread_t thread;
	CHECK(!pthread_create(&thread, NULL, write_byte, &w));
	struct uffd_msg msg = read_event(fd);
	CHECK(msg.event == UFFD_EVENT_PAGEFAULT &&
	      msg.arg.pagefault.address == (uintptr_t)p &&
	      msg.arg.pagefault.flags ==
		      (UFFD_PAGEFAULT_FLAG_WRITE | UFFD_PAGEFAULT_FLAG_WP));
	protect(fd, p, ps, 0);
	CHECK(!pthread_join(thread, NULL) && p[17] == w.value &&
	      q[17] == w.value);
	struct uffdio_copy c = { .dst = (uintptr_t)(p + ps),
				 .src = (uintptr_t)src,
				 .len = ps };
	CHECK(ioctl(fd, UFFDIO_COPY, &c) == -1 && errno == EEXIST &&
	      c.copy == -EEXIST);
	unsigned char bytes[256];
	CHECK(pread(memfd, bytes, sizeof(bytes), ps) == sizeof(bytes));
	CHECK(!memcmp(bytes, p, sizeof(bytes)));
	/* Resolve an untouched source outside VMA locks before taking a cache folio. */
	unsigned char *r = mmap(NULL, 2 * ps, PROT_READ | PROT_WRITE,
				MAP_SHARED, memfd, 6 * native);
	unsigned char *fresh = mmap(NULL, ps, PROT_READ | PROT_WRITE,
				    MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
	CHECK(r != MAP_FAILED && fresh != MAP_FAILED);
	int fd3 = register_shared(r, 2 * ps);
	fill(fd3, r, fresh, ps, 0);
	for (size_t i = 0; i < ps; i++)
		CHECK(r[i] == 0);
	CHECK(!mprotect(fresh, ps, PROT_NONE));
	c = (struct uffdio_copy){ .dst = (uintptr_t)(r + ps),
				  .src = (uintptr_t)fresh,
				  .len = ps };
	CHECK(ioctl(fd3, UFFDIO_COPY, &c) == -1 && errno == EFAULT &&
	      c.copy == -EFAULT);
	zero(fd3, r + ps, ps);
	for (size_t i = 0; i < ps; i++)
		CHECK(r[ps + i] == 0);
	CHECK(!close(fd3) && !munmap(r, 2 * ps) && !munmap(fresh, ps));
	CHECK(!ftruncate(memfd, 2 * ps + 17));
	c = (struct uffdio_copy){ .dst = (uintptr_t)(p + 2 * ps),
				  .src = (uintptr_t)src,
				  .len = ps };
	CHECK(ioctl(fd, UFFDIO_COPY, &c) == -1 &&
	      errno == (ps == native ? ENOMEM : EFAULT) &&
	      c.copy == -(ps == native ? ENOMEM : EFAULT));
	CHECK(!close(fd) && !close(fd2) && !close(memfd));
	CHECK(!munmap(p, 4 * ps) && !munmap(q, 4 * ps) && !munmap(src, 4 * ps));
	printf("ok - uffd ABI %zuK shared COPY/ZERO, independent missing/minor pagers, CONTINUE holes, packing, WP/coherence and EOF\n",
	       ps / 1024);
}

static void shared_buffered_write(size_t ps)
{
	int memfd = memfd_create("uffd-buffered-validity", 0);
	CHECK(memfd >= 0 && !ftruncate(memfd, 4 * ps));
	unsigned char *p = mmap(NULL, 4 * ps, PROT_READ | PROT_WRITE,
				MAP_SHARED, memfd, 0);
	unsigned char *src = mmap(NULL, ps, PROT_READ | PROT_WRITE,
				  MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
	CHECK(p != MAP_FAILED && src != MAP_FAILED);
	memset(src, 0x57, ps);
	int fd = register_shared(p, 4 * ps);
	fill(fd, p, src, ps, 0);
	CHECK(pwrite(memfd, src, 23, ps + 17) == 23);
	for (unsigned i = 1; i < 3; i++) {
		struct reader r = { .address = p + i * ps + 17 };
		pthread_t thread;
		CHECK(!pthread_create(&thread, NULL, read_byte, &r));
		struct uffd_msg msg = read_event(fd);
		CHECK(msg.event == UFFD_EVENT_PAGEFAULT &&
		      msg.arg.pagefault.address == (uintptr_t)(p + i * ps) &&
		      msg.arg.pagefault.flags ==
			      (i == 1 ? UFFD_PAGEFAULT_FLAG_MINOR : 0));
		if (i == 1)
			resume_minor(fd, p + ps, ps, 0);
		else
			zero(fd, p + i * ps, ps);
		CHECK(!pthread_join(thread, NULL) &&
		      r.value == (i == 1 ? 0x57 : 0));
	}
	for (size_t i = 0; i < ps; i++)
		CHECK(p[ps + i] == (i >= 17 && i < 40 ? 0x57 : 0));
	CHECK(!memcmp(p, src, ps));
	CHECK(!close(fd) && !close(memfd) && !munmap(p, 4 * ps) &&
	      !munmap(src, ps));
	printf("ok - uffd ABI %zuK buffered short-range write populates its leaf and leaves neighbours missing\n",
	       ps / 1024);
}

static int worker(unsigned long expected, unsigned long native)
{
	size_t ps = getauxval(AT_PAGESZ);
	CHECK(ps == expected);
	alarm(60);
	missing(ps, native);
	file_minor(ps, native);
	markers(ps);
	if (ps < native)
		marker_window(ps, native);
	cross_pager(ps, native, 0);
	cross_pager(ps, native, 1);
	private_missing(ps);
	shared_missing(ps, native);
	shared_buffered_write(ps);
	for (int exact = 0; exact < 2; exact++) {
		unsigned char *p = mmap(NULL, 3 * ps, PROT_READ | PROT_WRITE,
					MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
		CHECK(p != MAP_FAILED);
		for (unsigned i = 0; i < 3; i++)
			memset(p + i * ps, 0x40 + i, ps);
		int fd = syscall(SYS_userfaultfd,
				 O_CLOEXEC | O_NONBLOCK | UFFD_USER_MODE_ONLY);
		CHECK(fd >= 0);
		struct uffdio_api api = {
			.api = UFFD_API,
			.features = UFFD_FEATURE_PAGEFAULT_FLAG_WP |
				    UFFD_FEATURE_THREAD_ID |
				    (exact ? UFFD_FEATURE_EXACT_ADDRESS |
						     UFFD_FEATURE_EVENT_FORK :
					     0)
		};
		CHECK(!ioctl(fd, UFFDIO_API, &api));
		CHECK(api.features & UFFD_FEATURE_PAGEFAULT_FLAG_WP);
		struct uffdio_register reg = {
			.range = { (uintptr_t)p, 3 * ps },
			.mode = UFFDIO_REGISTER_MODE_WP
		};
		CHECK(!ioctl(fd, UFFDIO_REGISTER, &reg));
		CHECK(reg.ioctls & (1ULL << _UFFDIO_WRITEPROTECT));

		for (unsigned i = 0; i < 3; i++) {
			protect(fd, p + i * ps, ps,
				UFFDIO_WRITEPROTECT_MODE_WP);
			/* Neighbours remain directly writable while this page faults. */
			p[((i + 1) % 3) * ps + 31] ^= 1;
			resolve_write(fd, p, ps, i, 0x70 + i, exact);
		}
		protect(fd, p + ps, ps, UFFDIO_WRITEPROTECT_MODE_WP);
		pid_t child;
		int status;
		if (!exact) {
			child = fork();
			CHECK(child >= 0);
			if (!child) {
				alarm(10);
				p[ps + 17] = 0xa5;
				_exit(p[ps + 17] != 0xa5);
			}
		} else {
			struct forker f = { .base = p, .page_size = ps };
			pthread_t thread;
			CHECK(!pthread_create(&thread, NULL, fork_byte, &f));
			struct uffd_msg event = read_event(fd);
			CHECK(event.event == UFFD_EVENT_FORK);
			int child_fd = event.arg.fork.ufd;
			CHECK(child_fd >= 0);
			CHECK(!pthread_join(thread, NULL));
			child = f.pid;
			CHECK(child > 0);
			event = read_event(child_fd);
			CHECK(event.event == UFFD_EVENT_PAGEFAULT &&
			      event.arg.pagefault.flags ==
				      (UFFD_PAGEFAULT_FLAG_WP |
				       UFFD_PAGEFAULT_FLAG_WRITE));
			CHECK(event.arg.pagefault.address ==
			      (uintptr_t)(p + ps + 17));
			CHECK(event.arg.pagefault.feat.ptid ==
			      (unsigned int)child);
			protect(child_fd, p + ps, ps, 0);
			CHECK(!close(child_fd));
		}
		CHECK(waitpid(child, &status, 0) == child &&
		      WIFEXITED(status) && !WEXITSTATUS(status));
		CHECK(p[ps + 17] == 0x71);
		resolve_write(fd, p, ps, 1, 0x91, exact);
		protect(fd, p, 3 * ps, UFFDIO_WRITEPROTECT_MODE_WP);
		struct uffdio_range middle = { (uintptr_t)(p + ps), ps };
		CHECK(!ioctl(fd, UFFDIO_UNREGISTER, &middle));
		p[ps + 17] = 0xb2;
		resolve_write(fd, p, ps, 0, 0xb1, exact);
		CHECK(!close(fd));
		/* Close removes the remaining registration and write protection. */
		p[2 * ps + 17] = 0xb3;
		CHECK(p[17] == 0xb1 && p[ps + 17] == 0xb2 &&
		      p[2 * ps + 17] == 0xb3);
		CHECK(!munmap(p, 3 * ps));
		printf("ok - uffd ABI %zuK exact=%d events, neighbours, fork delivery/COW, partial unregister and close\n",
		       ps / 1024, exact);
	}
	return 0;
}
int main(int argc, char **argv)
{
	setvbuf(stdout, NULL, _IONBF, 0);
	if (argc == 4 && !strcmp(argv[1], "--pager"))
		return pager_worker(atoi(argv[2]), strtoul(argv[3], NULL, 10));
	if (argc == 4 && !strcmp(argv[1], "--worker"))
		return worker(strtoul(argv[2], NULL, 10),
			      strtoul(argv[3], NULL, 10));
	if (getpid() != 1 || argc != 1)
		return 2;
	CHECK(!mount("proc", "/proc", "proc", 0, NULL));
	unsigned long native = getauxval(AT_PAGESZ),
		      sizes[] = { native, 4096, 16384 };
	unsigned modes = native == 65536 ? 3 : 2;
	int failed = 0;
	for (unsigned i = 0; i < modes; i++) {
		pid_t pid = fork();
		CHECK(pid >= 0);
		if (!pid) {
			char ps[32], ns[32];
			snprintf(ps, sizeof(ps), "%lu", sizes[i]);
			snprintf(ns, sizeof(ns), "%lu", native);
			CHECK(!prctl(SET_EXEC_PAGE_SIZE, sizes[i], 0UL, 0UL,
				     0UL));
			execl("/init", "/init", "--worker", ps, ns, NULL);
			_exit(3);
		}
		int status = 0;
		int ok = waitpid(pid, &status, 0) == pid && WIFEXITED(status) &&
			 !WEXITSTATUS(status);
		printf("%s - uffd ABI %luK status=%#x\n", ok ? "ok" : "not ok",
		       sizes[i] / 1024, status);
		failed |= !ok;
	}
	printf("UFFD CONTRACT %s\n", failed ? "FAIL" : "PASS");
	sync();
	reboot(RB_POWER_OFF);
	for (;;)
		pause();
}
