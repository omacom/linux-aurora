// SPDX-License-Identifier: GPL-2.0-only OR MIT
/*
 * Passive diagnostics for the SEP driver: the "diag" attribute group on the SEP
 * platform device. Unattended test tools use it to see how far the bring-up
 * got without reading the kernel log, which may be restricted.
 *
 * Each attribute shows state that the driver caches here at the point where it
 * makes the decision. Reading one only loads that cached state: it sends
 * nothing to the SEP, takes no driver lock and starts no work, so a reader can
 * neither stall nor disturb the bring-up. Nothing here identifies a user, an
 * identity or a key.
 *
 * A machine has one SEP and the driver attaches to it at most once per boot,
 * so the state is static rather than per device. Every field starts at zero,
 * the state before the driver has decided anything.
 */
#include <linux/atomic.h>
#include <linux/bitmap.h>
#include <linux/bitops.h>
#include <linux/ctype.h>
#include <linux/device.h>
#include <linux/ktime.h>
#include <linux/minmax.h>
#include <linux/string.h>
#include <linux/sysfs.h>

#include "shim.h"

/* The contract version; see Documentation/ABI/testing/sysfs-driver-apple-sep. */
#define SEP_DIAG_ABI			3

#define SEP_DIAG_ATTACH_PENDING		0
#define SEP_DIAG_ATTACH_ATTACHED	1
#define SEP_DIAG_ATTACH_FAILED		2

#define SEP_DIAG_KEYSTORE_UNKNOWN	0
#define SEP_DIAG_KEYSTORE_OPEN		1
#define SEP_DIAG_KEYSTORE_CLOSED	2

/*
 * The sensor's state in the low bits and a count of its binds above them, in
 * one word. A reader racing a re-bind sees the old device's state or the new
 * one's, never a mix, and a bring-up that began before the sensor was unbound
 * or bound again cannot report its result against the device bound since.
 */
#define SEP_DIAG_SENSOR_UNBOUND		0U
#define SEP_DIAG_SENSOR_BOUND		1U
#define SEP_DIAG_SENSOR_ONLINE		2U
#define SEP_DIAG_SENSOR_FAILED		3U
#define SEP_DIAG_SENSOR_STATE		3U
#define SEP_DIAG_SENSOR_BIND		4U

/*
 * Touch ID is activated by the first open of /dev/sep-bio, not by the boot-time
 * bring-up, so publishing the node leaves the outcome undecided until then.
 * UNAVAILABLE and FAILED are final for the boot; STARTED is not, since a later
 * sensor bring-up can still bring the sensor online.
 */
#define SEP_DIAG_TOUCHID_PENDING	0
#define SEP_DIAG_TOUCHID_PUBLISHED	1
#define SEP_DIAG_TOUCHID_UNAVAILABLE	2
#define SEP_DIAG_TOUCHID_STARTED	3
#define SEP_DIAG_TOUCHID_FAILED		4

/* The SEP's endpoint numbers are one byte. */
#define SEP_DIAG_MAX_EP			256

static struct {
	/* Set at probe, before the group is added. */
	char profile[32];
	bool cold;
	bool sepos13;
	bool xart;
	bool trusted_keys;
	bool filevault_api;
	bool xarm_trace;
	bool refkey_v2;
	bool identity_override;
	atomic_t identity_source;
	/* The xART guard's effective level (0: none runs), and its list. */
	unsigned int xart_guard;
	bool xart_guard_list;

	atomic_t attach;
	atomic_t endpoints;
	atomic_t keystore;
	atomic_t keybag;
	atomic_t sensor;
	atomic_t touchid;
	atomic_t sensor_attach;
	atomic_t restore;
	atomic_t xart_refusals;
	atomic_t matching;

	/* Boot-time clock at probe, and at the attach and bring-up decisions. */
	u64 probe_ns;
	atomic64_t attach_ns;
	atomic64_t ready_ns;

	/*
	 * The four-character name each endpoint advertised, by endpoint number.
	 * A name is stored before its bit is set, so a reader that sees the bit
	 * sees the name.
	 */
	u32 ep_name[SEP_DIAG_MAX_EP];
	DECLARE_BITMAP(ep_seen, SEP_DIAG_MAX_EP);

	atomic_t os_identity;
	atomic_t hwrng;
	atomic_t stac;
	atomic_t stac_onlines;
	atomic_t pnon;
	atomic_t sse;
	atomic_t sse_events;
	atomic_t hibe;
	atomic_t skdl;
	atomic_t xarm_failures;
	atomic_t unhandled;
	atomic_t sleeps;
	atomic_t sleeping;
} sep_diag;

/* Written once per probe, before pnon reads SEP_DIAG_PNON_READ. */
static u8 sep_diag_pnon_digest[SEP_DIAG_PNON_DIGEST_LEN];

static unsigned int sep_diag_sensor(void)
{
	return (unsigned int)atomic_read(&sep_diag.sensor) & SEP_DIAG_SENSOR_STATE;
}

static ssize_t abi_show(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	return sysfs_emit(buf, "%d\n", SEP_DIAG_ABI);
}
static DEVICE_ATTR_RO(abi);

static ssize_t profile_show(struct device *dev, struct device_attribute *attr,
			    char *buf)
{
	return sysfs_emit(buf, "%s\n", sep_diag.profile);
}
static DEVICE_ATTR_RO(profile);

static ssize_t boot_show(struct device *dev, struct device_attribute *attr,
			 char *buf)
{
	return sysfs_emit(buf, "%s\n", sep_diag.cold ? "cold" : "warm");
}
static DEVICE_ATTR_RO(boot);

static ssize_t protocol_show(struct device *dev, struct device_attribute *attr,
			     char *buf)
{
	return sysfs_emit(buf, "%s\n", sep_diag.sepos13 ? "sepos13" : "variant5");
}
static DEVICE_ATTR_RO(protocol);

static ssize_t xart_show(struct device *dev, struct device_attribute *attr,
			 char *buf)
{
	return sysfs_emit(buf, "%s\n", READ_ONCE(sep_diag.xart) ? "enabled" : "disabled");
}
static DEVICE_ATTR_RO(xart);

/*
 * Parameter policy is captured at probe; the identity choice is cached later.
 * provision_keybag stays in the line for ABI 3 readers: the parameter is gone
 * and the driver always provisions (an existing keybag is kept).
 */
static ssize_t policy_show(struct device *dev, struct device_attribute *attr,
			   char *buf)
{
	const char *identity;

	switch (atomic_read(&sep_diag.identity_source)) {
	case 1:
		identity = "dt";
		break;
	case 2:
		identity = "override";
		break;
	case 3:
		identity = "missing";
		break;
	default:
		identity = "pending";
		break;
	}
	return sysfs_emit(buf,
		"xart_writes=%u provision_keybag=1 xart_guard=%u "
		"trusted_keys=%u filevault_api=%u xarm_trace=%u "
		"refkey_v2=%u identity_override=%u identity_source=%s\n",
		sep_diag.xart, sep_diag.xart_guard,
		sep_diag.trusted_keys, sep_diag.filevault_api, sep_diag.xarm_trace,
		sep_diag.refkey_v2, sep_diag.identity_override, identity);
}
static DEVICE_ATTR_RO(policy);

void sep_diag_set_policy(bool trusted_keys, bool filevault_api,
			 bool xarm_trace, bool refkey_v2, bool identity_override)
{
	sep_diag.trusted_keys = trusted_keys;
	sep_diag.filevault_api = filevault_api;
	sep_diag.xarm_trace = xarm_trace;
	sep_diag.refkey_v2 = refkey_v2;
	sep_diag.identity_override = identity_override;
	atomic_set(&sep_diag.identity_source, 0);
}

void sep_diag_set_identity_source(unsigned int source)
{
	atomic_set(&sep_diag.identity_source, source);
}

static ssize_t attach_show(struct device *dev, struct device_attribute *attr,
			   char *buf)
{
	const char *state;

	switch (atomic_read(&sep_diag.attach)) {
	case SEP_DIAG_ATTACH_ATTACHED:
		state = "attached";
		break;
	case SEP_DIAG_ATTACH_FAILED:
		state = "failed";
		break;
	default:
		state = "pending";
		break;
	}
	return sysfs_emit(buf, "%s\n", state);
}
static DEVICE_ATTR_RO(attach);

/* Discovery goes on after the attach, so the count can still grow. */
static ssize_t endpoints_show(struct device *dev, struct device_attribute *attr,
			      char *buf)
{
	unsigned int n = 0;

	if (atomic_read(&sep_diag.attach) == SEP_DIAG_ATTACH_ATTACHED)
		n = atomic_read(&sep_diag.endpoints);
	return sysfs_emit(buf, "%u\n", n);
}
static DEVICE_ATTR_RO(endpoints);

static ssize_t keystore_show(struct device *dev, struct device_attribute *attr,
			     char *buf)
{
	const char *state;

	switch (atomic_read(&sep_diag.keystore)) {
	case SEP_DIAG_KEYSTORE_OPEN:
		state = "open";
		break;
	case SEP_DIAG_KEYSTORE_CLOSED:
		state = "closed";
		break;
	default:
		state = "unknown";
		break;
	}
	return sysfs_emit(buf, "%s\n", state);
}
static DEVICE_ATTR_RO(keystore);

static ssize_t keybag_show(struct device *dev, struct device_attribute *attr,
			   char *buf)
{
	const char *state;

	switch (atomic_read(&sep_diag.keybag)) {
	case SEP_DIAG_KEYBAG_PRESENT:
		state = "present";
		break;
	case SEP_DIAG_KEYBAG_MISSING:
		state = "missing";
		break;
	case SEP_DIAG_KEYBAG_FAILED:
		state = "failed";
		break;
	default:
		state = "unknown";
		break;
	}
	return sysfs_emit(buf, "%s\n", state);
}
static DEVICE_ATTR_RO(keybag);

static ssize_t sensor_show(struct device *dev, struct device_attribute *attr,
			   char *buf)
{
	const char *state;

	switch (sep_diag_sensor()) {
	case SEP_DIAG_SENSOR_BOUND:
		state = "bound";
		break;
	case SEP_DIAG_SENSOR_ONLINE:
		state = "online";
		break;
	case SEP_DIAG_SENSOR_FAILED:
		state = "failed";
		break;
	default:
		state = "unbound";
		break;
	}
	return sysfs_emit(buf, "%s\n", state);
}
static DEVICE_ATTR_RO(sensor);

static ssize_t touchid_show(struct device *dev, struct device_attribute *attr,
			    char *buf)
{
	const char *state;

	switch (atomic_read(&sep_diag.touchid)) {
	case SEP_DIAG_TOUCHID_STARTED:
		state = sep_diag_sensor() == SEP_DIAG_SENSOR_ONLINE ?
			"ready" : "not-ready";
		break;
	case SEP_DIAG_TOUCHID_UNAVAILABLE:
	case SEP_DIAG_TOUCHID_FAILED:
		state = "failed";
		break;
	default:
		state = "unknown";
		break;
	}
	return sysfs_emit(buf, "%s\n", state);
}
static DEVICE_ATTR_RO(touchid);

static ssize_t sensor_attach_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	const char *state;

	switch (atomic_read(&sep_diag.sensor_attach)) {
	case SEP_DIAG_SENSOR_ATTACH_ATTACHED:
		state = "attached";
		break;
	case SEP_DIAG_SENSOR_ATTACH_RETRYING:
		state = "retrying";
		break;
	case SEP_DIAG_SENSOR_ATTACH_NOT_READY:
		state = "not-ready";
		break;
	default:
		state = "unknown";
		break;
	}
	return sysfs_emit(buf, "%s\n", state);
}
static DEVICE_ATTR_RO(sensor_attach);

static ssize_t restore_show(struct device *dev, struct device_attribute *attr,
			    char *buf)
{
	const char *state;

	switch (atomic_read(&sep_diag.restore)) {
	case SEP_DIAG_RESTORE_RESTORED:
		state = "restored";
		break;
	case SEP_DIAG_RESTORE_EMPTY:
		state = "empty";
		break;
	case SEP_DIAG_RESTORE_UNPROVEN:
		state = "unproven";
		break;
	default:
		state = "unknown";
		break;
	}
	return sysfs_emit(buf, "%s\n", state);
}
static DEVICE_ATTR_RO(restore);

static ssize_t xart_guard_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	return sysfs_emit(buf, "%u\n", READ_ONCE(sep_diag.xart_guard));
}
static DEVICE_ATTR_RO(xart_guard);

static ssize_t xart_guard_list_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	const char *state;

	if (!READ_ONCE(sep_diag.xart_guard))
		state = "none";
	else if (READ_ONCE(sep_diag.xart_guard_list))
		state = "loaded";
	else
		state = "unavailable";
	return sysfs_emit(buf, "%s\n", state);
}
static DEVICE_ATTR_RO(xart_guard_list);

static ssize_t xart_guard_refusals_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	return sysfs_emit(buf, "%u\n",
			  (unsigned int)atomic_read(&sep_diag.xart_refusals));
}
static DEVICE_ATTR_RO(xart_guard_refusals);

static ssize_t matching_show(struct device *dev, struct device_attribute *attr,
			     char *buf)
{
	const char *state;

	switch (atomic_read(&sep_diag.matching)) {
	case SEP_DIAG_MATCHING_USABLE:
		state = "usable";
		break;
	case SEP_DIAG_MATCHING_UNVERIFIED:
		state = "unverified";
		break;
	case SEP_DIAG_MATCHING_BLOCKED:
		state = "blocked";
		break;
	default:
		state = "unknown";
		break;
	}
	return sysfs_emit(buf, "%s\n", state);
}
static DEVICE_ATTR_RO(matching);

/* Milliseconds from probe to a decision, or 0 while it is still pending. */
static unsigned int sep_diag_ms_since_probe(atomic64_t *at)
{
	u64 t = atomic64_read(at);

	if (!t || t < sep_diag.probe_ns)
		return 0;
	return (unsigned int)min_t(u64, div_u64(t - sep_diag.probe_ns,
						NSEC_PER_MSEC), UINT_MAX);
}

static ssize_t attach_ms_show(struct device *dev, struct device_attribute *attr,
			      char *buf)
{
	return sysfs_emit(buf, "%u\n", sep_diag_ms_since_probe(&sep_diag.attach_ns));
}
static DEVICE_ATTR_RO(attach_ms);

static ssize_t ready_ms_show(struct device *dev, struct device_attribute *attr,
			     char *buf)
{
	return sysfs_emit(buf, "%u\n", sep_diag_ms_since_probe(&sep_diag.ready_ns));
}
static DEVICE_ATTR_RO(ready_ms);

/*
 * An endpoint's name as the SEP spells it, trailing spaces dropped ("sks ").
 * A name with a byte that is not a lower-case letter or digit is shown as its
 * number, so the list stays one line of space-separated tokens.
 */
static int sep_diag_ep_name(unsigned int id, char out[5])
{
	u32 name = READ_ONCE(sep_diag.ep_name[id]);
	int len = 0, i;

	for (i = 0; i < 4; i++) {
		char c = (char)(name >> (24 - 8 * i));

		if (c == ' ' && len == i)
			break;
		if (!(islower(c) || isdigit(c)) || len != i)
			return scnprintf(out, 5, "%02x", id);
		out[len++] = c;
	}
	if (!len)
		return scnprintf(out, 5, "%02x", id);
	out[len] = '\0';
	return len;
}

static ssize_t endpoint_names_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	unsigned int id;
	ssize_t n = 0;

	for_each_set_bit(id, sep_diag.ep_seen, SEP_DIAG_MAX_EP) {
		char name[5];

		/* Pairs with the barrier in sep_diag_set_endpoint(). */
		smp_rmb();
		sep_diag_ep_name(id, name);
		n += sysfs_emit_at(buf, n, "%s%s", n ? " " : "", name);
	}
	n += sysfs_emit_at(buf, n, "\n");
	return n;
}
static DEVICE_ATTR_RO(endpoint_names);

static ssize_t os_identity_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	const char *state;

	switch (atomic_read(&sep_diag.os_identity)) {
	case SEP_DIAG_OS_IDENTITY_EXPLICIT:
		state = "explicit";
		break;
	case SEP_DIAG_OS_IDENTITY_CHOSEN:
		state = "chosen";
		break;
	case SEP_DIAG_OS_IDENTITY_UNAVAILABLE:
		state = "unavailable";
		break;
	default:
		state = "unknown";
		break;
	}
	return sysfs_emit(buf, "%s\n", state);
}
static DEVICE_ATTR_RO(os_identity);

static ssize_t hwrng_show(struct device *dev, struct device_attribute *attr,
			  char *buf)
{
	const char *state;

	switch (atomic_read(&sep_diag.hwrng)) {
	case SEP_DIAG_HWRNG_REGISTERED:
		state = "registered";
		break;
	case SEP_DIAG_HWRNG_FAILED:
		state = "failed";
		break;
	default:
		state = "pending";
		break;
	}
	return sysfs_emit(buf, "%s\n", state);
}
static DEVICE_ATTR_RO(hwrng);

static ssize_t stac_show(struct device *dev, struct device_attribute *attr,
			 char *buf)
{
	const char *state;

	switch (atomic_read(&sep_diag.stac)) {
	case SEP_DIAG_STAC_ONLINE:
		state = "online";
		break;
	case SEP_DIAG_STAC_ABSENT:
		state = "absent";
		break;
	case SEP_DIAG_STAC_FAILED:
		state = "failed";
		break;
	default:
		state = "pending";
		break;
	}
	return sysfs_emit(buf, "%s\n", state);
}
static DEVICE_ATTR_RO(stac);

static ssize_t stac_onlines_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	return sysfs_emit(buf, "%u\n",
			  (unsigned int)atomic_read(&sep_diag.stac_onlines));
}
static DEVICE_ATTR_RO(stac_onlines);

static ssize_t pnon_show(struct device *dev, struct device_attribute *attr,
			 char *buf)
{
	const char *state;

	switch (atomic_read(&sep_diag.pnon)) {
	case SEP_DIAG_PNON_READ:
		state = "read";
		break;
	case SEP_DIAG_PNON_ABSENT:
		state = "absent";
		break;
	case SEP_DIAG_PNON_FAILED:
		state = "failed";
		break;
	default:
		state = "pending";
		break;
	}
	return sysfs_emit(buf, "%s\n", state);
}
static DEVICE_ATTR_RO(pnon);

/* Root only: it identifies the machine's boot policy. */
static ssize_t local_policy_digest_show(struct device *dev,
					struct device_attribute *attr, char *buf)
{
	if (atomic_read(&sep_diag.pnon) != SEP_DIAG_PNON_READ)
		return sysfs_emit(buf, "unavailable\n");
	/* Pairs with the smp_wmb() in sep_diag_set_pnon(). */
	smp_rmb();
	return sysfs_emit(buf, "%*phN\n", SEP_DIAG_PNON_DIGEST_LEN,
			  sep_diag_pnon_digest);
}
static DEVICE_ATTR_ADMIN_RO(local_policy_digest);

static ssize_t sep_diag_offer_emit(char *buf, atomic_t *state)
{
	switch (atomic_read(state)) {
	case SEP_DIAG_OFFER_IDLE:
		return sysfs_emit(buf, "idle\n");
	case SEP_DIAG_OFFER_ABSENT:
		return sysfs_emit(buf, "absent\n");
	default:
		return sysfs_emit(buf, "pending\n");
	}
}

static ssize_t sse_show(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	return sep_diag_offer_emit(buf, &sep_diag.sse);
}
static DEVICE_ATTR_RO(sse);

static ssize_t sse_events_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	return sysfs_emit(buf, "%u\n",
			  (unsigned int)atomic_read(&sep_diag.sse_events));
}
static DEVICE_ATTR_RO(sse_events);

static ssize_t hibe_show(struct device *dev, struct device_attribute *attr,
			 char *buf)
{
	return sep_diag_offer_emit(buf, &sep_diag.hibe);
}
static DEVICE_ATTR_RO(hibe);

static ssize_t skdl_show(struct device *dev, struct device_attribute *attr,
			 char *buf)
{
	return sep_diag_offer_emit(buf, &sep_diag.skdl);
}
static DEVICE_ATTR_RO(skdl);

static ssize_t xarm_failures_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	return sysfs_emit(buf, "%u\n",
			  (unsigned int)atomic_read(&sep_diag.xarm_failures));
}
static DEVICE_ATTR_RO(xarm_failures);

static ssize_t unhandled_messages_show(struct device *dev,
				       struct device_attribute *attr, char *buf)
{
	return sysfs_emit(buf, "%u\n",
			  (unsigned int)atomic_read(&sep_diag.unhandled));
}
static DEVICE_ATTR_RO(unhandled_messages);

static ssize_t sleeps_show(struct device *dev, struct device_attribute *attr,
			   char *buf)
{
	return sysfs_emit(buf, "%u\n", (unsigned int)atomic_read(&sep_diag.sleeps));
}
static DEVICE_ATTR_RO(sleeps);

static ssize_t sleep_show(struct device *dev, struct device_attribute *attr,
			  char *buf)
{
	return sysfs_emit(buf, "%s\n",
			  atomic_read(&sep_diag.sleeping) ? "held" : "awake");
}
static DEVICE_ATTR_RO(sleep);

static struct attribute *sep_diag_attrs[] = {
	&dev_attr_abi.attr,
	&dev_attr_profile.attr,
	&dev_attr_boot.attr,
	&dev_attr_protocol.attr,
	&dev_attr_xart.attr,
	&dev_attr_policy.attr,
	&dev_attr_attach.attr,
	&dev_attr_endpoints.attr,
	&dev_attr_keystore.attr,
	&dev_attr_keybag.attr,
	&dev_attr_sensor.attr,
	&dev_attr_touchid.attr,
	&dev_attr_sensor_attach.attr,
	&dev_attr_restore.attr,
	&dev_attr_xart_guard.attr,
	&dev_attr_xart_guard_list.attr,
	&dev_attr_xart_guard_refusals.attr,
	&dev_attr_matching.attr,
	&dev_attr_attach_ms.attr,
	&dev_attr_ready_ms.attr,
	&dev_attr_endpoint_names.attr,
	&dev_attr_os_identity.attr,
	&dev_attr_hwrng.attr,
	&dev_attr_stac.attr,
	&dev_attr_stac_onlines.attr,
	&dev_attr_pnon.attr,
	&dev_attr_local_policy_digest.attr,
	&dev_attr_sse.attr,
	&dev_attr_sse_events.attr,
	&dev_attr_hibe.attr,
	&dev_attr_skdl.attr,
	&dev_attr_xarm_failures.attr,
	&dev_attr_unhandled_messages.attr,
	&dev_attr_sleeps.attr,
	&dev_attr_sleep.attr,
	NULL
};

static const struct attribute_group sep_diag_group = {
	.name = "diag",
	.attrs = sep_diag_attrs,
};

/*
 * The fixed facts are stored before the group is added, so no reader sees them
 * unset. devres removes the group when the driver unbinds.
 */
int sep_diag_register(struct device *dev, const char *profile,
		      size_t profile_len, bool cold, bool sepos13, bool xart)
{
	profile_len = min(profile_len, sizeof(sep_diag.profile) - 1);
	memcpy(sep_diag.profile, profile, profile_len);
	sep_diag.profile[profile_len] = '\0';
	sep_diag.cold = cold;
	sep_diag.sepos13 = sepos13;
	sep_diag.xart = xart;
	sep_diag.probe_ns = ktime_get_boottime_ns();

	return devm_device_add_group(dev, &sep_diag_group);
}

void sep_diag_set_attach(bool attached)
{
	atomic64_cmpxchg(&sep_diag.attach_ns, 0, ktime_get_boottime_ns());
	atomic_set(&sep_diag.attach,
		   attached ? SEP_DIAG_ATTACH_ATTACHED : SEP_DIAG_ATTACH_FAILED);
}

void sep_diag_set_endpoints(unsigned int count)
{
	atomic_set(&sep_diag.endpoints, count);
}

void sep_diag_set_endpoint(u8 id, u32 name)
{
	WRITE_ONCE(sep_diag.ep_name[id], name);
	/* The name is visible before the bit; see endpoint_names_show(). */
	smp_wmb();
	set_bit(id, sep_diag.ep_seen);
}

void sep_diag_set_os_identity(int source)
{
	atomic_set(&sep_diag.os_identity, source);
}

void sep_diag_set_hwrng(bool registered)
{
	atomic_set(&sep_diag.hwrng, registered ? SEP_DIAG_HWRNG_REGISTERED :
						 SEP_DIAG_HWRNG_FAILED);
}

void sep_diag_set_stac(int state)
{
	if (state == SEP_DIAG_STAC_ONLINE)
		atomic_inc(&sep_diag.stac_onlines);
	atomic_set(&sep_diag.stac, state);
}

void sep_diag_set_pnon(int state, const u8 *digest, size_t len)
{
	if (state == SEP_DIAG_PNON_READ) {
		/* Never rewrite a digest a reader may be copying. */
		if (!digest || len != SEP_DIAG_PNON_DIGEST_LEN ||
		    atomic_read(&sep_diag.pnon) == SEP_DIAG_PNON_READ)
			return;
		memcpy(sep_diag_pnon_digest, digest, len);
		/* The digest is visible before the state says it is there. */
		smp_wmb();
	}
	atomic_set(&sep_diag.pnon, state);
}

unsigned int sep_diag_xarm_failed(void)
{
	return (unsigned int)atomic_inc_return(&sep_diag.xarm_failures);
}

void sep_diag_set_offer(u8 endpoint, int state)
{
	switch (endpoint) {
	case SEP_DIAG_EP_SSE:
		atomic_set(&sep_diag.sse, state);
		break;
	case SEP_DIAG_EP_HIBE:
		atomic_set(&sep_diag.hibe, state);
		break;
	case SEP_DIAG_EP_SKDL:
		atomic_set(&sep_diag.skdl, state);
		break;
	}
}

unsigned int sep_diag_sse_event(void)
{
	return (unsigned int)atomic_inc_return(&sep_diag.sse_events);
}

unsigned int sep_diag_unhandled(void)
{
	return (unsigned int)atomic_inc_return(&sep_diag.unhandled);
}

/* From the PM notifier (pm_shim.c), which runs its calls one at a time. */
void sep_diag_sleep(bool entering)
{
	if (entering) {
		atomic_set(&sep_diag.sleeping, 1);
	} else if (atomic_xchg(&sep_diag.sleeping, 0)) {
		atomic_inc(&sep_diag.sleeps);
	}
}

/*
 * One line for the kernel log when the boot-time bring-up has ended: what the
 * per-endpoint discovery lines (now debug level) used to show, and the states
 * a reader would otherwise collect from several attributes.
 */
void sep_diag_log_summary(struct device *dev)
{
	char names[192];
	unsigned int id;
	size_t n = 0;

	names[0] = '\0';
	for_each_set_bit(id, sep_diag.ep_seen, SEP_DIAG_MAX_EP) {
		char name[5];

		smp_rmb();
		sep_diag_ep_name(id, name);
		n += scnprintf(names + n, sizeof(names) - n, "%s%s",
			       n ? " " : "", name);
	}
	dev_info(dev, "ready in %u ms: %u endpoints (%s), key store %s\n",
		 sep_diag_ms_since_probe(&sep_diag.ready_ns),
		 (unsigned int)atomic_read(&sep_diag.endpoints), names,
		 atomic_read(&sep_diag.keystore) == SEP_DIAG_KEYSTORE_OPEN ?
		 "open" : "closed");
}

void sep_diag_set_keystore_open(void)
{
	atomic_set(&sep_diag.keystore, SEP_DIAG_KEYSTORE_OPEN);
}

void sep_diag_set_keybag(int state)
{
	atomic_set(&sep_diag.keybag, state);
}

/*
 * Called from the SPI probe and remove of the one sensor device (sensor_shim.c),
 * which the driver core never runs concurrently. A bring-up result racing this store is
 * either overwritten by it or, arriving later, refused by the bind check below.
 */
void sep_diag_set_sensor_bound(bool bound)
{
	unsigned int binds = (unsigned int)atomic_read(&sep_diag.sensor) &
			     ~SEP_DIAG_SENSOR_STATE;

	if (bound)
		atomic_set(&sep_diag.sensor,
			   (binds + SEP_DIAG_SENSOR_BIND) | SEP_DIAG_SENSOR_BOUND);
	else
		atomic_set(&sep_diag.sensor, binds | SEP_DIAG_SENSOR_UNBOUND);
}

/* Taken as a bring-up begins and handed back with its result. */
unsigned int sep_diag_sensor_bind(void)
{
	return (unsigned int)atomic_read(&sep_diag.sensor) &
	       ~SEP_DIAG_SENSOR_STATE;
}

void sep_diag_set_sensor_result(unsigned int bind, bool online)
{
	unsigned int state = online ? SEP_DIAG_SENSOR_ONLINE :
				      SEP_DIAG_SENSOR_FAILED;
	int old = atomic_read(&sep_diag.sensor);

	do {
		unsigned int word = old;

		/*
		 * With no device bound, "unbound" is the cause worth showing;
		 * after a re-bind, the result belongs to the previous device.
		 */
		if ((word & SEP_DIAG_SENSOR_STATE) == SEP_DIAG_SENSOR_UNBOUND ||
		    (word & ~SEP_DIAG_SENSOR_STATE) != bind)
			return;
	} while (!atomic_try_cmpxchg(&sep_diag.sensor, &old,
				     (old & ~SEP_DIAG_SENSOR_STATE) | state));
}

/*
 * A compare-exchange, not a store: an open of the new node can activate Touch
 * ID before the publisher gets here, and that outcome must stand.
 */
void sep_diag_set_bio_published(void)
{
	atomic_cmpxchg(&sep_diag.touchid, SEP_DIAG_TOUCHID_PENDING,
		       SEP_DIAG_TOUCHID_PUBLISHED);
}

void sep_diag_set_touchid(bool started)
{
	atomic_set(&sep_diag.touchid,
		   started ? SEP_DIAG_TOUCHID_STARTED : SEP_DIAG_TOUCHID_FAILED);
}

void sep_diag_set_sensor_attach(int state)
{
	atomic_set(&sep_diag.sensor_attach, state);
}

void sep_diag_set_restore(int state)
{
	atomic_set(&sep_diag.restore, state);
}

/*
 * Called at probe, before sep_diag_register() adds the group. matching is
 * derived in xart_guard.rs (MatchFacts), which has KUnit tests for each case.
 */
void sep_diag_set_xart_guard(unsigned int level, bool list_loaded,
			     int matching)
{
	WRITE_ONCE(sep_diag.xart_guard, level);
	WRITE_ONCE(sep_diag.xart_guard_list, list_loaded);
	atomic_set(&sep_diag.xart_refusals, 0);
	atomic_set(&sep_diag.matching, matching);
}

/* With matching as the refusal leaves it. */
void sep_diag_xart_guard_refused(int matching)
{
	atomic_inc(&sep_diag.xart_refusals);
	atomic_set(&sep_diag.matching, matching);
}

/*
 * Only the boot-time bring-up opens the key store and publishes /dev/sep-bio,
 * and it is not repeated. Whatever it left undecided is final for this boot:
 * the key store stays closed and Touch ID cannot start.
 */
void sep_diag_set_xart(bool writable)
{
	WRITE_ONCE(sep_diag.xart, writable);
}

void sep_diag_keystore_ended(void)
{
	atomic64_cmpxchg(&sep_diag.ready_ns, 0, ktime_get_boottime_ns());
	atomic_cmpxchg(&sep_diag.keystore, SEP_DIAG_KEYSTORE_UNKNOWN,
		       SEP_DIAG_KEYSTORE_CLOSED);
}

void sep_diag_bringup_ended(void)
{
	sep_diag_keystore_ended();
	atomic_cmpxchg(&sep_diag.touchid, SEP_DIAG_TOUCHID_PENDING,
		       SEP_DIAG_TOUCHID_UNAVAILABLE);
}
