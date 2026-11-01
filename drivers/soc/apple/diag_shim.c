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
#include <linux/device.h>
#include <linux/minmax.h>
#include <linux/string.h>
#include <linux/sysfs.h>

#include "shim.h"

/* The contract version; see Documentation/ABI/testing/sysfs-driver-apple-sep. */
#define SEP_DIAG_ABI			2

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

static struct {
	/* Set at probe, before the group is added. */
	char profile[32];
	bool cold;
	bool sepos13;
	bool xart;
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
} sep_diag;

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
	return sysfs_emit(buf, "%s\n", sep_diag.xart ? "enabled" : "disabled");
}
static DEVICE_ATTR_RO(xart);

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
	return sysfs_emit(buf, "%u\n", sep_diag.xart_guard);
}
static DEVICE_ATTR_RO(xart_guard);

static ssize_t xart_guard_list_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	const char *state;

	if (!sep_diag.xart_guard)
		state = "none";
	else if (sep_diag.xart_guard_list)
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

static struct attribute *sep_diag_attrs[] = {
	&dev_attr_abi.attr,
	&dev_attr_profile.attr,
	&dev_attr_boot.attr,
	&dev_attr_protocol.attr,
	&dev_attr_xart.attr,
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

	return devm_device_add_group(dev, &sep_diag_group);
}

/* A store that opens after probe. Does not add the sysfs group again. */
void sep_diag_set_xart(bool xart)
{
	sep_diag.xart = xart;
}

void sep_diag_set_attach(bool attached)
{
	atomic_set(&sep_diag.attach,
		   attached ? SEP_DIAG_ATTACH_ATTACHED : SEP_DIAG_ATTACH_FAILED);
}

void sep_diag_set_endpoints(unsigned int count)
{
	atomic_set(&sep_diag.endpoints, count);
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
	sep_diag.xart_guard = level;
	sep_diag.xart_guard_list = list_loaded;
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
void sep_diag_bringup_ended(void)
{
	atomic_cmpxchg(&sep_diag.keystore, SEP_DIAG_KEYSTORE_UNKNOWN,
		       SEP_DIAG_KEYSTORE_CLOSED);
	atomic_cmpxchg(&sep_diag.touchid, SEP_DIAG_TOUCHID_PENDING,
		       SEP_DIAG_TOUCHID_UNAVAILABLE);
}
