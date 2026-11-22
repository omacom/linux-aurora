// SPDX-License-Identifier: GPL-2.0

#ifndef __USB_TYPEC_MUX
#define __USB_TYPEC_MUX

#include <linux/err.h>
#include <linux/property.h>
#include <linux/usb/typec.h>
#include <linux/usb/typec_tbt.h>

struct device;
struct typec_mux;
struct typec_mux_dev;
struct typec_switch;
struct typec_switch_dev;
struct typec_altmode;
struct fwnode_handle;
struct notifier_block;

typedef int (*typec_switch_set_fn_t)(struct typec_switch_dev *sw,
				     enum typec_orientation orientation);

struct typec_switch_desc {
	struct fwnode_handle *fwnode;
	typec_switch_set_fn_t set;
	const char *name;
	void *drvdata;
};

#if IS_ENABLED(CONFIG_TYPEC)

struct typec_switch *fwnode_typec_switch_get(struct fwnode_handle *fwnode);
void typec_switch_put(struct typec_switch *sw);
int typec_switch_set(struct typec_switch *sw,
		     enum typec_orientation orientation);

struct typec_switch_dev *
typec_switch_register(struct device *parent,
		      const struct typec_switch_desc *desc);
void typec_switch_unregister(struct typec_switch_dev *sw);

void typec_switch_set_drvdata(struct typec_switch_dev *sw, void *data);
void *typec_switch_get_drvdata(struct typec_switch_dev *sw);

#else

static inline struct typec_switch *
fwnode_typec_switch_get(struct fwnode_handle *fwnode)
{
	return NULL;
}

static inline void typec_switch_put(struct typec_switch *sw) {}

static inline int typec_switch_set(struct typec_switch *sw,
				   enum typec_orientation orientation)
{
	return 0;
}

static inline struct typec_switch_dev *
typec_switch_register(struct device *parent,
		      const struct typec_switch_desc *desc)
{
	return ERR_PTR(-EOPNOTSUPP);
}

static inline void typec_switch_unregister(struct typec_switch_dev *sw) {}

static inline void typec_switch_set_drvdata(struct typec_switch_dev *sw, void *data) {}
static inline void *typec_switch_get_drvdata(struct typec_switch_dev *sw)
{
	return ERR_PTR(-EOPNOTSUPP);
}

#endif /* CONFIG_TYPEC */

static inline struct typec_switch *typec_switch_get(struct device *dev)
{
	return fwnode_typec_switch_get(dev_fwnode(dev));
}

struct typec_mux_state {
	struct typec_altmode *alt;
	unsigned long mode;
	void *data;
};

typedef void (*typec_mux_notify_fn_t)(struct typec_mux_dev *mux,
				     struct typec_mux_state *state);

typedef int (*typec_mux_set_fn_t)(struct typec_mux_dev *mux,
				  struct typec_mux_state *state);

struct typec_mux_desc {
	struct fwnode_handle *fwnode;
	typec_mux_set_fn_t set;
	const char *name;
	void *drvdata;
	/* Quiesce consumers before any provider changes the shared route. */
	typec_mux_set_fn_t prepare;
	/* Fallible finalization retains ownership; commit cannot fail. */
	typec_mux_set_fn_t finish;
	typec_mux_notify_fn_t commit;
	typec_mux_notify_fn_t abort;
	/* SAFE needs the USB role consumer to stop the shared PHY first. */
	bool safe_after_role;
	/*
	 * -ETIMEDOUT from @set only means that the hardware held the change up
	 * for now: the same state can be applied again later, and is worth
	 * applying again for longer than after other failures. Consumers learn
	 * it from typec_mux_timeout_transient().
	 */
	bool transient_timeouts;
	/*
	 * DisplayPort pin D comes beside a USB 3 link this provider already runs
	 * without that link stopping: the lanes USB 3 runs on stay as they are,
	 * so the USB role does not have to stop for the change. Only the
	 * provider that owns the port's lanes may declare it, and its word holds
	 * for the whole port (typec_mux_dp_joins_usb()).
	 */
	bool dp_joins_usb;
};

#if IS_ENABLED(CONFIG_TYPEC)

struct typec_mux *fwnode_typec_mux_get(struct fwnode_handle *fwnode);
void typec_mux_put(struct typec_mux *mux);
/* begin/end must run in the same task; failure retains provider recovery state. */
/* Same-task begin/end; retain the handle and state pointees until end/abort. */
int typec_mux_begin(struct typec_mux *mux, struct typec_mux_state *state);
/* Apply the providers while retaining the transaction and ownership. */
int typec_mux_apply(struct typec_mux *mux);
bool typec_mux_safe_after_role(struct typec_mux *mux);
bool typec_mux_timeout_transient(struct typec_mux *mux);
bool typec_mux_dp_joins_usb(struct typec_mux *mux);
int typec_mux_end(struct typec_mux *mux);
void typec_mux_abort(struct typec_mux *mux);
int typec_mux_prepare(struct typec_mux *mux, struct typec_mux_state *state);
int typec_mux_set(struct typec_mux *mux, struct typec_mux_state *state);

struct typec_mux_dev *
typec_mux_register(struct device *parent, const struct typec_mux_desc *desc);
void typec_mux_unregister(struct typec_mux_dev *mux);

void typec_mux_set_drvdata(struct typec_mux_dev *mux, void *data);
void *typec_mux_get_drvdata(struct typec_mux_dev *mux);

#else

static inline struct typec_mux *fwnode_typec_mux_get(struct fwnode_handle *fwnode)
{
	return NULL;
}

static inline void typec_mux_put(struct typec_mux *mux) {}

static inline int typec_mux_begin(struct typec_mux *mux, struct typec_mux_state *state)
{
	return 0;
}

static inline int typec_mux_apply(struct typec_mux *mux) { return 0; }

static inline bool typec_mux_safe_after_role(struct typec_mux *mux)
{
	return false;
}

static inline bool typec_mux_timeout_transient(struct typec_mux *mux)
{
	return false;
}

static inline bool typec_mux_dp_joins_usb(struct typec_mux *mux)
{
	return false;
}

static inline int typec_mux_end(struct typec_mux *mux) { return 0; }
static inline void typec_mux_abort(struct typec_mux *mux) {}

static inline int typec_mux_prepare(struct typec_mux *mux, struct typec_mux_state *state)
{
	return 0;
}

static inline int typec_mux_set(struct typec_mux *mux, struct typec_mux_state *state)
{
	return 0;
}

static inline struct typec_mux_dev *
typec_mux_register(struct device *parent, const struct typec_mux_desc *desc)
{
	return ERR_PTR(-EOPNOTSUPP);
}
static inline void typec_mux_unregister(struct typec_mux_dev *mux) {}

static inline void typec_mux_set_drvdata(struct typec_mux_dev *mux, void *data) {}
static inline void *typec_mux_get_drvdata(struct typec_mux_dev *mux)
{
	return ERR_PTR(-EOPNOTSUPP);
}

#endif /* CONFIG_TYPEC */

static inline struct typec_mux *typec_mux_get(struct device *dev)
{
	return fwnode_typec_mux_get(dev_fwnode(dev));
}

struct typec_thunderbolt_switch;
struct typec_thunderbolt_switch_dev;

/* Recheck fresh cable data and provider health; not a request to reset. */
#define TYPEC_THUNDERBOLT_SWITCH_REVALIDATE 1
/* A deferred activation completed; this is not a link-failure event. */
#define TYPEC_THUNDERBOLT_SWITCH_READY 2
/* The consumer powered down for system sleep and needs a fresh cable read. */
#define TYPEC_THUNDERBOLT_SWITCH_RESTORE 3
/* Sent to typec_thunderbolt_switch_watch() notifiers: a switch registered. */
#define TYPEC_THUNDERBOLT_SWITCH_ADDED 4

enum typec_thunderbolt_switch_state {
	TYPEC_THUNDERBOLT_SWITCH_OFF,
	TYPEC_THUNDERBOLT_SWITCH_TBT,
	TYPEC_THUNDERBOLT_SWITCH_USB4,
};

/**
 * struct typec_thunderbolt_switch_data - Type-C Thunderbolt/USB4 Switch Data
 * @state: Switch state (TBT, USB4, or OFF)
 * @orientation: Orientation of the connection
 * @tbt: Thunderbolt 3 specific data, only valid in
 *	 TYPEC_THUNDERBOLT_SWITCH_TBT
 * @usb4: Enter_USB data, only valid in TYPEC_THUNDERBOLT_SWITCH_USB4
 */
struct typec_thunderbolt_switch_data {
	enum typec_thunderbolt_switch_state state;
	enum typec_orientation orientation;
	union {
		struct typec_thunderbolt_data tbt;
		struct enter_usb_data usb4;
	};
};

typedef int (*typec_thunderbolt_switch_set_fn_t)(struct typec_thunderbolt_switch_dev *sw,
						 const struct typec_thunderbolt_switch_data *data);

/**
 * struct typec_thunderbolt_switch_desc - USB4/Thunderbolt switch description
 * @fwnode: The fwnode the switch is associated with
 * @set: Callback invoked with the cable details on connection changes
 * @name: Optional switch name, the parent device name is used if not set
 * @drvdata: Private data
 */
struct typec_thunderbolt_switch_desc {
	struct fwnode_handle *fwnode;
	typec_thunderbolt_switch_set_fn_t set;
	const char *name;
	void *drvdata;
};

#if IS_ENABLED(CONFIG_TYPEC)

struct typec_thunderbolt_switch *
fwnode_typec_thunderbolt_switch_get(struct fwnode_handle *fwnode);
void typec_thunderbolt_switch_put(struct typec_thunderbolt_switch *sw);
bool typec_thunderbolt_switch_is_live(struct typec_thunderbolt_switch *sw);
int typec_thunderbolt_switch_set(struct typec_thunderbolt_switch *sw,
				 const struct typec_thunderbolt_switch_data *data);
int typec_thunderbolt_switch_register_notifier(struct typec_thunderbolt_switch *sw,
					       struct notifier_block *nb);
void typec_thunderbolt_switch_unregister_notifier(struct typec_thunderbolt_switch *sw,
						  struct notifier_block *nb);
void typec_thunderbolt_switch_notify(struct typec_thunderbolt_switch_dev *sw);
void typec_thunderbolt_switch_notify_ready(struct typec_thunderbolt_switch_dev *sw);
void typec_thunderbolt_switch_notify_restore(struct typec_thunderbolt_switch_dev *sw);
int typec_thunderbolt_switch_watch(struct notifier_block *nb);
void typec_thunderbolt_switch_unwatch(struct notifier_block *nb);

struct typec_thunderbolt_switch_dev *
typec_thunderbolt_switch_register(struct device *parent,
				  const struct typec_thunderbolt_switch_desc *desc);
void typec_thunderbolt_switch_unregister(struct typec_thunderbolt_switch_dev *sw);

void typec_thunderbolt_switch_set_drvdata(struct typec_thunderbolt_switch_dev *sw,
					  void *data);
void *
typec_thunderbolt_switch_get_drvdata(struct typec_thunderbolt_switch_dev *sw);

#else

static inline struct typec_thunderbolt_switch *
fwnode_typec_thunderbolt_switch_get(struct fwnode_handle *fwnode)
{
	return NULL;
}

static inline void
typec_thunderbolt_switch_put(struct typec_thunderbolt_switch *sw)
{
}

static inline int
typec_thunderbolt_switch_set(struct typec_thunderbolt_switch *sw,
			     const struct typec_thunderbolt_switch_data *data)
{
	return 0;
}

static inline struct typec_thunderbolt_switch_dev *
typec_thunderbolt_switch_register(struct device *parent,
				  const struct typec_thunderbolt_switch_desc *desc)
{
	return ERR_PTR(-EOPNOTSUPP);
}

static inline bool
typec_thunderbolt_switch_is_live(struct typec_thunderbolt_switch *sw)
{
	return false;
}

static inline int
typec_thunderbolt_switch_register_notifier(struct typec_thunderbolt_switch *sw,
					   struct notifier_block *nb)
{
	return 0;
}

static inline void
typec_thunderbolt_switch_unregister_notifier(struct typec_thunderbolt_switch *sw,
					     struct notifier_block *nb)
{
}

static inline void
typec_thunderbolt_switch_notify(struct typec_thunderbolt_switch_dev *sw)
{
}

static inline void
typec_thunderbolt_switch_notify_ready(struct typec_thunderbolt_switch_dev *sw)
{
}

static inline void
typec_thunderbolt_switch_notify_restore(struct typec_thunderbolt_switch_dev *sw)
{
}

static inline int typec_thunderbolt_switch_watch(struct notifier_block *nb)
{
	return 0;
}

static inline void typec_thunderbolt_switch_unwatch(struct notifier_block *nb)
{
}

static inline void
typec_thunderbolt_switch_unregister(struct typec_thunderbolt_switch_dev *sw)
{
}

static inline void
typec_thunderbolt_switch_set_drvdata(struct typec_thunderbolt_switch_dev *sw,
				     void *data)
{
}

static inline void *
typec_thunderbolt_switch_get_drvdata(struct typec_thunderbolt_switch_dev *sw)
{
	return NULL;
}

#endif /* CONFIG_TYPEC */

static inline struct typec_thunderbolt_switch *
typec_thunderbolt_switch_get(struct device *dev)
{
	return fwnode_typec_thunderbolt_switch_get(dev_fwnode(dev));
}

#endif /* __USB_TYPEC_MUX */
