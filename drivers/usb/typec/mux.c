// SPDX-License-Identifier: GPL-2.0
/*
 * USB Type-C Multiplexer/DeMultiplexer Switch support
 *
 * Copyright (C) 2018 Intel Corporation
 * Author: Heikki Krogerus <heikki.krogerus@linux.intel.com>
 *         Hans de Goede <hdegoede@redhat.com>
 */

#include <linux/device.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/soc/apple/j613-display.h>
#include <linux/mutex.h>
#include <linux/property.h>
#include <linux/slab.h>
#include <linux/srcu.h>
#include <linux/sched.h>

#include "class.h"
#include "mux.h"

#define TYPEC_MUX_MAX_DEVS	3

struct typec_switch {
	struct typec_switch_dev *sw_devs[TYPEC_MUX_MAX_DEVS];
	unsigned int num_sw_devs;
};

static int switch_fwnode_match(struct device *dev, const void *fwnode)
{
	if (!is_typec_switch_dev(dev))
		return 0;

	return device_match_fwnode(dev, fwnode);
}

static void *typec_switch_match(const struct fwnode_handle *fwnode,
				const char *id, void *data)
{
	struct typec_switch_dev **sw_devs = data;
	struct device *dev;
	int i;

	/*
	 * Device graph (OF graph) does not give any means to identify the
	 * device type or the device class of the remote port parent that @fwnode
	 * represents, so in order to identify the type or the class of @fwnode
	 * an additional device property is needed. With typec switches the
	 * property is named "orientation-switch" (@id). The value of the device
	 * property is ignored.
	 */
	if (id && !fwnode_property_present(fwnode, id))
		return NULL;

	/*
	 * At this point we are sure that @fwnode is a typec switch in all
	 * cases. If the switch hasn't yet been registered for some reason, the
	 * function "defers probe" for now.
	 */
	dev = class_find_device(&typec_mux_class, NULL, fwnode,
				switch_fwnode_match);
	if (!dev)
		return ERR_PTR(-EPROBE_DEFER);

	/* Skip duplicates */
	for (i = 0; i < TYPEC_MUX_MAX_DEVS; i++)
		if (to_typec_switch_dev(dev) == sw_devs[i]) {
			put_device(dev);
			return NULL;
		}

	return to_typec_switch_dev(dev);
}

/**
 * fwnode_typec_switch_get - Find USB Type-C orientation switch
 * @fwnode: The caller device node
 *
 * Finds a switch linked with @dev. Returns a reference to the switch on
 * success, NULL if no matching connection was found, or
 * ERR_PTR(-EPROBE_DEFER) when a connection was found but the switch
 * has not been enumerated yet.
 */
struct typec_switch *fwnode_typec_switch_get(struct fwnode_handle *fwnode)
{
	struct typec_switch_dev *sw_devs[TYPEC_MUX_MAX_DEVS] = {};
	struct typec_switch *sw;
	int count;
	int err;
	int i;

	sw = kzalloc_obj(*sw);
	if (!sw)
		return ERR_PTR(-ENOMEM);

	count = fwnode_connection_find_matches(fwnode, "orientation-switch",
					       (void **)sw_devs,
					       typec_switch_match,
					       (void **)sw_devs,
					       ARRAY_SIZE(sw_devs));
	if (count <= 0) {
		kfree(sw);
		return NULL;
	}

	for (i = 0; i < count; i++) {
		if (IS_ERR(sw_devs[i])) {
			err = PTR_ERR(sw_devs[i]);
			goto put_sw_devs;
		}
	}

	for (i = 0; i < count; i++) {
		WARN_ON(!try_module_get(sw_devs[i]->dev.parent->driver->owner));
		sw->sw_devs[i] = sw_devs[i];
	}

	sw->num_sw_devs = count;

	return sw;

put_sw_devs:
	for (i = 0; i < count; i++) {
		if (!IS_ERR(sw_devs[i]))
			put_device(&sw_devs[i]->dev);
	}

	kfree(sw);

	return ERR_PTR(err);
}
EXPORT_SYMBOL_GPL(fwnode_typec_switch_get);

/**
 * typec_switch_put - Release USB Type-C orientation switch
 * @sw: USB Type-C orientation switch
 *
 * Decrement reference count for @sw.
 */
void typec_switch_put(struct typec_switch *sw)
{
	struct typec_switch_dev *sw_dev;
	unsigned int i;

	if (IS_ERR_OR_NULL(sw))
		return;

	for (i = 0; i < sw->num_sw_devs; i++) {
		sw_dev = sw->sw_devs[i];

		module_put(sw_dev->dev.parent->driver->owner);
		put_device(&sw_dev->dev);
	}
	kfree(sw);
}
EXPORT_SYMBOL_GPL(typec_switch_put);

static void typec_switch_release(struct device *dev)
{
	kfree(to_typec_switch_dev(dev));
}

const struct device_type typec_switch_dev_type = {
	.name = "orientation_switch",
	.release = typec_switch_release,
};

/**
 * typec_switch_register - Register USB Type-C orientation switch
 * @parent: Parent device
 * @desc: Orientation switch description
 *
 * This function registers a switch that can be used for routing the correct
 * data pairs depending on the cable plug orientation from the USB Type-C
 * connector to the USB controllers. USB Type-C plugs can be inserted
 * right-side-up or upside-down.
 */
struct typec_switch_dev *
typec_switch_register(struct device *parent,
		      const struct typec_switch_desc *desc)
{
	struct typec_switch_dev *sw_dev;
	int ret;

	if (!desc || !desc->set)
		return ERR_PTR(-EINVAL);

	sw_dev = kzalloc_obj(*sw_dev);
	if (!sw_dev)
		return ERR_PTR(-ENOMEM);

	sw_dev->set = desc->set;

	device_initialize(&sw_dev->dev);
	sw_dev->dev.parent = parent;
	sw_dev->dev.fwnode = desc->fwnode;
	sw_dev->dev.class = &typec_mux_class;
	sw_dev->dev.type = &typec_switch_dev_type;
	sw_dev->dev.driver_data = desc->drvdata;
	ret = dev_set_name(&sw_dev->dev, "%s-switch", desc->name ? desc->name : dev_name(parent));
	if (ret) {
		put_device(&sw_dev->dev);
		return ERR_PTR(ret);
	}

	ret = device_add(&sw_dev->dev);
	if (ret) {
		dev_err(parent, "failed to register switch (%d)\n", ret);
		put_device(&sw_dev->dev);
		return ERR_PTR(ret);
	}

	return sw_dev;
}
EXPORT_SYMBOL_GPL(typec_switch_register);

int typec_switch_set(struct typec_switch *sw,
		     enum typec_orientation orientation)
{
	struct typec_switch_dev *sw_dev;
	unsigned int i;
	int ret;

	if (IS_ERR_OR_NULL(sw))
		return 0;

	for (i = 0; i < sw->num_sw_devs; i++) {
		sw_dev = sw->sw_devs[i];

		ret = sw_dev->set(sw_dev, orientation);
		if (ret && ret != -EOPNOTSUPP)
			return ret;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(typec_switch_set);

/**
 * typec_switch_unregister - Unregister USB Type-C orientation switch
 * @sw_dev: USB Type-C orientation switch
 *
 * Unregister switch that was registered with typec_switch_register().
 */
void typec_switch_unregister(struct typec_switch_dev *sw_dev)
{
	if (!IS_ERR_OR_NULL(sw_dev))
		device_unregister(&sw_dev->dev);
}
EXPORT_SYMBOL_GPL(typec_switch_unregister);

void typec_switch_set_drvdata(struct typec_switch_dev *sw_dev, void *data)
{
	dev_set_drvdata(&sw_dev->dev, data);
}
EXPORT_SYMBOL_GPL(typec_switch_set_drvdata);

void *typec_switch_get_drvdata(struct typec_switch_dev *sw_dev)
{
	return dev_get_drvdata(&sw_dev->dev);
}
EXPORT_SYMBOL_GPL(typec_switch_get_drvdata);

/* ------------------------------------------------------------------------- */

/* Class, altmode and controller handles for one connector share this domain. */
struct typec_mux_context {
	struct list_head list;
	struct fwnode_handle *fwnode;
	struct mutex lock;
	unsigned int users;
};

static LIST_HEAD(typec_mux_contexts);
static DEFINE_MUTEX(typec_mux_contexts_lock);
DEFINE_STATIC_SRCU(typec_mux_srcu);

struct typec_mux {
	struct typec_mux_dev *mux_devs[TYPEC_MUX_MAX_DEVS];
	unsigned int num_mux_devs;
	struct typec_mux_context *context;
	struct typec_mux_state transaction;
	struct task_struct *owner;
	int srcu_index;
	unsigned int prepared;
	bool applied;
	/* The last request timed out in a provider with transient timeouts. */
	bool timeout_transient;
};

static struct typec_mux_context *typec_mux_context_get(struct fwnode_handle *fwnode)
{
	struct typec_mux_context *context;

	guard(mutex)(&typec_mux_contexts_lock);
	list_for_each_entry(context, &typec_mux_contexts, list) {
		if (context->fwnode == fwnode) {
			context->users++;
			return context;
		}
	}
	context = kzalloc_obj(*context);
	if (!context)
		return ERR_PTR(-ENOMEM);
	context->fwnode = fwnode_handle_get(fwnode);
	context->users = 1;
	mutex_init(&context->lock);
	list_add_tail(&context->list, &typec_mux_contexts);
	return context;
}

static void typec_mux_context_put(struct typec_mux_context *context)
{
	guard(mutex)(&typec_mux_contexts_lock);
	if (--context->users)
		return;
	list_del(&context->list);
	fwnode_handle_put(context->fwnode);
	kfree(context);
}

/* Immutable provider policy; the handle pins each provider until put. */
bool typec_mux_safe_after_role(struct typec_mux *mux)
{
	unsigned int i;

	if (IS_ERR_OR_NULL(mux))
		return false;

	for (i = 0; i < mux->num_mux_devs; i++) {
		if (mux->mux_devs[i]->safe_after_role)
			return true;
	}

	return false;
}
EXPORT_SYMBOL_GPL(typec_mux_safe_after_role);

/**
 * typec_mux_dp_joins_usb - Check whether DisplayPort pin D can join a running USB 3 link
 * @mux: Mux handle
 *
 * The provider that owns the port's lanes says so (&typec_mux_desc.dp_joins_usb) when it
 * adds DisplayPort pin D beside a USB 3 link it already runs and leaves that link's lanes
 * as they are. The port's USB role then need not stop for that change. As with
 * typec_mux_safe_after_role(), one provider's word holds for the whole port: the other
 * providers of @mux must not touch the lanes.
 *
 * Return: true if a provider of @mux declared it.
 */
bool typec_mux_dp_joins_usb(struct typec_mux *mux)
{
	unsigned int i;

	if (IS_ERR_OR_NULL(mux))
		return false;

	for (i = 0; i < mux->num_mux_devs; i++) {
		if (mux->mux_devs[i]->dp_joins_usb)
			return true;
	}

	return false;
}
EXPORT_SYMBOL_GPL(typec_mux_dp_joins_usb);

/**
 * typec_mux_timeout_transient - Check whether a mux request timed out for now
 * @mux: Mux handle
 *
 * For use after a request on @mux, by typec_mux_set() or by typec_mux_begin()
 * and the calls that follow it, failed with -ETIMEDOUT. A handle can hold more
 * than one provider, and only the one that timed out is asked: whether it
 * declared its timeouts transient (&typec_mux_desc.transient_timeouts). Its
 * hardware then only held the change up, and the same state can be applied
 * again later. A timeout of any other provider can be permanent.
 *
 * The answer describes the last request on @mux that failed with -ETIMEDOUT,
 * so ask before any other request on the same handle runs. One that fails
 * before it owns the providers, such as with -EBUSY, leaves the answer as it
 * was; any other request changes it.
 *
 * Return: true if the provider that timed out declared its timeouts transient.
 */
bool typec_mux_timeout_transient(struct typec_mux *mux)
{
	return !IS_ERR_OR_NULL(mux) && mux->timeout_transient;
}
EXPORT_SYMBOL_GPL(typec_mux_timeout_transient);

static int mux_fwnode_match(struct device *dev, const void *fwnode)
{
	if (!is_typec_mux_dev(dev))
		return 0;

	return device_match_fwnode(dev, fwnode);
}

/*
 * The connection lookup checks that a mux's own node is available, but the
 * device that registers a mux can sit further up: a display processor
 * registers its Type-C routes, which are child nodes of its own. Under a
 * node that is not available, such as one the boot loader left disabled,
 * nothing ever registers the mux.
 */
static bool typec_mux_ancestors_available(const struct fwnode_handle *fwnode)
{
	struct fwnode_handle *parent;

	fwnode_for_each_parent_node(fwnode, parent) {
		if (!fwnode_device_is_available(parent)) {
			fwnode_handle_put(parent);
			return false;
		}
	}
	return true;
}

static void *typec_mux_match(const struct fwnode_handle *fwnode,
			     const char *id, void *data)
{
	struct typec_mux_dev **mux_devs = data;
	struct device *dev;
	int i;

	/*
	 * Device graph (OF graph) does not give any means to identify the
	 * device type or the device class of the remote port parent that @fwnode
	 * represents, so in order to identify the type or the class of @fwnode
	 * an additional device property is needed. With typec muxes the
	 * property is named "mode-switch" (@id). The value of the device
	 * property is ignored.
	 */
	if (id && !fwnode_property_present(fwnode, id))
		return NULL;

	if (apple_j613_display_skip(fwnode, false))
		return NULL;

	dev = class_find_device(&typec_mux_class, NULL, fwnode,
				mux_fwnode_match);
	if (!dev)
		return !apple_j613_display_skip(fwnode, true) &&
		       typec_mux_ancestors_available(fwnode) ? ERR_PTR(-EPROBE_DEFER) : NULL;

	/* Skip duplicates */
	for (i = 0; i < TYPEC_MUX_MAX_DEVS; i++)
		if (to_typec_mux_dev(dev) == mux_devs[i]) {
			put_device(dev);
			return NULL;
		}


	return to_typec_mux_dev(dev);
}

/**
 * fwnode_typec_mux_get - Find USB Type-C Multiplexer
 * @fwnode: The caller device node
 *
 * Finds a mux linked to the caller. This function is primarily meant for the
 * Type-C drivers. Returns a reference to the mux on success, NULL if no
 * matching connection was found, or ERR_PTR(-EPROBE_DEFER) when a connection
 * was found but the mux has not been enumerated yet.
 */
struct typec_mux *fwnode_typec_mux_get(struct fwnode_handle *fwnode)
{
	struct typec_mux_dev *mux_devs[TYPEC_MUX_MAX_DEVS] = {};
	struct typec_mux *mux;
	int count;
	int err;
	int i;
	int index;

	mux = kzalloc_obj(*mux);
	if (!mux)
		return ERR_PTR(-ENOMEM);

	/* Unregister must drain lookup and module-reference conversion too. */
	index = srcu_read_lock(&typec_mux_srcu);
	count = fwnode_connection_find_matches(fwnode, "mode-switch",
					       (void **)mux_devs,
					       typec_mux_match,
					       (void **)mux_devs,
					       ARRAY_SIZE(mux_devs));
	if (count <= 0) {
		srcu_read_unlock(&typec_mux_srcu, index);
		kfree(mux);
		return NULL;
	}

	for (i = 0; i < count; i++) {
		if (IS_ERR(mux_devs[i])) {
			err = PTR_ERR(mux_devs[i]);
			goto put_mux_devs;
		}
	}

	for (i = 0; i < count; i++) {
		if (READ_ONCE(mux_devs[i]->removed) ||
		    !try_module_get(mux_devs[i]->owner)) {
			err = -ENODEV;
			while (i--)
				module_put(mux_devs[i]->owner);
			goto put_mux_devs;
		}
		mux->mux_devs[i] = mux_devs[i];
	}

	mux->num_mux_devs = count;
	mux->context = typec_mux_context_get(fwnode);
	if (IS_ERR(mux->context)) {
		err = PTR_ERR(mux->context);
		for (i = 0; i < count; i++)
			module_put(mux_devs[i]->owner);
		goto put_mux_devs;
	}

	srcu_read_unlock(&typec_mux_srcu, index);
	return mux;

put_mux_devs:
	for (i = 0; i < count; i++) {
		if (!IS_ERR(mux_devs[i]))
			put_device(&mux_devs[i]->dev);
	}

	srcu_read_unlock(&typec_mux_srcu, index);
	kfree(mux);

	return ERR_PTR(err);
}
EXPORT_SYMBOL_GPL(fwnode_typec_mux_get);

/**
 * typec_mux_put - Release handle to a Multiplexer
 * @mux: USB Type-C Connector Multiplexer/DeMultiplexer
 *
 * Decrements reference count for @mux.
 */
void typec_mux_put(struct typec_mux *mux)
{
	struct typec_mux_dev *mux_dev;
	unsigned int i;

	if (IS_ERR_OR_NULL(mux))
		return;

	for (i = 0; i < mux->num_mux_devs; i++) {
		mux_dev = mux->mux_devs[i];
		module_put(mux_dev->owner);
		put_device(&mux_dev->dev);
	}
	typec_mux_context_put(mux->context);
	kfree(mux);
}
EXPORT_SYMBOL_GPL(typec_mux_put);

static void typec_mux_unlock(struct typec_mux *mux)
{
	int index = mux->srcu_index;

	mux->prepared = 0;
	mux->applied = false;
	mux->owner = NULL;
	mutex_unlock(&mux->context->lock);
	srcu_read_unlock(&typec_mux_srcu, index);
}

void typec_mux_abort(struct typec_mux *mux)
{
	unsigned int i;

	if (IS_ERR_OR_NULL(mux))
		return;
	if (WARN_ON_ONCE(mux->owner != current))
		return;
	for (i = mux->prepared; i > 0; i--) {
		struct typec_mux_dev *dev = mux->mux_devs[i - 1];

		if (dev->abort)
			dev->abort(dev, &mux->transaction);
	}
	typec_mux_unlock(mux);
}
EXPORT_SYMBOL_GPL(typec_mux_abort);

int typec_mux_begin(struct typec_mux *mux, struct typec_mux_state *state)
{
	unsigned int i;
	int index, ret;

	if (IS_ERR_OR_NULL(mux))
		return 0;
	index = srcu_read_lock(&typec_mux_srcu);
	/* Nonblocking acquisition also rejects same-domain forwarding recursion. */
	if (!mutex_trylock(&mux->context->lock)) {
		srcu_read_unlock(&typec_mux_srcu, index);
		return -EBUSY;
	}
	mux->srcu_index = index;
	mux->owner = current;
	mux->transaction = *state;
	mux->timeout_transient = false;
	for (i = 0; i < mux->num_mux_devs; i++) {
		struct typec_mux_dev *dev = mux->mux_devs[i];

		if (READ_ONCE(dev->removed)) {
			typec_mux_unlock(mux);
			return -ENODEV;
		}
	}
	for (i = 0; i < mux->num_mux_devs; i++) {
		struct typec_mux_dev *dev = mux->mux_devs[i];

		mux->prepared = i + 1;
		if (!dev->prepare)
			continue;
		ret = dev->prepare(dev, &mux->transaction);
		if (ret)
			goto abort;
	}
	return 0;
abort:
	typec_mux_abort(mux);
	return ret;
}
EXPORT_SYMBOL_GPL(typec_mux_begin);

int typec_mux_apply(struct typec_mux *mux)
{
	unsigned int i;
	int ret;

	if (IS_ERR_OR_NULL(mux))
		return 0;
	if (WARN_ON_ONCE(mux->owner != current))
		return -EINVAL;
	if (mux->applied)
		return 0;
	for (i = 0; i < mux->num_mux_devs; i++) {
		struct typec_mux_dev *dev = mux->mux_devs[i];

		ret = dev->set(dev, &mux->transaction);
		if (ret && ret != -EOPNOTSUPP) {
			/* Only the provider that failed knows what its timeout means. */
			mux->timeout_transient = ret == -ETIMEDOUT &&
						 dev->transient_timeouts;
			typec_mux_abort(mux);
			return ret;
		}
	}
	mux->applied = true;
	return 0;
}
EXPORT_SYMBOL_GPL(typec_mux_apply);

int typec_mux_end(struct typec_mux *mux)
{
	unsigned int i;
	int ret;

	if (IS_ERR_OR_NULL(mux))
		return 0;
	ret = typec_mux_apply(mux);
	if (ret)
		return ret;
	/* Finalizers can fail, but must not release their software reservation. */
	for (i = 0; i < mux->num_mux_devs; i++) {
		struct typec_mux_dev *dev = mux->mux_devs[i];

		if (!dev->finish)
			continue;
		ret = dev->finish(dev, &mux->transaction);
		if (ret)
			goto abort;
	}
	/* Only this no-fail publication phase may release owner reservations. */
	for (i = 0; i < mux->num_mux_devs; i++) {
		struct typec_mux_dev *dev = mux->mux_devs[i];

		if (dev->commit)
			dev->commit(dev, &mux->transaction);
	}
	typec_mux_unlock(mux);
	return 0;
abort:
	typec_mux_abort(mux);
	return ret;
}
EXPORT_SYMBOL_GPL(typec_mux_end);

/* Preparation-only callers leave recovery ownership pending, never committed. */
int typec_mux_prepare(struct typec_mux *mux, struct typec_mux_state *state)
{
	int ret = typec_mux_begin(mux, state);

	if (!ret)
		typec_mux_abort(mux);
	return ret;
}
EXPORT_SYMBOL_GPL(typec_mux_prepare);

int typec_mux_set(struct typec_mux *mux, struct typec_mux_state *state)
{
	int ret = typec_mux_begin(mux, state);

	if (ret)
		return ret;
	return typec_mux_end(mux);
}
EXPORT_SYMBOL_GPL(typec_mux_set);

static void typec_mux_release(struct device *dev)
{
	kfree(to_typec_mux_dev(dev));
}

const struct device_type typec_mux_dev_type = {
	.name = "mode_switch",
	.release = typec_mux_release,
};

/**
 * typec_mux_register - Register Multiplexer routing USB Type-C pins
 * @parent: Parent device
 * @desc: Multiplexer description
 *
 * USB Type-C connectors can be used for alternate modes of operation besides
 * USB when Accessory/Alternate Modes are supported. With some of those modes,
 * the pins on the connector need to be reconfigured. This function registers
 * multiplexer switches routing the pins on the connector.
 */
struct typec_mux_dev *
typec_mux_register(struct device *parent, const struct typec_mux_desc *desc)
{
	struct typec_mux_dev *mux_dev;
	int ret;

	if (!desc || !desc->set)
		return ERR_PTR(-EINVAL);

	mux_dev = kzalloc_obj(*mux_dev);
	if (!mux_dev)
		return ERR_PTR(-ENOMEM);

	mux_dev->owner = parent->driver ? parent->driver->owner : NULL;
	mux_dev->set = desc->set;
	mux_dev->prepare = desc->prepare;
	mux_dev->finish = desc->finish;
	mux_dev->commit = desc->commit;
	mux_dev->abort = desc->abort;
	mux_dev->safe_after_role = desc->safe_after_role;
	mux_dev->transient_timeouts = desc->transient_timeouts;
	mux_dev->dp_joins_usb = desc->dp_joins_usb;

	device_initialize(&mux_dev->dev);
	mux_dev->dev.parent = parent;
	mux_dev->dev.fwnode = desc->fwnode;
	mux_dev->dev.class = &typec_mux_class;
	mux_dev->dev.type = &typec_mux_dev_type;
	mux_dev->dev.driver_data = desc->drvdata;
	ret = dev_set_name(&mux_dev->dev, "%s-mux", desc->name ? desc->name : dev_name(parent));
	if (ret) {
		put_device(&mux_dev->dev);
		return ERR_PTR(ret);
	}

	ret = device_add(&mux_dev->dev);
	if (ret) {
		dev_err(parent, "failed to register mux (%d)\n", ret);
		put_device(&mux_dev->dev);
		return ERR_PTR(ret);
	}

	return mux_dev;
}
EXPORT_SYMBOL_GPL(typec_mux_register);

/**
 * typec_mux_unregister - Unregister Multiplexer Switch
 * @mux_dev: USB Type-C Connector Multiplexer/DeMultiplexer
 *
 * Unregister mux that was registered with typec_mux_register().
 */
void typec_mux_unregister(struct typec_mux_dev *mux_dev)
{
	if (!IS_ERR_OR_NULL(mux_dev)) {
		WRITE_ONCE(mux_dev->removed, true);
		/* Callbacks may nest through forwarders; SRCU permits that nesting. */
		synchronize_srcu(&typec_mux_srcu);
		device_unregister(&mux_dev->dev);
	}
}
EXPORT_SYMBOL_GPL(typec_mux_unregister);

void typec_mux_set_drvdata(struct typec_mux_dev *mux_dev, void *data)
{
	dev_set_drvdata(&mux_dev->dev, data);
}
EXPORT_SYMBOL_GPL(typec_mux_set_drvdata);

void *typec_mux_get_drvdata(struct typec_mux_dev *mux_dev)
{
	return dev_get_drvdata(&mux_dev->dev);
}
EXPORT_SYMBOL_GPL(typec_mux_get_drvdata);

/* ------------------------------------------------------------------------- */

struct typec_thunderbolt_switch {
	struct typec_thunderbolt_switch_dev *sw_dev;
};

static int thunderbolt_switch_fwnode_match(struct device *dev,
					   const void *fwnode)
{
	if (!is_typec_thunderbolt_switch_dev(dev))
		return 0;

	return device_match_fwnode(dev, fwnode);
}

static void *typec_thunderbolt_switch_match(const struct fwnode_handle *fwnode,
					    const char *id, void *data)
{
	struct device *dev;

	if (id && !fwnode_property_present(fwnode, id))
		return NULL;

	dev = class_find_device(&typec_mux_class, NULL, fwnode,
				thunderbolt_switch_fwnode_match);

	return dev ? to_typec_thunderbolt_switch_dev(dev) :
		     ERR_PTR(-EPROBE_DEFER);
}

/**
 * fwnode_typec_thunderbolt_switch_get - Find USB4/Thunderbolt switch
 * @fwnode: The caller device node
 *
 * Finds a USB4/Thunderbolt switch linked with @fwnode. Returns a
 * reference to the switch on success, NULL if no matching connection
 * was found, or ERR_PTR(-EPROBE_DEFER) when a connection was found but
 * the switch has not been enumerated yet.
 */
struct typec_thunderbolt_switch *
fwnode_typec_thunderbolt_switch_get(struct fwnode_handle *fwnode)
{
	struct typec_thunderbolt_switch_dev *sw_dev;
	struct typec_thunderbolt_switch *sw;
	void *match;
	int ret = 0;

	match = fwnode_connection_find_match(fwnode, "thunderbolt-switch", NULL,
					     typec_thunderbolt_switch_match);
	if (!match)
		return NULL;
	if (IS_ERR(match))
		return ERR_CAST(match);
	sw_dev = match;

	sw = kzalloc_obj(*sw);
	if (!sw) {
		put_device(&sw_dev->dev);
		return ERR_PTR(-ENOMEM);
	}

	sw->sw_dev = sw_dev;
	scoped_guard(rwsem_read, &sw_dev->set_lock) {
		if (!sw_dev->set || !try_module_get(sw_dev->owner))
			ret = -ENODEV;
	}
	if (ret) {
		put_device(&sw_dev->dev);
		kfree(sw);
		return ERR_PTR(ret);
	}

	return sw;
}
EXPORT_SYMBOL_GPL(fwnode_typec_thunderbolt_switch_get);

/**
 * typec_thunderbolt_switch_put - Release handle to USB4/Thunderbolt switch
 * @sw: USB4/Thunderbolt switch
 *
 * Decrements the reference count of @sw and releases the handle.
 */
void typec_thunderbolt_switch_put(struct typec_thunderbolt_switch *sw)
{
	if (IS_ERR_OR_NULL(sw))
		return;

	module_put(sw->sw_dev->owner);
	put_device(&sw->sw_dev->dev);
	kfree(sw);
}
EXPORT_SYMBOL_GPL(typec_thunderbolt_switch_put);

/**
 * typec_thunderbolt_switch_is_live - Whether the switch provider is still bound
 * @sw: USB4/Thunderbolt switch
 *
 * A handle outlives the provider: once the provider unregisters its switch,
 * typec_thunderbolt_switch_set() fails with -ENODEV. A provider that binds
 * again registers a new switch, which must be looked up again.
 *
 * Return: %true if @sw is valid and its provider is still registered.
 */
bool typec_thunderbolt_switch_is_live(struct typec_thunderbolt_switch *sw)
{
	if (IS_ERR_OR_NULL(sw))
		return false;

	guard(rwsem_read)(&sw->sw_dev->set_lock);
	return sw->sw_dev->set;
}
EXPORT_SYMBOL_GPL(typec_thunderbolt_switch_is_live);

/**
 * typec_thunderbolt_switch_set - Forward cable details to USB4/Thunderbolt switch
 * @sw: USB4/Thunderbolt switch
 * @data: Cable state and details
 *
 * Called by the Type-C port driver to forward the cable details
 * out-of-band to the switch handler whenever a USB4/Thunderbolt
 * connection comes up or goes away.
 */
int typec_thunderbolt_switch_set(struct typec_thunderbolt_switch *sw,
				 const struct typec_thunderbolt_switch_data *data)
{
	if (IS_ERR_OR_NULL(sw))
		return 0;

	guard(rwsem_read)(&sw->sw_dev->set_lock);
	if (!sw->sw_dev->set)
		return -ENODEV;
	return sw->sw_dev->set(sw->sw_dev, data);
}
EXPORT_SYMBOL_GPL(typec_thunderbolt_switch_set);

/**
 * typec_thunderbolt_switch_register_notifier - Watch provider health changes
 * @sw: USB4/Thunderbolt switch held by the consumer
 * @nb: Atomic notifier, which must not sleep or call the switch's set method
 *
 * Notifications request fresh state validation in consumer work context.
 * Unregister the notifier before releasing @sw or its callback data.
 */
int typec_thunderbolt_switch_register_notifier(struct typec_thunderbolt_switch *sw,
					       struct notifier_block *nb)
{
	if (IS_ERR_OR_NULL(sw))
		return 0;

	return atomic_notifier_chain_register(&sw->sw_dev->notifiers, nb);
}
EXPORT_SYMBOL_GPL(typec_thunderbolt_switch_register_notifier);

/**
 * typec_thunderbolt_switch_unregister_notifier - Stop provider notifications
 * @sw: USB4/Thunderbolt switch held by the consumer
 * @nb: Previously registered notifier
 *
 * Drains callbacks. Call in process context, outside callback/worker locks,
 * before cancelling work queued by the callback.
 */
void typec_thunderbolt_switch_unregister_notifier(struct typec_thunderbolt_switch *sw,
						  struct notifier_block *nb)
{
	if (!IS_ERR_OR_NULL(sw))
		atomic_notifier_chain_unregister(&sw->sw_dev->notifiers, nb);
}
EXPORT_SYMBOL_GPL(typec_thunderbolt_switch_unregister_notifier);

/**
 * typec_thunderbolt_switch_notify - Request consumer revalidation
 * @sw: Registered provider switch
 *
 * May be called from interrupt context. No connection state is passed: a
 * delayed notification must validate the current connection, not reset it
 * using details from an earlier session. Stop notifications before unregister.
 */
void typec_thunderbolt_switch_notify(struct typec_thunderbolt_switch_dev *sw)
{
	atomic_notifier_call_chain(&sw->notifiers,
				   TYPEC_THUNDERBOLT_SWITCH_REVALIDATE, NULL);
}
EXPORT_SYMBOL_GPL(typec_thunderbolt_switch_notify);

/**
 * typec_thunderbolt_switch_notify_ready - Report completed activation
 * @sw: Registered provider switch
 *
 * Wake consumers waiting on a busy provider without requesting fault recovery.
 * As with revalidation notifications, drain producers before unregistering.
 */
void typec_thunderbolt_switch_notify_ready(struct typec_thunderbolt_switch_dev *sw)
{
	atomic_notifier_call_chain(&sw->notifiers,
				   TYPEC_THUNDERBOLT_SWITCH_READY, NULL);
}
EXPORT_SYMBOL_GPL(typec_thunderbolt_switch_notify_ready);

/**
 * typec_thunderbolt_switch_notify_restore - Request a cable read after sleep
 * @sw: Registered provider switch
 *
 * The provider powered itself down for system sleep and needs the current
 * cable state reapplied. Unlike a revalidation this reports no link fault.
 */
void typec_thunderbolt_switch_notify_restore(struct typec_thunderbolt_switch_dev *sw)
{
	atomic_notifier_call_chain(&sw->notifiers,
				   TYPEC_THUNDERBOLT_SWITCH_RESTORE, NULL);
}
EXPORT_SYMBOL_GPL(typec_thunderbolt_switch_notify_restore);

/* Type-C ports that probed before their USB4/Thunderbolt router */
static BLOCKING_NOTIFIER_HEAD(typec_thunderbolt_switch_watchers);

/**
 * typec_thunderbolt_switch_watch - Watch for USB4/Thunderbolt switch registration
 * @nb: Notifier called with TYPEC_THUNDERBOLT_SWITCH_ADDED and the new switch
 *
 * Lets a consumer that probed without its switch look it up again later. The
 * callback runs under the registering driver's locks, so it must only schedule
 * work. Look the switch up once more after registering the watcher: it may
 * have registered in between.
 */
int typec_thunderbolt_switch_watch(struct notifier_block *nb)
{
	return blocking_notifier_chain_register(&typec_thunderbolt_switch_watchers,
						nb);
}
EXPORT_SYMBOL_GPL(typec_thunderbolt_switch_watch);

/**
 * typec_thunderbolt_switch_unwatch - Stop watching for switch registration
 * @nb: Notifier registered with typec_thunderbolt_switch_watch()
 *
 * Waits for a callback that is already running to return.
 */
void typec_thunderbolt_switch_unwatch(struct notifier_block *nb)
{
	blocking_notifier_chain_unregister(&typec_thunderbolt_switch_watchers, nb);
}
EXPORT_SYMBOL_GPL(typec_thunderbolt_switch_unwatch);

static void typec_thunderbolt_switch_release(struct device *dev)
{
	kfree(to_typec_thunderbolt_switch_dev(dev));
}

const struct device_type typec_thunderbolt_switch_dev_type = {
	.name = "thunderbolt_switch",
	.release = typec_thunderbolt_switch_release,
};

/**
 * typec_thunderbolt_switch_register - Register USB4/Thunderbolt switch
 * @parent: Parent device
 * @desc: USB4/Thunderbolt switch description
 *
 * This function registers a handler for USB4/Thunderbolt mode switching
 * which Type-C port drivers use to forward the cable details once a
 * USB4/Thunderbolt connection comes up.
 */
struct typec_thunderbolt_switch_dev *
typec_thunderbolt_switch_register(struct device *parent,
				  const struct typec_thunderbolt_switch_desc *desc)
{
	struct typec_thunderbolt_switch_dev *sw_dev;
	int ret;

	if (!desc || !desc->set)
		return ERR_PTR(-EINVAL);

	sw_dev = kzalloc_obj(*sw_dev);
	if (!sw_dev)
		return ERR_PTR(-ENOMEM);

	init_rwsem(&sw_dev->set_lock);
	sw_dev->owner = parent->driver ? parent->driver->owner : NULL;
	sw_dev->set = desc->set;

	device_initialize(&sw_dev->dev);
	sw_dev->dev.parent = parent;
	sw_dev->dev.fwnode = desc->fwnode;
	sw_dev->dev.class = &typec_mux_class;
	sw_dev->dev.type = &typec_thunderbolt_switch_dev_type;
	ATOMIC_INIT_NOTIFIER_HEAD(&sw_dev->notifiers);
	sw_dev->dev.driver_data = desc->drvdata;
	ret = dev_set_name(&sw_dev->dev, "%s-thunderbolt-switch",
			   desc->name ? desc->name : dev_name(parent));
	if (ret) {
		put_device(&sw_dev->dev);
		return ERR_PTR(ret);
	}

	ret = device_add(&sw_dev->dev);
	if (ret) {
		dev_err(parent, "failed to register switch (%d)\n", ret);
		put_device(&sw_dev->dev);
		return ERR_PTR(ret);
	}

	blocking_notifier_call_chain(&typec_thunderbolt_switch_watchers,
				     TYPEC_THUNDERBOLT_SWITCH_ADDED, sw_dev);

	return sw_dev;
}
EXPORT_SYMBOL_GPL(typec_thunderbolt_switch_register);

/**
 * typec_thunderbolt_switch_unregister - Unregister USB4/Thunderbolt switch
 * @sw_dev: USB4/Thunderbolt switch
 *
 * Unregister switch that was registered with
 * typec_thunderbolt_switch_register().
 */
void typec_thunderbolt_switch_unregister(struct typec_thunderbolt_switch_dev *sw_dev)
{
	if (IS_ERR_OR_NULL(sw_dev))
		return;

	/* Drain callbacks before the parent releases its private data. */
	scoped_guard(rwsem_write, &sw_dev->set_lock)
		sw_dev->set = NULL;
	device_unregister(&sw_dev->dev);
}
EXPORT_SYMBOL_GPL(typec_thunderbolt_switch_unregister);

/**
 * typec_thunderbolt_switch_set_drvdata - Assign private data to the switch
 * @sw_dev: USB4/Thunderbolt switch
 * @data: Private data
 */
void typec_thunderbolt_switch_set_drvdata(struct typec_thunderbolt_switch_dev *sw_dev,
					  void *data)
{
	dev_set_drvdata(&sw_dev->dev, data);
}
EXPORT_SYMBOL_GPL(typec_thunderbolt_switch_set_drvdata);

/**
 * typec_thunderbolt_switch_get_drvdata - Get the private data from the switch
 * @sw_dev: USB4/Thunderbolt switch
 *
 * Returns the private data previously assigned with
 * typec_thunderbolt_switch_set_drvdata().
 */
void *typec_thunderbolt_switch_get_drvdata(struct typec_thunderbolt_switch_dev *sw_dev)
{
	return dev_get_drvdata(&sw_dev->dev);
}
EXPORT_SYMBOL_GPL(typec_thunderbolt_switch_get_drvdata);

const struct class typec_mux_class = {
	.name = "typec_mux",
};
