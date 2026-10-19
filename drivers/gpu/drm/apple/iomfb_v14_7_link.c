// SPDX-License-Identifier: GPL-2.0-only OR MIT

#include <linux/delay.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/jiffies.h>
#include <linux/slab.h>
#include <linux/soc/apple/rtkit.h>
#include <linux/unaligned.h>
#include <linux/workqueue.h>

#include "iomfb_v14_7_link.h"

#define DCP_V14_LINK_ALIGNMENT	64
#define DCP_V14_LINK_READY_MSG	((64ULL << 16) | 0x101)
#define DCP_V14_READY_TIMEOUT	msecs_to_jiffies(3000)
#define DCP_V14_CALL_TIMEOUT	(5 * HZ)
#define DCP_V14_TAG_D589	DCP_V14_TAG('D', 589)
#define DCP_V14_D589_IN_SIZE	0x6f0
/* A407 swap_submit: its reply flags a dropped swap at +0, its status is at +5. */
#define DCP_V14_TAG_A407	DCP_V14_TAG('A', 407)
#define DCP_V14_A407_OUT_SIZE	12

enum dcp_v14_event_kind {
	DCP_V14_EVENT_REPLY = 1,
	DCP_V14_EVENT_CALLBACK = 2,
};

struct dcp_v14_event {
	struct list_head list;
	u32 kind;
	u32 size;
	u64 message;
	u8 data[];
};

/* Called with the lock held. */
static void link_fail(struct dcp_v14_link *link)
{
	struct dcp_v14_event *e, *next;

	if (!link->failed)
		dev_err(link->dev, "DCP link stopped; firmware buffers are kept until reboot\n");
	link->failed = true;
	/* Nothing dequeues events from a failed link. */
	list_for_each_entry_safe(e, next, &link->events, list) {
		list_del(&e->list);
		kfree(e);
	}
	link->event_count = 0;
	wake_up(&link->wait);
}

/* Called with the lock held. */
static void link_event(struct dcp_v14_link *link, u32 kind, u64 message,
		       const void *data, u32 size)
{
	struct dcp_v14_event *e;

	if (link->event_count >= DCP_V14_LINK_MAX_EVENTS ||
	    size > APPLE_DCP_LINK_STREAM_BUFFER_SIZE) {
		link_fail(link);
		return;
	}
	e = kmalloc(struct_size(e, data, size), GFP_KERNEL);
	if (!e) {
		link_fail(link);
		return;
	}
	e->kind = kind;
	e->size = size;
	e->message = message;
	if (size)
		memcpy(e->data, data, size);
	list_add_tail(&e->list, &link->events);
	link->event_count++;
	wake_up(&link->wait);
	if (link->idle_work)
		queue_work(system_unbound_wq, link->idle_work);
}

void dcp_v14_link_init(struct dcp_v14_link *link, struct device *dev,
		       struct apple_rtkit *rtk)
{
	link->dev = dev;
	link->rtk = rtk;
	mutex_init(&link->lock);
	init_waitqueue_head(&link->wait);
	INIT_LIST_HEAD(&link->events);
	init_completion(&link->ready);
}

void dcp_v14_link_fail(struct dcp_v14_link *link)
{
	mutex_lock(&link->lock);
	link_fail(link);
	mutex_unlock(&link->lock);
	complete(&link->ready);
}

bool dcp_v14_link_failed(struct dcp_v14_link *link)
{
	return READ_ONCE(link->failed);
}

void dcp_v14_link_set_idle_work(struct dcp_v14_link *link, struct work_struct *work)
{
	mutex_lock(&link->lock);
	link->idle_work = work;
	if (link->event_count)
		queue_work(system_unbound_wq, work);
	mutex_unlock(&link->lock);
}

/* Host-driven start: endpoint, INIT with the RPC memory, then READY. */
int dcp_v14_link_start(struct dcp_v14_link *link)
{
	struct apple_dcp_link_init_desc *desc;
	u64 message;
	int ret;

	link->rpc = dma_alloc_coherent(link->dev, APPLE_DCP_LINK_RPC_MEMORY_SIZE,
				       &link->rpc_dva, GFP_KERNEL);
	if (!link->rpc)
		return -ENOMEM;
	memset(link->rpc, 0, APPLE_DCP_LINK_RPC_MEMORY_SIZE);
	desc = link->rpc;
	apple_dcp_link_init_desc_encode(desc, 0, false);
	/* This firmware reads the version field as the RPC alignment. */
	desc->version = cpu_to_le32(DCP_V14_LINK_ALIGNMENT);
	ret = apple_dcp_link_init_message(link->rpc_dva, &message);
	if (ret)
		return ret;

	link->ready_result = -ETIMEDOUT;
	ret = apple_rtkit_start_ep(link->rtk, APPLE_DCP_LINK_ENDPOINT);
	if (ret)
		return ret;
	dma_wmb();
	ret = apple_rtkit_send_message(link->rtk, APPLE_DCP_LINK_ENDPOINT, message,
				       NULL, false);
	if (ret)
		return ret;
	if (!wait_for_completion_timeout(&link->ready, DCP_V14_READY_TIMEOUT))
		return -ETIMEDOUT;

	mutex_lock(&link->lock);
	ret = link->failed ? -EIO : link->ready_result;
	if (!ret)
		link->enabled = true;
	mutex_unlock(&link->lock);
	if (!ret)
		dev_info(link->dev, "DCPLink ready, RPC alignment %d\n", DCP_V14_LINK_ALIGNMENT);
	return ret;
}

/* RTKit receive worker: check, record and queue; never blocks on a call. */
void dcp_v14_link_receive(struct dcp_v14_link *link, u64 message)
{
	struct apple_dcp_link_stream_layout layout;
	struct apple_dcp_link_rpc_view view;
	u8 stream = apple_dcp_link_message_stream(message);
	bool remote = apple_dcp_link_message_is_remote(message);
	bool nested;

	mutex_lock(&link->lock);
	if (link->failed)
		goto out;

	if (!link->enabled) {
		/* Firmware-side INIT needs no answer; READY ends the handshake. */
		if ((message & 3) == APPLE_DCP_LINK_MSG_FIRMWARE_INIT)
			goto out;
		link->ready_result = message == DCP_V14_LINK_READY_MSG ? 0 : -EPROTO;
		if (link->ready_result)
			dev_err(link->dev, "unexpected DCPLink ready message %#llx\n", message);
		mutex_unlock(&link->lock);
		complete(&link->ready);
		return;
	}

	if ((message & 3) != APPLE_DCP_LINK_MSG_RPC) {
		dev_err(link->dev, "unexpected DCPLink message %#llx\n", message);
		goto error;
	}
	dma_rmb();

	if (message & BIT_ULL(6)) {
		/* The reply to the innermost call, echoed in place. */
		nested = link->calls[1].active;
		if (stream || !link->calls[nested].active ||
		    link->calls[nested].remote != remote ||
		    apple_dcp_link_stream_layout(APPLE_DCP_LINK_SIDE_AP, 0, remote, &layout))
			goto error;
		if (memcmp(link->rpc + layout.local_offset + link->calls[nested].offset,
			   &link->calls[nested].header, sizeof(link->calls[nested].header)))
			goto error;
		link_event(link, DCP_V14_EVENT_REPLY, message,
			   link->rpc + layout.local_offset + link->calls[nested].offset,
			   link->calls[nested].size);
		link->calls[nested].active = false;
		goto out;
	}

	if (link->callback_count >= DCP_V14_LINK_MAX_CALLBACKS ||
	    apple_dcp_link_stream_layout(APPLE_DCP_LINK_SIDE_AP, stream, remote, &layout) ||
	    apple_dcp_link_rpc_decode(message, link->rpc + layout.remote_offset,
				      layout.capacity, &view))
		goto error;
	if ((view.call >> 24) != 'D') {
		dev_err(link->dev, "DCPLink callback with tag %#x\n", view.call);
		goto error;
	}
	link->callbacks[link->callback_count].message = message;
	link->callbacks[link->callback_count].output = view.output;
	link->callbacks[link->callback_count].size = view.output_size;
	link->callbacks[link->callback_count].call = view.call;
	link->callback_count++;
	dev_dbg(link->dev, "callback %#x in %u out %u\n",
		view.call, view.input_size, view.output_size);
	link_event(link, DCP_V14_EVENT_CALLBACK, message,
		   link->rpc + layout.remote_offset + view.offset, view.total_size);
	goto out;
error:
	link_fail(link);
out:
	mutex_unlock(&link->lock);
}

/*
 * Places a call packet (header, input, zeroed output) and sends it. A nested
 * call goes out on the stream of @parent, the callback its caller handles.
 */
static int link_submit(struct dcp_v14_link *link, const struct apple_dcp_link_rpc_header *rpc,
		       u32 size, bool nested, u64 parent,
		       dcp_v14_admit_fn admit, void *admit_cookie, bool *vetoed)
{
	struct apple_dcp_link_stream_layout layout;
	u32 total, offset = 0, input_size, output_size;
	bool remote = false;
	u64 message;
	int ret;

	mutex_lock(&link->lock);
	if (!link->enabled || link->failed) {
		ret = -EIO;
		goto out;
	}
	if (link->calls[nested].active || (nested && !link->callback_count) ||
	    (!nested && link->callback_count)) {
		ret = -EBUSY;
		goto out;
	}
	if (nested) {
		u64 callback = parent;

		if (apple_dcp_link_message_stream(callback)) {
			ret = -EOPNOTSUPP;
			goto out;
		}
		/* A nested call goes to the callback's side of stream 0, after the outer call. */
		remote = apple_dcp_link_message_is_remote(callback);
		if (link->calls[0].active && link->calls[0].remote == remote)
			offset = ALIGN(link->calls[0].size, DCP_V14_LINK_ALIGNMENT);
	}
	input_size = le32_to_cpu(rpc->input_size);
	output_size = le32_to_cpu(rpc->output_size);
	ret = apple_dcp_link_rpc_payload_size(input_size, output_size, &total);
	if (ret || total != size) {
		ret = -EMSGSIZE;
		goto out;
	}
	ret = apple_dcp_link_stream_layout(APPLE_DCP_LINK_SIDE_AP, 0, remote, &layout);
	if (ret)
		goto out;
	if (offset + size > layout.capacity) {
		ret = -EMSGSIZE;
		goto out;
	}
	/* No callback can run or enter the queue between this check and send. */
	if (admit && !admit(admit_cookie)) {
		*vetoed = true;
		ret = -EAGAIN;
		goto out;
	}
	memcpy(link->rpc + layout.local_offset + offset, rpc, size);
	memset(link->rpc + layout.local_offset + offset + sizeof(*rpc) + input_size, 0,
	       output_size);
	link->calls[nested].header = *rpc;
	link->calls[nested].size = size;
	link->calls[nested].offset = offset;
	link->calls[nested].remote = remote;
	link->calls[nested].active = true;
	apple_dcp_link_rpc_message(offset, size, false, &message);
	apple_dcp_link_stream_message(message, 0, remote, &message);
	dma_wmb();
	ret = apple_rtkit_send_message(link->rtk, APPLE_DCP_LINK_ENDPOINT, message, NULL, false);
	if (ret)
		link_fail(link);
out:
	mutex_unlock(&link->lock);
	return ret;
}

/*
 * Writes a callback's output in place and acknowledges it. Usually that is
 * the innermost callback; one from another firmware thread may still be
 * waiting below it, and is answered in its turn.
 */
static int link_reply(struct dcp_v14_link *link, u64 message, const void *output, u32 size)
{
	unsigned int i;
	int ret = -EINVAL;

	mutex_lock(&link->lock);
	if (!link->enabled || link->failed) {
		ret = -EIO;
		goto out;
	}
	for (i = link->callback_count; i > 0; i--)
		if (link->callbacks[i - 1].message == message)
			break;
	if (!i || size != link->callbacks[i - 1].size) {
		dev_err(link->dev, "reply to callback %#llx of %u bytes matches none of %u waiting\n",
			message, size, link->callback_count);
		goto out;
	}
	i--;
	memcpy(link->callbacks[i].output, output, size);
	dma_wmb();
	ret = apple_rtkit_send_message(link->rtk, APPLE_DCP_LINK_ENDPOINT,
				       apple_dcp_link_rpc_reply(message), NULL, false);
	if (ret) {
		link_fail(link);
	} else {
		link->callback_count--;
		memmove(&link->callbacks[i], &link->callbacks[i + 1],
			(link->callback_count - i) * sizeof(link->callbacks[0]));
	}
out:
	mutex_unlock(&link->lock);
	return ret;
}

/* Uninterruptible: an abandoned call would leave the firmware mid-call. */
static struct dcp_v14_event *link_next_event(struct dcp_v14_link *link, unsigned long timeout)
{
	struct dcp_v14_event *e;

	if (!wait_event_timeout(link->wait,
				READ_ONCE(link->event_count) || READ_ONCE(link->failed),
				timeout))
		return ERR_PTR(-ETIMEDOUT);
	mutex_lock(&link->lock);
	if (link->failed || list_empty(&link->events)) {
		mutex_unlock(&link->lock);
		return ERR_PTR(-EIO);
	}
	e = list_first_entry(&link->events, struct dcp_v14_event, list);
	list_del(&e->list);
	link->event_count--;
	mutex_unlock(&link->lock);
	return e;
}

static int link_callback(struct dcp_v14_link *link, struct dcp_v14_event *e,
			 dcp_v14_callback_fn callback, void *cookie, u32 *completed)
{
	struct apple_dcp_link_rpc_header *h = (void *)e->data;
	u32 in, out, tag;
	int ret;

	if (e->kind != DCP_V14_EVENT_CALLBACK || e->size < sizeof(*h) || !callback)
		return -EPROTO;
	in = le32_to_cpu(h->input_size);
	out = le32_to_cpu(h->output_size);
	tag = le32_to_cpu(h->call);
	if (in > e->size - sizeof(*h) || out != e->size - sizeof(*h) - in)
		return -EPROTO;
	memset(e->data + sizeof(*h) + in, 0, out);
	if (link->callback_depth >= DCP_V14_LINK_MAX_CALLBACKS)
		return -EPROTO;
	link->handling[link->callback_depth++] = e->message;
	ret = callback(cookie, tag, h + 1, in, e->data + sizeof(*h) + in, out);
	link->callback_depth--;
	if (ret) {
		dev_err(link->dev, "callback %#x (%u/%u bytes) failed: %d\n", tag, in, out, ret);
		return ret;
	}
	if (tag == DCP_V14_TAG_D589 && in == DCP_V14_D589_IN_SIZE) {
		u32 id = get_unaligned_le32(h + 1);

		/*
		 * The firmware dropped this swap and said so in its A407
		 * reply; a completion for it is not expected, but is not an
		 * error either.
		 */
		if (id && id == link->dropped_completion)
			link->dropped_completion = 0;
		else if (completed)
			*completed = id;
	}
	return link_reply(link, e->message, e->data + sizeof(*h) + in, out);
}

int dcp_v14_link_pump(struct dcp_v14_link *link, unsigned long timeout,
		      dcp_v14_callback_fn callback, void *cookie)
{
	struct dcp_v14_event *e = link_next_event(link, timeout);
	int ret;

	if (IS_ERR(e))
		return PTR_ERR(e);
	ret = link_callback(link, e, callback, cookie, NULL);
	kfree(e);
	return ret;
}

/*
 * Calls @tag and handles the callbacks that arrive until its reply, and, for
 * a non-zero @completion_id, until the swap-complete callback for that swap.
 * A call that fails once submitted, including by timing out, stops the link.
 * Must not be called concurrently with itself or dcp_v14_link_pump(), except
 * from inside a callback handler.
 */
int dcp_v14_link_call_guarded(struct dcp_v14_link *link, u32 tag, const void *input,
		      u32 input_size, void *output, u32 output_size,
		      u32 completion_id, dcp_v14_callback_fn callback, void *cookie,
		      dcp_v14_admit_fn admit, void *admit_cookie, bool *vetoed)
{
	struct apple_dcp_link_rpc_header *packet;
	unsigned long deadline = jiffies + DCP_V14_CALL_TIMEOUT;
	u64 expected = APPLE_DCP_LINK_MSG_RPC_REPLY, parent = 0;
	u32 size, completed = 0;
	bool nested, replied = false;
	int ret;

	*vetoed = false;
	ret = apple_dcp_link_rpc_payload_size(input_size, output_size, &size);
	if (ret || size > APPLE_DCP_LINK_STREAM_BUFFER_SIZE)
		return -EMSGSIZE;
	packet = kzalloc(size, GFP_KERNEL);
	if (!packet)
		return -ENOMEM;
	apple_dcp_link_rpc_header_encode(packet, tag, input_size, output_size);
	if (input_size)
		memcpy(packet + 1, input, input_size);

retry:
	/*
	 * Handle queued notifications before an outer call. A queued
	 * callback is not an enclosing call until its handler runs.
	 */
	if (!link->callback_depth) {
		while (READ_ONCE(link->event_count)) {
			ret = dcp_v14_link_pump(link, 0, callback, cookie);
			if (ret) {
				kfree(packet);
				/* The caller gives up on the session: stop queueing. */
				mutex_lock(&link->lock);
				link_fail(link);
				mutex_unlock(&link->lock);
				return ret;
			}
		}
	}
	mutex_lock(&link->lock);
	nested = link->callback_depth != 0;
	if (nested && !link->callback_count) {
		link_fail(link);
		mutex_unlock(&link->lock);
		kfree(packet);
		return -EPROTO;
	}
	/* The reply to a nested call carries the side of the callback handled. */
	if (nested) {
		parent = link->handling[link->callback_depth - 1];
		expected |= parent & APPLE_DCP_LINK_MSG_REMOTE;
	}
	mutex_unlock(&link->lock);
	ret = link_submit(link, packet, size, nested, parent, admit, admit_cookie, vetoed);
	/* A callback arrived between the drain and the submit. */
	if (ret == -EBUSY && !nested && time_before(jiffies, deadline))
		goto retry;
	kfree(packet);
	if (ret)
		return ret;

	while (time_before(jiffies, deadline)) {
		struct dcp_v14_event *e = link_next_event(link, max_t(long, deadline - jiffies, 0));
		struct apple_dcp_link_rpc_header *h;

		if (IS_ERR(e)) {
			ret = PTR_ERR(e);
			goto abandon;
		}
		h = (void *)e->data;
		if (e->kind == DCP_V14_EVENT_REPLY) {
			if (replied || e->size != size || e->message != expected ||
			    le32_to_cpu(h->call) != tag ||
			    le32_to_cpu(h->input_size) != input_size ||
			    le32_to_cpu(h->output_size) != output_size) {
				ret = -EPROTO;
			} else {
				if (output_size)
					memcpy(output, e->data + sizeof(*h) + input_size,
					       output_size);
				replied = true;
				/*
				 * A swap the firmware reports dropped (the flag at
				 * +0 with status 0) is never completed by D589:
				 * the call ends with its reply.
				 */
				if (completion_id && tag == DCP_V14_TAG_A407 &&
				    output_size == DCP_V14_A407_OUT_SIZE &&
				    ((u8 *)output)[0] &&
				    !get_unaligned_le32((u8 *)output + 5)) {
					mutex_lock(&link->lock);
					link->dropped_completion = completion_id;
					mutex_unlock(&link->lock);
					kfree(e);
					return 0;
				}
			}
		} else {
			ret = link_callback(link, e, callback, cookie, &completed);
			if (!ret && completion_id && completed && completed != completion_id)
				ret = -EPROTO;
		}
		kfree(e);
		if (ret)
			goto abandon;
		if (replied && (!completion_id || completed == completion_id))
			return 0;
	}
	ret = -ETIMEDOUT;
abandon:
	/* The firmware may still answer: queue nothing for a call nobody waits for. */
	mutex_lock(&link->lock);
	link->calls[nested].active = false;
	link_fail(link);
	mutex_unlock(&link->lock);
	return ret;
}

int dcp_v14_link_call(struct dcp_v14_link *link, u32 tag, const void *input,
		      u32 input_size, void *output, u32 output_size,
		      u32 completion_id, dcp_v14_callback_fn callback, void *cookie)
{
	bool vetoed;

	return dcp_v14_link_call_guarded(link, tag, input, input_size, output,
		output_size, completion_id, callback, cookie, NULL, NULL, &vetoed);
}
