// SPDX-License-Identifier: GPL-2.0-only

#include <linux/delay.h>
#include <linux/etherdevice.h>
#include <linux/dma-mapping.h>
#include <linux/iopoll.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/apple-centauri.h>
#include <linux/of.h>
#include <linux/pci.h>
#include <linux/property.h>
#include <linux/random.h>
#include <linux/sizes.h>
#include <linux/suspend.h>
#include <linux/unaligned.h>
#include <linux/workqueue.h>
#include <net/bluetooth/bluetooth.h>
#include <net/bluetooth/hci_core.h>

#define CEN_ARENA_SIZE SZ_1M
#define CEN_MTR 0x2000
#define CEN_MCR 0x2200
#define CEN_MESSAGES 0x4000
#define CEN_RX 0x8000
#define CEN_TX 0xa000
#define CEN_FOOTER 260
#define CEN_STRIDE (16 + CEN_FOOTER)
#define CEN_RX_SLOTS 17
#define CEN_TX_SLOTS 5
#define CEN_LE_RX 0xc000
#define CEN_LE_TX 0xe000
#define CEN_LE_FOOTER 256
#define CEN_LE_STRIDE (16 + CEN_LE_FOOTER)
#define CEN_LE_SLOTS 17
#define CEN_MESSAGE_SLOTS 16
#define CEN_AUX_BASE 0x20000
#define CEN_CAPTURE_SIZE SZ_1M

#define CEN_OP_READ_PROP_FEATURES 0xffa3
#define CEN_OP_CREATE_CONN 0xff7b
#define CEN_OP_FLUSH_DUPLICATES 0xff3f
#define CEN_PROP_CREATE_CONN BIT(9)

struct cen_aux_spec {
	u16 id, slots, footer;
	bool receive;
};

static const struct cen_aux_spec cen_aux_specs[] = {
	{ 2, 9, 2592, false },	{ 3, 9, 724, false },	 { 5, 9, 260, false },
	{ 6, 5, 1024, false },	{ 8, 9, 2592, true },	 { 9, 9, 724, true },
	{ 11, 9, 268, true },	{ 12, 128, 1024, true }, { 13, 17, 260, true },
	{ 14, 9, 1304, false }, { 15, 9, 1304, true },
};

struct cen_aux_state {
	u32 offset;
	u16 head, post;
	bool opened;
};

struct cen_bt {
	struct pci_dev *pdev;
	struct hci_dev *hdev;
	void __iomem *bar;
	u8 *arena;
	dma_addr_t dma;
	struct mutex lock;
	struct work_struct io_work;
	unsigned int irq_count;
	bool irq_vectors;
	struct sk_buff_head txq;
	struct sk_buff_head acl_txq, le_txq;
	u16 acl_tx_head, le_tx_head;
	u16 message_head, message_tag, tx_head, rx_head, rx_post;
	u16 le_rx_head, le_rx_post;
	struct cen_aux_state aux[ARRAY_SIZE(cen_aux_specs)];
	bool ipc_requested, running;
	bool hci_registered, quiescing, host_quiesced;
	bool sleep_registered;
	/* Host quiesced for sleep; arena ownership is tracked independently. */
	bool asleep;
	bool transport_paused, transport_sleep_sent;
	int suspend_error;
	u8 native_startup_steps;
	bool connection_command_supported, connection_status_pending;
};

static u16 cen_tail(struct cen_bt *bt, u16 ring)
{
	return le16_to_cpu(READ_ONCE(*(__le16 *)(bt->arena + 0x7a + ring * 2)));
}

static void cen_post(struct cen_bt *bt, u16 ring, u16 value)
{
	put_unaligned_le16(value, bt->arena + 0xa8 + ring * 2);
	dma_wmb();
	writel(value, bt->bar + 0x1000 + ring * 4);
}

static int cen_message(struct cen_bt *bt, const void *data, size_t length)
{
	u16 head = bt->message_head, tail,
	    next = (head + 1) % CEN_MESSAGE_SLOTS;
	u16 tag = bt->message_tag + 1;
	u8 *descriptor = bt->arena + CEN_MTR + head * 16;
	u8 *completion = bt->arena + CEN_MCR + head * 32;
	int ret;

	if (head >= CEN_MESSAGE_SLOTS || !length || length > 512)
		return -EINVAL;
	memset(descriptor, 0, 16);
	memset(completion, 0, 32);
	memcpy(bt->arena + CEN_MESSAGES + head * 512, data, length);
	put_unaligned_le32(1 | (length << 8), descriptor);
	put_unaligned_le64(bt->dma + CEN_MESSAGES + head * 512, descriptor + 4);
	put_unaligned_le16(tag, descriptor + 12);
	cen_post(bt, 0, next);
	ret = read_poll_timeout(cen_tail, tail, tail == next, 100, 1500000,
				false, bt, 0);
	if (ret)
		return ret;

	ret = read_poll_timeout(le16_to_cpu, tail, tail == next, 100, 1500000,
				false,
				READ_ONCE(*(__le16 *)(bt->arena + 0x78)));
	if (ret)
		return ret;
	dma_rmb();
	if (get_unaligned_le16(completion + 4) != tag ||
	    get_unaligned_le16(completion + 6) != length ||
	    get_unaligned_le32(completion + 8))
		return -EPROTO;
	bt->message_head = next;
	bt->message_tag = tag;
	put_unaligned_le16(bt->message_head, bt->arena + 0xa6);
	dma_wmb();
	return 0;
}

static int cen_open_ring(struct cen_bt *bt, u16 id, u16 slots, u32 offset,
			 u16 footer)
{
	u8 message[0x34] = { 1 };
	u32 words = footer / 4, shift = 0;

	if (!words || footer % 4)
		return -EINVAL;
	if (words > 255)
		shift = min_t(u32, __ffs(words), 4);
	if ((words >> shift) > 255)
		return -EINVAL;
	message[2] = words >> shift;
	message[3] = shift << 4;
	put_unaligned_le16(id, message + 4);
	put_unaligned_le64(bt->dma + offset, message + 8);
	put_unaligned_le64(~0ULL, message + 0x10);
	put_unaligned_le16(slots, message + 0x18);
	put_unaligned_le16(0xffff, message + 0x1a);
	put_unaligned_le16(id, message + 0x1c);
	put_unaligned_le16(0x40, message + 0x1e);
	return cen_message(bt, message, sizeof(message));
}

static void cen_rx_buffer(struct cen_bt *bt, u16 slot)
{
	u8 *descriptor = bt->arena + CEN_RX + slot * CEN_STRIDE;

	memset(descriptor, 0, CEN_STRIDE);
	put_unaligned_le32(1 | (CEN_FOOTER << 8), descriptor);
	put_unaligned_le16(slot + 1, descriptor + 12);
}

static void cen_le_rx_buffer(struct cen_bt *bt, u16 slot)
{
	u8 *descriptor = bt->arena + CEN_LE_RX + slot * CEN_LE_STRIDE;

	memset(descriptor, 0, CEN_LE_STRIDE);
	put_unaligned_le32(1 | (CEN_LE_FOOTER << 8), descriptor);
	put_unaligned_le16(slot + 1, descriptor + 12);
}

static int cen_receive_acl(struct cen_bt *bt, const u8 *data, u32 length)
{
	struct sk_buff *skb;
	u32 packet_length, pos;

	if (length < HCI_ACL_HDR_SIZE)
		return -EPROTO;
	packet_length = HCI_ACL_HDR_SIZE + get_unaligned_le16(data + 2);
	if (packet_length > length)
		return -EPROTO;

	for (pos = packet_length; pos < length;) {
		if (!data[pos])
			break;
		if (data[pos] == 1)
			pos++;
		else if (data[pos] == 2 && length - pos >= 9)
			pos += 9;
		else
			return -EPROTO;
	}
	skb = bt_skb_alloc(packet_length, GFP_KERNEL);
	if (!skb) {
		bt->hdev->stat.err_rx++;
		return 0;
	}
	skb_put_data(skb, data, packet_length);
	hci_skb_pkt_type(skb) = HCI_ACLDATA_PKT;
	bt->hdev->stat.byte_rx += packet_length;
	hci_recv_frame(bt->hdev, skb);
	return 0;
}

static void cen_aux_rx_buffer(struct cen_bt *bt, unsigned int index, u16 slot)
{
	const struct cen_aux_spec *spec = &cen_aux_specs[index];
	u8 *descriptor =
		bt->arena + bt->aux[index].offset + slot * (16 + spec->footer);

	memset(descriptor, 0, 16 + spec->footer);
	put_unaligned_le32(1 | (spec->footer << 8), descriptor);
	put_unaligned_le16(slot + 1, descriptor + 12);
}

static bool cen_aux_required(u16 id)
{
	return id == 2 || id == 8;
}

static int cen_aux_start(struct cen_bt *bt)
{
	u32 offset = CEN_AUX_BASE;
	unsigned int i, slot;
	int ret;

	for (i = 0; i < ARRAY_SIZE(cen_aux_specs); i++) {
		const struct cen_aux_spec *spec = &cen_aux_specs[i];
		struct cen_aux_state *state = &bt->aux[i];
		u32 length = spec->slots * (16U + spec->footer);

		if (offset > CEN_ARENA_SIZE || length > CEN_ARENA_SIZE - offset)
			return -ENOSPC;
		state->offset = offset;
		offset = ALIGN(offset + length, SZ_4K);
		if (!cen_aux_required(spec->id))
			continue;
		ret = cen_open_ring(bt, spec->id, spec->slots, state->offset,
				    spec->footer);
		if (ret) {
			dev_err(&bt->pdev->dev,
				"Auxiliary ring %u open failed: %d\n", spec->id,
				ret);
			return ret;
		}
		state->opened = true;
		if (!spec->receive)
			continue;
		for (slot = 0; slot < spec->slots - 1; slot++)
			cen_aux_rx_buffer(bt, i, slot);
		state->post = spec->slots - 1;
		cen_post(bt, spec->id, state->post);
	}
	return 0;
}

static int cen_aux_drain(struct cen_bt *bt, bool repost)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(cen_aux_specs); i++) {
		const struct cen_aux_spec *spec = &cen_aux_specs[i];
		struct cen_aux_state *state = &bt->aux[i];
		u16 tail;
		int budget = spec->slots;

		if (!state->opened || !spec->receive)
			continue;
		tail = cen_tail(bt, spec->id);
		if (tail >= spec->slots)
			return -EPROTO;
		while (state->head != tail && budget--) {
			u8 *descriptor = bt->arena + state->offset +
					 state->head * (16 + spec->footer);
			u32 length;

			dma_rmb();
			length = get_unaligned_le32(descriptor) >> 8;
			if (length > spec->footer)
				return -EPROTO;
			if (repost && spec->id == 8 &&
			    cen_receive_acl(bt, descriptor + 16, length))
				return -EPROTO;
			state->head = (state->head + 1) % spec->slots;
			if (repost) {
				cen_aux_rx_buffer(bt, i, state->post);
				state->post = (state->post + 1) % spec->slots;
				cen_post(bt, spec->id, state->post);
			}
		}
	}
	return 0;
}

static int cen_start(struct cen_bt *bt)
{
	static const u16 indices[] = { 0x78, 0x7a, 0xa6, 0xa8 };
	u8 config[4] = { 9, 0, 0xff, 0xff };
	u32 status;
	int i, ret;

	put_unaligned_le16(0x300, bt->arena);
	put_unaligned_le16(0x68, bt->arena + 2);
	put_unaligned_le64(bt->dma + 0x68, bt->arena + 8);
	for (i = 0; i < ARRAY_SIZE(indices); i++)
		put_unaligned_le64(bt->dma + indices[i],
				   bt->arena + 0x10 + i * 8);
	put_unaligned_le16(1, bt->arena + 0x30);
	put_unaligned_le16(22, bt->arena + 0x32);
	put_unaligned_le64(bt->dma + CEN_MCR, bt->arena + 0x34);
	put_unaligned_le64(bt->dma + CEN_MTR, bt->arena + 0x3c);
	put_unaligned_le16(16, bt->arena + 0x44);
	put_unaligned_le16(16, bt->arena + 0x46);

	bt->arena[0x53] = 4;
	put_unaligned_le64(bt->dma + 0x10000, bt->arena + 0x58);
	pci_set_master(bt->pdev);
	/* Account for publication before the first address reaches the device. */
	bt->ipc_requested = true;
	writel(lower_32_bits(bt->dma), bt->bar + 0xc);
	writel(upper_32_bits(bt->dma), bt->bar + 0x10);
	writel(CEN_ARENA_SIZE, bt->bar + 0x14);
	writel(1, bt->bar + 0x107c);
	ret = readl_poll_timeout(bt->bar, status, status == 1, 100, 2000000);
	if (ret)
		return ret;
	dma_wmb();
	writel(lower_32_bits(bt->dma), bt->bar + 4);
	writel(upper_32_bits(bt->dma), bt->bar + 8);
	writel(2, bt->bar + 0x107c);
	ret = readl_poll_timeout(bt->bar, status, status == 2, 100, 2000000);
	if (ret)
		return ret;
	ret = cen_message(bt, config, sizeof(config));
	if (!ret)
		ret = cen_open_ring(bt, 7, CEN_RX_SLOTS, CEN_RX, CEN_FOOTER);
	if (!ret)
		ret = cen_open_ring(bt, 1, CEN_TX_SLOTS, CEN_TX, CEN_FOOTER);
	if (!ret)
		ret = cen_open_ring(bt, 10, CEN_LE_SLOTS, CEN_LE_RX,
				    CEN_LE_FOOTER);
	if (!ret)
		ret = cen_open_ring(bt, 4, CEN_LE_SLOTS, CEN_LE_TX,
				    CEN_LE_FOOTER);
	if (ret)
		return ret;
	for (i = 0; i < CEN_RX_SLOTS - 1; i++)
		cen_rx_buffer(bt, i);
	bt->rx_post = CEN_RX_SLOTS - 1;
	cen_post(bt, 7, bt->rx_post);
	{
		for (i = 0; i < CEN_LE_SLOTS - 1; i++)
			cen_le_rx_buffer(bt, i);
		bt->le_rx_post = CEN_LE_SLOTS - 1;
		cen_post(bt, 10, bt->le_rx_post);
	}
	return cen_aux_start(bt);
}

static int cen_transmit(struct cen_bt *bt, struct sk_buff_head *queue, u16 ring,
			u16 slots, u32 offset, u16 footer, u16 *head)
{
	struct sk_buff *skb;
	u16 tail, next;
	u8 *descriptor;
	u32 wire_length;

	while (!skb_queue_empty(queue)) {
		tail = cen_tail(bt, ring);
		if (tail >= slots)
			return -EPROTO;
		next = (*head + 1) % slots;
		if (next == tail)
			break;
		dma_rmb();
		skb = skb_dequeue(queue);
		if (!skb)
			break;
		descriptor = bt->arena + offset + *head * (16 + footer);
		memset(descriptor, 0, 16 + footer);
		put_unaligned_le16(*head + 1, descriptor + 12);
		memcpy(descriptor + 16, skb->data, skb->len);
		wire_length = skb->len;
		if (ring == 1 && READ_ONCE(bt->connection_command_supported) &&
		    get_unaligned_le16(skb->data) == HCI_OP_CREATE_CONN) {
			u8 *command = descriptor + 16;

			put_unaligned_le16(CEN_OP_CREATE_CONN, command);
			command[2] = 15;
			command[HCI_COMMAND_HDR_SIZE + 9] = 0;
			command[HCI_COMMAND_HDR_SIZE + 12] = 0;
			wire_length = HCI_COMMAND_HDR_SIZE + 15;
			bt->connection_status_pending = true;
		}
		put_unaligned_le32(1 | (wire_length << 8), descriptor);
		*head = next;
		cen_post(bt, ring, next);
		if (hci_skb_pkt_type(skb) == HCI_COMMAND_PKT)
			bt->hdev->stat.cmd_tx++;
		else
			bt->hdev->stat.acl_tx++;
		bt->hdev->stat.byte_tx += wire_length;
		kfree_skb(skb);
	}
	return 0;
}

static void cen_fixup_scan_response(u8 *event, size_t length)
{
	struct hci_ev_le_ext_adv_info *info;
	size_t offset = HCI_EVENT_HDR_SIZE + 2;
	u8 count;

	if (length < offset || event[0] != HCI_EV_LE_META ||
	    event[HCI_EVENT_HDR_SIZE] != HCI_EV_LE_EXT_ADV_REPORT)
		return;
	count = event[HCI_EVENT_HDR_SIZE + 1];
	while (count-- && length - offset >= sizeof(*info)) {
		u16 type;

		info = (void *)(event + offset);
		if (info->length > length - offset - sizeof(*info))
			return;
		type = get_unaligned_le16(&info->type);
		if ((type & LE_EXT_ADV_EVT_TYPE_MASK) ==
		    (LE_EXT_ADV_LEGACY_PDU | LE_EXT_ADV_SCAN_RSP))
			put_unaligned_le16(type | LE_EXT_ADV_SCAN_IND,
					   &info->type);
		offset += sizeof(*info) + info->length;
	}
}

static void cen_io_work(struct work_struct *work)
{
	struct cen_bt *bt = container_of(work, struct cen_bt, io_work);
	struct hci_dev *hdev = bt->hdev;
	struct sk_buff *skb;
	u16 tail;
	u8 *descriptor;
	u32 length;
	int budget = CEN_RX_SLOTS;

	mutex_lock(&bt->lock);
	if (!bt->running || READ_ONCE(bt->transport_paused))
		goto unlock;
	if (readl(bt->bar + 0x18) != 2 || readl(bt->bar) != 2) {
		if (readl(bt->bar) == 2)
			cen_aux_drain(bt, false);
		goto protocol_error;
	}
	if (cen_aux_drain(bt, true))
		goto protocol_error;
	{
		tail = cen_tail(bt, 10);
		if (tail >= CEN_LE_SLOTS)
			goto protocol_error;
		while (bt->le_rx_head != tail && budget--) {
			dma_rmb();
			descriptor = bt->arena + CEN_LE_RX +
				     bt->le_rx_head * CEN_LE_STRIDE;
			length = get_unaligned_le32(descriptor) >> 8;
			if (length > CEN_LE_FOOTER)
				goto protocol_error;
			if (cen_receive_acl(bt, descriptor + 16, length))
				goto protocol_error;
			bt->le_rx_head = (bt->le_rx_head + 1) % CEN_LE_SLOTS;
			cen_le_rx_buffer(bt, bt->le_rx_post);
			bt->le_rx_post = (bt->le_rx_post + 1) % CEN_LE_SLOTS;
			cen_post(bt, 10, bt->le_rx_post);
		}
		budget = CEN_RX_SLOTS;
	}
	tail = cen_tail(bt, 7);
	if (tail >= CEN_RX_SLOTS)
		goto protocol_error;
	while (bt->rx_head != tail && budget--) {
		dma_rmb();
		descriptor = bt->arena + CEN_RX + bt->rx_head * CEN_STRIDE;
		length = get_unaligned_le32(descriptor) >> 8;
		if (length < HCI_EVENT_HDR_SIZE || length > CEN_FOOTER ||
		    length != descriptor[17] + HCI_EVENT_HDR_SIZE)
			goto protocol_error;
		skb = bt_skb_alloc(length, GFP_KERNEL);
		if (skb) {
			skb_put_data(skb, descriptor + 16, length);
			cen_fixup_scan_response(skb->data, skb->len);

			if (bt->connection_status_pending && length == 6 &&
			    skb->data[0] == HCI_EV_CMD_STATUS &&
			    get_unaligned_le16(skb->data + 4) ==
				    CEN_OP_CREATE_CONN) {
				put_unaligned_le16(HCI_OP_CREATE_CONN,
						   skb->data + 4);
				bt->connection_status_pending = false;
			}

			if (READ_ONCE(bt->connection_command_supported) &&
			    length ==
				    HCI_EVENT_HDR_SIZE +
					    sizeof(struct hci_ev_conn_complete) &&
			    skb->data[0] == HCI_EV_CONN_COMPLETE &&
			    (skb->data[11] & 0xfe) == 0xf0) {
				skb->data[11] = ACL_LINK;
			}
			hci_skb_pkt_type(skb) = HCI_EVENT_PKT;
			hdev->stat.byte_rx += length;
			hci_recv_frame(hdev, skb);
		} else {
			hdev->stat.err_rx++;
		}
		bt->rx_head = (bt->rx_head + 1) % CEN_RX_SLOTS;
		cen_rx_buffer(bt, bt->rx_post);
		bt->rx_post = (bt->rx_post + 1) % CEN_RX_SLOTS;
		cen_post(bt, 7, bt->rx_post);
	}
	if (cen_transmit(bt, &bt->txq, 1, CEN_TX_SLOTS, CEN_TX, CEN_FOOTER,
			 &bt->tx_head))
		goto protocol_error;

	if ((cen_transmit(bt, &bt->acl_txq, 2, cen_aux_specs[0].slots,
			  bt->aux[0].offset, cen_aux_specs[0].footer,
			  &bt->acl_tx_head) ||
	     cen_transmit(bt, &bt->le_txq, 4, CEN_LE_SLOTS, CEN_LE_TX,
			  CEN_LE_FOOTER, &bt->le_tx_head)))
		goto protocol_error;
	goto unlock;
protocol_error:
	bt->running = false;
	hdev->stat.err_rx++;
	dev_err(&bt->pdev->dev,
		"Beta transport stopped (exec=%u ipc=%u); interrupt processing stopped\n",
		readl(bt->bar + 0x18), readl(bt->bar));
unlock:
	mutex_unlock(&bt->lock);
}

static int cen_hci_open(struct hci_dev *hdev)
{
	struct cen_bt *bt = hci_get_drvdata(hdev);

	mutex_lock(&bt->lock);
	if (bt->quiescing || !bt->arena) {
		mutex_unlock(&bt->lock);
		return -ENETDOWN;
	}
	bt->running = true;
	schedule_work(&bt->io_work);
	mutex_unlock(&bt->lock);
	return 0;
}

static int cen_hci_close(struct hci_dev *hdev)
{
	struct cen_bt *bt = hci_get_drvdata(hdev);

	mutex_lock(&bt->lock);
	bt->running = false;
	bt->connection_status_pending = false;
	mutex_unlock(&bt->lock);
	cancel_work_sync(&bt->io_work);
	skb_queue_purge(&bt->txq);
	skb_queue_purge(&bt->acl_txq);
	skb_queue_purge(&bt->le_txq);
	return 0;
}

static int cen_vendor_command(struct hci_dev *hdev, u16 opcode,
			      const void *data, u8 length)
{
	struct sk_buff *skb;
	int ret;

	skb = __hci_cmd_sync(hdev, opcode, length, data, HCI_INIT_TIMEOUT);
	if (IS_ERR(skb))
		return PTR_ERR(skb);
	ret = skb->len < 1 ? -EPROTO : -bt_to_errno(skb->data[0]);
	kfree_skb(skb);
	return ret;
}

static int cen_hci_reset_dup_filter(struct hci_dev *hdev)
{
	static const u8 flags;
	int ret;

	ret = __hci_cmd_sync_status(hdev, CEN_OP_FLUSH_DUPLICATES,
				    sizeof(flags), &flags, HCI_CMD_TIMEOUT);
	if (ret) {
		bt_dev_warn(hdev, "Duplicate filter reset failed: %d", ret);
		return ret;
	}
	return 0;
}

static int cen_hci_post_init(struct hci_dev *hdev)
{
	struct cen_bt *bt = hci_get_drvdata(hdev);
	struct sk_buff *skb;
	u32 features;
	int ret;

	WRITE_ONCE(bt->connection_command_supported, false);
	skb = __hci_cmd_sync(hdev, CEN_OP_READ_PROP_FEATURES, 0, NULL,
			     HCI_INIT_TIMEOUT);
	if (IS_ERR(skb))
		return PTR_ERR(skb);
	ret = skb->len == 5 ? -bt_to_errno(skb->data[0]) : -EPROTO;
	features = skb->len == 5 ? get_unaligned_le32(skb->data + 1) : 0;
	kfree_skb(skb);
	if (!ret && !(features & CEN_PROP_CREATE_CONN))
		ret = -EOPNOTSUPP;
	if (ret) {
		bt_dev_warn(hdev, "Firmware connection command unavailable: %d",
			    ret);
		return 0;
	}
	WRITE_ONCE(bt->connection_command_supported, true);
	return 0;
}

/*
 * The factory address in network byte order, for the firmware's provisioning command: from
 * the standard local-bd-address (least significant byte first, as boot loaders fill it from
 * the bluetooth0 alias), else from local-mac-address (already in network order).
 */
static int cen_bt_address(struct device *dev, u8 address[6])
{
	u8 bd[6];
	int i, ret;

	if (device_property_count_u8(dev, "local-bd-address") == 6) {
		ret = device_property_read_u8_array(dev, "local-bd-address", bd, sizeof(bd));
		if (ret)
			return ret;
		for (i = 0; i < 6; i++)
			address[i] = bd[5 - i];
		return 0;
	}
	if (device_property_count_u8(dev, "local-mac-address") != 6)
		return -EINVAL;
	return device_property_read_u8_array(dev, "local-mac-address", address, 6);
}

static int cen_hci_setup(struct hci_dev *hdev)
{
	struct cen_bt *bt = hci_get_drvdata(hdev);
	u8 address[6], init_done[16], seed[24];
	int ret;

	ret = cen_bt_address(&bt->pdev->dev, address);
	if (ret)
		return ret;
	if (is_zero_ether_addr(address) || is_broadcast_ether_addr(address))
		return -EINVAL;
	ret = get_random_bytes_wait(seed, sizeof(seed));
	if (ret)
		return ret;
	memset(init_done, 0xff, sizeof(init_done));
	ret = cen_vendor_command(hdev, 0xfc06, address, sizeof(address));
	if (ret)
		goto out;
	WRITE_ONCE(bt->native_startup_steps, 1);
	ret = cen_vendor_command(hdev, 0xff0b, init_done, sizeof(init_done));
	if (ret)
		goto out;
	WRITE_ONCE(bt->native_startup_steps, 2);
	ret = cen_vendor_command(hdev, 0xfe44, seed, sizeof(seed));
	if (!ret)
		WRITE_ONCE(bt->native_startup_steps, 3);
out:
	memzero_explicit(seed, sizeof(seed));
	if (ret)
		bt_dev_err(
			hdev,
			"Firmware initialization stopped after %u commands: %d",
			bt->native_startup_steps, ret);
	return ret;
}

static int cen_hci_flush(struct hci_dev *hdev)
{
	struct cen_bt *bt = hci_get_drvdata(hdev);

	skb_queue_purge(&bt->txq);
	skb_queue_purge(&bt->acl_txq);
	skb_queue_purge(&bt->le_txq);
	return 0;
}

static int cen_hci_send(struct hci_dev *hdev, struct sk_buff *skb)
{
	struct cen_bt *bt = hci_get_drvdata(hdev);
	struct sk_buff_head *queue = &bt->txq;
	struct hci_conn *conn;
	u16 handle;
	u8 link_type;
	int ret = 0;

	if (hci_skb_pkt_type(skb) == HCI_COMMAND_PKT &&
	    (skb->len < HCI_COMMAND_HDR_SIZE ||
	     skb->len != HCI_COMMAND_HDR_SIZE + skb->data[2]))
		return -EMSGSIZE;
	if (hci_skb_pkt_type(skb) == HCI_ACLDATA_PKT) {
		if (skb->len < HCI_ACL_HDR_SIZE ||
		    skb->len != HCI_ACL_HDR_SIZE +
					get_unaligned_le16(skb->data + 2)) {
			ret = -EMSGSIZE;
			goto out;
		}
		handle = get_unaligned_le16(skb->data);
		rcu_read_lock();
		conn = hci_conn_hash_lookup_handle(hdev, hci_handle(handle));
		link_type = conn ? conn->type : 0xff;
		rcu_read_unlock();
		if (link_type == LE_LINK && (handle & 0x3000) != 0x3000) {
			queue = &bt->le_txq;
			if (skb->len > CEN_LE_FOOTER)
				ret = -EMSGSIZE;
		} else if (link_type == ACL_LINK) {
			queue = &bt->acl_txq;
			if (skb->len > cen_aux_specs[0].footer)
				ret = -EMSGSIZE;
		} else {
			ret = -EOPNOTSUPP;
		}
		if (ret)
			goto out;
	} else if (hci_skb_pkt_type(skb) != HCI_COMMAND_PKT) {
		ret = -EOPNOTSUPP;
		goto out;
	} else if (skb->len < HCI_COMMAND_HDR_SIZE || skb->len > CEN_FOOTER ||
		   skb->len != HCI_COMMAND_HDR_SIZE + skb->data[2]) {
		ret = -EMSGSIZE;
		goto out;
	}
	if (hci_skb_pkt_type(skb) == HCI_COMMAND_PKT) {
		u16 opcode = get_unaligned_le16(skb->data);

		if (opcode == CEN_OP_CREATE_CONN) {
			ret = -EOPNOTSUPP;
			goto out;
		}
		if (opcode == HCI_OP_CREATE_CONN &&
		    skb->len != HCI_COMMAND_HDR_SIZE +
					sizeof(struct hci_cp_create_conn)) {
			ret = -EMSGSIZE;
			goto out;
		}
	}
	mutex_lock(&bt->lock);
	if (!bt->running)
		ret = -ENETDOWN;
	else if (skb_queue_len(queue) >= 64)
		ret = -ENOBUFS;
	else {
		skb_queue_tail(queue, skb);
		if (!READ_ONCE(bt->transport_paused))
			schedule_work(&bt->io_work);
	}
	mutex_unlock(&bt->lock);
	if (!ret)
		return 0;
out:

	return ret;
}

static irqreturn_t cen_irq(int irq, void *data)
{
	struct cen_bt *bt = data;

	if (READ_ONCE(bt->running) && !READ_ONCE(bt->transport_paused))
		schedule_work(&bt->io_work);
	return IRQ_HANDLED;
}

static void cen_free_irqs(struct cen_bt *bt)
{
	while (bt->irq_count)
		free_irq(pci_irq_vector(bt->pdev, --bt->irq_count), bt);
	if (bt->irq_vectors) {
		pci_free_irq_vectors(bt->pdev);
		bt->irq_vectors = false;
	}
}

static int cen_request_irqs(struct cen_bt *bt)
{
	int ret = pci_alloc_irq_vectors(bt->pdev, 2, 2, PCI_IRQ_MSI);

	if (ret < 0)
		return ret;
	bt->irq_vectors = true;
	for (bt->irq_count = 0; bt->irq_count < 2; bt->irq_count++) {
		ret = request_irq(pci_irq_vector(bt->pdev, bt->irq_count),
				  cen_irq, 0, "btcentauri_t6050", bt);
		if (ret) {
			cen_free_irqs(bt);
			return ret;
		}
	}
	return 0;
}

/*
 * Called with the participant device locked, outside Control's boot_lock. Keep
 * the transport alive through HCI unregister: its close may send final commands.
 * HCI_UNREGISTER withdraws external HCI admission and drains HCI producers before
 * we close transport admission and settle the remaining IRQ/work users.
 */
static int cen_quiesce(void *data)
{
	struct cen_bt *bt = data;

	if (bt->host_quiesced)
		return 0;
	mutex_lock(&bt->lock);
	bt->quiescing = true;
	mutex_unlock(&bt->lock);
	if (bt->hci_registered) {
		hci_unregister_dev(bt->hdev);
		bt->hci_registered = false;
	}
	if (bt->hdev)
		cen_hci_close(bt->hdev);
	else {
		mutex_lock(&bt->lock);
		bt->running = false;
		mutex_unlock(&bt->lock);
	}
	cen_free_irqs(bt);
	/* free_irq synchronizes handlers which could still have queued I/O. */
	cancel_work_sync(&bt->io_work);
	skb_queue_purge(&bt->txq);
	skb_queue_purge(&bt->acl_txq);
	skb_queue_purge(&bt->le_txq);
	if (bt->hdev) {
		hci_free_dev(bt->hdev);
		bt->hdev = NULL;
	}
	bt->transport_sleep_sent = false;
	bt->asleep = true;
	bt->host_quiesced = true;
	return 0;
}

static int cen_dma_stopped(struct cen_bt *bt)
{
	u16 command;
	int ret;

	ret = pcibios_err_to_errno(pci_read_config_word(bt->pdev, PCI_COMMAND,
						      &command));
	if (ret)
		return ret;
	if (command & PCI_COMMAND_MASTER)
		return -EBUSY;
	if (!pci_wait_for_pending_transaction(bt->pdev))
		return -EBUSY;
	return 0;
}

/*
 * For published arenas only the Control owner calls this after ALL functions
 * have been fenced, while this original binding and DMA environment are live.
 * A local Command check is revalidation, not the all-function fence receipt.
 */
static int cen_release(void *data)
{
	struct cen_bt *bt = data;
	int ret;

	if (!bt->arena)
		return 0;
	if (!bt->host_quiesced || bt->hdev || bt->running || bt->irq_count ||
	    bt->irq_vectors || work_busy(&bt->io_work))
		return -EBUSY;
	if (bt->ipc_requested) {
		ret = cen_dma_stopped(bt);
		if (ret)
			return ret;
	}
	dma_free_coherent(&bt->pdev->dev, CEN_ARENA_SIZE, bt->arena, bt->dma);
	/* The allocation tuple remains authoritative until the actual free. */
	bt->arena = NULL;
	bt->dma = 0;
	bt->ipc_requested = false;
	return 0;
}

static bool cen_retained(void *data)
{
	struct cen_bt *bt = data;

	return bt->arena != NULL;
}

static const struct apple_centauri_sleep_ops cen_sleep_ops = {
	.owner = THIS_MODULE,
	.quiesce = cen_quiesce,
	.release = cen_release,
	.retained = cen_retained,
};

/* An unpublished allocation needs no all-function publication fence. */
static int cen_stop(struct cen_bt *bt)
{
	int ret;

	ret = cen_quiesce(bt);
	if (ret)
		return ret;
	pci_clear_master(bt->pdev);
	if (bt->ipc_requested)
		return -EBUSY;
	return cen_release(bt);
}

static int cen_probe(struct pci_dev *pdev, const struct pci_device_id *id)
{
	struct cen_bt *bt;
	struct hci_dev *hdev;
	int ret;
	struct pci_dev *control;
	struct device_link *link;
	struct pci_host_bridge *bridge = pci_find_host_bridge(pdev->bus);

	if (!bridge || !bridge->dev.parent ||
	    !of_device_is_compatible(bridge->dev.parent->of_node, "apple,t6050-pcie") ||
	    PCI_FUNC(pdev->devfn) != 2)
		return -ENODEV;
	control = pci_get_domain_bus_and_slot(
		pci_domain_nr(pdev->bus), pdev->bus->number,
		PCI_DEVFN(PCI_SLOT(pdev->devfn), 0));
	if (!control)
		return -EPROBE_DEFER;
	link = device_link_add(&pdev->dev, &control->dev,
			       DL_FLAG_AUTOREMOVE_CONSUMER);
	if (!link) {
		pci_dev_put(control);
		return -ENOMEM;
	}
	ret = apple_centauri_control_ready(control) ? 0 : -EPROBE_DEFER;
	pci_dev_put(control);
	if (ret)
		return ret;
	bt = devm_kzalloc(&pdev->dev, sizeof(*bt), GFP_KERNEL);
	if (!bt)
		return -ENOMEM;
	bt->pdev = pdev;
	mutex_init(&bt->lock);
	skb_queue_head_init(&bt->txq);
	skb_queue_head_init(&bt->acl_txq);
	skb_queue_head_init(&bt->le_txq);
	INIT_WORK(&bt->io_work, cen_io_work);
	ret = pcim_enable_device(pdev);
	if (ret)
		return ret;
	if (pci_resource_len(pdev, 0) != SZ_64K)
		return -ENODEV;
	ret = pcim_iomap_regions(pdev, BIT(0), "btcentauri_t6050");
	if (ret)
		return ret;
	bt->bar = pcim_iomap_table(pdev)[0];
	if (readl(bt->bar + 0x18) != 2 || readl(bt->bar) != 0)
		return dev_err_probe(
			&pdev->dev, -EBUSY,
			"Beta requires running firmware and unowned IPC (exec=%u ipc=%u)\n",
			readl(bt->bar + 0x18), readl(bt->bar));
	ret = dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(40));
	if (ret)
		return ret;
	bt->arena = dma_alloc_coherent(&pdev->dev, CEN_ARENA_SIZE, &bt->dma,
				       GFP_KERNEL);
	if (!bt->arena)
		return -ENOMEM;
	pci_set_drvdata(pdev, bt);
	ret = apple_centauri_sleep_register(pdev, &cen_sleep_ops, bt,
					    bt->arena, bt->dma, CEN_ARENA_SIZE);
	if (ret)
		goto stop;
	bt->sleep_registered = true;
	memset(bt->arena, 0, CEN_ARENA_SIZE);
	ret = cen_request_irqs(bt);
	if (ret)
		goto stop;
	ret = cen_start(bt);
	if (ret) {
		dev_err(&pdev->dev, "Beta IPC initialization failed: %d\n",
			ret);
		goto stop;
	}
	hdev = hci_alloc_dev();
	if (!hdev) {
		ret = -ENOMEM;
		goto stop;
	}
	bt->hdev = hdev;
	hdev->bus = HCI_PCI;
	hdev->open = cen_hci_open;
	hdev->close = cen_hci_close;
	hdev->flush = cen_hci_flush;
	hdev->send = cen_hci_send;
	hdev->setup = cen_hci_setup;
	hdev->post_init = cen_hci_post_init;
	hdev->reset_dup_filter = cen_hci_reset_dup_filter;

	hci_set_quirk(hdev, HCI_QUIRK_FIXUP_LE_EXT_ADV_REPORT_PHY);
	SET_HCIDEV_DEV(hdev, &pdev->dev);
	hci_set_drvdata(hdev, bt);
	ret = hci_register_dev(hdev);
	if (ret < 0)
		goto stop;
	bt->hci_registered = true;
	return 0;
stop:
	if (cen_stop(bt))
		apple_centauri_sleep_recovery_needed(pdev, ret);
	if (bt->sleep_registered) {
		/* Transfer any retained tuple before devm destroys this binding. */
		apple_centauri_sleep_unregister(pdev);
		bt->sleep_registered = false;
		bt->arena = NULL;
		bt->dma = 0;
	}
	return ret;
}

static void cen_remove(struct pci_dev *pdev)
{
	struct cen_bt *bt = pci_get_drvdata(pdev);
	int ret;

	/* Reconcile actual ownership even if suspend already stopped the HCI. */
	ret = cen_stop(bt);
	if (ret)
		apple_centauri_sleep_recovery_needed(pdev, bt->suspend_error ?: ret);
	if (bt->sleep_registered) {
		/*
		 * The independent owner takes any unreleased tuple and its explicit
		 * default-domain lease before our devm state goes away. That lease
		 * covers driver unbind, not arbitrary external physical device_del;
		 * managed recovery must release it before PCI function removal.
		 */
		apple_centauri_sleep_unregister(pdev);
		bt->sleep_registered = false;
		bt->arena = NULL;
		bt->dma = 0;
	}
}

static u32 cen_sleep_status(struct cen_bt *bt)
{
	return le32_to_cpu(READ_ONCE(*(__le32 *)(bt->arena + 0x70)));
}

static int cen_transport_state(struct cen_bt *bt, bool asleep)
{
	u32 status;
	int ret;

	if (!bt->arena || !bt->bar || readl(bt->bar + 0x18) != 2 || readl(bt->bar) != 2)
		return -EIO;
	dma_wmb();
	writel(asleep ? 3 : 0, bt->bar + 0x1078);
	ret = read_poll_timeout(cen_sleep_status, status,
				(asleep ? status == 3 : status <= 1) ||
				readl(bt->bar + 0x18) != 2,
				100, 2000000, false, bt);
	dma_rmb();
	if (readl(bt->bar + 0x18) != 2)
		ret = -EIO;
	dev_dbg(&bt->pdev->dev,
		"Beta ACIPC host %s: ret=%d status=%u exec=%u ipc=%u\n",
		asleep ? "sleep" : "wake", ret, status,
		readl(bt->bar + 0x18), readl(bt->bar));
	return ret;
}

/*
 * Keep firmware and HCI alive across deep sleep via ACIPC host-sleep;
 * fall back to teardown on fault.
 */
static int cen_suspend(struct device *dev)
{
	struct pci_dev *pdev = to_pci_dev(dev);
	struct cen_bt *bt = pci_get_drvdata(pdev);
	unsigned int i;
	int ret, save_ret;

	ret = bt->suspend_error;
	if (!bt->asleep && apple_centauri_sleep_reboot(pdev)) {
		if (!ret && bt->arena && bt->bar && bt->ipc_requested &&
		    readl(bt->bar + 0x18) == 2 && readl(bt->bar) == 2) {
			mutex_lock(&bt->lock);
			WRITE_ONCE(bt->transport_paused, true);
			mutex_unlock(&bt->lock);
			for (i = 0; i < bt->irq_count; i++)
				synchronize_irq(pci_irq_vector(pdev, i));
			cancel_work_sync(&bt->io_work);
			ret = cen_transport_state(bt, true);
			if (!ret) {
				bt->transport_sleep_sent = true;
				ret = pci_save_state(pdev);
				if (!ret)
					return 0;
				cen_transport_state(bt, false);
				bt->transport_sleep_sent = false;
			}
			dev_warn(dev, "Beta ACIPC host sleep failed (%d); falling back to teardown\n",
				 ret);
		}
		apple_centauri_sleep_mark_fallback(pdev);
		ret = cen_quiesce(bt);
		if (ret)
			return ret;
		bt->asleep = true;
		pci_clear_master(pdev);
		ret = cen_dma_stopped(bt);
		if (ret)
			dev_warn(dev, "Beta before sleep: DMA fence failed: %d; retaining arena\n",
				 ret);
	}
	save_ret = pci_save_state(pdev);
	/* A failed suspend is not guaranteed to receive its own resume callback. */
	if (bt->asleep && (ret || save_ret)) {
		if (!bt->suspend_error)
			bt->suspend_error = ret ?: save_ret;
		apple_centauri_sleep_recovery_needed(pdev, bt->suspend_error);
	}
	return ret ?: save_ret;
}

static int cen_suspend_noirq(struct device *dev)
{
	struct pci_dev *pdev = to_pci_dev(dev);
	struct cen_bt *bt = pci_get_drvdata(pdev);
	u16 pmctrl;

	if (!bt->transport_sleep_sent || !pm_suspend_via_firmware() || !pdev->pm_cap)
		return 0;
	if (!pci_read_config_word(pdev, pdev->pm_cap + PCI_PM_CTRL, &pmctrl)) {
		pci_write_config_word(pdev, pdev->pm_cap + PCI_PM_CTRL,
				      (pmctrl & ~PCI_PM_CTRL_STATE_MASK) |
				      PCI_D3hot | PCI_PM_CTRL_PME_ENABLE);
		mdelay(pdev->d3hot_delay);
	}
	return 0;
}

static u32 cen_pci_id(const struct pci_dev *pdev)
{
	return ((u32)pdev->device << 16) | pdev->vendor;
}

static int cen_resume_noirq(struct device *dev)
{
	struct pci_dev *pdev = to_pci_dev(dev);
	struct cen_bt *bt = pci_get_drvdata(pdev);
	u32 id = 0;
	u16 cmd = 0, pmctrl = 0;

	if (!bt->transport_sleep_sent)
		return 0;
	if (pci_read_config_dword(pdev, PCI_VENDOR_ID, &id) || id != cen_pci_id(pdev)) {
		dev_err(dev, "Beta after sleep: unexpected id %#x\n", id);
		return 0;
	}
	if (pdev->pm_cap &&
	    !pci_read_config_word(pdev, pdev->pm_cap + PCI_PM_CTRL, &pmctrl) &&
	    (pmctrl & PCI_PM_CTRL_STATE_MASK) != PCI_D0) {
		pci_write_config_word(pdev, pdev->pm_cap + PCI_PM_CTRL,
				      pmctrl & ~PCI_PM_CTRL_STATE_MASK);
		mdelay(pdev->d3hot_delay);
	}
	pci_set_master(pdev);
	pci_read_config_word(pdev, PCI_COMMAND, &cmd);
	if (pdev->pm_cap)
		pci_read_config_word(pdev, pdev->pm_cap + PCI_PM_CTRL, &pmctrl);
	dev_dbg(dev, "Beta noirq wake: id=%#x cmd=%#x pmcsr=%#x exec=%u ipc=%u\n",
		id, cmd, pmctrl, readl(bt->bar + 0x18), readl(bt->bar));
	return 0;
}

static int cen_resume(struct device *dev)
{
	struct pci_dev *pdev = to_pci_dev(dev);
	struct cen_bt *bt = pci_get_drvdata(pdev);
	int ret;

	/* Normally queued by Control's resume; for a suspend stopped before Control's */
	if (bt->asleep) {
		apple_centauri_sleep_rewake(pdev);
		return 0;
	}
	if (!bt->transport_sleep_sent)
		return 0;
	bt->transport_sleep_sent = false;
	{
		u32 id = 0;

		if (pci_read_config_dword(pdev, PCI_VENDOR_ID, &id) || id != cen_pci_id(pdev))
			ret = -ENODEV;
		else
			ret = cen_transport_state(bt, false);
	}
	if (ret) {
		dev_err(dev, "Beta ACIPC host wake failed (%d); scheduling warm-reset recovery\n",
			ret);
		apple_centauri_sleep_recovery_needed(pdev, ret);
		return 0;
	}
	mutex_lock(&bt->lock);
	WRITE_ONCE(bt->transport_paused, false);
	if (bt->running)
		schedule_work(&bt->io_work);
	mutex_unlock(&bt->lock);
	return 0;
}

static const struct dev_pm_ops cen_pm_ops = {
	SYSTEM_SLEEP_PM_OPS(cen_suspend, cen_resume)
	NOIRQ_SYSTEM_SLEEP_PM_OPS(cen_suspend_noirq, cen_resume_noirq)
};

static const struct pci_device_id cen_ids[] = {
	{ PCI_DEVICE(PCI_VENDOR_ID_APPLE, 0x1903) },
	{}
};
MODULE_DEVICE_TABLE(pci, cen_ids);

static struct pci_driver cen_driver = {
	.name = "btcentauri_t6050",
	.id_table = cen_ids,
	.probe = cen_probe,
	.remove = cen_remove,
	.driver.pm = pm_sleep_ptr(&cen_pm_ops),
};
module_pci_driver(cen_driver);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Apple N1 protocol-27 Bluetooth transport");
