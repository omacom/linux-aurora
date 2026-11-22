/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _LINUX_PCI_APPLE_H
#define _LINUX_PCI_APPLE_H

#include <linux/types.h>

struct device;
struct device_node;
struct notifier_block;
struct pci_dev;

/* Atomic notification; data is the host's parent device, valid in callback. */
#define APPLE_PCIE_TUNNEL_LINK_DOWN 1

int apple_pcie_tunnel_register_notifier(struct notifier_block *nb);
void apple_pcie_tunnel_unregister_notifier(struct notifier_block *nb);

bool apple_pcie_tunnel_needs_cold_init(struct device_node *np);
int apple_pcie_tunnel_prepare(struct device *dev, struct device_node *tunnel);
int apple_pcie_tunnel_quiesce(struct device *dev);
int apple_pcie_tunnel_remove_orderly(struct device *dev, bool in_use_too);
int apple_pcie_tunnel_restore(struct device *dev);
int apple_pcie_tunnel_check_state(struct device *dev);
bool apple_pcie_tunnel_link_kept(struct device *dev);

/* NHI prepares/finishes; PCIe device noirq callbacks own the port and DART. */
int apple_pcie_tunnel_cio_prepare(struct device *dev);
bool apple_pcie_tunnel_cio_stopped(struct device *dev);
void apple_pcie_tunnel_cio_lost(struct device *dev);
int apple_pcie_tunnel_cio_finish(struct device *dev);

/*
 * T6050, for the driver of the endpoint below root port @bridge. Whether the host gave the
 * endpoint its warm reset at this wake, before the link came up; and, for a link that never
 * went down, the port taken down, the reset given, the port brought up again. From the
 * endpoint's suspend, when nothing below the port is to wake the system from this sleep:
 * request a warm-reset replay after link shutdown, before the sleep; or, when the endpoint
 * keeps its firmware alive across this sleep, skip the wake warm reset. When the host's last
 * bring-up of the port had its link up, or 0.
 */
#if IS_REACHABLE(CONFIG_PCIE_APPLE)
bool apple_pcie_t6050_warm_reset_done(struct pci_dev *bridge);
ktime_t apple_pcie_t6050_link_up_time(struct pci_dev *bridge);
int apple_pcie_t6050_port_recycle(struct pci_dev *bridge);
void apple_pcie_t6050_sleep_reset(struct pci_dev *bridge);
void apple_pcie_t6050_sleep_keep_alive(struct pci_dev *bridge);
#else
static inline bool apple_pcie_t6050_warm_reset_done(struct pci_dev *bridge)
{
	return false;
}

static inline ktime_t apple_pcie_t6050_link_up_time(struct pci_dev *bridge)
{
	return 0;
}

static inline int apple_pcie_t6050_port_recycle(struct pci_dev *bridge)
{
	return -ENODEV;
}

static inline void apple_pcie_t6050_sleep_reset(struct pci_dev *bridge)
{
}

static inline void apple_pcie_t6050_sleep_keep_alive(struct pci_dev *bridge)
{
}
#endif

#endif /* _LINUX_PCI_APPLE_H */
