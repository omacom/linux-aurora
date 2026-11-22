/* SPDX-License-Identifier: GPL-2.0-only OR MIT */
#ifndef __APPLE_IOMFB_M4_H__
#define __APPLE_IOMFB_M4_H__

#include <linux/types.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/kconfig.h>

struct apple_dcp;
struct device;
struct dentry;
struct drm_connector;
struct drm_edid;
struct drm_display_mode;
struct drm_rect;
struct iosys_map;
struct drm_framebuffer;
struct iomfb_m4;

/* The firmware's IDs of an external mode (bootstrap 0: none). */
struct iomfb_m4_ext_ids {
	u32 timing, color, bootstrap;
};

/*
 * A framebuffer for direct scanout (XRGB8888/ARGB8888, linear, from its first
 * byte), by its @key: @pages gives the 16 KiB pages behind it (an array the
 * callee frees) for its first mapping; @get and @put take and drop the
 * reference a window holds on it while it may be scanned out (@put never under
 * the session lock).
 */
struct iomfb_m4_ext_fb {
	void *key;
	u32 width, height, pitch;
	int (*pages)(void *key, phys_addr_t **phys, unsigned int *count);
	void (*get)(void *key);
	void (*put)(void *key);
};

#if IS_ENABLED(CONFIG_DRM_APPLE_DCP_T8132)

/* Probe is inert; start adopts firmware after the common component is pinned. */
int iomfb_m4_init(struct apple_dcp *dcp);
int iomfb_m4_adopt_external(struct apple_dcp *dcp);
int iomfb_m4_start(struct apple_dcp *dcp, struct device *scanout);
int iomfb_m4_activate(struct apple_dcp *dcp);
int iomfb_m4_get_mode(struct apple_dcp *dcp, struct drm_display_mode *mode);
int iomfb_m4_health(struct apple_dcp *dcp);
int iomfb_m4_error(struct apple_dcp *dcp);
bool iomfb_m4_external_ready(struct apple_dcp *dcp);
void iomfb_m4_external_link_gone(struct apple_dcp *dcp);
/* An adopted external DCP's sink, for its DRM connector (t8132_ext_drm.c). */
int iomfb_m4_ext_detect(struct apple_dcp *dcp);
int iomfb_m4_ext_get_modes(struct apple_dcp *dcp, struct drm_connector *connector);
const struct drm_edid *iomfb_m4_ext_edid(struct apple_dcp *dcp);
/* DRM drives the external output: modeset, copied frames, disable. */
int iomfb_m4_ext_mode_ids(struct apple_dcp *dcp, const struct drm_display_mode *mode,
			  struct iomfb_m4_ext_ids *ids);
int iomfb_m4_ext_enable(struct apple_dcp *dcp, const struct iomfb_m4_ext_ids *ids,
			u32 width, u32 height);
int iomfb_m4_ext_present(struct apple_dcp *dcp, const struct iosys_map *src, u32 pitch,
			 const struct drm_rect *damage);
int iomfb_m4_ext_blank(struct apple_dcp *dcp);
/* 0 once submitted, its flip later (t8132_ext_drm_flip_done()); 1: its flip now. */
int iomfb_m4_ext_present_direct(struct apple_dcp *dcp, const struct iomfb_m4_ext_fb *fb);
/* Why an external DCP's display refuses system sleep, or NULL. */
const char *iomfb_m4_ext_sleep_veto(struct apple_dcp *dcp);
/* An external output through a system sleep. */
bool iomfb_m4_ext_sleep_mark(struct apple_dcp *dcp);
void iomfb_m4_ext_sleep_down(struct apple_dcp *dcp);
void iomfb_m4_ext_sleep_wake(struct apple_dcp *dcp);
bool iomfb_m4_ext_stale(struct apple_dcp *dcp);
int iomfb_m4_ext_restore_result(struct apple_dcp *dcp, unsigned int timeout_ms);
void iomfb_m4_ext_restore_end(struct apple_dcp *dcp);
void iomfb_m4_ext_relink_own(struct apple_dcp *dcp);
void iomfb_m4_ext_relink_expect(struct apple_dcp *dcp);
bool iomfb_m4_ext_relinking(struct apple_dcp *dcp);
void iomfb_m4_ext_relink_arm(struct apple_dcp *dcp, bool cable);
void iomfb_m4_ext_relink_lost(struct apple_dcp *dcp);
void iomfb_m4_ext_relink_wait(struct apple_dcp *dcp, unsigned long deadline);
int iomfb_m4_ext_relink_end(struct apple_dcp *dcp);
bool iomfb_m4_ext_stopped(struct apple_dcp *dcp);

/* The caller has waited for the producer fence before publishing the DMA GEM. */
int iomfb_m4_present(struct apple_dcp *dcp, struct drm_framebuffer *fb,
		     dma_addr_t dva, size_t mapped);
int iomfb_m4_blank(struct apple_dcp *dcp);
int iomfb_m4_suspend(struct apple_dcp *dcp);
int iomfb_m4_resume(struct apple_dcp *dcp);
void iomfb_m4_configure_backlight(struct apple_dcp *dcp);
void iomfb_m4_quiesce(struct apple_dcp *dcp);
void iomfb_m4_debugfs_init(struct apple_dcp *dcp, struct dentry *root);

#else
static inline int iomfb_m4_init(struct apple_dcp *dcp) { return -ENODEV; }
static inline int iomfb_m4_adopt_external(struct apple_dcp *dcp) { return -ENODEV; }
static inline int iomfb_m4_start(struct apple_dcp *dcp, struct device *scanout)
{
	return -ENODEV;
}
static inline int iomfb_m4_activate(struct apple_dcp *dcp) { return -ENODEV; }
static inline int iomfb_m4_get_mode(struct apple_dcp *dcp, struct drm_display_mode *mode)
{
	return -ENODEV;
}
static inline int iomfb_m4_health(struct apple_dcp *dcp) { return -ENODEV; }
static inline int iomfb_m4_error(struct apple_dcp *dcp) { return -ENODEV; }
static inline bool iomfb_m4_external_ready(struct apple_dcp *dcp) { return false; }
static inline void iomfb_m4_external_link_gone(struct apple_dcp *dcp) { }
static inline int iomfb_m4_ext_detect(struct apple_dcp *dcp) { return -ENODEV; }
static inline int iomfb_m4_ext_get_modes(struct apple_dcp *dcp, struct drm_connector *connector)
{
	return 0;
}

static inline const struct drm_edid *iomfb_m4_ext_edid(struct apple_dcp *dcp)
{
	return ERR_PTR(-ENODEV);
}

static inline int iomfb_m4_ext_mode_ids(struct apple_dcp *dcp,
					const struct drm_display_mode *mode,
					struct iomfb_m4_ext_ids *ids)
{
	return -ENODEV;
}

static inline int iomfb_m4_ext_enable(struct apple_dcp *dcp, const struct iomfb_m4_ext_ids *ids,
				      u32 width, u32 height)
{
	return -ENODEV;
}

static inline int iomfb_m4_ext_present(struct apple_dcp *dcp, const struct iosys_map *src,
				       u32 pitch, const struct drm_rect *damage)
{
	return -ENODEV;
}

static inline int iomfb_m4_ext_blank(struct apple_dcp *dcp) { return -ENODEV; }
static inline int iomfb_m4_ext_present_direct(struct apple_dcp *dcp,
					      const struct iomfb_m4_ext_fb *fb)
{
	return -ENODEV;
}

static inline const char *iomfb_m4_ext_sleep_veto(struct apple_dcp *dcp) { return NULL; }
static inline bool iomfb_m4_ext_sleep_mark(struct apple_dcp *dcp) { return false; }
static inline void iomfb_m4_ext_sleep_down(struct apple_dcp *dcp) { }
static inline void iomfb_m4_ext_sleep_wake(struct apple_dcp *dcp) { }
static inline bool iomfb_m4_ext_stale(struct apple_dcp *dcp) { return false; }
static inline int iomfb_m4_ext_restore_result(struct apple_dcp *dcp, unsigned int timeout_ms)
{
	return 0;
}

static inline void iomfb_m4_ext_restore_end(struct apple_dcp *dcp) { }
static inline void iomfb_m4_ext_relink_own(struct apple_dcp *dcp) { }
static inline void iomfb_m4_ext_relink_expect(struct apple_dcp *dcp) { }
static inline bool iomfb_m4_ext_relinking(struct apple_dcp *dcp) { return false; }
static inline void iomfb_m4_ext_relink_arm(struct apple_dcp *dcp, bool cable) { }
static inline void iomfb_m4_ext_relink_lost(struct apple_dcp *dcp) { }
static inline void iomfb_m4_ext_relink_wait(struct apple_dcp *dcp, unsigned long deadline) { }
static inline int iomfb_m4_ext_relink_end(struct apple_dcp *dcp) { return 0; }
static inline bool iomfb_m4_ext_stopped(struct apple_dcp *dcp) { return false; }

static inline int iomfb_m4_present(struct apple_dcp *dcp, struct drm_framebuffer *fb,
				  dma_addr_t dva, size_t mapped)
{
	return -ENODEV;
}
static inline int iomfb_m4_blank(struct apple_dcp *dcp) { return -ENODEV; }
static inline int iomfb_m4_suspend(struct apple_dcp *dcp) { return -ENODEV; }
static inline int iomfb_m4_resume(struct apple_dcp *dcp) { return -ENODEV; }
static inline void iomfb_m4_configure_backlight(struct apple_dcp *dcp) { }
static inline void iomfb_m4_quiesce(struct apple_dcp *dcp) { }
static inline void iomfb_m4_debugfs_init(struct apple_dcp *dcp, struct dentry *root) { }
#endif

#endif
