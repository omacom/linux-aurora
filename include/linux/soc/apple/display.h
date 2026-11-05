/* SPDX-License-Identifier: GPL-2.0-only OR MIT */
#ifndef _LINUX_SOC_APPLE_DISPLAY_H_
#define _LINUX_SOC_APPLE_DISPLAY_H_

#include <linux/errno.h>
#include <linux/kconfig.h>
#include <linux/of.h>
#include <linux/string.h>
#include <linux/types.h>

struct device;

/* Initial native display admission, before starting retained GPU firmware. */
int apple_drm_t6030_link_device(struct device *consumer);
int apple_drm_t6031_link_device(struct device *consumer);
int apple_drm_t8132_link_device(struct device *consumer);
int apple_drm_t6034_link_device(struct device *consumer);

/* T6041 DCP image shipped with macOS 27.2 (26B123). */
#define APPLE_T6041_DCP_26B123_UUID "964EDE5B-8F78-38C2-A727-519CC9F0AF27"

/*
 * @np is a @compat_name display processor that the boot loader handed over
 * with the 27.2 (26B123) image @image_uuid: firmware version and
 * compatibility 27.2.0, as aura writes them from the ADT.
 */
static inline bool apple_dcp_image_v27_2(const struct device_node *np,
					 const char *compat_name,
					 const char *image_uuid)
{
	const char *uuid;
	u32 fw[3], compat[3];

	if (!np || !of_device_is_compatible(np, compat_name) ||
	    of_property_count_strings(np, "apple,firmware-uuid") != 1 ||
	    of_property_read_string(np, "apple,firmware-uuid", &uuid) ||
	    strcmp(uuid, image_uuid) ||
	    of_property_count_u32_elems(np, "apple,firmware-version") != 3 ||
	    of_property_count_u32_elems(np, "apple,firmware-compat") != 3 ||
	    of_property_read_u32_array(np, "apple,firmware-version", fw, 3) ||
	    of_property_read_u32_array(np, "apple,firmware-compat", compat, 3))
		return false;

	return fw[0] == 27 && fw[1] == 2 && fw[2] == 0 &&
	       compat[0] == 27 && compat[1] == 2 && compat[2] == 0;
}

static inline bool apple_t6041_dcp_any_v27_2(const struct device_node *np,
					     const char *compat_name)
{
	return apple_dcp_image_v27_2(np, compat_name, APPLE_T6041_DCP_26B123_UUID);
}

static inline bool apple_t6041_dcp_v27_2(const struct device_node *np)
{
	return apple_t6041_dcp_any_v27_2(np, "apple,t6041-dcp");
}

static inline bool apple_t6041_dcpext_v27_2(const struct device_node *np)
{
	return apple_t6041_dcp_any_v27_2(np, "apple,t6041-dcpext");
}

/*
 * Software camera indicator (interim, not the secure indicator): a
 * kernel-owned overlay on the internal panel, requested best-effort by the
 * camera driver. set(true) returns 0 once the request is committed, without
 * waiting for the present; a square no present carries within 1 s is
 * withdrawn with a warning. The camera streams regardless of the answer.
 */
#if IS_ENABLED(CONFIG_DRM_APPLE)
int apple_dcp_camera_indicator_set(struct device *dcp, bool on);
#else
static inline int apple_dcp_camera_indicator_set(struct device *dcp, bool on)
{
	return -ENODEV;
}
#endif

/* T8122 internal DCP image shipped with macOS 27.2 (26B123). */
#define APPLE_T8122_DCP_26B123_UUID "93FA4B31-BB97-3B2A-8673-49EA489EE9C1"

/* The T8122 internal display processor, handed over on the 27.2 image. */
static inline bool apple_t8122_dcp_v27_2(const struct device_node *np)
{
	return apple_dcp_image_v27_2(np, "apple,t8122-dcp",
				     APPLE_T8122_DCP_26B123_UUID);
}

#endif
