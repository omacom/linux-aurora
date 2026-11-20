/* SPDX-License-Identifier: GPL-2.0-only OR MIT */
#ifndef _LINUX_SOC_APPLE_DISPLAY_H_
#define _LINUX_SOC_APPLE_DISPLAY_H_

#include <linux/of.h>
#include <linux/string.h>
#include <linux/types.h>

struct device;

/* Initial native display admission, before starting retained GPU firmware. */
int apple_drm_t6030_link_device(struct device *consumer);
int apple_drm_t6031_link_device(struct device *consumer);
int apple_drm_t8132_link_device(struct device *consumer);

/* T6041 DCP image shipped with macOS 27.2 (26B123). */
#define APPLE_T6041_DCP_26B123_UUID "964EDE5B-8F78-38C2-A727-519CC9F0AF27"

static inline bool apple_t6041_dcp_any_v27_2(const struct device_node *np,
					     const char *compat_name)
{
	const char *uuid;
	u32 fw[3], compat[3];

	if (!np || !of_device_is_compatible(np, compat_name) ||
	    of_property_count_strings(np, "apple,firmware-uuid") != 1 ||
	    of_property_read_string(np, "apple,firmware-uuid", &uuid) ||
	    strcmp(uuid, APPLE_T6041_DCP_26B123_UUID) ||
	    of_property_count_u32_elems(np, "apple,firmware-version") != 3 ||
	    of_property_count_u32_elems(np, "apple,firmware-compat") != 3 ||
	    of_property_read_u32_array(np, "apple,firmware-version", fw, 3) ||
	    of_property_read_u32_array(np, "apple,firmware-compat", compat, 3))
		return false;

	return fw[0] == 27 && fw[1] == 2 && fw[2] == 0 &&
	       compat[0] == 27 && compat[1] == 2 && compat[2] == 0;
}

static inline bool apple_t6041_dcp_v27_2(const struct device_node *np)
{
	return apple_t6041_dcp_any_v27_2(np, "apple,t6041-dcp");
}

static inline bool apple_t6041_dcpext_v27_2(const struct device_node *np)
{
	return apple_t6041_dcp_any_v27_2(np, "apple,t6041-dcpext");
}

#endif
