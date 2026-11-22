/* SPDX-License-Identifier: GPL-2.0-only OR MIT */
#ifndef __APPLE_DCP_CEC_H__
#define __APPLE_DCP_CEC_H__

#include <linux/kconfig.h>
#include <linux/types.h>

#include "afk.h"
#include "cec-proto.h"

struct apple_dcp;

#if IS_ENABLED(CONFIG_DRM_APPLE_CEC)
int dcp_cec_register(struct apple_dcp *dcp);
void dcp_cec_unregister(struct apple_dcp *dcp);
int dcp_cec_init_avep(struct apple_dcp *dcp);
void dcp_cec_close_port(struct apple_dcp *dcp,
			struct apple_epic_service *port_srv);
void dcp_cec_poll_edid(struct apple_dcp *dcp);
bool dcp_cec_busy(struct apple_dcp *dcp);

void dcp_cec_service_init(struct apple_epic_service *service, const char *name,
			  const char *class, s64 unit);
int dcp_cec_service_report(struct apple_epic_service *service,
			   enum epic_subtype type,
			   const void *data, size_t data_size);
void dcp_cec_service_teardown(struct apple_epic_service *service);
#else
static inline int dcp_cec_register(struct apple_dcp *dcp)
{
	return 0;
}

static inline void dcp_cec_unregister(struct apple_dcp *dcp)
{
}

static inline int dcp_cec_init_avep(struct apple_dcp *dcp)
{
	return 0;
}

static inline void dcp_cec_close_port(struct apple_dcp *dcp,
				      struct apple_epic_service *port_srv)
{
}

static inline void dcp_cec_poll_edid(struct apple_dcp *dcp)
{
}

static inline bool dcp_cec_busy(struct apple_dcp *dcp)
{
	return false;
}

static inline void dcp_cec_service_init(struct apple_epic_service *service,
					const char *name, const char *class,
					s64 unit)
{
}

static inline int dcp_cec_service_report(struct apple_epic_service *service,
					 enum epic_subtype type,
					 const void *data, size_t data_size)
{
	return 0;
}

static inline void dcp_cec_service_teardown(struct apple_epic_service *service)
{
	afk_service_disable(service);
}
#endif

/*
 * Shared compact EPIC service entry for "dcpav-cec-interface-epic:0" on
 * endpoint 0x2b (announced cut to "dispext0:dcpav-cec-interface-ep").
 */
#define DCP_CEC_COMPACT_SERVICE_OPS { \
	.name = "dcpav-cec-interface-epic", \
	.compact_full = "dcpav-cec-interface-epic:0", \
	.singleton = true, \
	.init = dcp_cec_service_init, \
	.report = dcp_cec_service_report, \
	.teardown = dcp_cec_service_teardown, \
}

#endif /* __APPLE_DCP_CEC_H__ */
