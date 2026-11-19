/* SPDX-License-Identifier: GPL-2.0-only OR MIT */
/* DDC/CI through the DCP firmware, see ddc.c */
#ifndef __APPLE_DCP_DDC_H__
#define __APPLE_DCP_DDC_H__

struct apple_dcp;

void dcp_ddc_register(struct apple_dcp *dcp);
void dcp_ddc_unregister(struct apple_dcp *dcp);

#endif
