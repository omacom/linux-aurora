// SPDX-License-Identifier: GPL-2.0-only OR MIT
/*
 * A run-time switch for admitting A18 Pro renders with mesh shading one client
 * at a time, as ray-traversal commands are (g17/traversal.rs). 1 (the default)
 * admits them so; 0 lets them run beside other clients' renders. Read on every
 * render submission.
 */
#include <linux/compiler.h>
#include <linux/moduleparam.h>

static unsigned int g17_mesh_exclusion = 1;
module_param(g17_mesh_exclusion, uint, 0644);
MODULE_PARM_DESC(g17_mesh_exclusion,
		 "A18 Pro: run renders with mesh shading for one client at a time (1: on, default; 0: off)");

unsigned int asahi_g17_mesh_exclusion(void);

unsigned int asahi_g17_mesh_exclusion(void)
{
	return READ_ONCE(g17_mesh_exclusion);
}
