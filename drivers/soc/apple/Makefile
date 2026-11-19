# SPDX-License-Identifier: GPL-2.0-only

obj-$(CONFIG_APPLE_CPU_CONTEXT_LAB) += apple-cpu-context-lab.o
apple-cpu-context-lab-y := cpu-context-lab.o cpu-context-lab-entry.o

obj-$(CONFIG_APPLE_DOCKCHANNEL) += apple-dockchannel.o
apple-dockchannel-y = dockchannel.o

obj-$(CONFIG_APPLE_MAILBOX) += apple-mailbox.o
apple-mailbox-y = mailbox.o

obj-$(CONFIG_APPLE_PMGR_MISC)	+= apple-pmgr-misc.o

obj-$(CONFIG_APPLE_RTKIT) += apple-rtkit.o
apple-rtkit-y = rtkit.o rtkit-crashlog.o

obj-$(CONFIG_APPLE_RTKIT_HELPER) += apple-rtkit-helper.o
apple-rtkit-helper-y = rtkit-helper.o

obj-$(CONFIG_APPLE_SART) += apple-sart.o
apple-sart-y = sart.o

obj-$(CONFIG_APPLE_T6030_DISPLAY_GATE) += apple-t6030-display.o
apple-t6030-display-y := t6030-display-gate.o t6030-pmp.dtbo.o t6030-j516s-pmp.dtbo.o
apple-t6030-display-$(CONFIG_APPLE_DISPLAY_GATE_KUNIT_TEST) += t6031-gate-test.dtbo.o \
							 t6030-gate-test.dtbo.o \
							 t8122-gate-test.dtbo.o
# The gate adds the interrupt parent from the base tree at boot.
DTC_FLAGS_t6030-pmp := -Wno-interrupts_property
DTC_FLAGS_t6030-j516s-pmp := -Wno-interrupts_property
# The KUnit trees add /chosen values and DCP nodes as fragments of /.
DTC_FLAGS_t6031-gate-test := -Wno-interrupts_property -Wno-chosen_node_is_root \
			     -Wno-avoid_unnecessary_addr_size
DTC_FLAGS_t6030-gate-test := -Wno-interrupts_property -Wno-chosen_node_is_root \
			     -Wno-avoid_unnecessary_addr_size
DTC_FLAGS_t8122-gate-test := -Wno-interrupts_property -Wno-chosen_node_is_root

obj-$(CONFIG_APPLE_TUNABLE) += apple-tunable.o
apple-tunable-y = tunable.o

obj-$(CONFIG_APPLE_AOP) += aop.o

obj-$(CONFIG_APPLE_SEP_FV_INTERFACE) += fv_shim.o

obj-$(CONFIG_APPLE_SEP) += apple-sep.o
apple-sep-y := sep.o hwrng_shim.o store_shim.o bio_shim.o sha_shim.o \
		crypto_shim.o sensor_shim.o p256_shim.o trusted_shim.o work_shim.o \
		pm_shim.o diag_shim.o

obj-$(CONFIG_APPLE_PMP) += apple-pmp.o
apple-pmp-y := pmp.o pmp-export.o

obj-$(CONFIG_APPLE_MCC) += apple-mcc.o
apple-mcc-y := mcc.o

obj-$(CONFIG_APPLE_CLPC) += clpc.o
