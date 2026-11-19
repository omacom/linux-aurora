# SPDX-License-Identifier: GPL-2.0-only OR MIT

CFLAGS_trace.o = -I$(src)

appledrm-y := apple_drv.o apple_resume.o
appledrm-y += afk.o dcp.o dcp-fabric.o dcp_backlight.o dptxep.o iomfb.o parser.o systemep.o
appledrm-y += dcp-hdmi.o
appledrm-y += ddc.o
appledrm-$(CONFIG_DRM_APPLE_AUDIO) += audio.o
appledrm-$(CONFIG_DRM_APPLE_AUDIO) += av.o
appledrm-y += connector.o dcp-display-metadata.o dcp-180hz.o dcp-panel-edid.o
appledrm-y += ibootep.o
appledrm-y += dcpext_scanout.o
appledrm-y += dcpext_drm.o
appledrm-y += iomfb_v12_3.o
appledrm-y += iomfb_v13_3.o
appledrm-y += iomfb_v14_7_tpl.o
appledrm-y += iomfb_v14_7.o iomfb_v14_7_link.o
appledrm-y += iomfb_h17p.o
appledrm-$(CONFIG_DRM_APPLE_KUNIT_TEST) += iomfb_h17p_test.o parser_test.o
appledrm-$(CONFIG_DRM_APPLE_KUNIT_TEST) += apple_resume_test.o
appledrm-$(CONFIG_DRM_APPLE_KUNIT_TEST) += apple_boot_display_test.o
appledrm-$(CONFIG_DRM_APPLE_KUNIT_TEST) += parser_mode_test.o
appledrm-$(CONFIG_DRM_APPLE_KUNIT_TEST) += dcp_external_resume_test.o
appledrm-$(CONFIG_DRM_APPLE_KUNIT_TEST) += dcp_external_ready_test.o
appledrm-$(CONFIG_DRM_APPLE_KUNIT_TEST) += dcp_external_announce_test.o
appledrm-$(CONFIG_DRM_APPLE_KUNIT_TEST) += dcp_external_relink_test.o
appledrm-$(CONFIG_DRM_APPLE_KUNIT_TEST) += dcp_dropped_swap_test.o
appledrm-$(CONFIG_DRM_APPLE_KUNIT_TEST) += dcp-panel-edid-test.o
appledrm-y += iomfb_v26_6.o iomfb_v26_6_link.o j613_dcp_activate.o
appledrm-y += epic/dpavservep.o
appledrm-y += plane.o


appledrm-$(CONFIG_TRACING) += trace.o

obj-$(CONFIG_DRM_APPLE) += appledrm.o
obj-$(CONFIG_DRM_APPLE_KUNIT_TEST) += afk_test.o
obj-$(CONFIG_DRM_APPLE_KUNIT_TEST) += dcp_backlight_test.o
obj-$(CONFIG_DRM_APPLE_KUNIT_TEST) += dcp-announce-test.o

obj-$(CONFIG_DRM_APPLE_KUNIT_TEST) += afk-service-test.o

obj-$(CONFIG_DRM_APPLE_PARSER_KUNIT_TEST) += afkv2-test.o

appledrm-$(CONFIG_DRM_APPLE_DISPLAY_KUNIT_TEST) += parser-test.o
appledrm-$(CONFIG_DRM_APPLE_DISPLAY_KUNIT_TEST) += hotplug-test.o
appledrm-$(CONFIG_DRM_APPLE_DISPLAY_KUNIT_TEST) += v14-board-test.o
appledrm-$(CONFIG_DRM_APPLE_DISPLAY_KUNIT_TEST) += v14-plane-test.o
appledrm-$(CONFIG_DRM_APPLE_DISPLAY_KUNIT_TEST) += audio-no-sio-test.dtbo.o
