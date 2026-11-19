# SPDX-License-Identifier: GPL-2.0-only

apple-avd-y := avd-drv.o avd-v4l2.o avd-hw.o avd-pm.o avd-h264.o avd-vp9.o \
	avd-hevc.o avd-av1.o avd-av1-entropymode.o avd-queue.o avd-backend-open.o \
	avd-backend-apple.o avd-backend-queue.o
apple-avd-$(CONFIG_VIDEO_APPLE_AVD_KUNIT_TEST) += avd-h264-test.o avd-queue-test.o \
	avd-pm-test.o
obj-$(CONFIG_VIDEO_APPLE_AVD) += apple-avd.o
