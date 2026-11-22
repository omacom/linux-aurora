# SPDX-License-Identifier: GPL-2.0
# macsmc-j713.py checks the J713 SMC GPIO callbacks against a mocked SMC transport.
# atc-t8132-usb3.py runs on the build host against a CPU MMIO model.
# pmgr-inherited.py checks admission and lifecycle against host register mocks.
# dwc3-initial-role.py checks initial host-role admission before core reset.
# sn201202x-transport.py checks the production SPMI selection and payload path.
# bcm4377 tests exercise DMA ownership, context addresses and firmware handshakes.
# aop-las.py checks the production angle decoder and IIO callback on the host.
# aop-motion.py checks the motion report decoder and calibration selector.
# centauri-t6050-probe.py checks sibling probe serialization on the build host.
# dart-s2idle-retention.py checks the DART resume quarantine against host mocks.
# sep-ioctl-fuzz.py feeds malformed /dev/sep-key and /dev/sep-bio requests (root, no finger).
TEST_PROGS := macsmc-j713.py atc-t8132-usb3.py pmgr-inherited.py dwc3-initial-role.py \
	      sn201202x-transport.py bcm4377-dma-window.py bcm4377-radio-sids.py \
	      bcm4377-state-wait.py bcm4377-rti-address.py aop-las.py \
	      aop-motion.py centauri-t6050-probe.py dart-s2idle-retention.py \
	      sep-ioctl-fuzz.py
TEST_FILES := j813-peripherals.sh

include ../../lib.mk
