# SPDX-License-Identifier: GPL-2.0-only
obj-$(CONFIG_MT7932_FULLMAC) += mt7932-fullmac.o
mt7932-fullmac-y := mt7932.o recovery.o calibration_transport.o data.o frontend.o link.o startup.o reset.o lifecycle.o station.o power.o wow.o
mt7932-fullmac-$(CONFIG_MT7932_AWDL) += awdl.o
mt7932-fullmac-$(CONFIG_MT7932_AWDL_KUNIT_TEST) += awdl-test.o
mt7932-fullmac-$(CONFIG_MT7932_RF_KUNIT_TEST) += rf-test.o
mt7932-fullmac-$(CONFIG_MT7932_RF_KUNIT_TEST) += power-test.o
mt7932-fullmac-$(CONFIG_MT7932_RF_KUNIT_TEST) += link-loss-test.o
mt7932-fullmac-$(CONFIG_MT7932_KUNIT_TEST) += input-test.o
mt7932-fullmac-$(CONFIG_MT7932_KUNIT_TEST) += regulatory-test.o
mt7932-fullmac-$(CONFIG_MT7932_KUNIT_TEST) += association-test.o
mt7932-fullmac-$(CONFIG_MT7932_KUNIT_TEST) += six-test.o
mt7932-fullmac-$(CONFIG_MT7932_KUNIT_TEST) += move-test.o
mt7932-fullmac-$(CONFIG_MT7932_KUNIT_TEST) += wow-test.o

mt7932-fullmac-$(CONFIG_MT7932_KUNIT_TEST) += pmf-lpi-test.o
