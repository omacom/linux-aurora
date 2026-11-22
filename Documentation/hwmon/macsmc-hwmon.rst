.. SPDX-License-Identifier: GPL-2.0-only

Kernel driver macsmc-hwmon
==========================

Supported hardware

    * Apple Silicon Macs (M1 and up)

Author: James Calligeros <jcalligeros99@gmail.com>

Description
-----------

macsmc-hwmon exposes the Apple System Management controller's
temperature, voltage, current and power sensors, as well as
fan speed and control capabilities, via hwmon.

Because each Apple Silicon Mac exposes a different set of sensors
(e.g. the MacBooks expose battery telemetry that is not present on
the desktop Macs), sensors present on any given machine are described
via Devicetree. The driver picks these up and registers them with
hwmon when probed.

Manual fan speed is available on the fans whose Devicetree node has
apple,fan-manual-control, which a machine's Devicetree sets once its SMC
firmware is known to accept a manual speed and to take the fan back. On
those fans fanX_target is writable; writing 0 hands the fan back to the
firmware. On other fans fanX_target is read-only; there is no module
parameter to override the Devicetree. The SMC does not check a manual speed
against its own thermal policy, so a machine's Devicetree should set
apple,fan-manual-control only where the firmware is known to take the fan
back.

The driver leaves fan policy to the SMC until userspace writes a nonzero
``fanX_target``. Thermal zones do not take fans into manual mode. Suspend,
shutdown and removal release only fans taken over by such a request; resume
does not reapply it. A new target write is required after resume.

A kernel thermal policy (the M5 GPU governor) sets the fans through
macsmc_hwmon_fans_request(), as a share of each fan's range, on every fan
whose node names its mode, target, minimum and maximum keys, with or without
apple,fan-manual-control. It uses the same manual mode, leaves fans userspace
holds alone, and its fans return to the SMC at suspend, shutdown and removal
like userspace's.

SoC die temperatures are described by the ``apple,soc-die-keys`` string list
on the hwmon device node. The driver appends these temperature channels after
the ``temperature-*`` children, in list order, with labels ``SoC die <key>``.
Only readable four-byte floating-point keys are used. When thermal support is
enabled, their hottest reading supplies the ``macsmc_soc_die`` thermal zone.
A failed or implausible reading fails the zone read rather than reporting a
partial result. Keys absent from the firmware are skipped.
With ``apple,discover-temperatures`` the listed keys add no channels: the
discovered channel of each listed key, labelled with the key, joins the zone.

The optional ``apple,thermal-pressure-key`` on the same node describes a
readable signed 32-bit firmware thermal pressure level. Neither feature needs
a module parameter. Kernels using these properties need an updated device
tree; the former downstream per-board fallback and its ``soc_die_keys`` and
``board_keys`` parameters have been removed.

sysfs interface
---------------

currX_input
    Ammeter value

currX_label
    Ammeter label

fanX_input
    Current fan speed

fanX_label
    Fan label

fanX_min
    Minimum possible fan speed

fanX_max
    Maximum possible fan speed

fanX_target
    Current fan setpoint

inX_input
    Voltmeter value

inX_label
    Voltmeter label

powerX_input
    Power meter value

powerX_label
    Power meter label

tempX_input
    Temperature sensor value

tempX_label
    Temperature sensor label

thermal_pressure_level
    Firmware thermal pressure level, exposed read-only when the device tree
    supplies a valid key. The integer is firmware-defined and is not a
    temperature or a percentage.
